# spec/10 — Parameter optimisation and multi-fidelity search

This document answers: can an optimiser find the best flight path/design without a full simulation for
every parameter change? It records what the code already supports, what is missing, and the fast search
architecture. Status: **implemented (first run, 2026-11-21)**; the first search and its honest
no-catch result are in [bench/02-optimizer](../05-benchmarks/02-optimizer.md).

> **Current outcome (2026-11-21):** the optimiser does **not** produce a catch. The best full-6-DOF
> candidate is 6.8 m lateral / 11.7 m/s, `lost`; later direction-gate and 150 m hand-over sweeps also
> end `lost`. The optimiser is a search tool, not evidence of a catch. The next step is the joint 3-D
> rate-limited PDG including the attitude state (review/02).

## 1. Current state (honest)

Optimiser-friendly:
- `runMission(design)` is deterministic, pure-ish and returns a recording; the same design gives the same run.
- The plant `step`, the guidance `guide`, the landing `planLanding`, the entry `predict`/`solveBank` and the
  MPC are pure functions; the recording is a typed data structure (`recording.ts`).
- The 3-DOF predictor/MPC and the full 6-DOF `step` already coexist (G/L split the entry and landing).
- The design is a schema-validated `DesignParams` (`craft/src/config.ts`), and the app cache keys on a
  physics hash, so a parameter set is a first-class value.

Not optimiser-ready:
- The tunables are scattered: `DesignParams`, `DEFAULT_PLAN` (`gnc/src/plan.ts`), `TRANSITION`
  (`sim/src/world.ts`), `AIM_ENTRY`/`AIM_HIGH`/`LANDING_ENGINES`/`CONTACT_SPEED`/`flipAltitude`
  (`gnc/src/guidance.ts`), `GAINS` (`gnc/src/control.ts`), `GLIDE_DAMPING` (`gnc/src/predictor.ts`),
  the landing MPC weights/tilt corridor (`gnc/src/landing.ts`, `mpc.ts`), the coil limits
  (`COIL_AUTH_MIN`, `COIL_OFF_HOLD`), the TPS thickness, and the mass inputs.
- There is no objective/constraint function, no parameter vector with bounds, no DOE/sensitivity screen,
  no parallel evaluator, and no separate fast evaluator. Every change costs a full mission.
- The full mission is the only ground truth, and it contains discrete events (phase changes, the catch,
  the throttle floor), so the response to a parameter is non-smooth.

## 2. Why full-sim-per-parameter is the wrong loop

The current manual sweeps (G/L) show the cost and the trap:
- ~30–95 s per mission; a 20-parameter screen is days.
- The response is non-monotone: the belly-flop miss vs gate, the flip speed vs Δv, the catch vs the delivery
  are not smooth. Finite differences and naive gradient descent are unreliable.
- The 3-DOF plan and the 6-DOF plant differ: L's PDG plan closed a 674 m hand-over to 0 m / 0.6 m/s, but the
  integrated run diverged by 1047 m because the plan lacked the torque-limited attitude state. **A fast
  surrogate can rank candidates, but only the 6-DOF can certify a catch.**

## 3. Target architecture

1. **`MissionTuning` schema** (new `gnc/src/tuning.ts`, or `sim/src/params.ts`): one `Schema.Class` holding every
   guidance/landing knob (gains, aim biases, transition q/V, bank rate, lead, coil thresholds, MPC weights,
   tilt corridor, gate shape, contact speed). `DesignParams` stays the vehicle/design half.
   `makeWorld(design, tuning)` and `runMission(design, tuning)`; the cache hash includes both.
2. **`score(recording)`** (new `sim/src/score.ts`): returns `{ cost, feasible, terms }`:
   - hard: catch (`armsClose && pinLoad`, ≤1 m lateral, ≤1 m/s), no cold gas after the flip, coil discharged,
     ≥2 SL Raptors, header margin, TPS limits;
   - soft/Pareto: landing propellant, tile/bondline margin, peak g/q, time, wind margin.
   The binary catch is replaced by a **continuous margin** (distance to the envelope + contact speed) so the
   optimiser has a gradient-like signal; the binary is the final constraint.
3. **Fast 3-DOF evaluator** (new `gnc/src/fast.ts` or `sim/src/fast.ts`): entry predictor + 3-DOF landing PDG,
   starting from the same entry state; returns the flip state and the predicted catch margin in **milliseconds**.
   It reuses the existing `predict`/`solveBank`/`planLanding`; the tables are cached per design.
4. **`scripts/optimize.ts`**: Sobol/DOE screen of the coupled parameters → CMA-ES or Bayesian optimisation over
   the top 5–8 → top-k candidates through the full 6-DOF `runMission` → Pareto table. Parallel workers
   (`sim.worker` pattern), `nice -n 19`, a bounded job count; the world/tables cached by design.
5. **Robustness**: the final candidates are scored over a small wind/κ/B/mass ensemble (the T-mc set), not a
   single nominal run.

## 4. Parameters and coupling

| Group | Examples | Coupled to |
|---|---|---|
| Entry | `dvBurn`, `AIM_ENTRY`, `AIM_HIGH`, `qPitch`, `vPitchMax`, `GLIDE_DAMPING.zeta`, bank rate/lead | speed at the flip, heat, range, delivery |
| Coil | `bStag`, `hallClosure`, `seedFraction`, `COIL_AUTH_MIN`, `COIL_OFF_HOLD` | trim, roll, switch-off time, heating |
| TPS | `picaCap`, `picaFlank`, tile thickness/ε | tile/bondline margin, mass, speed loss |
| Mass | `landingProp`, `n2Mass`, `rcsThrust`, dry items | CG, trim, burn, T/W, catch |
| Landing | `flipAltitude` shape, `LANDING_ENGINES`, `CONTACT_SPEED`, `GAINS.landing`, MPC weights, tilt corridor, `zFinal` | delivery, burn, catch margin, wind |
| Wind | `windScale`, `turbulence`, `seed` | robustness, T-mc |

The strong couplings (mass ↔ CG ↔ trim ↔ delivery ↔ gate ↔ burn; TPS ↔ speed loss) are why a
one-parameter sweep is misleading. The phase split (entry → **flip state** → landing) is the practical
decomposition: optimise the entry to a target flip state, then the landing from that state, then validate
the coupled 6-DOF.

## 5. The fast workflow

1. Build the tables once for a design; cache by the design hash.
2. Sobol/DOE screen ~20–30 parameters with the 3-DOF evaluator (ms each); rank by the margin/cost.
3. CMA-ES/BO on the top 5–8 parameters, constraints as penalties; thousands of 3-DOF evaluations.
4. Take the top ~10–20 candidates; run the full 6-DOF mission for each (parallel workers, one mission per
   core); score the real recording.
5. Re-rank on the 6-DOF, run the wind ensemble (T-mc) for the best 3–5, and report the Pareto front.
6. Never certify a candidate from the 3-DOF alone; the L PDG plan/plant mismatch is the warning.

## 6. What is missing / next steps

- [x] `MissionTuning` schema (`gnc/src/tuning.ts`) and threading through `GuideCtx` → `guide`/
  `bellyEnd`/`planLanding`/gains/roll bands. `makeWorld`/`runMission`/app-cache pass-through remains open.
- [x] `sim/src/score.ts`: continuous catch margin, sideways catch geometry, roll/N₂ per phase, landing
  trace diagnostics (azimuth swing, attitude lag, gimbal saturation) and hard constraints.
- [x] `gnc/src/fast.ts`: 3-DOF evaluator (entry hand-over → re-planned belly via the flight's `bellyEnd` →
  two-phase landing) with measured ms budget.
- [x] `scripts/optimize.ts`: DOE → CMA-ES → parallel full 6-DOF → Pareto table.
- [x] `docs/05-benchmarks/02-optimizer.md`: fast/full timings and the first search.
- [ ] The app's Design panel/cache key includes `MissionTuning`; the "Re-simulate" path stays.

The fast path is implemented and used for the first search; the 6-DOF remains the only certifier. The
first run found no true catch: the best full-6-DOF margin is 6.8 m lateral / 11.7 m/s at the 1.38 km gate,
and 2.55 m across / 5.5° roll / 1.17 m/s vertical (but 89 m along) at the 1.88 km gate. See bench/02.
