# review/01 — Adversarial physics review (T08)

- Reviewer: agent R (read-only), 2026-11-21
- Scope: the 144.05 t build before the Cs fix, the 99 m/s entry, Q23 tiles, Q25 coil
  switch-off, the R11c cold-gas change, and the T09 readiness
- Evidence: `bun test packages` 202 pass / 2 todo / 0 fail; `tsc` clean; `.tmp/mission_final.log`
- Post-review fixes made by the orchestrator are marked **[fixed]**; the rest are open.

## 1. Bottom line

The plant is clean: no added force, no phase-disabled physics, no state write outside the
RK4/thermal/coil integrators; the loss is honest (G0 respected). But **R6, R11a and R11c are
actively breached**, R11b is only half-true physically, and the entry is 17 km short of being a
T09 input. **T09 gate: NO-GO.**

## 2. P0 — blockers

### P0-1 · R11c: the final approach runs on one Raptor, not ≥ 2

`LANDING_ENGINES = 1` (`packages/gnc/src/guidance.ts:184`); the hand-over shuts two engines down
(`:497-509`, event at `:544`). Mission: `two Raptors shut down, landing burn on RSL1` at T+2780.7.
`useRcs:false` is correct (`:570`) and N₂ is constant after the flip, so the cold-gas clause passes;
only the engine count fails. `control.ts:38-43` still justifies the gains by cold-gas roll authority.

**Fix (T09):** two SL Raptors from flip ignition to pin contact, differential gimbal for roll (the
allocator already has the full-rank 3×2N path, `control.ts:220-273`), timed deceleration to the rails
at T/W ≈ 1.8, no hover. Add a test asserting `useRcs===false` and ≥ 2 SL engines in
`flip`/`landing`/`terminal`.

### P0-2 · R6: guidance commands the plant's exact aero moment as feed-forward

`Nav.aeroM` is the plant's instantaneous aero moment (`packages/sim/src/step.ts:120`), and the plant
cancels it exactly when guidance sets the flag (`guidance.ts:366` → `step.ts:134-135`). During entry
this includes the rate-damping moment, so the unstable mode is nulled by an oracle no flight computer
has. `review/00 §2` claims guidance never reads hidden truth; this channel is not in the R9 list and
is used for the whole nose-first entry (q̄ > 50 Pa).

**Fix:** compute the feed-forward from guidance's own table (`aeroLookup(ctx.aero, n.alpha, n.mach)`)
inside `guide`, keep the plant passive, and record the resulting α excursion / MHD demand. If kept,
it must be I9 in `review/00` with its measured effect.

### P0-3 · R6: the coil switch-off authority uses the plant's MHD basis

`coilAuthority` (`guidance.ts:156-163`) calls `allocateMhd(n.mhd, …)`; `n.mhd` is the plant's exact
per-sector force/moment matrix, flux cap and plasma state (`packages/mhd/src/mhd.ts:134-147`),
refreshed from truth. The demand is pitch/yaw only, so the criterion ignores roll. The logged
`mhdAuth` is a different estimator (`step.ts:144`) and is meaningless when demand < 1 N·m:
`auth=9515417` (T+2366), `4323605` (T+2517), `4639695` (T+2577), then `0.01`/`0.00` when the plasma
is gone (by construction, `guidance.ts:166`).

**Fix:** build the basis in guidance from its own air data and assumed κ/seed (or an on-board σ
estimate); use a combined three-axis demand; clamp/report `n/a` when demand < 1 N·m.

### P0-4 · R11a: cold gas is used with the coil on and never reported

N₂: 4792 kg at T+1642 → 3825 kg at the T+2606 switch-off (≈ 0.88 t during the coil phase), of which
≈ 560 kg was used while the MHD authority was already > 1. Mechanism: the allocator weights roll
1 : 30 against pitch/yaw (`mhdAlloc.ts:34`), and the plant lets cold gas take pitch/yaw whenever
q̄ < 1200 Pa even at authority > 1 (`step.ts:159`, magic 1.2). No event or results row reports RCS
use; only the raw `n2` channel. R11a is still `[ ]` in the roadmap (T14) and spec/03.

**Fix:** report per-phase cold-gas impulse in the event log/results (R11a "its use is reported");
move the q̄/authority gate into flight software; allocate roll with equal weight or report the
measured roll shortfall.

### P0-5 · Entry delivery: 17.0–17.7 km at the flip

`predicted miss 17712 m` at T+2774.3, pad offset `(3.2, 16.5) km`. Below ~26 km the bank solution
keeps flipping: σ target `+110° → −110°` at T+2624–2634, `DR +18.5 → −16.4 km` in 5 s, 7.53 g /
42.6 kPa at 22.8–26.3 km after the coil is off. `REV_LEAD`/the 15 s dwell floor (`guidance.ts:722,831`)
is in a limit cycle.

**Fix/experiment (Q28/T09):** freeze the bank once the predicted footprint contains the pad; lengthen
the reversal dwell with range; re-run the predictor's σ-rate with the post-switch-off actuator; log
bank command, `tRevPlan` and the competing minima per solve. Gate: belly < 500 m (spec/05).

## 3. P1 — physics risk

1. **The coil is never actually switched off.** The ramp-down is battery-limited at 450 kW (`coil.ts`,
   `config.ts`): dI/dt ≈ 2.32 A/s at 3 kA. From T+2606 to impact (T+2854) the current falls only to
   ≈ 2.35 kA (78 %), E ≈ 180 MJ. No event reports current zero. **Report I and E at impact**
   (summary now has `coilIEnd`/`coilEEnd` **[fixed]**); a faster dump or a higher P_max is an R4
   trade, not changed.
2. **The 1760 K tile surface is a pitch-up transient**, not peak-q: T+2306, α 43.3° vs 80° command,
   err 40.4°; the pitch-up exits on the 40 s timeout. `passiveTrim` returns 80° at M 16.3 while the
   plant holds ~28–43° — a guidance/table vs 6-DOF mismatch (`predictor.ts:52-59` claims ~28°).
   Q30/spec/08: 484 kW/m² at 37.4 m at the pitch-up vs 43 kW/m² at peak q; 35 s above 1530 K; no
   TPS-only fix. **Fix:** command the α the plant actually trims at, or pitch up at ≲ 4.5 km/s.
3. **No "outside limits" outcome.** Tile 1759.8 K > 1530/1673 K; bondline 613.8 K > 560 K. A future
   catch would read CAUGHT. **Badge now appends " · OUTSIDE TPS LIMITS"** (`main.ts`) **[fixed]**;
   the outcome enum itself is unchanged.
4. **Caesium counted twice (+400 kg).** `massSeed = 90 + 400` dry and `seed: 400` consumable. The
   physical wet mass was 143.65 t, not 144.05 t. **Dry item is now 90 kg; wet 143.65 t** (`config.ts`)
   **[fixed]**; every quoted total updated.
5. **Empty tanks still produce thrust.** `engineThrust` has no propellant interlock; `step.ts` only
   clamps the tank at zero. **Engines now zero when the feeding tank is empty** (`step.ts`) **[fixed]**;
   the depletion event still spams (see P2-6).
6. **Post-switch-off bank rate is still the coil-era 1.5°/s** (`SIGMA_RATE`, `guidance.ts:720`), used
   by the bank law and the predictor below 30 km after the coil is gone; the cold-gas couples give
   ≈ 1.25°/s². Combined with the 15 s dwell this drives the ±110° chatter and the 7.5 g pull-out.
7. **PICA and tile substrates differ:** the tile backing radiates to a 250 K interior (≈ 210 kW over
   1072 m² at peak, unbooked tank heat load), the PICA substrate is adiabatic. The PICA bondline is
   conservative, the tile backing optimistic. List both in R9.
8. **Thermal tile mass ≠ mass-budget tile mass:** thermal 144 × 0.03 = 4.32 kg/m²; mass 5.82 kg/m².
   The 1.5 kg/m² of pins/gap filler has no heat capacity; the bond is 0.2 mm under the tile, 0.3 mm
   under PICA. Surfaces heat slightly fast (conservative); backing optimistic.
9. **`massFinal: 0` was hard-coded** (`run.ts`). **Now `massProps(world.vehicle, s.cons).m`** **[fixed]**.
10. **`dvBurn = 99` is selected from a miss-minimising sweep**; recorded (Q28), but G1's "no
    hand-tuned trajectory" is not demonstrable. State that the deorbit burn is a targeted mission
    parameter and the bank law must fly from it.

## 4. P2 — documentation and test gaps

1. **Stale numbers:** the roadmap and plan/05 quoted the 105 m/s run (51 m/s @T+3110, tile 1868/307,
   switch-off 2360.7, N₂ 3333). The final 99 m/s run: **94.9 m/s @T+2854.4, tile 1760/328, switch-off
   T+2606.0, N₂ 2182 kg from the landing hand-over**. Updated in the roadmap **[fixed]**.
2. **Two todos as advertised, plus a third hidden by SLOW:** `aero.test.ts:127,148` (stale spec
   numbers) and `sim.test.ts:153` (`caught`, SLOW-only).
3. **Tests that pass for the wrong reason:** the tile energy test reproduces the code's own sink
   terms; the SLOW `steel < 1150 K` assertion now tests the tile backing (328 K), not the hull; no
   test asserts the tile surface or the bondline in the mission run.
4. **No tests for R11a/R11b/R11c:** nothing asserts the switch-off, `useRcs===false`, ≥ 2 engines,
   or MHD roll authority.
5. **Stale comments/labels:** `control.ts:38-43` (cold-gas roll); `survival.ts` header ("bare steel");
   `survival.ts` hot-spot `steelVerdict` on tile-surface temperatures; `DEFAULT_PLAN.mass = 115000`;
   spec/05 has two "## 3" headings.
6. **`mhdAuth` unusable when demand < 1 N·m** (values up to 9.5e6); clamp or mark `n/a`.
7. **Q24 still open:** the 26 t header is exhausted 61 s after the flip, at 1.2 km; no landing margin
   exists.
8. **Mission wall 36.6 s vs G4 ≤ 30 s** (bench/01: 35.4 s).

## 5. Rule verdicts

| Rule | Verdict | Reason |
|---|---|---|
| R1 | PASS | State moves only in the integrators; the aeroM/n.mhd leaks are knowledge, not force. |
| R2 | PASS | Moments only from coil, cold gas, gimbals, contact. |
| R3 | PASS (I4/I5 caveats) | Real MHD physics; κ and the basis are guidance-visible. |
| R4 | PARTIAL | Limits exist; no propellant interlock (fixed), 2-engine floor not flown. |
| R5 | PASS (bookkeeping error, fixed) | Tiles/header recorded; Cs double count fixed. Q24 open. |
| R6 | **FAIL** | `aeroM` feed-forward, `n.mhd` authority, plant-side RCS gate. |
| R7 | PASS | J2, rotation, wind, real gas, ablation, mass loss always on. |
| R8 | PARTIAL | Limits shown, not clipped; outcome/badge gap (badge fixed), massFinal fixed. |
| R9 | **FAIL** | Omits the aero feed-forward, plant MHD basis, cold-gas fallback, tile interior radiation, transition assumption. |
| R10 | PASS | `lost`, reason ground impact at 94.9 m/s; UI shows it. |
| R11a | **FAIL** | ~0.9 t cold gas with the coil on, half at authority > 1, unreported. |
| R11b | PARTIAL | Timely authority ramp-down, no re-ramp; coil still at ~78 % / 180 MJ at impact. |
| R11c | **FAIL** | No cold gas after the flip (pass), but the terminal flies one Raptor. |

## 6. T09 gate: NO-GO

1. Delivery: flip miss < 500 m; today 17.0–17.7 km (P0-5).
2. Actuator rule: ≥ 2 SL Raptors from flip to pins with differential-gimbal roll; a test asserting
   `useRcs===false` and ≥ 2 engines (P0-1).
3. Honesty: remove/reclassify `aeroM` and `n.mhd` (R6/R9), report cold-gas use (R11a), report the
   real coil current at touchdown (R11b) (P0-2/3/4, P1-1).
4. TPS: re-fly the pitch-up so the tile stays under 1530 K (α ≲ 22° / pitch at ≲ 4.5 km/s), or add
   the "outside limits" outcome so a catch is not mislabelled (P1-2/3).
5. Propellant: re-size the header against the actual hoverslam use; 26 t is exhausted 61 s after the
   flip at 1.2 km (Q24).

The plant and the honesty of the current outcome are good enough for T09 to inherit; the control
rules and the entry are not.
