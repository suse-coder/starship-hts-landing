# bench/02 — Parameter optimisation: fast evaluator, full 6-DOF gate, first search (O)

The first end-to-end run of the [spec/10](../03-spec/10-optimization.md) search. It answers whether a
parameter set reaches a true catch in the current tree. **It does not: the best full-6-DOF candidate ends
6.8 m off the pins at 11.7 m/s (margin 30.2 m), or 89 m off with 2.6 m across and a 1.17 m/s vertical at
the 1.88 km gate.** The baseline is 153.9 m off at 5.5 m/s. The search did find a non-obvious gate +
belly-plan + low-altitude-corridor combination that cuts the across-arms error from 95.8 m to 1.6 m; the
remaining blockers are contact speed, the along-arms offset and the roll geometry.

## 1. Architecture

| Piece | File | Role |
|---|---|---|
| `MissionTuning` | `gnc/src/tuning.ts` | one validated value for 70+ guidance/landing knobs; defaults are the flown constants |
| `score(recording)` | `sim/src/score.ts` | continuous catch margin, sideways catch geometry, roll/N₂ per phase, landing-trace diagnostics, hard constraints |
| Fast 3-DOF | `gnc/src/fast.ts` | entry hand-over → **re-planned** belly flop (the flight's own `bellyEnd` law) → two-phase landing with a direction lag; milliseconds |
| Search | `scripts/optimize.ts` | Halton/Sobol-style DOE → CMA-ES on the rankable knobs → full 6-DOF in bounded niced workers → Pareto table |

The tuning is threaded through `GuideCtx.tuning` into `guide`, `bellyEnd`, `planLanding`, the gains and
the roll bands. `makeWorld`/`runMission` do not yet accept it (out of this agent's file boundary); the
search applies it as `{ ...world, guide: { ...world.guide, tuning } }` and rebuilds the targeted plan
with `planMission` when `dvBurn` changes. That is an open wiring item, not a modelling difference.

## 2. Parameter vector and objective

**DOE screen (13):** `gateMiss` (log, 100–1800 m), `bellyPlanEnd` (800–3000 m), `bellyAimE/N` (±800 m),
`bellyBankRateDeg` (0.3–2.5 °/s, capped at the cold-gas authority), `tilt300/120/60/35/12/0mDeg`,
`terminalSwitchAlt`, `armCloseAlt`.

**CMA-ES refinement (8):** `gateMiss`, `bellyPlanEnd`, `bellyBankRateDeg`, `tilt35/12/0mDeg`,
`terminalSwitchAlt`, `armCloseAlt`.

**Full-6DOF variants (the fast model cannot rank these):** `landingWn/WMax/axisAccel`, `tilt*`,
`landingTauHigh/Low`, `landingOmegaMax`, `landingAimE/N`, `aimHigh`, and the four roll bands.

**Objective (3-DOF):** `margin + 0.02·|belly plan miss| + 0.002·max(0, flipH−1600) + 0.5·max(0,110−V)`.
`margin` is the `sim/score.ts` continuous miss: lateral excess over 1 m + 2× contact-speed excess over
1 m/s + 0.1× roll excess over 5° + 0.05× along + 0.1× across-velocity excess over 0.5 m/s; a run that
never crosses the rail plane adds the vertical stand-off and 100 m.

**Hard constraints (all from the score):** `armsClose && pinLoad`; pins ≤1 m; contact speed ≤1 m/s;
exactly 2 sea-level Raptors from the flip; no cold gas after the flip; coil discharged; flip speed
≥110 m/s (the 117 m/s entry budget with measurement room); header >200 kg; tile ≤1530 K, tile backing
≤600 K, bondline ≤560 K, steel ≤1150 K. Thermal failures do not invalidate a catch mechanism
(`catchFeasible` vs `feasible`), because they are a pre-existing entry/TPS issue.

## 3. Cost

| Quantity | Measured | Conditions |
|---|---|---|
| 3-DOF eval | median **179 ms**, p90 267 ms (n=304); 112/220 ms in the idle quick run | one core, Bun 1.4.4 |
| Full 6-DOF, alone | **28.5 s** (baseline) | load ≈4, `nice -n 19` |
| Full 6-DOF, 3–4 workers | **98–167 s per candidate** | load 15–23 (other agents' Chromium/git), one mission per worker |
| DOE 48 + CMA 240 + 10 full | ≈1 min fast + ≈5–10 min full | same load |

The full-mission number is dominated by the shared-CPU load at the time; the idle baseline is the
stable figure. The fast evaluator is ~160–300× cheaper per evaluation, which is the point of the
hierarchy.

## 4. Full 6-DOF results (default Δv 108, 2 Raptors, no cold gas after the flip)

| candidate | gate | flip m | belly m | margin | lat m | across | along | speed | roll | burn t | tile K | bond K |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline (default) | 630 | 1382 | 653 | 168.4 | 153.9 | 95.8 | 120.5 | 5.49 | 7.6° | 17.2 | 1650 | 695 |
| `fast-best` (CMA) | 621 | 1379 | 1456 | **30.2** | **6.8** | **1.6** | 6.6 | 11.72 | 24.0° | 16.3 | 1650 | 696 |
| `aim-300` | 621 | 1386 | 1357 | 34.9 | 7.3 | 0.5 | 7.3 | 14.60 | 13.7° | 16.1 | 1650 | 695 |
| `corridor-wide` | 621 | 1379 | 1456 | 34.3 | 11.9 | 9.7 | 6.9 | 11.48 | 24.2° | 17.3 | 1650 | 696 |
| `gain-mid` | 621 | 1379 | 1456 | 55.8 | 17.9 | 9.9 | 15.0 | 18.51 | 18.5° | 16.0 | 1650 | 695 |
| `gate1500` | 1500 | 1876 | 1456 | 118.4 | 89.5 | **2.55** | 89.4 | 13.50 | **5.5°** | 20.2 | 1650 | 696 |
| `gate1800` | 1800 | 2012 | 1456 | 139.4 | 83.0 | 29.6 | 77.5 | 26.91 | 11.4° | 20.7 | 1650 | 696 |
| `gate1200` | 1200 | 1722 | 1456 | 557.4 | 459.2 | 459.2 | 1.8 | 46.98 | 30.4° | 18.5 | 1650 | 696 |
| `aim-north` (gate 1500) | 1500 | 1876 | 1456 | 104.7 | 89.3 | 20.0 | 87.0 | **6.56** | 11.2° | 19.6 | 1650 | 696 |
| `roll-all` (0.2° bands) | 621 | 1371 | 1740 | 249.8 | 201.4 | 195.0 | 50.3 | 21.80 | 47.2° | 18.7 | 1675 | 695 |
| `term20-wide` | 621 | 1379 | 1456 | 91.9 | 43.6 | 36.1 | 24.4 | 23.32 | 22.2° | 18.0 | 1650 | 696 |
| `gate1500-term` | 1500 | 1876 | 1456 | 400.2 | 315.8 | 284.9 | 136.1 | 38.41 | 14.8° | 21.1 | 1650 | 695 |
| `gate1800-term` | 1800 | 2012 | 1456 | 242.6 | 120.7 | **0.0** | 120.7 | 56.94 | 8.7° | 23.3 | 1650 | 695 |

No candidate caught. `fast-best`, `aim-300`, `corridor-wide`, `gain-mid` and the terminal variants were
built on the CMA optimum; `gate1500/1800` isolate the flip gate at the same tuning.

The **vertical** contact speed is the second blocker: the baseline crosses at −4.2 m/s, `fast-best` at
−5.7, `gate1500` at −1.17 (nearest the 1 m/s catch). The two-phase vertical profile + 40 % engine floor
over-decelerates, then the ship climbs (all runs LOST with “climbing at 3.0 m/s”). `landingUzTrimMax`
has no effect in the active two-phase branch (the `uzTrim` argument is not applied there) — an open item.

## 5. Roll and N₂ per phase (pre-flip; the flip/landing has no cold gas)

| phase | baseline roll | N₂ kg | tight-roll variant roll | N₂ kg |
|---|---:|---:|---:|---:|
| entry | 2.23 °/s | 379.0 | 1.99 °/s | 381.1 |
| loft | 3.08 | 16.4 | 1.07 | 6.2 |
| pitch-up | 5.80 | 63.3 | 3.21 | 88.5 |
| high-α | 5.80 | 577.3 | 2.90 | 472.2 |
| belly flop | 3.19 | 1177.6 | 3.20 | 860.4 |
| **total** | | **2421** | | **2016** |

Tightening the four roll bands to 0.2°/s and raising the roll bandwidth cuts the loft/pitch-up/high-α
roll by 45–50 % and the N₂ by 17 %, but the belly roll is unchanged (it is driven by the bank-axis
attitude, not the roll band), and the full landing degrades (margin 249.8 m, belly delivery 1740 m). The
roll bands change the attitude/heading coupling, so the roll search must be evaluated with the landing,
not as an isolated gain.

## 6. What the optimiser found that the manual sweeps missed

- The coupled **gate 621 m design + belly PlanEnd 1835 m + a low-altitude corridor of 17°/17°/4°** gives
  a 1.6 m across-arms error and a 6.8 m total lateral, from 153.9 m. The manual sweeps moved the gate or
  the plan end, not the three together.
- The **1.88 km gate** is the best across/roll trade: 2.55 m across, 5.5° roll (the only candidate near
  the roll envelope) and 1.17 m/s vertical, at the cost of an 89 m along-arms offset. It is the gate to
  pursue once the terminal/along channel is fixed.
- `aimHigh = 300` (from 900) cuts the roll error from 24° to 13.7° and the across from 1.6 to 0.5 m, at
  a higher contact speed; the hand-over aim is a real geometry lever, not only a range bias.
- `landingAimN = +89` halves the contact speed at the 1.88 km gate (6.56 vs 13.5 m/s) but trades the
  across to 20 m: the aim is a speed/position trade, not a free correction.
- The **fast-only optimum is not certified**: the CMA's fastest candidates raised the bank rate to the
  cap and narrowed the low-altitude corridor; in 6-DOF the belly plan predicted 94 m and flew 1456 m.
  The ideal 3-DOF landing predicted a catch where the full run had 30 m.

## 7. Open items

1. **Fast model fidelity (review finding 1):** the landing still uses a PD on a linear-approach profile
   plus `LATERAL_EFFICIENCY = 0.45`, not the flight's two-phase lateral MPC. The propellant mass is now
   burned (≈16 t), which moved the fast 6-DOF-flip landing from a predicted catch (0.4 m) to a predicted
   climb (+3 m/s vertical, margin 227 m vs the flown 168 m) — closer in absolute, still the wrong
   controller. The belly is still perfect-tracking. It can rank the gate/corridor, not the belly or the
   landing gains. Next: call `planLanding` from the fast loop (the flight's controller) and add a belly
   lift-efficiency/perturbation ensemble so the ranker cannot call a 1456 m belly “94 m”.
2. **Terminal switch is a dead knob (review finding 2):** the phase needs `pinZ < alt && dHoriz < 8 &&
   vHoriz < 6`; no run met it, so `fast-best` and `terminal-late` were identical to the digit. The search
   must vary `terminalSwitchV`/`Lat` together with the altitude (the round-3 wide variants did, and the
   early switch destabilised the vertical).
3. **MPC weights are missing from the full variants (review finding 3):** `landingRd`,
   `landingTermQp/Qv`, `landingRollMaxDeg`, `landingWnRoll` must be in the next full-round DOE; the fast
   model cannot see them.
4. **Terminal gate / MPC weights in the search (round 2):** `terminalSwitchLat/V` are now in the DOE/CMA
   vector (the altitude alone was dead) and the next full set includes `landingRd`, `landingTermQp/Qv`,
   `landingRollMaxDeg`, `landingWnRoll`; no full runs are launched while the owner edits
   `control.ts`/`guidance.ts`.
5. **6-DOF root causes (review finding 4):** the two-phase lateral MPC has no thrust-slew model (the
   commanded azimuth reverses ~170° while the body needs ~5 s; attitude error 87–122°), and the
   `pz < 250 m` roll-to-N/S step + non-prioritised gimbal clipping steal pointing authority (the default
   run is 59 m off with 7 m/s at 346 m, then diverges at 250 m). Those are `control.ts`/`guidance.ts`
   fixes (in progress by the owner), not tuning.
6. **Wiring (review finding 5):** `makeWorld`/`runMission`/the app cache do not accept `MissionTuning`
   yet. The search overrides `world.guide.tuning`; the app should get the same pass-through and a cache
   key, so a tuning is a first-class value end to end.
7. **TPS:** tile 1650 K and bondline 695 K are over the limits in every candidate; this is the entry
   high-α transient, not a landing knob. It is reported and `feasible=false`, but `catchFeasible` is
   independent.
8. **Robustness:** the wind/κ/B/mass ensemble (T-mc) was not run; the best candidates are nominal only.

Reproduce:

```bash
nice -n 19 bun scripts/optimize.ts --screen 48 --gen 16 --full 10 --workers 3
nice -n 19 bun .tmp/agentO/round.ts .tmp/agentO/round3.json 3   # gate sweep from the first run
```

Artifacts: `.tmp/agentO/optimize-report.json` (all screen/CMA/full candidates and scores),
`.tmp/agentO/round{2,3,4}*.json`, `.tmp/review/trace_*.log`.
