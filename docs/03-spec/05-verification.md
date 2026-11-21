# spec/05 — Verification and validation

Commands: `bun run typecheck`, `bun run test` (tests in `packages/*/test`), `bun run mission`,
headless UI check `node .tmp/pw/shot.cjs`. Status: `[x]` passing (evidence) · `[~]` checked by
script, no automated test yet · `[ ]` open.

Current baseline (2026-11-21): `bun run typecheck` clean; `bun test packages` **238 pass, 2 todo,
0 fail** across 22 files. The two todos are the stale aero subsonic-trim band and the belly-flop
terminal-velocity number; T-mission remains open because the catch is not achieved. The public
release gate is in [spec/11](11-release.md).

## 1. Unit and model tests

| ID | Check | Acceptance | Evidence | Status |
|---|---|---|---|---|
| T-atm | US-1976 density at 0/11/20/32/50/86 km | within 0.5 % of the standard | `earth/test/atmosphere.test.ts` | [x] |
| T-orbit | two-body + J2 RK4, one orbit | relative energy drift < 1e-8 | `earth/test/gravity-orbit.test.ts` (drift 3.8e-11) | [x] |
| T-air | equilibrium air composition | 79/21 at 300 K; O₂ mostly dissociated at 4000 K; x_e 1–3 % at 10 000 K | `gas/test/air.test.ts` | [x] |
| T-shock | perfect-gas limit, M 2.4 | density ratio within 1 % of Rankine–Hugoniot | `gas/test/air.test.ts` (0.46 %) | [x] |
| T-coil | loop field on axis; far field | matches μ₀Ia²/2(a²+z²)^{3/2}; → dipole | `mhd/test/mhd.test.ts` | [x] |
| T-aero | trim and stability | subsonic broadside trim 80–88°, stable; nose-first unstable at M 8 | `aero/test/aero.test.ts` — stability passes; subsonic trim at the landing CG (27.75 m) is 74.1°/71.7° at M 0.3/0.5, outside 80–88° (band came from x_cg 25.6 m): re-baseline pending an independent check | [~] |
| T-heat | stagnation heating; wall energy | Sutton–Graves within 15 % of Detra–Kemp–Riddell at 7 km/s; PICA/steel conduction stores what it absorbs within 0.5 % | `heat/test/heat.test.ts` (F, T12) | [x] |
| T-alloc | MHD allocation | commanded +My realised within 1 % when authority suffices | `gnc/test/control.test.ts`: 1 % up to 30 % of authority; 60–90 % authority is T14 | [~] |
| T-gimbal | roll-priority projection and gimbal angles | abs(τx) ≤ cap always; gimbal angle from asin(s₁/cos gy) reproduces the torque within 1 % | `gnc/test/control.test.ts` (F, T12) | [x] |
| T-mpc | landing MPC on the plain model | 1.2 km divert ends < 1 m, < 0.5 m/s at the planned time; constraints satisfied | `gnc/test/mpc.test.ts`: constraints hold; on a plant with a 2 s thrust lag the τ-aware plan misses by 1.8 m (τ-blind 5.2 m); plain model < 0.2 m (`.tmp/scripts/mpctest.ts`) | [~] |
| T-ground | ground contact | run ends at the lowest hull point ≥ −v·dt | mission event "ground impact at … m/s" | [~] |
| T-mhd-phys | MHD physics sanity | moment = −Σ r × (j × B) dV; zero with the coil off, K_s = 1, or V < 2 km/s; force ≤ momentum flux | `mhd/test/mhd.test.ts` | [x] |
| T-coil-tcs | current-sharing temperature at I_op | T_cs(I_op) ≈ 41.6 K (craft design) | `mhd/test/mhd.test.ts` (F, T12) | [x] |
| T-quench-cp | quench end temperature with C(T) of the winding | well below room temperature for this coil | `mhd/test/mhd.test.ts` (F, T12) | [x] |
| T-friction | tower Coulomb friction | ‖f_t‖ ≤ μ F_n, opposite the slip | `tower/test/tower.test.ts` (F, T12) | [x] |
| T-tower-contact | solid tower/arm collision | hull cannot pass through the column, carriage, arm tops/tips or mount; closing arms squeeze an off-centre hull | `tower/test/tower.test.ts` 11 pass (column push-out, arm-top support, closing-arm squeeze); `sim/src/step.ts` contact in every phase within 260 m / 300 m | [x] |
| T-wall-energy | PICA/steel conduction energy balance | stored = absorbed − re-radiated within 0.5 % | `heat/test/heat.test.ts` (F, T12) | [x] |
| T-sim-det | step function determinism, norm(q) = 1, no hidden forces | bit-identical reruns; within 5 m of an independent J2 RK4 | `sim/test/sim.test.ts` | [x] |

## 2. Predictor and guidance validation

| ID | Check | Acceptance | Evidence | Status |
|---|---|---|---|---|
| G-pred | 3-DOF predictor vs 6-DOF over 60 s | position error < 5 km, speed < 30 m/s | `scripts/predacc.ts`: 1–5 km, 2–33 m/s | [x] |
| G-npc | footprint centre at Mach 2 | < 2 km from the pad | `scripts/plandbg.ts`: ≈ 1 km | [x] |
| G-belly | flip-point miss | < 500 m | 674–702 m at the 2.0 km gate (`mission_verify.log`); 1.3 km on the honest entry at the earlier gate | [ ] |
| G-land | landing burn to the hover point | < 3 m, < 1.5 m/s, ≥ 1 t propellant | not met (F1) | [ ] |

## 3. Mission-level acceptance

| ID | Check | Acceptance | Evidence | Status |
|---|---|---|---|---|
| T-mission | full return | outcome `caught`; steel < 1150 K; bondline < 560 K; no quench; N₂ > 0 | `sim/test/sim.test.ts` (SLOW=1) checks steel, quench and N₂; its bondline assertion (< 560 K) fails at the measured peak, which is the honest result; outcome `lost` (F1) | [ ] |
| T-bondline | PICA-X bondline peak after the energy-conserving conduction | < 560 K (adhesive limit, Q21) | latest sweeps: **695 K**; 14:08 integrated run: **701 K** (`.tmp/mission_verify.log`); earlier 99 m/s run: 614 K | [ ] outside limits (accepted per Q21, reported) |
| T-tile | tile heat shield thermal model (Q23) | tile layer stores + radiates + conducts out what it absorbs within 0.5 %; tile surface < **1673 K** (SpaceX flight-tile claim, user decision 2026-11-21; NASA LI-900 1530 K reported as margin); steel backing < 1150 K (adhesive < 600 K) | `heat/test/tile.test.ts` energy test passes; latest sweeps: surface **1650 K** (passes 1673 by 23 K; over NASA 1530 by 120 K), backing **363 K**; 14:08 integrated run: surface **1684 K** (fails 1673 by 11 K), backing **345 K**; **35 s above 1530 K**, at the pitch-up plateau (Q30 full-field replay; the earlier 246 s applied the stagnation pulse uniformly to every cell and is superseded); ε = 1 would still give 1664 K | [ ] best trajectory passes the flight-tile claim (+23 K), the 1684 K run does not; the bondline is the remaining thermal fail; the flux must fall 1.22× vs the claim or 1.75× vs LI-900; extend-PICA buys 0 K (Q30) |
| T-actuators | R11c post-flip actuation | no cold gas after the flip; ≥ 2 sea-level Raptors with differential gimbal | `bench/02` and the direction-gate sweeps: 2 SL Raptors lit from the flip; 0 kg N₂ after the flip; coil discharged | [x] |
| T-catch-envelope | tower catch with wind (Q34) | both pins on the rails: axis within ≈±1 m of the tower centreline, contact ≤1 m/s, **arms closed**; positive wind margin | **FALSE CATCH retracted 2026-11-21** (arms open, 12.8 m off). Integrated mission: pins at the rail plane **11 m off**, climbing 3.0 m/s, arms open, `lost`. Best 6-DOF optimiser candidate: 6.8 m lateral / 11.7 m/s, `lost`; later sweeps all `lost`. | [ ] not caught |
| T-mc | Monte Carlo (wind, κ, B, mass ±5 %) | ≥18/20 caught within the envelope | **not run**; a single nominal run does not close T09 | [ ] pending |
| T-fidelity | review/00 audit per change | no plant-state writes outside integrators; every force modelled; no physics disabled per phase; design changes recorded | review/00 | [x] |

## 3.1 Run procedure (2026-11-21)

Every check that runs the mission, the profile script or a headless browser follows
`docs/05-benchmarks/00-machine-budget.md`: heavy commands under `nice -n 19`, one mission or profile run at a
time, one headless browser at a time. The CPU is the limit; a starved OpenCode server restarts and loses the
running tools.

## 4. UI acceptance (spec/04)

| ID | Check | Status |
|---|---|---|
| U-load | worker run completes in the browser, playback unlocks | [x] screenshots 2026-11-21 |
| U-scrub | drag the playhead / track scrubs both directions | [x] |
| U-explode | checkbox pauses, springs apart, springs back, resumes | [x] |
| U-tree | eye toggles hide/show parts and overlays | [x] |
| U-text | crisp text at DPR 1–3 | [x] |
| U-pan | right-drag pans, left-drag orbits, context menu suppressed | [x] `view/src/index.ts`; review/02 §5 |
| U-altgraph | transparent altitude graph top-right with live marker | [x] `app/index.html#altgraph`, `app/src/main.ts`; review/02 §5 |
| U-path | predicted-path overlay with tree toggle | [x] `view/src/prediction.ts`; `app4-path-off/on.png` |
| U-clouds | global cloud shell + local deck over the descent and at the horizon | [x] `final.log`, `final2.log`, C4 screenshots |
| U-realism | spec/04 A4 items | [~] clouds/coast/UI verified; plasma/plume/bow-shock visual re-check pending |
