# Open questions — what I would ask you, and the default the engine runs with

Every default can be changed in the browser's **Design** panel (re-simulates in seconds) or in
`packages/craft/src/config.ts → DEFAULT_DESIGN`. "Conservative" is the value a sceptical
reviewer would pick; the engine reports honestly whether the mission still closes.

| # | Question | Default (why) | Conservative alternative | Affects |
|---|---|---|---|---|
| Q1 | The literal "nose-down to the ground, then flip" concept is physically infeasible (research/04 §1). Accept the physical variant (MHD nose-first glide → pitch-up → high-α → belly flop → flip)? | **Yes** — it keeps every hard requirement: no flaps, coil-only pitch/yaw in the plasma phase, PICA-X nose, cold gas in orbit, flip + catch. Q23 later keeps the real tiles on the barrel. | keep tiles on the belly (real Starship) | everything |
| Q2 | Orbit inclination of Flight 15 (not published) | **26.5°** (due-east launch from 26°N; ≥ 26.0° to overfly the pad) | 26.1° / 32° | ground track, crossrange |
| Q3 | Coil field at the stagnation point | **2.0 T** (MEESST target upper end; 291 MJ, 4.15 t system) | 1.0 T (closer to the 0.66 T demonstrated) | MHD authority, mass |
| Q4 | Hall-current closure κ in σ_eff = σ/(1+κβ²) | **0.03** — insulating, finely segmented wall that blocks most meridional current (optimistic) | 0.15 (authority then falls below demand) | whether the nose-first glide is controllable |
| Q5 | Caesium seeding fraction (mole fraction in the inner shock layer) | **1 %**, 400 kg carried, ≈ 225 kg used | 0.3 % or potassium | conductivity, β, plasma window |
| Q6 | Nose radius of the blunted ogive | **2.5 m** (stagnation 93 W/cm² peak; coil fits) | 1.5 m (sharper, hotter) | heating, drag, coil size |
| Q7 | PICA-X thickness | **6.5 cm cap → 2.5 cm aft** (bondline only 371 K: generous) | 4 cm → 2 cm (saves ≈ 1.3 t) | mass, bondline margin |
| Q8 | REBCO engineering current density at 25 K, ~14 T | **700 A/mm²** (thin-substrate tape, optimistic) | 400 A/mm² (+1 t of tape) | coil mass |
| Q9 | Cold-gas system | **5.0 t N2**, 12 thrusters × 4 kN (Isp 73 s), COPVs 0.6 kg per kg of gas — about 15× the real ship's capacity (raised 2026-11-20, see Answered) | hot-gas methalox RCS (Isp ≈ 250 s, if you allow it) | roll/heading control endurance |
| Q10 | Barbecue roll to spread heat on bare steel | **on** (4°/s nose-first > 10 kPa, 6°/s high-α > 1.3 km/s) | off (steel windward exceeds 1150 K) | steel temperature, gas |
| Q11 | Where Ship 42 starts | **in orbit, already in burn attitude, 8 min before the deorbit burn** (the 22 h of coasting add nothing) | start a full orbit earlier | run time only |
| Q12 | Pad 2 coordinates | **25.9925 N, 97.1573 W** (public maps, ±200 m) | your surveyed value | catch point |
| Q13 | Wind | **November Gulf climatology + light Dryden turbulence** | moderate turbulence, 1.5× mean wind | landing precision |
| Q14 | Structural load limit | **not enforced**, reported (peak ≈ 6 g at pitch-up) | stop the run above 5 g lateral | outcome |
| Q15 | Catch hardware | **two pins at x = 35 m (just below the nose cap) on the arms' top rails at 112 m** | pins in the nosecone like the real ship | approach geometry |
| Q16 | Steel limits | design 1150 K, failure 1400 K | 1050 K design | outcome |
| Q18 | Steel yield retention k(T) values (TS-16) | **EN 1993-1-2 Annex C, 1.4301, as recalled** (k 0.60 @ 400 °C, 0.40 @ 700 °C, 0.14 @ 900 °C) | your copy of the standard / 301 cold-worked data | heat-survival verdicts |
| Q19 | Dump resistor in the coil fast-discharge mode (agent F, T12) | **1.37 t resistor in the vehicle mass** (committed 2026-11-21) | graphite resistor 0.65 t (agent F's figure), or no fast-discharge hardware at all | the 1.37 t cuts the landing margin: impact 57.8 m/s instead of 41 m/s in the same run |
| Q20 | Dump voltage against the insulation class | **τ = 2 s gives 97 kV** (V = I·R) against an assumed 3 kV insulation limit (the 3 kV is our assumption, not sourced) | R = 1 Ω and τ ≈ 65 s keep V ≤ 3 kV, but the dump then takes about 5τ ≈ 5 min; or segmented high-voltage insulation (mass, cost) | decides whether a fast dump exists at all |
| Q21 | PICA-X bondline against the 560 K limit | **6.5 cm cap / 2.5 cm flank (Q7): peak bondline 633 K** (653 K with the dump mass), above the limit after the energy-conserving conduction fix | thicker PICA (base 4.62 t; +1 cm on the cap costs +0.35 t, +1 cm on the flank +0.55 t; bondline sensitivity per cm not yet computed); or accept "outside limits" and report it (R8); first check where the peak sits (flank or cap) with `.tmp/agentF/bondline.ts` (about 2 min, not run) | the thermal limit is violated; T-bondline stays open |
| Q22 | Switch-off method for the coil (R11b) | **battery ramp-down at P_max**, switched off when shielding and steering no longer need it (T13) | fast dump (Q19, Q20) | coil energy and current stay on the timeline until the physical switch-off |
| Q17 | Use the effect-ts library for the engine? | **No** for the hot loop (plain TS with pure functions is far faster per step and keeps the core dependency-free); the browser shell could adopt effect-ts later | effect-ts everywhere | architecture |

## Answered
- **2026-11-20, user:** all real physics must be followed and simulated; nothing may be fixed with
  fake rules or physics to make it work; it must be really realistic, including the
  superconducting (HTS-like) coil that steers and stabilises the ship instead of flaps. →
  spec/00 G0 (R1–R10), ADR-0, review/00, spec/05 gate T-fidelity.
- **2026-11-21, user:** UI like Fusion 360 (with ANSYS-like simulation results), without the
  ribbon: a 3D view with an overlaid component tree with visibility toggles (left), a timeline
  with drag-to-scrub, play/stop/speed (bottom), and an exploded-view checkbox that pauses
  playback, springs the parts apart, springs them back and resumes. → spec/04 A3.
- **2026-11-20, user:** browser only (GitHub Pages, no server-side code); public repo created with
  `gh` once everything works; simulate everything in a worker on page load with a loading
  animation, then slider + play to scrub fast. → spec/04 A0–A2, spec/00 G4b, spec/11 release gate. Open detail
  (default): repo name **`flapless-starship-catch`**, deploy via GitHub Actions.
- **2026-11-20, user:** HTS assumptions "optimistic but realistic" → Q3, Q4, Q5, Q8 defaults.
- **2026-11-20 (design change, R5):** Q9 cold gas raised to **5.0 t N2, 12 × 4 kN** (flapless
  attitude control in the belly flop needs it; +1.2 t COPVs carried). Landing propellant 16 t.

## Decided by the user (2026-11-21)

| ID | Decision | Consequence |
|---|---|---|
| Q21 | **Keep the current PICA-X design (6.5 cm cap, 2.5 cm flank).** The bondline is reported as outside the 560 K limit (peak 633 K) and is not clipped (R8). A thicker PICA stays available as an alternative (+0.35 t per cm on the cap, +0.55 t per cm on the flank). | T-bondline stays open and is reported as outside limits. |
| Q22 | **Planned coil switch-off by battery ramp-down at P_max** (energy back to the battery). The fast dump stays only as a fault path, pending Q19 and Q20. | The coil stays on the timeline until the physical switch-off (R11b). |
| Q23 | **All mass the real Starship has (only the flaps removed) is in the simulation.** The concept's replacements (bare steel instead of tiles, PICA-X nose, cold-gas RCS, HTS coil) are counted at their own masses. Every added mass is labelled sourced, computed or assumed, with its conservative alternative (R5). Open sub-question: does "only the flaps removed" keep the tiles? The concept has none (bare steel, PICA-X nose); until you say otherwise the concept holds. **Superseded by the third-round decision below: the real tiles are kept.** | Dry mass rises. Agent M records each gap and its source in spec/01. |
| Q24 | **Propellant is enough for the simulated landing for now.** The header propellant is sized from the measured landing use with at least 30 % margin. The tank volumes follow the consumable rule in spec/07. | A heavier vehicle at the flip; the tank geometry is recorded in spec/07 row 25. |

Still open: Q19 (dump resistor mass, 1.37 t in the vehicle) and Q20 (dump voltage, 97 kV against the assumed 3 kV insulation).

## Decided by the user (2026-11-21, second round)

| ID | Decision | Consequence |
|---|---|---|
| Q25 | **The coil stays on while the plasma steers with it** (so the ship does not tumble during the fall). It is switched off only when the plasma no longer steers enough (steering authority below 1); then the ship flips and the engines take the steering. This replaces the stagnation-flux criterion of the earlier session. | The switch-off comes later than 2292 s, and the bondline heating rate stays lower after it. The flip moves from the 1.7 km trigger to the switch-off, so the header propellant must be re-checked (Q24). |
| Q26 | Cold gas for the roll residual during the coil phase (35–60 % of roll in agent E's run) | **Open.** Options: keep the R11a fallback for the momentum-bound roll shortfall; let roll go free in the coil phase (steel heating risk); or add roll authority (mass and power). |
| Q27 | Phugoid damping gain ζ = 0.3 (agent E) | **Accepted for now.** It removes the high-α oscillation and keeps the flip delivery (1.24 km). To be confirmed. |

## Decided by the user (2026-11-21, third round)

| ID | Decision | Implementation |
|---|---|---|
| Q23 | **Keep the real tiles.** The vehicle is the real Starship minus the flaps. The tile heat shield is in the mass budget and in the thermal model (tile surface and steel backing). Bare steel is no longer the barrel's protection; PICA-X stays on the nose. Reading to confirm: "keep tiles" means the heat shield returns, thermal and mass, not mass only. | Agent M adds the tile mass (through `craft/src/tiles.ts`). Agent T builds the tile thermal model. |
| Q19 | Accepted with the current default: the 1.37 t dump resistor stays in the mass. | No change. The dump is not used in the planned path (Q22). |
| Q20 | Accepted with the current default (τ = 2 s, 97 kV on the fault path). | No change. Not used in the planned path. |
| Q24 | Landing propellant sized from the measured landing, with at least 30 % margin. | Agent M. Re-check after the tile mass and the flip change. |
| Q25 | Coil switch-off by steering authority; the flip follows the switch-off (spec/03 §8). | Agent C2. |
| Q26 | Default: keep the R11a fallback (cold gas for the momentum-bound roll shortfall). | No change. |
| Q27 | ζ = 0.3 accepted. | No change. |

## Session results (2026-11-21, fourth round)

| ID | Status | Evidence |
|---|---|---|
| Q23 | **Implemented.** Tiles are in the mass budget (6.24 t) and the thermal model (6-node layer, backing, channels). | Dry 107.40 t, wet 143.65 t (Cs double count fixed, T08); `heat/test/tile.test.ts`; 99 m/s run surface 1760 K / backing 328 K; latest sweeps 1650 K / 363 K. |
| Q24 | **Open again.** The 26 t header was sized for 137.8 t; the vehicle is now 143.65 t and the entry must be fixed before the landing use is measurable. | The 26 t mission lost in the entry, so no landing margin exists to measure. |
| Q25 | **Implemented.** Authority = allocateMhd max pitch/yaw moment ÷ torqueCommand demand; hold ≥ 1.0; ramp-down after 3 s below 1.0 or plasma gone. Flip gated on `coilOff` + ignition altitude. | Switch-off T+2360.7 s, authority 0.00; `useRcs:false` from the flip. |
| Q26 | Accepted default (cold gas only for the momentum-bound roll shortfall). | Unchanged. |
| Q28 | **Open, narrowed: entry guidance for the 144 t vehicle.** A deorbit sweep found 99 m/s: 7.5 g, 42.6 kPa, 17.7 km long at the flip (105 m/s: 44 g, 60 km short; 95 m/s: 12 g, 246 km long). The deorbit angle is no longer the blocker; the residual 17.7 km is the bank law/predictor authority. Tile surface 1760 K (over 1530 K), bondline 614 K (over 560 K). | `.tmp/mission_dv99.log`, `.tmp/mission_dv98.log`, `.tmp/mission_dv100.log`. |
| Q29 | **New: final-approach engine count.** R11c asks ≥ 2 Raptors; the current terminal hover uses 1 because two Raptors at 40 % give T/W ≈ 1.8 and cannot hover. The honest solution is a two-engine hoverslam with shutdown at the pins (T09). | `LANDING_ENGINES = 1`; N₂ is already zero. |

| Q30 | **Open, mechanism found.** At 99 m/s the tile surface peaks **1760 K**, but at the **pitch-up plateau** (t≈2306 s, α 43°, 51.5 km), not at peak q; the whole windward barrel is hot, so the tile is above 1530 K for only ≈35 s. **Extend-PICA buys 0 K** conservatively (x0 34.5 → 1711 K, still +181/+38 K; +0.38 t). Emissivity cannot fix it (ε = 1 → 1664 K). The absorbed flux must fall **1.75×**: hold the plateau at **α ≲ 22°** or pitch up at **≲ 4.5 km/s**. This is a guidance constraint for Q28, not a TPS mass item. | `docs/03-spec/08-tps-layout-study.md`; `.tmp/agentQ30/`; `.tmp/mission_final.log`. |

| Q31 | **User request (2026-11-21): use the HTS coil as a landing actuator where physics allows, possibly with several coils, for a stable fall.** Findings (research/03 §Low-altitude MHD actuation): (A) natural shock-layer plasma ends at **29–37 km, V ≈ 1.9–2.5 km/s**; 2 km is not reachable. (B) Artificial ionisation of the low-altitude air needs ≈ 10¹¹–10¹³ W against recombination (450 kW available); ruled out. (C) MHD deflection of the seeded Raptor plume with an **aft magnet** is physically possible (≈ 10–100 kN side force per engine vs ≈ 630 kN from the gimbal), research-stage, only while the engines burn. Below 30 km the stable fall comes from passive high-α trim + cold gas, then the gimbals. | **Open — user decision.** Options: (1) keep the current design (the coil works in the 95–30 km corridor only); (2) add an aft-magnet plume-MHD study (mass, power, authority) as a supplement to the gimbals; (3) drop the idea. The earlier "0.5–3 MW" figure for route B omitted recombination and is withdrawn. |
| Q31 | **Decided by the user (2026-11-21): option 2.** Agent P studies an aft-magnet plume-MHD actuator as a supplement to the gimbals (docs/03-spec/09-plume-mhd-study.md). The nose coil is not used in the landing: after the dynamic steering switch-off it is discharged (Q32). | Study running. |
| Q32 | **Coil switch-off method changed (user, 2026-11-21): the coil must be off in the landing.** Planned discharge through the existing 1.37 t resistor bank on a 1 Ω tap (3 kV = insulation limit, τ ≈ 65 s, all 291 MJ into the resistor, ΔT ≈ 400 K as designed). Replaces the Q22 battery ramp-down for the planned path (it left ≈ 2 kA at touchdown); the 2 s tap stays as quench protection. No mass change. | `craft/src/config.ts` `rDischarge`, `mhd/src/coil.ts` mode `discharge`, `mhd/test/mhd.test.ts`. |
| Q33 | **Landing (user, 2026-11-21): engines only, no cold gas after the flip, MPC/convex landing as fast and as controllable as possible, propellant added if needed.** Agent L: ≥ 2 Raptors to the pins, hoverslam, header re-sized from the measured use. | Running (T09 landing part). |
| Q28 | **Resolved to 1.3 km on the honest model (2026-11-21, agent G).** Root causes: (1) the guidance table stored the 80° fallback trim; the bare hull trims at ≈ 28° (L/D 1.24 vs 0.12) → `passiveTrim`; (2) the own-table feed-forward moment was applied about body y only; at a bank it must split into M_y and M_z (`momentProjection`); (3) bank chatter (freeze, 1.9°/s post-coil rate, dwell); (4) the belly planner ended at the wrong flip altitude; AIM_HIGH 1800. Result: flip miss **1307 m** (predicted 1271), 5.76 g, 25.6 kPa, Δv 105, coil discharge 2799 s; the landing reaches 21 m of the pad and the header empties (impact 40.5 m/s). Floor: at the 775 m flip the belly flop moves only ≈ 35 m/s horizontally. Next: belly-flop flattening law (G) and a landing robust to ≤ 1.5 km (L). | `.tmp/agentG/m14_final_ff.log`; 209 tests pass. |
| Q31 | **Closed (2026-11-21): option 2 studied and rejected** (agent P, docs/03-spec/09-plume-mhd-study.md). Raptor exit at 1310 K / 0.83 bar / 3.08 km/s: with 1 % Cs only σ ≈ 0.02 S/m there; the conducting gas is the first metre of the nozzle. Side force ≤ 2.54 kN per engine at 2 T (0.4 % of a 15° gimbal's 634 kN; a plain ring gives zero), 0.99 t magnet + 1.58 t Cs per engine for 40 s, 615 kW plume radiation on the shield, 0.29 rad/s bandwidth. Useful force would need a seeded MHD nozzle stage (an engine programme). The gimbals stay the landing actuator; the nose coil discharges after the steering switch-off. | Closed. |
| Q34 | **Retracted false catch (2026-11-21).** The earlier "CAUGHT" was a single pin grazing an open rail at 12.8 m off; the catch condition now requires `armsClose`. **No true catch.** The 14:08 integrated run is **lost** at the rail plane, **11 m off**, climbing 3.0 m/s; L's 2-engine TVC gains reached 7 m in an earlier run, the optimiser's best candidate is 6.8 m / 11.7 m/s, and the later direction-gate sweeps are 74–231 m off. The 2.0 km gate has +4.8 m/s² lateral authority. The lossless-convex PDG plan (cone + min-throttle slack) closes 674 m to 0 m/0.6 m/s in isolation, 5.8–9 ms at 10 Hz; the integrated closed loop still diverges because the plan lacks the torque-limited attitude state. Next: 6-DOF/rate-limited plan; entry hand-over ≤200–400 m if that is not enough. T-mc not run. | `.tmp/mission_verify.log`; `bench/02`; `review/02`; `.tmp/review/smooth.log`. |
