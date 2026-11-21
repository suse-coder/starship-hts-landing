# plan/01 — Decisions (ADR) and trade studies

Format per record: **context → options → criteria (weights) → scores (1 = poor … 5 = excellent) →
decision → consequences → evidence**. A weighted total is Σ wᵢ·sᵢ / Σ wᵢ. Evidence cites
journal entries (`E##`), scripts, or review findings. Status: ✅ accepted · 🔄 accepted, under
evaluation · ⛔ superseded.

Index

| ID | Topic | Decision | Status |
|---|---|---|---|
| ADR-00 | Fidelity rule | real physics only; honest failure beats faked success | ✅ |
| ADR-01 | Monorepo layout | `packages/` by physical domain, source-first | ✅ |
| ADR-02 | Programming model | pure functional core, imperative shell | ✅ |
| ADR-03 | Loops in numeric kernels | allowed inside pure kernels | ✅ |
| ADR-04 | Language / runtime | TypeScript (Bun + browser), no effect-ts in the hot loop | ✅ |
| ADR-05 | Thermochemistry | equilibrium air from partition functions, tabulated | ✅ |
| ADR-06 | Concept variant | MHD-trimmed nose-first glide → pitch-up → high-α → belly flop | ✅ |
| ADR-07 | MHD actuator | 8 electrode sectors, box-constrained LS allocation | ✅ |
| ADR-08 | Predictor consistency | predictor uses the flight models and allocator tables | ✅ |
| ADR-09 | Free axes | never fight passive aero (roll, α-axis) | ✅ |
| TS-01 | **Attitude control law** | rate-command + free axes + per-axis time-optimal limits | 🔄 |
| TS-02 | **Landing-burn guidance** | linear MPC (QP, FISTA) | 🔄 |
| TS-03 | **Entry / high-α guidance** | numerical predictor–corrector, planned single reversal | ✅ |
| TS-04 | **Belly-flop steering variable** | bank σ (shallow) → ground azimuth ψ (steep), lift transport | ✅ |
| TS-05 | MHD allocation solver | box-constrained least squares | ✅ |
| TS-06 | Time integrator | fixed-step RK4 per phase | ✅ |
| TS-07 | Aerodynamics model | modified Newtonian panels + Jorgensen crossflow, tabulated | ✅ |
| TS-08 | Rendering engine | three.js (WebGL2) | ✅ |
| TS-09 | Vehicle 3D model source | procedural from the physics OML (no imported mesh) | ✅ |
| TS-10 | Delivery | static GitHub Pages via GitHub Actions | ✅ |
| TS-11 | Turbulence model | Dryden (MIL-F-8785C) | ✅ |
| TS-12 | Landing engine | choose the SL Raptor whose radial axis matches the divert | 🔄 |
| TS-13 | **Sky and atmosphere rendering** | analytic sky near the ground + single-scattering ray march (Rayleigh/Mie) from altitude | ✅ |
| TS-14 | **Clouds** | global cloud shell + ray-marched volumetric cumulus deck (Perlin–Worley, Beer–powder, HG) in the pad frame | ✅ (T02, verified 2026-11-21) |
| TS-15 | **Landing MPC plant model** | double integrator + first-order thrust-pointing lag τ | ✅ |
| TS-16 | **Steel survival criterion** | EN 1993-1-2 yield-retention curve k(T) + 1150 K design band | ✅ |
| TS-17 | **Glow / bloom** | HDR threshold bloom above lit surfaces (emitters only), sky clamped | ✅ |
| TS-18 | **Recording cache** | key = hash of the physics sources (comments and whitespace removed) + design parameters; IndexedDB | ✅ |

---

## ADR-00 — Real physics only (user, 2026-11-20) ✅
Context: the success criterion (a catch) must never be reached by non-physical means.
Decision: spec/00 G0 rules R1–R10: plant state changes only by integration; every force has a
model and a real actuator; limits come with their mass/power cost; idealisations are listed
(review/00); failures are reported. Consequence: guidance fixes are allowed (R6), plant fixes only
when more faithful. Evidence: review/00 F2, where a non-physical "lift floor" was found and removed.

## ADR-01 — Monorepo by physical domain ✅
Options: (a) single package, (b) packages by layer (models / control / ui), (c) packages by
physical domain (math, earth, gas, craft, aero, heat, mhd, tower, gnc, sim, view, app).
Criteria: dependency clarity (3), testability (2), reuse in the browser (2), build simplicity (1).
Scores: a 2/3/3/5 → 2.9 · b 3/3/4/4 → 3.4 · **c 5/5/5/3 → 4.8**. Decision (c), after foldkit,
source-first exports, Bun isolated linker so undeclared imports fail.

## ADR-02 / ADR-03 — Pure core, loops in kernels ✅
`step: (World, SimState) → SimState` is a fold. Kernels (panel integrals, tridiagonal heat solves,
MHD cell sums, QP/LS solvers) use local loops over typed arrays and are pure at their boundary;
this measured 5–20× faster in V8 than combinator chains.

## ADR-04 — Language / runtime ✅
| Option | Browser-native (3) | Perf (3) | Dev speed (2) | One codebase (2) | Total |
|---|---|---|---|---|---|
| **TypeScript (Bun/V8)** | 5 | 4 | 5 | 5 | **4.7** |
| Rust → WASM | 4 | 5 | 2 | 3 | 3.7 |
| Python (Pyodide) | 2 | 1 | 4 | 3 | 2.3 |
| C++ → WASM | 4 | 5 | 2 | 3 | 3.7 |
Evidence: a full mission takes 26–28 s in headless Chromium and 10–20 s in Bun (journal 2026-11-21).

## ADR-05 — Thermochemistry ✅
Options: perfect gas γ = 1.4; curve fits (Tannehill); **equilibrium from partition functions,
tabulated**; finite-rate chemistry. Equilibrium is accurate below ≈ 70 km at entry speeds; non-equilibrium
matters above that, where MHD and heating are small (map/20). Finite-rate costs 100× more and
needs CFD coupling.

## ADR-06 — Concept variant ✅
The literal "nose-down to the ground" is infeasible: 20–150 kPa at the end of the plasma phase, and
10–30 g to flip (research/04). The physical variant keeps every hard requirement.

## ADR-07 — MHD actuator ✅
The main coil is persistent (64.6 H, so it cannot be modulated at control rates); electrode loading
K_s per sector changes in µs. Allocation: TS-05.

## ADR-08 — Predictor consistency ✅
Evidence E10: a symmetric-loading table mis-predicted range by ~700 km; with the flight allocator
the downrange error is ≈ 0 through entry.

## ADR-09 — Free axes ✅
Evidence E12: removing torque about the passively stable α axis saved ≈ 1 t of N₂.

---

## TS-01 — Attitude control law 🔄
**Context.** Actuators differ by orders of magnitude and are saturating and anisotropic: MHD
electrodes (pitch/yaw, only with plasma), cold-gas couples (156 kN·m pitch/yaw, 36 kN·m roll), Raptor
gimbals (MN·m, but one off-axis engine makes roll when tilting tangentially). Inertia Ix 1.4e6,
Iy = Iz 3.6e7 kg m². The user observed **wobble during the descent** (coning in the landing burn,
journal E30) and asked for a better controller such as MPC.

**Options.**
1. PD rate-command with vector saturation (previous baseline).
2. LQR on linearised rigid-body + actuator model, gain-scheduled by phase / q̄.
3. INDI — incremental nonlinear dynamic inversion (uses measured angular acceleration; robust to
   aerodynamic-model error).
4. Sliding-mode control.
5. Linear MPC on attitude (QP per step with actuator limits).
6. Rate-command + **per-axis explicit time-optimal limit** ω_c,i ≤ 0.8·√(2 a_i |e_i|). This is the
   closed-form constrained-MPC law of a bounded double integrator, with a_i the real per-axis
   authority.

**Criteria (weight):** handles saturation + anisotropy (3), robustness to model error (3), CPU at
100 Hz × 70 000 steps in a worker (2), smoothness / no limit cycles (3), verifiability (2), tuning
effort (1).

| Option | Sat./aniso. | Robust | CPU | Smooth | Verif. | Tuning | Total |
|---|---|---|---|---|---|---|---|
| 1 PD rate-command | 2 | 3 | 5 | 2 | 4 | 4 | 3.0 |
| 2 LQR scheduled | 2 | 3 | 5 | 4 | 4 | 3 | 3.4 |
| 3 INDI | 3 | 5 | 4 | 4 | 3 | 3 | 3.8 |
| 4 Sliding mode | 3 | 4 | 5 | 1 | 3 | 2 | 3.0 |
| 5 Attitude MPC | 5 | 3 | 2 | 5 | 3 | 2 | 3.6 |
| **6 Rate-cmd + per-axis time-optimal** | **4** | **4** | **5** | **4** | **4** | **4** | **4.1** |

**Decision.** Option 6 now (implemented, `axisAccel` in `GAINS`), with the anisotropy of the
landing engine computed from the engine actually lit (TS-12). Allocation enforces the roll
priority (`allocateGimbal` projects onto |τx| ≤ RCS capacity). **INDI is the next candidate** if
aerodynamic-model error becomes the limiting factor.
**Evidence.** Landing trace before/after (`scripts/landtrace.ts`): coning with roll rate 25–30°/s,
tilt azimuth sweeping 360° → rates < 6°/s, tilt held at the 25° command limit, no coning.

## TS-02 — Landing-burn guidance 🔄
**Context.** After the flip: 0.7–1.7 km horizontal offset, 60–80 m/s horizontal speed, a single
off-axis Raptor (thrust 1.0–2.45 MN, tilt ≤ 25°), ≈ 300 kg/s propellant, a slow weak axis.
**Options.**
1. Velocity-profile law (√(2 a d) toward the target).
2. ZEM/ZEV (energy-optimal feedback, analytic).
3. Apollo polynomial guidance.
4. Convex G-FOLD (lossless convexification, SOCP, fuel-optimal, free final time).
5. **Linear MPC**: receding horizon, QP solved by FISTA, rate cost, constraints.
6. Nonlinear MPC (full attitude + thrust).

**Criteria:** constraint handling (3), smoothness / no wobble (3), propellant (3), robustness to
disturbances (2), CPU (2), implementation risk (1).

| Option | Constraints | Smooth | Propellant | Robust | CPU | Risk | Total |
|---|---|---|---|---|---|---|---|
| 1 Velocity profile | 2 | 2 | 3 | 3 | 5 | 5 | 3.0 |
| 2 ZEM/ZEV | 1 | 3 | 3 | 3 | 5 | 5 | 3.0 |
| 3 Apollo polynomial | 1 | 3 | 3 | 2 | 5 | 4 | 2.8 |
| 4 G-FOLD SOCP | 5 | 4 | 5 | 3 | 2 | 2 | 3.9 |
| **5 Linear MPC (QP)** | **4** | **5** | **4** | **4** | **4** | **4** | **4.2** |
| 6 Nonlinear MPC | 5 | 5 | 5 | 4 | 1 | 1 | 3.9 |

**Decision.** Linear MPC (`packages/gnc/src/mpc.ts`): 25 steps over the remaining time; terminal
+ rate + effort costs; thrust-range and tilt-cone constraints; weak-axis slew penalty; soft glide
slope. 0.3–1 ms per solve at 20 Hz. G-FOLD stays the reference for a later fuel-optimality check.
**Evidence.** ZEM/ZEV saturated the tilt at the end and arrived at 19–25 m/s (journal E28). The
velocity profile looped 340 m past the tower. MPC plain-model test (`.tmp/scripts/mpctest.ts`):
a 1.2 km divert ends within 0.2 m at the planned 32 s, ∫|u|dt = 41 s hover-equivalent. In the full
6-DOF run (2026-11-21) it is smooth and controlled but still overshoots (F1).

## TS-03 — Entry and high-α guidance ✅
**Options.** Apollo reference tracking; Shuttle drag-velocity tracking; FNPEG-style numerical
predictor–corrector (NPC) with deadband reversals; **NPC with planned single reversal**
(|σ| for downrange, t_rev for crossrange).
**Criteria:** accuracy at the handover (3), robustness to dispersions (3), respects the slow lift
rotation ≈ 2°/s (3), CPU (1).
Scores: Apollo 2/2/2/5 → 2.3 · Shuttle 3/3/2/5 → 3.0 · NPC + deadband 3/4/1/4 → 2.8 ·
**NPC planned reversal 5/4/5/3 → 4.5**.
**Evidence.** E17–E19: deadband reversals need 40–50 s slews (≈ 3.5 km of crossrange per second
of timing). The planned reversal with a 5 s lead and a bias toward the correctable side delivers
the footprint centre within ≈ 1 km.

## TS-04 — Belly-flop steering variable ✅
**Options.** Lift heading ψ throughout; bank σ throughout; **σ while shallow, ψ when steeper than
−60°, both with physical lift transport** (the lift is carried with v̂ for free; only rotation about
v̂ is actuated).
**Evidence.** ψ throughout is ill-conditioned at shallow γ (a 70° rotation looks like 180°, E19).
σ throughout lets the reference spin at steep γ (E21). The hybrid with lift transport is consistent
between predictor and command.

## TS-05 — MHD allocation solver ✅
Options: pseudo-inverse; box-constrained least squares (projected Gauss–Seidel); QP with
power limits. The box-LS realises a commanded moment within 1 % when authority suffices (T-alloc)
at ≈ 20 µs per call. A QP adds power limits not yet binding (electrode power ≤ 70 MW recorded).

## TS-06 — Time integrator ✅
Options: RK4 fixed per phase (0.5 s vacuum → 4 ms catch); Dormand–Prince 5(4) adaptive;
symplectic leapfrog. Discontinuous actuators (valve duty, phase switches) make adaptive stepping
chatter; RK4 conserves energy to 1e-8 per orbit (T-orbit) and gives fixed, deterministic
recording times.

## TS-07 — Aerodynamics model ✅
Options: modified Newtonian + Jorgensen crossflow on panels (tabulated); HABP panel methods;
DSMC (rarefied); CFD (Navier–Stokes). Real-time browser execution rules out DSMC/CFD. Uncertainty
± 10–20 % in C_N/C_m at transonic speed is listed as idealisation I3.

## TS-08 — Rendering engine ✅
| Option | Ecosystem (3) | Post-processing (2) | WebGPU path (1) | Bundle (1) | Total |
|---|---|---|---|---|---|
| **three.js** | 5 | 5 | 4 | 4 | **4.7** |
| Babylon.js | 4 | 5 | 4 | 2 | 4.0 |
| raw WebGPU | 1 | 2 | 5 | 5 | 2.3 |

## TS-09 — Vehicle 3D model source ✅
Options: (a) import a third-party model (BlendKit "Starship" by Niels Van Ingen, royalty-free,
262k polys; GitHub repos); (b) **procedural from the physics OML + component list**.
(a) is excluded: the licence does not allow redistributing the raw asset (a public Pages site
would); the 2023 Block 1 has flaps and tiles; the shape would differ from the simulated OML (R2).
Decision (b), at a target of 150–250 k triangles with components from research/05.

## TS-10 — Delivery ✅
Static GitHub Pages built by GitHub Actions (tests in CI, no build output in git) rather than a
`gh-pages` branch.

## TS-11 — Turbulence ✅
Dryden (rational transfer functions, cheap to discretise) vs von Kármán (irrational spectra).
Dryden: MIL-F-8785C light, seeded and deterministic.

## TS-12 — Landing engine selection 🔄
Context: one of three SL Raptors (at 90°/210°/330°, r = 1.3 m) stays lit; its radial gimbal
direction is roll-free, its tangential one costs roll. Options: always RSL1; **choose by the
zero-effort-miss direction**; roll the vehicle to align the engine with the divert (needs up to
60°, ≈ 15 s of cold-gas roll). Decision: choose by ZEM at flip end; roll-to-steer left as an
option if F1 needs it.

**Engine count (user question, 2026-11-21).** Real flights: Flight 13 flew its landing burn on all
three sea-level Raptors (Wikipedia "SpaceX Raptor"); SN9+ used all three for the flip. Benefit:
symmetric thrust, differential gimballing controls roll, no weak axis. Implemented:
`allocateGimbals` (minimum-norm per-engine gimbals over the 3 torque equations, 2N unknowns).
**Physics limit:** three Raptors at the 40 % minimum give T/W ≈ 2.5–2.7 at ≈ 110 t. Even at 50° tilt the
vertical component (16 m/s²) exceeds g, so the ship climbs (6-DoF run: climbed to 1.9 km, propellant
exhausted). Three engines can only decelerate (hoverslam), as in the splashdowns. A slow approach
into the arms needs one engine. Schedule: flip + deceleration on 3 (differential gimbal) → 1 engine
for the approach (`LANDING_ENGINES` = 1). The jointly planned 3 → 1 sequence and divert is part of
the SCvx task (M3).

## TS-13 — Sky and atmosphere rendering ✅
**Context.** The ship is seen from the ground, from inside the atmosphere (10–90 km) and from orbit.
The first version (Preetham sky + an additive "rim" shell) looked wrong at altitude: from 60 km the
whole upper sky was light blue (the camera sat inside the rim shell), and lowering the Rayleigh
coefficient with altitude made the Preetham sky white, not black (screenshots 2026-11-21).
**Options.**
1. Preetham/Hosek analytic sky everywhere (three/addons Sky).
2. Rim shell (Fresnel hack) for the limb.
3. **Single-scattering ray march** (Nishita 1993 / O'Neil GPU Gems 2 ch. 16): per pixel, 16 view
   samples × 6 light samples, Rayleigh β = (5.8, 13.5, 33.1)·10⁻⁶ m⁻¹, H_R = 8 km, Mie β = 21·10⁻⁶
   m⁻¹, H_M = 1.2 km, g = 0.76, analytic Earth shadow on the light path.
4. Precomputed multiple scattering (Bruneton & Neyret 2008; Hillaire 2020 LUTs).

**Criteria:** correct from ground, inside and orbit (3), sunset/terminator colours (2), cost (2),
implementation risk (1).

| Option | Correct | Colours | Cost | Risk | Total |
|---|---|---|---|---|---|
| 1 Preetham | 2 | 3 | 5 | 5 | 3.4 |
| 2 Rim shell | 1 | 1 | 5 | 5 | 2.3 |
| **3 Single scattering + Preetham near the ground** | **4** | **4** | **3** | **4** | **3.8** |
| 4 Bruneton/Hillaire LUTs | 5 | 5 | 4 | 2 | 4.4 |

**Decision.** Option 3 now (Preetham carries the low-altitude sky and fades as e^{−h/8 km}; the ray
march weight ramps in from 5 to 15 km). Option 4 is the upgrade path if multiple scattering
(brighter twilight, Earth-shine) is needed. **Evidence:** `.tmp/shots/r-0.62.png` (black sky,
blue limb band, hazy Earth at 60 km), `r-0.05.png` (orange–blue terminator band from orbit).

## TS-14 — Clouds 🔄
**Context.** The sky shader's cloud layer is a 2D texture projected on the sky dome. It cannot
look volumetric. In the first version it also faded in between 4 and 2 km, which looked like a
grey flash during the descent (`.tmp/pw/flicker.cjs`, frames 32→33, mean luminance 163 → 138).
**Options.** (1) Sky-dome 2D layer. (2) Billboard/impostor puffs. (3) **Ray-marched volumetric deck**
(Schneider & Vos 2015 "Nubis"; Hillaire 2016): 3D Perlin–Worley texture, coverage map, height
gradient, Beer–Lambert + powder, dual-lobe Henyey–Greenstein, light march toward the Sun.
(4) Voxel/VDB clouds.
**Criteria:** realism from below, inside and above (3), no popping (3), cost (2), risk (1).
Scores: 2D layer 1/2/5/5 → 2.6 · billboards 2/3/4/4 → 3.0 · **volumetric 5/5/3/3 → 4.3** ·
VDB 5/5/1/1 → 3.7.
**Decision.** Volumetric deck fixed in the pad frame (base ≈ 1.5 km, top ≈ 3.2 km), no altitude
switches. Implemented in T02 (`view/src/clouds.ts`), integrated in T02i.

## TS-15 — Landing MPC plant model ✅
**Context.** The 6-DoF landing showed the planned thrust direction changing faster than the vehicle
can re-point (single off-axis Raptor: ≈ 0.12 rad/s² strong axis, 0.018 weak axis) — tilt overshoot
to 46° against a 25° command, a looping overshoot of 340 m.
**Options.** (1) Double integrator, instantaneous thrust (old). (2) **Double integrator + first-order
lag of the realised thrust acceleration**, a⁺ = (1−β)a + βu, β = 1 − e^{−h/τ}, a₀ measured.
(3) Full attitude in the plan (SCvx 6-DoF, Szmuk & Açıkmeşe 2018; SCpp).
Scores (constraint realism 3, CPU 2, risk 2, effort 1): 1 → 2.6 · **2 → 3.9** · 3 → 3.6 (best
model, highest risk; planned for M3).
**Decision.** (2) now, keeps the QP and FISTA (projection stays separable on u); τ = 2.0 s (one
engine), 1.2 s (three), 0.8 s terminal. **Evidence:** `packages/gnc/test/mpc.test.ts` — on a plant
with a real 2 s lag the τ-aware plan lands the 1.2 km divert within 1.8 m vs 5.2 m τ-blind. The
6-DoF mission is still `lost` (51.5 m/s at T+3110, flip kick 80 m/s with 1.16 km to go): the root
cause is the delivery + single-engine authority, kept for T09 (SCvx, last).

## TS-16 — Steel survival criterion ✅
**Context.** The user asked to see where the 301/30X steel loses strength.
**Options.** (1) Single design temperature (1150 K). (2) **Yield-retention curve k(T)** for austenitic
stainless (EN 1993-1-2 Annex C, grade 1.4301: k = 1.00 at 20 °C, 0.60 at 400 °C, 0.40 at 700 °C,
0.14 at 900 °C, 0.03 at 1100 °C) with bands 700/900/1150/1400 K. (3) Full creep/oxidation model
(Larson–Miller).
**Decision.** (2) in `heat/src/limits.ts` (one source of truth for the views, the results and the
tests); (3) listed as idealisation. Values to verify against the standard text (open question Q18).

## TS-17 — Glow / bloom ✅
**Context.** A global bloom with a low threshold (0.82) put a white veil over every daylight frame,
and the Sun disc of the analytic sky (≈ 10³ × the sky radiance) produced a huge halo.
**Options.** (1) Global low-threshold bloom. (2) **HDR-threshold bloom above lit surfaces (2.5) +
sky radiance clamped (≤ 3)**. (3) Selective bloom (separate emissive layer pass).
**Decision.** (2); (3) if emitters ever need bloom below the surface level. Plumes no longer use a
30 Hz flicker term (a source of the white flicker in descent).

## TS-18 — Recording cache ✅ (user, 2026-11-21)
**Context.** Every edit of the view or the app re-ran the whole mission in the browser: about 40 s to 2 min, depending on machine load. The user asked to keep the last run and re-simulate only after a real physics change.
**Options.** (1) No cache. (2) **IndexedDB keyed by a hash of the physics sources** (chosen). (3) A pre-simulated recording committed to the repository: about 20 MB, stale after every physics edit, and it conflicts with "simulate on load". (4) Precompute at build time in Vite: the same staleness, and longer CI.
**Decision.** (2). A Vite plugin hashes the sources of the physics packages after comments and whitespace are removed, so documentation-only edits keep the cache. The key also contains the design parameters. `?resim` and the Re-simulate button force a new run.
**Evidence.** Cold and warm load timings are pending the reload test (T15), which runs after the physics agents finish.
