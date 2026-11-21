# map/30 — Prior art: algorithms, papers, open-source implementations

Question this map answers: **for each sub-problem, what does the literature and open source already
solve, what is usable here (licence, language, fit), and what did we adopt?** Searched 2026-11-21
(GitHub API, repository READMEs and licences read directly). Repositories are third-party code: we
port formulations, never copy-paste without the licence notice. MIT/Apache-2.0 → allowed with
attribution; no licence → read for ideas only.

## 1. Powered descent, flip and landing (M3 / F1)

| Ref | Contribution | Relevance |
|---|---|---|
| Açıkmeşe & Ploen, *Convex Programming Approach to Powered Descent Guidance for Mars Landing*, JGCD 30(5), 2007 | lossless convexification: thrust-magnitude lower bound becomes convex (SOCP), fuel-optimal | core of G-FOLD; our MPC drops the lower bound into a box on u_z (simplification) |
| Blackmore, Açıkmeşe & Scharf, *Minimum-Landing-Error Powered-Descent Guidance…*, JGCD 33(4), 2010 | two-stage min-error → min-fuel | handles infeasible targets (our F1 overshoot case) |
| Açıkmeşe, Carson & Blackmore, *Lossless convexification of non-convex control bound and pointing constraints…*, IEEE TCST 21(6), 2013 | pointing (tilt) constraints remain lossless | justifies our tilt cone |
| Szmuk & Açıkmeşe, *Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time*, AIAA GNC 2018, arXiv:1802.03827 | **6-DoF (quaternion) flip + landing as one problem**, free final time, SCvx | the right formulation for our flip + landing (attitude and translation co-planned) |
| Mao, Szmuk & Açıkmeşe, *Successive Convexification…superlinearly convergent*, arXiv:1804.06539 | SCvx convergence, trust region, virtual control | solver design |
| Szmuk, Reynolds & Açıkmeşe, *SCvx for real-time 6-DoF PDG with state-triggered constraints*, JGCD 43(8), 2020, arXiv:1811.10803 | real-time SCvx, conditional constraints | e.g. glide slope only near the tower |
| Malyuta et al., *Convex Optimization for Trajectory Generation (tutorial)*, IEEE CSM 42(5), 2022, arXiv:2106.09125 | survey + SCP toolbox | reference implementation details |
| Blackmore, *Autonomous Precision Landing of Space Rockets*, NAE The Bridge 46(4), 2016 | SpaceX uses convex optimisation (CVXGEN-generated solvers) on Falcon 9 | industrial precedent for MPC/SOCP landing |
| D'Souza 1997; Guo, Hawkins & Wie, JGCD 2013 | ZEM/ZEV analytic feedback | our former landing law (TS-02 option 2) |
| Klumpp, *Apollo Lunar Descent Guidance*, Automatica 10, 1974 | polynomial guidance | TS-02 option 3 |
| Beck & Teboulle, *FISTA*, SIAM J. Imaging Sci. 2009 | accelerated projected gradient | **our MPC solver** |
| Stellato et al., *OSQP*, Math. Prog. Comp. 2020; Domahidi et al., *ECOS*, ECC 2013; Goulart & Chen, *Clarabel*, 2024 | embedded QP/SOCP solvers (ADMM / interior point) | candidates to port to TS for SCvx (ADMM is simplest) |

Open-source implementations

| Repository | Licence | Language | What it does | Usable? |
|---|---|---|---|---|
| **EmbersArc/SCpp** (S. Niederberger) | MIT | C++ (Eigen, CppAD, Epigraph SOCP) | efficient SCvx, receding-horizon SOCP MPC, LQR; **"SpaceX Starship landing trajectory" example** (RocketQuat: m 140 t, J = 2.77e7/2.77e7/1.32e6, T 2–3 MN, gimbal 15°, glide slope 45°, belly attitude 70° at 500 m, −60 m/s, free t_f ≈ 11 s, K = 30) | **yes, formulation and parameters**: port the SCvx loop to TS with our own SOCP/ADMM; attribution in NOTICE |
| EmbersArc/SuccessiveConvexificationFreeFinalTime | MIT | Python (cvxpy) | reference SCvx free-final-time implementation | yes (cross-check of discretisation) |
| x7k2m9q/starship-belly-flop-landing | Apache-2.0 | Python | 6-DoF belly flop → flip → landing, SCvx guidance, 4-flap allocation, Bouc-Wen actuators, 15-state MEKF, 20/20 Monte Carlo (lands at 3.7 m/s, 13°) | yes for ideas (MEKF nav, Monte-Carlo harness); flaps not applicable |
| BenChung/SuccessiveConvexification | Apache-2.0 | Julia | Szmuk SCvx | cross-check |
| sametuzun781/CT-cSTC | MIT | Python | SCP 6-DoF PDG with continuous-time state-triggered constraints | glide slope / tilt constraints |
| Say43/Rocketlanding_MPC | MIT | Python + web | G-FOLD SOCP re-solved every 0.5 s as MPC, full booster return vs real telemetry | validates the "SOCP as MPC" approach |
| SihaoSun/MPC-rocket-landing-lab | none | JavaScript | MPC landing demo | ideas only |

**Adopted.** TS-02 linear MPC (FISTA) now. **Next (F1): port SCvx 6-DoF free-final-time flip +
landing (Szmuk 2018; SCpp formulation, MIT)** with a TS ADMM SOCP solver, solved once at flip
ignition and re-solved at 1–2 Hz. This replaces the separate flip/landing laws.

## 2. Entry and high-α guidance (M2)

| Ref | Contribution | Relevance |
|---|---|---|
| Lu, *Entry Guidance: A Unified Method*, JGCD 37(3), 2014 (FNPEG) | fully numerical predictor–corrector, bank-angle parametrisation | **our NPC** (TS-03) |
| Lu, *Predictor–Corrector Entry Guidance for Low-Lifting Vehicles*, JGCD 31(4), 2008 | NPC for L/D ≈ 0.3 | high-α phase |
| Harpold & Graves, *Shuttle Entry Guidance*, J. Astronaut. Sci. 27(3), 1979 | drag-velocity tracking, deadband bank reversals | TS-03 option; reversal logic baseline |
| Mendeck & Craig, *Entry Guidance for the 2011 Mars Science Laboratory*, AIAA 2011 | Apollo-derived guidance, reversal deadbands | dispersion handling |
| Brunner & Lu, *Skip Entry Trajectory Planning and Guidance*, JGCD 31(5), 2008 | skip bifurcations | our E9 (skip) issue |

Open source: no licensed FNPEG implementation found ("entry guidance predictor corrector" search: only
unlicensed missile-guidance repos). Our NPC is original code following Lu 2014.

## 3. Attitude control and control allocation

| Ref | Contribution | Relevance |
|---|---|---|
| Wie, *Space Vehicle Dynamics and Control*, AIAA 2008 | quaternion feedback, rate saturation | baseline law |
| Sieberling, Chu & Mulder, *Robust flight control using INDI…*, JGCD 33(6), 2010 | incremental nonlinear dynamic inversion | TS-01 next candidate (robust to aero error) |
| Athans & Falb, *Optimal Control*, 1966 (bang-bang, double integrator) | time-optimal switching curve √(2a|e|) | **TS-01 per-axis limit** |
| Durham, *Constrained Control Allocation*, JGCD 16(4), 1993; Bodson, JGCD 25(4), 2002; Johansen & Fossen, *Control allocation — a survey*, Automatica 49, 2013 | constrained allocation, priorities | gimbal roll-priority projection, MHD box-LS |

Open source: INDI implementations exist for quadrotors/aircraft (SihaoSun/INDI_Quadrotor_FTC, no
licence; iff-gsc/SE2A_GLA_INDI, GPL-3.0). They are not reusable (licence/language) but confirm the
algorithm's structure.

## 4. MHD flow control and the HTS magnet (see research/03 for the full list)

| Ref | Relevance |
|---|---|
| Poggie & Gaitonde, *Magnetic control of flow past a blunt body*, Phys. Fluids 14, 2002 | standoff growth with the Stuart number; our Q_eff scaling |
| Fujino & Ishikawa, MHD heat shield simulations, J. Spacecraft Rockets (2006–2010) | Hall effect, electrode segmentation, κ |
| Bityurin et al., MHD flow control reviews (AIAA 2000s) | Hall parameter limits, seeding |
| MEESST (EU H2020, 2021–2024, Gülhan et al.) | REBCO magnet in plasma wind tunnel; 0.66 T demonstrated |

No open-source code was found that couples MHD shock layers to flight control; the engine's MHD
model is original (spec/02 §5).

## 5. Vehicle geometry and rendering (M5/M6)

See research/05: no reusable licensed Ship model. BlendKit "Starship" (N. Van Ingen, royalty-free,
262k polys, 2023 Block 1) cannot be redistributed. Rendering references: three.js examples
`Sky` (Preetham 1999 analytic sky), `Water`, `UnrealBloomPass` (MIT); NASA Blue Marble textures
(public domain).

## 6. Gaps (no prior art found)

- Flapless, cold-gas-only belly-flop control with an off-axis single-engine landing (our F1).
- MHD-actuated attitude control in the plasma phase (original).
