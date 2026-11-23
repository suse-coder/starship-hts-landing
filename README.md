# Ship 42 — flapless Starship return and catch simulation

> ## ⚠️ WORK IN PROGRESS — no flaps, cold gas until the 200 m ring, caught in still air
>
> **Design decision (2026-11-22):** the ship flies **without flaps**. The cold-gas RCS is the attitude
> actuator from the deorbit flip until the ship is inside the **200 m radius above the tower**
> (`CATCH_RADIUS` in `packages/gnc/src/guidance.ts`); inside that radius everything is on the Raptors.
> The flap model and its wiring remain in the tree but are not flown (`FLIGHT_DESIGN`).
>
> The recorded no-wind mission **catches**: flip complete 363 m above the rails, 111 m off; arms
> close at 26 m; both pins bear on the rails at 0.01 m/s, offset **0.6 m**, with **6.2 t** of landing
> propellant left. Nitrogen: 5000 → 3123 kg over the whole mission (coast 213 kg, high-α 949 kg,
> belly 415 kg, landing ≈300 kg).
>
> The windy case is still open, as are the TPS bondline and T‑mc.

**Live WIP site:** <https://suse-coder.github.io/starship-hts-landing/> — it plays the committed
recording, which is now the **`lost`** run without cold gas.

This is a research and engineering simulation project, not SpaceX software and not a flight-proven
vehicle. It exists to answer one question honestly: **can a Starship-derived vehicle with no flaps
return from orbit and be caught by the tower, using a superconducting (REBCO) coil in the nose to
steer and stabilise the ship in the plasma phase?**

## What the project is

The project re-flies the return of **Ship 42 (Flight 15)** from a ≈ 275 km orbit to Pad 2: deorbit,
entry, MHD-trimmed glide, pitch-up, high-α glide, belly flop, flip, landing burn and tower catch. It
is one closed-loop 6-DOF mission, not a scripted animation.

The design removes the four flaps. The nose carries **PICA-X**; the barrel keeps the **real silica
tile heat shield** on a 301/30X stainless-steel backing (user decision Q23). A **high-temperature
superconducting coil** in the nose interacts with the shock-layer plasma through the real Lorentz
force, and the flight software must respect the plasma corridor (roughly 95 → 30 km): below it the
coil has no authority, and the ship falls on passive trim, cold gas and finally the Raptor gimbals.

**The physical fidelity rule is the top requirement (spec/00 G0):** no fake forces, no state
overrides, no physics disabled in a phase to make the catch work. An honest `lost` is better than a
faked `caught`.

## Current status at a glance

| Item | State (2026-11-22) |
|---|---|
| M3 — flip, landing, terminal, catch | **CAUGHT in still air** (no flaps, cold gas until the 200 m ring); windy case open |
| Recorded no-wind mission (committed `recording.bin`) | **`caught`**: pins on the closed arms, offset **0.61 m**, 6.2 t of landing propellant left; no flaps, cold gas until the 200 m ring |
| Windy design (default wind scale) | `lost`: delivery ≈ 550–700 m off, descent runs out of propellant ≈ 5 m above the rails |
| Delivery method | belly plan chosen by 6-DOF rollouts to the flip-complete state (defect-corrected aim), then a single-engine velocity-field descent |
| TPS | **bondline over:** tile 1650 K vs the SpaceX flight-tile claim 1673 K; NASA LI-900 1530 K is the conservative margin. Bondline 695–701 K vs 560 K **fails**; backing passes |
| T‑mc (20-run wind/κ/B/mass ensemble) | **not run** |
| Model verification (`packages/verify`) | isolated landing rig + plant identification: the gimbal pitch authority matches the analytic `T·L/I` within **2 %**, the catch pins sit at the hull centre of percussion (12.6 m vs 12.8 m), the throttle slews at 0.60 /s and reaches 90 % in 0.74 s. The reworked no-hover descent law (not flown yet) measured on the rig from 1 km: caught, **0.00 m** offset, contact 0.35 m/s, 2.1 s below 2 m/s |
| Per-phase flight log | `docs/03-spec/flight/nowind/flight.md` (+ `spline.json`): every phase with its 10 spline knots and the flown geometry table (pins, velocity, tilt, gimbal, command, plan), written to be read, not replayed |
| Typecheck | clean (`bun run typecheck`) |
| Tests | 324 pass, 2 todo, 0 fail across 30 files (`bun test packages`) |
| Full mission cost | ≈ 22–28 s wall for the whole 2900 s mission including 20 onboard prediction rollouts |

The numbers above come from the evidence logs named in the roadmap; they are measured or simulated
values, not targets. The 3D view, the browser tree, the catch-tower contact model, the arm timing,
the approach-direction gates and the rendering features (global clouds, predicted-path overlay,
right-mouse pan, altitude graph) are implemented and verified as described in the roadmap. The
no-wind catch is a genuine end-to-end result; the all-weather release is not.

## Architecture

Bun workspaces, one domain package per physical concern, with a pure simulation core and a
browser/UI shell:

| Package | Responsibility |
|---|---|
| `packages/math` | immutable vectors/quaternions, interpolation, root finding, tridiagonal solves, RNG |
| `packages/earth` | WGS84 frames, J2 gravity, US Standard Atmosphere 1976, winds + Dryden turbulence, geomagnetic dipole, Sun, time scales |
| `packages/gas` | 9-species equilibrium air from partition functions, real-gas normal shock, plasma transport, shock tables |
| `packages/craft` | outer mould line, mass/CG/inertia budget, tiles, Raptor and cold-gas RCS models |
| `packages/aero` | modified Newtonian panels blended with Jorgensen crossflow |
| `packages/heat` | Sutton–Graves/Lees/Eckert heating, PICA-X ablation/pyrolysis, tile/steel thermal response |
| `packages/mhd` | HTS coil field, shock-layer Lorentz force with Hall effect and caesium seeding, coil circuit and cryogenics |
| `packages/tower` | Mechazilla arms, rails, solid tower/carriage/arm collision, pin friction and contact |
| `packages/gnc` | mission targeting, 3-DOF predictor-corrector entry, belly-flop laws, landing MPC, attitude control, actuator allocation |
| `packages/sim` | pure phase machine and RK4 step, columnar recording, scoring, Web Worker entry |
| `packages/view` | three.js rendering of a recording: vehicle and parts, plasma/shock/field, tower, Earth, clouds, predicted path |
| `packages/app` | browser app: worker, playback timeline, browser tree, results inspector, charts, design panel |

The dependency direction is one-way: domain packages do not import the view or the app; `sim` is the
only place that composes models into a step; `view` renders a `Recording` and never runs physics.
See [docs/03-spec/06-architecture.md](docs/03-spec/06-architecture.md).

## How to run

Requirements: [Bun](https://bun.sh/) (tested with 1.4.4) and Node-compatible tooling for Vite.

```bash
bun install
bun run dev        # Vite dev server on http://localhost:5199
bun run typecheck  # tsc --noEmit, strict
bun run test       # bun test packages
bun run build      # production build of packages/app
bash scripts/quick-check.sh  # fast module gate: tower/gnc/score + MPC/model bench (no full mission)
```

Other entry points:

```bash
bun run mission                # one full mission in Bun, prints the phase log and summary
bun scripts/bench.ts           # run-time benchmark per phase (bench/01)
bun scripts/optimize.ts        # fast 3-DOF screen + full 6-DOF certification (bench/02)
```

The module-first iteration rule and the “catch fails — test only the catch” recipe are in
[docs/03-spec/12-testability.md](docs/03-spec/12-testability.md): full missions are the integration
gate, not the edit loop.

Heavy runs (mission, optimiser, headless browser) should follow
[docs/05-benchmarks/00-machine-budget.md](docs/05-benchmarks/00-machine-budget.md): `nice -n 19`, one
mission or profile run at a time, one headless browser at a time. The shared devcontainer is CPU-bound,
and starving the OpenCode server restarts it.

## Physics and design summary

- **Dynamics:** 6-DOF rigid body with quaternion attitude, RK4 per phase, fixed step selected by phase
  (0.5 s in vacuum … 0.02 s in the glide … 0.004 s in terminal).
- **Environment:** WGS84 rotating Earth, J2 gravity, US-1976 atmosphere, November Gulf climatology
  plus Dryden turbulence, Sun position and eclipse, geomagnetic dipole.
- **Aerodynamics:** modified Newtonian panels for the hypersonic regime, Jorgensen crossflow for
  supersonic/subsonic, integrated over the actual hull geometry; no flaps.
- **Thermal:** Sutton–Graves/Lees/Eckert stagnation heating, PICA-X ablation and pyrolysis, a 6-node
  LI-900 tile layer with RTV bond and steel backing, bare-steel temperature-dependent strength.
- **MHD:** Biot–Savart field of the REBCO winding pack, shock-layer conductivity with Hall closure
  and caesium seeding, electrode loading, Lorentz force/moment, coil current/energy/quench, cryogenics.
- **Propulsion:** Raptor 3 sea-level and vacuum models with spool, throttle, Isp and gimbal limits;
  cold-gas RCS with finite N₂ and COPV mass; differential gimbal roll.
- **Guidance:** 3-DOF predictor-corrector entry with a planned bank reversal, lift-transport belly-flop
  laws, a lag-aware landing MPC, and a terminal deceleration to the rails. The belly plan is chosen
  and corrected by **onboard 6-DOF rollouts** (the vehicle model, forecast wind, no turbulence, no
  knowledge of future failures) to the flip-complete state; the post-flip descent is a velocity-field
  law whose sink is coupled to the time the lateral correction needs. After the flip the actuators are
  the **sea-level Raptors** (one engine from flip completion) plus the cold-gas couples for roll only —
  a deliberate deviation from "Raptors only", because one off-centre gimbal cannot hold roll.
- **Catch hardware:** Mechazilla column, carriage, V-shaped chopstick arms (44 m long, inner faces
  flared 2 m outward over their 2.8 m depth so the ship's aft section passes between them), rails and
  pins with solid contact and Coulomb friction. A catch requires the arms to be closed and both pins
  to settle on the rails.
- **Known limit (terminal lateral authority):** one gimballed Raptor at the throttle floor gives
  ≈ 1–2 m/s² of lateral acceleration in the last 30 m. A zero-effort-miss (ZEM/ZEV) descent law was
  implemented and measured: it keeps the along-track motion (no stop-and-reverse) and closes 380 m to
  ≈ 20 m, but it cannot finish inside that authority, so the flown law remains the position/velocity
  law that closes the last metres. The belly plan's delivery (≈ 380–490 m at flip completion) is what
  the terminal has to absorb; the flip also translates the ship by ≈ v_h·4.5 s after ignition, which
  the point-mass belly model omits.

## Verification, per-phase precision and the flight log

`packages/verify` is the model-verification package: it builds the landing state directly (the same
6-DOF plant, tower and flight software, no wind, no entry) so a landing change can be attributed to
the landing and not to what 2 900 s of entry handed it.

- `bun scripts/verify-model.ts` — plant identification and step responses. Measured: one gimbal
  turns the hull at **0.54 /s² at the throttle floor and 1.47 /s² at full thrust** (`T·L/I`), the
  side force at the *catch pins* cancels there (pins 12.6 m ahead of the CG, centre of percussion
  12.8 m) — the pin position is a clean chain of integrators, while the CG has a right-half-plane
  zero at 0.74 rad/s. That zero is why the old terminal's lateral law (`ω_n` 0.8, `k_p` 0.25,
  `k_d` 1.1) is *linearly unstable*: its closed loop `s⁴+1.6s³+0.64s²+0.70s+0.16` fails the Routh
  test. The ±4 m terminal hunting was that instability, bounded only by the acceleration caps.
- `scripts/flight-log.ts` — writes `docs/03-spec/flight/nowind/`: per phase, the guidance's plan and
  the flown geometry as tables and spline knots (positions in the catch frame: E across the rails,
  N along them, U above the rail plane). This is the input for refining a phase: it shows at a
  glance whether a phase's control is exact or merely survivable.
- `scripts/refine-phases.ts` (phase-aware search) and `packages/verify/src/precision.ts` — a compute
  budget is split over phases in proportion to the measured sensitivity of the final catch to an
  error introduced there (a quadratic loss), inverse to the cost of one evaluation there. The
  terminal gets resolution, the coast does not. The landing value is measured on the rig (`refine-phases.ts --measure`: 0.8 m per m/s); the
  flip/belly/terminal values are still estimates and the code says so (`sensitivitySource`).
- Open item, measured: **the ship is not yet over the arms at 1 km.** In the flown no-wind run it
  crosses 1 km during the belly flop, 566 m from the catch axis, and the flip completes at 363 m
  with a 111 m delivery. Raising the flip gate so the flip completes at 1 050 m (the `flipComplete`
  knob) makes the *belly plan* miss by 1 074 m at the new ignition point — the aim and the gate are
  one problem, so the gate stays at the flown value until the belly aim is searched with the new
  per-phase optimizer. A verified terminal that enters vertically into the arms (the no-hover
  hoverslam in `slamCommand`, measured 0.00 m / 0.35 m/s on the rig) is implemented but **not flown
  in the published recording**: its 4.5° tilt cap cannot absorb the 111 m delivery, so the flown
  descent is still the velocity-field law.

## Documentation

Start with:

- [docs/README.md](docs/README.md) — how the documentation is organised and the citation rules
- [docs/02-plan/00-roadmap.md](docs/02-plan/00-roadmap.md) — milestones, current status, verified
  features, release checklist
- [docs/03-spec/00-goal.md](docs/03-spec/00-goal.md) — goal and the physical-fidelity rules (G0, R1–R11)
- [docs/03-spec/03-gnc.md](docs/03-spec/03-gnc.md) — guidance, navigation, control, catch envelope
- [docs/03-spec/04-app.md](docs/03-spec/04-app.md) — browser application and rendering requirements
- [docs/03-spec/05-verification.md](docs/03-spec/05-verification.md) — verification matrix and current test evidence
- [docs/03-spec/11-release.md](docs/03-spec/11-release.md) — release gate and private-WIP policy
- [docs/04-review/02-subagent-review.md](docs/04-review/02-subagent-review.md) — the landing review and
  the current root cause
- [docs/05-benchmarks/01-run-time.md](docs/05-benchmarks/01-run-time.md) and
  [02-optimizer.md](docs/05-benchmarks/02-optimizer.md) — measured run time and the first parameter search
- [docs/06-journal/2026-11-21.md](docs/06-journal/2026-11-21.md) — the experiment log

## Safety, scope and release policy

This is a simulation study. It is **not** an operational flight model, not a vehicle design ready for
construction, and not affiliated with or endorsed by SpaceX. The public repository and GitHub Pages
site remain **private/unpublished** until the release gate in
[docs/03-spec/11-release.md](docs/03-spec/11-release.md) is satisfied and the user explicitly approves
the release. Until then, do not present the project as flight-ready, do not publish a public URL, and
do not describe the false catch as a success.
