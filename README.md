# Ship 42 — flapless Starship return and catch simulation

> ## ⚠️ WORK IN PROGRESS — no flaps, cold gas until the 200 m ring, caught in still air
>
> **Design decision (2026-11-22):** the ship flies **without flaps**. The cold-gas RCS is the attitude
> actuator from the deorbit flip until the ship is inside the **200 m radius above the tower**
> (`CATCH_RADIUS` in `packages/gnc/src/guidance.ts`); inside that radius everything is on the Raptors.
> The flap model and its wiring remain in the tree but are not flown (`FLIGHT_DESIGN`).
>
> The recorded no-wind mission **catches**: flip complete 363 m above the rails, 111 m off; arms
> close at 26 m; both pins bear on the rails at 0.18 m/s, offset **1.5 m**, with **5.8 t** of landing
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
| Recorded no-wind mission (committed `recording.bin`) | **`caught`**: pins on the closed arms at 0.85 m/s, offset **3.03 m**, 7.3 t left; no flaps, cold gas until the 200 m ring |
| Windy design (default wind scale) | `lost`: delivery ≈ 550–700 m off, descent runs out of propellant ≈ 5 m above the rails |
| Delivery method | belly plan chosen by 6-DOF rollouts to the flip-complete state (defect-corrected aim), then a single-engine velocity-field descent |
| TPS | **bondline over:** tile 1650 K vs the SpaceX flight-tile claim 1673 K; NASA LI-900 1530 K is the conservative margin. Bondline 695–701 K vs 560 K **fails**; backing passes |
| T‑mc (20-run wind/κ/B/mass ensemble) | **not run** |
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
