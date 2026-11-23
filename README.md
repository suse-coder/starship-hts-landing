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

| Item | State (2026-11-23) |
|---|---|
| M3 — flip, landing, terminal, catch | **CAUGHT in still air** (no flaps; cold gas for attitude until the flip, then for roll only); windy case open |
| Recorded no-wind mission (`recording.bin`) | **`caught`**: flip-and-stop complete **1205 m** above the rails, 10 m off; at **1 km** the pins are **2.5 m** from the axis at 0.5 m/s lateral; then a vertical fall and a hoverslam — the pins touch the closed rails at ≈ 0.7 m/s, square to the arms (pin-axis error 0.3° below 500 m, 0.0° at the rails), offset **0.04 m**, 0.5 s below 2 m/s, never climbing; engine cut on pin contact (thrust gone 0.44 s later), the ship then sags 4.7 cm on the rails, slides 1 cm and the locked arms do not move; **5.0 t** of the 26 t landing propellant left |
| Windy design (default wind scale) | `lost`: delivery ≈ 550–700 m off, descent runs out of propellant ≈ 5 m above the rails |
| Delivery method | bank-plan aim corrected by verified 6-DOF rollouts → belly-flop plan by defect correction (fast model proposes, rollout decides) → nose pointed at the axis → flip ignited when its stop point reaches the axis → two-engine flip-and-stop → single-engine descent law on the verified pin-chain model. **Tuned point, not yet robust:** of 12 neighbouring tunings of the entry aim and gate, 5 delivered inside the belly flop's reach |
| TPS | **bondline over:** tile 1650 K vs the SpaceX flight-tile claim 1673 K; NASA LI-900 1530 K is the conservative margin. Bondline 695–701 K vs 560 K **fails**; backing passes |
| T‑mc (20-run wind/κ/B/mass ensemble) | **not run** |
| Model verification (`packages/verify`) | isolated landing rig on the 6-DOF plant: thrust and gimbal authority match `T/m` and `T·L/I` within **2 %** for one and two engines; the guidance's tail-first aero database equals the plant's (moment 2.8 MN·m at 10°, 90 m/s — 65 % of the gimbal's holding torque at the floor); descent capture set: **12 of 12** cases (60 m / 5 m/s in any direction from 1300 m) caught at ≤ 0.5 m with ≤ 0.25 m of overshoot |
| Per-phase flight log | `docs/03-spec/flight/nowind/flight.md` (+ `spline.json`): per phase, spline knots and the flown geometry against the guidance's frozen reference. Plan vs flown: belly flop RMS 54 m (max 187 m), powered descent RMS 6.3 m (max 10.9 m, vertical; lateral ≈ 1 m) |
| Typecheck | clean (`bun run typecheck`) |
| MPC diagrams tab | second tab beside `Flight15_Ship42`: 14 x–y panels (pins E/N/U, speeds, tilt, roll, thrust, per-engine throttle and gimbal, ground track, side profile) with plan, flown, command, the recoverable band and the hard limits; shared zoom/pan/brush, hover readout, playhead, click-to-seek. Logic unit-tested; **not yet checked in a browser** |
| 3D view | tower rebuilt as a lattice mast with carriage tracks, carriage, truss chopsticks (36 m) and QD arm; physically based sky/sun/exposure, haze, water and ground, steel hull material. Typecheck/tests/build pass; **not yet seen on a GPU** |
| Tests | 458 pass, 2 todo, 0 fail across 38 files (`bun test packages`) |
| Full mission cost | 25–27 s wall for the whole 2 870 s mission on a quiet machine, including 36 onboard prediction rollouts (12.6 s of it) — the 30 s budget is met with ≈ 3 s to spare, not more |

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
  the **sea-level Raptors** (two for the flip-and-stop, one from its completion) plus the cold-gas
  couples for roll only — a deliberate deviation from "Raptors only", because one off-centre gimbal
  cannot hold roll.
- **Catch hardware:** Mechazilla lattice mast, carriage on its tracks, two truss chopsticks (36 m,
  SpaceX's stated arm length; inner faces flared outward below the rails so the ship's aft section
  passes between them), rails and pins with solid contact and Coulomb friction. The arms close when
  the ship's aft end is below the rail tops (≈ 37 m of pin height, ≈ 6 s before the pins arrive; the
  drive needs 3 s) and are compliant while closing: they stop at the hull and yield to it. Once both
  pins bear on the rails the drives lock — the pair neither tracks nor yields, it carries the ship.
  The landing rails are stiff shock absorbers (12 MN/m per pin, critically damped: the caught ship
  sags ≈ 5 cm) and the pins do not creep along them. A catch requires the arms to be closed and both
  pins to settle on the rails; the engine is cut on pin contact.
- **Known limits of the descent:** one gimballed Raptor at the throttle floor turns the hull at
  ≈ 6°/s² and needs 1.3 s to swing its gimbal from stop to stop, and tail-first the hull is
  statically unstable (the air's moment at 15° of tilt and 90 m/s is 96 % of the gimbal's holding
  torque). The law therefore keeps the tilt at 5–6° and slews its lateral command at 0.32 m/s³: it
  removes 60 m of offset over a 1300 m fall but only ≈ 8 m in the 6 s above the 1 km gate.

## Verification, per-phase precision and the flight log

`packages/verify` is the model-verification package: it builds the landing state directly (the same
6-DOF plant, tower and flight software; no wind, no entry), so a landing result can be attributed to
the landing and not to what 2 800 s of entry handed it.

- `bun scripts/verify-model.ts` — plant identification, the tail-first aero check and the descent
  capture set. Measured: one gimbal turns the hull at **0.54 /s² at the throttle floor and 1.47 /s²
  at full thrust** (`T·L/I`, within 2 %); the *catch pins* sit at the hull's centre of percussion
  (12.6 m ahead of the CG against 12.8 m), so a gimbal step moves the CG and not the pins — the pin
  position is a clean chain of integrators, the CG has a right-half-plane zero at 0.74 rad/s. The
  terminal law that this replaced closed its loop on the CG with gains above that zero; its
  characteristic polynomial `s⁴+1.6s³+0.64s²+0.70s+0.16` fails the Routh test. The "overshoot over
  the tower" was that instability, bounded only by the acceleration caps.
- **What the descent law is now** (`slamCommand`, `packages/gnc/src/landingLaw.ts`): pole placement
  on the pin chain (four equal real poles, no overshoot by construction), the aerodynamic side
  force and moment of the tail-first hull cancelled from the model, a tilt limit from the gimbal's
  holding torque against the destabilising aero moment, a slew limit on the lateral command, and a
  hoverslam profile for the vertical channel (fall at the throttle floor, brake at 7.5 m/s²,
  1.5 m/s² over the last metre, touching at the 0.7 m/s design contact speed).
- **Adversarial findings of this round**, all measured and fixed: the two-engine identification
  was silently run on one engine (the flight software staged during the test: a 17 % "model
  error" that was the test); the flight log's plan-vs-flown figure compared the flown point with a
  prediction re-made from that same point (it measured nothing — it now uses the frozen plan); the
  belly-flop plan was drawn 112 m too low and its tail came from the last candidate tried, flown by
  the retired law; the "precision budget" table carried estimated sensitivities that nothing used
  (replaced by the table of what is actually flown, `precision.ts`).
- `scripts/flight-log.ts` → `docs/03-spec/flight/nowind/`: per phase, plan and flown geometry as
  tables and spline knots. `scripts/gate-check.ts` prints the state at the 1 km gate and the
  budgets; `scripts/phase-profile.ts` the wall time per phase and the cost of the rollouts.
- **What "over the arms at 1 km" costs, and what it does not yet give.** The ship cannot be over
  the arms at 1 km while it is still belly-flopping (it drifts at 35 m/s and the flip throws it
  another 560 m along its nose), so the flip is ignited ≈ 2.2 km up and brakes the horizontal speed
  itself on two engines (lean back against the velocity until it is gone). Measured cost: 9 t for
  the flip-and-stop and 11.8 t for the descent, leaving 5.0 t (the lower flip left 6.2 t). At 1 km
  the pins are 2.5 m from the axis, not 0: the flip hands over 10 m off, and one engine at 0.5 m/s²
  cannot remove that in the 6 s above the gate.
- **Roll.** One off-centre gimballed engine has no roll authority (its roll torque is a by-product
  of its yaw torque). Without gas a 10 m correction across the rails left 0.7°/s of roll, which
  turned the pins 26° off the rails by touchdown: position 0.01 m, catch lost. The descent
  therefore keeps the cold-gas roll couples on (pitch and yaw stay on the gimbal) and they hold the
  pin azimuth from the hand-over down, not only near the arms: the two-engine flip hands the pins
  over within ≈ 4° of the rails, the first single-engine correction rolls the hull up to 15°
  through the gimbal's coupling, and the couples have it back inside 0.3° by 500 m. This is a
  deliberate deviation from "Raptors only inside the 200 m ring"; the alternative is a second
  running engine, which cannot throttle below 1.5 g and so cannot fall into the arms.
- **Not robust yet.** The hand-over from the high-α glide is sensitive: a change of metres in a
  rolled-out prediction can flip a decision and move the belly-flop start by kilometres. The flown
  tuning (entry aim 1500 m, gate 1550 m) catches; so did 5 of the 12 neighbouring points tried.

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
