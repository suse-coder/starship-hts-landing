# spec/00 — Goal

**G0 Physical fidelity — the overriding rule (user, 2026-11-20).** The mission must succeed or
fail **only through real, simulated physics**. Nothing may be "fixed" with fake rules,
invented forces, state overrides or tuned-away physics to make the catch happen. If the
flapless, coil-stabilised ship cannot do something physically, the engine must show that it
fails (and why) instead of hiding it. G0 outranks G1: an honest failure is acceptable, a faked
success is not. The rules, R1–R10, are normative and every change is checked against them
(review/00, spec/05 gate T-fidelity):

| # | Rule |
|---|---|
| R1 | **The state changes only by integrating forces and moments.** Position, velocity, attitude, angular rate, mass, temperatures and coil current move only through the equations of motion, heat equations and circuit equations of spec/02. No snapping, teleporting, setting α/attitude/velocity by hand, "assist" forces or damping that has no physical source. |
| R2 | **Every force and moment has a physical model and a physical actuator.** Aerodynamics come from the hull geometry and the gas state; control moments come only from the HTS coil/electrodes (plasma phase), the cold-gas thrusters, the Raptor gimbals/throttle and the tower contact. There are no flaps and no hidden "control torque". |
| R3 | **The coil is a real magnet in a real plasma.** Its steering/stabilising moment is the reaction of the Lorentz force −∫ r × (j × B) dV on the shock layer (Newton 3 through the magnet). B comes from Biot–Savart over the winding pack; j comes from the plasma's Ohm's law with conductivity from equilibrium (and Cs-seeded) ionisation, the Hall parameter and electrode loading; power, momentum-flux limit, heating feedback, REBCO critical current vs (B, T), cryogenic heat load, stored energy and quench are all modelled. No plasma (low speed, low density) means no authority, which the guidance must respect. |
| R4 | **Actuators have physical limits and dynamics.** Thrust, Isp, propellant/gas consumption, valve duty, gimbal range and rate, engine spool and throttle limits, electrode loading range, coil ramp voltage/power, arm closing time. A limit is never raised in code without the mass/volume/power it costs (e.g. more N2 ⇒ heavier COPVs, more mass and inertia). |
| R5 | **Design parameters are physical choices, not tuning knobs.** Every design value is sourced, computed or explicitly labelled "assumed — optimistic but realistic" in plan/05, with its conservative alternative. Changing one to make the mission close must be recorded there, with its physical consequences, never done silently. |
| R6 | **Guidance and control are flight software.** They may use simplified internal models (predictor, tables, allocation), but act on the plant **only** through actuator commands. They never read hidden truth that a real vehicle could not sense, except for the navigation idealisation disclosed in R9. |
| R7 | **Conservation and environment are complete.** J2 gravity, rotating Earth and atmosphere, winds and turbulence, real-gas thermochemistry, heating/ablation and propellant mass loss are always on; none of them is disabled in a phase to make that phase work. |
| R8 | **Constraints are measured, not enforced.** g-load, dynamic pressure, steel/bondline/coil temperatures, quench margin, gas and propellant left are recorded and judged against limits (spec/05). A violation is a failed or "outside limits" mission, never clipped away. |
| R9 | **Every idealisation is listed.** Anything not modelled at full fidelity (perfect navigation, rigid body without slosh, engineering-level aerodynamics instead of CFD, reduced-order MHD shock layer, …) is listed in review/00 with its expected effect. The list only shrinks. |
| R11 | **One actuator family per flight regime (user, 2026-11-21).** (a) While the HTS coil is energised, **all** steering — pitch, yaw and roll, including the barbecue roll — comes from the coil/electrode interaction with the plasma (MHD roll by asymmetric electrode patterns, j_r × B_x); cold gas is only a fallback where the MHD authority/demand < 1, and its use is reported. (b) The coil is switched off (controlled fast discharge or ramp-down) as soon as shielding and steering no longer need it; the timeline shows the real switch-off. (c) After the flip **no cold gas**: attitude and translation come only from the sea-level Raptors' gimbals and throttle — at least two engines lit so that differential gimbal gives roll (one off-axis engine has only 2 gimbal DOF for 3 torque axes). Physical consequence: two Raptors at the 40 % minimum give T/W ≈ 1.8 at ≈ 110 t, so the terminal approach is a timed deceleration to the rails (no hover), like the booster catches. |
| R10 | **An honest outcome.** If the mission does not close with realistic values, the result says `lost`/`outside limits` with the physical reason (for example "cold gas exhausted", "no MHD authority below 2.5 km/s", "landing propellant exhausted"). The UI shows it the same way. |

**G1 Mission.** Simulate Ship 42's Flight 15 return — from orbit (≈ 275 km) to the catch at Pad 2
at the real catch time (2026-11-14 21:20:06 UTC) — for the flapless, coil-controlled derivative
of research/04, in one closed-loop run with no hand-tuned trajectory, under G0.

**G2 Physics.** Every force, moment, heat flux and electrical quantity comes from a model in
map/20; every model is a pure function; numbers in the docs are traceable (docs/README).

**G3 Hard requirements from the user.** No flaps (none modelled). PICA-X on the nose; the real
silica tile heat shield on the barrel over 301/30X steel (Q23, 2026-11-21). Cold gas in orbit. In
the plasma phase the **superconducting (HTS, REBCO) coil stabilises and steers the ship instead of
flaps**: pitch/yaw are controlled **only** by the coil (the engine disables cold-gas pitch/yaw
whenever the coil has authority above the q threshold). Flip with the engines, catch by Mechazilla,
belly flop included. Exploded view in the 3D UI.

**G4 Speed.** A full mission ≤ 30 s in a browser worker on the user's machine (target 10–15 s);
playback scrubbing at 60 fps.

**G4b Static web delivery (user, 2026-11-20).** Browser only, published on GitHub Pages from a
public repo (GitHub Actions deploy). The mission is fully simulated in a Web Worker when the page
loads (with a loading animation), then played back with a slider and play button (spec/04 A0–A2).

**G5 Architecture.** Monorepo of small domain packages (ADR-1), FP core (ADR-2), modular and
composable: a new actuator or model is a new pure function plugged into `step`.

**G6 Honesty.** Where the concept needs optimistic assumptions, they are defaults in plan/05 with
the conservative alternative, and the UI shows the margins (authority ratio, temperatures, gas).
