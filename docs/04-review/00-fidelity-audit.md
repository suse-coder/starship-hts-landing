# review/00 — Physical-fidelity audit (G0, rules R1–R10 of spec/00)

Re-run this audit whenever guidance, control, a model or a design default changes. Each line
cites the code that was read. Started 2026-11-20.

## 1. Plant: R1/R2/R7, state changes only through physics

| Check | Finding | Code |
|---|---|---|
| Integration | RK4 on (r, v, q, ω); derivative = gravity J2 + rotated body loads / m + contact; Euler's equations I ω̇ = M − ω × Iω. Nothing writes r, v, q or ω outside the integrator. | `packages/sim/src/step.ts` (`Rigid`, `addR`, k1–k4) |
| Aerodynamics | Integrated over the hull geometry from the local flow (Newtonian/Jorgensen + real-gas shock), with relative wind including rotation and turbulence. No added damping terms. | `packages/aero/src` |
| MHD | −∫ j × B dV and −∫ r × (j × B) dV on the vehicle (Newton 3 through the magnet); j from Ohm's law with σ, Hall β and electrode loading. | `packages/mhd/src/mhd.ts` |
| Coil | Circuit dI/dt = V/L with voltage/power limit, persistent mode, cryo energy balance, quench + dump. | `packages/mhd/src/coil.ts` |
| Engines | Spool-up curve, 40–100 % throttle with 60 %/s slew, gimbal ±15° at 20°/s, propellant drawn from the header tanks (mass ⇒ CG and inertia). | `packages/craft/src/propulsion.ts`, `step.ts` |
| Cold gas | Thrust and Isp 73 s; N2 mass consumed; COPV dry mass scales with the gas carried (400 kg + 0.6 kg/kg). Moments only from firing real thruster pairs (pure couples). | `packages/craft/src/config.ts` |
| Tower | Penalty spring–dampers on the pins + Coulomb friction; hull against the arm faces; arms close at a finite rate (3 s). No attach/snap. | `packages/tower/src/tower.ts` |
| Environment | J2, rotating atmosphere, climatological wind + Dryden turbulence, real-gas tables: always on in every phase. | `step.ts` lines 60–75 |

No violation of R1/R2/R7 found.

## 2. Flight software: R6

Guidance (`packages/gnc/src/guidance.ts`) outputs only commands: attitude target, coil/electrode
command, RCS/gimbal/throttle/engine on/off, arms close. Its internal predictor
(`predictor.ts`) and tables (`tables.ts`) are simplified models; that is allowed, because a mismatch
just causes real guidance errors in the plant. Rules like "no bank reversals in the last 50 km",
deadbands and hysteresis are **flight-software logic**, not physics, so they are allowed under R6.
**Not allowed** (and absent): guidance that changes plant state, forces or limits.

## 3. Idealisations still present (R9: this list must only shrink)

| # | Idealisation | Expected effect | Plan |
|---|---|---|---|
| I1 | Perfect navigation (state from the simulator) | Optimistic landing precision (real GPS/IMU errors would be ~1–5 m, ~0.1 m/s) | add a noise + bias nav filter model |
| I2 | Rigid body, no propellant slosh, no structural modes | Optimistic during flip (slosh is a known flip driver) | slosh pendulum model in the header tanks |
| I3 | Engineering-level aerodynamics (panel Newtonian/Jorgensen) instead of CFD | ± 10–20 % in C_N/C_m at transonic speeds; trim α uncertain by a few degrees | calibrate against published Starship trim data |
| I4 | Reduced-order MHD shock layer (cells along the cap, Saha scaling, Hall closure parameter κ) | κ is the dominant uncertainty (κ 0.03 assumed vs 0.15 conservative, plan/05 Q4) | sensitivity sweep in bench/01 |
| I5 | Momentum-flux bound on the MHD force, s = 1/(1 + \|F\|/F_cap), F_cap = 1.2 q∞ π (R_n+Δ)² | A smooth saturation set by momentum conservation; its exact form is an engineering choice | compare with a quasi-1D MHD layer solution |
| I6 | Cold-gas valves open instantly (duty-cycle average per step) | Ignores the minimum impulse bit (~10 ms) | add a minimum-on-time PWM model |
| I7 | 1-D through-thickness heat conduction per wall strip, no lateral conduction | Somewhat conservative hot spots on the steel | — |
| I8 | Electrode loading responds instantly (physically µs, so negligible) | none | — |

## 4. Design values changed while closing the loop (R4/R5, all recorded in plan/05)

| Date | Change | Physical consequence carried in the model |
|---|---|---|
| 2026-11-20 | N2 3.0 → 5.0 t, thrusters 2.5 → 4 kN | +1.2 t COPVs (0.6 kg/kg); larger mass and inertia; ≈ 17 m³ of gas at 300 bar |
| 2026-11-20 | landing propellant 14 → 16 t | heavier header tanks content, more entry mass, CG further forward |

## 5. Open findings

- **F1 (open; parked 2026-11-20 at the user's request to build the browser UI first)**: no catch
  yet. Belly-flop delivery is 1.7–1.8 km from the tower: flapless cold-gas steering can rotate the
  lift at only ≈ 2°/s. The landing burn diverts and reaches the catch point with the pins aligned,
  but it fails on propellant or in the final approach (flip handover with vertical overshoot, slow
  east–west axis of the single off-axis Raptor). The UI shows the outcome honestly ("lost") per
  G0/R10. Next: flip handover, weak-axis landing control, landing propellant (20 t, plan/05). Any
  fix is guidance (R6) or a recorded design change (R5), never a non-physical aid.
- **F2 (resolved)**: a non-physical lift floor ("belly must face down") was removed (journal E20+).
