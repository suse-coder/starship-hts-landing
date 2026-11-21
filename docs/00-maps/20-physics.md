# Physics map — model, regime, equation, file

| Domain | Model | Regime / limits | File |
|---|---|---|---|
| Gravity | point mass + J2 | all | `packages/earth/src/earth.ts` |
| Frames | ECI (z = spin axis), ECEF by GMST θ(t) = θ0 + ωt, WGS84 geodetic (Bowring), local ENU, pad ENU | UT1 ≈ UTC | `earth/earth.ts`, `earth/time.ts` |
| Atmosphere | US 1976: 7 geopotential layers to 86 km (exact hydrostatics), log-interpolated tables to 1000 km; Sutherland viscosity | quiet Sun | `earth/atmosphere.ts` |
| Wind | climatological mean profile (Gulf, November; jet 36 m/s at 11.5 km) + Dryden turbulence (MIL-F-8785C, exact first-order discretisation, counter-based RNG) | < 30 km | `earth/wind.ts` |
| Geomagnetism | IGRF degree-1 tilted dipole | torque on the coil dipole in orbit | `earth/magfield.ts` |
| Real gas | 9-species equilibrium air from partition functions (trans., rigid rotor, harmonic oscillator, electronic levels), law of mass action, Newton in log densities | 300–15 000 K, no Ar/double ionisation | `gas/air.ts` |
| Shock | real-gas normal shock (ε iteration) + isentropic stagnation | M > 1.05 | `gas/air.ts` → table `gas/shock-table.json` (53 × 42, 2.8 s to build) |
| Plasma transport | e–neutral (Itikawa-like cross sections), Spitzer e–ion, σ = n_e e²/(m_e ν), Hall β = ω_ce/ν, Saha Cs seeding, plasma/cyclotron frequency | weakly → moderately ionised | `gas/plasma.ts` |
| Aerodynamics | modified Newtonian panels (592) with real-gas Cp,max, panel skin friction, local velocity v + ω×r ⇒ damping; Jorgensen strips (36): slender-body potential + η·Cdc(Mc) crossflow; CA0(M) table; blend smoothstep(2.5, 5.5, M); free-molecular blend by Kn | 0 ≤ α ≤ 180° | `aero/aero.ts` |
| Heating | Sutton–Graves stagnation (hot-wall), Tauber–Sutton radiative, Lees on the cap, Eckert reference enthalpy flat plate at Newtonian edge pressure with isentropic edge expansion (entropy-layer edge), Re_θ/M_e transition 200–300 | 16 stations × 8 rays | `heat/heating.ts` |
| Wall response | PICA-X: implicit 1-D conduction (14 graded nodes), Arrhenius pyrolysis, transpiration, blowing Ψ = φ/(e^φ−1), B'c = 0.17 oxidation + recession, RTV + steel substrate; steel: lumped with re-radiation | 4 Hz | `heat/thermal.ts` |
| Coil field | exact filament-loop field (complete elliptic integrals by AGM), 5 filaments across the winding pack; tabulated on a meridional grid | — | `mhd/mhd.ts` |
| MHD interaction | shock-layer cells (10 θ × 2 layers × 16 φ) at the α-shifted stagnation point; isentropic layer expansion; Saha-scaled n_e; j = σ_eff(1−K_s)(u×B), σ_eff = σ/(1+κβ²); f = j×B; reaction −∫f, −∫r×f; Stuart number → standoff Δ = Δ0(1+Q)^¼; momentum bound 1/(1+|F|/F_cap); heat factor (Δ0/Δ)^½ | T0 > 1500 K | `mhd/mhd.ts` |
| Magnet | voltage/power-limited ramp, persistent mode, ramp-down to battery, quench → dump (τ = L/R), cryostat MLI radiation + statics, cryocooler lift, T_cs(I) | — | `mhd/coil.ts` |
| Propulsion | Raptor 3 SL: F = τF_vac − p∞A_e, ṁ = τF_vac/(Isp_vac g0), spool S-curve, throttle 40–100 %, TVC ±15° at 20°/s; N2 cold gas Isp 73 s, PWM duty | — | `craft/propulsion.ts` |
| Mass properties | 24 components + 6 consumables, parallel-axis inertia | — | `craft/mass.ts` |
| Tower | chopsticks (open/close in 3 s), pin–rail penalty contact with dampers + Coulomb friction, hull vs arm faces | — | `tower/tower.ts` |
| Rigid body | RK4 on (r, v, q, ω); body loads held over the step, gravity and contact per stage | dt 4 ms…0.5 s by phase | `sim/step.ts` |

## Deliberately neglected (and why it is acceptable here)

- Thermochemical **non-equilibrium** in the shock layer (equilibrium is reasonable below ~70 km
  for a 2.5 m nose; above, MHD and heating are small anyway).
- Coupled CFD/MHD (the shock-layer flow is prescribed, not solved): the momentum bound and the
  Q-standoff correlation stand in for the feedback of the Lorentz force on the flow.
- Propellant slosh, structural flexibility, plume–aero interaction, engine-out cases.
- Navigation errors (perfect state knowledge).
