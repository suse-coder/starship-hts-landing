# research/03 — Magnetohydrodynamic control with an HTS coil

## Physics in one page

Behind the bow shock the air is a weakly ionised plasma (equilibrium at 6000 K, 0.1 atm:
n_e ≈ 10²⁰ m⁻³, σ ≈ 100–1000 S/m). The gas flows outward along the nose; a coil behind the nose
makes B mostly axial there, so u × B is azimuthal and drives an azimuthal current
j = σ_eff (1 − K)(u × B). The Lorentz force on the gas, f = j × B, opposes the tangential flow
(and pushes upstream). By Newton's third law the coil feels the opposite force; for a sector
that is outward (toward that side) plus drag. Consequences:

1. **Symmetric loading** = drag + standoff growth + heat-flux reduction (the MEESST purpose).
2. **Asymmetric loading** of 8 electrode sectors = side force at the nose ≈ 26 m ahead of the
   CG = pitch/yaw moment. Electrode loading changes in microseconds — the main field stays
   persistent (no inductive bandwidth limit; the 64.6 H coil could never be modulated fast).
3. **Generator mode** (0 < K < 1) extracts electrical power P = K(1−K)σ|u×B|²; driven
   electrodes (K < 0) increase the force at the cost of power.

## What limits it (computed, `scripts/mhdtest.ts`, `mhdauth.ts`)

| Limiting mechanism | Number |
|---|---|
| Hall parameter β = ω_ce/ν at 2 T, unseeded, 55–75 km | 35–330 → σ_eff = σ/(1+κβ²) collapses by 70–5000× |
| with 1 % Cs seeding | n_e ×10, Coulomb + Cs collisions raise ν → β ≈ 3–15 |
| Hall-dominated regime | f ≈ n_e m_e ν u/κ: **independent of B** — a stronger magnet does not help |
| momentum bound | F ≤ F_cap ≈ 1.2 q∞ π (R_n + Δ)² ≈ 0.2–0.4 MN at q = 10 kPa |
| restoring authority vs α | adequate (1.5–2.5× demand) for α ≤ 15°; **collapses for α ≥ 20°**: windward-sector drag acts ahead of the CG and is destabilising |
| plasma window | unseeded down to ≈ 4.5 km/s; seeded down to ≈ 2.5 km/s (T0 ≈ 2600 K); none below. Flown 2026-11-21: the authority switch-off (Q25) came at **29–37 km, V ≈ 1.9–2.5 km/s** (`.tmp/mission_ff.log`: T+2734 s, 29 km, 2.25 km/s; `.tmp/mission_G.log`: T+2836 s, ≈ 37 km, 1.9 km/s). Upper edge: the flight-software gate q̄ > 30 Pa is met from ≈ 95 km at entry speed. **The coil's actuation corridor is ≈ 95 km → ≈ 30 km; below it there is no plasma to push on** (see §Low-altitude MHD actuation) |
| trim cost | ΔCL −0.15…−0.23, ΔCD +0.22…+0.37: L/D of the nose-first glide 0.70 → 0.20 (α 8°), 1.28 → 0.55 (α 14°) |

## HTS sizing (designed, `packages/craft/src/config.ts → designCoil`)

| Item | Value | Basis |
|---|---|---|
| B at stagnation point | 2.0 T (design parameter) | MEESST target 1–2 T; 0.66 T demonstrated |
| loop radius / position | 1.75 m, 1.2 m behind the tip | fits inside the Rn = 2.5 m cap (body r = 2.14 m there) |
| ampere-turns | 9.93 MA·t (3310 turns × 3 kA) | on-axis loop formula |
| inductance / energy | 64.6 H / 291 MJ | thick-loop GMD formula |
| peak field on the winding | 14.3 T (REBCO at 25 K) | estimate |
| tape mass | 1.39 t at J_e 700 A/mm² | NI·2πa/J_e × 8900 kg/m³ |
| structure | 0.77 t | virial theorem, CF composite 1.2 GPa, safety 2 |
| cryostat / cryocoolers | 0.35 / 0.25 t | 15 kg/m², 1.4 kg per W at 25 K |
| battery / power / seed | 0.50 / 0.40 / 0.49 t | 100 kWh at 200 Wh/kg; 400 kg Cs |
| **total** | **4.15 t** | vs 12–15 t of flaps |
| ramp | ≈ 11 min at 450 kW from the battery, finished at entry interface | |
| current-sharing T | 41.6 K at I/Ic 0.55 → 16 K margin | REBCO scaling |

## Verdict carried into the design (research/04)

An optimistic but physical HTS coil **can** be the only pitch/yaw actuator of a nose-first
Starship in the plasma phase — **if** it is seeded, if the Hall current is largely blocked
(κ ≈ 0.03) and if α stays ≤ 14°. It cannot provide roll torque (cold gas does), it cannot act
below ≈ 2.5 km/s, and it costs L/D. It is ≈ 1/3 the mass of the flaps it replaces.

## Low-altitude MHD actuation — can the coil steer to 2 km or land the ship? (2026-11-21, Q31)

User request: use the coil as the landing actuator where physics allows, possibly with several
coils, so that the fall stays stable (no tumbling). This section records what is possible.
Labels: **sourced** (literature), **computed** (from the numbers below), **assumed** (stated value).

### Principle

The coil itself exerts no force on the air. The force density is the Lorentz force on currents in a
conducting medium, f = j × B with j = σ (E + u × B), so in the short-circuit (Faraday) limit
|f| ≈ σ u B² (sourced: MHD-drive principle; Wikipedia *Magnetohydrodynamic drive*). Without a
conductor (σ ≈ 0) there is no force, whatever the field. A stronger magnet or more coils only help
where σ u is already large. Neutral air at 2 km (T ≈ 280 K, u ≈ 100–150 m/s) has σ ≈ 0.

### Route A — natural shock-layer plasma (current design): ends at ≈ 30 km

The shock layer conducts only while the flow is hypersonic (T0 ≳ 1800 K, seeded). Flown: the
authority falls to zero at 29–37 km, V ≈ 1.9–2.5 km/s (computed, mission logs). To have a seeded
plasma at 2 km the ship would have to fly there at ≈ Mach 5.5–6 (≈ 1.9 km/s): q̄ ≈ 2 MPa and a
nose heat flux of the order of 80 W/cm² (computed). That is far outside the tile/PICA limits and incompatible
with a landing burn. **Not possible for this mission.**

### Route B — artificial ionisation of the low-altitude air (e-beam, RF, arc, alkali seeding)

The literature on magneto-aerodynamics ionises air with e-beams, RF/microwave discharges or arcs,
with or without alkali seeding (sourced: Wikipedia *Magnetohydrodynamic drive*, active flow
control; refs Froning 1999, Lineberry 2000). It works at high altitude, where the density is low.
At 2 km it fails on power, because recombination and electron attachment remove the electrons within microseconds:

| Quantity | Value | Basis |
|---|---|---|
| electron–neutral collision frequency, air near 1 atm | ν ≈ 2·10¹² s⁻¹ | assumed (textbook order) |
| conductivity σ = n_e e²/(m_e ν) | 1.4 S/m per 10²⁰ m⁻³ | computed |
| n_e for σ = 1 / 3 / 10 S/m | 0.7 / 2 / 7 ·10²⁰ m⁻³ | computed |
| dissociative recombination coefficient | α ≈ 2·10⁻¹³ m³/s | assumed (O₂⁺, 300 K); three-body attachment to O₂ makes it worse |
| energy per ion pair in air | W ≈ 34 eV | assumed (W-value of air) |
| power to hold n_e in V = 100 m³: P = α n_e² V W | **≈ 5·10¹¹ W at 1 S/m, ≈ 5·10¹³ W at 10 S/m** | computed |
| available on board | 450 kW converter, 100 kWh battery | design (`designCoil`) |
| force if the plasma existed, f = σ u B² · V | 30 kN (σ 3 S/m, u 100 m/s, B 1 T) … 0.9 MN (σ 10 S/m, B 3 T over 100 m³) | computed; the vehicle weighs ≈ 1.1–1.4 MN |
| surface plasma actuators (DBD) at 1 atm | induced flow < 10 m/s | sourced: Wikipedia *Plasma actuator* |

The power shortfall is 10⁶–10⁸×, so this is **not possible in principle with any realistic on-board power**.
The earlier session figure of "0.5–3 MW" left out recombination and is withdrawn.

### Route C — the Raptor plume (only while the engines fire)

The exhaust is already hot and partly conducting, so a magnet around it can deflect it (MHD thrust-vector
control; cross-field accelerators in rocket nozzles studied since the late 1950s, sourced: Wikipedia
*Magnetohydrodynamic drive*, spacecraft propulsion). Order of magnitude with alkali seeding of the plume
(σ ≈ 10 S/m, the open-cycle MHD-generator class, assumed), u ≈ 3.3 km/s, B = 1 T over ≈ 4 m³ near the
nozzle exit: |f| V ≈ 130 kN per engine (first estimate, **superseded**: spec/09 computes the exit at 1310 K, 0.83 bar, where 1 % Cs gives
only σ ≈ 0.02 S/m; σ of 5–70 S/m exists only in the first metre of the nozzle, so the side force is ≤ 2.5 kN per engine). Most of that force brakes the jet along u (a thrust loss);
a side force needs an asymmetric field or electrodes, so it is a fraction of this. By comparison one Raptor
gimballed 15° gives ≈ 630 kN of side force (computed: 2.45 MN · sin 15°). Constraints:

- The current coil sits in the nose, ≈ 50 m from the plume; its field there is negligible. This route
  needs a separate aft magnet near the hot engine bay (new mass, cryostat, power).
- The HTS coil cannot be modulated fast (L = 64.6 H, dI/dt ≈ 2.3 A/s at 450 kW). Fast control would come
  from electrodes and the plasma, as in the nose system, not from the coil current.
- It only exists while an engine burns, i.e. after the flip, where the gimbals already exist.
- Status: laboratory concept, no flight heritage.

**Physically possible but useless at this scale** (spec/09): ≤ 2.5 kN per engine (0.4 % of a gimbal), 0.83–0.99 t
of magnet per engine plus 2.4 t of caesium per engine-minute, 0.29 rad/s bandwidth vs 0.8 needed. Rejected.

### What keeps the ship from tumbling below the coil corridor

| Altitude band | Stabilising mechanism (current design) |
|---|---|
| ≈ 95 → 30 km (hypersonic) | HTS coil + electrodes (R11a); passive trim after the pitch-up |
| ≈ 30 → 0.8 km (belly flop, subsonic) | **passive aerodynamic trim** of the flapless hull at high α (statically stable in pitch/yaw, `passiveTrim`) plus cold gas for bank and heading; the coil has no plasma here |
| flip → catch | Raptor gimbals, ≥ 2 engines (R11c) |

### Conclusion

- With natural plasma the coil steers from ≈ 95 km down to ≈ 30 km. **2 km is not reachable.**
- Artificial ionisation at low altitude is ruled out by power, by 6–8 orders of magnitude.
- The plume route is real physics but gives ≤ 2.5 kN per engine for 2.6–5.1 t (spec/09): rejected.
- The stable fall below 30 km comes from aerodynamics plus cold gas, and after the flip from the engines.
  Q31 records the user's request and these options.
