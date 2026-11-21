# research/04 — Can a flapless, nose-first Starship work? What physics allows

## 1. The literal concept

"Nose-first from orbit, PICA only on the nose, the coil keeps it from tumbling, near the ground
flip with the engines and get caught." Checked with a 3-DOF exploration (journal, first entry):

| Quantity | nose-first (CD·A ≈ 57 m², m 100 t) | belly-first reference (CD·A ≈ 546 m²) |
|---|---|---|
| ballistic coefficient | ≈ 1750–2000 kg/m² | ≈ 185 kg/m² |
| peak q (ballistic, Δv 90 m/s) | 152 kPa, 8.9 g | 15.6 kPa |
| q at V = 1.5 km/s (end of seeded plasma) | 54–102 kPa | 10 kPa |
| subsonic terminal q (= β g) | 20–44 kPa | ≈ 2 kPa |

Three facts kill it: (a) a body of revolution without fins is **statically unstable nose-first**
(slender-body normal force acts near the nose, the CG is aft); (b) the coil has **no plasma
below ≈ 2.5 km/s**; (c) any rotation to tail-down passes broadside, and broadside at 20–100 kPa
is a 10–50 g side load. No actuator on board (RCS 0.1 MN·m, coil 0, engines ≈ 15–40 MN·m at a
cost of tens of tonnes of propellant) can hold the unstable nose-first attitude through the
supersonic/subsonic regime.

## 2. What is stable without flaps (computed, `scripts/aerotest.ts`)

With the CG 1 m forward of the planform centroid the hull has exactly one stable equilibrium:
a high-α trim — α ≈ 51° hypersonic (Newtonian), ≈ 73° supersonic, ≈ 84° subsonic. Rotation
about the velocity vector (bank) and about the body axis (roll) are neutral (axisymmetric
body). So the belly flop needs **no** control surface in pitch; cold gas suffices for heading.

## 3. The physical variant flown by the engine

1. **Cold gas** in orbit: attitude hold, slews at 0.5°/s, geomagnetic torque on the persistent
   coil (≈ 2.5 kN·m) — the coil is ramped only in the last ≈ 13 min before entry interface.
2. **MHD-trimmed nose-first lifting glide** (120 km → V 6 km/s): α 10 → 14°, the coil is the only
   pitch/yaw actuator (authority 1.5–2.5× demand), roll is free (axisymmetric), a 4°/s barbecue
   roll above 10 kPa; numerical predictor–corrector on bank.
3. **Loft** (bank 0) below 6 km/s, **pitch-up at the q minimum** (≈ 12.7 kPa at 5.4 km/s): the
   coil releases, the aerodynamic instability rotates the ship into the passive trim (4–6 g).
4. **High-α glide** (α ≈ 51–55°, L/D ≈ 0.5) with a 6°/s barbecue roll (bare steel ≤ 1000 K),
   bank control on cold gas, the α-pitch axis left to the aerodynamics.
5. **Belly flop** (M < 2): passive trim α 72–84°, heading toward the pad on cold gas.
6. **Flip** at ≈ 1.7 km (three Raptors), **landing burn** on one Raptor, ZEM/ZEV guidance to a
   point above the arms, **terminal descent** into the chopsticks, catch.

## 4. Numbers from the simulation (run of 2026-11-20, see journal)

| Metric | Value |
|---|---|
| peak load factor | 6.1 g (pitch-up transient) |
| peak q | 22.9 kPa |
| stagnation heat flux | 93 W/cm² (with MHD factor) |
| PICA-X surface / bondline | 1880 K / 371 K |
| bare steel max | 994 K |
| coil cold mass | ≤ 25.5 K (no quench) |
| Cs used | ≈ 225 kg of 400 |
| MHD trim force | up to 0.65 MN |

## 5. Honest summary for the user

- The coil works as a primary hypersonic actuator only with optimistic assumptions (κ = 0.03,
  1 % Cs seeding, 2 T) — with κ = 0.15 the authority falls below the demand (try it in the
  Design panel).
- It does **not** remove the need for a passively stable high-α phase and a belly flop.
- It replaces ≈ 13.5 t of flaps with ≈ 4.2 t of magnet system plus ≈ 1.7 t of extra cold gas.
