# research/02 — Thermal protection materials

## PICA-X (nose cap)

PICA (phenolic-impregnated carbon ablator, NASA Ames, 1990s): a carbon-fibre preform (FiberForm)
partly filled with phenolic resin. Flown on Stardust (peak 1.2 kW/cm² at 12.5 km/s) and MSL.
SpaceX's PICA-X (2006–2010, Dragon) and PICA-3 (Crew Dragon) are proprietary improvements;
their open-literature properties are PICA's, which we use as a proxy.

| Property | Value used | Note |
|---|---|---|
| virgin density | 270 kg/m³ | PICA 0.22–0.27 g/cm³ |
| char density | 220 kg/m³ | ≈ 80 % char yield of the resin fraction |
| k virgin | 0.20 + 1e-4 (T−300) W/m K | through-thickness |
| k char | 0.4 + 6e-4 (T−300), ≤ 2.2 W/m K | radiation in pores at high T |
| cp virgin / char | 1000…1900 / 1100…2000 J/kg K | |
| pyrolysis | first order, A = 1.4e4 s⁻¹, E/R = 8556 K, ΔH 0.3 MJ/kg (endo) | onset ≈ 600 K (lumped) |
| surface | ε 0.9; diffusion-limited C oxidation B'c = 0.17 above ≈ 1700 K (half of 9.2 MJ/kg released at the wall) | |
| bond | RTV-560-like, 0.3 mm, limit ≈ 560 K | design limit for the bondline |

Design: 6.5 cm on the spherical cap, tapering to 2.5 cm at x = 35.5 m (the cylinder junction),
389 m² → 5.37 t including 3 kg/m² carrier and bond.

**Simulated (current run):** stagnation flux 93 W/cm² peak (≈ 8 % of PICA's Stardust rating),
surface ≈ 1880–2070 K, bondline ≤ 373 K, recession ≈ 0.1 mm — the cap is conservative; plan/05
Q7 offers thinning it.

## Bare steel (everything aft of x = 35.5 m)

30X (SpaceX alloy, 304L-family austenitic): ρ 7900 kg/m³, cp 450 → 650 J/kg K (300 → 1000 K),
k 16–25 W/m K (thin wall: Biot ≪ 1 → lumped), high-emissivity oxidised/coated surface
ε = 0.85. Strength falls steeply above ≈ 1100–1150 K (design limit **1150 K**), melting ≈ 1670 K
(failure assumed at 1400 K). Wall 3.97 mm → 17 kJ/m² K areal heat capacity, radiative time
constant ≈ 80 s at 1000 K.

**Implication (computed, `scripts/heattest.ts`):** windward bare steel at α 20–35° and
V > 4 km/s reaches 1300–1700 K radiative equilibrium once the boundary layer is turbulent;
at α 10–14° it stays at 700–1000 K. In the passive high-α trim (α ≈ 51–55°) the windward ray
would sit at 1130–1470 K for V 3–5.5 km/s — hence the barbecue roll (each ray windward only
part of a turn) and the thermal mass. **Simulated:** steel peak ≈ 920–1000 K.

### Flight 15 tile cross-check (2026-11-21)

The real Flight 15 vehicle's tiles survived reentry (user-provided observation). That is consistent
with the public **SpaceX 1673 K** flight-tile claim and indicates the conservative NASA LI-900/HRSI
1530 K limit is a design margin, not the flight tile's actual limit. Against the model's speed/heat
profile:

| run | tile surface | vs NASA 1530 K | vs SpaceX 1673 K |
|---|---:|---:|---:|
| best high-α | 1650 K | −120 K (fail) | **+23 K (pass)** |
| 14:08 integrated | 1684 K | −154 K (fail) | −11 K (fail) |
| zero-α unretargeted | 2417 K | far fail | far fail |

Tile survival does **not** validate the PICA bondline: the current 695 K is above the RTV-560 560 K
limit. The steel backing (301/30X) is 363 K and safe. Per user decision 2026-11-21 the release gate
uses the **SpaceX 1673 K flight-tile claim**; the NASA 1530 K line is reported as a margin, not hidden.
The best 1650 K run passes by 23 K; the 1684 K run fails by 11 K.

### Strength at temperature (2026-11-21, plan/01 TS-16)

0.2 %-proof strength retention k = f_y(T)/f_y(20 °C) for austenitic stainless grade 1.4301 (304),
EN 1993-1-2 Annex C, Table C.1. Values as recalled; to be checked against the standard (open
question Q18). Cold-worked 301/30X loses its cold-work premium above ≈ 800 K, so the annealed curve
is a fair proxy in the hot condition.

| T (°C) | T (K) | k_0.2p | Band used in the views |
|---|---|---|---|
| 20 | 293 | 1.00 | nominal |
| 400 | 673 | 0.60 | nominal (≤ 700 K) |
| 600 | 873 | 0.49 | reduced (≤ 900 K) |
| 700 | 973 | 0.40 | within design |
| 800 | 1073 | 0.27 | within design |
| 900 | 1173 | 0.14 | design limit at 1150 K (k ≈ 0.17) |
| 1000 | 1273 | 0.06 | strength loss |
| 1100 | 1373 | 0.03 | strength loss |
| 1200 | 1473 | 0.00 | failure (≥ 1400 K) |

Consequence: at the simulated peak (≈ 1000 K) the windward steel keeps ≈ 35 % of its room-temperature
yield. That is acceptable only because the tanks are pressure-stabilised and descent loads are low.
Code: \`packages/heat/src/limits.ts\` (\`steelYieldRetention\`, \`steelVerdict\`, \`THERMAL_LIMITS\`).
