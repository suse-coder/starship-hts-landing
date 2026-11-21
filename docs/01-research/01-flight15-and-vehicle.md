# research/01 — Flight 15 and the Block 3 ship

## Flight 15 (sourced: Wikipedia "Starship flight 15", read 2026-11-20)

| Item | Value |
|---|---|
| Launch | 2026-11-13 22:47:36 UTC, Orbital Launch Pad 2, Starbase |
| Vehicles | Booster 22 (lost in a contingency splashdown), **Ship 42** (Block 3, "Don't Panic") |
| Orbit | low Earth orbit, ≈ 275 km |
| Mission | 26 Starlink V3 deployed by ≈ T+01:17:30; ≈ 14 orbits, ≈ 22.5 h |
| Return | deorbit burn targeting the Pacific "several thousand km west of Starbase", glide around populated areas of northern Mexico and south Texas |
| Catch | Pad 2 arms at T+22:32:30 → **2026-11-14 21:20:06 UTC** (3:20 pm CST) |

Not published: inclination, deorbit Δv and time, entry interface state. Assumptions (plan/05):
**i = 26.5°** (a Starbase launch due east gives i ≈ 26°; returning over the pad needs i ≥ 26.0°),
**Δv = 105 m/s retrograde, 480 s after the simulation start**, RAAN and phase solved so the
trajectory ends at Pad 2 at the real catch time (`packages/gnc/src/plan.ts`). The ground track
then approaches from the WSW across Mexico — the same geometry as the real return.

## Block 3 ship (sourced where marked; reconstructed otherwise)

| Item | Value | Status |
|---|---|---|
| Height / diameter | 52.1 m / 9 m | sourced (Block 2 height; stack 124.4 m − booster 72.3 m) |
| Barrel rings | 1.83 m tall, 3.97 mm, ≈ 1.6 t each, 301/30X stainless | sourced |
| Dry mass | "≈ 100 t" (2021 statement) | sourced, dated |
| Propellant | 1500 t (Block 2); ≈ 1600 t Block 3 | sourced / assumed |
| Engines | 3 Raptor 3 SL (250 tf, 1525 kg) + 3 RVac (275 tf) | sourced |
| Raptor 3 SL Isp | 350 s SL → 369 s vacuum (A_e 1.33 m²) | assumed / computed |
| RCS | cold gas from ullage; 4 thrusters below the payload bay, 2 on the LOX tank | sourced |
| Header tanks | in the nose; LOX header forms the nose tip, CH4 header below | sourced |
| Heat shield | ≈ 18 000 silica tiles, 1400 °C, ≈ 10.5 t incl. ablative backing (Block 1) | sourced |
| Flaps | 4 (2 forward, 2 aft), estimated 12–15 t together | user-provided estimate |
| Catch points | lifting points in the nosecone | sourced |

## Our flapless derivative ("Ship 42-M")

> **Superseded design note (2026-11-21, Q23).** The current design keeps the real silica tile heat
> shield; the early-concept removals and the 95.3 t / 116.5 t masses below are **not current**.
> The current budget is dry **107.40 t**, wet **143.65 t** at entry, x_cg 27.57 m (spec/01, printed
> by `bun scripts/massbudget.ts`). See Q23 in plan/05 and spec/11 for the release status.

Removed in the early concept (superseded by Q23): four flaps (≈ 13.5 t), tiles + ablative backing
(≈ 10.5 t), flap actuators. The current design keeps the tiles.
Added: PICA-X nose cap 5.37 t (389 m²), HTS coil system 4.15 t, larger cold-gas system 1.5 t +
2.0 t N2, Cs seed 0.4 t, high-ε coating. Dry 95.3 t; start mass 116.5 t (spec/01 has the table,
printed by `bun scripts/massbudget.ts`). Planform 439 m², planform centroid x = 24.55 m; entry CG
x ≈ 25.6 m (forward of the centroid → the belly flop trims nose-low at α ≈ 84° subsonic).
