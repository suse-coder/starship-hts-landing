# spec/01 — Vehicle: Ship 42 (flapless HTS-MHD derivative of Starship V3)

Numbers printed by `bun scripts/massbudget.ts` (2026-11-21). Geometry sources: research/05.
Body frame: x along the axis from the aft skirt plane (x = 0) to the nose tip (x = 52.1 m); y, z
complete a right-handed frame; landing engine RSL1 at +z.

Status columns: **M** modelled (mass, CG, inertia) · **P** physics coupling (forces, heat,
electrics) · **R** rendered in the 3D view. `[x]` done · `[~]` partial · `[ ]` open.

## 1. Requirements

| ID | Requirement | Status |
|---|---|---|
| V-01 | OML = 21 barrel rings × 1.83 m (38.43 m) + 13.67 m nosecone, Ø 9 m, total 52.1 m | [x] |
| V-02 | Blunted ogive nose, R_n = 2.5 m (concept, plan/05 Q6), PICA-X over the nosecone only | [x] |
| V-03 | No flaps; silica tile heat shield on the barrel, PICA-X nose (Q23, 2026-11-21); 301/30X steel is the tiled backing | [x] |
| V-04 | 3 Raptor 3 SL (gimballed ±15°, 20°/s) + 3 RVac (fixed) | [x] M/P · [~] R |
| V-05 | Cold-gas RCS: 12 thrusters × 4 kN, 6 pure couples, 5 t N₂ at 300 bar | [x] |
| V-06 | HTS REBCO coil in the nose (a = 1.75 m, 9.93 MA·t, 291 MJ, 25 K) with 8 electrode sectors | [x] |
| V-07 | Catch pins at the nosecone base (x = 39.63 m, ±y, tip radius 5.3 m) | [x] |
| V-08 | Every component in §3 rendered with real proportions; PICA-X honeycomb on the nosecone | [ ] |
| V-09 | Mass properties recomputed from consumables each step (CG, inertia) | [x] |
| V-10 | Total ≈ 150–250 k triangles for the detailed model at 60 fps (desktop) | [ ] |

## 2. Outer mould line (`packages/craft/src/geometry.ts`)

| Item | Value | Status |
|---|---|---|
| Barrel | x ∈ [0, 38.43] m, r = 4.5 m (21 rings) | [x] |
| Nosecone | tangent ogive, length 13.67 m, spherical cap R_n = 2.5 m, cap end x = 50.47 m | [x] |
| Planform | 442.7 m², centroid x = 24.70 m | [x] |
| Aero panels / heating cells | 16 per station (16 stations) / station × 16 rays | [x] |
| Ring seams (21), nosecone segment seams (2), PEZ door outline, vents | rendered detail | [ ] |

## 3. Product breakdown structure (PBS)

### 3.1 Structure

| ID | Component | Real reference | Model value | M | P | R | Source |
|---|---|---|---|---|---|---|---|
| S-01 | Barrel rings 1–21 | 301 steel, 3.97 mm, 1.83 m, ≈ 1.6 t each (Wikipedia) | 34.21 t (1.63 t per ring), x 19.2 | [x] | [x] heat cells | [~] hull, no seams | computed (21 × 2πR·h·t·ρ, ρ 7930 kg/m³) |
| S-02 | Nosecone (2 segments, frames, lift-point reinforcement) | two segments, reinforced for chopstick lift | 11.96 t, x 45.3 | [x] | [x] | [~] | assumed (inside the ≈ 100 t dry statement) |
| S-03 | Aft skirt / engine bay, 6 shield compartments | shielded compartment per engine, purge vents | 4.5 t, x 2.8 | [x] | [ ] base drag only | [ ] | assumed |
| S-04 | Aft dome + thrust puck + thrust cone | thrust puck under the LOX tank | 3.5 t, x 4.0 | [x] | – | [~] disk | assumed |
| S-05 | Common dome (LOX / CH₄) | between the LOX (aft) and CH₄ (fwd) tanks | 2.0 t, x 21.5 | [x] | – | [~] disk | assumed |
| S-06 | Forward dome | top of the CH₄ tank | 1.8 t, x 31.6 | [x] | – | [~] | assumed |
| S-07 | Stringers + internal structure | stringers increased Block 2+ | 3.5 t | [x] | – | [~] | assumed |
| S-08 | Payload bay + PEZ door + dispenser | PEZ door folds into the bay | 4.0 t, x 35.0 | [x] | – | [ ] door | assumed |
| S-09 | Catch pins ×2 | chopstick lift points | in S-02 | [x] | [x] contact | [x] | sourced (research/01) |
| S-10 | Tile heat shield (LI-900 class, 3 cm + RTV/pins), no ablative backing | ≈ 18 000 hex tiles, 10.5 t incl. ablative backing (Block 1, Wikipedia) | 6.24 t, 1072 m², x 18.97; `craft/src/tiles.ts`, thermal layer `heat/src/thermal.ts` | [x] | [x] | [ ] | computed: 1072 m² × 5.82 kg/m²; lower than 10.5 t because the flap tiles are removed and there is no ablative backing. Conservative alternative: 10.5 t (+4.26 t). **Survival:** latest sweeps peak at 1650 K (14:08 integrated run: 1684 K). Against the conservative **NASA LI-900/HRSI 1530 K** limit: over by 120 K (1650) / 154 K (1684). Against the **public SpaceX 1673 K claim** (secondary source, torch test, not a qualified reuse limit): 1650 K passes by 23 K, 1684 K fails by 11 K; the unretargeted zero-α run is 2417 K (fails both). Per user decision 2026-11-21 the release gate uses the **SpaceX 1673 K flight-tile claim**: 1650 K passes by 23 K, the 1684 K run fails by 11 K; NASA 1530 K is reported as a margin. Backing 363 K (345 K integrated) passes; PICA bondline 695 K (701 K) vs 560 K fails. Q30: no TPS-only fix (extend-PICA buys 0 K); the flux must fall 1.75×, i.e. hold the high-α plateau at α ≲ 22° or pitch up at ≲ 4.5 km/s |

### 3.2 Propulsion

| ID | Component | Real reference | Model value | M | P | R | Source |
|---|---|---|---|---|---|---|---|
| P-01 | Raptor 3 SL ×3 (RSL1–3) | 250 tf SL, gimballed, r = 1.3 m at 90/210/330°; 1525 kg each (research/01) | 2.45 MN, I_sp 350/369 s, 40–100 %, spool 2.5 s, 4.58 t | [x] | [x] thrust, gimbal, TVC | [~] bells only | sourced 1525 kg/engine |
| P-02 | Raptor 3 Vac ×3 | Ø 2.3 m exit, ≈ 4.3 m long, r = 3.2 m | not fired on return, 6.30 t | [x] | – | [~] bells only | assumed 2100 kg/engine |
| P-03 | Raptor subcomponents: powerhead, LOX/CH₄ turbopumps, preburners, MCC, nozzle, TVC actuators ×2 | full-flow staged combustion; electric TVC since flight 2 (Wikipedia) | geometry only (mass in P-01/P-02) | – | – | [ ] | assumed (actuator mass not published) |
| P-04 | Downcomers ×4 (3 RVac + 1 centre to the sump) + CH₄ sump | research/05; 4 downcomers + sump (Wikipedia) | 3.0 t incl. valves | [x] | – | [ ] | assumed |
| P-05 | LOX header (nose tip in the real ship; here below the coil) + CH₄ header | concept moves the LOX header aft of the coil | 20.35 t LOX + 5.65 t CH₄ at start; tank structure 1.91 t (Ø 3.24 / 2.94) | [x] | [x] mass/CG | [ ] | computed (Q24: 26 t, O/F 3.6; tank mass ∝ area) |
| P-06 | Main LOX / CH₄ tanks (residuals) | aft LOX, fwd CH₄ | 3.80 / 1.05 t | [x] | [x] | [ ] | computed (deorbit 3.3 t + residual 1.55 t at O/F 3.6) |
| P-07 | Purge / fire suppression (N₂) | engine bay purge, GN₂ (Wikipedia) | 0.8 t | [x] | – | [ ] | assumed |
| P-08 | Engine-start COPVs ×6 around the CH₄ header | several COPVs around the CH₄ header for start gas (Wikipedia) | 0.90 t (150 kg each) | [x] | – | [ ] | assumed (optimistic; conservative 300 kg each) |
| P-09 | Header lines ×2 (vacuum-jacketed) + chill lines ×4, vent lines, bellows | vacuum-jacketed propellant lines, 4 chill lines (Wikipedia) | 1.66 t (1.26 t lines + 0.40 t) | [x] | – | [ ] | computed lines (Ø 0.20/0.25 m, 2/1 mm, 39 m); assumed rest (conservative 1.0 t) |

### 3.3 Reaction control

| ID | Component | Real reference | Model value | M | P | R | Source |
|---|---|---|---|---|---|---|---|
| C-01 | N₂ COPVs (300 bar) | real ship: cold gas from residual ullage gas, no dedicated N₂ load (Wikipedia) | 5.0 t gas + 3.4 t dry (0.6 kg/kg) | [x] | [x] | [ ] | computed COPV mass (0.6 kg/kg); concept addition, not a real mass item |
| C-02 | Nose thrusters ×4 (x 44.6) | real: 4 below the payload bay | 4 kN, I_sp 73 s | [x] | [x] couples | [~] puffs | assumed |
| C-03 | Aft thrusters ×4 + roll ×4 (x 5.5) | real: 2 on the LOX tank | 4 kN | [x] | [x] | [~] | assumed |

### 3.4 Thermal protection

| ID | Component | Real reference | Model value | M | P | R | Source |
|---|---|---|---|---|---|---|---|
| T-01 | PICA-X nosecone TPS | (concept replacement for the nose tiles) | 332 m², 6.5 → 2.5 cm, 4.62 t, ablation + charring | [x] | [x] 1-D conduction, recession | [~] dark, no honeycomb | computed (270 kg/m³ + 3 kg/m² carrier) |
| T-02 | High-ε coating on bare steel | – | ε = 0.85, 0.25 t | [x] | [x] radiation | [x] | assumed |

### 3.5 HTS-MHD system (concept)

| ID | Component | Model value | M | P | R | Source |
|---|---|---|---|---|---|---|
| H-01 | REBCO winding | NI 9.93 MA·t, 3310 turns × 3 kA, J_e 700 A/mm², 1.39 t | [x] | [x] Biot–Savart, I_c(B, T) | [x] torus | computed (designCoil) |
| H-02 | CF hoop structure (virial-sized) | 0.77 t | [x] | [x] mass ∝ E | [ ] | computed (virial) |
| H-03 | Cryostat, MLI, shield, SN₂ buffer | 0.35 t | [x] | [x] heat leak | [ ] | computed |
| H-04 | Cryocoolers ×2 (25 K) | 180 W lift, 45 W/W | [x] | [x] | [ ] | computed (1.4 kg/W) |
| H-05 | Electrode sectors ×8 + power converter + dump resistor | loading K_s ∈ [−0.6, 1], τ_dump 2 s | [x] | [x] | [x] glow | computed (dump resistor 1.37 t, Q19 open) |
| H-06 | Coil charge battery | 100 kWh, 0.5 t | [x] | [x] ramp | [ ] | computed (200 Wh/kg) |
| H-07 | Caesium seed tank + injectors | 400 kg Cs, 1 % mole fraction; dry tank 90 kg | [x] | [x] seeding | [ ] | assumed; T08: Cs was double-counted, dry is 90 kg + 400 kg consumable |

### 3.6 Avionics, power, communication

| ID | Component | Model value | M | P | R | Source |
|---|---|---|---|---|---|---|
| A-01 | Flight computers, IMU/GNSS | in 3.2 t avionics | [x] | perfect nav (I1) | [ ] | assumed |
| A-02 | Batteries (vehicle), harness, external cable raceway | in 3.2 t | [x] | – | [ ] | assumed |
| A-03 | Starlink antennas ×4 (nosecone) | – | [x] | blackout flag | [ ] | assumed (4 antennas sourced, Wikipedia) |

## 4. Mass budget (start of the return)

Q23 (2026-11-21): all the real Starship's mass except the flaps is carried, with the concept's
replacements counted at their own mass. Printed by `bun scripts/massbudget.ts` (2026-11-21).

Dry **107.40 t** = structure 66.27 + propulsion 18.35 + TPS 11.11 (PICA-X 4.62 + tiles 6.24 +
coating 0.25) + HTS-MHD 5.07 + RCS 3.40 + avionics 3.20. The tile mass is the S-10 item
(`craft/src/tiles.ts`, Q23); it is computed from the tiled area, not the Block 1 10.5 t figure.

Consumables LOX header 20.35 t, CH₄ header 5.65 t (26 t, Q24), main residuals 3.80 / 1.05 t,
N₂ 5.0 t, Cs 0.40 t → start mass **143.65 t**, x_cg 27.57 m, I = (1.65e6, 4.33e7, 4.33e7) kg m².

Gaps closed in this pass (R5: each has a conservative alternative in §3):

| Item | Before | After | Basis |
|---|---|---|---|
| S-01 barrel | 31.5 t | 34.21 t | 1.63 t per ring from Ø 9 m × 1.83 m × 3.97 mm × 7930 kg/m³ (sourced ring mass ≈ 1.6 t) |
| P-05 header tank structure | 1.60 t | 1.91 t | 26 t of propellant (31.2 m³ vs 24.0 m³); mass ∝ V^2/3 |
| P-08 engine-start COPVs | – | 0.90 t | 6 × 150 kg, assumed (conservative 300 kg each) |
| P-09 header lines, chill/vent lines, bellows | – | 1.66 t | 1.26 t computed (2 × Ø 0.20/0.25 m, 39 m) + 0.40 t assumed (conservative 1.0 t) |
| S-10 tiles (Q23) | – | 6.24 t | 1072 m² × 5.82 kg/m² (LI-900 3 cm + RTV/pins); conservative 10.5 t Block 1 figure with ablative backing (+4.26 t) |

Reference for the total: real dry mass ≈ 100 t (2021 statement, Block 1, sourced; research/01) and
85 t for Block 2 (Wikipedia, flagged "better source needed"). The bottom-up 107.40 t is above the
dated Block 1 number; the conservative tile alternative (10.5 t instead of 6.24 t) gives 112.06 t.
That is consistent with the longer Block 3 (52.1 m vs 50.3 m) and six larger engines.

## 5. HTS magnet (design function `designCoil`)

a = 1.75 m at x = 50.9 m, L = 64.6 H, E = 291 MJ, B_centre 3.57 T, B_stag 2.0 T, B_peak 14.3 T,
T_op 25 K, T_cs 41.6 K, ramp ≈ 11 min at 450 kW. Mass scales with the design field.

## 6. Catch interface

Pins at x = 39.63 m (nosecone base), ±y, tip radius 5.3 m. Mechazilla: tower 146 m, 23 m north of
the mount; arm rails at 112 m; gap 26 m open / 9.2 m closed; closing 3 s; pin dampers 2.4 MN/m,
0.9 MN s/m; μ 0.35.
