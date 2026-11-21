# spec/07 — CAD model: assembly tree, part geometry, materials

The 3D model is built like a Fusion 360 / Blender assembly: every part is a named body with a
parametric primitive, real dimensions, a position in body coordinates, a material, and a link to
its mass item (spec/01 PBS) and its physics coupling. Code: `packages/view/src/ship.ts` (+ part
builders). The hull itself is the **physics OML** (`craft/geometry.ts`), so what is drawn is what is
simulated (TS-09).

Conventions: body x from the aft skirt plane (0) to the nose tip (52.1 m); y, z right-handed; φ
measured from +y toward +z; dimensions in metres; Ø = diameter. Sources: research/05, Wikipedia
(2026-11-21), SCpp/x7k2m9q cross-checks (map/30), our design (concept items marked ◆). Computed
sizes come from the consumable masses (e.g. LOX header 20.35 t / 1141 kg m⁻³ = 17.83 m³).

Status: `[x]` built and rendered · `[~]` simplified · `[ ]` open.

## 1. Assembly tree

```
SHIP-42 (flapless HTS-MHD)
├── 10 STRUCTURE
│   ├── 11 Barrel: rings R01–R21 (Ø 9.00 × 1.83 × 0.00397), weld seams W01–W20
│   ├── 12 Aft skirt + engine bay: skirt shell, 6 engine shield compartments, purge vents ×24
│   ├── 13 Thrust puck + thrust cone + aft dome (LOX tank bottom)
│   ├── 14 Common dome (LOX | CH₄), CH₄ sump
│   ├── 15 Forward dome (top of CH₄ tank)
│   ├── 16 Payload bay: barrel section, deck, PEZ door + frame, dispenser rails
│   ├── 17 Nosecone: lower segment, upper segment, frames ×8, stringers ×24
│   ├── 18 Catch hardpoints: pins ×2 (±y), bumpers ×2
│   └── 19 External cable raceway (leeward, x 2 → 41)
├── 20 PROPULSION
│   ├── 21 Raptor 3 SL ×3 (RSL1 90°, RSL2 210°, RSL3 330°; r = 1.3 m)
│   │       powerhead · LOX turbopump · CH₄ turbopump · preburners ×2 · MCC · nozzle · TVC actuators ×2
│   ├── 22 Raptor 3 Vac ×3 (r = 3.2 m; 30°, 150°, 270°): as 21 + large nozzle with stiffener bands ×4
│   ├── 23 Main tanks: LOX tank (x 3.6–21.5), CH₄ tank (x 21.5–31.6)
│   ├── 24 Downcomers: centre CH₄ (Ø 0.80) to the sump, RVac CH₄ ×3 (Ø 0.30), header lines ×2 (Ø 0.20)
│   ├── 25 Header tanks ◆ (moved aft of the coil): LOX sphere Ø 3.24 at x 46.6, CH₄ sphere Ø 2.94 at x 43.6 (clash: x ≤ 43.50)
│   └── 26 Vents: header vents ×2 (nose), engine bay vents (see 12)
├── 30 REACTION CONTROL ◆ (concept: 15× capacity)
│   ├── 31 COPVs ×12 (Ø 0.90 × 2.40, 15.6 m³ N₂ at 300 bar) on a ring at x 32.5
│   ├── 32 Nose thruster pods ×4 at x 44.6 (±y, ±z)
│   └── 33 Aft thruster pods ×4 + roll pods ×2 at x 5.5
├── 40 THERMAL PROTECTION
│   ├── 41 PICA-X nosecone TPS ◆: hexagonal-cell honeycomb carrier, 6.5 → 2.5 cm, x ≥ 37.93
│   └── 42 High-ε coating on bare steel (ε 0.85)
├── 50 HTS-MHD SYSTEM ◆
│   ├── 51 REBCO coil (torus R = 1.75, winding pack 0.50 × 0.08) at x 50.90
│   ├── 52 CF hoop structure (around 51)
│   ├── 53 Cryostat (torus, tube Ø 0.70) + MLI + thermal shield
│   ├── 54 Cryocoolers ×2 (0.6 × 0.6 × 0.8) at x 48.9, φ 0°/180°
│   ├── 55 Electrode sectors ×8 on the cap (0.9 × 0.9, x ≈ 51.2) + power converter ring (x 49.6) + dump resistor
│   ├── 56 Coil charge battery (disk Ø 2.0 × 0.6) at x 48.4
│   └── 57 Caesium seed tank (sphere Ø 0.80, on axis inside the coil bore) at x 50.2 + injector ring
└── 60 AVIONICS · POWER · COMMS
    ├── 61 Flight computers / IMU / GNSS bay (payload-bay deck)
    ├── 62 Vehicle batteries ×4 (aft and forward)
    ├── 63 Harness trunks (inside the raceway 19)
    └── 64 Starlink antennas ×4 (flush patches on the nosecone, φ 45/135/225/315°)
```

## 2. Part table

| Part | Primitive / profile | Key dimensions | Position (x; r, φ) | Material | Mass item | Physics | Status |
|---|---|---|---|---|---|---|---|
| 11 Barrel | lathe of the OML | Ø 9.00, x 0–38.43 | – | 301/30X steel, ε 0.85 | S-01 | aero panels, heat cells | [x] |
| 11 Weld seams | thin torus ×20 | every 1.83 m | x = 1.83 k | polished weld bead | S-01 | – | [ ] |
| 12 Skirt shield compartments | 6 vertical plates + ring | x 0–4.4 | engine positions | steel | S-03 | base drag | [ ] |
| 12 Bay vents ×24 | recessed boxes 0.40 × 0.25 | x 2.2 | φ 15° steps | steel | S-03 | – | [ ] |
| 13 Thrust puck | cylinder Ø 3.6 × 0.6 + cone | x 3.6–4.6 | axis | steel forging | S-04 | – | [ ] |
| 13 Aft dome | ellipsoid cap, depth 2.2 | apex x 3.6 | axis | steel | S-04 | – | [~] disk |
| 14 Common dome | ellipsoid cap, depth 2.2 | x 21.5 | axis | steel | S-05 | – | [~] disk |
| 14 CH₄ sump | cone Ø 1.4 → 0.8, h 1.2 | x 20.3 | axis | steel | P-04 | – | [ ] |
| 15 Forward dome | ellipsoid cap, depth 2.2 | x 31.6 | axis | steel | S-06 | – | [~] |
| 16 PEZ door | panel 3.0 (circumf.) × 6.0 | x 32.0–38.0 | φ 90° (+z, leeward) | steel | S-08 | – | [ ] |
| 17 Nosecone frames ×8 | torus at local r | x 39–50 | – | steel | S-02 | – | [ ] |
| 18 Pins ×2 | box 1.0 × 1.6 × 0.9 | – | x 39.63; ±y, tip r 5.3 | steel | S-09 | tower contact | [x] |
| 19 Raceway | box 0.30 × 0.20 × 39 | x 2–41 | r 4.6, φ 90° | steel | A-02 | – | [ ] |
| 21 Raptor SL powerhead | cylinder Ø 1.20 × 0.9 + pumps | x 3.6–4.5 | r 1.3, φ 90/210/330 | Inconel | P-01 | mass/inertia | [ ] |
| 21 Raptor SL nozzle | lathe: throat r 0.145 → exit r 0.65 | length 2.2 | exit x ≈ 1.0 | Inconel/copper | P-01 | thrust, gimbal | [~] cone |
| 21 TVC actuators ×2 | cylinders Ø 0.12 × 1.0 | – | per engine | steel | P-01 | gimbal rate | [ ] |
| 22 RVac nozzle | lathe: exit r 1.15, 4 stiffener bands | length 4.3 | exit x ≈ 0.3; r 3.2 | Nb alloy | P-02 | – | [~] cone |
| 23 LOX tank | cylinder volume (internal) | x 3.6–21.5 | axis | – | P-06 | propellant mass | [ ] (internal view) |
| 23 CH₄ tank | cylinder | x 21.5–31.6 | axis | – | P-06 | – | [ ] |
| 24 Centre downcomer | tube Ø 0.80 | x 4.6 → 20.3 | axis | steel | P-04 | – | [ ] |
| 24 RVac downcomers ×3 | tube Ø 0.30 | x 4.6 → 21.5 | r 2.4 | steel | P-04 | – | [ ] |
| 24 Header lines ×2 | tube Ø 0.20 | x 4.6 → 43.6 | r 4.1 | steel | P-04 | – | [ ] |
| 25 LOX header ◆ | sphere Ø 3.24 (R 1.6205), 3.24 m long + sump cone | x 46.6 | axis | steel | P-05 | 20.35 t (17.83 m³), CG | [ ] |
| 25 CH₄ header ◆ | sphere Ø 2.94 (R 1.4725), 2.94 m long + sump cone | x 43.6 (clash: R sum 3.093 m > 3.00 m spacing, so x ≤ 43.50) | axis | steel | P-05 | 5.65 t (13.37 m³), CG | [ ] |
| 31 COPVs ×12 | capsule Ø 0.90 × 2.40 | ring r 3.4 | x 32.5, φ 30° steps | CFRP/Al liner | C-01 | N₂ mass | [ ] |
| 32/33 RCS pods | nozzle cone 0.25 × 0.4 | – | per thruster | steel | C-02/03 | couples | [~] puffs |
| 41 PICA-X | OML offset + hex-cell normal/colour map, 3 cm cells | x ≥ 37.93 | – | PICA-X (carbon/phenolic) | T-01 | ablation, char | [~] dark colour |
| 51 REBCO coil | torus R 1.75, tube 0.25 | x 50.90 | axis | REBCO/Cu | H-01 | B field | [x] |
| 53 Cryostat | torus R 1.75, tube 0.35 (transparent shell) | x 50.90 | axis | Al/MLI | H-03 | heat leak | [ ] |
| 54 Cryocoolers ×2 | box 0.6 × 0.6 × 0.8 | x 48.9 | r 1.0, φ 0/180 | Al | H-04 | lift | [ ] |
| 55 Electrodes ×8 | curved patches 0.9 × 0.9 | x ≈ 51.2 | cap, φ 22.5 + 45 k | W/Cu | H-05 | loading | [x] |
| 56 Battery | disk Ø 2.0 × 0.6 | x 48.4 | axis | Li-ion | H-06 | ramp | [ ] |
| 57 Cs tank | sphere Ø 0.80 | x 50.2 | axis | steel | H-07 | seeding | [ ] |
| 64 Starlink antennas ×4 | flush panels 1.2 × 0.8 | x 41.5 | φ 45 + 90 k | composite | A-03 | blackout | [ ] |

## 3. Materials (PBR)

| Material | Base colour (sRGB) | Metalness | Roughness | Notes |
|---|---|---|---|---|
| 301/30X steel (bare, brushed) | #b9bec4 | 0.9 | 0.30 | anisotropic sheen, weld beads darker |
| High-ε coating | #8f969d | 0.6 | 0.55 | over the barrel when selected |
| PICA-X | #1d1b19 | 0.0 | 0.9 | hex honeycomb, char darkening with recession |
| Inconel nozzle | #6b5f55 | 0.8 | 0.45 | heat tint at the throat |
| REBCO / cryostat | #c58b3c / #b5bcc4 | 0.7 | 0.35 | |
| CFRP COPV | #2b2e33 | 0.3 | 0.5 | weave normal map |

## 4. Level of detail and budgets

LOD0 (≤ 150 m camera distance): all parts, ≈ 180 k triangles. LOD1 (> 150 m): hull, engines,
pins, overlays. Instancing for repeated parts (seams, vents, COPVs, pods, electrodes).

## 4. Departures from the part table (forced by clashes, agent C, 2026-11-21)

| Row | Spec | Built | Reason |
|---|---|---|---|
| 13 aft dome | depth 2.2 m | depth 3.8 m | at 2.2 m the dome would cut through the RVac mounts at x 4.6, r 3.2 |
| 19 raceway | φ 90° | φ 112.5° | at 90° it runs through the PEZ door and the +z RCS pods |
| 53 cryostat | tube Ø 0.70 m | Ø 0.64 m | the R_n 2.5 m nose cap leaves 0.32 m from the coil centre line to the hull |
| 54 cryocoolers | r 1.0 m | r 1.45 m | at r 1.0 m they intersect the coil battery |
| 13–15 domes | x read as the tank boundary | x read as the apex position | the only reading consistent with the tank ranges |
| 11 aft end | closed by a disc (not in the table) | open skirt | the engines and shield deck are visible from below, as on the real vehicle |

## 5. Pad 2 environment (T03, 2026-11-21)

Tags: [T] = `TOWER` constants, [W] = Wikipedia "SpaceX Starbase" (read 2026-11-21), [E] = estimated from photos.

| Component | Dimensions | Source |
|---|---|---|
| Tower lattice | 12 m square, 9 sections × 14.8 m, top 134 m; corner columns; X-bracing; splice collars | sections [W]; rest [E] |
| Chopsticks | rail top 112 m; inner faces at ±gap/2 (parallel: they slide, they do not rotate); tip at n −17 m; 11-panel Warren truss; 9 bumper pads | [T] contact model; truss [E] |
| Carriage | 15 m tall frame at 103–118 m around the tower | cable hoist and side bearings [W]; sizes [E] |
| Launch mount | 46 × 29 m pedestal, 6.5 m high; table 27.2 × 26 m; cooled plate R 11.6 m; plate top at 20 m; 20 hold-down clamps | clamps and cooled plate [W]; 20 m [T]; rest [E] |
| Flame trench | 17 m wide, running south (away from the tower) | steel-pipe flame diverter [W]; layout [E] |
| Tank farm | 8 vertical tanks Ø 10.4 × 28.8 m; 10 horizontal Ø 3.8 × 22 m | stored fluids [W]; sizes [E] |
