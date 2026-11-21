# research/05 — Vehicle geometry and components: sources and cross-check (2026-11-21)

**Question (user):** are the geometry and all components correct? Use a GitHub Blender project
for Starship if possible, otherwise recreate the model.

## 1. GitHub search (2026-11-21)

Searched via the GitHub API: "starship blender", "spacex starship 3d model", "starship gltf/glb".

| Repository | Licence | Content | Usable? |
|---|---|---|---|
| farorizero2-ux/starship_V3_blender-project | MIT | `.blend` + `.glb` (5.6 MB): **Super Heavy booster only**, 14 primitive meshes | no: no ship, crude |
| flaceja/starship-edit | MIT | procedural Blender scripts for a stylised animation (booster hull, fins) | no: stylised, not a ship OML |
| batturamesh7771-sketch/Starship-V3-…-SolidWorks | MIT | SolidWorks parts + VBS scripts; partly AI-generated ("150 m stack", "70 m Stage 2") | cross-check only: Raptor 3 SL Ø 1.3 m × 3.1 m, RVac Ø 2.4 m × 4.3 m |
| others (Spiri0/SpaceX-Starship, …) | none | — | no licence: cannot be reused |

**Decision:** recreate the model procedurally. Our ship is a modified vehicle (no flaps, blunt
PICA-X cap R_n = 2.5 m, HTS coil in the nose), and its outer mould line drives the physics
(aero panels, heating cells, MHD cells). It must therefore stay one parametric source
(`packages/craft/src/geometry.ts`), rendered from the same function. An imported mesh would show a
different vehicle from the one simulated (G0/R2).

## 2. Facts used (Wikipedia "SpaceX Starship (spacecraft)", "SpaceX Raptor", read 2026-11-21)

| Item | Real vehicle | Model before | Model now |
|---|---|---|---|
| Height | 52.1 m (Block 2; V3 stack 124.4 m − booster 72.3 m = 52.1 m) | 52.1 m | 52.1 m |
| Diameter | 9 m | 9 m | 9 m |
| Barrel | 21 rings × 1.83 m (= 38.4 m), 3.97 mm 301 steel, ≈ 1600 kg each | 36 m cylinder, "20 rings" | **38.43 m**, 21 rings, ring seams rendered |
| Nosecone | 52.1 − 38.4 ≈ 13.7 m, two segments | 16.1 m | **13.67 m** (blunted: R_n 2.5 m PICA-X cap) |
| Engines | 3 Raptor 3 SL (gimballed) + 3 RVac, each in a shielded compartment; SL 250 tf ≈ 2.45 MN | 3 + 3, 2.45 MN | same; bells Ø 1.30 m / Ø 2.30 m, SL 3.1 m and RVac 4.3 m long |
| Header tanks | LOX header forms the nose tip, CH₄ header below it | headers at x ≈ 44–47 m | unchanged (concept: coil + cryostat take the tip) |
| Chopstick lift points | inside the nosecone (reinforced) | pins at x = 35 m (payload bay) | **pins at x = 39.6 m**, nosecone base |
| Cold-gas RCS | 4 below the payload bay + 2 on the LOX tank (ullage gas) | 12 thrusters at x = 44.6 / 5.5 m | unchanged: concept needs ≈ 15× capacity and maximum lever arm (plan/05 Q9) |
| Flaps | 2 forward + 2 aft | none | none (concept) |
| Tiles | hexagonal tile heat shield | bare steel + PICA-X cap | same (concept) |
| PEZ door | in the payload bay | not shown | rendered as a door outline |

## 3. Consequences

The longer barrel and shorter nose move the payload bay, forward dome, nose shell and pins (see
`makeVehicle`). Aero, heating and MHD tables are rebuilt from the shape at start-up, so no table
needs a manual change. The guidance retunes itself (predictor tables come from the same model).
