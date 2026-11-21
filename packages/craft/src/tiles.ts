import { type Shape, wettedArea } from "./geometry"
import { type DesignParams, designPica } from "./config"

/**
 * Silica tile heat shield of the flapless Ship 42 derivative (user decision Q23, 2026-11-21).
 *
 * The real vehicle's windward and leeward barrel is covered with hexagonal silica tiles of the
 * LI-900 class; the steel skin behind them is only the backing structure, and the nose keeps
 * PICA-X. This module owns the tile *design* (thickness, areal mass, material data) so that the
 * mass budget and the wall model in `@ship/heat` use one source of truth.
 *
 * Material data (LI-900 reusable surface insulation, NASA/Shuttle TPS):
 *  - density 144 kg/m³ (9 lb/ft³, LI-900; NASA Space Shuttle TPS data sheet);
 *  - thermal conductivity rising from ≈ 0.045 W/m·K at 300 K (LI-900 ≈ 0.048) with a linear
 *    fit 0.045 + 1.2e-4 (T − 300) W/m·K (≈ 0.12 at 900 K, 0.19 at 1500 K);
 *  - specific heat 1000 J/kg·K at 300 K rising to 1300 J/kg·K (amorphous silica, 1000–1300);
 *  - reaction-cured borosilicate glass coating: ε = 0.8 (LI-900 coated 0.85; 0.8 is the value
 *    used here, slightly conservative for the surface temperature);
 *  - maximum use temperature 1530 K (1260 °C, LI-900/HRSI limit; the limit itself lives in
 *    `@ship/heat` THERMAL_LIMITS.tileSurface). The public SpaceX claim for the flight tile is
 *    higher (1400 °C = 1673 K); that reading is carried below as `starshipHex` and is **not**
 *    used by the wall model or the verdict, because it is a secondary-source public claim.
 *
 * Thickness: no public SpaceX figure for the flight tile thickness was found (Wikipedia gives
 * 18 000 tiles and 1400 °C but no thickness). Shuttle HRSI tiles varied 1–5 in (2.5–12.7 cm),
 * so the value below is within the real range; it is **assumed**, 3.0 cm, with its conservative
 * alternatives stated in the report: 2.0 cm for the thermal check (a thinner tile lets the
 * backing run hotter) and 5.0 cm for the mass check (a thicker tile is heavier).
 *
 * Adhesive/mount: a 0.2 mm RTV-class bond with 0.2 W/m·K (assumed; the pins and gap filler are
 * lumped into the areal mass). Its continuous limit is 600 K (assumed; the conservative
 * alternative is the 560 K RTV-560 limit already used for the PICA bondline).
 */
export const TILE = {
  density: 144, // kg/m³, LI-900 (sourced)
  thickness: 0.03, // m, assumed (public estimates 0.025–0.076 m)
  bondThickness: 0.0002, // m, RTV-class adhesive, assumed
  bondConductivity: 0.2, // W/m·K, assumed
  bondArealMass: 1.5, // kg/m², adhesive + pins + gap filler, assumed (conservative 2.5)
  bondArealHeatCapacity: 1500 * 0.0002 * 1200, // J/m²K, adhesive film (ρ 1500, c 1200)
  emissivity: 0.8, // coated LI-900 (sourced 0.85; 0.8 used)
  conductivityAt300: 0.045, // W/m·K (sourced ≈ 0.048)
  conductivitySlope: 1.2e-4, // W/m·K per K
  cpAt300: 1000, // J/kg·K (sourced 1000–1300)
  cpSlope: 0.2, // J/kg·K per K
  cpMax: 1300, // J/kg·K
} as const

/**
 * Surface-temperature readings of the same installed tile design (Q23).
 *
 * The wall model and every verdict use `li900`, the conservative reading: the flight material is
 * assumed to be LI-900-class silica, whose black HRSI form is rated to 1260 °C (1533 K)
 * (NASA Orbiter TPS fact sheet, 2006; Wikipedia `Space Shuttle thermal protection system`).
 *
 * `starshipHex` records the public SpaceX claim for the actual hexagonal silica tile: 1400 °C
 * (1673 K), repeated by Wikipedia `SpaceX Starship` citing Digital Trends (2019), plus Musk's
 * March 2019 torch test, which reached ~1650 K and held tile integrity. No public material
 * datasheet, composition or certified reuse limit exists for the flight tile, and the test was a
 * short torch exposure, so this is a **public claim, not a qualified material limit**. It is
 * therefore an option for reporting, never the default that the model judges against.
 */
export const TILE_MATERIAL_OPTIONS = {
  li900: {
    label: "LI-900 / Shuttle HRSI (conservative default)",
    surfaceLimitK: 1530,
    density: 144,
    source: "NASA Orbiter TPS fact sheet (2006): black HRSI rated to 1260 °C; LI-900 bulk density 144 kg/m³",
  },
  starshipHex: {
    label: "SpaceX hexagonal silica tile (public claim; not a certified reuse limit)",
    surfaceLimitK: 1673,
    demonstratedK: 1650,
    density: 144, // assumed equal to LI-900 for the mass budget; no public figure
    source: "Wikipedia `SpaceX Starship`: silica-based hex tiles rated to 1400 °C (1673 K), citing Digital Trends 2019; Musk 2019 torch test reached ~1650 K",
  },
} as const

/** tile-material reading used by the design and its verdicts (conservative) */
export type TileMaterial = keyof typeof TILE_MATERIAL_OPTIONS

/** areal density of the installed tile layer, kg/m² (tile + bond/pins/gap filler) */
export const tileArealDensity = (): number => TILE.density * TILE.thickness + TILE.bondArealMass

/** body station where the tiles stop and the PICA-X nose begins, m */
export const tileCoverageX0 = (shape: Shape, design: DesignParams): number => designPica(design, shape).x0

/**
 * Barrel area covered by tiles, m². The whole barrel up to the PICA-X boundary is tiled
 * (windward and leeward); the nose keeps PICA-X and the real ship's flaps — absent on this
 * flapless derivative — contribute no tile area. The aft skirt is inside the barrel OML, so it
 * is counted (conservative for mass).
 */
export const tiledArea = (shape: Shape, design: DesignParams): number => wettedArea(shape, 0, tileCoverageX0(shape, design))

/** area centroid of the tiled barrel, m — the CG station of the tile mass item */
export const tileCentroidX = (shape: Shape, design: DesignParams): number => {
  const x0 = tileCoverageX0(shape, design)
  const N = 200
  let area = 0, moment = 0
  for (let i = 0; i < N; i++) {
    const xa = (x0 * i) / N, xb = (x0 * (i + 1)) / N
    const dA = Math.PI * (shape.r(xa) + shape.r(xb)) * Math.hypot(xb - xa, shape.r(xb) - shape.r(xa))
    area += dA
    moment += dA * 0.5 * (xa + xb)
  }
  return area > 0 ? moment / area : 0.5 * x0
}

/** fraction of the vehicle's wetted area that carries tiles (nose excluded, no flaps) */
export const tileAreaFraction = (shape: Shape, design: DesignParams): number =>
  tiledArea(shape, design) / wettedArea(shape, 0, shape.p.length)

/** installed tile-layer mass, kg: areal density × tiled area */
export const tileLayerMass = (shape: Shape, design: DesignParams): number => tileArealDensity() * tiledArea(shape, design)
