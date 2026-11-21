/**
 * Thermal survival limits (spec/04 A4.8, spec/05 T-mission). One source of truth for the verdicts in
 * the results panel, the hull false-colour map and the tests.
 *
 * Steel strength: 0.2 %-proof-strength retention k(T) = f_y(T)/f_y(20 °C) for austenitic stainless
 * grade 1.4301 (304), EN 1993-1-2 Annex C, Table C.1 (values as recalled, research/02, to be
 * verified against the standard: open question Q-T3). Cold-worked 301/30X loses its cold-work
 * premium above ≈ 800 K, so the 1.4301 annealed curve is a fair (slightly optimistic at low T,
 * representative at high T) proxy for the hot condition.
 *
 * Bands used in the views: ≤ 700 K nominal (k ≥ 0.58), ≤ 900 K reduced (k ≥ 0.47),
 * ≤ 1150 K design limit (k ≥ 0.17, pressure-stabilised tanks, low descent loads), < 1400 K strength
 * loss (k → 0.02), ≥ 1400 K failure. Solidus ≈ 1670 K.
 * PICA-X bondline (RTV-560-class adhesive to steel): 560 K continuous limit.
 */
export const THERMAL_LIMITS = {
  steelFull: 700,
  steelReduced: 900,
  steelDesign: 1150,
  steelFail: 1400,
  bondline: 560,
  tileSurface: 1530, // LI-900/HRSI maximum use temperature (Q23); sourced SpaceX claim 1673 K is in TILE_MATERIAL_OPTIONS.starshipHex
  tileBond: 600, // RTV-class tile adhesive, assumed (Q23; conservative alternative 560)
} as const

/** EN 1993-1-2 Table C.1, grade 1.4301: [T °C, k_0.2p] */
const K02P: readonly (readonly [number, number])[] = [
  [20, 1.0], [100, 0.82], [200, 0.68], [300, 0.64], [400, 0.6], [500, 0.54], [600, 0.49],
  [700, 0.4], [800, 0.27], [900, 0.14], [1000, 0.06], [1100, 0.03], [1200, 0.0],
]

/** yield-strength retention of the hull steel at temperature T (K), 0..1 */
export const steelYieldRetention = (T: number): number => {
  const c = T - 273.15
  if (c <= K02P[0][0]) return 1
  const i = K02P.findIndex(([ci]) => c <= ci)
  if (i < 0) return 0
  const [c0, k0] = K02P[i - 1], [c1, k1] = K02P[i]
  return k0 + ((k1 - k0) * (c - c0)) / (c1 - c0)
}

export type SteelVerdict = "nominal" | "reduced strength" | "within design" | "strength loss" | "failure"

export const steelVerdict = (T: number): SteelVerdict =>
  T <= THERMAL_LIMITS.steelFull ? "nominal"
    : T <= THERMAL_LIMITS.steelReduced ? "reduced strength"
      : T <= THERMAL_LIMITS.steelDesign ? "within design"
        : T < THERMAL_LIMITS.steelFail ? "strength loss"
          : "failure"
