import { interp1 } from "@ship/math"

/**
 * U.S. Standard Atmosphere 1976.
 * 0–86 km: the seven geopotential layers (exact hydrostatic solution).
 * 86–1000 km: tabulated density/temperature/molar mass from the 1976 tables, log-interpolated.
 */
export interface AtmState {
  readonly T: number // K
  readonly p: number // Pa
  readonly rho: number // kg/m^3
  readonly a: number // speed of sound m/s
  readonly mu: number // dynamic viscosity Pa s
  readonly M: number // molar mass kg/mol
  readonly n: number // number density 1/m^3
}

const R0 = 6356766 // m, US76 effective Earth radius for geopotential
const RSTAR = 8.31432 // US76 gas constant
const M0 = 0.0289644
const G0 = 9.80665
const HB = [0, 11000, 20000, 32000, 47000, 51000, 71000, 84852]
const LB = [-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002, 0]
const TB = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946]
const PB = [101325, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.95642, 0.3733836]

const UZ = [86, 90, 95, 100, 110, 120, 130, 140, 150, 160, 180, 200, 250, 300, 350, 400, 500, 600, 800, 1000].map((k) => k * 1000)
const UT = [186.87, 186.87, 188.42, 195.08, 240.0, 360.0, 469.27, 559.63, 634.39, 696.29, 790.07, 854.56, 941.33, 976.01, 990.06, 995.83, 999.24, 999.85, 999.99, 1000]
const ULR = [6.958e-6, 3.416e-6, 1.393e-6, 5.604e-7, 9.708e-8, 2.222e-8, 8.152e-9, 3.831e-9, 2.076e-9, 1.233e-9, 5.194e-10, 2.541e-10, 6.073e-11, 1.916e-11, 7.014e-12, 2.803e-12, 5.215e-13, 1.137e-13, 1.136e-14, 3.561e-15].map(Math.log)
const UM = [28.95, 28.91, 28.82, 28.4, 27.27, 26.2, 25.44, 24.75, 24.1, 23.49, 22.34, 21.3, 19.19, 17.73, 16.64, 15.98, 14.33, 11.51, 5.54, 3.94].map((m) => m / 1000)

const sutherland = (T: number): number => (1.458e-6 * T ** 1.5) / (T + 110.4)

export const atmosphere = (zGeom: number): AtmState => {
  const z = Math.max(zGeom, -500)
  if (z < 86000) {
    const H = (R0 * z) / (R0 + z)
    let i = 6
    while (i > 0 && H < HB[i]) i--
    const dH = H - HB[i]
    const L = LB[i]
    const T = TB[i] + L * dH
    const p = L === 0 ? PB[i] * Math.exp((-G0 * M0 * dH) / (RSTAR * TB[i])) : PB[i] * (TB[i] / T) ** ((G0 * M0) / (RSTAR * L))
    const rho = (p * M0) / (RSTAR * T)
    return { T, p, rho, a: Math.sqrt((1.4 * RSTAR * T) / M0), mu: sutherland(T), M: M0, n: (p / (1.380649e-23 * T)) }
  }
  const zc = Math.min(z, 1000000)
  const T = interp1(UZ, UT, zc)
  const rho = Math.exp(interp1(UZ, ULR, zc))
  const M = interp1(UZ, UM, zc)
  const p = (rho * RSTAR * T) / M
  // above 86 km the gas is still treated as calorically perfect with gamma 1.4 for a; it only feeds Kn/Mach diagnostics
  return { T, p, rho, a: Math.sqrt((1.4 * RSTAR * T) / M), mu: sutherland(T), M, n: p / (1.380649e-23 * T) }
}
