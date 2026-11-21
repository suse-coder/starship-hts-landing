import { type V3, v3 } from "@ship/math"

/** Low-precision solar direction in ECI (Astronomical Almanac, ~0.01° accuracy). */
export const sunDirectionEci = (jd: number): V3 => {
  const n = jd - 2451545.0
  const L = ((280.46 + 0.9856474 * n) * Math.PI) / 180
  const g = ((357.528 + 0.9856003 * n) * Math.PI) / 180
  const lam = L + ((1.915 * Math.sin(g) + 0.02 * Math.sin(2 * g)) * Math.PI) / 180
  const eps = ((23.439 - 0.0000004 * n) * Math.PI) / 180
  return v3(Math.cos(lam), Math.cos(eps) * Math.sin(lam), Math.sin(eps) * Math.sin(lam))
}
