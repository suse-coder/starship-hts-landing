/**
 * spec/05 T-atm — U.S. Standard Atmosphere 1976 (NOAA/NASA/USAF, NOAA-S/T 76-1562), Table I (geometric
 * altitude, SI). Reference values (ρ kg/m³, p Pa, T K):
 *    0 km  1.2250        101325     288.150   a = 340.294 m/s, μ = 1.7894e-5 Pa s, n = 2.5470e25 m⁻³
 *   11 km  3.6480e-1     2.2700e4   216.774
 *   20 km  8.8910e-2     5.5293e3   216.650
 *   32 km  1.3555e-2     8.8906e2   228.490
 *   50 km  1.0269e-3     7.9779e1   270.650
 *   86 km  6.958e-6      3.7338e-1  186.87
 *  100 km  5.604e-7, 200 km 2.541e-10, 400 km 2.803e-12 (Table I, upper atmosphere)
 * Acceptance (spec/05): within 0.5 % of the standard.
 */
import { describe, expect, test } from "bun:test"
import { atmosphere } from "../src/atmosphere"

const rel = (a: number, b: number): number => Math.abs(a / b - 1)

const US76: readonly (readonly [number, number, number, number])[] = [
  [0, 1.225, 101325, 288.15],
  [11000, 3.648e-1, 2.27e4, 216.774],
  [20000, 8.891e-2, 5.5293e3, 216.65],
  [32000, 1.3555e-2, 8.8906e2, 228.49],
  [50000, 1.0269e-3, 7.9779e1, 270.65],
  [86000, 6.958e-6, 3.7338e-1, 186.87],
]

describe("US Standard Atmosphere 1976", () => {
  for (const [z, rho, p, T] of US76) {
    test(`density, pressure, temperature at ${z / 1000} km within 0.5 %`, () => {
      const a = atmosphere(z)
      expect(rel(a.rho, rho)).toBeLessThan(0.005)
      expect(rel(a.p, p)).toBeLessThan(0.005)
      expect(rel(a.T, T)).toBeLessThan(0.005)
    })
  }

  test("sea-level speed of sound, Sutherland viscosity and number density (US-76 Table I)", () => {
    const a = atmosphere(0)
    expect(rel(a.a, 340.294)).toBeLessThan(1e-4)
    expect(rel(a.mu, 1.7894e-5)).toBeLessThan(1e-3)
    expect(rel(a.n, 2.547e25)).toBeLessThan(1e-3)
  })

  test("upper-atmosphere table nodes (100, 200, 400 km)", () => {
    expect(rel(atmosphere(100e3).rho, 5.604e-7)).toBeLessThan(0.005)
    expect(rel(atmosphere(200e3).rho, 2.541e-10)).toBeLessThan(0.005)
    expect(rel(atmosphere(400e3).rho, 2.803e-12)).toBeLessThan(0.005)
  })

  test("continuous across the 86 km seam (layer solution → tabulated thermosphere)", () => {
    const lo = atmosphere(86000 - 1e-3), hi = atmosphere(86000 + 1e-3)
    expect(rel(lo.rho, hi.rho)).toBeLessThan(1e-3)
    expect(rel(lo.p, hi.p)).toBeLessThan(1e-3)
  })

  test("density decreases monotonically from 0 to 1000 km", () => {
    let prev = Infinity
    for (let z = 0; z <= 1000e3; z += 500) {
      const r = atmosphere(z).rho
      expect(r).toBeLessThan(prev)
      prev = r
    }
  })

  test("ideal-gas law p = ρ R* T / M and hydrostatic balance dp/dz = −ρ g(z), g = g0 (r0/(r0+z))²", () => {
    const R = 8.31432, g0 = 9.80665, r0 = 6356766
    for (const z of [1000, 15000, 30000, 45000, 60000, 80000]) {
      const a = atmosphere(z)
      expect(rel(a.p, (a.rho * R * a.T) / a.M)).toBeLessThan(1e-12)
      const h = 5
      const dpdz = (atmosphere(z + h).p - atmosphere(z - h).p) / (2 * h)
      const g = g0 * (r0 / (r0 + z)) ** 2
      expect(rel(-dpdz, a.rho * g)).toBeLessThan(1e-3)
    }
  })

  test("sea-level pressure scale height RT/(Mg) ≈ 8.43 km", () => {
    const a = atmosphere(0)
    const H = (8.31432 * a.T) / (a.M * 9.80665)
    expect(H).toBeGreaterThan(8400)
    expect(H).toBeLessThan(8460)
  })
})
