/**
 * Time scales, Sun direction, winds/turbulence and the geomagnetic dipole.
 * References:
 *  - IAU 1982 GMST: GMST(J2000.0 = JD 2451545.0) = 280.46061837°; sidereal day 86164.0905 s.
 *  - USNO: March equinox 2025-03-20 09:01 UTC (apparent λ☉ = 0), June solstice 2025-06-21 02:42 UTC
 *    (λ☉ = 90°, δ = ε = 23.436°). The low-precision Almanac formula is good to ~0.01°.
 *  - Sun at Starbase at the Flight 15 catch time (2026-11-14 21:20:06 UTC): δ ≈ −18.4°, equation of
 *    time ≈ +15.5 min → hour angle ≈ 46.8°, elevation sin h = sin φ sin δ + cos φ cos δ cos H ≈ 26.5°.
 *  - MIL-F-8785C Dryden low-altitude model: σ_w = 0.1 W20, σ_u = σ_v = σ_w/(0.177 + 0.000823 h)^0.4,
 *    L_u = h/(0.177 + 0.000823 h)^1.2, L_w = h (h in ft); W20 = 15 kt (7.7 m/s) for "light".
 *  - IGRF-14 degree-1 (dipole) coefficients: B0 = √(g10² + g11² + h11²) ≈ 29.73 μT at r = R on the
 *    dipole equator, 2 B0 on the dipole axis; the field points downward in the northern hemisphere.
 */
import { describe, expect, test } from "bun:test"
import { type V3, v3, norm, dot, scale } from "@ship/math"
import {
  julianDate, gmst, sunDirectionEci, rotZ, enuBasis, ecefToEnu, PAD2, FLIGHT15, meanWindEnu, turbStep, turbInit,
  totalWindEnu, type WindParams, geomagneticEcef, R_EQ,
} from "../src/index"

const DEG = Math.PI / 180

describe("time and the Sun", () => {
  test("Julian date of J2000.0 and GMST(J2000.0) = 280.46061837°", () => {
    expect(julianDate("2000-01-01T12:00:00Z")).toBe(2451545.0)
    expect(gmst(2451545.0) / DEG).toBeCloseTo(280.46061837, 8)
  })

  test("GMST advances 360° in one sidereal day (86164.0905 s)", () => {
    const jd = julianDate(FLIGHT15.catchUtc)
    const d = gmst(jd + 86164.0905 / 86400) - gmst(jd)
    const wrapped = Math.atan2(Math.sin(d), Math.cos(d))
    expect(Math.abs(wrapped)).toBeLessThan(1e-5)
  })

  test("Sun at the 2025 March equinox points along +x (vernal equinox) within 0.05°", () => {
    const s = sunDirectionEci(julianDate("2025-03-20T09:01:00Z"))
    expect(Math.acos(Math.min(1, s.x)) / DEG).toBeLessThan(0.05)
  })

  test("Sun at the 2025 June solstice: declination 23.44°, right ascension 90° (±0.05°)", () => {
    const s = sunDirectionEci(julianDate("2025-06-21T02:42:00Z"))
    expect(Math.asin(s.z) / DEG).toBeCloseTo(23.436, 1)
    expect(Math.abs(Math.atan2(s.y, s.x) / DEG - 90)).toBeLessThan(0.05)
    expect(norm(s)).toBeCloseTo(1, 14)
  })

  test("Sun elevation at Starbase at the catch time ≈ 26.5° in the south-west (spherical astronomy)", () => {
    const jd = julianDate(FLIGHT15.catchUtc)
    const sEcef = rotZ(sunDirectionEci(jd), -gmst(jd))
    const enu = ecefToEnu(enuBasis(PAD2.lat, PAD2.lon), sEcef)
    // independent: δ = −18.37° (Almanac, 14 Nov 21h UT), EoT = +15.5 min
    const dec = -18.37 * DEG
    const H = ((21 + 20 / 60 + 6 / 3600 - 12 + 15.5 / 60) * 15 + PAD2.lon / DEG) * DEG
    const elRef = Math.asin(Math.sin(PAD2.lat) * Math.sin(dec) + Math.cos(PAD2.lat) * Math.cos(dec) * Math.cos(H))
    expect(Math.abs(Math.asin(enu.z) - elRef) / DEG).toBeLessThan(0.3)
    const az = (Math.atan2(enu.x, enu.y) / DEG + 360) % 360
    expect(az).toBeGreaterThan(200)
    expect(az).toBeLessThan(250)
  })
})

describe("wind and Dryden turbulence", () => {
  const P: WindParams = { scaleMean: 1, turbulence: 1, seed: 42 }

  test("mean wind: climatology nodes, linear in the scale factor, horizontal only", () => {
    const w = meanWindEnu(11500, P)
    expect(w.x).toBeCloseTo(36, 12) // subtropical jet core (A-WIND)
    expect(w.y).toBeCloseTo(5, 12)
    expect(w.z).toBe(0)
    const w2 = meanWindEnu(11500, { ...P, scaleMean: 2 })
    expect(w2.x).toBeCloseTo(72, 12)
  })

  test("turbulence off → zero gusts; no turbulence above 30 km", () => {
    let s = turbInit
    for (let k = 0; k < 50; k++) s = turbStep(s, 0.1, 100, 500, { ...P, turbulence: 0 })
    expect([s.u, s.v, s.w]).toEqual([0, 0, 0])
    const hi = turbStep({ u: 3, v: 2, w: 1, k: 0 }, 0.1, 100, 40000, P)
    expect([hi.u, hi.v, hi.w]).toEqual([0, 0, 0])
  })

  test("deterministic: same seed → same gust history; different seed → different", () => {
    const run = (seed: number): V3 => {
      let s = turbInit
      for (let k = 0; k < 500; k++) s = turbStep(s, 0.05, 80, 300, { ...P, seed })
      return v3(s.u, s.v, s.w)
    }
    expect(run(7)).toEqual(run(7))
    expect(run(7).x).not.toBe(run(8).x)
  })

  test("low-altitude Dryden statistics match MIL-F-8785C intensities (σ within 8 %)", () => {
    const h = 150 // m (492 ft)
    const hft = h / 0.3048
    const sw = 0.1 * 7.7
    const su = sw / (0.177 + 0.000823 * hft) ** 0.4
    let s = turbInit
    let su2 = 0, sv2 = 0, sw2 = 0, mu = 0
    const n = 120000
    for (let k = 0; k < 2000; k++) s = turbStep(s, 0.2, 60, h, P) // spin-up
    for (let k = 0; k < n; k++) {
      s = turbStep(s, 0.2, 60, h, P)
      su2 += s.u * s.u
      sv2 += s.v * s.v
      sw2 += s.w * s.w
      mu += s.u
    }
    expect(Math.abs(Math.sqrt(su2 / n) / su - 1)).toBeLessThan(0.08)
    expect(Math.abs(Math.sqrt(sv2 / n) / su - 1)).toBeLessThan(0.08)
    expect(Math.abs(Math.sqrt(sw2 / n) / sw - 1)).toBeLessThan(0.08)
    expect(Math.abs(mu / n)).toBeLessThan(0.15 * su)
    const tot = totalWindEnu(h, s, P)
    expect(tot.x).toBeCloseTo(meanWindEnu(h, P).x + s.u, 12)
  })
})

describe("geomagnetic dipole (IGRF degree 1)", () => {
  const G = v3(-1410e-9, 4545e-9, -29350e-9)
  const B0 = norm(G)
  const axis = scale(G, -1 / B0) // unit vector to the northern geomagnetic pole (the moment points the other way)
  test("|B| = B0 ≈ 29.7 μT on the dipole equator and 2 B0 on the axis at r = R", () => {
    expect(B0 * 1e6).toBeCloseTo(29.73, 1)
    const onAxis = geomagneticEcef(scale(axis, -R_EQ)) // northern geomagnetic pole (opposite the moment)
    expect(norm(onAxis) / B0).toBeCloseTo(2, 10)
    // a point on the dipole equator: any r ⟂ the axis
    const eq = v3(axis.y, -axis.x, 0)
    const e = geomagneticEcef(scale(eq, R_EQ / norm(eq)))
    expect(norm(e) / B0).toBeCloseTo(1, 10)
  })
  test("field points downward in the northern hemisphere and falls off as r⁻³", () => {
    const r = v3(0, 0, R_EQ)
    expect(dot(geomagneticEcef(r), r)).toBeLessThan(0)
    expect(norm(geomagneticEcef(scale(r, 2))) / norm(geomagneticEcef(r))).toBeCloseTo(1 / 8, 12)
  })
})
