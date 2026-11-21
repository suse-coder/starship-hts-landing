/**
 * spec/05 T-orbit and the planet model.
 * References:
 *  - WGS 84 (NIMA TR8350.2, 3rd ed.): GM = 3.986004418e14 m³/s², a = 6378137 m, normal gravity
 *    γ_e = 9.7803253359 m/s² (equator), γ_p = 9.8321849378 m/s² (pole), ω = 7.292115e-5 rad/s.
 *  - Vis-viva / circular speed v = √(μ/a), Kepler period T = 2π√(a³/μ).
 *  - J2 secular nodal regression per revolution ΔΩ = −3π J2 (R/p)² cos i (Vallado, Fundamentals of
 *    Astrodynamics, 4th ed., eq. 9-41 integrated over one period).
 *  - spec/05 T-orbit: two-body + J2 RK4 over one orbit, relative energy drift < 1e-8.
 */
import { describe, expect, test } from "bun:test"
import { type V3, v3, add, scale, norm, cross, dot, sub } from "@ship/math"
import {
  gravityJ2, potentialJ2, MU_EARTH, R_EQ, R_POL, J2, OMEGA_EARTH, ecefToGeodetic, geodeticToEcef, PAD2,
  corotation, greatCircle, enuBasis, enuToEcef, ecefToEnu, rotZ, eciToEcef, ecefToEci,
} from "../src/index"

const rel = (a: number, b: number): number => Math.abs(a / b - 1)

/** circular orbit state (radius a, inclination i, RAAN Ω, argument of latitude u) */
const circular = (a: number, inc: number, raan: number, u: number): { r: V3; v: V3 } => {
  const cO = Math.cos(raan), sO = Math.sin(raan), ci = Math.cos(inc), si = Math.sin(inc), cu = Math.cos(u), su = Math.sin(u)
  const vc = Math.sqrt(MU_EARTH / a)
  return {
    r: v3(a * (cO * cu - sO * su * ci), a * (sO * cu + cO * su * ci), a * su * si),
    v: v3(vc * (-cO * su - sO * cu * ci), vc * (-sO * su + cO * cu * ci), vc * cu * si),
  }
}

const rk4 = (r: V3, v: V3, h: number, acc: (r: V3) => V3): { r: V3; v: V3 } => {
  const k1r = v, k1v = acc(r)
  const k2r = add(v, scale(k1v, h / 2)), k2v = acc(add(r, scale(k1r, h / 2)))
  const k3r = add(v, scale(k2v, h / 2)), k3v = acc(add(r, scale(k2r, h / 2)))
  const k4r = add(v, scale(k3v, h)), k4v = acc(add(r, scale(k3r, h)))
  return {
    r: add(r, scale(add(add(k1r, scale(add(k2r, k3r), 2)), k4r), h / 6)),
    v: add(v, scale(add(add(k1v, scale(add(k2v, k3v), 2)), k4v), h / 6)),
  }
}

const energy = (r: V3, v: V3): number => 0.5 * dot(v, v) + potentialJ2(r)

describe("gravity (two-body + J2)", () => {
  test("surface gravity minus centrifugal = WGS-84 normal gravity at the equator and pole (< 2e-5)", () => {
    const gEq = norm(gravityJ2(v3(R_EQ, 0, 0))) - OMEGA_EARTH ** 2 * R_EQ
    const gPole = norm(gravityJ2(v3(0, 0, R_POL)))
    // J2-only field: the residual (≈ 1e-5) is J4 and higher zonal terms
    expect(rel(gEq, 9.7803253359)).toBeLessThan(2e-5)
    expect(rel(gPole, 9.8321849378)).toBeLessThan(2e-5)
  })

  test("g = −∇U (acceleration is the gradient of potentialJ2)", () => {
    for (const r of [v3(5e6, 3e6, 4e6), v3(-6.5e6, 1e5, -2e6), v3(1e6, -2e6, 6.3e6)]) {
      const h = 1
      const U = (x: V3): number => potentialJ2(x)
      const g = gravityJ2(r)
      const gx = -(U(v3(r.x + h, r.y, r.z)) - U(v3(r.x - h, r.y, r.z))) / (2 * h)
      const gy = -(U(v3(r.x, r.y + h, r.z)) - U(v3(r.x, r.y - h, r.z))) / (2 * h)
      const gz = -(U(v3(r.x, r.y, r.z + h)) - U(v3(r.x, r.y, r.z - h))) / (2 * h)
      expect(norm(sub(g, v3(gx, gy, gz))) / norm(g)).toBeLessThan(1e-7)
    }
  })

  test("equatorial symmetry: g_z(x, y, −z) = −g_z(x, y, z)", () => {
    const a = gravityJ2(v3(4e6, 2e6, 3e6)), b = gravityJ2(v3(4e6, 2e6, -3e6))
    expect(b.z).toBeCloseTo(-a.z, 12)
    expect(b.x).toBeCloseTo(a.x, 12)
  })
})

describe("orbits", () => {
  const a0 = R_EQ + 275e3 // Flight 15 parking orbit ≈ 275 km
  test("circular state satisfies vis-viva v = √(μ/a), r ⟂ v", () => {
    const s = circular(a0, (26.5 * Math.PI) / 180, 0.3, 1.1)
    expect(rel(norm(s.r), a0)).toBeLessThan(1e-14)
    expect(rel(norm(s.v), Math.sqrt(MU_EARTH / a0))).toBeLessThan(1e-14)
    expect(Math.abs(dot(s.r, s.v))).toBeLessThan(1e-6)
    expect(Math.sqrt(MU_EARTH / a0)).toBeCloseTo(7740.2, 0) // circular speed at 275 km
  })

  test("T-orbit: RK4 two-body + J2 over one orbit — relative energy drift < 1e-8, h_z conserved", () => {
    const s = circular(a0, (26.5 * Math.PI) / 180, 0.3, 0.1)
    const T = 2 * Math.PI * Math.sqrt(a0 ** 3 / MU_EARTH)
    const h = 10
    let r = s.r, v = s.v
    const E0 = energy(r, v)
    const hz0 = cross(r, v).z
    let maxDrift = 0, maxHz = 0
    for (let k = 0; k < Math.round(T / h); k++) {
      ;({ r, v } = rk4(r, v, h, gravityJ2))
      maxDrift = Math.max(maxDrift, Math.abs(energy(r, v) / E0 - 1))
      maxHz = Math.max(maxHz, Math.abs(cross(r, v).z / hz0 - 1)) // J2 field is axisymmetric about z
    }
    expect(maxDrift).toBeLessThan(1e-8)
    expect(maxHz).toBeLessThan(1e-9)
  })

  test("Kepler: with J2 removed the orbit closes after T = 2π√(a³/μ) (to < 2 m)", () => {
    const kepler = (r: V3): V3 => scale(r, -MU_EARTH / norm(r) ** 3)
    const s = circular(a0, 0.9, 0.2, 0.4)
    const T = 2 * Math.PI * Math.sqrt(a0 ** 3 / MU_EARTH)
    const n = 2000
    let r = s.r, v = s.v
    for (let k = 0; k < n; k++) ({ r, v } = rk4(r, v, T / n, kepler))
    expect(norm(sub(r, s.r))).toBeLessThan(2)
  })

  test("J2 nodal regression per revolution = −3π J2 (R/p)² cos i (within 2 %)", () => {
    const inc = (26.5 * Math.PI) / 180
    const s = circular(a0, inc, 0.5, 0) // start at the ascending node
    const raan = (r: V3, v: V3): number => {
      const hv = cross(r, v)
      return Math.atan2(hv.x, -hv.y)
    }
    const O0 = raan(s.r, s.v)
    let r = s.r, v = s.v, t = 0
    const h = 1
    let O1 = NaN
    for (let k = 0; k < 8000; k++) {
      const n = rk4(r, v, h, gravityJ2)
      t += h
      if (t > 3000 && r.z < 0 && n.r.z >= 0) {
        // osculating Ω at the next ascending node (short-period terms repeat at equal argument of latitude)
        const f = -r.z / (n.r.z - r.z)
        const rc = add(r, scale(sub(n.r, r), f)), vc = add(v, scale(sub(n.v, v), f))
        O1 = raan(rc, vc)
        break
      }
      r = n.r
      v = n.v
    }
    const expected = -3 * Math.PI * J2 * (R_EQ / a0) ** 2 * Math.cos(inc) // ≈ −8.4e-3 rad (−0.48°)
    expect(Math.abs((O1 - O0) / expected - 1)).toBeLessThan(0.02)
  })
})

describe("frames and geodesy", () => {
  test("geodetic ↔ ECEF round trip (WGS-84) to < 1 mm at Pad 2 and at 120 km", () => {
    for (const g of [PAD2, { lat: 0.45, lon: -1.7, alt: 120e3 }, { lat: -1.2, lon: 2.9, alt: 3e5 }]) {
      const b = ecefToGeodetic(geodeticToEcef(g))
      expect(Math.abs(b.alt - g.alt)).toBeLessThan(1e-3)
      expect(Math.abs(b.lat - g.lat) * R_EQ).toBeLessThan(1e-3)
      expect(Math.abs(b.lon - g.lon) * R_EQ).toBeLessThan(1e-3)
    }
    expect(norm(geodeticToEcef({ lat: Math.PI / 2, lon: 0, alt: 0 }))).toBeCloseTo(R_POL, 3)
  })

  test("ENU basis is orthonormal and right-handed (e × n = u); ECI↔ECEF are inverse rotations", () => {
    const b = enuBasis(PAD2.lat, PAD2.lon)
    expect(norm(sub(cross(b.e, b.n), b.u))).toBeLessThan(1e-15)
    const x = v3(3, -4, 12)
    expect(norm(sub(ecefToEnu(b, enuToEcef(b, x)), x))).toBeLessThan(1e-12)
    expect(norm(sub(ecefToEci(eciToEcef(x, 1.234), 1.234), x))).toBeLessThan(1e-12)
    expect(norm(sub(rotZ(v3(1, 0, 0), Math.PI / 2), v3(0, 1, 0)))).toBeLessThan(1e-15)
  })

  test("co-rotation speed at the equator ω R = 465.1 m/s; quarter meridian on the mean sphere = 10 007.5 km", () => {
    expect(norm(corotation(v3(R_EQ, 0, 0)))).toBeCloseTo(465.1, 1)
    expect(greatCircle(0, 0, Math.PI / 2, 0) / 1000).toBeCloseTo(10007.5, 0)
  })
})
