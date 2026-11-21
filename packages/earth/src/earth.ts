import { type V3, v3, scale, cross, sub, add, dot, norm } from "@ship/math"
import { MU_EARTH, R_EQ, J2, OMEGA_EARTH, E2_WGS } from "./constants"

/**
 * Frames: ECI is an Earth-centred quasi-inertial frame with z along the spin axis and x
 * toward the equinox of date. ECEF rotates about z by theta(t) = theta0 + ω t.
 */
export const OMEGA_VEC: V3 = v3(0, 0, OMEGA_EARTH)

export const rotZ = (v: V3, a: number): V3 => {
  const c = Math.cos(a)
  const s = Math.sin(a)
  return v3(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
}
export const eciToEcef = (r: V3, theta: number): V3 => rotZ(r, -theta)
export const ecefToEci = (r: V3, theta: number): V3 => rotZ(r, theta)

/** Two-body + J2 gravitational acceleration in ECI (z = pole). */
export const gravityJ2 = (r: V3): V3 => {
  const r2 = dot(r, r)
  const rn = Math.sqrt(r2)
  const k = -MU_EARTH / (r2 * rn)
  const zr2 = (r.z * r.z) / r2
  const j = 1.5 * J2 * (R_EQ * R_EQ) / r2
  const fxy = 1 + j * (1 - 5 * zr2)
  const fz = 1 + j * (3 - 5 * zr2)
  return v3(k * r.x * fxy, k * r.y * fxy, k * r.z * fz)
}

/** Potential energy per unit mass including J2 (for energy-conservation tests). */
export const potentialJ2 = (r: V3): number => {
  const rn = norm(r)
  const s = r.z / rn
  return -(MU_EARTH / rn) * (1 - J2 * (R_EQ / rn) ** 2 * 0.5 * (3 * s * s - 1))
}

export interface Geodetic {
  readonly lat: number
  readonly lon: number
  readonly alt: number
}

/** ECEF → geodetic (WGS84), Bowring with two refinements: sub-mm at all altitudes used here. */
export const ecefToGeodetic = (p: V3): Geodetic => {
  const a = R_EQ
  const e2 = E2_WGS
  const b = a * Math.sqrt(1 - e2)
  const ep2 = (a * a - b * b) / (b * b)
  const rxy = Math.hypot(p.x, p.y)
  const lon = Math.atan2(p.y, p.x)
  let beta = Math.atan2(p.z * a, rxy * b)
  let lat = 0
  for (let i = 0; i < 3; i++) {
    lat = Math.atan2(p.z + ep2 * b * Math.sin(beta) ** 3, rxy - e2 * a * Math.cos(beta) ** 3)
    beta = Math.atan2((1 - 0.0033528106647474805) * Math.sin(lat), Math.cos(lat))
  }
  const sl = Math.sin(lat)
  const N = a / Math.sqrt(1 - e2 * sl * sl)
  const alt = Math.abs(Math.cos(lat)) > 1e-6 ? rxy / Math.cos(lat) - N : Math.abs(p.z) - b
  return { lat, lon, alt }
}

export const geodeticToEcef = (g: Geodetic): V3 => {
  const sl = Math.sin(g.lat)
  const cl = Math.cos(g.lat)
  const N = R_EQ / Math.sqrt(1 - E2_WGS * sl * sl)
  return v3((N + g.alt) * cl * Math.cos(g.lon), (N + g.alt) * cl * Math.sin(g.lon), (N * (1 - E2_WGS) + g.alt) * sl)
}

export interface Enu {
  readonly e: V3
  readonly n: V3
  readonly u: V3
}
/** local East-North-Up basis expressed in ECEF */
export const enuBasis = (lat: number, lon: number): Enu => {
  const sl = Math.sin(lat), cl = Math.cos(lat), so = Math.sin(lon), co = Math.cos(lon)
  return { e: v3(-so, co, 0), n: v3(-sl * co, -sl * so, cl), u: v3(cl * co, cl * so, sl) }
}
export const enuToEcef = (b: Enu, v: V3): V3 => add(add(scale(b.e, v.x), scale(b.n, v.y)), scale(b.u, v.z))
export const ecefToEnu = (b: Enu, v: V3): V3 => v3(dot(v, b.e), dot(v, b.n), dot(v, b.u))

/** velocity of the co-rotating atmosphere at r (ECI) */
export const corotation = (r: V3): V3 => cross(OMEGA_VEC, r)
/** ECI velocity relative to the rotating Earth (still in ECI axes) */
export const relVelocity = (r: V3, v: V3): V3 => sub(v, corotation(r))

/** great-circle distance on a sphere of mean radius, m */
export const greatCircle = (lat1: number, lon1: number, lat2: number, lon2: number): number => {
  const s = Math.sin((lat2 - lat1) / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin((lon2 - lon1) / 2) ** 2
  return 2 * 6371008.8 * Math.asin(Math.min(1, Math.sqrt(s)))
}

/** Starbase Orbital Launch Pad 2 (approximate public coordinates; assumption A-PAD in plan/05). */
export const PAD2: Geodetic = { lat: (25.9925 * Math.PI) / 180, lon: (-97.1573 * Math.PI) / 180, alt: 5 }
