import { interp1, clamp } from "@ship/math"
import { gauss } from "@ship/math"
import { type V3, v3 } from "@ship/math"

/**
 * Mean wind (East, North) in m/s vs geometric altitude: a mid-November South-Texas / Gulf
 * climatology shape — onshore south-easterly at the surface, subtropical jet near 11–12 km,
 * weak stratospheric flow, mesospheric westerlies. (Design assumption A-WIND, plan/05.)
 */
const WZ = [0, 100, 500, 1000, 2000, 4000, 6000, 9000, 11500, 14000, 18000, 25000, 35000, 50000, 65000, 80000, 100000]
const WE = [-4.2, -5.0, -5.5, -3.0, 1.0, 6.0, 12.0, 24.0, 36.0, 28.0, 10.0, -4.0, -8.0, 15.0, 35.0, 20.0, 0]
const WN = [4.0, 4.6, 5.0, 4.0, 2.0, 1.0, 2.0, 4.0, 5.0, 3.0, 1.0, 0.0, 2.0, 4.0, 6.0, 5.0, 0]

export interface WindParams {
  readonly scaleMean: number // multiplies the mean profile
  readonly turbulence: number // 0 = off, 1 = MIL-F-8785C "light", 2 = "moderate"
  readonly seed: number
}

export const meanWindEnu = (h: number, p: WindParams): V3 =>
  v3(interp1(WZ, WE, h) * p.scaleMean, interp1(WZ, WN, h) * p.scaleMean, 0)

/** Dryden turbulence state: gust velocity in the local (East, North, Up) frame. */
export interface TurbState {
  readonly u: number
  readonly v: number
  readonly w: number
  readonly k: number // noise counter
}
export const turbInit: TurbState = { u: 0, v: 0, w: 0, k: 0 }

/** MIL-F-8785C Dryden scale lengths and intensities (SI). */
const drydenParams = (h: number, W20: number, level: number): { L: number; sig: number; Lw: number; sigw: number } => {
  const hft = clamp(h / 0.3048, 10, 1e6)
  if (hft < 1000) {
    const sw = 0.1 * W20
    const su = sw / (0.177 + 0.000823 * hft) ** 0.4
    const Lu = (hft / (0.177 + 0.000823 * hft) ** 1.2) * 0.3048
    return { L: Lu, sig: su, Lw: hft * 0.3048, sigw: sw }
  }
  // medium/high altitude: probability-of-exceedance intensities, fade out above 20 km
  const base = level >= 2 ? 3.0 : 1.5
  const fade = clamp(1 - (h - 15000) / 15000, 0, 1)
  const s = base * fade
  return { L: 533, sig: s, Lw: 533, sigw: s }
}

/** one first-order Gauss-Markov step per axis (exact discretisation of the Dryden u-filter). */
export const turbStep = (s: TurbState, dt: number, V: number, h: number, p: WindParams): TurbState => {
  if (p.turbulence <= 0 || h > 30000) return { u: 0, v: 0, w: 0, k: s.k + 1 }
  const W20 = (p.turbulence >= 2 ? 15 : 7.7) // wind speed at 20 ft, m/s
  const d = drydenParams(h, W20, p.turbulence)
  const Vs = Math.max(V, 5)
  const a = Math.exp((-Vs * dt) / d.L)
  const aw = Math.exp((-Vs * dt) / d.Lw)
  const b = Math.sqrt(1 - a * a)
  const bw = Math.sqrt(1 - aw * aw)
  return {
    u: a * s.u + d.sig * b * gauss(p.seed, s.k, 1),
    v: a * s.v + d.sig * b * gauss(p.seed, s.k, 2),
    w: aw * s.w + d.sigw * bw * gauss(p.seed, s.k, 3),
    k: s.k + 1,
  }
}

export const totalWindEnu = (h: number, t: TurbState, p: WindParams): V3 => {
  const m = meanWindEnu(h, p)
  return v3(m.x + t.u, m.y + t.v, m.z + t.w)
}
