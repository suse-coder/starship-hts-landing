/**
 * The reference path both the feedforward and the PD controller track, and every run is scored
 * against for cross-track RMS.
 *
 * Vertical: the exact constant-deceleration profile that starts at (z0, vz0) and reaches the gate
 * at vz_f. This is the same timed-descent family the flight's hoverslam uses, so it is flyable
 * inside the two-engine net-acceleration band.
 *
 * Horizontal: a quintic in the same final time with zero initial/final lateral acceleration,
 * matching the start and gate velocities. The quintic is C², so the feedforward acceleration is
 * continuous and the PD controller has a well-defined acceleration reference.
 *
 * A "least-time" path would be different; this is a smooth, feasible *reference*, not a
 * minimum-time plan. The PDG plan path is a separate strategy, not this shared reference.
 */
import { type V3, v3 } from "@ship/math"

export interface RefSample {
  readonly t: number
  readonly p: V3
  readonly v: V3
  /** total kinematic acceleration (m/s²), gravity included in the vertical component */
  readonly a: V3
}

/** C² quintic on a normalised time τ = t/T. */
interface Quintic {
  readonly c: readonly [number, number, number, number, number, number]
  readonly T: number
}

const quintic = (
  p0: number,
  v0: number,
  a0: number,
  pf: number,
  vf: number,
  af: number,
  T: number,
): Quintic => {
  const c0 = p0
  const c1 = v0 * T
  const c2 = 0.5 * a0 * T * T
  const d0 = pf - c0 - c1 - c2
  const d1 = vf * T - c1 - 2 * c2
  const d2 = af * T * T - 2 * c2
  return {
    c: [c0, c1, c2, 10 * d0 - 4 * d1 + 0.5 * d2, -15 * d0 + 7 * d1 - d2, 6 * d0 - 3 * d1 + 0.5 * d2],
    T,
  }
}

const qPos = (q: Quintic, t: number): number => {
  const u = Math.max(0, Math.min(1, t / q.T))
  const [c0, c1, c2, c3, c4, c5] = q.c
  return c0 + c1 * u + c2 * u * u + c3 * u ** 3 + c4 * u ** 4 + c5 * u ** 5
}

const qVel = (q: Quintic, t: number): number => {
  const u = Math.max(0, Math.min(1, t / q.T))
  const [, c1, c2, c3, c4, c5] = q.c
  return (c1 + 2 * c2 * u + 3 * c3 * u * u + 4 * c4 * u ** 3 + 5 * c5 * u ** 4) / q.T
}

const qAcc = (q: Quintic, t: number): number => {
  const u = Math.max(0, Math.min(1, t / q.T))
  const [, , c2, c3, c4, c5] = q.c
  return (2 * c2 + 6 * c3 * u + 12 * c4 * u * u + 20 * c5 * u ** 3) / (q.T * q.T)
}

export interface ReferencePath {
  readonly start: V3
  readonly end: V3
  readonly endV: V3
  /** total duration of the vertical profile (s) */
  readonly duration: number
  /** dense samples for cross-track distance (uniform in time) */
  readonly samples: readonly RefSample[]
  readonly at: (t: number) => RefSample
  /** distance from a point to the path polyline (m) */
  readonly distance: (p: V3) => number
}

const pointSegDist = (p: V3, a: V3, b: V3): number => {
  const abx = b.x - a.x
  const aby = b.y - a.y
  const abz = b.z - a.z
  const den = abx * abx + aby * aby + abz * abz
  const f = den > 1e-12 ? Math.max(0, Math.min(1, ((p.x - a.x) * abx + (p.y - a.y) * aby + (p.z - a.z) * abz) / den)) : 0
  const dx = p.x - (a.x + abx * f)
  const dy = p.y - (a.y + aby * f)
  const dz = p.z - (a.z + abz * f)
  return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

/**
 * Build the shared reference from a start state to the terminal gate. The vertical final time is
 * the altitude-matched constant-deceleration time; lateral and vertical share it.
 */
export const makeReferencePath = (p0: V3, v0: V3, gate: V3, gateV: V3): ReferencePath => {
  const z0 = Math.max(p0.z, 1)
  const vz0 = v0.z
  const vzf = gateV.z
  const denom = vz0 + vzf
  // z(T) = z0 + vz0·T + ½·a·T² with a = (vzf − vz0)/T reduces to z0 + ½(vz0 + vzf)T = 0.
  const duration = denom < -1e-6 ? Math.max(1, (-2 * z0) / denom) : Math.max(6, Math.min(60, (2 * z0) / Math.max(Math.abs(vz0) + Math.abs(vzf), 1)))
  const az = (vzf - vz0) / duration
  const qx = quintic(p0.x, v0.x, 0, gate.x, gateV.x, 0, duration)
  const qy = quintic(p0.y, v0.y, 0, gate.y, gateV.y, 0, duration)
  const at = (t: number): RefSample => {
    const tt = Math.max(0, Math.min(duration, t))
    const z = z0 + vz0 * tt + 0.5 * az * tt * tt
    return {
      t: tt,
      p: v3(qPos(qx, tt), qPos(qy, tt), Math.max(0, z)),
      v: v3(qVel(qx, tt), qVel(qy, tt), vz0 + az * tt),
      // after the profile ends the reference holds the gate (zero acceleration) so a late
      // controller is not chased by an ever-growing acceleration
      a: t >= duration ? v3(0, 0, 0) : v3(qAcc(qx, tt), qAcc(qy, tt), az),
    }
  }
  const n = 1201
  const samples: RefSample[] = []
  for (let k = 0; k < n; k++) samples.push(at((k / (n - 1)) * duration))
  const distance = (p: V3): number => {
    let best = Infinity
    for (let k = 0; k + 1 < samples.length; k++) {
      const d = pointSegDist(p, samples[k].p, samples[k + 1].p)
      if (d < best) best = d
    }
    return best
  }
  return { start: p0, end: gate, endV: gateV, duration, samples, at, distance }
}

/** Convenience for the feedforward term: the thrust acceleration that produces a kinematic
 *  acceleration `a` against gravity `g` along +z. */
export const thrustForAcceleration = (a: V3, g: number): V3 => v3(a.x, a.y, a.z + g)
