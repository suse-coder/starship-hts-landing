import { type Q, type V3, v3, add, clamp, cross, dot, norm, qrot, qrotInv, scale, sub, ZERO, hadamard, normalize } from "@ship/math"
import type { Couple, Engine, Thruster } from "@ship/craft"

/**
 * Attitude control as pure functions.
 * Rate-command law with saturation (slew-rate limit) and model feedforward:
 *   ω_c = sat(k·e, ω_max),  τ = I·Kd·(ω_c − ω) + ω × Iω − M_ff
 * e: rotation-vector error in body axes (pointing of +x plus roll of +z, or roll *rate* for spin).
 */
export interface AttitudeTarget {
  readonly x: V3 // desired body +x in ECI
  readonly z?: V3 // desired body +z hint in ECI (full attitude)
  readonly spin?: number // desired roll rate (rad/s) instead of a roll angle
  readonly free?: V3 // ECI axis left to the aerodynamics (no torque about it), e.g. the α-pitch axis
  readonly bankAxis?: V3 // ECI unit v̂: measure the pointing error as a rotation about v̂ (bank), never through it
}

export interface Gains {
  readonly wn: number // rad/s closed-loop natural frequency
  readonly zeta: number
  readonly wMax: number // rad/s slew-rate limit
  readonly wnRoll?: number // roll-loop bandwidth (defaults to wn)
  readonly deadband?: number // rad, phase-plane attitude deadband (rate deadband = deadband/10 per s)
  readonly rollBand?: number // rad/s, no roll torque while the roll-rate error is inside this band
  readonly rollMax?: number // rad/s roll-rate command limit (default wMax)
  /** per-axis angular acceleration authority (rad/s², body x/y/z): each axis never commands more rate
   *  than it can stop within its error — the explicit (time-optimal) MPC law of a bounded double integrator */
  readonly axisAccel?: readonly [number, number, number]
  readonly accelLimit?: number // rad/s², actuator-limited angular acceleration: time-optimal braking profile
}

export const GAINS = {
  vacuum: { wn: 0.1, zeta: 1.0, wMax: (0.5 * Math.PI) / 180, deadband: (0.6 * Math.PI) / 180, accelLimit: 0.0035 },
  mhd: { wn: 1.4, zeta: 0.85, wMax: (4 * Math.PI) / 180, wnRoll: 0.25, rollBand: (1.0 * Math.PI) / 180 },
  pitchUp: { wn: 0.6, zeta: 0.9, wMax: (8 * Math.PI) / 180, wnRoll: 0.25 },
  aero: { wn: 0.4, zeta: 0.9, wMax: (2.5 * Math.PI) / 180, wnRoll: 0.25, deadband: (3.0 * Math.PI) / 180, rollBand: (1.0 * Math.PI) / 180, accelLimit: 0.0035 },
  belly: { wn: 0.45, zeta: 1.0, wMax: (3.2 * Math.PI) / 180, wnRoll: 0.2, deadband: (5.0 * Math.PI) / 180, rollBand: (1.5 * Math.PI) / 180, accelLimit: 0.0035 },
  // the flip only pitches: three engines hold roll by differential gimbal (no cold gas after ignition, R11c)
  flip: { wn: 1.3, zeta: 0.9, wMax: (32 * Math.PI) / 180, wnRoll: 0.35, rollMax: (2 * Math.PI) / 180 },
  // Two sea-level Raptors (T09): 2 × 2.45 MN at ±15°, lever ≈ 24.6 m → ≈ 31 MN·m pitch/yaw
  // (≈ 0.72 rad/s² at Iyy 4.33e7), and ≈ 0.29 rad/s² at the 40 % throttle floor; differential gimbal
  // gives ≈ 1.0 rad/s² of roll at Ixx 1.6e6. The old single-engine set (0.12 rad/s² pitch, 8°/s) was
  // the limiting factor for the 45° divert attitude. axisAccel uses the *floor* authority (0.3)
  // so the time-optimal rate cap does not over-promise braking; roll is strong (differential).
  landing: { wn: 1.1, zeta: 0.9, wMax: (24 * Math.PI) / 180, wnRoll: 0.8, rollMax: (18 * Math.PI) / 180, axisAccel: [0.5, 0.32, 0.32] },
} as const satisfies Record<string, Gains>

/** body-axis error vector (rad) */
export const attitudeError = (q: Q, t: AttitudeTarget): V3 => {
  const xb = qrot(q, v3(1, 0, 0))
  const xd = normalize(t.x)
  if (t.bankAxis) {
    // signed angle about v̂ between the current and desired lift-plane directions
    const a = normalize(t.bankAxis)
    const pb = sub(xb, scale(a, dot(xb, a)))
    const pd = sub(xd, scale(a, dot(xd, a)))
    if (norm(pb) > 1e-6 && norm(pd) > 1e-6) {
      const ang = Math.atan2(dot(cross(pb, pd), a), dot(pb, pd))
      const eb = qrotInv(q, scale(a, ang))
      // optional roll target about the body axis (aerodynamically free for the axisymmetric hull)
      if (t.z && t.spin === undefined) {
        const zb = qrot(q, v3(0, 0, 1))
        const zp = sub(t.z, scale(xb, dot(t.z, xb)))
        if (norm(zp) > 1e-6) {
          const zpn = normalize(zp)
          return v3(Math.atan2(dot(cross(zb, zpn), xb), dot(zb, zpn)), eb.y, eb.z)
        }
      }
      return eb
    }
  }
  const ax = cross(xb, xd)
  const s = norm(ax)
  const c = dot(xb, xd)
  const ang = Math.atan2(s, c)
  const eI = s > 1e-12 ? scale(ax, ang / s) : c < 0 ? qrot(q, v3(0, 0, Math.PI)) : ZERO
  let e = qrotInv(q, eI)
  if (t.z && t.spin === undefined) {
    const zb = qrot(q, v3(0, 0, 1))
    const zp = sub(t.z, scale(xb, dot(t.z, xb)))
    if (norm(zp) > 1e-6) {
      const zpn = normalize(zp)
      const roll = Math.atan2(dot(cross(zb, zpn), xb), dot(zb, zpn))
      e = v3(roll, e.y, e.z)
    }
  }
  return e
}

export const torqueCommand = (q: Q, w: V3, I: V3, t: AttitudeTarget, g: Gains, mFeedForward: V3): { tau: V3; err: V3 } => {
  const e = attitudeError(q, t)
  const kd = 2 * g.zeta * g.wn
  const k = g.wn / (2 * g.zeta)
  // pointing (y,z) is saturated as a vector so a diagonal slew does not exceed wMax; roll is
  // saturated on its own so an uncorrectable roll error never starves pointing control
  const py = k * e.y, pz = k * e.z
  const pn = Math.hypot(py, pz)
  // time-optimal braking: never command more rate than the actuators can stop within the error
  const eMag = Math.hypot(e.y, e.z)
  const wBrake = g.accelLimit ? Math.sqrt(2 * g.accelLimit * eMag) * 0.85 : Infinity
  const wCap = Math.min(g.wMax, wBrake)
  const s = pn > wCap ? wCap / pn : 1
  const rMax = g.rollMax ?? g.wMax
  let wc = v3(t.spin !== undefined ? t.spin : clamp(k * e.x, -rMax, rMax), py * s, pz * s)
  if (g.axisAccel) {
    const cap = (ei: number, a: number): number => 0.8 * Math.sqrt(2 * a * Math.abs(ei)) + 0.002
    wc = v3(t.spin !== undefined ? wc.x : clamp(wc.x, -cap(e.x, g.axisAccel[0]), cap(e.x, g.axisAccel[0])), clamp(wc.y, -cap(e.y, g.axisAccel[1]), cap(e.y, g.axisAccel[1])), clamp(wc.z, -cap(e.z, g.axisAccel[2]), cap(e.z, g.axisAccel[2])))
  }
  const Iw = hadamard(I, w)
  const kdRoll = 2 * g.zeta * (g.wnRoll ?? g.wn)
  // rates about a free axis belong to the aerodynamics: exclude them from the rate error too
  const fB = t.free ? normalize(qrotInv(q, t.free)) : null
  const wCtl = fB ? sub(w, scale(fB, dot(w, fB))) : w
  const dw = sub(wc, wCtl)
  let tau = sub(add(v3(I.x * dw.x * kdRoll, I.y * dw.y * kd, I.z * dw.z * kd), cross(w, Iw)), mFeedForward)
  if (g.deadband) {
    // phase-plane deadband per axis: no firing while attitude and rate errors are both small
    const db = g.deadband
    const inBand = (ei: number, dwi: number): boolean => Math.abs(ei) < db && Math.abs(dwi) < db / 10
    tau = v3(inBand(t.spin !== undefined ? 0 : e.x, dw.x) ? 0 : tau.x, inBand(e.y, dw.y) ? 0 : tau.y, inBand(e.z, dw.z) ? 0 : tau.z)
  }
  if (g.rollBand !== undefined && (t.spin !== undefined || Math.abs(e.x) < 5 * g.rollBand) && Math.abs(dw.x) < g.rollBand) tau = v3(0, tau.y, tau.z)
  if (t.free) {
    // remove the torque component about the aerodynamically stable axis (passive trim does it)
    const f = normalize(qrotInv(q, t.free))
    tau = sub(tau, scale(f, dot(tau, f)))
  }
  return { tau, err: e }
}

/**
 * Cold-gas allocation by pure torque couples (two thrusters fired with equal duty, zero net
 * force). Each couple serves the body axis/sign its torque points along; duty = demand/capacity
 * (PWM average over the control period); a small deadband models the minimum impulse bit.
 */
export const allocateRcs = (
  thrusters: readonly Thruster[], couples: readonly Couple[], xcg: number, tau: V3, enabled: boolean,
): { duty: number[]; achieved: V3 } => {
  const duty = thrusters.map(() => 0)
  if (!enabled) return { duty, achieved: ZERO }
  const torqueOf = (i: number): V3 => cross(sub(thrusters[i].pos, v3(xcg, 0, 0)), scale(thrusters[i].dir, thrusters[i].thrust))
  const ct = couples.map((c) => add(torqueOf(c.a), torqueOf(c.b)))
  const comp = (m: V3, k: number): number => (k === 0 ? m.x : k === 1 ? m.y : m.z)
  const axis = ct.map((m) => {
    const a = [Math.abs(m.x), Math.abs(m.y), Math.abs(m.z)]
    return a[0] >= a[1] && a[0] >= a[2] ? 0 : a[1] >= a[2] ? 1 : 2
  })
  const dutyC = ct.map((m, i) => {
    const k = axis[i]
    const s = Math.sign(comp(m, k))
    const want = comp(tau, k) * s
    const cap = ct.reduce((acc, mm, j) => (axis[j] === k && Math.sign(comp(mm, k)) === s ? acc + Math.abs(comp(mm, k)) : acc), 0)
    const d = want > 0 && cap > 0 ? clamp(want / cap, 0, 1) : 0
    return d < 0.004 ? 0 : d
  })
  couples.forEach((c, i) => {
    duty[c.a] += dutyC[i]
    duty[c.b] += dutyC[i]
  })
  const achieved = ct.reduce((acc, m, i) => add(acc, scale(m, dutyC[i])), ZERO)
  return { duty: duty.map((d) => clamp(d, 0, 1)), achieved }
}

/**
 * Thrust-vector allocation (equal gimbal on all running engines):
 * τ_y = Σ T_i (z_i − L δp),  τ_z = −Σ T_i (L δy + y_i),  L = x_cg − x_gimbal.
 */
/**
 * Common gimbal (gp about body y, gy about body z) for the active sea-level engines. An off-axis
 * engine also makes roll: τx = −Σ T_i (y_i sin gp + z_i sin gy). With one engine (the landing burn)
 * that roll would overwhelm the cold-gas roll couples, so the roll-producing gimbal component is
 * limited to what the RCS can hold (`rollCap`, N·m): roll stability has priority.
 */
export const allocateGimbal = (engines: readonly Engine[], thrusts: readonly number[], xcg: number, tau: V3, rollCap = Infinity): { gp: number; gy: number } => {
  const active = engines.map((e, i) => ({ e, T: thrusts[i] ?? 0 })).filter((a) => a.e.kind === "SL" && a.T > 1e3)
  const Tsum = active.reduce((s, a) => s + a.T, 0)
  if (Tsum < 1e3) return { gp: 0, gy: 0 }
  const L = xcg - active[0].e.gimbal.x
  const zT = active.reduce((s, a) => s + a.T * a.e.gimbal.z, 0)
  const yT = active.reduce((s, a) => s + a.T * a.e.gimbal.y, 0)
  const gmax = active[0].e.gimbalMax
  // Exact plant model: thrust along the unit vector (c, s₂, −s₁), s₁ = sin gp cos gy, s₂ = sin gy,
  // c = cos gp cos gy = √(1 − s₁² − s₂²); r = (−L, y, z) from the CG:
  //   τx = −(yT s₁ + zT s₂) (exact, linear),  τy = zT·c − L·T·s₁,  τz = −yT·c − L·T·s₂.
  // The pitch/yaw map is isotropic, so the best command within the roll slab |τx| ≤ rollCap is the
  // Euclidean projection of the unconstrained solution onto the slab, then the gimbal box. The weak
  // dependence on c (≥ 0.93 within 15°) is resolved by a fixed point (converges in 2–3 passes).
  const c1 = -yT, c2 = -zT
  const cc = c1 * c1 + c2 * c2
  let s1 = 0, s2 = 0, cAx = 1
  for (let it = 0; it < 4; it++) {
    s1 = (zT * cAx - tau.y) / (L * Tsum)
    s2 = -(tau.z + yT * cAx) / (L * Tsum)
    if (Number.isFinite(rollCap) && cc > 1e6) {
      const r = c1 * s1 + c2 * s2
      if (Math.abs(r) > rollCap) {
        const d = (r - Math.sign(r) * rollCap) / cc
        s1 -= d * c1
        s2 -= d * c2
      }
    }
    cAx = Math.sqrt(Math.max(0, 1 - s1 * s1 - s2 * s2))
  }
  // The plant (craft engineThrust) points the thrust along (cos gp cos gy, sin gy, −sin gp cos gy), so the
  // lateral force components are exactly T·sin gy and −T·sin gp·cos gy. With s₂ := sin gy and
  // s₁ := sin gp·cos gy the torque model above is exact (no small-angle error in τx, τy, τz):
  //   gy = asin(s₂),  gp = asin(s₁ / cos gy), each clamped to the gimbal limit.
  const sm = Math.sin(gmax)
  const gy = Math.asin(clamp(s2, -sm, sm))
  const gp = Math.asin(clamp(s1 / Math.cos(gy), -sm, sm))
  return { gp, gy }
}

/**
 * Per-engine (differential) gimbal allocation for all running sea-level engines.
 * Small-angle model with s_i = (sin gp_i, sin gy_i) for engine i at (y_i, z_i), lever L = x_cg − x_gimbal:
 *   τx = −Σ T_i (y_i s_pi + z_i s_yi),   τy = Σ T_i z_i − L Σ T_i s_pi,   τz = −Σ T_i y_i − L Σ T_i s_yi.
 * With ≥ 2 engines the 3×2N map has full rank: roll is controlled by differential gimballing (as on the
 * real vehicle), so there is no weak axis. Minimum-norm solution s = Aᵀ(AAᵀ)⁻¹(τ − b), clipped to the
 * gimbal limit. With one engine it falls back to the roll-priority projection (allocateGimbal).
 */
export const allocateGimbals = (engines: readonly Engine[], thrusts: readonly number[], xcg: number, tau: V3, rollCap = Infinity): { gp: number[]; gy: number[] } => {
  const idx = engines.map((_, i) => i).filter((i) => engines[i].kind === "SL" && (thrusts[i] ?? 0) > 1e3)
  const gp = engines.map(() => 0), gy = engines.map(() => 0)
  if (idx.length === 0) return { gp, gy }
  if (idx.length === 1) {
    const one = allocateGimbal(engines, thrusts, xcg, tau, rollCap)
    idx.forEach((i) => { gp[i] = one.gp; gy[i] = one.gy })
    return { gp, gy }
  }
  const L = xcg - engines[idx[0]].gimbal.x
  // rows of A (3 × 2N) as arrays over the unknowns [s_p0, s_y0, s_p1, s_y1, …]
  const ax: number[] = [], ay: number[] = [], az: number[] = []
  for (const i of idx) {
    const T = thrusts[i], y = engines[i].gimbal.y, z = engines[i].gimbal.z
    ax.push(-T * y, -T * z); ay.push(-L * T, 0); az.push(0, -L * T)
  }
  const rows = [ax, ay, az]
  const M = rows.map((r1) => rows.map((r2) => r1.reduce((s, v, k) => s + v * r2[k], 0)))
  // 3×3 inverse (Cramer)
  const det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
  if (Math.abs(det) < 1e-30) {
    const one = allocateGimbal(engines, thrusts, xcg, tau, rollCap)
    idx.forEach((i) => { gp[i] = one.gp; gy[i] = one.gy })
    return { gp, gy }
  }
  const inv = [
    [(M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det, (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det, (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det],
    [(M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det, (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det, (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det],
    [(M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det, (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det, (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det],
  ]
  // exact plant: s_p = sin gp cos gy, s_y = sin gy and axial thrust T·c_i, c_i = √(1 − s_p² − s_y²), so
  //   τy = Σ T_i z_i c_i − L Σ T_i s_pi,  τz = −Σ T_i y_i c_i − L Σ T_i s_yi  (τx row exact as above).
  // A is independent of c; the offsets Σ T z c, Σ T y c are updated by a fixed point (2–3 passes).
  // Priority allocation (pointing first, roll second). The minimum-norm solution is linear in the
  // right-hand side, so it splits exactly into a pitch/yaw part (rhs = (0, τy−b_z, τz+b_y)) and a
  // roll part (rhs = (τx, 0, 0), the differential-gimbal mode). When a feasible command fits the
  // gimbal box the sum is the old solution unchanged. At saturation the old code clipped each engine
  // on its own, which bent the pitch/yaw torque whenever roll used the gimbal range — the thrust
  // axis (= the trajectory during the burn) then ran away while the ship rolled (the 250 m divergence
  // in the landing traces). Here the pitch/yaw part is scaled uniformly (direction preserved) and the
  // roll part gets only the margin that is left.
  const sm = Math.sin(engines[idx[0]].gimbalMax)
  const pPY = idx.map(() => 0), yPY = idx.map(() => 0), pR = idx.map(() => 0), yR = idx.map(() => 0)
  const sp = idx.map(() => 0), sy = idx.map(() => 0)
  const solve = (rhs: readonly number[], P: number[], Y: number[]): void => {
    const lam = inv.map((r) => r[0] * rhs[0] + r[1] * rhs[1] + r[2] * rhs[2])
    idx.forEach((_, k) => {
      P[k] = ax[2 * k] * lam[0] + ay[2 * k] * lam[1] + az[2 * k] * lam[2]
      Y[k] = ax[2 * k + 1] * lam[0] + ay[2 * k + 1] * lam[1] + az[2 * k + 1] * lam[2]
    })
  }
  for (let it = 0; it < 4; it++) {
    let bz = 0, by = 0
    idx.forEach((i, k) => {
      const c = Math.sqrt(Math.max(0, 1 - sp[k] * sp[k] - sy[k] * sy[k]))
      bz += thrusts[i] * engines[i].gimbal.z * c
      by += thrusts[i] * engines[i].gimbal.y * c
    })
    solve([0, tau.y - bz, tau.z + by], pPY, yPY)
    solve([tau.x, 0, 0], pR, yR)
    // 1) pitch/yaw: uniform scale into the box (keeps the torque direction)
    let a = 1
    idx.forEach((_, k) => { a = Math.min(a, sm / Math.max(Math.abs(pPY[k]), 1e-12), sm / Math.max(Math.abs(yPY[k]), 1e-12)) })
    // 2) roll: the largest β ∈ [0, 1] that keeps every component inside the box
    let b = 1
    const fit = (base: number, r: number): void => {
      if (Math.abs(r) < 1e-12) return
      const hi = ((r > 0 ? sm : -sm) - base) / r
      b = Math.min(b, Math.max(0, hi))
    }
    idx.forEach((_, k) => { fit(a * pPY[k], pR[k]); fit(a * yPY[k], yR[k]) })
    idx.forEach((_, k) => { sp[k] = a * pPY[k] + b * pR[k]; sy[k] = a * yPY[k] + b * yR[k] })
  }
  idx.forEach((i, k) => {
    gy[i] = Math.asin(clamp(sy[k], -sm, sm))
    gp[i] = Math.asin(clamp(sp[k] / Math.cos(gy[i]), -sm, sm))
  })
  return { gp, gy }
}
