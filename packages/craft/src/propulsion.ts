import { clamp } from "@ship/math"
import { type V3, v3, ZERO, cross, sub, add, scale } from "@ship/math"
import { G0 } from "@ship/earth"
import type { Engine, Thruster, VehicleConfig } from "@ship/craft"

/**
 * Raptor 3 (sea level) model: F = τ·F_vac,100 − p∞·A_e, ṁ = τ·F_vac,100/(Isp_vac g0),
 * F_vac,100 = F_SL + p_SL·A_e (2.45 MN SL, Isp_SL 350 s → Isp_vac ≈ 369 s).
 * Start: spool 0→1 over spoolTime (first-order shaped), throttle range 40–100 %,
 * electric TVC gimbal ±15° at 20°/s. Cold-gas RCS: N2 at Isp 73 s, PWM-averaged duty.
 */
export interface EngineState {
  readonly on: boolean
  readonly spool: number // 0..1
  readonly throttle: number // commanded and achieved (slew 60 %/s)
  readonly gp: number // gimbal pitch (rotation about body y), rad
  readonly gy: number // gimbal yaw (about body z), rad
}
export const engineOff: EngineState = { on: false, spool: 0, throttle: 0.4, gp: 0, gy: 0 }

export interface EngineCmd {
  readonly on: boolean
  readonly throttle: number
  readonly gp: number
  readonly gy: number
}

const P_SL = 101325

export const fVac100 = (e: Engine): number => e.thrustSL + P_SL * e.exitArea
export const ispVac = (e: Engine): number => e.isp0 * (fVac100(e) / e.thrustSL)

export const engineStep = (e: Engine, s: EngineState, c: EngineCmd, dt: number): EngineState => {
  if (e.kind !== "SL") return s
  const spool = c.on ? Math.min(1, s.spool + dt / e.spoolTime) : Math.max(0, s.spool - dt / 0.8)
  const thr = clamp(c.throttle, e.minThrottle, 1)
  const throttle = s.throttle + clamp(thr - s.throttle, -0.6 * dt, 0.6 * dt)
  const rate = e.gimbalRate * dt
  const gp = clamp(s.gp + clamp(clamp(c.gp, -e.gimbalMax, e.gimbalMax) - s.gp, -rate, rate), -e.gimbalMax, e.gimbalMax)
  const gy = clamp(s.gy + clamp(clamp(c.gy, -e.gimbalMax, e.gimbalMax) - s.gy, -rate, rate), -e.gimbalMax, e.gimbalMax)
  return { on: c.on, spool, throttle, gp, gy }
}

/** thrust shaping during start: ignition delay then S-curve */
const spoolCurve = (s: number): number => (s < 0.25 ? 0 : clamp(((s - 0.25) / 0.75) ** 1.5, 0, 1))

export interface EngineOut {
  readonly F: V3 // body
  readonly M: V3 // about CG
  readonly mdot: number
  readonly thrust: number
}

export const engineThrust = (e: Engine, s: EngineState, pAmb: number, xcg: number): EngineOut => {
  if (e.kind !== "SL" || s.spool <= 0) return { F: ZERO, M: ZERO, mdot: 0, thrust: 0 }
  const k = spoolCurve(s.spool)
  const fv = fVac100(e) * s.throttle * k
  const thrust = Math.max(0, fv - pAmb * e.exitArea * k)
  const mdot = fv / (ispVac(e) * G0)
  // thrust direction: +x rotated by gimbal pitch (about y) then yaw (about z)
  const dir = v3(Math.cos(s.gp) * Math.cos(s.gy), Math.sin(s.gy), -Math.sin(s.gp) * Math.cos(s.gy))
  const F = scale(dir, thrust)
  const r = sub(e.gimbal, v3(xcg, 0, 0))
  return { F, M: cross(r, F), mdot, thrust }
}

export const sumEngines = (outs: readonly EngineOut[]): EngineOut =>
  outs.reduce((a, b) => ({ F: add(a.F, b.F), M: add(a.M, b.M), mdot: a.mdot + b.mdot, thrust: a.thrust + b.thrust }), { F: ZERO, M: ZERO, mdot: 0, thrust: 0 })

/** RCS: duty in [0,1] per thruster (PWM average over the control period). */
export interface RcsOut {
  readonly F: V3
  readonly M: V3
  readonly mdot: number
}
export const rcsWrench = (v: VehicleConfig, duty: readonly number[], xcg: number, gasLeft: number): RcsOut => {
  if (gasLeft <= 0) return { F: ZERO, M: ZERO, mdot: 0 }
  let F = ZERO
  let M = ZERO
  let mdot = 0
  v.thrusters.forEach((t: Thruster, i) => {
    const d = clamp(duty[i] ?? 0, 0, 1)
    if (d <= 0) return
    const f = scale(t.dir, t.thrust * d)
    F = add(F, f)
    M = add(M, cross(sub(t.pos, v3(xcg, 0, 0)), f))
    mdot += (t.thrust * d) / (v.rcsIsp * G0)
  })
  return { F, M, mdot }
}
