import { type V3, v3, add, cross, norm, scale, ZERO } from "@ship/math"
import type { BellFinDesign, Engine } from "@ship/craft"

/**
 * Open-base gimballed-bell fins.
 *
 * With the Raptor SL engines cold, the existing ±15° / 20°/s electric gimbals are used as small aft
 * aerodynamic fins: each bell is tilted about its gimbal point into the base flow. This gives the
 * belly/high-α descent a bank actuator several times stronger than the cold-gas couples, without
 * firing anything.
 *
 * Model (deliberately simple and conservative):
 *  - a bell is a blunt body at incidence β to the body axis; the lateral force obeys the Newtonian
 *    flat-plate normal force C_N = 2 sin²β, so a deflection δ changes the side force by
 *    dC_N/dβ = 4 sinβ·cosβ per radian. Only that *control increment* is added here; the base normal
 *    force on the foreshortened bell is already part of the base drag in the main aero table.
 *  - the local dynamic pressure is q·wakeFactor. The base of a belly-first ship is on the lee side,
 *    so its wake is not freestream even with the skirt open; `wakeFactor` is the single measured
 *    design assumption (0.3 fully shadowed, 1.0 freestream).
 *  - the force acts at the gimbal point (x_gimbal, y, z), lever L = x_cg − x_gimbal about the CG.
 *    Three bells produce a full 3-axis moment (pitch, yaw and differential roll).
 *  - the side force is along the gimbal deflection, the same convention as `engineThrust`: the
 *    lateral thrust direction is (0, sin gy, −sin gp cos gy).
 *  - an open base vents the cavity, which raises the base pressure: the base-drag coefficient is
 *    scaled by `baseDragFactor`, and the bay sees a fraction `baseHeatFraction` of the stagnation
 *    flux (reported by the thermal model, not used here).
 *  - stowed (zero gimbal) produces exactly zero force and moment, so a closed-base or cold-gimbal
 *    baseline is unchanged.
 */

export interface BellFinInput {
  /** body-frame velocity relative to the air, m/s */
  readonly vAir: V3
  readonly rho: number
  /** actual gimbal angles per engine, rad (engine state from the actuator model) */
  readonly gimbals: readonly { readonly gp: number; readonly gy: number }[]
  readonly xcg: number
}

export interface BellFinOut {
  readonly F: V3
  readonly M: V3
  /** freestream dynamic pressure, Pa */
  readonly q: number
  /** bell incidence to the body axis, rad */
  readonly beta: number
  /** lateral force per bell per radian of gimbal (N/rad), before the sin(δ) */
  readonly thrustEq: number
}

export const BELL_FIN_OFF: BellFinOut = { F: ZERO, M: ZERO, q: 0, beta: 0, thrustEq: 0 }

/** dC_N/dβ of the Newtonian flat plate, per radian (positive, peak at 45°) */
export const bellCnDerivative = (beta: number): number => {
  const s = Math.sin(beta)
  const c = Math.cos(beta)
  return 4 * Math.abs(s * c)
}

/**
 * Lateral force per bell per radian of gimbal: q·wake·A·dC_N/dβ. The allocator uses this as an
 * equivalent thrust (`F = thrustEq·sin δ`) so the same minimum-norm gimbal allocation can be reused.
 */
export const bellFinThrustEq = (d: BellFinDesign, rho: number, vAir: V3): number => {
  const V = norm(vAir)
  if (V < 0.3) return 0
  const q = 0.5 * rho * V * V
  const beta = Math.acos(Math.min(1, Math.abs(vAir.x) / V))
  return q * d.wakeFactor * d.area * bellCnDerivative(beta) * d.ductFactor
}

/** bell incidence to the body axis (0 = axial flow), rad */
export const bellFinBeta = (vAir: V3): number => {
  const V = norm(vAir)
  return V < 1e-9 ? 0 : Math.acos(Math.min(1, Math.abs(vAir.x) / V))
}

/** Force and moment from the actual gimbal deflections. */
export const bellFinWrench = (
  d: BellFinDesign | null,
  engines: readonly Engine[],
  inp: BellFinInput,
): BellFinOut => {
  if (!d) return BELL_FIN_OFF
  const V = norm(inp.vAir)
  if (V < 0.3 || inp.rho < 1e-14) return { ...BELL_FIN_OFF, q: 0, beta: bellFinBeta(inp.vAir) }
  const q = 0.5 * inp.rho * V * V
  const beta = bellFinBeta(inp.vAir)
  const eq = q * d.wakeFactor * d.area * bellCnDerivative(beta) * d.ductFactor
  let F: V3 = ZERO
  let M: V3 = ZERO
  for (let i = 0; i < Math.min(3, engines.length); i++) {
    const e = engines[i]
    if (e.kind !== "SL") continue
    const g = inp.gimbals[i] ?? { gp: 0, gy: 0 }
    const s1 = Math.sin(g.gp) * Math.cos(g.gy)
    const s2 = Math.sin(g.gy)
    // lateral force along the deflection (same sign convention as the engine thrust vector)
    const Fi = scale(v3(0, s2, -s1), eq)
    const r = v3(e.gimbal.x - inp.xcg, e.gimbal.y, e.gimbal.z)
    F = add(F, Fi)
    M = add(M, cross(r, Fi))
  }
  return { F, M, q, beta, thrustEq: eq }
}
