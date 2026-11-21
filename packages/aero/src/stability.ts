import { v3, ZERO } from "@ship/math"
import { aeroWrench, type AeroModel } from "./aero"
import type { AftFlapState } from "./flaps"

/**
 * Static-margin study for the bare hull and the optional aft flap set.
 *
 * Convention in this repository: body x = 0 at the aft skirt plane and grows toward the nose.
 * The centre of pressure `cp` is the x station of the resultant normal force. A flap set mounted
 * near the engine section therefore has `cp << xcg` and a **positive** static margin
 *
 *   SM = xcg − cp        (positive = CP aft of the CG = stable)
 *
 * `cpMinusCg = cp − xcg` is also returned because the parent report quotes `CP − CG` with the
 * opposite sign; the raw CP numbers are the common reference.
 *
 * The CP is evaluated at a finite α (default 2°, where the nose-first body is no longer in its
 * tiny stable band). α = 1° is also useful for comparison with the α≈0 linearisation. The pitch
 * stiffness dCm/dα is a central difference over ±0.5°.
 *
 * Caution: a fixed differential flap deflection can move the apparent CP (a trim force) while
 * `pitchStiffness` stays negative. A positive `margin` alone is therefore not a stability proof;
 * require `pitchStiffness > 0` as well (or an α-scheduled active controller).
 */
export interface StaticMarginInput {
  readonly mach: number
  readonly xcg: number
  readonly alpha?: number // rad, default 2°
  readonly cpMax?: number // default 1.84
  readonly flaps?: AftFlapState
}

export interface StaticMarginOut {
  readonly cp: number // body x from the aft skirt plane, m
  readonly cpFromNose: number // m
  readonly margin: number // xcg − cp, positive = CP aft of CG
  readonly cpMinusCg: number // cp − xcg, the parent report's raw sign
  readonly CN: number // normal-force coefficient at α
  readonly Cm: number // pitch-moment coefficient about the CG at α
  readonly normalForce: number // N, signed body +z
  readonly drag: number // N, positive aft
  readonly pitchStiffness: number // dCm/dα, per rad
  readonly normalForceSlope: number // dFz/dα, N/rad
}

const A = 300 // m/s reference sound speed used by staticCoeffs
const RHO = 1e-3 // kg/m³; the coefficients are q-normalised
const MU = 1.5e-5

const wrenchAt = (m: AeroModel, inp: StaticMarginInput, alpha: number): ReturnType<typeof aeroWrench> =>
  aeroWrench(m, {
    vAir: v3(inp.mach * A * Math.cos(alpha), 0, -inp.mach * A * Math.sin(alpha)),
    w: ZERO, rho: RHO, a: A, mu: MU, xcg: inp.xcg,
    cpMax: inp.cpMax ?? 1.84, enginesOn: false, knudsen: 0, flaps: inp.flaps,
  })

export const staticMarginAt = (m: AeroModel, inp: StaticMarginInput): StaticMarginOut => {
  const alpha = inp.alpha ?? (2 * Math.PI) / 180
  const o = wrenchAt(m, inp, alpha)
  const h = (0.5 * Math.PI) / 180
  const p = wrenchAt(m, inp, alpha + h)
  const n = wrenchAt(m, inp, alpha - h)
  const cp = o.xcp
  const cm = o.M.y / (o.q * m.sref * m.lref)
  return {
    cp,
    cpFromNose: m.lref - cp,
    margin: inp.xcg - cp,
    cpMinusCg: cp - inp.xcg,
    CN: o.CN,
    Cm: cm,
    normalForce: o.F.z,
    drag: -o.F.x,
    pitchStiffness: (p.M.y - n.M.y) / (2 * h) / (o.q * m.sref * m.lref),
    normalForceSlope: (p.F.z - n.F.z) / (2 * h),
  }
}
