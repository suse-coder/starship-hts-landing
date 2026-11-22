import { interp2, bracket } from "@ship/math"
import { type V3, v3, add, cross, dot, norm, normalize, scale, sub, ZERO } from "@ship/math"
import { atmosphere } from "@ship/earth"
import { type AeroModel, aeroWrench, staticCoeffs } from "@ship/aero"
import { makeMhdModel, mhdBasis } from "@ship/mhd"
import { lookupStag, type ShockTable } from "@ship/gas"
import type { VehicleConfig } from "@ship/craft"
import { allocateMhd } from "./mhdAlloc"
import { alphaNoseFirst } from "./schedule"

/** Static aerodynamic tables for guidance (predictor) on (α, Mach), and the stable trim α(M). */
export interface AeroTable {
  readonly alphas: number[] // rad
  readonly machs: number[]
  readonly CL: number[]
  readonly CD: number[]
  readonly Cm: number[]
  readonly trim: number[] // per Mach: stable trim α in [30°, 95°]
}

export const buildAeroTable = (m: AeroModel, xcg: number): AeroTable => {
  const alphas = Array.from({ length: 39 }, (_, i) => (i * 2.5 * Math.PI) / 180) // 0 … 95°
  const machs = [0.3, 0.6, 0.85, 1.0, 1.2, 1.6, 2, 3, 4, 6, 8, 12, 20, 28]
  const CL: number[] = [], CD: number[] = [], Cm: number[] = []
  alphas.forEach((a) =>
    machs.forEach((M) => {
      const c = staticCoeffs(m, a, M, xcg, M > 5 ? 1.9 : 1.84)
      CL.push(c.CL)
      CD.push(c.CD)
      Cm.push(c.Cm)
    }),
  )
  const nM = machs.length
  const trim = machs.map((_, j) => {
    // stable crossing: Cm goes from < 0 to > 0 with increasing α (restoring)
    for (let i = alphas.length - 1; i > 0; i--) {
      const a0 = alphas[i - 1], a1 = alphas[i]
      if (a0 < (30 * Math.PI) / 180) break
      const c0 = Cm[(i - 1) * nM + j], c1 = Cm[i * nM + j]
      if (c0 <= 0 && c1 > 0) return a0 + ((a1 - a0) * -c0) / (c1 - c0)
    }
    return (80 * Math.PI) / 180
  })
  return { alphas, machs, CL, CD, Cm, trim }
}

export const aeroLookup = (t: AeroTable, alpha: number, mach: number): { CL: number; CD: number; Cm: number } => ({
  CL: interp2(t.alphas, t.machs, t.CL, alpha, mach),
  CD: interp2(t.alphas, t.machs, t.CD, alpha, mach),
  Cm: interp2(t.alphas, t.machs, t.Cm, alpha, mach),
})

export const trimAlpha = (t: AeroTable, mach: number): number => {
  const [i, f] = bracket(t.machs, mach)
  return t.trim[i] * (1 - f) + t.trim[i + 1] * f
}

/**
 * MHD drag increment ΔCD on (altitude, velocity) for the nominal nose-first glide: at each point
 * the electrode loading that trims the aerodynamic moment at the scheduled α is solved with the
 * flight allocator, and its drag component is tabulated (so the predictor dissipates energy the
 * way the flight does).
 */
export interface MhdDragTable {
  readonly hs: number[]
  readonly us: number[]
  readonly dCD: number[]
  readonly dCL: number[] // trim lift loss (negative): the restoring nose force points to the windward side
}

export const buildMhdDragTable = (v: VehicleConfig, aero: AeroModel, shock: ShockTable, xcg: number): MhdDragTable => {
  const mm = makeMhdModel(v)
  const hs = Array.from({ length: 12 }, (_, i) => 35000 + i * 5000)
  const us = Array.from({ length: 14 }, (_, i) => 1500 + i * 500)
  const dCD: number[] = []
  const dCL: number[] = []
  hs.forEach((h) => {
    const a = atmosphere(h)
    us.forEach((U) => {
      const q = 0.5 * a.rho * U * U
      const al = alphaNoseFirst(U)
      const vhat = normalize(v3(Math.cos(al), 0, -Math.sin(al)))
      const st = lookupStag(shock, h, U)
      const b = mhdBasis(mm, { NI: v.coil.NI, vhat, U, rhoInf: a.rho, pInf: a.p, qInf: q, stag: st, xcg, kappa: v.design.hallClosure, seedFraction: v.design.seedFraction })
      const need = -staticCoeffs(aero, al, U / a.a, xcg, st.cpMax).Cm * q * aero.sref * aero.lref
      const o = allocateMhd(b, v3(0, need, 0), q, v.shape.p.noseRadius, 0).out
      const drag = -(o.F.x * vhat.x + o.F.z * vhat.z)
      const lift = o.F.x * Math.sin(al) + o.F.z * Math.cos(al)
      dCD.push(q > 1 ? Math.max(0, drag) / (q * aero.sref) : 0)
      dCL.push(q > 1 ? lift / (q * aero.sref) : 0)
    })
  })
  return { hs, us, dCD, dCL }
}

export const mhdDragLookup = (t: MhdDragTable, h: number, U: number): number => interp2(t.hs, t.us, t.dCD, h, U)
export const mhdLiftLookup = (t: MhdDragTable, h: number, U: number): number => interp2(t.hs, t.us, t.dCL, h, U)

/**
 * Landing aerodynamic database: body-axis force and pitch-moment coefficients for the powered
 * descent, α from 60° (end of the belly flop) to 180° (tail-first), low subsonic, engines running
 * (no base drag). The entry table above stops at 95°, so before this table existed the landing had
 * no aerodynamic model at all: the planner used a constant C_D·A guess and the attitude loop fed
 * forward the moment of the α = 95° row while the ship flew tail-first.
 *
 * Convention (same as `staticCoeffs`): the air-relative velocity in body axes is
 * V·(cos α, 0, −sin α); CX, CZ are the force coefficients along body x and body z of that frame,
 * Cm the moment about body y at `xcgRef`, all referred to q·S (and L for the moment).
 */
export interface LandingAero {
  readonly alphas: number[]
  readonly CX: number[]
  readonly CZ: number[]
  readonly Cm: number[]
  readonly sref: number
  readonly lref: number
  readonly xcgRef: number
}

export const buildLandingAero = (m: AeroModel, xcgRef: number): LandingAero => {
  const alphas = Array.from({ length: 25 }, (_, i) => ((60 + i * 5) * Math.PI) / 180)
  const V = 70, rho = 1.1, a = 340
  const q = 0.5 * rho * V * V
  const CX: number[] = [], CZ: number[] = [], Cm: number[] = []
  for (const al of alphas) {
    const o = aeroWrench(m, { vAir: v3(V * Math.cos(al), 0, -V * Math.sin(al)), w: ZERO, rho, a, mu: 1.8e-5, xcg: xcgRef, cpMax: 1.84, enginesOn: true, knudsen: 0 })
    CX.push(o.F.x / (q * m.sref)); CZ.push(o.F.z / (q * m.sref)); Cm.push(o.M.y / (q * m.sref * m.lref))
  }
  return { alphas, CX, CZ, Cm, sref: m.sref, lref: m.lref, xcgRef }
}

const lerpTab = (xs: readonly number[], ys: readonly number[], x: number): number => {
  if (x <= xs[0]) return ys[0]
  const n = xs.length
  if (x >= xs[n - 1]) return ys[n - 1]
  const i = Math.min(n - 2, Math.floor((x - xs[0]) / (xs[1] - xs[0])))
  const f = (x - xs[i]) / (xs[i + 1] - xs[i])
  return ys[i] * (1 - f) + ys[i + 1] * f
}

/**
 * Aerodynamic force (N) and moment about the CG (N·m) for a body axis `xHat` and an air-relative
 * velocity `vAir`, both in the same (arbitrary) frame. The force lies in the plane of the two
 * vectors; the moment is about their normal, shifted from the table's reference CG to `xcg`.
 */
export const landingAero = (t: LandingAero, rho: number, xHat: V3, vAir: V3, xcg: number): { readonly F: V3; readonly M: V3 } => {
  const V = norm(vAir)
  if (V < 0.5) return { F: ZERO, M: ZERO }
  const vh = scale(vAir, 1 / V)
  const ca = Math.max(-1, Math.min(1, dot(xHat, vh)))
  const alpha = Math.acos(ca)
  const sa = Math.sin(alpha)
  const qS = 0.5 * rho * V * V * t.sref
  const cx = lerpTab(t.alphas, t.CX, alpha)
  if (sa < 1e-4) return { F: scale(xHat, qS * cx), M: ZERO }
  // body z of the table frame: v̂ = cos α x̂ − sin α ẑ
  const zHat = scale(sub(scale(xHat, ca), vh), 1 / sa)
  const fz = qS * lerpTab(t.alphas, t.CZ, alpha)
  const yHat = cross(zHat, xHat)
  const my = qS * t.lref * lerpTab(t.alphas, t.Cm, alpha) + (xcg - t.xcgRef) * fz
  return { F: add(scale(xHat, qS * cx), scale(zHat, fz)), M: scale(yHat, my) }
}
