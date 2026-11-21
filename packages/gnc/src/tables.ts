import { interp2, bracket } from "@ship/math"
import { v3, normalize } from "@ship/math"
import { atmosphere } from "@ship/earth"
import { type AeroModel, staticCoeffs } from "@ship/aero"
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
