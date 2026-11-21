import { clamp, interp1, smoothstep } from "@ship/math"
import { type V3, v3, ZERO } from "@ship/math"
import type { AftFlapSet, VehicleConfig } from "@ship/craft"
import { makePanels, type Panel } from "@ship/craft"
import { aftFlapWrench, type AftFlapState } from "./flaps"

/**
 * Aerodynamic loads by integration over the actual outer mould line, each element using its
 * local velocity (v_air + ω × r) so pitch/yaw damping emerges from the same integral:
 *  - hypersonic: modified Newtonian panels, Cp = Cp,max·sin²δ with Cp,max from the real-gas
 *    stagnation pressure (shock table), plus panel skin friction;
 *  - supersonic → subsonic: Jorgensen's component build-up per strip (NASA TR R-474):
 *    slender-body potential term ∝ dS/dx·sin2α·cos(α/2) + viscous crossflow η·Cdc(Mc)·sin²α·D;
 *    axial force from a CA0(M) table (base drag removed when the engines fill the base).
 *  Blend weight smoothstep(2.5, 5.5, M).
 */
export interface Strip {
  readonly x: number
  readonly dx: number
  readonly r: number
  readonly dSdx: number
}

export interface AeroModel {
  readonly panels: readonly Panel[]
  readonly strips: readonly Strip[]
  readonly sref: number
  readonly lref: number
  readonly baseArea: number
  readonly wetted: number
  /** optional deployable aft flaps; null on the flown baseline */
  readonly aftFlaps: AftFlapSet | null
}

export const makeAeroModel = (v: VehicleConfig): AeroModel => {
  const { panels, edges } = makePanels(v.shape, 16)
  const strips: Strip[] = []
  for (let i = 0; i < edges.length - 1; i++) {
    const a = edges[i]
    const b = edges[i + 1]
    const ra = v.shape.r(a)
    const rb = v.shape.r(b)
    strips.push({ x: 0.5 * (a + b), dx: b - a, r: 0.5 * (ra + rb), dSdx: (Math.PI * (rb * rb - ra * ra)) / (b - a) })
  }
  const wetted = panels.filter((p) => p.station >= 0).reduce((s, p) => s + p.area, 0)
  const R = v.shape.p.radius
  return { panels, strips, sref: Math.PI * R * R, lref: v.shape.p.length, baseArea: Math.PI * R * R, wetted, aftFlaps: v.aftFlaps }
}

const CA0_M = [0, 0.6, 0.8, 0.95, 1.05, 1.2, 1.5, 2, 3, 4, 6, 10]
const CA0_V = [0.2, 0.21, 0.28, 0.55, 0.8, 0.85, 0.8, 0.7, 0.58, 0.5, 0.45, 0.42]
const CAB_V = [0.12, 0.13, 0.17, 0.25, 0.3, 0.28, 0.24, 0.18, 0.1, 0.06, 0.03, 0.01] // base-drag part
const CDC_M = [0, 0.3, 0.5, 0.7, 0.85, 1.0, 1.3, 2, 3, 5, 10]
const CDC_V = [0.62, 0.62, 0.75, 1.05, 1.35, 1.55, 1.6, 1.45, 1.36, 1.3, 1.25]

export interface AeroInput {
  readonly vAir: V3 // vehicle velocity relative to air, body frame
  readonly w: V3 // body rates
  readonly rho: number
  readonly a: number
  readonly mu: number
  readonly xcg: number
  readonly cpMax: number
  readonly enginesOn: boolean
  readonly knudsen: number
  /** deployable aft flap state; omitted/zero means stowed */
  readonly flaps?: AftFlapState
}

export interface AeroOut {
  readonly F: V3
  readonly M: V3
  readonly q: number
  readonly mach: number
  readonly alpha: number
  readonly CA: number
  readonly CN: number
  readonly xcp: number
}

const NULL_OUT: AeroOut = { F: ZERO, M: ZERO, q: 0, mach: 0, alpha: 0, CA: 0, CN: 0, xcp: 0 }

const skinFriction = (Re: number, M: number): number => {
  const lam = 1.328 / Math.sqrt(Math.max(Re, 1e3))
  const turb = (0.074 * Math.max(Re, 1e3) ** -0.2) / (1 + 0.144 * M * M) ** 0.65
  const t = smoothstep(5e5, 5e6, Re)
  return lam * (1 - t) + turb * t
}

export const aeroWrench = (m: AeroModel, inp: AeroInput): AeroOut => {
  const { vAir, w, rho, xcg } = inp
  const V = Math.hypot(vAir.x, vAir.y, vAir.z)
  if (V < 0.3 || rho < 1e-14) return NULL_OUT
  const q = 0.5 * rho * V * V
  const mach = V / inp.a
  const wN = smoothstep(2.5, 5.5, mach)
  let Fx = 0, Fy = 0, Fz = 0, Mx = 0, My = 0, Mz = 0
  // deployable aft flaps: independent of the continuum blend (the model is a local flat plate)
  if (m.aftFlaps && inp.flaps) {
    const flap = aftFlapWrench(m.aftFlaps, inp.flaps, { vAir, w, rho, xcg })
    Fx += flap.F.x; Fy += flap.F.y; Fz += flap.F.z
    Mx += flap.M.x; My += flap.M.y; Mz += flap.M.z
  }
  // rarefied flow: Cp,max → 2 (free molecular, diffuse/specular blend), friction rises
  const fm = smoothstep(0.1, 10, inp.knudsen)
  if (wN > 0) {
    const cpMax = inp.cpMax * (1 - fm) + 2 * fm
    const Re = (rho * V * m.lref) / inp.mu
    const cf = skinFriction(Re, mach) * (1 - fm) + 0.2 * fm
    const cpLee = clamp(-0.5 / (mach * mach), -0.3, 0)
    let fx = 0, fy = 0, fz = 0, mx = 0, my = 0, mz = 0
    for (const p of m.panels) {
      const rx = p.c.x - xcg, ry = p.c.y, rz = p.c.z
      // local velocity v + ω × r
      const vx = vAir.x + w.y * rz - w.z * ry
      const vy = vAir.y + w.z * rx - w.x * rz
      const vz = vAir.z + w.x * ry - w.y * rx
      const Vp = Math.hypot(vx, vy, vz)
      const s = (vx * p.n.x + vy * p.n.y + vz * p.n.z) / Vp
      const qp = 0.5 * rho * Vp * Vp
      let cp: number
      if (s > 0) cp = cpMax * s * s
      else cp = p.station < 0 && inp.enginesOn ? 0 : cpLee
      const k = -cp * qp * p.area
      let dfx = k * p.n.x, dfy = k * p.n.y, dfz = k * p.n.z
      if (p.station >= 0) {
        // friction opposes the tangential motion
        const vn = s * Vp
        const tx = vx - vn * p.n.x, ty = vy - vn * p.n.y, tz = vz - vn * p.n.z
        const tn = Math.hypot(tx, ty, tz)
        if (tn > 1e-9) {
          const kf = (-cf * qp * p.area) / tn
          dfx += kf * tx
          dfy += kf * ty
          dfz += kf * tz
        }
      }
      fx += dfx; fy += dfy; fz += dfz
      mx += ry * dfz - rz * dfy
      my += rz * dfx - rx * dfz
      mz += rx * dfy - ry * dfx
    }
    Fx += wN * fx; Fy += wN * fy; Fz += wN * fz
    Mx += wN * mx; My += wN * my; Mz += wN * mz
  }
  if (wN < 1) {
    const ws = 1 - wN
    let fy = 0, fz = 0, my = 0, mz = 0
    // potential term: the leading end supplies dS/dξ
    const tailFirst = vAir.x < 0
    for (const s of m.strips) {
      const rx = s.x - xcg
      const vy = vAir.y + w.z * rx
      const vz = vAir.z - w.y * rx
      const ua = vAir.x
      const Vc = Math.hypot(vy, vz)
      if (Vc < 1e-9) continue
      const Vs2 = ua * ua + Vc * Vc
      const as = Math.atan2(Vc, ua)
      const mc = (Vc / inp.a)
      const eta = interp1([0, 0.8, 1.2], [0.68, 0.68, 1.0], mc)
      const cdc = interp1(CDC_M, CDC_V, mc)
      let dN = eta * cdc * 0.5 * rho * Vc * Vc * 2 * s.r * s.dx
      // dS/dξ with ξ measured from the nose tip backward = −dS/dx (x grows toward the tip)
      if (!tailFirst) dN += 0.5 * rho * Vs2 * -s.dSdx * s.dx * Math.sin(2 * as) * Math.cos(as / 2)
      const ux = -vy / Vc, uz = -vz / Vc
      fy += dN * ux
      fz += dN * uz
      my += -rx * dN * uz
      mz += rx * dN * ux
    }
    if (tailFirst) {
      // flat leading face: potential normal force lumped at the base
      const rx = 0 - xcg
      const vy = vAir.y + w.z * rx
      const vz = vAir.z - w.y * rx
      const Vc = Math.hypot(vy, vz)
      if (Vc > 1e-9) {
        const asp = Math.atan2(Vc, -vAir.x)
        const dN = q * m.baseArea * Math.sin(2 * asp) * Math.cos(asp / 2)
        const ux = -vy / Vc, uz = -vz / Vc
        fy += dN * ux
        fz += dN * uz
        my += -rx * dN * uz
        mz += rx * dN * ux
      }
    }
    const ca0 = interp1(CA0_M, CA0_V, mach) - (inp.enginesOn ? interp1(CA0_M, CAB_V, mach) : 0)
    const ca = vAir.x >= 0 ? ca0 : 0.8
    const cosA = vAir.x / V
    const fx = -ca * q * m.sref * cosA * Math.abs(cosA)
    Fx += ws * fx; Fy += ws * fy; Fz += ws * fz
    My += ws * my; Mz += ws * mz
  }
  // light roll damping (skin friction on the barrel)
  Mx += -0.002 * q * m.sref * m.lref * ((w.x * 4.5) / Math.max(V, 1))
  const N = Math.hypot(Fy, Fz)
  return {
    F: v3(Fx, Fy, Fz),
    M: v3(Mx, My, Mz),
    q,
    mach,
    alpha: Math.atan2(Math.hypot(vAir.y, vAir.z), vAir.x),
    CA: -Fx / (q * m.sref),
    CN: N / (q * m.sref),
    // force (0,Fy,Fz) at x_p gives My = -(x_p - xcg)Fz, Mz = (x_p - xcg)Fy
    xcp: N > 1e-6 ? xcg + (Mz * Fy - My * Fz) / (N * N) : xcg,
  }
}

/** static coefficients CL, CD (wind axes) for guidance tables */
export const staticCoeffs = (m: AeroModel, alpha: number, mach: number, xcg: number, cpMax: number): { CL: number; CD: number; Cm: number } => {
  const a = 300
  const V = mach * a
  const rho = 0.01
  const out = aeroWrench(m, {
    vAir: v3(V * Math.cos(alpha), 0, -V * Math.sin(alpha)),
    w: ZERO, rho, a, mu: 1.5e-5, xcg, cpMax, enginesOn: false, knudsen: 0,
  })
  const q = 0.5 * rho * V * V
  // wind axes: drag along -v, lift ⟂ v in the x-z plane (+z body side for +α nose-up → lift toward +z)
  const vx = Math.cos(alpha), vz = -Math.sin(alpha)
  const D = -(out.F.x * vx + out.F.z * vz)
  const L = out.F.x * Math.sin(alpha) + out.F.z * Math.cos(alpha)
  return { CL: L / (q * m.sref), CD: D / (q * m.sref), Cm: out.M.y / (q * m.sref * m.lref) }
}
