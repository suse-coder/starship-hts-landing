import { bracket, clamp } from "@ship/math"
import { type V3, v3, ZERO } from "@ship/math"
import { E_CHARGE, EV, K_B, M_E, MU0 } from "@ship/earth"
import type { VehicleConfig } from "@ship/craft"
import { cesiumIonisation, coulombLog, electronSpeed, Q_CS } from "@ship/gas"
import type { StagState } from "@ship/gas"

/**
 * Magnetohydrodynamic interaction of the nose HTS coil with the shock-layer plasma.
 *
 *  B field:   exact field of circular filaments (complete elliptic integrals, AGM), the winding
 *             pack represented by 5 axial filaments; tabulated once on a meridional grid.
 *  Plasma:    equilibrium stagnation state from the real-gas shock table; along the cap the
 *             layer expands isentropically (γ_e), n_e follows a Saha scaling with E_a = I_NO/2,
 *             optional Cs seeding (Saha, I = 3.894 eV) in the inner half of the layer.
 *  Ohm:       j = σ_eff (1 − K_s)(u × B),  σ_eff = σ / (1 + κ β²)  (Hall parameter β = ω_ce/ν,
 *             κ = fraction of Hall current that closes inside the layer; κ = 0 is ideal blocking).
 *  Control:   8 azimuthal electrode sectors with loading factors K_s (K = 1 open circuit → no
 *             current, 0 < K < 1 generator, K < 0 driven/accelerator). Per-sector force/moment/
 *             power "basis" is computed once per call; any loading pattern is then a linear map.
 *  Reaction:  the vehicle receives −∫ j × B dV (Newton 3 via the magnet) and −∫ r × (j × B) dV.
 *  Bound:     the gas cannot be decelerated by more than its momentum flux: the linear result is
 *             scaled by 1/(1 + |F|/F_cap), F_cap = 1.2 q∞ π (R_n + Δ)².
 *  Standoff:  Δ = Δ0 √(1 + Q_eff), Q_eff = |F_MHD|/(q∞ π R_n²); stagnation heating ∝ Δ^-1/2.
 */
export const N_SECT = 8
const N_TH = 10
const N_J = 2
const N_PHI = 16
const TH_MAX = (80 * Math.PI) / 180

// ---------- elliptic integrals and loop field ----------
const ellipKE = (m: number): readonly [number, number] => {
  // m = k², AGM; returns [K(m), E(m)]
  let a = 1
  let b = Math.sqrt(1 - m)
  let c = Math.sqrt(m)
  let sum = 0.5 * c * c
  let p = 0.5
  for (let i = 0; i < 12 && Math.abs(c) > 1e-15; i++) {
    const an = 0.5 * (a + b)
    const bn = Math.sqrt(a * b)
    c = 0.5 * (a - b)
    a = an
    b = bn
    p *= 2
    sum += p * c * c
  }
  const K = Math.PI / (2 * a)
  return [K, K * (1 - sum)]
}

/** field of a filament loop (radius a, at x = xc) per ampere at (x, ρ) → [Bx, Bρ] */
export const loopField = (a: number, xc: number, x: number, rho: number): readonly [number, number] => {
  const z = x - xc
  const r = Math.max(rho, 1e-9)
  const d1 = (a + r) ** 2 + z * z
  const d2 = (a - r) ** 2 + z * z
  const m = (4 * a * r) / d1
  const [K, E] = ellipKE(Math.min(m, 1 - 1e-12))
  const k0 = MU0 / (2 * Math.PI * Math.sqrt(d1))
  const Bx = k0 * (K + ((a * a - r * r - z * z) / d2) * E)
  const Br = rho < 1e-6 ? 0 : ((k0 * z) / r) * (-K + ((a * a + r * r + z * z) / d2) * E)
  return [Bx, Br]
}

/** field of the winding pack (5 filaments over the axial width) per ampere-turn */
export const coilField = (v: VehicleConfig, x: number, rho: number): readonly [number, number] => {
  const c = v.coil
  let bx = 0
  let br = 0
  for (let k = 0; k < 5; k++) {
    const xf = c.x + ((k - 2) / 4) * c.wpAxial
    const [fx, fr] = loopField(c.a + c.wpRadial / 2, xf, x, rho)
    bx += fx / 5
    br += fr / 5
  }
  return [bx, br]
}

export interface MhdModel {
  readonly Rn: number
  readonly xc: number // cap sphere centre (body x)
  // B per ampere-turn on a meridional grid: polar angle θ (from +x) × height above the cap
  readonly gTh: number[]
  readonly gH: number[]
  readonly gBx: Float64Array
  readonly gBr: Float64Array
  readonly antennaB: number // T per A·t at the nosecone Starlink antennas
}

export const makeMhdModel = (v: VehicleConfig): MhdModel => {
  const Rn = v.shape.p.noseRadius
  const xc = v.shape.p.length - Rn
  const gTh = Array.from({ length: 33 }, (_, i) => (i / 32) * (TH_MAX + 0.15))
  const gH = Array.from({ length: 12 }, (_, i) => (i / 11) ** 1.5 * 2.5)
  const gBx = new Float64Array(gTh.length * gH.length)
  const gBr = new Float64Array(gTh.length * gH.length)
  gTh.forEach((th, i) =>
    gH.forEach((h, j) => {
      const R = Rn + h
      const [bx, br] = coilField(v, xc + R * Math.cos(th), R * Math.sin(th))
      gBx[i * gH.length + j] = bx
      gBr[i * gH.length + j] = br
    }),
  )
  const [abx, abr] = coilField(v, 47.0, 2.6)
  return { Rn, xc, gTh, gH, gBx, gBr, antennaB: Math.hypot(abx, abr) }
}

const interpB = (m: MhdModel, th: number, h: number): readonly [number, number] => {
  const [i, fi] = bracket(m.gTh, th)
  const [j, fj] = bracket(m.gH, h)
  const n = m.gH.length
  const g = (arr: Float64Array): number =>
    (arr[i * n + j] * (1 - fj) + arr[i * n + j + 1] * fj) * (1 - fi) + (arr[(i + 1) * n + j] * (1 - fj) + arr[(i + 1) * n + j + 1] * fj) * fi
  return [g(m.gBx), g(m.gBr)]
}

export interface MhdInput {
  readonly NI: number // present ampere-turns
  readonly vhat: V3 // body-frame direction of motion through the air
  readonly U: number
  readonly rhoInf: number
  readonly pInf: number
  readonly qInf: number
  readonly stag: StagState
  readonly xcg: number
  readonly kappa: number
  readonly seedFraction: number // 0 = no seeding
}

/** Per-sector basis (K_s = 0) plus global diagnostics. */
export interface MhdBasis {
  readonly F: readonly V3[] // reaction force on the vehicle, per sector, body frame
  readonly M: readonly V3[] // reaction moment about the CG
  readonly P: readonly number[] // ∫ σ_eff |u×B|² dV per sector (W at K(1−K) = 1)
  readonly fCap: number
  readonly delta0: number
  readonly sigmaStag: number // scalar σ at the stagnation point (incl. seed)
  readonly sigmaEffStag: number
  readonly hallStag: number
  readonly neStag: number
  readonly bStag: number
  readonly qInteraction: number // Stuart number σ_eff B² R_n /(ρ∞ U)
  readonly seedMassFlow: number // kg/s of Cs consumed when seeding
}

export const ZERO_BASIS: MhdBasis = {
  F: Array(N_SECT).fill(ZERO), M: Array(N_SECT).fill(ZERO), P: Array(N_SECT).fill(0),
  fCap: 0, delta0: 0, sigmaStag: 0, sigmaEffStag: 0, hallStag: 0, neStag: 0, bStag: 0, qInteraction: 0, seedMassFlow: 0,
}

const EA_NO = (9.264 / 2) * EV

export const mhdBasis = (m: MhdModel, inp: MhdInput): MhdBasis => {
  const st = inp.stag
  if (inp.NI < 1 || inp.qInf < 1 || st.T0 < 1500) return { ...ZERO_BASIS, bStag: Math.abs(inp.NI) * Math.hypot(...interpB(m, 0, 0)) }
  const Rn = m.Rn
  const delta0 = 0.78 * st.eps * Rn
  // stagnation-point conductivity first → Stuart number → standoff growth Δ/Δ0 = (1+Q)^¼
  const [b0xs, b0rs] = interpB(m, 0, 0)
  const bS = Math.abs(inp.NI) * Math.hypot(b0xs, b0rs)
  const neS = st.ne0 + (inp.seedFraction > 0 ? cesiumIonisation(st.T0, inp.seedFraction * st.n0, st.ne0) : 0)
  const nuS = st.nuEn0 + (inp.seedFraction > 0 ? inp.seedFraction * st.n0 * Q_CS * electronSpeed(st.T0) : 0) +
    1.47e-12 * neS * coulombLog(neS, st.T0) * ((K_B * st.T0) / EV) ** -1.5 + 1
  const sigS = (neS * E_CHARGE * E_CHARGE) / (M_E * nuS)
  const betaS = (E_CHARGE * bS) / (M_E * nuS)
  const sEffS = sigS / (1 + inp.kappa * betaS * betaS)
  const qStuart = (sEffS * bS * bS * Rn) / Math.max(inp.rhoInf * inp.U, 1e-12)
  const delta = Math.min(delta0 * (1 + qStuart) ** 0.25, 1.2 * Rn, delta0 * 6)
  const ge = clamp(st.gammaE, 1.08, 1.4)
  const ex = (ge - 1) / ge
  const cpE = (ge / (ge - 1)) * (st.p0 / (st.rho0 * st.T0))
  const F: { x: number; y: number; z: number }[] = Array.from({ length: N_SECT }, () => ({ x: 0, y: 0, z: 0 }))
  const M: { x: number; y: number; z: number }[] = Array.from({ length: N_SECT }, () => ({ x: 0, y: 0, z: 0 }))
  const P = new Array<number>(N_SECT).fill(0)
  const dTh = TH_MAX / N_TH
  const dPhi = (2 * Math.PI) / N_PHI
  let sigStag = 0, sigEffStag = 0, hallStag = 0, neStag = 0
  const vh = inp.vhat
  for (let i = 0; i < N_TH; i++) {
    const th = (i + 0.5) * dTh
    const sth = Math.sin(th), cth = Math.cos(th)
    const deltaTh = delta * (1 + 0.6 * th * th)
    for (let jl = 0; jl < N_J; jl++) {
      const hgt = ((jl + 0.5) / N_J) * deltaTh
      const R = Rn + hgt
      const dV = R * R * sth * dTh * dPhi * (deltaTh / N_J)
      const [bx1, br1] = interpB(m, th, hgt)
      const Bx0 = bx1 * inp.NI
      const Br0 = br1 * inp.NI
      const seeded = inp.seedFraction > 0 && jl === 0
      for (let k = 0; k < N_PHI; k++) {
        const phi = (k + 0.5) * dPhi
        const cph = Math.cos(phi), sph = Math.sin(phi)
        // outward direction of the cell from the cap centre, and its position
        const dx = cth, dy = sth * cph, dz = sth * sph
        const px = m.xc + R * dx, py = R * dy, pz = R * dz
        // local plasma state from the α-shifted stagnation point
        const cosp = clamp(dx * vh.x + dy * vh.y + dz * vh.z, -1, 1)
        if (cosp <= 0.05) continue
        const pRatio = clamp(cosp * cosp + (inp.pInf / st.p0) * (1 - cosp * cosp), 1e-6, 1)
        const T = st.T0 * pRatio ** ex
        const nTot = st.n0 * pRatio ** (1 / ge)
        const ue = Math.sqrt(Math.max(0, 2 * cpE * st.T0 * (1 - pRatio ** ex)))
        // tangential direction away from the stagnation point
        let tx = dx * cosp - vh.x, ty = dy * cosp - vh.y, tz = dz * cosp - vh.z
        const tn = Math.hypot(tx, ty, tz)
        if (tn < 1e-9) { tx = 0; ty = 0; tz = 0 } else { tx /= tn; ty /= tn; tz /= tn }
        const ux = ue * tx, uy = ue * ty, uz = ue * tz
        // electron density (Saha scaling from the stagnation value) and collisions
        let ne = st.ne0 * Math.sqrt(nTot / st.n0) * Math.exp((-EA_NO / K_B) * (1 / T - 1 / st.T0))
        let nu = st.nuEn0 * (nTot / st.n0) * Math.sqrt(T / st.T0)
        if (seeded) {
          const nCs = inp.seedFraction * nTot
          ne += cesiumIonisation(T, nCs, ne)
          nu += nCs * Q_CS * electronSpeed(T)
        }
        const TeV = (K_B * T) / EV
        nu += 1.47e-12 * ne * coulombLog(ne, T) * TeV ** -1.5 + 1
        const sigma = (ne * E_CHARGE * E_CHARGE) / (M_E * nu)
        // field in body axes (axisymmetric coil): B = Bx x̂ + Bρ ρ̂
        const Bx = Bx0, By = Br0 * cph, Bz = Br0 * sph
        const B = Math.hypot(Bx, Br0)
        const beta = (E_CHARGE * B) / (M_E * nu)
        const sEff = sigma / (1 + inp.kappa * beta * beta)
        if (i === 0 && jl === 0 && k === 0) {
          sigStag = sigma
          sigEffStag = sEff
          hallStag = beta
          neStag = ne
        }
        // u × B
        const ex_ = uy * Bz - uz * By, ey = uz * Bx - ux * Bz, ez = ux * By - uy * Bx
        // j = σ_eff (u×B); f_gas = j × B
        const jx = sEff * ex_, jy = sEff * ey, jz = sEff * ez
        const fx = (jy * Bz - jz * By) * dV
        const fy = (jz * Bx - jx * Bz) * dV
        const fz = (jx * By - jy * Bx) * dV
        const s = Math.min(N_SECT - 1, Math.floor((phi / (2 * Math.PI)) * N_SECT))
        // reaction on the vehicle
        F[s].x -= fx; F[s].y -= fy; F[s].z -= fz
        const rx = px - inp.xcg
        M[s].x -= py * fz - pz * fy
        M[s].y -= pz * fx - rx * fz
        M[s].z -= rx * fy - py * fx
        P[s] += sEff * (ex_ * ex_ + ey * ey + ez * ez) * dV
      }
    }
  }
  const fCap = 1.2 * inp.qInf * Math.PI * (Rn + delta) ** 2
  const seedArea = Math.PI * (0.6 * Rn) ** 2 * 0.3
  void sigStag; void sigEffStag; void hallStag; void neStag
  return {
    F: F.map((f) => v3(f.x, f.y, f.z)),
    M: M.map((f) => v3(f.x, f.y, f.z)),
    P,
    fCap,
    delta0: delta,
    sigmaStag: sigS,
    sigmaEffStag: sEffS,
    hallStag: betaS,
    neStag: neS,
    bStag: bS,
    qInteraction: qStuart,
    seedMassFlow: inp.seedFraction > 0 ? inp.seedFraction * (132.9 / 28.96) * inp.rhoInf * inp.U * seedArea : 0,
  }
}

export interface MhdOut {
  readonly F: V3
  readonly M: V3
  readonly power: number // W extracted (>0) or consumed (<0)
  readonly scale: number // momentum-bound factor
  readonly standoffFactor: number
  readonly heatFactor: number
  readonly forceMag: number
}

export const ZERO_MHD: MhdOut = { F: ZERO, M: ZERO, power: 0, scale: 1, standoffFactor: 1, heatFactor: 1, forceMag: 0 }

/** apply electrode loading u_s = 1 − K_s to a basis */
export const mhdApply = (b: MhdBasis, u: readonly number[], qInf: number, Rn: number): MhdOut => {
  if (b.fCap <= 0) return ZERO_MHD
  let fx = 0, fy = 0, fz = 0, mx = 0, my = 0, mz = 0, p = 0
  for (let s = 0; s < N_SECT; s++) {
    fx += u[s] * b.F[s].x; fy += u[s] * b.F[s].y; fz += u[s] * b.F[s].z
    mx += u[s] * b.M[s].x; my += u[s] * b.M[s].y; mz += u[s] * b.M[s].z
    p += (1 - u[s]) * u[s] * b.P[s]
  }
  const Flin = Math.hypot(fx, fy, fz)
  const sc = 1 / (1 + Flin / b.fCap)
  const Fm = Flin * sc
  const qEff = Fm / Math.max(qInf * Math.PI * Rn * Rn, 1)
  const sf = Math.sqrt(1 + qEff)
  return {
    F: v3(fx * sc, fy * sc, fz * sc),
    M: v3(mx * sc, my * sc, mz * sc),
    power: p * sc,
    scale: sc,
    standoffFactor: sf,
    heatFactor: 1 / Math.sqrt(sf),
    forceMag: Fm,
  }
}

/** Earth-field torque on the coil dipole (body frame): τ = m × B, m = NI·π a² x̂ */
export const dipoleTorque = (NI: number, a: number, bBody: V3): V3 => {
  const mm = NI * Math.PI * a * a
  return v3(0, -mm * bBody.z, mm * bBody.y)
}
