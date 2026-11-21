import { clamp, interp1, smoothstep } from "@ship/math"
import { type V3, v3 } from "@ship/math"
import type { VehicleConfig } from "@ship/craft"
import type { StagState } from "@ship/gas"

/**
 * Surface heating on a grid of thermal cells (stations × rays).
 *  - stagnation point: Sutton–Graves convective (k = 1.7415e-4 SI) with hot-wall correction,
 *    Tauber–Sutton radiative (zero below 9 km/s), MHD reduction factor;
 *  - spherical cap: Lees laminar distribution vs angle from the (α-shifted) stagnation point;
 *  - ogive and barrel: Eckert reference-enthalpy flat plate at local edge conditions
 *    (Newtonian pressure, isentropic expansion from the stagnation state = entropy-layer edge),
 *    laminar/turbulent with Re_θ/M_e transition (200–300).
 * Heat is returned as a film coefficient Ch = ρ_e u_e St (kg/m²s) and recovery enthalpy h_r,
 * so the wall model can apply q = Ψ_blow·Ch·(h_r − h_w(T_w)).
 */
export const N_RAYS = 8

export interface ThermalGrid {
  readonly xs: readonly number[] // station x
  readonly nSt: number
  readonly cellX: Float64Array
  readonly cellPhi: Float64Array
  readonly cellNormal: readonly V3[] // outward unit normal (body)
  readonly cellS: Float64Array // running length from the tip along the meridian
  readonly cellCap: Uint8Array // 1 = on the spherical cap
  readonly cellPica: Uint8Array // 1 = PICA-X covered
  readonly cellTile: Uint8Array // 1 = silica tile covered (steel is the backing)
  readonly cellThetaCap: Float64Array // polar angle on the cap sphere
  readonly cellArea: Float64Array
}

export const THERMAL_STATIONS = [52.1, 51.85, 51.3, 50.5, 49.2, 47.4, 45.2, 42.6, 39.8, 37.4, 35.0, 31.0, 25.0, 18.0, 10.0, 3.0]

export const makeThermalGrid = (v: VehicleConfig): ThermalGrid => {
  const s = v.shape
  const xs = THERMAL_STATIONS
  const n = xs.length * N_RAYS
  const cellX = new Float64Array(n)
  const cellPhi = new Float64Array(n)
  const cellS = new Float64Array(n)
  const cellCap = new Uint8Array(n)
  const cellPica = new Uint8Array(n)
  const cellTile = new Uint8Array(n)
  const cellTh = new Float64Array(n)
  const cellArea = new Float64Array(n)
  const normals: V3[] = []
  const Rn = s.p.noseRadius
  const xc = s.p.length - Rn
  // running length along the meridian from the tip
  const sAt = (x: number): number => {
    let acc = 0
    const N = 200
    for (let i = 0; i < N; i++) {
      const a = s.p.length - ((s.p.length - x) * i) / N
      const b = s.p.length - ((s.p.length - x) * (i + 1)) / N
      acc += Math.hypot(a - b, s.r(a) - s.r(b))
    }
    return acc
  }
  const edges = xs.map((x, i) => {
    const prev = i === 0 ? s.p.length : 0.5 * (xs[i - 1] + x)
    const next = i === xs.length - 1 ? 0 : 0.5 * (x + xs[i + 1])
    return [next, prev] as const
  })
  for (let i = 0; i < xs.length; i++) {
    const x = Math.min(xs[i], s.p.length - 1e-3)
    const slope = s.drdx(x)
    const cap = x >= s.xCapEnd
    const th = cap ? Math.acos(clamp((x - xc) / Rn, -1, 1)) : Math.PI / 2
    const [a, b] = edges[i]
    let area = 0
    for (let k = 0; k < 50; k++) {
      const xa = a + ((b - a) * k) / 50
      const xb = a + ((b - a) * (k + 1)) / 50
      area += Math.PI * (s.r(xa) + s.r(xb)) * Math.hypot(xb - xa, s.r(xb) - s.r(xa))
    }
    for (let j = 0; j < N_RAYS; j++) {
      const id = i * N_RAYS + j
      const phi = (j / N_RAYS) * 2 * Math.PI
      const nn = Math.hypot(1, slope)
      cellX[id] = x
      cellPhi[id] = phi
      cellS[id] = sAt(x)
      cellCap[id] = cap ? 1 : 0
      cellPica[id] = x >= v.pica.x0 ? 1 : 0
      cellTile[id] = x >= v.pica.x0 ? 0 : 1 // barrel behind the PICA-X nose is tiled (Q23)
      cellTh[id] = th
      cellArea[id] = area / N_RAYS
      normals.push(i === 0 ? v3(1, 0, 0) : v3(-slope / nn, Math.cos(phi) / nn, Math.sin(phi) / nn))
    }
  }
  return { xs, nSt: xs.length, cellX, cellPhi, cellNormal: normals, cellS, cellCap, cellPica, cellTile, cellThetaCap: cellTh, cellArea }
}

const SG_K = 1.7415e-4
const PR = 0.71

/** Tauber–Sutton f(V) for Earth (W/cm², V km/s) */
const TS_V = [8.5, 9.0, 9.25, 9.5, 9.75, 10.0, 10.25, 10.5, 11.0, 12.0]
const TS_F = [0, 1.5, 4.3, 9.7, 19.5, 35, 55, 81, 151, 359]

export const radiativeStag = (rho: number, V: number, Rn: number): number => {
  const vk = V / 1000
  if (vk < 8.5 || rho <= 0) return 0
  let a = 1.072e6 * V ** -1.88 * rho ** -0.325
  a = Math.min(a, Rn <= 1 ? 1 : Rn <= 2 ? 0.6 : 0.5)
  return 4.736e4 * Rn ** a * rho ** 1.22 * interp1(TS_V, TS_F, vk) * 1e4
}

export interface HeatInput {
  readonly rhoInf: number
  readonly pInf: number
  readonly U: number
  readonly vhat: V3 // body-frame direction of motion through the air
  readonly stag: StagState
  readonly Rn: number
  readonly mhdHeatFactor: number // q/q0 at the stagnation point (≤ 1)
  readonly qInf: number
}

export interface HeatOut {
  readonly ch: Float64Array // film coefficient kg/m²s (cold-wall basis)
  readonly hr: Float64Array // recovery enthalpy J/kg
  readonly qRad: Float64Array // W/m²
  readonly pe: Float64Array
  readonly qStagCold: number // W/m² (300 K wall), after MHD
  readonly qStagRad: number
  readonly turbulentFrac: number
}

const sutherland = (T: number): number => {
  const Tc = Math.min(T, 3000)
  const mu = (1.458e-6 * Tc ** 1.5) / (Tc + 110.4)
  return T > 3000 ? mu * (T / 3000) ** 0.7 : mu
}

export const surfaceHeating = (g: ThermalGrid, inp: HeatInput): HeatOut => {
  const n = g.cellX.length
  const ch = new Float64Array(n)
  const hr = new Float64Array(n)
  const qRad = new Float64Array(n)
  const pe = new Float64Array(n)
  const st = inp.stag
  if (inp.rhoInf < 1e-12 || inp.U < 50) return { ch, hr, qRad, pe, qStagCold: 0, qStagRad: 0, turbulentFrac: 0 }
  const h0 = Math.max(st.h0, 3e5)
  const hw0 = 3.0e5
  const qsg = SG_K * Math.sqrt(inp.rhoInf / inp.Rn) * inp.U ** 3 * inp.mhdHeatFactor
  const chStag = qsg / Math.max(h0 - hw0, 1e4)
  const qr = radiativeStag(inp.rhoInf, inp.U, inp.Rn)
  const ge = clamp(st.gammaE, 1.08, 1.4)
  const ex = (ge - 1) / ge
  const cpE = (ge / (ge - 1)) * (st.p0 / (st.rho0 * st.T0))
  let turb = 0
  for (let i = 0; i < n; i++) {
    const nrm = g.cellNormal[i]
    const sd = nrm.x * inp.vhat.x + nrm.y * inp.vhat.y + nrm.z * inp.vhat.z // sin δ (windward > 0)
    if (g.cellCap[i]) {
      // angle from the stagnation point (which sits where the cap normal = vhat)
      const thp = Math.acos(clamp(sd, -1, 1))
      const lees = thp < Math.PI / 2 ? 0.1 + 0.9 * Math.cos(thp) ** 1.6 : 0.1 * Math.max(0, Math.cos((thp - Math.PI / 2) * 1.5))
      const mhdBoost = 1 - (1 - inp.mhdHeatFactor) * Math.exp(-((thp / 0.8) ** 2))
      ch[i] = (chStag * lees * mhdBoost) / inp.mhdHeatFactor
      hr[i] = h0
      qRad[i] = qr * Math.max(0, Math.cos(thp))
      pe[i] = st.p0 * Math.max(sd, 0) ** 2 + inp.pInf
      continue
    }
    const p = sd > 0 ? inp.pInf + inp.qInf * st.cpMax * sd * sd : inp.pInf * (1 + 0.3 * sd)
    const pr = clamp(p / st.p0, 1e-7, 1)
    const Te = st.T0 * pr ** ex
    const rhoE = st.rho0 * pr ** (1 / ge)
    const he = cpE * Te
    const ue = Math.sqrt(Math.max(2 * (h0 - he), 1))
    const ae = Math.sqrt((ge * p) / rhoE)
    const Me = ue / ae
    const s = Math.max(g.cellS[i], 0.5)
    const muE = sutherland(Te)
    const ReE = (rhoE * ue * s) / muE
    const reTheta = 0.664 * Math.sqrt(ReE)
    const tw = smoothstep(200, 300, reTheta / Math.max(Me, 0.3))
    const rLam = Math.sqrt(PR)
    const rTur = Math.cbrt(PR)
    const r = rLam * (1 - tw) + rTur * tw
    const hrec = he + 0.5 * r * ue * ue
    const hstar = 0.28 * he + 0.5 * hw0 + 0.22 * hrec
    const Tstar = Te * (hstar / he)
    const rhoS = (rhoE * Te) / Tstar
    const muS = sutherland(Tstar)
    const ReS = Math.max((rhoS * ue * s) / muS, 10)
    const stLam = 0.332 * PR ** (-2 / 3) * ReS ** -0.5
    const stTur = 0.0296 * PR ** (-2 / 3) * ReS ** -0.2
    ch[i] = (stLam * (1 - tw) + stTur * tw) * rhoS * ue
    hr[i] = hrec
    pe[i] = p
    turb += tw
  }
  return { ch, hr, qRad, pe, qStagCold: qsg, qStagRad: qr, turbulentFrac: turb / n }
}
