import { clamp, solveTridiag } from "@ship/math"
import { SIGMA_SB } from "@ship/earth"
import { type DesignParams, picaThicknessAt, TILE, type VehicleConfig } from "@ship/craft"
import type { HeatOut, ThermalGrid } from "./heating"

/**
 * Wall response per thermal cell.
 * PICA-X cells: 1-D implicit conduction (graded mesh, 14 nodes) with first-order Arrhenius
 * pyrolysis (virgin 270 → char 220 kg/m³), transpiration of pyrolysis gas, blowing reduction
 * Ψ = φ/(e^φ−1), diffusion-limited carbon oxidation (B'c = 0.17) with surface recession,
 * re-radiation (ε 0.9), and a lumped RTV bond + steel substrate node behind the bondline.
 * Bare-steel cells (30X/304L, 3.97 mm, ε 0.85): lumped (Biot ≪ 1) with re-radiation and
 * interior radiation to 250 K.
 * Properties: Tran et al. NASA TM-110440 (1997) and later PICA material-response models, rounded.
 */
export const NP = 14
const RHO_V = 270
const RHO_C = 220
const A_PYRO = 1.4e4 // 1/s
const E_PYRO = 8556 // K
const H_PYRO = 3.0e5 // J/kg (endothermic)
const CP_GAS = 2500
const EPS_PICA = 0.9
const EPS_STEEL = 0.85
const BPRIME_C = 0.17
const H_OX = 9.2e6 * 0.5 // C → CO, half released at the wall
const STEEL_RHO = 7900
const T_INNER = 250
/**
 * Tile layer (Q23): six nodes through the LI-900 thickness, geometric grading toward the hot face,
 * then the RTV bond and the lumped steel backing. Areal data live in `@ship/craft` TILE.
 */
const NT = 6
const TILE_FR = ((): Float64Array => {
  const r = 1.35
  const w = Array.from({ length: NT }, (_, i) => r ** i)
  const sum = w.reduce((a, b) => a + b, 0)
  return Float64Array.from(w.map((x) => x / sum))
})()
const kTile = (T: number): number => TILE.conductivityAt300 + TILE.conductivitySlope * (T - 300)
const cpTile = (T: number): number => clamp(TILE.cpAt300 + TILE.cpSlope * (T - 300), TILE.cpAt300, TILE.cpMax)
/** tile layer node count and fractions (exposed for the energy-conservation test) */
export const TILE_NODES = NT
export const tileNodeFractions = (): Float64Array => Float64Array.from(TILE_FR)
/** tile material properties used by the wall model (exposed for the test) */
export const tileConductivity = kTile
export const tileHeatCapacity = cpTile

const kV = (T: number): number => 0.2 + 1.0e-4 * (T - 300)
const kC = (T: number): number => clamp(0.4 + 6e-4 * (T - 300), 0.4, 2.2)
const cpV = (T: number): number => clamp(1000 + 1.0 * (T - 300), 900, 1900)
const cpC = (T: number): number => clamp(1100 + 0.6 * (T - 300), 1000, 2000)
export const cpSteel = (T: number): number => clamp(450 + 0.28 * (T - 300), 420, 700)

export interface ThermalState {
  readonly T: Float64Array // [cell*NP + node] for PICA cells; steel cells use node 0
  readonly rho: Float64Array // density per node (PICA)
  readonly Tsub: Float64Array // substrate (bond+steel) per cell
  readonly recession: Float64Array // m
  readonly thick: Float64Array // m, initial PICA thickness per cell
  readonly mdotGas: Float64Array // kg/m²s last step
  readonly qNet: Float64Array // W/m² net into wall last step (diagnostic)
  readonly qConv: Float64Array // W/m² convective (hot wall) last step
}

export const nodeFractions = (): Float64Array => {
  // geometric grading toward the surface: first cell ~1.5 % of the thickness
  const r = 1.25
  const w = Array.from({ length: NP }, (_, i) => r ** i)
  const sum = w.reduce((a, b) => a + b, 0)
  return Float64Array.from(w.map((x) => x / sum))
}
const FR = nodeFractions()

export const initThermal = (g: ThermalGrid, v: VehicleConfig, T0 = 290): ThermalState => {
  const n = g.cellX.length
  const thick = new Float64Array(n)
  for (let i = 0; i < n; i++) thick[i] = g.cellPica[i] ? picaThicknessAt(v.design as DesignParams, v.shape, g.cellX[i], v.pica.x0) : 0
  return {
    T: new Float64Array(n * NP).fill(T0),
    rho: new Float64Array(n * NP).fill(RHO_V),
    Tsub: new Float64Array(n).fill(T0),
    recession: new Float64Array(n),
    thick,
    mdotGas: new Float64Array(n),
    qNet: new Float64Array(n),
    qConv: new Float64Array(n),
  }
}

/** One thermal step of length dt (pure: returns a new state). */
export const thermalStep = (g: ThermalGrid, v: VehicleConfig, s: ThermalState, h: HeatOut, dt: number): ThermalState => {
  const n = g.cellX.length
  const T = new Float64Array(s.T)
  const rho = new Float64Array(s.rho)
  const Tsub = new Float64Array(s.Tsub)
  const rec = new Float64Array(s.recession)
  const mg = new Float64Array(n)
  const qNet = new Float64Array(n)
  const qConv = new Float64Array(n)
  const tSteel = v.steelThickness
  const a = new Float64Array(NP + 1), b = new Float64Array(NP + 1), c = new Float64Array(NP + 1), d = new Float64Array(NP + 1)
  const kN = new Float64Array(NP), gF = new Float64Array(NP - 1)
  const dxT = new Float64Array(NT)
  for (let i = 0; i < n; i++) {
    const ch = h.ch[i]
    const hr = h.hr[i]
    if (g.cellTile[i]) {
      // tiled barrel: 1-D conduction through the tile, RTV bond, lumped steel backing. The backing
      // sees no external radiation (the tile covers it) — only the interior sink, so it is the
      // structural check (1150 K); the tile surface is the TPS check (1530 K).
      const base = i * NP
      const Tb = Tsub[i]
      const Cs = (STEEL_RHO * tSteel * cpSteel(Tb) + TILE.bondArealHeatCapacity) / dt
      for (let k = 0; k < NT; k++) {
        kN[k] = kTile(T[base + k])
        dxT[k] = TILE_FR[k] * TILE.thickness
      }
      for (let k = 0; k < NT - 1; k++) gF[k] = 1 / (dxT[k] / (2 * kN[k]) + dxT[k + 1] / (2 * kN[k + 1]))
      for (let k = 0; k < NT; k++) {
        const Tk = T[base + k]
        const C = (TILE.density * dxT[k] * cpTile(Tk)) / dt
        const gW = k > 0 ? gF[k - 1] : 0
        const gE = k < NT - 1 ? gF[k] : 0
        a[k] = -gW
        c[k] = -gE
        b[k] = C + gW + gE
        d[k] = C * Tk
      }
      const Ts = T[base]
      const qc = ch * (hr - 1000 * Ts) + h.qRad[i]
      qConv[i] = qc
      const e = TILE.emissivity
      const Ts3 = Ts ** 3
      b[0] += 4 * e * SIGMA_SB * Ts3
      d[0] += qc + 3 * e * SIGMA_SB * Ts3 * Ts
      const Gb = 1 / (dxT[NT - 1] / (2 * kN[NT - 1]) + TILE.bondThickness / TILE.bondConductivity)
      b[NT - 1] += Gb
      c[NT - 1] = -Gb
      a[NT] = -Gb
      b[NT] = Gb + Cs + 4 * 0.3 * SIGMA_SB * Tb ** 3
      c[NT] = 0
      d[NT] = Cs * Tb + 3 * 0.3 * SIGMA_SB * Tb ** 3 * Tb + 0.3 * SIGMA_SB * T_INNER ** 4
      const solT = solveTridiag(a.subarray(0, NT + 1), b.subarray(0, NT + 1), c.subarray(0, NT + 1), d.subarray(0, NT + 1))
      for (let k = 0; k < NT; k++) T[base + k] = solT[k]
      Tsub[i] = solT[NT]
      qNet[i] = qc - e * SIGMA_SB * solT[0] ** 4
      continue
    }
    if (!g.cellPica[i]) {
      // lumped steel wall (only cells with no tile and no PICA; none on the default vehicle)
      const Tw = T[i * NP]
      const hw = 1000 * Tw
      const qc = ch * (hr - hw) + h.qRad[i] // negative when the wall is above recovery (cooling)
      const mc = STEEL_RHO * tSteel * cpSteel(Tw)
      const qrad = EPS_STEEL * SIGMA_SB * Tw ** 4
      const qin = 0.3 * SIGMA_SB * (Tw ** 4 - T_INNER ** 4)
      // semi-implicit in the T^4 terms
      const dq = 4 * (EPS_STEEL + 0.3) * SIGMA_SB * Tw ** 3 + ch * 1000
      const Tn = Tw + (dt * (qc - qrad - qin)) / (mc + dt * dq)
      T[i * NP] = Tn
      qNet[i] = qc - qrad - qin
      qConv[i] = qc
      continue
    }
    const L = Math.max(s.thick[i] - rec[i], 0.002)
    const base = i * NP
    // pyrolysis (explicit, first-order Arrhenius), gas flux toward the surface
    let gas = 0
    const dx = new Float64Array(NP)
    for (let k = 0; k < NP; k++) {
      dx[k] = FR[k] * L
      const Tk = T[base + k]
      const r = rho[base + k]
      if (r > RHO_C + 1e-6 && Tk > 450) {
        const rate = A_PYRO * Math.exp(-E_PYRO / Tk)
        const rn = RHO_C + (r - RHO_C) * Math.exp(-rate * dt)
        gas += ((r - rn) / dt) * dx[k]
        rho[base + k] = rn
      }
    }
    mg[i] = gas
    const Ts = T[base]
    // carbon oxidation (diffusion-limited above ~1700 K)
    const fox = clamp((Ts - 1100) / 600, 0, 1)
    const mc = BPRIME_C * ch * fox
    const Bp = (gas + mc) / Math.max(ch, 1e-9)
    const phi = 2 * 0.5 * Bp
    const psi = phi > 1e-6 ? phi / Math.expm1(phi) : 1
    const hw = 1200 * Ts
    const qc = psi * ch * Math.max(hr - hw, 0) + h.qRad[i]
    qConv[i] = qc
    // assemble implicit conduction with linearised radiation and transpiration at the surface.
    // Node conductivities first, then ONE interface conductance per face shared by both rows
    // (series resistance of the two half cells = harmonic mean):
    //   G_{k+½} = 1 / (Δx_k/(2k_k) + Δx_{k+1}/(2k_{k+1}))
    // so the matrix is symmetric and the flux leaving node k is exactly the flux entering k+1
    // (finite-volume conservation, Patankar 1980 §4.2).
    for (let k = 0; k < NP; k++) {
      const beta = clamp((RHO_V - rho[base + k]) / (RHO_V - RHO_C), 0, 1)
      kN[k] = (1 - beta) * kV(T[base + k]) + beta * kC(T[base + k])
    }
    for (let k = 0; k < NP - 1; k++) gF[k] = 1 / (dx[k] / (2 * kN[k]) + dx[k + 1] / (2 * kN[k + 1]))
    for (let k = 0; k < NP; k++) {
      const Tk = T[base + k]
      const beta = clamp((RHO_V - rho[base + k]) / (RHO_V - RHO_C), 0, 1)
      const cp = (1 - beta) * cpV(Tk) + beta * cpC(Tk)
      const C = (rho[base + k] * cp * dx[k]) / dt
      const gW = k > 0 ? gF[k - 1] : 0
      const gE = k < NP - 1 ? gF[k] : 0
      a[k] = -gW
      c[k] = -gE
      b[k] = C + gW + gE
      d[k] = C * Tk
      // pyrolysis heat sink in this node
      const r0 = s.rho[base + k]
      d[k] -= ((r0 - rho[base + k]) * dx[k] * H_PYRO) / dt
    }
    // surface node: q_conv + oxidation − εσT⁴ (linearised) − transpiration
    const Ts3 = Ts ** 3
    b[0] += 4 * EPS_PICA * SIGMA_SB * Ts3 + gas * CP_GAS
    d[0] += qc + mc * H_OX + 3 * EPS_PICA * SIGMA_SB * Ts3 * Ts + gas * CP_GAS * 600
    // back node ↔ substrate (bond + steel) via contact conductance
    const kLast = kV(T[base + NP - 1])
    const Gb = 1 / (dx[NP - 1] / (2 * kLast) + 0.0003 / 0.3)
    const Csub = (STEEL_RHO * tSteel * cpSteel(Tsub[i]) + 1500 * 0.0003 * 1200) / dt
    b[NP - 1] += Gb
    c[NP - 1] = -Gb
    a[NP] = -Gb
    b[NP] = Gb + Csub
    c[NP] = 0
    d[NP] = Csub * Tsub[i]
    const sol = solveTridiag(a, b, c, d)
    for (let k = 0; k < NP; k++) T[base + k] = sol[k]
    Tsub[i] = sol[NP]
    rec[i] = rec[i] + (mc / RHO_C) * dt
    qNet[i] = qc + mc * H_OX - EPS_PICA * SIGMA_SB * sol[0] ** 4
  }
  return { T, rho, Tsub, recession: rec, thick: s.thick, mdotGas: mg, qNet, qConv }
}

/** surface temperature per cell */
export const surfaceTemps = (g: ThermalGrid, s: ThermalState): Float64Array => {
  const n = g.cellX.length
  const out = new Float64Array(n)
  for (let i = 0; i < n; i++) out[i] = s.T[i * NP]
  return out
}

export interface ThermalSummary {
  readonly picaSurfaceMax: number
  readonly bondlineMax: number
  readonly tileSurfaceMax: number
  readonly tileBackMax: number
  readonly steelMax: number
  readonly steelMaxX: number
  readonly recessionMax: number
  readonly coilWallT: number // substrate temperature around the coil (cryostat warm boundary)
}

export const thermalSummary = (g: ThermalGrid, s: ThermalState, coilX: number): ThermalSummary => {
  let ps = 0, bl = 0, ts = 0, tb = 0, st = 0, stx = 0, rc = 0, cw = 0, cwn = 0
  for (let i = 0; i < g.cellX.length; i++) {
    if (g.cellPica[i]) {
      ps = Math.max(ps, s.T[i * NP])
      bl = Math.max(bl, s.Tsub[i])
      rc = Math.max(rc, s.recession[i])
      if (Math.abs(g.cellX[i] - coilX) < 1.0) {
        cw += s.Tsub[i]
        cwn++
      }
    } else if (g.cellTile[i]) {
      ts = Math.max(ts, s.T[i * NP])
      tb = Math.max(tb, s.Tsub[i])
      if (s.Tsub[i] > st) {
        st = s.Tsub[i]
        stx = g.cellX[i]
      }
    } else if (s.T[i * NP] > st) {
      st = s.T[i * NP]
      stx = g.cellX[i]
    }
  }
  return { picaSurfaceMax: ps, bondlineMax: bl, tileSurfaceMax: ts, tileBackMax: tb, steelMax: st, steelMaxX: stx, recessionMax: rc, coilWallT: cwn ? cw / cwn : 290 }
}
