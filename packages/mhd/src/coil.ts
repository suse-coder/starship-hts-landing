import { clamp } from "@ship/math"
import { SIGMA_SB } from "@ship/earth"
import type { CoilDesign } from "@ship/craft"

/**
 * Electrical + cryogenic state of the HTS magnet.
 *  ramp:        V = min(V_max, P_max/I), dI/dt = V/L (bidirectional converter, battery-fed)
 *  persistent:  I constant (joint resistance neglected over one hour)
 *  rampdown:    energy returned to the battery at P_max (η 0.9)
 *  dump:        controlled fast discharge: persistent switch opened, I = I₀ e^{−t R_d/L}; ALL of ½LI²
 *               goes into the dump resistor (winding stays superconducting: T_cs rises as I falls);
 *               only the hysteretic AC loss of the tape reaches the cold mass
 *  discharge:   planned switch-off (Q31): same as dump but on the low-ohm tap R_dis = V_ins/I_op, so
 *               the coil voltage never exceeds the insulation rating; τ = L/R_dis ≈ 65 s
 *  quench:      if T_cold > T_cs(I) → normal zone + dump, τ = L/R; 30 % of the energy heats the cold
 *               mass, 70 % the dump resistor
 *  thermal:     cold-mass enthalpy H(T) of the winding pack (Cu, Hastelloy, Ag, CFRP, SN2 buffer):
 *               dH = (Q_static + Q_rad(T_warm) + Q_AC − Q_cryo(T)) dt (+ quench share), T = H⁻¹
 *               Q_rad through MLI: ε_eff = 0.003 over the cryostat area; cryocooler lift ∝ T
 *  dump resistor: lumped AISI 304 plates, own enthalpy H_d(T), no cooling over the mission (conservative)
 *  bus:         battery energy; MHD generator power charges it (η 0.9), driven electrodes drain it
 */
export type CoilMode = "off" | "ramp" | "persistent" | "rampdown" | "quench" | "dump" | "discharge"
export interface CoilState {
  readonly mode: CoilMode
  readonly I: number
  readonly Tc: number // cold-mass temperature, K
  readonly battery: number // J
  readonly dumped: number // J dissipated in the dump resistor
  readonly heatLeak: number // W (diagnostic)
  readonly busPower: number // W, + = charging the battery
  readonly tDump: number // K, dump-resistor temperature
  readonly vDump: number // V, present voltage across the dump resistor (= coil terminal voltage)
  readonly vDumpMax: number // V, peak dump voltage so far (judge against CoilDesign.vInsulation)
}

export const initCoil = (c: CoilDesign): CoilState => ({
  mode: "off", I: 0, Tc: c.tOp, battery: c.batteryEnergy, dumped: 0, heatLeak: 0, busPower: 0,
  tDump: c.dumpT0, vDump: 0, vDumpMax: 0,
})

const P_MAX = 450e3 // W converter rating
const V_MAX = 1200 // V terminal voltage limit (quench-detection safe)
const Q_RAMP = 40 // W AC loss in the tape while ramping at the voltage limit
/** hysteretic AC loss per ampere of current change, J/A (Q_RAMP at dI/dt = V_MAX/L) */
const acLossPerAmp = (c: CoilDesign): number => Q_RAMP / (V_MAX / c.L)

// ---------------------------------------------------------------------------------------------
// Material specific heats, J/(kg K)
// NIST TRC cryogenic material properties (trc.nist.gov/cryogenics/materials), 4–300 K fits
// log10 c_p = Σ a_i (log10 T)^i; checked: Cu 389 J/kg K @300 K, 196 @77 K; SS304 469 @300 K;
// G-10 999 @300 K. Above 300 K: Incropera & DeWitt Table A.1 (Cu 397 @400 K, 417 @600 K;
// AISI 304: 515 @400, 557 @600, 582 @800, 611 @1000, 640 @1200 K).
const nistFit = (a: readonly number[]) => (T: number): number => {
  const x = Math.log10(T)
  let s = 0, p = 1
  for (const c of a) {
    s += c * p
    p *= x
  }
  return 10 ** s
}
const CU_FIT = nistFit([-1.91844, -0.15973, 8.61013, -18.996, 21.9661, -12.7328, 3.54322, -0.3797])
const SS304_FIT = nistFit([22.0061, -127.5528, 303.647, -381.0098, 274.0328, -112.9212, 24.7593, -2.239153])
const G10_FIT = nistFit([-2.4083, 7.6006, -8.2982, 7.3301, -4.2386, 1.4294, -0.24396, 0.015236])
const lerpTable = (xs: readonly number[], ys: readonly number[], x: number): number => {
  if (x <= xs[0]) return ys[0]
  for (let i = 1; i < xs.length; i++) if (x <= xs[i]) return ys[i - 1] + ((ys[i] - ys[i - 1]) * (x - xs[i - 1])) / (xs[i] - xs[i - 1])
  return ys[ys.length - 1]
}
export const cpCopper = (T: number): number => (T <= 300 ? CU_FIT(Math.max(T, 4)) : lerpTable([300, 400, 600, 1000], [389.4, 397, 417, 451], T))
export const cpSS304 = (T: number): number =>
  T <= 300 ? SS304_FIT(Math.max(T, 4)) : lerpTable([300, 400, 600, 800, 1000, 1200, 1700], [469.5, 515, 557, 582, 611, 640, 700], T)
/** CFRP hoop structure: NIST G-10CR (glass/epoxy) as the epoxy-dominated composite proxy; > 300 K +2.5 J/kg K² */
export const cpComposite = (T: number): number => (T <= 300 ? G10_FIT(Math.max(T, 4)) : 998.7 + 2.5 * (T - 300))
const R_GAS = 8.314462618
/** Debye lattice + Sommerfeld electronic term, J/(kg K): θ_D, γ in J/(mol K²), molar mass kg/mol */
const debyeCp = (theta: number, gamma: number, M: number) => (T: number): number => {
  const xD = theta / T
  // C_v = 9R (T/θ)³ ∫₀^{xD} x⁴eˣ/(eˣ−1)² dx  (Simpson, 64 intervals)
  const n = 64, h = xD / n
  let s = 0
  for (let i = 0; i <= n; i++) {
    const x = i * h
    const f = x < 1e-6 ? x * x : (x ** 4 * Math.exp(x)) / Math.expm1(x) ** 2
    s += f * (i === 0 || i === n ? 1 : i % 2 ? 4 : 2)
  }
  const cv = 9 * R_GAS * (s * h / 3) / xD ** 3
  return (cv + gamma * T) / M
}
/** Hastelloy C-276 (Ni57 Mo16 Cr15.5 Fe5 W4: M̄ = 64.6 g/mol, θ_D ≈ 400 K, γ ≈ 7 mJ/mol K² as Ni):
 *  386 J/kg K at 300 K vs 427 (Haynes datasheet): ≈ 10 % low, i.e. conservative for the quench rise */
export const cpHastelloy = debyeCp(400, 7.0e-3, 0.0646)
/** silver: θ_D 225 K, γ 0.65 mJ/mol K², M 107.87 g/mol (Kittel, Introduction to Solid State Physics) */
export const cpSilver = debyeCp(225, 0.65e-3, 0.10787)
/** solid/liquid N2 buffer (Giauque & Clayton 1933; Scott, Cryogenic Engineering), J/(kg K), M 28.013 g/mol;
 *  α→β 35.61 K (8.17 kJ/kg), melt 63.15 K (25.7 kJ/kg), boil 77.36 K at 1 bar (199 kJ/kg, vented: no
 *  sensible heat afterwards) */
const N2_T = [4, 10, 15, 20, 25, 30, 35.6, 36, 50, 63.15, 63.2, 77.36]
const N2_CP = [0.4, 5.0, 11.0, 17.0, 23.0, 29.5, 37.0, 38.5, 42.0, 46.0, 56.0, 56.0].map((c) => c / 0.028013)
const N2_LATENT: readonly (readonly [number, number])[] = [[35.61, 8.17e3], [63.15, 25.7e3], [77.36, 199e3]]
const cpN2 = (T: number): number => (T > 77.36 ? 0 : lerpTable(N2_T, N2_CP, T))

// REBCO tape (SuperPower SCS4050-class: 50 µm Hastelloy C-276, 2×20 µm Cu, 2×2 µm Ag, 1.5 µm REBCO):
// mass fractions Hastelloy (+REBCO) 0.531, Cu 0.419, Ag 0.049 (densities 8890 / 8960 / 10490 kg/m³)
const TAPE_FR = { hastelloy: 0.531, copper: 0.419, silver: 0.049 } as const
/** solid-N2 thermal buffer in the cold mass, kg (craft designCoil: 60 kg SN2) */
export const SN2_BUFFER = 60

// enthalpy tables H(T) on a uniform grid (trapezoidal integration of c_p, latent heats as steps
// spread over one grid cell), linear between nodes → exact inverse by bisection
interface HTable { readonly t0: number; readonly dT: number; readonly H: Float64Array }
const buildTable = (t0: number, t1: number, dT: number, cpOf: (T: number) => number, latent: readonly (readonly [number, number])[] = []): HTable => {
  const n = Math.round((t1 - t0) / dT) + 1
  const H = new Float64Array(n)
  for (let i = 1; i < n; i++) {
    const Ta = t0 + (i - 1) * dT, Tb = Ta + dT
    let dh = 0.5 * (cpOf(Ta) + cpOf(Tb)) * dT
    for (const [Tl, L] of latent) if (Tl > Ta && Tl <= Tb) dh += L
    H[i] = H[i - 1] + dh
  }
  return { t0, dT, H }
}
const hAt = (tb: HTable, T: number): number => {
  const n = tb.H.length
  const u = (T - tb.t0) / tb.dT
  if (u <= 0) return tb.H[0] + u * (tb.H[1] - tb.H[0])
  if (u >= n - 1) return tb.H[n - 1] + (u - (n - 1)) * (tb.H[n - 1] - tb.H[n - 2])
  const i = Math.floor(u)
  return tb.H[i] + (u - i) * (tb.H[i + 1] - tb.H[i])
}
const tAt = (tb: HTable, h: number): number => {
  const H = tb.H, n = H.length
  if (h <= H[0]) return tb.t0 + ((h - H[0]) / (H[1] - H[0])) * tb.dT
  if (h >= H[n - 1]) return tb.t0 + (n - 1) * tb.dT + ((h - H[n - 1]) / (H[n - 1] - H[n - 2])) * tb.dT
  let lo = 0, hi = n - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (H[mid] <= h) lo = mid
    else hi = mid
  }
  return tb.t0 + (lo + (h - H[lo]) / (H[hi] - H[lo])) * tb.dT
}

const coldTables = new WeakMap<CoilDesign, HTable>()
/** cold-mass enthalpy table (J, relative to 4 K) for this coil design (memoised per design object) */
const coldTable = (c: CoilDesign): HTable => {
  let tb = coldTables.get(c)
  if (!tb) {
    const mH = c.massTape * TAPE_FR.hastelloy, mCu = c.massTape * TAPE_FR.copper, mAg = c.massTape * TAPE_FR.silver
    const cp = (T: number): number =>
      mH * cpHastelloy(T) + mCu * cpCopper(T) + mAg * cpSilver(T) + c.massStructure * cpComposite(T) + SN2_BUFFER * cpN2(T)
    tb = buildTable(4, 800, 0.25, cp, N2_LATENT.map(([T, L]) => [T, L * SN2_BUFFER] as const))
    coldTables.set(c, tb)
  }
  return tb
}
const dumpTables = new WeakMap<CoilDesign, HTable>()
const dumpTable = (c: CoilDesign): HTable => {
  let tb = dumpTables.get(c)
  if (!tb) {
    tb = buildTable(4, 1700, 0.5, (T) => c.dumpMass * cpSS304(T))
    dumpTables.set(c, tb)
  }
  return tb
}
/** cold-mass enthalpy H(T) − H(4 K), J */
export const coldEnthalpy = (c: CoilDesign, T: number): number => hAt(coldTable(c), T)
/** cold-mass temperature for an enthalpy (inverse of coldEnthalpy), K */
export const coldTemperature = (c: CoilDesign, H: number): number => tAt(coldTable(c), H)
/** cold-mass heat capacity dH/dT at T, J/K */
export const coldHeatCapacity = (c: CoilDesign, T: number): number => (coldEnthalpy(c, T + 0.125) - coldEnthalpy(c, T - 0.125)) / 0.25
/** dump-resistor enthalpy H(T) − H(4 K), J */
export const dumpEnthalpy = (c: CoilDesign, T: number): number => hAt(dumpTable(c), T)
export const dumpTemperature = (c: CoilDesign, H: number): number => tAt(dumpTable(c), H)

/**
 * REBCO current-sharing temperature: I_c(T) = I_c(T_op)·((1 − T/T*)/(1 − T_op/T*))^1.3, T* ≈ 70 K at
 * ~10 T, so I = I_c(T_cs) gives T_cs = T* − (T* − T_op)·(I/I_c(T_op))^{1/1.3}. The design margin
 * I_op/I_c(T_op) (0.55) is recovered from the craft design value tCs so both use one number.
 */
const T_STAR = 70
export const currentSharingT = (c: CoilDesign, I: number): number => {
  const fracOp = ((T_STAR - c.tCs) / (T_STAR - c.tOp)) ** 1.3 // I_op / I_c(T_op)
  return T_STAR - (T_STAR - c.tOp) * ((fracOp * Math.max(I, 1)) / c.Iop) ** (1 / 1.3)
}

export interface CoilInput {
  readonly command: "hold" | "ramp" | "rampdown" | "dump" | "discharge"
  readonly warmWallT: number // K, structure temperature around the cryostat
  readonly mhdPower: number // W from the electrodes (+ generator, − driven)
  readonly avionicsLoad: number // W (cryocooler compressors + converters)
}

export const coilStep = (c: CoilDesign, s: CoilState, inp: CoilInput, dt: number): CoilState => {
  let { mode, I, battery, dumped } = s
  let hCold = coldEnthalpy(c, s.Tc)
  let hDump = dumpEnthalpy(c, s.tDump)
  let vDump = 0
  // electrical
  let pCoil = 0
  if (mode !== "quench" && mode !== "dump" && mode !== "discharge" && inp.command === "dump" && I > 0) mode = "dump"
  if (mode !== "quench" && mode !== "dump" && mode !== "discharge" && inp.command === "discharge" && I > 0) mode = "discharge"
  if (mode === "quench" || mode === "dump" || mode === "discharge") {
    // exact exponential decay through the dump resistor (quench/dump tap or the low-ohm discharge tap)
    const R = mode === "discharge" ? c.rDischarge : c.rDump
    vDump = I * R
    const Ie = I * Math.exp(-(R / c.L) * dt)
    const I1 = Ie < 1 ? 0 : Ie // switch fully open below 1 A: the residue (< ½L·1 A²) goes to the resistor too
    const de = 0.5 * c.L * (I * I - I1 * I1)
    if (mode === "quench") {
      dumped += 0.7 * de
      hDump += 0.7 * de
      hCold += 0.3 * de
    } else {
      dumped += de
      hDump += de
      hCold += acLossPerAmp(c) * (I - I1) // hysteretic tape loss only (no normal zone)
    }
    I = I1
    if (I === 0) mode = "off"
  } else if (inp.command === "ramp" && I < c.Iop && battery > 0) {
    mode = "ramp"
    const V = Math.min(V_MAX, P_MAX / Math.max(I, 1))
    I = Math.min(c.Iop, I + (V / c.L) * dt)
    pCoil = -V * I
    if (I >= c.Iop) mode = "persistent"
  } else if (inp.command === "rampdown" && I > 0) {
    mode = "rampdown"
    const V = Math.min(V_MAX, P_MAX / Math.max(I, 1))
    I = Math.max(0, I - (V / c.L) * dt)
    pCoil = 0.9 * V * I
    if (I <= 0) mode = "off"
  } else if (mode === "ramp" || mode === "rampdown") {
    mode = I > 0 ? "persistent" : "off"
  }
  let Tc = coldTemperature(c, hCold)
  // cryogenics
  const area = 2 * Math.PI * c.a * 2 * (c.wpAxial + c.wpRadial + 0.3)
  const qRad = 0.003 * SIGMA_SB * area * (inp.warmWallT ** 4 - Tc ** 4)
  const qStatic = 25
  const qRamp = mode === "ramp" || mode === "rampdown" ? Q_RAMP : 0 // AC loss in the tape while ramping
  const leak = qRad + qStatic + qRamp
  const lift = mode === "off" && I <= 0 ? 0 : c.cryoLift * clamp(Tc / c.tOp, 0.6, 2.5)
  const lifted = Tc > c.tOp - 0.5 ? lift : Math.min(lift, leak)
  Tc = Math.max(c.tOp - 1, coldTemperature(c, hCold + (leak - lifted) * dt))
  // current-sharing temperature falls as I/Ic rises: Tcs(I)
  if (mode !== "quench" && mode !== "off" && I > 10 && Tc > currentSharingT(c, I)) mode = "quench"
  const cryoElectric = lifted * 45 // W electrical (COP ≈ 1/45 at 25 K)
  const pBus = pCoil + 0.9 * Math.max(inp.mhdPower, 0) + Math.min(inp.mhdPower, 0) / 0.9 - cryoElectric - inp.avionicsLoad
  battery = clamp(battery + pBus * dt, 0, c.batteryEnergy * 1.05)
  return {
    mode, I, Tc, battery, dumped, heatLeak: leak, busPower: pBus,
    tDump: dumpTemperature(c, hDump), vDump, vDumpMax: Math.max(s.vDumpMax, vDump),
  }
}

export const coilNI = (c: CoilDesign, s: CoilState): number => s.I * c.N
export const coilEnergy = (c: CoilDesign, s: CoilState): number => 0.5 * c.L * s.I * s.I
