/**
 * spec/05 T-heat — aerothermal heating, wall response and thermal limits.
 * References:
 *  - Sutton & Graves, NASA TR R-376 (1971): q_s = k √(ρ∞/R_n) V³, k = 1.7415e-4 kg^½/m (Earth air).
 *    At 70 km (US-76 ρ = 8.283e-5 kg/m³), 7 km/s, R_n = 2.5 m: q_s = 34.4 W/cm².
 *  - Detra, Kemp & Riddell (1957) correlation q = 1.1035e8/√R_n · √(ρ/ρ_SL) · (V/7924.8)^3.15 W/m²
 *    (17 600 BTU/ft²s form): 38.8 W/cm² at the same point; Sutton–Graves sits ≈ 10 % lower.
 *  - Lees (1956) laminar hemisphere distribution q/q_s ≈ 0.1 + 0.9 cos^1.6 θ (spec/02 §4 form).
 *  - Tauber & Sutton (1991) radiative heating: f(V) = 0 below ≈ 8.5–9 km/s (LEO entry is convective).
 *  - Radiative equilibrium of a thin wall: q_abs = ε σ T⁴ + q_back.
 *  - Semi-infinite solid under a constant surface flux: ΔT_s = 2 q √(t/(π k ρ c)) (Carslaw & Jaeger §2.9);
 *    thermal penetration depth √(α t).
 *  - EN 1993-1-2 Annex C, Table C.1, grade 1.4301: k_0.2p(T) = 1.00 (20 °C), 0.82 (100), 0.68 (200),
 *    0.64 (300), 0.60 (400), 0.54 (500), 0.49 (600), 0.40 (700), 0.27 (800), 0.14 (900), 0.06 (1000),
 *    0.03 (1100), 0.00 (1200 °C).
 */
import { describe, expect, test } from "bun:test"
import { v3, normalize, findRoot } from "@ship/math"
import { atmosphere, SIGMA_SB } from "@ship/earth"
import { lookupStag, SHOCK_TABLE } from "@ship/gas"
import { makeVehicle } from "@ship/craft"
import {
  makeThermalGrid, surfaceHeating, radiativeStag, initThermal, thermalStep, NP, N_RAYS, cpSteel, nodeFractions,
  steelYieldRetention, steelVerdict, THERMAL_LIMITS, type HeatOut, type HeatInput,
} from "../src/index"

const v = makeVehicle()
const g = makeThermalGrid(v)
const n = g.cellX.length
const rel = (a: number, b: number): number => Math.abs(a / b - 1)
/** the same grid with the tile flag cleared: tests the bare-steel fallback branch (no tile, no PICA) */
const gBare = { ...g, cellTile: new Uint8Array(n) }
const firstCellBare = (): number => {
  for (let i = 0; i < n; i++) if (!gBare.cellPica[i] && !gBare.cellTile[i]) return i
  return -1
}

const entryInput = (hkm: number, U: number, alphaDeg: number, heatFactor = 1): HeatInput => {
  const a = atmosphere(hkm * 1000)
  const al = (alphaDeg * Math.PI) / 180
  return {
    rhoInf: a.rho, pInf: a.p, U, vhat: normalize(v3(Math.cos(al), 0, -Math.sin(al))), stag: lookupStag(SHOCK_TABLE, hkm * 1000, U),
    Rn: v.shape.p.noseRadius, mhdHeatFactor: heatFactor, qInf: 0.5 * a.rho * U * U,
  }
}
/** a pure radiative (absorbed) flux on every cell, no convection */
const radiativeLoad = (q: number): HeatOut => ({
  ch: new Float64Array(n), hr: new Float64Array(n), qRad: new Float64Array(n).fill(q), pe: new Float64Array(n), qStagCold: 0, qStagRad: 0, turbulentFrac: 0,
})
const firstCell = (pica: boolean): number => {
  for (let i = 0; i < n; i++) if (Boolean(g.cellPica[i]) === pica) return i
  return -1
}

describe("stagnation-point heating", () => {
  test("Sutton–Graves at 7 km/s, 70 km, R_n 2.5 m = 34.4 W/cm², within 15 % of Detra–Kemp–Riddell", () => {
    const inp = entryInput(70, 7000, 0)
    const h = surfaceHeating(g, inp)
    expect(rel(h.qStagCold, 1.7415e-4 * Math.sqrt(inp.rhoInf / 2.5) * 7000 ** 3)).toBeLessThan(1e-12)
    expect(h.qStagCold / 1e4).toBeCloseTo(34.4, 1)
    const dkr = (1.1035e8 / Math.sqrt(2.5)) * Math.sqrt(inp.rhoInf / 1.225) * (7000 / 7924.8) ** 3.15
    expect(rel(h.qStagCold, dkr)).toBeLessThan(0.15)
  })

  test("film coefficient reproduces q_s at the 300 K reference wall: Ch (h_r − h_w) = q_s at the tip", () => {
    const h = surfaceHeating(g, entryInput(65, 6800, 0))
    expect(rel(h.ch[0] * (h.hr[0] - 3e5), h.qStagCold)).toBeLessThan(1e-9)
  })

  test("Lees distribution on the cap (α = 0): q/q_s = 0.1 + 0.9 cos^1.6 θ, decreasing down the body", () => {
    const h = surfaceHeating(g, entryInput(70, 7000, 0))
    let prev = Infinity
    for (let i = 0; i < g.nSt; i++) {
      const id = i * N_RAYS
      const q = h.ch[id] * (h.hr[id] - 3e5)
      if (g.cellCap[id] && i > 0) expect(q / h.qStagCold).toBeCloseTo(0.1 + 0.9 * Math.cos(g.cellThetaCap[id]) ** 1.6, 6)
      expect(q).toBeLessThan(prev * 1.0000001)
      prev = q
    }
  })

  test("MHD heat factor scales the stagnation heating linearly", () => {
    const h1 = surfaceHeating(g, entryInput(65, 7000, 10, 1))
    const h7 = surfaceHeating(g, entryInput(65, 7000, 10, 0.7))
    expect(h7.qStagCold / h1.qStagCold).toBeCloseTo(0.7, 12)
  })

  test("Tauber–Sutton radiative heating vanishes for LEO entry speeds and grows with V above 9 km/s", () => {
    expect(radiativeStag(3e-4, 7800, 2.5)).toBe(0)
    const q10 = radiativeStag(3e-4, 10000, 2.5), q11 = radiativeStag(3e-4, 11000, 2.5)
    expect(q10).toBeGreaterThan(0)
    expect(q11).toBeGreaterThan(q10)
  })

  test("windward barrel heats more than the leeward barrel at α = 60° (Newtonian edge pressure)", () => {
    const h = surfaceHeating(g, entryInput(55, 4500, 60))
    const st = g.xs.findIndex((x) => x === 25.0)
    const qOf = (j: number): number => h.ch[st * N_RAYS + j] * (h.hr[st * N_RAYS + j] - 3e5)
    // rays at φ = j·45°: normal (cos φ, sin φ) in (y, z); windward is −z (j = 6), leeward +z (j = 2)
    expect(qOf(6)).toBeGreaterThan(3 * qOf(2))
  })
})

describe("wall response", () => {
  test("bare steel reaches radiative equilibrium q = ε σ T⁴ + 0.3 σ (T⁴ − 250⁴) (0.01 K)", () => {
    const i = firstCellBare()
    const q = 5e4
    let s = initThermal(gBare, v, 290)
    for (let k = 0; k < 4000; k++) s = thermalStep(gBare, v, s, radiativeLoad(q), 1)
    const Teq = findRoot((T) => q - 0.85 * SIGMA_SB * T ** 4 - 0.3 * SIGMA_SB * (T ** 4 - 250 ** 4), 200, 3000)
    expect(Math.abs(s.T[i * NP] - Teq)).toBeLessThan(0.01)
  })

  test("bare steel transient: one small step stores ρ t c ΔT = q_net Δt (3.97 mm 30X)", () => {
    const i = firstCellBare()
    const q = 1e5, dt = 0.01, T0 = 290
    const s = thermalStep(gBare, v, initThermal(gBare, v, T0), radiativeLoad(q), dt)
    const qNet = q - 0.85 * SIGMA_SB * T0 ** 4 - 0.3 * SIGMA_SB * (T0 ** 4 - 250 ** 4)
    expect(rel(7900 * v.steelThickness * cpSteel(T0) * (s.T[i * NP] - T0), qNet * dt)).toBeLessThan(1e-3)
  })

  test("PICA-X: bondline untouched by a 10 s pulse (penetration √(αt) ≈ 3 mm ≪ 65 mm)", () => {
    const i = firstCell(true)
    let s = initThermal(g, v, 290)
    for (let k = 0; k < 20; k++) s = thermalStep(g, v, s, radiativeLoad(2e4), 0.5)
    expect(s.T[i * NP]).toBeGreaterThan(400)
    expect(Math.abs(s.Tsub[i] - 290)).toBeLessThan(0.01)
  })

  test("PICA-X surface rise matches the semi-infinite solid 2 q_net √(t/(π k ρ c)) within 15 %", () => {
    const i = firstCell(true)
    const q = 6e3, t = 10, dt = 0.05
    let s = initThermal(g, v, 290)
    for (let k = 0; k < t / dt; k++) s = thermalStep(g, v, s, radiativeLoad(q), dt)
    const Ts = s.T[i * NP]
    expect(Ts).toBeLessThan(450) // virgin material, no pyrolysis
    const Tm = 0.5 * (290 + Ts)
    const k = 0.2 + 1e-4 * (Tm - 300), c = 1000 + (Tm - 300), rho = 270 // virgin PICA (Tran et al. 1997)
    const qNet = q - 0.9 * SIGMA_SB * (0.5 * (290 ** 4 + Ts ** 4))
    const dT = 2 * qNet * Math.sqrt(t / (Math.PI * k * rho * c))
    expect(rel(Ts - 290, dT)).toBeLessThan(0.15)
  })

  // Fixed (T12): the conductance node k → k+1 used node k's conductivity in row k and node k+1's in row
  // k+1 (non-symmetric matrix, −1.8 % … −43 % energy). Now one interface conductance
  // G_{k+½} = 1/(Δx_k/(2k_k) + Δx_{k+1}/(2k_{k+1})) is shared by both rows.
  test("PICA-X 1-D conduction conserves energy (stored = absorbed, non-pyrolysing, < 1e-6)", () => {
    const i = firstCell(true)
    const q = 3e3, dt = 1
    const FR = nodeFractions()
    let s = initThermal(g, v, 290)
    let ein = 0, de = 0
    for (let k = 0; k < 100; k++) {
      const s1 = thermalStep(g, v, s, radiativeLoad(q), dt)
      const Ts = s.T[i * NP], Tn = s1.T[i * NP]
      ein += dt * (q - 0.9 * SIGMA_SB * (4 * Ts ** 3 * Tn - 3 * Ts ** 4)) // the scheme's linearised re-radiation
      for (let j = 0; j < NP; j++) de += 270 * Math.min(1900, Math.max(900, 1000 + (s.T[i * NP + j] - 300))) * FR[j] * s.thick[i] * (s1.T[i * NP + j] - s.T[i * NP + j])
      de += (7900 * v.steelThickness * cpSteel(s.Tsub[i]) + 1500 * 0.0003 * 1200) * (s1.Tsub[i] - s.Tsub[i])
      s = s1
    }
    expect(s.T[i * NP]).toBeLessThan(450)
    expect(Math.abs(de / ein - 1)).toBeLessThan(1e-6)
  })

  test("PICA-X charring energy balance at 150 kW/m²: stored = absorbed − re-radiated − pyrolysis − transpiration", () => {
    // Model constants (thermal.ts; Tran et al. NASA TM-110440): ρ virgin 270 → char 220 kg/m³,
    // ΔH_pyro 3.0e5 J/kg, gas c_p 2500 J/kg K (reference 600 K), ε 0.9.
    const i = firstCell(true)
    const q = 1.5e5, dt = 0.2, nStep = 300
    const FR = nodeFractions()
    const clampN = (x: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, x))
    const cpMix = (T: number, rho: number): number => {
      const b = clampN((270 - rho) / 50, 0, 1)
      return (1 - b) * clampN(1000 + (T - 300), 900, 1900) + b * clampN(1100 + 0.6 * (T - 300), 1000, 2000)
    }
    let s = initThermal(g, v, 290)
    let eAbs = 0, eRad = 0, eRadTrue = 0, eStore = 0, ePyro = 0, eGas = 0
    for (let k = 0; k < nStep; k++) {
      const s1 = thermalStep(g, v, s, radiativeLoad(q), dt)
      const L = s.thick[i] - s.recession[i]
      const Ts = s.T[i * NP], Tn = s1.T[i * NP]
      eAbs += q * dt
      eRad += dt * 0.9 * SIGMA_SB * (4 * Ts ** 3 * Tn - 3 * Ts ** 4) // the scheme's linearised re-radiation
      eRadTrue += dt * 0.9 * SIGMA_SB * Tn ** 4
      for (let j = 0; j < NP; j++) {
        const dx = FR[j] * L
        eStore += s1.rho[i * NP + j] * cpMix(s.T[i * NP + j], s1.rho[i * NP + j]) * dx * (s1.T[i * NP + j] - s.T[i * NP + j])
        ePyro += (s.rho[i * NP + j] - s1.rho[i * NP + j]) * dx * 3.0e5
      }
      eStore += (7900 * v.steelThickness * cpSteel(s.Tsub[i]) + 1500 * 0.0003 * 1200) * (s1.Tsub[i] - s.Tsub[i])
      eGas += s1.mdotGas[i] * 2500 * (Tn - 600) * dt
      s = s1
    }
    expect(s.T[i * NP]).toBeGreaterThan(1000) // charring regime (radiative equilibrium ≈ 1300 K)
    expect(s.rho[i * NP]).toBeLessThan(225)
    expect(ePyro).toBeGreaterThan(0)
    // exact discrete balance of the implicit scheme (symmetric conduction matrix)
    expect(Math.abs((eStore + ePyro + eGas) - (eAbs - eRad)) / eAbs).toBeLessThan(1e-6)
    // physical balance with the true εσT⁴ (linearisation error only): < 0.5 %
    expect(Math.abs((eStore + ePyro + eGas) - (eAbs - eRadTrue)) / eAbs).toBeLessThan(0.005)
  })
})

describe("thermal limits (EN 1993-1-2, research/02)", () => {
  const C = (c: number): number => c + 273.15
  test("k_0.2p of grade 1.4301 at the EN 1993-1-2 Table C.1 nodes", () => {
    const table: [number, number][] = [[20, 1], [100, 0.82], [200, 0.68], [300, 0.64], [400, 0.6], [500, 0.54], [600, 0.49], [700, 0.4], [800, 0.27], [900, 0.14], [1000, 0.06], [1100, 0.03], [1200, 0]]
    for (const [c, k] of table) expect(steelYieldRetention(C(c))).toBeCloseTo(k, 12)
    expect(steelYieldRetention(C(850))).toBeCloseTo(0.205, 12) // linear between 800 and 900 °C
    expect(steelYieldRetention(200)).toBe(1)
    expect(steelYieldRetention(2000)).toBe(0)
  })

  test("retention never increases with temperature", () => {
    let prev = 1
    for (let T = 250; T < 1800; T += 1) {
      const k = steelYieldRetention(T)
      expect(k).toBeLessThanOrEqual(prev + 1e-15)
      prev = k
    }
  })

  test("verdict bands: 700 / 900 / 1150 / 1400 K; design limit 1150 K keeps k ≥ 0.17", () => {
    expect(steelVerdict(700)).toBe("nominal")
    expect(steelVerdict(700.1)).toBe("reduced strength")
    expect(steelVerdict(900.1)).toBe("within design")
    expect(steelVerdict(1150.1)).toBe("strength loss")
    expect(steelVerdict(1400)).toBe("failure")
    expect(steelYieldRetention(THERMAL_LIMITS.steelDesign)).toBeCloseTo(0.17, 2)
    expect(steelYieldRetention(THERMAL_LIMITS.steelReduced)).toBeCloseTo(0.466, 3)
    // limits.ts comment says "≤ 700 K nominal (k ≥ 0.59)"; Table C.1 interpolation gives 0.584
    expect(steelYieldRetention(THERMAL_LIMITS.steelFull)).toBeCloseTo(0.584, 3)
    expect(THERMAL_LIMITS.bondline).toBe(560)
  })
})
