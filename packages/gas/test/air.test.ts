/**
 * spec/05 T-air, T-shock — equilibrium air, real-gas normal shock, stagnation state, plasma transport.
 * References:
 *  - Hansen, NASA TR R-50 (1959) and Gupta, Yos, Thompson & Lee, NASA RP-1232 (1990): equilibrium air
 *    at 1 atm — O₂ largely dissociated at 4000 K, N₂ intact; electron mole fraction ~1e-3 at 7000 K and
 *    ≈ 2–3 % at 10 000 K (spec/05 T-air: x_e 1–3 % at 10 000 K).
 *  - Zeldovich/JANAF: N₂ + O₂ ⇌ 2NO, K_p(2000 K) ≈ 4e-4 → equilibrium NO in air at 2000 K ≈ 0.8 %.
 *  - Calorically perfect diatomic air at 300 K: h = (7/2) kT/m̄ (rotational + translational modes).
 *  - Rankine–Hugoniot (γ = 1.4): ρ2/ρ1 = (γ+1)M²/((γ−1)M²+2), p2/p1 = 1 + 2γ(M²−1)/(γ+1),
 *    T2/T1 = [2γM²−(γ−1)][(γ−1)M²+2]/((γ+1)²M²); Rayleigh pitot formula for p02/p1.
 *    At M = 10: ρ2/ρ1 = 5.714, p2/p1 = 116.5 (NACA 1135 tables).
 *  - Strong-shock stagnation (Newtonian–Busemann): C_p,max = 2 − ε, ε = ρ1/ρ2 (Anderson, Hypersonic and
 *    High-Temperature Gas Dynamics, ch. 3).
 *  - Equilibrium effective γ = h/e of dissociated air behind re-entry shocks ≈ 1.1–1.2 (Anderson ch. 14).
 *  - NRL Plasma Formulary: f_pe = 8.98 √n_e Hz (n_e in m⁻³), Spitzer resistivity η ≈ 5.2e-5 lnΛ T_eV^-1.5 Ω m.
 */
import { describe, expect, test } from "bun:test"
import { atmosphere, K_B, E_CHARGE, M_E } from "@ship/earth"
import {
  equilibrium, normalShock, tempFromRhoH, S, SPECIES, SHOCK_TABLE, lookupStag, DEFAULT_GRID, nuElectronNeutral,
  nuElectronIon, coulombLog, plasmaProps, criticalDensity, cesiumIonisation, hallParameter, cyclotronFreq, type AirState,
} from "../src/index"

const rel = (a: number, b: number): number => Math.abs(a / b - 1)
/** equilibrium state at 1 atm and temperature T (fixed point on ρ) */
const atOneAtm = (T: number): AirState => {
  let rho = (101325 * 0.0289) / (8.314 * T)
  for (let i = 0; i < 40; i++) rho *= 101325 / equilibrium(T, rho).p
  return equilibrium(T, rho)
}
const x = (s: AirState, i: number): number => s.n[i] / s.ntot

describe("equilibrium air composition (T-air)", () => {
  test("300 K: 79 % N₂ / 21 % O₂, h = (7/2) kT/m̄", () => {
    const s = atOneAtm(300)
    expect(x(s, S.N2)).toBeCloseTo(0.79, 10)
    expect(x(s, S.O2)).toBeCloseTo(0.21, 10)
    expect(rel(s.h, (3.5 * K_B * 300) / s.mbar)).toBeLessThan(0.005)
  })

  test("conservation: N/O nuclei = 0.79/0.21, charge neutrality, Σ n_i m_i = ρ (1000–15 000 K)", () => {
    for (const T of [1000, 2500, 4000, 6000, 8000, 10000, 15000]) {
      const s = atOneAtm(T)
      const n = s.n
      const nN = 2 * n[S.N2] + n[S.N] + n[S.NO] + n[S.NOp] + n[S.Np]
      const nO = 2 * n[S.O2] + n[S.O] + n[S.NO] + n[S.NOp] + n[S.Op]
      expect(rel(nN / nO, 0.79 / 0.21)).toBeLessThan(1e-9)
      if (T >= 2500) expect(Math.abs(n[S.e] - n[S.NOp] - n[S.Np] - n[S.Op]) / n[S.e]).toBeLessThan(1e-9)
      let m = 0
      for (let i = 0; i < 9; i++) m += n[i] * SPECIES[i].m
      expect(rel(m, s.rho)).toBeLessThan(1e-5) // 4-digit tabulated ion masses
      expect(rel(s.p, s.ntot * K_B * T)).toBeLessThan(1e-12)
    }
  })

  test("2000 K, 1 atm: thermal NO ≈ 0.8 % (Zeldovich equilibrium, K_p ≈ 4e-4)", () => {
    const s = atOneAtm(2000)
    expect(x(s, S.NO)).toBeGreaterThan(0.005)
    expect(x(s, S.NO)).toBeLessThan(0.011)
  })

  test("4000 K, 1 atm: O₂ mostly dissociated, N₂ essentially intact", () => {
    const s = atOneAtm(4000)
    const alphaO2 = x(s, S.O) / (x(s, S.O) + 2 * x(s, S.O2))
    expect(alphaO2).toBeGreaterThan(0.6)
    expect(x(s, S.N2)).toBeGreaterThan(0.6)
    expect(x(s, S.N)).toBeLessThan(0.01)
  })

  test("7000 K, 1 atm: n_e ~ 1e21 m⁻³ (x_e ~ 1e-3, Saha/NO⁺ dominated)", () => {
    const s = atOneAtm(7000)
    expect(x(s, S.e)).toBeGreaterThan(2e-4)
    expect(x(s, S.e)).toBeLessThan(3e-3)
    expect(s.n[S.e]).toBeGreaterThan(2e20)
    expect(s.n[S.e]).toBeLessThan(3e21)
  })

  test("10 000 K, 1 atm: electron mole fraction 1–3 % (spec/05 T-air)", () => {
    const s = atOneAtm(10000)
    expect(x(s, S.e)).toBeGreaterThan(0.01)
    expect(x(s, S.e)).toBeLessThan(0.03)
  })

  test("T(ρ, h) inverts h(T, ρ)", () => {
    for (const [T, rho] of [[700, 1], [3500, 0.05], [7000, 1e-3], [11000, 1e-4]]) {
      expect(rel(tempFromRhoH(rho, equilibrium(T, rho).h), T)).toBeLessThan(1e-6)
    }
  })
})

describe("real-gas normal shock (T-shock)", () => {
  const g = 1.4
  const rh = (M: number): number => ((g + 1) * M * M) / ((g - 1) * M * M + 2)
  const machFlow = (hkm: number, M: number): { a: ReturnType<typeof atmosphere>; U: number } => {
    const a = atmosphere(hkm * 1000)
    return { a, U: M * Math.sqrt((1.4 * equilibrium(a.T, a.rho).p) / a.rho) }
  }

  test("perfect-gas limit: M 2.4 at 30 km (T2 ≈ 460 K) density ratio within 1 % of Rankine–Hugoniot", () => {
    const { a, U } = machFlow(30, 2.4)
    const r = normalShock(a.T, a.rho, U)
    expect(rel(1 / r.eps, rh(2.4))).toBeLessThan(0.01)
    // Rayleigh pitot C_p,max (γ = 1.4) within 1 %
    const M = 2.4
    const p02 = (((g + 1) ** 2 * M * M) / (4 * g * M * M - 2 * (g - 1))) ** (g / (g - 1)) * ((1 - g + 2 * g * M * M) / (g + 1))
    expect(rel(r.cpMax, (p02 - 1) / (0.5 * g * M * M))).toBeLessThan(0.01)
  })

  test("Rankine–Hugoniot jump conditions hold exactly (mass, momentum, total enthalpy)", () => {
    const a = atmosphere(60000)
    const U = 7000
    const r = normalShock(a.T, a.rho, U)
    const s1 = equilibrium(a.T, a.rho)
    expect(rel(r.s2.rho * r.u2, a.rho * U)).toBeLessThan(1e-12)
    expect(rel(r.s2.p + r.s2.rho * r.u2 ** 2, s1.p + a.rho * U * U)).toBeLessThan(1e-6)
    expect(rel(r.s2.h + 0.5 * r.u2 ** 2, s1.h + 0.5 * U * U)).toBeLessThan(1e-6)
  })

  test("M 10: real gas compresses more and is cooler than γ = 1.4 (ρ2/ρ1 > 5.714, T2 < perfect), p2/p1 ≈ 116.5 ± 10 %", () => {
    const { a, U } = machFlow(40, 10)
    const r = normalShock(a.T, a.rho, U)
    const M = 10
    const T2perf = (a.T * (2 * g * M * M - (g - 1)) * ((g - 1) * M * M + 2)) / ((g + 1) ** 2 * M * M)
    expect(1 / r.eps).toBeGreaterThan(rh(10))
    expect(r.s2.T).toBeLessThan(T2perf)
    expect(rel(r.s2.p / equilibrium(a.T, a.rho).p, 1 + (2 * g * (M * M - 1)) / (g + 1))).toBeLessThan(0.1)
  })

  test("LEO entry (60 km, 7 km/s): ρ2/ρ1 ≈ 14–18, T2 ≈ 5500–7000 K, C_p,max = 2 − ε", () => {
    const a = atmosphere(60000)
    const r = normalShock(a.T, a.rho, 7000)
    expect(1 / r.eps).toBeGreaterThan(14)
    expect(1 / r.eps).toBeLessThan(18)
    expect(r.s2.T).toBeGreaterThan(5500)
    expect(r.s2.T).toBeLessThan(7000)
    expect(Math.abs(r.cpMax - (2 - r.eps))).toBeLessThan(0.01)
  })
})

describe("shipped shock table (gas/src/shock-table.json)", () => {
  test("matches the current solver at grid nodes (table is up to date)", () => {
    for (const [ih, iu] of [[24, 34], [26, 20], [16, 10], [8, 5]]) {
      const h = DEFAULT_GRID.hs[ih], U = DEFAULT_GRID.us[iu]
      const a = atmosphere(h)
      const r = normalShock(a.T, a.rho, U)
      const t = lookupStag(SHOCK_TABLE, h, U)
      expect(rel(t.eps, r.eps)).toBeLessThan(1e-4)
      expect(rel(t.T0, r.s0.T)).toBeLessThan(1e-4)
      expect(rel(t.p0, r.s0.p)).toBeLessThan(1e-4)
      expect(rel(t.cpMax, r.cpMax)).toBeLessThan(1e-4)
    }
  })

  test("γ_eff: 1.400 for cold air at low speed, 1.10–1.20 for the LEO-entry stagnation state", () => {
    expect(lookupStag(SHOCK_TABLE, 0, 100).gammaE).toBeCloseTo(1.4, 2)
    const st = lookupStag(SHOCK_TABLE, 65000, 7000)
    expect(st.gammaE).toBeGreaterThan(1.1)
    expect(st.gammaE).toBeLessThan(1.2)
  })

  test("stagnation state at 65 km, 7 km/s: T0 5500–7000 K, h0 = h∞ + U²/2, n_e 5e18–3e20 m⁻³", () => {
    const a = atmosphere(65000)
    const st = lookupStag(SHOCK_TABLE, 65000, 7000)
    expect(st.T0).toBeGreaterThan(5500)
    expect(st.T0).toBeLessThan(7000)
    expect(rel(st.h0, equilibrium(a.T, a.rho).h + 0.5 * 7000 ** 2)).toBeLessThan(1e-3)
    expect(st.ne0).toBeGreaterThan(5e18)
    expect(st.ne0).toBeLessThan(3e20)
  })
})

describe("plasma transport", () => {
  test("electron–neutral collision frequency in sea-level air ~ 1e11–1e12 s⁻¹ (Raizer, Gas Discharge Physics)", () => {
    const nu = nuElectronNeutral(equilibrium(300, 1.225))
    expect(nu).toBeGreaterThan(1e11)
    expect(nu).toBeLessThan(1e12)
  })

  test("Spitzer conductivity from ν_ei within 5 % of the NRL formulary (1 eV, 1e21 m⁻³)", () => {
    const T = 11604.5, ne = 1e21
    const sigma = (ne * E_CHARGE ** 2) / (M_E * nuElectronIon(ne, T))
    const lnL = coulombLog(ne, T)
    expect(rel(sigma, 1 / (5.2e-5 * lnL))).toBeLessThan(0.05)
  })

  test("plasma frequency at the critical density equals the wave frequency (Ku band 12 GHz)", () => {
    const f = 12e9
    const nc = criticalDensity(f)
    expect(rel(nc, 1.786e18)).toBeLessThan(1e-3)
    expect(rel(plasmaProps(nc, 1e9, 3000).fp, f)).toBeLessThan(1e-3)
  })

  test("Hall parameter β = ω_ce/ν and cyclotron frequency 28 GHz/T", () => {
    expect(rel(cyclotronFreq(1), 27.99e9)).toBeLessThan(1e-3)
    expect(hallParameter(2, 1e10)).toBeCloseTo((2 * Math.PI * cyclotronFreq(2)) / 1e10, 10)
  })

  test("Cs seed Saha solution satisfies n⁺(n⁺ + n_e,air) = K(T)(n_Cs − n⁺) and grows with T", () => {
    const K = (T: number): number => ((2 * Math.PI * M_E * K_B * T) / 6.62607015e-34 ** 2) ** 1.5 * Math.exp((-3.894 * E_CHARGE) / (K_B * T))
    for (const [T, nCs, ne] of [[2000, 1e21, 1e15], [3000, 1e22, 1e18], [5000, 1e21, 1e20]]) {
      const np = cesiumIonisation(T, nCs, ne)
      expect(rel(np * (np + ne), K(T) * (nCs - np))).toBeLessThan(1e-9)
    }
    expect(cesiumIonisation(2500, 1e21, 0)).toBeGreaterThan(cesiumIonisation(2000, 1e21, 0))
    expect(cesiumIonisation(2500, 0, 0)).toBe(0)
  })
})
