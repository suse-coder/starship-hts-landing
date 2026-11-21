import { AMU, EV, H_PLANCK, K_B, M_E } from "@ship/earth"
import { findRoot } from "@ship/math"

/**
 * Equilibrium air from statistical mechanics: 9 species (N2, O2, NO, N, O, NO+, N+, O+, e-).
 * Equilibrium constants come from partition functions (translational, rigid-rotor, harmonic
 * oscillator, tabulated electronic levels) — no curve fits — and the composition is solved by a
 * Newton iteration in log-number-densities with element and charge conservation.
 * Valid ~300–15 000 K (no double ionisation, no Ar).
 */
interface Species {
  readonly name: string
  readonly m: number // kg
  readonly thetaR: number // K (0 = atom)
  readonly sym: number
  readonly thetaV: number // K (0 = atom)
  readonly levels: readonly (readonly [number, number])[] // [g, energy K]
  readonly e0: number // formation energy relative to N2,O2 at 0 K, J
}

const KJ = (ev: number): number => ev * EV
const D_N2 = 9.759
const D_O2 = 5.116
const D_NO = 6.497
const I_NO = 9.264
const I_N = 14.534
const I_O = 13.618

export const SPECIES: readonly Species[] = [
  { name: "N2", m: 28.0134 * AMU, thetaR: 2.88, sym: 2, thetaV: 3395, levels: [[1, 0], [3, 72231]], e0: 0 },
  { name: "O2", m: 31.9988 * AMU, thetaR: 2.08, sym: 2, thetaV: 2239, levels: [[3, 0], [2, 11393], [1, 18985]], e0: 0 },
  { name: "NO", m: 30.0061 * AMU, thetaR: 2.45, sym: 1, thetaV: 2817, levels: [[2, 0], [2, 174]], e0: KJ((D_N2 + D_O2) / 2 - D_NO) },
  { name: "N", m: 14.0067 * AMU, thetaR: 0, sym: 1, thetaV: 0, levels: [[4, 0], [10, 27658], [6, 41495]], e0: KJ(D_N2 / 2) },
  { name: "O", m: 15.9994 * AMU, thetaR: 0, sym: 1, thetaV: 0, levels: [[5, 0], [3, 228], [1, 326], [5, 22830], [1, 48620]], e0: KJ(D_O2 / 2) },
  { name: "NO+", m: 30.0055 * AMU, thetaR: 2.87, sym: 1, thetaV: 3419, levels: [[1, 0]], e0: KJ((D_N2 + D_O2) / 2 - D_NO + I_NO) },
  { name: "N+", m: 14.0062 * AMU, thetaR: 0, sym: 1, thetaV: 0, levels: [[1, 0], [3, 70], [5, 188], [5, 22037]], e0: KJ(D_N2 / 2 + I_N) },
  { name: "O+", m: 15.9989 * AMU, thetaR: 0, sym: 1, thetaV: 0, levels: [[4, 0], [10, 38575]], e0: KJ(D_O2 / 2 + I_O) },
  { name: "e", m: M_E, thetaR: 0, sym: 1, thetaV: 0, levels: [[2, 0]], e0: 0 },
]
export const S = { N2: 0, O2: 1, NO: 2, N: 3, O: 4, NOp: 5, Np: 6, Op: 7, e: 8 } as const

/** ln of the per-volume partition function (energies from each species' own ground state) */
const lnQ = (s: Species, T: number): number => {
  const tr = 1.5 * Math.log((2 * Math.PI * s.m * K_B * T) / (H_PLANCK * H_PLANCK))
  const rot = s.thetaR > 0 ? Math.log(T / (s.sym * s.thetaR)) : 0
  const vib = s.thetaV > 0 ? -Math.log(1 - Math.exp(-s.thetaV / T)) : 0
  let el = 0
  for (const [g, th] of s.levels) el += g * Math.exp(-th / T)
  return tr + rot + vib + Math.log(el)
}

/** specific internal energy per particle (J) excluding formation energy */
const eInt = (s: Species, T: number): number => {
  const tr = 1.5 * K_B * T
  const rot = s.thetaR > 0 ? K_B * T : 0
  const vib = s.thetaV > 0 ? (K_B * s.thetaV) / (Math.exp(s.thetaV / T) - 1) : 0
  let q = 0
  let qe = 0
  for (const [g, th] of s.levels) {
    const w = g * Math.exp(-th / T)
    q += w
    qe += w * th
  }
  return tr + rot + vib + (K_B * qe) / q
}

/** ln Kc (number-density form, m^-3) for the 6 reactions */
const lnK = (T: number): Float64Array => {
  const L = SPECIES.map((s) => lnQ(s, T))
  const kT = K_B * T
  return Float64Array.from([
    2 * L[S.N] - L[S.N2] - KJ(D_N2) / kT,
    2 * L[S.O] - L[S.O2] - KJ(D_O2) / kT,
    L[S.N] + L[S.O] - L[S.NO] - KJ(D_NO) / kT,
    L[S.NOp] + L[S.e] - L[S.NO] - KJ(I_NO) / kT,
    L[S.Np] + L[S.e] - L[S.N] - KJ(I_N) / kT,
    L[S.Op] + L[S.e] - L[S.O] - KJ(I_O) / kT,
  ])
}

const N_TO_O = 0.79 / 0.21 // element ratio in air (Ar folded into N2)
const M_NUC = (N_TO_O * 14.0067 + 15.9994) * AMU // mass per O-nucleus "unit"

export interface AirState {
  readonly T: number
  readonly rho: number
  readonly n: Float64Array // number densities, m^-3 (SPECIES order)
  readonly p: number // Pa
  readonly h: number // J/kg (relative to N2, O2 at 0 K)
  readonly ntot: number
  readonly mbar: number // mean particle mass, kg
}

/** equilibrium composition at (T, rho) */
export const equilibrium = (T: number, rho: number): AirState => {
  const nO = rho / M_NUC
  const nN = N_TO_O * nO
  const n = new Float64Array(9)
  if (T < 600) {
    n[S.N2] = nN / 2
    n[S.O2] = nO / 2
  } else {
    const K = lnK(T)
    // unknowns: a = ln[N], b = ln[O], c = ln[e]
    let a = Math.min(Math.log(nN), 0.5 * (K[0] + Math.log(nN / 2)))
    let b = Math.min(Math.log(nO), 0.5 * (K[1] + Math.log(nO / 2)))
    let c = Math.min(Math.log(nN), 0.5 * (K[3] + a + b - K[2]))
    for (let it = 0; it < 60; it++) {
      const N = Math.exp(a), O = Math.exp(b), e = Math.exp(c)
      const N2 = Math.exp(2 * a - K[0])
      const O2 = Math.exp(2 * b - K[1])
      const NO = Math.exp(a + b - K[2])
      const NOp = Math.exp(K[3] + a + b - K[2] - c)
      const Np = Math.exp(K[4] + a - c)
      const Op = Math.exp(K[5] + b - c)
      const F1 = (2 * N2 + N + NO + NOp + Np - nN) / nN
      const F2 = (2 * O2 + O + NO + NOp + Op - nO) / nO
      const sc = e + NOp + Np + Op
      const F3 = (e - NOp - Np - Op) / sc
      const J11 = (4 * N2 + N + NO + NOp + Np) / nN, J12 = (NO + NOp) / nN, J13 = (-NOp - Np) / nN
      const J21 = (NO + NOp) / nO, J22 = (4 * O2 + O + NO + NOp + Op) / nO, J23 = (-NOp - Op) / nO
      const J31 = (-NOp - Np) / sc, J32 = (-NOp - Op) / sc, J33 = (e + NOp + Np + Op) / sc
      const det = J11 * (J22 * J33 - J23 * J32) - J12 * (J21 * J33 - J23 * J31) + J13 * (J21 * J32 - J22 * J31)
      const dA = (-F1 * (J22 * J33 - J23 * J32) + F2 * (J12 * J33 - J13 * J32) - F3 * (J12 * J23 - J13 * J22)) / det
      const dB = (F1 * (J21 * J33 - J23 * J31) - F2 * (J11 * J33 - J13 * J31) + F3 * (J11 * J23 - J13 * J21)) / det
      const dC = (-F1 * (J21 * J32 - J22 * J31) + F2 * (J11 * J32 - J12 * J31) - F3 * (J11 * J22 - J12 * J21)) / det
      const lim = Math.max(1, Math.abs(dA), Math.abs(dB), Math.abs(dC)) / 2
      const f = lim > 1 ? 1 / lim : 1
      a += dA * f
      b += dB * f
      c += dC * f
      if (Math.abs(F1) + Math.abs(F2) + Math.abs(F3) < 1e-11) break
    }
    n[S.N] = Math.exp(a)
    n[S.O] = Math.exp(b)
    n[S.e] = Math.exp(c)
    n[S.N2] = Math.exp(2 * a - K[0])
    n[S.O2] = Math.exp(2 * b - K[1])
    n[S.NO] = Math.exp(a + b - K[2])
    n[S.NOp] = Math.exp(K[3] + a + b - K[2] - c)
    n[S.Np] = Math.exp(K[4] + a - c)
    n[S.Op] = Math.exp(K[5] + b - c)
  }
  let ntot = 0
  let E = 0
  for (let i = 0; i < 9; i++) {
    ntot += n[i]
    E += n[i] * (eInt(SPECIES[i], T) + SPECIES[i].e0)
  }
  const p = ntot * K_B * T
  const h = (E + p) / rho
  return { T, rho, n, p, h, ntot, mbar: rho / ntot }
}

/** temperature for given (rho, h) */
export const tempFromRhoH = (rho: number, h: number): number =>
  findRoot((T) => equilibrium(T, rho).h - h, 150, 30000, 1e-6, 80)

/** (rho, T) for given (p, h): nested solve (p is nearly ∝ rho at fixed h) */
export const stateFromPH = (p: number, h: number, rhoGuess: number): AirState => {
  let rho = rhoGuess
  let st = equilibrium(tempFromRhoH(rho, h), rho)
  for (let i = 0; i < 30; i++) {
    const r = p / st.p
    rho *= r
    st = equilibrium(tempFromRhoH(rho, h), rho)
    if (Math.abs(r - 1) < 1e-7) break
  }
  return st
}

export interface ShockResult {
  readonly eps: number // rho1/rho2
  readonly s2: AirState // post normal shock
  readonly u2: number
  readonly s0: AirState // stagnation (isentropic deceleration approx.)
  readonly cpMax: number // (p0 - p1)/q1
}

/** Real-gas normal shock + stagnation state. Upstream: T1, rho1, U (m/s). */
export const normalShock = (T1: number, rho1: number, U: number): ShockResult => {
  const s1 = equilibrium(T1, rho1)
  const p1 = s1.p
  const h1 = s1.h
  const g = (eps: number): number => {
    const p2 = p1 + rho1 * U * U * (1 - eps)
    const h2 = h1 + 0.5 * U * U * (1 - eps * eps)
    const rho2 = rho1 / eps
    return equilibrium(tempFromRhoH(rho2, h2), rho2).p / p2 - 1
  }
  // perfect-gas bounds: eps in [1/40, 1]; find root away from the trivial eps = 1
  const M1 = U / Math.sqrt((1.4 * p1) / rho1)
  const epsPerfect = M1 > 1.0001 ? ((1.4 - 1) * M1 * M1 + 2) / ((1.4 + 1) * M1 * M1) : 0.999
  const eps = M1 > 1.05 ? findRoot(g, 0.02, Math.min(0.995, epsPerfect + 0.02), 1e-9, 80) : 0.999
  const p2 = p1 + rho1 * U * U * (1 - eps)
  const h2 = h1 + 0.5 * U * U * (1 - eps * eps)
  const rho2 = rho1 / eps
  const s2 = equilibrium(tempFromRhoH(rho2, h2), rho2)
  const u2 = U * eps
  const h0 = h2 + 0.5 * u2 * u2
  // stagnation pressure: low-Mach isentropic compression behind the shock
  const p0 = p2 + 0.5 * rho2 * u2 * u2 * (1 + 0.25 * (u2 * u2 * rho2) / (1.2 * p2))
  const s0 = stateFromPH(p0, h0, rho2 * (p0 / p2) ** 0.85)
  return { eps, s2, u2, s0, cpMax: (p0 - p1) / (0.5 * rho1 * U * U) }
}
