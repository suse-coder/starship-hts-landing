import { clamp, type V3 } from "@ship/math"
import { mhdApply, type MhdBasis, type MhdOut, N_SECT } from "@ship/mhd"

/**
 * Electrode-loading allocation: u_s = 1 − K_s ∈ [0, U_MAX] for the 8 sectors so that the
 * realised MHD moment (after the momentum bound) matches the commanded (Mx, My, Mz).
 *
 *   minimise  Σ_k w_k (Σ_s u_s M_s,k · σ − M_k,cmd)²  + ε Σ u_s²   s.t. 0 ≤ u_s ≤ U_MAX
 *
 * σ is the momentum-bound scale 1/(1 + |Σ u F|/F_cap), solved as a scalar root (regula falsi);
 * the inner box-constrained least squares (8 unknowns, ≤ 3 rows) is solved exactly by projected
 * Newton on the free set (Cholesky of the reduced Hessian, Armijo search along the projection arc;
 * Bertsekas 1982). Coordinate descent stalled here: realising a small roll next to a large pitch
 * trim needs cancellation between pitch columns of ≈ 6·10⁴ MN·m per unit loading.
 */
export const U_MAX = 1.6 // K = −0.6: driven (accelerator) electrodes, consumes power
export const U_BASE = 0.35

export interface Allocation {
  readonly u: readonly number[]
  readonly out: MhdOut
  readonly maxMoment: number // achievable |M| along the commanded pitch/yaw direction
  readonly saturated: boolean
}

/**
 * Weights on the row-normalised (Mx, My, Mz) errors. Each row is first divided by its own norm, so every
 * axis is resolved relative to its own authority: the roll row is ≈ 100× smaller than pitch/yaw, and with
 * absolute weights the coordinate descent never resolved it (realised roll stuck at a pitch by-product of
 * ≈ 0.45 % of M_y whatever roll was commanded, so the cold gas flew half the roll — T14,
 * .tmp/agentE/rollcheck.ts). Under saturation pitch/yaw win (they hold the unstable nose-first attitude);
 * roll then falls back to the cold gas.
 */
const W = [1, 30, 30]
const EPS = 1e-6

/** Cholesky solve of the SPD system M x = r (M n×n, row-major), in place on copies */
const cholSolve = (M: readonly number[][], r: readonly number[]): number[] => {
  const n = r.length
  const L = M.map((row) => row.slice())
  for (let j = 0; j < n; j++) {
    let d = L[j][j]
    for (let k = 0; k < j; k++) d -= L[j][k] * L[j][k]
    const ljj = Math.sqrt(Math.max(d, 1e-300))
    L[j][j] = ljj
    for (let i = j + 1; i < n; i++) {
      let s = L[i][j]
      for (let k = 0; k < j; k++) s -= L[i][k] * L[j][k]
      L[i][j] = s / ljj
    }
  }
  const y = new Array<number>(n).fill(0)
  for (let i = 0; i < n; i++) {
    let s = r[i]
    for (let k = 0; k < i; k++) s -= L[i][k] * y[k]
    y[i] = s / L[i][i]
  }
  const x = new Array<number>(n).fill(0)
  for (let i = n - 1; i >= 0; i--) {
    let s = y[i]
    for (let k = i + 1; k < n; k++) s -= L[k][i] * x[k]
    x[i] = s / L[i][i]
  }
  return x
}

const solveBoxLs = (A: readonly (readonly number[])[], b: readonly number[], uMax: number): number[] => {
  // A: m × 8 (m ≤ 3). minimise ½ uᵀH u − cᵀu, 0 ≤ u ≤ uMax, with the row-normalised weighted
  // objective Σ_k W_k ((A_k·u − b_k)/|A_k|)² + ε|u|²  (H = Σ W_k âₖâₖᵀ + εI, c = Σ W_k (b_k/|A_k|) âₖ)
  const n = A[0].length
  const As = A.map((row) => { const r = Math.hypot(...row) || 1; return row.map((x) => x / r) })
  const bs = b.map((bk, k) => bk / (Math.hypot(...A[k]) || 1))
  const H = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => As.reduce((acc, row, k) => acc + W[k] * row[i] * row[j], 0)))
  const eps = EPS * Math.max(...H.map((row, i) => row[i]), 1e-30)
  H.forEach((row, i) => { row[i] += eps })
  const c = Array.from({ length: n }, (_, i) => As.reduce((acc, row, k) => acc + W[k] * bs[k] * row[i], 0))
  const f = (x: readonly number[]): number => x.reduce((acc, xi, i) => acc + xi * (0.5 * H[i].reduce((s, hij, j) => s + hij * x[j], 0) - c[i]), 0)
  let x = new Array<number>(n).fill(0)
  for (let it = 0; it < 40; it++) {
    const g = H.map((row, i) => row.reduce((s, hij, j) => s + hij * x[j], 0) - c[i])
    // free set: interior, or on a bound with the gradient pointing inward
    const free = x.map((xi, i) => !((xi <= 0 && g[i] > 0) || (xi >= uMax && g[i] < 0)))
    const F = free.flatMap((fr, i) => (fr ? [i] : []))
    if (F.length === 0) break
    const pF = cholSolve(F.map((i) => F.map((j) => H[i][j])), F.map((i) => -g[i]))
    const p = new Array<number>(n).fill(0)
    F.forEach((i, k) => { p[i] = pF[k] })
    // Armijo backtracking along the projection arc x(t) = clamp(x + t p)
    const f0 = f(x)
    let t = 1
    let y = x
    for (let ls = 0; ls < 40; ls++) {
      y = x.map((xi, i) => clamp(xi + t * p[i], 0, uMax))
      const dec = g.reduce((s, gi, i) => s + gi * (y[i] - x[i]), 0)
      if (f(y) <= f0 + 1e-4 * dec) break
      t *= 0.5
    }
    const step = y.reduce((m, yi, i) => Math.max(m, Math.abs(yi - x[i])), 0)
    x = y
    if (step <= 1e-10 * (uMax + x.reduce((m, xi) => Math.max(m, xi), 0))) break
  }
  return x
}

/** momentum-bound scale floor and the relative tolerance of the scale root (realised moment error) */
const S_MIN = 0.02
const S_TOL = 0.002
/** relative realised-moment error above which the allocation is reported as saturated */
const SAT_TOL = 0.01

export const allocateMhd = (b: MhdBasis, mCmd: V3, qInf: number, Rn: number, _baseline: number): Allocation => {
  const zero = Array<number>(N_SECT).fill(0)
  if (b.fCap <= 0) return { u: zero, out: mhdApply(b, zero, qInf, Rn), maxMoment: 0, saturated: true }
  const A = [b.M.map((m) => m.x), b.M.map((m) => m.y), b.M.map((m) => m.z)]
  const mag = Math.hypot(mCmd.y, mCmd.z)
  // achievable pitch/yaw moment along the command (diagnostic): maximise along d
  const d = mag > 1 ? [mCmd.y / mag, mCmd.z / mag] : [1, 0]
  const along = (o: MhdOut): number => o.M.y * d[0] + o.M.z * d[1]
  const uMaxDir = solveBoxLs([A[1].map((y, s) => y * d[0] + A[2][s] * d[1])], [1e15], U_MAX)
  const maxMoment = along(mhdApply(b, uMaxDir, qInf, Rn))
  // Momentum bound: the plant realises M(u) = sc(u)·A u with sc(u) = 1/(1 + |F(u)|/F_cap). With
  // u(s) = boxLS(M_cmd / s), the realised moment is (sc(u(s))/s)·M_cmd, so the allocation is the root of
  // the scalar equation φ(s) = sc(u(s)) − s on [S_MIN, 1]. φ(1) ≤ 0 always; the plain fixed point
  // s ← sc(u(s)) approaches the root from above and slows down as the bound tightens (it stopped at
  // −6 % / −16.8 % at 60 % / 90 % authority after 4 passes), so bracket and solve by Illinois regula falsi.
  const evalAt = (s: number): { s: number; u: number[]; out: MhdOut; f: number } => {
    const u = solveBoxLs(A, [mCmd.x / s, mCmd.y / s, mCmd.z / s], U_MAX)
    const out = mhdApply(b, u, qInf, Rn)
    return { s, u, out, f: out.scale - s }
  }
  let best = evalAt(1)
  if (best.f < -S_TOL) {
    let hi = best
    let lo = evalAt(S_MIN)
    if (lo.f <= 0) best = lo // even at the scale floor the bound wins: report the floor solution (saturated below)
    else {
      let side = 0
      for (let it = 0; it < 40; it++) {
        const s = (lo.s * hi.f - hi.s * lo.f) / (hi.f - lo.f)
        const m = evalAt(clamp(s, S_MIN, 1))
        best = m
        if (Math.abs(m.f) < S_TOL * m.s) break
        if (m.f < 0) {
          hi = m
          if (side === -1) lo = { ...lo, f: lo.f / 2 }
          side = -1
        } else {
          lo = m
          if (side === 1) hi = { ...hi, f: hi.f / 2 }
          side = 1
        }
      }
    }
  }
  // honest saturation flag: pitch/yaw beyond the directional authority, or the realised moment (all three
  // axes, roll included) off the command by more than SAT_TOL
  const cmdNorm = Math.hypot(mCmd.x, mCmd.y, mCmd.z)
  const err = Math.hypot(best.out.M.x - mCmd.x, best.out.M.y - mCmd.y, best.out.M.z - mCmd.z)
  return { u: best.u, out: best.out, maxMoment, saturated: maxMoment < mag || (cmdNorm > 1 && err > SAT_TOL * cmdNorm) }
}
