/**
 * Small conic QP solver (ADMM, OSQP/COSMO form) for the powered-descent planner.
 *
 *   minimise  ½ xᵀP x + qᵀx   subject to  A x ∈ C,   C = C₁ × C₂ × …
 *
 * with C_i one of: a point (equality), a box, a (shifted) second-order cone or a (shifted) ball.
 * The iteration is the standard operator splitting
 *   x̃ = (P + σI + AᵀRA)⁻¹ (σx − q + Aᵀ(Rz − y)),  x ← αx̃ + (1−α)x,
 *   ẑ = αAx̃ + (1−α)z,  z ← Π_C(ẑ + R⁻¹y),  y ← y + R(ẑ − z),
 * which converges for every convex C with a projection. The KKT matrix is dense, small (≤ ~130) and
 * factored by Cholesky; A is stored by sparse rows. The solver returns the primal and dual residuals
 * and a `converged` flag — the caller never has to trust a fixed iteration count.
 *
 * Numeric kernel: loops and typed arrays are intentional here (ADR-03).
 */

export type Cone =
  | { readonly kind: "eq"; readonly n: number; readonly b: Float64Array }
  | { readonly kind: "box"; readonly n: number; readonly lo: Float64Array; readonly hi: Float64Array }
  /** { z : (z + shift) ∈ SOC }, SOC = { (t, w) : ‖w‖ ≤ t } */
  | { readonly kind: "soc"; readonly n: number; readonly shift?: Float64Array }
  /** { z : ‖z + shift‖ ≤ r } */
  | { readonly kind: "ball"; readonly n: number; readonly r: number; readonly shift?: Float64Array }

/** A by sparse rows */
export interface SparseRows {
  readonly ptr: Int32Array
  readonly col: Int32Array
  readonly val: Float64Array
  readonly m: number
}

export const sparseRows = (rows: readonly (readonly (readonly [number, number])[])[]): SparseRows => {
  const nnz = rows.reduce((s, r) => s + r.length, 0)
  const ptr = new Int32Array(rows.length + 1), col = new Int32Array(nnz), val = new Float64Array(nnz)
  let k = 0
  for (let i = 0; i < rows.length; i++) {
    for (const [c, v] of rows[i]) { col[k] = c; val[k] = v; k++ }
    ptr[i + 1] = k
  }
  return { ptr, col, val, m: rows.length }
}

export interface ConicProblem {
  readonly n: number
  /** dense symmetric n×n, row-major (may be all zero) */
  readonly P: Float64Array
  readonly q: Float64Array
  readonly A: SparseRows
  readonly cones: readonly Cone[]
}

export interface ConicOptions {
  readonly maxIter?: number
  readonly epsAbs?: number
  readonly epsRel?: number
  readonly rho?: number
  readonly sigma?: number
  readonly alpha?: number
  readonly warm?: { readonly x: Float64Array; readonly z: Float64Array; readonly y: Float64Array } | null
}

export interface ConicSolution {
  readonly x: Float64Array
  readonly z: Float64Array
  readonly y: Float64Array
  readonly iters: number
  /** ‖Ax − z‖∞ and ‖Px + q + Aᵀy‖∞ at exit */
  readonly rPrim: number
  readonly rDual: number
  readonly converged: boolean
}

/** in-place Cholesky L Lᵀ of a dense SPD matrix (lower triangle); returns false if not SPD */
const cholesky = (M: Float64Array, n: number): boolean => {
  for (let j = 0; j < n; j++) {
    let d = M[j * n + j]
    for (let k = 0; k < j; k++) d -= M[j * n + k] * M[j * n + k]
    if (!(d > 1e-300)) return false
    const dj = Math.sqrt(d)
    M[j * n + j] = dj
    for (let i = j + 1; i < n; i++) {
      let s = M[i * n + j]
      for (let k = 0; k < j; k++) s -= M[i * n + k] * M[j * n + k]
      M[i * n + j] = s / dj
    }
  }
  return true
}

const cholSolve = (L: Float64Array, n: number, b: Float64Array, x: Float64Array): void => {
  for (let i = 0; i < n; i++) {
    let s = b[i]
    for (let k = 0; k < i; k++) s -= L[i * n + k] * x[k]
    x[i] = s / L[i * n + i]
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = x[i]
    for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k]
    x[i] = s / L[i * n + i]
  }
}

const mulA = (A: SparseRows, x: Float64Array, out: Float64Array): void => {
  for (let i = 0; i < A.m; i++) {
    let s = 0
    for (let k = A.ptr[i]; k < A.ptr[i + 1]; k++) s += A.val[k] * x[A.col[k]]
    out[i] = s
  }
}

/** out += Aᵀ v */
const addAt = (A: SparseRows, v: Float64Array, out: Float64Array): void => {
  for (let i = 0; i < A.m; i++) {
    const vi = v[i]
    if (vi === 0) continue
    for (let k = A.ptr[i]; k < A.ptr[i + 1]; k++) out[A.col[k]] += A.val[k] * vi
  }
}

const projectSoc = (v: Float64Array, o: number, n: number): void => {
  const t = v[o]
  let w2 = 0
  for (let i = 1; i < n; i++) w2 += v[o + i] * v[o + i]
  const w = Math.sqrt(w2)
  if (w <= t) return
  if (w <= -t) { for (let i = 0; i < n; i++) v[o + i] = 0; return }
  const f = (t + w) / (2 * w)
  v[o] = 0.5 * (t + w)
  for (let i = 1; i < n; i++) v[o + i] *= f
}

/** Euclidean projection of v onto C, in place */
export const projectCones = (cones: readonly Cone[], v: Float64Array): void => {
  let o = 0
  for (const c of cones) {
    if (c.kind === "eq") {
      for (let i = 0; i < c.n; i++) v[o + i] = c.b[i]
    } else if (c.kind === "box") {
      for (let i = 0; i < c.n; i++) v[o + i] = Math.min(c.hi[i], Math.max(c.lo[i], v[o + i]))
    } else if (c.kind === "soc") {
      if (c.shift) for (let i = 0; i < c.n; i++) v[o + i] += c.shift[i]
      projectSoc(v, o, c.n)
      if (c.shift) for (let i = 0; i < c.n; i++) v[o + i] -= c.shift[i]
    } else {
      let s2 = 0
      for (let i = 0; i < c.n; i++) { const w = v[o + i] + (c.shift ? c.shift[i] : 0); s2 += w * w }
      const s = Math.sqrt(s2)
      if (s > c.r) {
        const f = c.r / s
        for (let i = 0; i < c.n; i++) { const sh = c.shift ? c.shift[i] : 0; v[o + i] = (v[o + i] + sh) * f - sh }
      }
    }
    o += c.n
  }
}

/** normalise every row of A to unit norm, scaling b/conic data to match (never changes the set) */
export const normalizeRows = (A: SparseRows, cones: readonly Cone[]): void => {
  let o = 0
  for (const c of cones) {
    for (let i = 0; i < c.n; i++) {
      const r = o + i
      let nrm = 0
      for (let k = A.ptr[r]; k < A.ptr[r + 1]; k++) nrm += A.val[k] * A.val[k]
      nrm = Math.sqrt(nrm)
      if (nrm > 1e-12 && Math.abs(nrm - 1) > 1e-9) {
        const f = 1 / nrm
        for (let k = A.ptr[r]; k < A.ptr[r + 1]; k++) A.val[k] *= f
        // the affine offset lives in the projection set, not in A: scale it by the same factor
        if (c.kind === "eq") c.b[i] *= f
        else if (c.kind === "soc" && c.shift) c.shift[i] *= f
        else if (c.kind === "ball" && c.shift) c.shift[i] *= f
        // box bounds are on z, not on A z: nothing to scale
      }
      o++
    }
  }
}

const normInf = (v: Float64Array): number => {
  let m = 0
  for (let i = 0; i < v.length; i++) { const a = Math.abs(v[i]); if (a > m) m = a }
  return m
}

export const solveConic = (pr: ConicProblem, opt: ConicOptions = {}): ConicSolution => {
  const n = pr.n, m = pr.A.m, A = pr.A
  const maxIter = opt.maxIter ?? 2000, epsAbs = opt.epsAbs ?? 1e-3, epsRel = opt.epsRel ?? 1e-3
  const sigma = opt.sigma ?? 1e-6, alpha = opt.alpha ?? 1.6
  let rho = opt.rho ?? 0.3
  // per-row penalty: equality rows are weighted 1e3× (OSQP), uniform inside every cone. Rows are
  // expected to be scaled to O(1) by the model builder; scaling here would hide a badly scaled row.
  const rowScale = new Float64Array(m)
  { let o = 0; for (const c of pr.cones) { for (let i = 0; i < c.n; i++) rowScale[o + i] = c.kind === "eq" ? 1e3 : 1; o += c.n } }
  const L = new Float64Array(n * n)
  const factor = (): boolean => {
    L.set(pr.P)
    for (let i = 0; i < n; i++) L[i * n + i] += sigma
    for (let r = 0; r < m; r++) {
      const w = rho * rowScale[r]
      for (let a = A.ptr[r]; a < A.ptr[r + 1]; a++) {
        const ia = A.col[a], va = w * A.val[a]
        for (let b = A.ptr[r]; b < A.ptr[r + 1]; b++) { const ib = A.col[b]; if (ib <= ia) L[ia * n + ib] += va * A.val[b] }
      }
    }
    return cholesky(L, n)
  }
  const x = new Float64Array(n), z = new Float64Array(m), y = new Float64Array(m)
  if (opt.warm && opt.warm.x.length === n && opt.warm.z.length === m) { x.set(opt.warm.x); z.set(opt.warm.z); y.set(opt.warm.y) }
  else projectCones(pr.cones, z)
  const rhs = new Float64Array(n), xt = new Float64Array(n), tmpM = new Float64Array(m), ax = new Float64Array(m), zh = new Float64Array(m)
  const px = new Float64Array(n), aty = new Float64Array(n)
  let rPrim = Infinity, rDual = Infinity, it = 0, converged = false
  if (!factor()) return { x, z, y, iters: 0, rPrim, rDual, converged: false }
  for (it = 1; it <= maxIter; it++) {
    for (let i = 0; i < n; i++) rhs[i] = sigma * x[i] - pr.q[i]
    for (let r = 0; r < m; r++) tmpM[r] = rho * rowScale[r] * z[r] - y[r]
    addAt(A, tmpM, rhs)
    cholSolve(L, n, rhs, xt)
    mulA(A, xt, ax)
    for (let i = 0; i < n; i++) x[i] = alpha * xt[i] + (1 - alpha) * x[i]
    for (let r = 0; r < m; r++) {
      zh[r] = alpha * ax[r] + (1 - alpha) * z[r]
      z[r] = zh[r] + y[r] / (rho * rowScale[r])
    }
    projectCones(pr.cones, z)
    for (let r = 0; r < m; r++) y[r] += rho * rowScale[r] * (zh[r] - z[r])
    if (it % 10 === 0 || it === maxIter) {
      mulA(A, x, ax)
      let rp = 0
      for (let r = 0; r < m; r++) { const d = Math.abs(ax[r] - z[r]); if (d > rp) rp = d }
      px.fill(0)
      for (let i = 0; i < n; i++) { let s = 0; for (let k = 0; k < n; k++) s += pr.P[i * n + k] * x[k]; px[i] = s }
      aty.fill(0)
      addAt(A, y, aty)
      let rd = 0
      for (let i = 0; i < n; i++) { const d = Math.abs(px[i] + pr.q[i] + aty[i]); if (d > rd) rd = d }
      rPrim = rp; rDual = rd
      const tolP = epsAbs + epsRel * Math.max(normInf(ax), normInf(z))
      const tolD = epsAbs + epsRel * Math.max(normInf(px), normInf(aty), normInf(pr.q))
      if (rp <= tolP && rd <= tolD) { converged = true; break }
      // residual balancing (refactor is cheap at this size)
      if (it % 50 === 0) {
        const sP = rp / Math.max(tolP, 1e-12), sD = rd / Math.max(tolD, 1e-12)
        if (sP > 5 * sD || sD > 5 * sP) {
          rho = Math.min(1e4, Math.max(1e-4, rho * Math.sqrt(sP / Math.max(sD, 1e-12))))
          if (!factor()) break
        }
      }
    }
  }
  return { x, z, y, iters: Math.min(it, maxIter), rPrim, rDual, converged }
}
