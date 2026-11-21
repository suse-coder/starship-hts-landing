/** Small pure numeric helpers. */
export const clamp = (x: number, lo: number, hi: number): number => (x < lo ? lo : x > hi ? hi : x)
export const lerp = (a: number, b: number, t: number): number => a + (b - a) * t
export const smoothstep = (e0: number, e1: number, x: number): number => {
  const t = clamp((x - e0) / (e1 - e0), 0, 1)
  return t * t * (3 - 2 * t)
}
export const DEG = Math.PI / 180
export const RAD = 180 / Math.PI
export const sq = (x: number): number => x * x
export const sign = (x: number): number => (x > 0 ? 1 : x < 0 ? -1 : 0)
export const wrapPi = (a: number): number => {
  const t = (a + Math.PI) % (2 * Math.PI)
  return (t < 0 ? t + 2 * Math.PI : t) - Math.PI
}

/** Linear interpolation in a sorted table; clamps at the ends. */
export const interp1 = (xs: ArrayLike<number>, ys: ArrayLike<number>, x: number): number => {
  const n = xs.length
  if (x <= xs[0]) return ys[0]
  if (x >= xs[n - 1]) return ys[n - 1]
  let lo = 0
  let hi = n - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (xs[mid] <= x) lo = mid
    else hi = mid
  }
  const t = (x - xs[lo]) / (xs[hi] - xs[lo])
  return ys[lo] + (ys[hi] - ys[lo]) * t
}

/** index i such that xs[i] <= x < xs[i+1] (clamped) and fraction */
export const bracket = (xs: ArrayLike<number>, x: number): readonly [number, number] => {
  const n = xs.length
  if (x <= xs[0]) return [0, 0]
  if (x >= xs[n - 1]) return [n - 2, 1]
  let lo = 0
  let hi = n - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (xs[mid] <= x) lo = mid
    else hi = mid
  }
  return [lo, (x - xs[lo]) / (xs[hi] - xs[lo])]
}

/** Bilinear interpolation on a regular table z[i*ny + j] over axes xs (i) and ys (j). */
export const interp2 = (xs: ArrayLike<number>, ys: ArrayLike<number>, z: ArrayLike<number>, x: number, y: number): number => {
  const [i, fx] = bracket(xs, x)
  const [j, fy] = bracket(ys, y)
  const ny = ys.length
  const z00 = z[i * ny + j]
  const z01 = z[i * ny + j + 1]
  const z10 = z[(i + 1) * ny + j]
  const z11 = z[(i + 1) * ny + j + 1]
  return (z00 * (1 - fy) + z01 * fy) * (1 - fx) + (z10 * (1 - fy) + z11 * fy) * fx
}

/** Solve tridiagonal system a_i x_{i-1} + b_i x_i + c_i x_{i+1} = d_i (Thomas). Returns new array. */
export const solveTridiag = (a: Float64Array, b: Float64Array, c: Float64Array, d: Float64Array): Float64Array => {
  const n = b.length
  const cp = new Float64Array(n)
  const dp = new Float64Array(n)
  const x = new Float64Array(n)
  cp[0] = c[0] / b[0]
  dp[0] = d[0] / b[0]
  for (let i = 1; i < n; i++) {
    const m = b[i] - a[i] * cp[i - 1]
    cp[i] = c[i] / m
    dp[i] = (d[i] - a[i] * dp[i - 1]) / m
  }
  x[n - 1] = dp[n - 1]
  for (let i = n - 2; i >= 0; i--) x[i] = dp[i] - cp[i] * x[i + 1]
  return x
}

/** Brent-free robust bisection+secant root finder on [a,b] (f(a), f(b) must bracket). */
export const findRoot = (f: (x: number) => number, a0: number, b0: number, tol = 1e-10, maxIt = 100): number => {
  let a = a0
  let b = b0
  let fa = f(a)
  let fb = f(b)
  if (fa * fb > 0) return Math.abs(fa) < Math.abs(fb) ? a : b
  for (let it = 0; it < maxIt; it++) {
    // secant guess, fall back to bisection when it leaves the bracket
    let m = b - (fb * (b - a)) / (fb - fa)
    if (!(m > Math.min(a, b) && m < Math.max(a, b)) || it % 3 === 2) m = 0.5 * (a + b)
    const fm = f(m)
    if (Math.abs(fm) < tol || Math.abs(b - a) < tol * (1 + Math.abs(m))) return m
    if (fa * fm < 0) {
      b = m
      fb = fm
    } else {
      a = m
      fa = fm
    }
  }
  return 0.5 * (a + b)
}
