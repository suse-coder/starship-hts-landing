/**
 * CPU-baked procedural data for the environment shaders (no DOM, no three.js: plain typed arrays,
 * so the bakes are unit-testable and can run in a worker).
 *
 *  - `bakeWaves`: a tileable patch of ocean surface slopes built from a sum of sinusoids with
 *    integer wave vectors (exactly periodic), plus the fraction of the slope variance that survives
 *    each mip level. The water shader turns the variance lost to filtering into micro-roughness, so
 *    the sun glitter has the same energy whether the waves are resolved or sub-pixel.
 *  - `bakeTerrainNoise`: four independent tileable fBm fields, histogram-equalised to a uniform
 *    distribution (a threshold is then an area fraction), for the coastal ground cover.
 */

/** mulberry32: deterministic uniform [0, 1) */
export const rng = (seed: number): (() => number) => () => {
  seed = (seed + 0x6d2b79f5) | 0
  let t = Math.imul(seed ^ (seed >>> 15), 1 | seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}

export interface WaveField {
  /** texels per side (power of two) */
  readonly n: number
  /**
   * RGBA per texel: R, G = surface slopes ∂h/∂x, ∂h/∂y of the patch normalised to unit total slope
   * variance (⟨R² + G²⟩ = 1), B = height normalised to unit variance, A = 1. The dominant travel
   * direction of the waves is +x.
   */
  readonly data: Float32Array
  /** fraction of the slope variance left at mip level l (box filter), l = 0 … log2(n) */
  readonly retained: Float32Array
}

/**
 * Sea-surface patch. `count` wave components with integer wave vectors of 2 … 40 cycles per tile,
 * log-uniform in |k| with equal slope amplitude: the slope variance per octave is constant, as for a
 * k⁻⁴ (Phillips-type) elevation spectrum. Directions spread about +x, wide for short waves (±80°)
 * and narrow for the long ones (±30°). The steepest third of the components carries a phase-locked
 * second harmonic (Stokes second-order: sharper crests, flatter troughs).
 */
export const bakeWaves = (n = 256, seed = 7, count = 64): WaveField => {
  const rnd = rng(seed)
  const kx = new Int32Array(count), ky = new Int32Array(count), ph = new Float64Array(count), st = new Float64Array(count)
  const K_MIN = 2, K_MAX = 40
  for (let i = 0; i < count; i++) {
    const k = K_MIN * Math.pow(K_MAX / K_MIN, rnd())
    const spread = ((30 + 50 * Math.min(1, Math.log(k / K_MIN) / Math.log(8))) * Math.PI) / 180
    // sum of two uniforms: triangular spreading, most energy near the mean direction
    const th = (rnd() + rnd() - 1) * spread
    let a = Math.round(k * Math.cos(th)), b = Math.round(k * Math.sin(th))
    if (a === 0 && b === 0) a = K_MIN
    kx[i] = a; ky[i] = b
    ph[i] = rnd() * Math.PI * 2
    st[i] = 2 * Math.hypot(a, b) <= K_MAX && rnd() < 0.5 ? 0.3 : 0 // second-harmonic steepness k·a
  }
  const data = new Float32Array(n * n * 4)
  const ux = new Float64Array(count), uy = new Float64Array(count), ik = new Float64Array(count)
  for (let c = 0; c < count; c++) { const km = Math.hypot(kx[c], ky[c]); ux[c] = kx[c] / km; uy[c] = ky[c] / km; ik[c] = 1 / km }
  const twoPi = Math.PI * 2
  for (let j = 0; j < n; j++) {
    const y = (j + 0.5) / n
    for (let i = 0; i < n; i++) {
      const x = (i + 0.5) / n
      let sx = 0, sy = 0, h = 0
      for (let c = 0; c < count; c++) {
        const t = twoPi * (kx[c] * x + ky[c] * y) + ph[c]
        const sn = Math.sin(t), cs = Math.cos(t)
        // unit slope amplitude along k̂: h = cos(t)/|k|, slope = −k̂ sin(t); second harmonic of
        // steepness s: sin 2t = 2 sin t cos t, cos 2t = 1 − 2 sin² t
        const s1 = -sn * (1 + 2 * st[c] * cs)
        sx += ux[c] * s1; sy += uy[c] * s1
        h += (cs + 0.5 * st[c] * (1 - 2 * sn * sn)) * ik[c]
      }
      const p = (j * n + i) * 4
      data[p] = sx; data[p + 1] = sy; data[p + 2] = h; data[p + 3] = 1
    }
  }
  // normalise: unit total slope variance, unit height variance
  let vs = 0, vh = 0, mh = 0
  for (let p = 0; p < n * n; p++) { vs += data[p * 4] ** 2 + data[p * 4 + 1] ** 2; mh += data[p * 4 + 2] }
  mh /= n * n
  for (let p = 0; p < n * n; p++) vh += (data[p * 4 + 2] - mh) ** 2
  const ks = 1 / Math.sqrt(Math.max(vs / (n * n), 1e-12)), kh = 1 / Math.sqrt(Math.max(vh / (n * n), 1e-12))
  for (let p = 0; p < n * n; p++) { data[p * 4] *= ks; data[p * 4 + 1] *= ks; data[p * 4 + 2] = (data[p * 4 + 2] - mh) * kh }
  // slope variance per mip level (2×2 box filter, as the GPU builds the chain)
  const levels = Math.round(Math.log2(n)) + 1
  const retained = new Float32Array(levels)
  let m = n, ax = new Float32Array(n * n), ay = new Float32Array(n * n)
  for (let p = 0; p < n * n; p++) { ax[p] = data[p * 4]; ay[p] = data[p * 4 + 1] }
  for (let l = 0; l < levels; l++) {
    let v = 0
    for (let p = 0; p < m * m; p++) v += ax[p] * ax[p] + ay[p] * ay[p]
    retained[l] = Math.min(1, v / (m * m))
    if (m === 1) break
    const h2 = m / 2, bx = new Float32Array(h2 * h2), by = new Float32Array(h2 * h2)
    for (let j = 0; j < h2; j++) for (let i = 0; i < h2; i++) {
      const q = 2 * j * m + 2 * i
      bx[j * h2 + i] = 0.25 * (ax[q] + ax[q + 1] + ax[q + m] + ax[q + m + 1])
      by[j * h2 + i] = 0.25 * (ay[q] + ay[q + 1] + ay[q + m] + ay[q + m + 1])
    }
    ax = bx; ay = by; m = h2
  }
  retained[0] = 1
  return { n, data, retained }
}

/** periodic value noise on a p × p lattice over the unit square, quintic interpolation, added × w */
const valueNoise = (dst: Float32Array, n: number, p: number, w: number, rnd: () => number): void => {
  const g = new Float32Array(p * p)
  for (let i = 0; i < g.length; i++) g[i] = rnd() * 2 - 1
  for (let j = 0; j < n; j++) {
    const y = ((j + 0.5) / n) * p, j0 = Math.floor(y), fy = y - j0, uy = fy * fy * fy * (fy * (fy * 6 - 15) + 10)
    const r0 = (j0 % p) * p, r1 = ((j0 + 1) % p) * p
    for (let i = 0; i < n; i++) {
      const x = ((i + 0.5) / n) * p, i0 = Math.floor(x), fx = x - i0, ux = fx * fx * fx * (fx * (fx * 6 - 15) + 10)
      const c0 = i0 % p, c1 = (i0 + 1) % p
      const a = g[r0 + c0] + (g[r0 + c1] - g[r0 + c0]) * ux, b = g[r1 + c0] + (g[r1 + c1] - g[r1 + c0]) * ux
      dst[j * n + i] += w * (a + (b - a) * uy)
    }
  }
}

/** histogram equalisation in place: values → their rank in [0, 1] */
export const equalise = (a: Float32Array): void => {
  let lo = Infinity, hi = -Infinity
  for (let i = 0; i < a.length; i++) { if (a[i] < lo) lo = a[i]; if (a[i] > hi) hi = a[i] }
  const B = 2048, h = new Float64Array(B + 1), s = B / Math.max(hi - lo, 1e-9)
  for (let i = 0; i < a.length; i++) h[Math.min(B - 1, Math.floor((a[i] - lo) * s))]++
  let acc = 0
  for (let b = 0; b < B; b++) { const c = h[b]; h[b] = acc / a.length; acc += c }
  h[B] = 1
  for (let i = 0; i < a.length; i++) {
    const x = (a[i] - lo) * s, b = Math.min(B - 1, Math.floor(x))
    a[i] = h[b] + (h[b + 1] - h[b]) * (x - b)
  }
}

/**
 * Four independent tileable fBm fields (lattice periods 4, 8, 16, 32 cells per tile, gain 0.5),
 * each equalised to a uniform distribution, packed as RGBA bytes (n × n × 4).
 */
export const bakeTerrainNoise = (n = 256, seed = 11): Uint8Array => {
  const rnd = rng(seed)
  const out = new Uint8Array(n * n * 4)
  for (let c = 0; c < 4; c++) {
    const a = new Float32Array(n * n)
    let w = 0.5
    for (const p of [4, 8, 16, 32]) { valueNoise(a, n, p, w, rnd); w *= 0.5 }
    equalise(a)
    for (let i = 0; i < n * n; i++) out[i * 4 + c] = Math.round(Math.max(0, Math.min(1, a[i])) * 255)
  }
  return out
}
