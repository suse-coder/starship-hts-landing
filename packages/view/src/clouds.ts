import * as THREE from "three"

/**
 * Volumetric fair-weather cumulus deck, ray-marched in a box fixed to the pad frame.
 *
 * Density (Schneider, "The real-time volumetric cloudscapes of Horizon Zero Dawn", SIGGRAPH 2015;
 * GPU Pro 7 ch. 4; "Nubis" 2017):
 *  - 2-D weather map (CPU, tileable): coverage potential from Worley cells (one cumulus per cell) mixed
 *    with Perlin fBm, histogram-equalised so that `coverage` is the true area fraction of cloud bases;
 *    a second channel varies the tower height, a third modulates the coverage regionally (de-tiling);
 *  - column profile: the cloud top rises with the coverage strength towards the cell core (domes, flat
 *    bases cut by the condensation level);
 *  - 3-D shape noise (CPU, 96³ RG8, tileable): Perlin–Worley + Worley fBm, Nubis remap against the
 *    column profile (billowy erosion at edges and tops);
 *  - 3-D detail noise (CPU, 32³ R8): Worley fBm erosion, wispy at the base, billowy higher up.
 * Lighting (Hillaire, "Physically based sky, atmosphere and cloud rendering in Frostbite", SIGGRAPH 2016;
 * Wrenninge et al., "Oz: the great and volumetric", SIGGRAPH 2013):
 *  - Beer–Lambert transmittance along the view ray, energy-conserving step integration;
 *  - 6-sample light march towards the Sun with geometrically growing steps (≈ 1.8 km);
 *  - dual-lobe Henyey–Greenstein phase (g = 0.6 / −0.3) and a 4-octave multiple-scattering series
 *    (extinction × 0.4ᵏ, energy × 0.6ᵏ, eccentricity × 0.5ᵏ);
 *  - powder term on thin edges (view dependent), ambient sky light (height and core occlusion) and a
 *    warm ground bounce on the bases;
 *  - aerial perspective along the camera → cloud path through an exponential atmosphere
 *    (H = 8 km, 30 km sea-level extinction length).
 * The ray start is jittered with a fixed 8×8 Bayer pattern (no temporal noise).
 * A child mesh at sea level darkens the ground under the clouds (cloud shadows).
 *
 * Extinction of cumulus: σ ≈ 0.02–0.1 m⁻¹ (liquid water 0.2–0.5 g m⁻³, r_eff ≈ 8 µm); here 0.05 m⁻¹ at
 * density 1, single-scattering albedo ≈ 1.
 */

export interface CloudFrame {
  /** unit, world (three.js) frame */
  readonly sunDir: THREE.Vector3
  /** linear RGB irradiance colour of the direct Sun at the deck (reddens at low Sun) */
  readonly sunColor: THREE.Color
  /** linear ambient sky colour (mean radiance of the upper sky), scaled by the day factor */
  readonly skyColor: THREE.Color
  /** world position of the pad-frame origin (sea level at the pad); the clouds are fixed in that frame */
  readonly origin: THREE.Vector3
  /** seconds (slow drift/evolution, wind) */
  readonly time: number
  /** m/s East, North at cloud level (drift of the field) */
  readonly wind: readonly [number, number]
  /** 0..1 fraction of the Sun visible (orbital night → 0) */
  readonly eclipse: number
  /** half-width of the region covered this frame, m (the deck grows with altitude; ≤ maxExtent) */
  readonly extent?: number
  /** 0..1 hand-off weight; the deck fades out above the global layer and in from orbit */
  readonly fade?: number
}

export interface CloudDeck {
  readonly mesh: THREE.Mesh
  readonly update: (f: CloudFrame) => void
}

// ---------------------------------------------------------------------------------------------------
// CPU noise (tileable)

const rng = (seed: number): (() => number) => () => {
  seed = (seed + 0x6d2b79f5) | 0
  let t = Math.imul(seed ^ (seed >>> 15), 1 | seed)
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296
}

/** wrapped lattice index table: t[i + n] = i mod n for i ∈ [−n, 2n) */
const wrapTable = (n: number): Int32Array => {
  const t = new Int32Array(3 * n)
  for (let i = 0; i < 3 * n; i++) t[i] = i % n
  return t
}

/**
 * 1 − min(1, F1) of a tileable 3-D Worley field on an n³ grid, c cells per period (F1 in cell units).
 * Each feature point is splatted into the voxels within one cell radius: exact for the clamped F1 and
 * ≈ 6× cheaper than the 27-cell search.
 */
const worley3 = (n: number, c: number, rnd: () => number): Float32Array => {
  const best = new Float32Array(n * n * n).fill(1)
  const wt = wrapTable(n)
  const r = n / c, r2 = r * r, inv = 1 / r2
  for (let cz = 0; cz < c; cz++) for (let cy = 0; cy < c; cy++) for (let cx = 0; cx < c; cx++) {
    const fx = (cx + rnd()) * r, fy = (cy + rnd()) * r, fz = (cz + rnd()) * r
    const z1 = Math.floor(fz + r - 0.5)
    for (let iz = Math.ceil(fz - r - 0.5); iz <= z1; iz++) {
      const dz = iz + 0.5 - fz, rz = r2 - dz * dz
      if (rz < 0) continue
      const ry = Math.sqrt(rz), wz = wt[iz + n] * n, y1 = Math.floor(fy + ry - 0.5)
      for (let iy = Math.ceil(fy - ry - 0.5); iy <= y1; iy++) {
        const dy = iy + 0.5 - fy, ryz = rz - dy * dy
        if (ryz < 0) continue
        const rx = Math.sqrt(ryz), row = (wz + wt[iy + n]) * n, dyz = dz * dz + dy * dy
        const x1 = Math.floor(fx + rx - 0.5)
        for (let ix = Math.ceil(fx - rx - 0.5); ix <= x1; ix++) {
          const dx = ix + 0.5 - fx, d = (dx * dx + dyz) * inv, k = row + wt[ix + n]
          if (d < best[k]) best[k] = d
        }
      }
    }
  }
  for (let i = 0; i < best.length; i++) best[i] = 1 - Math.sqrt(best[i])
  return best
}

/** 2-D version of worley3 on an n² grid */
const worley2 = (n: number, c: number, rnd: () => number): Float32Array => {
  const best = new Float32Array(n * n).fill(1)
  const wt = wrapTable(n)
  const r = n / c, r2 = r * r, inv = 1 / r2
  for (let cy = 0; cy < c; cy++) for (let cx = 0; cx < c; cx++) {
    const fx = (cx + rnd()) * r, fy = (cy + rnd()) * r
    const y1 = Math.floor(fy + r - 0.5)
    for (let iy = Math.ceil(fy - r - 0.5); iy <= y1; iy++) {
      const dy = iy + 0.5 - fy, ry = r2 - dy * dy
      if (ry < 0) continue
      const rx = Math.sqrt(ry), row = wt[iy + n] * n, x1 = Math.floor(fx + rx - 0.5)
      for (let ix = Math.ceil(fx - rx - 0.5); ix <= x1; ix++) {
        const dx = ix + 0.5 - fx, d = (dx * dx + dy * dy) * inv, k = row + wt[ix + n]
        if (d < best[k]) best[k] = d
      }
    }
  }
  for (let i = 0; i < best.length; i++) best[i] = 1 - Math.sqrt(best[i])
  return best
}

/** voxel-centre offsets inside one lattice cell (v voxels per cell) and their quintic fade weights */
const cellAxis = (v: number): { f: Float64Array; u: Float64Array } => {
  const f = new Float64Array(v), u = new Float64Array(v)
  for (let j = 0; j < v; j++) { const t = (j + 0.5) / v; f[j] = t; u[j] = t * t * t * (t * (t * 6 - 15) + 10) }
  return { f, u }
}

/**
 * tileable 3-D Perlin gradient noise (≈ −1..1), p lattice cells per period (n divisible by p), added
 * × w into dst. Cell by cell with the 8 corner gradients hoisted and the y/z parts of the dot products
 * factored per row.
 */
const perlin3 = (dst: Float32Array, n: number, p: number, w: number, rnd: () => number): void => {
  const G = new Float64Array(p * p * p * 3)
  for (let i = 0; i < p * p * p; i++) {
    const z = rnd() * 2 - 1, a = rnd() * Math.PI * 2, s = Math.sqrt(1 - z * z)
    G[i * 3] = s * Math.cos(a); G[i * 3 + 1] = s * Math.sin(a); G[i * 3 + 2] = z
  }
  const v = n / p, { f, u } = cellAxis(v)
  const gi = (x: number, y: number, z: number): number => (((z % p) * p + (y % p)) * p + (x % p)) * 3
  for (let cz = 0; cz < p; cz++) for (let cy = 0; cy < p; cy++) for (let cx = 0; cx < p; cx++) {
    const o0 = gi(cx, cy, cz), o1 = gi(cx + 1, cy, cz), o2 = gi(cx, cy + 1, cz), o3 = gi(cx + 1, cy + 1, cz)
    const o4 = gi(cx, cy, cz + 1), o5 = gi(cx + 1, cy, cz + 1), o6 = gi(cx, cy + 1, cz + 1), o7 = gi(cx + 1, cy + 1, cz + 1)
    const ax = G[o0], bx = G[o1], cxg = G[o2], dx = G[o3], ex = G[o4], fxg = G[o5], gx = G[o6], hx = G[o7]
    for (let vz = 0; vz < v; vz++) {
      const fz = f[vz], fz1 = fz - 1, uz = u[vz]
      for (let vy = 0; vy < v; vy++) {
        const fy = f[vy], fy1 = fy - 1, uy = u[vy]
        const a0 = G[o0 + 1] * fy + G[o0 + 2] * fz, b0 = G[o1 + 1] * fy + G[o1 + 2] * fz
        const c0 = G[o2 + 1] * fy1 + G[o2 + 2] * fz, d0 = G[o3 + 1] * fy1 + G[o3 + 2] * fz
        const e0 = G[o4 + 1] * fy + G[o4 + 2] * fz1, f0 = G[o5 + 1] * fy + G[o5 + 2] * fz1
        const g0 = G[o6 + 1] * fy1 + G[o6 + 2] * fz1, h0 = G[o7 + 1] * fy1 + G[o7 + 2] * fz1
        let k = ((cz * v + vz) * n + cy * v + vy) * n + cx * v
        for (let vx = 0; vx < v; vx++, k++) {
          const fx = f[vx], fx1 = fx - 1, ux = u[vx]
          const l0 = ax * fx + a0 + (bx * fx1 + b0 - ax * fx - a0) * ux
          const l1 = cxg * fx + c0 + (dx * fx1 + d0 - cxg * fx - c0) * ux
          const l2 = ex * fx + e0 + (fxg * fx1 + f0 - ex * fx - e0) * ux
          const l3 = gx * fx + g0 + (hx * fx1 + h0 - gx * fx - g0) * ux
          const m0 = l0 + (l1 - l0) * uy, m1 = l2 + (l3 - l2) * uy
          dst[k] += w * (m0 + (m1 - m0) * uz)
        }
      }
    }
  }
}

/** tileable 2-D Perlin gradient noise (≈ −0.7..0.7), p cells per period (n divisible by p), added × w */
const perlin2 = (dst: Float32Array, n: number, p: number, w: number, rnd: () => number): void => {
  const G = new Float64Array(p * p * 2)
  for (let i = 0; i < p * p; i++) { const a = rnd() * Math.PI * 2; G[i * 2] = Math.cos(a); G[i * 2 + 1] = Math.sin(a) }
  const v = n / p, { f, u } = cellAxis(v)
  const gi = (x: number, y: number): number => ((y % p) * p + (x % p)) * 2
  for (let cy = 0; cy < p; cy++) for (let cx = 0; cx < p; cx++) {
    const o0 = gi(cx, cy), o1 = gi(cx + 1, cy), o2 = gi(cx, cy + 1), o3 = gi(cx + 1, cy + 1)
    const ax = G[o0], bx = G[o1], cxg = G[o2], dx = G[o3]
    for (let vy = 0; vy < v; vy++) {
      const fy = f[vy], fy1 = fy - 1, uy = u[vy]
      const a0 = G[o0 + 1] * fy, b0 = G[o1 + 1] * fy, c0 = G[o2 + 1] * fy1, d0 = G[o3 + 1] * fy1
      let k = (cy * v + vy) * n + cx * v
      for (let vx = 0; vx < v; vx++, k++) {
        const fx = f[vx], fx1 = fx - 1, ux = u[vx]
        const l0 = ax * fx + a0 + (bx * fx1 + b0 - ax * fx - a0) * ux
        const l1 = cxg * fx + c0 + (dx * fx1 + d0 - cxg * fx - c0) * ux
        dst[k] += w * (l0 + (l1 - l0) * uy)
      }
    }
  }
}

/** trilinear 2× upsampling of a tileable n³ field (low octaves are baked at half resolution) */
const up3 = (a: Float32Array, n: number): Float32Array => {
  const m = 2 * n, o = new Float32Array(m * m * m)
  const i0 = new Int32Array(m), i1 = new Int32Array(m), fr = new Float32Array(m)
  for (let i = 0; i < m; i++) { const x = (i + 0.5) / 2 - 0.5, k = Math.floor(x); i0[i] = (k + n) % n; i1[i] = (k + 1) % n; fr[i] = x - k }
  let q = 0
  for (let z = 0; z < m; z++) {
    const z0 = i0[z] * n * n, z1 = i1[z] * n * n, fz = fr[z]
    for (let y = 0; y < m; y++) {
      const y0 = i0[y] * n, y1 = i1[y] * n, fy = fr[y]
      const r00 = z0 + y0, r01 = z0 + y1, r10 = z1 + y0, r11 = z1 + y1
      for (let x = 0; x < m; x++, q++) {
        const x0 = i0[x], x1 = i1[x], fx = fr[x]
        const a00 = a[r00 + x0] + (a[r00 + x1] - a[r00 + x0]) * fx, a01 = a[r01 + x0] + (a[r01 + x1] - a[r01 + x0]) * fx
        const a10 = a[r10 + x0] + (a[r10 + x1] - a[r10 + x0]) * fx, a11 = a[r11 + x0] + (a[r11 + x1] - a[r11 + x0]) * fx
        const b0 = a00 + (a01 - a00) * fy, b1 = a10 + (a11 - a10) * fy
        o[q] = b0 + (b1 - b0) * fz
      }
    }
  }
  return o
}

/** bilinear k× upsampling of a tileable n² field */
const up2 = (a: Float32Array, n: number, k: number): Float32Array => {
  const m = k * n, o = new Float32Array(m * m)
  const i0 = new Int32Array(m), i1 = new Int32Array(m), fr = new Float32Array(m)
  for (let i = 0; i < m; i++) { const x = (i + 0.5) / k - 0.5, j = Math.floor(x); i0[i] = (j + n) % n; i1[i] = (j + 1) % n; fr[i] = x - j }
  let q = 0
  for (let y = 0; y < m; y++) {
    const y0 = i0[y] * n, y1 = i1[y] * n, fy = fr[y]
    for (let x = 0; x < m; x++, q++) {
      const x0 = i0[x], x1 = i1[x], fx = fr[x]
      const b0 = a[y0 + x0] + (a[y0 + x1] - a[y0 + x0]) * fx, b1 = a[y1 + x0] + (a[y1 + x1] - a[y1 + x0]) * fx
      o[q] = b0 + (b1 - b0) * fy
    }
  }
  return o
}

/** histogram equalisation in place: values → their CDF rank in [0, 1] (uniform distribution) */
const equalise = (a: Float32Array): void => {
  let lo = Infinity, hi = -Infinity
  for (let i = 0; i < a.length; i++) { lo = Math.min(lo, a[i]); hi = Math.max(hi, a[i]) }
  const B = 4096, h = new Float64Array(B + 1), s = B / Math.max(hi - lo, 1e-9)
  for (let i = 0; i < a.length; i++) h[Math.min(B - 1, Math.floor((a[i] - lo) * s))]++
  let acc = 0
  for (let b = 0; b < B; b++) { const c = h[b]; h[b] = acc / a.length; acc += c }
  h[B] = 1
  for (let i = 0; i < a.length; i++) {
    const x = (a[i] - lo) * s, b = Math.min(B - 1, Math.floor(x))
    a[i] = h[b] + (h[b + 1] - h[b]) * (x - b)
  }
}

/** linear normalisation of a zero-mean field to [0, 1]: ±2 standard deviations → 0 / 1 */
const normalise = (a: Float32Array): void => {
  let s = 0
  for (let i = 0; i < a.length; i++) s += a[i] * a[i]
  const k = 0.25 / Math.max(Math.sqrt(s / a.length), 1e-9)
  for (let i = 0; i < a.length; i++) a[i] = Math.max(0, Math.min(1, 0.5 + a[i] * k))
}

/** bilinear sample of a tileable n² field, coordinates in texels */
const sampleWrap2 = (a: Float32Array, n: number, x: number, y: number): number => {
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0
  const i0 = ((x0 % n) + n) % n, i1 = (i0 + 1) % n, j0 = ((y0 % n) + n) % n, j1 = (j0 + 1) % n
  const a00 = a[j0 * n + i0], a10 = a[j0 * n + i1], a01 = a[j1 * n + i0], a11 = a[j1 * n + i1]
  const b0 = a00 + (a10 - a00) * fx, b1 = a01 + (a11 - a01) * fx
  return b0 + (b1 - b0) * fy
}

/** squeeze a tileable field horizontally (features k× wider), keeping it tileable for k = 1/2/4 */
const stretch2 = (a: Float32Array, n: number, k: number): Float32Array => {
  const o = new Float32Array(n * n)
  for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) o[y * n + x] = sampleWrap2(a, n, x * k, y)
  return o
}

// texture tiles (metres) and resolutions (n divisible by every Perlin period / Worley cell count used)
const WEATHER_N = 512, WEATHER_TILE = 30000
const SHAPE_N = 64, SHAPE_TILE = 3000
const DETAIL_N = 32, DETAIL_TILE = 380
const CIRRUS_N = 512, CIRRUS_TILE = 80000

interface CloudNoise {
  /** RGBA: coverage potential (equalised), tower height, regional modulation, 1 */
  readonly weather: Float32Array
  /** billowy base-shape noise, equalised to a uniform distribution */
  readonly shape: Float32Array
  /** Worley fBm detail noise, equalised */
  readonly detail: Float32Array
  /** thin high-cloud veil, streaky (stretched horizontally), values in 0..1 */
  readonly cirrus: Float32Array
}

const bakeNoise = (seed: number): CloudNoise => {
  const rnd = rng(seed)
  // weather (2-D): cumulus cells (Worley 2.5 km + 1.1 km) broken up by Perlin fBm
  const n2 = WEATHER_N * WEATHER_N
  const cellsBig = worley2(WEATHER_N, 12, rnd), cellsSmall = worley2(WEATHER_N, 28, rnd)
  const pot = new Float32Array(n2)
  perlin2(pot, WEATHER_N, 4, 0.5, rnd); perlin2(pot, WEATHER_N, 16, 0.2, rnd)
  for (let i = 0; i < n2; i++) pot[i] = 0.62 * cellsBig[i] + 0.28 * cellsSmall[i] + 0.45 * pot[i]
  equalise(pot)
  // low-frequency maps at a quarter of the resolution, upsampled
  const nq = WEATHER_N / 4
  const tallQ = new Float32Array(nq * nq)
  perlin2(tallQ, nq, 8, 0.6, rnd); perlin2(tallQ, nq, 32, 0.3, rnd)
  normalise(tallQ)
  const regionQ = new Float32Array(nq * nq)
  perlin2(regionQ, nq, 2, 0.6, rnd); perlin2(regionQ, nq, 4, 0.35, rnd)
  normalise(regionQ)
  const tall = up2(tallQ, nq, 4), region = up2(regionQ, nq, 4)
  const weather = new Float32Array(n2 * 4)
  for (let i = 0; i < n2; i++) { weather[i * 4] = pot[i]; weather[i * 4 + 1] = tall[i]; weather[i * 4 + 2] = region[i]; weather[i * 4 + 3] = 1 }
  // shape (3-D): Perlin–Worley (Perlin fBm dilated by Worley fBm), Nubis remap against a finer Worley
  // fBm, equalised: values are uniform in [0, 1], so the erosion threshold is an area fraction
  const n3 = SHAPE_N ** 3
  const w4 = up3(worley3(SHAPE_N / 2, 4, rnd), SHAPE_N / 2), w8 = worley3(SHAPE_N, 8, rnd), w16 = worley3(SHAPE_N, 16, rnd)
  const shape = new Float32Array(n3)
  perlin3(shape, SHAPE_N, 4, 0.55, rnd); perlin3(shape, SHAPE_N, 8, 0.3, rnd)
  for (let i = 0; i < n3; i++) {
    const wf = 0.625 * w4[i] + 0.25 * w8[i] + 0.125 * w16[i]
    const p = Math.max(0, Math.min(1, 0.5 + 1.25 * shape[i]))
    const pw = wf + p * (1 - wf) // remap(perlin, 0, 1, worleyFbm, 1)
    const wfine = 0.625 * w8[i] + 0.375 * w16[i]
    shape[i] = (pw + 1 - wfine) / (2 - wfine) // remap(pw, wfine − 1, 1, 0, 1)
  }
  equalise(shape)
  // detail (3-D): Worley fBm (cells 190 / 95 / 48 m), equalised
  const nd = DETAIL_N ** 3
  const d2 = worley3(DETAIL_N, 2, rnd), d4 = worley3(DETAIL_N, 4, rnd), d8 = worley3(DETAIL_N, 8, rnd)
  const detail = new Float32Array(nd)
  for (let i = 0; i < nd; i++) detail[i] = 0.625 * d2[i] + 0.25 * d4[i] + 0.125 * d8[i]
  equalise(detail)
  // cirrus (2-D): a horizontally stretched Perlin veil (jet-stream streaks) plus a fine isotropic
  // component; remapped to sparse, soft wisps so the high layer reads as ice cloud, not fog
  const nc = CIRRUS_N * CIRRUS_N
  const cirrusBase = new Float32Array(nc)
  perlin2(cirrusBase, CIRRUS_N, 4, 0.55, rnd); perlin2(cirrusBase, CIRRUS_N, 8, 0.28, rnd); perlin2(cirrusBase, CIRRUS_N, 16, 0.12, rnd)
  const cirrusFine = new Float32Array(nc)
  perlin2(cirrusFine, CIRRUS_N, 16, 0.45, rnd); perlin2(cirrusFine, CIRRUS_N, 32, 0.25, rnd); perlin2(cirrusFine, CIRRUS_N, 64, 0.12, rnd)
  const cirrusWide = stretch2(cirrusBase, CIRRUS_N, 0.25), cirrusWide2 = stretch2(cirrusBase, CIRRUS_N, 0.5)
  const cirrus = new Float32Array(nc)
  for (let i = 0; i < nc; i++) {
    const v = 0.55 * cirrusWide[i] + 0.28 * cirrusWide2[i] + 0.17 * cirrusFine[i]
    cirrus[i] = Math.max(0, Math.min(1, (v - 0.38) / 0.42))
  }
  normalise(cirrus)
  return { weather, shape, detail, cirrus }
}

const half = (a: Float32Array): Uint16Array => {
  const h = new Uint16Array(a.length)
  for (let i = 0; i < a.length; i++) h[i] = THREE.DataUtils.toHalfFloat(a[i])
  return h
}

const tex3 = (data: Float32Array, n: number): THREE.Data3DTexture => {
  const t = new THREE.Data3DTexture(half(data), n, n, n)
  t.format = THREE.RedFormat
  t.type = THREE.HalfFloatType
  t.wrapS = t.wrapT = t.wrapR = THREE.RepeatWrapping
  t.magFilter = THREE.LinearFilter
  t.minFilter = THREE.LinearMipmapLinearFilter
  t.generateMipmaps = true
  t.unpackAlignment = 2
  t.needsUpdate = true
  return t
}

// ---------------------------------------------------------------------------------------------------
// shaders

const VERT = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_vertex>
  varying vec3 vW;
  void main(){ vec4 w = modelMatrix * vec4(position, 1.0); vW = w.xyz; gl_Position = projectionMatrix * viewMatrix * w;
    #include <logdepthbuf_vertex>
  }`

/** profile / erosion constants shared by the shaders and the CPU probe */
const K = { topBand: 0.25, baseSoft: 0.03, baseLift: 0.1, edgeBand: 0.5, sharp: 4.0, sigma: 0.05, hMin: 0.22, lean: 250, wobble: 90, covGain: 1.3, detail: 0.3, filterLen: 120 }

/** density model shared by the deck and the ground-shadow shader (mirrored by makeSunProbe) */
const DENSITY = /* glsl */ `
  uniform sampler2D uWeather;
  uniform highp sampler3D uShape;
  uniform highp sampler3D uDetail;
  uniform vec2 uDrift, uLean;
  uniform float uTime, uCov, uBase, uTop, uExtent, uSigma, uYLo;
  const float CURV = ${(0.5 / 6371e3).toExponential(6)}; // 1 / (2 R_E): the deck follows the Earth's curvature
  const float WEATHER_TILE = ${WEATHER_TILE.toFixed(1)}, SHAPE_TILE = ${SHAPE_TILE.toFixed(1)}, DETAIL_TILE = ${DETAIL_TILE.toFixed(1)};
  const float WEATHER_TEXEL = ${(WEATHER_TILE / WEATHER_N).toFixed(3)}, SHAPE_TEXEL = ${(SHAPE_TILE / SHAPE_N).toFixed(3)}, DETAIL_TEXEL = ${(DETAIL_TILE / DETAIL_N).toFixed(3)};
  float remap(float x, float a, float b, float c, float d){ return c + (x - a) * (d - c) / (b - a); }

  // field coordinates of a pad-frame point: wind drift, towers leaning downwind with height (shear) and
  // a slow sideways wobble so that the walls are not vertical extrusions of the weather map
  vec2 fieldXZ(vec2 xz, float h){
    // the field origin is offset from the pad: a texture-tile boundary at the camera/pad would show
    // as a faint mip/derivative seam up the middle of the deck
    vec2 f = xz - uDrift - uLean * h + vec2(11300.0, 7300.0);
    return f + vec2(sin(f.y * 0.0011 + h * 4.0), cos(f.x * 0.0013 + h * 3.3)) * (${K.wobble.toFixed(1)} * h);
  }

  // column at field position f: q = coverage strength (0 at the cloud edge, 1 at the core),
  // top / bottom of the cloud body as slab fractions (dome over the cell, base lifted at the rim)
  vec3 cloudColumn(vec2 f, vec2 xz, float lod){
    vec4 w = textureLod(uWeather, f * (1.0 / WEATHER_TILE), lod);
    float m = textureLod(uWeather, mat2(0.8, 0.6, -0.6, 0.8) * f * (1.0 / (WEATHER_TILE * 2.37)) + vec2(0.31, 0.67), lod).b;
    float r = length(xz);
    // a very low-frequency coverage field over a few hundred km: the 30 km weather tile is repeated
    // across the regional deck, and this breaks the visible tiling without a second texture fetch
    float macro = 0.74 + 0.26 * sin(f.x * 2.1e-5 + 1.3) * cos(f.y * 1.7e-5 - 0.7) + 0.10 * sin(f.x * 7.3e-5 + f.y * 5.1e-5);
    // tower height is a 7.5 km field: use it as a cluster gate so the 30 km weather tile reads as
    // streets/clusters instead of a uniform carpet of identical cumulus cells
    float gate = smoothstep(0.10, 0.82, w.g);
    float cov = uCov * mix(0.6, 1.4, m) * macro * (0.55 + 0.75 * gate) * (1.0 - smoothstep(0.55 * uExtent, 0.97 * uExtent, r));
    float q = clamp((w.r - (1.0 - cov)) / max(cov, 1e-3), 0.0, 1.0);
    float e = 1.0 - q;
    return vec3(q, mix(${K.hMin.toFixed(3)}, 1.0, w.g * w.g) * q * (2.0 - q), ${K.baseLift.toFixed(3)} * e * e * e);
  }

  // base cloud density 0..1 (no detail erosion); h = height fraction, occ = sky occlusion (0 at the
  // exposed top/rim, 1 deep under a thick core)
  float cloudBase(vec3 p, float lodW, float lodS, out float h, out float occ, out float env, out float sdf){
    h = (p.y + dot(p.xz, p.xz) * CURV - uBase) / (uTop - uBase);
    occ = 0.0; env = 0.0; sdf = -1.0;
    if (h <= 0.0 || h >= 1.0) return 0.0;
    vec2 f = fieldXZ(p.xz, h);
    vec3 col = cloudColumn(f, p.xz, lodW);
    if (col.x <= 0.0) return 0.0;
    float prof = clamp((col.y - h) * ${(1 / K.topBand).toFixed(4)}, 0.0, 1.0) * smoothstep(col.z, col.z + ${K.baseSoft.toFixed(3)}, h) * smoothstep(0.0, ${K.edgeBand.toFixed(3)}, col.x);
    if (prof <= 0.0) return 0.0;
    env = prof;
    occ = clamp((col.y - h) * 2.5, 0.0, 1.0) * smoothstep(0.0, 0.5, col.x);
    float n = textureLod(uShape, vec3(f.x, p.y - uTime * 0.4, f.y) * (1.0 / SHAPE_TILE), lodS).r;
    sdf = n - (1.0 - prof); // > 0 inside the eroded cloud (noise threshold against the profile)
    return clamp(sdf * ${K.sharp.toFixed(2)}, 0.0, 1.0);
  }
`

const FRAG = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_fragment>
  ${DENSITY}
  uniform vec3 uOrigin, uSun, uSunColor, uAmbient, uGround, uHaze;
  uniform float uPixAng, uSteps, uFade;
  varying vec3 vW;

  float hg(float mu, float g){ float g2 = g * g; return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0 * g * mu, 1e-4), 1.5)); }
  // 8×8 Bayer ordered dither in [0, 1): fixed per pixel, evenly spread start offsets
  float bayer2(vec2 a){ a = floor(a); return fract(dot(a, vec2(0.5, a.y * 0.75))); }
  float bayer8(vec2 a){ return (bayer2(0.25 * a) * 0.25 + bayer2(0.5 * a)) * 0.25 + bayer2(a); }

  // eroded cloud density 0..1: base field, detail erosion (Worley fBm: wispy at the base, billowy higher
  // up, faded out below the footprint), sharpened with the step-dependent factor shK
  float cloudDensity(vec3 p, float lodW, float lodS, float lodD, float kD, float shK, out float h, out float occ, out float env){
    float sd;
    cloudBase(p, lodW, lodS, h, occ, env, sd);
    if (sd <= 0.0) return 0.0;
    if (kD > 0.0) {
      vec2 f = fieldXZ(p.xz, h);
      float hf = textureLod(uDetail, vec3(f.x, p.y + uTime * 0.8, f.y) * (1.0 / DETAIL_TILE), lodD).r;
      sd -= mix(hf, 1.0 - hf, clamp(h * 8.0, 0.0, 1.0)) * ${K.detail.toFixed(3)} * kD;
    }
    return clamp(sd * shK, 0.0, 1.0);
  }

  // optical depth towards the Sun (6 samples, steps growing × 1.9: 35 m … 0.9 km, ≈ 1.8 km)
  float sunDepth(vec3 p, float lod){
    float tau = 0.0, s = 0.0, ds = 35.0, h, pr, ev, sd;
    for (int j = 0; j < 6; j++) {
      vec3 q = p + uSun * (s + 0.5 * ds);
      if (q.y > uTop) break;
      float l = lod + float(j) * 0.6;
      tau += cloudBase(q, max(l - 1.0, 0.0), l, h, pr, ev, sd) * ds;
      s += ds; ds *= 1.9;
    }
    return tau * uSigma;
  }

  void main(){
    #include <logdepthbuf_fragment>
    vec3 ro = cameraPosition - uOrigin;
    vec3 rd = normalize(vW - cameraPosition);
    // analytic box intersection in pad coordinates
    vec3 inv = 1.0 / (abs(rd) + 1e-9) * sign(rd + 1e-12);
    vec3 b0 = (vec3(-uExtent, uYLo, -uExtent) - ro) * inv, b1 = (vec3(uExtent, uTop, uExtent) - ro) * inv;
    vec3 tlo = min(b0, b1), thi = max(b0, b1);
    float tn = max(max(tlo.x, tlo.y), max(tlo.z, 0.0));
    float tf = min(min(thi.x, thi.y), thi.z);
    if (tf <= tn) discard;
    float L = tf - tn;
    // coarse samples u = 0..N on a geometric spacing (fine near the camera, coarser far away) that
    // always spans [tn, tf]; inside clouds the march backs up and refines to quarter steps
    float N = floor(uSteps * mix(1.0, 1.5, smoothstep(8000.0, 30000.0, L)));
    float g = 1.0 + 0.05 * smoothstep(1500.0, 16000.0, L);
    float s0 = g > 1.0001 ? L * (g - 1.0) / (pow(g, N) - 1.0) : L / N;
    float lg = g > 1.0001 ? log(g) / (g - 1.0) : 1.0;
    float jit = bayer8(gl_FragCoord.xy);
    float mu = dot(rd, uSun);
    // phases of the multiple-scattering octaves (constant along the ray)
    float ph0 = mix(hg(mu, 0.6), hg(mu, -0.3), 0.25);
    float ph1 = mix(hg(mu, 0.3), hg(mu, -0.15), 0.25);
    float ph2 = mix(hg(mu, 0.15), hg(mu, -0.075), 0.25);
    float ph3 = mix(hg(mu, 0.075), hg(mu, -0.0375), 0.25);
    float powderK = 0.7 * smoothstep(0.6, -0.6, mu); // dark thin edges only when looking away from the Sun
    vec3 Lsum = vec3(0.0);
    float T = 1.0, tw = 0.0, ww = 0.0, h, pr, ev;
    float u = jit, fineLeft = 0.0;
    bool prevEmpty = true;
    const float FINE = 0.25, COARSE = 0.5;
    for (int i = 0; i < 256; i++) {
      if (u >= N) break;
      float gu = pow(g, u);
      float t = tn + (g > 1.0001 ? s0 * (gu - 1.0) / (g - 1.0) : s0 * u);
      float dtc = s0 * gu * lg;
      bool fine = fineLeft > 0.0;
      float dt = dtc * (fine ? FINE : COARSE);
      vec3 p = ro + rd * t;
      // footprint (pixel cone, step length) → texture LODs: distant clouds blur instead of aliasing; the
      // density ramp at the surface widens with the step length (edge pre-filter)
      float fp = max(t * uPixAng, 0.35 * dt);
      float lodS = log2(max(fp / SHAPE_TEXEL, 1.0));
      float lodW = log2(max(fp / WEATHER_TEXEL, 1.0));
      float lodD = log2(max(fp / DETAIL_TEXEL, 1.0));
      float kD = 1.0 - smoothstep(1.5, 3.5, lodD);
      float shK = ${K.sharp.toFixed(2)} / (1.0 + dt * ${(1 / K.filterLen).toFixed(5)});
      float c = cloudDensity(p, lodW, lodS, lodD, kD, shK, h, pr, ev);
      // refinement is driven by the smooth envelope (not the eroded density): thin turrets between two
      // coarse samples are not missed, so cloud edges do not flicker from pixel to pixel
      if (!fine) {
        if (ev > 0.0) { fineLeft = 4.0; prevEmpty = true; u = max(u - COARSE + FINE, 0.0); } else u += COARSE;
        continue;
      }
      if (ev <= 0.0) { fineLeft -= 1.0; prevEmpty = true; u += FINE; continue; }
      fineLeft = 4.0;
      if (c <= 0.002) { prevEmpty = true; u += FINE; continue; }
      if (prevEmpty) {
        // bisect the cloud surface between the previous (empty) sample and this one and put the first
        // in-cloud sample half a step below it: the surface depth no longer follows the per-pixel jitter
        float lo = t - dt, hi = t, hh, oo, ee;
        for (int k = 0; k < 3; k++) {
          float m = 0.5 * (lo + hi);
          if (cloudDensity(ro + rd * m, lodW, lodS, lodD, kD, shK, hh, oo, ee) > 0.002) hi = m; else lo = m;
        }
        t = hi + 0.5 * dt;
        p = ro + rd * t;
        c = cloudDensity(p, lodW, lodS, lodD, kD, shK, h, pr, ev);
        u = g > 1.0001 ? log(1.0 + (t - tn) * (g - 1.0) / s0) / log(g) : (t - tn) / s0;
      }
      prevEmpty = false;
      u += FINE;
      if (c <= 0.002) continue;
      float sig = c * uSigma;
      // light: Sun (multiple-scattering octaves, powder), sky ambient, ground bounce
      float tauL = sunDepth(p, lodS);
      float ms = ph0 * exp(-tauL) + 0.6 * ph1 * exp(-0.4 * tauL) + 0.36 * ph2 * exp(-0.16 * tauL) + 0.216 * ph3 * exp(-0.064 * tauL);
      float powder = mix(1.0, 1.0 - exp(-sig * 90.0), powderK);
      vec3 S = uSunColor * (ms * powder)
             + uAmbient * (1.0 - 0.8 * pr)
             + uGround * (1.0 - h) * (1.0 - h);
      float Ti = exp(-sig * dt);
      float a = T * (1.0 - Ti);
      Lsum += S * a;
      tw += t * a; ww += a;
      T *= Ti;
      if (T < 0.015) break;
    }
    if (ww <= 0.0) discard;
    // aerial perspective: air column between camera and cloud (exponential atmosphere, H = 8 km)
    float tm = tw / ww;
    float y0 = max(ro.y, 0.0);
    float k = rd.y * tm / 8000.0;
    float colm = tm * exp(-y0 / 8000.0) * (abs(k) < 1e-3 ? 1.0 - 0.5 * k : (1.0 - exp(-k)) / k);
    float aer = exp(-colm / 30000.0);
    vec3 haze = uHaze * (1.0 + 2.5 * hg(mu, 0.7));
    float alpha = (1.0 - T) * uFade;
    vec3 outc = (Lsum * aer + haze * alpha * (1.0 - aer)) * uFade;
    if (alpha < 0.002) discard;
    // soft knee above luminance 1.6 (→ ≤ 2.3): silver linings stay below the bloom threshold
    float Y = dot(outc, vec3(0.2126, 0.7152, 0.0722)) / max(alpha, 1e-3);
    if (Y > 1.6) outc *= (1.6 + 0.7 * (1.0 - exp(-(Y - 1.6) / 0.7))) / Y;
    gl_FragColor = vec4(outc, alpha);
  }`

const CIRRUS_VERT = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_vertex>
  varying vec3 vW;
  void main(){ vec4 w = modelMatrix * vec4(position, 1.0); vW = w.xyz; gl_Position = projectionMatrix * viewMatrix * w;
    #include <logdepthbuf_vertex>
  }`

/** thin high-cloud veil above the cumulus: 2-D stretched noise, wind-advected, day/night lit */
const CIRRUS_FRAG = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_fragment>
  uniform sampler2D uCirrus;
  uniform vec3 uOrigin, uSun, uSunColor, uAmbient;
  uniform vec2 uDrift;
  uniform float uTime, uExtent, uFade, uCov;
  varying vec3 vW;
  const float TILE = ${CIRRUS_TILE.toFixed(1)};

  void main(){
    #include <logdepthbuf_fragment>
    vec2 rel = vW.xz - uOrigin.xz;
    float r = length(rel);
    vec2 p = rel + vec2(17000.0, -9000.0);
    float edge = 1.0 - smoothstep(0.55 * uExtent, 0.97 * uExtent, r);
    if (edge <= 0.0) discard;
    // two scales of the same veil, moving at slightly different speeds: a thin sheet with internal
    // parallax instead of a frozen decal
    float n1 = texture2D(uCirrus, (p - uDrift * 1.6) / TILE + vec2(uTime * 0.00035, 0.0)).r;
    float n2 = texture2D(uCirrus, (p - uDrift * 2.3 + vec2(1700.0, -900.0)) / (TILE * 0.57)).r;
    float m = clamp(n1 * 0.72 + n2 * 0.42 - 0.06, 0.0, 1.0);
    float a = smoothstep(0.40, 0.76, m) * 0.38 * edge * uFade * (0.65 + 0.35 * uCov);
    if (a < 0.004) discard;
    float sy = max(uSun.y, 0.0);
    vec3 col = uSunColor * (0.10 + 0.42 * sy) + uAmbient * 0.9;
    col *= mix(vec3(1.08, 0.86, 0.80), vec3(1.0), smoothstep(0.0, 0.28, sy));
    gl_FragColor = vec4(col * a, a);
  }`

const SHADOW_FRAG = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_fragment>
  ${DENSITY}
  uniform vec3 uOrigin, uSun;
  uniform float uFloor, uFade;
  varying vec3 vW;
  void main(){
    #include <logdepthbuf_fragment>
    if (uSun.y < 0.02 || uFloor >= 0.999) discard;
    vec3 p = vW - uOrigin;
    float ta = (uBase - p.y) / uSun.y, tb = (uTop - p.y) / uSun.y;
    float ds = (tb - ta) / 10.0, tau = 0.0, h, pr, ev, sd;
    float lod = log2(max(ds * 0.25 / SHAPE_TEXEL, 1.0));
    for (int j = 0; j < 10; j++) tau += cloudBase(p + uSun * (ta + ds * (float(j) + 0.5)), lod, lod, h, pr, ev, sd) * ds;
    // forward scattering lets ≈ 1/3 of the extinction through as diffuse light: soften with τ/3
    float T = exp(-tau * uSigma * 0.33);
    float k = mix(uFloor, 1.0, T);
    // the shadow is part of the local deck: it fades with the deck during the hand-off
    k = mix(1.0, k, uFade);
    if (k > 0.998) discard;
    gl_FragColor = vec4(vec3(k), 1.0);
  }`

// ---------------------------------------------------------------------------------------------------

/**
 * Radiance calibration against the app's sky (env.ts makeAtmosphere, single scattering, Sun 18): its
 * cos-weighted mean upper-sky radiance at sea level is ≈ (0.16, 0.27, 0.42) at 35° Sun elevation, i.e. a
 * physically plausible sky/Sun ratio. The octave series under-counts high-order scattering, so the Sun
 * term carries a gain that brings sunlit cloud faces to ≈ 1.0–1.5 (≈ 4–6× the sky, white after ACES,
 * below the bloom threshold 2.5); a soft knee above 1.6 keeps silver linings from blooming.
 */
const SUN_GAIN = 6.0

/**
 * Fair-weather cumulus deck with a thin cirrus veil. `coverage` = area fraction of cloud bases (0..1,
 * default 0.35), `base`/`top` in metres above sea level (1500 / 3200), `extent` = half-width of the
 * deck around the pad at low altitude (60 km, soft radial fade from 55 % to 97 %; the deck follows the
 * Earth's curvature, so it stays at constant altitude out to the rim). The covered region can grow to
 * `maxExtent` (240 km) at altitude, where the per-frame `extent` grows and the shader LOD pre-filter
 * keeps the ray march cheap; the pad-scale weather/detail tiles are unchanged. `fade` (0..1) fades the
 * deck, its ground shadow and the cirrus out during the hand-off to the global shell. Add `mesh` to
 * the scene and call `update` every frame.
 * After `update`, `mesh.userData.sunTransmittance` is the cloud transmittance of direct sunlight at
 * the world origin (the ship), e.g. to dim the shadow-casting DirectionalLight in a cloud shadow.
 */
export const makeClouds = (opts?: { coverage?: number; base?: number; top?: number; extent?: number; maxExtent?: number }): CloudDeck => {
  const coverage = opts?.coverage ?? 0.35, base = opts?.base ?? 1500, top = opts?.top ?? 3200, extent = opts?.extent ?? 60000
  const maxExtent = Math.max(extent, opts?.maxExtent ?? 240000)
  // Keep the box bottom above the ground: the deck is drawn BackSide, so from above the visible
  // fragment is the inside of the bottom face. If that face is below the sea/coast plane (which
  // writes depth) the whole deck is occluded — the old fixed extent got away with it; a 240 km
  // regional box would not. The far corners of the deck follow the Earth's curvature and fade out
  // radially before the reduced box height could cut them visually.
  const yLo = Math.max(100, base - 800)
  const noise = bakeNoise(20261121)
  const weatherTex = new THREE.DataTexture(half(noise.weather), WEATHER_N, WEATHER_N, THREE.RGBAFormat, THREE.HalfFloatType)
  weatherTex.wrapS = weatherTex.wrapT = THREE.RepeatWrapping
  weatherTex.magFilter = THREE.LinearFilter
  weatherTex.minFilter = THREE.LinearMipmapLinearFilter
  weatherTex.generateMipmaps = true
  weatherTex.needsUpdate = true
  const shapeTex = tex3(noise.shape, SHAPE_N)
  const detailTex = tex3(noise.detail, DETAIL_N)
  const shared = {
    uWeather: { value: weatherTex }, uShape: { value: shapeTex }, uDetail: { value: detailTex },
    uDrift: { value: new THREE.Vector2() }, uLean: { value: new THREE.Vector2() }, uTime: { value: 0 }, uCov: { value: Math.min(1, coverage * K.covGain) },
    uBase: { value: base }, uTop: { value: top }, uExtent: { value: extent }, uSigma: { value: K.sigma }, uYLo: { value: yLo },
    uOrigin: { value: new THREE.Vector3() }, uSun: { value: new THREE.Vector3(0, 1, 0) }, uFade: { value: 1 },
  }
  const uniforms = {
    ...shared,
    uSunColor: { value: new THREE.Color() }, uAmbient: { value: new THREE.Color() },
    uGround: { value: new THREE.Color() }, uHaze: { value: new THREE.Color() },
    uPixAng: { value: 1e-3 }, uSteps: { value: 56 },
  }
  const mat = new THREE.ShaderMaterial({
    uniforms, vertexShader: VERT, fragmentShader: FRAG, side: THREE.BackSide,
    transparent: true, depthWrite: false, depthTest: true,
    blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
  })
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(2 * maxExtent, top - yLo, 2 * maxExtent), mat)
  mesh.renderOrder = -1
  mesh.frustumCulled = false
  mesh.name = "cloud-deck"
  // ground shadows: multiplies the radiance of whatever lies at sea level under the deck
  const shadowUniforms = { ...shared, uFloor: { value: 0.35 } }
  const shadowMat = new THREE.ShaderMaterial({
    uniforms: shadowUniforms, vertexShader: VERT, fragmentShader: SHADOW_FRAG,
    transparent: true, depthWrite: false, depthTest: true,
    blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.ZeroFactor, blendDst: THREE.SrcColorFactor,
  })
  const shadow = new THREE.Mesh(new THREE.CircleGeometry(maxExtent * 0.97, 96), shadowMat)
  shadow.rotation.x = -Math.PI / 2
  shadow.position.y = -(yLo + top) / 2 + 0.5
  shadow.renderOrder = -2
  shadow.frustumCulled = false
  shadow.name = "cloud-shadows"
  mesh.add(shadow)
  // thin cirrus/high layer: a single 2-D sheet above the cumulus (cheap: it is not ray-marched)
  const cirrusTex = new THREE.DataTexture(new Uint8Array(noise.cirrus.map((v) => Math.round(v * 255))), CIRRUS_N, CIRRUS_N, THREE.RedFormat, THREE.UnsignedByteType)
  cirrusTex.wrapS = cirrusTex.wrapT = THREE.RepeatWrapping
  cirrusTex.magFilter = THREE.LinearFilter
  cirrusTex.minFilter = THREE.LinearMipmapLinearFilter
  cirrusTex.generateMipmaps = true
  cirrusTex.needsUpdate = true
  const cirrusMat = new THREE.ShaderMaterial({
    uniforms: {
      uCirrus: { value: cirrusTex }, uOrigin: shared.uOrigin, uSun: shared.uSun, uDrift: shared.uDrift,
      uTime: shared.uTime, uExtent: shared.uExtent, uFade: shared.uFade, uCov: shared.uCov,
      uSunColor: uniforms.uSunColor, uAmbient: uniforms.uAmbient,
    },
    vertexShader: CIRRUS_VERT, fragmentShader: CIRRUS_FRAG,
    transparent: true, depthWrite: false, depthTest: true, side: THREE.DoubleSide,
    blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
  })
  const cirrus = new THREE.Mesh(new THREE.PlaneGeometry(2 * maxExtent, 2 * maxExtent), cirrusMat)
  cirrus.rotation.x = -Math.PI / 2
  cirrus.position.y = 9000 - (yLo + top) / 2
  cirrus.renderOrder = -1
  cirrus.frustumCulled = false
  cirrus.name = "cloud-cirrus"
  mesh.add(cirrus)
  const size = new THREE.Vector2()
  mesh.onBeforeRender = (renderer, _scene, camera) => {
    const rt = renderer.getRenderTarget()
    const hpx = rt ? rt.height : renderer.getDrawingBufferSize(size).y
    uniforms.uPixAng.value = 2 / (Math.abs(camera.projectionMatrix.elements[5]) * Math.max(hpx, 1))
  }
  const lum = (c: THREE.Color): number => 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b
  const probe = makeSunProbe(noise, base, top, extent, Math.min(1, coverage * K.covGain))
  const update = (f: CloudFrame): void => {
    mesh.position.set(f.origin.x, f.origin.y + (yLo + top) / 2, f.origin.z)
    shared.uOrigin.value.copy(f.origin)
    shared.uSun.value.copy(f.sunDir).normalize()
    shared.uTime.value = f.time
    shared.uExtent.value = Math.max(1000, Math.min(maxExtent, f.extent ?? extent))
    shared.uFade.value = Math.max(0, Math.min(1, f.fade ?? 1))
    shared.uDrift.value.set(f.wind[0] * f.time, -f.wind[1] * f.time)
    // shear lean: tops displaced downwind (≈ 250 m at the top for 10 m/s)
    const ws = Math.hypot(f.wind[0], f.wind[1])
    const lean = (K.lean * Math.min(ws, 15)) / 10 / Math.max(ws, 1e-6)
    shared.uLean.value.set(f.wind[0] * lean, -f.wind[1] * lean)
    const e = Math.max(0, Math.min(1, f.eclipse))
    const sun = uniforms.uSunColor.value.copy(f.sunColor).multiplyScalar(e)
    const sk = f.skyColor, l = lum(sk)
    // ambient: sky light after multiple scattering in the cloud (less saturated than the sky dome).
    // slightly above the single-scattering mean: the 4-octave series under-counts high orders and the
    // bases otherwise read as dark grey boxes under a bright top
    uniforms.uAmbient.value.setRGB(0.5 * sk.r + 0.5 * l, 0.5 * sk.g + 0.5 * l, 0.5 * sk.b + 0.5 * l).multiplyScalar(0.72)
    const sy = Math.max(0, shared.uSun.value.y)
    // ground bounce under the bases: Gulf water + coastal scrub, albedo ≈ 0.1, Lambertian
    uniforms.uGround.value.setRGB(0.07, 0.065, 0.05).multiplyScalar(lum(sun) * sy + Math.PI * l * 0.3)
    // aerial-perspective colour: horizon sky (desaturated, brighter than the mean sky) + a little sunlight
    uniforms.uHaze.value.setRGB(0.35 * sk.r + 0.65 * l, 0.35 * sk.g + 0.65 * l, 0.35 * sk.b + 0.65 * l).multiplyScalar(2.4)
      .add(new THREE.Color().copy(sun).multiplyScalar(0.02))
    sun.multiplyScalar(SUN_GAIN)
    // cloud-shadow floor: fraction of the ground irradiance that is diffuse sky light
    const eSun = lum(f.sunColor) * e * sy, eSky = Math.PI * l
    shadowUniforms.uFloor.value = eSun + eSky > 1e-6 ? Math.min(1, Math.max(0.52, (1.4 * eSky) / (eSun + eSky))) : 1
    mesh.userData.sunTransmittance = shared.uFade.value > 0.01 ? probe(f, shared.uSun.value, shared.uLean.value) : 1
  }
  return { mesh, update }
}

/**
 * CPU probe of the cloud transmittance towards the Sun at the world origin (the ship). Mirrors the
 * shader's cloudBase() at LOD 0 (bilinear weather, trilinear shape), 16 samples through the slab.
 */
const makeSunProbe = (noise: CloudNoise, base: number, top: number, extent: number, cov0: number) => {
  const W = noise.weather, S = noise.shape, NW = WEATHER_N, NS = SHAPE_N
  const md = (i: number, n: number): number => ((i % n) + n) % n
  const bil = (u: number, v: number, ch: number): number => {
    const x = u * NW - 0.5, y = v * NW - 0.5
    const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0
    const at = (i: number, j: number): number => W[(md(j, NW) * NW + md(i, NW)) * 4 + ch]
    return (at(x0, y0) * (1 - fx) + at(x0 + 1, y0) * fx) * (1 - fy) + (at(x0, y0 + 1) * (1 - fx) + at(x0 + 1, y0 + 1) * fx) * fy
  }
  const tri = (u: number, v: number, w: number): number => {
    const x = u * NS - 0.5, y = v * NS - 0.5, z = w * NS - 0.5
    const x0 = Math.floor(x), y0 = Math.floor(y), z0 = Math.floor(z), fx = x - x0, fy = y - y0, fz = z - z0
    const at = (i: number, j: number, k: number): number => S[(md(k, NS) * NS + md(j, NS)) * NS + md(i, NS)]
    const l = (j: number, k: number): number => at(x0, j, k) * (1 - fx) + at(x0 + 1, j, k) * fx
    return (l(y0, z0) * (1 - fy) + l(y0 + 1, z0) * fy) * (1 - fz) + (l(y0, z0 + 1) * (1 - fy) + l(y0 + 1, z0 + 1) * fy) * fz
  }
  const clamp01 = (x: number): number => Math.max(0, Math.min(1, x))
  const smooth = (a: number, b: number, x: number): number => { const t = clamp01((x - a) / (b - a)); return t * t * (3 - 2 * t) }
  const density = (px: number, py: number, pz: number, dx: number, dz: number, lx: number, lz: number, time: number): number => {
    const h = (py + (px * px + pz * pz) * (0.5 / 6371e3) - base) / (top - base)
    if (h <= 0 || h >= 1) return 0
    const ax = px - dx - lx * h + 11300, az = pz - dz - lz * h + 7300
    const fx = ax + Math.sin(az * 0.0011 + h * 4) * K.wobble * h, fz = az + Math.cos(ax * 0.0013 + h * 3.3) * K.wobble * h
    const w0 = bil(fx / WEATHER_TILE, fz / WEATHER_TILE, 0), w1 = bil(fx / WEATHER_TILE, fz / WEATHER_TILE, 1)
    const gx = (0.8 * fx - 0.6 * fz) / (WEATHER_TILE * 2.37) + 0.31, gz = (0.6 * fx + 0.8 * fz) / (WEATHER_TILE * 2.37) + 0.67
    const macro = 0.74 + 0.26 * Math.sin(fx * 2.1e-5 + 1.3) * Math.cos(fz * 1.7e-5 - 0.7) + 0.10 * Math.sin(fx * 7.3e-5 + fz * 5.1e-5)
    const gate = smooth(0.10, 0.82, w1)
    const cov = cov0 * (0.6 + 0.8 * bil(gx, gz, 2)) * macro * (0.55 + 0.75 * gate) * (1 - smooth(0.55 * extent, 0.97 * extent, Math.hypot(px, pz)))
    const q = clamp01((w0 - (1 - cov)) / Math.max(cov, 1e-3))
    if (q <= 0) return 0
    const e = 1 - q, colTop = (K.hMin + (1 - K.hMin) * w1 * w1) * q * (2 - q), colBot = K.baseLift * e * e * e
    const prof = clamp01((colTop - h) / K.topBand) * smooth(colBot, colBot + K.baseSoft, h) * smooth(0, K.edgeBand, q)
    if (prof <= 0) return 0
    const n = tri(fx / SHAPE_TILE, (py - time * 0.4) / SHAPE_TILE, fz / SHAPE_TILE)
    return clamp01((n - (1 - prof)) * K.sharp)
  }
  return (f: CloudFrame, sun: THREE.Vector3, lean: THREE.Vector2): number => {
    if (sun.y < 0.02) return 1
    // ship at the world origin → pad frame
    const ox = -f.origin.x, oy = -f.origin.y, oz = -f.origin.z
    const ta = Math.max(0, (base - 600 - oy) / sun.y), tb = (top - oy) / sun.y
    if (tb <= ta) return 1
    const dx = f.wind[0] * f.time, dz = -f.wind[1] * f.time
    const n = 20, ds = (tb - ta) / n
    let tau = 0
    for (let j = 0; j < n; j++) {
      const t = ta + ds * (j + 0.5)
      tau += density(ox + sun.x * t, oy + sun.y * t, oz + sun.z * t, dx, dz, lean.x, lean.y, f.time) * ds
    }
    return Math.exp(-tau * K.sigma)
  }
}
