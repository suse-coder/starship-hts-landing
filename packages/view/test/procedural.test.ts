/** CPU bakes behind the ocean and ground shaders (packages/view/src/procedural.ts). */
import { describe, expect, test } from "bun:test"
import { bakeTerrainNoise, bakeWaves, equalise, rng } from "../src/procedural"

describe("rng", () => {
  test("deterministic, uniform in [0, 1)", () => {
    const a = rng(5), b = rng(5)
    let s = 0
    for (let i = 0; i < 20000; i++) { const x = a(); expect(x).toBe(b()); expect(x).toBeGreaterThanOrEqual(0); expect(x).toBeLessThan(1); s += x }
    expect(s / 20000).toBeCloseTo(0.5, 1)
  })
})

describe("bakeWaves", () => {
  const n = 64
  const w = bakeWaves(n, 7, 32)
  test("unit total slope variance, unit height variance, zero means (integer wave vectors)", () => {
    let vs = 0, mx = 0, my = 0, mh = 0, vh = 0
    for (let p = 0; p < n * n; p++) { const x = w.data[p * 4], y = w.data[p * 4 + 1], h = w.data[p * 4 + 2]; vs += x * x + y * y; mx += x; my += y; mh += h; vh += h * h }
    expect(vs / (n * n)).toBeCloseTo(1, 4)
    expect(vh / (n * n)).toBeCloseTo(1, 4)
    expect(Math.abs(mx / (n * n))).toBeLessThan(1e-3)
    expect(Math.abs(my / (n * n))).toBeLessThan(1e-3)
    expect(Math.abs(mh / (n * n))).toBeLessThan(1e-3)
  })
  test("waves travel along +x: the slope variance is larger along x than across", () => {
    let vx = 0, vy = 0
    for (let p = 0; p < n * n; p++) { vx += w.data[p * 4] ** 2; vy += w.data[p * 4 + 1] ** 2 }
    expect(vx).toBeGreaterThan(vy)
  })
  test("tileable: the field is as smooth across the wrap as inside the tile", () => {
    let inner = 0, wrap = 0
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n - 1; i++) inner += Math.abs(w.data[(j * n + i + 1) * 4 + 2] - w.data[(j * n + i) * 4 + 2])
      wrap += Math.abs(w.data[(j * n) * 4 + 2] - w.data[(j * n + n - 1) * 4 + 2])
    }
    expect(wrap / n).toBeLessThan(2.5 * (inner / (n * (n - 1))))
  })
  test("retained variance per mip level: 1 at the base, decreasing, nothing left at the 1×1 level", () => {
    expect(w.retained.length).toBe(Math.log2(n) + 1)
    expect(w.retained[0]).toBe(1)
    for (let l = 1; l < w.retained.length; l++) expect(w.retained[l]).toBeLessThanOrEqual(w.retained[l - 1] + 1e-6)
    expect(w.retained[w.retained.length - 1]).toBeLessThan(1e-6)
    expect(w.retained[1]).toBeLessThan(1)
  })
  test("deterministic for a seed, different for another", () => {
    const a = bakeWaves(32, 3, 16), b = bakeWaves(32, 3, 16), c = bakeWaves(32, 4, 16)
    expect([...a.data.subarray(0, 64)]).toEqual([...b.data.subarray(0, 64)])
    expect([...a.data.subarray(0, 64)]).not.toEqual([...c.data.subarray(0, 64)])
  })
  test("the default bake fits the water shader: 256 texels, 9 mip levels, all finite", () => {
    const d = bakeWaves()
    expect(d.n).toBe(256)
    expect(d.retained.length).toBe(9)
    for (let i = 0; i < d.data.length; i += 997) expect(Number.isFinite(d.data[i])).toBe(true)
    // half-float storage: the slopes must be far inside ±65504 and resolvable (|s| ≲ 5 σ)
    let mx = 0
    for (let i = 0; i < d.data.length; i++) if ((i & 3) < 3) mx = Math.max(mx, Math.abs(d.data[i]))
    expect(mx).toBeLessThan(8)
  })
})

describe("equalise", () => {
  test("maps any distribution onto a uniform one, preserving order", () => {
    const r = rng(1), a = new Float32Array(20000)
    for (let i = 0; i < a.length; i++) a[i] = Math.exp(3 * r()) // skewed
    const b = Float32Array.from(a)
    equalise(b)
    let below = 0
    for (let i = 0; i < b.length; i++) { expect(b[i]).toBeGreaterThanOrEqual(0); expect(b[i]).toBeLessThanOrEqual(1); if (b[i] < 0.25) below++ }
    expect(below / b.length).toBeCloseTo(0.25, 1)
    expect(a[10] < a[20]).toBe(b[10] < b[20])
  })
})

describe("bakeTerrainNoise", () => {
  const n = 128
  const t = bakeTerrainNoise(n, 11)
  test("four channels, each uniform: a threshold is an area fraction", () => {
    expect(t.length).toBe(n * n * 4)
    for (let c = 0; c < 4; c++) {
      let s = 0, q = 0
      for (let i = 0; i < n * n; i++) { s += t[i * 4 + c]; if (t[i * 4 + c] < 64) q++ }
      expect(s / (n * n) / 255).toBeCloseTo(0.5, 1)
      expect(q / (n * n)).toBeCloseTo(0.25, 1)
    }
  })
  test("channels are independent fields", () => {
    let same = 0
    for (let i = 0; i < n * n; i++) if (Math.abs(t[i * 4] - t[i * 4 + 1]) < 8) same++
    expect(same / (n * n)).toBeLessThan(0.2)
  })
  test("tileable in both directions", () => {
    for (let c = 0; c < 4; c++) {
      let inner = 0, wrapX = 0, wrapY = 0
      for (let j = 0; j < n; j++) {
        for (let i = 0; i < n - 1; i++) inner += Math.abs(t[(j * n + i + 1) * 4 + c] - t[(j * n + i) * 4 + c])
        wrapX += Math.abs(t[(j * n) * 4 + c] - t[(j * n + n - 1) * 4 + c])
        wrapY += Math.abs(t[j * 4 + c] - t[((n - 1) * n + j) * 4 + c])
      }
      const mean = inner / (n * (n - 1))
      expect(wrapX / n).toBeLessThan(2.5 * mean)
      expect(wrapY / n).toBeLessThan(2.5 * mean)
    }
  })
  test("deterministic", () => {
    expect([...bakeTerrainNoise(32, 2).subarray(0, 256)]).toEqual([...bakeTerrainNoise(32, 2).subarray(0, 256)])
  })
})
