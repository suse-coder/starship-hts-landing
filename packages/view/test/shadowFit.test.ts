/**
 * Fitted shadow frustum (packages/view/src/shadowFit.ts): the frustum must contain what it claims,
 * and its centre must stay on the pad-anchored texel grid while the world slides past the ship —
 * that invariant is what keeps the tower's shadows from crawling during the descent.
 */
import { describe, expect, test } from "bun:test"
import * as THREE from "three"
import { fitShadow, lightAxes, makeShadowFit, quantiseHalf, SHADOW_FIT } from "../src/shadowFit"

const unit = (x: number, y: number, z: number): [number, number, number] => { const l = Math.hypot(x, y, z); return [x / l, y / l, z / l] }
const SUNS: [number, number, number][] = [unit(0.5, 0.7, 0.3), unit(-0.8, 0.2, 0.1), unit(0.1, 0.05, -0.9), unit(0, 1, 0), unit(1e-9, 1, 0), unit(0.3, -0.2, 0.6)]

describe("quantiseHalf", () => {
  test("√2 ladder: the smallest rung that covers r, clamped at both ends", () => {
    expect(quantiseHalf(10, 64, 362.04)).toBe(64)
    expect(quantiseHalf(64, 64, 362.04)).toBe(64)
    expect(quantiseHalf(65, 64, 362.04)).toBeCloseTo(64 * Math.SQRT2, 9)
    expect(quantiseHalf(128, 64, 362.04)).toBeCloseTo(128, 9)
    expect(quantiseHalf(129, 64, 362.04)).toBeCloseTo(128 * Math.SQRT2, 9)
    expect(quantiseHalf(5000, 64, 362.04)).toBe(362.04)
    expect(quantiseHalf(NaN, 64, 362.04)).toBe(64)
    for (let r = 1; r < 360; r += 0.7) expect(quantiseHalf(r, 64, 362.04)).toBeGreaterThanOrEqual(Math.min(r, 362.04) - 1e-6)
  })
})

describe("lightAxes", () => {
  test("orthonormal, perpendicular to the Sun, and identical to three.js lookAt (the shadow camera's axes)", () => {
    const ax = new Float64Array(6)
    for (const [sx, sy, sz] of SUNS) {
      lightAxes(sx, sy, sz, ax)
      const r = new THREE.Vector3(ax[0], ax[1], ax[2]), u = new THREE.Vector3(ax[3], ax[4], ax[5]), s = new THREE.Vector3(sx, sy, sz)
      expect(r.length()).toBeCloseTo(1, 9)
      expect(u.length()).toBeCloseTo(1, 9)
      expect(Math.abs(r.dot(u))).toBeLessThan(1e-9)
      expect(Math.abs(r.dot(s))).toBeLessThan(1e-4)
      expect(Math.abs(u.dot(s))).toBeLessThan(1e-4)
      const m = new THREE.Matrix4().lookAt(s.clone().multiplyScalar(1500), new THREE.Vector3(), new THREE.Vector3(0, 1, 0))
      const mx = new THREE.Vector3().setFromMatrixColumn(m, 0), my = new THREE.Vector3().setFromMatrixColumn(m, 1)
      expect(mx.distanceTo(r)).toBeLessThan(1e-9)
      expect(my.distanceTo(u)).toBeLessThan(1e-9)
    }
  })
})

describe("fitShadow", () => {
  const N = 4096
  /** pad-relative coordinates of the frustum centre in texels along the light's right / up axes */
  const gridCoords = (f: ReturnType<typeof makeShadowFit>, sun: [number, number, number], pad: [number, number, number]): [number, number] => {
    const ax = new Float64Array(6)
    lightAxes(...sun, ax)
    const d = [f.cx - pad[0], f.cy - pad[1], f.cz - pad[2]]
    return [(d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2]) / f.texel, (d[0] * ax[3] + d[1] * ax[4] + d[2] * ax[5]) / f.texel]
  }
  test("far from the pad: the smallest frustum, around the ship", () => {
    const f = fitShadow(makeShadowFit(), ...SUNS[0], 3000, -20000, 500, false, N)
    expect(f.half).toBe(SHADOW_FIT.minHalf)
    expect(Math.hypot(f.cx, f.cy, f.cz)).toBeLessThan(f.texel) // within the snap of the origin
    expect(f.texel).toBeCloseTo((2 * f.half) / N, 12)
  })
  test("the light direction is exactly the Sun's (snapping only translates the camera)", () => {
    for (const sun of SUNS) {
      const f = fitShadow(makeShadowFit(), ...sun, 12.3, -180.7, -44.1, true, N)
      expect((f.px - f.cx) / SHADOW_FIT.distance).toBeCloseTo(sun[0], 9)
      expect((f.py - f.cy) / SHADOW_FIT.distance).toBeCloseTo(sun[1], 9)
      expect((f.pz - f.cz) / SHADOW_FIT.distance).toBeCloseTo(sun[2], 9)
    }
  })
  test("the centre sits on the pad-anchored texel grid for any pad position and Sun", () => {
    for (const sun of SUNS) for (const pad of [[10, -100, 5], [-37.123, -251.77, 88.4], [3.3, -1203.5, -7.7], [0.01, -96.37, 0.02]] as [number, number, number][]) {
      const f = fitShadow(makeShadowFit(), ...sun, ...pad, true, N)
      const [u, v] = gridCoords(f, sun, pad)
      expect(Math.abs(u - Math.round(u))).toBeLessThan(1e-6)
      expect(Math.abs(v - Math.round(v))).toBeLessThan(1e-6)
    }
  })
  test("a descending ship moves the frustum over the pad in whole texels only", () => {
    const sun = SUNS[0]
    const base = fitShadow(makeShadowFit(), ...sun, 4, -150, -6, true, N)
    const [u0, v0] = gridCoords(base, sun, [4, -150, -6])
    for (let k = 1; k <= 40; k++) {
      // the ship sinks 0.37 m and drifts sideways per frame; in the world frame the pad moves the other way
      const pad: [number, number, number] = [4 - 0.013 * k, -150 + 0.37 * k, -6 + 0.021 * k]
      const f = fitShadow(makeShadowFit(), ...sun, ...pad, true, N)
      if (f.half !== base.half) continue
      const [u, v] = gridCoords(f, sun, pad)
      expect(Math.abs(u - u0 - Math.round(u - u0))).toBeLessThan(1e-6)
      expect(Math.abs(v - v0 - Math.round(v - v0))).toBeLessThan(1e-6)
    }
  })
  test("contains the ship sphere always, and the tower sphere once the ship is within reach", () => {
    const ax = new Float64Array(6)
    const inside = (f: ReturnType<typeof makeShadowFit>, sun: [number, number, number], p: [number, number, number], r: number): boolean => {
      lightAxes(...sun, ax)
      const d = [p[0] - f.cx, p[1] - f.cy, p[2] - f.cz]
      const u = d[0] * ax[0] + d[1] * ax[1] + d[2] * ax[2], v = d[0] * ax[3] + d[1] * ax[4] + d[2] * ax[5], w = d[0] * sun[0] + d[1] * sun[1] + d[2] * sun[2]
      const depth = SHADOW_FIT.distance - w // distance from the shadow camera along its view axis
      return Math.abs(u) + r <= f.half + f.texel && Math.abs(v) + r <= f.half + f.texel && depth - r >= f.near && depth + r <= f.far
    }
    for (const sun of SUNS) for (const up of [20, 80, 150, 250, 320, 390, 600, 5000]) {
      const pad: [number, number, number] = [15, -up, -8]
      const f = fitShadow(makeShadowFit(), ...sun, ...pad, true, N)
      expect(inside(f, sun, [0, 0, 0], SHADOW_FIT.shipRadius)).toBe(true)
      const tower: [number, number, number] = [pad[0], pad[1] + SHADOW_FIT.towerHeight, pad[2]]
      const d = Math.hypot(...tower)
      if (d < SHADOW_FIT.mergeDistance) expect(inside(f, sun, tower, SHADOW_FIT.towerRadius)).toBe(true)
      else expect(f.half).toBe(SHADOW_FIT.minHalf)
      expect(f.half).toBeLessThanOrEqual(SHADOW_FIT.maxHalf)
      expect(f.near).toBeGreaterThan(0)
      expect(f.far).toBeGreaterThan(f.near)
    }
  })
  test("the pad is ignored while the tower is not drawn", () => {
    const f = fitShadow(makeShadowFit(), ...SUNS[0], 10, -60, 0, false, N)
    expect(f.half).toBe(SHADOW_FIT.minHalf)
  })
  test("texel size at the catch is finer than the old fixed ±120 m / 2048 map", () => {
    const f = fitShadow(makeShadowFit(), ...SUNS[0], 6, -95, 3, true, N)
    expect(f.texel).toBeLessThan(240 / 2048)
  })
})
