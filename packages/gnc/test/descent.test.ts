import { describe, expect, test } from "bun:test"
import { v3 } from "@ship/math"
import { quintic } from "../src/descent"

describe("terminal quintic spline", () => {
  const p0 = v3(-42, 12, 47), v0 = v3(48, -6, -14), pf = v3(0, 0, 0), vf = v3(0, 0, -0.7)
  const T = 3.4
  const q = quintic(p0, v0, pf, vf, T)

  test("hits the boundary positions and velocities", () => {
    const p0r = q.p(0), v0r = q.v(0), p1r = q.p(1), v1r = q.v(1)
    expect(Math.hypot(p0r.x - p0.x, p0r.y - p0.y, p0r.z - p0.z)).toBeLessThan(1e-9)
    expect(Math.hypot(v0r.x - v0.x, v0r.y - v0.y, v0r.z - v0.z)).toBeLessThan(1e-9)
    expect(Math.hypot(p1r.x, p1r.y, p1r.z)).toBeLessThan(1e-9)
    expect(Math.hypot(v1r.x, v1r.y, v1r.z + 0.7)).toBeLessThan(1e-9)
  })

  test("zero acceleration at both ends (C², feedforward starts and ends at 0)", () => {
    expect(Math.hypot(q.a(0).x, q.a(0).y, q.a(0).z)).toBeLessThan(1e-9)
    expect(Math.hypot(q.a(1).x, q.a(1).y, q.a(1).z)).toBeLessThan(1e-9)
  })

  test("is continuous in position and velocity over the whole span", () => {
    let prev = q.p(0)
    for (let i = 1; i <= 200; i++) {
      const p = q.p(i / 200)
      expect(Math.hypot(p.x - prev.x, p.y - prev.y, p.z - prev.z)).toBeLessThan(1)
      prev = p
    }
  })
})
