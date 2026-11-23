import { describe, expect, it } from "bun:test"
import { v3 } from "@ship/math"
import { identifyPlant, landingRig, lateralOf, splineKnots, trackingOf } from "@ship/verify"
import type { LogSample } from "@ship/verify"

describe("landing rig", () => {
  it("places the ship upright, pins on the axis, engines at the trim that puts thrust through the CG", () => {
    const r = landingRig({ pin: v3(0, 0, 600), engines: [0], throttle: 0.5 })
    const s = r.state
    expect(s.mem.phase).toBe("landing")
    expect(s.eng[0].on).toBe(true)
    expect(Math.abs(s.eng[0].gp)).toBeGreaterThan(0.01) // the trim cant of the off-centre engine
    expect(s.cons.loxHeader + s.cons.ch4Header).toBeGreaterThan(10e3)
  })
})

describe("plant identification", () => {
  it("measures the pitch authority the model predicts (1 engine, floor and full thrust)", () => {
    for (const th of [0.4, 1.0]) {
      const p = identifyPlant([0], th)
      expect(Math.abs(p.accel - p.accelModel)).toBeLessThan(0.06 * p.accelModel + 0.1)
      expect(Math.abs(p.alpha - p.alphaModel)).toBeLessThan(0.08 * Math.abs(p.alphaModel) + 1e-4)
      expect(p.b).toBeGreaterThan(0.4)
    }
    const lo = identifyPlant([0], 0.4)
    const hi = identifyPlant([0], 1.0)
    // the gimbal's authority grows with thrust (b = T·L/I): that is what the loop must schedule
    expect(hi.b).toBeGreaterThan(2 * lo.b)
  })

  it("puts the catch pins near the centre of percussion (the gimbal's side force cancels there)", () => {
    const p = identifyPlant([0], 0.5)
    expect(Math.abs(p.pinLever - p.percussion)).toBeLessThan(2.5)
  })
})

describe("flight log geometry", () => {
  const mk = (t: number, x: number, y: number, z: number): LogSample => ({
    t, pin: { x, y, z }, vel: { x: 0, y: 0, z: -1 }, tilt: 2, rollRate: 0.1, attErr: 0.01,
    gp: 3, gy: 0, thrust: 1e6, u: { x: 0, y: 0, z: 9.8 }, plan: { x, y, z }, header: 1e4, arms: 1,
  })
  it("reports lateral error and plan deviation", () => {
    const s = [mk(0, 4, 0, 100), mk(1, 2, 0, 50), mk(2, 0, 0, 1)]
    expect(lateralOf(s)).toEqual({ end: 0, max: 4 })
    expect(trackingOf(s)?.max).toBe(0)
    expect(splineKnots(s, 3).map((k) => k.p.z)).toEqual([100, 50, 1])
  })
})
