import { describe, expect, it } from "bun:test"
import { v3 } from "@ship/math"
import { captureCase, checkTailFirstAero, identifyPlant, landingRig, lateralOf, PHASE_PRECISION, splineKnots, trackingOf } from "@ship/verify"
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

describe("plant identification (open-loop steps on the 6-DOF plant)", () => {
  it("one engine: thrust and pitch authority match T/m and T·L/I at the floor and at full thrust", () => {
    for (const th of [0.4, 1.0]) {
      const p = identifyPlant([0], th)
      expect(Math.abs(p.accel - p.accelModel)).toBeLessThan(0.03 * p.accelModel)
      expect(Math.abs(p.alpha - p.alphaModel)).toBeLessThan(0.03 * Math.abs(p.alphaModel))
    }
    // the gimbal's authority grows with the thrust (b = T·L/I)
    expect(identifyPlant([0], 1.0).b).toBeGreaterThan(2 * identifyPlant([0], 0.4).b)
  })

  it("two engines: both keep running under the override and match the model (regression: the flight software used to shut one down mid-identification, a 17 % 'model error')", () => {
    const p = identifyPlant([1, 2], 0.4)
    expect(Math.abs(p.accel - p.accelModel)).toBeLessThan(0.03 * p.accelModel)
    expect(Math.abs(p.alpha - p.alphaModel)).toBeLessThan(0.03 * Math.abs(p.alphaModel))
  })

  it("the catch pins sit at the centre of percussion: a gimbal step moves the CG, not the pins", () => {
    const p = identifyPlant([0], 0.5)
    expect(Math.abs(p.pinLever - p.percussion)).toBeLessThan(0.5)
    expect(Math.abs(p.aPin)).toBeLessThan(0.1 * Math.abs(p.aCg))
  })

  it("one off-centre engine makes roll whenever it yaws (it has no roll authority of its own)", () => {
    expect(Math.abs(identifyPlant([0], 0.7).alphaRoll)).toBeGreaterThan(0.01)
  })
})

describe("tail-first aerodynamics: the guidance's database is the plant's", () => {
  it("moment and side force agree within 8 %, and the moment is destabilising and grows as V²", () => {
    const a = checkTailFirstAero(70, 6), b = checkTailFirstAero(90, 10), c = checkTailFirstAero(45, 10)
    for (const x of [a, b]) {
      expect(Math.abs(x.mGuide - x.mPlant)).toBeLessThan(0.08 * Math.abs(x.mPlant))
      expect(Math.abs(x.fGuide - x.fPlant)).toBeLessThan(0.08 * Math.abs(x.fPlant))
    }
    expect(b.mPlant / c.mPlant).toBeCloseTo(4, 0)
    // 10° of tilt at 90 m/s takes most of the one gimbal's holding torque at the throttle floor
    expect(Math.abs(b.mPlant) / b.gimbalTorque).toBeGreaterThan(0.5)
  })
})

describe("single-engine descent on the rig (flight software, no wind)", () => {
  it("on the axis: caught on the pins, centred, no hover", () => {
    const c = captureCase(0, 0)
    expect(c.outcome).toBe("caught")
    expect(c.atRails).toBeLessThan(0.2)
    expect(c.at1km).toBeLessThan(0.5)
    expect(c.contactSpeed).toBeLessThan(0.8)
    expect(c.hover).toBeLessThan(3)
  })

  it("10 m across the rails: caught without overshoot, the pin azimuth held by the roll couples", () => {
    const c = captureCase(10, 0)
    expect(c.outcome).toBe("caught")
    expect(c.atRails).toBeLessThan(0.3)
    expect(c.overshoot).toBeLessThan(1)
    expect(c.rollAngMax).toBeLessThan(20)
    expect(c.rollAngRails).toBeLessThan(0.5) // the pins arrive square to the arms
    expect(c.tiltMax).toBeLessThan(8)
  })

  it("30 m along the rails with 0 overshoot; 5 m/s of residual speed is absorbed", () => {
    const a = captureCase(0, 30)
    expect(a.outcome).toBe("caught")
    expect(a.overshoot).toBeLessThan(0.5)
    const b = captureCase(0, 0, 5, 0)
    expect(b.outcome).toBe("caught")
    expect(b.atRails).toBeLessThan(1)
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
  it("the phase precision table covers the planned phases", () => {
    expect(PHASE_PRECISION.map((p) => p.phase)).toEqual(["highAlpha", "bellyFlop", "flip", "landing", "terminal"])
  })
})
