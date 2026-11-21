/** Optional aft flap set: geometry, mass, CG and inertia impact; disabled by default. */
import { describe, expect, test } from "bun:test"
import {
  DEFAULT_AFT_FLAPS, DEFAULT_DESIGN, dryMass, initialConsumables, makeAftFlapSet, makeVehicle, massProps,
} from "../src/index"

const off = makeVehicle()
const on = makeVehicle({ ...DEFAULT_DESIGN, aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true } })
const c0 = initialConsumables(off)

describe("design option", () => {
  test("off by default: no set, no extra component, baseline mass unchanged", () => {
    expect(off.aftFlaps).toBeNull()
    expect(off.components.some((c) => c.id === "aftFlaps")).toBe(false)
    expect(dryMass(off)).toBe(dryMass(makeVehicle(DEFAULT_DESIGN)))
  })

  test("count 3, titanium, near the engine section, 90° fold-out", () => {
    const f = on.aftFlaps!
    expect(f).not.toBeNull()
    expect(f.count).toBe(3)
    expect(f.design.material).toBe("Ti-6Al-4V")
    expect(f.design.density).toBe(4430)
    expect(f.xHinge).toBeLessThan(6) // engine end of a 52.1 m body
    expect(f.deploymentAngle).toBeCloseTo(Math.PI / 2, 12)
    expect(f.wakeFactor).toBeGreaterThanOrEqual(0.3)
    expect(f.wakeFactor).toBeLessThanOrEqual(1)
    expect(f.area).toBeCloseTo(1.4 * 1.0, 12)
  })

  test("mass is titanium envelope + hinge/actuator, and total = 3 × per-flap", () => {
    const f = on.aftFlaps!
    const titanium = f.area * f.thickness * f.design.solidity * f.design.density
    expect(f.materialMass).toBeCloseTo(titanium, 9)
    expect(f.mass).toBeCloseTo(titanium + f.design.hingeActuatorMass, 9)
    expect(f.totalMass).toBeCloseTo(3 * f.mass, 9)
  })
})

describe("mass / CG / inertia", () => {
  test("enabling the set adds exactly its total mass", () => {
    expect(dryMass(on) - dryMass(off)).toBeCloseTo(on.aftFlaps!.totalMass, 6)
  })

  test("aft-mounted mass moves the CG aft and raises pitch/yaw inertia", () => {
    const a = massProps(off, c0)
    const b = massProps(on, c0)
    expect(b.m - a.m).toBeCloseTo(on.aftFlaps!.totalMass, 6)
    expect(b.xcg).toBeLessThan(a.xcg)
    expect(a.xcg - b.xcg).toBeGreaterThan(0.05)
    expect(a.xcg - b.xcg).toBeLessThan(0.2)
    expect(b.I.y).toBeGreaterThan(a.I.y)
    expect(b.I.z).toBeGreaterThan(a.I.z)
  })

  test("per-flap geometry is consistent (area, centroid, azimuth spacing)", () => {
    const f = on.aftFlaps!
    expect(f.area).toBeCloseTo(f.span * f.chord, 12)
    expect(f.rCentroid).toBeCloseTo(f.hingeRadius + f.span / 2, 12)
    expect(f.xCentroid).toBeCloseTo(f.xHinge - f.chord / 2, 12)
    expect(f.azimuths.length).toBe(3)
    expect(f.azimuths[1] - f.azimuths[0]).toBeCloseTo((2 * Math.PI) / 3, 12)
  })

  test("makeAftFlapSet clamps the wake factor and honours a custom count", () => {
    const f = makeAftFlapSet({ ...DEFAULT_AFT_FLAPS, count: 4, wakeFactor: 1.7 })
    expect(f.count).toBe(4)
    expect(f.wakeFactor).toBe(1)
    expect(f.azimuths.length).toBe(4)
  })
})
