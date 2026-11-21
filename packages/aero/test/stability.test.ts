/**
 * Aft flaps as static stabilizers of the nose-first fall.
 *
 * Convention: body x = 0 at the aft skirt plane, +x toward the nose. `margin = xcg − cp` is
 * positive when the CP is aft of the CG (stable). The parent report quotes `cp − xcg` (negative
 * for an aft CP); `cpMinusCg` is returned for that comparison.
 *
 * The baseline is evaluated at α = 1° (the tiny linear band) and α = 2° (where the CP has moved
 * forward of the CG in this aero model). The flap study uses α = 2°, because a nose-first fall
 * does not sit at 0.01°.
 */
import { describe, expect, test } from "bun:test"
import { v3, ZERO } from "@ship/math"
import {
  DEFAULT_AFT_FLAPS, DEFAULT_DESIGN, initialConsumables, makeVehicle, massProps,
  type AftFlapDesign, type VehicleConfig,
} from "@ship/craft"
import {
  aftFlapWrench, aeroWrench, makeAeroModel, staticMarginAt, type AftFlapState,
} from "../src/index"

const DEG = Math.PI / 180
const base = makeVehicle(DEFAULT_DESIGN)
const c0 = initialConsumables(base)
const landingCons = { ...c0, loxMain: 0, ch4Main: 0 }

const withFlaps = (over: Partial<AftFlapDesign> = {}): VehicleConfig =>
  makeVehicle({ ...DEFAULT_DESIGN, aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true, ...over } })

const xcgOf = (v: VehicleConfig): number => massProps(v, landingCons).xcg
const deployed = (deg: number): AftFlapState => ({
  deployment: 1,
  deflections: [deg * DEG, deg * DEG, deg * DEG],
})

const sm = (v: VehicleConfig, mach: number, alphaDeg: number, flaps?: AftFlapState): ReturnType<typeof staticMarginAt> =>
  staticMarginAt(makeAeroModel(v), { mach, xcg: xcgOf(v), alpha: alphaDeg * DEG, flaps })

describe("bare-hull reference", () => {
  test("M6: CP aft at 1° but already negative pitch stiffness; clearly unstable at 2°", () => {
    const tiny = sm(base, 6, 0.25)
    const one = sm(base, 6, 1)
    const two = sm(base, 6, 2)
    expect(tiny.pitchStiffness).toBeGreaterThan(0) // the very small stable band
    expect(tiny.margin).toBeGreaterThan(3)
    expect(one.margin).toBeGreaterThan(1) // instantaneous CP is still aft of the CG
    expect(one.pitchStiffness).toBeLessThan(0) // but dCm/dα already flips sign
    expect(two.margin).toBeLessThan(0) // CP forward of CG at the fall angle
    expect(two.pitchStiffness).toBeLessThan(0)
  })

  test("M4 loses the small-angle margin; M2/M1.2/M0.6 are strongly nose-first unstable", () => {
    expect(sm(base, 4, 2).margin).toBeLessThan(0)
    for (const mach of [2, 1.2, 0.6]) expect(sm(base, mach, 2).margin).toBeLessThan(-10)
  })
})

describe("3 small aft flaps", () => {
  const small = withFlaps() // 3 × 1.4 m², 0.5 t total
  const stowed = undefined
  const deflected = deployed(30)

  test("mass and CG: 3 × 1.4 m² titanium set is ≈0.5 t and shifts the CG aft ≈0.1 m", () => {
    expect(small.aftFlaps?.count).toBe(3)
    expect(small.aftFlaps?.totalMass).toBeGreaterThan(400)
    expect(small.aftFlaps?.totalMass).toBeLessThan(650)
    expect(xcgOf(small)).toBeLessThan(xcgOf(base))
    expect(xcgOf(base) - xcgOf(small)).toBeLessThan(0.25)
  })

  test("deployed flaps move the CP aft but cannot make M6 stable at α = 2°", () => {
    const b = sm(base, 6, 2, stowed)
    const s = sm(small, 6, 2, deflected)
    expect(s.cp).toBeLessThan(b.cp) // aft
    expect(s.margin).toBeGreaterThan(b.margin) // less unstable
    expect(s.margin).toBeLessThan(0) // still unstable
  })

  test("all five Mach points: deployed flaps move the CP aft but leave the nose-first margin negative", () => {
    for (const mach of [6, 4, 2, 1.2, 0.6]) {
      const b = sm(base, mach, 2)
      const s = sm(small, mach, 2, deflected)
      expect(s.cp).toBeLessThan(b.cp)
      expect(s.margin).toBeLessThan(0)
      expect(s.pitchStiffness).toBeLessThan(0)
    }
  })

  test("flap normal force and drag are a small fraction of the hull at M6 α = 2°", () => {
    const model = makeAeroModel(small)
    const inp = { vAir: v3(1800 * Math.cos(2 * DEG), 0, -1800 * Math.sin(2 * DEG)), w: ZERO, rho: 1e-3, xcg: xcgOf(small) }
    const hull = aeroWrench(model, { ...inp, a: 300, mu: 1.5e-5, cpMax: 1.84, enginesOn: false, knudsen: 0 })
    const flap = aftFlapWrench(small.aftFlaps, deflected, inp)
    expect(flap.forceMag / Math.hypot(hull.F.y, hull.F.z)).toBeLessThan(0.1)
    expect(-flap.F.x / -hull.F.x).toBeLessThan(0.05)
  })

  test("flaps improve pitch stiffness only slightly and leave dCm/dα < 0", () => {
    const b = sm(base, 6, 2)
    const s = sm(small, 6, 2, deflected)
    expect(s.pitchStiffness).toBeGreaterThan(b.pitchStiffness)
    expect(s.pitchStiffness).toBeLessThan(0)
  })

  test("a differential trim can move the apparent CP, but it is trim, not positive pitch stiffness", () => {
    const trim = sm(small, 6, 2, { deployment: 1, deflections: [-30 * DEG, 30 * DEG, 30 * DEG] })
    expect(trim.margin).toBeGreaterThan(0) // apparent CP aft at this one α
    expect(trim.pitchStiffness).toBeLessThan(0) // dCm/dα is still destabilising
  })
})

describe("area and position needed for a positive margin", () => {
  test("7.3 t / 75.6 m² class set still does not stabilise M6 at α = 2°", () => {
    const large = withFlaps({ span: 6, chord: 4.2, maxDeflection: 60 * DEG })
    const s = sm(large, 6, 2, deployed(30))
    expect(large.aftFlaps?.totalMass).toBeGreaterThan(7000)
    expect(s.margin).toBeLessThan(0.5) // only approaches neutral, and is still negative here
    expect(s.cp).toBeLessThan(sm(base, 6, 2).cp)
  })

  test("≈118 m² / 11.4 t is the smallest scanned set that reaches apparent neutral at M6 α = 2°", () => {
    const near = withFlaps({ span: 7.5, chord: 5.25, maxDeflection: 60 * DEG })
    const below = withFlaps({ span: 7.0, chord: 4.9, maxDeflection: 60 * DEG })
    const n = sm(near, 6, 2, deployed(30))
    const b = sm(below, 6, 2, deployed(30))
    expect(near.aftFlaps?.totalArea).toBeCloseTo(118.1, 1)
    expect(b.margin).toBeLessThan(0)
    expect(n.margin).toBeGreaterThan(0)
    expect(n.margin).toBeLessThan(0.3)
    expect(n.pitchStiffness).toBeLessThan(0) // still no positive stiffness
  })

  test("20 t / 210 m² is the order needed for neutral M6 at α = 2°; at α = 3° it is still short", () => {
    const huge = withFlaps({ span: 10, chord: 7, maxDeflection: 60 * DEG })
    const two = sm(huge, 6, 2, deployed(30))
    const three = sm(huge, 6, 3, deployed(30))
    expect(huge.aftFlaps?.totalMass).toBeGreaterThan(15000)
    expect(two.margin).toBeGreaterThan(0)
    expect(two.margin).toBeLessThan(1)
    expect(two.pitchStiffness).toBeLessThan(0) // apparent CP, still no positive pitch stiffness
    expect(three.margin).toBeLessThan(0)
    expect(xcgOf(huge)).toBeLessThan(xcgOf(base) - 2) // the mass itself moves the CG aft by >2 m
  })

  test("moving the hinge further aft (x = 2 m vs 4.6 m) buys only ≈0.1 m of margin", () => {
    const aft = withFlaps({ span: 6, chord: 4.2, xHinge: 2.0, maxDeflection: 60 * DEG })
    const nominal = withFlaps({ span: 6, chord: 4.2, xHinge: 4.6, maxDeflection: 60 * DEG })
    const a = sm(aft, 6, 2, deployed(30))
    const n = sm(nominal, 6, 2, deployed(30))
    expect(a.margin).toBeGreaterThan(n.margin)
    expect(a.margin - n.margin).toBeLessThan(0.3)
  })

  test("the belly-first high-α trim stays stable with the flaps deployed (M0.5, 70–75°)", () => {
    const small = withFlaps()
    for (const a of [70, 75]) {
      const b = sm(base, 0.5, a)
      const s = sm(small, 0.5, a, { deployment: 1, deflections: [0, 0, 0] })
      expect(b.pitchStiffness).toBeGreaterThan(0)
      expect(s.pitchStiffness).toBeGreaterThan(0)
      expect(Math.abs(s.pitchStiffness - b.pitchStiffness)).toBeLessThan(0.1)
    }
  })

  test("M2: the 20 t set moves the CP aft but still leaves a large negative margin", () => {
    const huge = withFlaps({ span: 10, chord: 7, maxDeflection: 60 * DEG })
    const b = sm(base, 2, 2)
    const h = sm(huge, 2, 2, deployed(30))
    expect(b.margin).toBeLessThan(-10)
    expect(h.cp).toBeLessThan(b.cp) // aft
    expect(h.margin).toBeGreaterThan(b.margin) // improved
    expect(h.margin).toBeLessThan(0) // still unstable
  })

  test("large stabilization surfaces become the dominant normal-force item (not the small set)", () => {
    const huge = withFlaps({ span: 10, chord: 7, maxDeflection: 60 * DEG })
    const model = makeAeroModel(huge)
    const inp = { vAir: v3(1800 * Math.cos(2 * DEG), 0, -1800 * Math.sin(2 * DEG)), w: ZERO, rho: 1e-3, xcg: xcgOf(huge) }
    const hull = aeroWrench(model, { ...inp, a: 300, mu: 1.5e-5, cpMax: 1.84, enginesOn: false, knudsen: 0 })
    const flap = aftFlapWrench(huge.aftFlaps, deployed(30), inp)
    expect(flap.forceMag).toBeGreaterThan(Math.hypot(hull.F.y, hull.F.z))
  })
})
