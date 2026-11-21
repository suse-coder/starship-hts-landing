/** Sim plumbing for the optional aft flaps: stowed baseline, command → actuator state, thermal node. */
import { describe, expect, test } from "bun:test"
import { DEFAULT_AFT_FLAPS, DEFAULT_DESIGN } from "@ship/craft"
import { initState, makeWorld, step } from "../src/index"

const enabled = {
  ...DEFAULT_DESIGN,
  aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true },
}
const worldOn = makeWorld(enabled)
const worldOff = makeWorld(DEFAULT_DESIGN)

describe("baseline", () => {
  test("the disabled design is bit-identical to omitting the option", () => {
    const worldAbsent = makeWorld({ ...DEFAULT_DESIGN, aftFlaps: undefined })
    let a = initState(worldOff)
    let b = initState(worldAbsent)
    for (let k = 0; k < 5; k++) {
      const oa = step(worldOff, a)
      const ob = step(worldAbsent, b)
      a = oa.s
      b = ob.s
      expect(a.r).toEqual(b.r)
      expect(a.v).toEqual(b.v)
      expect(a.q).toEqual(b.q)
      expect(a.w).toEqual(b.w)
    }
  })

  test("disabled design has no flap set, no thermal node, and stays stowed", () => {
    const s0 = initState(worldOff)
    expect(worldOff.vehicle.aftFlaps).toBeNull()
    expect(s0.flapTh).toBeNull()
    expect(s0.flaps.deployment).toBe(0)
    const out = step(worldOff, s0, { deploy: true, deflections: [0.3, 0.3, 0.3] })
    expect(out.s.flaps.deployment).toBe(0)
  })
})

describe("deployment command", () => {
  test("initState: flaps stowed, titanium node at 285 K", () => {
    const s0 = initState(worldOn)
    expect(worldOn.vehicle.aftFlaps?.count).toBe(3)
    expect(s0.flaps.deployment).toBe(0)
    expect(s0.flaps.deflections).toEqual([0, 0, 0])
    expect(s0.flapCmd.deploy).toBe(false)
    expect(s0.flapTh?.T).toBe(285)
  })

  test("deploy command ramps to full over the design deploy time and is stored in the state", () => {
    let s = initState(worldOn)
    const cmd = { deploy: true, deflections: [0, 0, 0] }
    for (let k = 0; k < 4; k++) s = step(worldOn, s, cmd).s
    expect(s.t).toBeCloseTo(2, 9) // 4 × 0.5 s coast step
    expect(s.flaps.deployment).toBeCloseTo(1, 9)
    expect(s.flapCmd).toEqual(cmd)
  })

  test("per-flap deflections are rate-limited and clamped", () => {
    let s = initState(worldOn)
    const cmd = { deploy: true, deflections: [1, -1, 0] }
    s = step(worldOn, s, cmd).s
    const max = worldOn.vehicle.aftFlaps!.maxDeflection
    expect(Math.abs(s.flaps.deflections[0])).toBeLessThanOrEqual(max)
    expect(Math.abs(s.flaps.deflections[1])).toBeLessThanOrEqual(max)
    expect(s.flaps.deflections[0]).toBeGreaterThan(0)
    expect(s.flaps.deflections[1]).toBeLessThan(0)
  })
})

describe("thermal node", () => {
  test("a hot flap cools radiatively in the vacuum coast (base and plume fluxes ≈ 0)", () => {
    const s0 = initState(worldOn)
    const hot = { ...s0, t: 1, flapTh: { ...s0.flapTh!, T: 1000, peakT: 1000 } }
    const out = step(worldOn, hot)
    expect(out.s.flapTh!.T).toBeLessThan(1000)
    expect(out.s.flapTh!.T).toBeGreaterThan(0)
  })
})
