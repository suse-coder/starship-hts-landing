/**
 * Fast 3-DOF evaluator: the belly re-plan must be finite and gate-aware, the landing must run from
 * the flip state, and the whole evaluation must stay in the millisecond-second budget (the spec's
 * "ms" claim is measured in docs/05-benchmarks/02-optimizer.md; the test only bounds the sanity).
 */
import { describe, expect, test } from "bun:test"
import { v3 } from "@ship/math"
import { DEFAULT_DESIGN } from "@ship/craft"
import { makeWorld } from "@ship/sim"
import { evaluateFast, flyBelly, flyLanding, MISSION_TUNING_DEFAULT, type FastFlip, type FastHandover } from "../src/index"
import { flipAltitude } from "../src/landing"

const world = makeWorld(DEFAULT_DESIGN)
const hand: FastHandover = {
  t: 2896.8,
  pinRel: v3(-20425, -174, 30_763),
  velRel: v3(489, 322, -127),
  hBase: 30_763,
  mass: 137_780,
  V: 603,
  psi0: (122.6 * Math.PI) / 180,
}

describe("fast evaluator: belly re-plan", () => {
  test("the plan flies to the flip gate and returns a finite flip state", () => {
    const { plan, flip } = flyBelly(hand, world.guide.pred, MISSION_TUNING_DEFAULT)
    expect(Number.isFinite(plan.miss)).toBe(true)
    expect(Number.isFinite(plan.psi1)).toBe(true)
    expect(Number.isFinite(plan.psi2)).toBe(true)
    expect(flip.hBase).toBeGreaterThan(100)
    expect(flip.hBase).toBeLessThan(hand.hBase)
    expect(flip.vz).toBeLessThan(-40) // still falling at the flip
    expect(flip.V).toBeGreaterThan(50)
    expect(flip.pinRel.z).toBe(0)
    expect(plan.miss).toBeLessThan(3000) // the plan can reach the pad from the hand-over
  })

  test("a higher gate design raises the flip altitude (the tuning is threaded into flipAltitude)", () => {
    const low = flyBelly(hand, world.guide.pred, MISSION_TUNING_DEFAULT.with({ gateMiss: 250 }))
    const high = flyBelly(hand, world.guide.pred, MISSION_TUNING_DEFAULT.with({ gateMiss: 900 }))
    expect(high.flip.hBase).toBeGreaterThan(low.flip.hBase)
    expect(flipAltitude(-110, MISSION_TUNING_DEFAULT.with({ gateMiss: 900 }))).toBeGreaterThan(
      flipAltitude(-110, MISSION_TUNING_DEFAULT.with({ gateMiss: 250 })),
    )
    expect(flipAltitude(-110, MISSION_TUNING_DEFAULT)).toBeGreaterThan(600) // the low two-engine gate
  })
})

describe("fast evaluator: landing from the flip state", () => {
  test("a representative flip closes within the 3-DOF model", () => {
    // this evaluator's 3-DOF landing was tuned to the high gate; keep its own design here, the flight
    // uses the low two-engine gate in tuning.ts
    const LEGACY = MISSION_TUNING_DEFAULT.with({ gateMiss: 630, flipDv: 35, flipFall: 5.9, bellyPlanEnd: 1750 })
    const { flip } = flyBelly(hand, world.guide.pred, LEGACY)
    const c = flyLanding(flip, world.vehicle, LEGACY, 0.6)
    expect(Number.isFinite(c.lat)).toBe(true)
    expect(c.lat).toBeLessThan(10)
    expect(c.contactSpeed).toBeLessThan(6)
  })

  test("a flip with a huge lateral offset is reported as a miss, not a catch", () => {
    const { flip } = flyBelly(hand, world.guide.pred, MISSION_TUNING_DEFAULT)
    const bad: FastFlip = { ...flip, pinRel: v3(1500, 0, 0) }
    const c = flyLanding(bad, world.vehicle, MISSION_TUNING_DEFAULT)
    expect(c.feasible).toBe(false)
    expect(c.lat).toBeGreaterThan(1)
  })
})

describe("fast evaluator: end-to-end budget", () => {
  test("runs the re-planned belly + landing and reports finite timings", () => {
    const p = evaluateFast(hand, world.guide.pred, world.vehicle, MISSION_TUNING_DEFAULT)
    expect(Number.isFinite(p.margin)).toBe(true)
    expect(Number.isFinite(p.bellyMiss)).toBe(true)
    expect(p.timingMs.total).toBeGreaterThan(0)
    expect(p.timingMs.total).toBeLessThan(5000) // generous CI bound; measured ≈0.1–0.3 s
    expect(p.bellyPlan.miss).toBeGreaterThanOrEqual(0)
  })
})
