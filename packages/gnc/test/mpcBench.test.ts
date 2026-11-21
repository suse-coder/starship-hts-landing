/**
 * Plant model checks for the MPC bake-off (`scripts/mpc-bench.ts`).
 *
 * These are the numbers that decide whether the fast plant is allowed to rank controllers at all:
 * the thrust acceleration at 40/60/100 % for 2 and 3 SL Raptors, the propellant flow over 10 s, the
 * gimbal moment at ±15°, and the sea-level/1 km pressure correction. Every value comes from the real
 * `@ship/craft` engine record; the analytic expectations are written out independently of the plant
 * helpers where it matters (acceleration, pressure, flow), so a change in the plant cannot silently
 * move both sides of the comparison.
 */
import { describe, expect, test } from "bun:test"
import { v3 } from "@ship/math"
import { G0, atmosphere } from "@ship/earth"
import {
  BENCH_ENGINE,
  BENCH_MASS,
  ENGINE_MODEL,
  P_SL,
  gimbalAngularAccel,
  gimbalMoment,
  initialPlantState,
  massFlowPerEngine,
  netVerticalAcceleration,
  pressureAt,
  stepPlant,
  thrustAcceleration,
  thrustPerEngine,
} from "../src/bench/plant"
import { runModelChecks } from "../src/bench/checks"
import { makeReferencePath } from "../src/bench/reference"
import { GATE, GATE_V, SCENARIOS } from "../src/bench/harness"
import type { BenchState, PlantConfig } from "../src/bench/types"

const G = 9.79
const rel = (a: number, b: number): number => Math.abs(a - b) / Math.max(Math.abs(b), 1e-12)
const expectRel = (actual: number, expected: number, tol = 1e-12): void => {
  expect(rel(actual, expected)).toBeLessThanOrEqual(tol)
}

const rateCfg = (nEng: number): PlantConfig => ({
  nEng,
  mode: "rate",
  rateCap: 0.45,
  tauDir: 0.35,
  tauThrottle: 0.3,
  gimbalKp: 1.5,
  gimbalKd: 1.8,
  gravity: G,
  dt: 1e-3,
  disturbance: v3(0, 0, 0),
})

const stateAt = (z: number, throttle: number): BenchState => ({
  t: 0,
  p: v3(0, 0, z),
  v: v3(0, 0, 0),
  d: v3(0, 0, 1),
  a: v3(0, 0, 0),
  mass: BENCH_MASS,
  throttle,
  fuel: 0,
  gimbal: 0,
  omega: 0,
})

describe("plant: Raptor thrust and acceleration", () => {
  test("the engine record is the real sea-level Raptor", () => {
    expect(BENCH_ENGINE.kind).toBe("SL")
    expect(BENCH_ENGINE.thrustSL).toBe(2.45e6)
    expect(BENCH_ENGINE.isp0).toBe(350)
    expect(BENCH_ENGINE.exitArea).toBe(1.33)
    expect(BENCH_ENGINE.minThrottle).toBe(0.4)
    expect(BENCH_ENGINE.gimbalMax).toBeCloseTo((15 * Math.PI) / 180, 12)
    expect(BENCH_ENGINE.gimbalRate).toBeCloseTo((20 * Math.PI) / 180, 12)
    expectRel(ENGINE_MODEL.fVac100, 2.45e6 + P_SL * 1.33)
  })

  test("commanded 2/3 engines at 40/60/100 % match n·throttle·T/m − g within 1 %", () => {
    const z = 800
    const pAmb = pressureAt(z)
    for (const nEng of [2, 3]) {
      for (const throttle of [0.4, 0.6, 1.0]) {
        const cfg = rateCfg(nEng)
        const s0 = stateAt(z, throttle)
        const { next } = stepPlant(s0, { dir: v3(0, 0, 1), throttle }, cfg)
        const measured = (next.v.z - s0.v.z) / cfg.dt
        // independent expectation: n·(throttle·F_vac − p∞·A_e)/m − g
        const expected =
          (nEng * (throttle * (BENCH_ENGINE.thrustSL + P_SL * BENCH_ENGINE.exitArea) - pAmb * BENCH_ENGINE.exitArea)) /
            BENCH_MASS -
          G
        expect(Math.abs(measured - expected) / Math.abs(expected)).toBeLessThan(0.01)
        expectRel(measured, netVerticalAcceleration(nEng, throttle, BENCH_MASS, pAmb, G), 1e-9)
        expectRel(measured, thrustAcceleration(nEng, throttle, BENCH_MASS, pAmb) - G, 1e-9)
      }
    }
  })

  test("two engines at the 40 % floor cannot hover (net acceleration is positive)", () => {
    const pAmb = pressureAt(0)
    const a = netVerticalAcceleration(2, 0.4, BENCH_MASS, pAmb, G)
    expect(a).toBeGreaterThan(0)
    expect(a).toBeLessThan(4) // ≈ 3.17 m/s² at the landing mass, not a hover
  })
})

describe("plant: propellant flow", () => {
  test("10 s at fixed throttle loses exactly ∫T/(Isp·g0) dt", () => {
    const cfg = rateCfg(2)
    for (const throttle of [0.4, 0.6, 1.0]) {
      let s = stateAt(10_000, throttle)
      const steps = Math.round(10 / cfg.dt)
      for (let k = 0; k < steps; k++) s = stepPlant(s, { dir: v3(0, 0, 1), throttle }, cfg).next
      const expected = 10 * 2 * massFlowPerEngine(throttle)
      expect(Math.abs(s.mass - (BENCH_MASS - expected))).toBeLessThan(1e-6)
      expect(Math.abs(s.fuel - expected)).toBeLessThan(1e-6)
    }
  })

  test("the flow uses Isp_vac, not Isp_sl", () => {
    const expected = ENGINE_MODEL.fVac100 / (ENGINE_MODEL.ispVac * G0)
    expectRel(massFlowPerEngine(1), expected)
    expectRel(ENGINE_MODEL.ispVac, 350 * (ENGINE_MODEL.fVac100 / 2.45e6))
    expect(ENGINE_MODEL.ispVac).toBeGreaterThan(369)
  })
})

describe("plant: gimbal moment", () => {
  test("±15° moment is n·T·arm·sin δ", () => {
    const T = thrustPerEngine(1, pressureAt(800))
    for (const sign of [1, -1]) {
      const angle = sign * ENGINE_MODEL.gimbalMax
      const expected = 2 * T * ENGINE_MODEL.arm * Math.sin(angle)
      expectRel(gimbalMoment(2, T, angle), expected)
      expectRel(gimbalAngularAccel(2, T, angle), expected / ENGINE_MODEL.Iyy)
    }
    expect(ENGINE_MODEL.arm).toBeGreaterThan(24)
    expect(ENGINE_MODEL.Iyy).toBeGreaterThan(4e7)
  })

  test("torque mode holds the gimbal at +15° and produces the analytic angular acceleration", () => {
    const cfg: PlantConfig = { ...rateCfg(2), mode: "torque" }
    const s: BenchState = {
      ...stateAt(800, 1),
      d: v3(1, 0, 0), // 90° error keeps the gimbal PD saturated
      gimbal: ENGINE_MODEL.gimbalMax,
    }
    const { applied } = stepPlant(s, { dir: v3(0, 0, 1), throttle: 1 }, cfg)
    const T = thrustPerEngine(1, pressureAt(800))
    expectRel(applied.alpha, gimbalAngularAccel(2, T, ENGINE_MODEL.gimbalMax))
  })

  test("rate mode caps the direction step at rate·dt", () => {
    const cfg: PlantConfig = { ...rateCfg(2), tauDir: 0.05 }
    const s: BenchState = { ...stateAt(800, 0.4), d: v3(1, 0, 0) }
    const { next, applied } = stepPlant(s, { dir: v3(0, 0, 1), throttle: 0.4 }, cfg)
    const fromVertical = Math.acos(Math.max(-1, Math.min(1, next.d.z)))
    const stepAngle = Math.PI / 2 - fromVertical
    expect(stepAngle).toBeGreaterThan(0)
    expect(stepAngle).toBeLessThanOrEqual(cfg.rateCap * cfg.dt + 1e-12)
    expect(applied.dirErrRad).toBeLessThanOrEqual(Math.PI / 2)
  })
})

describe("plant: pressure correction", () => {
  test("sea level and 1 km thrust match the vacuum-minus-p∞A_e law", () => {
    expectRel(thrustPerEngine(1, P_SL), 2.45e6)
    const p1 = atmosphere(1000).p
    expectRel(thrustPerEngine(1, p1), 2.45e6 + (P_SL - p1) * 1.33)
    expectRel(thrustPerEngine(0.4, P_SL), 0.4 * ENGINE_MODEL.fVac100 - P_SL * 1.33)
    expect(p1).toBeLessThan(P_SL)
  })
})

describe("bench: shared reference and model-check bundle", () => {
  test("the reference starts and ends on the scenario/gate state", () => {
    for (const sc of SCENARIOS) {
      const ref = makeReferencePath(sc.p0, sc.v0, GATE, GATE_V)
      expect(ref.duration).toBeGreaterThan(10)
      const a = ref.at(0)
      const b = ref.at(ref.duration)
      expectRel(a.p.x, sc.p0.x, 1e-9)
      expectRel(a.p.y, sc.p0.y, 1e-9)
      expectRel(a.p.z, sc.p0.z, 1e-9)
      expectRel(a.v.z, sc.v0.z, 1e-9)
      expect(Math.abs(b.p.x - GATE.x)).toBeLessThan(1e-9)
      expectRel(b.p.y, GATE.y, 1e-9)
      expect(Math.abs(b.p.z - GATE.z)).toBeLessThan(1e-9)
      expect(Math.abs(b.v.x - GATE_V.x)).toBeLessThan(1e-9)
      expectRel(b.v.y, GATE_V.y, 1e-9)
      expectRel(b.v.z, GATE_V.z, 1e-9)
    }
  })

  test("every bundled model check passes", () => {
    const checks = runModelChecks()
    expect(checks.length).toBeGreaterThanOrEqual(17)
    for (const c of checks) {
      expect({ id: c.id, pass: c.pass, relErr: c.relErr, tolerance: c.tolerance }).toMatchObject({ pass: true })
      expect(c.relErr).toBeLessThanOrEqual(c.tolerance)
    }
  })

  test("initial state has the throttle floor and a vertical thrust axis", () => {
    const s = initialPlantState(v3(0, 0, 800), v3(0, 0, -100), BENCH_MASS, 2)
    expect(s.throttle).toBe(BENCH_ENGINE.minThrottle)
    expect(s.d.x).toBe(0)
    expect(s.d.y).toBe(0)
    expect(s.d.z).toBe(1)
    expect(s.a.z).toBeGreaterThan(G)
    expect(s.fuel).toBe(0)
  })
})
