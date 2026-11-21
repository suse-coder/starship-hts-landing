/**
 * Deployable aft flap model: folded zero-force contract, Newtonian flat-plate magnitude,
 * deployment and wake scaling, broadside drag, differential steering, actuator rates.
 */
import { describe, expect, test } from "bun:test"
import { v3, ZERO, norm } from "@ship/math"
import { DEFAULT_AFT_FLAPS, DEFAULT_DESIGN, makeVehicle } from "@ship/craft"
import { aftFlapWrench, stepAftFlaps, stowedAftFlapCommand, stowedAftFlapState, type AftFlapState } from "../src/index"

const DEG = Math.PI / 180
const v = makeVehicle({ ...DEFAULT_DESIGN, aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true } })
const set = v.aftFlaps!
const rel = (a: number, b: number): number => Math.abs(a / b - 1)

const axial = (V = 1000, rho = 0.1): { vAir: ReturnType<typeof v3>; w: typeof ZERO; rho: number; xcg: number } => ({
  vAir: v3(V, 0, 0), w: ZERO, rho, xcg: 28,
})
const all = (deg: number): AftFlapState => ({
  deployment: 1,
  deflections: [deg * DEG, deg * DEG, deg * DEG],
})

describe("folded flaps", () => {
  test("deployment 0 gives exactly zero force and moment", () => {
    const o = aftFlapWrench(set, { deployment: 0, deflections: [30 * DEG, -30 * DEG, 10 * DEG] }, axial())
    expect(o.F).toEqual(ZERO)
    expect(o.M).toEqual(ZERO)
    expect(o.forceMag).toBe(0)
  })

  test("no flap set gives zero force", () => {
    const o = aftFlapWrench(null, all(30), axial())
    expect(o.forceMag).toBe(0)
    expect(o.momentMag).toBe(0)
  })
})

describe("Newtonian flat-plate magnitude", () => {
  test("axial flow at +20°: drag = 3·2 sin³δ · q_wake · A", () => {
    const inp = axial()
    const o = aftFlapWrench(set, all(20), inp)
    const q = 0.5 * inp.rho * 1000 ** 2
    const expected = 3 * 2 * Math.sin(20 * DEG) ** 3 * q * set.wakeFactor * set.area
    expect(rel(-o.F.x, expected)).toBeLessThan(1e-12)
    expect(Math.hypot(o.F.y, o.F.z)).toBeLessThan(1e-9 * Math.abs(o.F.x))
  })

  test("sign flips with deflection sign; both signs produce axial drag", () => {
    const p = aftFlapWrench(set, all(20), axial())
    const n = aftFlapWrench(set, all(-20), axial())
    expect(p.F.x).toBeLessThan(0)
    expect(n.F.x).toBeLessThan(0)
    expect(p.F.x).toBeCloseTo(n.F.x, 6)
    expect(Math.hypot(p.F.y, p.F.z)).toBeCloseTo(Math.hypot(n.F.y, n.F.z), 6)
  })

  test("force scales linearly with deployment fraction", () => {
    const f = (deployment: number): number => aftFlapWrench(set, { deployment, deflections: [20 * DEG, 20 * DEG, 20 * DEG] }, axial()).forceMag
    expect(f(0.25)).toBeCloseTo(0.25 * f(1), 9)
    expect(f(0.5)).toBeCloseTo(0.5 * f(1), 9)
    expect(f(1)).toBeGreaterThan(0)
  })
})

describe("base/wake factor", () => {
  test("0.3, 0.5 and 1.0 scale the force linearly and monotonically", () => {
    const force = (wakeFactor: number): number => {
      const w = makeVehicle({ ...DEFAULT_DESIGN, aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true, wakeFactor } })
      return aftFlapWrench(w.aftFlaps, all(20), axial()).forceMag
    }
    const f03 = force(0.3), f05 = force(0.5), f10 = force(1.0)
    expect(f03).toBeLessThan(f05)
    expect(f05).toBeLessThan(f10)
    expect(rel(f03, 0.3 * f10)).toBeLessThan(1e-12)
    expect(rel(f05, 0.5 * f10)).toBeLessThan(1e-12)
  })
})

describe("belly-first drag and steering", () => {
  test("broadside flow (α = 90°) on deployed flaps adds drag; folded adds nothing", () => {
    const inp = { vAir: v3(0, 0, -80), w: ZERO, rho: 1.225, xcg: 28 }
    const deployed = aftFlapWrench(set, { deployment: 1, deflections: [0, 0, 0] }, inp)
    const folded = aftFlapWrench(set, { deployment: 0, deflections: [0, 0, 0] }, inp)
    expect(deployed.F.z).toBeGreaterThan(0) // opposite the -z motion
    expect(deployed.forceMag).toBeGreaterThan(1e3)
    expect(folded.forceMag).toBe(0)
  })

  test("differential deflection produces a lateral moment; equal deflection does not", () => {
    const symmetric = aftFlapWrench(set, all(20), axial())
    const differential = aftFlapWrench(set, { deployment: 1, deflections: [20 * DEG, 0, 0] }, axial())
    expect(Math.hypot(symmetric.M.y, symmetric.M.z)).toBeLessThan(1)
    expect(Math.hypot(differential.M.y, differential.M.z)).toBeGreaterThan(1e3)
    expect(differential.momentMag).toBeGreaterThan(0)
  })

  test("force and moment are finite at every deflection and deployment (no NaN)", () => {
    for (const d of [-30, -10, 0, 10, 30]) {
      for (const dep of [0, 0.5, 1]) {
        const o = aftFlapWrench(set, { deployment: dep, deflections: [d * DEG, d * DEG, d * DEG] }, axial())
        expect(Number.isFinite(o.forceMag)).toBe(true)
        expect(Number.isFinite(o.momentMag)).toBe(true)
      }
    }
  })
})

describe("actuator", () => {
  test("deploys over deployTime and holds at 1", () => {
    let s = stowedAftFlapState(set.count)
    const cmd = stowedAftFlapCommand(set.count)
    s = stepAftFlaps(set, s, { ...cmd, deploy: true }, set.deployTime / 2)
    expect(s.deployment).toBeCloseTo(0.5, 12)
    s = stepAftFlaps(set, s, { ...cmd, deploy: true }, set.deployTime / 2)
    expect(s.deployment).toBeCloseTo(1, 12)
    s = stepAftFlaps(set, s, { ...cmd, deploy: true }, 10)
    expect(s.deployment).toBe(1)
  })

  test("retracts and rate-limits/clamps deflection", () => {
    let s: AftFlapState = { deployment: 1, deflections: [0, 0, 0] }
    const cmd = { deploy: false, deflections: [100 * DEG, -100 * DEG, 0] }
    s = stepAftFlaps(set, s, cmd, 0.1)
    expect(s.deployment).toBeLessThan(1)
    expect(s.deflections[0]).toBeCloseTo(set.deflectionRate * 0.1, 12)
    expect(s.deflections[1]).toBeCloseTo(-set.deflectionRate * 0.1, 12)
    for (let k = 0; k < 100; k++) s = stepAftFlaps(set, s, cmd, 0.1)
    expect(s.deployment).toBe(0)
    expect(s.deflections[0]).toBeCloseTo(set.maxDeflection, 12)
    expect(s.deflections[1]).toBeCloseTo(-set.maxDeflection, 12)
    expect(norm(v3(s.deflections[0], s.deflections[1], s.deflections[2]))).toBeLessThan(2)
  })

  test("deploy-only command is valid (deflections omitted)", () => {
    const s = stepAftFlaps(set, stowedAftFlapState(set.count), { deploy: true }, 0.5)
    expect(s.deployment).toBeGreaterThan(0)
    expect(s.deflections).toEqual([0, 0, 0])
  })

  test("no installed set keeps the state untouched", () => {
    const s = stowedAftFlapState(3)
    expect(stepAftFlaps(null, s, { deploy: true, deflections: [1, 1, 1] }, 10)).toBe(s)
  })
})
