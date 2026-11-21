/**
 * Mechazilla catch contact (spec/02 §7): pins on the arm rails as vertical spring–dampers with Coulomb
 * friction μ = 0.35, hull against the arms' inner faces 3 MN/m with damping.
 * References / physical expectations:
 *  - Static equilibrium of a spring support: two pins carry the weight with penetration δ = m g/(2 k).
 *  - Coulomb friction: |F_t| ≤ μ F_n; a contact can only push (no adhesion, F_n ≥ 0).
 *  - Dampers dissipate: F_n increases while compressing (v_z < 0) and decreases while rebounding.
 *  - Penalty contact pushes a penetrating body back out along the face normal; the moment about the
 *    CG is (p − r_cg) × F.
 *  - Geometry: Ship Ø 9 m between arms closing to a 9.2 m gap (0.1 m clearance per side) in 3 s.
 */
import { describe, expect, test } from "bun:test"
import { v3, cross, sub, norm } from "@ship/math"
import { TOWER, armsInit, armStep, armGap, armInnerX, towerContact, scaleContact, type ArmState } from "../src/index"

const closed: ArmState = { open: 0, cmdClose: true }
const yMid = TOWER.pos.y - 2 - TOWER.armLength / 2 // middle of the arms along y
const [xl, xr] = armInnerX(closed)
const pinXs = [xl - 0.5, xr + 0.5] // over the left and right rails
const G = 9.80665
const rel = (a: number, b: number): number => Math.abs(a / b - 1)

describe("arms", () => {
  test("close from 26 m to 9.2 m in 3.0 s; closed gap leaves 0.1 m per side around the 9 m hull", () => {
    expect(armGap(armsInit)).toBe(TOWER.openGap)
    let s = armsInit
    for (let k = 0; k < 150; k++) s = armStep(s, true, 0.01) // 1.5 s: half closed
    expect(s.open).toBeCloseTo(0.5, 9)
    for (let k = 0; k < 151; k++) s = armStep(s, true, 0.01) // ≥ 3.0 s: closed
    expect(s.open).toBe(0)
    expect(TOWER.closeTime).toBe(3)
    expect(armGap(s)).toBe(TOWER.closedGap)
    expect((TOWER.closedGap - 9.0) / 2).toBeCloseTo(0.1, 12)
    expect(xl).toBeCloseTo(-xr, 12)
  })
})

describe("pin contact on the rails", () => {
  const pins = (z: number, vz = 0, vx = 0, vy = 0): { p: ReturnType<typeof v3>; v: ReturnType<typeof v3> }[] =>
    pinXs.map((x) => ({ p: v3(x, yMid, z), v: v3(vx, vy, vz) }))

  test("static equilibrium: penetration δ = m g/(2 k_pin) makes the pin load equal the weight", () => {
    const m = 110e3
    const delta = (m * G) / (2 * TOWER.kPin)
    const c = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), pins(TOWER.armHeight - delta), [])
    expect(c.pinLoad).toBeCloseTo(m * G, 3)
    expect(c.F.z).toBeCloseTo(m * G, 3)
    expect(Math.hypot(c.F.x, c.F.y)).toBe(0)
    // symmetric pins about the CG: no net moment
    expect(norm(c.M)).toBeLessThan(1e-6 * m * G)
  })

  test("no force when the pins are above the rails, between the arms, or beyond the arm ends", () => {
    expect(towerContact(closed, v3(0, 0, 0), pins(TOWER.armHeight + 0.2), []).pinLoad).toBe(0)
    const inGap = [{ p: v3(0, yMid, TOWER.armHeight - 0.1), v: v3(0, 0, 0) }]
    expect(towerContact(closed, v3(0, 0, 0), inGap, []).pinLoad).toBe(0)
    const pastEnd = [{ p: v3(pinXs[0], TOWER.pos.y - 2 - TOWER.armLength - 1, TOWER.armHeight - 0.1), v: v3(0, 0, 0) }]
    expect(towerContact(closed, v3(0, 0, 0), pastEnd, []).pinLoad).toBe(0)
  })

  test("dampers dissipate: compressing raises the load, rebounding lowers it, never pulls (F_n ≥ 0)", () => {
    const z = TOWER.armHeight - 0.1
    const rest = towerContact(closed, v3(0, 0, 0), pins(z, 0), []).pinLoad
    expect(towerContact(closed, v3(0, 0, 0), pins(z, -1), []).pinLoad).toBeGreaterThan(rest)
    expect(towerContact(closed, v3(0, 0, 0), pins(z, 0.1), []).pinLoad).toBeLessThan(rest)
    expect(towerContact(closed, v3(0, 0, 0), pins(z, 5), []).pinLoad).toBe(0)
  })

  test("Coulomb friction along a rail axis: opposes sliding and never exceeds μ F_n", () => {
    const z = TOWER.armHeight - 0.2
    for (const [vx, vy] of [[0.01, 0], [2, 0], [0, -3], [0, 0.2]]) {
      const c = towerContact(closed, v3(0, 0, 0), pins(z, 0, vx, vy), [])
      expect(Math.hypot(c.F.x, c.F.y)).toBeLessThanOrEqual(TOWER.mu * c.F.z + 1e-6)
      expect(c.F.x * vx + c.F.y * vy).toBeLessThan(0)
    }
  })

  // Fixed (T12): friction was clipped per axis (pyramid, up to √2·μF_n diagonally). Now a cone:
  // F_t = −μF_n·tanh(c|v_t|/(μF_n))·v̂_t.
  test("Coulomb friction cone: |F_t| ≤ μ F_n and F_t ∥ −v_t for any sliding direction", () => {
    const z = TOWER.armHeight - 0.2
    for (const [vx, vy] of [[1.5, 1.5], [2, 0.5], [-0.7, 3]]) {
      const c = towerContact(closed, v3(0, 0, 0), pins(z, 0, vx, vy), [])
      expect(Math.hypot(c.F.x, c.F.y)).toBeLessThanOrEqual(TOWER.mu * c.F.z + 1e-6)
      expect(Math.abs(c.F.x * vy - c.F.y * vx)).toBeLessThan(1e-9 * Math.hypot(c.F.x, c.F.y) * Math.hypot(vx, vy))
    }
  })

  test("friction saturates at μ F_n when sliding fast and is smooth (viscous slope 4e5 N s/m) near zero slip", () => {
    const z = TOWER.armHeight - 0.2
    const fast = towerContact(closed, v3(0, 0, 0), pins(z, 0, 3, -3), [])
    expect(Math.hypot(fast.F.x, fast.F.y) / (TOWER.mu * fast.F.z)).toBeGreaterThan(0.9999)
    const slow = towerContact(closed, v3(0, 0, 0), pins(z, 0, 1e-4, 0), [])
    expect(rel(slow.F.x / 2, -4e5 * 1e-4)).toBeLessThan(1e-6) // per pin: viscous slope at zero slip
    const halfway = towerContact(closed, v3(0, 0, 0), pins(z, 0, 0, 0.6), [])
    expect(Math.abs(halfway.F.y)).toBeLessThan(TOWER.mu * halfway.F.z)
  })

  test("moment about the CG is Σ (p − r_cg) × f", () => {
    const cg = v3(0.3, yMid + 2, TOWER.armHeight + 15)
    const ps = [{ p: v3(pinXs[0], yMid, TOWER.armHeight - 0.2), v: v3(0.5, 0, -0.3) }, { p: v3(pinXs[1], yMid, TOWER.armHeight - 0.05), v: v3(0, 0, 0) }]
    const c = towerContact(closed, cg, ps, [])
    const per = ps.map((pp) => towerContact(closed, cg, [pp], []))
    const M = per.reduce((acc, pc, i) => {
      const r = sub(ps[i].p, cg)
      const m = cross(r, pc.F)
      return v3(acc.x + m.x, acc.y + m.y, acc.z + m.z)
    }, v3(0, 0, 0))
    expect(norm(sub(c.M, M))).toBeLessThan(1e-6 * norm(M))
  })
})

describe("hull against the arms", () => {
  test("a centred 9 m hull between closed arms does not touch; an offset hull is pushed back toward the centre", () => {
    const z = TOWER.armHeight - 1
    const centred = towerContact(closed, v3(0, 0, 0), [], [{ p: v3(0, yMid, z), v: v3(0, 0, 0), r: 4.5 }])
    expect(centred.bodyContact).toBe(false)
    const left = towerContact(closed, v3(0, 0, 0), [], [{ p: v3(-0.4, yMid, z), v: v3(0, 0, 0), r: 4.5 }])
    const right = towerContact(closed, v3(0, 0, 0), [], [{ p: v3(0.4, yMid, z), v: v3(0, 0, 0), r: 4.5 }])
    expect(left.bodyContact).toBe(true)
    expect(left.F.x).toBeCloseTo(3e6 * 0.3, 3) // 3 MN/m × 0.3 m penetration
    expect(right.F.x).toBeCloseTo(-3e6 * 0.3, 3)
  })

  test("solid tower structure: a hull point inside the tower column is pushed south, not passed through", () => {
    const z = 120
    const inside = towerContact(closed, v3(0, 0, 0), [], [{ p: v3(0, TOWER.pos.y - 4.8, z), v: v3(0, 0, 0), r: 4.5 }])
    expect(inside.bodyContact).toBe(true)
    expect(inside.F.y).toBeLessThan(-1e5) // pushed away from the column centre (south)
    // and the arm top is solid: a hull point driven down onto a closed arm cannot pass through it
    const onArm = towerContact(closed, v3(0, 0, 0), [], [{ p: v3(xl - 0.8, yMid, TOWER.armHeight + 4.0), v: v3(0, 0, -2), r: 4.5 }])
    expect(onArm.bodyContact).toBe(true)
    expect(onArm.F.z).toBeGreaterThan(0)
  })

  test("scaleContact scales force and moment, keeps the load diagnostics", () => {
    const c = towerContact(closed, v3(0, 0, 0), [{ p: v3(pinXs[0], yMid, TOWER.armHeight - 0.1), v: v3(0, 0, 0) }], [])
    const h = scaleContact(c, 0.5)
    expect(h.F.z).toBeCloseTo(0.5 * c.F.z, 9)
    expect(h.pinLoad).toBe(c.pinLoad)
  })
})
