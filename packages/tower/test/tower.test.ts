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
import { v3, add, cross, dot, scale, sub, norm } from "@ship/math"
import {
  TOWER, armsInit, armStep, armGap, armInnerX, towerContact, towerBoxes, firstHullImpact, resolveHullImpact,
  bothPinsLoaded, scaleContact, type ArmState,
} from "../src/index"

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
    expect(h.pinLoads).toEqual(c.pinLoads)
    expect(h.pinCount).toBe(c.pinCount)
    expect(h.pinLoadMin).toBe(c.pinLoadMin)
  })
})

describe("per-pin catch gate", () => {
  const m = 110e3
  const weight = m * G
  const delta = weight / (2 * TOWER.kPin)
  const pin = (x: number, z: number, vz = 0): { p: ReturnType<typeof v3>; v: ReturnType<typeof v3> } =>
    ({ p: v3(x, yMid, z), v: v3(0, 0, vz) })
  const gapPin = pin(0, TOWER.armHeight - delta)
  const leftPin = pin(pinXs[0], TOWER.armHeight - delta)
  const rightPin = pin(pinXs[1], TOWER.armHeight - delta)

  test("two pins loaded on the rails → catch candidate true, and the per-pin loads sum to the total", () => {
    const c = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), [leftPin, rightPin], [])
    expect(c.pinCount).toBe(2)
    expect(c.pinLoads.length).toBe(2)
    expect(c.pinLoads[0]).toBeCloseTo(weight / 2, 3)
    expect(c.pinLoads[1]).toBeCloseTo(weight / 2, 3)
    expect(c.pinLoads[0] + c.pinLoads[1]).toBeCloseTo(c.pinLoad, 6)
    expect(c.pinLoad).toBeCloseTo(weight, 3)
    expect(c.pinLoadMin).toBeCloseTo(weight / 2, 3)
    expect(bothPinsLoaded(c.pinCount, c.pinLoadMin, 0.15 * weight)).toBe(true)
  })

  test("one pin loaded on a rail and the other in the gap → catch candidate false", () => {
    const c = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), [leftPin, gapPin], [])
    expect(c.pinCount).toBe(1)
    expect(c.pinLoads[1]).toBe(0)
    expect(c.pinLoadMin).toBe(0)
    expect(c.pinLoad).toBeCloseTo(weight / 2, 3) // the single pin still carries a large total
    expect(bothPinsLoaded(c.pinCount, c.pinLoadMin, 0.15 * weight)).toBe(false)
  })

  test("one pin carrying the whole weight and the other merely touching → false", () => {
    const deep = pin(pinXs[0], TOWER.armHeight - weight / TOWER.kPin)
    const touching = pin(pinXs[1], TOWER.armHeight)
    const c = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), [deep, touching], [])
    expect(c.pinCount).toBe(1)
    expect(c.pinLoad).toBeCloseTo(weight, 3)
    expect(c.pinLoadMin).toBe(0)
    expect(bothPinsLoaded(c.pinCount, c.pinLoadMin, 0.15 * weight)).toBe(false)
  })

  test("no pin on a rail → catch candidate false", () => {
    const c = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), [], [])
    expect(c.pinCount).toBe(0)
    expect(c.pinLoads.length).toBe(0)
    expect(c.pinLoad).toBe(0)
    expect(c.pinLoadMin).toBe(0)
    expect(bothPinsLoaded(c.pinCount, c.pinLoadMin, 0.15 * weight)).toBe(false)
  })
})

describe("solid tower envelope (view/tower.ts audit)", () => {
  test("every visible component group has a conservative, ordered AABB", () => {
    const boxes = towerBoxes(closed)
    const ids = new Set(boxes.map((b) => b.id))
    expect(ids.size).toBe(boxes.length)
    for (const id of [
      "tower-lattice", "tower-floodlights-low", "tower-floodlights-mid", "tower-floodlights-high", "tower-pile-caps", "tower-crown", "carriage", "traverse-beams", "arm-west", "arm-east",
      "arm-west-root", "arm-east-root", "arm-west-trolley", "arm-east-trolley", "ship-qd-arm",
      "launch-mount", "bqd-hood", "stair-tower", "pedestal-west", "pedestal-east", "trench-west", "trench-east",
      "trench-wing-west", "trench-wing-east", "drawworks", "hpu-containers", "electrical-house",
      "pipe-rack", "tank-farm", "subcoolers", "flare-stack", "deluge-tanks",
      "light-mast-sw", "light-mast-se", "light-mast-wnw", "light-mast-ene", "light-mast-nw", "light-mast-ne",
    ]) expect(ids.has(id)).toBe(true)
    for (const b of boxes) {
      expect(b.lo.x).toBeLessThanOrEqual(b.hi.x)
      expect(b.lo.y).toBeLessThanOrEqual(b.hi.y)
      expect(b.lo.z).toBeLessThanOrEqual(b.hi.z)
    }
  })

  test("a hull point inside each component is reported against that component", () => {
    const [xl] = armInnerX(closed)
    const cases: readonly { readonly id: string; readonly p: ReturnType<typeof v3> }[] = [
      { id: "tower-lattice", p: v3(0, TOWER.pos.y, 120) },
      { id: "tower-floodlights-mid", p: v3(0, TOWER.pos.y - 6.7, 82.8) },
      { id: "tower-pile-caps", p: v3(-7, TOWER.pos.y, 0.5) },
      { id: "tower-crown", p: v3(0, TOWER.pos.y - 7.5, 136) },
      { id: "carriage", p: v3(7.5, TOWER.pos.y, 120) },
      { id: "traverse-beams", p: v3(14, 14.2, 107) },
      { id: "arm-west", p: v3(xl - 0.8, 0, 111) },
      { id: "arm-west-root", p: v3(xl - 0.8, 10, 108) },
      { id: "arm-west-trolley", p: v3(-4, 13, 115) },
      { id: "ship-qd-arm", p: v3(-20, 15.6, 95) },
      { id: "launch-mount", p: v3(0, 0, 10) },
      { id: "bqd-hood", p: v3(0, 9.8, 23) },
      { id: "stair-tower", p: v3(-19, 0, 10) },
      { id: "pedestal-west", p: v3(-20, 0, 3) },
      { id: "trench-east", p: v3(9, -30, 2) },
      { id: "drawworks", p: v3(0, 37, 3) },
      { id: "hpu-containers", p: v3(15, 24, 1.5) },
      { id: "electrical-house", p: v3(27, 39, 2) },
      { id: "pipe-rack", p: v3(-50, 22, 3) },
      { id: "tank-farm", p: v3(-130, 12, 10) },
      { id: "subcoolers", p: v3(-189, 12, 6) },
      { id: "flare-stack", p: v3(-212, -40, 20) },
      { id: "deluge-tanks", p: v3(-48, -45, 10) },
      { id: "light-mast-sw", p: v3(-62, -98, 15) },
    ]
    for (const c of cases) {
      const hit = firstHullImpact(closed, [{ p: c.p, p0: c.p, r: 0.1 }])
      expect(hit?.box).toBe(c.id)
    }
  })

  test("a lower-hull station (not just the nose) hits the tower column", () => {
    const atColumn = towerContact(closed, v3(0, 0, 0), [], [{ p: v3(0, TOWER.pos.y - 4.8, 40), v: v3(0, 0, 0), r: 4.5 }])
    expect(atColumn.bodyContact).toBe(true)
    expect(atColumn.F.y).toBeLessThan(-1e5)
  })

  test("closing arms squeeze an off-centre hull: force grows as the gap shrinks", () => {
    const hullAt = (open: number): ReturnType<typeof towerContact> =>
      towerContact({ open, cmdClose: open > 0 }, v3(0, yMid, 0), [], [{ p: v3(4.4, yMid, TOWER.armHeight - 1), v: v3(0, 0, 0), r: 4.5 }])
    const early = hullAt(0.5)
    const late = hullAt(0.2)
    expect(early.F.x).toBeLessThan(0) // pushed west, away from the east arm's inner face
    expect(late.F.x).toBeLessThan(early.F.x)
    expect(late.bodyContact).toBe(true)
  })
})

describe("swept hull guard (fast strike, no pass-through)", () => {
  const r = 4.5
  const mass = 120e3

  test("a 50 m/s column crossing is detected by the swept sphere, not the endpoint alone", () => {
    const p0 = v3(0, 10, 60)
    const p = v3(0, 36, 60) // the centre ends past the north face of the column
    const hit = firstHullImpact(closed, [{ p, p0, r }])
    if (!hit) throw new Error("swept column crossing was not detected")
    expect(hit.box).toBe("tower-lattice")
    expect(hit.t).toBeGreaterThan(0)
    expect(hit.t).toBeLessThan(1)
    expect(hit.n.y).toBeLessThan(0) // outward toward the incoming (south) side
    const resp = resolveHullImpact(closed, [{ p, p0, r }], mass, v3(0, 50, 0))
    if (!resp) throw new Error("column impact was not resolved")
    const after = add(v3(0, 50, 0), resp.dv)
    expect(dot(after, hit.n)).toBeGreaterThan(0) // inward velocity removed, small rebound
    expect(dot(v3(0, 50, 0), hit.n)).toBeLessThan(-40)
    expect(resp.push.y).toBeLessThan(0)
  })

  test("a 50 m/s crossing of the 1.6 m arm is stopped at the first-touch plane, never inside", () => {
    const [, xr] = armInnerX(closed)
    const outer = xr + TOWER.armWidth
    let p = v3(15, 0, TOWER.armHeight - 1)
    let v = v3(-50, 0, 0)
    const dt = 0.01
    let reversed = false
    let box = ""
    for (let k = 0; k < 30; k++) {
      const p0 = p
      p = add(p, scale(v, dt))
      const resp = resolveHullImpact(closed, [{ p, p0, r }], mass, v)
      if (resp) {
        box = resp.impact.box
        v = add(v, resp.dv)
        p = add(p, resp.push)
        reversed = true
      }
      // the sphere stays east of the east arm's outer face: no endpoint is ever inside the arm
      expect(p.x).toBeGreaterThanOrEqual(outer + r - 1e-6)
      expect(p.y).toBeGreaterThan(-TOWER.armLength)
    }
    expect(reversed).toBe(true)
    expect(box).toBe("arm-east")
    expect(v.x).toBeGreaterThan(0)
  })

  test("a fast column strike loops without ever leaving the station inside the column box", () => {
    let p = v3(0, 10, 60)
    let v = v3(0, 50, 0)
    const dt = 0.01
    let bumped = false
    for (let k = 0; k < 30; k++) {
      const p0 = p
      p = add(p, scale(v, dt))
      const resp = resolveHullImpact(closed, [{ p, p0, r }], mass, v)
      if (resp) {
        bumped = true
        v = add(v, resp.dv)
        p = add(p, resp.push)
      }
      // column south face is TOWER.pos.y - 6.1; the 4.5 m hull stays south of it
      expect(p.y).toBeLessThanOrEqual(TOWER.pos.y - 6.1 - r + 1e-6)
    }
    expect(bumped).toBe(true)
    expect(v.y).toBeLessThan(0)
  })
})

describe("caught ship support after engine cut", () => {
  test("two pin springs alone hold the weight for 20 s (no thrust, no adhesion)", () => {
    const m = 120e3
    const dt = 0.004
    let z = TOWER.armHeight - 0.3
    let vz = -2
    let last = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), [{ p: v3(pinXs[0], yMid, z), v: v3(0, 0, vz) }, { p: v3(pinXs[1], yMid, z), v: v3(0, 0, vz) }], [])
    for (let k = 0; k < 5000; k++) {
      last = towerContact(closed, v3(0, yMid, TOWER.armHeight + 10), [{ p: v3(pinXs[0], yMid, z), v: v3(0, 0, vz) }, { p: v3(pinXs[1], yMid, z), v: v3(0, 0, vz) }], [])
      expect(z).toBeGreaterThan(TOWER.armHeight - 1.5) // stays inside the pin contact band
      expect(last.pinLoad).toBeGreaterThanOrEqual(0) // pins can only push
      vz += (last.F.z / m - G) * dt
      z += vz * dt
    }
    const delta = (m * G) / (2 * TOWER.kPin)
    expect(last.pinCount).toBe(2)
    expect(z).toBeCloseTo(TOWER.armHeight - delta, 2)
    expect(Math.abs(vz)).toBeLessThan(0.02)
  })
})
