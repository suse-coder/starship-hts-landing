/**
 * Tower collision through the real step function: a ship on a 50 m/s collision course with the
 * Mechazilla structure must bump (swept impulse or penalty contact) and must not pass through the
 * column plane. This complements the tower-level box/sweep tests with the actual RK4 + substep path.
 */
import { describe, expect, test } from "bun:test"
import { add, qFromBasis, scale, v3 } from "@ship/math"
import { corotation, rotZ } from "@ship/earth"
import { massProps } from "@ship/craft"
import { TOWER } from "@ship/tower"
import { CH, initState, makeWorld, step, type SimState } from "../src/index"

const world = makeWorld()
const theta = world.plan.theta0
const padEci = rotZ(world.padEcef, theta)
const padE = rotZ(world.padEnu.e, theta)
const padN = rotZ(world.padEnu.n, theta)
const padU = rotZ(world.padEnu.u, theta)
const fromPad = (x: ReturnType<typeof v3>): ReturnType<typeof v3> => add(add(scale(padE, x.x), scale(padN, x.y)), scale(padU, x.z))

describe("tower collision through step", () => {
  test("50 m/s northbound hull bumps the carriage/column and never crosses the column plane", () => {
    const s0 = initState(world)
    const r = add(padEci, fromPad(v3(0, -20, 80))) // 20 m south of the pad centre, 80 m up
    const q = qFromBasis(padU, padE, padN) // body +x up, +y east, +z north
    const v = add(corotation(r), fromPad(v3(0, 50, 0)))
    let s: SimState = { ...s0, t: 0, r, v, q, w: v3(0, 0, 0), mem: { ...s0.mem, phase: "lost", reason: "collision test" } }

    let bumpT = -1
    let maxN = -Infinity
    let minN = Infinity
    let lastVN = 0
    for (let k = 0; k < 400; k++) {
      const out = step(world, s)
      const n = out.row[CH.pn]
      if (out.row[CH.hullBump] > 0 && bumpT < 0) bumpT = out.row[CH.t]
      maxN = Math.max(maxN, n)
      minN = Math.min(minN, n)
      lastVN = out.row[CH.vn]
      s = out.s
    }
    // the ship started at n = −20 and moved north into the tower
    expect(minN).toBeLessThan(-19)
    expect(bumpT).toBeGreaterThanOrEqual(0) // the swept guard fired
    // the CG never crosses the column's south face (n = TOWER.pos.y − 6.1) minus the 4.5 m hull
    expect(maxN).toBeLessThanOrEqual(TOWER.pos.y - 6.1 - 4.5 + 0.2)
    // and the normal velocity is reversed by the strike, not carried through
    expect(lastVN).toBeLessThan(0)
  }, 30000)

  test("a caught ship (engines off) stays on the two pin springs", () => {
    const s0 = initState(world)
    const mp = massProps(world.vehicle, s0.cons)
    const cgU = TOWER.armHeight - (world.vehicle.pinX - mp.xcg)
    const r = add(padEci, fromPad(v3(0, 0, cgU)))
    const q = qFromBasis(padU, padE, padN) // body +x up, +y east over the two rails
    let s: SimState = {
      ...s0, t: 0, r, v: corotation(r), q, w: v3(0, 0, 0),
      arms: { open: 0, cmdClose: true },
      mem: { ...s0.mem, phase: "caught", armsClose: true, caughtT: 0, reason: "" },
    }
    let maxBody = 0
    let maxBump = 0
    let both = true
    let minZ = Infinity
    let lastLoad = 0
    let lastVU = 0
    for (let k = 0; k < 1000; k++) {
      const out = step(world, s)
      maxBody = Math.max(maxBody, out.row[CH.bodyContact])
      maxBump = Math.max(maxBump, out.row[CH.hullBump])
      both = both && out.row[CH.pinCount] === 2
      minZ = Math.min(minZ, out.row[CH.pinZ])
      lastLoad = out.row[CH.pinLoad]
      lastVU = out.row[CH.vu]
      s = out.s
    }
    expect(maxBody).toBe(0) // no hull structure contact while resting on the pins
    expect(maxBump).toBe(0)
    expect(both).toBe(true)
    expect(minZ).toBeGreaterThan(-1.5) // the pins never leave the contact band
    expect(Math.abs(lastLoad / (mp.m * 9.80665) - 1)).toBeLessThan(0.01) // both pin springs carry the weight
    expect(Math.abs(lastVU)).toBeLessThan(0.05)
  }, 30000)
})
