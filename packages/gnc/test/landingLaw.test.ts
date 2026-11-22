/**
 * Landing law, closed-loop, without wind.
 *
 * Point-mass plant driven by the law's own thrust command (perfect attitude), with the real landing
 * aero database, the real engine thrust limits and the real mass. This isolates the guidance/model
 * fidelity from the 6-DOF attitude loop: if the model is complete, the residual the disturbance
 * observer sees is small and the ship reaches the pins.
 */
import { describe, expect, test } from "bun:test"
import { v3, add, norm, scale } from "@ship/math"
import { DEFAULT_DESIGN, fVac100, makeVehicle, massProps, initialConsumables } from "@ship/craft"
import { makeWorld } from "@ship/sim"
import { guideLanding, LANDING_LAW_STATE0, type LandingLawState, selectEngines, trimThrustLine, monitorEngines, ENGINE_HEALTH0 } from "../src/index"
import { landingAero } from "../src/tables"

const world = makeWorld(DEFAULT_DESIGN)
const veh = makeVehicle(DEFAULT_DESIGN)
const cons = initialConsumables(veh)
const mass = massProps(veh, cons).m
const eng = veh.engines[0]
const pAmb = 101325
const thrustMax = fVac100(eng) - pAmb * eng.exitArea
const thrustMin = eng.minThrottle * fVac100(eng) - pAmb * eng.exitArea
const rho = 1.225

describe("landing law, closed loop, no wind", () => {
  test("reaches the pins from a reachable off-axis state, with a small model residual", () => {
    let p = v3(300, -150, 800)
    let v = v3(-20, 10, -60)
    let st: LandingLawState = LANDING_LAW_STATE0
    let resMax = 0, steps = 0
    const dt = 0.01
    let prevU = v3(0, 0, 9.79)
    while (p.z > 0 && steps < 30000) {
      const uMag = norm(prevU)
      const xB = uMag > 0.5 ? scale(prevU, 1 / uMag) : v3(0, 0, 1)
      const aero = (xh: typeof xB, va: typeof v) => landingAero(world.guide.landAero!, rho, xh, va, massProps(veh, cons).xcg).F
      const out = guideLanding({
        t: steps * dt, rel: p, vel: v, mass, uReal: prevU, uAxis: prevU, uAxisDot: null, xBody: xB,
        nEng: 2, thrustMax, thrustMin, ve: 369.25 * 9.80665, pAe: pAmb * eng.exitArea,
        wind: v3(0, 0, 0), aeroForce: aero, contactSpeed: 0.7,
      }, st)
      st = out.state
      resMax = Math.max(resMax, Math.hypot(out.residual.x, out.residual.y, out.residual.z))
      prevU = out.u
      const drag = scale(aero(xB, v), 1 / mass)
      v = add(v, scale(add(add(out.u, drag), v3(0, 0, -9.79)), dt))
      p = add(p, scale(v, dt))
      steps++
    }
    console.log(`touchdown after ${(steps * dt).toFixed(1)} s at (${p.x.toFixed(1)}, ${p.y.toFixed(1)}, ${p.z.toFixed(1)}) m, |v| ${norm(v).toFixed(2)} m/s, residual max ${resMax.toFixed(2)} m/s²`)
    expect(Math.hypot(p.x, p.y)).toBeLessThan(5)
    expect(Math.abs(v.z + 0.7)).toBeLessThan(1.5)
    expect(resMax).toBeLessThan(0.6)
  }, 120000)
})

describe("engine-out reconfiguration", () => {
  test("a failed engine is excluded and replaced by the spare", () => {
    expect(selectEngines([false, false, false], [], 3)).toEqual([0, 1, 2])
    expect(selectEngines([false, true, false], [0, 1, 2], 2)).toEqual([0, 2])
    expect(selectEngines([false, true, true], [0, 1, 2], 2)).toEqual([0])
    expect(selectEngines([true, true, true], [0, 1, 2], 2)).toEqual([])
  })

  test("an engine commanded on that produces no thrust is flagged after its start sequence", () => {
    let h = ENGINE_HEALTH0
    h = monitorEngines(h, 10, [0, 1], [2.0e6, 2.0e6], 2.5, 0.9e6).health
    expect(h.failed).toEqual([false, false, false])
    h = monitorEngines(h, 13.2, [0, 1], [2.0e6, 0], 2.5, 0.9e6).health
    expect(h.failed).toEqual([false, true, false])
  })

  test("the trimmed thrust line passes through the CG: symmetric set = body axis, 2 of 3 = canted", () => {
    const sym = trimThrustLine(veh.engines.slice(0, 3), [1e6, 1e6, 1e6], 27.6)
    expect(Math.abs(sym.x - 1)).toBeLessThan(1e-6)
    const two = trimThrustLine(veh.engines.slice(0, 3), [1e6, 1e6, 0], 27.6)
    // engines 0 and 1 are symmetric about the body x axis at ±60°: the centroid is opposite engine 2
    expect(two.x).toBeGreaterThan(0.99)
    expect(Math.hypot(two.y, two.z)).toBeGreaterThan(0.01)
    const one = trimThrustLine(veh.engines.slice(0, 3), [1e6, 0, 0], 27.6)
    expect(Math.hypot(one.y, one.z)).toBeGreaterThan(0.03)
  })
})
