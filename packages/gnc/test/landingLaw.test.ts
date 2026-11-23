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

// ---------------------------------------------------------------------------------------------
// single-engine vertical descent ("slam"): the pure pieces of the law
// ---------------------------------------------------------------------------------------------
import { pinLoop, SLAM, slamCommand, slamPredict, slamProfile, slamSinkLimit, slamTiltCap, slamTube, slewLat } from "../src/landingLaw"

describe("slam: pin-loop pole placement", () => {
  test("the closed loop s⁴ + 2ζω s³ + ω² s² + ω²k_d s + ω²k_p is (s + p)⁴", () => {
    const g = pinLoop(0.8)
    const p = g.p
    // coefficients of (s + p)^4: 4p, 6p², 4p³, p⁴
    expect(2 * g.zeta * g.wn).toBeCloseTo(4 * p, 9)
    expect(g.wn * g.wn).toBeCloseTo(6 * p * p, 9)
    expect(g.wn * g.wn * g.kd).toBeCloseTo(4 * p ** 3, 9)
    expect(g.wn * g.wn * g.kp).toBeCloseTo(p ** 4, 9)
  })

  test("the law it replaced (ω 0.8, ζ 1, k_p 0.25, k_d 1.1 on the CG) fails the Routh test", () => {
    // s⁴ + a3 s³ + a2 s² + a1 s + a0 is stable iff a3·a2 > a1 and a3·a2·a1 > a1² + a3²·a0
    const w = 0.8, z = 1.0, kp = 0.25, kd = 1.1
    const a3 = 2 * z * w, a2 = w * w, a1 = w * w * kd, a0 = w * w * kp
    expect(a3 * a2 > a1).toBe(true)
    expect(a3 * a2 * a1 > a1 * a1 + a3 * a3 * a0).toBe(false) // unstable
    const g = pinLoop()
    const b3 = 2 * g.zeta * g.wn, b2 = g.wn * g.wn, b1 = b2 * g.kd, b0 = b2 * g.kp
    expect(b3 * b2 * b1 > b1 * b1 + b3 * b3 * b0).toBe(true) // the pole placement is stable
  })
})

describe("slam: vertical profile and limits", () => {
  const env = { ez: 0, lo: 7.2, hi: 19.6, contactSpeed: 0.7 }
  test("the profile ends at the contact speed and is continuous at the end-segment height", () => {
    expect(slamProfile(0, 0.7).v).toBeCloseTo(-0.7, 9)
    expect(slamProfile(SLAM.hEnd - 1e-9, 0.7).v).toBeCloseTo(slamProfile(SLAM.hEnd + 1e-9, 0.7).v, 6)
    expect(slamProfile(500, 0.7).a).toBe(SLAM.aD)
  })
  test("falls at the throttle floor above the profile and brakes on it (never before)", () => {
    // 1000 m up at −40 m/s the braking curve is at −122 m/s: the engine stays at its floor
    const fall = slamCommand({ x: 0, y: 0, z: 1000 }, { x: 0, y: 0, z: 0 }, -40, env)
    expect(fall.braking).toBe(false)
    expect(fall.u.z).toBe(env.lo)
    const onCurve = slamCommand({ x: 0, y: 0, z: 300 }, { x: 0, y: 0, z: 0 }, slamProfile(300, 0.7).v - 2, env)
    expect(onCurve.braking).toBe(true)
    expect(onCurve.u.z).toBeGreaterThan(9.81 + 0.8 * SLAM.aD)
  })
  test("the lateral command points at the axis and respects the tilt cap", () => {
    const c = slamCommand({ x: 40, y: 0, z: 500 }, { x: 0, y: 0, z: 0 }, -60, env)
    expect(c.u.x).toBeLessThan(0)
    expect(Math.atan2(Math.hypot(c.u.x, c.u.y), c.u.z)).toBeLessThanOrEqual(slamTiltCap(500) + 1e-9)
    expect(slamTiltCap(2000)).toBeGreaterThan(slamTiltCap(200))
  })
  test("the predicted plan reaches the rails near the contact speed without stopping on the way", () => {
    const pl = slamPredict(0, { rel: { x: 3, y: -2, z: 1200 }, vPin: { x: 0, y: 0, z: 0 }, vz: -25, tilt: { e: 0, n: 0 }, tiltRate: { e: 0, n: 0 }, aNow: 7.2, mass: 128e3, ve: 3620, pAe: 1.3e5, slew: 11.8 }, env)
    const n = pl.z.length
    expect(pl.z[n - 1]).toBe(0)
    expect(Math.abs(pl.vz[n - 1])).toBeLessThan(2)
    expect(Math.hypot(pl.x[n - 1], pl.y[n - 1])).toBeLessThan(0.3)
    let up = 0, slow = 0
    for (let i = 1; i < n; i++) { if (pl.vz[i] > 0) up++; if (pl.z[i] > 5 && pl.vz[i] > -2) slow++ }
    expect(up).toBe(0) // never climbs
    expect(slow).toBe(0) // never hovers above the last metres
    expect(pl.fuel).toBeGreaterThan(8e3)
    expect(pl.fuel).toBeLessThan(16e3)
  })
  test("constraint band: narrows with the time to go, never below the rail tolerance; sink limit grows with height", () => {
    expect(slamTube(0, 0.8, 1.2)).toBeCloseTo(0.3, 9)
    expect(slamTube(20, 0.8, 1.2)).toBeGreaterThan(slamTube(5, 0.8, 1.2))
    expect(slamTube(5, 0.8, 1.2)).toBeGreaterThan(slamTube(1, 0.8, 1.2))
    expect(slamSinkLimit(1000, 19.6, 0.7)).toBeLessThan(slamSinkLimit(100, 19.6, 0.7))
    expect(slamSinkLimit(0, 19.6, 0.7)).toBeCloseTo(-0.7, 9)
  })
  test("the lateral command is slew-limited, faster when it shrinks", () => {
    const grow = slewLat(0, 0, 1, 0, 1)
    expect(grow.x).toBeCloseTo(SLAM.jerk, 9)
    const shrink = slewLat(1, 0, 0, 0, 1)
    expect(1 - shrink.x).toBeCloseTo(3 * SLAM.jerk, 9)
  })
})
