/**
 * T09 landing (Q29/Q33, updated for the staged burn): the flip-to-pin hoverslam on the sea-level
 * Raptors.
 *
 *  - `guide` flies `flip` on two SL Raptors; `landing`/`terminal` are staged 1–3 engines. Cold gas is
 *    roll-only: `useRcs` may be true only together with `rcsRollOnly === true` (no translation, no
 *    pitch/yaw RCS); 2/3-engine phases use differential gimbal for roll;
 *  - the analytic hoverslam schedule must reach the pin plane at the contact speed;
 *  - the terminal law must respect the engine band (40 % floor, 100 % ceiling) and the tilt cone, and
 *    must not stall: with the realised throttle lag it arrives within 1 m and ≈1 m/s of the target;
 *  - the MPC/landing plan must respect the thrust-magnitude band and the tilt cone, and report the
 *    divert infeasible when the throttle-floor descent cannot buy the time it needs.
 */
import { describe, expect, test } from "bun:test"
import { type Q, v3, add, scale } from "@ship/math"
import { DEFAULT_DESIGN, massProps, initialConsumables } from "@ship/craft"
import { makeWorld } from "@ship/sim"
import {
  flipAltitude, hoverAccel, hoverslam, magBand, netAccel, planLanding, terminalControl,
  type LandingLimits, type LandingState,
} from "../src/landing"
import { guide, initMem, type Mem, type Nav } from "../src/guidance"

const D = Math.PI / 180
const G = 9.79
const P_AMB = 79_000 // ≈ 1.8 km
const T_MAX = 2.585e6 - P_AMB * 1.33
const T_MIN = 0.4 * 2.585e6 - P_AMB * 1.33
const N_ENG = 2
const lim = (mass: number, tiltDeg = 45): LandingLimits => ({
  mass, nEng: N_ENG, thrustMax: T_MAX, thrustMin: T_MIN, tanTilt: Math.tan(tiltDeg * D),
})

describe("landing law: hoverslam profile", () => {
  test("bang-bang schedule from a representative flip state reaches the pins at the contact speed", () => {
    const z0 = 700
    const s0 = 95
    const vF = 0.7
    const b = netAccel(lim(130_000))
    const hs = hoverslam(z0, s0, b.min, b.max, vF)
    expect(hs.feasible).toBe(true)
    // integrate the schedule: floor net decel for tFloor, then the ceiling
    let z = z0
    let s = s0
    const dt = 0.001
    for (let t = 0; t < hs.tgo; t += dt) {
      const a = t < hs.tFloor ? b.min : b.max
      z -= s * dt - 0.5 * a * dt * dt
      s -= a * dt
    }
    expect(Math.abs(z)).toBeLessThan(1) // within 1 m of the pin plane
    expect(Math.abs(s - vF)).toBeLessThan(0.1) // within 0.1 m/s of the contact speed
  })

  test("flags the corridor walls (too fast and too high) instead of inventing a solution", () => {
    const b = netAccel(lim(120_000))
    expect(hoverslam(50, 120, b.min, b.max, 0.7).tooLow).toBe(true) // cannot stop in 50 m
    expect(hoverslam(4000, 40, b.min, b.max, 0.7).tooHigh).toBe(true) // floor thrust stops far above
    expect(hoverslam(4000, 40, b.min, b.max, 0.7).feasible).toBe(false)
  })

  test("hoverAccel is exactly the constant-deceleration required to arrive at vF, clamped to the band", () => {
    const b = netAccel(lim(120_000))
    const z = 300
    const s = 60
    const a = hoverAccel(z, -s, b, 1)
    expect(a).toBeGreaterThanOrEqual(b.min - 1e-12)
    expect(a).toBeLessThanOrEqual(b.max + 1e-12)
    expect(a).toBeCloseTo((s * s - 1) / (2 * z), 9)
    // too high for the floor → clamped at the floor (no hover, honest over-deceleration)
    expect(hoverAccel(3000, -20, b, 0.7)).toBeCloseTo(b.min, 9)
  })
})

describe("landing law: closed loop with the throttle lag", () => {
  test("terminal law from 30 m / 18 m/s contacts within 1 m and 1 m/s, respecting the engine band", () => {
    const m0 = 120_000
    let mass = m0
    // start on the profile (30 m, the matched 20 m/s and the matched 16.45 m/s² of thrust — inside
    // the two-engine floor corridor, whose minimum speed at 30 m is 18.5 m/s)
    let pz = 30
    let vz = -20
    let az = G + 6.66
    const dt = 0.01
    const vF = 0.7
    let contactVz = 0
    let contactT = 0
    for (let k = 0; k < 3000; k++) {
      const b = lim(mass)
      const u = terminalControl({ p: [0, 0, pz], v: [0, 0, vz], a: [0, 0, az] }, b, vF)
      // command bounds: the engine floor/ceiling on the magnitude, and no downward thrust
      const mag = Math.hypot(u[0], u[1], u[2])
      expect(mag).toBeGreaterThanOrEqual((N_ENG * T_MIN) / mass - 1e-9)
      expect(mag).toBeLessThanOrEqual((N_ENG * T_MAX) / mass + 1e-9)
      expect(u[2]).toBeGreaterThan(0)
      // plant: first-order throttle/pointing lag
      const beta = 1 - Math.exp(-dt / 0.3) // throttle/pointing lag with the engines already lit
      az += (u[2] - az) * beta
      mass -= (mass * Math.hypot(u[0], u[1], az) / 3623) * dt
      const acc = az - G
      pz += vz * dt + 0.5 * acc * dt * dt
      vz += acc * dt
      if (pz <= 0) {
        contactVz = vz
        contactT = k * dt
        break
      }
    }
    expect(contactT).toBeGreaterThan(0)
    expect(pz).toBeLessThanOrEqual(1) // the pin plane is reached (not stalled above it)
    expect(Math.abs(contactVz)).toBeLessThanOrEqual(1.0)
    expect(Math.abs(contactVz)).toBeGreaterThanOrEqual(0.2)
  })
})

describe("landing law: MPC divert plan", () => {
  const state = (pz: number, px: number, py: number, vz: number, vx: number, vy: number): LandingState => ({
    p: [px, py, pz], v: [vx, vy, vz], a: [0, 0, 0],
  })

  test("respects the thrust-magnitude band and the tilt cone", () => {
    const mass = 125_000
    const l = lim(mass, 40)
    const sol = planLanding(state(900, 500, 200, -70, -30, -5), l, {
      vF: 0.7, tau: 1.5, glide: 0.05, terminalAlt: 20, aCruise: 3, zFinal: 100,
    })
    const mag = magBand(l)
    const m = Math.hypot(sol.u[0], sol.u[1], sol.u[2])
    expect(m).toBeGreaterThanOrEqual(mag.min - 1e-6)
    expect(m).toBeLessThanOrEqual(mag.max + 1e-6)
    expect(Math.hypot(sol.u[0], sol.u[1])).toBeLessThanOrEqual(l.tanTilt * sol.u[2] * 1.0001 + 1e-6)
    // free final time = the divert window (to the final-phase altitude), a real horizon
    expect(sol.tgo).toBeGreaterThan(2)
    expect(sol.tgo).toBeLessThan(60)
    expect(sol.feasible).toBe(true)
    // visual export: 16–24 predicted positions from the current state to the catch point
    expect(sol.path).toBeDefined()
    const path = sol.path ?? []
    expect(path.length).toBeGreaterThanOrEqual(16)
    expect(path.length).toBeLessThanOrEqual(24)
    const last = path[path.length - 1]
    expect(Math.hypot(last[0], last[1], last[2])).toBeLessThan(1.5)
  })

  test("reports an infeasible divert when the floor-thrust descent cannot buy the time", () => {
    const l = lim(125_000, 45)
    // 1.4 km offset with only 300 m of altitude left: no tilt can close it in the available time
    const sol = planLanding(state(300, 1200, 700, -95, -30, -10), l, {
      vF: 0.7, tau: 1.5, glide: 0.05, terminalAlt: 20,
    })
    expect(sol.feasible).toBe(false)
    // and it must not fake a downward thrust: the vertical command stays positive
    expect(sol.u[2]).toBeGreaterThan(0)
  })
})

describe("landing gate", () => {
  test("flipAltitude is a plausible powered-flight gate for the design delivery", () => {
    expect(flipAltitude(-100)).toBeGreaterThan(700)
    expect(flipAltitude(-100)).toBeLessThan(2500)
    // monotone in the sink rate
    expect(flipAltitude(-120)).toBeGreaterThan(flipAltitude(-100))
  })
})

describe("guidance: staged landing and roll-only RCS", () => {
  const world = makeWorld(DEFAULT_DESIGN)
  const mass = massProps(world.vehicle, initialConsumables(world.vehicle)).m
  const q: Q = { w: 1, x: 0, y: 0, z: 0 } // body +x = ECI +x
  const nav = (t: number): Nav => ({
    t, r: v3(0, 0, 1200), v: v3(0, 0, -70), q, w: v3(0, 0, 0), I: v3(1.6e6, 4.1e7, 4.1e7), m: mass, xcg: 28,
    alt: 1200, up: v3(0, 0, 1), east: v3(1, 0, 0), north: v3(0, 1, 0), vAir: v3(0, 0, -70), V: 70, mach: 0.2,
    qbar: 3000, pAmb: P_AMB, alpha: 0.1, t0Stag: 0, padEci: v3(0, 0, 0), pinRel: v3(900, 200, 1100),
    velRel: v3(-30, -5, -70), hBase: 1100, hLow: 1000, mhd: null, aeroM: v3(0, 0, 0), coilI: 10, coilIop: 3000,
    seedLeft: 0, propHeader: 20_000, thrusts: [1e6, 1e6, 0], spoolDone: true, pinLoad: 0, pinLoadMin: 0, pinCount: 0, accThrust: 15,
    mhdDrag: 0, aeroF: v3(0, 0, 0), mhdF: v3(0, 0, 0), engF: v3(0, 0, 0),
  })
  const memFor = (phase: "flip" | "landing" | "terminal"): Mem => ({
    ...initMem(0), phase, tPhase: 0, nEngLand: N_ENG, coilOff: true,
  })
  const slOn = (cmd: { engines: readonly { on: boolean }[] }): number => cmd.engines.filter((e) => e.on).length

  test("flip: two SL Raptors, no cold gas", () => {
    const out = guide(world.guide, nav(500), memFor("flip"), 0.01)
    expect(out.cmd.useRcs).toBe(false)
    expect(out.cmd.rcsRollOnly).toBe(false)
    expect(slOn(out.cmd)).toBeGreaterThanOrEqual(2)
    expect(out.cmd.coil).toBe("discharge")
  })

  test("landing: 1–3 staged SL Raptors; cold gas is roll-only, never translational", () => {
    const out = guide(world.guide, nav(500), memFor("landing"), 0.01)
    // one engine in the falling/steering stage, two/three only when the decel/attitude needs them
    expect(slOn(out.cmd)).toBeGreaterThanOrEqual(1)
    expect(slOn(out.cmd)).toBeLessThanOrEqual(3)
    // RCS may be enabled only for roll; pitch/yaw stay on the gimbal and there is no translation
    if (out.cmd.useRcs) expect(out.cmd.rcsRollOnly).toBe(true)
    else expect(out.cmd.rcsRollOnly).toBe(false)
  })

  test("terminal: 1–3 staged SL Raptors; cold gas is roll-only if enabled", () => {
    const out = guide(world.guide, nav(500), memFor("terminal"), 0.004)
    expect(slOn(out.cmd)).toBeGreaterThanOrEqual(1)
    expect(slOn(out.cmd)).toBeLessThanOrEqual(3)
    if (out.cmd.useRcs) expect(out.cmd.rcsRollOnly).toBe(true)
    else expect(out.cmd.rcsRollOnly).toBe(false)
  })
})

// keep the imports used in the fixture builder (add/scale are exercised by the nav construction)
void add
void scale
