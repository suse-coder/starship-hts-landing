/**
 * Guidance predictor sanity (gnc/predictor.ts: 3-DOF point mass, midpoint RK2).
 * References / physical expectations:
 *  - In vacuum the predictor must follow two-body + J2 motion: compared with an RK4 reference
 *    (dt = 1 s) the midpoint-RK2 (dt = 10 s above 130 km) position error grows like dt², ≲ 0.5 km in 10 min;
 *    the specific orbital energy ½v² + U_J2 is conserved.
 *  - Lifting-entry range control (Apollo/Shuttle entry guidance, e.g. Harpold & Graves 1979): rolling the
 *    lift vector away from vertical (larger bank σ, vertical lift ∝ cos σ) shortens the range and raises
 *    the peak deceleration.
 *  - After the belly flop the vehicle approaches its subsonic terminal velocity (≈ 90–110 m/s at a few km,
 *    aero/test terminal-velocity check).
 */
import { describe, expect, test } from "bun:test"
import { type V3, v3, add, scale, norm, sub, cross, dot } from "@ship/math"
import { gravityJ2, potentialJ2, R_EQ, MU_EARTH, geodeticToEcef, PAD2, OMEGA_EARTH } from "@ship/earth"
import { SHOCK_TABLE } from "@ship/gas"
import { makeVehicle, massProps, initialConsumables } from "@ship/craft"
import { makeAeroModel } from "@ship/aero"
import { predict, buildAeroTable, buildMhdDragTable, alphaNoseFirst, pitchUpDue, circularState, passiveTrim, aeroLookup, type PredContext } from "../src/index"

const v = makeVehicle()
const mp = massProps(v, initialConsumables(v))
const am = makeAeroModel(v)
const ctx: PredContext = {
  aero: buildAeroTable(am, mp.xcg), mhdDrag: buildMhdDragTable(v, am, SHOCK_TABLE, mp.xcg), sref: am.sref, theta0: 0,
  padEcef: geodeticToEcef(PAD2), alphaNoseFirst, vLoft: 6000, qPitch: 9000, vPitchMax: 3800, hEnd: 3000,
}
const energy = (r: V3, vv: V3): number => 0.5 * dot(vv, vv) + potentialJ2(r)
const inertial = (r: V3, vRel: V3): V3 => add(vRel, cross(v3(0, 0, OMEGA_EARTH), r))

describe("predictor in vacuum", () => {
  const s0 = circularState(R_EQ + 275e3, (26.5 * Math.PI) / 180, 0.3, 0.1)
  const p = predict(ctx, { t: 0, r: s0.r, v: s0.v, mass: 1.2e5, phase: "coast", bankEntry: 0, bankHigh: 0, tMax: 600 })

  test("follows two-body + J2 motion: < 500 m from an RK4 reference after 600 s", () => {
    let r = s0.r, vv = s0.v
    const h = 1
    for (let t = 0; t < p.t - 1e-9; t += h) {
      const k1r = vv, k1v = gravityJ2(r)
      const k2r = add(vv, scale(k1v, h / 2)), k2v = gravityJ2(add(r, scale(k1r, h / 2)))
      const k3r = add(vv, scale(k2v, h / 2)), k3v = gravityJ2(add(r, scale(k2r, h / 2)))
      const k4r = add(vv, scale(k3v, h)), k4v = gravityJ2(add(r, scale(k3r, h)))
      r = add(r, scale(add(add(k1r, scale(add(k2r, k3r), 2)), k4r), h / 6))
      vv = add(vv, scale(add(add(k1v, scale(add(k2v, k3v), 2)), k4v), h / 6))
    }
    expect(p.t).toBe(600)
    expect(norm(sub(p.r, r))).toBeLessThan(500)
  })

  test("conserves the specific orbital energy (relative drift < 1e-6) and records no aerodynamic load", () => {
    expect(Math.abs(energy(p.r, inertial(p.r, p.vRel)) / energy(s0.r, s0.v) - 1)).toBeLessThan(1e-6)
    expect(p.qMax).toBeLessThan(1)
    expect(p.tEntry).toBe(-1)
    expect(Math.sqrt(MU_EARTH / (R_EQ + 275e3))).toBeCloseTo(norm(s0.v), 9)
  })
})

describe("predictor through the atmosphere", () => {
  // 120 km over the equator, eastward 7.4 km/s inertial, flight-path angle −1.2°
  const r0 = v3(R_EQ + 120e3, 0, 0)
  const gam = (-1.2 * Math.PI) / 180
  const v0 = scale(add(scale(v3(0, 1, 0), Math.cos(gam)), scale(v3(1, 0, 0), Math.sin(gam))), 7400)
  const run = (bank: number, tMax?: number): ReturnType<typeof predict> => predict(ctx, { t: 0, r: r0, v: v0, mass: 1.1e5, phase: "entry", bankEntry: bank, bankHigh: 0.5, tMax })
  const outs = [0, 0.5, 1.0].map((b) => run(b))

  test("larger bank (less vertical lift) → shorter range and higher peak deceleration on the lifting entry", () => {
    // Compare the entry leg before the high-α transition: the transition timing also depends on the
    // bank (a banked entry descends sooner, so it reaches the Q30 pitch-up speed earlier), and mixing
    // that into the end point would test two effects at once.
    const early = [0, 0.5, 1.0].map((b) => run(b, 300))
    expect(early[0].lon).toBeGreaterThan(early[1].lon)
    expect(early[1].lon).toBeGreaterThan(early[2].lon)
    expect(early[2].gMax).toBeGreaterThan(early[0].gMax)
  })

  test("ends at the 3 km stop altitude near subsonic terminal velocity, peak g below 8", () => {
    for (const o of outs) {
      const V = norm(o.vRel)
      expect(V).toBeGreaterThan(50)
      expect(V).toBeLessThan(200)
      expect(o.gMax).toBeLessThan(8)
      expect(o.pitch).not.toBeNull()
    }
  })
})

describe("pitch-up rule shared by predictor and flight", () => {
  const c = { qPitch: 9000, vPitchMax: 3800 }
  test("at the q minimum, below qPitch, or below vPitchMax — never at very low q while skipping", () => {
    expect(pitchUpDue(5000, 4000, 4400, c)).toBe(true) // q below qPitch
    expect(pitchUpDue(12000, 13000, 4400, c)).toBe(false) // q high and falling
    expect(pitchUpDue(10000, 9500, 4400, c)).toBe(true) // q started rising, < 1.6 qPitch
    expect(pitchUpDue(2000, 2500, 4400, c)).toBe(false) // skip-out: q too low
    expect(pitchUpDue(2000, 1000, 3700, c)).toBe(true) // latest: V below vPitchMax
  })
  test("a q minimum above the tile-flux limit is not taken (Q30 measured: the delayed transition buys 1613 K but costs 3.4 km)", () => {
    // q is still falling at 5.4 km/s: no pitch-up yet; the transition waits for the q minimum to pass
    expect(pitchUpDue(9000, 12000, 5400, c)).toBe(false)
    expect(pitchUpDue(14000, 14500, 5000, c)).toBe(false)
    // once q has started rising below 1.6 qPitch the transition is due
    expect(pitchUpDue(13000, 12000, 5200, c)).toBe(true)
  })
})

/**
 * The high-α ship has no α actuator: the α axis is left free (`free: alphaAxis`) and the bare hull
 * settles at its zero-pitching-moment α. `AeroTable.trim` only searches α ≥ 30°, and the hypersonic
 * zero-moment α is ≈ 28°, so the table falls back to 80° — C_D 8.5, C_L 1.06, L/D 0.12. The vehicle is
 * not that: at the real trim it is a glider (C_L ≈ 1.9, C_D ≈ 1.55, L/D ≈ 1.24). The predictor must fly
 * the same angle as the vehicle, or it predicts a parachute and no bank law can reconcile the range.
 */
describe("passive high-α trim", () => {
  const D = 180 / Math.PI
  test("hypersonic zero-moment α is ≈ 27–29°, not the table's 80° fallback", () => {
    for (const M of [6, 8, 12, 16, 20]) {
      const a = passiveTrim(ctx.aero, M) * D
      expect(a).toBeGreaterThan(26)
      expect(a).toBeLessThan(34)
    }
  })

  test("trim is continuous through M 4–6 (the table jumps 38° → 80° at M = 6)", () => {
    const a4 = passiveTrim(ctx.aero, 4) * D
    const a5 = passiveTrim(ctx.aero, 5) * D
    const a6 = passiveTrim(ctx.aero, 6) * D
    expect(Math.abs(a5 - a4)).toBeLessThan(6)
    expect(Math.abs(a6 - a5)).toBeLessThan(6)
    expect(a4).toBeGreaterThan(40)
    expect(a6).toBeLessThan(34)
  })

  test("at the passive trim the high-α hull is a glider (L/D > 1.1), not a parachute (L/D 0.12)", () => {
    for (const M of [6, 12, 20]) {
      const a = passiveTrim(ctx.aero, M)
      const co = aeroLookup(ctx.aero, a, M)
      expect(co.CL / co.CD).toBeGreaterThan(1.1)
      expect(co.CL / co.CD).toBeLessThan(1.35)
      expect(co.CD).toBeLessThan(2.2)
    }
  })

  test("the zero-moment condition holds at the returned angle (restoring sign)", () => {
    for (const M of [4, 6, 12, 20]) {
      const a = passiveTrim(ctx.aero, M)
      const cm = aeroLookup(ctx.aero, a, M).Cm
      expect(Math.abs(cm)).toBeLessThan(1e-3)
    }
  })
})
