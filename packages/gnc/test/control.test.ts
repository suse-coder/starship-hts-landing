/**
 * spec/05 T-alloc, T-gimbal and the attitude law.
 * References / physical expectations:
 *  - Allocation must reproduce the commanded torque through the *plant* models (craft engineThrust with
 *    exact gimbal trigonometry, rcsWrench, mhdApply), not just the allocator's own linear model.
 *  - Thrust-vector moments M = r × F (r from the CG to the gimbal); a single off-axis engine cannot
 *    produce independent roll, so the roll it creates is limited to the cold-gas capacity (roll priority).
 *  - spec/05 T-alloc: commanded +My realised within 1 % when MHD authority suffices.
 *    T-gimbal: |τx| ≤ cap always; unconstrained solution when feasible.
 *  - Rigid-body attitude: I ω̇ = τ − ω × Iω, q̇ = ½ q⊗(0,ω); a rate-command PD law with ζ ≈ 0.9 converges
 *    without sustained oscillation and respects its slew-rate limit.
 */
import { describe, expect, test } from "bun:test"
import {
  type Q, type V3, v3, norm, sub, add, scale, cross, hadamard, normalize, qFromAxisAngle, qrot, qDot, qAddScaled, qnormalize,
  angleBetween, q4,
} from "@ship/math"
import { atmosphere } from "@ship/earth"
import { lookupStag, SHOCK_TABLE } from "@ship/gas"
import { makeVehicle, massProps, initialConsumables, engineThrust, sumEngines, rcsWrench, DEFAULT_DESIGN } from "@ship/craft"
import { makeMhdModel, mhdBasis, ZERO_BASIS } from "@ship/mhd"
import { allocateGimbal, allocateGimbals, allocateRcs, allocateMhd, torqueCommand, attitudeError, GAINS, alphaNoseFirst, type Gains } from "../src/index"

const v = makeVehicle()
const mp = massProps(v, { ...initialConsumables(v), loxMain: 0, ch4Main: 0 })
const SL = v.engines.slice(0, 3)
const DEG = Math.PI / 180
const rel = (a: number, b: number): number => Math.abs(a / b - 1)

/** realised engine moment from the plant model at the allocated gimbals (vacuum, steady thrust) */
const plantMoment = (thrusts: readonly number[], gp: readonly number[], gy: readonly number[]): V3 =>
  sumEngines(SL.map((e, i) => engineThrust(e, { on: thrusts[i] > 0, spool: thrusts[i] > 0 ? 1 : 0, throttle: thrusts[i] / (e.thrustSL + 101325 * e.exitArea), gp: gp[i], gy: gy[i] }, 0, mp.xcg))).M

describe("cold-gas RCS allocation", () => {
  test("realised plant torque equals the command within capacity, with zero net force (pure couples)", () => {
    // capacities: pitch/yaw T·(x_nose − x_aft) = 4 kN × 39.1 m = 156.4 kN m, roll 2 R T = 36 kN m
    for (const tau of [v3(1e4, 5e4, -3e4), v3(-2e4, -1e5, 1.2e5), v3(0, 0, 1e3)]) {
      const r = allocateRcs(v.thrusters, v.couples, mp.xcg, tau, true)
      r.duty.forEach((d) => {
        expect(d).toBeGreaterThanOrEqual(0)
        expect(d).toBeLessThanOrEqual(1)
      })
      const w = rcsWrench(v, r.duty, mp.xcg, 100)
      expect(norm(sub(w.M, tau))).toBeLessThan(1e-6 * norm(tau))
      expect(norm(w.F)).toBeLessThan(1e-6)
    }
  })

  test("saturates at the couple capacity with the commanded sign; disabled → nothing", () => {
    // one couple: nose thruster T·(x_n − x_cg) + aft thruster T·(x_cg − x_a) = T·(x_n − x_a), independent of the CG
    const cap = DEFAULT_DESIGN.rcsThrust * (v.thrusters[0].pos.x - v.thrusters[4].pos.x)
    const r = allocateRcs(v.thrusters, v.couples, mp.xcg, v3(0, 1e8, 0), true)
    expect(r.achieved.y).toBeCloseTo(cap, 3)
    const n = allocateRcs(v.thrusters, v.couples, mp.xcg, v3(-1e8, 0, -1e8), true)
    expect(n.achieved.x).toBeCloseTo(-2 * 4.5 * DEFAULT_DESIGN.rcsThrust, 3)
    expect(n.achieved.z).toBeCloseTo(-cap, 3)
    expect(allocateRcs(v.thrusters, v.couples, mp.xcg, v3(0, 1e8, 0), false).duty.every((d) => d === 0)).toBe(true)
  })
})

describe("thrust-vector allocation", () => {
  test("three engines (differential gimbal): plant torque within 1 % of the 3-axis command", () => {
    const T = [2.2e6, 2.2e6, 2.2e6]
    for (const tau of [v3(2e5, 4e6, -3e6), v3(-5e5, -2e6, 1e6), v3(0, 6e6, 0)]) {
      const g = allocateGimbals(SL, T, mp.xcg, tau)
      expect(norm(sub(plantMoment(T, g.gp, g.gy), tau)) / norm(tau)).toBeLessThan(0.01)
    }
  })

  test("gimbal limit ±15° respected for an infeasible command", () => {
    const g = allocateGimbals(SL, [2.2e6, 2.2e6, 2.2e6], mp.xcg, v3(1e8, 1e9, -1e9))
    for (const a of [...g.gp, ...g.gy]) expect(Math.abs(a)).toBeLessThanOrEqual(15 * DEG + 1e-12)
  })

  test("T-gimbal (single engine): unconstrained pitch/yaw solution when feasible (plant within 1 % of |τ|)", () => {
    const T = [0, 0, 2.0e6]
    for (const tau of [v3(0, 1e6, 0.5e6), v3(0, -8e5, 2e5)]) {
      const g = allocateGimbal(SL, T, mp.xcg, tau)
      const M = plantMoment(T, [0, 0, g.gp], [0, 0, g.gy])
      expect(Math.hypot(M.y - tau.y, M.z - tau.z) / norm(tau)).toBeLessThan(0.01)
    }
  })

  test("T-gimbal (single engine): the roll it makes stays within the cold-gas cap at gimbal angles ≤ 3°", () => {
    const T = [0, 0, 2.0e6]
    const cap = 5e4
    for (const tau of [v3(0, 3e6, -2e6), v3(0, -4e6, -4e6), v3(0, 2e5, 1e5)]) {
      const g = allocateGimbal(SL, T, mp.xcg, tau, cap)
      expect(Math.max(Math.abs(g.gp), Math.abs(g.gy))).toBeLessThan(3 * DEG)
      const M = plantMoment(T, [0, 0, g.gp], [0, 0, g.gy])
      expect(Math.abs(M.x)).toBeLessThanOrEqual(cap * 1.01)
    }
    // feasible within the cap → identical to the unconstrained solution
    const small = v3(0, 2e5, 1e5)
    const a = allocateGimbal(SL, T, mp.xcg, small, cap), b = allocateGimbal(SL, T, mp.xcg, small)
    expect(Math.abs(plantMoment(T, [0, 0, b.gp], [0, 0, b.gy]).x)).toBeLessThan(cap)
    expect(a.gp).toBe(b.gp)
    expect(a.gy).toBe(b.gy)
  })

  // Fixed (T12): gimbal angles were recovered with gp = asin(s1) although the plant thrust direction is
  // (cos gp cos gy, sin gy, −sin gp cos gy). Now gy = asin(s2), gp = asin(s1/cos gy), and the axial
  // component T·√(1 − s1² − s2²) is used in τy, τz (fixed point) — the allocator model equals the plant.
  test("T-gimbal (single engine): |τx| ≤ cap always — also at large gimbal angles", () => {
    const T = [0, 0, 2.0e6]
    const cap = 5e4
    for (const tau of [v3(0, 1e5, 9e6), v3(0, -9e6, 7e6)]) {
      const g = allocateGimbal(SL, T, mp.xcg, tau, cap)
      expect(Math.abs(plantMoment(T, [0, 0, g.gp], [0, 0, g.gy]).x)).toBeLessThanOrEqual(cap * 1.001)
    }
  })

  test("large gimbal (11° and 15°): plant torque within 1 % of the command, single and three engines", () => {
    const L = mp.xcg - SL[0].gimbal.x
    for (const deg of [11, 15]) {
      const s = Math.sin(deg * DEG) * 0.999 // just inside the 15° box
      // single engine, no roll cap: a pure pitch, a pure yaw and a diagonal command at |gimbal| ≈ deg
      const T1 = [0, 0, 2.0e6]
      for (const [a, b] of [[1, 0], [0, 1], [Math.SQRT1_2, Math.SQRT1_2]]) {
        const z = SL[2].gimbal.z, y = SL[2].gimbal.y
        const sp = s * a, sy = s * b, c = Math.sqrt(1 - sp * sp - sy * sy)
        const tau = v3(-T1[2] * (y * sp + z * sy), T1[2] * (z * c - L * sp), -T1[2] * (y * c + L * sy))
        const g = allocateGimbal(SL, T1, mp.xcg, tau)
        const M = plantMoment(T1, [0, 0, g.gp], [0, 0, g.gy])
        expect(Math.hypot(M.y - tau.y, M.z - tau.z) / Math.hypot(tau.y, tau.z)).toBeLessThan(0.01)
        // total thrust deflection acos(cos gp cos gy) ≈ deg
        expect(Math.acos(Math.cos(g.gp) * Math.cos(g.gy))).toBeGreaterThan((deg - 1) * DEG)
      }
      // three engines: common pitch near deg plus some roll (kept feasible: the differential roll gimbal
      // must not push any engine onto the 15° stop)
      const T3 = [2.2e6, 2.2e6, 2.2e6]
      const tau3 = v3(3e5, 3 * 2.2e6 * L * s * 0.85, -1e6)
      const g3 = allocateGimbals(SL, T3, mp.xcg, tau3)
      expect(Math.max(...g3.gp.map(Math.abs))).toBeGreaterThan((deg - 2) * DEG)
      expect(norm(sub(plantMoment(T3, g3.gp, g3.gy), tau3)) / norm(tau3)).toBeLessThan(0.01)
    }
  })
})

describe("MHD electrode allocation (T-alloc)", () => {
  const model = makeMhdModel(v)
  const at = atmosphere(65000)
  const U = 7000, al = 10 * DEG, q = 0.5 * at.rho * U * U
  const basis = mhdBasis(model, {
    NI: v.coil.NI, vhat: normalize(v3(Math.cos(al), 0, -Math.sin(al))), U, rhoInf: at.rho, pInf: at.p, qInf: q,
    stag: lookupStag(SHOCK_TABLE, 65000, U), xcg: mp.xcg, kappa: DEFAULT_DESIGN.hallClosure, seedFraction: DEFAULT_DESIGN.seedFraction,
  })
  test("commanded +My realised within 1 % up to 30 % of the authority (65 km, 7 km/s, α 10°)", () => {
    const max = allocateMhd(basis, v3(0, 1e12, 0), q, 2.5, 0).maxMoment
    expect(max).toBeGreaterThan(1e5)
    for (const f of [0.02, 0.1, 0.3]) {
      const a = allocateMhd(basis, v3(0, f * max, 0), q, 2.5, 0)
      expect(rel(a.out.M.y, f * max)).toBeLessThan(0.01)
      expect(Math.hypot(a.out.M.x, a.out.M.z)).toBeLessThan(0.02 * f * max)
      expect(a.saturated).toBe(false)
      a.u.forEach((u) => {
        expect(u).toBeGreaterThanOrEqual(0)
        expect(u).toBeLessThanOrEqual(1.6)
      })
    }
  })

  // Formerly a known bug: the momentum-bound fixed point sc ← scale(u(M_cmd/sc)) stopped after 4 passes
  // (−6.0 % at 60 %, −16.8 % at 90 % of maxMoment with `saturated` false). mhdAlloc.ts now solves the
  // scalar equation φ(s) = scale(u(M_cmd/s)) − s by bracketed regula falsi to 0.2 %, and `saturated` is
  // set when the realised moment misses the command by more than 1 %.
  test("T-alloc: commanded +My realised within 1 % at 60 % and 90 % of the authority", () => {
    const max = allocateMhd(basis, v3(0, 1e12, 0), q, 2.5, 0).maxMoment
    for (const f of [0.1, 0.6, 0.9]) {
      const a = allocateMhd(basis, v3(0, f * max, 0), q, 2.5, 0)
      expect(a.saturated).toBe(false)
      expect(rel(a.out.M.y, f * max)).toBeLessThan(0.01)
      expect(Math.hypot(a.out.M.x, a.out.M.z)).toBeLessThan(0.02 * f * max)
    }
  })

  test("demand beyond authority is flagged saturated; no plasma → no allocation", () => {
    const a = allocateMhd(basis, v3(0, 1e9, 0), q, 2.5, 0)
    expect(a.saturated).toBe(true)
    expect(a.out.M.y).toBeLessThanOrEqual(a.maxMoment * 1.05)
    const z = allocateMhd(ZERO_BASIS, v3(0, 1e6, 0), q, 2.5, 0)
    expect(norm(z.out.M)).toBe(0)
  })
})

describe("attitude law on a rigid body", () => {
  const I = mp.I
  /** RK4 rigid body with a zero-order-hold torque law at 100 Hz */
  const simulate = (q0: Q, gains: Gains, target: Parameters<typeof torqueCommand>[3], tEnd: number): { q: Q; w: V3; maxRate: number; errs: number[] } => {
    let q = q0
    let w = v3(0, 0, 0)
    let maxRate = 0
    const errs: number[] = []
    const dt = 0.01
    for (let k = 0; k < tEnd / dt; k++) {
      const { tau } = torqueCommand(q, w, I, target, gains, v3(0, 0, 0))
      const f = (qq: Q, ww: V3): { dq: Q; dw: V3 } => {
        const c = cross(ww, hadamard(I, ww))
        return { dq: qDot(qq, ww), dw: v3((tau.x - c.x) / I.x, (tau.y - c.y) / I.y, (tau.z - c.z) / I.z) }
      }
      const k1 = f(q, w), k2 = f(qAddScaled(q, k1.dq, dt / 2), add(w, scale(k1.dw, dt / 2)))
      const k3 = f(qAddScaled(q, k2.dq, dt / 2), add(w, scale(k2.dw, dt / 2))), k4 = f(qAddScaled(q, k3.dq, dt), add(w, scale(k3.dw, dt)))
      q = qnormalize(q4(
        q.w + (dt / 6) * (k1.dq.w + 2 * k2.dq.w + 2 * k3.dq.w + k4.dq.w), q.x + (dt / 6) * (k1.dq.x + 2 * k2.dq.x + 2 * k3.dq.x + k4.dq.x),
        q.y + (dt / 6) * (k1.dq.y + 2 * k2.dq.y + 2 * k3.dq.y + k4.dq.y), q.z + (dt / 6) * (k1.dq.z + 2 * k2.dq.z + 2 * k3.dq.z + k4.dq.z),
      ))
      w = add(w, scale(add(add(k1.dw, scale(add(k2.dw, k3.dw), 2)), k4.dw), dt / 6))
      maxRate = Math.max(maxRate, Math.hypot(w.y, w.z))
      if (k % 10 === 0) errs.push(angleBetween(qrot(q, v3(1, 0, 0)), target.x))
    }
    return { q, w, maxRate, errs }
  }

  test("attitudeError is the body-frame rotation vector that closes a pure pitch offset", () => {
    const q = qFromAxisAngle(v3(0, 1, 0), 10 * DEG)
    const e = attitudeError(q, { x: v3(1, 0, 0), z: v3(0, 0, 1) })
    expect(e.y).toBeCloseTo(-10 * DEG, 10)
    expect(Math.abs(e.x) + Math.abs(e.z)).toBeLessThan(1e-12)
  })

  test("pitch-up gains: 20° error converges below 0.05° in 15 s, slew ≤ ω_max, no sustained oscillation", () => {
    const target = { x: v3(1, 0, 0), z: v3(0, 0, 1) }
    const r = simulate(qFromAxisAngle(normalize(v3(0, 1, 0.4)), 20 * DEG), GAINS.pitchUp, target, 15)
    expect(r.errs[r.errs.length - 1]).toBeLessThan(0.05 * DEG)
    expect(r.maxRate).toBeLessThanOrEqual(GAINS.pitchUp.wMax * 1.1)
    // error decreases monotonically once below 5° (overdamped-like: ζ = 0.9 with a rate limit)
    const i5 = r.errs.findIndex((e) => e < 5 * DEG)
    for (let k = i5 + 1; k < r.errs.length; k++) expect(r.errs[k]).toBeLessThanOrEqual(r.errs[k - 1] + 0.01 * DEG)
    expect(norm(r.w)).toBeLessThan(1e-3)
  })

  test("roll target is acquired (full attitude, pitch-up gains, no roll deadband)", () => {
    const target = { x: v3(1, 0, 0), z: v3(0, 0, 1) }
    const r = simulate(qFromAxisAngle(v3(1, 0, 0), 8 * DEG), GAINS.pitchUp, target, 60)
    expect(angleBetween(qrot(r.q, v3(0, 0, 1)), v3(0, 0, 1))).toBeLessThan(0.05 * DEG)
  })

  test("MHD-phase gains hold roll inside the designed rate deadband: |e_roll| ≤ rollBand·2ζ/ω_n (≈ 1.2°)", () => {
    // gnc/control.ts: no roll torque while |ω_c − ω| < rollBand and |e| < 5·rollBand (saves cold gas)
    const target = { x: v3(1, 0, 0), z: v3(0, 0, 1) }
    const g = GAINS.mhd
    const r = simulate(qFromAxisAngle(v3(1, 0, 0), 8 * DEG), g, target, 60)
    const band = g.rollBand / (g.wn / (2 * g.zeta))
    const e = angleBetween(qrot(r.q, v3(0, 0, 1)), v3(0, 0, 1))
    expect(e).toBeLessThanOrEqual(band + 0.05 * DEG)
    expect(e).toBeLessThan(2 * DEG)
  })
})

describe("guidance schedule", () => {
  test("nose-first α schedule stays inside the coil trim envelope 10–14° (gnc/schedule.ts)", () => {
    expect(alphaNoseFirst(7800) / DEG).toBeCloseTo(10, 12)
    expect(alphaNoseFirst(4000) / DEG).toBeCloseTo(14, 12)
    for (let V = 1000; V < 8000; V += 100) {
      const a = alphaNoseFirst(V) / DEG
      expect(a).toBeGreaterThanOrEqual(10)
      expect(a).toBeLessThanOrEqual(14)
    }
  })
})
