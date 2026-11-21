import { type V3, v3 } from "@ship/math"
import { MU_EARTH, OMEGA_EARTH, R_EQ } from "@ship/earth"
import { geodeticToEcef, PAD2, type Geodetic } from "@ship/earth"
import { FLIGHT15, gmst, julianDate } from "@ship/earth"
import { type PredContext, predict, type PredOut } from "./predictor"

/**
 * Pre-mission targeting. The Flight 15 ship was in a ~275 km orbit for ~14 revolutions and was
 * caught at Pad 2 at 2026-11-14 21:20:06 UTC. We start one deorbit-preparation interval before
 * the burn and solve the orbit's RAAN Ω and argument of latitude u0 (2-D Newton on the
 * predictor's downrange/crossrange miss) and the epoch (so the catch happens at the real time).
 */
export interface MissionPlan {
  readonly epochJd: number
  readonly theta0: number
  readonly r0: V3
  readonly v0: V3
  readonly inclination: number
  readonly raan: number
  readonly u0: number
  readonly altitude: number
  readonly tBurn: number // s after epoch (burn centre)
  readonly dvBurn: number
  readonly bankEntry0: number
  readonly pred: PredOut
  readonly iterations: number
  readonly pad: Geodetic
}

export const circularState = (a: number, inc: number, raan: number, u: number): { r: V3; v: V3 } => {
  const cO = Math.cos(raan), sO = Math.sin(raan), ci = Math.cos(inc), si = Math.sin(inc), cu = Math.cos(u), su = Math.sin(u)
  const vc = Math.sqrt(MU_EARTH / a)
  return {
    r: v3(a * (cO * cu - sO * su * ci), a * (sO * cu + cO * su * ci), a * su * si),
    v: v3(vc * (-cO * su - sO * cu * ci), vc * (-sO * su + cO * cu * ci), vc * cu * si),
  }
}

export interface PlanOptions {
  readonly altitude: number
  readonly inclination: number
  readonly tBurn: number
  readonly dvBurn: number
  readonly bankEntry0: number
  readonly mass: number
  readonly tailAfterPredEnd: number // s from predictor end (3 km) to catch
  readonly aim: number // m past the pad (same bias as the entry guidance)
}

export const DEFAULT_PLAN: PlanOptions = {
  altitude: FLIGHT15.orbitAltitudeKm * 1000,
  inclination: (26.5 * Math.PI) / 180,
  tBurn: 480,
  // Q28 (143.65 t vehicle, corrected high-α trim + T08 fixes). With the bare hull actually trimming at
  // ≈ 29° (L/D ≈ 1.2, not the 80°/0.12 the table fallback assumed) the entry has much more range than
  // the old model, and the high-α phase must fly lift-down (|σ| > 90°) to dissipate it. That forces the
  // mandatory crossrange reversal to roll through lift-vertical at ~20 kPa, a ~250 m/s vertical impulse.
  // A slightly steeper deorbit (102 m/s) removes the excess energy: less negative lift, an earlier and
  // cheaper reversal, and a lower horizontal speed at the M = 2 belly-flop hand-over (which is the
  // terminal braking-distance floor). Measured 2026-11-21: 5.74 g, 24.0 kPa, flip miss 762 m at 766 m.
  dvBurn: 108,
  bankEntry0: (35 * Math.PI) / 180,
  mass: 115000,
  tailAfterPredEnd: 75,
  aim: 180e3,
}

export const planMission = (ctxFor: (theta0: number, padEcef: V3) => PredContext, o: PlanOptions = DEFAULT_PLAN): MissionPlan => {
  const padEcef = geodeticToEcef(PAD2)
  const a = R_EQ + o.altitude
  const n = Math.sqrt(MU_EARTH / a ** 3)
  const catchJd = julianDate(FLIGHT15.catchUtc)
  let duration = 4200
  let raan = 0
  let u0 = 0
  let pred: PredOut | null = null
  let iterations = 0
  for (let pass = 0; pass < 3; pass++) {
    const epochJd = catchJd - duration / 86400
    const theta0 = gmst(epochJd)
    const ctx = ctxFor(theta0, padEcef)
    // first guess: orbit plane through the pad (ascending branch) at the predicted end time
    const uPad = Math.asin(Math.min(1, Math.sin(PAD2.lat) / Math.sin(o.inclination)))
    const tEnd = duration - o.tailAfterPredEnd
    const raEci = PAD2.lon + theta0 + OMEGA_EARTH * tEnd
    if (pass === 0) {
      raan = raEci - Math.atan2(Math.cos(o.inclination) * Math.sin(uPad), Math.cos(uPad))
      u0 = uPad - n * tEnd + 0.9 // atmospheric flight is slower than orbital: start further along
    }
    const run = (rn: number, uu: number): PredOut => {
      const s = circularState(a, o.inclination, rn, uu)
      return predict(ctx, { t: 0, r: s.r, v: s.v, mass: o.mass, phase: "coast", bankEntry: o.bankEntry0, bankHigh: 0.5, impulse: { t: o.tBurn, dv: o.dvBurn } })
    }
    for (let k = 0; k < 14; k++) {
      iterations++
      const p0 = run(raan, u0)
      pred = p0
      if (Math.abs(p0.downrangeErr - o.aim) < 2000 && Math.abs(p0.crossrangeErr) < 2000) break
      const du = 2e-3, dO = 2e-3
      const pu = run(raan, u0 + du)
      const pO = run(raan + dO, u0)
      const J11 = (pu.downrangeErr - p0.downrangeErr) / du, J12 = (pO.downrangeErr - p0.downrangeErr) / dO
      const J21 = (pu.crossrangeErr - p0.crossrangeErr) / du, J22 = (pO.crossrangeErr - p0.crossrangeErr) / dO
      const det = J11 * J22 - J12 * J21
      const e1 = p0.downrangeErr - o.aim
      let dU = -(J22 * e1 - J12 * p0.crossrangeErr) / det
      let dRa = -(-J21 * e1 + J11 * p0.crossrangeErr) / det
      const lim = 0.2
      const sc = Math.max(1, Math.abs(dU) / lim, Math.abs(dRa) / lim)
      dU /= sc
      dRa /= sc
      u0 += dU
      raan += dRa
    }
    const newDuration = (pred?.t ?? duration) + o.tailAfterPredEnd
    if (Math.abs(newDuration - duration) < 5) break
    // keep the same ground track: shifting the epoch rotates the Earth, compensate RAAN
    raan += OMEGA_EARTH * (newDuration - duration) * 0 // theta0 is recomputed from the epoch
    duration = newDuration
  }
  const epochJd = catchJd - duration / 86400
  const theta0 = gmst(epochJd)
  const s = circularState(a, o.inclination, raan, u0)
  const finalPred = predict(ctxFor(theta0, padEcef), { t: 0, r: s.r, v: s.v, mass: o.mass, phase: "coast", bankEntry: o.bankEntry0, bankHigh: 0.5, impulse: { t: o.tBurn, dv: o.dvBurn }, record: true })
  return {
    epochJd, theta0, r0: s.r, v0: s.v, inclination: o.inclination, raan, u0, altitude: o.altitude,
    tBurn: o.tBurn, dvBurn: o.dvBurn, bankEntry0: o.bankEntry0, pred: finalPred, iterations, pad: PAD2,
  }
}
