/**
 * Fast 3-DOF evaluator — entry hand-over → **re-planned** belly flop → landing divert, in milliseconds.
 *
 * The full 6-DOF mission is the only ground truth (the plan/plant mismatch is exactly what a 3-DOF
 * model cannot see: L's PDG plan closed 674 m to 0 m while the integrated run diverged by 1047 m).
 * This evaluator exists to *rank* candidates, never to certify: the same point-mass physics as the
 * flight's own planner, with a first-order thrust-direction lag standing in for the torque-limited
 * attitude loop.
 *
 * The belly flop is **re-planned** from the hand-over state: the same two-segment lift-azimuth search
 * as `guidance.ts:steepGlide` (headings, τ grid, fine pass), then the plan is flown to the *flip
 * gate* (`flipAltitude`, which depends on the tuning gate) — not sampled at a gate crossing. The
 * landing starts from the flip state with the flip transient (vertical Δv + ballistic fall), then
 * flies the two-phase hoverslam profile with the lateral divert and the engine floor/tilt cone from
 * `landing.ts`.
 */
import { type V3, v3, add, clamp, dot, norm, scale, sub, DEG } from "@ship/math"
import { atmosphere, OMEGA_EARTH, PAD2, rotZ } from "@ship/earth"
import { type VehicleConfig, fVac100 } from "@ship/craft"
import { type PredContext, predict, solveBank } from "./predictor"
import { bellyEnd, type BellyInput } from "./guidance"
import { assemble, flipAltitude, G_LAND, magBand, terminalControl, type LandingLimits, type LandingState } from "./landing"
import { MISSION_TUNING_DEFAULT, type MissionTuning } from "./tuning"

/** the entry hand-over state the belly flop starts from (pad ENU, pin station) */
export interface FastHandover {
  readonly t: number
  /** pin station relative to the catch point (pad ENU, m) */
  readonly pinRel: V3
  /** ground-relative velocity (pad ENU, m/s) */
  readonly velRel: V3
  /** aft-end height above the pad ground (m) */
  readonly hBase: number
  readonly mass: number
  /** airspeed at the hand-over (m/s) */
  readonly V: number
  /** lift azimuth flown at the hand-over (rad from north, clockwise); the plan starts from it */
  readonly psi0: number
}

export interface FastFlip {
  readonly t: number
  readonly pinRel: V3
  readonly velRel: V3
  readonly hBase: number
  readonly mass: number
  readonly V: number
  readonly vz: number
}

export interface FastContact {
  readonly t: number
  readonly pinRel: V3
  readonly velRel: V3
  readonly lat: number
  readonly across: number
  readonly along: number
  readonly vE: number
  readonly vN: number
  readonly vU: number
  readonly contactSpeed: number
  readonly feasible: boolean
  readonly climbed: boolean
}

export interface FastBellyPlan {
  readonly psi1: number
  readonly psi2: number
  readonly tau: number
  /** the plan's predicted miss at the belly plan end (m) */
  readonly miss: number
}

export interface FastPrediction {
  readonly bellyPlan: FastBellyPlan
  readonly bellyMiss: number
  readonly flip: FastFlip
  readonly contact: FastContact
  /** continuous catch miss (m-equivalent, same shape as `sim/score.ts`) */
  readonly margin: number
  readonly timingMs: { readonly belly: number; readonly landing: number; readonly total: number }
}

const D = Math.PI / 180
/** thrust-corridor schedule (same shape as guidance.ts, from the tuning) */
const tiltDegFor = (pz: number, T: MissionTuning): number =>
  pz > 300 ? T.tilt300mDeg
    : pz > 120 ? T.tilt120mDeg
      : pz > 60 ? T.tilt60mDeg
        : pz > 35 ? T.tilt35mDeg
          : pz > 12 ? T.tilt12mDeg
            : T.tilt0mDeg

/** Re-plan the belly flop from the hand-over, then fly the chosen plan to the flip gate.
 *  The plan and the flight both call guidance's `bellyEnd` — the same point-mass law the flight's
 *  own `steepGlide` uses, so the evaluator cannot drift from the planner. The search is the flight's
 *  (12 headings; if > 40 m add the τ/±45…180° grid and a fine pass), minimised against the tuning aim
 *  offset. Returns the flip state, not a gate-crossing sample. */
export const flyBelly = (hand: FastHandover, pred: PredContext, T: MissionTuning): { plan: FastBellyPlan; flip: FastFlip } => {
  const rate = T.bellyBankRateDeg * DEG
  const alignH = T.bellyAlignH
  const planH = T.bellyPlanEnd
  const b: BellyInput = {
    pinRel: hand.pinRel, velRel: hand.velRel, hBase: hand.hBase, mass: hand.mass,
    windOfs: v3(0, 0, 0), aero: pred.aero, sref: pred.sref,
    windForecast: pred.windForecast ?? (() => v3(0, 0, 0)),
  }
  // Position-only objective here: this evaluator's 3-DOF landing has ideal lateral authority and no
  // propellant budget, so it measures the belly's *reach*, not the 6-DOF catch. The flight's
  // steepGlide adds the landing ZEM and the lateral-velocity penalty (guidance.ts endToEndMiss).
  const miss = (a: number, tau: number, b2: number): number => {
    const e = bellyEnd(b, a, tau, b2, hand.psi0, planH, rate, alignH)
    return Math.hypot(e.x - T.bellyAimE, e.y - T.bellyAimN)
  }
  interface Plan { readonly a: number; readonly tau: number; readonly b: number; readonly e: number }
  const pick = (ps: readonly Plan[]): Plan => ps.reduce((x, y) => (y.e < x.e ? y : x))
  const heads = Array.from({ length: 12 }, (_, i) => hand.psi0 + (i - 6) * 30 * D)
  let best = pick(heads.map((a) => ({ a, tau: Infinity, b: a, e: miss(a, Infinity, a) })))
  if (best.e > 40) {
    const taus = [8, 16, 32, 50, 70, 95, 125]
    const two = heads.flatMap((a) => [45, 90, 180, -45, -90].flatMap((db) => taus.map((tau) => ({ a, tau, b: a + db * D, e: miss(a, tau, a + db * D) }))))
    best = pick([best, ...two])
  }
  const b0 = best
  const fine = [[-8, 0, 1], [8, 0, 1], [0, -15, 1], [0, 15, 1], [0, 0, 0.7], [0, 0, 1.4]]
    .map(([da, dbb, k]) => ({ a: b0.a + da * D, b: b0.b + dbb * D, tau: b0.tau * k }))
    .map((x) => ({ ...x, e: miss(x.a, x.tau, x.b) }))
  const chosen = pick([best, ...fine])
  const plan: FastBellyPlan = { psi1: chosen.a, psi2: chosen.b, tau: chosen.tau, miss: chosen.e }
  // fly the plan to the *current-sink* flip gate (guidance checks it every control step). The gate
  // depends on the sink at the crossing, so a short fixed-point iteration settles it.
  let hIgn = flipAltitude(hand.velRel.z, T)
  let end = bellyEnd(b, plan.psi1, plan.tau, plan.psi2, hand.psi0, hIgn, rate, alignH, 0.1)
  for (let k = 0; k < 4; k++) {
    const next = flipAltitude(end.vz, T)
    if (Math.abs(next - hIgn) < 0.5) break
    hIgn = next
    end = bellyEnd(b, plan.psi1, plan.tau, plan.psi2, hand.psi0, hIgn, rate, alignH, 0.1)
  }
  const pinRel = v3(end.x, end.y, 0)
  const velRel = v3(end.vx, end.vy, end.vz)
  const wind = pred.windForecast ?? (() => v3(0, 0, 0))
  const V = norm(sub(velRel, wind(Math.max(end.h, 0))))
  const flip: FastFlip = { t: hand.t + end.t, pinRel, velRel, hBase: Math.max(end.h, 0), mass: hand.mass, V, vz: end.vz }
  return { plan, flip }
}

interface LandingStep {
  readonly p: V3
  readonly v: V3
  readonly d: V3
}

/**
 * Fly the landing burn from the flip state: the flip transient (vertical Δv + ballistic fall), then
 * the two-phase vertical profile + free-final-time lateral divert, with a first-order direction lag
 * at the attitude bandwidth and the engine floor/tilt cone. Pure point mass, no aero: the 6-DOF gate
 * decides how much the real plant diverges.
 */
export const flyLanding = (flip: FastFlip, vehicle: VehicleConfig, T: MissionTuning, lateralEff: number = LATERAL_EFFICIENCY): FastContact => {
  const eng = vehicle.engines[0]
  // start after the flip rotation: the gate model's vertical Δv and ballistic fall
  const vz0 = flip.velRel.z + T.flipDv
  const h0 = Math.max(flip.hBase - T.flipFall * Math.abs(flip.velRel.z), 2)
  const p0 = v3(flip.pinRel.x, flip.pinRel.y, h0)
  const v0 = v3(flip.velRel.x, flip.velRel.y, vz0)
  let st: LandingStep = { p: p0, v: v0, d: v3(0, 0, 1) }
  let mass = flip.mass
  const isp = Math.max(eng.isp0, 100)
  const atm0 = atmosphere(Math.max(h0, 0))
  const tMaxN = fVac100(eng) - atm0.p * eng.exitArea
  const tMinN = eng.minThrottle * fVac100(eng) - atm0.p * eng.exitArea
  const sHand = Math.max(-vz0, 0.7)
  const mag0 = magBand({ mass: flip.mass, nEng: T.landingEngines, thrustMax: tMaxN, thrustMin: tMinN, tanTilt: 1 })
  const aBrake = Math.max(mag0.min - G_LAND, 0.6)
  const aCruise = clamp((sHand * sHand - T.contactSpeed * T.contactSpeed) / (2 * Math.max(h0, 1)), T.landingACruiseMin, T.landingACruiseMax)
  const zProf = clamp(
    (sHand * sHand - T.contactSpeed * T.contactSpeed - 2 * aCruise * h0) / (2 * Math.max(aBrake - aCruise, 0.2)),
    T.landingZProfMin, T.landingZProfMax,
  )
  const dt = 0.02
  let t = 0
  let prev = st
  let climbed = false
  // the achieved-direction lag: the MPC's assumed pointing lag (tau) is the plant measurement the
  // 6-DOF divergence exposed, not the idealised 1/wn step response
  const tauHigh = Math.max(0.3, T.landingTauHigh)
  const tauLow = Math.max(0.25, T.landingTauLow)
  const rateMax = T.landingWMaxDeg * DEG
  for (let i = 0; i < 5000; i++) {
    const pz = st.p.z
    if (pz <= 0) break
    if (pz < 200 && st.v.z > 3) { climbed = true; break }
    const alt = Math.max(pz, 0)
    const atm = atmosphere(alt)
    const tMax = fVac100(eng) - atm.p * eng.exitArea
    const tMin = eng.minThrottle * fVac100(eng) - atm.p * eng.exitArea
    const tiltDeg = tiltDegFor(pz, T)
    const lim: LandingLimits = { mass, nEng: T.landingEngines, thrustMax: tMax, thrustMin: tMin, tanTilt: Math.tan(tiltDeg * DEG) }
    const mag = magBand(lim)
    const s = Math.max(-st.v.z, 0)
    const pzEff = Math.max(pz, 0.2)
    const aCmd = pz > zProf ? aCruise : aBrake
    const vT = -Math.sqrt(
      T.contactSpeed * T.contactSpeed +
      2 * aBrake * Math.min(pz, zProf) +
      2 * aCruise * Math.max(pz - zProf, 0),
    )
    const uz = clamp(aCmd + G_LAND + 0.9 * (vT - st.v.z), 0.5, mag.max)
    const terminal = pz <= T.terminalSwitchAlt && Math.hypot(st.p.x, st.p.y) < T.terminalSwitchLat && Math.hypot(st.v.x, st.v.y) < T.terminalSwitchV
    let u: readonly [number, number, number]
    if (terminal) {
      const ls: LandingState = { p: [st.p.x, st.p.y, pz], v: [st.v.x, st.v.y, st.v.z], a: [0, 0, uz - G_LAND], d: [st.d.x, st.d.y, st.d.z] }
      u = terminalControl(ls, lim, T.contactSpeed, pz > zProf ? 0 : aBrake)
    } else {
      const tRem = Math.max(1, (2 * pzEff) / Math.max(s + T.contactSpeed, 0.5))
      // linear-approach velocity profile: the pad should be reached as tRem expires, then the
      // velocity nulls with it. A critically damped PD on that profile uses the available authority
      // early (a finite-horizon LQR spends it late and leaves residual speed at the pins).
      // divert aim offset (pre-compensation); the terminal law and the stop still target the pins
      const vCmdX = (-2 * (st.p.x - T.landingAimE)) / tRem
      const vCmdY = (-2 * (st.p.y - T.landingAimN)) / tRem
      const axr = (vCmdX - st.v.x) / 0.5
      const ayr = (vCmdY - st.v.y) / 0.5
      // saturate at the lateral budget before the floor projection: the flight MPC commands a
      // near-minimum-time divert, while a bare proportional law under-commands and then the engine
      // floor's surplus dominates the direction (a pure ranking model would diverge from the flight)
      const aLat = lateralEff * Math.min(lim.tanTilt * Math.max(uz, 1), Math.sqrt(Math.max(0, mag.max * mag.max - uz * uz)))
      const ar = Math.hypot(axr, ayr)
      const k = ar > aLat ? aLat / Math.max(ar, 1e-6) : 1
      u = assemble([axr * k, ayr * k], uz, lim, mag, [st.p.x, st.p.y])
    }
    const uv = v3(u[0], u[1], u[2])
    const um = Math.max(norm(uv), 1e-6)
    const dbg = (globalThis as { FAST_LAND_DEBUG?: (s: string) => void }).FAST_LAND_DEBUG
    if (dbg && i % 50 === 0) dbg(`t ${t.toFixed(1)} p ${st.p.x.toFixed(0)},${st.p.y.toFixed(0)},${pz.toFixed(0)} v ${st.v.x.toFixed(1)},${st.v.y.toFixed(1)},${st.v.z.toFixed(1)} d ${st.d.x.toFixed(2)},${st.d.y.toFixed(2)},${st.d.z.toFixed(2)} u ${u[0].toFixed(1)},${u[1].toFixed(1)},${u[2].toFixed(1)} mag ${mag.min.toFixed(1)}-${mag.max.toFixed(1)} uz ${uz.toFixed(1)} term ${terminal}`)
    const target = scale(uv, 1 / um)
    // first-order direction lag at the attitude bandwidth, rate-capped by the TVC/gimbal limit
    const stepAng = Math.acos(clamp(dot(st.d, target), -1, 1))
    const tau = pz > 120 ? tauHigh : tauLow
    const lagFrac = 1 - Math.exp(-dt / tau)
    const ang = Math.min(stepAng * lagFrac, rateMax * dt)
    let dNew = target
    if (stepAng > 1e-9) {
      const axisDir = sub(target, scale(st.d, dot(target, st.d)))
      const axisLen = norm(axisDir)
      if (axisLen > 1e-9) {
        const rot = scale(axisDir, Math.sin(ang) / axisLen)
        dNew = add(scale(st.d, Math.cos(ang)), rot)
      }
    }
    const m = clamp(um, mag.min, mag.max)
    const a = v3(m * dNew.x, m * dNew.y, m * dNew.z - G_LAND)
    // propellant: ṁ = T/(Isp g0) = m·a/(Isp g0); the guidance's profile is fixed at the flip, but
    // the engine floor and the T/W rise as the header drains (≈16 t on the flown run)
    mass = Math.max(0.5 * flip.mass, mass - (m * mass * dt) / (isp * 9.80665))
    prev = st
    st = { p: add(st.p, add(scale(st.v, dt), scale(a, 0.5 * dt * dt))), v: add(st.v, scale(a, dt)), d: dNew }
    t += dt
  }
  const crossed = st.p.z <= 0 || (prev.p.z > 0 && st.p.z <= 0)
  const f = crossed && prev.p.z > 0 ? clamp(prev.p.z / Math.max(prev.p.z - st.p.z, 1e-9), 0, 1) : 0
  const pinRel = v3(prev.p.x + (st.p.x - prev.p.x) * f, prev.p.y + (st.p.y - prev.p.y) * f, 0)
  const velRel = v3(prev.v.x + (st.v.x - prev.v.x) * f, prev.v.y + (st.v.y - prev.v.y) * f, prev.v.z + (st.v.z - prev.v.z) * f)
  const lat = Math.hypot(pinRel.x, pinRel.y)
  const contactSpeed = norm(velRel)
  const feasible = crossed && !climbed && lat <= 1 && contactSpeed <= 1
  return {
    t, pinRel, velRel, lat, across: Math.abs(pinRel.x), along: Math.abs(pinRel.y),
    vE: velRel.x, vN: velRel.y, vU: velRel.z, contactSpeed, feasible, climbed,
  }
}

/**
 * Lateral-authority efficiency of the integrated plant: the measured 7–12° under-tilt against the
 * command costs 20–40 % of the divert acceleration (T09/L). The ideal 3-DOF closes every candidate to
 * the pins, so the evaluator derates the lateral budget to keep the *ranking* honest; 0.45 is
 * calibrated so the baseline flip state reproduces the integrated 1.4 km residual (≈145 m vs 154 m
 * measured). It is a plant measurement, not a tuning knob.
 */
export const LATERAL_EFFICIENCY = 0.45

/** continuous catch miss from the fast landing state (same shape as `sim/score.ts`) */
export const fastMargin = (contact: FastContact): number =>
  (contact.climbed ? 100 : 0) +
  Math.max(0, contact.lat - 1) +
  2 * Math.max(0, contact.contactSpeed - 1) +
  0.05 * contact.along +
  0.1 * Math.max(0, Math.abs(contact.vE) - 0.5)

/**
 * Entry predictor → belly hand-over. Runs the guidance predictor from the entry interface through
 * the pitch-up, solves the high-α bank for `aimHigh` at the hand-over, and returns the state where
 * the ship reaches Mach 2. This is the honest *downrange* entry model; the real entry's lateral
 * plan (bank reversals) is not modelled here, so the script uses the 6-DOF hand-over for the
 * lateral and this for the aim sensitivity.
 */
export const entryToHandover = (
  pred: PredContext,
  entry: { readonly t: number; readonly r: V3; readonly v: V3; readonly mass: number },
  T: MissionTuning,
  bankEntry = 35 * D,
): FastHandover => {
  const pitch = predict(pred, {
    t: entry.t, r: entry.r, v: entry.v, mass: entry.mass, phase: "entry",
    bankEntry, bankHigh: 45 * D, stopAtPitchUp: true,
  })
  const sol = solveBank(pred, {
    t: pitch.t, r: pitch.r, v: pitch.v, mass: entry.mass, phase: "highAlpha",
    bankEntry, bankHigh: 45 * D, stopAtBelly: true,
  }, "high", 45 * D, T.aimHigh)
  const o = sol.out
  const theta = pred.theta0 + OMEGA_EARTH * o.t
  const padEci = rotZ(pred.padEcef, theta)
  const rel = sub(o.r, padEci)
  // pad ENU basis in ECI at the hand-over
  const padLat = PAD2.lat
  const padLon = PAD2.lon
  const sl = Math.sin(padLat), cl = Math.cos(padLat), so = Math.sin(padLon), co = Math.cos(padLon)
  const east = rotZ(v3(-so, co, 0), theta)
  const north = rotZ(v3(-sl * co, -sl * so, cl), theta)
  const up = rotZ(v3(cl * co, cl * so, sl), theta)
  const hBase = dot(rel, up)
  const pinRel = v3(dot(rel, east), dot(rel, north), hBase)
  const vRelEci = o.vRel
  const velRel = v3(dot(vRelEci, east), dot(vRelEci, north), dot(vRelEci, up))
  const V = norm(sub(velRel, (pred.windForecast?.(Math.max(hBase, 0)) ?? v3(0, 0, 0))))
  return { t: o.t, pinRel, velRel, hBase, mass: entry.mass, V, psi0: Math.atan2(velRel.x, velRel.y) }
}

/** full fast evaluation: entry hand-over → re-planned belly → landing (timings in ms) */
export const evaluateFast = (hand: FastHandover, pred: PredContext, vehicle: VehicleConfig, T: MissionTuning = MISSION_TUNING_DEFAULT, lateralEff: number = LATERAL_EFFICIENCY): FastPrediction => {
  const t0 = performance.now()
  const { plan, flip } = flyBelly(hand, pred, T)
  const id = performance.now()
  const contact = flyLanding(flip, vehicle, T, lateralEff)
  const t1 = performance.now()
  const bellyMiss = plan.miss
  return {
    bellyPlan: plan, bellyMiss, flip, contact, margin: fastMargin(contact),
    timingMs: { belly: id - t0, landing: t1 - id, total: t1 - t0 },
  }
}

/** default hand-over for a unit test: a representative belly-flop start */
export const TEST_HANDOVER: FastHandover = {
  t: 0,
  pinRel: v3(-3200, 5700, 0),
  velRel: v3(0, 0, 0),
  hBase: 30_000,
  mass: 120_000,
  V: 400,
  psi0: 0,
}

