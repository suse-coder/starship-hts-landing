/**
 * Landing burn guidance — the flip-to-pin hoverslam on the sea-level Raptors alone (R11c, Q29/Q33, T09).
 *
 * From flip ignition to pin contact the only actuators are the gimbals and the throttle of at least
 * two sea-level Raptors: no cold gas, no coil (spec/00 R11c). Two Raptors at the 40 % throttle floor
 * give T/W ≈ 1.3–1.8 at 110–140 t, so a hover is physically impossible. The terminal is therefore a
 * *timed deceleration* (hoverslam): the vertical channel follows the exact constant-deceleration
 * profile that arrives at the pin plane (z = 0) with the contact speed vF, and the horizontal channel
 * is an MPC divert with the same free final time (lossless-convexification style thrust-magnitude
 * band + tilt cone, lag-aware, cf. `mpc.ts`).
 *
 * Free final time. The arrival time is the altitude-matched constant-deceleration time
 * `2z/(s+vF)` — the only profile that neither dives (too fast, then a hard brake) nor hovers (too
 * slow, then a climb, which the 40 % floor makes impossible to recover). The lateral MPC uses the
 * same horizon, so the divert is spread over exactly the time the descent gives it. If the divert
 * needs longer, the plan is infeasible and the run reports the miss — the vehicle cannot wait.
 *
 * Everything here is a pure function of the navigation state and the design limits (R6): no plant
 * truth, no hidden state.
 */
import { clamp } from "@ship/math"
import { solveLandingMpc, type MpcPlan, type MpcWeights } from "./mpc"
import { NOSE_FIRST } from "./predictor"

/** surface gravity at the pad (the same value the landing MPC integrates) */
export const G_LAND = 9.79

/** Thrust/limits of the lit engines at the current ambient pressure. */
export interface LandingLimits {
  readonly mass: number // kg
  readonly nEng: number // SL Raptors lit (≥ 2 from the flip to the pins)
  readonly thrustMax: number // N per engine at this ambient pressure
  readonly thrustMin: number // N per engine at the 40 % floor
  readonly tanTilt: number // tan of the thrust-direction cone half-angle from vertical
}

/** net vertical acceleration band (thrust accel − g), m/s², both typically > 0 (no hover) */
export interface NetAccel {
  readonly min: number
  readonly max: number
}

export const netAccel = (l: LandingLimits, g = G_LAND): NetAccel => ({
  min: (l.nEng * l.thrustMin) / l.mass - g,
  max: (l.nEng * l.thrustMax) / l.mass - g,
})

/** thrust acceleration magnitude band (|u|), m/s² */
export const magBand = (l: LandingLimits): { min: number; max: number } => ({
  min: (l.nEng * l.thrustMin) / l.mass,
  max: (l.nEng * l.thrustMax) / l.mass,
})

export interface Hoverslam {
  /** time to reach z = 0 with vz = −vF (s); ∞ when no positive-thrust solution exists */
  readonly tgo: number
  /** true when the state is inside the corridor the engine band can fly */
  readonly feasible: boolean
  /** the state is too high (even the throttle floor over-decelerates) */
  readonly tooHigh: boolean
  /** the state is too low/fast (even full thrust cannot stop at z = 0) */
  readonly tooLow: boolean
  /** duration of the throttle-floor arc before the full-thrust arc (s) */
  readonly tFloor: number
}

/**
 * Minimum-time 1-D descent from (z, down-speed s) to (0, down-speed vF) with a *constant* net
 * acceleration that may be switched once between the engine floor `aMin` and the ceiling `aMax`
 * (both net of gravity and positive → the vehicle always decelerates). The fastest schedule uses the
 * floor first (least deceleration) and the ceiling last, i.e. the classic hoverslam: the switching
 * time is the root of `distance(t) = z`, which is monotone, so a bisection is exact to machine
 * precision (40 halvings).
 */
export const hoverslam = (z: number, s: number, aMin: number, aMax: number, vF: number): Hoverslam => {
  const s0 = Math.max(s, 0)
  if (z <= 0) return { tgo: 0, feasible: true, tooHigh: false, tooLow: false, tFloor: 0 }
  const amin = Math.max(aMin, 1e-3)
  const amax = Math.max(aMax, amin + 1e-6)
  const sf = Math.max(vF, 0)
  const dMax = Math.max(0, (s0 * s0 - sf * sf) / (2 * amax)) // shortest stop (ceiling all the way)
  const dMin = Math.max(0, (s0 * s0 - sf * sf) / (2 * amin)) // longest stop (floor all the way)
  if (dMax >= z) {
    // cannot slow down enough: even the ceiling reaches vF before the pin plane
    return { tgo: Math.max(0, (s0 - sf) / amax), feasible: false, tooHigh: false, tooLow: true, tFloor: 0 }
  }
  if (dMin < z) {
    // cannot descend far enough: the floor stops the vehicle above the pins (no hover to wait)
    return { tgo: Math.max(0, (s0 - sf) / amin), feasible: false, tooHigh: true, tooLow: false, tFloor: 0 }
  }
  const floorDist = (t: number): number => {
    const s1 = s0 - amin * t
    return s0 * t - 0.5 * amin * t * t + Math.max(0, (s1 * s1 - sf * sf) / (2 * amax))
  }
  let lo = 0
  let hi = Math.max(0, (s0 - sf) / amin)
  for (let k = 0; k < 40; k++) {
    const mid = 0.5 * (lo + hi)
    if (floorDist(mid) < z) lo = mid
    else hi = mid
  }
  const tFloor = 0.5 * (lo + hi)
  const s1 = Math.max(sf, s0 - amin * tFloor)
  return { tgo: tFloor + (s1 - sf) / amax, feasible: true, tooHigh: false, tooLow: false, tFloor }
}

/**
 * Required constant net deceleration to arrive at the pin plane at vF, clamped to the engine band.
 * This is the terminal feedback law: it re-evaluates every control step from the measured state, so
 * it is insensitive to an early/late ignition as long as the state stays inside the corridor.
 */
export const hoverAccel = (z: number, vz: number, b: NetAccel, vF: number): number => {
  const s = Math.max(-vz, 0)
  const zz = Math.max(z, 0.3)
  const req = (s * s - vF * vF) / (2 * zz)
  return clamp(req, b.min, b.max)
}

/**
 * Minimum time for the horizontal divert: bring the offset `dist` to zero and the closing speed `v0`
 * (positive toward the target) to zero with |a| ≤ aLat. Bang-bang (accelerate, then decelerate);
 * a coast is not needed because aLit ≥ 0 and the formula reduces to 2√(L/a) for v0 = 0.
 */
export const divertTime = (dist: number, v0: number, aLat: number): number => {
  const L = Math.max(dist, 0)
  const a = Math.max(aLat, 1e-3)
  const u = clamp(v0, -200, 200)
  return Math.max(0, (Math.sqrt(2 * u * u + 4 * a * L) - u) / a)
}

export interface LandingState {
  /** pin station relative to the catch point (pad ENU, m) */
  readonly p: readonly [number, number, number]
  /** ground-relative velocity (pad ENU, m/s) */
  readonly v: readonly [number, number, number]
  /** realised thrust acceleration (pad ENU, m/s²), gravity excluded — the lag model's initial value */
  readonly a: readonly [number, number, number]
  /** realised thrust *direction* (pad ENU unit vector = body +x): the second-order pointing state */
  readonly d?: readonly [number, number, number]
}

export interface LandingOptions {
  /** contact speed at the pins (m/s, positive magnitude) */
  readonly vF: number
  /** closed-loop pointing/throttle time constant handed to the MPC (s) */
  readonly tau: number
  /** soft glide-slope ratio z ≥ glide·|p_h| (keeps the divert above the tower) */
  readonly glide: number
  /** terminal switch: inside this pin height (m) the analytic hoverslam owns the vertical channel */
  readonly terminalAlt: number
  /** MPC horizon steps */
  readonly steps?: number
  /** previous plan (receding-horizon warm start) */
  readonly warm?: Float64Array | null
  /** fuel (thrust-magnitude) cost weight; the burn is minimised subject to the catch */
  readonly wMag?: number
  /** thrust-direction slew limit of the plan (rad/s): the TVC rate the attitude can deliver */
  readonly omegaMax?: number
  /** achievable thrust-axis slew ω (rad/s) for the two-phase lateral plan; 0/undefined = free-vector plan */
  readonly slewRate?: number
  /** previous 6-DOF knot controls (warm start) */
  readonly warm6?: Float64Array | null
  /** use the nonlinear shooting 6-DOF plan (trackable direction) instead of the FISTA plan */
  readonly use6?: boolean
  /** legacy two-phase vertical + lateral MPC (the best integrated configuration) */
  readonly mode?: "twoPhase" | "pdg"
  /** vertical-thrust trim (1 = nominal): > 1 commands more vertical thrust, < 1 less. Used to close
   *  the loop on the *achieved* vertical acceleration when the attitude lags the commanded tilt. */
  readonly uzTrim?: number
  /** cruise (divert) vertical deceleration fixed at the hand-over (m/s²) */
  readonly aCruise?: number
  /** altitude where the tilted divert cruise ends and the near-vertical final descent begins (m) */
  readonly zFinal?: number
  /** override the MPC stage/terminal weights (mission tuning); the defaults are the flown set */
  readonly weights?: Partial<MpcWeights>
  /** direct lateral velocity-field guidance (the two-phase branch). Off/undefined = the lateral MPC. */
  readonly field?: LateralField | false
  /** true during the flip rotation: the body cannot follow a lateral law, so keep the MPC delivery */
  readonly flipping?: boolean
  /** engine count the vertical profile is designed for (the staged burn may fly fewer now) */
  readonly profileEngines?: number
}

/**
 * Direct lateral guidance for the two-phase landing burn: a per-axis velocity field
 * `v_ref = −(p − p_gate)/τ` tracked by a first-order acceleration loop `a = (v_ref − v)/τ_a`, with a
 * speed cap that falls with pin height. Every term is the measured navigation state, so the law is
 * continuous and the attitude loop can follow it; the MPC's free-vector plan could jump a 40–50° tilt
 * command in one replan (review/02 L1/L2). x (across the arms, ±1 m) gets the lateral budget first;
 * y (along the 38 m rails) takes the remainder. The gate is south of the catch (`gateY`), so the
 * terminal law flies the last stretch northward down the arm centreline.
 */
export interface LateralField {
  /** position-to-velocity time constant, x (s) — the tight catch axis */
  readonly tauX: number
  /** position-to-velocity time constant, y (s) */
  readonly tauY: number
  /** velocity-loop time constant (s) */
  readonly tauA: number
  /** phase-lead time on the position error (s): v_ref = −(p + v·tauLead)/τ */
  readonly tauLeadX: number
  /** phase-lead time on the along-rail error (s) */
  readonly tauLeadY: number
  /** pin height where the terminal law takes over (m): the field tightens as the time to it shrinks */
  readonly zHand: number
  /** lower bound of the time-to-go-scheduled position time constant (s) */
  readonly tauMin: number
  /** lateral speed cap, x (m/s) — the across-arm divert can be long */
  readonly vMaxX: number
  /** lateral speed cap, y (m/s) — the along-arm approach is short, slow and northward */
  readonly vMaxY: number
  /** x speed-cap slope with pin height (m/s per m); the cap is min(vMaxX, vMaxAltX·z) */
  readonly vMaxAltX: number
  /** y speed-cap slope with pin height (m/s per m) */
  readonly vMaxAltY: number
  /** deceleration used for the stopping-distance cap, x (m/s²) — v ≤ √(2·aStopX·|x|) */
  readonly aStopX: number
  /** deceleration used for the stopping-distance cap, y (m/s²) */
  readonly aStopY: number
  /** minimum share of the lateral budget reserved for y when y needs it (0–1) */
  readonly yShare: number
  /** along-rail gate south of the catch point (m) */
  readonly gateY: number
}

export const LATERAL_FIELD_DEFAULT: LateralField = {
  tauX: 5,
  tauY: 10,
  tauA: 1.2,
  tauLeadX: 1.2,
  tauLeadY: 1.5,
  zHand: 45,
  tauMin: 1.2,
  vMaxX: 42,
  vMaxY: 4,
  vMaxAltX: 0.45,
  vMaxAltY: 0.08,
  aStopX: 2.5,
  aStopY: 1.5,
  yShare: 0.4,
  gateY: -12,
}

export interface LandingSolution {
  /** commanded thrust acceleration (pad ENU, m/s²), gravity excluded */
  readonly u: readonly [number, number, number]
  /** free final time used by the MPC (s) */
  readonly tgo: number
  /** MPC horizon step (s) — the plan is re-solved every control step, not every h */
  readonly h: number
  readonly feasible: boolean
  readonly vertical: NetAccel
  /** true inside the terminal altitude (the analytic vertical law owns u_z; no MPC solve) */
  readonly terminal: boolean
  /** solved plan (for the warm start), null in the terminal phase */
  readonly plan: MpcPlan | null
  /** the 6-DOF solver's knot controls (warm start); undefined when the FISTA plan is used */
  readonly u6?: Float64Array
  /** predicted catch path for the display: positions relative to the *catch point* (pad ENU), the
   *  MPC horizon sampled at its steps (n+1 points, 16–24 typical) */
  readonly path?: ReadonlyArray<readonly [number, number, number]>
}

/** clamp a thrust-acceleration vector into the magnitude band and the tilt cone (MPC-independent) */
export const shapeAccel = (u: readonly [number, number, number], l: LandingLimits): [number, number, number] => {
  const b = magBand(l)
  let x = u[0]
  let y = u[1]
  let z = Math.max(u[2], 1)
  const hMax = l.tanTilt * z
  const hn = Math.hypot(x, y)
  if (hn > hMax) {
    x *= hMax / hn
    y *= hMax / hn
  }
  const m = Math.hypot(x, y, z)
  const sc = m < b.min ? b.min / Math.max(m, 1e-6) : m > b.max ? b.max / m : 1
  return [x * sc, y * sc, z * sc]
}

/**
 * Terminal control inside the last ≈ 25 m of pin height. The vertical channel tracks the
 * constant-deceleration profile (`v_target(z) = −√(vF² + 2a·z)`) with a proportional velocity term,
 * which absorbs the throttle/pointing lag; the lateral channel is a PD brake inside the shrinking
 * cone. Called every control step (no solver), so the contact speed does not depend on a plan's last
 * iteration.
 */
export const terminalControl = (st: LandingState, l: LandingLimits, vF: number, aProf = 0): [number, number, number] => {
  const g = G_LAND
  const mag = magBand(l)
  const [px, py, pz] = st.p
  const vz = st.v[2]
  const s = Math.max(-vz, 0)
  const aNom = aProf > 0.2 ? aProf : Math.max(0.4, (s * s - vF * vF) / (2 * Math.max(pz, 0.2)))
  const vTarget = -Math.sqrt(vF * vF + 2 * aNom * Math.max(pz, 0))
  const uz = clamp(aNom + g + 0.9 * (vTarget - vz), 0.6, mag.max)
  // lateral: kill the residual velocity and offset before contact
  const tRem = Math.max(0.55, Math.min(3.0, pz / Math.max(s, 0.6)))
  const ax = clamp(-(2 * st.v[0] / tRem + px / (tRem * tRem)), -8, 8)
  const ay = clamp(-(2 * st.v[1] / tRem + py / (tRem * tRem)), -8, 8)
  return assemble([ax, ay], uz, l, mag, [px, py])
}

/** lateral budget at a given vertical thrust draw (tilt cone and thrust-magnitude ceiling) */
export const lateralBudget = (uz: number, l: LandingLimits, mag: { min: number; max: number }): number =>
  Math.min(l.tanTilt * Math.max(uz, 1), Math.sqrt(Math.max(0, mag.max * mag.max - uz * uz)))

/**
 * Assemble the thrust-acceleration vector: vertical as commanded (it is the timed profile), lateral
 * clamped to the budget, and the engine magnitude floor absorbed *laterally* (the divert law
 * re-plans every step, so it corrects the extra push) — never by scaling the vertical, which would
 * turn the timed deceleration into a hover or a climb.
 */
export const assemble = (aH: readonly [number, number], uz: number, l: LandingLimits, mag: { min: number; max: number }, pxRef: readonly [number, number]): [number, number, number] => {
  const aL = lateralBudget(uz, l, mag)
  let ax = aH[0]
  let ay = aH[1]
  const hn = Math.hypot(ax, ay)
  if (hn > aL) { ax *= aL / hn; ay *= aL / hn }
  const aLmin = Math.min(Math.sqrt(Math.max(0, mag.min * mag.min - uz * uz)), l.tanTilt * Math.max(uz, 1))
  if (Math.hypot(ax, ay) < aLmin) {
    // the floor sets a *minimum* lateral component: keeping the vertical exact at the engine floor
    // means the thrust axis must be tilted at least acos(u_z/|u|min). The surplus leaves laterally —
    // in the divert direction (the law re-plans every step) — never vertically (that would stop the
    // descent and climb away, the 47 m stall).
    const need = aLmin
    const dir = Math.hypot(ax, ay)
    if (dir > 1e-6) { ax = (ax / dir) * need; ay = (ay / dir) * need }
    else {
      const ph = Math.hypot(pxRef[0], pxRef[1])
      if (ph > 0.2) { ax = (pxRef[0] / ph) * need; ay = (pxRef[1] / ph) * need }
      else if (need > 1e-6) { ax = -need; ay = 0 }
    }
  }
  const uzz = Math.max(uz, 0.5)
  const m = Math.hypot(ax, ay, uzz)
  // last resort: the throttle cannot go below 40 %, so scale the whole vector up (the vertical
  // over-decelerates slightly; that is the physical floor, not a guidance choice)
  if (m > 1e-6 && m < mag.min) { const k = mag.min / m; ax *= k; ay *= k }
  return [ax, ay, m < mag.min ? Math.min(mag.max, uzz * (mag.min / Math.max(m, 1e-6))) : uzz]
}

/**
 * The landing law — re-planned every 250 ms in closed loop (the vertical channel is re-evaluated at
 * the same cadence; the terminal law runs every control step):
 *  - vertical: the hoverslam profile. The arrival time is the altitude-matched constant-deceleration
 *    time `2z/(s+vF)` (the only profile that neither dives nor hovers, and the minimum impulse);
 *    inside `terminalAlt` a profile-tracking controller absorbs the lag.
 *  - lateral: an MPC over the same free final time, with the lateral acceleration budget that the
 *    vertical draw leaves (tilt cone + thrust magnitude), the realised acceleration as its lag state.
 *    When the divert cannot be closed in the available time the plan is infeasible and reported.
 */
export const planLanding = (st: LandingState, l: LandingLimits, opt: LandingOptions): LandingSolution => {
  const g = G_LAND
  const mag = magBand(l)
  const b = netAccel(l, g)
  const [px, py, pz] = st.p
  const [vx, vy, vz] = st.v
  const s = Math.max(-vz, 0)
  // Free final time: the altitude-matched constant-deceleration time 2z/(s+vF) — the slowest sensible
  // descent (a longer one needs a dive/hang). The whole 3-D trajectory is planned over it.
  const tgo = clamp((2 * pz) / Math.max(s + opt.vF, 0.5), 2.5, 60)
  // terminal safety net for the last metres (the MPC runs down to `terminalAlt`)
  if (pz <= opt.terminalAlt) {
    return { u: terminalControl(st, l, opt.vF, 0), tgo, h: Math.max(tgo / 20, 1e-3), feasible: true, vertical: b, terminal: true, plan: null }
  }
  const n = opt.steps ?? 30
  const h = Math.max(tgo / n, 1e-3)
  if (opt.mode === "twoPhase") {
    // Two-phase vertical (fixed cruise decel then upright brake) + lateral MPC over the divert
    // window — the configuration that integrated best (7 m at the plane). Kept as the flight path
    // while the nonlinear solvers above are still being converged.
    const aBrake = Math.max((opt.profileEngines && opt.profileEngines !== l.nEng ? (opt.profileEngines * l.thrustMin) / l.mass : mag.min) - g, 0.6)
    const zF = opt.zFinal && opt.zFinal > 2 ? opt.zFinal : Math.max(25, Math.min(400, pz * 0.2))
    const aCruise = opt.aCruise && opt.aCruise > 0.1 ? opt.aCruise : Math.max(0.6, (s * s - opt.vF * opt.vF) / (2 * Math.max(pz, 1)))
    const cruise = pz > zF
    const aCmd = cruise ? aCruise : aBrake
    const vT = -Math.sqrt(opt.vF * opt.vF + 2 * aBrake * Math.min(pz, zF) + 2 * aCruise * Math.max(pz - zF, 0))
    // Vertical lag trim from guidance (it measures the body-vs-command tilt lag): without it the
    // body stays more vertical than commanded, the vertical thrust is too large, the descent
    // over-decelerates and the ship stalls/climbs above the pins. `uzTrim` was passed in but never
    // applied in this branch (review/02 L2).
    const uz = clamp((aCmd + g + 0.9 * (vT - vz)) * (opt.uzTrim && opt.uzTrim > 0 ? opt.uzTrim : 1), 0.5, mag.max)
    // One-engine stage: a single SL at 100 % gives 18 m/s² (net +8.1), but the gimbal authority
    // scales with thrust. While the sink is high, fly the engine hard (≈95 % of the magnitude cap)
    // instead of the two-engine profile's ~60 %; once the sink is low, the normal feedback throttles
    // down and tracks the profile for the stage-2 hand-over.
    const uzCmd = l.nEng === 1 && !opt.flipping ? (NOSE_FIRST ? mag.min : Math.max(uz, 0.95 * mag.max)) : uz
    if (!cruise) return { u: terminalControl(st, l, opt.vF, aBrake), tgo, h: Math.max(tgo / 20, 1e-3), feasible: true, vertical: b, terminal: true, plan: null }
    const sF = Math.sqrt(opt.vF * opt.vF + 2 * aBrake * zF)
    const tDiv = Math.max(0.6, (Math.max(s - sF, 0)) / Math.max(aCruise, 0.2))
    const nn = opt.steps ?? 25
    const hh = Math.max(tDiv / nn, 1e-3)
    const plan2 = solveLandingMpc({
      p0: [px, py, 0], v0: [vx, vy, 0], a0: [st.a[0], st.a[1], 0], tau: opt.tau,
      n: nn, h: hh, vRef: [0, 0, 0], vF: [0, 0, 0], gravity: 0,
      // Terminal gate SOUTH of the catch point: the divert ends at y = −GATE_Y, x = 0, and the
      // terminal law then flies north (+y) down the arm centreline. This enforces the *approach
      // direction*, not just the point: the ship cannot arrive from the tower side.
      pRefH: [0, -12], vRefH: [0, 0.4],
      aZMin: 0, aZMax: 0, aMagMin: 0, aMagMax: 0.98 * lateralBudget(uzCmd, l, mag), tanTilt: l.tanTilt,
      weak: null, aWeak: Infinity, uPrev: [st.a[0], st.a[1], 0], glide: 0,
      // across-arm x is the tight catch axis (±1 m); along-arm y has the whole 38 m rail span. Weight
      // x ≫ y so the terminal path runs down the arm centreline instead of trading x for y.
      weights: { qp: 0.0002, qv: 0.4, wvXY: 0, ru: 0.01, rd: 0.1, Qp: 60, Qv: 120, wX: 4, wY: 0.35, ...opt.weights },
      // trackable plan: the lateral thrust acceleration can only turn at |u|·ω (the body rotates the
      // thrust axis); starting from the realised lateral acceleration
      slew: opt.slewRate && opt.slewRate > 0 ? { u0: [st.a[0], st.a[1], 0], rate: Math.max(Math.hypot(uzCmd, lateralBudget(uzCmd, l, mag)), mag.min) * opt.slewRate } : undefined,
    }, opt.warm ?? null, opt.slewRate && opt.slewRate > 0 ? 160 : 60)
    const field = opt.flipping ? false : opt.field
    const u2 = field
      ? (() => {
        // direct lateral velocity field (closed loop on the measured state, no free-vector plan)
        const f = field
        const stopX = Math.sqrt(2 * f.aStopX * Math.max(Math.abs(px), 0.5))
        const stopY = Math.sqrt(2 * f.aStopY * Math.max(Math.abs(py - f.gateY), 0.5))
        const vCapX = Math.max(2, Math.min(f.vMaxX, f.vMaxAltX * Math.max(pz, 0.5), stopX))
        const vCapY = Math.max(1, Math.min(f.vMaxY, f.vMaxAltY * Math.max(pz, 0.5), stopY))
        const tgo = Math.max(0, pz - f.zHand) / Math.max(s, 0.5)
        const tauXEff = clamp(tgo, f.tauMin, f.tauX)
        const tauYEff = clamp(tgo, f.tauMin, f.tauY)
        const vrx = clamp(-(px + vx * f.tauLeadX) / tauXEff, -vCapX, vCapX)
        const vry = clamp(-(py - f.gateY + vy * f.tauLeadY) / tauYEff, -vCapY, vCapY)
        const budget = 0.95 * lateralBudget(uzCmd, l, mag)
        const axNeed = (vrx - vx) / f.tauA
        const ayNeed = (vry - vy) / f.tauA
        // x (the ±1 m across-arm axis) has priority, but y may not be starved: reserve up to
        // `yShare` of the budget for the along-rail correction when it needs it.
        const yWant = clamp(ayNeed, -f.yShare * budget, f.yShare * budget)
        const axRoom = Math.sqrt(Math.max(0, budget * budget - yWant * yWant))
        let ax = clamp(axNeed, -axRoom, axRoom)
        let ay = clamp(ayNeed, -Math.sqrt(Math.max(0, budget * budget - ax * ax)), Math.sqrt(Math.max(0, budget * budget - ax * ax)))
        // the engine floor sets a *minimum* lateral component at this vertical draw. Keep the surplus
        // in the field's own direction (toward the gate); `assemble`'s radial fallback would push away.
        const aLmin = Math.min(Math.sqrt(Math.max(0, mag.min * mag.min - uz * uz)), l.tanTilt * Math.max(uz, 1))
        const am = Math.hypot(ax, ay)
        if (am < aLmin) {
          const gx = -px, gy = -(py - f.gateY)
          const gn = Math.hypot(gx, gy)
          const dx = am > 1e-6 ? ax / am : gn > 0.2 ? gx / gn : 0
          const dy = am > 1e-6 ? ay / am : gn > 0.2 ? gy / gn : 0
          if (Math.hypot(dx, dy) > 1e-6) {
            const need = Math.min(aLmin, budget)
            ax = dx * need
            ay = dy * need
          }
        }
        return assemble([ax, ay], uz, l, mag, [px, py])
      })()
      : assemble([plan2.u[0], plan2.u[1]], uz, l, mag, [px, py])
    // visual path: 22 points over the *whole* descent — the divert (lateral plan + cruise vertical
    // profile) and then the upright brake from zF to the catch point (below zF the ship is over the pad)
    const nPts = 22
    const tBrake = Math.max(0.1, (sF - opt.vF) / Math.max(aBrake, 0.3))
    const path = Array.from({ length: nPts }, (_, k) => {
      const t = (k / (nPts - 1)) * (Math.max(tDiv, 0.1) + tBrake)
      if (t <= tDiv) {
        const nL = plan2.path.length / 3 - 1
        const fq = nL > 0 ? (t / Math.max(tDiv, 1e-6)) * nL : 0
        const k0 = Math.min(nL - 1, Math.max(0, Math.floor(fq)))
        const fr = Math.min(1, Math.max(0, fq - k0))
        const x = plan2.path[3 * k0] * (1 - fr) + plan2.path[3 * (k0 + 1)] * fr
        const y = plan2.path[3 * k0 + 1] * (1 - fr) + plan2.path[3 * (k0 + 1) + 1] * fr
        const z = Math.max(0, pz - (s * t - 0.5 * aCmd * t * t))
        return [x, y, z] as const
      }
      const tb = t - tDiv
      const zb = Math.max(0, zF - (sF * tb - 0.5 * aBrake * tb * tb))
      return [0, 0, zb] as const
    })
    return { u: u2, tgo: tDiv, h: hh, feasible: true, vertical: b, terminal: false, plan: plan2, path }
  }
  if (opt.use6) {
    const p6 = solveLanding6D(st, l, { ...opt, steps: 24 })
    const pN = p6.pN
    const vN = p6.vN
    const feasible6 = Math.hypot(pN[0], pN[1]) <= 1.2 && Math.abs(pN[2]) <= 1.5 && Math.abs(vN[2] + opt.vF) <= 1.0 && Math.hypot(vN[0], vN[1]) <= 0.8
    return { u: [p6.u[0], p6.u[1], p6.u[2]], tgo, h: Math.max(tgo / 24, 1e-3), feasible: feasible6, vertical: b, terminal: false, plan: null, u6: p6.u }
  }
  const plan = solveLandingMpc({
    p0: [px, py, pz],
    v0: [vx, vy, vz],
    a0: st.a,
    tau: opt.tau,
    n,
    h,
    // stage velocity reference is weak (the path is free); the terminal velocity is the catch speed
    vRef: [0, 0, -opt.vF],
    vF: [0, 0, -opt.vF],
    // terminal catch geometry: x across the arms → 0 (±1 m), y along the arms, entering from the open
    // south side at a slow along-arm speed (|vx| ≈ 0: entering across the arm sides would hit them)
    pRefH: [0, 0],
    vRefH: [0, 0.4],
    // the *achievable* vertical floor at the max tilt (the engine floor forces |u| ≥ Γ, so u_z can
    // only go this low): a convex box bound the plan must respect
    aZMin: 0.95 * mag.min * Math.cos(Math.atan(l.tanTilt)),
    aZMax: 0.98 * (b.max + g),
    aMagMin: 1.0 * mag.min,
    aMagMax: 0.98 * mag.max,
    tanTilt: l.tanTilt,
    // lossless-convex PDG: |u| ≤ Γ, Γ ∈ [40 % floor, 100 %] (2 engines), u_z ≥ cos(θ)|u|
    cone: { cosTilt: Math.cos(Math.atan(l.tanTilt)), aMin: mag.min, aMax: 0.98 * mag.max },
    // trackability: the plan may only slew the thrust direction at the TVC rate from the realised one,
    // and the engine floor raises the magnitude along that direction
    dyn: st.d ? { d0: st.d, omegaMax: opt.omegaMax ?? 0.35, aMin: mag.min } : undefined,
    weak: null,
    aWeak: Infinity,
    uPrev: st.a,
    glide: opt.glide,
    // terminal catch constraint dominates; the stage costs + fuel weight shape a minimum-burn path
    weights: { qp: 0.0005, qv: 0.01, wvXY: 1, ru: 0.02, rd: 0.3, wMag: opt.wMag ?? 0.05, wDir: 5, Qp: 200, Qv: 400, wGlide: 0.8, ...opt.weights },
  }, opt.warm ?? null, 90)
  // command: the plan's first control. The plan already models the engine floor along the thrust
  // direction and the rate-limited direction slew, so scaling to the floor here (shapeAccel) is the
  // same model — not the old lateral-surplus trick (which fought the plan).
  const u = shapeAccel([plan.u[0], plan.u[1], plan.u[2]], l)
  // catch feasibility of the *plan* (not the current state)
  const pN = plan.pN
  const vN = plan.vN
  const feasible = Math.hypot(pN[0], pN[1]) <= 1.2 && pN[2] >= -0.8 && pN[2] <= 1.5 && Math.abs(vN[2] + opt.vF) <= 1.0 && Math.hypot(vN[0], vN[1]) <= 0.8
  // visual export: the rollout's predicted positions (already computed, no extra rollout), subsampled
  // to at most 24 points including both ends
  const nAll = plan.path.length / 3
  const stride = Math.max(1, Math.ceil(nAll / 24))
  const path: Array<readonly [number, number, number]> = []
  for (let k = 0; k < nAll; k += stride) path.push([plan.path[3 * k], plan.path[3 * k + 1], plan.path[3 * k + 2]])
  const lastK = nAll - 1
  path.push([plan.path[3 * lastK], plan.path[3 * lastK + 1], plan.path[3 * lastK + 2]])
  return { u, tgo, h, feasible, vertical: b, terminal: false, plan, path }
}

/**
 * Nonlinear (shooting) 6-DOF-ish landing plan — the trackable alternative to the projected-gradient
 * MPC, whose adjoint cannot see the direction-rate limiter.
 *
 * State p, v, d (thrust direction) with the *exact* rate-limited direction dynamics and the engine
 * floor along the direction. The control is the desired thrust acceleration at N knots (piecewise
 * linear). The terminal catch (p → 0, v → v_F, d → vertical) plus a fuel term is minimised by a
 * finite-difference SQP-style descent, warm-started from the previous plan. The plan therefore cannot
 * ask for a direction slew the TVC cannot deliver.
 */
export interface Plan6 {
  readonly u: Float64Array // the applied (rate-limited, floored) acceleration at every step (3n)
  readonly pN: readonly [number, number, number]
  readonly vN: readonly [number, number, number]
  readonly dN: readonly [number, number, number]
  readonly cost: number
  readonly iters: number
}

/** clamp the knot controls into the magnitude band and the thrust cone (projection helper) */
const clampKnotsInit = (P: Float64Array, aMin: number, aMax: number, cosTilt: number): void => {
  for (let k = 0; k < P.length / 3; k++) {
    let x = P[3 * k], y = P[3 * k + 1], z = P[3 * k + 2]
    const m = Math.hypot(x, y, z) || 1e-6
    if (m < aMin) { const sc = aMin / m; x *= sc; y *= sc; z *= sc }
    const m2 = Math.hypot(x, y, z)
    if (m2 > aMax) { const sc = aMax / m2; x *= sc; y *= sc; z *= sc }
    const r = Math.hypot(x, y) || 1e-6
    if (z < cosTilt * r) {
      const f = z - cosTilt * r
      const rp = r + (cosTilt * f) / (1 + cosTilt * cosTilt)
      const zp = z - f / (1 + cosTilt * cosTilt)
      const sc = rp / r
      x *= sc; y *= sc; z = zp
    }
    P[3 * k] = x; P[3 * k + 1] = y; P[3 * k + 2] = z
  }
}

export const solveLanding6D = (st: LandingState, l: LandingLimits, opt: LandingOptions): Plan6 => {
  const g = G_LAND
  const mag = magBand(l)
  const aMin = mag.min
  const aMax = 0.98 * mag.max
  const n = opt.steps ?? 24
  const s = Math.max(-st.v[2], 0)
  const tgo = clamp((2 * Math.max(st.p[2], 0.5)) / Math.max(s + opt.vF, 0.5), 2.5, 60)
  const h = tgo / n
  const cap = (opt.omegaMax ?? 0.35) * h
  const cosTilt = Math.cos(Math.atan(l.tanTilt))
  const vF: [number, number, number] = [0, 0, -opt.vF]
  const d0: [number, number, number] = st.d ? [st.d[0], st.d[1], st.d[2]] : [0, 0, 1]
  const knots = 6
  const dtk = tgo / (knots - 1)
  const P = new Float64Array(3 * knots)
  const warm6 = opt.warm6
  if (warm6 && warm6.length === 3 * knots) P.set(warm6)
  else {
    // Initial guess: the closed-form linear-acceleration divert (p → 0, v → v_F over tgo) sampled at
    // the knots. (A tilted-glide guess was tried: it diverges laterally; a hover guess sits in the
    // floor-climb minimum. The proper fix is the SCvx inner convex solve, not another guess.)
    const a0x = (6 * (0 - st.p[0] - st.v[0] * tgo)) / (tgo * tgo) - (2 * (0 - st.v[0])) / tgo
    const a0y = (6 * (0 - st.p[1] - st.v[1] * tgo)) / (tgo * tgo) - (2 * (0 - st.v[1])) / tgo
    const a1x = (2 * (0 - st.v[0] - a0x * tgo)) / (tgo * tgo)
    const a1y = (2 * (0 - st.v[1] - a0y * tgo)) / (tgo * tgo)
    for (let k = 0; k < knots; k++) {
      const t = k * dtk
      P[3 * k] = a0x + a1x * t
      P[3 * k + 1] = a0y + a1y * t
      P[3 * k + 2] = Math.max(g, aMin)
    }
    clampKnotsInit(P, aMin, aMax, cosTilt)
  }

  // exact simulation of the rate-limited direction + floor; returns the terminal state, the applied
  // accelerations and the cost
  const run = (Q: Float64Array, keep: boolean): { p: [number, number, number]; v: [number, number, number]; d: [number, number, number]; J: number; u: Float64Array | null; minZ: number } => {
    let px = st.p[0], py = st.p[1], pz = st.p[2]
    let vx = st.v[0], vy = st.v[1], vz = st.v[2]
    let dx = d0[0], dy = d0[1], dz = d0[2]
    const uOut = keep ? new Float64Array(3 * n) : null
    let fuel = 0
    let minZ = pz
    for (let k = 0; k < n; k++) {
      const t = k * h
      const kk = Math.min(knots - 2, Math.max(0, Math.floor(t / dtk)))
      const f = clamp((t - kk * dtk) / dtk, 0, 1)
      let ux = (Q[3 * kk] * (1 - f) + Q[3 * (kk + 1)] * f)
      let uy = (Q[3 * kk + 1] * (1 - f) + Q[3 * (kk + 1) + 1] * f)
      let uz = (Q[3 * kk + 2] * (1 - f) + Q[3 * (kk + 1) + 2] * f)
      const um = Math.hypot(ux, uy, uz) || 1e-6
      // rate-limit the direction toward the desired one
      const tx = ux / um, ty = uy / um, tz = uz / um
      const dtp = Math.max(-1, Math.min(1, dx * tx + dy * ty + dz * tz))
      const ang = Math.acos(dtp)
      if (ang > cap && ang > 1e-9) {
        const nx = tx - dtp * dx, ny = ty - dtp * dy, nz = tz - dtp * dz
        const nn = Math.hypot(nx, ny, nz)
        if (nn > 1e-9) {
          const ca = Math.cos(cap), sa = Math.sin(cap)
          dx = dx * ca + (nx / nn) * sa
          dy = dy * ca + (ny / nn) * sa
          dz = dz * ca + (nz / nn) * sa
          const dn = Math.hypot(dx, dy, dz) || 1
          dx /= dn; dy /= dn; dz /= dn
        }
      } else { dx = tx; dy = ty; dz = tz }
      const m = Math.max(um, aMin) // engine floor along the realised direction
      const ax = m * dx, ay = m * dy, az = m * dz
      fuel += m * h
      if (uOut) { uOut[3 * k] = ax; uOut[3 * k + 1] = ay; uOut[3 * k + 2] = az }
      px += vx * h + 0.5 * ax * h * h
      py += vy * h + 0.5 * ay * h * h
      pz += vz * h + 0.5 * (az - g) * h * h
      vx += ax * h; vy += ay * h; vz += (az - g) * h
      if (pz < minZ) minZ = pz
    }
    const ep = Math.hypot(px, py) * Math.hypot(px, py) + pz * pz
    const ev = (vx - vF[0]) ** 2 + (vy - vF[1]) ** 2 + (vz - vF[2]) ** 2
    const ed = (dx * dx + dy * dy) + (dz - 1) * (dz - 1) // direction → vertical
    const J = 200 * ep + 400 * ev + 60 * ed + (opt.wMag ?? 0.05) * fuel * 10 + (minZ < -0.5 ? 500 * (minZ + 0.5) ** 2 : 0)
    return { p: [px, py, pz], v: [vx, vy, vz], d: [dx, dy, dz], J, u: uOut, minZ }
  }

  // finite-difference projected descent on the knot controls (clamped to the magnitude band)
  const clampKnots = (): void => {
    for (let k = 0; k < knots; k++) {
      let x = P[3 * k], y = P[3 * k + 1], z = P[3 * k + 2]
      const m = Math.hypot(x, y, z) || 1e-6
      if (m < aMin) { const sc = aMin / m; x *= sc; y *= sc; z *= sc }
      const m2 = Math.hypot(x, y, z)
      if (m2 > aMax) { const sc = aMax / m2; x *= sc; y *= sc; z *= sc }
      if (z / Math.max(Math.hypot(x, y, z), 1e-6) < cosTilt) {
        // radial project onto the cone (same as the SOC projection)
        const r = Math.hypot(x, y) || 1e-6
        const f = z - cosTilt * r
        const rp = r + (cosTilt * f) / (1 + cosTilt * cosTilt)
        const zp = z - f / (1 + cosTilt * cosTilt)
        const sc = rp / r
        x *= sc; y *= sc; z = zp
      }
      P[3 * k] = x; P[3 * k + 1] = y; P[3 * k + 2] = z
    }
  }
  let J = run(P, false).J
  const dbg6 = (globalThis as { DBG6?: (s: string) => void }).DBG6
  if (dbg6) dbg6(`init J=${J.toFixed(0)}`)
  const eps = 0.4
  const gq = new Float64Array(3 * knots)
  for (let it = 0; it < 14; it++) {
    for (let i = 0; i < 3 * knots; i++) {
      P[i] += eps
      clampKnots()
      const Jp = run(P, false).J
      P[i] -= eps
      clampKnots()
      gq[i] = (Jp - J) / eps
    }
    let gn = 0
    for (let i = 0; i < 3 * knots; i++) gn += gq[i] * gq[i]
    gn = Math.sqrt(gn) || 1
    let step = 2.0
    const P0 = Float64Array.from(P)
    for (let ls = 0; ls < 6; ls++) {
      for (let i = 0; i < 3 * knots; i++) P[i] = P0[i] - (step * gq[i]) / gn
      clampKnots()
      const Jn = run(P, false).J
      if (dbg6) dbg6(`it${it} ls${ls} step=${step.toFixed(2)} J=${Jn.toFixed(0)}`)
      if (Jn < J) { J = Jn; break }
      step *= 0.45
      if (ls === 5) { P.set(P0); }
    }
  }
  const out = run(P, true)
  return { u: out.u ?? new Float64Array(3 * n), pN: out.p, vN: out.v, dN: out.d, cost: out.J, iters: 14 }
}

/**
 * Flip ignition altitude above the catch plane (m) for a sink rate vz — the gate the landing law
 * needs, and the `hFlipPred` end altitude the belly-flop plan reads. Kept a pure function of vz
 * (signature fixed) because the gate is evaluated on the current sink only.
 *
 * The gate is set by the *divert*, because the vertical corridor alone would allow a much lower
 * ignition: the belly flop delivers the flip ≈ 1.3 km from the pad with ≈ 35 m/s closing, and the
 * lateral authority that closes it is the diverted thrust (design ≈ 11.8 m/s², a ≈ 45° tilt of the
 * two-engine thrust). The altitude that buys the divert time is
 *
 *   z = ½·(v_post + vF)·t_divert + FLIP_FALL·v,   t_divert = (√(2·v_c² + 4·a_lat·L) − v_c)/a_lat
 *
 * with v_post = v − FLIP_DV (the flip's rotation sweep costs ≈ 35 m/s of vertical speed and
 * FLIP_FALL·v ≈ 650 m of altitude). At vz = −107 m/s this gives ≈ 1.29 km above the catch point, i.e.
 * a 1.41 km gate on hBase; the old ad-hoc gate (0.70 km) leaves ≈ 8 s of powered flight for the
 * divert, which cannot close 1.3 km (it needs ≈ 80 m/s² of lateral acceleration). A *higher* gate
 * buys divert time but makes the required vertical deceleration fall below the 40 % floor, so the
 * ship over-decelerates and cannot descend (no hover) — 1.3–1.4 km is the latest gate that is
 * vertically flyable. `hFlipPred` reads this function, so its end altitude moves with it.
 */
export const LANDING_MISS_DESIGN = 630 // the 1.4 km gate (G's entry delivers ≈440–530 m there)
export const LANDING_CLOSING_DESIGN = 35
export const LANDING_LAT_DESIGN = 4.5
/** vertical Δv the ≈ 5 s flip burn costs while the thrust sweeps from horizontal to vertical (m/s) */
export const FLIP_DV = 35
/** seconds of ballistic fall during the flip (m per m/s of sink) */
export const FLIP_FALL = 8

/** The parameters of the flip gate. Mission tuning supplies its own instance; the defaults are the
 *  flown values. The field names match `MissionTuning`, so a tuning value is a valid gate. */
export interface FlipGate {
  readonly gateMiss: number
  readonly gateClosing: number
  readonly gateLat: number
  readonly flipDv: number
  readonly flipFall: number
  /** pin height above the rails at which the flip must be COMPLETE (m); 0 = the legacy lateral-stop gate */
  readonly flipComplete: number
}

export const DEFAULT_FLIP_GATE: FlipGate = {
  gateMiss: LANDING_MISS_DESIGN,
  gateClosing: LANDING_CLOSING_DESIGN,
  gateLat: LANDING_LAT_DESIGN,
  flipDv: FLIP_DV,
  flipFall: FLIP_FALL,
  flipComplete: 1550,
}

export const flipAltitude = (vz: number, gate: FlipGate = DEFAULT_FLIP_GATE): number => {
  const s = Math.abs(vz)
  // completion-height gate: the flip (a fixed manoeuvre, `flipFall` seconds of fall per m/s of
  // sink — measured 7.1 at 108 m/s on two engines) must be finished `flipComplete` above the rails
  if (gate.flipComplete > 0) return gate.flipComplete + gate.flipFall * s
  const tDiv = (Math.sqrt(2 * gate.gateClosing ** 2 + 4 * gate.gateLat * gate.gateMiss) - gate.gateClosing) / gate.gateLat
  return 0.5 * (Math.max(s - gate.flipDv, 5) + 0.7) * tDiv + gate.flipFall * s
}
