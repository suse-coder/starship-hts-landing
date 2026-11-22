/**
 * Powered-descent guidance law: from the end of the flip to the catch pins.
 *
 * One law, one plan, all the way down:
 *
 *  - **Plan** (`descent.ts`): a fuel-optimal convex trajectory from the measured state to the pins
 *    (position 0, velocity (0, 0, −v_contact), thrust upright), re-solved every `LAND.replan` seconds
 *    with a shrinking horizon. The throttle floor is a constraint of the plan — two Raptors cannot
 *    hover, so the descent is a timed deceleration and the plan is what times it. There is no
 *    separate "terminal" controller that could ask for a deceleration the engines cannot fly.
 *  - **Track**: thrust acceleration = plan feed-forward + PD on the plan's position/velocity.
 *  - **Observe**: v̇ − (real thrust vector)/m − g − (aero model)/m is the unmodelled acceleration; it
 *    is fed to the next plan. In still air it should be ≈ 0 — that residual is the model check.
 *  - **Degrade honestly**: when the exact problem has no solution the constraint envelope is widened
 *    (tilt), then the plan switches to minimum-miss mode (land as close as possible). It never
 *    replaces the plan by a law that ignores the engine limits.
 *
 * Deterministic: fixed candidate counts and iteration limits, no wall-clock budgets.
 * Frame: pad ENU relative to the catch point. Pure function of its input and previous state.
 */
import { type V3, v3, add, clamp, norm, scale, sub } from "@ship/math"
import { type DescentPlan, type DescentProblem, type DisturbanceState, descentTimeGuess, fitThrust, G_DESCENT, observeDisturbance, sampleDescent, solveDescent } from "./descent"

export const LAND = {
  /** replan period and the time-to-go below which the last plan is frozen and only tracked (s) */
  replan: 0.4,
  freeze: 1.6,
  /** tracker gains (1/s², 1/s): ω_n ≈ 0.9 rad/s, ζ ≈ 0.9 — below the attitude loop's bandwidth */
  kp: 0.8,
  kd: 1.6,
  /** thrust direction: upright within `tiltEnd` over the last `upright` seconds */
  tiltEnd: 4,
  upright: 3.5,
  /** catch corridor: horizontal radius r0 + rate·(time to go) over the last `time` seconds */
  corridor: { time: 5, r0: 1.0, rate: 2.5 },
  /** glide-slope cone about the pins (deg above the horizon) and its radius at the pins (m) */
  glide: { deg: 25, margin: 6 },
  /** minimum-miss weight (per m²) */
  softPos: 0.05,
  /** throttle margins kept for the tracker */
  floorMargin: 0.04,
  ceilMargin: 0.05,
} as const

/**
 * Constraint ladder. The attitude limits are the vehicle's: the gimbals at the throttle floor give
 * ≈ 0.26 rad/s² in pitch/yaw and the loop slews at ≤ 26°/s; at |u| ≈ 14 m/s² that is ≈ 3.6 m/s⁴ and
 * ≈ 6 m/s³ of horizontal thrust acceleration. Level 0 keeps a margin; the wider levels are used only
 * when level 0 has no solution.
 */
export const LAND_LEVELS = [
  { tilt: 45, rate: 4.2, accel: 2.8 },
  { tilt: 55, rate: 5.0, accel: 3.2 },
  { tilt: 65, rate: 6.0, accel: 3.6 },
] as const

export interface LandingInput {
  readonly t: number
  /** catch pins relative to the catch point and ground velocity, pad ENU */
  readonly rel: V3
  readonly vel: V3
  readonly mass: number
  /** real thrust acceleration (engine force vector / mass), ENU — for the observer */
  readonly uReal: V3
  /** steady thrust acceleration the attitude state stands for (|T|/m along the trimmed thrust line) and its rate, ENU */
  readonly uAxis: V3
  readonly uAxisDot: V3 | null
  /** body axis, ENU */
  readonly xBody: V3
  /** engines available to the plan, per-engine thrust at the ambient pressure (N), exhaust velocity, p∞·A_e */
  readonly nEng: number
  readonly thrustMax: number
  readonly thrustMin: number
  readonly ve: number
  readonly pAe: number
  readonly wind: V3
  /** aerodynamic force (N) for a body axis and an air-relative velocity (ENU) */
  readonly aeroForce: (xHat: V3, vAir: V3) => V3
  readonly contactSpeed: number
}

export interface LandingLawState {
  readonly plan: DescentPlan | null
  readonly planT: number
  readonly dob: DisturbanceState | null
  readonly level: number
  /** engine count the current plan was made for (a change forces a new plan) */
  readonly nEng: number
  /** consecutive failed replans */
  readonly fails: number
  /** earliest time of the next planning attempt after a failure (deterministic backoff, s) */
  readonly backoffT: number
}

export const LANDING_LAW_STATE0: LandingLawState = { plan: null, planT: -1e9, dob: null, level: 0, nEng: 0, fails: 0, backoffT: -1 }

export interface LandingOutput {
  /** commanded thrust acceleration (ENU) and the rate of its reference (attitude feed-forward) */
  readonly u: V3
  readonly uDot: V3
  readonly state: LandingLawState
  readonly events: readonly string[]
  /** the plan from now to the pins, pad ENU relative to the catch point */
  readonly path: readonly V3[]
  /** time to the pins on the plan (s); Infinity without a plan */
  readonly tgo: number
  /** the plan is frozen (final seconds: track only) */
  readonly frozen: boolean
  /** predicted horizontal miss of the current plan (m): 0 for an exact plan */
  readonly miss: number
  /** unmodelled acceleration estimate (m/s²) — the model-fidelity residual */
  readonly residual: V3
}

const ok = (pl: DescentPlan): boolean => pl.converged && pl.slackMax <= 0.05

export const guideLanding = (inp: LandingInput, prev: LandingLawState): LandingOutput => {
  const ev: string[] = []
  const { rel, vel, t } = inp
  const vAir = sub(vel, inp.wind)
  const aeroNow = scale(inp.aeroForce(inp.xBody, vAir), 1 / inp.mass)
  const dob = observeDisturbance(prev.dob, t, vel, add(inp.uReal, aeroNow))
  const lo = (inp.nEng * inp.thrustMin) / inp.mass, hi = (inp.nEng * inp.thrustMax) / inp.mass
  const base = (lv: number, soft: boolean, maxIter = 700, nodes = 25): Omit<DescentProblem, "tf"> => ({
    p0: rel, v0: vel, uNow: inp.uAxis, uDotNow: inp.uAxisDot, mass: inp.mass,
    veh: { nEng: inp.nEng, thrustMax: inp.thrustMax, thrustMin: inp.thrustMin, ve: inp.ve, pAe: inp.pAe },
    pf: v3(0, 0, 0), vf: v3(0, 0, -inp.contactSpeed), dist: dob.d, aero: { wind: inp.wind, force: inp.aeroForce },
    tiltMaxDeg: LAND_LEVELS[lv].tilt, tiltEndDeg: LAND.tiltEnd, uprightTime: LAND.upright,
    rate: LAND_LEVELS[lv].rate, accel: LAND_LEVELS[lv].accel,
    glide: soft ? null : LAND.glide, corridor: soft ? null : LAND.corridor,
    softPos: soft ? LAND.softPos : undefined,
    floorMargin: LAND.floorMargin, ceilMargin: LAND.ceilMargin, maxIter, nodes,
  })
  const last = LAND_LEVELS.length - 1
  let plan = prev.nEng === inp.nEng ? prev.plan : null
  let { planT, level, fails, backoffT } = prev
  const tgoPlan = plan ? plan.t0 + plan.tf - t : 0
  const where = `${Math.hypot(rel.x, rel.y).toFixed(0)} m off at ${rel.z.toFixed(0)} m, ${norm(vel).toFixed(0)} m/s`
  // The final time is the *kinematic* constant-deceleration time to the pins (the hoverslam), not a
  // fuel-optimal free time. Two Raptors cannot throttle below 40 %: a fuel-optimal plan spends the
  // tail-first drag for most of the deceleration and asks for ~3 m/s² of thrust, which the engines
  // cannot deliver — the surplus would have to be dumped as a 75° lean that the aero-stable body
  // cannot hold. The kinematic time keeps the thrust above the floor and the body upright.
  const tfGuess = (): number => descentTimeGuess({ p0: rel, v0: vel, pf: v3(0, 0, 0), vf: v3(0, 0, -inp.contactSpeed) })
  if ((!plan || tgoPlan <= 0) && rel.z > 0.5) {
    // (re)start: the kinematic time through the envelope ladder, then minimum miss on the same time.
    // After a failure the next attempt waits `backoffT`: an infeasible state (a kilometric miss) must
    // not cost a fresh search on every control step — the fallback is cheap and honest.
    if (t >= backoffT && (t - planT >= 0.5 || !prev.plan)) {
      planT = t
      const tf0 = tfGuess()
      let found: DescentPlan | null = null
      for (let lv = 0; lv <= last && !found; lv++) {
        const ks = lv === 0 ? [1, 0.92, 1.1] : [1]
        for (const k of ks) {
          const cand = solveDescent({ ...base(lv, false), tf: tf0 * k }, t)
          if (ok(cand)) { found = cand; level = lv; break }
        }
      }
      if (!found) {
        const cand = solveDescent({ ...base(last, true), tf: tf0 }, t)
        if (ok(cand)) { found = cand; level = last }
      }
      if (found) {
        plan = found; fails = 0; backoffT = -1
        ev.push(found.soft
          ? `landing plan: the catch point is NOT reachable from ${where} on ${inp.nEng} engine(s) — minimum-miss plan, predicted miss ${found.termPos.toFixed(0)} m in ${found.tf.toFixed(1)} s`
          : `landing plan: ${found.tf.toFixed(1)} s to the pins on ${inp.nEng} engine(s), ${(found.fuel / 1e3).toFixed(1)} t, from ${where}; tilt ≤ ${LAND_LEVELS[level].tilt}°, solver residuals ${found.rPrim.toExponential(1)}/${found.rDual.toExponential(1)}`)
      } else {
        fails++
        backoffT = t + 3
        if (fails === 1) ev.push(`landing plan: no solution from ${where} on ${inp.nEng} engine(s) — braking upright, next attempt in 3 s`)
      }
    }
  } else if (plan && t - planT >= LAND.replan && tgoPlan > LAND.freeze && t >= backoffT) {
    planT = t
    const tfR = tfGuess()
    let next: DescentPlan | null = null
    if (plan.soft) {
      // keep trying to recover the exact problem (widest envelope), otherwise re-solve minimum miss
      const ex = solveDescent({ ...base(last, false, 700), tf: tfR, prev: plan }, t)
      if (ok(ex)) { next = ex; level = last; ev.push(`landing plan: the catch point is reachable again (${where})`) }
      else {
        const so = solveDescent({ ...base(last, true, 700), tf: tfR, prev: plan }, t)
        if (ok(so)) next = so
      }
    } else {
      const cand = solveDescent({ ...base(level, false, 700), tf: tfR, prev: plan }, t)
      if (ok(cand)) next = cand
      else {
        // one reopen of the envelope, then the kinematic time ±8 %, then minimum miss
        for (let lv = level + 1; lv <= last && !next; lv++) {
          for (const k of [1, 1.08]) {
            const c = solveDescent({ ...base(lv, false, 700), tf: tfR * k, prev: plan }, t)
            if (ok(c)) { next = c; level = lv; break }
          }
        }
        if (!next) {
          const so = solveDescent({ ...base(last, true, 700), tf: tfR, prev: plan }, t)
          if (ok(so)) { next = so; level = last; ev.push(`landing plan: catch point no longer reachable from ${where} — minimum-miss plan, predicted miss ${so.termPos.toFixed(1)} m`) }
        }
      }
    }
    if (next) { plan = next; fails = 0; backoffT = -1 } else { fails++; backoffT = t + 3 }
  }
  let u: V3, uDot = v3(0, 0, 0)
  let path: V3[] = []
  let tgo = Infinity, frozen = false, miss = 0
  if (plan) {
    const ref = sampleDescent(plan, t)
    tgo = plan.t0 + plan.tf - t
    frozen = tgo <= LAND.freeze
    miss = plan.soft ? plan.termPos : 0
    u = v3(
      ref.u.x + LAND.kp * (ref.p.x - rel.x) + LAND.kd * (ref.v.x - vel.x),
      ref.u.y + LAND.kp * (ref.p.y - rel.y) + LAND.kd * (ref.v.y - vel.y),
      ref.u.z + LAND.kp * (ref.p.z - rel.z) + LAND.kd * (ref.v.z - vel.z),
    )
    uDot = ref.uDot
    // past the end of the plan (pins not yet on the rails): hold the contact speed, upright
    if (tgo <= 0) u = v3(-LAND.kd * vel.x, -LAND.kd * vel.y, G_DESCENT + 2 * (-inp.contactSpeed - vel.z) - aeroNow.z - dob.d.z)
    u = fitThrust(u, lo, hi, Math.tan((LAND_LEVELS[level].tilt + 5) * Math.PI / 180))
    const pl = plan
    for (let k = 0; k < pl.K; k++) if (pl.t0 + k * pl.h >= t - pl.h) path.push(v3(pl.p[3 * k], pl.p[3 * k + 1], Math.max(pl.p[3 * k + 2], 0)))
  } else {
    // no plan at all: kill the horizontal speed and brake at the deceleration the height requires
    const need = (vel.z * vel.z - inp.contactSpeed * inp.contactSpeed) / (2 * Math.max(rel.z, 1))
    u = fitThrust(v3(-0.5 * vel.x, -0.5 * vel.y, G_DESCENT + clamp(need, 0, hi) - aeroNow.z - dob.d.z), lo, hi, Math.tan(30 * Math.PI / 180))
    path = [v3(rel.x, rel.y, Math.max(rel.z, 0))]
  }
  return { u, uDot, state: { plan, planT, dob, level, nEng: inp.nEng, fails, backoffT }, events: ev, path, tgo, frozen, miss, residual: dob.d }
}

/**
 * Trimmed thrust line in body axes for a set of running engines: the resultant of the engine forces
 * acts at their thrust centroid, and in steady flight it must pass through the centre of gravity.
 * With a symmetric set this is the body axis; with an asymmetric one (two of three engines, or one)
 * the gimbals cant the thrust by atan(offset / lever) and the body flies tilted by that angle.
 */
export const trimThrustLine = (engines: readonly { readonly gimbal: V3 }[], thrusts: readonly number[], xcg: number): V3 => {
  let T = 0, cy = 0, cz = 0, gx = 0
  for (let i = 0; i < engines.length; i++) {
    const Ti = thrusts[i] ?? 0
    if (!(Ti > 1e3)) continue
    T += Ti; cy += Ti * engines[i].gimbal.y; cz += Ti * engines[i].gimbal.z; gx += Ti * engines[i].gimbal.x
  }
  if (T < 1e3) return v3(1, 0, 0)
  const L = xcg - gx / T
  const d = v3(L, -cy / T, -cz / T)
  return scale(d, 1 / norm(d))
}

/**
 * Engine health monitor: an engine that has been commanded on for longer than its start sequence
 * and still delivers less than a quarter of the throttle-floor thrust has failed. Pure.
 */
export interface EngineHealth {
  readonly failed: readonly boolean[]
  /** time each engine was commanded on (−1: off) */
  readonly onT: readonly number[]
}

export const ENGINE_HEALTH0: EngineHealth = { failed: [false, false, false], onT: [-1, -1, -1] }

export const monitorEngines = (h: EngineHealth, t: number, commanded: readonly number[], thrusts: readonly number[], spoolTime: number, floorThrust: number): { readonly health: EngineHealth; readonly newlyFailed: readonly number[] } => {
  const failed = h.failed.slice(), onT = h.onT.slice()
  const newly: number[] = []
  for (let i = 0; i < failed.length; i++) {
    const on = commanded.includes(i)
    if (!on) { onT[i] = -1; continue }
    if (onT[i] < 0) onT[i] = t
    if (!failed[i] && t - onT[i] > spoolTime + 0.4 && (thrusts[i] ?? 0) < 0.25 * floorThrust) { failed[i] = true; newly.push(i) }
  }
  return { health: { failed, onT }, newlyFailed: newly }
}

/** engines to run: keep the ones already running, fill up from the healthy ones, never a failed one */
export const selectEngines = (failed: readonly boolean[], running: readonly number[], want: number): number[] => {
  const healthy = failed.map((f, i) => (f ? -1 : i)).filter((i) => i >= 0)
  const keep = running.filter((i) => healthy.includes(i))
  const rest = healthy.filter((i) => !keep.includes(i))
  return [...keep, ...rest].slice(0, Math.max(0, want)).sort((a, b) => a - b)
}
