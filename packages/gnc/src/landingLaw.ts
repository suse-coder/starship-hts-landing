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
  kp: 0.3,
  kd: 1.0,
  /** thrust direction: upright within `tiltEnd` over the last `upright` seconds */
  tiltEnd: 4,
  upright: 3.5,
  /** catch corridor: horizontal radius r0 + rate·(time to go) over the last `time` seconds */
  corridor: { time: 5, r0: 1.0, rate: 2.5 },
  /** glide-slope cone about the pins (deg above the horizon) and its radius at the pins (m) */
  glide: { deg: 25, margin: 6 },
  /** minimum-miss weight (per m²) */
  softPos: 0.05,
  /** plan-vs-state deviation (position + 1 s × velocity, m) above which the plan is re-solved */
  replanAt: 12,
  /** staging check period (s) and envelope: height above the rails (m), speed and horizontal speed (m/s) */
  stageCheck: 0.5,
  stageMinH: 25,
  stageMaxH: 380,
  stageMaxV: 30,
  stageMaxVh: 20,
  /** horizontal offset (m) inside which the final approach may start, and propellant reserve (kg) */
  stageMaxOff: 80,
  stageReserve: 1500,
  /**
   * Staging gate above the pins: the burn set brakes to (0, 0, h) with sink v and thrust within
   * `tilt` of vertical; below h + minAbove at the first plan the burn goes to the pins directly.
   * From 300 m at 45 m/s one Raptor needs ≈ 13 m/s² for ≈ 13 s (two thirds of its thrust, ≈ 6 t).
   * The gate is this high because the flip ends ≈ 650 m above the rails: a brake that reaches the
   * gate in ≈ 6 s runs at ≈ 70 % throttle, where the gimbals hold the attitude.
   */
  gate: { h: 110, v: 14, tilt: 8, upright: 2.5, minAbove: 40, tMin: 0.8, throttle: 0.7, tiltMax: 60, flipTilt: 50 },
  /**
   * Final approach on the reduced set. One off-centre gimbal makes roll whenever it pushes
   * tangentially (740 kN·m at the stop against 36 kN·m of cold-gas couples: measured roll rate
   * −77°/s after a 48° righting manoeuvre), so the final approach starts upright and stays gentle:
   * small tilt, slow slews — the gimbal then works around its trim angle and the roll impulses of
   * its rest-to-rest corrections cancel.
   */
  final: { tilt: 14, rate: 2.0, accel: 1.2, startTiltDeg: 20 },
  /**
   * Flip-and-brake on the burn set (roll stays controlled by differential gimbal). The flip does not
   * stop at vertical: it leans back against the horizontal velocity by `tilt0 + tiltPerV·v_h`
   * (capped), so the lean fades as the speed is killed and the body arrives near upright and slow.
   * It is a function of the ship's own motion only — the same manoeuvre from every ignition point.
   * Thrust: `FLIP_THRUST` while rotating up, `thrust` (× one engine's maximum, in total) while
   * braking. It ends below `endVh` m/s and `endTilt`° (or at `tMax` s).
   */
  flip: { tilt0: 50, tiltPerV: 0, tiltMax: 50, endVh: 1e9, endTilt: 22, thrust: 1.2, tMax: 9, rollAlign: true },
  /**
   * Hand-over from the burn set to the single final engine (see the field branch of
   * `guideLanding`): armed below `vMax` m/s (aerodynamic moment small) and `hMax` m; executed when
   * the roll coupling is nulled and the tilt is below `tiltMax`°, or after `wait` s. `sinkMax`: the
   * single engine's sink limit (m/s), so it does not speed up again.
   */
  stage: { vMax: 25, hMax: 380, tiltMax: 30, wait: 2.5, sinkMax: 18, rollLead: -1 },
  /**
   * Tower keep-out of the final approach (pin frame: x across the rails, y along them, m). `xIn`:
   * half-width of the corridor between the *open* arms that the hull fits with margin (open gap
   * 26 m, hull Ø 9 m); `yMax`: north limit before the carriage; `zClear`: pin height above the
   * rails at which the whole hull (38 m below the pins) is above the arm tops.
   */
  // Tower keep-out of the final approach. Disabled by default (`xIn` large): it is correct but it
  // changes the last 40 m and the terminal timing has not been retuned for it yet (measured: the
  // ship crossed the rail plane at 10.8 m/s with the arms open).
  keepOut: { xIn: 1e6, yMax: 8, zClear: 0 },
  /** hand over to the final-approach set as soon as the flip is complete (no two-engine brake) */
  stageAtStart: true,
  /**
   * Hover gate of the throttleable set: its plan ends `h` metres above the rails at `v` m/s; from
   * there the ship is centred and lowered onto the arms (hold-until-centred). Centred means inside
   * what the tracking arms can still reach (`across`, m) and the rails' length (`along`, m), slow.
   */
  hover: { h: 1.0, v: 1.0, across: 4, along: 8, vh: 0.9, down: 1.2, kp: 0.25, kd: 0.9, aMax: 0.8, lead: 0.8, settle: 1.2, settleV: 0.3, settleKd: 0.8 },
  /**
   * Lateral tracking gains of the single-engine descent. The tilt → attitude path of one gimbal has
   * a lag of over a second; a lateral loop at the two-engine gains (0.3 / 1.0, ω_n 0.55 rad/s) sat
   * on that lag and limit-cycled at ±10 m/s. The bandwidth must sit below it: ω_n ≈ 0.25 rad/s,
   * well damped, with the correction capped (the plan's feed-forward does the manoeuvring).
   */
  finalLat: { kp: 0.06, kd: 0.55, aMax: 2.0 },
  /**
   * Single-engine descent law (velocity field). After the flip one gimballed Raptor turns the ship
   * at ≈ 0.2 rad/s² — a timed optimal plan asked for thrust-direction changes the body reached a
   * second late, and the tracker then chased its own lag (catches scattered over the ignition
   * grid). The descent is therefore a feedback law the attitude loop can follow from any state:
   *   horizontal: approach speed v(d) = min(kp·d, √(2·aB·(d − d_l/2))) towards the catch axis
   *               (linear inside d_l = aB/kp², constant-deceleration outside), a_h = kv·(v_des − v),
   *               tilt ≤ `tilt` (≤ `tiltLow` below `hLow` metres above the rails);
   *   vertical:   sink v_z(h) = −√(2·aV·(h − h_hover)) − vGate down to the hover gate, where the
   *               ship is held until centred and then lowered onto the arms.
   * The descent stays inside a cone above the hover gate (height ≥ `cone` × the horizontal offset
   * beyond `coneFree` metres): a ship that is still off-axis waits higher up instead of coming down
   * beside the arms.
   * The path shown and the arm-closing lead are this same law integrated forward with the measured
   * attitude lag `tau` — a prediction of what will be flown, not a reference to be chased.
   */
  field: { kp: 0.25, aB: 2.5, kv: 0.5, tilt: 28, tiltLow: 10, hLow: 30, aV: 2.4, vGate: 0.8, kz: 1.5, tau: 2.0 },
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

/**
 * Constraint ladder of the single-engine descent: the same idea as LAND_LEVELS with the slews one
 * gimbal can follow (≈ 0.2 rad/s², 7°/s) — at |u| ≈ 12 m/s² that is ≈ 1.5 m/s³ and ≈ 2.4 m/s⁴.
 */
export const FINAL_LEVELS = [
  { tilt: 35, rate: 1.4, accel: 1.0 },
  { tilt: 45, rate: 1.8, accel: 1.4 },
  { tilt: 55, rate: 2.4, accel: 2.0 },
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
  /** engine count of the final approach (< nEng): staged to when a plan on it is feasible */
  readonly stageTo?: number
  /** landing propellant left (kg): a staged plan must fit it with a reserve */
  readonly propellant?: number
  readonly thrustMax: number
  readonly thrustMin: number
  readonly ve: number
  readonly pAe: number
  readonly wind: V3
  /** aerodynamic force (N) for a body axis and an air-relative velocity (ENU) */
  readonly aeroForce: (xHat: V3, vAir: V3) => V3
  readonly contactSpeed: number
  /** tracker gains (tuning); default to the module constants */
  readonly kp?: number
  readonly kd?: number
  readonly hoverKp?: number
  readonly hoverKd?: number
  /** the tower has started closing the arms (the lowering is committed) */
  readonly armed?: boolean
  /** the single-engine roll coupling is nulled (guidance: |H| small); undefined = not checked */
  readonly quiet?: boolean
  /**
   * Lateral acceleration the model may use (m/s²). The flight passes nothing (the geometric cap);
   * *scoring* a belly plan passes a conservative value, because one gimballed Raptor at the throttle
   * floor delivers ≈ 1–2 m/s², far less than the commanded tilt implies.
   */
  readonly aLat?: number
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
  /** the law has switched to the reduced engine set (`LandingInput.stageTo`) */
  readonly staged: boolean
  readonly stageT: number
  /** absolute time of the staging gate (the brake's fixed end time); null before the brake starts */
  readonly gateT: number | null
  /** field law: last predicted path and its time to go at `planT` (refreshed at 5 Hz) */
  readonly path: readonly V3[]
  readonly tgoField: number
}

export const LANDING_LAW_STATE0: LandingLawState = { plan: null, planT: -1e9, dob: null, level: 0, nEng: 0, fails: 0, backoffT: -1, staged: false, stageT: -1e9, gateT: null, path: [], tgoField: Infinity }

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
  /** time until the pins reach the rails (s), for the arm-closing lead; Infinity while undecided */
  readonly tRail: number
  /** the plan is frozen (final seconds: track only) */
  readonly frozen: boolean
  /** predicted horizontal miss of the current plan (m): 0 for an exact plan */
  readonly miss: number
  /** unmodelled acceleration estimate (m/s²) — the model-fidelity residual */
  readonly residual: V3
}

// The plan's thrust magnitude may sit below the engine floor: the command stage (`fitThrust`) spends
// the mandatory surplus laterally, so the plan's vertical profile is exact and the floor is a command
// constraint, not a trajectory one. Requiring the plan itself to meet the floor forced the 2-engine
// plan upright (the linearised floor is n̂·u ≥ ρ, and n̂ was vertical), which is a climb.
const ok = (pl: DescentPlan): boolean => pl.converged && pl.termPos < 1

/** position + 1 s of velocity error against the plan at time t (m) */
const deviation = (pl: DescentPlan, t: number, p: V3, v: V3): number => {
  const r = sampleDescent(pl, t)
  return Math.hypot(r.p.x - p.x, r.p.y - p.y, r.p.z - p.z) + Math.hypot(r.v.x - v.x, r.v.y - v.y, r.v.z - v.z)
}

/**
 * Fit a brake command into the engines' band without changing what it is for: the magnitude goes
 * into [lo, hi] along the commanded direction, and a direction below `maxTiltDeg` from the vertical
 * is raised to it at the same magnitude. (`fitThrust` is vertical-first — right for the timed final
 * descent, wrong here: it cut a near-horizontal 90 % brake to a third and the ship overshot.)
 */
export const fitBrake = (u: V3, lo: number, hi: number, maxTiltDeg: number): V3 => {
  const mag = clamp(norm(u), lo, hi)
  const h = Math.hypot(u.x, u.y)
  const tilt = Math.min(Math.atan2(h, u.z), (maxTiltDeg * Math.PI) / 180)
  const hx = h > 1e-9 ? u.x / h : 0, hy = h > 1e-9 ? u.y / h : 0
  return v3(mag * Math.sin(tilt) * hx, mag * Math.sin(tilt) * hy, mag * Math.cos(tilt))
}

/**
 * Thrust-acceleration profile to the staging gate, linear in time (ZEM/ZEV form): `u0` now, `u1` at
 * the gate, for a time to go `tg` and a known environment acceleration `env` (gravity + aero).
 */
export const gateProfile = (rel: V3, vel: V3, env: V3, tg: number): { u0: V3; u1: V3 } => {
  const pg = v3(0, 0, LAND.gate.h), vg = v3(0, 0, -LAND.gate.v)
  const zem = v3(pg.x - (rel.x + vel.x * tg + 0.5 * env.x * tg * tg), pg.y - (rel.y + vel.y * tg + 0.5 * env.y * tg * tg), pg.z - (rel.z + vel.z * tg + 0.5 * env.z * tg * tg))
  const zev = v3(vg.x - (vel.x + env.x * tg), vg.y - (vel.y + env.y * tg), vg.z - (vel.z + env.z * tg))
  const a = 6 / (tg * tg), b = 2 / tg
  return {
    u0: v3(a * zem.x - b * zev.x, a * zem.y - b * zev.y, a * zem.z - b * zev.z),
    u1: v3(-a * zem.x + 2 * b * zev.x, -a * zem.y + 2 * b * zev.y, -a * zem.z + 2 * b * zev.z),
  }
}

/**
 * Gate time: the brake whose start thrust is nearest the design throttle (`LAND.gate.throttle` of
 * the ceiling) inside the engines' band — never pointing down, start tilt ≤ `tiltMax`, and at the
 * gate vertical and above the floor. Gimbal torque scales with thrust (≈ 10 MN·m for two Raptors at
 * the floor, about the hull's crossflow moment at 75–100 m/s: at the floor the body could not be
 * turned to the braking attitude), so the brake is flown well above the floor; and it must last
 * long enough for the body to lean back and come upright again (26°/s).
 */
export const gateTime = (rel: V3, vel: V3, env: V3, lo: number, hi: number): number => {
  const guess = descentTimeGuess({ p0: rel, v0: vel, pf: v3(0, 0, LAND.gate.h), vf: v3(0, 0, -LAND.gate.v) })
  let pick = guess, best = Infinity
  for (let k = 0.5; k <= 1.8; k += 0.025) {
    const tg = guess * k
    const { u0, u1 } = gateProfile(rel, vel, env, tg)
    const peak = Math.max(norm(u0), norm(u1))
    const tilt0 = Math.atan2(Math.hypot(u0.x, u0.y), Math.max(u0.z, 1e-6))
    // hard limits as penalties (so there is always an answer), then closeness to the design throttle
    const bad = 20 * Math.max(0, peak - 0.9 * hi) + 20 * Math.max(0, 1.05 * lo - u1.z) + 20 * Math.max(0, -u0.z)
      + 30 * Math.max(0, tilt0 - (LAND.gate.tiltMax * Math.PI) / 180)
    const cost = bad + Math.abs(norm(u0) - LAND.gate.throttle * hi)
    if (cost < best) { best = cost; pick = tg }
  }
  return pick
}

/** environment of the field law: known non-thrust acceleration without gravity (aero + residual), band */
interface FieldEnv {
  readonly ez: number
  readonly lo: number
  readonly hi: number
  readonly contactSpeed: number
  readonly hoverKp: number
  readonly hoverKd: number
  /** the tower has started closing the arms: the lowering is committed (the arms will stop at the hull) */
  readonly armed?: boolean
  /** sink limit of the approach (m/s); default 45 */
  readonly sinkMax?: number
  /**
   * Lateral acceleration the model may use (m/s²). The flight passes nothing (the geometric cap);
   * *scoring* a belly plan passes a conservative value, because one gimballed Raptor at the throttle
   * floor delivers ≈ 1–2 m/s², far less than the commanded tilt implies.
   */
  readonly aLat?: number
}

/**
 * Velocity-field command of the single-engine descent (see `LAND.field`): thrust acceleration for a
 * state, and whether the ship is on its way down to the rails (`tRail`, s; Infinity while it waits).
 */
export const fieldCommand = (rel: V3, vel: V3, e: FieldEnv): { u: V3; tRail: number } => {
  const F = LAND.field, H = LAND.hover, K = LAND.keepOut
  // Lateral law with the actuator lag in the reference. The command is a thrust direction, and the
  // body takes ≈ 0.8 s to get there (one gimbal, ~1 g): a law on the *current* position therefore
  // always answers late, passes the axis and comes back — the ±4 m terminal oscillation. Leading the
  // position by the lag (x + v·τ) makes the loop answer where the ship *will* be, and inside the
  // settle band only the velocity is nulled, so it parks on the pins instead of hunting.
  const lat = (x: number, v: number): number => {
    const lead = LAND.hover.lead
    if (Math.abs(x) < H.settle && Math.abs(v) < H.settleV) return -H.settleKd * v
    return -H.kp * (x + v * lead) - H.kd * v
  }
  const d = Math.hypot(rel.x, rel.y), vh = Math.hypot(vel.x, vel.y)
  // Tower keep-out. The hull hangs ≈ 38 m below the pins, so long before the pins reach the rails
  // the aft section is level with the arms. The only way down is the corridor between the open
  // arms (|x| < xIn) south of the carriage (y < yMax); anywhere else the pins stay `zClear` above
  // the rails, i.e. the whole hull above the arm tops. Without this the ship came down beside the
  // tower and leaned on the outside of an open arm until the propellant ran out (measured, 19 m off).
  const inCorridor = Math.abs(rel.x) < K.xIn && rel.y < K.yMax
  const zMin = inCorridor ? H.h : K.zClear
  const above = rel.z - zMin
  const centred = Math.abs(rel.x) < H.across && Math.abs(rel.y) < H.along && vh < H.vh
  // once the arms are closing the tower has committed: keep going down (the arms are compliant and
  // stop at the hull); likewise below the gate between the arms
  // the arms being closed only commits the *last* metres: above that the law keeps translating
  // (the arms close as soon as the ship is between them, which can be 50 m up)
  const committed = (e.armed === true && rel.z < 12) || (rel.z < 0.6 * H.h && Math.abs(rel.x) < H.across + 1.5)
  let vzT: number, tRail = Infinity
  let ax: number, ay: number
  if (!committed && (!inCorridor || above > 1.5 || d > 2 * H.along)) {
    // horizontal: approach speed towards the catch axis, linear inside d_l, constant deceleration outside
    const dl = F.aB / (F.kp * F.kp)
    const vd = d <= dl ? F.kp * d : Math.sqrt(2 * F.aB * (d - dl / 2))
    const dx = d > 1e-6 ? rel.x / d : 0, dy = d > 1e-6 ? rel.y / d : 0
    // near the axis the lead/band law takes over from the approach profile
    if (d < H.settle * 3) { ax = lat(rel.x, vel.x); ay = lat(rel.y, vel.y) }
    else { ax = F.kv * (-dx * vd - vel.x); ay = F.kv * (-dy * vd - vel.y) }
    // vertical: the sink is coupled to the time the lateral correction still needs (the ship must
    // reach its floor when the horizontal motion is nulled, not before); below the floor it climbs
    const aLat = Math.max(G_DESCENT * Math.tan((F.tilt * Math.PI) / 180), 1.5)
    const away = d > 1e-6 ? -(rel.x * vel.x + rel.y * vel.y) / d : 0
    const tLat = (Math.max(away, 0) + Math.sqrt(Math.max(away, 0) ** 2 + 2 * aLat * d)) / aLat
    vzT = above > 0 ? -clamp(above / Math.max(tLat, 1.5), 1.5, e.sinkMax ?? 45) : Math.min(4, -0.5 * above)
    // time to the rails if the descent continues: floor, then the lowering
    const toGate = Math.max(rel.z - H.h, 0)
    tRail = inCorridor ? (Math.sqrt(2 * toGate / F.aV) + toGate / Math.max(-vel.z, 3)) / 2 + H.h / H.down : Infinity
  } else {
    // hold-until-centred above the rails, then down at the lowering speed; the last half metre at
    // the contact speed
    const go = centred || committed
    vzT = go ? (rel.z > 0.5 ? -H.down : -e.contactSpeed) : above > 0 ? -0.6 : 0
    tRail = go ? Math.max(rel.z, 0) / H.down : Infinity
    const kp = e.hoverKp ?? H.kp, kd = e.hoverKd ?? H.kd
    const lead = H.lead
    const band = (x: number, v: number): number =>
      Math.abs(x) < H.settle && Math.abs(v) < H.settleV ? -H.settleKd * v : -kp * (x + v * lead) - kd * v
    ax = clamp(band(rel.x, vel.x), -H.aMax, H.aMax)
    ay = clamp(band(rel.y, vel.y), -H.aMax, H.aMax)
  }
  const uz = clamp(G_DESCENT + F.kz * (vzT - vel.z) - e.ez, 0.3 * e.lo, e.hi)
  const tiltMax = (rel.z > F.hLow ? F.tilt : F.tiltLow + (F.tilt - F.tiltLow) * Math.max(rel.z, 0) / F.hLow) * Math.PI / 180
  const aMax = Math.min(Math.max(uz, e.lo) * Math.tan(tiltMax), e.aLat ?? Infinity), am = Math.hypot(ax, ay), sc = am > aMax ? aMax / am : 1
  return { u: fitThrust(v3(ax * sc, ay * sc, uz), e.lo, e.hi, Math.tan(tiltMax + 5 * Math.PI / 180)), tRail }
}

/**
 * The field law integrated forward (point mass, thrust direction lagging the command by
 * `LAND.field.tau`): the predicted path to the rails and the time to them.
 */
export const fieldPredict = (rel: V3, vel: V3, uNow: V3, e: FieldEnv, tMax = 70, dt = 0.2): { path: V3[]; t: number; reached: boolean } => {
  let px = rel.x, py = rel.y, pz = rel.z, vx = vel.x, vy = vel.y, vz = vel.z
  let ux = uNow.x, uy = uNow.y
  const path: V3[] = [v3(px, py, Math.max(pz, 0))]
  const k = dt / LAND.field.tau
  let t = 0
  for (; t < tMax && pz > 0.05; t += dt) {
    const c = fieldCommand(v3(px, py, pz), v3(vx, vy, vz), e).u
    ux += k * (c.x - ux); uy += k * (c.y - uy)
    vx += ux * dt; vy += uy * dt; vz += (c.z + e.ez - G_DESCENT) * dt
    px += vx * dt; py += vy * dt; pz += vz * dt
    path.push(v3(px, py, Math.max(pz, 0)))
  }
  return { path, t, reached: pz <= 0.05 }
}

export const guideLanding = (inp: LandingInput, prev: LandingLawState): LandingOutput => {
  const ev: string[] = []
  const { rel, vel, t } = inp
  // No high-rate re-solving near the rails: measured, a 30 Hz terminal re-plan changed nothing, and
  // every re-plan re-anchors the reference. The fine, fast part of the terminal is the tracker and
  // the attitude loop (every control step); the plan is re-solved only on a real deviation.
  const replan = LAND.replan
  const freeze = LAND.freeze
  const replanAt = LAND.replanAt
  // Engine staging: the burn runs `inp.nEng` engines; once a plan on `inp.stageTo` engines is
  // feasible from the current state the law switches to it for good. One Raptor throttles from
  // ≈ 0.75 g to 2 g at landing mass, so the final approach is no longer a no-hover hoverslam.
  let staged = (prev.staged || LAND.stageAtStart) && inp.stageTo !== undefined && inp.stageTo < inp.nEng
  let stageT = prev.stageT
  const nE = (): number => (staged && inp.stageTo !== undefined ? inp.stageTo : inp.nEng)
  const vAir = sub(vel, inp.wind)
  const aeroNow = scale(inp.aeroForce(inp.xBody, vAir), 1 / inp.mass)
  const kp = inp.kp ?? LAND.kp
  const kd = inp.kd ?? LAND.kd
  const dob = observeDisturbance(prev.dob, t, vel, add(inp.uReal, aeroNow))
  // Two-stage descent. While the burn set (which cannot throttle below the weight) is running and a
  // final-approach set exists, the plan does NOT go to the pins: it brakes to a *staging gate* above
  // them (LAND.gate: height, sink rate, no horizontal speed), where the law hands over to the
  // throttleable set. To the gate the required thrust stays above the floor (no surplus to dump
  // sideways) and the thrust direction stays near retrograde, where the aerodynamic moment is small:
  // at the throttle floor two gimbals have ≈ 10 MN·m, about what the hull's crossflow moment is at
  // 75–100 m/s, so a plan that leans 45° across the airflow saturated them (measured: the body
  // azimuth ran 70° past the command and the ship coned away).
  const finalSet = (nEng: number): boolean => inp.stageTo !== undefined && inp.stageTo < inp.nEng && nEng === inp.stageTo
  const toGate = (): boolean => inp.stageTo !== undefined && inp.stageTo < inp.nEng && !staged && rel.z > LAND.gate.h + LAND.gate.minAbove
  const base = (lv: number, soft: boolean, maxIter = 700, nodes = 25, nEng = nE(), gate = toGate()): Omit<DescentProblem, "tf"> => ({
    p0: rel, v0: vel, uNow: inp.uAxis, uDotNow: inp.uAxisDot, mass: inp.mass,
    veh: { nEng, thrustMax: inp.thrustMax, thrustMin: inp.thrustMin, ve: inp.ve, pAe: inp.pAe },
    pf: gate ? v3(0, 0, LAND.gate.h) : finalSet(nEng) ? v3(0, 0, LAND.hover.h) : v3(0, 0, 0),
    vf: v3(0, 0, gate ? -LAND.gate.v : finalSet(nEng) ? -LAND.hover.v : -inp.contactSpeed), dist: dob.d, aero: { wind: inp.wind, force: inp.aeroForce },
    tiltMaxDeg: finalSet(nEng) ? FINAL_LEVELS[lv].tilt : LAND_LEVELS[lv].tilt, tiltEndDeg: gate ? LAND.gate.tilt : LAND.tiltEnd, uprightTime: gate ? LAND.gate.upright : LAND.upright,
    rate: finalSet(nEng) ? FINAL_LEVELS[lv].rate : LAND_LEVELS[lv].rate, accel: finalSet(nEng) ? FINAL_LEVELS[lv].accel : LAND_LEVELS[lv].accel,
    // No glide-slope cone: it forced the descent above 25° elevation from the pad, which a fast low
    // approach after the flip cannot satisfy (it was the main source of infeasibility, measured).
    // The catch corridor protects the final seconds above the arms.
    glide: null, corridor: soft || gate ? null : LAND.corridor,
    softPos: soft ? LAND.softPos : undefined,
    floorMargin: LAND.floorMargin, ceilMargin: LAND.ceilMargin, maxIter, nodes,
  })
  const last = LAND_LEVELS.length - 1
  const where = `${Math.hypot(rel.x, rel.y).toFixed(0)} m off at ${rel.z.toFixed(0)} m, ${norm(vel).toFixed(0)} m/s`
  let plan = prev.nEng === nE() ? prev.plan : null
  // ---- brake to the staging gate (burn set). Explicit guidance, no optimiser: the thrust
  // acceleration is the linear-in-time profile that reaches the gate position and velocity at the
  // gate time (zero-effort-miss / zero-effort-velocity form), with gravity, the aero model and the
  // observed residual as the known environment. For a ship delivered ≈ v_h·t/3 short of the tower
  // the lateral part decays linearly to zero — the brake starts beyond retrograde and ends upright
  // by itself, which is what the single-engine final needs. The gate time is fixed once (the time
  // to go then counts down) so that the vertical thrust at the gate sits just above the floor.
  const canStage = inp.stageTo !== undefined && inp.stageTo < inp.nEng
  void canStage
  let gateT = prev.gateT
  // the timed gate brake is retired: with a final set the whole descent is the field law
  const brakeMode = false as boolean
  let { planT, level, fails, backoffT } = prev
  const tgoPlan = plan ? plan.t0 + plan.tf - t : 0
  // The final time is the *kinematic* constant-deceleration time to the pins (the hoverslam), not a
  // fuel-optimal free time. Two Raptors cannot throttle below 40 %: a fuel-optimal plan spends the
  // tail-first drag for most of the deceleration and asks for ~3 m/s² of thrust, which the engines
  // cannot deliver — the surplus would have to be dumped as a 75° lean that the aero-stable body
  // cannot hold. The kinematic time keeps the thrust above the floor and the body upright.
  const tfGuess = (gate = toGate()): number => descentTimeGuess({
    p0: rel, v0: vel,
    pf: gate ? v3(0, 0, LAND.gate.h) : finalSet(nE()) ? v3(0, 0, LAND.hover.h) : v3(0, 0, 0),
    vf: v3(0, 0, gate ? -LAND.gate.v : finalSet(nE()) ? -LAND.hover.v : -inp.contactSpeed),
  })
  // once the throttleable set's plan has ended near the hover gate the ship is lowered by the
  // hold-until-centred law; a new plan is made only if it ended far above the gate
  const hovering = staged && plan !== null && tgoPlan <= 0 && rel.z < LAND.hover.h + 25
  const field = staged
  if (!brakeMode && !hovering && !field && (!plan || tgoPlan <= 0) && rel.z > 0.5) {
    // (re)start: the kinematic time through the envelope ladder, then minimum miss on the same time.
    // After a failure the next attempt waits `backoffT`: an infeasible state (a kilometric miss) must
    // not cost a fresh search on every control step — the fallback is cheap and honest.
    if (t >= backoffT && (t - planT >= 0.5 || !prev.plan)) {
      planT = t
      // a set that hovers is not bound to the kinematic time: from a slow start use an
      // accelerate-then-brake time (≈ 1.5 m/s²) instead of crawling down at the current sink rate
      const tf0 = finalSet(nE()) ? Math.min(tfGuess(), 2 * Math.sqrt(Math.max(rel.z - LAND.hover.h, 1) / 1.5) + 2) : tfGuess()
      let found: DescentPlan | null = null
      // The kinematic time is the shortest flyable descent; a lateral divert may need longer, which
      // two Raptors at the floor buy by tilting (the surplus goes sideways — exactly the divert).
      for (let lv = 0; lv <= last && !found; lv++) {
        // a set that throttles through the weight is free in time: constant deceleration first
        const ks = finalSet(nE()) ? [0.9, 1, 0.8, 1.15, 1.3] : lv === 0 ? [1.15, 1.3, 1, 1.5, 0.92] : [1.15, 1.3, 1.5]
        for (const k of ks) {
          const cand = solveDescent({ ...base(lv, false), tf: tf0 * k }, t)
          if (ok(cand)) { found = cand; level = lv; break }
        }
      }
      if (!found) {
        let bestSoft: DescentPlan | null = null
        for (const k of [1, 1.2, 1.4]) {
          const cand = solveDescent({ ...base(last, true), tf: tf0 * k }, t)
          if (ok(cand) && (!bestSoft || cand.termPos < bestSoft.termPos)) bestSoft = cand
        }
        if (bestSoft) { found = bestSoft; level = last }
      }
      if (found) {
        plan = found; fails = 0; backoffT = -1
        ev.push(found.soft
          ? `landing plan: the catch point is NOT reachable from ${where} on ${nE()} engine(s) — minimum-miss plan, predicted miss ${found.termPos.toFixed(0)} m in ${found.tf.toFixed(1)} s`
          : `landing plan: ${found.tf.toFixed(1)} s to the ${toGate() ? `staging gate (${LAND.gate.h} m, ${LAND.gate.v} m/s)` : "pins"} on ${nE()} engine(s), ${(found.fuel / 1e3).toFixed(1)} t, from ${where}; tilt ≤ ${LAND_LEVELS[level].tilt}°, solver residuals ${found.rPrim.toExponential(1)}/${found.rDual.toExponential(1)}`)
      } else {
        fails++
        backoffT = t + 3
        if (fails === 1) ev.push(`landing plan: no solution from ${where} on ${nE()} engine(s) — braking upright, next attempt in 3 s`)
      }
    }
  } else if (!brakeMode && !field && plan && t - planT >= replan && tgoPlan > freeze && t >= backoffT && deviation(plan, t, rel, vel) > replanAt) {
    // Re-plan only on a real deviation from the plan being tracked. The 0.4 s unconditional re-solve
    // was the landing's failure: tracking was 0–5 m, yet every replan re-derived the final time and
    // re-routed the lateral reference; near 200 m the new problem became infeasible, the law switched
    // to a minimum-miss plan with a different route, the attitude lost the thrust direction (90–112°)
    // and the ship overflew by 550–700 m. A plan that is being followed is kept.
    planT = t
    // Shrinking horizon: re-plan to the SAME end time as the plan being flown. Re-deriving the final
    // time from the current state on every re-plan (the kinematic guess) made the time-to-go grow
    // (measured 20.0 → 24.2 s while tracking within 1–2 m), so the reference kept moving and the
    // plan re-routed until it became infeasible at 130 m. The end time only moves if the fixed-time
    // problem has no solution.
    const tfR = tgoPlan
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
          for (const k of [1, 1.1]) {
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
  const lo = (nE() * inp.thrustMin) / inp.mass, hi = (nE() * inp.thrustMax) / inp.mass
  let u: V3, uDot = v3(0, 0, 0)
  let path: V3[] = []
  let tgo = Infinity, frozen = false, miss = 0
  // time until the pins reach the rails (for the arm-closing lead); Infinity while that is open
  let tRail = Infinity
  if (field) {
    // One law for the whole descent (`fieldCommand`), flown by whichever engine set is running.
    // The burn set (two Raptors, roll by differential gimbal) flies it while the ship is fast: the
    // hull's crossflow moment ∝ V² has to be held by the gimbals, and on ONE off-centre engine that
    // steady pitch/yaw torque is also a steady roll torque, (y·τ_y + z·τ_z)/L — it spun the ship to
    // 31°/s without cold gas (measured). Two engines cannot throttle below ≈ 1.5 g, so a command
    // below their floor is flown at the floor along the commanded direction (the ship just slows
    // sooner). Once slow, the roll coupling is nulled (`inp.quiet`, guidance) and the law hands
    // over to the single throttleable engine for good; its sink is then capped so it stays slow.
    const loF = ((inp.stageTo ?? inp.nEng) * inp.thrustMin) / inp.mass
    const e: FieldEnv = {
      ez: aeroNow.z + dob.d.z, lo: loF, hi, contactSpeed: inp.contactSpeed, hoverKp: inp.hoverKp ?? LAND.hover.kp, hoverKd: inp.hoverKd ?? LAND.hover.kd,
      armed: inp.armed === true, sinkMax: staged ? LAND.stage.sinkMax : undefined,
    }
    const c = fieldCommand(rel, vel, e)
    u = c.u; tRail = c.tRail
    if (!staged) {
      const um = norm(u)
      if (um < lo) u = scale(u, lo / Math.max(um, 1e-6))
      const tilt = Math.acos(clamp(inp.xBody.z, -1, 1)) * 180 / Math.PI
      const slow = norm(vel) < LAND.stage.vMax && rel.z < LAND.stage.hMax
      if (slow && gateT === null) gateT = t // hand-over armed: guidance nulls the roll coupling
      if (!slow) gateT = null
      if (gateT !== null && tilt < LAND.stage.tiltMax && (inp.quiet !== false || t - gateT > LAND.stage.wait)) {
        staged = true; stageT = t
        ev.push(`staging to ${inp.stageTo} engine(s) at ${where}, sink ${(-vel.z).toFixed(0)} m/s, ${Math.hypot(vel.x, vel.y).toFixed(1)} m/s horizontal, tilt ${tilt.toFixed(0)}°${inp.quiet === false ? " (roll coupling not nulled)" : ""} — throttleable final approach`)
      }
    }
    // the prediction (path, time to go) is refreshed at 5 Hz: it is display and arm timing only
    if (t - planT >= 0.2 || !prev.path || prev.path.length === 0) {
      planT = t
      const pr = fieldPredict(rel, vel, inp.uAxis, { ...e, sinkMax: LAND.stage.sinkMax })
      path = pr.path.filter((_, i) => i % 3 === 0 || i === pr.path.length - 1)
      tgo = pr.reached ? pr.t : Infinity
    } else { path = prev.path as V3[]; tgo = prev.tgoField - (t - planT) }
    if (Number.isFinite(c.tRail) && Number.isFinite(tgo)) tRail = Math.min(c.tRail, tgo)
  } else if (plan) {
    const ref = sampleDescent(plan, t)
    tgo = plan.t0 + plan.tf - t
    // only the final plan (to the pins) is ever frozen/terminal; the gate plan hands over by staging
    const finalPlan = plan.p[3 * (plan.K - 1) + 2] < LAND.hover.h + 1
    if (!finalPlan) tgo = Infinity
    // the throttleable set stops at the hover gate first: the rails are a hover-descent later
    tRail = !finalPlan ? Infinity : staged ? Math.max(tgo, 0) + LAND.hover.h / LAND.hover.down : tgo
    frozen = finalPlan && tgo <= freeze
    miss = plan.soft ? plan.termPos : 0
    if (staged) {
      const cx = LAND.finalLat.kp * (ref.p.x - rel.x) + LAND.finalLat.kd * (ref.v.x - vel.x)
      const cy = LAND.finalLat.kp * (ref.p.y - rel.y) + LAND.finalLat.kd * (ref.v.y - vel.y)
      const cm = Math.hypot(cx, cy), sc = cm > LAND.finalLat.aMax ? LAND.finalLat.aMax / cm : 1
      u = v3(ref.u.x + cx * sc, ref.u.y + cy * sc, ref.u.z + kp * (ref.p.z - rel.z) + kd * (ref.v.z - vel.z))
    } else {
      u = v3(
        ref.u.x + kp * (ref.p.x - rel.x) + kd * (ref.v.x - vel.x),
        ref.u.y + kp * (ref.p.y - rel.y) + kd * (ref.v.y - vel.y),
        ref.u.z + kp * (ref.p.z - rel.z) + kd * (ref.v.z - vel.z),
      )
    }
    uDot = ref.uDot
    // past the end of the plan (pins not yet on the rails): hold the contact speed, upright. With a
    // set that can throttle below the weight the position is held too (gently: ≤ ~4° of tilt).
    const tEnd = plan.t0 + plan.tf - t
    if (tEnd <= 0) {
      const hover = lo < 0.95 * G_DESCENT
      if (hover) {
        // Hold-until-centred: one Raptor throttles through the weight, so the ship waits above the
        // rails until it is inside the arms' reach and slow, then goes down at the contact speed.
        // Lateral authority is deliberately small (≤ ≈ 4° of tilt): the body stays upright.
        const centred = Math.abs(rel.x) < LAND.hover.across && Math.abs(rel.y) < LAND.hover.along && Math.hypot(vel.x, vel.y) < LAND.hover.vh
        // committed once centred below the gate: the arms are closing on the hull by then
        const vzT = centred || rel.z < 0.6 * LAND.hover.h ? (rel.z > 3 ? -LAND.hover.down : -inp.contactSpeed) : rel.z > LAND.hover.h ? -0.6 : 0
        tRail = centred || rel.z < 0.6 * LAND.hover.h ? Math.max(rel.z, 0) / LAND.hover.down : Infinity
        const H = LAND.hover
        const hkp = inp.hoverKp ?? H.kp, hkd = inp.hoverKd ?? H.kd
        u = v3(clamp(-hkp * rel.x - hkd * vel.x, -H.aMax, H.aMax), clamp(-hkp * rel.y - hkd * vel.y, -H.aMax, H.aMax), G_DESCENT + 2 * (vzT - vel.z) - aeroNow.z - dob.d.z)
      } else u = v3(-kd * vel.x, -kd * vel.y, G_DESCENT + 2 * (-inp.contactSpeed - vel.z) - aeroNow.z - dob.d.z)
    }
    u = fitThrust(u, lo, hi, Math.tan(((staged ? FINAL_LEVELS[level].tilt : LAND_LEVELS[level].tilt) + 5) * Math.PI / 180))
    const pl = plan
    for (let k = 0; k < pl.K; k++) if (pl.t0 + k * pl.h >= t - pl.h) path.push(v3(pl.p[3 * k], pl.p[3 * k + 1], Math.max(pl.p[3 * k + 2], 0)))
  } else {
    // no plan at all: kill the horizontal speed and brake at the deceleration the height requires
    const need = (vel.z * vel.z - inp.contactSpeed * inp.contactSpeed) / (2 * Math.max(rel.z, 1))
    u = fitThrust(v3(-0.5 * vel.x, -0.5 * vel.y, G_DESCENT + clamp(need, 0, hi) - aeroNow.z - dob.d.z), lo, hi, Math.tan(30 * Math.PI / 180))
    path = [v3(rel.x, rel.y, Math.max(rel.z, 0))]
  }
  return { u, uDot, state: { plan, planT, dob, level, nEng: nE(), fails, backoffT, staged, stageT, gateT, path: field ? path : [], tgoField: field ? tgo + (t - planT) : Infinity }, events: ev, path, tgo, tRail, frozen, miss, residual: dob.d }
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
