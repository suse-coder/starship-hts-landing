/**
 * The fast 3-DOF pin-station plant for the bake-off.
 *
 * It uses the real `@ship/craft` Ship-42 engine data: the sea-level Raptor's `thrustSL`, `isp0`,
 * `exitArea`, `minThrottle` and `gimbalMax`, and the mass/`Iyy`/CG from `massProps`. Thrust is
 * `n·(throttle·F_vac,100 − p_amb·A_e)` (the same pressure correction as `engineThrust`), mass flow
 * is `n·throttle·F_vac,100/(Isp_vac·g0)`, and the 40 % floor is enforced at the command boundary.
 *
 * Pointing has two modes:
 *  - `rate`: first-order direction tracking with a hard rad/s cap (a stand-in for the attitude loop).
 *  - `torque`: a transverse rigid axis driven by the gimbal moment `n·T·arm·sin δ` over the real
 *    `Iyy`. A PD gimbal command and the 20 °/s gimbal slew produce the direction. This is the more
 *    honest comparison because it ties the direction response to the actual engine torque and inertia.
 *
 * No aero. The optional disturbance is a constant acceleration in pad ENU (used as the wind-equivalent
 * crosswind case; without an aero model a velocity wind has no dynamic effect).
 */
import { type V3, add, anyPerp, clamp, cross, dot, norm, normalize, scale, v3 } from "@ship/math"
import { atmosphere, G0 } from "@ship/earth"
import {
  type Engine,
  type MassProps,
  type VehicleConfig,
  fVac100,
  initialConsumables,
  ispVac,
  makeVehicle,
  massProps,
} from "@ship/craft"
import type { AppliedStep, BenchState, Command, PlantConfig } from "./types"

export const P_SL = 101325

/** The ship model the bench uses (the same default the sim builds). */
export const BENCH_VEHICLE: VehicleConfig = makeVehicle()

/** Landing-burn consumables: full header tanks, main tanks empty. */
export const BENCH_LANDING_CONSUMABLES = {
  ...initialConsumables(BENCH_VEHICLE),
  loxMain: 0,
  ch4Main: 0,
}

export const BENCH_MASS_PROPS: MassProps = massProps(BENCH_VEHICLE, BENCH_LANDING_CONSUMABLES)
export const BENCH_MASS = BENCH_MASS_PROPS.m

/** The three sea-level Raptors; the bench lights `nEng` of them. */
export const SL_ENGINES: readonly Engine[] = BENCH_VEHICLE.engines.filter((e) => e.kind === "SL")
export const BENCH_ENGINE: Engine = SL_ENGINES[0]

export interface EngineModel {
  readonly engine: Engine
  readonly fVac100: number
  readonly ispVac: number
  readonly thrustSL: number
  readonly exitArea: number
  readonly minThrottle: number
  readonly gimbalMax: number
  readonly gimbalRate: number
  /** axial distance from the CG to the gimbal point (m), the moment arm of a gimbaled thrust */
  readonly arm: number
  /** transverse inertia about the CG (kg m²) */
  readonly Iyy: number
}

export const ENGINE_MODEL: EngineModel = {
  engine: BENCH_ENGINE,
  fVac100: fVac100(BENCH_ENGINE),
  ispVac: ispVac(BENCH_ENGINE),
  thrustSL: BENCH_ENGINE.thrustSL,
  exitArea: BENCH_ENGINE.exitArea,
  minThrottle: BENCH_ENGINE.minThrottle,
  gimbalMax: BENCH_ENGINE.gimbalMax,
  gimbalRate: BENCH_ENGINE.gimbalRate,
  arm: Math.abs(BENCH_MASS_PROPS.xcg - BENCH_ENGINE.gimbal.x),
  Iyy: BENCH_MASS_PROPS.I.y,
}

/** Ambient pressure at a pin-station altitude (pad-relative, m). */
export const pressureAt = (z: number): number => atmosphere(Math.max(z, 0)).p

/** Per-engine thrust at a throttle and ambient pressure (N), the `engineThrust` law. */
export const thrustPerEngine = (throttle: number, pAmb: number, model: EngineModel = ENGINE_MODEL): number =>
  Math.max(0, throttle * model.fVac100 - pAmb * model.exitArea)

export const thrustTotal = (nEng: number, throttle: number, pAmb: number, model: EngineModel = ENGINE_MODEL): number =>
  nEng * thrustPerEngine(throttle, pAmb, model)

/** Per-engine propellant flow (kg/s) — independent of ambient pressure, as in `engineThrust`. */
export const massFlowPerEngine = (throttle: number, model: EngineModel = ENGINE_MODEL): number =>
  (throttle * model.fVac100) / (model.ispVac * G0)

export const massFlowTotal = (nEng: number, throttle: number, model: EngineModel = ENGINE_MODEL): number =>
  nEng * massFlowPerEngine(throttle, model)

/** Thrust acceleration along the thrust axis (m/s²), gravity excluded. */
export const thrustAcceleration = (nEng: number, throttle: number, mass: number, pAmb: number, model: EngineModel = ENGINE_MODEL): number =>
  thrustTotal(nEng, throttle, pAmb, model) / mass

/** Net vertical acceleration (m/s², +up) at a steady throttle. */
export const netVerticalAcceleration = (nEng: number, throttle: number, mass: number, pAmb: number, gravity: number, model: EngineModel = ENGINE_MODEL): number =>
  thrustAcceleration(nEng, throttle, mass, pAmb, model) - gravity

/** Gimbal moment magnitude (N·m) for a deflection, about a transverse axis. */
export const gimbalMoment = (nEng: number, thrustPerEngineN: number, angleRad: number, arm: number = ENGINE_MODEL.arm): number =>
  nEng * thrustPerEngineN * arm * Math.sin(angleRad)

/** Rigid-axis angular acceleration from a gimbal deflection (rad/s²). */
export const gimbalAngularAccel = (nEng: number, thrustPerEngineN: number, angleRad: number, Iyy: number = ENGINE_MODEL.Iyy, arm: number = ENGINE_MODEL.arm): number =>
  gimbalMoment(nEng, thrustPerEngineN, angleRad, arm) / Iyy

/** Rotate unit vector `d` toward unit vector `t` by `angle` radians (shortest arc). */
const rotateToward = (d: V3, t: V3, angle: number): V3 => {
  if (angle <= 1e-12) return d
  const c = clamp(dot(d, t), -1, 1)
  const total = Math.acos(c)
  if (total <= 1e-12) return t
  if (angle >= total) return t
  let axis = cross(d, t)
  const an = norm(axis)
  axis = an > 1e-9 ? scale(axis, 1 / an) : anyPerp(d)
  const ca = Math.cos(angle)
  const sa = Math.sin(angle)
  // Rodrigues: d cosθ + (axis × d) sinθ
  return normalize(add(scale(d, ca), scale(cross(axis, d), sa)))
}

const angleBetween = (a: V3, b: V3): number => Math.acos(clamp(dot(a, b), -1, 1))

/** Resolve a command into a desired direction and throttle. A `u` command is interpreted as a
 *  thrust acceleration: F = m·|u|, throttle = (F/n + p_amb·A_e)/F_vac,100. */
export const commandToDirThrottle = (
  cmd: Command,
  mass: number,
  pAmb: number,
  nEng: number,
  model: EngineModel = ENGINE_MODEL,
): { readonly dirCmd: V3; readonly throttleCmd: number } => {
  if ("u" in cmd) {
    const mag = norm(cmd.u)
    if (mag < 1e-9) return { dirCmd: v3(0, 0, 1), throttleCmd: model.minThrottle }
    const throttleCmd = clamp((mass * mag) / nEng / model.fVac100 + (pAmb * model.exitArea) / model.fVac100, model.minThrottle, 1)
    return { dirCmd: scale(cmd.u, 1 / mag), throttleCmd }
  }
  return {
    dirCmd: normalize(cmd.dir),
    throttleCmd: clamp(cmd.throttle, model.minThrottle, 1),
  }
}

/** Initial state: engines already lit at the throttle floor, thrust axis vertical. */
export const initialPlantState = (
  p: V3,
  v: V3,
  mass: number,
  nEng: number,
  model: EngineModel = ENGINE_MODEL,
): BenchState => {
  const pAmb = pressureAt(p.z)
  const aThrust = thrustAcceleration(nEng, model.minThrottle, mass, pAmb, model)
  return {
    t: 0,
    p,
    v,
    d: v3(0, 0, 1),
    a: v3(0, 0, aThrust),
    mass,
    throttle: model.minThrottle,
    fuel: 0,
    gimbal: 0,
    omega: 0,
  }
}

/**
 * Integrate one plant step. The command is resolved to direction + throttle, throttle and pointing
 * respond with their first-order limits, then the point mass integrates with the pressure-corrected
 * thrust and the real mass flow.
 */
export const stepPlant = (
  s: BenchState,
  cmd: Command,
  cfg: PlantConfig,
  model: EngineModel = ENGINE_MODEL,
): { readonly next: BenchState; readonly applied: AppliedStep } => {
  const dt = cfg.dt
  const pAmb = pressureAt(s.p.z)
  const { dirCmd, throttleCmd } = commandToDirThrottle(cmd, s.mass, pAmb, cfg.nEng, model)

  const bThr = 1 - Math.exp(-dt / Math.max(cfg.tauThrottle, 1e-6))
  const throttle = clamp(s.throttle + (throttleCmd - s.throttle) * bThr, model.minThrottle, 1)

  let d = s.d
  let omega = s.omega
  let gimbal = s.gimbal
  let alpha = 0
  const theta = angleBetween(s.d, dirCmd)
  const thrustNow = thrustTotal(cfg.nEng, throttle, pAmb, model)

  if (cfg.mode === "rate") {
    const step = Math.min(theta * (1 - Math.exp(-dt / Math.max(cfg.tauDir, 1e-6))), Math.max(cfg.rateCap, 0) * dt)
    d = rotateToward(s.d, dirCmd, step)
    omega = step / Math.max(dt, 1e-9)
    gimbal = 0
  } else {
    const gMax = model.gimbalMax
    const deltaCmd = clamp(cfg.gimbalKp * theta - cfg.gimbalKd * s.omega, -gMax, gMax)
    gimbal = s.gimbal + clamp(deltaCmd - s.gimbal, -model.gimbalRate * dt, model.gimbalRate * dt)
    alpha = (thrustNow * model.arm * Math.sin(gimbal)) / model.Iyy
    const dTheta = s.omega * dt + 0.5 * alpha * dt * dt
    const step = clamp(dTheta, -theta, theta)
    d = rotateToward(s.d, dirCmd, step)
    // once the axis catches the command there is no residual closing rate to carry through it
    omega = Math.abs(step) >= theta - 1e-9 ? 0 : s.omega + alpha * dt
  }

  const thrust = thrustTotal(cfg.nEng, throttle, pAmb, model)
  const mdot = massFlowTotal(cfg.nEng, throttle, model)
  const aThrust = thrust / s.mass
  const accel = v3(
    aThrust * d.x + cfg.disturbance.x,
    aThrust * d.y + cfg.disturbance.y,
    aThrust * d.z - cfg.gravity + cfg.disturbance.z,
  )
  const mass = Math.max(0.4 * s.mass, s.mass - mdot * dt)
  const v = v3(s.v.x + accel.x * dt, s.v.y + accel.y * dt, s.v.z + accel.z * dt)
  const p = v3(
    s.p.x + s.v.x * dt + 0.5 * accel.x * dt * dt,
    s.p.y + s.v.y * dt + 0.5 * accel.y * dt * dt,
    s.p.z + s.v.z * dt + 0.5 * accel.z * dt * dt,
  )
  const next: BenchState = {
    t: s.t + dt,
    p,
    v,
    d,
    a: scale(d, aThrust),
    mass,
    throttle,
    fuel: s.fuel + mdot * dt,
    gimbal,
    omega,
  }
  const applied: AppliedStep = {
    dirCmd,
    throttleCmd,
    throttle,
    thrust,
    mdot,
    dirErrRad: angleBetween(d, dirCmd),
    alpha,
    accel,
  }
  return { next, applied }
}

/** Useful for metrics and tests: the vertical component of a direction. */
export const vertical = (d: V3): number => clamp(d.z, -1, 1)

/** The direction error metric used by the harness. */
export const directionErrorDeg = (a: V3, b: V3): number => (angleBetween(a, b) * 180) / Math.PI
