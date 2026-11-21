/**
 * Shared types for the 3-DOF controller bake-off (`scripts/mpc-bench.ts`).
 *
 * The bench is deliberately small: a point-mass pin-station state in pad ENU, a thrust
 * direction of its own (the first-order / torque-limited pointing state) and a mass that
 * follows the real Raptor flow. Controllers are pure `command(state, t)` functions; the
 * plant is the only thing that integrates. Nothing here reads plant truth in a controller.
 */
import type { V3 } from "@ship/math"

/** Full observable state handed to a controller. `a` is the realised *thrust* acceleration
 *  (gravity excluded), the same quantity the flight planners restart from. */
export interface BenchState {
  readonly t: number
  /** pin station relative to the catch point (pad ENU, m) */
  readonly p: V3
  /** ground-relative velocity (pad ENU, m/s) */
  readonly v: V3
  /** realised thrust axis (pad ENU unit vector) */
  readonly d: V3
  /** realised thrust acceleration (pad ENU, m/s²), gravity excluded */
  readonly a: V3
  readonly mass: number
  /** achieved throttle in [minThrottle, 1] */
  readonly throttle: number
  /** propellant consumed since ignition (kg) */
  readonly fuel: number
  /** torque-mode gimbal deflection (rad), 0 in rate mode */
  readonly gimbal: number
  /** direction closing rate (rad/s) — the pointing state's rate */
  readonly omega: number
}

/** A controller may command a thrust-acceleration vector (m/s², gravity excluded) or a
 *  direction + throttle pair. The bake-off strategies use `u`; model checks use `dir`. */
export interface AccelCommand {
  readonly u: V3
}
export interface DirThrottleCommand {
  readonly dir: V3
  readonly throttle: number
}
export type Command = AccelCommand | DirThrottleCommand

export type PlantMode = "rate" | "torque"

/** Plant configuration. `rate` mode caps the thrust-direction slew; `torque` mode integrates
 *  a rigid transverse axis driven by the gimbal moment `n·T·arm·sin δ` over `Iyy`. */
export interface PlantConfig {
  readonly nEng: number
  readonly mode: PlantMode
  /** rate-mode direction slew cap (rad/s) */
  readonly rateCap: number
  /** first-order direction time constant (s), rate mode */
  readonly tauDir: number
  /** first-order throttle response (s) */
  readonly tauThrottle: number
  /** torque-mode gimbal PD gains (rad per rad, rad per rad/s) */
  readonly gimbalKp: number
  readonly gimbalKd: number
  /** surface gravity (m/s²) */
  readonly gravity: number
  /** integration step (s) */
  readonly dt: number
  /** constant acceleration disturbance (m/s²), e.g. the wind-equivalent crosswind */
  readonly disturbance: V3
}

/** Diagnostics of one plant step. */
export interface AppliedStep {
  readonly dirCmd: V3
  readonly throttleCmd: number
  readonly throttle: number
  readonly thrust: number
  readonly mdot: number
  readonly dirErrRad: number
  readonly alpha: number
  readonly accel: V3
}

export interface ControllerStats {
  replans: number
  /** wall time inside `command` (includes solver time) */
  commandMs: number
  /** replans whose planner reported an infeasible terminal */
  infeasible: number
  lastFeasible: boolean
  lastTgo: number
}

/** The only interface a strategy exposes to the harness. */
export interface Controller {
  readonly id: string
  readonly label: string
  readonly command: (state: BenchState, t: number) => Command
  readonly stats: ControllerStats
}
