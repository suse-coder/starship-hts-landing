/**
 * The six strategies in the bake-off. Each is a pure `command(state, t)` closure over the shared
 * reference path and the real engine limits; none reads hidden plant state.
 *
 *   open-loop   — the reference's feedforward acceleration only (no feedback).
 *   pd-ff       — PD on position/velocity plus the reference's acceleration feedforward. This is the
 *                 "perfect path" lower bound: it is allowed to know the whole reference path.
 *   two-phase   — the flight's `planLanding({ mode: "twoPhase" })`, re-planned at 10 Hz.
 *   pdg         — the joint `planPdg` (lossless-convex powered descent), re-planned at 5 Hz.
 *   zem-zev     — ZEM/ZEV lateral guidance over the analytic vertical hoverslam profile.
 *   bang-bang   — a one-switch double-integrator lateral law over the same vertical profile.
 */
import { type V3, clamp, sub, v3 } from "@ship/math"
import {
  G_LAND,
  type LandingLimits,
  assemble,
  hoverAccel,
  lateralBudget,
  magBand,
  netAccel,
  planLanding,
  shapeAccel,
} from "../landing"
import { planPdg, type PdgWarm } from "../pdg"
import type { ReferencePath } from "./reference"
import { pressureAt, thrustPerEngine, ENGINE_MODEL } from "./plant"
import type { BenchState, Command, Controller, ControllerStats } from "./types"

export interface StrategyDeps {
  readonly reference: ReferencePath
  readonly gate: V3
  readonly gateV: V3
  readonly nEng: number
  /** hard thrust-corridor half-angle (rad), shared by every strategy */
  readonly tiltRad: number
  readonly vF: number
}

export const STRATEGIES = [
  { id: "open-loop", label: "Open-loop feedforward" },
  { id: "pd-ff", label: "PD + accel feedforward" },
  { id: "two-phase", label: "planLanding two-phase @10 Hz" },
  { id: "pdg", label: "joint planPdg @5 Hz" },
  { id: "zem-zev", label: "ZEM/ZEV lateral + profile" },
  { id: "bang-bang", label: "one-switch bang-bang" },
] as const

export type StrategyKind = (typeof STRATEGIES)[number]["id"]

const stats = (): ControllerStats => ({
  replans: 0,
  commandMs: 0,
  infeasible: 0,
  lastFeasible: true,
  lastTgo: 0,
})

/** Engine limits at the current altitude, shared by the model-based strategies. */
export const limitsAt = (s: BenchState, deps: StrategyDeps): LandingLimits => {
  const pAmb = pressureAt(s.p.z)
  return {
    mass: s.mass,
    nEng: deps.nEng,
    thrustMax: thrustPerEngine(1, pAmb),
    thrustMin: thrustPerEngine(ENGINE_MODEL.minThrottle, pAmb),
    tanTilt: Math.tan(deps.tiltRad),
  }
}

const openLoop = (deps: StrategyDeps): Controller => {
  const st = stats()
  return {
    id: "open-loop",
    label: "Open-loop feedforward",
    stats: st,
    command: (_s, t): Command => {
      const r = deps.reference.at(t)
      return { u: v3(r.a.x, r.a.y, r.a.z + G_LAND) }
    },
  }
}

const pdFeedforward = (deps: StrategyDeps): Controller => {
  const st = stats()
  // critically damped double-integrator loop; slow enough for the 0.3–1 s pointing lag
  const kp = 1.0
  const kd = 2.0
  return {
    id: "pd-ff",
    label: "PD + accel feedforward",
    stats: st,
    command: (s, t): Command => {
      const r = deps.reference.at(t)
      const ex = r.p.x - s.p.x
      const ey = r.p.y - s.p.y
      const ez = r.p.z - s.p.z
      const evx = r.v.x - s.v.x
      const evy = r.v.y - s.v.y
      const evz = r.v.z - s.v.z
      return {
        u: v3(
          r.a.x + kp * ex + kd * evx,
          r.a.y + kp * ey + kd * evy,
          r.a.z + G_LAND + kp * ez + kd * evz,
        ),
      }
    },
  }
}

const twoPhase = (deps: StrategyDeps): Controller => {
  const st = stats()
  const dtReplan = 0.1
  let last = -Infinity
  let u: V3 | null = null
  let prevU: V3 | null = null
  let warm: Float64Array | null = null
  let aCruise = 0
  let aBrake = 0
  let zFinal = 0
  return {
    id: "two-phase",
    label: "planLanding two-phase @10 Hz",
    stats: st,
    command: (s, t): Command => {
      if (u === null || t - last >= dtReplan - 1e-9) {
        const t0 = performance.now()
        const lim = limitsAt(s, deps)
        const mag = magBand(lim)
        if (aCruise <= 0) {
          // hold the flight's cruise/phase break at ignition (the flight re-uses them between plans)
          const sink = Math.max(-s.v.z, 0.7)
          aBrake = Math.max(mag.min - G_LAND, 0.6)
          aCruise = clamp((sink * sink - deps.vF * deps.vF) / (2 * Math.max(s.p.z, 1)), 0.6, 8)
          zFinal = clamp(
            (sink * sink - deps.vF * deps.vF - 2 * aCruise * s.p.z) / (2 * Math.max(aBrake - aCruise, 0.2)),
            30,
            400,
          )
        }
        // The flight's measured vertical trim (guidance.ts): the body lags the commanded tilt, so a
        // plan that ignores the lag over-thrusts vertically, over-decelerates and stalls above the
        // pins. Here the trim is the direct measured form: desired vertical thrust ÷ realised
        // vertical thrust, clamped like the flight's `uzTrim`. PlanLanding accepts it explicitly.
        const aProf = s.p.z > zFinal ? aCruise : aBrake
        const vT = -Math.sqrt(
          deps.vF * deps.vF +
          2 * aBrake * Math.min(Math.max(s.p.z, 0), zFinal) +
          2 * aCruise * Math.max(s.p.z - zFinal, 0),
        )
        const aDes = aProf + G_LAND + 0.9 * (vT - s.v.z)
        const uzTrim = prevU ? clamp(aDes / Math.max(s.a.z, 2), 0.4, 1.2) : 1
        const sol = planLanding(
          {
            p: [s.p.x, s.p.y, s.p.z],
            v: [s.v.x, s.v.y, s.v.z],
            a: [s.a.x, s.a.y, s.a.z],
            d: [s.d.x, s.d.y, s.d.z],
          },
          lim,
          {
            vF: deps.vF,
            tau: s.p.z > 120 ? 1.4 : 0.7,
            glide: 0.04,
            terminalAlt: 3,
            aCruise,
            zFinal,
            steps: 25,
            wMag: 0.05,
            omegaMax: 0.35,
            mode: "twoPhase",
            flipping: false,
            uzTrim,
            warm,
          },
        )
        prevU = u
        u = v3(sol.u[0], sol.u[1], sol.u[2])
        warm = sol.plan?.u ?? null
        st.replans += 1
        st.lastFeasible = sol.feasible
        st.lastTgo = sol.tgo
        if (!sol.feasible) st.infeasible += 1
        st.commandMs += performance.now() - t0
        last = t
      }
      return { u: u ?? v3(0, 0, G_LAND) }
    },
  }
}

const pdg = (deps: StrategyDeps): Controller => {
  const st = stats()
  const dtReplan = 0.2
  let last = -Infinity
  let u: V3 | null = null
  let warm: PdgWarm | null = null
  let lastFeasible = true
  return {
    id: "pdg",
    label: "joint planPdg @5 Hz",
    stats: st,
    command: (s, t): Command => {
      if (u === null || t - last >= dtReplan - 1e-9) {
        const t0 = performance.now()
        const pAmb = pressureAt(s.p.z)
        const out = planPdg(
          {
            // the planner targets the origin; shift the world state so the origin is the gate
            p: sub(s.p, deps.gate),
            v: s.v,
            a: s.a,
            mass: s.mass,
            nEng: deps.nEng,
            thrustMaxN: thrustPerEngine(1, pAmb),
            thrustMinN: thrustPerEngine(ENGINE_MODEL.minThrottle, pAmb),
          },
          {
            vF: deps.vF,
            vAlong: deps.gateV.y,
            tiltDeg: (deps.tiltRad * 180) / Math.PI,
            gateAlt: 0,
            candidates: 9,
            steps: 28,
            iters: 45,
          },
          warm,
        )
        u = out.best.u
        warm = out.warm
        lastFeasible = out.best.feasible
        st.replans += 1
        st.lastFeasible = lastFeasible
        st.lastTgo = out.best.tgo
        if (!lastFeasible) st.infeasible += 1
        st.commandMs += performance.now() - t0
        last = t
      }
      return { u: u ?? v3(0, 0, G_LAND) }
    },
  }
}

/** Shared vertical-profile + shaping for the two guidance-law strategies. */
const verticalProfile = (s: BenchState, deps: StrategyDeps): { readonly lim: LandingLimits; readonly mag: ReturnType<typeof magBand>; readonly uz: number } => {
  const lim = limitsAt(s, deps)
  const mag = magBand(lim)
  const b = netAccel(lim, G_LAND)
  const z = Math.max(s.p.z, 0.2)
  const aNet = hoverAccel(z, s.v.z, b, deps.vF)
  const vTarget = -Math.sqrt(deps.vF * deps.vF + 2 * aNet * z)
  const uz = clamp(aNet + G_LAND + 0.9 * (vTarget - s.v.z), 0.6, mag.max)
  return { lim, mag, uz }
}

const zemZev = (deps: StrategyDeps): Controller => {
  const st = stats()
  return {
    id: "zem-zev",
    label: "ZEM/ZEV lateral + profile",
    stats: st,
    command: (s): Command => {
      const { lim, mag, uz } = verticalProfile(s, deps)
      const z = Math.max(s.p.z, 0.2)
      const sink = Math.max(-s.v.z, 0)
      const tgo = clamp((2 * z) / Math.max(sink + deps.vF, 0.5), 2.5, 60)
      const t2 = tgo * tgo
      // ZEM/ZEV: a = 6·(p_f − p − v·tgo)/tgo² − 2·(v_f − v)/tgo
      const ax = (6 * (deps.gate.x - s.p.x - s.v.x * tgo)) / t2 - (2 * (deps.gateV.x - s.v.x)) / tgo
      const ay = (6 * (deps.gate.y - s.p.y - s.v.y * tgo)) / t2 - (2 * (deps.gateV.y - s.v.y)) / tgo
      const [ux, uy, uzz] = assemble([ax, ay], uz, lim, mag, [s.p.x - deps.gate.x, s.p.y - deps.gate.y])
      return { u: v3(ux, uy, uzz) }
    },
  }
}

const bangBang = (deps: StrategyDeps): Controller => {
  const st = stats()
  const axis = (e: number, relV: number, aMax: number): number => {
    // one-switch double-integrator law: accelerate toward the target unless the stopping distance
    // at the current closing speed already exceeds the remaining error
    if (Math.abs(e) < 0.4 && Math.abs(relV) < 0.4) return 0
    if (e >= 0) return relV > Math.sqrt(2 * aMax * e) ? -aMax : aMax
    return relV < -Math.sqrt(2 * aMax * -e) ? aMax : -aMax
  }
  return {
    id: "bang-bang",
    label: "one-switch bang-bang",
    stats: st,
    command: (s): Command => {
      const { lim, mag, uz } = verticalProfile(s, deps)
      const aLat = 0.9 * lateralBudget(uz, lim, mag)
      const ax = axis(deps.gate.x - s.p.x, s.v.x - deps.gateV.x, aLat)
      const ay = axis(deps.gate.y - s.p.y, s.v.y - deps.gateV.y, aLat)
      const [ux, uy, uzz] = assemble([ax, ay], uz, lim, mag, [s.p.x - deps.gate.x, s.p.y - deps.gate.y])
      return { u: v3(ux, uy, uzz) }
    },
  }
}

const FACTORIES: Readonly<Record<StrategyKind, (deps: StrategyDeps) => Controller>> = {
  "open-loop": openLoop,
  "pd-ff": pdFeedforward,
  "two-phase": twoPhase,
  pdg,
  "zem-zev": zemZev,
  "bang-bang": bangBang,
}

export const makeController = (kind: StrategyKind, deps: StrategyDeps): Controller => FACTORIES[kind](deps)

/** Shape a `u` command into the shared engine band and thrust corridor. Direct dir/throttle
 *  commands (model checks) bypass the shaper. */
export const shapeCommand = (s: BenchState, cmd: Command, deps: StrategyDeps): Command => {
  if (!("u" in cmd)) return cmd
  const lim = limitsAt(s, deps)
  const [x, y, z] = shapeAccel([cmd.u.x, cmd.u.y, cmd.u.z], lim)
  return { u: v3(x, y, z) }
}
