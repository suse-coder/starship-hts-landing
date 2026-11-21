/**
 * Joint 3-D powered-descent guidance (lossless-convex PDG) for the catch.
 *
 * The two-phase landing law splits the vertical profile (analytic) from the lateral MPC and stops
 * lateral planning at `zProf`. That split cannot represent the engine floor: below `u_z = Γ_min` the
 * lateral acceleration is forced, while the split plan believes it is nearly zero. It also plans a
 * free lateral vector, so it asks for ~170° thrust reversals that the torque-limited body needs ~5 s
 * to fly (review/02 L1–L6).
 *
 * Here one convex problem covers the whole burn: |u| ≤ Γ (Γ ∈ [Γ_min, Γ_max], the lossless slack),
 * u_z ≥ cosθ|u|, linear translation with gravity, and the catch as a terminal position/velocity
 * equality (x = 0, along-arm entry at +y, descent at the contact speed). A line search over the
 * final time picks the plan with the smallest terminal error; it is re-solved at 5 Hz from the
 * realised state, so the plan always remains a feasible path to the pins. The first control is
 * executed by the ordinary attitude loop.
 *
 * This is still a point-mass plan: the 6-DOF plant and the attitude lag are the ground truth. The
 * plan is restarted from the realised thrust acceleration and the direction-rate penalty keeps it
 * trackable; it never certifies a catch by itself.
 */
import { type V3, v3, clamp } from "@ship/math"
import { solveLandingMpc, type MpcPlan } from "./mpc"
import { G_LAND } from "./landing"

/** pin-station state in the pad ENU frame, relative to the catch point */
export interface PdgState {
  readonly p: V3
  readonly v: V3
  /** realised thrust acceleration now (ENU, gravity excluded), m/s² */
  readonly a: V3
  readonly mass: number
  readonly nEng: number
  readonly thrustMaxN: number
  readonly thrustMinN: number
}

export interface PdgOptions {
  readonly vF?: number
  /** along-arm (north) velocity at the pins; the ship enters from the open south side */
  readonly vAlong?: number
  readonly tiltDeg?: number
  /** terminal gate height above the rails (0 = plan all the way to the pins) */
  readonly gateAlt?: number
  /** sink rate at the gate (m/s, positive down) */
  readonly gateVz?: number
  readonly candidates?: number
  readonly steps?: number
  readonly iters?: number
  readonly wMag?: number
  readonly rd?: number
  readonly Qp?: number
  readonly Qv?: number
}

export interface PdgPlan {
  readonly u: V3
  readonly tgo: number
  readonly plan: MpcPlan
  readonly feasible: boolean
  /** terminal error of the chosen plan (m + m/s) */
  readonly terminalError: number
}

const DEFAULT_PDG: Required<Pick<PdgOptions, "vF" | "vAlong" | "tiltDeg" | "gateAlt" | "gateVz" | "candidates" | "steps" | "iters" | "wMag" | "rd" | "Qp" | "Qv">> = {
  vF: 0.7,
  vAlong: 0.4,
  tiltDeg: 48,
  gateAlt: 0,
  gateVz: 6,
  candidates: 9,
  steps: 28,
  iters: 45,
  wMag: 0.02,
  rd: 0.5,
  Qp: 400,
  Qv: 800,
}

/** a warm start per time-to-go candidate, so repeated replans converge in a few ms */
export type PdgWarm = readonly (Float64Array | null)[]

const terminalCost = (p: readonly [number, number, number], v: readonly [number, number, number], vF: readonly [number, number, number]): number =>
  Math.hypot(p[0], p[1], p[2]) +
  1.5 * Math.hypot(v[0] - vF[0], v[1] - vF[1], v[2] - vF[2]) +
  10 * Math.max(0, -p[2])

/**
 * Plan one burn from `st`. The candidate times bracket the natural hoverslam time
 * `(s − vF)/(Γ_min − g)`; the terminal cost picks the one that actually reaches the pins.
 */
export const planPdg = (st: PdgState, opt: PdgOptions = {}, warm: PdgWarm | null = null): { readonly best: PdgPlan; readonly warm: PdgWarm } => {
  const o = { ...DEFAULT_PDG, ...opt }
  const g = G_LAND
  const aMin = (st.nEng * st.thrustMinN) / st.mass
  const aMax = (st.nEng * st.thrustMaxN) / st.mass
  const s = Math.max(-st.v.z, 0.1)
  // The time-to-go must bracket the *kinematic* time to cover the remaining height at the mean of
  // the current and target sink: t ≈ 2·dz/(s + vTarget). The old nominal used the minimum-thrust
  // time (≈ (s − vF)/(aMin − g)), which is far longer than the feasible kinematic time and made the
  // search plan a late, climbing trajectory (the gate was never reached).
  const dz = Math.max(st.p.z - o.gateAlt, 1)
  const vTgt = o.gateAlt > 0 ? o.gateVz : o.vF
  const tFloor = (s - vTgt) / Math.max(aMin - g, 0.4) + dz / Math.max(s, 1)
  const tKin = (2 * dz) / Math.max(s + vTgt, 0.6)
  const tNom = clamp(o.gateAlt > 0 ? tKin : 0.5 * (tKin + tFloor), 3.0, 60)
  const tgoMin = Math.max(2.2, 0.5 * tNom)
  const tgoMax = Math.min(70, 1.6 * tNom)
  const nCand = Math.max(3, o.candidates)
  const n = Math.max(12, o.steps)
  const vF = v3(0, o.vAlong, -o.vF)
  const cosTilt = Math.cos((o.tiltDeg * Math.PI) / 180)
  const warmNext: (Float64Array | null)[] = []
  const plans: PdgPlan[] = []
  for (let c = 0; c < nCand; c++) {
    const frac = nCand === 1 ? 0.5 : c / (nCand - 1)
    const tgo = tgoMin + (tgoMax - tgoMin) * frac
    const h = tgo / n
    const plan = solveLandingMpc(
      {
        p0: [st.p.x, st.p.y, st.p.z],
        v0: [st.v.x, st.v.y, st.v.z],
        a0: [st.a.x, st.a.y, st.a.z],
        tau: 0.25,
        n,
        h,
        vRef: o.gateAlt > 0 ? [0, 0, -o.gateVz] : [0, 0, -o.vF],
        vF: o.gateAlt > 0 ? [0, 0, -o.gateVz] : [vF.x, vF.y, vF.z],
        pRefH: [0, 0],
        pRefZ: o.gateAlt > 0 ? o.gateAlt : undefined,
        vRefH: [0, o.vAlong],
        gravity: g,
        aZMin: 0,
        aZMax: 0,
        tanTilt: 1,
        // lossless convexification: Γ is free in [aMin, aMax] via the fuel cost; the cone caps the
        // direction, and the engine floor is the Γ lower bound, not a non-convex |u| constraint
        cone: { cosTilt, aMin, aMax: 0.98 * aMax, hardMin: true },
        weak: null,
        aWeak: Infinity,
        uPrev: [st.a.x, st.a.y, st.a.z],
        glide: 0,
        weights: {
          qp: 0.0004,
          qv: 0.02,
          wvXY: 0.6,
          ru: 0.02,
          rd: o.rd,
          Qp: o.Qp,
          Qv: o.Qv,
          wMag: o.wMag,
          wDir: 3,
        },
      },
      warm && warm[c] ? warm[c] : null,
      o.iters,
    )
    warmNext[c] = plan.u
    const err = terminalCost(plan.pN, plan.vN, o.gateAlt > 0 ? [0, 0, -o.gateVz] : [vF.x, vF.y, vF.z])
    plans.push({
      u: v3(plan.u[0], plan.u[1], plan.u[2]),
      tgo,
      plan,
      feasible: Math.hypot(plan.pN[0], plan.pN[1], plan.pN[2] - o.gateAlt) < 1.5 && Math.hypot(plan.vN[0] - vF.x, plan.vN[1] - vF.y, plan.vN[2] + (o.gateAlt > 0 ? o.gateVz : o.vF)) < 1.5,
      terminalError: err,
    })
  }
  const best = plans.reduce((a, b) => (b.terminalError < a.terminalError ? b : a))
  return { best, warm: warmNext }
}
