/**
 * Powered-descent guidance for the catch: trajectory generation + tracking.
 *
 * Architecture (the standard split for precision landing):
 *
 *  1. **Trajectory generation** (`planDescent`, 2–3 Hz): a fuel-optimal, free-final-time powered
 *     descent from the current state to a terminal gate, solved as a convex program by lossless
 *     convexification (Açıkmeşe & Ploen 2007; Blackmore, Açıkmeşe & Scharf 2010):
 *       min ∫Γ dt   s.t.  ‖u‖ ≤ Γ,  ρ₁(m) ≤ Γ ≤ ρ₂(m),  ‖u_h‖ ≤ tanθ·u_z,
 *                         ‖u_{k+1} − u_k‖ ≤ r·h,  glide-slope cone,  p(t_f) = p_f,  v(t_f) = v_f.
 *     The thrust floor (two Raptors at 40 % give T/W > 1: no hover) is the Γ lower bound — the
 *     non-convex |u| ≥ ρ₁ constraint is not approximated, it is relaxed losslessly. The plan models
 *     what the old law ignored: propellant depletion (ρ₁, ρ₂ follow the predicted mass), drag and a
 *     measured disturbance acceleration, the thrust-vector slew the attitude loop can follow, and a
 *     thrust direction that is upright at the gate. Controls are first-order-hold (continuous).
 *     The final time is found by a line search on the fuel of the converged solutions.
 *  2. **Tracking** (`trackDescent`, every control step): a_cmd = u_ref(t) + K_p e_p + K_d e_v.
 *     The reference carries the feed-forward; feedback only removes model error.
 *  3. **Disturbance observer** (`observeDisturbance`): the difference between the measured and the
 *     predicted velocity change is the unmodelled acceleration (wind, drag error); it is fed to the
 *     next plan.
 *
 * Everything is in the pad ENU frame, relative to the catch point; accelerations are thrust/mass.
 * Pure functions; the solver is `conic.ts`.
 */
import { type V3, v3, clamp } from "@ship/math"
import { type Cone, type ConicSolution, normalizeRows, solveConic, sparseRows } from "./conic"

export const G_DESCENT = 9.79

export interface DescentVehicle {
  readonly nEng: number
  /** N per engine at the ambient pressure, 100 % and at the throttle floor */
  readonly thrustMax: number
  readonly thrustMin: number
  /** effective exhaust velocity I_sp,vac·g₀ (m/s) and the ambient back-pressure term p∞·A_e per engine (N) */
  readonly ve: number
  readonly pAe: number
}

export interface DescentProblem {
  readonly p0: V3
  readonly v0: V3
  /** realised thrust acceleration now (continuity of the command); null on a cold start */
  readonly uNow: V3 | null
  readonly mass: number
  readonly veh: DescentVehicle
  readonly pf: V3
  readonly vf: V3
  readonly tf: number
  /** control nodes (first-order hold), default 25 */
  readonly nodes?: number
  /** constant unmodelled acceleration from the disturbance observer (m/s², gravity excluded) */
  readonly dist?: V3
  /**
   * Aerodynamic force model: `force(xHat, vAir)` returns the force (N) on the vehicle for a body axis
   * and an air-relative velocity (the landing aero database). The plan is linearised about the
   * previous iterate (successive convexification): node k uses that iterate's velocity and thrust
   * direction (the body axis follows the thrust), so the aerodynamic acceleration enters as a known
   * time profile and the problem stays convex.
   */
  readonly aero?: { readonly wind: V3; readonly force: (xHat: V3, vAir: V3) => V3 }
  /**
   * Catch corridor: for the last `time` seconds the horizontal distance from the target is bounded
   * by r0 + rate·(t_f − t), so the ship comes down onto the pins from above, between the arms.
   */
  readonly corridor?: { readonly time: number; readonly r0: number; readonly rate: number } | null
  /**
   * Minimum-miss mode: the horizontal terminal position is not an equality but a quadratic penalty
   * with this weight (1/m² scale). Used when the exact problem has no solution: the plan then lands
   * as close to the target as the vehicle can, with every other constraint still enforced.
   */
  readonly softPos?: number
  /** thrust-direction cone half-angle from vertical: wide during the divert, `tiltEndDeg` at the gate */
  readonly tiltMaxDeg?: number
  readonly tiltEndDeg?: number
  /** time before the gate over which the cone narrows (s) */
  readonly uprightTime?: number
  /** rate of the realised thrust acceleration now (m/s³): the body's current rotation, for continuity */
  readonly uDotNow?: V3 | null
  /**
   * Attitude dynamics in the plan. The thrust direction is the body axis, so the *horizontal* thrust
   * acceleration can only change as fast as the body rotates: ‖u̇_h‖ ≤ rate (≈ ω_max·|u|, m/s³) and
   * ‖ü_h‖ ≤ accel (≈ ω̇_max·|u|, m/s⁴ — the gimbal torque at the throttle floor over the pitch
   * inertia). The vertical component follows the throttle: |u̇_z| ≤ rateZ.
   */
  readonly rate?: number
  readonly accel?: number
  readonly rateZ?: number
  /** glide-slope cone about the gate: elevation angle (deg from horizontal) and radius margin at the gate (m) */
  readonly glide?: { readonly deg: number; readonly margin: number } | null
  /** margins kept from the throttle limits for the tracker (fractions) */
  readonly floorMargin?: number
  readonly ceilMargin?: number
  readonly wSmooth?: number
  readonly wStart?: number
  /** weight on matching the first node's horizontal thrust to the realised direction */
  readonly wStartH?: number
  readonly prev?: DescentPlan | null
  readonly maxIter?: number
}

export interface DescentPlan {
  /** absolute time of node 0 (set by the caller), horizon and node spacing */
  readonly t0: number
  readonly tf: number
  readonly h: number
  readonly K: number
  /** node thrust acceleration (3K), slack Γ (K), position/velocity (3K), mass (K), disturbance incl. gravity (3K) */
  readonly u: Float64Array
  readonly gamma: Float64Array
  readonly p: Float64Array
  readonly v: Float64Array
  readonly m: Float64Array
  readonly d: Float64Array
  /** propellant over the plan (kg) */
  readonly fuel: number
  readonly converged: boolean
  readonly rPrim: number
  readonly rDual: number
  readonly iters: number
  /** unused floor margin (the plan no longer enforces the engine floor; `fitThrust` does) */
  readonly slackMax: number
  /** terminal position/velocity error of the returned controls (m, m/s) */
  readonly termPos: number
  readonly termVel: number
  /** true when the plan was solved in minimum-miss mode (termPos is then the predicted miss) */
  readonly soft: boolean
  readonly sol: ConicSolution
}

const DEG = Math.PI / 180

/** first-order-hold influence matrices: v_j = v0 + Σ Cv[j][k] a_k, p_j = p0 + j h v0 + Σ Cp[j][k] a_k */
const influence = (K: number, h: number): { Cv: Float64Array; Cp: Float64Array } => {
  const Cv = new Float64Array(K * K), Cp = new Float64Array(K * K)
  for (let j = 0; j + 1 < K; j++) {
    for (let k = 0; k < K; k++) {
      Cp[(j + 1) * K + k] = Cp[j * K + k] + h * Cv[j * K + k]
      Cv[(j + 1) * K + k] = Cv[j * K + k]
    }
    Cv[(j + 1) * K + j] += h / 2
    Cv[(j + 1) * K + j + 1] += h / 2
    Cp[(j + 1) * K + j] += (h * h) / 3
    Cp[(j + 1) * K + j + 1] += (h * h) / 6
  }
  return { Cv, Cp }
}

/** resample a previous plan's node array (stride 1 or 3) at absolute time t */
const resample = (prev: DescentPlan, arr: Float64Array, stride: number, t: number, out: number[]): void => {
  const s = clamp((t - prev.t0) / prev.h, 0, prev.K - 1)
  const k = Math.min(prev.K - 2, Math.floor(s)), f = s - k
  for (let j = 0; j < stride; j++) out[j] = arr[stride * k + j] * (1 - f) + arr[stride * (k + 1) + j] * f
}

/**
 * One convex solve at a fixed final time. `t0` is the absolute time of the first node (used to
 * resample the previous plan for the mass/drag profiles and the warm start).
 */
export const solveDescent = (pr: DescentProblem, t0 = 0): DescentPlan => {
  const K = pr.nodes ?? 25
  const h = pr.tf / (K - 1)
  const g = G_DESCENT
  const { Cv, Cp } = influence(K, h)
  const veh = pr.veh
  const tiltMax = (pr.tiltMaxDeg ?? 45) * DEG, tiltEnd = (pr.tiltEndDeg ?? 6) * DEG, tUp = pr.uprightTime ?? 3
  const rate = pr.rate ?? 4.2, accel = pr.accel ?? 2.8, rateZ = pr.rateZ ?? 15
  const floorM = 1 + (pr.floorMargin ?? 0.03), ceilM = 1 - (pr.ceilMargin ?? 0.05)
  const wSmooth = pr.wSmooth ?? 0.02, wStart = pr.wStart ?? 0.05, wStartH = pr.wStartH ?? 0.5
  const dist = pr.dist ?? v3(0, 0, 0)
  const prev = pr.prev && pr.prev.K >= 2 ? pr.prev : null
  // ---- profiles the convex problem is linearised about: mass and velocity along the horizon
  const mass = new Float64Array(K), vGuess = new Float64Array(3 * K)
  // body axis along the horizon (for the aerodynamic profile): the realised axis now, upright later
  const xGuess = new Float64Array(3 * K)
  {
    const un = pr.uNow ? Math.hypot(pr.uNow.x, pr.uNow.y, pr.uNow.z) : 0
    for (let k = 0; k < K; k++) {
      if (pr.uNow && un > 1e-3 && k === 0) { xGuess[0] = pr.uNow.x / un; xGuess[1] = pr.uNow.y / un; xGuess[2] = pr.uNow.z / un }
      else xGuess[3 * k + 2] = 1
    }
  }
  const tmp: number[] = [0, 0, 0]
  const thrustMid = 0.5 * (veh.thrustMin + veh.thrustMax) * veh.nEng
  const fillProfiles = (gam: Float64Array | null, vel: Float64Array | null): void => {
    mass[0] = pr.mass
    for (let k = 0; k < K; k++) {
      if (k > 0) {
        const Tprev = gam ? mass[k - 1] * gam[k - 1] : thrustMid
        mass[k] = Math.max(0.5 * pr.mass, mass[k - 1] - (h * (Tprev + veh.nEng * veh.pAe)) / veh.ve)
      }
      for (let a = 0; a < 3; a++) {
        const v0a = a === 0 ? pr.v0.x : a === 1 ? pr.v0.y : pr.v0.z
        const vfa = a === 0 ? pr.vf.x : a === 1 ? pr.vf.y : pr.vf.z
        vGuess[3 * k + a] = vel ? vel[3 * k + a] : v0a + ((vfa - v0a) * k) / (K - 1)
      }
    }
  }
  if (prev) {
    const gam = new Float64Array(K), vel = new Float64Array(3 * K)
    for (let k = 0; k < K; k++) {
      resample(prev, prev.gamma, 1, t0 + k * h, tmp); gam[k] = tmp[0]
      resample(prev, prev.v, 3, t0 + k * h, tmp); vel[3 * k] = tmp[0]; vel[3 * k + 1] = tmp[1]; vel[3 * k + 2] = tmp[2]
      resample(prev, prev.u, 3, t0 + k * h, tmp)
      const um = Math.hypot(tmp[0], tmp[1], tmp[2])
      if (um > 1e-3) { xGuess[3 * k] = tmp[0] / um; xGuess[3 * k + 1] = tmp[1] / um; xGuess[3 * k + 2] = tmp[2] / um }
    }
    fillProfiles(gam, vel)
  } else fillProfiles(null, null)

  let best: DescentPlan | null = null
  let warm: ConicSolution | null = null
  // Thrust floor by successive convexification. The lossless relaxation (Γ ≥ ρ₁, ‖u‖ ≤ Γ) is only
  // tight for the pure fuel problem; with the slew limit, the continuity term and the glide slope the
  // optimum can sit at ‖u‖ < ρ₁, which two Raptors cannot fly. So the non-convex ‖u_k‖ ≥ ρ₁ is
  // imposed through its linearisation about the previous iterate's direction n̂_k:  n̂_k·u_k ≥ ρ₁
  // (a half-space inside the feasible set, so every iterate satisfies the true constraint).
  const nHat = new Float64Array(3 * K)
  let haveDir = false
  if (prev) {
    for (let k = 0; k < K; k++) {
      resample(prev, prev.u, 3, t0 + k * h, tmp)
      const um = Math.hypot(tmp[0], tmp[1], tmp[2])
      if (um > 1e-3) { nHat[3 * k] = tmp[0] / um; nHat[3 * k + 1] = tmp[1] / um; nHat[3 * k + 2] = tmp[2] / um } else nHat[3 * k + 2] = 1
    }
    haveDir = true
  }
  const passes = prev ? 2 : 5
  for (let pass = 0; pass < passes; pass++) {
    // ---- disturbance acceleration per node: gravity + observer + drag on the profile velocity
    const d = new Float64Array(3 * K)
    for (let k = 0; k < K; k++) {
      let dx = dist.x, dy = dist.y, dz = dist.z - g
      if (pr.aero) {
        const va = v3(vGuess[3 * k] - pr.aero.wind.x, vGuess[3 * k + 1] - pr.aero.wind.y, vGuess[3 * k + 2] - pr.aero.wind.z)
        const f = pr.aero.force(v3(xGuess[3 * k], xGuess[3 * k + 1], xGuess[3 * k + 2]), va)
        dx += f.x / mass[k]; dy += f.y / mass[k]; dz += f.z / mass[k]
      }
      d[3 * k] = dx; d[3 * k + 1] = dy; d[3 * k + 2] = dz
    }
    // constant parts of the node states
    const cP = new Float64Array(3 * K), cV = new Float64Array(3 * K)
    const p0 = [pr.p0.x, pr.p0.y, pr.p0.z], v0 = [pr.v0.x, pr.v0.y, pr.v0.z]
    for (let j = 0; j < K; j++) for (let a = 0; a < 3; a++) {
      let sp = p0[a] + j * h * v0[a], sv = v0[a]
      for (let k = 0; k < K; k++) { sp += Cp[j * K + k] * d[3 * k + a]; sv += Cv[j * K + k] * d[3 * k + a] }
      cP[3 * j + a] = sp; cV[3 * j + a] = sv
    }
    // ---- variables x = [u (3K), Γ (K)]
    const n = 4 * K
    const iu = (k: number, a: number): number => 3 * k + a
    const ig = (k: number): number => 3 * K + k
    const rows: (readonly [number, number])[][] = []
    const cones: Cone[] = []
    // (1) terminal equalities, row-normalised
    const pf = [pr.pf.x, pr.pf.y, pr.pf.z], vf = [pr.vf.x, pr.vf.y, pr.vf.z]
    const bEq = new Float64Array(6)
    let nP = 0, nV = 0
    for (let k = 0; k < K; k++) { nP += Cp[(K - 1) * K + k] ** 2; nV += Cv[(K - 1) * K + k] ** 2 }
    nP = Math.sqrt(nP); nV = Math.sqrt(nV)
    const soft = pr.softPos !== undefined && pr.softPos > 0
    const eqB: number[] = []
    for (let a = 0; a < 3; a++) {
      if (!(soft && a < 2)) {
        rows.push(Array.from({ length: K }, (_, k) => [iu(k, a), Cp[(K - 1) * K + k] / nP] as const))
        eqB.push((pf[a] - cP[3 * (K - 1) + a]) / nP)
      }
      rows.push(Array.from({ length: K }, (_, k) => [iu(k, a), Cv[(K - 1) * K + k] / nV] as const))
      eqB.push((vf[a] - cV[3 * (K - 1) + a]) / nV)
    }
    void bEq
    cones.push({ kind: "eq", n: eqB.length, b: Float64Array.from(eqB) })
    // (2) throttle band on Γ, following the predicted mass
    // The plan may command any thrust from 0 to the ceiling. The 40 % *floor* is a property of the
    // engines, not of the trajectory: `fitThrust` spends the mandatory surplus laterally at the
    // command stage, which keeps the vertical (the timed deceleration) exactly as planned. Putting
    // the floor in the QP made the terminal (upright, contact speed) infeasible at low sink rates
    // and the solver spent its budget on the resulting conflict.
    const lo = new Float64Array(K), hi = new Float64Array(K)
    for (let k = 0; k < K; k++) {
      rows.push([[ig(k), 1]])
      lo[k] = (floorM * veh.nEng * veh.thrustMin) / mass[k]
      hi[k] = Math.max(lo[k] + 0.2, (ceilM * veh.nEng * veh.thrustMax) / mass[k])
    }
    cones.push({ kind: "box", n: K, lo, hi })
    // (3) ‖u_k‖ ≤ Γ_k
    for (let k = 0; k < K; k++) {
      rows.push([[ig(k), 1]], [[iu(k, 0), 1]], [[iu(k, 1), 1]], [[iu(k, 2), 1]])
      cones.push({ kind: "soc", n: 4 })
    }
    // (4) pointing cone ‖u_h‖ ≤ tanθ_k·u_z. The first nodes are opened to contain the realised
    //     direction (after the flip the body is still far from vertical); the rate limit walks it in.
    const tiltNow = pr.uNow ? Math.atan2(Math.hypot(pr.uNow.x, pr.uNow.y), Math.max(pr.uNow.z, 1e-3)) : 0
    for (let k = 0; k < K; k++) {
      const tLeft = pr.tf - k * h
      let th = tLeft >= tUp ? tiltMax : tiltEnd + ((tiltMax - tiltEnd) * tLeft) / tUp
      const open = tiltNow + 6 * DEG - (0.5 * k * h * rate) / Math.max(lo[0], 1)
      th = Math.min(80 * DEG, Math.max(th, open))
      rows.push([[iu(k, 2), Math.tan(th)]], [[iu(k, 0), 1]], [[iu(k, 1), 1]])
      cones.push({ kind: "soc", n: 3 })
    }
    // (5) attitude dynamics: horizontal slew rate and slew acceleration, throttle slew on the vertical
    const zr = new Float64Array(K - 1).fill(rateZ * h), zl = new Float64Array(K - 1).fill(-rateZ * h)
    for (let k = 0; k + 1 < K; k++) {
      for (let a = 0; a < 2; a++) rows.push([[iu(k + 1, a), 1], [iu(k, a), -1]])
      cones.push({ kind: "ball", n: 2, r: rate * h })
    }
    for (let k = 0; k + 1 < K; k++) rows.push([[iu(k + 1, 2), 1], [iu(k, 2), -1]])
    cones.push({ kind: "box", n: K - 1, lo: zl, hi: zr })
    for (let k = 0; k + 1 < K; k++) {
      if (k === 0) {
        if (!pr.uDotNow) continue
        for (let a = 0; a < 2; a++) rows.push([[iu(1, a), 1], [iu(0, a), -1]])
        // the current rotation may already exceed the planning rate: keep the constraint consistent
        const dn = Math.hypot(pr.uDotNow.x, pr.uDotNow.y), f = dn > rate ? rate / dn : 1
        cones.push({ kind: "ball", n: 2, r: accel * h * h, shift: Float64Array.of(-pr.uDotNow.x * f * h, -pr.uDotNow.y * f * h) })
      } else {
        for (let a = 0; a < 2; a++) rows.push([[iu(k + 1, a), 1], [iu(k, a), -2], [iu(k - 1, a), 1]])
        cones.push({ kind: "ball", n: 2, r: accel * h * h })
      }
    }
    // (6) glide-slope cone about the gate (relaxed just enough to contain the start)
    if (pr.glide) {
      const dz0 = pr.p0.z - pr.pf.z, d0 = Math.hypot(pr.p0.x - pr.pf.x, pr.p0.y - pr.pf.y)
      const cot = Math.max(1 / Math.tan(pr.glide.deg * DEG), dz0 > 1 ? (1.2 * Math.max(d0 - pr.glide.margin, 0)) / dz0 : 0)
      for (let j = 1; j < K - 1; j++) {
        let nr = 0
        for (let k = 0; k < K; k++) nr += Cp[j * K + k] ** 2
        const s = 1 / Math.max(Math.sqrt(nr), 1e-9)
        rows.push(Array.from({ length: K }, (_, k) => [iu(k, 2), s * cot * Cp[j * K + k]] as const))
        rows.push(Array.from({ length: K }, (_, k) => [iu(k, 0), s * Cp[j * K + k]] as const))
        rows.push(Array.from({ length: K }, (_, k) => [iu(k, 1), s * Cp[j * K + k]] as const))
        cones.push({
          kind: "soc", n: 3,
          shift: Float64Array.of(s * (cot * (cP[3 * j + 2] - pr.pf.z) + pr.glide.margin), s * (cP[3 * j] - pr.pf.x), s * (cP[3 * j + 1] - pr.pf.y)),
        })
      }
    }
    // (6b) catch corridor over the last seconds: ‖p_h(t_j) − p_f‖ ≤ r0 + rate·(t_f − t_j)
    if (pr.corridor && !soft) {
      for (let j = 1; j < K - 1; j++) {
        const left = pr.tf - j * h
        if (left > pr.corridor.time) continue
        let nr = 0
        for (let k = 0; k < K; k++) nr += Cp[j * K + k] ** 2
        const sc = 1 / Math.max(Math.sqrt(nr), 1e-9)
        rows.push(Array.from({ length: K }, (_, k) => [iu(k, 0), sc * Cp[j * K + k]] as const))
        rows.push(Array.from({ length: K }, (_, k) => [iu(k, 1), sc * Cp[j * K + k]] as const))
        cones.push({ kind: "ball", n: 2, r: sc * (pr.corridor.r0 + pr.corridor.rate * left), shift: Float64Array.of(sc * (cP[3 * j] - pr.pf.x), sc * (cP[3 * j + 1] - pr.pf.y)) })
      }
    }
    // (7) thrust floor: the engines cannot throttle below 40 %. The magnitude constraint |u| ≥ ρ₁ is
    // non-convex; it is imposed through its half-space linearisation about the previous iterate's
    // direction (n̂·u ≥ ρ₁). With it the plan is a *timed* descent (there is no hover), which is the
    // only vertical profile two Raptors can fly.
    if (haveDir) {
      const flo = new Float64Array(K), fhi = new Float64Array(K).fill(1e9)
      for (let k = 0; k < K; k++) {
        rows.push([[iu(k, 0), nHat[3 * k]], [iu(k, 1), nHat[3 * k + 1]], [iu(k, 2), nHat[3 * k + 2]]])
        flo[k] = lo[k]
      }
      cones.push({ kind: "box", n: K, lo: flo, hi: fhi })
    }
    // ---- objective: fuel ∫Γ dt (trapezoid) + smoothness + continuity with the realised thrust
    const P = new Float64Array(n * n), q = new Float64Array(n)
    for (let k = 0; k < K; k++) q[ig(k)] = k === 0 || k === K - 1 ? h / 2 : h
    for (let k = 0; k + 1 < K; k++) for (let a = 0; a < 3; a++) {
      const i = iu(k, a), j = iu(k + 1, a)
      P[i * n + i] += 2 * wSmooth; P[j * n + j] += 2 * wSmooth; P[i * n + j] -= 2 * wSmooth; P[j * n + i] -= 2 * wSmooth
    }
    // continuity with the realised thrust: the body cannot jump to a new direction, so the first
    // node is pulled (softly — a hard pin can make short-horizon problems infeasible) to it
    if (pr.uNow) {
      const un = [pr.uNow.x, pr.uNow.y, pr.uNow.z]
      for (let a = 0; a < 3; a++) { const w = a < 2 ? wStartH : wStart; P[iu(0, a) * n + iu(0, a)] += 2 * w; q[iu(0, a)] -= 2 * w * un[a] }
    }
    // minimum-miss mode: w·‖p_h(t_f) − p_f‖² (rank-one per axis over the controls)
    if (soft) {
      const w = pr.softPos as number
      for (let a = 0; a < 2; a++) {
        const e0 = cP[3 * (K - 1) + a] - pf[a]
        for (let k = 0; k < K; k++) {
          const ck = Cp[(K - 1) * K + k]
          if (ck === 0) continue
          q[iu(k, a)] += 2 * w * ck * e0
          for (let l = 0; l < K; l++) P[iu(k, a) * n + iu(l, a)] += 2 * w * ck * Cp[(K - 1) * K + l]
        }
      }
    }
    const A = sparseRows(rows)
    normalizeRows(A, cones)
    const sol = solveConic({ n, P, q, A, cones }, { maxIter: pr.maxIter ?? 700, warm, epsAbs: 1e-3, epsRel: 1e-3, alpha: 1 })
    warm = haveDir ? sol : null // the first pass has no floor rows: its duals do not fit the next one
    // ---- unpack
    const u = sol.x.slice(0, 3 * K), gamma = sol.x.slice(3 * K, 4 * K)
    const p = new Float64Array(3 * K), v = new Float64Array(3 * K)
    for (let j = 0; j < K; j++) for (let a = 0; a < 3; a++) {
      let sp = cP[3 * j + a], sv = cV[3 * j + a]
      for (let k = 0; k < K; k++) { sp += Cp[j * K + k] * u[3 * k + a]; sv += Cv[j * K + k] * u[3 * k + a] }
      p[3 * j + a] = sp; v[3 * j + a] = sv
    }
    let slackMax = 0, fuel = 0
    const mOut = new Float64Array(K)
    mOut[0] = pr.mass
    for (let k = 0; k < K; k++) {
      const um = Math.hypot(u[3 * k], u[3 * k + 1], u[3 * k + 2])
      slackMax = Math.max(slackMax, lo[k] - um)
      if (k > 0) {
        const mdot = (mOut[k - 1] * Math.max(gamma[k - 1], lo[k - 1] / floorM) + veh.nEng * veh.pAe) / veh.ve
        mOut[k] = mOut[k - 1] - h * mdot
        fuel += h * mdot
      }
    }
    const e = 3 * (K - 1)
    best = {
      t0, tf: pr.tf, h, K, u, gamma, p, v, m: mOut, d, fuel,
      converged: sol.converged, rPrim: sol.rPrim, rDual: sol.rDual, iters: sol.iters, slackMax,
      termPos: Math.hypot(p[e] - pr.pf.x, p[e + 1] - pr.pf.y, p[e + 2] - pr.pf.z),
      termVel: Math.hypot(v[e] - pr.vf.x, v[e + 1] - pr.vf.y, v[e + 2] - pr.vf.z),
      soft, sol,
    }
    // relinearise about this solution for the next pass
    mass.set(mOut)
    vGuess.set(v)
    for (let k = 0; k < K; k++) {
      const um = Math.hypot(u[3 * k], u[3 * k + 1], u[3 * k + 2])
      if (um > 1e-3) { xGuess[3 * k] = u[3 * k] / um; xGuess[3 * k + 1] = u[3 * k + 1] / um; xGuess[3 * k + 2] = u[3 * k + 2] / um }
    }
    const hadDir = haveDir
    for (let k = 0; k < K; k++) {
      const um = Math.hypot(u[3 * k], u[3 * k + 1], u[3 * k + 2])
      if (um > 0.05 * lo[k]) { nHat[3 * k] = u[3 * k] / um; nHat[3 * k + 1] = u[3 * k + 1] / um; nHat[3 * k + 2] = u[3 * k + 2] / um }
      else if (!haveDir) { nHat[3 * k] = 0; nHat[3 * k + 1] = 0; nHat[3 * k + 2] = 1 }
    }
    haveDir = true
    if (!sol.converged) break
    if (hadDir && pass >= 1 && slackMax <= 0.01 * lo[0]) break
  }
  return best as DescentPlan
}

/** kinematic estimate of the time to the gate at the mean of the current and the gate sink rate */
export const descentTimeGuess = (pr: Pick<DescentProblem, "p0" | "v0" | "pf" | "vf">): number => {
  const dz = Math.max(pr.p0.z - pr.pf.z, 1)
  return clamp((2 * dz) / Math.max(-pr.v0.z - pr.vf.z, 2), 2, 80)
}

export interface DescentSearch {
  readonly plan: DescentPlan | null
  /** (t_f, fuel, converged) of every candidate, for diagnostics */
  readonly tried: readonly { readonly tf: number; readonly fuel: number; readonly ok: boolean }[]
}

/**
 * Free final time: fuel is unimodal in t_f over the feasible interval (Blackmore 2010), so scan a
 * coarse grid for the feasible interval and refine around the best by golden-section steps.
 */
export const planDescent = (base: Omit<DescentProblem, "tf">, t0 = 0, range?: readonly [number, number], grid = 6, refine = 2): DescentSearch => {
  // Deterministic by construction: a fixed number of candidates and iterations, never a wall-clock
  // budget (a time-boxed search makes the flight depend on the machine load — the same state then
  // gives different plans on different runs).
  const guess = descentTimeGuess(base)
  const [lo, hi] = range ?? [0.55 * guess, 1.9 * guess]
  const tried: { tf: number; fuel: number; ok: boolean }[] = []
  const run = (tf: number): DescentPlan => {
    const pl = solveDescent({ ...base, tf }, t0)
    tried.push({ tf, fuel: pl.fuel, ok: pl.converged && pl.slackMax <= 0.05 })
    return pl
  }
  let best: DescentPlan | null = null
  const ok = (pl: DescentPlan): boolean => pl.converged && pl.slackMax <= 0.05
  // exact mode: least propellant; minimum-miss mode: least miss, then least propellant
  const cost = (pl: DescentPlan): number => (pl.soft ? 1e4 * pl.termPos : 0) + pl.fuel
  const consider = (pl: DescentPlan): void => { if (ok(pl) && (!best || cost(pl) < cost(best))) best = pl }
  const n = Math.max(2, grid)
  for (let i = 0; i < n; i++) consider(run(lo + ((hi - lo) * i) / (n - 1)))
  let step = (hi - lo) / (n - 1)
  for (let r = 0; r < refine && best; r++) {
    step *= 0.5
    const c = (best as DescentPlan).tf
    consider(run(Math.max(lo, c - step)))
    consider(run(Math.min(hi, c + step)))
  }
  return { plan: best, tried }
}

export interface DescentSample {
  readonly p: V3
  readonly v: V3
  /** reference thrust acceleration (the feed-forward) */
  readonly u: V3
  /** rate of the reference thrust acceleration (m/s³): attitude-rate feed-forward */
  readonly uDot: V3
  /** time left on the plan (s) */
  readonly tgo: number
}

/** the plan at absolute time t (first-order hold: exact integrals of the piecewise-linear controls) */
export const sampleDescent = (pl: DescentPlan, t: number): DescentSample => {
  const tau = clamp(t - pl.t0, 0, pl.tf)
  const k = Math.min(pl.K - 2, Math.floor(tau / pl.h)), s = tau - k * pl.h
  const ax = (a: number): [number, number, number] => {
    const A0 = pl.u[3 * k + a] + pl.d[3 * k + a], A1 = pl.u[3 * (k + 1) + a] + pl.d[3 * (k + 1) + a]
    const dA = (A1 - A0) / pl.h
    return [
      pl.p[3 * k + a] + pl.v[3 * k + a] * s + 0.5 * A0 * s * s + (dA * s * s * s) / 6,
      pl.v[3 * k + a] + A0 * s + 0.5 * dA * s * s,
      pl.u[3 * k + a] + ((pl.u[3 * (k + 1) + a] - pl.u[3 * k + a]) * s) / pl.h,
    ]
  }
  const x = ax(0), y = ax(1), z = ax(2)
  const inside = t - pl.t0 >= 0 && t - pl.t0 <= pl.tf
  const ud = (a: number): number => (inside ? (pl.u[3 * (k + 1) + a] - pl.u[3 * k + a]) / pl.h : 0)
  return { p: v3(x[0], y[0], z[0]), v: v3(x[1], y[1], z[1]), u: v3(x[2], y[2], z[2]), uDot: v3(ud(0), ud(1), ud(2)), tgo: pl.tf - tau }
}

export interface TrackGains {
  readonly kp: number
  readonly kd: number
}

/** PD + acceleration feed-forward on the plan: the commanded thrust acceleration (ENU) */
export const trackDescent = (pl: DescentPlan, t: number, p: V3, v: V3, g: TrackGains): { readonly u: V3; readonly ref: DescentSample } => {
  const r = sampleDescent(pl, t)
  return {
    u: v3(r.u.x + g.kp * (r.p.x - p.x) + g.kd * (r.v.x - v.x), r.u.y + g.kp * (r.p.y - p.y) + g.kd * (r.v.y - v.y), r.u.z + g.kp * (r.p.z - p.z) + g.kd * (r.v.z - v.z)),
    ref: r,
  }
}

/**
 * Fit a commanded thrust acceleration into what the engines can deliver, vertical channel first:
 * the vertical component is kept (it is the timed deceleration), the horizontal one is limited by
 * the tilt cone and the throttle ceiling, and a command below the throttle floor is raised by
 * tilting further along its own horizontal direction (never by adding vertical thrust).
 */
export const fitThrust = (u: V3, lo: number, hi: number, tanTilt: number): V3 => {
  const uz = clamp(u.z, 0.3 * lo, hi)
  let hx = u.x, hy = u.y
  const hMax = Math.min(tanTilt * uz, Math.sqrt(Math.max(0, hi * hi - uz * uz)))
  const hn = Math.hypot(hx, hy)
  if (hn > hMax) { hx *= hMax / hn; hy *= hMax / hn }
  const mag = Math.hypot(hx, hy, uz)
  if (mag < lo) {
    // The engine floor cannot be spent vertically: raising |u| along the current direction would
    // over-decelerate and stall the descent (the old bug). Spend the mandatory surplus *laterally*,
    // along the commanded horizontal direction (or +x if it is exactly zero), so the vertical
    // component — the timed deceleration — is preserved.
    const h1 = Math.hypot(hx, hy)
    const need = Math.min(Math.sqrt(Math.max(0, lo * lo - uz * uz)), tanTilt * uz)
    const dx = h1 > 1e-6 ? hx / h1 : 1, dy = h1 > 1e-6 ? hy / h1 : 0
    hx = dx * need; hy = dy * need
  }
  return v3(hx, hy, uz)
}

export interface DisturbanceState {
  readonly t: number
  readonly v: V3
  /** estimated unmodelled acceleration (m/s², gravity and thrust excluded) */
  readonly d: V3
}

/**
 * Disturbance observer: v̇ = u_thrust + g + d. The measured velocity change over the last step minus
 * the thrust and gravity part is a sample of d (drag, wind, thrust misalignment); it is low-passed
 * with time constant `tau`. `uReal` is the realised thrust acceleration over the step.
 */
export const observeDisturbance = (s: DisturbanceState | null, t: number, v: V3, uReal: V3, tau = 1.5, limit = 6): DisturbanceState => {
  if (!s || t - s.t <= 1e-6 || t - s.t > 0.5) return { t, v, d: s ? s.d : v3(0, 0, 0) }
  const dt = t - s.t
  const k = clamp(dt / tau, 0, 1)
  const mx = (v.x - s.v.x) / dt - uReal.x, my = (v.y - s.v.y) / dt - uReal.y, mz = (v.z - s.v.z) / dt - uReal.z + G_DESCENT
  return {
    t, v,
    d: v3(clamp(s.d.x + k * (mx - s.d.x), -limit, limit), clamp(s.d.y + k * (my - s.d.y), -limit, limit), clamp(s.d.z + k * (mz - s.d.z), -limit, limit)),
  }
}
