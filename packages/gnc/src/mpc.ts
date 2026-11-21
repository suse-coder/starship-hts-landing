/**
 * Model predictive control of the landing burn (translational, pad ENU frame).
 *
 *   state   p (relative to the aim point), v, a (realised thrust acceleration)
 *   control u = commanded thrust acceleration (m/s²)
 *   model   a⁺ = (1−β)a + βu, β = 1 − e^{−h/τ}   (attitude + throttle loop as a first-order lag τ:
 *           the vehicle cannot re-point its thrust instantly; the plan knows it)
 *           ā = ½(a + a⁺),  p⁺ = p + v·h + ½(ā + g)·h²,  v⁺ = v + (ā + g)·h        (h = horizon step)
 *   cost    Σ_k [ q_p|p_k|² + q_v|v_k − v_ref|² + r_u|u_k − u_hov|² + r_d|u_k − u_{k−1}|² ]
 *           + Q_p|p_N|² + Q_v|v_N − v_f|² + glide-slope soft penalty (stay above the arm height
 *           until close)
 *   limits  u_z ∈ [a_zmin, a_zmax] (single-Raptor throttle range), |u_h| ≤ tan θ · u_z (tilt cone),
 *           weak axis ŵ: holding a tilt costs no torque, but SLEWING the thrust toward ŵ needs torque
 *           about the engine's radial axis, which only the roll-limited tangential gimbal provides
 *           (allocateGimbal). Modelled as a strong rate penalty r_w·((u_k − u_{k−1})·ŵ)².
 *
 * Solved by accelerated projected gradient (FISTA) with an adjoint gradient, warm-started from the
 * previous solution shifted by one step. Pure function: (problem, warm start) → plan.
 */

export interface MpcProblem {
  readonly p0: readonly [number, number, number]
  readonly v0: readonly [number, number, number]
  /** realised thrust acceleration now (ENU, m/s²) and the closed-loop pointing time constant τ (s);
   *  τ = 0 → the old instantaneous model */
  readonly a0?: readonly [number, number, number]
  readonly tau?: number
  readonly n: number // horizon steps
  readonly h: number // s per step
  readonly vRef: readonly [number, number, number] // stage velocity reference (usually = vF)
  /** terminal horizontal position target (x across the arms, y along them); default (0,0) */
  readonly pRefH?: readonly [number, number]
  /** terminal vertical position target (default 0 = the rail plane); the landing uses a gate above it */
  readonly pRefZ?: number
  /** terminal horizontal velocity target; the approach should be *along* the arms (y), |vx| ≈ 0 */
  readonly vRefH?: readonly [number, number]
  /** optional per-stage vertical velocity reference (n+1 values, the hoverslam profile). The
   *  horizontal stage reference stays `vRef`; tracking the timed vertical profile with a high
   *  weight is what gives the vertical channel priority over the lateral divert. */
  readonly vRefZ?: Float64Array
  readonly vF: readonly [number, number, number] // terminal velocity
  /** gravitational acceleration along z (default G). 0 for a purely lateral problem whose vertical
   *  channel is flown by the analytic hoverslam profile. */
  readonly gravity?: number
  readonly aZMin: number
  readonly aZMax: number
  /** thrust-magnitude range |u| ∈ [aMagMin, aMagMax] (engines cannot throttle below their minimum;
   *  the non-convex lower bound is handled by projection, cf. lossless convexification) */
  readonly aMagMin?: number
  readonly aMagMax?: number
  readonly tanTilt: number
  /** lossless-convexification thrust cone (Blackmore/Açıkmeşe PDG): |u| ≤ Γ, Γ ∈ [aMin, aMax],
   *  u_z ≥ cosθ|u|. When set, the projection is the exact Euclidean projection onto the cone and the
   *  fuel cost integrates Γ = max(|u|, aMin) (the slack is tight at the optimum). */
  readonly cone?: { readonly cosTilt: number; readonly aMin: number; readonly aMax: number; readonly hardMin?: boolean }
  /** second-order (rate-limited) thrust-direction model: the plan applies the thrust along a
   *  direction that can only slew at `omegaMax` rad/s from the current realised direction `d0`, and
   *  the engine floor raises the magnitude to `aMin` along that direction. This is the minimum model
   *  that makes the plan trackable by a torque-limited attitude loop. */
  readonly dyn?: { readonly d0: readonly [number, number, number]; readonly omegaMax: number; readonly aMin: number }
  readonly weak: readonly [number, number] | null // horizontal unit vector of the weak axis
  readonly aWeak: number // optional hard band on u·ŵ (Infinity = off)
  readonly rdWeak?: number // rate penalty on the weak-axis component
  readonly uPrev: readonly [number, number, number] // last applied command (rate cost)
  readonly glide: number // glide-slope ratio z ≥ glide·|p_h| (soft), 0 = off
  readonly weights?: Partial<MpcWeights>
  /** approach corridor along the arms: penalise stages with y > `yMax` (tower side) or y < `yMin`
   *  (past the open south end), so the final path stays in the arm slot. Soft quadratic hinges. */
  readonly approach?: { readonly yMax: number; readonly yMin: number; readonly w: number }
  /** first-order velocity field v_ref = −p_h/τ: the plan approaches the centreline at a controlled
   *  speed and decelerates into the gate, instead of accelerating and asking for a late reversal. */
  readonly vField?: { readonly tau: number; readonly vMax: number; readonly w: number }
  /**
   * Slew-rate-parametrised plan (trackability by construction). The decision variables become the
   * per-step increments δ_k of the commanded acceleration, u_k = u0 + Σ_{j≤k} δ_j, with the exact
   * separable bound |δ_k| ≤ rate·h. `u0` is the *realised* thrust acceleration, so the first command
   * is always reachable; `rate` (m/s³) is |u|·ω for the attitude loop's achievable slew ω. The
   * magnitude limit |u_k| ≤ aMagMax is enforced by a forward clamp in the feasibility map (optional
   * interior penalty `wBox`, default 0). Without it the plan asked for ≈170° thrust reversals in one
   * replan that the torque-limited attitude loop needed ≈ 5 s to fly (landing traces, review/02).
   */
  readonly slew?: { readonly u0: readonly [number, number, number]; readonly rate: number; readonly wBox?: number }
}

export interface MpcWeights {
  readonly qp: number
  readonly qv: number
  readonly ru: number
  readonly rd: number
  readonly Qp: number
  readonly Qv: number
  readonly wGlide: number
  /** scale of the horizontal stage-velocity cost (1 = as `qv`; 0 = the lateral divert is driven only
   *  by the stage position and terminal costs, so it spreads over the whole horizon) */
  readonly wvXY?: number
  /** cost on the thrust magnitude |u| (≈ propellant flow): the burn is minimised subject to the
   *  terminal catch constraint */
  readonly wMag?: number
  /** penalty on the *direction* rate of the thrust vector (rad per step)²: the TVC cannot slew the
   *  thrust axis faster than its torque allows, so the plan must not ask for it */
  readonly wDir?: number
  /** anisotropy of the horizontal position/velocity cost: x (across the arms, ±1 m) vs y (along the
   *  rails, ±17 m). Defaults 1/1; the catch sets x ≫ y so the plan centres across the arms first. */
  readonly wX?: number
  readonly wY?: number
}

const W0: MpcWeights = { qp: 0.002, qv: 0.02, ru: 0.05, rd: 0.15, Qp: 4, Qv: 12, wGlide: 0.5 }
const G = 9.79

export interface MpcPlan {
  readonly u: Float64Array // n × 3
  readonly cost: number
  readonly pN: readonly [number, number, number]
  readonly vN: readonly [number, number, number]
  readonly iters: number
  /** predicted positions over the horizon (3(n+1); frame = the problem's p0 frame) — visual export */
  readonly path: Float64Array
}

/** lag factor β of one horizon step */
const lagBeta = (pr: MpcProblem): number => (pr.tau && pr.tau > 1e-3 && pr.a0 ? 1 - Math.exp(-pr.h / pr.tau) : 1)
const grav = (pr: MpcProblem): number => pr.gravity ?? G

/** forward simulation; positions/velocities/realised accelerations (n+1)×3 */
const rollout = (pr: MpcProblem, u: Float64Array, P: Float64Array, V: Float64Array, A?: Float64Array): void => {
  const h = pr.h, b = lagBeta(pr), g = grav(pr)
  const a0 = pr.a0 ?? [u[0], u[1], u[2]]
  P[0] = pr.p0[0]; P[1] = pr.p0[1]; P[2] = pr.p0[2]
  V[0] = pr.v0[0]; V[1] = pr.v0[1]; V[2] = pr.v0[2]
  let ax = a0[0], ay = a0[1], az = a0[2]
  if (A) { A[0] = ax; A[1] = ay; A[2] = az }
  let dx = pr.dyn ? pr.dyn.d0[0] : 0, dy = pr.dyn ? pr.dyn.d0[1] : 0, dz = pr.dyn ? pr.dyn.d0[2] : 1
  const cap = pr.dyn ? pr.dyn.omegaMax * h : Math.PI
  for (let k = 0; k < pr.n; k++) {
    let ux = u[3 * k], uy = u[3 * k + 1], uz = u[3 * k + 2]
    if (pr.dyn) {
      const um = Math.hypot(ux, uy, uz) || 1e-6
      const tx = ux / um, ty = uy / um, tz = uz / um
      const dt = Math.max(-1, Math.min(1, dx * tx + dy * ty + dz * tz))
      const ang = Math.acos(dt)
      if (ang > cap && ang > 1e-9) {
        // rotate d toward t by exactly `cap` (the TVC rate limit)
        const nx0 = tx - dt * dx, ny0 = ty - dt * dy, nz0 = tz - dt * dz
        const nn = Math.hypot(nx0, ny0, nz0)
        if (nn > 1e-9) {
          const ca = Math.cos(cap), sa = Math.sin(cap)
          dx = dx * ca + (nx0 / nn) * sa
          dy = dy * ca + (ny0 / nn) * sa
          dz = dz * ca + (nz0 / nn) * sa
          const dn = Math.hypot(dx, dy, dz) || 1
          dx /= dn; dy /= dn; dz /= dn
        }
      } else { dx = tx; dy = ty; dz = tz }
      const mag = Math.max(um, pr.dyn.aMin) // the engine floor applies along the realised direction
      ux = mag * dx; uy = mag * dy; uz = mag * dz
    }
    const nx = (1 - b) * ax + b * ux, ny = (1 - b) * ay + b * uy, nz = (1 - b) * az + b * uz
    const mean = [0.5 * (ax + nx), 0.5 * (ay + ny), 0.5 * (az + nz) - g]
    for (let j = 0; j < 3; j++) {
      P[3 * (k + 1) + j] = P[3 * k + j] + V[3 * k + j] * h + 0.5 * mean[j] * h * h
      V[3 * (k + 1) + j] = V[3 * k + j] + mean[j] * h
    }
    ax = nx; ay = ny; az = nz
    if (A) { A[3 * (k + 1)] = ax; A[3 * (k + 1) + 1] = ay; A[3 * (k + 1) + 2] = az }
  }
}

/** cost and gradient (adjoint pass) */
const costGrad = (pr: MpcProblem, w: MpcWeights, u: Float64Array, g: Float64Array, P: Float64Array, V: Float64Array, lp: Float64Array, lv: Float64Array): number => {
  rollout(pr, u, P, V)
  const n = pr.n, h = pr.h, b = lagBeta(pr)
  let J = 0
  const la = [0, 0, 0] // adjoint of the realised acceleration
  // terminal
  for (let j = 0; j < 3; j++) {
    const ep = j < 2 && pr.pRefH ? P[3 * n + j] - pr.pRefH[j] : j === 2 && pr.pRefZ !== undefined ? P[3 * n + 2] - pr.pRefZ : P[3 * n + j]
    const ev = j < 2 && pr.vRefH ? V[3 * n + j] - pr.vRefH[j] : V[3 * n + j] - pr.vF[j]
    const wx = j === 0 ? (w.wX ?? 1) : j === 1 ? (w.wY ?? 1) : 1
    J += w.Qp * wx * ep * ep + w.Qv * wx * ev * ev
    lp[j] = 2 * w.Qp * wx * ep
    lv[j] = 2 * w.Qv * wx * ev
  }
  for (let k = n - 1; k >= 0; k--) {
    // gradient w.r.t. u_k from the dynamics into step k+1: through ā_k (weight β/2) and a_{k+1} (weight β)
    for (let j = 0; j < 3; j++) {
      const dAbar = lv[j] * h + lp[j] * 0.5 * h * h
      g[3 * k + j] = b === 1 ? dAbar : dAbar * 0.5 * b + la[j] * b
      la[j] = b === 1 ? 0 : dAbar * (1 - 0.5 * b) + la[j] * (1 - b)
    }
    // control costs
    for (let j = 0; j < 3; j++) {
      const du = u[3 * k + j] - (j === 2 ? grav(pr) : 0)
      J += w.ru * du * du
      g[3 * k + j] += 2 * w.ru * du
      const prev = k === 0 ? pr.uPrev[j] : u[3 * (k - 1) + j]
      const dd = u[3 * k + j] - prev
      J += w.rd * dd * dd
      g[3 * k + j] += 2 * w.rd * dd
      if (k > 0) g[3 * (k - 1) + j] -= 2 * w.rd * dd
    }
    if (w.wDir) {
      const mx = Math.hypot(u[3 * k], u[3 * k + 1], u[3 * k + 2]) || 1e-6
      const pxp = k === 0 ? pr.uPrev : u.subarray(3 * (k - 1), 3 * k)
      const pm = Math.hypot(pxp[0], pxp[1], pxp[2]) || 1e-6
      // 1 − cos(angle) between consecutive thrust directions
      const dotp = (u[3 * k] * pxp[0] + u[3 * k + 1] * pxp[1] + u[3 * k + 2] * pxp[2]) / (mx * pm)
      const dphi = Math.max(0, 1 - dotp)
      J += w.wDir * dphi
      const dm = w.wDir * (1 / (mx * pm))
      for (let j = 0; j < 3; j++) g[3 * k + j] -= dm * (pxp[j] - dotp * u[3 * k + j] / mx)
    }
    if (w.wMag) {
      const um = Math.hypot(u[3 * k], u[3 * k + 1], u[3 * k + 2]) || 1e-6
      const gam = pr.cone ? Math.max(um, pr.cone.aMin) : um // Γ (slack): tight at the optimum
      J += w.wMag * gam
      if (um > (pr.cone ? pr.cone.aMin : 0)) for (let j = 0; j < 3; j++) g[3 * k + j] += (w.wMag * u[3 * k + j]) / um
    }
    if (pr.weak && pr.rdWeak) {
      const [wx, wy] = pr.weak
      const px = k === 0 ? pr.uPrev[0] : u[3 * (k - 1)], py = k === 0 ? pr.uPrev[1] : u[3 * (k - 1) + 1]
      const dw = (u[3 * k] - px) * wx + (u[3 * k + 1] - py) * wy
      J += pr.rdWeak * dw * dw
      g[3 * k] += 2 * pr.rdWeak * dw * wx; g[3 * k + 1] += 2 * pr.rdWeak * dw * wy
      if (k > 0) { g[3 * (k - 1)] -= 2 * pr.rdWeak * dw * wx; g[3 * (k - 1) + 1] -= 2 * pr.rdWeak * dw * wy }
    }
    // stage cost at step k (k ≥ 1) and adjoint propagation to step k
    const nlp = [0, 0, 0], nlv = [0, 0, 0]
    if (k >= 1) {
      for (let j = 0; j < 3; j++) {
        const ep = P[3 * k + j], ev = V[3 * k + j] - (j === 2 && pr.vRefZ ? pr.vRefZ[k] : pr.vRef[j])
        const wv = j === 2 ? w.qv : w.qv * (w.wvXY ?? 1)
        const wp = j === 0 ? (w.wX ?? 1) : j === 1 ? (w.wY ?? 1) : 1
        J += w.qp * wp * ep * ep + wv * ev * ev
        nlp[j] += 2 * w.qp * wp * ep
        nlv[j] += 2 * wv * ev
        if (pr.vField && j < 2) {
          const pref = pr.pRefH ? pr.pRefH[j] : 0
          const vr = Math.max(-pr.vField.vMax, Math.min(pr.vField.vMax, -(P[3 * k + j] - pref) / pr.vField.tau))
          const ef = V[3 * k + j] - vr
          J += pr.vField.w * ef * ef
          nlv[j] += 2 * pr.vField.w * ef
          nlp[j] += (2 * pr.vField.w * ef) / pr.vField.tau
        }
      }
      if (pr.approach) {
        const yvHi = P[3 * k + 1] - pr.approach.yMax
        if (yvHi > 0) {
          J += pr.approach.w * yvHi * yvHi
          nlp[1] += 2 * pr.approach.w * yvHi
        }
        const yvLo = pr.approach.yMin - P[3 * k + 1]
        if (yvLo > 0) {
          J += pr.approach.w * yvLo * yvLo
          nlp[1] -= 2 * pr.approach.w * yvLo
        }
      }
      if (pr.glide > 0) {
        const ph = Math.hypot(P[3 * k], P[3 * k + 1])
        const viol = pr.glide * ph - P[3 * k + 2]
        if (viol > 0) {
          J += w.wGlide * viol * viol
          nlp[2] -= 2 * w.wGlide * viol
          if (ph > 1e-6) { nlp[0] += 2 * w.wGlide * viol * pr.glide * (P[3 * k] / ph); nlp[1] += 2 * w.wGlide * viol * pr.glide * (P[3 * k + 1] / ph) }
        }
      }
    }
    for (let j = 0; j < 3; j++) {
      const lpk = nlp[j] + lp[j]
      const lvk = nlv[j] + lv[j] + lp[j] * h
      lp[j] = lpk
      lv[j] = lvk
    }
  }
  return J
}

/**
 * Exact Euclidean projection of one control vector onto the thrust-pointing cone with a magnitude cap
 * K = { u : u_z ≥ cosθ·|u|, |u| ≤ aMax } (the lower magnitude bound is the slack Γ ≥ aMin in the fuel
 * cost, not a constraint on |u| — lossless convexification). The (r, z) projection is analytic.
 */
const projectCone = (u: Float64Array, k: number, c: number, aMax: number, aHardMin = 0): void => {
  let x = u[3 * k], y = u[3 * k + 1], z = u[3 * k + 2]
  let m = Math.hypot(x, y, z)
  if (m > aMax) { const s = aMax / m; x *= s; y *= s; z *= s; m = aMax }
  let r = Math.hypot(x, y)
  if (z < c * r) {
    const f = z - c * r
    const rp = r + (c * f) / (1 + c * c)
    const zp = z - f / (1 + c * c)
    const s = r > 1e-9 ? rp / r : 0
    x *= s; y *= s; z = zp
    m = Math.hypot(x, y, z)
    if (m > aMax) { const sc = aMax / m; x *= sc; y *= sc; z *= sc }
  }
  // hard engine floor: the throttle cannot go below 40 %. Scaling along the direction keeps the
  // cone/upper bounds (the set is star-shaped about the origin outside the inner ball). The two-phase
  // plan leaves this off and applies the floor at execution; the joint PDG needs it in the plan or
  // it optimises a hover the ship cannot fly.
  if (aHardMin > 0) {
    const mm = Math.hypot(x, y, z)
    if (mm < aHardMin) {
      const s = aHardMin / Math.max(mm, 1e-9)
      x *= s; y *= s; z *= s
    }
  }
  u[3 * k] = x; u[3 * k + 1] = y; u[3 * k + 2] = z
}

/** feasible-set projection of one control vector (box on u_z, weak-axis band, tilt cone) */
const project = (pr: MpcProblem, u: Float64Array, k: number): void => {
  if (pr.cone) { projectCone(u, k, pr.cone.cosTilt, pr.cone.aMax, pr.cone.hardMin ? pr.cone.aMin : 0); return }
  let x = u[3 * k], y = u[3 * k + 1]
  let z = Math.min(pr.aZMax, Math.max(pr.aZMin, u[3 * k + 2]))
  if (pr.weak) {
    const [wx, wy] = pr.weak
    const c = x * wx + y * wy
    const cc = Math.max(-pr.aWeak, Math.min(pr.aWeak, c))
    x += (cc - c) * wx
    y += (cc - c) * wy
  }
  // z = 0 marks a purely lateral problem (the vertical channel is flown by the hoverslam profile);
  // there the tilt cone is not applied and the magnitude band is the lateral budget.
  if (z > 1e-9) {
    const hMax = pr.tanTilt * z
    const hn = Math.hypot(x, y)
    if (hn > hMax) { x *= hMax / hn; y *= hMax / hn }
  }
  if (pr.aMagMin !== undefined || pr.aMagMax !== undefined) {
    const m = Math.hypot(x, y, z)
    const lo = pr.aMagMin ?? 0, hi = pr.aMagMax ?? Infinity
    const sc = m < lo ? lo / Math.max(m, 1e-6) : m > hi ? hi / m : 1
    x *= sc; y *= sc; z *= sc
  }
  u[3 * k] = x; u[3 * k + 1] = y; u[3 * k + 2] = z
}

/**
 * FISTA over the increments δ (see `MpcProblem.slew`). u = u0 + cumsum(δ); ∂J/∂δ_j = Σ_{k≥j} ∂J/∂u_k.
 * The feasible set of δ is a product of balls (exact projection); the magnitude cap is a convex
 * penalty. A purely lateral problem (aZMin = aZMax = 0) keeps the z components at zero.
 */
const solveSlew = (pr: MpcProblem, sl: NonNullable<MpcProblem["slew"]>, w: MpcWeights, warm: Float64Array | null, iters: number): MpcPlan => {
  const n = pr.n, m = 3 * n
  const lateral = pr.aZMin === 0 && pr.aZMax === 0
  const r = Math.max(sl.rate * pr.h, 1e-6)
  const aMax = pr.aMagMax ?? Infinity
  // the hard clamp in `projBall` enforces the cap; the optional penalty only shapes the interior and
  // its curvature enters the step size, so it defaults to off (40 froze the solver: step ∝ 1/(wBox·n²))
  const wBox = sl.wBox ?? 0
  const u0 = [sl.u0[0], sl.u0[1], lateral ? 0 : sl.u0[2]]
  const u = new Float64Array(m), gu = new Float64Array(m)
  const P = new Float64Array(3 * (n + 1)), V = new Float64Array(3 * (n + 1))
  const lp = new Float64Array(3), lv = new Float64Array(3)
  const toU = (d: Float64Array): void => {
    let x = u0[0], y = u0[1], z = u0[2]
    for (let k = 0; k < n; k++) { x += d[3 * k]; y += d[3 * k + 1]; z += d[3 * k + 2]; u[3 * k] = x; u[3 * k + 1] = y; u[3 * k + 2] = z }
  }
  // feasibility map: each increment into its slew ball, then a forward pass that clamps the running
  // command into the magnitude disk. The disk projection is non-expansive and u_{k−1} lies in the
  // disk, so the clamped increment stays inside its ball: both limits hold exactly afterwards.
  const projBall = (d: Float64Array): void => {
    let x = u0[0], y = u0[1], z = u0[2]
    const cap = Math.max(aMax, Math.hypot(x, y, z)) // a realised start above the cap is kept, not jumped
    for (let k = 0; k < n; k++) {
      if (lateral) d[3 * k + 2] = 0
      const nn = Math.hypot(d[3 * k], d[3 * k + 1], d[3 * k + 2])
      if (nn > r) { const s = r / nn; d[3 * k] *= s; d[3 * k + 1] *= s; d[3 * k + 2] *= s }
      let nx = x + d[3 * k], ny = y + d[3 * k + 1], nz = z + d[3 * k + 2]
      const mm = Math.hypot(nx, ny, nz)
      if (Number.isFinite(cap) && mm > cap) { const s = cap / mm; nx *= s; ny *= s; nz *= s }
      d[3 * k] = nx - x; d[3 * k + 1] = ny - y; d[3 * k + 2] = nz - z
      x = nx; y = ny; z = nz
    }
  }
  const f = (q: MpcProblem, d: Float64Array, gd: Float64Array, box: boolean): number => {
    toU(d)
    let J = costGrad(q, w, u, gu, P, V, lp, lv)
    if (box && Number.isFinite(aMax)) {
      for (let k = 0; k < n; k++) {
        const mm = Math.hypot(u[3 * k], u[3 * k + 1], u[3 * k + 2])
        if (mm > aMax) {
          const ex = mm - aMax
          J += wBox * ex * ex
          for (let j = 0; j < 3; j++) gu[3 * k + j] += (2 * wBox * ex * u[3 * k + j]) / mm
        }
      }
    }
    // suffix sums: δ_j moves every u_k with k ≥ j
    let sx = 0, sy = 0, sz = 0
    for (let k = n - 1; k >= 0; k--) {
      sx += gu[3 * k]; sy += gu[3 * k + 1]; sz += gu[3 * k + 2]
      gd[3 * k] = sx; gd[3 * k + 1] = sy; gd[3 * k + 2] = lateral ? 0 : sz
    }
    return J
  }
  // warm start: the previous plan's commands shifted by one step, re-expressed as feasible increments
  const d = new Float64Array(m)
  if (warm && warm.length >= 3) {
    let px = u0[0], py = u0[1], pz = u0[2]
    for (let k = 0; k < n; k++) {
      const s = Math.min(k + 1, warm.length / 3 - 1)
      let dx = warm[3 * s] - px, dy = warm[3 * s + 1] - py, dz = lateral ? 0 : warm[3 * s + 2] - pz
      const nn = Math.hypot(dx, dy, dz)
      if (nn > r) { const sc = r / nn; dx *= sc; dy *= sc; dz *= sc }
      d[3 * k] = dx; d[3 * k + 1] = dy; d[3 * k + 2] = dz
      px += dx; py += dy; pz += dz
    }
  }
  // Diagonal majoriser. The δ parametrisation is ill-conditioned (an early increment moves the whole
  // tail: curvature ∝ (n−j)²…(n−j)⁴), so one scalar step crawls on the late increments. The problem is
  // quadratic (glide off), so the exact Hessian costs m gradient evaluations; its Gershgorin row sums,
  // maximised over each step's 3 components, give per-block steps D_j ≥ H (guaranteed majoriser) and
  // keep the per-step ball projection exact. Box-penalty bound: 2·wBox·(n−j) per block.
  const q0: MpcProblem = { ...pr, glide: 0 }
  const zero = new Float64Array(m), g0 = new Float64Array(m), gq = new Float64Array(m), e = new Float64Array(m)
  f(q0, zero, g0, false)
  const rowAbs = new Float64Array(m)
  for (let i = 0; i < m; i++) {
    if (lateral && i % 3 === 2) continue
    e.fill(0); e[i] = 1
    f(q0, e, gq, false)
    for (let k = 0; k < m; k++) rowAbs[k] += Math.abs(gq[k] - g0[k])
  }
  const stepK = new Float64Array(n)
  for (let k = 0; k < n; k++) {
    const dk = Math.max(rowAbs[3 * k], rowAbs[3 * k + 1], rowAbs[3 * k + 2]) + 2 * wBox * (n - k) + 2 * w.wGlide * h4(pr)
    stepK[k] = 1 / (1.02 * dk + 1e-9)
  }
  let step = 1 // scale on the per-block steps (the monotone safeguard may halve it)
  // FISTA with function-value adaptive restart (O'Donoghue & Candès 2015) and a monotone safeguard:
  // the forward clamp makes the feasibility map non-Euclidean, so plain momentum can oscillate.
  projBall(d)
  const yv = Float64Array.from(d), gd = new Float64Array(m), gTmp = new Float64Array(m)
  let dPrev = Float64Array.from(d)
  let jPrev = f(pr, dPrev, gTmp, true)
  let t = 1
  let it = 0
  for (; it < iters; it++) {
    f(pr, yv, gd, true)
    const dn = new Float64Array(m)
    for (let i = 0; i < m; i++) dn[i] = yv[i] - step * stepK[(i / 3) | 0] * gd[i]
    projBall(dn)
    const jn = f(pr, dn, gTmp, true)
    if (jn > jPrev) {
      // restart: drop the momentum; if a plain step from the last iterate still rises, shrink the step
      t = 1
      yv.set(dPrev)
      if (jn > jPrev * (1 + 1e-9) && it > 0) step *= 0.5
      continue
    }
    const tn = (1 + Math.sqrt(1 + 4 * t * t)) / 2
    for (let i = 0; i < m; i++) yv[i] = dn[i] + ((t - 1) / tn) * (dn[i] - dPrev[i])
    dPrev = dn
    jPrev = jn
    t = tn
  }
  const J = f(pr, dPrev, gd, true)
  const uf = Float64Array.from(u)
  rollout(pr, uf, P, V)
  return { u: uf, cost: J, pN: [P[3 * n], P[3 * n + 1], P[3 * n + 2]], vN: [V[3 * n], V[3 * n + 1], V[3 * n + 2]], iters: it, path: Float64Array.from(P) }
}

export const solveLandingMpc = (pr: MpcProblem, warm: Float64Array | null, iters = 80): MpcPlan => {
  const w: MpcWeights = { ...W0, ...pr.weights }
  if (pr.slew) return solveSlew(pr, pr.slew, w, warm, iters)
  const n = pr.n, m = 3 * n
  const u = new Float64Array(m)
  if (warm && warm.length >= 3) {
    // shift the previous plan by one step, repeat the last control
    for (let k = 0; k < n; k++) { const s = Math.min(k + 1, warm.length / 3 - 1); u.set(warm.subarray(3 * s, 3 * s + 3), 3 * k) }
  } else for (let k = 0; k < n; k++) { u[3 * k + 2] = G }
  for (let k = 0; k < n; k++) project(pr, u, k)
  const g = new Float64Array(m), P = new Float64Array(3 * (n + 1)), V = new Float64Array(3 * (n + 1))
  const lp = new Float64Array(3), lv = new Float64Array(3)
  // Lipschitz constant of the quadratic part by power iteration on the Hessian (gradient differences)
  const zero = new Float64Array(m), g0 = new Float64Array(m), gq = new Float64Array(m)
  costGrad({ ...pr, glide: 0 }, w, zero, g0, P, V, lp, lv)
  let vec = new Float64Array(m).fill(1 / Math.sqrt(m)), L = 1
  for (let it = 0; it < 8; it++) {
    costGrad({ ...pr, glide: 0 }, w, vec, gq, P, V, lp, lv)
    let nn = 0
    for (let i = 0; i < m; i++) { gq[i] -= g0[i]; nn += gq[i] * gq[i] }
    nn = Math.sqrt(nn)
    L = nn
    for (let i = 0; i < m; i++) vec[i] = gq[i] / (nn || 1)
  }
  const step = 1 / (1.3 * L + 8 * (pr.rdWeak ?? 0) + 2 * w.wGlide * h4(pr) + 1e-9)
  // FISTA
  const yv = Float64Array.from(u)
  let uPrevIt = Float64Array.from(u)
  let t = 1
  let J = 0
  let it = 0
  for (; it < iters; it++) {
    J = costGrad(pr, w, yv, g, P, V, lp, lv)
    const un = new Float64Array(m)
    for (let i = 0; i < m; i++) un[i] = yv[i] - step * g[i]
    for (let k = 0; k < n; k++) project(pr, un, k)
    const tn = (1 + Math.sqrt(1 + 4 * t * t)) / 2
    for (let i = 0; i < m; i++) yv[i] = un[i] + ((t - 1) / tn) * (un[i] - uPrevIt[i])
    uPrevIt = un
    t = tn
  }
  const uf = uPrevIt
  J = costGrad(pr, w, uf, g, P, V, lp, lv)
  rollout(pr, uf, P, V)
  const path = new Float64Array(3 * (n + 1))
  path.set(P)
  return { u: uf, cost: J, pN: [P[3 * n], P[3 * n + 1], P[3 * n + 2]], vN: [V[3 * n], V[3 * n + 1], V[3 * n + 2]], iters: it, path }
}

/** curvature contribution scale of the glide penalty (position sensitivity to controls ~ (n·h²)²) */
const h4 = (pr: MpcProblem): number => (pr.glide > 0 ? (pr.n * pr.h * pr.h) ** 2 * pr.n : 0)

/**
 * Vertical velocity reference for the hoverslam: the throttle-floor arc for t1 then the constant
 * deceleration that arrives at (z = 0, vz = −vF) at `tgo`. Sampled at the MPC stage times.
 * Pure; exported for the landing tests.
 */
export const hoverslamProfileZ = (z0: number, s0: number, aMin: number, vF: number, tgo: number, n: number): Float64Array => {
  const out = new Float64Array(n + 1)
  const z = Math.max(z0, 0)
  const s = Math.max(s0, vF)
  const a = Math.max(aMin, 1e-3)
  const tFloorMax = Math.max(0, (s - vF) / a)
  // total time of a floor-arc of length t1 followed by the constant decel that reaches the target
  const total = (t1: number): number => {
    const s1 = s - a * t1
    const z1 = z - (s * t1 - 0.5 * a * t1 * t1)
    if (z1 <= 1e-6) return t1 + (s1 - vF) / a
    const a2 = (s1 * s1 - vF * vF) / (2 * z1)
    return t1 + (s1 - vF) / Math.max(a2, 1e-3)
  }
  let t1 = 0
  if (tgo > total(0)) {
    let lo = 0, hi = tFloorMax
    for (let k = 0; k < 40; k++) {
      const mid = 0.5 * (lo + hi)
      if (total(mid) < tgo) lo = mid
      else hi = mid
    }
    t1 = 0.5 * (lo + hi)
  }
  const s1 = s - a * t1
  const z1 = Math.max(z - (s * t1 - 0.5 * a * t1 * t1), 0)
  const a2 = z1 > 1e-6 ? Math.max((s1 * s1 - vF * vF) / (2 * z1), 0.05) : Math.max(s1 - vF, 0.05)
  const t2 = Math.max(0, (s1 - vF) / a2)
  for (let k = 0; k <= n; k++) {
    const t = (k / n) * tgo
    if (t <= t1) out[k] = -(s - a * t)
    else {
      const tau = Math.min(t - t1, t2)
      out[k] = -(Math.max(s1 - a2 * tau, vF))
    }
  }
  out[n] = -vF
  return out
}
