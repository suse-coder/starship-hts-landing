import { clamp, bracket } from "@ship/math"
import { type V3, v3, add, cross, dot, norm, normalize, scale, sub } from "@ship/math"
import { atmosphere } from "@ship/earth"
import { corotation, ecefToEcef0, ecefToGeodetic, eciToEcef, gravityJ2 } from "./predictorFrames"
import { aeroLookup, mhdDragLookup, mhdLiftLookup, type AeroTable, type MhdDragTable, trimAlpha } from "./tables"
import { MU_EARTH, OMEGA_EARTH } from "@ship/earth"

/**
 * Fast 3-DOF point-mass predictor (same gravity, atmosphere, Earth rotation and aerodynamic
 * tables as the 6-DOF model; trimmed attitude assumed). Used for pre-mission targeting and as
 * the numerical predictor of the predictor–corrector entry guidance.
 */
export type PredPhase = "coast" | "entry" | "loft" | "highAlpha" | "belly"

/**
 * Q30 trade-off (measured 2026-11-21, `.tmp/agentG/m4_t08.log`): gating this rule on the plateau
 * airspeed (pitch up at V ≤ 4.5 km/s) cuts the tile peak 1702 → 1613 K and the time above 1530 K
 * 35 s → 9 s, because the pitch-up is then shallower in q — but it moves the transition into denser
 * air (q 14 → 27 kPa), costs 3.4 km of belly-flop delivery and cannot reach 1530 K: the plateau α is
 * the bare hull's passive trim (≈ 29°), and the only lever Q30 identifies that would reach the limit
 * (α ≤ 22°) needs an α actuator this vehicle does not have. Delivery (the Q28 gate) wins; the tile
 * overshoot is reported, not tuned away.
 */
export const pitchUpDue = (q: number, qPrev: number, V: number, c: { readonly qPitch: number; readonly vPitchMax: number }): boolean =>
  V < c.vPitchMax || (q > 3000 && (q < c.qPitch || (q > qPrev && q < 1.6 * c.qPitch)))

/**
 * High-α glide damping (T11). Shuttle-style altitude-rate feedback on the vertical lift component,
 * flown identically by the flight law and by this predictor (ADR-08).
 *
 * Linearised about an equilibrium glide, a_L cos σ = G with G = g − V²/r and a_L = q S C_L / m ∝ e^{−h/H}:
 *   δḧ = −(G/H) δh  →  an undamped altitude oscillation (the entry phugoid), ω = √(G/H) ≈ 0.028 rad/s,
 *   T ≈ 225 s at 55 km. A constant |σ| (the predictor–corrector's downrange solution) has no ḣ term,
 *   so after the pitch-up (lift ≈ 3 G) the ship zooms ≈ 13 km and swings back (journal T11).
 * The law modulates the vertical lift around the corrector's |σ₀|:
 *   cos σ = cos σ₀ + k (ḣ_ref − ḣ) / a_L,   k = 2 ζ √(G/H)   ⇒  δḧ = −(G/H) δh − k δḣ (damping ratio ζ),
 * with the equilibrium-glide sink rate ḣ_ref = −2 H g / (V (L/D) cos σ₀) (d/dt of a_L cos σ₀ = G with
 * V̇ = −D/m). Only the rotation of the lift ABOUT v̂ is commanded; the caller rate-limits it (cold gas).
 * C_L, C_D are the trim values of the guidance table (not the instantaneous α, which carries the
 * short-period oscillation); H is the local density scale height of the same atmosphere model.
 *
 * ζ = 0.3 (T11 measurement, scripts/profile.ts; flip miss in brackets): ζ 0.3 → one pitch-up overshoot
 * 52.3 → 59.3 km and no second cycle, flip miss 1.2 km; ζ 0.7 → 56.4 km and 8.6 km; ζ 1.2 → 56.0 km and
 * 6.3 km. The bank slews at ≤ 1.5°/s, a lag of 20–40 s against a ≈ 225 s phugoid, so a larger gain
 * pushes the lift past the target and degrades the belly-flop delivery. Without damping the swing was
 * 64.3 → 48.4 → 51.1 km (≈ 200 s period, ζ ≈ 0.5 from two swings).
 */
/** Nose-first (alpha = 0) mode: no high-alpha lift, no belly-flop lift. The high-alpha mode remains the default. */
export const NOSE_FIRST = false
export const GLIDE_DAMPING = {
  zeta: 0.3,
  cosMin: -0.5, // the modulation may roll the lift to |σ| ≤ 120° (lift partly down) to stop a climb
  cosRefMin: 0.2, // |σ₀| > 78.5°: no equilibrium glide, no modulation
  hdotRefMin: -400, // m/s
  /**
   * High-α descent guard (ADR-08). The damped law alone still lets the post-pitch-up lift (~2–3 G)
   * zoom the ship: 6-DOF measured 52.3 → 57.0 km and a +70–80 m/s ḣ plateau for ~100 s. When the sink
   * is shallower than `hdotGuard`, cap the vertical lift component at G − `descentMargin` so the net
   * vertical acceleration is downward. Gated on ḣ so a fast sink can still un-bank (the guard is a
   * climb stopper, not a permanent extra deceleration). High-α only: the entry/loft lift-up is what
   * sets the range and the heat pulse, and must not be capped.
   */
  descentMargin: 3.5, // m/s² of net downward acceleration while the guard is active
  hdotGuard: -10, // m/s: hard monotonic-descent target (no positive ḣ interval)
  hdotBand: 1, // m/s: narrow ramp so the guard is effectively hard but not discontinuous
  liftSafety: 1.3, // measured lift / table lift (measured 1.2–1.6 g vs ~1.1 g table)
} as const

/** local density scale height H = −1 / (d ln ρ / dh) of the guidance atmosphere (m) */
export const scaleHeight = (h: number): number => {
  const hh = clamp(h, 0, 120e3)
  return 1000 / Math.log(atmosphere(hh).rho / atmosphere(hh + 1000).rho)
}

/**
 * Passive trim angle of the bare hull: the α where the pitching-moment coefficient vanishes with the
 * restoring sign. `AeroTable.trim` (tables.ts) searches only α ≥ 30°, so at hypersonic Mach — where the
 * naked hull's zero-moment α is ≈ 28° — it falls back to the 80° default. The high-α ship has no α
 * actuator (the α axis is deliberately left to the aerodynamics), so it settles at the *real*
 * zero-moment α, and the predictor must fly the same angle or it models a parachute (C_D 8.5, C_L 1.06,
 * L/D 0.12) instead of the glider the vehicle is (C_D 1.6, C_L 1.9, L/D 1.24).
 */
const passiveTrims = new WeakMap<AeroTable, readonly number[]>()
const passiveTrimTable = (t: AeroTable): readonly number[] => {
  const memo = passiveTrims.get(t)
  if (memo) return memo
  const out = t.machs.map((M) => {
    let c1 = aeroLookup(t, t.alphas[t.alphas.length - 1], M).Cm
    for (let i = t.alphas.length - 2; i >= 0; i--) {
      const c0 = aeroLookup(t, t.alphas[i], M).Cm
      if (c0 <= 0 && c1 > 0) return t.alphas[i] + ((t.alphas[i + 1] - t.alphas[i]) * -c0) / (c1 - c0)
      c1 = c0
    }
    return trimAlpha(t, M)
  })
  passiveTrims.set(t, out)
  return out
}
export const passiveTrim = (t: AeroTable, mach: number): number => {
  const tr = passiveTrimTable(t)
  const [i, f] = bracket(t.machs, mach)
  return tr[i] * (1 - f) + tr[i + 1] * f
}

export interface GlideState {
  readonly h: number // geodetic altitude, m
  readonly hdot: number // geodetic altitude rate, m/s
  readonly V: number // airspeed, m/s
  readonly r: number // geocentric radius, m
  readonly q: number // dynamic pressure, Pa
  readonly CL: number // trim lift coefficient
  readonly CD: number // trim drag coefficient
  readonly mass: number // kg
}

/** bank magnitude of the damped high-α law around the corrector's |σ₀| (rad, 0…π) */
export const dampedBank = (mag0: number, s: GlideState, sref: number, zeta: number = GLIDE_DAMPING.zeta, descentMargin = 0): number => {
  const aL = (s.q * sref * s.CL) / s.mass
  if (aL < 0.5 || s.V < 300) return mag0 // no usable lift: nothing to modulate
  const c0 = Math.cos(mag0)
  // |σ₀| beyond ≈ 78°: the corrector is dissipating range (lift sideways/down), there is no equilibrium
  // glide to damp around — the law steps aside (T11 run after1: ḣ_ref → −400 m/s pushed the lift down)
  if (c0 < GLIDE_DAMPING.cosRefMin) return mag0
  const g = MU_EARTH / (s.r * s.r)
  const G = Math.max(g - (s.V * s.V) / s.r, 0.5)
  const H = scaleHeight(s.h)
  const ld = Math.max(s.CL / Math.max(s.CD, 1e-3), 0.1)
  const hdRef = Math.max(-(2 * H * g) / (s.V * ld * c0), GLIDE_DAMPING.hdotRefMin)
  const k = 2 * zeta * Math.sqrt(G / H)
  let cT = c0 + (k * (hdRef - s.hdot)) / aL
  // High-α-only descent guard: while the sink is shallower than `hdotGuard`, the vertical lift may not
  // exceed G − descentMargin. It is a ceiling on cos σ (a floor on |σ|), applied only to the high-α
  // call sites; entry/loft/pitch-up pass descentMargin = 0 and are unchanged. The activation is ramped
  // over `hdotBand` so a sink hovering near the threshold does not chatter the bank.
  if (descentMargin > 0 && aL > G) {
    const active = clamp((s.hdot - GLIDE_DAMPING.hdotGuard) / GLIDE_DAMPING.hdotBand, 0, 1)
    if (active > 0) cT = Math.min(cT, (G - descentMargin * active) / (aL * GLIDE_DAMPING.liftSafety))
  }
  // Monotonic-descent bound (user 2026-11-21) was trialled here and reverted: see the report — an
  // entry-wide cap removes the high-α hold but cannot prevent the bank-reversal zoom and costs ~650 m
  // of delivery. The high-α guard above is the narrower replacement.
  return Math.acos(clamp(cT, GLIDE_DAMPING.cosMin, 1))
}

export interface PredContext {
  readonly aero: AeroTable
  readonly mhdDrag: MhdDragTable
  readonly sref: number
  readonly theta0: number
  readonly padEcef: V3
  readonly alphaNoseFirst: (V: number) => number
  readonly vLoft: number // start lofting below this airspeed
  readonly qPitch: number // pitch up when q below this (while lofting)
  readonly vPitchMax: number // pitch up at the latest at this speed
  readonly hEnd: number // stop altitude
  /** high-α glide-damping ratio override (mission tuning); defaults to GLIDE_DAMPING.zeta */
  readonly glideZeta?: number
  /** wind forecast (ENU m/s vs altitude); the air moves with the Earth plus this wind */
  readonly windForecast?: (h: number) => V3
}

export interface PredInput {
  readonly t: number
  readonly r: V3
  readonly v: V3
  readonly mass: number
  readonly phase: PredPhase
  readonly bankEntry: number // rad (magnitude, vertical-lift component cos σ)
  readonly bankHigh: number
  readonly impulse?: { readonly t: number; readonly dv: number } // retrograde Δv (planning)
  readonly mhdDragScale?: number // online estimate: actual / tabulated MHD drag
  readonly bankSign?: number // ±1: fly the signed bank (lateral lift included); undefined: vertical component only
  readonly tRev?: number // absolute time of the one planned bank reversal (high-α; default never)
  readonly sigma0?: number // signed bank flown now; the bank slews from it at sigmaRate
  readonly sigmaRate?: number // rad/s achievable bank slew (rotation about v̂)
  readonly tMax?: number // stop at this absolute time (verification)
  readonly hEnd?: number // stop altitude override (m above the pad's ellipsoid height)
  readonly liftFloor?: number // signed mode: minimum vertical component of the unit lift vector
  readonly windOfs?: V3 // measured local wind minus forecast (ENU), fading with altitude from the start
  readonly record?: boolean
  /** stop at the high-α → belly-flop hand-over (Mach 2) and return that state */
  readonly stopAtBelly?: boolean
  /** stop when the loft pitches up into high-α (the flight's high-α hand-over decision) */
  readonly stopAtPitchUp?: boolean
}

export interface PredOut {
  readonly t: number
  readonly r: V3
  /** inertial velocity at the end state (ECI) */
  readonly v: V3
  readonly vRel: V3
  readonly lat: number
  readonly lon: number
  readonly downrangeErr: number // + = overshoot of the pad
  readonly crossrangeErr: number // + = right of track
  readonly qMax: number
  readonly gMax: number
  readonly tEntry: number // time at 120 km
  readonly path: readonly V3[] // ECEF samples if recorded
  readonly pitch: { readonly t: number; readonly V: number; readonly h: number; readonly q: number } | null
}

export const predict = (c: PredContext, inp: PredInput): PredOut => {
  let { t, r, v } = inp
  let phase = inp.phase
  let impulseDone = !inp.impulse
  let qMax = 0, gMax = 0, tEntry = -1
  let qPrev = 1e9
  let pitch: PredOut["pitch"] = null
  const path: V3[] = []
  let lastRec = -1e9
  // reference frame for downrange/crossrange: fixed in the pad's tangent plane, along the inbound
  // direction from the start position to the pad, so the miss does not depend on the predicted path
  const theta00 = c.theta0 + OMEGA_EARTH * inp.t
  const upPad = normalize(c.padEcef)
  const d00 = sub(c.padEcef, eciToEcef(inp.r, theta00))
  const d00H = sub(d00, scale(upPad, dot(d00, upPad)))
  // close to the pad the start-to-pad direction is ill-conditioned: use the ground track instead
  const v00 = sub(inp.v, corotation(inp.r))
  const trk = sub(v00, scale(normalize(inp.r), dot(v00, normalize(inp.r))))
  const inbound = norm(d00H) < 15e3 && norm(trk) > 3 ? normalize(ecefToEcef0(trk, theta00)) : normalize(d00H)
  const h00 = ecefToGeodetic(eciToEcef(inp.r, theta00)).alt
  // signed bank state (high-α planned-reversal mode)
  const signed = inp.bankSign !== undefined && inp.bankSign !== 0
  let sg = inp.sigma0 ?? (inp.bankSign ?? 1) * inp.bankHigh
  const tRev = inp.tRev ?? Infinity
  const rate = inp.sigmaRate ?? 0.03
  let lv: V3 | null = null // lift unit vector (ECI), signed mode
  const airVel = (rr: V3, vv: V3, hh: number): V3 => {
    const vr0 = sub(vv, corotation(rr))
    if (!c.windForecast || hh >= 60e3) return vr0
    const w0 = c.windForecast(hh)
    const kOfs = inp.windOfs ? Math.exp(-Math.abs(hh - h00) / 3000) : 0
    const w = inp.windOfs ? add(w0, scale(inp.windOfs, kOfs)) : w0
    const upL = normalize(rr)
    const eastL = normalize(cross(v3(0, 0, 1), upL))
    const northL = cross(upL, eastL)
    return sub(vr0, add(scale(eastL, w.x), scale(northL, w.y)))
  }
  for (let it = 0; it < 400000; it++) {
    const theta = c.theta0 + OMEGA_EARTH * t
    const g = ecefToGeodetic(eciToEcef(r, theta))
    const h = g.alt
    if (inp.record && t - lastRec > (h > 100e3 ? 30 : 5)) {
      path.push(eciToEcef(r, theta))
      lastRec = t
    }
    if (h < (inp.hEnd ?? c.hEnd)) break
    if (inp.tMax !== undefined && t >= inp.tMax) break
    if (tEntry < 0 && h < 120e3) tEntry = t
    if (!impulseDone && inp.impulse && t >= inp.impulse.t) {
      v = add(v, scale(normalize(v), -inp.impulse.dv))
      impulseDone = true
    }
    const dt = h > 130e3 ? 10 : h > 60e3 ? 1.0 : phase === "highAlpha" || phase === "belly" ? 1.0 : 0.5
    const signedPhase = signed && (phase === "highAlpha" || phase === "belly")
    // high-α: the damped bank law around |σ₀| = bankHigh (same law as the flight, GLIDE_DAMPING)
    let bankH = inp.bankHigh
    if ((phase === "highAlpha" || phase === "belly") && h < 120e3) {
      const vr0 = airVel(r, v, h)
      const V0 = norm(vr0)
      if (V0 > 1) {
        const lam = g.lon + theta
        const upG = v3(Math.cos(g.lat) * Math.cos(lam), Math.cos(g.lat) * Math.sin(lam), Math.sin(g.lat)) // geodetic normal (ECI)
        const at0 = atmosphere(h)
        const M0 = V0 / at0.a
        const co0 = aeroLookup(c.aero, passiveTrim(c.aero, M0), M0)
        bankH = dampedBank(inp.bankHigh, { h, hdot: dot(upG, vr0), V: V0, r: norm(r), q: 0.5 * at0.rho * V0 * V0, CL: co0.CL, CD: co0.CD, mass: inp.mass }, c.sref, c.glideZeta, GLIDE_DAMPING.descentMargin)
      }
    }
    if (signedPhase) {
      // Lift-direction dynamics. The passive trim carries the lift along with v̂ for free (only α is
      // aerodynamically restored); a rotation ABOUT v̂ is actuated and rate-limited. So: transport
      // the lift vector ⟂ the new v̂, then rotate it about v̂ toward the target bank by ≤ rate·dt.
      const hh0 = ecefToGeodetic(eciToEcef(r, c.theta0 + OMEGA_EARTH * t)).alt
      const vh = normalize(airVel(r, v, hh0))
      const up0 = normalize(r)
      const upP0 = normalize(sub(up0, scale(vh, dot(up0, vh))))
      const right0 = cross(vh, upP0)
      if (lv === null) lv = add(scale(upP0, Math.cos(sg)), scale(right0, Math.sin(sg)))
      lv = normalize(sub(lv, scale(vh, dot(lv, vh))))
      const target = (t < tRev ? inp.bankSign! : -inp.bankSign!) * (phase === "highAlpha" || phase === "belly" ? bankH : inp.bankHigh)
      const lt = add(scale(upP0, Math.cos(target)), scale(right0, Math.sin(target)))
      const th = Math.atan2(dot(cross(lv, lt), vh), dot(lv, lt))
      const d = clamp(th, -rate * dt, rate * dt)
      lv = add(scale(lv, Math.cos(d)), scale(cross(vh, lv), Math.sin(d)))
    }
    const acc = (rr: V3, vv: V3): V3 => {
      const th = c.theta0 + OMEGA_EARTH * t
      const hh = ecefToGeodetic(eciToEcef(rr, th)).alt
      const ag = gravityJ2(rr)
      if (hh > 140e3) return ag
      const a = atmosphere(hh)
      const vr = airVel(rr, vv, hh)
      const V = norm(vr)
      if (V < 1) return ag
      const M = V / a.a
      const q = 0.5 * a.rho * V * V
      let alpha: number
      let bank: number
      if (NOSE_FIRST) { alpha = 0; bank = 0 }
      else if (phase === "entry" || phase === "coast") { alpha = c.alphaNoseFirst(V); bank = inp.bankEntry }
      else if (phase === "loft") { alpha = c.alphaNoseFirst(V); bank = 0 }
      else if (phase === "highAlpha") { alpha = passiveTrim(c.aero, M); bank = bankH }
      else { alpha = passiveTrim(c.aero, M); bank = 0 }
      // belly flop: the flight law can steer its lift heading anywhere; the reachable footprint
      // (scripts/footprint.ts, ≈ 10 km radius from Mach 2) is centred on the zero-net-lift carry,
      // so the predictor flies that centre and guidance targets the pad at the footprint centre.
      // belly flop: zero net lift (footprint centre) unless the signed bank law is flown through it
      const liftOn = phase !== "belly" || signedPhase
      const co = aeroLookup(c.aero, alpha, M)
      const mhdOn = phase === "entry" || phase === "loft" || phase === "coast"
      const ms = inp.mhdDragScale ?? 1
      const dCD = mhdOn ? mhdDragLookup(c.mhdDrag, hh, V) * ms : 0
      const dCL = mhdOn ? mhdLiftLookup(c.mhdDrag, hh, V) * ms : 0
      const D = q * c.sref * (co.CD + dCD)
      const L = liftOn ? q * c.sref * (co.CL + dCL) : 0
      const vh = scale(vr, 1 / V)
      const up = normalize(rr)
      const upPerp = normalize(sub(up, scale(vh, dot(up, vh))))
      // lift: vertical component × cos σ, plus the lateral component when a bank sign is held
      const right = cross(vh, upPerp)
      const lift = signedPhase && lv !== null
        ? normalize(sub(lv, scale(vh, dot(lv, vh))))
        : add(scale(upPerp, Math.cos(bank)), signed && phase === "entry" ? scale(right, inp.bankSign! * Math.sin(bank)) : scale(right, 0))
      const f = add(scale(vh, -D), scale(lift, L))
      return add(ag, scale(f, 1 / inp.mass))
    }
    // midpoint RK2
    const a1 = acc(r, v)
    const rm = add(r, scale(v, dt / 2))
    const vm = add(v, scale(a1, dt / 2))
    const a2 = acc(rm, vm)
    r = add(r, scale(vm, dt))
    v = add(v, scale(a2, dt))
    t += dt
    // phase logic (mirrors the 6-DOF phase machine)
    const vr = sub(v, corotation(r))
    const V = norm(vr)
    const a = atmosphere(Math.max(h, 0))
    const q = 0.5 * a.rho * V * V
    qMax = Math.max(qMax, q)
    gMax = Math.max(gMax, norm(sub(a2, gravityJ2(r))) / 9.80665)
    if (phase === "coast" && h < 120e3) phase = "entry"
    if (phase === "entry" && V < c.vLoft) phase = "loft"
    if (phase === "loft" && pitchUpDue(q, qPrev + 300, V, c)) {
      phase = "highAlpha"
      pitch = { t, V, h, q }
      if (inp.stopAtPitchUp) break
    }
    qPrev = phase === "loft" ? Math.min(qPrev, q) : 1e9
    if (phase === "highAlpha" && V / a.a < 2.0) {
      phase = "belly"
      if (inp.stopAtBelly) break
    }
  }
  const theta = c.theta0 + OMEGA_EARTH * t
  const pE = eciToEcef(r, theta)
  const g = ecefToGeodetic(pE)
  const vRel = sub(v, corotation(r))
  const up = normalize(c.padEcef)
  const d = sub(pE, c.padEcef)
  const dH = sub(d, scale(up, dot(d, up)))
  const hdg = inbound
  const right = cross(hdg, up)
  return {
    t, r, v, vRel, lat: g.lat, lon: g.lon,
    downrangeErr: dot(dH, hdg), crossrangeErr: dot(dH, right),
    qMax, gMax, tEntry, path, pitch,
  }
}

/** secant/bisection on the entry bank magnitude to null the downrange error */
/**
 * Bank magnitude that puts the predicted end point `aim` metres past the pad (a positive aim keeps
 * energy margin for the later phases, which can only dissipate range).
 */
export const solveBank = (c: PredContext, inp: PredInput, which: "entry" | "high", guess: number, aim = 0): { bank: number; out: PredOut } => {
  const f0 = (b: number): PredOut => predict(c, which === "entry" ? { ...inp, bankEntry: b } : { ...inp, bankHigh: b })
  const f = (b: number): PredOut => {
    const o = f0(b)
    return { ...o, downrangeErr: o.downrangeErr - aim }
  }
  const D = Math.PI / 180
  // local bracket around the guess first (cheap), coarse grid as fallback (range vs bank can be non-monotone)
  const lo0 = clamp(guess - 12 * D, 0, 85 * D)
  const hi0 = clamp(guess + 12 * D, 0, 85 * D)
  let a = lo0, b = hi0
  let oa = f(a), ob = f(b)
  if (!(oa.downrangeErr >= 0 && ob.downrangeErr <= 0)) {
    const grid = [0, 15, 30, 45, 60, 72, 85].map((x) => x * D)
    const outs = grid.map(f)
    let best = -1
    for (let i = 0; i + 1 < grid.length; i++) {
      if (outs[i].downrangeErr >= 0 && outs[i + 1].downrangeErr <= 0) {
        if (best < 0 || Math.abs(grid[i] - guess) < Math.abs(grid[best] - guess)) best = i
      }
    }
    if (best < 0) {
      // no crossing: take the bank with the smallest |miss|
      const k = outs.reduce((bi, o, i) => (Math.abs(o.downrangeErr) < Math.abs(outs[bi].downrangeErr) ? i : bi), 0)
      return { bank: grid[k], out: outs[k] }
    }
    a = grid[best]; b = grid[best + 1]; oa = outs[best]; ob = outs[best + 1]
  }
  let x = a, ox = oa
  for (let i = 0; i < 6; i++) {
    const ea = oa.downrangeErr, eb = ob.downrangeErr
    x = ea === eb ? 0.5 * (a + b) : clamp(a + ((b - a) * ea) / (ea - eb), a + 0.05 * (b - a), b - 0.05 * (b - a))
    ox = f(x)
    if (Math.abs(ox.downrangeErr) < 500) break
    if (ox.downrangeErr > 0) { a = x; oa = ox } else { b = x; ob = ox }
  }
  return { bank: x, out: ox }
}
