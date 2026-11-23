import { v3 } from "@ship/math"
import type { Phase } from "@ship/gnc"

/**
 * Flight-log geometry: the per-phase, per-DOF record of what the guidance planned and what the
 * vehicle flew, written in a form a language model can reason about directly (tables of numbers
 * with units and a few spline knots), not a binary recording.
 *
 * One `PhaseTrace` holds a phase's samples at a fixed cadence: pin position (catch frame), pin
 * velocity, body tilt, roll rate, first-running-engine gimbal, total thrust, and — where the phase
 * has one — the *plan* (the reference the controller was following) in the same coordinates. The
 * Markdown writer turns each into a table plus a knot summary; the JSON writer keeps every sample.
 */
export interface LogSample {
  readonly t: number
  readonly pin: { readonly x: number; readonly y: number; readonly z: number }
  readonly vel: { readonly x: number; readonly y: number; readonly z: number }
  /** body-axis tilt from the vertical (deg), roll rate (deg/s), attitude error (deg) */
  readonly tilt: number
  readonly rollRate: number
  readonly attErr: number
  /** first running engine's gimbals (deg) and its thrust (N) */
  readonly gp: number
  readonly gy: number
  readonly thrust: number
  /** thrust acceleration the guidance commanded (m/s², catch frame) and the plan's, if any */
  readonly u: { readonly x: number; readonly y: number; readonly z: number }
  readonly plan: { readonly x: number; readonly y: number; readonly z: number } | null
  /** header propellant (kg), coil/plasma state, arms opening (1 open) */
  readonly header: number
  readonly arms: number
}

export interface PhaseTrace {
  readonly phase: Phase
  readonly t0: number
  readonly t1: number
  readonly samples: readonly LogSample[]
  readonly events: readonly { readonly t: number; readonly msg: string }[]
}

const round = (x: number, n: number): number => {
  const k = 10 ** n
  return Math.round(x * k) / k
}

/** the phase at sample `i`: what the controller was trying to hold, per axis */
const ctrlOf = (s: LogSample): string => `(${round(s.u.x, 2)}, ${round(s.u.y, 2)}, ${round(s.u.z, 1)})`

/** downsampled spline: `n` knots of position and time, plus the velocity there */
export const splineKnots = (samples: readonly LogSample[], n = 10): { t: number; p: { x: number; y: number; z: number }; v: { x: number; y: number; z: number } }[] => {
  if (samples.length === 0) return []
  const out: { t: number; p: { x: number; y: number; z: number }; v: { x: number; y: number; z: number } }[] = []
  for (let k = 0; k < n; k++) {
    const i = Math.min(samples.length - 1, Math.round((k / Math.max(n - 1, 1)) * (samples.length - 1)))
    const s = samples[i]
    out.push({ t: round(s.t, 1), p: { x: round(s.pin.x, 1), y: round(s.pin.y, 1), z: round(s.pin.z, 1) }, v: { x: round(s.vel.x, 1), y: round(s.vel.y, 1), z: round(s.vel.z, 1) } })
  }
  return out
}

/** plan-vs-flown statistics of a phase (m): RMS, max, and the largest axis component */
export const trackingOf = (samples: readonly LogSample[]): { rms: number; max: number; axis: "x" | "y" | "z" | "-" } | null => {
  let n = 0, sum = 0, max = 0, axis: "x" | "y" | "z" | "-" = "-"
  for (const s of samples) {
    if (!s.plan) continue
    const e = v3(s.pin.x - s.plan.x, s.pin.y - s.plan.y, s.pin.z - s.plan.z)
    const m = Math.hypot(e.x, e.y, e.z)
    n++; sum += m * m
    if (m > max) { max = m; axis = Math.abs(e.x) > Math.abs(e.y) && Math.abs(e.x) > Math.abs(e.z) ? "x" : Math.abs(e.y) > Math.abs(e.z) ? "y" : "z" }
  }
  return n === 0 ? null : { rms: Math.sqrt(sum / n), max, axis }
}

/** control effort of a phase: gimbal/tilt ranges, roll, saturation, propellant */
export interface Effort {
  readonly gimbalMax: number
  readonly gimbalRms: number
  readonly tiltMax: number
  readonly rollRateMax: number
  readonly attErrMax: number
  /** fraction of samples with |gimbal| ≥ 14° (the 15° stop) or thrust ≥ 99 % of the phase maximum */
  readonly satGimbal: number
  readonly fuel: number
}

export const effortOf = (samples: readonly LogSample[]): Effort => {
  if (samples.length === 0) return { gimbalMax: 0, gimbalRms: 0, tiltMax: 0, rollRateMax: 0, attErrMax: 0, satGimbal: 0, fuel: 0 }
  let gm = 0, g2 = 0, tilt = 0, roll = 0, att = 0, sat = 0
  for (const s of samples) {
    const g = Math.hypot(s.gp, s.gy)
    gm = Math.max(gm, g); g2 += g * g
    tilt = Math.max(tilt, s.tilt); roll = Math.max(roll, Math.abs(s.rollRate)); att = Math.max(att, s.attErr)
    if (g >= 14) sat++
  }
  return {
    gimbalMax: gm, gimbalRms: Math.sqrt(g2 / samples.length), tiltMax: tilt, rollRateMax: roll, attErrMax: att,
    satGimbal: sat / samples.length, fuel: samples[0].header - samples[samples.length - 1].header,
  }
}

/** largest lateral distance of the pins from the catch axis during a phase (m) */
export const lateralOf = (samples: readonly LogSample[]): { end: number; max: number } => {
  let end = 0, max = 0
  for (const s of samples) {
    const d = Math.hypot(s.pin.x, s.pin.y)
    end = d; max = Math.max(max, d)
  }
  return { end, max }
}

/**
 * Markdown of one phase: the header (time, outcome-relevant numbers), the geometry table at a
 * chosen cadence (default 1 s, 0.2 s below 200 m), the spline knots, the plan-vs-flown and effort
 * summaries, and its events. Numbers are rounded so a reader can compare rows by eye.
 */
export const phaseMarkdown = (tr: PhaseTrace, cadence = 1): string => {
  const L: string[] = []
  // Phases far from the pad are not flown in the catch frame (the pins are megametres below the
  // rail plane there): they get a summary and their trajectory knots, not a geometry table.
  const far = tr.samples.length > 0 && Math.hypot(tr.samples[0].pin.x, tr.samples[0].pin.y, tr.samples[0].pin.z) > 50e3
  if (far) {
    L.push(`### ${tr.phase}  ${tr.t0.toFixed(1)} → ${tr.t1.toFixed(1)} s  (${(tr.t1 - tr.t0).toFixed(1)} s, ${tr.samples.length} samples)`)
    const knots = splineKnots(tr.samples, 10)
    L.push("")
    L.push("trajectory knots (t s | pin E,N,U km | v E,N,U m/s):")
    L.push("```")
    for (const k of knots) L.push(`${k.t.toFixed(1).padStart(7)} | ${(k.p.x / 1e3).toFixed(1).padStart(10)} ${(k.p.y / 1e3).toFixed(1).padStart(9)} ${(k.p.z / 1e3).toFixed(1).padStart(9)} | ${k.v.x.toFixed(0).padStart(6)} ${k.v.y.toFixed(0).padStart(6)} ${k.v.z.toFixed(0).padStart(6)}`)
    L.push("```")
    if (tr.events.length > 0) {
      L.push("")
      L.push("events:")
      for (const e of tr.events) L.push(`- ${e.t.toFixed(1)} ${e.msg}`)
    }
    return L.join("\n")
  }
  const lat = lateralOf(tr.samples)
  const eff = effortOf(tr.samples)
  const trk = trackingOf(tr.samples)
  const descents = tr.samples.filter((s, i) => i > 0 && s.pin.z > tr.samples[i - 1].pin.z + 0.2).length
  L.push(`### ${tr.phase}  ${tr.t0.toFixed(1)} → ${tr.t1.toFixed(1)} s  (${(tr.t1 - tr.t0).toFixed(1)} s, ${tr.samples.length} samples)`)
  L.push(`- lateral: end ${lat.end.toFixed(2)} m, max ${lat.max.toFixed(1)} m; descent monotonic: ${descents === 0 ? "yes" : `no (${descents} non-monotonic samples)`}`)
  L.push(`- control: gimbal max ${eff.gimbalMax.toFixed(1)}° rms ${eff.gimbalRms.toFixed(1)}° (${(eff.satGimbal * 100).toFixed(0)} % at the stop), tilt max ${eff.tiltMax.toFixed(1)}°, roll rate max ${eff.rollRateMax.toFixed(1)}°/s, attitude error max ${(eff.attErrMax * 57.3).toFixed(1)}°`)
  L.push(`- propellant: ${(eff.fuel / 1e3).toFixed(2)} t${trk ? `; plan-vs-flown: RMS ${trk.rms.toFixed(2)} m, max ${trk.max.toFixed(2)} m (axis ${trk.axis})` : "; no plan in this phase"}`)
  const knots = splineKnots(tr.samples, 10)
  if (knots.length > 0) {
    L.push("")
    L.push("spline knots (t s | pin E,N,U m | v E,N,U m/s):")
    L.push("```")
    for (const k of knots) L.push(`${k.t.toFixed(1).padStart(7)} | ${k.p.x.toFixed(1).padStart(8)} ${k.p.y.toFixed(1).padStart(8)} ${k.p.z.toFixed(1).padStart(7)} | ${k.v.x.toFixed(1).padStart(6)} ${k.v.y.toFixed(1).padStart(6)} ${k.v.z.toFixed(1).padStart(6)}`)
    L.push("```")
  }
  L.push("")
  L.push("flown geometry (t | pin E,N,U | v E,N,U | tilt° roll°/s | gimbal gp,gy° | thrust MN | command a E,N,U | plan E,N,U | header t):")
  L.push("```")
  let last = -1e9
  for (const s of tr.samples) {
    const cad = s.pin.z < 200 ? Math.min(cadence, 0.2) : cadence
    if (s.t - last < cad) continue
    last = s.t
    const pl = s.plan ? `${s.plan.x.toFixed(1).padStart(7)} ${s.plan.y.toFixed(1).padStart(7)} ${s.plan.z.toFixed(1).padStart(6)}` : "      -       -      -"
    L.push(`${s.t.toFixed(1).padStart(7)} | ${s.pin.x.toFixed(1).padStart(7)} ${s.pin.y.toFixed(1).padStart(7)} ${s.pin.z.toFixed(1).padStart(6)} | ${s.vel.x.toFixed(1).padStart(6)} ${s.vel.y.toFixed(1).padStart(6)} ${s.vel.z.toFixed(1).padStart(6)} | ${s.tilt.toFixed(1).padStart(4)} ${s.rollRate.toFixed(1).padStart(5)} | ${s.gp.toFixed(1).padStart(5)} ${s.gy.toFixed(1).padStart(5)} | ${(s.thrust / 1e6).toFixed(2).padStart(4)} | ${ctrlOf(s).padEnd(20)} | ${pl} | ${(s.header / 1e3).toFixed(1)}`)
  }
  L.push("```")
  if (tr.events.length > 0) {
    L.push("")
    L.push("events:")
    for (const e of tr.events) L.push(`- ${e.t.toFixed(1)} ${e.msg}`)
  }
  return L.join("\n")
}

/**
 * JSON form of a phase (one object per phase, everything an analysis needs). Phases flown outside
 * the catch frame keep only their knots (their pin coordinates are megametres), and the near phases
 * are decimated to at most 400 samples so the file stays a document, not a recording.
 */
export const phaseJson = (tr: PhaseTrace): Record<string, unknown> => {
  const far = tr.samples.length > 0 && Math.hypot(tr.samples[0].pin.x, tr.samples[0].pin.y, tr.samples[0].pin.z) > 50e3
  const step = Math.max(1, Math.ceil(tr.samples.length / 400))
  const kept = far ? [] : tr.samples.filter((_, i) => i % step === 0)
  return {
  phase: tr.phase, t0: tr.t0, t1: tr.t1,
  lateral: lateralOf(tr.samples), effort: effortOf(tr.samples), tracking: trackingOf(tr.samples),
  knots: splineKnots(tr.samples, 16),
  samples: kept.map((s) => ({
    t: round(s.t, 3), pin: [round(s.pin.x, 3), round(s.pin.y, 3), round(s.pin.z, 3)], vel: [round(s.vel.x, 3), round(s.vel.y, 3), round(s.vel.z, 3)],
    tilt: round(s.tilt, 2), roll: round(s.rollRate, 2), attErr: round(s.attErr, 4), gimbal: [round(s.gp, 2), round(s.gy, 2)],
    thrustMN: round(s.thrust / 1e6, 3), u: [round(s.u.x, 3), round(s.u.y, 3), round(s.u.z, 3)], plan: s.plan ? [round(s.plan.x, 2), round(s.plan.y, 2), round(s.plan.z, 2)] : null,
    header: round(s.header, 0), arms: round(s.arms, 3),
  })),
  events: tr.events,
  }
}
