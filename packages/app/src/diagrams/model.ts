/**
 * MPC diagram workspace — the pure half (no DOM, no canvas; tested in packages/app/test/diagrams.test.ts).
 *
 * One x–y diagram per degree of freedom / control of the landing guidance: what the plan asked for,
 * what was commanded and what was flown, with the constraint band around the plan. This module holds
 * the arithmetic the canvas code (./view.ts) needs:
 *   - the shared time window (default descent window, range presets, zoom, pan),
 *   - axis ticks at nice numbers and their labels,
 *   - min–max decimation of a sampled line to one bucket per pixel column,
 *   - plan-vs-flown tracking statistics,
 *   - the panel definitions and the extraction of their series from a recording.
 *
 * Gaps are NaN: a series holds NaN wherever it must not be drawn (plan and tube channels where
 * `planOn` is 0, limits where they are not set), and every consumer here skips non-finite samples.
 */
import type { ChannelKey } from "@ship/sim"

// ---------------- data source ----------------

/** the channels of one recording; `ch` never fails (an unknown channel is all NaN) */
export interface Source {
  readonly n: number
  readonly ch: (key: ChannelKey) => Float64Array
}

/** adapter for a `Recording` (`index` is `CH` from @ship/sim, injected so this module stays runtime-free) */
export const recordingSource = (rec: { readonly n: number; readonly data: readonly Float64Array[] }, index: Readonly<Record<ChannelKey, number>>): Source => {
  let missing: Float64Array | null = null
  return {
    n: rec.n,
    ch: (key) => {
      const c = rec.data[index[key]]
      if (c && c.length >= rec.n) return c
      // a recording written before the channel existed: draw nothing instead of throwing
      missing ??= new Float64Array(rec.n).fill(NaN)
      return missing
    },
  }
}

export const EMPTY_SOURCE: Source = { n: 0, ch: () => new Float64Array(0) }

// ---------------- time window ----------------

export interface TimeWindow { readonly t0: number; readonly t1: number }
export interface PhaseStart { readonly phase: string; readonly t: number }

/** shortest time span the axis can be zoomed to, s */
export const MIN_SPAN = 0.05

export const phaseStart = (phaseStarts: readonly PhaseStart[], phase: string): number | undefined =>
  phaseStarts.find((p) => p.phase === phase)?.t

/**
 * Default window = the descent: 20 s before the flip to the end of the recording. The pin offsets
 * are megametres before that, so a wider default would flatten every diagram. Without a flip phase
 * (a run lost earlier) it is the last 80 s.
 */
export const defaultWindow = (phaseStarts: readonly PhaseStart[], tStart: number, tEnd: number): TimeWindow => {
  const flip = phaseStart(phaseStarts, "flip")
  if (flip !== undefined && Number.isFinite(flip) && flip < tEnd) return { t0: Math.max(tStart, flip - 20), t1: tEnd }
  return { t0: Math.max(tStart, tEnd - 80), t1: tEnd }
}

export type RangeId = "belly" | "flip" | "last15" | "full"
export const RANGES: readonly { readonly id: RangeId; readonly label: string; readonly title: string }[] = [
  { id: "belly", label: "Belly flop → catch", title: "From the start of the belly flop to the end of the recording" },
  { id: "flip", label: "Flip → catch", title: "From 20 s before the flip to the end of the recording (default)" },
  { id: "last15", label: "Last 15 s", title: "The last 15 s of the recording" },
  { id: "full", label: "Full", title: "The whole recording" },
]

/** the window of a range button, or null when the recording has no such range (button disabled) */
export const rangeWindow = (id: RangeId, phaseStarts: readonly PhaseStart[], tStart: number, tEnd: number): TimeWindow | null => {
  if (!(tEnd > tStart)) return null
  if (id === "full") return { t0: tStart, t1: tEnd }
  if (id === "last15") return { t0: Math.max(tStart, tEnd - 15), t1: tEnd }
  if (id === "flip") return defaultWindow(phaseStarts, tStart, tEnd)
  const belly = phaseStart(phaseStarts, "bellyFlop")
  return belly !== undefined && belly < tEnd ? { t0: Math.max(tStart, belly), t1: tEnd } : null
}

/** keep the window inside `bounds` (shifting before shrinking) and no shorter than `minSpan` */
export const clampWindow = (w: TimeWindow, bounds: TimeWindow, minSpan = MIN_SPAN): TimeWindow => {
  const full = Math.max(bounds.t1 - bounds.t0, 0)
  const span = Math.min(Math.max(w.t1 - w.t0, Math.min(minSpan, full)), full)
  const t0 = Math.min(Math.max(w.t0, bounds.t0), bounds.t1 - span)
  return { t0, t1: t0 + span }
}

/** scale the span by `factor` (< 1 zooms in) keeping the time `anchor` at the same screen position */
export const zoomWindow = (w: TimeWindow, anchor: number, factor: number, bounds: TimeWindow, minSpan = MIN_SPAN): TimeWindow => {
  const span = w.t1 - w.t0
  const full = Math.max(bounds.t1 - bounds.t0, 0)
  const next = Math.min(Math.max(span * factor, Math.min(minSpan, full)), full)
  const f = span > 0 ? Math.min(1, Math.max(0, (anchor - w.t0) / span)) : 0.5
  return clampWindow({ t0: anchor - f * next, t1: anchor + (1 - f) * next }, bounds, minSpan)
}

/** shift the window by `dt` seconds, stopping at the bounds */
export const panWindow = (w: TimeWindow, dt: number, bounds: TimeWindow): TimeWindow =>
  clampWindow({ t0: w.t0 + dt, t1: w.t1 + dt }, bounds, 0)

export const sameWindow = (a: TimeWindow, b: TimeWindow | null): boolean =>
  b !== null && Math.abs(a.t0 - b.t0) < 1e-6 && Math.abs(a.t1 - b.t1) < 1e-6

// ---------------- sample lookup ----------------

/** first index i in [0, n] with ts[i] ≥ t (ts ascending) */
export const lowerBound = (ts: ArrayLike<number>, n: number, t: number): number => {
  let lo = 0, hi = n
  while (lo < hi) {
    const m = (lo + hi) >> 1
    if (ts[m] < t) lo = m + 1
    else hi = m
  }
  return lo
}

/** inclusive index range; empty when i1 < i0 */
export interface IndexRange { readonly i0: number; readonly i1: number }

/** the samples with t0 ≤ ts[i] ≤ t1 (used for autoscale and statistics) */
export const visibleRange = (ts: ArrayLike<number>, n: number, w: TimeWindow): IndexRange => {
  const i0 = lowerBound(ts, n, w.t0)
  let i1 = lowerBound(ts, n, w.t1)
  if (i1 >= n || ts[i1] > w.t1) i1--
  return { i0, i1 }
}

/** one more sample on each side, so a drawn line enters and leaves the frame instead of ending inside it */
export const widen = (r: IndexRange, n: number): IndexRange =>
  n <= 0 ? { i0: 0, i1: -1 } : { i0: Math.max(0, Math.min(r.i0, n - 1) - 1), i1: Math.min(n - 1, Math.max(r.i1, 0) + 1) }

/** linear interpolation of `ys` at time `t`; next to a gap it is the nearer sample (NaN inside a gap) */
export const sampleAt = (ts: ArrayLike<number>, ys: ArrayLike<number>, n: number, t: number): number => {
  if (n <= 0) return NaN
  if (n === 1 || t <= ts[0]) return ys[0]
  if (t >= ts[n - 1]) return ys[n - 1]
  const hi = Math.max(1, lowerBound(ts, n, t)), lo = hi - 1
  const dt = ts[hi] - ts[lo]
  const f = dt > 0 ? (t - ts[lo]) / dt : 0
  const a = ys[lo], b = ys[hi]
  if (Number.isFinite(a) && Number.isFinite(b)) return a + (b - a) * f
  return f < 0.5 ? a : b
}

// ---------------- value ranges (autoscale) ----------------

export type Range = readonly [number, number]

/** min and max of the finite samples in [i0, i1], or null when there are none */
export const finiteRange = (ys: ArrayLike<number>, r: IndexRange): Range | null => {
  let lo = Infinity, hi = -Infinity
  for (let i = Math.max(0, r.i0); i <= r.i1; i++) {
    const y = ys[i]
    if (!Number.isFinite(y)) continue
    if (y < lo) lo = y
    if (y > hi) hi = y
  }
  return lo <= hi ? [lo, hi] : null
}

export const unionRange = (a: Range | null, b: Range | null): Range | null =>
  a === null ? b : b === null ? a : [Math.min(a[0], b[0]), Math.max(a[1], b[1])]

/**
 * Axis range for the data range: `pad` (8 %) of the span added on both sides. A flat line gets a
 * span from its magnitude (never zero); no data at all gives [-1, 1].
 */
export const padRange = (r: Range | null, pad = 0.08): Range => {
  if (r === null || !Number.isFinite(r[0]) || !Number.isFinite(r[1])) return [-1, 1]
  const lo = Math.min(r[0], r[1]), hi = Math.max(r[0], r[1])
  const span = hi - lo
  if (span <= Math.max(Math.abs(lo), Math.abs(hi)) * 1e-9) {
    const half = Math.max(Math.abs(lo) * 0.05, 0.5)
    return [lo - half, hi + half]
  }
  return [lo - span * pad, hi + span * pad]
}

// ---------------- ticks ----------------

/** the 1–2–5 step nearest above span/target */
export const niceStep = (span: number, target: number): number => {
  if (!(span > 0) || !Number.isFinite(span)) return 1
  const raw = span / Math.max(1, target)
  const p = Math.pow(10, Math.floor(Math.log10(raw)))
  const m = raw / p
  return (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10) * p
}

export interface Ticks { readonly step: number; readonly ticks: readonly number[] }

/** tick values at multiples of a 1–2–5 step inside [lo, hi], about `target` of them */
export const niceTicks = (lo: number, hi: number, target: number): Ticks => {
  if (!Number.isFinite(lo) || !Number.isFinite(hi) || !(hi > lo)) return { step: 1, ticks: [] }
  const step = niceStep(hi - lo, target)
  const eps = step * 1e-9
  const ticks: number[] = []
  for (let k = Math.ceil((lo - eps) / step); k * step <= hi + eps && ticks.length < 200; k++) ticks.push(k * step === 0 ? 0 : k * step)
  return { step, ticks }
}

/** label for a tick of an axis with this step: as many decimals as the step needs, k / M above 10⁴ */
export const tickLabel = (v: number, step: number): string => {
  const [div, suffix] = step >= 1e6 ? [1e6, "M"] : step >= 1e4 ? [1e3, "k"] : [1, ""]
  const s = step / div
  const digits = Math.max(0, Math.min(8, -Math.floor(Math.log10(s) + 1e-9)))
  const text = (v / div).toFixed(digits)
  return (Number(text) === 0 ? (0).toFixed(digits) : text).replace("-", "−") + suffix
}

const TIME_STEPS = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600] as const

/** time-axis ticks at clock-friendly steps (…, 5, 10, 15, 30, 60 s, …) */
export const timeTicks = (t0: number, t1: number, target: number): Ticks => {
  if (!Number.isFinite(t0) || !Number.isFinite(t1) || !(t1 > t0)) return { step: 1, ticks: [] }
  const raw = (t1 - t0) / Math.max(1, target)
  const step = TIME_STEPS.find((s) => s >= raw) ?? niceStep(t1 - t0, target)
  const eps = step * 1e-9
  const ticks: number[] = []
  for (let k = Math.ceil((t0 - eps) / step); k * step <= t1 + eps && ticks.length < 200; k++) ticks.push(k * step)
  return { step, ticks }
}

/** mission time as m:ss, with the decimals a tick step of `step` seconds needs */
export const formatClock = (t: number, step = 1): string => {
  const digits = step < 0.1 ? 2 : step < 1 ? 1 : 0
  const k = Math.pow(10, digits)
  const total = Math.round(Math.abs(t) * k) / k
  const m = Math.floor(total / 60)
  const s = total - m * 60
  return `${t < 0 && total > 0 ? "−" : ""}${m}:${s.toFixed(digits).padStart(digits ? digits + 3 : 2, "0")}`
}

/** readout value: about four significant figures, a real minus sign, "—" for a gap */
export const formatValue = (v: number): string => {
  if (!Number.isFinite(v)) return "—"
  const a = Math.abs(v)
  const digits = a >= 1000 ? 0 : a >= 100 ? 1 : a >= 1 ? 2 : 3
  const text = v.toFixed(digits)
  return (Number(text) === 0 ? (0).toFixed(digits) : text).replace("-", "−")
}

// ---------------- decimation ----------------

/** a polyline in data coordinates; a NaN y is a pen-up (gap) */
export interface Poly { readonly x: Float64Array; readonly y: Float64Array; readonly n: number }

const poly = (x: number[], y: number[]): Poly => {
  // no pen-up at the end
  let n = x.length
  while (n > 0 && Number.isNaN(y[n - 1])) n--
  return { x: Float64Array.from(x.slice(0, n)), y: Float64Array.from(y.slice(0, n)), n }
}

/**
 * Min–max decimation for drawing: the samples [i0, i1] of (xs, ys), xs ascending, are bucketed into
 * `cols` pixel columns over [x0, x1]; each column contributes its first, lowest, highest and last
 * sample in their original order (at most four points), so every peak a full-resolution line would
 * show survives and 30 000 samples cost ≤ 4·cols line segments. Samples outside [x0, x1] fall into
 * one bucket on each side (the line still enters and leaves the frame). A non-finite sample ends the
 * current run: the output carries one NaN there, which the drawing code treats as a pen-up.
 */
export const decimate = (xs: ArrayLike<number>, ys: ArrayLike<number>, i0: number, i1: number, x0: number, x1: number, cols: number): Poly => {
  const ox: number[] = [], oy: number[] = []
  if (i1 < i0 || !(x1 > x0) || !(cols >= 1)) return poly(ox, oy)
  const k = cols / (x1 - x0)
  let open = false, col = 0, first = 0, last = 0, lo = 0, hi = 0
  const put = (i: number): void => { ox.push(xs[i]); oy.push(ys[i]) }
  const flush = (): void => {
    if (!open) return
    open = false
    put(first)
    const a = Math.min(lo, hi), b = Math.max(lo, hi)
    if (a !== first && a !== last) put(a)
    if (b !== a && b !== first && b !== last) put(b)
    if (last !== first) put(last)
  }
  for (let i = Math.max(0, i0); i <= i1; i++) {
    const x = xs[i], y = ys[i]
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      flush()
      if (oy.length > 0 && !Number.isNaN(oy[oy.length - 1])) { ox.push(NaN); oy.push(NaN) }
      continue
    }
    const c = Math.max(-1, Math.min(cols, Math.floor((x - x0) * k)))
    if (!open || c !== col) {
      flush()
      open = true; col = c; first = last = lo = hi = i
    } else {
      last = i
      if (y < ys[lo]) lo = i
      if (y > ys[hi]) hi = i
    }
  }
  flush()
  return poly(ox, oy)
}

/**
 * Thinning for an x–y curve (not monotonic in x, so column buckets do not apply): a sample is kept
 * when it is at least `tolPx` pixels from the last kept one (`kx`, `ky` = pixels per data unit), plus
 * the last sample of every run. Non-finite samples are pen-ups, as in `decimate`.
 */
export const thinPath = (xs: ArrayLike<number>, ys: ArrayLike<number>, i0: number, i1: number, kx: number, ky: number, tolPx = 0.75): Poly => {
  const ox: number[] = [], oy: number[] = []
  let open = false, lx = 0, ly = 0, pending = -1
  const close = (): void => {
    if (pending >= 0) { ox.push(xs[pending]); oy.push(ys[pending]); pending = -1 }
    open = false
  }
  for (let i = Math.max(0, i0); i <= i1; i++) {
    const x = xs[i], y = ys[i]
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      if (open) close()
      if (oy.length > 0 && !Number.isNaN(oy[oy.length - 1])) { ox.push(NaN); oy.push(NaN) }
      continue
    }
    if (!open || Math.hypot((x - lx) * kx, (y - ly) * ky) >= tolPx) {
      ox.push(x); oy.push(y)
      open = true; lx = x; ly = y; pending = -1
    } else pending = i
  }
  if (open) close()
  return poly(ox, oy)
}

/** the runs of a polyline between pen-ups, as [start, end) index pairs */
export const polyRuns = (p: Poly): readonly (readonly [number, number])[] => {
  const runs: [number, number][] = []
  let s = -1
  for (let i = 0; i < p.n; i++) {
    const gap = Number.isNaN(p.y[i])
    if (!gap && s < 0) s = i
    if (gap && s >= 0) { runs.push([s, i]); s = -1 }
  }
  if (s >= 0) runs.push([s, p.n])
  return runs
}

/** sample of (xs, ys) in [i0, i1] nearest to the point (x, y) in pixels (`kx`, `ky` = pixels per unit) */
export const nearestSample = (xs: ArrayLike<number>, ys: ArrayLike<number>, r: IndexRange, x: number, y: number, kx: number, ky: number): { readonly i: number; readonly dist: number } => {
  let best = -1, d2 = Infinity
  for (let i = Math.max(0, r.i0); i <= r.i1; i++) {
    const dx = (xs[i] - x) * kx, dy = (ys[i] - y) * ky
    const d = dx * dx + dy * dy
    if (d < d2) { d2 = d; best = i }
  }
  return { i: best, dist: Math.sqrt(d2) }
}

// ---------------- tracking statistics ----------------

export interface TrackStats {
  /** time-weighted RMS of flown − plan */
  readonly rms: number
  /** largest |flown − plan| and the time it occurred */
  readonly max: number
  readonly tMax: number
  /** samples that entered the statistic */
  readonly n: number
}

/**
 * Tracking quality over the samples [i0, i1] where both the plan and the flown value are finite
 * (the plan series is NaN where no plan exists). The RMS is weighted by the time each sample stands
 * for, because the recording is sampled more densely where things happen — an unweighted mean would
 * count the last seconds many times over. Null when no sample has a plan.
 */
export const trackingStats = (ts: ArrayLike<number>, flown: ArrayLike<number>, plan: ArrayLike<number>, r: IndexRange): TrackStats | null => {
  let sw = 0, swe = 0, se = 0, n = 0, max = 0, tMax = NaN
  for (let i = Math.max(0, r.i0); i <= r.i1; i++) {
    const e = flown[i] - plan[i]
    if (!Number.isFinite(e)) continue
    const w = Math.max(0, (ts[Math.min(i + 1, r.i1)] - ts[Math.max(i - 1, r.i0)]) / 2)
    sw += w; swe += w * e * e; se += e * e; n++
    if (Math.abs(e) >= max) { max = Math.abs(e); tMax = ts[i] }
  }
  if (n === 0) return null
  return { rms: Math.sqrt(sw > 0 ? swe / sw : se / n), max, tMax, n }
}

/** true when any sample in the range carries a plan (`on` = the planOn channel) */
export const hasPlan = (on: ArrayLike<number>, r: IndexRange): boolean => {
  for (let i = Math.max(0, r.i0); i <= r.i1; i++) if (on[i] > 0.5) return true
  return false
}

// ---------------- series extraction ----------------

const DEG = 180 / Math.PI

/** channel × scale (the channel itself when the scale is 1) */
export const scaled = (ys: Float64Array, k: number): Float64Array => (k === 1 ? ys : ys.map((y) => y * k))

/** `ys` where `on` is set, NaN elsewhere: plan channels are meaningless (0) without a plan */
export const gated = (ys: ArrayLike<number>, on: ArrayLike<number>): Float64Array => {
  const out = new Float64Array(ys.length)
  for (let i = 0; i < out.length; i++) out[i] = on[i] > 0.5 ? ys[i] : NaN
  return out
}

/** centre + sign·half where `half` > 0 and the centre is finite, NaN elsewhere (band and limit edges) */
export const offsetBy = (centre: ArrayLike<number>, half: ArrayLike<number>, sign: number): Float64Array => {
  const out = new Float64Array(centre.length)
  for (let i = 0; i < out.length; i++) out[i] = half[i] > 0 && Number.isFinite(half[i]) ? centre[i] + sign * half[i] : NaN
  return out
}

/** `ys` where it is set (non-zero), NaN elsewhere: a limit channel reads 0 while it does not apply */
export const whereSet = (ys: ArrayLike<number>): Float64Array => {
  const out = new Float64Array(ys.length)
  for (let i = 0; i < out.length; i++) out[i] = ys[i] !== 0 && Number.isFinite(ys[i]) ? ys[i] : NaN
  return out
}

export const hypot2 = (a: ArrayLike<number>, b: ArrayLike<number>): Float64Array => {
  const out = new Float64Array(a.length)
  for (let i = 0; i < out.length; i++) out[i] = Math.hypot(a[i], b[i])
  return out
}

// ---------------- panels ----------------

/** colour convention, the same in every panel and in the toolbar legend */
export const COLORS = {
  plan: "#eef4fa", plan2: "#d9ccff",
  flown: "#4fe6ff", flown2: "#b48cff", flown3: "#ff7ab8",
  cmd: "#ffb347",
  band: "rgba(90,240,160,0.17)", bandEdge: "rgba(90,240,160,0.55)", bandText: "#7cf0b0",
  hard: "#ff5a52",
  playhead: "#ff9a3c",
} as const

/** plan = pale dashed, flown = solid, command = amber solid, hard = red dashed limit */
export type Role = "plan" | "flown" | "cmd" | "hard"

export interface Line {
  readonly label: string
  readonly unit: string
  readonly role: Role
  readonly color: string
  /** scaled samples, NaN where the line must not be drawn */
  readonly y: Float64Array
}

/** the recoverable band around the plan: filled between `lo` and `hi`; `half` is its half-width */
export interface Band { readonly label: string; readonly unit: string; readonly lo: Float64Array; readonly hi: Float64Array; readonly half: Float64Array }

export interface TimePanel {
  readonly kind: "time"
  readonly id: string
  readonly title: string
  readonly unit: string
  readonly lines: readonly Line[]
  readonly band?: Band
  /** constant hard limits (e.g. ±15° gimbal travel), in the panel unit */
  readonly limits?: readonly number[]
  /** the pair the header statistic compares */
  readonly track?: { readonly flown: Float64Array; readonly plan: Float64Array; readonly unit: string }
  /** the panel has plan series: show "no plan recorded" when the window holds none */
  readonly planned: boolean
}

export interface Curve { readonly label: string; readonly role: Role; readonly color: string; readonly x: Float64Array; readonly y: Float64Array }

export interface XYPanel {
  readonly kind: "xy"
  readonly id: string
  readonly title: string
  readonly unit: string
  readonly xLabel: string
  readonly yLabel: string
  /** same scale on both axes (a ground track must not be stretched) */
  readonly equal: boolean
  readonly curves: readonly Curve[]
  /** draw the catch point at (0, 0) and keep it in view */
  readonly catchPoint: boolean
  /** circles at the playhead around the plan point: recoverable radius and hard limit */
  readonly tube?: { readonly x: Float64Array; readonly y: Float64Array; readonly r: Float64Array; readonly rHard: Float64Array; /** centre of the hard corridor across the rails (arm-pair centre, m E) */ readonly cHard: Float64Array }
  readonly planned: boolean
}

export type Panel = TimePanel | XYPanel

export interface PanelSet {
  readonly n: number
  readonly t: Float64Array
  /** the planOn channel */
  readonly on: Float64Array
  readonly panels: readonly Panel[]
}

/** gimbal travel of the sea-level Raptors, deg (limit lines of the gimbal panels) */
export const GIMBAL_LIMIT_DEG = 15

/**
 * The fourteen diagrams, with their series extracted from the recording. Plan and tube series are
 * gated by `planOn`; angles are converted to degrees. The lateral band is plan ± tubeLat (what the
 * guidance can still recover around its reference). The hard lateral limit is the tower's: the
 * corridor between the arms, arm-pair centre (`armX`) ± tubeHard, across the rails (E) only and only
 * while the hull is level with the arms — it is a property of the structure, not of the plan.
 */
export const buildPanels = (src: Source): PanelSet => {
  const on = src.ch("planOn")
  const c = (key: ChannelKey, k = 1): Float64Array => scaled(src.ch(key), k)
  const p = (key: ChannelKey, k = 1): Float64Array => gated(c(key, k), on)
  const line = (label: string, unit: string, role: Role, y: Float64Array, color: string = COLORS[role]): Line => ({ label, unit, role, color, y })

  const pinX = c("pinX"), pinY = c("pinY"), pinZ = c("pinZ")
  const planX = p("planX"), planY = p("planY"), planZ = p("planZ")
  const tubeLat = p("tubeLat"), tubeHard = c("tubeHard"), armX = c("armX")

  const lateral = (id: string, title: string, plan: Float64Array, flown: Float64Array, corridor: boolean): TimePanel => ({
    kind: "time", id, title, unit: "m", planned: true,
    lines: [
      ...(corridor ? [line("arm +", "m", "hard", offsetBy(armX, tubeHard, 1)), line("arm −", "m", "hard", offsetBy(armX, tubeHard, -1))] : []),
      line("flown", "m", "flown", flown),
      line("plan", "m", "plan", plan),
    ],
    band: { label: "band ±", unit: "m", lo: offsetBy(plan, tubeLat, -1), hi: offsetBy(plan, tubeLat, 1), half: tubeLat },
    track: { flown, plan, unit: "m" },
  })
  const tilt = (id: string, title: string, planKey: ChannelKey, cmdKey: ChannelKey, flownKey: ChannelKey): TimePanel => {
    const plan = p(planKey, DEG), flown = c(flownKey, DEG)
    return {
      kind: "time", id, title, unit: "°", planned: true,
      lines: [line("command", "°", "cmd", c(cmdKey, DEG)), line("flown", "°", "flown", flown), line("plan", "°", "plan", plan)],
      track: { flown, plan, unit: "°" },
    }
  }
  const engines = (id: string, title: string, unit: string, keys: readonly [ChannelKey, ChannelKey, ChannelKey], k: number, limits?: readonly number[]): TimePanel => ({
    kind: "time", id, title, unit, planned: false, limits,
    lines: [
      line("SL1", unit, "flown", c(keys[0], k), COLORS.flown),
      line("SL2", unit, "flown", c(keys[1], k), COLORS.flown2),
      line("SL3", unit, "flown", c(keys[2], k), COLORS.flown3),
    ],
  })

  const vu = c("vu"), planVz = p("planVz")
  const accT = c("accT"), planAcc = p("planAcc")
  const panels: Panel[] = [
    lateral("pinE", "Pins E · across the rails", planX, pinX, true),
    lateral("pinN", "Pins N · along the rails", planY, pinY, false),
    {
      kind: "time", id: "height", title: "Height above the rails", unit: "m", planned: true,
      lines: [line("flown", "m", "flown", pinZ), line("plan", "m", "plan", planZ)],
      track: { flown: pinZ, plan: planZ, unit: "m" },
    },
    {
      kind: "time", id: "vz", title: "Vertical speed", unit: "m/s", planned: true,
      lines: [line("sink limit", "m/s", "hard", whereSet(c("vzLimit"))), line("flown", "m/s", "flown", vu), line("plan", "m/s", "plan", planVz)],
      track: { flown: vu, plan: planVz, unit: "m/s" },
    },
    {
      kind: "time", id: "vlat", title: "Lateral speed E / N", unit: "m/s", planned: true,
      lines: [
        line("flown E", "m/s", "flown", c("pinVx"), COLORS.flown), line("flown N", "m/s", "flown", c("pinVy"), COLORS.flown2),
        line("plan E", "m/s", "plan", p("planVx"), COLORS.plan), line("plan N", "m/s", "plan", p("planVy"), COLORS.plan2),
      ],
    },
    tilt("tiltE", "Tilt E", "planTiltE", "cmdTiltE", "tiltE"),
    tilt("tiltN", "Tilt N", "planTiltN", "cmdTiltN", "tiltN"),
    {
      kind: "time", id: "roll", title: "Roll · pin-axis azimuth error and rate", unit: "° · °/s", planned: false,
      lines: [line("roll", "°", "flown", c("rollAng", DEG), COLORS.flown), line("rate", "°/s", "flown", c("wx", DEG), COLORS.flown2)],
    },
    {
      kind: "time", id: "acc", title: "Thrust acceleration", unit: "m/s²", planned: true,
      lines: [line("command", "m/s²", "cmd", c("accCmd")), line("flown", "m/s²", "flown", accT), line("plan", "m/s²", "plan", planAcc)],
      track: { flown: accT, plan: planAcc, unit: "m/s²" },
    },
    engines("throttle", "Throttle per engine", "", ["thr1", "thr2", "thr3"], 1),
    engines("gimbalP", "Gimbal pitch per engine", "°", ["gp1", "gp2", "gp3"], DEG, [-GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG]),
    engines("gimbalY", "Gimbal yaw per engine", "°", ["gy1", "gy2", "gy3"], DEG, [-GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG]),
    {
      kind: "xy", id: "track", title: "Ground track · pins N vs pins E", unit: "m", xLabel: "pins E", yLabel: "pins N", equal: true, planned: true, catchPoint: true,
      curves: [
        { label: "flown", role: "flown", color: COLORS.flown, x: pinX, y: pinY },
        { label: "plan", role: "plan", color: COLORS.plan, x: planX, y: planY },
      ],
      tube: { x: planX, y: planY, r: tubeLat, rHard: tubeHard, cHard: armX },
    },
    {
      kind: "xy", id: "profile", title: "Side profile · height vs lateral distance", unit: "m", xLabel: "lateral distance", yLabel: "height", equal: false, planned: true, catchPoint: true,
      curves: [
        { label: "flown", role: "flown", color: COLORS.flown, x: hypot2(pinX, pinY), y: pinZ },
        { label: "plan", role: "plan", color: COLORS.plan, x: gated(hypot2(c("planX"), c("planY")), on), y: planZ },
      ],
    },
  ]
  return { n: src.n, t: src.ch("t"), on, panels }
}

/** y-axis range of a time panel over the visible samples; hard limits count only when `fitLimits` */
export const timePanelRange = (panel: TimePanel, r: IndexRange, fitLimits: boolean): Range => {
  let range: Range | null = null
  for (const l of panel.lines) if (l.role !== "hard" || fitLimits) range = unionRange(range, finiteRange(l.y, r))
  if (panel.band) range = unionRange(unionRange(range, finiteRange(panel.band.lo, r)), finiteRange(panel.band.hi, r))
  // a constant limit never sets the scale on its own: without data the panel stays [-1, 1]
  if (fitLimits && range !== null && panel.limits?.length) range = unionRange(range, [Math.min(...panel.limits), Math.max(...panel.limits)])
  return padRange(range)
}

/** x and y ranges of an x–y panel over the visible samples (the catch point at the origin stays in view) */
export const xyPanelRange = (panel: XYPanel, r: IndexRange): { readonly x: Range; readonly y: Range } => {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity
  for (const c of panel.curves) {
    for (let i = Math.max(0, r.i0); i <= r.i1; i++) {
      const a = c.x[i], b = c.y[i]
      // a point counts only where both coordinates are drawn
      if (!Number.isFinite(a) || !Number.isFinite(b)) continue
      if (a < x0) x0 = a
      if (a > x1) x1 = a
      if (b < y0) y0 = b
      if (b > y1) y1 = b
    }
  }
  if (x0 > x1) return { x: padRange(null), y: padRange(null) }
  if (panel.catchPoint) { x0 = Math.min(x0, 0); x1 = Math.max(x1, 0); y0 = Math.min(y0, 0); y1 = Math.max(y1, 0) }
  return { x: padRange([x0, x1]), y: padRange([y0, y1]) }
}

/** widen one of the two ranges so both axes have the same pixels per unit in a w × h plot area */
export const equalAspect = (x: Range, y: Range, w: number, h: number): { readonly x: Range; readonly y: Range } => {
  const sx = x[1] - x[0], sy = y[1] - y[0]
  if (!(w > 0) || !(h > 0) || !(sx > 0) || !(sy > 0)) return { x, y }
  const unitsPerPx = Math.max(sx / w, sy / h)
  const grow = (r: Range, span: number): Range => { const mid = (r[0] + r[1]) / 2; return [mid - span / 2, mid + span / 2] }
  return { x: grow(x, unitsPerPx * w), y: grow(y, unitsPerPx * h) }
}

// ---------------- grid ----------------

export interface Grid { readonly cols: number; readonly rows: number; readonly cellH: number; readonly scroll: boolean }

/**
 * Columns × rows for `count` panels in a w × h area. All panels fit without scrolling when a layout
 * with cells at least `minH` high (and `minW` wide) exists — the one whose cell shape is closest to
 * `aspect` (width / height) wins. Otherwise the rows keep `minH` and the grid scrolls.
 */
export const gridLayout = (count: number, w: number, h: number, minW = 250, minH = 150, aspect = 2.3, gap = 6): Grid => {
  if (count <= 0) return { cols: 1, rows: 0, cellH: minH, scroll: false }
  const maxCols = Math.max(1, Math.min(count, Math.floor((w + gap) / (minW + gap))))
  let best: Grid | null = null, score = Infinity
  for (let cols = 1; cols <= maxCols; cols++) {
    const rows = Math.ceil(count / cols)
    const cellW = (w - gap * (cols - 1)) / cols, cellH = (h - gap * (rows - 1)) / rows
    if (cellH < minH) continue
    const s = Math.abs(Math.log(cellW / cellH / aspect))
    if (s < score) { score = s; best = { cols, rows, cellH, scroll: false } }
  }
  return best ?? { cols: maxCols, rows: Math.ceil(count / maxCols), cellH: minH, scroll: true }
}
