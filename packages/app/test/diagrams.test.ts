/**
 * MPC diagram workspace, pure half (app/src/diagrams/model.ts): the time window, axis ticks, min–max
 * decimation, plan-vs-flown statistics and the series extraction. Expectations:
 *  - the default window is the descent (flip − 20 s → end), because the pin offsets are megametres
 *    before it; without a flip phase it is the last 80 s;
 *  - ticks sit on multiples of a 1–2–5 step (clock steps on the time axis);
 *  - decimation to one bucket per pixel column keeps every column's minimum and maximum, so a peak
 *    of the full-resolution line is never lost, and costs at most four points per column;
 *  - tracking statistics use only the samples that carry a plan and weight them by time;
 *  - plan and tube series are gaps (NaN) wherever `planOn` is 0, and a recording without any plan
 *    still yields the flown lines.
 */
import { describe, expect, test } from "bun:test"
import type { ChannelKey } from "@ship/sim"
import {
  buildPanels, clampWindow, decimate, defaultWindow, equalAspect, finiteRange, formatClock, formatValue, gated, gridLayout, hasPlan,
  lowerBound, nearestSample, niceStep, niceTicks, offsetBy, padRange, panWindow, polyRuns, rangeWindow, recordingSource, sampleAt,
  thinPath, tickLabel, timePanelRange, timeTicks, trackingStats, visibleRange, whereSet, widen, xyPanelRange, zoomWindow,
  type Source, type TimePanel, type XYPanel,
} from "../src/diagrams/model"

const PHASES = [
  { phase: "deorbitAlign", t: 0 }, { phase: "entry", t: 900 }, { phase: "bellyFlop", t: 2950 },
  { phase: "flip", t: 3040 }, { phase: "landing", t: 3046 }, { phase: "terminal", t: 3080 }, { phase: "caught", t: 3091 },
]
const linspace = (a: number, b: number, n: number): Float64Array => Float64Array.from({ length: n }, (_, i) => a + ((b - a) * i) / (n - 1))
/** a source with the given channels; every other channel reads 0, as an unwired channel does */
const source = (n: number, cols: Partial<Record<ChannelKey, ArrayLike<number>>>): Source => {
  const zero = new Float64Array(n)
  return { n, ch: (k) => (cols[k] ? Float64Array.from(cols[k]) : zero) }
}

describe("time window", () => {
  test("default = 20 s before the flip to the end of the recording", () => {
    expect(defaultWindow(PHASES, 0, 3095)).toEqual({ t0: 3020, t1: 3095 })
  })

  test("without a flip phase the default is the last 80 s", () => {
    expect(defaultWindow(PHASES.filter((p) => p.phase !== "flip"), 0, 3095)).toEqual({ t0: 3015, t1: 3095 })
    expect(defaultWindow([], 0, 50)).toEqual({ t0: 0, t1: 50 })
  })

  test("the default never starts before the recording", () => {
    expect(defaultWindow([{ phase: "flip", t: 8 }], 0, 60)).toEqual({ t0: 0, t1: 60 })
  })

  test("range buttons", () => {
    expect(rangeWindow("belly", PHASES, 0, 3095)).toEqual({ t0: 2950, t1: 3095 })
    expect(rangeWindow("flip", PHASES, 0, 3095)).toEqual({ t0: 3020, t1: 3095 })
    expect(rangeWindow("last15", PHASES, 0, 3095)).toEqual({ t0: 3080, t1: 3095 })
    expect(rangeWindow("full", PHASES, 0, 3095)).toEqual({ t0: 0, t1: 3095 })
    // a run lost before the belly flop has no such range
    expect(rangeWindow("belly", PHASES.slice(0, 2), 0, 1200)).toBeNull()
  })

  test("zoom keeps the time under the cursor in place", () => {
    const w = zoomWindow({ t0: 100, t1: 200 }, 125, 0.5, { t0: 0, t1: 1000 })
    expect(w.t1 - w.t0).toBeCloseTo(50, 9)
    // 125 was a quarter of the way across and still is
    expect((125 - w.t0) / (w.t1 - w.t0)).toBeCloseTo(0.25, 9)
  })

  test("zoom stops at the bounds and at the shortest span", () => {
    const bounds = { t0: 0, t1: 1000 }
    expect(zoomWindow({ t0: 100, t1: 900 }, 500, 10, bounds)).toEqual(bounds)
    const tiny = zoomWindow({ t0: 100, t1: 100.1 }, 100.05, 1e-6, bounds)
    expect(tiny.t1 - tiny.t0).toBeCloseTo(0.05, 9)
  })

  test("pan keeps the span and stops at the bounds", () => {
    const bounds = { t0: 0, t1: 1000 }
    expect(panWindow({ t0: 100, t1: 200 }, 50, bounds)).toEqual({ t0: 150, t1: 250 })
    expect(panWindow({ t0: 100, t1: 200 }, -500, bounds)).toEqual({ t0: 0, t1: 100 })
    expect(panWindow({ t0: 100, t1: 200 }, 5000, bounds)).toEqual({ t0: 900, t1: 1000 })
    expect(clampWindow({ t0: -50, t1: 5000 }, bounds)).toEqual(bounds)
  })
})

describe("ticks", () => {
  test("steps are 1–2–5 multiples of a power of ten", () => {
    expect(niceStep(10, 5)).toBe(2)
    expect(niceStep(88, 5)).toBe(20)
    expect(niceStep(1, 5)).toBeCloseTo(0.2, 12)
    expect(niceStep(0.03, 4)).toBeCloseTo(0.01, 12)
    expect(niceStep(7.3e6, 6)).toBe(2e6)
  })

  test("ticks are the multiples of the step inside the range", () => {
    expect(niceTicks(0, 10, 5)).toEqual({ step: 2, ticks: [0, 2, 4, 6, 8, 10] })
    const t = niceTicks(-0.33, 0.61, 5)
    expect(t.step).toBeCloseTo(0.2, 12)
    expect(t.ticks.map((v) => Number(v.toFixed(6)))).toEqual([-0.2, 0, 0.2, 0.4, 0.6])
    // zero is +0 (no "−0" label)
    expect(Object.is(t.ticks[1], 0)).toBe(true)
  })

  test("a degenerate or non-finite range has no ticks", () => {
    expect(niceTicks(3, 3, 5).ticks).toEqual([])
    expect(niceTicks(NaN, 1, 5).ticks).toEqual([])
    expect(niceTicks(0, Infinity, 5).ticks).toEqual([])
  })

  test("labels carry the decimals the step needs", () => {
    expect(tickLabel(0.4, 0.2)).toBe("0.4")
    expect(tickLabel(-12, 2)).toBe("−12")
    expect(tickLabel(0.05, 0.05)).toBe("0.05")
    expect(tickLabel(-1e-17, 0.5)).toBe("0.0")
    expect(tickLabel(40000, 20000)).toBe("40k")
    expect(tickLabel(-6e6, 2e6)).toBe("−6M")
  })

  test("the time axis uses clock steps", () => {
    const t = timeTicks(3020, 3095, 6)
    expect(t.step).toBe(15)
    expect(t.ticks).toEqual([3030, 3045, 3060, 3075, 3090])
    expect(timeTicks(0, 3095, 8).step).toBe(600)
    expect(timeTicks(10, 10.4, 5).step).toBe(0.1)
  })

  test("clock and value formats", () => {
    expect(formatClock(3045)).toBe("50:45")
    expect(formatClock(3045.25, 0.5)).toBe("50:45.3")
    expect(formatClock(59.97, 0.5)).toBe("1:00.0")
    expect(formatClock(5.5, 0.01)).toBe("0:05.50")
    expect(formatValue(1234.56)).toBe("1235")
    expect(formatValue(-12.345)).toBe("−12.35")
    expect(formatValue(0.01234)).toBe("0.012")
    expect(formatValue(-0.0001)).toBe("0.000")
    expect(formatValue(NaN)).toBe("—")
  })
})

describe("sample lookup and autoscale", () => {
  const ts = Float64Array.of(0, 1, 2, 4, 8)

  test("visible range = the samples inside the window; widen adds one on each side", () => {
    expect(lowerBound(ts, 5, 2)).toBe(2)
    expect(lowerBound(ts, 5, 2.5)).toBe(3)
    expect(lowerBound(ts, 5, 99)).toBe(5)
    expect(visibleRange(ts, 5, { t0: 1, t1: 4 })).toEqual({ i0: 1, i1: 3 })
    expect(visibleRange(ts, 5, { t0: 0.5, t1: 3 })).toEqual({ i0: 1, i1: 2 })
    expect(widen({ i0: 1, i1: 2 }, 5)).toEqual({ i0: 0, i1: 3 })
    expect(widen({ i0: 0, i1: 4 }, 5)).toEqual({ i0: 0, i1: 4 })
    // a window between two samples is empty, and widening it gives the two neighbours
    const gap = visibleRange(ts, 5, { t0: 4.5, t1: 5 })
    expect(gap.i1).toBeLessThan(gap.i0)
    expect(widen(gap, 5)).toEqual({ i0: 3, i1: 4 })
  })

  test("sampleAt interpolates and does not invent values inside a gap", () => {
    const ys = Float64Array.of(0, 10, NaN, 40, 80)
    expect(sampleAt(ts, ys, 5, 0.5)).toBe(5)
    expect(sampleAt(ts, ys, 5, 6)).toBe(60)
    expect(sampleAt(ts, ys, 5, -3)).toBe(0)
    expect(sampleAt(ts, ys, 5, 99)).toBe(80)
    expect(sampleAt(ts, ys, 5, 1.2)).toBe(10) // next to the gap: the nearer sample
    expect(sampleAt(ts, ys, 5, 1.8)).toBeNaN()
  })

  test("range ignores NaN and ±Infinity", () => {
    const ys = Float64Array.of(NaN, 3, -Infinity, -2, Infinity, 7)
    expect(finiteRange(ys, { i0: 0, i1: 5 })).toEqual([-2, 7])
    expect(finiteRange(ys, { i0: 0, i1: 2 })).toEqual([3, 3])
    expect(finiteRange(Float64Array.of(NaN, NaN), { i0: 0, i1: 1 })).toBeNull()
  })

  test("8 % padding; a flat line and missing data still give a usable axis", () => {
    expect(padRange([0, 100])).toEqual([-8, 108])
    const flat = padRange([5, 5])
    expect(flat[0]).toBeLessThan(5)
    expect(flat[1]).toBeGreaterThan(5)
    expect(padRange([0, 0])).toEqual([-0.5, 0.5])
    expect(padRange(null)).toEqual([-1, 1])
    expect(padRange([NaN, 3])).toEqual([-1, 1])
  })
})

describe("min–max decimation", () => {
  test("a short series is passed through unchanged", () => {
    const xs = Float64Array.of(0, 1, 2, 3), ys = Float64Array.of(5, -1, 4, 2)
    const p = decimate(xs, ys, 0, 3, 0, 4, 400)
    expect(Array.from(p.x)).toEqual([0, 1, 2, 3])
    expect(Array.from(p.y)).toEqual([5, -1, 4, 2])
  })

  test("30 000 samples in 100 columns: ≤ 4 points per column, every column's extremes kept", () => {
    const n = 30000, cols = 100
    const xs = linspace(0, 100, n)
    const ys = Float64Array.from(xs, (x, i) => Math.sin(x * 1.7) * 40 + Math.sin(i * 0.9) * 3)
    const p = decimate(xs, ys, 0, n - 1, 0, 100, cols)
    expect(p.n).toBeLessThanOrEqual(4 * (cols + 2))
    expect(p.n).toBeGreaterThan(cols)
    const envelope = (x: ArrayLike<number>, y: ArrayLike<number>, m: number): [number, number][] => {
      const env: [number, number][] = Array.from({ length: cols + 1 }, () => [Infinity, -Infinity])
      for (let i = 0; i < m; i++) {
        const c = Math.min(cols, Math.floor(x[i]))
        env[c] = [Math.min(env[c][0], y[i]), Math.max(env[c][1], y[i])]
      }
      return env
    }
    expect(envelope(p.x, p.y, p.n)).toEqual(envelope(xs, ys, n))
    // still a function of time: x never goes backwards
    for (let i = 1; i < p.n; i++) expect(p.x[i]).toBeGreaterThanOrEqual(p.x[i - 1])
  })

  test("a one-sample spike survives", () => {
    const n = 20000
    const xs = linspace(0, 10, n)
    const ys = new Float64Array(n)
    ys[12345] = 99
    const p = decimate(xs, ys, 0, n - 1, 0, 10, 300)
    expect(Math.max(...p.y)).toBe(99)
    expect(p.x[Array.from(p.y).indexOf(99)]).toBe(xs[12345])
  })

  test("non-finite samples break the line (one pen-up, none at the ends)", () => {
    const xs = linspace(0, 9, 10)
    const ys = Float64Array.of(NaN, 1, 2, NaN, NaN, Infinity, 3, 4, 5, NaN)
    const p = decimate(xs, ys, 0, 9, 0, 9, 1000)
    expect(Array.from(p.y)).toEqual([1, 2, NaN, 3, 4, 5])
    expect(polyRuns(p)).toEqual([[0, 2], [3, 6]])
  })

  test("samples outside the window collapse into one bucket per side", () => {
    const n = 10000
    const xs = linspace(0, 100, n), ys = Float64Array.from(xs, (x) => x)
    const p = decimate(xs, ys, 0, n - 1, 40, 41, 50)
    expect(p.n).toBeLessThanOrEqual(4 * 52)
    expect(p.x[0]).toBe(0)
    expect(p.x[p.n - 1]).toBe(100)
  })

  test("an empty range or a zero-width window gives an empty line", () => {
    const xs = Float64Array.of(0, 1), ys = Float64Array.of(0, 1)
    expect(decimate(xs, ys, 1, 0, 0, 1, 10).n).toBe(0)
    expect(decimate(xs, ys, 0, 1, 1, 1, 10).n).toBe(0)
  })

  test("x–y thinning drops sub-pixel steps and keeps the ends of every run", () => {
    const n = 5000
    const xs = linspace(0, 10, n), ys = Float64Array.from(xs, (x) => x * x)
    ys[2500] = NaN
    const p = thinPath(xs, ys, 0, n - 1, 20, 2, 1)
    expect(p.n).toBeLessThan(600)
    expect(p.x[0]).toBe(0)
    expect(p.x[p.n - 1]).toBe(10)
    const runs = polyRuns(p)
    expect(runs.length).toBe(2)
    expect(p.x[runs[0][1] - 1]).toBe(xs[2499])
    expect(p.x[runs[1][0]]).toBe(xs[2501])
  })

  test("nearest sample in pixels", () => {
    const xs = Float64Array.of(0, 10, 20), ys = Float64Array.of(0, 5, 0)
    expect(nearestSample(xs, ys, { i0: 0, i1: 2 }, 11, 4, 2, 2)).toEqual({ i: 1, dist: Math.hypot(2, 2) })
    expect(nearestSample(xs, ys, { i0: 1, i1: 0 }, 0, 0, 1, 1).i).toBe(-1)
  })
})

describe("plan-vs-flown statistics", () => {
  test("a constant offset: RMS = max = the offset", () => {
    const ts = linspace(0, 10, 11)
    const plan = linspace(100, 0, 11), flown = plan.map((v) => v + 0.4)
    const s = trackingStats(ts, flown, plan, { i0: 0, i1: 10 })!
    expect(s.rms).toBeCloseTo(0.4, 12)
    expect(s.max).toBeCloseTo(0.4, 12)
    expect(s.n).toBe(11)
  })

  test("time-weighted RMS and the time of the largest error", () => {
    // errors 0, 2, 0 at t = 0, 1, 2: weights ½, 1, ½ → mean square 2
    const s = trackingStats(Float64Array.of(0, 1, 2), Float64Array.of(1, 3, 1), Float64Array.of(1, 1, 1), { i0: 0, i1: 2 })!
    expect(s.rms).toBeCloseTo(Math.SQRT2, 12)
    expect(s.max).toBe(2)
    expect(s.tMax).toBe(1)
  })

  test("dense sampling does not outweigh a sparse stretch", () => {
    // error 1 for 0..10 s (2 samples), error 3 for 10..11 s (101 samples)
    const ts = Float64Array.from([0, ...Array.from(linspace(10, 11, 101))])
    const err = Float64Array.from(ts, (t) => (t <= 10 ? 1 : 3))
    const s = trackingStats(ts, err, new Float64Array(ts.length), { i0: 0, i1: ts.length - 1 })!
    // an unweighted RMS would be ≈ 2.97; by time it is mostly the 10 s at error 1
    expect(s.rms).toBeLessThan(1.5)
    expect(s.rms).toBeGreaterThan(1)
    expect(s.max).toBe(3)
  })

  test("only samples that carry a plan count; none → null", () => {
    const ts = linspace(0, 4, 5)
    const flown = Float64Array.of(50, 1, 1, 1, 50)
    const plan = gated(Float64Array.of(0, 0.5, 0.5, 0.5, 0), Float64Array.of(0, 1, 1, 1, 0))
    const s = trackingStats(ts, flown, plan, { i0: 0, i1: 4 })!
    expect(s.n).toBe(3)
    expect(s.rms).toBeCloseTo(0.5, 12)
    expect(s.max).toBeCloseTo(0.5, 12)
    expect(trackingStats(ts, flown, plan, { i0: 4, i1: 4 })).toBeNull()
    expect(trackingStats(ts, flown, new Float64Array(5).fill(NaN), { i0: 0, i1: 4 })).toBeNull()
  })
})

describe("series extraction", () => {
  test("gating, band edges and limit channels", () => {
    expect(Array.from(gated(Float64Array.of(1, 2, 3), Float64Array.of(0, 1, 1)))).toEqual([NaN, 2, 3])
    expect(Array.from(offsetBy(Float64Array.of(10, 10, NaN, 10), Float64Array.of(2, 0, 2, NaN), -1))).toEqual([8, NaN, NaN, NaN])
    expect(Array.from(whereSet(Float64Array.of(0, -35, NaN, -2)))).toEqual([NaN, -35, NaN, -2])
  })

  const n = 6
  const t = linspace(0, 5, n)
  const planOn = Float64Array.of(0, 0, 1, 1, 1, 1)
  const src = source(n, {
    t, planOn,
    pinX: [9e6, 500, 40, 20, 5, 0.2], planX: [0, 0, 38, 21, 4, 0], pinY: [1, 2, 3, 4, 5, 6], pinZ: [9000, 800, 300, 120, 30, 0.5], planZ: [0, 0, 310, 118, 31, 0],
    tubeLat: [0, 0, 6, 4, 2, 0.5], tubeHard: [0, 0, 0, 0, 8, 8],
    vu: [-90, -80, -60, -30, -8, -0.5], planVz: [0, 0, -58, -31, -8, -0.4], vzLimit: [0, 0, -75, -50, -20, 0],
    tiltE: [0, 0, 0.1, 0.05, 0, 0], cmdTiltE: [0, 0, 0.12, 0.04, 0, 0], planTiltE: [0, 0, Math.PI / 18, 0.05, 0, 0],
    gp1: [0, 0, Math.PI / 36, 0, 0, 0], wx: [0, 0, Math.PI / 180, 0, 0, 0],
  })
  const set = buildPanels(src)
  const byId = (id: string): TimePanel => set.panels.find((p) => p.id === id) as TimePanel
  const lineOf = (p: TimePanel, label: string): Float64Array => p.lines.find((l) => l.label === label)!.y

  test("fourteen panels: twelve over time and two x–y", () => {
    expect(set.panels.map((p) => p.id)).toEqual(["pinE", "pinN", "height", "vz", "vlat", "tiltE", "tiltN", "roll", "acc", "throttle", "gimbalP", "gimbalY", "track", "profile"])
    expect(set.panels.filter((p) => p.kind === "xy").map((p) => p.id)).toEqual(["track", "profile"])
    expect(set.t).toEqual(t)
  })

  test("plan and tube are gaps where planOn is 0; the flown line is untouched", () => {
    const p = byId("pinE")
    expect(Array.from(lineOf(p, "plan"))).toEqual([NaN, NaN, 38, 21, 4, 0])
    expect(Array.from(lineOf(p, "flown"))).toEqual([9e6, 500, 40, 20, 5, 0.2])
    expect(Array.from(p.band!.lo)).toEqual([NaN, NaN, 32, 17, 2, -0.5])
    expect(Array.from(p.band!.hi)).toEqual([NaN, NaN, 44, 25, 6, 0.5])
    // the tower corridor (hard limit) only where tubeHard > 0, centred on the arm pair (armX), E only
    expect(Array.from(lineOf(p, "arm +"))).toEqual([NaN, NaN, NaN, NaN, 8, 8])
    expect(Array.from(lineOf(p, "arm −"))).toEqual([NaN, NaN, NaN, NaN, -8, -8])
    expect(byId("pinN").kind === "time" && byId("pinN").lines.some((l) => l.role === "hard")).toBe(false)
  })

  test("units: radians to degrees; the sink limit only where it is set", () => {
    expect(lineOf(byId("tiltE"), "plan")[2]).toBeCloseTo(10, 9)
    expect(lineOf(byId("tiltE"), "command")[2]).toBeCloseTo(0.12 * 180 / Math.PI, 9)
    expect(lineOf(byId("gimbalP"), "SL1")[2]).toBeCloseTo(5, 9)
    expect(lineOf(byId("roll"), "rate")[2]).toBeCloseTo(1, 9)
    expect(byId("gimbalP").limits).toEqual([-15, 15])
    expect(Array.from(lineOf(byId("vz"), "sink limit"))).toEqual([NaN, NaN, -75, -50, -20, NaN])
  })

  test("autoscale follows the visible data; megametre samples outside the window do not count", () => {
    const vis = visibleRange(t, n, { t0: 2, t1: 5 })
    const p = byId("height")
    const [lo, hi] = timePanelRange(p, vis, true)
    expect(lo).toBeCloseTo(-310 * 0.08, 9)
    expect(hi).toBeCloseTo(310 * 1.08, 9)
    // pins E: flown 0.2..40, band −0.5..44, hard limits −8..12 → −8..44 with limits, −0.5..44 without
    expect(timePanelRange(byId("pinE"), vis, true)[0]).toBeCloseTo(-8 - 52 * 0.08, 9)
    expect(timePanelRange(byId("pinE"), vis, false)[0]).toBeCloseTo(-0.5 - 44.5 * 0.08, 9)
    // gimbal: ±15° limit lines widen the axis only when asked to
    expect(timePanelRange(byId("gimbalP"), vis, false)[1]).toBeLessThan(6)
    expect(timePanelRange(byId("gimbalP"), vis, true)[1]).toBeGreaterThan(15)
  })

  test("statistics of panels 1–4 use the plan samples only", () => {
    const vis = { i0: 0, i1: n - 1 }
    const tr = byId("pinE").track!
    const s = trackingStats(t, tr.flown, tr.plan, vis)!
    expect(s.n).toBe(4)
    expect(s.max).toBe(2) // |40 − 38|, not the megametre sample before the plan
    for (const id of ["pinE", "pinN", "height", "vz"]) expect(byId(id).track).toBeDefined()
  })

  test("ground track and side profile", () => {
    const track = set.panels.find((p) => p.id === "track") as XYPanel
    const profile = set.panels.find((p) => p.id === "profile") as XYPanel
    expect(track.equal).toBe(true)
    expect(Array.from(track.curves.find((c) => c.role === "plan")!.x)).toEqual([NaN, NaN, 38, 21, 4, 0])
    expect(profile.curves.find((c) => c.role === "flown")!.x[2]).toBeCloseTo(Math.hypot(40, 3), 9)
    expect(profile.curves.find((c) => c.role === "plan")!.x[0]).toBeNaN()
    const r = xyPanelRange(track, { i0: 2, i1: 5 })
    // flown E 0.2..40 and the catch point at 0
    expect(r.x[0]).toBeCloseTo(-40 * 0.08, 9)
    expect(r.x[1]).toBeCloseTo(40 * 1.08, 9)
    const eq = equalAspect([0, 10], [0, 10], 400, 100)
    expect(eq.y).toEqual([0, 10])
    expect(eq.x).toEqual([-15, 25])
    expect((eq.x[1] - eq.x[0]) / 400).toBeCloseTo((eq.y[1] - eq.y[0]) / 100, 12)
  })

  test("a recording without any plan (channels still zero): flown only, no statistics", () => {
    const bare = buildPanels(source(n, { t, pinX: [5, 4, 3, 2, 1, 0], vu: [-5, -4, -3, -2, -1, 0] }))
    const all = { i0: 0, i1: n - 1 }
    expect(hasPlan(bare.on, all)).toBe(false)
    expect(hasPlan(set.on, all)).toBe(true)
    expect(hasPlan(set.on, { i0: 0, i1: 1 })).toBe(false)
    const p = bare.panels[0] as TimePanel
    expect(p.planned).toBe(true)
    expect(lineOf(p, "plan").every(Number.isNaN)).toBe(true)
    expect(p.band!.hi.every(Number.isNaN)).toBe(true)
    expect(Array.from(lineOf(p, "flown"))).toEqual([5, 4, 3, 2, 1, 0])
    expect(trackingStats(t, p.track!.flown, p.track!.plan, all)).toBeNull()
    expect(timePanelRange(p, all, true)).toEqual([-0.4, 5.4])
    // nothing to draw at all (no data, constant limits only) is still a valid axis
    expect(timePanelRange(bare.panels.find((q) => q.id === "gimbalP") as TimePanel, { i0: 3, i1: 2 }, true)).toEqual([-1, 1])
  })

  test("a channel missing from an older recording reads as a gap instead of throwing", () => {
    const index = { t: 0, pinX: 1, planOn: 7 } as Record<ChannelKey, number>
    const s = recordingSource({ n: 3, data: [Float64Array.of(0, 1, 2), Float64Array.of(4, 5, 6)] }, index)
    expect(Array.from(s.ch("pinX"))).toEqual([4, 5, 6])
    expect(Array.from(s.ch("planOn"))).toEqual([NaN, NaN, NaN])
    expect(Array.from(s.ch("planX"))).toEqual([NaN, NaN, NaN])
    expect(() => buildPanels(s)).not.toThrow()
  })
})

describe("panel grid", () => {
  test("all fourteen panels fit a desktop stage without scrolling", () => {
    const g = gridLayout(14, 1900, 760)
    expect(g.scroll).toBe(false)
    expect(g.cols * g.rows).toBeGreaterThanOrEqual(14)
    expect(g.cellH).toBeGreaterThanOrEqual(150)
    expect(1900 / g.cols).toBeGreaterThanOrEqual(250)
  })

  test("a small stage keeps the minimum panel size and scrolls", () => {
    const g = gridLayout(14, 700, 300)
    expect(g.scroll).toBe(true)
    expect(g.cellH).toBe(150)
    expect(g.cols).toBe(2)
    expect(g.rows).toBe(7)
  })
})
