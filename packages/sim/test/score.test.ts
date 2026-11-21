/**
 * score(recording): the continuous catch margin, the sideways catch geometry, and the hard rules.
 * The fixtures are synthetic columnar recordings (one crossing at the rail plane) so the test does
 * not depend on a 30 s full mission.
 */
import { describe, expect, test } from "bun:test"
import { THERMAL_LIMITS } from "@ship/heat"
import { CH, N_CH, type Recording, recorder, score } from "../src/index"

interface Fixture {
  readonly outcome?: "caught" | "lost"
  /** mid-point of the two pins relative to the catch point (pad ENU) */
  readonly axisE?: number
  readonly axisN?: number
  readonly contactV?: readonly [number, number, number]
  /** 1 = arms open, 0 = closed */
  readonly arms?: number
  readonly pinLoad?: number
  readonly flipV?: number
  readonly tileSurf?: number
  readonly bondline?: number
  readonly coldGasAfterFlip?: number
  readonly coilCurrent?: number
  readonly engines?: number
  readonly neverCross?: boolean
}

const make = (f: Fixture = {}): Recording => {
  const n = 22
  const rec = recorder(1)
  const pinOut = 5.3
  const axisE = f.axisE ?? 0.3
  const axisN = f.axisN ?? 0.2
  const v = f.contactV ?? [0.3, 0.2, -0.35]
  for (let i = 0; i < n; i++) {
    const t = i
    const row = new Float64Array(N_CH)
    const set = (k: keyof typeof CH, val: number): void => { row[CH[k]] = val }
    set("t", t)
    set("phase", t >= 18 ? 12 : t >= 16 ? 10 : t >= 8 ? 8 : t >= 4 ? 6 : 0)
    set("lat", 0); set("lon", 0); set("theta", 0) // ECEF at lon 0: body x up, y east, z north
    set("qw", 1); set("qx", 0); set("qy", 0); set("qz", 0)
    set("alt", 1500 - 70 * t); set("hBase", 1500 - 70 * t); set("V", f.flipV ?? 113); set("mach", 0.3); set("g", 1.2)
    set("wx", 0.01)
    // the pin crosses the rail plane around t=17
    const pinZ = f.neverCross ? 40 - 1.5 * t : t <= 17 ? 30 - 2 * t : -1
    set("pinX", axisE + pinOut); set("pinY", axisN); set("pinZ", pinZ)
    set("ve", v[0]); set("vn", v[1]); set("vu", v[2])
    set("arms", f.arms ?? 0)
    set("pinLoad", t >= 15 ? f.pinLoad ?? 9e5 : 0)
    set("n2", t >= 4 ? 2500 - (f.coldGasAfterFlip ?? 0) * (t - 4) / 18 : 2600)
    set("header", 14000 - 300 * t)
    set("coilI", f.coilCurrent ?? 20); set("coilMode", 6)
    set("T1", t >= 4 ? 1.2e6 : 0); set("T2", t >= 4 ? 1.2e6 : 0); set("T3", 0)
    if ((f.engines ?? 2) === 3) set("T3", 1.2e6)
    set("bellyMiss", 650)
    rec.push(row, [290])
  }
  return rec.finish([], {
    epochJd: 0, theta0: 0, padLat: 0, padLon: 0, padAlt: 0,
    outcome: f.outcome ?? "caught",
    summary: {
      outcome: f.outcome ?? "caught", peakG: 5.2, peakQbar: 24e3,
      peakTileSurf: f.tileSurf ?? 1500, peakTileBack: 350, peakBondline: f.bondline ?? 540, peakSteel: 350,
      peakPicaSurf: 1900, n2Left: 2500,
    },
    phaseStarts: [{ phase: "pitchUp", t: 0 }, { phase: "flip", t: 4 }, { phase: "landing", t: 8 }, { phase: "terminal", t: 16 }, { phase: "caught", t: 18 }],
    predPath: [],
    vehicle: { pinOut, coil: { Iop: 3000 } },
    wallClockMs: 1000, steps: 100,
  })
}

describe("score: continuous catch margin", () => {
  test("a clean crossing is a catch with zero margin", () => {
    const s = score(make())
    expect(s.catch).toBe(true)
    expect(s.catchFeasible).toBe(true)
    expect(s.feasible).toBe(true)
    expect(s.margin).toBeLessThan(0.1)
    expect(s.terms.catchPoint.lateral).toBeCloseTo(Math.hypot(0.3, 0.2), 3)
    expect(s.terms.catchPoint.across).toBeCloseTo(0.3, 3)
    expect(s.terms.catchPoint.along).toBeCloseTo(0.2, 3)
    expect(s.terms.catchPoint.contactSpeed).toBeCloseTo(Math.hypot(0.3, 0.2, 0.35), 3)
    expect(s.terms.catchPoint.rollErrorDeg).toBeLessThan(1)
    expect(s.terms.catchPoint.yawErrorDeg).toBeLessThan(1)
    expect(s.terms.landingEngines).toBe(2)
    expect(s.terms.coldGasAfterFlip).toBe(0)
    expect(s.terms.coilDischarged).toBe(true)
  })

  test("the miss is continuous and a 145 m offset fails the pins", () => {
    const s = score(make({ axisE: 145, outcome: "lost", arms: 1, pinLoad: 0 }))
    expect(s.catch).toBe(false)
    expect(s.margin).toBeGreaterThan(140)
    expect(s.constraints.find((c) => c.id === "lateral")?.ok).toBe(false)
  })

  test("a run that never reaches the rail plane reports the vertical stand-off", () => {
    const s = score(make({ neverCross: true, outcome: "lost" }))
    expect(s.catch).toBe(false)
    expect(s.terms.verticalMiss).toBeGreaterThan(5)
    expect(s.margin).toBeGreaterThan(5)
  })

  test("a 'caught' recording with the arms open is a false catch", () => {
    const s = score(make({ arms: 1 }))
    expect(s.catch).toBe(false)
    expect(s.constraints.find((c) => c.id === "catch")?.ok).toBe(false)
  })

  test("contact speed above 1 m/s fails even with a small offset", () => {
    const s = score(make({ contactV: [1.4, 0.2, -0.4] }))
    expect(s.catch).toBe(false)
    expect(s.margin).toBeGreaterThan(1)
    expect(s.constraints.find((c) => c.id === "contactSpeed")?.ok).toBe(false)
  })

  test("three engines, cold gas after the flip and a live coil are mechanism failures", () => {
    const s = score(make({ engines: 3, coldGasAfterFlip: 1, coilCurrent: 500 }))
    expect(s.catchFeasible).toBe(false)
    expect(s.constraints.find((c) => c.id === "landingEngines")?.ok).toBe(false)
    expect(s.constraints.find((c) => c.id === "noColdGasAfterFlip")?.ok).toBe(false)
    expect(s.constraints.find((c) => c.id === "coilDischarge")?.ok).toBe(false)
  })

  test("a tile/bondline over-temperature is reported but does not hide a catch", () => {
    const s = score(make({ tileSurf: THERMAL_LIMITS.tileSurface + 120, bondline: THERMAL_LIMITS.bondline + 135 }))
    expect(s.catch).toBe(true)
    expect(s.catchFeasible).toBe(true)
    expect(s.feasible).toBe(false)
    expect(s.constraints.find((c) => c.id === "tileSurface")?.ok).toBe(false)
    expect(s.constraints.find((c) => c.id === "bondline")?.ok).toBe(false)
  })
})
