/**
 * Mission scoring — turn a recording into a continuous catch margin and a hard-constraint verdict.
 *
 * The binary catch (`armsClose && pinLoad`, pins within ±1 m, ≤1 m/s) is the final constraint. The
 * optimiser needs a signal before it crosses the envelope, so the primary term is a **continuous
 * miss in metres**: the pin-station offset at the rail-plane crossing, the contact speed excess, the
 * roll/geometry error and, when the ship never reaches the plane, the vertical stand-off. The binary
 * outcome is still checked (and a "caught" recording with the arms open is rejected as a false catch).
 *
 * Catch geometry (Q26/T09): the arms' rails run north–south, the pins are body ±y and must lie
 * east–west over the rails, the belly (body +z) faces north or south, the across-arms offset (pad x)
 * is the tight axis, and the residual horizontal velocity should run along the rails (pad y), not
 * across them.
 */
import { type Q, type V3, qrot, v3 } from "@ship/math"
import { rotZ } from "@ship/earth"
import { THERMAL_LIMITS } from "@ship/heat"
import { CH, type ChannelKey, type Recording } from "./recording"

/** design floor for the entry speed at the flip (G's 117 m/s budget; 110 leaves measurement room) */
export const FLIP_SPEED_FLOOR = 110
/** catch envelope: pin-station offset at the rail plane (m) */
export const CATCH_LATERAL = 1.0
/** catch envelope: contact speed at the pins (m/s, 3-D ground-relative) */
export const CATCH_SPEED = 1.0
/** roll/geometry envelope at the pins (deg): body y within this of east–west */
export const CATCH_ROLL = 5.0

export interface CatchPoint {
  readonly t: number
  readonly found: boolean
  /** pin-station (body +y pin) position relative to the catch point, pad ENU */
  readonly pinE: number
  readonly pinN: number
  readonly pinU: number
  /** mid-point between the two pins (the ship axis at the pin station) */
  readonly axisE: number
  readonly axisN: number
  readonly axisU: number
  /** pin-station offset from the catch point (m) */
  readonly lateral: number
  readonly across: number
  readonly along: number
  readonly vE: number
  readonly vN: number
  readonly vU: number
  readonly contactSpeed: number
  readonly armsOpen: number
  readonly pinLoad: number
  /** body +y and +z in pad ENU, and the roll/yaw errors they imply */
  readonly bodyYE: number
  readonly bodyYN: number
  readonly bodyYU: number
  readonly bodyZE: number
  readonly bodyZN: number
  readonly rollErrorDeg: number
  readonly yawErrorDeg: number
}

export interface PhaseRoll {
  readonly phase: string
  readonly maxRollDeg: number
  readonly n2Used: number
}

/** The landing trace diagnostics the 6-DOF review identified (T09): the thrust-azimuth reversals,
 *  the attitude lag, gimbal saturation and the lateral state at the roll-step altitudes. */
export interface LandingTrace {
  readonly lateralAt346: number
  readonly lateralAt250: number
  readonly vHorizAt346: number
  readonly vHorizAt250: number
  readonly rollRateAt250: number
  readonly attErrMaxDeg: number
  readonly azSwingDeg: number
  readonly azRateMaxDeg: number
  readonly gimbalSatFrac: number
}

export interface ScoreTerms {
  readonly outcome: string
  readonly caught: boolean
  /** continuous miss (m-equivalent): 0 inside the catch envelope */
  readonly margin: number
  readonly catchPoint: CatchPoint
  /** vertical stand-off when the pins never reach the rail plane (m) */
  readonly verticalMiss: number
  readonly flipSpeed: number
  readonly flipAlt: number
  readonly bellyMiss: number
  readonly landingBurn: number
  readonly landingEngines: number
  readonly coldGasAfterFlip: number
  readonly coldGasNonRoll: number
  readonly headerAtFlip: number
  readonly headerLeft: number
  readonly coilCurrentAtFlip: number
  readonly coilDischarged: boolean
  readonly peakG: number
  readonly peakQbar: number
  readonly tileSurf: number
  readonly tileBack: number
  readonly bondline: number
  readonly steel: number
  readonly picaSurf: number
  readonly roll: readonly PhaseRoll[]
  readonly trace: LandingTrace
  readonly n2Total: number
  readonly wallClockMs: number
}

export interface ConstraintVerdict {
  readonly id: string
  readonly ok: boolean
  readonly detail: string
  /** thermal constraints are design-level; they do not invalidate the catch mechanism */
  readonly kind: "mechanism" | "thermal" | "propellant"
}

export interface Score {
  /** minimise: catch margin + a small propellant/load term (metres-equivalent) */
  readonly cost: number
  /** mechanism constraints only (catch, engines, cold gas, coil, entry speed) */
  readonly catchFeasible: boolean
  /** every hard constraint, including TPS */
  readonly feasible: boolean
  readonly catch: boolean
  readonly margin: number
  readonly terms: ScoreTerms
  readonly constraints: readonly ConstraintVerdict[]
}

const num = (summary: Record<string, number | string>, key: string, fallback = 0): number => {
  const v = summary[key]
  return typeof v === "number" && Number.isFinite(v) ? v : fallback
}

const clamp01 = (x: number): number => Math.min(1, Math.max(0, x))

/** index of the first sample at or after mission time t */
const indexAt = (rec: Recording, t: number): number => {
  const ts = rec.data[CH.t]
  if (rec.n === 0) return 0
  if (t <= ts[0]) return 0
  if (t >= ts[rec.n - 1]) return rec.n - 1
  let lo = 0
  let hi = rec.n - 1
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1
    if (ts[m] <= t) lo = m
    else hi = m
  }
  return lo
}

const phaseTime = (rec: Recording, phase: string): number =>
  rec.meta.phaseStarts.find((p) => p.phase === phase)?.t ?? -1

/** linear interpolation of a channel between samples i and i+1 */
const lerp = (rec: Recording, key: keyof typeof CH, i: number, f: number): number => {
  const c = rec.data[CH[key]]
  return c[i] + (c[Math.min(i + 1, rec.n - 1)] - c[i]) * f
}

const pinOutOf = (rec: Recording): number => {
  const v: unknown = rec.meta.vehicle
  if (typeof v !== "object" || v === null) return 5.3
  if ("pinOut" in v && typeof v.pinOut === "number") return v.pinOut
  return 5.3
}

const coilIopOf = (rec: Recording): number => {
  const v: unknown = rec.meta.vehicle
  if (typeof v !== "object" || v === null) return 3000
  if ("coil" in v && typeof v.coil === "object" && v.coil !== null && "Iop" in v.coil && typeof v.coil.Iop === "number") return v.coil.Iop
  return 3000
}

/**
 * The pad ENU basis in ECI at a recording sample (same construction as `sim/step.ts`).
 * `lat`/`lon` are geodetic, `theta` the Earth rotation angle at that sample.
 */
const enuAt = (rec: Recording, i: number): { east: V3; north: V3; up: V3 } => {
  const lat = rec.data[CH.lat][i]
  const lon = rec.data[CH.lon][i]
  const theta = rec.data[CH.theta][i]
  const sl = Math.sin(lat), cl = Math.cos(lat), so = Math.sin(lon), co = Math.cos(lon)
  return {
    east: rotZ(v3(-so, co, 0), theta),
    north: rotZ(v3(-sl * co, -sl * so, cl), theta),
    up: rotZ(v3(cl * co, cl * so, sl), theta),
  }
}

const dot3 = (a: V3, b: V3): number => a.x * b.x + a.y * b.y + a.z * b.z

/** geometry at a crossing/interpolated sample */
const catchAt = (rec: Recording, i: number, f: number, t: number): CatchPoint => {
  const pinE = lerp(rec, "pinX", i, f)
  const pinN = lerp(rec, "pinY", i, f)
  const pinU = lerp(rec, "pinZ", i, f)
  const vE = lerp(rec, "ve", i, f)
  const vN = lerp(rec, "vn", i, f)
  const vU = lerp(rec, "vu", i, f)
  const armsOpen = Math.min(lerp(rec, "arms", i, 0), lerp(rec, "arms", i + 1, 0))
  const pinLoad = Math.max(lerp(rec, "pinLoad", i, 0), lerp(rec, "pinLoad", i + 1, 0))
  const q: Q = { w: lerp(rec, "qw", i, f), x: lerp(rec, "qx", i, f), y: lerp(rec, "qy", i, f), z: lerp(rec, "qz", i, f) }
  const enu = enuAt(rec, i)
  const by = qrot(q, v3(0, 1, 0))
  const bz = qrot(q, v3(0, 0, 1))
  const bodyYE = dot3(by, enu.east), bodyYN = dot3(by, enu.north), bodyYU = dot3(by, enu.up)
  const bodyZE = dot3(bz, enu.east), bodyZN = dot3(bz, enu.north)
  const pinOut = pinOutOf(rec)
  // the recorded pin is body +y; the axis point is the midpoint of the two pins
  const axisE = pinE - pinOut * bodyYE
  const axisN = pinN - pinOut * bodyYN
  const axisU = pinU - pinOut * bodyYU
  const lateral = Math.hypot(axisE, axisN)
  const contactSpeed = Math.hypot(vE, vN, vU)
  // body y must lie east–west; body z north–south
  const rollErrorDeg = (Math.acos(clamp01(Math.abs(bodyYE))) * 180) / Math.PI
  const yawErrorDeg = (Math.acos(clamp01(Math.abs(bodyZN))) * 180) / Math.PI
  return {
    t, found: true, pinE, pinN, pinU, axisE, axisN, axisU,
    lateral, across: Math.abs(axisE), along: Math.abs(axisN),
    vE, vN, vU, contactSpeed, armsOpen, pinLoad,
    bodyYE, bodyYN, bodyYU, bodyZE, bodyZN, rollErrorDeg, yawErrorDeg,
  }
}

const findCatchPoint = (rec: Recording, fromT: number): { point: CatchPoint; verticalMiss: number } => {
  const start = indexAt(rec, fromT)
  const z = rec.data[CH.pinZ]
  let best = start
  for (let i = start; i < rec.n - 1; i++) {
    if (Math.abs(z[i]) < Math.abs(z[best])) best = i
    if (z[i] > 0 && z[i + 1] <= 0) {
      const f = z[i] / Math.max(z[i] - z[i + 1], 1e-9)
      const t = lerp(rec, "t", i, f)
      return { point: catchAt(rec, i, f, t), verticalMiss: 0 }
    }
  }
  // never reached the rail plane: the vertical stand-off is the closest approach
  const point = catchAt(rec, best, 0, rec.data[CH.t][best])
  const verticalMiss = Math.max(0, z[best])
  return { point: { ...point, armsOpen: rec.data[CH.arms][best], pinLoad: rec.data[CH.pinLoad][best] }, verticalMiss }
}

/** max |roll rate| and N₂ used between two mission times */
const phaseStats = (rec: Recording, name: string, t0: number, t1: number): PhaseRoll => {
  if (t0 < 0) return { phase: name, maxRollDeg: 0, n2Used: 0 }
  const i0 = indexAt(rec, t0)
  const i1 = indexAt(rec, t1 < 0 ? rec.data[CH.t][rec.n - 1] : t1)
  let maxRoll = 0
  for (let i = i0; i <= i1; i++) maxRoll = Math.max(maxRoll, Math.abs(rec.data[CH.wx][i]))
  const n2 = Math.max(0, rec.data[CH.n2][i0] - rec.data[CH.n2][i1])
  return { phase: name, maxRollDeg: (maxRoll * 180) / Math.PI, n2Used: n2 }
}

const engineCount = (rec: Recording, i0: number, i1: number): number => {
  let max = 0
  for (let i = i0; i <= i1; i++) {
    const n = (rec.data[CH.T1][i] > 1e5 ? 1 : 0) + (rec.data[CH.T2][i] > 1e5 ? 1 : 0) + (rec.data[CH.T3][i] > 1e5 ? 1 : 0)
    max = Math.max(max, n)
  }
  return max
}

/**
 * Score one recording.
 *
 * `cost` is the primary optimisation objective: the continuous catch miss in metres plus a small
 * propellant/peak-load tie-break. Thermal margins do not enter the cost (they are a pre-existing
 * design issue in this vehicle) but they do gate `feasible`; `catchFeasible` isolates the catch
 * mechanism so a catch is never hidden by the tile over-temperature.
 */
export const score = (rec: Recording): Score => {
  const summary = rec.meta.summary
  const flipT = phaseTime(rec, "flip")
  const landingT = phaseTime(rec, "landing")
  const tEnd = rec.data[CH.t][rec.n - 1]
  const fromT = flipT >= 0 ? flipT : landingT >= 0 ? landingT : 0
  const { point, verticalMiss } = findCatchPoint(rec, fromT)
  const flipIdx = indexAt(rec, flipT >= 0 ? flipT : fromT)
  const flipSpeed = rec.data[CH.V][flipIdx]
  const flipAlt = rec.data[CH.hBase][flipIdx]
  // last belly-flop plan miss before the flip (or the terminal value if no flip)
  let bellyMiss = 0
  if (flipT >= 0) {
    for (let i = 0; i < flipIdx; i++) bellyMiss = rec.data[CH.bellyMiss][i]
  }
  const headerArr = rec.data[CH.header]
  const headerAtFlip = headerArr[flipIdx]
  const headerLeft = headerArr[rec.n - 1]
  const landingBurn = Math.max(0, headerAtFlip - headerLeft)
  const n2Arr = rec.data[CH.n2]
  const coldGasAfterFlip = flipT >= 0 ? Math.max(0, n2Arr[flipIdx] - n2Arr[rec.n - 1]) : 0
  // Roll-only cold gas is allowed after the flip (user 2026-11-21): the roll couples are r8–r11;
  // any nose/aft +y/±z jet (r0–r7) is translational and still forbidden.
  let coldGasNonRoll = 0
  for (let i = flipT >= 0 ? flipIdx : 0; i < rec.n; i++) {
    let s = 0
    for (let k = 0; k < 8; k++) s += rec.data[CH[`r${k}` as ChannelKey]][i]
    coldGasNonRoll = Math.max(coldGasNonRoll, s)
  }
  const coilArr = rec.data[CH.coilI]
  const coilCurrentAtFlip = flipT >= 0 ? coilArr[flipIdx] : coilArr[rec.n - 1]
  const coilIop = coilIopOf(rec)
  const modeArr = rec.data[CH.coilMode]
  // mode index 6 = discharge (see craft/mhd coil modes)
  const coilDischarged = (flipT >= 0 ? modeArr[flipIdx] : modeArr[rec.n - 1]) >= 5 && coilCurrentAtFlip <= 0.1 * coilIop
  const nEngEnd = flipT >= 0 ? engineCount(rec, flipIdx, rec.n - 1) : 0
  const peakG = num(summary, "peakG")
  const peakQbar = num(summary, "peakQbar")
  const tileSurf = num(summary, "peakTileSurf")
  const tileBack = num(summary, "peakTileBack")
  const bondline = num(summary, "peakBondline")
  const steel = num(summary, "peakSteel")
  const picaSurf = num(summary, "peakPicaSurf")
  const caughtChannel = rec.meta.outcome === "caught"
  const catchOk = caughtChannel && point.found && verticalMiss < 0.2 && point.armsOpen < 0.5 && point.pinLoad > 0 &&
    point.lateral <= CATCH_LATERAL && point.contactSpeed <= CATCH_SPEED
  const margin =
    verticalMiss +
    Math.max(0, point.lateral - CATCH_LATERAL) +
    2 * Math.max(0, point.contactSpeed - CATCH_SPEED) +
    0.1 * Math.max(0, point.rollErrorDeg - CATCH_ROLL) +
    0.05 * point.along +
    0.1 * Math.max(0, Math.abs(point.vE) - 0.5)
  const tpsOk = tileSurf <= THERMAL_LIMITS.tileSurface && tileBack <= THERMAL_LIMITS.tileBond && bondline <= THERMAL_LIMITS.bondline && steel <= THERMAL_LIMITS.steelDesign
  const constraints: ConstraintVerdict[] = [
    { id: "catch", ok: catchOk, kind: "mechanism", detail: catchOk ? `pins ${point.lateral.toFixed(2)} m off, ${point.contactSpeed.toFixed(2)} m/s, arms closed` : `outcome ${rec.meta.outcome}; pins ${point.lateral.toFixed(2)} m off, ${point.contactSpeed.toFixed(2)} m/s, arms ${point.armsOpen < 0.5 ? "closed" : "open"}` },
    { id: "lateral", ok: point.found && point.lateral <= CATCH_LATERAL, kind: "mechanism", detail: `${point.lateral.toFixed(2)} m (across ${point.across.toFixed(2)}, along ${point.along.toFixed(2)}); limit ${CATCH_LATERAL}` },
    { id: "contactSpeed", ok: point.contactSpeed <= CATCH_SPEED, kind: "mechanism", detail: `${point.contactSpeed.toFixed(2)} m/s (vE ${point.vE.toFixed(2)}, vN ${point.vN.toFixed(2)}, vU ${point.vU.toFixed(2)}); limit ${CATCH_SPEED}` },
    { id: "roll", ok: point.rollErrorDeg <= CATCH_ROLL, kind: "mechanism", detail: `body y ${point.rollErrorDeg.toFixed(1)}° off east–west, body z ${point.yawErrorDeg.toFixed(1)}° off north–south; roll limit ${CATCH_ROLL}°` },
    { id: "landingEngines", ok: nEngEnd >= 1 && nEngEnd <= 3, kind: "mechanism", detail: `${nEngEnd} sea-level Raptors lit at once after the flip (1-engine gimbal phase + staging 2/3)` },
    { id: "coldGasAfterFlip", ok: coldGasAfterFlip <= 25 && coldGasNonRoll < 0.02, kind: "mechanism", detail: `${coldGasAfterFlip.toFixed(2)} kg N₂ after the flip (roll-only; translational ${coldGasNonRoll > 0.02 ? "USED" : "none"}; limit 25 kg)` },
    { id: "coilDischarge", ok: coilDischarged, kind: "mechanism", detail: `coil ${coilCurrentAtFlip.toFixed(0)} A at the flip (${(100 * coilCurrentAtFlip / coilIop).toFixed(1)} % of I_op)` },
    { id: "entrySpeed", ok: flipSpeed >= FLIP_SPEED_FLOOR, kind: "mechanism", detail: `${flipSpeed.toFixed(1)} m/s at the flip; floor ${FLIP_SPEED_FLOOR}` },
    { id: "header", ok: headerLeft > 200, kind: "propellant", detail: `${headerLeft.toFixed(0)} kg header left; landing burn ${landingBurn.toFixed(0)} kg` },
    { id: "tileSurface", ok: tileSurf <= THERMAL_LIMITS.tileSurface, kind: "thermal", detail: `${tileSurf.toFixed(0)} K vs ${THERMAL_LIMITS.tileSurface} K` },
    { id: "tileBack", ok: tileBack <= THERMAL_LIMITS.tileBond, kind: "thermal", detail: `${tileBack.toFixed(0)} K vs ${THERMAL_LIMITS.tileBond} K` },
    { id: "bondline", ok: bondline <= THERMAL_LIMITS.bondline, kind: "thermal", detail: `${bondline.toFixed(0)} K vs ${THERMAL_LIMITS.bondline} K` },
    { id: "steel", ok: steel <= THERMAL_LIMITS.steelDesign, kind: "thermal", detail: `${steel.toFixed(0)} K vs ${THERMAL_LIMITS.steelDesign} K` },
  ]
  const mechanismOk = constraints.filter((c) => c.kind === "mechanism").every((c) => c.ok)
  const propellantOk = constraints.filter((c) => c.kind === "propellant").every((c) => c.ok)
  // phase windows for the roll/N₂ report (pre-flip only; the flip/landing roll is L's, no cold gas)
  const phases = ["entry", "loft", "pitchUp", "highAlpha", "bellyFlop"]
  const roll = phases.map((name, i) => {
    const t0 = phaseTime(rec, name)
    const t1 = phaseTime(rec, phases[i + 1] ?? "flip")
    return phaseStats(rec, name, t0, t1 < 0 ? flipT : t1)
  })
  // ---- landing trace diagnostics (T09: azimuth reversal / attitude lag / gimbal saturation) ----
  const enuBasisAt = (i: number): { east: V3; north: V3; up: V3 } => enuAt(rec, i)
  const traceFrom = indexAt(rec, flipT >= 0 ? flipT : landingT >= 0 ? landingT : 0)
  let azPrev: number | null = null
  let azSwing = 0
  let azRateMax = 0
  let attErrMax = 0
  let gimbalSat = 0
  let gimbalN = 0
  for (let i = traceFrom; i < rec.n; i++) {
    const q: Q = { w: rec.data[CH.qw][i], x: rec.data[CH.qx][i], y: rec.data[CH.qy][i], z: rec.data[CH.qz][i] }
    const bx = qrot(q, v3(1, 0, 0))
    const enu = enuBasisAt(i)
    const az = Math.atan2(dot3(bx, enu.east), dot3(bx, enu.north))
    if (azPrev !== null) {
      const dAz = Math.atan2(Math.sin(az - azPrev), Math.cos(az - azPrev))
      azSwing += Math.abs(dAz)
      const dt = Math.max(rec.data[CH.t][i] - rec.data[CH.t][i - 1], 1e-6)
      azRateMax = Math.max(azRateMax, (Math.abs(dAz) / dt) * (180 / Math.PI))
    }
    azPrev = az
    attErrMax = Math.max(attErrMax, rec.data[CH.attErr][i])
    const gm = 15 * (Math.PI / 180)
    if (Math.abs(rec.data[CH.gp][i]) > 0.98 * gm || Math.abs(rec.data[CH.gy][i]) > 0.98 * gm) gimbalSat++
    gimbalN++
  }
  const atPinHeight = (alt: number): { lateral: number; vHoriz: number; rollRate: number } => {
    let best = traceFrom
    for (let i = traceFrom; i < rec.n; i++) {
      if (Math.abs(rec.data[CH.pinZ][i] - alt) < Math.abs(rec.data[CH.pinZ][best] - alt)) best = i
    }
    return {
      lateral: Math.hypot(rec.data[CH.pinX][best], rec.data[CH.pinY][best]),
      vHoriz: Math.hypot(rec.data[CH.ve][best], rec.data[CH.vn][best]),
      rollRate: Math.abs(rec.data[CH.wx][best]),
    }
  }
  const at346 = atPinHeight(346)
  const at250 = atPinHeight(250)
  const trace: LandingTrace = {
    lateralAt346: at346.lateral, lateralAt250: at250.lateral,
    vHorizAt346: at346.vHoriz, vHorizAt250: at250.vHoriz,
    rollRateAt250: (at250.rollRate * 180) / Math.PI,
    attErrMaxDeg: (attErrMax * 180) / Math.PI,
    azSwingDeg: (azSwing * 180) / Math.PI,
    azRateMaxDeg: azRateMax,
    gimbalSatFrac: gimbalN > 0 ? gimbalSat / gimbalN : 0,
  }
  const n2Final = num(summary, "n2Left", n2Arr[rec.n - 1])
  const n2Total = Math.max(0, n2Arr[0] - n2Final)
  const terms: ScoreTerms = {
    outcome: rec.meta.outcome, caught: catchOk, margin,
    catchPoint: point, verticalMiss,
    flipSpeed, flipAlt, bellyMiss, landingBurn, landingEngines: nEngEnd,
    coldGasAfterFlip, coldGasNonRoll, headerAtFlip, headerLeft, coilCurrentAtFlip, coilDischarged,
    peakG, peakQbar, tileSurf, tileBack, bondline, steel, picaSurf,
    roll, trace, n2Total, wallClockMs: rec.meta.wallClockMs,
  }
  const cost = margin + 0.01 * landingBurn + 0.05 * Math.max(0, peakG - 6) + 0.001 * tEnd
  return {
    cost, catchFeasible: mechanismOk && propellantOk, feasible: mechanismOk && propellantOk && tpsOk,
    catch: catchOk, margin, terms, constraints,
  }
}
