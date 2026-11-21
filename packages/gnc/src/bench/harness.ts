/**
 * The bake-off harness: fixed scenarios, the shared reference path, one plant loop and the metrics.
 *
 * The terminal is measured where the pin station crosses the rail plane z = 0 (linearly interpolated
 * across the last step), so a controller that arrives fast and a controller that arrives late are
 * compared at the same geometric event. Cross-track RMS is the point-to-polyline distance of the
 * flown path to the shared reference, sampled every 50 ms.
 */
import { type V3, clamp, lerpV, norm, sub, v3 } from "@ship/math"
import { G_LAND } from "../landing"
import { directionErrorDeg, initialPlantState, stepPlant, BENCH_MASS } from "./plant"
import { makeReferencePath } from "./reference"
import { makeController, shapeCommand, STRATEGIES, type StrategyKind, type StrategyDeps } from "./strategies"
import type { PlantConfig } from "./types"

/** Terminal gate (pin station, pad ENU): 12 m south of the catch point, on the rail plane. */
export const GATE: V3 = v3(0, -12, 0)
export const GATE_V: V3 = v3(0, 0.4, -0.7)

export interface Scenario {
  readonly id: "nominal" | "hard"
  readonly label: string
  readonly p0: V3
  readonly v0: V3
}

export const SCENARIOS: readonly Scenario[] = [
  { id: "nominal", label: "nominal flip", p0: v3(500, -100, 800), v0: v3(-60, 0, -100) },
  { id: "hard", label: "hard flip", p0: v3(1000, -450, 800), v0: v3(-85, 30, -85) },
]

/** A 10 m/s crosswind is applied as the equivalent constant lateral acceleration `k·W`.
 *  Without an aero model a velocity wind has no dynamic effect; k = 0.1 s⁻¹ is stated in the doc. */
export const WIND_ACCEL_PER_MPS = 0.1
export const crosswindAccel = (speed: number): V3 => v3(WIND_ACCEL_PER_MPS * speed, 0, 0)

export interface RunConfig {
  readonly plant: PlantConfig
  readonly tMax: number
  readonly recordEvery: number
  readonly mass0: number
  readonly tiltDeg: number
  readonly vF: number
}

export const defaultRunConfig = (overrides: Partial<PlantConfig> = {}): RunConfig => ({
  plant: {
    nEng: 2,
    mode: "torque",
    rateCap: 0.45,
    tauDir: 0.35,
    tauThrottle: 0.3,
    gimbalKp: 1.5,
    gimbalKd: 1.8,
    gravity: G_LAND,
    dt: 0.01,
    disturbance: v3(0, 0, 0),
    ...overrides,
  },
  tMax: 40,
  recordEvery: 0.05,
  mass0: BENCH_MASS,
  tiltDeg: 48,
  vF: 0.7,
})

export interface RunResult {
  readonly scenario: string
  readonly strategy: StrategyKind
  readonly label: string
  readonly crossed: boolean
  readonly tContact: number
  /** duration of the shared reference profile for this scenario (s) */
  readonly referenceDuration: number
  readonly terminalP: readonly [number, number, number]
  readonly terminalV: readonly [number, number, number]
  /** |p_contact − gate| (m) */
  readonly terminalPos: number
  /** |v_contact − v_gate| (m/s) */
  readonly terminalVel: number
  /** RMS point-to-reference-path distance over the run (m) */
  readonly rmsCross: number
  readonly maxDirErrDeg: number
  readonly maxTiltDeg: number
  readonly fuelKg: number
  readonly massFinal: number
  readonly peakThrottle: number
  readonly peakThrustAccel: number
  readonly controllerMs: number
  readonly wallMs: number
  readonly replans: number
  readonly infeasibleReplans: number
  /** composite for ranking: terminalPos + 2·terminalVel + rmsCross (m-equivalent) */
  readonly score: number
}

const tiltOf = (d: V3): number => (Math.acos(clamp(d.z, -1, 1)) * 180) / Math.PI

export const runScenario = (scenario: Scenario, kind: StrategyKind, config: RunConfig): RunResult => {
  const deps: StrategyDeps = {
    reference: makeReferencePath(scenario.p0, scenario.v0, GATE, GATE_V),
    gate: GATE,
    gateV: GATE_V,
    nEng: config.plant.nEng,
    tiltRad: (config.tiltDeg * Math.PI) / 180,
    vF: config.vF,
  }
  const controller = makeController(kind, deps)
  let s = initialPlantState(scenario.p0, scenario.v0, config.mass0, config.plant.nEng)
  let prev = s
  let crossed = false
  let tContact = 0
  let terminalP: V3 = s.p
  let terminalV: V3 = s.v
  let closestP: V3 = s.p
  let closestV: V3 = s.v
  let closestT = 0
  let closestAlt = s.p.z
  let closestFuel = s.fuel
  let closestMass = s.mass
  let maxDirErrDeg = 0
  let maxTiltDeg = 0
  let peakThrottle = s.throttle
  let peakThrustAccel = norm(s.a)
  const track: V3[] = []
  let nextRecord = 0
  const wall0 = performance.now()

  while (s.t < config.tMax) {
    if (s.t >= nextRecord - 1e-9) {
      track.push(s.p)
      nextRecord += config.recordEvery
    }
    const cmd = controller.command(s, s.t)
    const shaped = shapeCommand(s, cmd, deps)
    const { next, applied } = stepPlant(s, shaped, config.plant)
    const dirErr = directionErrorDeg(next.d, applied.dirCmd)
    if (dirErr > maxDirErrDeg) maxDirErrDeg = dirErr
    const tilt = tiltOf(next.d)
    if (tilt > maxTiltDeg) maxTiltDeg = tilt
    if (next.throttle > peakThrottle) peakThrottle = next.throttle
    const aMag = norm(next.a)
    if (aMag > peakThrustAccel) peakThrustAccel = aMag
    prev = s
    s = next
    if (s.p.z >= 0 && s.p.z < closestAlt) {
      closestAlt = s.p.z
      closestP = s.p
      closestV = s.v
      closestT = s.t
      closestFuel = s.fuel
      closestMass = s.mass
    }
    if (s.p.z <= 0) {
      const den = prev.p.z - s.p.z
      const f = den > 1e-12 ? clamp(prev.p.z / den, 0, 1) : 0
      terminalP = lerpV(prev.p, s.p, f)
      terminalV = lerpV(prev.v, s.v, f)
      tContact = prev.t + f * config.plant.dt
      crossed = true
      break
    }
  }
  if (!crossed) {
    // never reached the rail plane: score the closest approach instead of the state after an
    // arbitrary climb-away at tMax
    terminalP = closestP
    terminalV = closestV
    tContact = closestT
  }
  const wallMs = performance.now() - wall0
  track.push(terminalP)
  let sumSq = 0
  for (const p of track) {
    const d = deps.reference.distance(p)
    sumSq += d * d
  }
  const rmsCross = Math.sqrt(sumSq / Math.max(track.length, 1))
  const terminalPos = norm(sub(terminalP, GATE))
  const terminalVel = norm(sub(terminalV, GATE_V))
  const fuelKg = crossed ? s.fuel : closestFuel
  const massFinal = crossed ? s.mass : closestMass
  return {
    scenario: scenario.id,
    strategy: kind,
    label: STRATEGIES.find((x) => x.id === kind)?.label ?? kind,
    crossed,
    tContact,
    referenceDuration: deps.reference.duration,
    terminalP: [terminalP.x, terminalP.y, terminalP.z],
    terminalV: [terminalV.x, terminalV.y, terminalV.z],
    terminalPos,
    terminalVel,
    rmsCross,
    maxDirErrDeg,
    maxTiltDeg,
    fuelKg,
    massFinal,
    peakThrottle,
    peakThrustAccel,
    controllerMs: controller.stats.commandMs,
    wallMs,
    replans: controller.stats.replans,
    infeasibleReplans: controller.stats.infeasible,
    score: terminalPos + 2 * terminalVel + rmsCross,
  }
}

export interface StrategyAggregate {
  readonly strategy: StrategyKind
  readonly label: string
  readonly score: number
  readonly worstTerminalPos: number
  readonly worstTerminalVel: number
  readonly worstRms: number
  readonly fuelKg: number
  readonly controllerMs: number
  readonly replans: number
}

/** Sum the per-scenario composite; also keep the worst per-axis numbers for the table. */
export const aggregate = (runs: readonly RunResult[]): readonly StrategyAggregate[] => {
  const ids = STRATEGIES.map((s) => s.id)
  return ids
    .map((id) => {
      const rs = runs.filter((r) => r.strategy === id)
      return {
        strategy: id,
        label: rs[0]?.label ?? id,
        score: rs.reduce((sum, r) => sum + r.score, 0),
        worstTerminalPos: Math.max(...rs.map((r) => r.terminalPos), 0),
        worstTerminalVel: Math.max(...rs.map((r) => r.terminalVel), 0),
        worstRms: Math.max(...rs.map((r) => r.rmsCross), 0),
        fuelKg: rs.reduce((sum, r) => sum + r.fuelKg, 0),
        controllerMs: rs.reduce((sum, r) => sum + r.controllerMs, 0),
        replans: rs.reduce((sum, r) => sum + r.replans, 0),
      }
    })
    .filter((r) => runs.some((x) => x.strategy === r.strategy))
    .sort((a, b) => a.score - b.score)
}

/** Crosswind runs for the best two strategies. */
export const runCrosswind = (
  scenario: Scenario,
  kind: StrategyKind,
  config: RunConfig,
  speed = 10,
): RunResult => {
  const plant: PlantConfig = { ...config.plant, disturbance: crosswindAccel(speed) }
  return runScenario(scenario, kind, { ...config, plant })
}
