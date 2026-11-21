/**
 * The plant model checks used by both `scripts/mpc-bench.ts` and
 * `packages/gnc/test/mpcBench.test.ts`.
 *
 * They are deliberately about the *plant*, not about a controller:
 *  - steady acceleration at 40/60/100 % for 2 and 3 SL Raptors (pressure correction included),
 *  - propellant flow over 10 s against T/(Isp·g0),
 *  - gimbal moment at ±15° against n·T·arm·sin δ / Iyy,
 *  - thrust/vacuum pressure correction at sea level and 1 km.
 *
 * Every check returns the measured value, the analytic value and the relative error so the script
 * can print a table and the test can assert tight tolerances.
 */
import { v3 } from "@ship/math"
import { atmosphere } from "@ship/earth"
import {
  ENGINE_MODEL,
  P_SL,
  BENCH_MASS,
  gimbalAngularAccel,
  massFlowTotal,
  netVerticalAcceleration,
  pressureAt,
  stepPlant,
  thrustPerEngine,
} from "./plant"
import type { BenchState, PlantConfig } from "./types"

export interface ModelCheck {
  readonly id: string
  readonly label: string
  readonly actual: number
  readonly expected: number
  readonly unit: string
  /** |actual − expected| / max(|expected|, 1e-12) */
  readonly relErr: number
  readonly tolerance: number
  readonly pass: boolean
}

const check = (
  id: string,
  label: string,
  actual: number,
  expected: number,
  unit: string,
  tolerance: number,
): ModelCheck => {
  const relErr = Math.abs(actual - expected) / Math.max(Math.abs(expected), 1e-12)
  return { id, label, actual, expected, unit, relErr, tolerance, pass: relErr <= tolerance }
}

const baseState = (z: number, mass: number): BenchState => ({
  t: 0,
  p: v3(0, 0, z),
  v: v3(0, 0, 0),
  d: v3(0, 0, 1),
  a: v3(0, 0, 0),
  mass,
  throttle: 0.4,
  fuel: 0,
  gimbal: 0,
  omega: 0,
})

const plantCfg = (nEng: number): PlantConfig => ({
  nEng,
  mode: "rate",
  rateCap: 0.45,
  tauDir: 0.35,
  tauThrottle: 0.3,
  gimbalKp: 1.5,
  gimbalKd: 1.8,
  gravity: 9.79,
  dt: 1e-3,
  disturbance: v3(0, 0, 0),
})

/** Steady vertical acceleration measured through one plant step. */
const measuredAccel = (nEng: number, throttle: number, z: number, mass: number): number => {
  const cfg = plantCfg(nEng)
  const s0: BenchState = { ...baseState(z, mass), throttle }
  const { next } = stepPlant(s0, { dir: v3(0, 0, 1), throttle }, cfg)
  return (next.v.z - s0.v.z) / cfg.dt
}

/** Integrate a fixed-throttle vertical burn for `seconds`, starting high enough not to hit the pad. */
const measuredBurn = (nEng: number, throttle: number, seconds: number): { readonly massLoss: number; readonly fuel: number } => {
  const cfg = plantCfg(nEng)
  let s: BenchState = { ...baseState(10_000, BENCH_MASS), throttle }
  const steps = Math.round(seconds / cfg.dt)
  for (let k = 0; k < steps; k++) s = stepPlant(s, { dir: v3(0, 0, 1), throttle }, cfg).next
  return { massLoss: BENCH_MASS - s.mass, fuel: s.fuel }
}

/** Torque-mode angular acceleration with the gimbal already held at its limit. */
const measuredGimbalAlpha = (nEng: number, throttle: number, angle: number): number => {
  const cfg: PlantConfig = { ...plantCfg(nEng), mode: "torque" }
  const s: BenchState = {
    ...baseState(800, BENCH_MASS),
    throttle,
    d: v3(1, 0, 0), // 90° from the command, so the gimbal PD stays saturated
    gimbal: angle,
  }
  const { applied } = stepPlant(s, { dir: v3(0, 0, 1), throttle }, cfg)
  return applied.alpha
}

export const runModelChecks = (): readonly ModelCheck[] => {
  const out: ModelCheck[] = []
  const g = 9.79
  const z = 800
  const pAmb = pressureAt(z)

  // 1. Commanded 2/3 engines at 40/60/100 %: initial acceleration within 1 %.
  for (const nEng of [2, 3]) {
    for (const throttle of [0.4, 0.6, 1.0]) {
      const expected = netVerticalAcceleration(nEng, throttle, BENCH_MASS, pAmb, g)
      const actual = measuredAccel(nEng, throttle, z, BENCH_MASS)
      out.push(check(
        `accel-${nEng}x-${Math.round(throttle * 100)}`,
        `${nEng}× Raptor ${Math.round(throttle * 100)} %: thrust/m − g`,
        actual,
        expected,
        "m/s²",
        0.01,
      ))
    }
  }

  // 2. Mass flow over 10 s against T/(Isp·g0). Pressure does not enter mdot; throttle is fixed.
  for (const throttle of [0.6, 1.0]) {
    const burn = measuredBurn(2, throttle, 10)
    const expected = 10 * massFlowTotal(2, throttle)
    out.push(check(
      `flow-${Math.round(throttle * 100)}`,
      `2× Raptor ${Math.round(throttle * 100)} %: 10 s propellant`,
      burn.massLoss,
      expected,
      "kg",
      0.001,
    ))
    out.push(check(
      `collector-${Math.round(throttle * 100)}`,
      `2× Raptor ${Math.round(throttle * 100)} %: fuel collector`,
      burn.fuel,
      burn.massLoss,
      "kg",
      0.001,
    ))
  }

  // 3. Gimbal moment at ±15° against n·T·arm·sin δ / Iyy. The plant's signed gimbal state always
  // drives the shortest rotation, so the positive case is measured through a plant step; the negative
  // case checks the signed moment formula (the same magnitude with the opposite sign).
  for (const sign of [1, -1]) {
    const angle = sign * ENGINE_MODEL.gimbalMax
    const T = thrustPerEngine(1, pAmb)
    const expected = 2 * T * ENGINE_MODEL.arm * Math.sin(angle) / ENGINE_MODEL.Iyy
    const actual = sign > 0
      ? measuredGimbalAlpha(2, 1, angle)
      : gimbalAngularAccel(2, T, angle)
    out.push(check(
      `gimbal-${sign > 0 ? "plus" : "minus"}-15`,
      `2× Raptor gimbal ${sign > 0 ? "+" : "−"}15°: T·arm·sin δ / Iyy`,
      actual,
      expected,
      "rad/s²",
      0.005,
    ))
  }
  // and the moment magnitude itself, at the same condition
  const T15 = thrustPerEngine(1, pAmb)
  const expectedMoment = 2 * T15 * ENGINE_MODEL.arm * Math.sin(ENGINE_MODEL.gimbalMax)
  const actualMoment = measuredGimbalAlpha(2, 1, ENGINE_MODEL.gimbalMax) * ENGINE_MODEL.Iyy
  out.push(check(
    "gimbal-moment-15",
    "2× Raptor +15°: gimbal moment",
    actualMoment,
    expectedMoment,
    "N·m",
    0.005,
  ))

  // 4. Thrust/vacuum pressure correction at sea level and 1 km.
  out.push(check(
    "vacuum-fvac",
    "sea-level vacuum thrust F_vac,100",
    ENGINE_MODEL.fVac100,
    ENGINE_MODEL.thrustSL + P_SL * ENGINE_MODEL.exitArea,
    "N",
    1e-12,
  ))
  out.push(check(
    "pressure-sea-level",
    "1× Raptor 100 % at sea level",
    thrustPerEngine(1, P_SL),
    ENGINE_MODEL.thrustSL,
    "N",
    1e-12,
  ))
  const p1km = atmosphere(1000).p
  out.push(check(
    "pressure-1km",
    "1× Raptor 100 % at 1 km",
    thrustPerEngine(1, p1km),
    ENGINE_MODEL.thrustSL + (P_SL - p1km) * ENGINE_MODEL.exitArea,
    "N",
    1e-12,
  ))
  out.push(check(
    "pressure-sea-level-40",
    "1× Raptor 40 % at sea level",
    thrustPerEngine(0.4, P_SL),
    0.4 * ENGINE_MODEL.fVac100 - P_SL * ENGINE_MODEL.exitArea,
    "N",
    1e-12,
  ))

  return out
}

export const allChecksPass = (checks: readonly ModelCheck[]): boolean => checks.every((c) => c.pass)
