import { clamp } from "@ship/math"
import { SIGMA_SB } from "@ship/earth"
import type { AftFlapSet } from "@ship/craft"

/**
 * Lumped thermal node for one titanium aft flap.
 *
 * The node represents the exposed 6 mm skin of a grid-fin/body-flap (the design's
 * `thermalThickness`), not the full grid envelope: the lattice's internal webs are not resolved.
 * Energy balance per flap:
 *
 *   m_skin c_p dT/dt = (q_base + q_plume) A − 2 ε σ A (T⁴ − T_space⁴)
 *
 * Two-sided radiation is used because a flap is a thin plate; the base/plume flux is applied to
 * the projected planform area. The implicit-in-T⁴ update is unconditionally stable for the long
 * 0.25 s thermal sub-steps.
 *
 * Material: Ti-6Al-4V. Density 4430 kg/m³, c_p ≈ 520–780 J/kg·K over 300–1200 K, ε ≈ 0.6
 * (oxidised titanium; polished Ti is ≈ 0.3, which would run hotter — 0.6 is used and stated).
 * Usable continuous temperature ≈ 600 °C (873 K) for strength/oxidation; melting ≈ 1660 °C
 * (1933 K). The usable limit is a design assumption, not a certified material allowable.
 */
export const TITANIUM_6AL4V = {
  density: 4430, // kg/m³ (AMS 4911 mill-annealed sheet)
  emissivity: 0.6, // oxidised Ti-6Al-4V, assumed
  conductivity: 6.7, // W/m·K at room temperature
  usable: 873.15, // K, ≈600 °C
  melting: 1933.15, // K, ≈1660 °C
  usableC: 600,
  meltingC: 1660,
} as const

/** c_p of Ti-6Al-4V, J/kg·K (rounded fit through 300–1200 K). */
export const titaniumCp = (T: number): number => clamp(520 + 0.25 * (T - 300), 500, 780)

export interface AftFlapThermalState {
  readonly T: number // K, lumped skin temperature
  readonly peakT: number // K
  readonly exposure: number // s with a significant external flux
}

export const initAftFlapThermal = (T0 = 290): AftFlapThermalState => ({ T: T0, peakT: T0, exposure: 0 })

export interface AftFlapThermalInput {
  readonly qBase: number // W/m², base/wake convective/radiative flux
  readonly qPlume: number // W/m², engine-plume flux
  readonly Tspace?: number // K, radiative sink (default 3 K)
}

/** One lumped-node step. Pure: returns a new state. */
export const aftFlapThermalStep = (
  state: AftFlapThermalState,
  set: AftFlapSet,
  inp: AftFlapThermalInput,
  dt: number,
): AftFlapThermalState => {
  if (dt <= 0 || set.area <= 0 || set.thermalThickness <= 0) return state
  const area = set.area
  const cp = titaniumCp(state.T)
  const C = set.design.density * cp * set.thermalThickness * area // J/K per flap skin
  if (C <= 0) return state
  const qIn = Math.max(0, inp.qBase) + Math.max(0, inp.qPlume)
  const tSpace = inp.Tspace ?? 3
  const eps = TITANIUM_6AL4V.emissivity
  const qRad = 2 * eps * SIGMA_SB * area * (state.T ** 4 - tSpace ** 4)
  const dq = 8 * eps * SIGMA_SB * area * state.T ** 3 // d(qRad)/dT
  const T = Math.max(tSpace, state.T + (dt * (qIn * area - qRad)) / (C + dt * dq))
  return {
    T,
    peakT: Math.max(state.peakT, T),
    exposure: state.exposure + (qIn > 1000 ? dt : 0),
  }
}

/** Radiative-equilibrium temperature of a thin two-sided plate, K. */
export const aftFlapEquilibriumTemperature = (
  qFlux: number,
  emissivity: number = TITANIUM_6AL4V.emissivity,
  sides = 2,
): number => (qFlux <= 0 ? 0 : (qFlux / (sides * emissivity * SIGMA_SB)) ** 0.25)

/**
 * Base/wake heat flux seen by the aft-mounted flap. `fraction` of the stagnation flux is a stated
 * assumption (0.02 nominal); the aft end is in the lee of the belly-first body, so this is small.
 */
export const aftBaseHeatFlux = (qStag: number, fraction: number): number =>
  Math.max(0, fraction) * Math.max(0, qStag)

/**
 * Engine-plume heat flux at the flap, stated as an assumption rather than a CFD result: a fraction
 * `fraction` of the jet thrust appears as a thermal load spread over a plume radius that grows
 * linearly from the 1.3 m Raptor exit. Nominal fraction 0.12; the 20–150 kW/m² band quoted in the
 * report is the sensitivity range for 1–3 engines at 2–5 m.
 */
export const plumeHeatFlux = (
  thrust: number,
  distance: number,
  fraction = 0.12,
  exitRadius = 1.3,
  spread = 0.25,
): number => {
  if (thrust <= 0) return 0
  const r = exitRadius + spread * Math.max(0, distance)
  return (Math.max(0, fraction) * thrust) / (Math.PI * r * r)
}

export type TitaniumVerdict = "nominal" | "over usable" | "melt"

export const titaniumVerdict = (T: number): TitaniumVerdict =>
  T <= TITANIUM_6AL4V.usable ? "nominal" : T < TITANIUM_6AL4V.melting ? "over usable" : "melt"

/** LI-900-class silica tile data used for the required-TPS estimate. */
export const FLAP_TPS = {
  label: "LI-900-class silica tile",
  density: 144, // kg/m³
  cp: 1000, // J/kg·K
  maxUse: 1530, // K
} as const

/**
 * Adiabatic heat-sink estimate of the insulation thickness needed to keep the titanium below
 * `Tlimit` for `duration` at a constant absorbed flux. Radiation is neglected, so this is a
 * conservative (thick) estimate; an ablator would be thinner but is single-use.
 */
export const requiredTpsThickness = (
  qFlux: number,
  duration: number,
  Tlimit = TITANIUM_6AL4V.usable,
  T0 = 400,
  material = FLAP_TPS,
): number => {
  const dT = Math.max(1, Tlimit - T0)
  return Math.max(0, (qFlux * Math.max(0, duration)) / (material.density * material.cp * dT))
}
