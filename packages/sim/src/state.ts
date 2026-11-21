import { type Q, type V3, qLookX, normalize, scale, ZERO } from "@ship/math"
import { type Consumables, engineOff, type EngineState, initialConsumables } from "@ship/craft"
import { initCoil, type CoilState, type MhdBasis, type MhdOut, ZERO_MHD } from "@ship/mhd"
import { initThermal, initAftFlapThermal, type AftFlapThermalState, type ThermalState } from "@ship/heat"
import { type AftFlapCommand, type AftFlapState, stowedAftFlapCommand, stowedAftFlapState } from "@ship/aero"
import { armsInit, type ArmState } from "@ship/tower"
import { turbInit, type TurbState } from "@ship/earth"
import { initMem, type Mem } from "@ship/gnc"
import type { World } from "./world"

/** Everything that evolves. Immutable: `step` returns a new value. */
export interface SimState {
  readonly t: number
  readonly r: V3 // ECI position, m
  readonly v: V3 // ECI velocity, m/s
  readonly q: Q // body → ECI
  readonly w: V3 // body rates, rad/s
  readonly cons: Consumables
  readonly eng: readonly EngineState[] // the three sea-level Raptors
  readonly coil: CoilState
  readonly th: ThermalState
  readonly thT: number
  readonly qStag: number // last computed stagnation heat flux (thermal sub-steps), W/m²
  readonly arms: ArmState
  readonly turb: TurbState
  /** deployable aft flap actuator state (stowed on the flown baseline) */
  readonly flaps: AftFlapState
  /** command consumed by the next step; guidance/MPC sets this, default stowed */
  readonly flapCmd: AftFlapCommand
  /** lumped titanium flap temperature; null when the vehicle has no aft flap set */
  readonly flapTh: AftFlapThermalState | null
  readonly mem: Mem
  readonly mhdBasis: MhdBasis | null
  readonly mhdT: number
  readonly mhdU: readonly number[]
  readonly mhdOut: MhdOut
  readonly rcsDuty: readonly number[]
  readonly seeding: boolean
  readonly events: readonly { readonly t: number; readonly msg: string }[]
  readonly done: boolean
  readonly outcome: "running" | "caught" | "lost" | "timeout"
  readonly peak: Peaks
}

export interface Peaks {
  readonly g: number
  readonly qbar: number
  readonly qStag: number
  readonly picaSurf: number
  readonly bondline: number
  readonly tileSurf: number
  readonly tileBack: number
  readonly steel: number
  readonly mhdForce: number
  readonly coilT: number
}
const PEAK0: Peaks = { g: 0, qbar: 0, qStag: 0, picaSurf: 0, bondline: 0, tileSurf: 0, tileBack: 0, steel: 0, mhdForce: 0, coilT: 0 }

export const initState = (w: World): SimState => {
  const p = w.plan
  // in orbit, already holding the deorbit-burn attitude (engines prograde, nose retrograde)
  const up = normalize(p.r0)
  const q = qLookX(scale(normalize(p.v0), -1), up)
  return {
    t: 0, r: p.r0, v: p.v0, q, w: ZERO,
    cons: initialConsumables(w.vehicle),
    eng: [engineOff, engineOff, engineOff],
    coil: initCoil(w.vehicle.coil),
    th: initThermal(w.grid, w.vehicle, 285),
    thT: 0,
    qStag: 0,
    arms: armsInit,
    turb: turbInit,
    flaps: stowedAftFlapState(w.vehicle.aftFlaps?.count ?? 3),
    flapCmd: stowedAftFlapCommand(w.vehicle.aftFlaps?.count ?? 3),
    flapTh: w.vehicle.aftFlaps ? initAftFlapThermal(285) : null,
    mem: initMem(p.bankEntry0),
    mhdBasis: null, mhdT: -1, mhdU: Array(8).fill(0), mhdOut: ZERO_MHD,
    rcsDuty: w.vehicle.thrusters.map(() => 0),
    seeding: false,
    events: [{ t: 0, msg: `in orbit ${(p.altitude / 1000).toFixed(0)} km, i ${((p.inclination * 180) / Math.PI).toFixed(1)}° (Ship 42, Flight 15, rev 14)` }],
    done: false, outcome: "running", peak: PEAK0,
  }
}

export const zeroV = scale(ZERO, 0)
