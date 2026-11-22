import { type V3 } from "@ship/math"
import { enuBasis, type Enu, geodeticToEcef, PAD2, type WindParams, meanWindEnu } from "@ship/earth"
import { SHOCK_TABLE, type ShockTable } from "@ship/gas"
import { type DesignParams, DEFAULT_DESIGN, makeVehicle, massProps, initialConsumables, type VehicleConfig } from "@ship/craft"
import { type AeroModel, makeAeroModel } from "@ship/aero"
import { makeThermalGrid, type ThermalGrid } from "@ship/heat"
import { makeMhdModel, type MhdModel } from "@ship/mhd"
import { TOWER } from "@ship/tower"
import {
  type AeroTable, alphaNoseFirst, buildAeroTable, buildLandingAero, buildMhdDragTable, DEFAULT_PLAN, type GuideCtx, type MhdDragTable,
  type MissionPlan, planMission, type PredContext,
} from "@ship/gnc"

/**
 * The immutable "world" of one simulation run: vehicle definition, precomputed physical models,
 * guidance tables and the targeted mission plan. Built once; every step is a pure function of
 * (world, state).
 */
export interface World {
  readonly design: DesignParams
  readonly vehicle: VehicleConfig
  readonly aero: AeroModel
  readonly mhd: MhdModel
  readonly grid: ThermalGrid
  readonly shock: ShockTable
  readonly aeroTab: AeroTable
  readonly mhdDrag: MhdDragTable
  readonly plan: MissionPlan
  readonly guide: GuideCtx
  readonly padEcef: V3
  readonly padEnu: Enu
  readonly wind: WindParams
}

export const TRANSITION = { vLoft: 6000, qPitch: 9000, vPitchMax: 3800 } as const

export const predContext = (aeroTab: AeroTable, mhdDrag: MhdDragTable, sref: number, theta0: number, padEcef: V3, windForecast?: (h: number) => V3): PredContext => ({
  aero: aeroTab, mhdDrag, sref, theta0, padEcef, alphaNoseFirst,
  vLoft: TRANSITION.vLoft, qPitch: TRANSITION.qPitch, vPitchMax: TRANSITION.vPitchMax, hEnd: 3000, windForecast,
})

export const makeWorld = (design: DesignParams = DEFAULT_DESIGN, log: (s: string) => void = () => {}): World => {
  const vehicle = makeVehicle(design)
  const aero = makeAeroModel(vehicle)
  const mhd = makeMhdModel(vehicle)
  const grid = makeThermalGrid(vehicle)
  const shock = SHOCK_TABLE
  const c0 = initialConsumables(vehicle)
  const entryMass = massProps(vehicle, { ...c0, loxMain: c0.loxMain * 0.3, ch4Main: c0.ch4Main * 0.3 })
  log("aerodynamic tables")
  const aeroTab = buildAeroTable(aero, entryMass.xcg)
  log("MHD drag table")
  const mhdDrag = buildMhdDragTable(vehicle, aero, shock, entryMass.xcg)
  const padEcef = geodeticToEcef(PAD2)
  log("targeting Flight 15 orbit → Pad 2")
  const windP: WindParams = { scaleMean: design.windScale, turbulence: design.turbulence, seed: design.seed }
  const windForecast = (h: number): V3 => meanWindEnu(h, windP)
  const plan = planMission((theta0, pad) => predContext(aeroTab, mhdDrag, aero.sref, theta0, pad, windForecast), { ...DEFAULT_PLAN, mass: massProps(vehicle, c0).m })
  const guide: GuideCtx = {
    vehicle, pred: predContext(aeroTab, mhdDrag, aero.sref, plan.theta0, padEcef, windForecast), aero: aeroTab,
    tBurn: plan.tBurn, dvBurn: plan.dvBurn, armHeight: TOWER.armHeight, catchHold: 8,
    tRamp: Math.max(plan.tBurn + 20, plan.pred.tEntry - 780),
    windForecast,
    // landing aero database, referenced to the CG with the main tanks empty and the headers full
    landAero: buildLandingAero(aero, massProps(vehicle, { ...c0, loxMain: 0, ch4Main: 0 }).xcg),
  }
  return {
    design, vehicle, aero, mhd, grid, shock, aeroTab, mhdDrag, plan, guide, padEcef,
    padEnu: enuBasis(PAD2.lat, PAD2.lon),
    wind: windP,
  }
}
