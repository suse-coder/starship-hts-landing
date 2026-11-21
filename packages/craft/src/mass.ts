import { type V3, v3 } from "@ship/math"
import type { Body, VehicleConfig } from "./config"

/** Consumable masses (kg). */
export interface Consumables {
  readonly loxHeader: number
  readonly ch4Header: number
  readonly loxMain: number
  readonly ch4Main: number
  readonly n2: number
  readonly seed: number
}

export interface MassProps {
  readonly m: number
  readonly xcg: number
  readonly I: V3 // principal moments about the CG (Ixx roll, Iyy, Izz)
}

/** moments of a primitive about its own centre: [Ixx, Iyy] (Izz = Iyy by symmetry) */
const own = (b: Body, m: number): readonly [number, number] => {
  switch (b.kind) {
    case "point": return [m * b.r * b.r, 0.5 * m * b.r * b.r]
    case "shell": return [m * b.r * b.r, m * (b.r * b.r / 2 + b.l * b.l / 12)]
    case "solid": return [0.5 * m * b.r * b.r, m * (b.r * b.r / 4 + b.l * b.l / 12)]
    case "ring": return [m * b.r * b.r, 0.5 * m * b.r * b.r]
    case "disk": return [0.5 * m * b.r * b.r, 0.25 * m * b.r * b.r]
    case "sphere": return [0.4 * m * b.r * b.r, 0.4 * m * b.r * b.r]
  }
}

interface Item { m: number; x: number; ixx: number; iyy: number }

export const dryItems = (v: VehicleConfig): Item[] =>
  v.components.map((c) => {
    const [ixx, iyy] = own(c.body, c.mass)
    return { m: c.mass, x: c.x, ixx, iyy }
  })

export const consumableItems = (v: VehicleConfig, c: Consumables): Item[] => {
  // liquids: treated as solid spheres/disks at the tank sump (slosh neglected: header tanks are baffled)
  const sph = (m: number, x: number, r: number): Item => ({ m, x, ixx: 0.4 * m * r * r, iyy: 0.4 * m * r * r })
  return [
    sph(c.loxHeader, v.xLoxHeader, 1.3),
    sph(c.ch4Header, v.xCh4Header, 1.2),
    { m: c.loxMain, x: v.xLoxMain, ixx: 0.5 * c.loxMain * 16, iyy: 0.25 * c.loxMain * 16 },
    { m: c.ch4Main, x: v.xCh4Main, ixx: 0.5 * c.ch4Main * 16, iyy: 0.25 * c.ch4Main * 16 },
    { m: c.n2, x: v.xN2, ixx: c.n2 * 9, iyy: c.n2 * 5 },
    sph(c.seed, 50.2, 0.3),
  ]
}

export const massProps = (v: VehicleConfig, c: Consumables): MassProps => {
  const items = [...dryItems(v), ...consumableItems(v, c)]
  const m = items.reduce((s, i) => s + i.m, 0)
  const xcg = items.reduce((s, i) => s + i.m * i.x, 0) / m
  const Ixx = items.reduce((s, i) => s + i.ixx, 0)
  const Iyy = items.reduce((s, i) => s + i.iyy + i.m * (i.x - xcg) ** 2, 0)
  return { m, xcg, I: v3(Ixx, Iyy, Iyy) }
}

export const dryMass = (v: VehicleConfig): number => v.components.reduce((s, c) => s + c.mass, 0)

/** consumables at simulation start (in orbit, before the deorbit burn) */
export const initialConsumables = (v: VehicleConfig): Consumables => {
  const mr = v.mixtureRatio
  const lp = v.design.landingProp
  const deorbit = 3300
  const residual = 1550
  return {
    loxHeader: (lp * mr) / (1 + mr),
    ch4Header: lp / (1 + mr),
    loxMain: ((deorbit + residual) * mr) / (1 + mr),
    ch4Main: (deorbit + residual) / (1 + mr),
    n2: v.design.n2Mass,
    seed: v.coil.seedMass,
  }
}
