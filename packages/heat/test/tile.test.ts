/**
 * spec/05 T-tile — silica tile heat shield on the steel backing (Q23).
 *
 * The tile is a 1-D conduction layer (6 nodes), an RTV bond and the lumped steel backing. The checks:
 *  - the surface reaches the radiative equilibrium q = εσT⁴ + conduction into the tile;
 *  - one step stores exactly what the surface absorbs minus what it re-radiates and leaks inward;
 *  - the tile really insulates: the backing stays far below the surface over the entry timescale;
 *  - the tile and PICA cover disjoint cells (no double protection, no bare barrel on the default ship).
 */
import { describe, expect, test } from "bun:test"
import { SIGMA_SB } from "@ship/earth"
import { makeVehicle } from "@ship/craft"
import {
  makeThermalGrid, initThermal, thermalStep, NP, TILE_NODES, tileNodeFractions, tileConductivity, tileHeatCapacity,
  type HeatOut,
} from "../src/index"

const v = makeVehicle()
const g = makeThermalGrid(v)
const n = g.cellX.length
const rel = (a: number, b: number): number => Math.abs(a / b - 1)

/** a pure radiative (absorbed) flux on every cell, no convection */
const radiativeLoad = (q: number): HeatOut => ({
  ch: new Float64Array(n), hr: new Float64Array(n), qRad: new Float64Array(n).fill(q), pe: new Float64Array(n), qStagCold: 0, qStagRad: 0, turbulentFrac: 0,
})

const firstTile = (): number => {
  for (let i = 0; i < n; i++) if (g.cellTile[i]) return i
  return -1
}

describe("tile heat shield (Q23)", () => {
  test("coverage: tiles and PICA are disjoint, and the default barrel is tiled", () => {
    let tiles = 0, pica = 0, both = 0
    for (let i = 0; i < n; i++) {
      if (g.cellTile[i]) tiles++
      if (g.cellPica[i]) pica++
      if (g.cellTile[i] && g.cellPica[i]) both++
    }
    expect(tiles).toBeGreaterThan(0)
    expect(pica).toBeGreaterThan(0)
    expect(both).toBe(0)
    expect(tiles + pica).toBe(n)
  })

  test("surface radiative equilibrium within 3 % of q = εσT⁴ (conduction loss included)", () => {
    const i = firstTile()
    const q = 5e4
    let s = initThermal(g, v, 290)
    for (let k = 0; k < 600; k++) s = thermalStep(g, v, s, radiativeLoad(q), 1)
    const Ts = s.T[i * NP] // node 0 of cell i; the tile uses the first TILE_NODES nodes
    const Tb = s.Tsub[i]
    const qRad = 0.8 * SIGMA_SB * Ts ** 4
    // the tile's own conduction is the only other sink; estimate it with k ΔT / L
    const L = 0.03
    const qTile = tileConductivity(0.5 * (Ts + Tb)) * (Ts - Tb) / L
    expect(qRad + qTile).toBeGreaterThan(0.97 * q)
    expect(qRad + qTile).toBeLessThan(1.03 * q)
  })

  test("one step conserves energy: stored = absorbed − re-radiated − interior leak (< 0.5 %)", () => {
    const i = firstTile()
    const q = 1e5, dt = 0.01, T0 = 290
    const s0 = initThermal(g, v, T0)
    const s1 = thermalStep(g, v, s0, radiativeLoad(q), dt)
    const FR = tileNodeFractions()
    let dE = 0
    for (let k = 0; k < TILE_NODES; k++) {
      const Tm = 0.5 * (s0.T[i * NP + k] + s1.T[i * NP + k])
      dE += 144 * FR[k] * 0.03 * tileHeatCapacity(Tm) * (s1.T[i * NP + k] - s0.T[i * NP + k])
    }
    dE += (7900 * v.steelThickness * (450 + 0.28 * (T0 - 300)) + 1500 * 0.0002 * 1200) * (s1.Tsub[i] - s0.Tsub[i])
    const qNet = q - 0.8 * SIGMA_SB * T0 ** 4 - 0.3 * SIGMA_SB * (T0 ** 4 - 250 ** 4)
    expect(rel(dE, qNet * dt)).toBeLessThan(0.005)
  })

  test("the tile insulates: after 300 s at 50 kW/m² the backing stays below 400 K", () => {
    const i = firstTile()
    let s = initThermal(g, v, 290)
    for (let k = 0; k < 300; k++) s = thermalStep(g, v, s, radiativeLoad(5e4), 1)
    expect(s.T[i * NP]).toBeGreaterThan(800)
    expect(s.Tsub[i]).toBeLessThan(400)
  })

  test("surface limit is set by the heat flux, not the coating: no physical ε holds 1530 K", () => {
    // mission peak (2026-11-21, 99 m/s entry): tile surface 1760 K at the model's ε = 0.8
    const qAbs = 0.8 * SIGMA_SB * 1760 ** 4 // 435 kW/m² absorbed
    const blackBody = (qAbs / SIGMA_SB) ** 0.25 // what ε = 1 would give
    expect(blackBody).toBeGreaterThan(1530) // 1664 K: even a black body still exceeds LI-900
    expect(blackBody).toBeLessThan(1670)
    const epsReq = qAbs / (SIGMA_SB * 1530 ** 4)
    expect(epsReq).toBeCloseTo(1.4, 1) // unphysical (> 1)
    expect(epsReq).toBeGreaterThan(1)
    // equivalently the absorbed flux must fall by (1760/1530)^4 = 1.75x
    expect((1760 / 1530) ** 4).toBeCloseTo(1.75, 2)
  })

  test("mission-scale pulse (435 kW/m² for the ~250 s above 1530 K) leaves the backing far below 600 K", () => {
    const i = firstTile()
    const q = 0.8 * SIGMA_SB * 1760 ** 4
    let s = initThermal(g, v, 290)
    for (let k = 0; k < 250; k++) s = thermalStep(g, v, s, radiativeLoad(q), 1)
    expect(s.T[i * NP]).toBeGreaterThan(1700) // surface is still over the limit
    expect(s.Tsub[i]).toBeLessThan(400) // measured mission peak 328 K; adhesive limit 600 K
  })
})
