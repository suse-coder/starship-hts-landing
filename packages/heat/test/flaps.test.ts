/**
 * Lumped Ti-6Al-4V aft-flap thermal node: energy balance, radiative equilibrium, the 35/20/5 km
 * base-flow points and the landing-plume sensitivity.
 *
 * Base flux is a stated 2 % of stagnation flux (aft/lee surface). Plume flux uses the documented
 * 12 %-of-thrust plume-spreading assumption in `plumeHeatFlux`, not a CFD result.
 */
import { describe, expect, test } from "bun:test"
import { SIGMA_SB } from "@ship/earth"
import { DEFAULT_AFT_FLAPS, DEFAULT_DESIGN, makeVehicle } from "@ship/craft"
import {
  FLAP_TPS, TITANIUM_6AL4V, aftBaseHeatFlux, aftFlapEquilibriumTemperature, aftFlapThermalStep,
  initAftFlapThermal, plumeHeatFlux, requiredTpsThickness, titaniumCp, titaniumVerdict,
} from "../src/index"

const v = makeVehicle({ ...DEFAULT_DESIGN, aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true } })
const set = v.aftFlaps!
const rel = (a: number, b: number): number => Math.abs(a / b - 1)

describe("lumped-node energy", () => {
  test("one small step stores the net flux: C ΔT = q_net A Δt (0.1 %)", () => {
    const T0 = 400, dt = 0.01, q = 5e4
    const s0 = initAftFlapThermal(T0)
    const s1 = aftFlapThermalStep(s0, set, { qBase: q, qPlume: 0 }, dt)
    const C = TITANIUM_6AL4V.density * titaniumCp(T0) * set.thermalThickness * set.area
    const qRad = 2 * TITANIUM_6AL4V.emissivity * SIGMA_SB * (T0 ** 4 - 3 ** 4)
    expect(rel(C * (s1.T - T0), dt * (q - qRad) * set.area)).toBeLessThan(0.01)
    expect(s1.T).toBeGreaterThan(T0)
    expect(s1.exposure).toBeCloseTo(dt, 12)
  })

  test("constant flux converges to the two-sided radiative equilibrium (1 K)", () => {
    const q = 5e4
    let s = initAftFlapThermal(290)
    for (let k = 0; k < 6000; k++) s = aftFlapThermalStep(s, set, { qBase: q, qPlume: 0 }, 1)
    expect(s.T).toBeCloseTo(aftFlapEquilibriumTemperature(q), 0)
    expect(s.peakT).toBeGreaterThanOrEqual(s.T)
  })

  test("no flux cools the node toward space (never below the sink)", () => {
    let s = initAftFlapThermal(800)
    for (let k = 0; k < 5000; k++) s = aftFlapThermalStep(s, set, { qBase: 0, qPlume: 0 }, 1)
    expect(s.T).toBeLessThan(800)
    expect(s.T).toBeGreaterThanOrEqual(3)
  })
})

describe("base flow at 35 / 20 / 5 km", () => {
  // representative trajectory (agentL m14): q_stag ≈ 50, 1.6, 0.23 kW/m² at 35, 20, 5 km
  const qStag = [50e3, 1.6e3, 0.23e3] as const
  const qBase = qStag.map((q) => aftBaseHeatFlux(q, set.baseHeatFraction))
  const Teq = qBase.map((q) => aftFlapEquilibriumTemperature(q))

  test("2 % of stagnation flux: 35 km is 300–450 K; 20 and 5 km are colder and negligible", () => {
    expect(qBase[0]).toBeCloseTo(1000, 12)
    expect(Teq[0]).toBeGreaterThan(300)
    expect(Teq[0]).toBeLessThan(450)
    expect(Teq[1]).toBeLessThan(Teq[0])
    expect(Teq[2]).toBeLessThan(Teq[1])
    expect(Teq[2]).toBeLessThan(250)
  })

  test("even a 10 % base-heating fraction stays below the 873 K usable limit at 35 km", () => {
    expect(aftFlapEquilibriumTemperature(aftBaseHeatFlux(50e3, 0.1))).toBeLessThan(TITANIUM_6AL4V.usable)
  })
})

describe("landing plume", () => {
  test("2 Raptors at the design plume distance: over 600 °C usable, below 1660 °C melting", () => {
    const q = plumeHeatFlux(2 * 2.45e6, set.plumeDistance)
    const T = aftFlapEquilibriumTemperature(q)
    expect(q).toBeGreaterThan(2e4)
    expect(q).toBeLessThan(8e4)
    expect(T).toBeGreaterThan(TITANIUM_6AL4V.usable)
    expect(T).toBeLessThan(TITANIUM_6AL4V.melting)
    expect(titaniumVerdict(T)).toBe("over usable")
  })

  test("3 Raptors at 2 m is hotter but still far from melting", () => {
    const q = plumeHeatFlux(3 * 2.45e6, 2.0)
    const T = aftFlapEquilibriumTemperature(q)
    expect(T).toBeGreaterThan(aftFlapEquilibriumTemperature(plumeHeatFlux(2 * 2.45e6, set.plumeDistance)))
    expect(T).toBeLessThan(TITANIUM_6AL4V.melting)
  })

  test("melting needs ≈1 MW/m²: no plausible aft-plume flux reaches it", () => {
    expect(aftFlapEquilibriumTemperature(1e6)).toBeGreaterThan(TITANIUM_6AL4V.melting)
    expect(aftFlapEquilibriumTemperature(3e5)).toBeLessThan(TITANIUM_6AL4V.melting)
  })

  test("a high-emissivity coating (ε 0.9) helps but does not clear 600 °C at 100 kW/m²", () => {
    expect(aftFlapEquilibriumTemperature(1e5, 0.9)).toBeGreaterThan(TITANIUM_6AL4V.usable)
  })

  test("required TPS thickness to hold 600 °C for a 20 s plume: 10–30 mm LI-900 class", () => {
    const q = plumeHeatFlux(2 * 2.45e6, set.plumeDistance)
    const d = requiredTpsThickness(q, 20)
    expect(d).toBeGreaterThan(0.01)
    expect(d).toBeLessThan(0.03)
    expect(FLAP_TPS.maxUse).toBeGreaterThan(TITANIUM_6AL4V.usable)
  })

  test("small set is a modest heat load; the area needed for stabilisation is not", () => {
    const q = plumeHeatFlux(2 * 2.45e6, set.plumeDistance)
    expect(q * set.totalArea).toBeLessThan(250e3) // < 250 kW over 4.2 m²
    const large = makeVehicle({
      ...DEFAULT_DESIGN,
      aftFlaps: { ...DEFAULT_AFT_FLAPS, enabled: true, span: 6, chord: 4.2 },
    }).aftFlaps!
    expect(q * large.totalArea).toBeGreaterThan(2e6) // > 2 MW over 75.6 m²
  })

  test("material verdict bands", () => {
    expect(titaniumVerdict(TITANIUM_6AL4V.usable)).toBe("nominal")
    expect(titaniumVerdict(TITANIUM_6AL4V.usable + 1)).toBe("over usable")
    expect(titaniumVerdict(TITANIUM_6AL4V.melting - 1)).toBe("over usable")
    expect(titaniumVerdict(TITANIUM_6AL4V.melting)).toBe("melt")
  })
})
