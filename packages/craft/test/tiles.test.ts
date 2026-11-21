import { describe, expect, test } from "bun:test"
import { makeVehicle, TILE, TILE_MATERIAL_OPTIONS, tileArealDensity, tileAreaFraction, tileCentroidX, tileLayerMass } from "../src/index"

const v = makeVehicle()

describe("silica tile mass (Q23)", () => {
  test("installed areal density = LI-900 tile + bond/pins/gap filler", () => {
    expect(tileArealDensity()).toBeCloseTo(TILE.density * TILE.thickness + TILE.bondArealMass, 9)
  })

  test("tile mass is positive and plausible for the flapless barrel (5–12 t)", () => {
    const m = tileLayerMass(v.shape, v.design)
    expect(m).toBeGreaterThan(5000)
    expect(m).toBeLessThan(12000)
  })

  test("tile area excludes the PICA-X nose and the absent flaps", () => {
    const f = tileAreaFraction(v.shape, v.design)
    expect(f).toBeGreaterThan(0.5)
    expect(f).toBeLessThan(1)
  })

  test("the mass item's centroid lies inside the tiled barrel", () => {
    const x = tileCentroidX(v.shape, v.design)
    expect(x).toBeGreaterThan(0)
    expect(x).toBeLessThan(v.shape.p.length)
  })

  test("surface-limit options: conservative LI-900 default, higher sourced SpaceX claim", () => {
    // the conservative reading is the one the verdict uses and must stay at the Shuttle limit
    expect(TILE_MATERIAL_OPTIONS.li900.surfaceLimitK).toBe(1530)
    // 1400 °C public claim for the flight tile, carried as an option only
    expect(TILE_MATERIAL_OPTIONS.starshipHex.surfaceLimitK).toBe(1673)
    expect(TILE_MATERIAL_OPTIONS.starshipHex.surfaceLimitK).toBeGreaterThan(TILE_MATERIAL_OPTIONS.li900.surfaceLimitK)
    // the option must not silently change the installed mass; material data is separate from mass
    expect(TILE_MATERIAL_OPTIONS.starshipHex.density).toBe(TILE.density)
  })
})
