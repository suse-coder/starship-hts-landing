/**
 * MissionTuning: the defaults must be the flown tree's values (so threading it in changes nothing),
 * the schema must reject out-of-range/unknown input, and the optimisation knobs must stay inside the
 * field table.
 */
import { describe, expect, test } from "bun:test"
import {
  MISSION_TUNING_DEFAULT, MissionTuning, REFINE_KNOBS, SCREEN_KNOBS, TUNING_FIELDS, TuningError,
  clampKnob, knobFromUnit, knobToUnit, type TuningValues,
} from "../src/tuning"

describe("MissionTuning defaults", () => {
  test("reproduce the flown constants", () => {
    const T = MISSION_TUNING_DEFAULT
    expect(T.aimEntry).toBe(180e3)
    expect(T.aimHigh).toBe(900) // still air: the no-wind delivery is tuned here
    expect(T.bellyPlanEnd).toBe(1300) // the flip fall + the two-engine stop distance
    expect(T.bellyBankRateDeg).toBe(1.8)
    expect(T.gateMiss).toBe(420)
    expect(T.gateClosing).toBe(35)
    expect(T.gateLat).toBe(4.5)
    expect(T.flipDv).toBe(35)
    expect(T.flipFall).toBe(5.9)
    expect(T.landingEngines).toBe(2) // hard rule
    expect(T.contactSpeed).toBe(0.7)
    expect(T.landingWn).toBe(1.7)
    expect(T.landingWMaxDeg).toBe(26)
    expect(T.landingAxisAccelX).toBe(0.2)
    expect(T.landingAxisAccelYZ).toBe(0.7)
    expect(T.dvBurn).toBe(108)
    expect(T.rollBandEntryDeg).toBe(1.0)
    expect(T.rollBandAeroDeg).toBe(1.0)
    expect(T.rollBandBellyDeg).toBe(1.5)
    expect(T.rollBandPitchDeg).toBe(0) // pitch-up had no roll band
    expect(T.landingAimE).toBe(0)
    expect(T.landingAimN).toBe(0)
    expect(T.armCloseLat).toBe(1)
    expect(T.armCloseAlong).toBe(12)
    expect(T.armCloseAcrossV).toBe(0.6)
  })

  test("the schema table covers every field and every default is in range", () => {
    const keys = TUNING_FIELDS.map((f) => f.key)
    expect(new Set(keys).size).toBe(keys.length)
    for (const f of TUNING_FIELDS) {
      expect(f.min).toBeLessThanOrEqual(f.default)
      expect(f.default).toBeLessThanOrEqual(f.max)
      expect(MISSION_TUNING_DEFAULT[f.key]).toBe(f.default)
    }
    // every declared value field has a schema row
    const value = MISSION_TUNING_DEFAULT.toValues()
    for (const k of Object.keys(value)) expect(keys).toContain(k as keyof TuningValues)
  })
})

describe("MissionTuning validation", () => {
  test("with() returns a new validated tuning", () => {
    const T = MISSION_TUNING_DEFAULT.with({ gateMiss: 900, landingWn: 2.4 })
    expect(T.gateMiss).toBe(900)
    expect(T.landingWn).toBe(2.4)
    expect(MISSION_TUNING_DEFAULT.gateMiss).toBe(420) // immutable
  })

  test("rejects unknown fields, non-finite values and out-of-range values", () => {
    expect(() => MissionTuning.decode({ bogus: 1 })).toThrow(TuningError)
    expect(() => MissionTuning.decode({ gateMiss: Number.NaN })).toThrow(TuningError)
    expect(() => MissionTuning.decode({ gateMiss: 10 })).toThrow(TuningError)
    expect(() => MissionTuning.decode({ landingEngines: 3.5 })).toThrow(TuningError)
    expect(() => MISSION_TUNING_DEFAULT.with({ landingEngines: 4 })).toThrow(TuningError)
    // 3 engines are allowed when the 2-engine authority is not enough (the final gate is faster)
    expect(MISSION_TUNING_DEFAULT.with({ landingEngines: 3 }).landingEngines).toBe(3)
  })

  test("decode accepts a partial JSON object and fills the defaults", () => {
    const T = MissionTuning.decode({ gateMiss: 700, rollBandAeroDeg: 0.2 })
    expect(T.gateMiss).toBe(700)
    expect(T.rollBandAeroDeg).toBe(0.2)
    expect(T.aimHigh).toBe(900)
    expect(MissionTuning.decode(T.toJSON()).toValues()).toEqual(T.toValues())
  })
})

describe("optimisation knobs", () => {
  test("screen/refine knobs are inside the schema and round-trip through the unit mapping", () => {
    for (const k of [...SCREEN_KNOBS, ...REFINE_KNOBS]) {
      const f = TUNING_FIELDS.find((x) => x.key === k.key)
      expect(f).toBeDefined()
      if (!f) continue
      expect(k.min).toBeGreaterThanOrEqual(f.min)
      expect(k.max).toBeLessThanOrEqual(f.max)
      expect(clampKnob(k, k.min - 1)).toBe(k.min)
      expect(clampKnob(k, k.max + 1)).toBe(k.max)
      const u = knobToUnit(k, k.default)
      expect(knobFromUnit(k, u)).toBeCloseTo(k.default, 6)
    }
  })
})
