/**
 * The atmosphere model on the CPU (packages/view/src/atmosphere.ts): the same integral the sky
 * shader evaluates, so these tests pin the physics of the sky colour, the colour of sunlight, the
 * aerial perspective and the exposure. Numbers only — what the GPU draws is not covered here.
 */
import { describe, expect, test } from "bun:test"
import {
  ATM, ATMOSPHERE_GLSL, aerialPerspective, exposureForSun, henyeyGreenstein, luminance, raySphere, scatter, skyAmbient, skyRadiance, sunDepth,
  sunTransmittance,
} from "../src/atmosphere"

const rad = (d: number): number => (d * Math.PI) / 180
const sunAt = (elevDeg: number): [number, number, number] => [Math.cos(rad(elevDeg)), Math.sin(rad(elevDeg)), 0]
const vec = (): Float64Array => new Float64Array(3)

describe("ray / sphere", () => {
  test("entry and exit distances through the centre, and a miss", () => {
    const o = new Float64Array(2)
    raySphere(0, 0, -10, 0, 0, 1, 2, o)
    expect(o[0]).toBeCloseTo(8, 12)
    expect(o[1]).toBeCloseTo(12, 12)
    raySphere(0, 5, -10, 0, 0, 1, 2, o)
    expect(o[0]).toBeGreaterThan(o[1]) // miss: empty interval
  })
})

describe("phase function", () => {
  test("Henyey–Greenstein integrates to 1 over the sphere and peaks forward for g > 0", () => {
    for (const g of [0, 0.3, 0.6, 0.76, -0.3]) {
      let s = 0
      const n = 20000
      for (let i = 0; i < n; i++) { const mu = -1 + (2 * (i + 0.5)) / n; s += henyeyGreenstein(mu, g) * (2 / n) * 2 * Math.PI }
      expect(s).toBeCloseTo(1, 3)
    }
    expect(henyeyGreenstein(1, 0.6)).toBeGreaterThan(henyeyGreenstein(-1, 0.6))
    expect(henyeyGreenstein(0.3, 0)).toBeCloseTo(1 / (4 * Math.PI), 12)
  })
})

describe("direct sunlight", () => {
  test("vertical column densities equal the scale heights (Rayleigh 8 km, Mie 1.2 km, ozone 15 km)", () => {
    const od = vec()
    sunDepth(0, ATM.RE, 0, 0, 1, 0, od, 400)
    expect(od[0]).toBeCloseTo(ATM.HR * (1 - Math.exp(-(ATM.RA - ATM.RE) / ATM.HR)), 2)
    expect(od[1]).toBeCloseTo(ATM.HM, 2)
    expect(od[2]).toBeCloseTo(ATM.OZ_W, 1)
  })
  test("zenith Sun at sea level: Beer–Lambert with the vertical optical depths", () => {
    const t = vec()
    sunTransmittance(0, 1, t)
    for (let c = 0; c < 3; c++) {
      const tau = ATM.BR[c] * ATM.HR + ATM.BM * ATM.BM_EXT * ATM.HM + ATM.BO[c] * ATM.OZ_W
      expect(t[c]).toBeCloseTo(Math.exp(-tau), 2)
    }
  })
  test("reddens and dims towards the horizon, is white above the atmosphere, black behind the Earth", () => {
    const hi = vec(), lo = vec(), top = vec(), night = vec()
    sunTransmittance(0, Math.sin(rad(60)), hi)
    sunTransmittance(0, Math.sin(rad(3)), lo)
    expect(lo[0]).toBeLessThan(hi[0])
    expect(lo[0]).toBeGreaterThan(lo[1])
    expect(lo[1]).toBeGreaterThan(lo[2])
    expect(lo[2] / lo[0]).toBeLessThan(hi[2] / hi[0]) // redder
    sunTransmittance(150, 0.3, top)
    expect([...top]).toEqual([1, 1, 1])
    sunTransmittance(0, Math.sin(rad(-5)), night)
    expect([...night]).toEqual([0, 0, 0])
    // from 30 km the Sun 3° below the local horizontal is still above the limb (dip ≈ 5.5°)
    sunTransmittance(30, Math.sin(rad(-3)), night)
    expect(night[0]).toBeGreaterThan(0)
  })
  test("less air above: transmittance rises with altitude", () => {
    let prev = 0
    for (const h of [0, 2, 5, 10, 20, 40]) {
      const t = vec()
      sunTransmittance(h, Math.sin(rad(20)), t)
      expect(luminance(t)).toBeGreaterThan(prev)
      prev = luminance(t)
    }
  })
})

describe("sky radiance", () => {
  test("clear midday sky: blue zenith, brighter and whiter horizon, brightest towards the Sun", () => {
    const [sx, sy, sz] = sunAt(35)
    const zen = vec(), hor = vec(), sunSide = vec(), away = vec()
    skyRadiance(0.05, 0, 1, 0, sx, sy, sz, zen)
    skyRadiance(0.05, 0, Math.sin(rad(3)), Math.cos(rad(3)), sx, sy, sz, hor)
    skyRadiance(0.05, Math.cos(rad(3)), Math.sin(rad(3)), 0, sx, sy, sz, sunSide)
    skyRadiance(0.05, -Math.cos(rad(3)), Math.sin(rad(3)), 0, sx, sy, sz, away)
    expect(zen[2]).toBeGreaterThan(zen[1])
    expect(zen[1]).toBeGreaterThan(zen[0])
    expect(luminance(hor)).toBeGreaterThan(luminance(zen))
    expect(hor[2] / hor[0]).toBeLessThan(zen[2] / zen[0]) // less saturated at the horizon
    expect(luminance(sunSide)).toBeGreaterThan(luminance(hor))
    expect(luminance(sunSide)).toBeGreaterThan(luminance(away))
  })
  test("the mean sky radiance matches the calibration the cloud lighting was tuned to (35° Sun)", () => {
    // clouds.ts: cos-weighted mean upper-sky radiance ≈ (0.16, 0.27, 0.42) at sea level
    const a = vec()
    const [sx, sy, sz] = sunAt(35)
    skyAmbient(0.05, sx, sy, sz, a)
    expect(a[0]).toBeGreaterThan(0.12); expect(a[0]).toBeLessThan(0.22)
    expect(a[1]).toBeGreaterThan(0.21); expect(a[1]).toBeLessThan(0.33)
    expect(a[2]).toBeGreaterThan(0.34); expect(a[2]).toBeLessThan(0.52)
  })
  test("darkens with a setting Sun, is black at night and in space", () => {
    const day = vec(), dusk = vec(), night = vec(), space = vec()
    skyAmbient(0.05, ...sunAt(35), day)
    skyAmbient(0.05, ...sunAt(2), dusk)
    skyAmbient(0.05, ...sunAt(-25), night)
    expect(luminance(dusk)).toBeLessThan(luminance(day))
    expect(luminance(night)).toBeLessThan(1e-6)
    skyRadiance(200, 0, 1, 0, ...sunAt(35), space) // above the atmosphere, looking up
    expect(luminance(space)).toBe(0)
  })
  test("thinner with altitude (≈ exp(−h/8 km) overhead)", () => {
    const a = vec(), b = vec()
    skyRadiance(0.05, 0, 1, 0, ...sunAt(50), a)
    skyRadiance(16, 0, 1, 0, ...sunAt(50), b)
    const ratio = luminance(b) / luminance(a)
    expect(ratio).toBeGreaterThan(0.05)
    expect(ratio).toBeLessThan(0.35)
  })
  test("every value is finite and non-negative over the sphere of directions and Sun positions", () => {
    const o = vec(), t = vec()
    for (const alt of [0, 0.05, 3, 30, 99, 150]) for (const el of [-30, -2, 0, 5, 40, 90]) {
      const [sx, sy, sz] = sunAt(el)
      for (let i = 0; i < 12; i++) for (let j = 0; j < 6; j++) {
        const th = Math.acos(1 - (2 * (i + 0.5)) / 12), ph = (j / 6) * Math.PI * 2
        skyRadiance(alt, Math.sin(th) * Math.cos(ph), Math.cos(th), Math.sin(th) * Math.sin(ph), sx, sy, sz, o, t)
        for (let c = 0; c < 3; c++) {
          expect(Number.isFinite(o[c])).toBe(true)
          expect(o[c]).toBeGreaterThanOrEqual(0)
          expect(t[c]).toBeGreaterThanOrEqual(0)
          expect(t[c]).toBeLessThanOrEqual(1)
        }
      }
    }
  })
})

describe("segment integration", () => {
  const o: [number, number, number] = [0, ATM.RE + 0.2, 0]
  const d: [number, number, number] = [Math.cos(rad(1)), Math.sin(rad(1)), 0]
  const s = sunAt(30)
  test("transmittance of a path is the product of its parts; in-scatter adds up along it", () => {
    const L = vec(), T = vec(), L1 = vec(), T1 = vec(), L2 = vec(), T2 = vec()
    scatter(...o, ...d, 0, 40, ...s, false, L, T)
    scatter(...o, ...d, 0, 15, ...s, false, L1, T1)
    scatter(...o, ...d, 15, 40, ...s, false, L2, T2)
    for (let c = 0; c < 3; c++) {
      expect(T1[c] * T2[c]).toBeCloseTo(T[c], 2)
      const sum = L1[c] + T1[c] * L2[c]
      expect(Math.abs(sum - L[c]) / L[c]).toBeLessThan(0.05)
    }
  })
  test("uniform and quadratic sample spacing agree on a short path", () => {
    const a = vec(), b = vec(), t = vec()
    scatter(...o, ...d, 0, 8, ...s, false, a, t)
    scatter(...o, ...d, 0, 8, ...s, true, b, t)
    for (let c = 0; c < 3; c++) expect(Math.abs(a[c] - b[c]) / a[c]).toBeLessThan(0.03)
  })
})

describe("aerial perspective", () => {
  const s = sunAt(35)
  const at = (distKm: number): { L: Float64Array; T: Float64Array } => {
    const L = vec(), T = vec()
    aerialPerspective(0, ATM.RE + 0.1, 0, 0, ATM.RE + 0.1 * (1 - Math.min(1, distKm / 30)), distKm, ...s, L, T)
    return { L, T }
  }
  test("no path, no haze", () => {
    const { L, T } = at(0)
    expect([...L]).toEqual([0, 0, 0])
    expect([...T]).toEqual([1, 1, 1])
  })
  test("haze grows and the scene fades with distance; blue is extinguished first", () => {
    let pl = 0, pt = 1
    for (const dkm of [0.5, 2, 8, 20, 30]) {
      const { L, T } = at(dkm)
      expect(luminance(L)).toBeGreaterThan(pl)
      expect(luminance(T)).toBeLessThan(pt)
      expect(T[2]).toBeLessThan(T[0])
      pl = luminance(L); pt = luminance(T)
    }
  })
  test("a long level path converges on the colour of the sky at the horizon (no seam at the horizon)", () => {
    const L = vec(), T = vec(), sky = vec()
    // 300 km along a grazing ray (the target is where that ray is 300 km out)
    const el = rad(0.3), dkm = 300
    aerialPerspective(0, ATM.RE + 0.1, 0, 0, ATM.RE + 0.1 + dkm * Math.sin(el), dkm * Math.cos(el), ...s, L, T)
    skyRadiance(0.1, 0, Math.sin(el), Math.cos(el), ...s, sky)
    expect(luminance(T)).toBeLessThan(0.05)
    for (let c = 0; c < 3; c++) expect(Math.abs(L[c] - Math.min(sky[c], ATM.SKY_MAX)) / sky[c]).toBeLessThan(0.15)
  })
})

describe("exposure", () => {
  test("base value at the 35° reference and above the weather", () => {
    expect(exposureForSun(Math.sin(rad(35)), 0)).toBeCloseTo(0.6, 6)
    for (const se of [-1, -0.1, 0, 0.3, 1]) expect(exposureForSun(se, 80e3)).toBeCloseTo(0.6, 9)
  })
  test("opens up monotonically as the Sun goes down, within 0.54 … 1.08", () => {
    let prev = 0
    for (let el = 90; el >= -30; el -= 2) {
      const e = exposureForSun(Math.sin(rad(el)), 0)
      expect(e).toBeGreaterThanOrEqual(prev - 1e-9)
      expect(e).toBeGreaterThanOrEqual(0.54 - 1e-9)
      expect(e).toBeLessThanOrEqual(1.08 + 1e-9)
      prev = e
    }
    expect(exposureForSun(1, 0)).toBeCloseTo(0.54, 6)
    expect(exposureForSun(-0.5, 0)).toBeCloseTo(1.08, 6)
  })
  test("continuous through the 20–60 km blend and robust to bad input", () => {
    let prev = exposureForSun(0.05, 0)
    for (let h = 0; h <= 80e3; h += 500) { const e = exposureForSun(0.05, h); expect(Math.abs(e - prev)).toBeLessThan(0.02); prev = e }
    expect(Number.isFinite(exposureForSun(NaN, NaN))).toBe(true)
  })
})

describe("shader source", () => {
  test("the GLSL carries the same constants and sample counts as the CPU port", () => {
    expect(ATMOSPHERE_GLSL).toContain("ATM_RE = 6371.0")
    expect(ATMOSPHERE_GLSL).toContain("ATM_RA = 6471.0")
    expect(ATMOSPHERE_GLSL).toContain("ATM_BR = vec3(0.0058, 0.0135, 0.0331)")
    expect(ATMOSPHERE_GLSL).toContain("ATM_BO = vec3(0.00065, 0.001881, 0.000085)")
    expect(ATMOSPHERE_GLSL).toContain(`i < ${ATM.N_VIEW};`)
    expect(ATMOSPHERE_GLSL).toContain(`j < ${ATM.N_LIGHT};`)
    expect(ATMOSPHERE_GLSL).toContain("const float g = 0.76;")
    const open = ATMOSPHERE_GLSL.split("{").length, close = ATMOSPHERE_GLSL.split("}").length
    expect(open).toBe(close)
  })
})
