/** Quality levels and the frame-time governor (packages/view/src/quality.ts). */
import { describe, expect, test } from "bun:test"
import { makeGovernor, msaaSamples, QUALITY_LEVELS } from "../src/quality"

const feed = (g: ReturnType<typeof makeGovernor>, dt: number, frames: number): number => {
  let l = g.level()
  for (let i = 0; i < frames; i++) l = g.sample(dt)
  return l
}

describe("quality levels", () => {
  test("every step is cheaper than the one before, and none of them lowers the resolution or removes the anti-aliasing", () => {
    for (const q of QUALITY_LEVELS) {
      expect(q.pixelRatioScale).toBe(1) // the vehicle stays at native resolution on every level
      expect(q.msaa).toBeGreaterThanOrEqual(2)
    }
    for (let i = 1; i < QUALITY_LEVELS.length; i++) {
      const a = QUALITY_LEVELS[i - 1], b = QUALITY_LEVELS[i]
      expect(b.cloudSteps).toBeLessThan(a.cloudSteps)
      expect(b.shadowMap).toBeLessThanOrEqual(a.shadowMap)
      expect(b.msaa).toBeLessThanOrEqual(a.msaa)
    }
    for (const q of QUALITY_LEVELS) {
      expect(Math.log2(q.shadowMap) % 1).toBe(0)
      expect([0, 2, 4]).toContain(q.msaa)
      expect(q.cloudSteps).toBeGreaterThanOrEqual(16)
    }
  })
  test("MSAA: 4 samples up to 1440p-class buffers, 2 up to ≈ 5 MP, none above; capped by the level", () => {
    expect(msaaSamples(1920 * 1080, 4)).toBe(4)
    expect(msaaSamples(2560 * 1440, 4)).toBe(2)
    expect(msaaSamples(3840 * 2160, 4)).toBe(0)
    expect(msaaSamples(1280 * 720, 2)).toBe(2)
    expect(msaaSamples(1280 * 720, 0)).toBe(0)
  })
})

describe("governor", () => {
  test("holds full quality at 60 fps, at 40 fps and on a 30 Hz display", () => {
    const g = makeGovernor()
    expect(feed(g, 16.7, 5000)).toBe(0)
    expect(feed(g, 24, 5000)).toBe(0)
    expect(feed(g, 33.4, 5000)).toBe(0)
  })
  test("drops one level after a sustained slow stretch, then waits for the new level to settle", () => {
    const g = makeGovernor({ warmupFrames: 10, holdFrames: 30 })
    expect(feed(g, 55, 10)).toBe(0) // warm-up ignored
    expect(feed(g, 55, 29)).toBe(0)
    expect(feed(g, 55, 2)).toBe(1)
    expect(feed(g, 55, 10)).toBe(1) // warm-up of the new level
    expect(feed(g, 16, 500)).toBe(1) // never climbs back
  })
  test("goes down to the lowest level and stops there", () => {
    const g = makeGovernor({ warmupFrames: 5, holdFrames: 20 })
    expect(feed(g, 60, 5000)).toBe(QUALITY_LEVELS.length - 1)
  })
  test("ignores pauses and nonsense (hidden tab, first frame, NaN)", () => {
    const g = makeGovernor({ warmupFrames: 0, holdFrames: 10 })
    for (let i = 0; i < 200; i++) { g.sample(16); g.sample(900); g.sample(NaN); g.sample(-5); g.sample(0) }
    expect(g.level()).toBe(0)
  })
  test("a short burst of slow frames does not trip it", () => {
    const g = makeGovernor({ warmupFrames: 0, holdFrames: 90 })
    for (let k = 0; k < 20; k++) { feed(g, 60, 10); feed(g, 12, 120) }
    expect(g.level()).toBe(0)
  })
})
