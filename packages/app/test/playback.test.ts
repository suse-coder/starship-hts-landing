import { describe, expect, test } from "bun:test"
import { isSlowed, playbackRate } from "../src/playback"

describe("playback rate", () => {
  test("the selected speed applies outside the landing phases", () => {
    expect(playbackRate("orbitBurn", 50, 0.5)).toBe(50)
    expect(playbackRate("bellyFlop", 50, 0.5)).toBe(50)
    expect(playbackRate(undefined, 50, 0.5)).toBe(50)
  })

  test("the landing phases play at the landing rate when that is slower", () => {
    expect(playbackRate("flip", 50, 0.5)).toBe(0.5)
    expect(playbackRate("landing", 1000, 0.5)).toBe(0.5)
    expect(playbackRate("terminal", 50, 0.1)).toBe(0.1)
  })

  test("a slower selected speed is never sped up in the landing phases", () => {
    expect(playbackRate("landing", 0.2, 0.5)).toBe(0.2)
  })

  test("isSlowed reports only real slow motion", () => {
    expect(isSlowed("landing", 50, 0.5)).toBe(true)
    expect(isSlowed("landing", 0.2, 0.5)).toBe(false)
    expect(isSlowed("orbitBurn", 50, 0.5)).toBe(false)
  })
})
