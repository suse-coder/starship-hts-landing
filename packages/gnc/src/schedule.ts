import { clamp, DEG } from "@ship/math"

/**
 * Nose-first angle-of-attack schedule: stays inside the coil's trim envelope (α ≤ 15°, where
 * restoring MHD authority exists) and low at orbital speed to keep the bare-steel windward cool.
 */
export const alphaNoseFirst = (V: number): number => clamp(10 + ((6800 - V) / 1300) * 4, 10, 14) * DEG
