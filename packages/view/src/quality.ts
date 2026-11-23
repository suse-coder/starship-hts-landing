/**
 * The single quality switch of the viewer. Everything expensive reads its setting from the active
 * level; `makeGovernor` lowers the level when the measured frame time stays above budget (it never
 * raises it again: an oscillating resolution is worse than a stable lower one).
 */

export interface QualityLevel {
  /** multiplier on the device pixel ratio (already capped at 2) of the HDR scene buffers */
  readonly pixelRatioScale: number
  /** coarse ray-march samples of the volumetric cloud deck (clouds.ts `steps`) */
  readonly cloudSteps: number
  /** Sun shadow-map size, texels per side */
  readonly shadowMap: number
  /** largest MSAA sample count of the HDR scene buffer (reduced further for large buffers) */
  readonly msaa: number
  /** PMREM cube size of the sky environment (image-based lighting) */
  readonly envSize: number
}

/**
 * Level 0 = full quality; each step is cheaper. The resolution is NOT part of the trade: the levels
 * used to scale the scene buffer to 0.8 and 0.62 of the display and to drop the anti-aliasing, and
 * on a machine that ran below 40 fps (or on a 30 Hz display, which can never be faster) the
 * governor went straight to the last level — the ship was then drawn at 38 % of the pixels without
 * anti-aliasing and looked blurred and stair-stepped. A slow machine now gets cheaper clouds and
 * shadows; the vehicle and the tower stay at native resolution with their edges anti-aliased.
 */
export const QUALITY_LEVELS: readonly QualityLevel[] = [
  { pixelRatioScale: 1, cloudSteps: 56, shadowMap: 4096, msaa: 4, envSize: 256 },
  { pixelRatioScale: 1, cloudSteps: 40, shadowMap: 2048, msaa: 4, envSize: 256 },
  { pixelRatioScale: 1, cloudSteps: 26, shadowMap: 2048, msaa: 2, envSize: 128 },
]

/**
 * MSAA samples for a scene buffer of `pixels` (width × height in device pixels): the multisampled
 * RGBA16F + depth buffers cost ≈ 12 bytes × samples per pixel (twice: the composer ping-pongs), so
 * large (HiDPI) buffers, which need the anti-aliasing least, get fewer samples.
 */
export const msaaSamples = (pixels: number, max: number): number => {
  const want = pixels <= 2.6e6 ? 4 : pixels <= 5.2e6 ? 2 : 0
  return Math.min(want, max)
}

export interface Governor {
  /** feed one frame time (ms); returns the level index to use */
  readonly sample: (dtMs: number) => number
  readonly level: () => number
}

export interface GovernorOptions {
  /** frame-time budget, ms: the level drops when the smoothed frame time stays above it */
  readonly budgetMs?: number
  /** frames ignored after start and after every level change (shader compilation, reallocation) */
  readonly warmupFrames?: number
  /** consecutive over-budget frames (of the smoothed time) required before dropping a level */
  readonly holdFrames?: number
  readonly levels?: number
}

/**
 * Frame-time governor. The frame time is smoothed with an exponential moving average (time constant
 * ≈ 20 frames); pauses (dt > 250 ms: hidden tab, debugger, initial compile) are ignored. With the
 * defaults a level is dropped after ≈ 3 s below ≈ 24 fps — a display that refreshes at 30 Hz
 * (33 ms per frame) is not a slow machine and keeps full quality.
 */
export const makeGovernor = (opts: GovernorOptions = {}): Governor => {
  const budget = opts.budgetMs ?? 42, warmup = opts.warmupFrames ?? 120, hold = opts.holdFrames ?? 90
  const maxLevel = (opts.levels ?? QUALITY_LEVELS.length) - 1
  let level = 0, ema = 0, skip = warmup, over = 0
  const sample = (dtMs: number): number => {
    if (!(dtMs > 0) || dtMs > 250) return level
    if (skip > 0) { skip--; ema = 0; return level }
    ema = ema === 0 ? dtMs : ema + (dtMs - ema) * 0.05
    over = ema > budget ? over + 1 : 0
    if (over >= hold && level < maxLevel) { level++; over = 0; ema = 0; skip = warmup }
    return level
  }
  return { sample, level: () => level }
}
