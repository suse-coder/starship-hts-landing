/**
 * Playback-rate policy for the timeline (pure; tested in packages/app/test/playback.test.ts).
 *
 * The selected speed applies to the whole flight, except the landing phases (flip, landing, terminal,
 * and the catch or impact that ends them). Those never play faster than the landing rate: the
 * 51 s flip-and-landing sequence would otherwise be over in one second at ×50.
 */
export const LANDING_PHASES: readonly string[] = ["flip", "landing", "terminal", "caught", "lost"]

/** playback rate (simulated seconds per wall second) in the phase `phase` */
export const playbackRate = (phase: string | undefined, selected: number, landing: number): number =>
  phase !== undefined && LANDING_PHASES.includes(phase) ? Math.min(selected, landing) : selected

/** true when the landing rate actually slows the playback below the selected speed */
export const isSlowed = (phase: string | undefined, selected: number, landing: number): boolean =>
  playbackRate(phase, selected, landing) < selected
