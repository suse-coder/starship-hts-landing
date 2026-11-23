import { allocateBudget, PHASE_PRECISION, type Allocation, type PhasePrecision } from "./precision"
import type { Phase } from "@ship/gnc"

/**
 * Phase-aware optimisation on the 6-DOF plant.
 *
 * There is no separate surrogate here: the score of a candidate is obtained by flying the *real*
 * `step` (the same rigid-body plant, aero, engines and guidance) from the phase's state, so the
 * optimiser cannot reward a plan the vehicle cannot fly. What differs per phase is how much of the
 * budget it gets — `precision.ts` allocates it from the measured sensitivity of the final catch to
 * an error introduced there — and how expensive one evaluation is (a belly rollout is 70× a
 * terminal one).
 */
export interface Knob {
  readonly name: string
  readonly lo: number
  readonly hi: number
  /** initial step as a fraction of the range */
  readonly step: number
}

export interface SearchTrace {
  readonly x: readonly number[]
  readonly score: number
  /** the candidate order actually evaluated (for the log) */
  readonly evals: number
}

/**
 * Coordinate pattern search with a shrinking step (deterministic, no gradients, bounded number of
 * evaluations — the contract the compute budget needs). The score is minimised; `score` must be
 * deterministic for a given x.
 */
export const coordinateSearch = (x0: readonly number[], knobs: readonly Knob[], score: (x: readonly number[]) => number, maxEvals: number): SearchTrace => {
  let x = [...x0]
  let best = score(x)
  let evals = 1
  const span = knobs.map((k) => k.hi - k.lo)
  let step = knobs.map((k, i) => k.step * span[i])
  while (evals < maxEvals) {
    let improved = false
    for (let i = 0; i < knobs.length && evals < maxEvals; i++) {
      for (const s of [step[i], -step[i]]) {
        if (evals >= maxEvals) break
        const cand = [...x]
        cand[i] = Math.min(knobs[i].hi, Math.max(knobs[i].lo, cand[i] + s))
        if (cand[i] === x[i]) continue
        const sc = score(cand)
        evals++
        if (sc < best - 1e-6) { x = cand; best = sc; improved = true; break }
      }
    }
    if (!improved) {
      const next = step.map((s) => s * 0.5)
      if (next.every((s, i) => s < 1e-3 * Math.max(span[i], 1e-6))) break
      step = next
    }
  }
  return { x, score: best, evals }
}

export interface PhaseRefine {
  readonly phase: Phase
  readonly allocated: Allocation
  readonly x0: readonly number[]
  readonly x: readonly number[]
  readonly score0: number
  readonly score: number
  readonly evals: number
}

export interface RefineSpec {
  readonly phase: Phase
  readonly knobs: readonly Knob[]
  readonly x0: readonly number[]
  readonly score: (x: readonly number[]) => number
}

/**
 * Refine several phases under one wall-clock budget: `allocateBudget` decides how many evaluations
 * each phase buys, then each phase's knobs are searched with that many. The phases are independent
 * (each is flown from a fixed snapshot), so the order does not matter and the total cost is the sum
 * of the allocated ones.
 */
export const refinePhases = (specs: readonly RefineSpec[], totalSeconds: number, table: readonly PhasePrecision[] = PHASE_PRECISION): readonly PhaseRefine[] => {
  const alloc = allocateBudget(totalSeconds, table)
  const out: PhaseRefine[] = []
  for (const spec of specs) {
    const a = alloc.find((r) => r.phase === spec.phase)
    const evals = Math.max(1, a?.evals ?? 1)
    const s0 = spec.score(spec.x0)
    if (evals <= 1) {
      out.push({ phase: spec.phase, allocated: a ?? { phase: spec.phase, evals: 1, cost: 0, share: 0 }, x0: spec.x0, x: spec.x0, score0: s0, score: s0, evals: 1 })
      continue
    }
    const tr = coordinateSearch(spec.x0, spec.knobs, spec.score, evals)
    out.push({ phase: spec.phase, allocated: a ?? { phase: spec.phase, evals, cost: 0, share: 0 }, x0: spec.x0, x: tr.x, score0: s0, score: tr.score, evals: tr.evals })
  }
  return out
}

/** bisection of the largest value of a scalar error that still catches — the phase's capture radius */
export const captureRadius = (catches: (m: number) => boolean, lo: number, hi: number, steps = 6): number => {
  if (!catches(lo)) return 0
  if (catches(hi)) return hi
  let a = lo, b = hi
  for (let i = 0; i < steps; i++) {
    const m = 0.5 * (a + b)
    if (catches(m)) a = m; else b = m
  }
  return a
}
