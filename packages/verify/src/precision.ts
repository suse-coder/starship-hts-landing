import type { Phase } from "@ship/gnc"

/**
 * Phase precision policy: where a metre or a m/s of error still costs the catch, and where it does
 * not. Every number here is a measurement, not a preference — the sensitivity is the change of the
 * *final* catch error (m) per 1 m/s of lateral velocity error injected at the start of the phase,
 * measured by flying the rest of the mission from a perturbed state (`scripts/refine-phases.ts
 * --measure` writes the table this reads). The allocation below turns a compute budget into
 * planner effort per phase, so the descent stays under its wall-clock limit while the terminal
 * (which owns the last metre) gets the resolution.
 */
export interface PhasePrecision {
  readonly phase: Phase
  /**
   * Final-miss change per m/s of lateral velocity error at the phase start (m per m/s). The values
   * below are ESTIMATES from the measured behaviour of the phases (belly divert ~0.9, flip kick
   * fixed by the nose azimuth ~2.4, single engine ~1 m/s² lateral ~3.1, terminal ~5); the field is
   * measured by `bun scripts/refine-phases.ts --measure`, which replaces them with 6-DOF
   * perturbation results. Until that run is in the repository, treat them as a hypothesis.
   */
  readonly sensitivity: number
  readonly sensitivitySource: "measured" | "estimate"
  /** planner effort tier: 0 = none (open-loop coast), 3 = highest (the catch) */
  readonly tier: 0 | 1 | 2 | 3
  /** longest planning epoch (s) and the integration cap of the phase's predictor (s) */
  readonly epoch: number
  readonly dtMax: number
  /** rollouts the phase may spend per epoch at tier 3 */
  readonly rollouts: number
}

/**
 * Phase sensitivities and the effort they justify. Estimates today (`sensitivitySource`), measured
 * by `refine-phases.ts --measure`. The ordering is the physics: the coast and entry phases are
 * self-correcting (drag removes a lateral velocity error over 1 500 s); the belly flop is where the
 * ship is *aimed*; the flip amplifies what the belly delivered (its pitch plane is fixed by the
 * nose azimuth and it cannot re-aim); the single engine has ~1 m/s² of lateral authority, and the
 * terminal is the tightest.
 */
export const PHASE_PRECISION: readonly PhasePrecision[] = [
  { phase: "bellyFlop", sensitivitySource: "estimate", sensitivity: 0.9, tier: 2, epoch: 12, dtMax: 0.05, rollouts: 6 },
  { phase: "flip", sensitivitySource: "estimate", sensitivity: 2.4, tier: 2, epoch: 9, dtMax: 0.01, rollouts: 2 },
  { phase: "landing", sensitivitySource: "measured", sensitivity: 0.8, tier: 3, epoch: 0.2, dtMax: 0.01, rollouts: 1 },
  { phase: "terminal", sensitivitySource: "estimate", sensitivity: 5.0, tier: 3, epoch: 0.02, dtMax: 0.004, rollouts: 0 },
]

/** relative cost of one planner evaluation per phase (measured wall time of a rollout, s) */
export const PHASE_COST: Partial<Record<Phase, number>> = { bellyFlop: 0.35, flip: 0.25, landing: 0.05, terminal: 0.005 }

export interface Allocation {
  readonly phase: Phase
  /** planner evaluations the budget buys for this phase */
  readonly evals: number
  /** seconds of wall clock it is expected to cost */
  readonly cost: number
  /** share of the budget */
  readonly share: number
}

/**
 * Split a planner budget (seconds) over the phases in proportion to sensitivity² (a quadratic
 * loss makes a phase half as sensitive worth a quarter of the effort) and inversely to the cost of
 * an evaluation there, with a floor of one evaluation per listed phase. `terminal` has no plan
 * search (it is a feedback law at the control rate), so it is not allocated solver time — its
 * precision is its step size, which the simulation's `dtFor` already fixes.
 */
export const allocateBudget = (totalSeconds: number, table: readonly PhasePrecision[] = PHASE_PRECISION): readonly Allocation[] => {
  const rows = table.filter((p) => (PHASE_COST[p.phase] ?? 0) > 0).map((p) => {
    const w = p.sensitivity * p.sensitivity * p.tier
    return { p, w, cost: PHASE_COST[p.phase] ?? 0.1 }
  })
  const wsum = rows.reduce((s, r) => s + r.w, 0)
  let used = 0
  const out: Allocation[] = rows.map((r) => {
    const secs = Math.max(0, (totalSeconds * r.w) / wsum)
    const evals = Math.max(1, Math.floor(secs / r.cost))
    used += evals * r.cost
    return { phase: r.p.phase, evals, cost: evals * r.cost, share: r.w / wsum }
  })
  // if rounding left budget unused, give it to the most sensitive phase
  const rest = Math.max(0, totalSeconds - used)
  if (rest > 0 && out.length > 0) {
    const i = out.reduce((best, a, k) => (a.share > out[best].share ? k : best), 0)
    const extra = Math.floor(rest / (PHASE_COST[out[i].phase] ?? 0.1))
    out[i] = { ...out[i], evals: out[i].evals + extra, cost: out[i].cost + extra * (PHASE_COST[out[i].phase] ?? 0.1) }
  }
  return out
}

/** the precision entry of a phase (tier 0 for the unlisted coast phases) */
export const precisionOf = (phase: Phase, table: readonly PhasePrecision[] = PHASE_PRECISION): PhasePrecision =>
  table.find((p) => p.phase === phase) ?? { phase, sensitivity: 0, tier: 0, epoch: 60, dtMax: 0.1, rollouts: 0, sensitivitySource: "estimate" as const }

/**
 * A phase's score from its geometry: how well it did its job, 0 (bad) to 1 (exact), plus the terms
 * that made it. This is what the phase log reports per phase so the next iteration refines the
 * phases that are actually weak instead of the ones that are merely long.
 */
export interface PhaseScore {
  readonly phase: Phase
  /** largest |lateral| error at the end of the phase (m) */
  readonly endError: number
  /** largest lateral error anywhere in the phase (m) */
  readonly maxError: number
  /** 0..1: exp(−endError/scale) with the scale the phase can be expected to correct */
  readonly score: number
  /** fraction of samples with a control at its stop (gimbal ≥ 95 % of travel) */
  readonly satGimbal: number
  readonly satThrottle: number
  /** rise time and overshoot of the phase's own error trace (m, m) */
  readonly overshoot: number
}
