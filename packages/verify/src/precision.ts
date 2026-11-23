import type { Phase } from "@ship/gnc"

/**
 * Phase precision: how each phase of the descent is planned and how finely, as flown.
 *
 * The descent is not solved by one optimiser at one resolution. Each phase gets the method and
 * the effort that its leverage on the catch justifies, inside the wall-clock budget of the whole
 * mission (30 s). The table states what the flight software does (`packages/gnc`) and what it
 * costs, measured with `scripts/phase-profile.ts`; `reach` is how large an error handed over by the
 * previous phase this phase still removes, measured on the 6-DOF model (`scripts/verify-model.ts`,
 * `scripts/gate-check.ts`) — or marked as not measured.
 */
export interface PhasePrecision {
  readonly phase: Phase
  /** what decides the path in this phase */
  readonly method: string
  /** integration/control step of the flight (s) and of its prediction rollouts (s; 0 = none) */
  readonly dt: number
  readonly dtRollout: number
  /** planning cadence (s) and the vehicle-model rollouts spent per planning epoch */
  readonly epoch: number
  readonly rolloutsPerEpoch: number
  /** measured wall time of this phase's planning in the no-wind mission (s) */
  readonly planCost: number
  /** error from upstream that this phase still removes, and how it was established */
  readonly reach: string
}

export const PHASE_PRECISION: readonly PhasePrecision[] = [
  {
    phase: "highAlpha", method: "bank plan on the 3-DOF predictor; its aim is corrected by 6-DOF rollouts to flip completion, each correction verified by a second rollout before it is flown (trust region, ≤ 3 rollouts)",
    dt: 0.05, dtRollout: 0.1, epoch: 50, rolloutsPerEpoch: 3, planCost: 11.7,
    reach: "leaves 0.1–1.5 km for the belly flop; NOT robust — of 12 neighbouring tuning points 5 delivered inside the belly flop's reach (measured)",
  },
  {
    phase: "bellyFlop", method: "two-segment lift-azimuth plan; defect correction: the fast point-mass model proposes, the 6-DOF rollout (exact in still air) decides; nose pointed at the axis in the last 1.8 km",
    dt: 0.05, dtRollout: 0.05, epoch: 30, rolloutsPerEpoch: 4, planCost: 6.0,
    reach: "≈ 0.5 km across the track, almost nothing along it (drag-dominated); polishes to 1–15 m at flip completion when the hand-over is inside that reach (measured)",
  },
  {
    phase: "flip", method: "fixed manoeuvre (flip-and-stop on two engines), ignited when its stop point reaches the axis, inside ± 300 m of the gate altitude",
    dt: 0.01, dtRollout: 0.01, epoch: 0, rolloutsPerEpoch: 0, planCost: 0,
    reach: "± 90 m along the track through the ignition timing; carries 545–589 m along the nose, ≤ 28 m across (measured over 9 flights)",
  },
  {
    phase: "landing", method: "feedback law on the verified pin-chain model (pole placement), hoverslam profile; the plan is the same law integrated forward at 5 Hz",
    dt: 0.01, dtRollout: 0, epoch: 0.2, rolloutsPerEpoch: 0, planCost: 0.2,
    reach: "60 m and 5 m/s in any direction from 1300 m: caught at ≤ 0.7 m, overshoot ≤ 0.22 m (12 of 12 rig cases); ≈ 150 m still caught in a mission, 160 m lost",
  },
  {
    phase: "terminal", method: "the same law at the contact step; arms close when the aft end is below the rail tops",
    dt: 0.004, dtRollout: 0, epoch: 0, rolloutsPerEpoch: 0, planCost: 0,
    reach: "the pin overlap on the closed rails (0.7 m)",
  },
]

/** the precision entry of a phase (undefined for the coast and entry phases, which are not planned by rollout) */
export const precisionOf = (phase: Phase): PhasePrecision | undefined => PHASE_PRECISION.find((p) => p.phase === phase)
