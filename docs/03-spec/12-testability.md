# spec/12 — Modular snapshots and fast iteration

Status: **normative** (2026-11-21). This document defines the testing architecture, not a list of
one-off commands. The rule is general: **any part of the flight or software can be entered from a
serialized snapshot and advanced locally**, so a failure is debugged at its own phase/module without
re-running the whole mission.

## 1. The unit of iteration is a snapshot, not a mission

A **snapshot** is the minimal serializable input needed to continue a computation:

- identity: `simVersion` (the physics hash), design/tuning hash, phase/event id, mission time;
- dynamic state: rigid body, mass/consumables, engines, coil, thermal state, arms, RCS, RNG, guidance
  memory;
- environment: atmosphere/wind model parameters and the current measured wind estimate;
- for rendering: the recording id + playhead + camera/mode, not the simulation state.

Every module and every phase boundary has one. A test loads the nearest snapshot **upstream** of the
failure and runs only the affected code forward. The full mission is the **snapshot generator and
integration validator**, not the inner loop.

## 2. Contracts

| Layer | Input | Pure operation | Output / invariant |
|---|---|---|---|
| Physics step | `World` + `SimState` | `step(world, state)` | next state, forces, temperatures, events |
| Guidance/landing | `Nav` + `Mem` + `Tuning` | `guide`, `planLanding`, `planPdg` | command, plan path, predicted state |
| Actuators | command + engine geometry + mass | allocator | per-engine thrust, gimbal, RCS; residual ΣF−m·u |
| Contact/catch | arm state + pin/hull poses/velocities | `towerContact` | per-pin loads, impulse/moment, catch eligibility |
| Thermal | tile/steel state + heat flux/time | thermal step | energy-conserving temperatures, margins |
| Score | recording (synthetic or real) | `score` | continuous margin + hard constraints |
| Render | recording + time + camera | view update | pixels/screenshot; no physics |

None of these may read ambient time, random or global mutable state; the snapshot carries the seed
and the clock. A module that needs the environment gets it from the snapshot/world, not from a hidden
singleton.

## 3. Snapshot hierarchy

| Kind | Contains | Reuse rule |
|---|---|---|
| **Mission** | design, tuning, version, entry state | regenerate only when design/tuning/physics changes |
| **Phase** | mission + `SimState` at a phase start/event | reuse all upstream phases; rerun only the failing phase |
| **Module** | only that module's input (e.g. landing `Nav`/`Mem`, contact scene, thermal block) | rerun in ms; no entry/landing before it |
| **Render** | recording + playhead + camera | seek/replay; never re-simulate |

A catch failure, for example, is tested from the **terminal/phase snapshot**: load the arm state,
pin/hull state and velocities, run the contact and catch gate forward, and compare. If that passes,
move one snapshot upstream (landing) and repeat. The entry is never re-run for a contact change.
Conversely, a thermal change is tested from the thermal module snapshot, not from the full entry.

## 4. Snapshot lifecycle

1. **Capture** at every phase start and at named events (coil loss, pitch-up, belly-flop start,
   flip ignition, terminal switch, arms close, first pin contact, catch/loss).
2. **Version** by `simVersion` and design/tuning hash. A change upstream invalidates only snapshots
   downstream of the changed model; a local module change reuses its upstream snapshot.
3. **Replay** the module/phase for a bounded number of steps or until the next boundary, recording
   the same columnar channels plus module-specific diagnostics.
4. **Compare** against a golden run or invariants (energy, ΣF−m·u, per-pin load, contact speed,
   thermal energy, score). A changed result is reviewed at that layer, not hidden in a full mission.
5. **Promote**: a passing module run can regenerate the next snapshot; a full mission validates the
   whole chain before release.

Snapshot files are build artifacts (`.tmp`/CI cache), not committed physics. The **schema** and the
snapshot ids are part of the contract; a snapshot must declare its `simVersion`, design/tuning and
phase, or be rejected.

## 5. Test tiers (abstract)

| Tier | Meaning | Pass criterion |
|---|---|---|
| 1. Unit/contract | one pure function with a hand-written or snapshotted input | numeric invariants and limits |
| 2. Module continuation | snapshot → module → N steps | error/RMS/constraint thresholds; no boundary crossed |
| 3. Phase handoff | snapshot A → phase B → snapshot C | state continuity + invariants at C |
| 4. Chain/integration | several snapshots or the full mission | all phase checks + score; release only |

A test may use a **golden snapshot** (a reviewed run) or a **synthetic fixture** (constructed state).
Both are valid; the former catches regressions, the latter isolates a mechanism. Neither replaces the
other's role.

## 6. Fast interaction

The intended loop is:

```
nearest failing phase snapshot → change the module → run only that module/phase → compare invariants
→ if green, move one snapshot downstream; only then run the chain/full mission
```

The tools are interchangeable as long as they honor the snapshot/contract: a Bun test, a Node/Bun
script, a worker, or a browser seeking a recording. The architecture does not depend on a particular
file name or command.

## 7. Rendering and UI

Rendering is tested from a **recording/playhead snapshot**: the recording is immutable, so any visual
change can be checked by seeking to the same phase/time and comparing screenshots. Do not run a new
simulation to test a material, cloud, plume, layout or camera change; use a cached recording and the
same snapshot time.

## 8. Current implementation status

- Pure modules and package fixtures exist for math, earth, craft, aero, heat, mhd, tower, gnc/sim.
- The controller/model bench is a module-level snapshot runner (3-DOF plant + reference).
- Synthetic recordings cover score/contact fixtures.
- A generic persistent snapshot registry (capture/replay JSON or binary) is the next step; the
  contract in this document is the target, and ad-hoc scripts that duplicate the full mission are
  explicitly non-normative.

The full mission remains the only **certification** run (T-mc, release), not the edit loop.
