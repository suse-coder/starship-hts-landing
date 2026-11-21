# Code map

Workspace layout follows foldkit's convention: a root workspace, small source-first packages in
`packages/*` (each `package.json` exports `./src/index.ts`; no build step between packages),
a shared `tsconfig.base.json`, and dev scripts in `scripts/`. Bun's isolated linker means every
package sees only the dependencies it declares — the graph below is enforced, not aspirational.

```
@ship/math   ← immutable V3/Q/M3, interpolation, roots, tridiagonal, counter-based RNG
@ship/earth  ← math              : constants, time/GMST, frames, J2, US-1976, wind+Dryden, IGRF dipole, Sun
@ship/gas    ← math earth        : equilibrium air, real-gas shock, plasma transport, SHOCK_TABLE
@ship/craft  ← math earth        : shape (blunted ogive), components & design sizing, mass, Raptor/RCS
@ship/aero   ← math craft        : Newtonian panels + Jorgensen strips
@ship/heat   ← math earth gas craft : heating correlations, PICA/steel wall response
@ship/mhd    ← math earth gas craft : coil field, MHD shock-layer interaction, magnet circuit/cryo
@ship/tower  ← math              : Mechazilla arms and contact
@ship/gnc    ← math earth gas craft aero mhd : tables, predictor, planner, control, allocation, guidance
@ship/sim    ← all of the above  : World, SimState, step, recording, runMission, worker protocol
@ship/view   ← math earth craft tower sim : three.js scene for one recorded instant
@ship/app    ← sim view three    : browser shell (Vite), timeline, telemetry, charts, design panel
```

| Package | Files | Public entry |
|---|---|---|
| math | `vec3.ts quat.ts num.ts rng.ts` | `index.ts` |
| earth | `constants.ts time.ts earth.ts atmosphere.ts wind.ts magfield.ts sun.ts` | `index.ts` |
| gas | `air.ts plasma.ts shockTable.ts shock-table.json`, `scripts/gen-table.ts` | `index.ts` (+ `SHOCK_TABLE`) |
| craft | `geometry.ts config.ts mass.ts propulsion.ts` | `index.ts` |
| aero | `aero.ts` | `index.ts` |
| heat | `heating.ts thermal.ts` | `index.ts` |
| mhd | `mhd.ts coil.ts` | `index.ts` |
| tower | `tower.ts` | `index.ts` |
| gnc | `tables.ts predictor.ts plan.ts schedule.ts control.ts mhdAlloc.ts guidance.ts` | `index.ts` |
| sim | `world.ts state.ts step.ts recording.ts run.ts protocol.ts` | `index.ts` |

## One step (`sim/step.ts`), as data flow

```
SimState ──► environment (θ, geodetic, atmosphere, wind+turbulence, shock table)
         ──► pad frame, pins, aft-end height
         ──► aero wrench (held)          ──► MHD basis (10 Hz, per-sector F/M/P)
         ──► Nav ──► guide (phase machine) ──► Cmd
         ──► torqueCommand ──► allocate: MHD electrodes → Raptor gimbals → cold-gas couples
         ──► RK4 rigid body (contact per stage) ──► consumables ──► thermal (4 Hz)
         ──► magnet circuit/cryo ──► arms ──► peaks/events ──► (SimState', diagnostics row)
```

Every function on this path is pure (inputs are never mutated; new values are returned).
Numeric kernels use local loops over freshly allocated typed arrays for speed — the
documented exception to the "no imperative loops" lint (plan/01 ADR-3).

## Scripts (`scripts/`)

| Script | Purpose |
|---|---|
| `mission.ts` (`bun run mission`) | full run: events, phases, peaks, condensed trace |
| `trace.ts t0 t1 dt` | attitude/RCS trace window |
| `predcheck.ts t` | 3-DOF predictor vs 6-DOF from a recorded instant |
| `massbudget.ts` | component table, CG, coil design |
| `mhdauth.ts`, `mhdroll.ts`, `mhdsect.ts`, `mhdtab.ts`, `mhdtest.ts` | MHD authority/allocation studies |
| `aerotest.ts`, `heattest.ts`, `airtest.ts` | model spot checks |
