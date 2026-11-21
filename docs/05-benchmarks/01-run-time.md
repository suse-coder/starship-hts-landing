# bench/01 — Full mission run time per phase (T07)

One run of the complete mission (`runMission(DEFAULT_DESIGN)`, deorbit → entry → flip → impact) on the
development machine, measured per phase. The machine and run budget are in
[00-machine-budget](00-machine-budget.md); the speed target is [G4](../03-spec/00-goal.md): a full mission
**≤ 30 s in a browser worker** on the user's machine, target 10–15 s.

## 1. Verdict

**G4 is not met in the Bun harness on this machine: 35.4 s.** No browser-worker recording was exported, so
the worker itself is **not measured here** (bench/00 recorded 26–125 s in the worker on this box). The run
is 18 % above the 30 s ceiling and 2.4–3.5× above the 10–15 s target.

The measurement is also a **lower bound** for a successful catch: this mission ends `lost` (ground impact)
at T+3071.5 s, so the terminal phase — which integrates at 0.004 s, 2.5× finer than `flip`/`landing` and
5× finer than the 0.02 s glide — is never flown. Twenty seconds of terminal descent would add ≈ 5 000
steps ≈ 2.5 s at the current cost.

## 2. Method

`scripts/bench.ts` runs the mission once and reports:

| Quantity | How it is obtained | Kind |
|---|---|---|
| Total wall time | `performance.now()` around `runMission`; planning (`makeWorld`: tables + Flight-15 targeting) and simulation are separated | **measured** |
| Steps | `recording.meta.steps`, counted by the run loop | **measured** (total only) |
| Samples | `recording.n` (row count) | **measured** |
| Wall time per phase | assigned from the run's own `onProgress` callbacks (~150 ms cadence): each interval goes to the phase the ending callback reports; the tail after the last callback goes to the final phase | **measured**, ±1 callback interval |
| Samples per phase | counted from the recorded `phase` channel | **measured** |
| Steps per phase | total steps × the phase's measured wall-time share | **estimated** — the recording has no per-step timing |

A single run is used because the run budget allows one mission at a time
([00-machine-budget](00-machine-budget.md) §3); there is no variance estimate, and the load at the start is
reported instead. The harness refuses to start while `scripts/mission.ts` or `scripts/profile.ts` is
running, and it never starts a browser.

Reproduce:

```bash
nice -n 19 bun scripts/bench.ts            # table
nice -n 19 bun scripts/bench.ts --json     # same numbers as JSON (orchestrator)
nice -n 19 bun scripts/bench.ts --recording <path>   # add meta.wallClockMs from a worker recording
```

The browser-worker column is filled in only from an exported recording (`meta.wallClockMs`), read from
`.tmp/bench/worker.json`, `.tmp/bench/recording.json`, or `--recording <path>`.

## 3. Machine

| Item | Value |
|---|---|
| CPU | 12 vCPU, shared devcontainer |
| Load average at start | 6.73 / 9.86 / 8.74 (1 / 5 / 15 min) |
| Niceness | 19 (`nice -n 19 bun scripts/bench.ts`) |
| Memory | 40 GB (00-machine-budget) |
| Runtime | Bun 1.4.4 |
| Browser | none started |

Shared CPU and a load of ~7 mean the absolute number is pessimistic by an unknown factor; a relative
breakdown (which phases cost the time) is the stable part.

## 4. Results

### 4.1 Total

| Quantity | Value |
|---|---|
| Wall time, external timer | **35 400 ms** (35.4 s) |
| — simulation | ≈ 35 300 ms |
| — planning (`makeWorld`) | 78 ms |
| Steps | 72 148 |
| Samples | 7 661 |
| Cost per step | 491 µs |
| Mission time simulated | 3 071.5 s |
| Outcome | `lost` — ground impact |
| Samples per step | 1 per 9.4 steps |

### 4.2 Per phase

Mission span and samples are exact; wall time is measured at the ~150 ms callback resolution; steps are
estimated from the wall share.

| Phase | Mission span (s) | Samples | Steps (est.) | Wall ms (measured) | Share of sim wall |
|---|---:|---:|---:|---:|---:|
| `deorbitAlign` | 476.0 | 238 | 936 | 458 | 1.3 % |
| `deorbitBurn` | 14.4 | 59 | 0 | 0 | 0.0 % |
| `entryAlign` | 1200.7 | 601 | 1 231 | 603 | 1.7 % |
| `entry` | 539.7 | 2 025 | 24 663 | 12 074 | **34.2 %** |
| `loft` | 53.0 | 204 | 2 767 | 1 355 | 3.8 % |
| `pitchUp` | 4.1 | 35 | 307 | 150 | 0.4 % |
| `highAlpha` | 594.0 | 2 700 | 31 359 | 15 352 | **43.5 %** |
| `bellyFlop` | 128.9 | 586 | 3 918 | 1 918 | 5.4 % |
| `flip` | 6.5 | 130 | 306 | 150 | 0.4 % |
| `landing` | 54.1 | 1 082 | 6 457 | 3 161 | 8.9 % |
| `lost` | 0.0 | 1 | 204 | 100 | 0.3 % |
| *(planning)* | – | – | – | 78 | 0.2 % |
| **Total** | **3 071.5** | **7 661** | **72 148** | **35 400** | **100 %** |

Share is of the simulated part (≈ 35.3 s), so `(planning)` is outside the 100 %: the phase shares sum to
99.9 % by rounding and the planning row adds 0.2 %. Reading the table:

- **`deorbitBurn` (0 ms measured, 0 steps est.)** is below one callback interval: its ≈ 290 steps
  (14.4 s at dt = 0.05 s, ≈ 140 ms of wall time) fall inside intervals attributed to `entryAlign`.
  `entryAlign` therefore also carries the deorbit burn.
- **`lost` (0.0 s span, 100 ms)** is not phase work: it is the tail after the last callback — the final
  partial interval plus recording finalisation. The estimated 204 steps are the last steps of the descent,
  not a 0 s `lost` phase.
- **`deorbitAlign` / `entryAlign`** are 1 677 s of mostly vacuum coast at dt = 0.5 s; they cost 1 s of wall
  time in total. Long mission time does not mean long run time.

## 5. Bottleneck

The wall time is step count × per-step cost (72 148 × 491 µs). Step count is set by the timestep rule in
`packages/sim/src/step.ts`:

| Regime | dt (s) | What it costs |
|---|---:|---|
| vacuum align | 0.5 | 2 steps per mission second |
| deorbit burn | 0.05 | 20 per second |
| entry (above / below 100 km) | 0.1 / 0.02 | 10 / 50 per second |
| loft, pitchUp, highAlpha, bellyFlop | 0.02 | 50 per second |
| flip, landing | 0.01 | 100 per second |
| terminal (`caught`/`lost`) | 0.004 | 250 per second |

**The 0.02 s regime is the bottleneck.** `entry` (below 100 km; above it dt is 0.1 s) and `highAlpha`
together take 77.7 % of the run over ≈ 1 134 s of mission time at 0.02 s — roughly 5.6 × 10⁴ of the
7.2 × 10⁴ steps; `bellyFlop` (128.9 s) adds another 5.4 %. Per-step cost is the second lever: each step is
an RK4 pass plus the guidance, thermal, MHD, coil and aero models, 491 µs in Bun.

Consequences for G4:

- **30 s** needs ≤ 61 100 steps at 491 µs (−15 %), or 416 µs per step at the current step count (−15 %).
- **15 s** needs ≈ 30 500 steps or ≈ 208 µs per step (2.4×).
- **10 s** needs ≈ 20 300 steps or ≈ 139 µs per step (3.5×).
- A caught mission adds terminal at 0.004 s: 250 steps per mission second, ≈ 123 ms of wall per second of
  terminal descent. Even 60 s of terminal would add ≈ 7.4 s and widen the gap.

No step is dominated by recording: 7 661 samples at 0.04–2 s intervals cost far less than 72 148 steps.

## 6. Limitations

- **One run, shared CPU.** No variance, no repeated-run minimum. Load 6.7 at start; the absolute number is
  not a clean-machine number.
- **Per-step timing is absent from the recording.** The per-phase step split is an estimate proportional to
  measured wall time; the total is exact. Consequence: `deorbitBurn` and the final `lost` instant are
  invisible (< 1 callback interval) and their time is charged to their neighbours.
- **Callback quantisation.** Per-phase wall time has ±150 ms resolution, which is 0.4 % of the run — fine
  for the two dominant phases, not for `flip` (150 ms) or `pitchUp` (150 ms).
- **Bun, not the browser worker.** G4 is specified for a browser worker on the user's machine. No worker
  recording was exported for this measurement, so the worker column in bench/00 (26–125 s) remains the only
  worker evidence.
- **Comparison with bench/00.** bench/00 §4 quotes ≈ 67 s for the full mission through `sim.test.ts`
  (`SLOW=1`); this table is one `runMission` call from `scripts/bench.ts` at 35.4 s. Both are single
  observations on a shared machine, neither is a clean-machine figure.
- **This mission is `lost`.** The measured run stops at ground impact and does not include the terminal
  descent that a successful catch requires; see §1 for the estimate.

## 7. Files

- `scripts/bench.ts` — the harness (`--json`, `--recording <path>`).
- `packages/sim/src/run.ts` — `wallClockMs`, `steps`, `phaseStarts` (read only).
- `packages/sim/src/recording.ts` — sample channels (read only).
