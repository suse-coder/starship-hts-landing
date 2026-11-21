/**
 * bench/01 — per-phase run time of the full mission (T07).
 *
 * Runs the mission once and reports total wall time, steps, samples and the wall time per phase.
 *
 *   nice -n 19 bun scripts/bench.ts [--json] [--recording <path>]
 *
 * Measurement:
 *  - total wall time is measured around `runMission` and reported alongside `meta.wallClockMs`,
 *    which the simulator measures itself inside the run;
 *  - per-phase wall time is attributed from the run's own progress callbacks (~150 ms cadence):
 *    every interval goes to the phase reported by the callback that ends it, and the interval after
 *    the last callback goes to the final phase. Resolution is one callback interval, so a phase
 *    shorter than that shows 0 measured ms and is flagged `tickMs: null`;
 *  - the recording has no per-step timing, so the per-phase **step** split is an estimate
 *    proportional to each phase's measured wall-time share. The total step count is exact;
 *  - recorded samples per phase are counted exactly from the `phase` channel;
 *  - the browser-worker time is read from a recording JSON if one exists (`meta.wallClockMs`);
 *    no browser is started here.
 *
 * Run rules (docs/05-benchmarks/00-machine-budget.md): one mission at a time under `nice -n 19`.
 * The script refuses to start while `scripts/mission.ts` or `scripts/profile.ts` is running.
 */
import { getPriority, cpus } from "node:os"
import { DEFAULT_DESIGN } from "@ship/craft"
import { PHASES } from "@ship/gnc"
import { CH, runMission } from "@ship/sim"

interface Tick {
  readonly wallMs: number
  readonly phase: string
}

interface PhaseRow {
  readonly phase: string
  readonly label: string
  readonly startS: number
  readonly endS: number
  readonly spanS: number
  readonly samples: number
  readonly stepsEst: number
  readonly wallMs: number
  readonly wallSharePct: number
  /** callback interval that bounded the phase, or null if the phase was shorter than one interval */
  readonly tickMs: number | null
}

interface WorkerReading {
  readonly source: string
  readonly wallMs: number
  readonly steps: number | null
  readonly samples: number | null
}

const TICK_MS = 150 // progress callback cadence in packages/sim/src/run.ts

const has = (argv: readonly string[], flag: string): boolean => argv.includes(flag)

const argValue = (argv: readonly string[], flag: string): string | null => {
  const i = argv.indexOf(flag)
  return i >= 0 && i + 1 < argv.length ? argv[i + 1] : null
}

/** Refuse to start while another mission/profile run is in flight (run budget, rule 2). */
const assertClear = (): void => {
  const probe = Bun.spawnSync(["pgrep", "-fa", "scripts/(mission|profile)"])
  const text = probe.stdout ? new TextDecoder().decode(probe.stdout) : ""
  const lines = text
    .split("\n")
    .map((l) => l.trim())
    .filter((l) => l.length > 0 && !l.includes("pgrep") && !l.includes("bench.ts"))
  if (lines.length > 0) {
    console.error("another mission/profile run is active; wait for it to finish:")
    console.error(lines.map((l) => `  ${l}`).join("\n"))
    process.exit(1)
  }
}

const cpuLoad = async (): Promise<{ load1: number; load5: number; load15: number }> => {
  const parts = (await Bun.file("/proc/loadavg").text()).split(/\s+/)
  return { load1: Number(parts[0]), load5: Number(parts[1]), load15: Number(parts[2]) }
}

/** One candidate recording, or null if it is absent or carries no `meta.wallClockMs`. */
const readRecording = async (path: string): Promise<WorkerReading | null> => {
  const file = Bun.file(path)
  if (!(await file.exists())) return null
  const raw: unknown = await file.json()
  if (typeof raw !== "object" || raw === null || !("meta" in raw)) return null
  const meta = raw.meta
  if (typeof meta !== "object" || meta === null || !("wallClockMs" in meta)) return null
  if (typeof meta.wallClockMs !== "number") return null
  const steps = "steps" in meta && typeof meta.steps === "number" ? meta.steps : null
  const samples = "n" in raw && typeof raw.n === "number" ? raw.n : null
  return { source: path, wallMs: meta.wallClockMs, steps, samples }
}

/** `meta.wallClockMs` from a recording JSON, if the orchestrator exported one. */
const workerReading = async (argv: readonly string[]): Promise<WorkerReading | null> => {
  const explicit = argValue(argv, "--recording")
  const candidates = explicit ? [explicit] : [".tmp/bench/worker.json", ".tmp/bench/recording.json"]
  const readings = await Promise.all(candidates.map(readRecording))
  return readings.find((r) => r !== null) ?? null
}

const phase1 = (x: number): string => x.toFixed(1)

const main = async (): Promise<void> => {
  const argv = process.argv.slice(2)
  assertClear()

  const loadBefore = await cpuLoad()
  const t0 = performance.now()
  let planningMs = 0
  const ticks: Tick[] = []
  const { recording: rec, final } = runMission(DEFAULT_DESIGN, (p) => {
    const wallMs = performance.now() - t0
    if (p.stage === "simulating") ticks.push({ wallMs, phase: p.phase })
    else planningMs = wallMs
  })
  const externalMs = performance.now() - t0
  const loadAfter = await cpuLoad()

  const d = rec.data
  const n = rec.n
  const missionEndS = n > 0 ? d[CH.t][n - 1] : 0
  const phaseStarts = rec.meta.phaseStarts
  const totalSteps = rec.meta.steps

  // exact sample counts per phase, from the recorded phase channel
  const samplesByPhase = Array.from(d[CH.phase]).reduce((acc, idx) => {
    const id = PHASES[Math.round(idx)]?.id ?? "?"
    return new Map(acc).set(id, (acc.get(id) ?? 0) + 1)
  }, new Map<string, number>())

  // measured wall time per phase from the progress callbacks
  const attributed = ticks.reduce(
    (acc, tick) => ({
      wall: new Map(acc.wall).set(tick.phase, (acc.wall.get(tick.phase) ?? 0) + (tick.wallMs - acc.prev)),
      ticks: new Map(acc.ticks).set(tick.phase, (acc.ticks.get(tick.phase) ?? 0) + 1),
      prev: tick.wallMs,
    }),
    { wall: new Map<string, number>(), ticks: new Map<string, number>(), prev: planningMs },
  )
  const wallByPhase = attributed.wall
  const ticksByPhase = attributed.ticks
  const lastPhase = phaseStarts.length > 0 ? phaseStarts[phaseStarts.length - 1].phase : final.mem.phase
  wallByPhase.set(lastPhase, (wallByPhase.get(lastPhase) ?? 0) + (externalMs - attributed.prev))

  const simMs = Math.max(1e-9, externalMs - planningMs)
  const rows: PhaseRow[] = phaseStarts.map((p, i) => {
    const endS = i + 1 < phaseStarts.length ? phaseStarts[i + 1].t : missionEndS
    const wall = wallByPhase.get(p.phase) ?? 0
    const share = wall / simMs
    return {
      phase: p.phase,
      label: PHASES.find((x) => x.id === p.phase)?.label ?? p.phase,
      startS: p.t,
      endS,
      spanS: Math.max(0, endS - p.t),
      samples: samplesByPhase.get(p.phase) ?? 0,
      stepsEst: Math.round(totalSteps * share),
      wallMs: wall,
      wallSharePct: 100 * share,
      tickMs: (ticksByPhase.get(p.phase) ?? 0) > 0 ? TICK_MS : null,
    }
  })
  // keep the estimate consistent with the exact total after rounding
  const estSum = rows.reduce((a, r) => a + r.stepsEst, 0)
  const balanced = rows.map((r, i) =>
    i === rows.length - 1 ? { ...r, stepsEst: Math.max(0, r.stepsEst + (totalSteps - estSum)) } : r,
  )

  const worker = await workerReading(argv)
  const load1 = loadBefore.load1

  const report = {
    bench: "bench/01 run time",
    machine: {
      cpus: cpus().length,
      nice: getPriority(0),
      load1,
      load5: loadBefore.load5,
      load15: loadBefore.load15,
      loadAfter: { load1: loadAfter.load1, load5: loadAfter.load5, load15: loadAfter.load15 },
      runtime: `bun ${Bun.version}`,
    },
    total: {
      wallMs: Math.round(externalMs),
      metaWallMs: Math.round(rec.meta.wallClockMs),
      planningMs: Math.round(planningMs),
      simMs: Math.round(simMs),
      missionSeconds: missionEndS,
      steps: totalSteps,
      samples: n,
      usPerStep: (1000 * externalMs) / totalSteps,
      outcome: final.outcome,
      reason: final.mem.reason,
    },
    phases: balanced,
    worker,
  }

  if (has(argv, "--json")) {
    console.log(JSON.stringify(report, null, 2))
    return
  }

  const pad = (s: string | number, w: number): string => String(s).padStart(w)
  console.log("bench/01 — full mission run time (T07)")
  console.log(
    `machine: ${report.machine.cpus} vCPU, nice ${report.machine.nice}, load ${phase1(load1)} / ${phase1(report.machine.load5)} / ${phase1(report.machine.load15)} (1/5/15 min), ${report.machine.runtime}`,
  )
  console.log(
    `run: ${phase1(externalMs / 1000)} s wall (sim ${phase1(simMs / 1000)} s + planning ${phase1(planningMs / 1000)} s), ${totalSteps} steps, ${n} samples, ${report.total.usPerStep.toFixed(0)} µs/step`,
  )
  console.log(`mission: ${phase1(missionEndS)} s simulated, outcome ${final.outcome} (${final.mem.reason})`)
  console.log("")
  console.log("phase          span s  samples  steps est  wall ms  share %")
  const tableRows = balanced.map((r) => {
    const flag = r.tickMs === null ? "  (<1 tick, unmeasured)" : ""
    return `${r.phase.padEnd(13)}${pad(phase1(r.spanS), 8)}${pad(r.samples, 9)}${pad(r.stepsEst, 11)}${pad(Math.round(r.wallMs), 9)}${pad(r.wallSharePct.toFixed(1), 9)}${flag}`
  })
  console.log(tableRows.join("\n"))
  console.log(
    `${"(planning)".padEnd(13)}${pad("-", 8)}${pad("-", 9)}${pad("-", 11)}${pad(Math.round(planningMs), 9)}${pad(((100 * planningMs) / externalMs).toFixed(1), 9)}`,
  )
  console.log(
    `${"TOTAL".padEnd(13)}${pad(phase1(missionEndS), 8)}${pad(n, 9)}${pad(totalSteps, 11)}${pad(Math.round(externalMs), 9)}${pad("100.0", 9)}`,
  )
  console.log("")
  console.log("steps per phase are estimated from the measured wall-time share (no per-step timing in the recording);")
  console.log("per-phase wall time is measured at the ~150 ms progress-callback cadence.")
  if (worker) {
    console.log(
      `browser worker: ${phase1(worker.wallMs / 1000)} s (${worker.source}, ${worker.steps ?? "?"} steps, ${worker.samples ?? "?"} samples)`,
    )
  } else {
    console.log("browser worker: no recording exported (.tmp/bench/worker.json or --recording <path>) — not measured here")
  }
  console.log(
    `G4 (full mission ≤ 30 s in a browser worker): this Bun run is ${phase1(externalMs / 1000)} s, ${externalMs / 1000 <= 30 ? "within" : "above"} 30 s; the browser worker itself was not measured here`,
  )
}

await main()
