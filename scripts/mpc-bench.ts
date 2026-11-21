/**
 * Controller bake-off: fast 3-DOF pin-station plant fed by the real `@ship/craft` Raptor data.
 *
 *   nice -n 19 bun scripts/mpc-bench.ts
 *
 * Prints the model checks, the torque-mode ranking (the honest comparison), a rate-mode
 * sensitivity and the 10 m/s crosswind runs for the best two strategies. Writes
 * `.tmp/mpc-bench.json` and `.tmp/mpc-bench.csv`; the final table lives in
 * `docs/05-benchmarks/03-mpc-strategies.md`.
 */
import { allChecksPass, runModelChecks } from "../packages/gnc/src/bench/checks"
import {
  SCENARIOS,
  aggregate,
  defaultRunConfig,
  runCrosswind,
  runScenario,
  type RunConfig,
  type RunResult,
} from "../packages/gnc/src/bench/harness"
import { STRATEGIES } from "../packages/gnc/src/bench/strategies"
import { BENCH_MASS, BENCH_MASS_PROPS, ENGINE_MODEL } from "../packages/gnc/src/bench/plant"

const pad = (s: string, n: number): string => s.padEnd(n)
const padL = (s: string, n: number): string => s.padStart(n)
const f = (x: number, n = 2): string => x.toFixed(n)

const printChecks = (): boolean => {
  const checks = runModelChecks()
  console.log("\n== model checks (plant vs @ship/craft engine data) ==")
  console.log(`${pad("check", 34)} ${padL("actual", 14)} ${padL("expected", 14)} ${padL("rel.err", 9)}  result`)
  for (const c of checks) {
    const unit = c.unit === "N" ? "N" : c.unit
    console.log(
      `${pad(c.id, 34)} ${padL(`${f(c.actual, 4)} ${unit}`, 14)} ${padL(`${f(c.expected, 4)} ${unit}`, 14)} ${padL(`${(c.relErr * 100).toFixed(4)} %`, 9)}  ${c.pass ? "PASS" : "FAIL"}`,
    )
  }
  return allChecksPass(checks)
}

const printRuns = (title: string, runs: readonly RunResult[]): void => {
  console.log(`\n== ${title} ==`)
  console.log(
    `${pad("strategy", 28)} ${pad("scenario", 8)} ${padL("dp m", 8)} ${padL("dv m/s", 8)} ${padL("t s", 7)} ${padL("RMS m", 8)} ${padL("dir err", 8)} ${padL("fuel kg", 9)} ${padL("replans", 8)} ${padL("ctrl ms", 9)} ${padL("wall ms", 9)}`,
  )
  for (const r of runs) {
    const tCell = r.crossed ? f(r.tContact, 2) : `${f(r.tContact, 2)}*`
    console.log(
      `${pad(r.label, 28)} ${pad(r.scenario, 8)} ${padL(f(r.terminalPos, 1), 8)} ${padL(f(r.terminalVel, 2), 8)} ${padL(tCell, 7)} ${padL(f(r.rmsCross, 1), 8)} ${padL(f(r.maxDirErrDeg, 1), 8)} ${padL(f(r.fuelKg, 0), 9)} ${padL(String(r.replans), 8)} ${padL(f(r.controllerMs, 1), 9)} ${padL(f(r.wallMs, 0), 9)}`,
    )
  }
  console.log("  (* never crossed the rail plane: the closest approach is scored)")
}

const printAggregate = (title: string, runs: readonly RunResult[]): void => {
  console.log(`\n== ${title} ==`)
  console.log(
    `${pad("rank", 5)} ${pad("strategy", 28)} ${padL("score", 8)} ${padL("worst dp", 9)} ${padL("worst dv", 9)} ${padL("worst RMS", 10)} ${padL("fuel kg", 9)} ${padL("ctrl ms", 9)}`,
  )
  const agg = aggregate(runs)
  agg.forEach((a, i) => {
    console.log(
      `${pad(String(i + 1), 5)} ${pad(a.label, 28)} ${padL(f(a.score, 1), 8)} ${padL(f(a.worstTerminalPos, 1), 9)} ${padL(f(a.worstTerminalVel, 2), 9)} ${padL(f(a.worstRms, 1), 10)} ${padL(f(a.fuelKg, 0), 9)} ${padL(f(a.controllerMs, 1), 9)}`,
    )
  })
  return
}

const csvRows = (runs: readonly RunResult[], mode: string, wind: number): string[] => {
  const head = [
    "mode", "wind_mps", "scenario", "strategy", "label", "crossed", "t_contact_s", "ref_duration_s",
    "dp_m", "dv_mps", "rms_m", "dir_err_deg", "tilt_deg", "fuel_kg", "mass_final_kg",
    "peak_throttle", "peak_thrust_accel", "ctrl_ms", "wall_ms", "replans", "infeasible_replans", "score",
    "px", "py", "pz", "vx", "vy", "vz",
  ]
  const out = [head.join(",")]
  for (const r of runs) {
    out.push([
      mode, String(wind), r.scenario, r.strategy, JSON.stringify(r.label), String(r.crossed), r.tContact,
      r.referenceDuration,
      r.terminalPos, r.terminalVel, r.rmsCross, r.maxDirErrDeg, r.maxTiltDeg, r.fuelKg, r.massFinal,
      r.peakThrottle, r.peakThrustAccel, r.controllerMs, r.wallMs, r.replans, r.infeasibleReplans, r.score,
      ...r.terminalP, ...r.terminalV,
    ].join(","))
  }
  return out
}

const main = async (): Promise<void> => {
  const checksOk = printChecks()
  console.log(
    `\nmass ${f(BENCH_MASS, 0)} kg, xcg ${f(BENCH_MASS_PROPS.xcg, 2)} m, Iyy ${(BENCH_MASS_PROPS.I.y / 1e6).toFixed(1)}e6 kg·m², arm ${f(ENGINE_MODEL.arm, 2)} m`,
  )
  console.log(
    `engine: F_SL ${(ENGINE_MODEL.thrustSL / 1e6).toFixed(3)} MN, F_vac ${(ENGINE_MODEL.fVac100 / 1e6).toFixed(4)} MN, Isp_vac ${f(ENGINE_MODEL.ispVac, 1)} s, A_e ${ENGINE_MODEL.exitArea} m², min ${(ENGINE_MODEL.minThrottle * 100).toFixed(0)} %, gimbal ±${((ENGINE_MODEL.gimbalMax * 180) / Math.PI).toFixed(0)}° @ ${((ENGINE_MODEL.gimbalRate * 180) / Math.PI).toFixed(0)}°/s`,
  )

  const base = defaultRunConfig()
  const torque: RunConfig = { ...base, plant: { ...base.plant, mode: "torque" } }
  const rate: RunConfig = { ...base, plant: { ...base.plant, mode: "rate" } }

  const torqueRuns: RunResult[] = []
  for (const sc of SCENARIOS) {
    for (const s of STRATEGIES) torqueRuns.push(runScenario(sc, s.id, torque))
  }
  const rateRuns: RunResult[] = []
  for (const sc of SCENARIOS) {
    for (const s of STRATEGIES) rateRuns.push(runScenario(sc, s.id, rate))
  }

  printRuns("torque-mode runs (the honest comparison)", torqueRuns)
  printAggregate("torque-mode ranking: terminal + 2·velocity + cross-track RMS", torqueRuns)
  printAggregate("rate-mode sensitivity", rateRuns)

  const top = aggregate(torqueRuns).slice(0, 2)
  const windRuns: RunResult[] = []
  for (const a of top) {
    for (const sc of SCENARIOS) {
      windRuns.push(runCrosswind(sc, a.strategy, torque, 10))
    }
  }
  printRuns("10 m/s crosswind (equivalent 1.0 m/s² lateral disturbance) for the best two", windRuns)

  const fastest = [...STRATEGIES]
    .map((s) => {
      const rs = torqueRuns.filter((r) => r.strategy === s.id)
      return { label: s.label, ms: rs.reduce((a, r) => a + r.controllerMs, 0) / Math.max(rs.length, 1) }
    })
    .sort((a, b) => a.ms - b.ms)
  console.log("\n== controller time per run (torque mode, mean of two scenarios) ==")
  for (const t of fastest) console.log(`${pad(t.label, 28)} ${f(t.ms, 2).padStart(9)} ms`)
  const pdg = torqueRuns.filter((r) => r.strategy === "pdg")
  const two = torqueRuns.filter((r) => r.strategy === "two-phase")
  if (pdg.length > 0) {
    const ms = pdg.reduce((a, r) => a + r.controllerMs, 0) / Math.max(pdg.reduce((a, r) => a + r.replans, 0), 1)
    console.log(`pdg: ${f(pdg.reduce((a, r) => a + r.replans, 0), 0)} replans, ${f(ms, 2)} ms/replan`)
  }
  if (two.length > 0) {
    const ms = two.reduce((a, r) => a + r.controllerMs, 0) / Math.max(two.reduce((a, r) => a + r.replans, 0), 1)
    console.log(`two-phase: ${f(two.reduce((a, r) => a + r.replans, 0), 0)} replans, ${f(ms, 2)} ms/replan`)
  }

  const payload = {
    meta: {
      note: "3-DOF controller/model comparison, not a catch claim",
      gate: [0, -12, 0],
      gateV: [0, 0.4, -0.7],
      mass: BENCH_MASS,
      tiltDeg: torque.tiltDeg,
      plant: {
        nEng: torque.plant.nEng,
        mode: torque.plant.mode,
        dt: torque.plant.dt,
        gravity: torque.plant.gravity,
        tauThrottle: torque.plant.tauThrottle,
        tauDir: torque.plant.tauDir,
        rateCap: torque.plant.rateCap,
        gimbalKp: torque.plant.gimbalKp,
        gimbalKd: torque.plant.gimbalKd,
      },
      scenarios: SCENARIOS.map((s) => ({ id: s.id, p0: [s.p0.x, s.p0.y, s.p0.z], v0: [s.v0.x, s.v0.y, s.v0.z] })),
      engine: {
        thrustSL: ENGINE_MODEL.thrustSL,
        isp0: ENGINE_MODEL.engine.isp0,
        ispVac: ENGINE_MODEL.ispVac,
        exitArea: ENGINE_MODEL.exitArea,
        minThrottle: ENGINE_MODEL.minThrottle,
        gimbalMax: ENGINE_MODEL.gimbalMax,
        gimbalRate: ENGINE_MODEL.gimbalRate,
        arm: ENGINE_MODEL.arm,
        Iyy: ENGINE_MODEL.Iyy,
      },
      windEquivalent: "k·W with k = 0.1 s^-1; 10 m/s -> 1.0 m/s²",
    },
    checks: runModelChecks(),
    runs: {
      torque: torqueRuns,
      rate: rateRuns,
      crosswind10: windRuns,
    },
    ranking: {
      torque: aggregate(torqueRuns),
      rate: aggregate(rateRuns),
    },
  }
  const json = JSON.stringify(payload, null, 2)
  await Bun.write(".tmp/mpc-bench.json", json)
  const csv = [
    ...csvRows(torqueRuns, "torque", 0),
    ...csvRows(rateRuns, "rate", 0),
    ...csvRows(windRuns, "torque", 10),
  ].join("\n")
  await Bun.write(".tmp/mpc-bench.csv", `${csv}\n`)
  console.log("\nwrote .tmp/mpc-bench.json and .tmp/mpc-bench.csv")
  console.log(`checks: ${checksOk ? "all pass" : "FAILED"}`)
  if (!checksOk) process.exitCode = 1
}

await main()
