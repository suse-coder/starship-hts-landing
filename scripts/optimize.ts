/**
 * Parameter search for the Ship 42 catch (spec/10).
 *
 *   DOE screen (Halton/Sobol-style low-discrepancy, 3-DOF) → CMA-ES on the influential knobs
 *   (3-DOF) → top candidates through the **full 6-DOF** `step` loop in bounded, niced workers →
 *   Pareto table.
 *
 * The 3-DOF evaluator ranks; only the 6-DOF certifies. The full worker applies `MissionTuning` to
 * `world.guide` (and rebuilds the mission plan if the deorbit Δv changes), then runs the same loop
 * as `runMission` and scores the recording with `score(recording)`.
 *
 * Usage:
 *   nice -n 19 bun scripts/optimize.ts [--screen 64] [--gen 24] [--full 10] [--workers 4] [--quick]
 *   nice -n 19 bun scripts/optimize.ts --worker=<base64 JSON tuning>   (internal)
 */
import { writeFileSync } from "node:fs"
import { cpus } from "node:os"
import { type Q, dot, normalize, qrot, scale, sub, v3 } from "@ship/math"
import { rotZ } from "@ship/earth"
import {
  DEFAULT_DESIGN, initialConsumables, massProps,
} from "@ship/craft"
import {
  CH, initState, makeWorld, predContext, recorder, score, step, type Recording, type World,
} from "@ship/sim"
import {
  DEFAULT_PLAN, MISSION_TUNING_DEFAULT, MissionTuning, PHASES, REFINE_KNOBS, SCREEN_KNOBS,
  evaluateFast, knobFromUnit, knobToUnit, planMission, type FastHandover, type TuningKnob,
  type TuningValues,
} from "@ship/gnc"

// --------------------------------------------------------------------------------------------------
// arguments
// --------------------------------------------------------------------------------------------------
const argv = process.argv.slice(2)
const argNum = (name: string, dflt: number): number => {
  const hit = argv.find((a) => a.startsWith(`--${name}=`))
  if (hit) {
    const n = Number(hit.slice(name.length + 3))
    return Number.isFinite(n) ? n : dflt
  }
  const i = argv.indexOf(`--${name}`)
  if (i >= 0 && i + 1 < argv.length) {
    const n = Number(argv[i + 1])
    return Number.isFinite(n) ? n : dflt
  }
  return dflt
}
const argFlag = (name: string): boolean => argv.includes(`--${name}`)
const workerArg = argv.find((a) => a.startsWith("--worker="))
const outFile = new URL("../.tmp/agentO/optimize-report.json", import.meta.url)

// --------------------------------------------------------------------------------------------------
// full 6-DOF world/run (the worker and the baseline)
// --------------------------------------------------------------------------------------------------
const recDt = (phase: string): number => {
  switch (phase) {
    case "deorbitAlign": case "entryAlign": return 2.0
    case "deorbitBurn": return 0.25
    case "entry": case "loft": return 0.25
    case "pitchUp": return 0.1
    case "highAlpha": case "bellyFlop": return 0.2
    case "flip": case "landing": return 0.05
    default: return 0.04
  }
}

/** Build the world with the tuning applied; rebuild the targeted plan if the deorbit Δv moved. */
const tunedWorld = (tuning: MissionTuning): World => {
  const base = makeWorld(DEFAULT_DESIGN)
  let plan = base.plan
  if (Math.abs(tuning.dvBurn - DEFAULT_PLAN.dvBurn) > 1e-6) {
    const c0 = initialConsumables(base.vehicle)
    plan = planMission(
      (theta0, pad) => predContext(base.aeroTab, base.mhdDrag, base.aero.sref, theta0, pad, base.guide.windForecast),
      { ...DEFAULT_PLAN, dvBurn: tuning.dvBurn, mass: massProps(base.vehicle, c0).m },
    )
  }
  return {
    ...base, plan,
    guide: { ...base.guide, tBurn: plan.tBurn, dvBurn: plan.dvBurn, tuning },
  }
}

/** The same loop as `runMission`, with the tuned world and no progress callback. */
const runWorld = (world: World): Recording => {
  let s = initState(world)
  const rec = recorder(world.grid.cellX.length)
  const phaseStarts: { phase: string; t: number }[] = [{ phase: s.mem.phase, t: 0 }]
  let lastRec = -1e9
  while (!s.done) {
    const out = step(world, s)
    if (out.s.mem.phase !== s.mem.phase) phaseStarts.push({ phase: out.s.mem.phase, t: out.s.t })
    if (s.t - lastRec >= recDt(s.mem.phase) || out.s.done || out.s.mem.phase !== s.mem.phase) {
      rec.push(out.row, out.temps)
      lastRec = s.t
    }
    s = out.s
  }
  const p = s.peak
  const summary: Record<string, number | string> = {
    outcome: s.outcome, reason: s.mem.reason, duration: s.t,
    peakG: p.g, peakQbar: p.qbar, peakQStag: p.qStag, peakPicaSurf: p.picaSurf, peakBondline: p.bondline,
    peakTileSurf: p.tileSurf, peakTileBack: p.tileBack, peakSteel: p.steel, peakMhdForce: p.mhdForce, peakCoilT: p.coilT,
    n2Left: s.cons.n2, seedLeft: s.cons.seed, headerLeft: s.cons.loxHeader + s.cons.ch4Header,
    massFinal: massProps(world.vehicle, s.cons).m, coilIEnd: s.coil.I,
    coilEEnd: 0.5 * world.vehicle.coil.L * s.coil.I ** 2,
  }
  return rec.finish(s.events, {
    epochJd: world.plan.epochJd, theta0: world.plan.theta0,
    padLat: world.plan.pad.lat, padLon: world.plan.pad.lon, padAlt: world.plan.pad.alt,
    outcome: s.outcome, summary, phaseStarts, predPath: world.plan.pred.path,
    vehicle: { design: world.design, components: world.vehicle.components, coil: world.vehicle.coil, pica: world.vehicle.pica, pinX: world.vehicle.pinX, pinOut: world.vehicle.pinOut, phases: PHASES },
    wallClockMs: 0, steps: 0,
  })
}

// --------------------------------------------------------------------------------------------------
// baseline hand-over for the fast evaluator
// --------------------------------------------------------------------------------------------------
const indexAt = (rec: Recording, t: number): number => {
  const ts = rec.data[CH.t]
  if (t <= ts[0]) return 0
  if (t >= ts[rec.n - 1]) return rec.n - 1
  let lo = 0, hi = rec.n - 1
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1
    if (ts[m] <= t) lo = m
    else hi = m
  }
  return lo
}

/** The belly-flop hand-over in pad ENU, including the lift azimuth actually flown (from q + wind). */
const handoverOf = (rec: Recording): FastHandover => {
  const tBelly = rec.meta.phaseStarts.find((p) => p.phase === "bellyFlop")?.t
  if (tBelly === undefined) throw new Error("no bellyFlop phase in the baseline recording")
  const i = indexAt(rec, tBelly)
  const d = rec.data
  const lat = d[CH.lat][i], lon = d[CH.lon][i], theta = d[CH.theta][i]
  const sl = Math.sin(lat), cl = Math.cos(lat), so = Math.sin(lon), co = Math.cos(lon)
  const east = rotZ(v3(-so, co, 0), theta)
  const north = rotZ(v3(-sl * co, -sl * so, cl), theta)
  const q: Q = { w: d[CH.qw][i], x: d[CH.qx][i], y: d[CH.qy][i], z: d[CH.qz][i] }
  const bx = qrot(q, v3(1, 0, 0))
  const velRel = v3(d[CH.ve][i], d[CH.vn][i], d[CH.vu][i])
  const vAir = sub(velRel, v3(d[CH.windE][i], d[CH.windN][i], 0))
  const vh = normalize(vAir)
  const lb = sub(bx, scale(vh, dot(bx, vh)))
  return {
    t: d[CH.t][i],
    pinRel: v3(d[CH.pinX][i], d[CH.pinY][i], d[CH.pinZ][i]),
    velRel,
    hBase: d[CH.hBase][i], mass: d[CH.mass][i], V: d[CH.V][i],
    psi0: Math.atan2(dot(lb, east), dot(lb, north)),
  }
}

// --------------------------------------------------------------------------------------------------
// low-discrepancy DOE + a compact CMA-ES
// --------------------------------------------------------------------------------------------------
const PRIMES = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71]
const halton = (index: number, base: number): number => {
  let f = 1, r = 0, i = index
  while (i > 0) { f /= base; r += f * (i % base); i = Math.floor(i / base) }
  return r
}
/** scrambled-ish Halton point in [0,1)^d (the spec's Sobol/DOE screen) */
const doePoint = (index: number, dims: number): number[] => {
  const shift = 0 // deterministic: reproducible runs
  return Array.from({ length: dims }, (_, d) => (halton(index + 1 + shift, PRIMES[d % PRIMES.length]) + shift) % 1)
}

const gauss = (): number => {
  const u = Math.max(Math.random(), 1e-12)
  const v = Math.random()
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v)
}

interface Candidate { readonly tuning: MissionTuning; readonly x: number[]; readonly j: number }

const weightedMean = (xs: readonly number[][], w: readonly number[]): number[] => {
  const d = xs[0].length
  const m = Array.from({ length: d }, () => 0)
  xs.forEach((x, k) => { for (let i = 0; i < d; i++) m[i] += w[k] * x[i] })
  return m
}

/** Jacobi eigen-decomposition of a symmetric matrix (n ≤ 10), eigen-values descending. */
const eig = (A: number[][]): { values: number[]; vectors: number[][] } => {
  const n = A.length
  const a = A.map((r) => r.slice())
  const v: number[][] = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)))
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j]
    if (off < 1e-20) break
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-18) continue
      const theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1))
      const c = 1 / Math.sqrt(t * t + 1), s = t * c
      for (let k = 0; k < n; k++) {
        const akp = a[k][p], akq = a[k][q]
        a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq
      }
      for (let k = 0; k < n; k++) {
        const apk = a[p][k], aqk = a[q][k]
        a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk
      }
      for (let k = 0; k < n; k++) {
        const vkp = v[k][p], vkq = v[k][q]
        v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq
      }
    }
  }
  const order = Array.from({ length: n }, (_, i) => i).sort((i, j) => a[j][j] - a[i][i])
  return { values: order.map((i) => Math.max(a[i][i], 1e-12)), vectors: order.map((i) => v.map((row) => row[i])) }
}

/**
 * Compact (μ/μ_w, λ)-CMA-ES on [0,1]^d. Bounds are handled by clipping (a penalty-free box, since
 * every objective is evaluated at the clipped point and the mean is projected after each update).
 */
const cmaEs = (dims: number, seed: number[], evaluate: (x: number[]) => number, generations: number): Candidate[] => {
  const n = dims
  const lambda = 4 + Math.floor(3 * Math.log(n))
  const mu = Math.floor(lambda / 2)
  const wRaw = Array.from({ length: mu }, (_, i) => Math.log(mu + 0.5) - Math.log(i + 1))
  const wSum = wRaw.reduce((a, b) => a + b, 0)
  const w = wRaw.map((x) => x / wSum)
  const mueff = 1 / w.reduce((a, b) => a + b * b, 0)
  const cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n)
  const cs = (mueff + 2) / (n + mueff + 5)
  const c1 = 2 / ((n + 1.3) ** 2 + mueff)
  const cmu = Math.min(1 - c1, (2 * (mueff - 2 + 1 / mueff)) / ((n + 2) ** 2 + mueff))
  const damps = 1 + 2 * Math.max(0, Math.sqrt((mueff - 1) / (n + 1)) - 1) + cs
  const chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
  let m = seed.slice()
  let sigma = 0.22
  let C: number[][] = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)))
  let pc = Array.from({ length: n }, () => 0)
  let ps = Array.from({ length: n }, () => 0)
  const all: Candidate[] = []
  for (let gen = 0; gen < generations; gen++) {
    const { values, vectors } = eig(C)
    const B = vectors // rows are eigenvectors
    const D = values.map((x) => Math.sqrt(x))
    const pop: { x: number[]; y: number[]; j: number }[] = []
    for (let k = 0; k < lambda; k++) {
      const z = Array.from({ length: n }, () => gauss())
      const y = Array.from({ length: n }, (_, i) => {
        let s = 0
        for (let j = 0; j < n; j++) s += B[j][i] * D[j] * z[j]
        return s
      })
      const x = m.map((mi, i) => Math.min(1, Math.max(0, mi + sigma * y[i])))
      pop.push({ x, y, j: evaluate(x) })
    }
    pop.sort((a, b) => a.j - b.j)
    const mOld = m.slice()
    m = weightedMean(pop.slice(0, mu).map((p) => p.x), w)
    const invSqrt = (v: number[]): number[] => {
      // C^{-1/2} v = B D^{-1} Bᵀ v
      const bt = Array.from({ length: n }, (_, i) => B.reduce((s, row, j) => s + row[i] * v[j], 0))
      const d = bt.map((x, i) => x / D[i])
      return Array.from({ length: n }, (_, j) => B[j].reduce((s, row, i) => s + row * d[i], 0))
    }
    const invY = invSqrt(m.map((x, i) => x - mOld[i]))
    ps = ps.map((p, i) => (1 - cs) * p + Math.sqrt(cs * (2 - cs) * mueff) * invY[i])
    const psNorm = Math.hypot(...ps)
    const hsig: number = psNorm / Math.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) < (1.4 + 2 / (n + 1)) * chiN ? 1 : 0
    pc = pc.map((p, i) => (1 - cc) * p + hsig * Math.sqrt(cc * (2 - cc) * mueff) * (m[i] - mOld[i]) / sigma)
    const Cnew: number[][] = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (1 - c1 - cmu) * C[i][j]))
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      Cnew[i][j] += c1 * (pc[i] * pc[j] + (1 - hsig) * cc * (2 - cc) * C[i][j])
      for (let k = 0; k < mu; k++) Cnew[i][j] += cmu * w[k] * pop[k].y[i] * pop[k].y[j]
    }
    C = Cnew
    sigma *= Math.exp((cs / damps) * (psNorm / chiN - 1))
    sigma = Math.min(0.5, Math.max(0.01, sigma))
    for (const p of pop) all.push({ tuning: tuningFromX(p.x), x: p.x, j: p.j })
  }
  return all
}

// --------------------------------------------------------------------------------------------------
// tuning vector ↔ knobs
// --------------------------------------------------------------------------------------------------
let activeKnobs: readonly TuningKnob[] = SCREEN_KNOBS

const tuningFromX = (x: number[]): MissionTuning => {
  const patch: Record<string, number> = {}
  activeKnobs.forEach((k, i) => { patch[k.key] = knobFromUnit(k, x[i]) })
  return new MissionTuning(patch as Partial<TuningValues>)
}

// --------------------------------------------------------------------------------------------------
// worker: one full 6-DOF mission with a tuning
// --------------------------------------------------------------------------------------------------
const isWorker = workerArg !== undefined
if (isWorker) {
  const b64 = workerArg.slice("--worker=".length)
  const tuning = MissionTuning.decode(JSON.parse(Buffer.from(b64, "base64").toString("utf8")))
  const t0 = performance.now()
  const world = tunedWorld(tuning)
  const rec = runWorld(world)
  const ms = performance.now() - t0
  const s = score(rec)
  const out = { tuning: tuning.toValues(), wallMs: ms, outcome: rec.meta.outcome, score: s, events: rec.events.map((e) => `${e.t.toFixed(1)} ${e.msg}`) }
  console.log(`RESULT:${JSON.stringify(out)}`)
  process.exit(0)
}

// --------------------------------------------------------------------------------------------------
// parent
// --------------------------------------------------------------------------------------------------
const log = (s: string): void => { process.stdout.write(`${s}\n`) }
const sleep = (ms: number): Promise<void> => new Promise((r) => setTimeout(r, ms))

const main = async (): Promise<void> => {
  const screenN = argNum("screen", argFlag("quick") ? 24 : 64)
  const generations = argNum("gen", argFlag("quick") ? 8 : 24)
  const fullN = argNum("full", argFlag("quick") ? 4 : 10)
  const workers = Math.max(1, Math.min(argNum("workers", 4), cpus().length - 1))
  const lateralEff = argNum("latEff", 0.45)

  log("=== Ship 42 catch search (spec/10) ===")
  log(`vector: ${SCREEN_KNOBS.map((k) => `${k.key}[${k.min},${k.max}]`).join(" ")}`)
  log(`objective: 3-DOF continuous catch margin (lateral + 2·speed excess + roll/along/velocity terms) + 0.02·belly plan miss + gate/V guards`)
  log(`constraints (hard): catch armsClose&&pinLoad, lat ≤1 m, speed ≤1 m/s, 2 engines, no cold gas after flip, coil discharged, flip V ≥110 m/s, TPS limits`)
  log(`full mission gate: ${fullN} candidates on ${workers} workers (nice -n 19, one per core)`)

  // baseline full mission for the hand-over + a reference score
  const baseT0 = performance.now()
  const baselineTuning = MISSION_TUNING_DEFAULT
  const baseRec = runWorld(tunedWorld(baselineTuning))
  const baseScore = score(baseRec)
  const hand = handoverOf(baseRec)
  const baseWorld = tunedWorld(baselineTuning)
  const baseMs = performance.now() - baseT0
  log(`\nbaseline 6-DOF: ${(baseMs / 1000).toFixed(1)} s, outcome ${baseRec.meta.outcome}, margin ${baseScore.margin.toFixed(1)} m, catch ${baseScore.catch}, flip V ${baseScore.terms.flipSpeed.toFixed(1)} h ${baseScore.terms.flipAlt.toFixed(0)}`)
  log(`hand-over: pin ${hand.pinRel.x.toFixed(0)},${hand.pinRel.y.toFixed(0)} h ${hand.hBase.toFixed(0)} v ${hand.velRel.x.toFixed(0)},${hand.velRel.y.toFixed(0)},${hand.velRel.z.toFixed(0)} psi0 ${(hand.psi0 * 180 / Math.PI).toFixed(0)}°`)
  log(`baseline constraints: ${baseScore.constraints.map((c) => `${c.id}:${c.ok ? "ok" : "FAIL"}`).join(" ")}`)

  // fast evaluation cache
  const cache = new Map<string, { tun: MissionTuning; j: number; p: ReturnType<typeof evaluateFast> }>()
  const fastTimings: number[] = []
  const evalFast = (T: MissionTuning): { tun: MissionTuning; j: number; p: ReturnType<typeof evaluateFast> } => {
    const key = JSON.stringify(T.toValues())
    const hit = cache.get(key)
    if (hit) return hit
    const p = evaluateFast(hand, baseWorld.guide.pred, baseWorld.vehicle, T, lateralEff)
    fastTimings.push(p.timingMs.total)
    const j =
      p.margin +
      0.02 * Math.abs(p.bellyPlan.miss) +
      0.002 * Math.max(0, p.flip.hBase - 1600) +
      0.5 * Math.max(0, 110 - p.flip.V) +
      (p.contact.climbed ? 50 : 0)
    const out = { tun: T, j, p }
    cache.set(key, out)
    return out
  }

  // ---------------- DOE screen ----------------
  activeKnobs = SCREEN_KNOBS
  const dims = activeKnobs.length
  const screenT0 = performance.now()
  const screen: { x: number[]; r: ReturnType<typeof evalFast> }[] = []
  for (let i = 0; i < screenN; i++) {
    const x = doePoint(i, dims)
    const T = tuningFromX(x)
    screen.push({ x, r: evalFast(T) })
    if ((i + 1) % 16 === 0 || i + 1 === screenN) {
      const best = screen.reduce((a, b) => (b.r.j < a.r.j ? b : a))
      log(`screen ${i + 1}/${screenN}: best J ${best.r.j.toFixed(1)} margin ${best.r.p.margin.toFixed(1)} bellyMiss ${best.r.p.bellyPlan.miss.toFixed(0)} flipH ${best.r.p.flip.hBase.toFixed(0)} gate ${best.r.tun.gateMiss.toFixed(0)}`)
    }
  }
  const screenMs = performance.now() - screenT0
  screen.sort((a, b) => a.r.j - b.r.j)
  log(`\nscreen: ${screenN} evals in ${(screenMs / 1000).toFixed(1)} s (${(screenMs / screenN).toFixed(0)} ms/eval)`)
  log("top 8 screen:")
  screen.slice(0, 8).forEach((s, i) => {
    log(`  ${i + 1}. J ${s.r.j.toFixed(1)} margin ${s.r.p.margin.toFixed(1)} bellyMiss ${s.r.p.bellyPlan.miss.toFixed(0)} flip ${s.r.p.flip.hBase.toFixed(0)} m V ${s.r.p.flip.V.toFixed(0)} | gate ${s.r.tun.gateMiss.toFixed(0)} planEnd ${s.r.tun.bellyPlanEnd.toFixed(0)} aimHigh ${s.r.tun.aimHigh.toFixed(0)} aimE ${s.r.tun.bellyAimE.toFixed(0)} aimN ${s.r.tun.bellyAimN.toFixed(0)} bank ${s.r.tun.bellyBankRateDeg.toFixed(1)} wn ${s.r.tun.landingWn.toFixed(1)} tilt35 ${s.r.tun.tilt35mDeg.toFixed(0)} tilt12 ${s.r.tun.tilt12mDeg.toFixed(0)} termAlt ${s.r.tun.terminalSwitchAlt.toFixed(0)} roll ${s.r.tun.rollBandAeroDeg.toFixed(1)}`)
  })

  // sensitivity: |correlation| of each knob's unit value with J
  const meanJ = screen.reduce((a, s) => a + s.r.j, 0) / screen.length
  const sens = activeKnobs.map((k, d) => {
    const xs = screen.map((s) => s.x[d])
    const mx = xs.reduce((a, b) => a + b, 0) / xs.length
    let cov = 0, vx = 0, vj = 0
    screen.forEach((s) => { cov += (s.x[d] - mx) * (s.r.j - meanJ); vx += (s.x[d] - mx) ** 2; vj += (s.r.j - meanJ) ** 2 })
    return { k, corr: vx > 1e-12 && vj > 1e-12 ? cov / Math.sqrt(vx * vj) : 0 }
  }).sort((a, b) => Math.abs(b.corr) - Math.abs(a.corr))
  log(`sensitivity: ${sens.slice(0, 10).map((s) => `${s.k.key}:${s.corr.toFixed(2)}`).join(" ")}`)

  // ---------------- CMA-ES on the influential knobs ----------------
  // refine the whole physically-rankable set (the sensitivity ranking is reported above; the gate
  // and the aim biases are often low-correlated in a 24-sample screen yet dominate the delivery)
  activeKnobs = REFINE_KNOBS
  log(`\nCMA-ES on ${activeKnobs.map((k) => k.key).join(", ")}`)
  const seed = activeKnobs.map((k) => {
    // seed at the best screen value for the knob when it is in the refinement set
    const hit = screen[0]?.r.tun[k.key]
    return knobToUnit(k, hit ?? k.default)
  })
  const cmaT0 = performance.now()
  const cma = cmaEs(activeKnobs.length, seed, (x) => evalFast(tuningFromX(x)).j, generations)
  const cmaMs = performance.now() - cmaT0
  cma.sort((a, b) => a.j - b.j)
  log(`CMA-ES: ${generations} generations, ${cma.length} evals in ${(cmaMs / 1000).toFixed(1)} s`)
  log("top 6 CMA:")
  cma.slice(0, 6).forEach((c, i) => {
    const p = evalFast(c.tuning).p
    log(`  ${i + 1}. J ${c.j.toFixed(1)} margin ${p.margin.toFixed(1)} bellyMiss ${p.bellyPlan.miss.toFixed(0)} flipH ${p.flip.hBase.toFixed(0)} | ${activeKnobs.map((k) => `${k.key} ${c.tuning[k.key].toFixed(k.key.includes("Deg") ? 2 : 0)}`).join(" ")}`)
  })
  const fastMs = fastTimings.length > 0 ? [...fastTimings].sort((a, b) => a - b) : [0]
  log(`fast eval: median ${fastMs[Math.floor(fastMs.length / 2)].toFixed(0)} ms, p90 ${fastMs[Math.floor(0.9 * fastMs.length)].toFixed(0)} ms, n ${fastMs.length}`)

  // ---------------- full 6-DOF candidate set ----------------
  const bestFast = cma[0]?.tuning ?? screen[0].r.tun
  interface Labeled { readonly label: string; readonly tuning: MissionTuning }
  const labeled = (xs: (Labeled | null)[]): Labeled[] => {
    const seen = new Set<string>()
    const out: Labeled[] = []
    for (const x of xs) {
      if (!x) continue
      const k = JSON.stringify(x.tuning.toValues())
      if (seen.has(k)) continue
      seen.add(k)
      out.push(x)
    }
    return out
  }
  // The fast model cannot see the attitude-loop gains or the roll pulses, so the full set deliberately
  // includes those variants (the 6-DOF must decide them) plus aimHigh (which the fast hand-over cannot
  // rank) and the DOE/CMA winners for the knobs it *can* see.
  const set = labeled([
    { label: "fast-best", tuning: bestFast },
    { label: "gain-mid", tuning: bestFast.with({ landingWn: 2.4, landingAxisAccelYZ: 1.1, landingWMaxDeg: 35 }) },
    { label: "corridor-wide", tuning: bestFast.with({ tilt35mDeg: 40, tilt12mDeg: 26, tilt0mDeg: 18, terminalSwitchAlt: 40, armCloseAlt: 75 }) },
    { label: "roll-pitch", tuning: bestFast.with({ rollBandPitchDeg: 0.2, rollBandAeroDeg: 0.24, rollWnPitch: 0.8, rollWnAero: 0.8 }) },
    { label: "roll-all", tuning: bestFast.with({ rollBandEntryDeg: 0.2, rollBandAeroDeg: 0.2, rollBandBellyDeg: 0.2, rollBandPitchDeg: 0.2, rollWnEntry: 0.8, rollWnAero: 0.8, rollWnBelly: 0.6, rollWnPitch: 0.8 }) },
    { label: "land-aim-1", tuning: bestFast.with({ landingAimE: -95, landingAimN: -120 }) },
    { label: "land-aim-2", tuning: bestFast.with({ landingAimE: -190, landingAimN: -240 }) },
    { label: "aim-1500", tuning: bestFast.with({ aimHigh: 1500 }) },
    { label: "terminal-late", tuning: bestFast.with({ terminalSwitchAlt: 35, terminalSwitchLat: 12, armCloseAlt: 75 }) },
    { label: "cma-2", tuning: cma[1]?.tuning ?? null },
    { label: "aim-300", tuning: bestFast.with({ aimHigh: 300 }) },
    { label: "mpc-stiff", tuning: bestFast.with({ landingRd: 0.3, landingTermQp: 200, landingTermQv: 400, landingWMag: 0.05 }) },
    { label: "mpc-roll", tuning: bestFast.with({ landingRollMaxDeg: 30, landingWnRoll: 2.0, landingAxisAccelX: 0.5 }) },
    { label: "terminal-full", tuning: bestFast.with({ terminalSwitchAlt: 30, terminalSwitchLat: 20, terminalSwitchV: 15, tilt35mDeg: 30, tilt12mDeg: 25, tilt0mDeg: 20 }) },
  ])
  const cands = set.slice(0, Math.max(fullN, 1))
  const labelOf = new Map(set.map((x) => [JSON.stringify(x.tuning.toValues()), x.label]))
  log(`\nfull 6-DOF candidates: ${cands.length} (${workers} at a time)`)
  cands.forEach((c, i) => log(`  ${i + 1}. ${c.label.padEnd(14)} gate ${c.tuning.gateMiss.toFixed(0)} planEnd ${c.tuning.bellyPlanEnd.toFixed(0)} aimHigh ${c.tuning.aimHigh.toFixed(0)} bank ${c.tuning.bellyBankRateDeg.toFixed(1)} wn ${c.tuning.landingWn.toFixed(1)} tilt35 ${c.tuning.tilt35mDeg.toFixed(0)} termAlt ${c.tuning.terminalSwitchAlt.toFixed(0)} rollAero ${c.tuning.rollBandAeroDeg.toFixed(1)}`))

  // one full mission at a time on the machine: refuse if an external mission is running
  const pg = Bun.spawnSync(["pgrep", "-fa", "scripts/(mission|profile)"], { stdout: "pipe" })
  const pgOut = new TextDecoder().decode(pg.stdout).trim()
  if (pgOut.length > 0) {
    log(`\nanother mission is running:\n${pgOut}\nwaiting 30 s...`)
    await sleep(30_000)
  }

  const b64 = (T: MissionTuning): string => Buffer.from(JSON.stringify(T.toValues()), "utf8").toString("base64")
  const runFull = async (T: MissionTuning): Promise<{ tuning: MissionTuning; result: WorkerResult; label: string }> => {
    const proc = Bun.spawn(["nice", "-n", "19", process.execPath, import.meta.path, `--worker=${b64(T)}`], { stdout: "pipe", stderr: "pipe" })
    const [out, err] = await Promise.all([new Response(proc.stdout).text(), new Response(proc.stderr).text()])
    const code = await proc.exited
    if (code !== 0) throw new Error(`worker failed (${code}): ${err.slice(0, 500)}`)
    const line = out.split("\n").find((l) => l.startsWith("RESULT:"))
    if (!line) throw new Error(`worker produced no RESULT: ${out.slice(0, 500)}`)
    const result = JSON.parse(line.slice("RESULT:".length)) as WorkerResult
    return { tuning: T, result, label: labelOf.get(JSON.stringify(T.toValues())) ?? "candidate" }
  }

  interface WorkerResult {
    readonly tuning: Record<string, number>
    readonly wallMs: number
    readonly outcome: string
    readonly score: {
      readonly cost: number; readonly catchFeasible: boolean; readonly feasible: boolean; readonly catch: boolean; readonly margin: number
      readonly constraints: readonly { readonly id: string; readonly ok: boolean; readonly detail: string; readonly kind: string }[]
      readonly terms: {
        readonly catchPoint: { readonly lateral: number; readonly across: number; readonly along: number; readonly contactSpeed: number; readonly rollErrorDeg: number; readonly yawErrorDeg: number; readonly armsOpen: number; readonly pinLoad: number }
        readonly flipSpeed: number; readonly flipAlt: number; readonly bellyMiss: number; readonly landingBurn: number; readonly landingEngines: number
        readonly coldGasAfterFlip: number; readonly headerLeft: number; readonly coilCurrentAtFlip: number; readonly coilDischarged: boolean
        readonly peakG: number; readonly tileSurf: number; readonly tileBack: number; readonly bondline: number; readonly steel: number; readonly picaSurf: number
        readonly roll: readonly { readonly phase: string; readonly maxRollDeg: number; readonly n2Used: number }[]
        readonly n2Total: number
      }
    }
    readonly events: readonly string[]
  }

  const results: { tuning: MissionTuning; result: WorkerResult; label: string }[] = [{
    tuning: baselineTuning, label: "baseline",
    result: { tuning: baselineTuning.toValues(), wallMs: baseMs, outcome: baseRec.meta.outcome, score: baseScore, events: baseRec.events.map((e) => `${e.t.toFixed(1)} ${e.msg}`) },
  }]
  let cursor = 0
  const fullT0 = performance.now()
  const lane = async (): Promise<void> => {
    for (;;) {
      const i = cursor++
      if (i >= cands.length) return
      const T = cands[i].tuning
      try {
        const r = await runFull(T)
        results.push(r)
        const s = r.result.score
        log(`  done ${results.length}/${cands.length}: ${r.result.outcome} margin ${s.margin.toFixed(1)} m lat ${s.terms.catchPoint.lateral.toFixed(1)} speed ${s.terms.catchPoint.contactSpeed.toFixed(2)} catch ${s.catch} feasible ${s.feasible} | flip ${s.terms.flipAlt.toFixed(0)} m V ${s.terms.flipSpeed.toFixed(0)} belly ${s.terms.bellyMiss.toFixed(0)} burn ${s.terms.landingBurn.toFixed(0)} tile ${s.terms.tileSurf.toFixed(0)} bond ${s.terms.bondline.toFixed(0)} (${(r.result.wallMs / 1000).toFixed(0)} s)`)
      } catch (e) {
        log(`  worker ${i + 1} failed: ${String(e)}`)
      }
    }
  }
  await Promise.all(Array.from({ length: Math.min(workers, cands.length) }, () => lane()))
  const fullMs = performance.now() - fullT0

  // ---------------- Pareto table ----------------
  results.sort((a, b) => Number(b.result.score.catch) - Number(a.result.score.catch) || a.result.score.margin - b.result.score.margin)
  const dominates = (a: WorkerResult, b: WorkerResult): boolean => {
    const ta = a.score.terms, tb = b.score.terms
    const a1 = [a.score.margin, ta.landingBurn, ta.tileSurf], b1 = [b.score.margin, tb.landingBurn, tb.tileSurf]
    const le = a1.every((x, i) => x <= b1[i] + 1e-9)
    const lt = a1.some((x, i) => x < b1[i] - 1e-9)
    return le && lt
  }
  log(`\nfull 6-DOF results (${(fullMs / 1000).toFixed(0)} s):`)
  log("  #  label          catch feas  margin   lat  across along  speed  roll  yaw  flipH  V   belly  burn   tile  bond  g   engines n2  header  ref")
  results.forEach((r, i) => {
    const s = r.result.score, t = s.terms, c = t.catchPoint
    log(`  ${String(i + 1).padStart(2)} ${r.label.padEnd(14)} ${s.catch ? "YES  " : " no  "} ${s.feasible ? "YES " : "no  "} ${s.margin.toFixed(1).padStart(6)} ${c.lateral.toFixed(1).padStart(5)} ${c.across.toFixed(1).padStart(6)} ${c.along.toFixed(1).padStart(5)} ${c.contactSpeed.toFixed(2).padStart(6)} ${c.rollErrorDeg.toFixed(1).padStart(5)} ${c.yawErrorDeg.toFixed(1).padStart(4)} ${t.flipAlt.toFixed(0).padStart(5)} ${t.flipSpeed.toFixed(0).padStart(3)} ${t.bellyMiss.toFixed(0).padStart(5)} ${t.landingBurn.toFixed(0).padStart(5)} ${t.tileSurf.toFixed(0).padStart(5)} ${t.bondline.toFixed(0).padStart(5)} ${t.peakG.toFixed(1).padStart(4)} ${String(t.landingEngines).padStart(7)} ${t.n2Total.toFixed(0).padStart(4)} ${t.headerLeft.toFixed(0).padStart(6)} ${r.result.wallMs.toFixed(0).padStart(4)}`)
  })
  const front = results.filter((a) => !results.some((b) => b !== a && dominates(b.result, a.result)))
  log(`\nPareto front (margin, landing burn, tile): ${front.map((r) => r.result.score.margin.toFixed(1)).join(", ")}`)
  const best = results[0]
  log(`\nbest: catch ${best?.result.score.catch} margin ${best?.result.score.margin.toFixed(1)} m`)
  if (best) {
    log(`tuning: ${JSON.stringify(best.tuning.toValues())}`)
    log(`events: ${best.result.events.filter((e) => /flip|catch|CAUGHT|LOST|terminal|coil discharge/.test(e)).join(" | ")}`)
    log(`constraints: ${best.result.score.constraints.map((c) => `${c.id}:${c.ok ? "ok" : "FAIL"}(${c.detail})`).join("; ")}`)
  }
  const rollBase = results.find((r) => JSON.stringify(r.tuning.toValues()) === JSON.stringify(baselineTuning.toValues()))
  if (rollBase) {
    log(`baseline roll per phase: ${rollBase.result.score.terms.roll.map((p) => `${p.phase} ${p.maxRollDeg.toFixed(2)}°/s N2 ${p.n2Used.toFixed(1)} kg`).join("  ")}`)
  }

  writeFileSync(outFile, JSON.stringify({
    baseline: { tuning: baselineTuning.toValues(), wallMs: baseMs, score: baseScore, handover: hand },
    screen: screen.map((s) => ({ x: s.x, j: s.r.j, tuning: s.r.tun.toValues(), margin: s.r.p.margin, bellyMiss: s.r.p.bellyPlan.miss, flipH: s.r.p.flip.hBase })),
    cma: cma.map((c) => ({ j: c.j, tuning: c.tuning.toValues() })),
    fastTimings,
    full: results.map((r) => ({ tuning: r.tuning.toValues(), result: r.result })),
    pareto: front.map((r) => r.tuning.toValues()),
  }, null, 2))
  log(`\nreport: ${outFile.pathname}`)
}

main().catch((e) => { console.error(e); process.exit(1) })
