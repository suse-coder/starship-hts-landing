import { type DesignParams, DEFAULT_DESIGN, massProps } from "@ship/craft"
import { PHASES } from "@ship/gnc"
import { CH, type Recording, recorder } from "./recording"
import { initState, type SimState } from "./state"
import { step } from "./step"
import { makeWorld, type World } from "./world"

/** recording interval by phase (s) */
const recDt = (s: SimState): number => {
  switch (s.mem.phase) {
    case "deorbitAlign":
    case "entryAlign":
      return 2.0
    case "deorbitBurn":
      return 0.25
    case "entry":
    case "loft":
      return 0.25
    case "pitchUp":
      return 0.1
    case "highAlpha":
    case "bellyFlop":
      return 0.2
    case "flip":
    case "landing":
      return 0.05
    case "terminal":
    case "caught":
    case "lost":
      return 0.04
  }
}

export interface RunProgress {
  readonly stage: string
  readonly t: number
  readonly phase: string
  readonly alt: number
  readonly fraction: number
}

export interface RunResult {
  readonly recording: Recording
  readonly world: World
  readonly final: SimState
}

/** Run a complete mission: deorbit → entry → catch. Pure apart from progress callbacks. */
export const runMission = (design: DesignParams = DEFAULT_DESIGN, onProgress: (p: RunProgress) => void = () => {}): RunResult => {
  const t0 = performance.now()
  const world = makeWorld(design, (stage) => onProgress({ stage, t: 0, phase: "planning", alt: 0, fraction: 0 }))
  let s = initState(world)
  const rec = recorder(world.grid.cellX.length)
  const phaseStarts: { phase: string; t: number }[] = [{ phase: s.mem.phase, t: 0 }]
  let lastRec = -1e9
  let lastProg = 0
  let steps = 0
  const tEnd = world.plan.pred.t + 200
  while (!s.done) {
    const out = step(world, s)
    steps++
    if (out.s.mem.phase !== s.mem.phase) phaseStarts.push({ phase: out.s.mem.phase, t: out.s.t })
    if (s.t - lastRec >= recDt(s) || out.s.done || out.s.mem.phase !== s.mem.phase) {
      rec.push(out.row, out.temps, out.pred)
      lastRec = s.t
    }
    s = out.s
    if (performance.now() - lastProg > 150) {
      lastProg = performance.now()
      onProgress({ stage: "simulating", t: s.t, phase: s.mem.phase, alt: out.row[CH.alt], fraction: Math.min(0.999, s.t / tEnd) })
    }
  }
  const p = s.peak
  const wallClockMs = performance.now() - t0
  const summary: Record<string, number | string> = {
    outcome: s.outcome,
    reason: s.mem.reason,
    duration: s.t,
    peakG: p.g,
    peakQbar: p.qbar,
    peakQStag: p.qStag,
    peakPicaSurf: p.picaSurf,
    peakBondline: p.bondline,
    peakTileSurf: p.tileSurf,
    peakTileBack: p.tileBack,
    peakSteel: p.steel,
    peakMhdForce: p.mhdForce,
    peakCoilT: p.coilT,
    n2Left: s.cons.n2,
    seedLeft: s.cons.seed,
    headerLeft: s.cons.loxHeader + s.cons.ch4Header,
    massFinal: massProps(world.vehicle, s.cons).m,
    coilIEnd: s.coil.I,
    coilEEnd: 0.5 * world.vehicle.coil.L * s.coil.I ** 2,
  }
  const recording = rec.finish(s.events, {
    epochJd: world.plan.epochJd, theta0: world.plan.theta0,
    padLat: world.plan.pad.lat, padLon: world.plan.pad.lon, padAlt: world.plan.pad.alt,
    outcome: s.outcome, summary, phaseStarts,
    predPath: world.plan.pred.path,
    vehicle: { design: world.design, components: world.vehicle.components, coil: world.vehicle.coil, pica: world.vehicle.pica, pinX: world.vehicle.pinX, pinOut: world.vehicle.pinOut, phases: PHASES },
    wallClockMs, steps,
  })
  return { recording, world, final: s }
}
