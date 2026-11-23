import { type V3, v3, add, sub, scale, cross, norm, normalize, dot, qFromAxisAngle, qFromBasis, qmul, qrot } from "@ship/math"
import { corotation, OMEGA_EARTH, rotZ } from "@ship/earth"
import { type EngineCmd, type EngineState, FLIGHT_DESIGN, fVac100, massProps, type MassProps } from "@ship/craft"
import { ENGINE_HEALTH0, LANDING_LAW_STATE0, trimThrustLine, type MissionTuning } from "@ship/gnc"
import { TOWER } from "@ship/tower"
import { initState, makeWorld, type SimState, type World } from "@ship/sim"

/**
 * Isolated landing rig.
 *
 * The full mission delivers the landing its initial state through 2 900 s of entry, so a change in
 * the landing control cannot be told apart from a change in what it was handed. The rig builds the
 * landing state directly: the same 6-DOF plant (`step`), the same vehicle, tower and flight
 * software, no wind and no turbulence, the ship placed upright at a chosen pin position and
 * velocity with a chosen engine set already running. Everything measured here is therefore a
 * property of the vehicle model and its control, nothing else.
 */
export interface RigOptions {
  /** catch pins relative to the catch point, pad ENU (m): x across the rails (east), y along them (north), z above the rails */
  readonly pin: V3
  /** ground velocity, pad ENU (m/s) */
  readonly vel?: V3
  /** running sea-level engines (indices 0–2). One engine: 0 is the one in the pin plane. */
  readonly engines?: readonly number[]
  readonly throttle?: number
  /** landing propellant in the header tanks (kg) */
  readonly header?: number
  /** extra tilt of the body axis from the trimmed attitude (deg) towards the compass azimuth `tiltAzDeg` */
  readonly tiltDeg?: number
  readonly tiltAzDeg?: number
  /** start in the flight phase "landing" (default) — the law then owns the descent */
  readonly t?: number
}

/** consumables of the nominal no-wind mission at flip completion (measured), header overridable */
export const RIG_CONS = { loxMain: 485.5, ch4Main: 134.9, n2: 3123, seed: 0, header: 19600 } as const

let cached: World | null = null
/** the no-wind world of the flown design; prediction-lite (no thermal integration) */
export const rigWorld = (tuning?: MissionTuning): World => {
  cached ??= { ...makeWorld({ ...FLIGHT_DESIGN, windScale: 0, turbulence: 0 }), lite: true }
  return tuning ? { ...cached, guide: { ...cached.guide, tuning } } : cached
}

/** gimbal angles that put an engine's thrust line through the centre of gravity */
export const trimGimbal = (W: World, i: number, xcg: number): { gp: number; gy: number } => {
  const g = W.vehicle.engines[i].gimbal
  const d = normalize(v3(xcg - g.x, -g.y, -g.z))
  const gy = Math.asin(d.y)
  return { gp: Math.asin(-d.z / Math.cos(gy)), gy }
}

export interface Rig {
  readonly world: World
  readonly state: SimState
  readonly mass: MassProps
  /** pad frame at the rig time (ECI unit vectors and the pad position) */
  readonly east: V3
  readonly north: V3
  readonly up: V3
  readonly padEci: V3
}

export const landingRig = (o: RigOptions, world: World = rigWorld()): Rig => {
  const W = world
  const t = o.t ?? 3000
  const theta = W.plan.theta0 + OMEGA_EARTH * t
  const padEci = rotZ(W.padEcef, theta)
  const east = rotZ(W.padEnu.e, theta), north = rotZ(W.padEnu.n, theta), up = rotZ(W.padEnu.u, theta)
  const fromPad = (x: V3): V3 => add(add(scale(east, x.x), scale(north, x.y)), scale(up, x.z))
  const s0 = initState(W)
  const header = o.header ?? RIG_CONS.header
  const mr = W.vehicle.mixtureRatio
  const cons = { ...s0.cons, loxMain: RIG_CONS.loxMain, ch4Main: RIG_CONS.ch4Main, n2: RIG_CONS.n2, seed: RIG_CONS.seed, loxHeader: (header * mr) / (1 + mr), ch4Header: header / (1 + mr) }
  const mp = massProps(W.vehicle, cons)
  const sel = [...(o.engines ?? [0])].sort((a, b) => a - b)
  const throttle = o.throttle ?? 0.6
  // body x up, y east (pins over the east/west rails), z north; then the trim tilt of the running
  // set (thrust line through the CG is vertical) and the requested extra tilt
  let q = qFromBasis(up, east, north)
  const eng0 = W.vehicle.engines[0]
  const tNom = (i: number): number => (sel.includes(i) ? throttle * fVac100(eng0) : 0)
  const line = qrot(q, trimThrustLine(W.vehicle.engines.slice(0, 3), [0, 1, 2].map(tNom), mp.xcg))
  const ax = cross(line, up)
  if (norm(ax) > 1e-9) q = qmul(qFromAxisAngle(normalize(ax), Math.asin(Math.min(1, norm(ax)))), q)
  if (o.tiltDeg) {
    const az = ((o.tiltAzDeg ?? 0) * Math.PI) / 180
    const toward = add(scale(east, Math.sin(az)), scale(north, Math.cos(az)))
    q = qmul(qFromAxisAngle(normalize(cross(up, toward)), (o.tiltDeg * Math.PI) / 180), q)
  }
  const pinPad = v3(o.pin.x, o.pin.y, o.pin.z + TOWER.armHeight)
  const r = sub(add(padEci, fromPad(pinPad)), qrot(q, v3(W.vehicle.pinX - mp.xcg, 0, 0)))
  const v = add(corotation(r), fromPad(o.vel ?? v3(0, 0, 0)))
  // the attitude is Earth-fixed: body rate = Earth rate (7e-5 rad/s), expressed in body axes
  const eng: EngineState[] = [0, 1, 2].map((i) => {
    const tg = trimGimbal(W, i, mp.xcg)
    return sel.includes(i) ? { on: true, spool: 1, throttle, gp: sel.length === 1 ? tg.gp : 0, gy: sel.length === 1 ? tg.gy : 0 } : { on: false, spool: 0, throttle: 0.4, gp: 0, gy: 0 }
  })
  const single = sel.length === 1
  const state: SimState = {
    ...s0, t, r, v, q, w: v3(0, 0, 0), cons, eng,
    coil: { ...s0.coil, mode: "discharge", I: 0 },
    mem: {
      ...s0.mem, phase: "landing", tPhase: t, coilOff: true, tFlip: t - 9, engSel: sel,
      engHealth: { ...ENGINE_HEALTH0, onT: [0, 1, 2].map((i) => (sel.includes(i) ? t - 10 : -1)) },
      land: single ? { ...LANDING_LAW_STATE0, staged: true, stageT: t, nEng: 1 } : null,
    },
    events: [],
  }
  void dot
  return { world: W, state, mass: mp, east, north, up, padEci }
}

/** open-loop actuator injection for plant identification: the commands that reach the engines */
export type ActuatorOverride = (t: number, cmds: readonly EngineCmd[]) => readonly EngineCmd[]
export const withActuators = (W: World, f: ActuatorOverride): World => ({ ...W, actuatorOverride: f })
