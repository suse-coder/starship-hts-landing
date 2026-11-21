import { type Q, type V3, v3, add, clamp, cross, dot, norm, normalize, qrotInv, scale, sub, DEG } from "@ship/math"
import { type EngineCmd, fVac100, type VehicleConfig } from "@ship/craft"
import { makeMhdModel, mhdBasis, type MhdBasis } from "@ship/mhd"
import { SHOCK_TABLE, lookupStag } from "@ship/gas"
import { type AttitudeTarget, GAINS, type Gains, torqueCommand } from "./control"
import { allocateMhd, U_BASE } from "./mhdAlloc"
import { planLanding, terminalControl, flipAltitude, LATERAL_FIELD_DEFAULT } from "./landing"
export { flipAltitude }
import { dampedBank, GLIDE_DAMPING, NOSE_FIRST, type GlideState, passiveTrim, pitchUpDue, predict, type PredContext, type PredInput, type PredOut, type PredPhase, solveBank } from "./predictor"
import { aeroLookup, type AeroTable, mhdDragLookup } from "./tables"
import { atmosphere } from "@ship/earth"
import { alphaNoseFirst } from "./schedule"
import { MISSION_TUNING_DEFAULT, type MissionTuning } from "./tuning"
import { TOWER, bothPinsLoaded } from "@ship/tower"
export { alphaNoseFirst }

/** Mission phases, in order. */
export const PHASES = [
  { id: "deorbitAlign", label: "Deorbit attitude (cold gas)" },
  { id: "deorbitBurn", label: "Deorbit burn (Raptor SL ×1)" },
  { id: "entryAlign", label: "Nose-prograde slew + coil ramp" },
  { id: "entry", label: "Entry: MHD-trimmed nose-first glide" },
  { id: "loft", label: "Pre-transition loft (lift up)" },
  { id: "pitchUp", label: "Pitch-up to aero trim" },
  { id: "highAlpha", label: "High-α glide + barbecue roll" },
  { id: "bellyFlop", label: "Belly flop (cold-gas heading)" },
  { id: "flip", label: "Landing flip (3 engines)" },
  { id: "landing", label: "Landing burn (1 engine)" },
  { id: "terminal", label: "Terminal descent into the arms" },
  { id: "caught", label: "Caught by Mechazilla" },
  { id: "lost", label: "Vehicle lost" },
] as const
export type Phase = (typeof PHASES)[number]["id"]
export const phaseIndex = (p: Phase): number => PHASES.findIndex((x) => x.id === p)

/** Perfect-navigation state handed to guidance by the simulator. */
export interface Nav {
  readonly t: number
  readonly r: V3
  readonly v: V3
  readonly q: Q
  readonly w: V3
  readonly I: V3
  readonly m: number
  readonly xcg: number
  readonly alt: number
  readonly up: V3 // ECI
  readonly east: V3
  readonly north: V3
  readonly vAir: V3 // ECI air-relative velocity
  readonly V: number
  readonly mach: number
  readonly qbar: number
  readonly pAmb: number
  readonly alpha: number
  readonly t0Stag: number
  readonly padEci: V3 // pad origin in ECI
  readonly pinRel: V3 // ENU of the axis point at the pin station, relative to the catch point
  readonly velRel: V3 // ENU ground-relative velocity of the CG
  readonly hBase: number // height of the aft end above the pad ground, m
  readonly hLow: number // height of the lowest hull point above the ground, m
  readonly mhd: MhdBasis | null
  readonly aeroM: V3 // current aerodynamic moment about the CG (body) — feedforward
  readonly coilI: number
  readonly coilIop: number
  readonly seedLeft: number
  readonly propHeader: number
  readonly thrusts: readonly number[]
  readonly spoolDone: boolean
  readonly pinLoad: number
  /** smallest per-pin vertical load (N); 0 when a pin is not on a rail */
  readonly pinLoadMin: number
  /** pins carrying load on a rail (0..2) */
  readonly pinCount: number
  readonly accThrust: number // |thrust| / m, m/s²
  readonly mhdDrag: number // N, MHD force component opposing the motion
}

export interface Mem {
  readonly phase: Phase
  readonly tPhase: number
  readonly bank: number // magnitude, rad
  readonly bankSign: number
  readonly bankHigh: number
  readonly npcT: number
  readonly revT: number // time of the last bank reversal
  readonly pred: PredOut | null
  readonly burnDv: number
  readonly armsClose: boolean
  readonly caughtT: number
  readonly reason: string
  readonly mhdScale: number // filtered actual/tabulated MHD drag (feeds the predictor)
  readonly qPrev: number
  readonly heading: number | null // belly-flop glide heading (rad from North, clockwise)
  readonly bellyMiss: number
  readonly sigmaCmd: number | null // bank-glide law: rate-limited commanded bank (rad, signed)
  readonly sigmaTgt: number // bank-glide law: current target bank
  readonly tRevPlan: number // absolute time of the planned high-α bank reversal (Infinity: none)
  readonly hdgCmd: number | null // belly flop: rate-limited commanded lift heading (rad from North)
  readonly liftCmd: V3 | null // commanded lift unit vector (ECI): carried with v̂, rotated about v̂ at a limited rate
  readonly windOfs: V3 // measured local wind minus forecast (low-pass), ENU
  readonly tLandEnd: number // landing burn: fixed arrival time at the hover point above the rails
  readonly mpcU: Float64Array | null // landing MPC: last plan (warm start)
  readonly landEng: number // sea-level Raptor kept for the single-engine final approach
  readonly nEngLand: number // landing engines (2: differential gimbal for roll)
  readonly aProf: number // landing cruise (divert) vertical deceleration, fixed at the hand-over (m/s²)
  readonly zProf: number // altitude where the cruise ends and the near-vertical final descent starts (m)
  readonly uDirPrev: V3 // previous commanded thrust direction (ECI) for the steering-rate lead
  readonly uDirT: number // time of uDirPrev
  readonly leadF: V3 // filtered steering-rate lead (ECI), the attitude-target feed-forward
  /** landing MPC predicted positions from the current state to the catch, pad ENU metres measured from
   *  the pad ground origin (E, N, U; U includes the arm height). Visual export only — no control use. */
  readonly predPath: readonly V3[]
  readonly mpcT: number // time of the last MPC solve
  readonly uCmd: readonly [number, number, number] // MPC thrust-acceleration command (ENU)
  readonly tSwitch: number // belly flop: absolute time of the switch to the second bank σ₂ (Infinity: never)
  readonly sigma2: number // belly flop: second-segment bank
  readonly coilOff: boolean
  readonly coilLowT: number // first time the steering authority fell below 1 (hysteresis clock) // the coil has been dumped (no longer needed for shielding or steering, T13)
  readonly rcsT: number // landing: time of the last logged roll-RCS pulse
  readonly flapsDeployed: boolean // nose-first: aft flaps deployed at coil-authority loss
}

export interface Cmd {
  readonly att: AttitudeTarget
  readonly gains: Gains
  readonly feedForward: boolean
  /**
   * Body-frame aerodynamic moment guidance predicts from its *own* aero table at the current α/Mach
   * (R6, T08 P0-2). The plant should cancel this instead of its own `aero.M` (which is an oracle: it
   * includes the plant's rate damping and every coefficient exactly). The static part is what the
   * tables can predict; short-period rate damping is left to the rate loop.
   */
  readonly ffMoment: V3
  readonly useMhd: boolean
  readonly useRcs: boolean
  /** aft flaps deployed (3) for the nose-first drag/steering fall; the flap worker owns the aero/thermal force */
  readonly flapsDeployed: boolean
  /** the RCS is enabled for roll-only pulses (no translation, no pitch/yaw): one-engine landing stage */
  readonly rcsRollOnly: boolean
  readonly engines: readonly EngineCmd[]
  readonly seeding: boolean
  readonly coil: "hold" | "ramp" | "rampdown" | "dump" | "discharge"
  readonly armsClose: boolean
  readonly alphaCmd: number
}

export interface GuideCtx {
  readonly vehicle: VehicleConfig
  readonly pred: PredContext
  readonly aero: AeroTable
  readonly tBurn: number
  readonly dvBurn: number
  readonly armHeight: number
  readonly catchHold: number // s after load transfer
  readonly tRamp: number // s, coil ramp start (ends just before entry interface)
  /** wind forecast (ENU, m/s) vs altitude: the pre-flight climatology/forecast (review/00 I9) */
  readonly windForecast: (h: number) => V3
  /**
   * Mission tuning. Optional so a world built before the tuning refactor is identical to the flown
   * mission; `guide` falls back to `MISSION_TUNING_DEFAULT` (the current constants).
   */
  readonly tuning?: MissionTuning
}

/** range bias (m past the pad) per phase: later phases can only dissipate range, never add it */
export const AIM_ENTRY = 180e3
/**
 * High-α hand-over bias (m past the pad). The high-α corrector aims its *predicted* endpoint while the
 * belly-flop terminal law decelerates and drifts, so the hand-over needs a bias. Measured on the flown
 * trajectories (flip miss at the 2.0 km trigger / at 1.4 km): 0 → 1167/923 m, 900 → 770/530 m,
 * 1800 → 883/637 m. 900 is the optimum; it is a bias on the hand-over *position*, not a control gain.
 */
export const AIM_HIGH = 900

export const initMem = (bank0: number): Mem => ({
  phase: "deorbitAlign", tPhase: 0, bank: bank0, bankSign: 1, bankHigh: 30 * DEG, npcT: -1e9, revT: -1e9, pred: null,
  burnDv: 0, armsClose: false, caughtT: 0, reason: "", mhdScale: 1.0, qPrev: 1e9, heading: null, bellyMiss: 0, sigmaCmd: null, sigmaTgt: 0, tRevPlan: Infinity, hdgCmd: null, liftCmd: null, windOfs: v3(0, 0, 0), tSwitch: Infinity, sigma2: 0, tLandEnd: Infinity, mpcU: null, mpcT: -1e9, landEng: 0, nEngLand: 1, aProf: 0, zProf: 0, uDirPrev: v3(0, 0, 1), uDirT: -1e9, leadF: v3(0, 0, 0), predPath: [], uCmd: [0, 0, 9.79],
  coilOff: false, coilLowT: -1e9, rcsT: -1e9, flapsDeployed: false,
})

/**
 * Coil switch-off (Q25, 2026-11-21; R11b). The field is the pitch/yaw/roll actuator of the nose-first
 * glide, so it stays on while it can still steer: the ship must not tumble during the fall.
 *
 * Authority is a flight-software estimate (R6), never the simulator's internal `mhdAuth` or `n.mhd`:
 * guidance rebuilds the MHD interaction basis from its *own* air data (altitude, airspeed, dynamic
 * pressure, static pressure and its own shock-table stagnation state) with the design hall closure and
 * the seed actually available, then asks its own `allocateMhd` what moment it can realise against the
 * **three-axis** torque the attitude law demands. (T08 P0-3: the previous version consumed the plant's
 * `n.mhd` basis, an oracle.) The coil ramps down (battery, P_max 450 kW, no dump) only after the
 * authority has stayed below 1 for 3 s, or when the plasma is gone entirely. There is no re-ramp.
 * The fast dump (τ = 2 s, ≈ 97 kV) remains a fault path only (Q19/Q20 open).
 */
const COIL_AUTH_MIN = 1.0 // assumed design parameter (R5); sensitivity tested at 0.8 and 1.2
const COIL_OFF_HOLD = 3.0 // s below the threshold before the planned switch-off (assumed)
const mhdModels = new WeakMap<object, ReturnType<typeof makeMhdModel>>()
const mhdModelOf = (v: VehicleConfig): ReturnType<typeof makeMhdModel> => {
  const memo = mhdModels.get(v)
  if (memo) return memo
  const made = makeMhdModel(v)
  mhdModels.set(v, made)
  return made
}
const coilAuthority = (ctx: GuideCtx, n: Nav, att: AttitudeTarget, gains: Gains): number => {
  const tc = torqueCommand(n.q, n.w, n.I, att, gains, v3(0, 0, 0))
  const demand = Math.hypot(tc.tau.x, tc.tau.y, tc.tau.z) // three-axis: roll is part of the demand
  if (demand < 1) return Infinity // no steering demand: authority is not what keeps the coil on (n/a)
  if (n.coilI < 1 || n.V < 150) return 0
  const h = Math.max(n.alt, 0)
  const atm = atmosphere(h)
  const stag = lookupStag(SHOCK_TABLE, h, n.V)
  const vhatB = qrotInv(n.q, normalize(n.vAir))
  const b = mhdBasis(mhdModelOf(ctx.vehicle), {
    NI: n.coilI * ctx.vehicle.coil.N, vhat: vhatB, U: n.V, rhoInf: atm.rho, pInf: atm.p, qInf: n.qbar,
    stag, xcg: n.xcg, kappa: ctx.vehicle.design.hallClosure,
    seedFraction: n.seedLeft > 0 ? ctx.vehicle.design.seedFraction : 0,
  })
  const al = allocateMhd(b, tc.tau, n.qbar, ctx.vehicle.shape.p.noseRadius, U_BASE)
  return al.maxMoment / demand
}
const coilCommand = (ctx: GuideCtx, n: Nav, mem: Mem, ev: string[], att: AttitudeTarget, gains: Gains, plasma: boolean): { coil: Cmd["coil"]; mem: Mem } => {
  if (mem.coilOff) return { coil: "discharge", mem }
  const auth = plasma ? coilAuthority(ctx, n, att, gains) : 0
  if (auth >= COIL_AUTH_MIN) return { coil: "hold", mem: { ...mem, coilLowT: -1e9 } }
  const tLow = mem.coilLowT < 0 ? n.t : mem.coilLowT
  if (n.t - tLow < COIL_OFF_HOLD) return { coil: "hold", mem: { ...mem, coilLowT: tLow } }
  ev.push(`coil discharge (1 Ω tap, ≤ 3 kV, τ ≈ 65 s): steering authority ${auth.toFixed(2)} < ${COIL_AUTH_MIN} for ${COIL_OFF_HOLD} s${plasma ? "" : " (plasma gone)"}; cold gas and the engines steer from here`)
  return { coil: "discharge", mem: { ...mem, coilOff: true } }
}


const ENGINES_OFF: readonly EngineCmd[] = [0, 1, 2].map(() => ({ on: false, throttle: 0.4, gp: 0, gy: 0 }))
const enginesOn = (n: number, throttle: number): EngineCmd[] => [0, 1, 2].map((i) => ({ on: i < n, throttle, gp: 0, gy: 0 }))

/** pre-flip roll damping: tighten the roll-rate band and give the roll loop its own bandwidth */
const rollGains = (g: Gains, bandDeg: number, wnRoll: number): Gains => (bandDeg > 0 ? { ...g, rollBand: bandDeg * DEG, wnRoll } : { ...g, wnRoll })
/** the thrust-direction corridor height schedule (tuning fields; defaults are the flown ramp) */
const tiltDegFor = (pz: number, T: MissionTuning): number =>
  pz > 300 ? T.tilt300mDeg
    : pz > 120 ? T.tilt120mDeg
      : pz > 60 ? T.tilt60mDeg
        : pz > 35 ? T.tilt35mDeg
          : pz > 12 ? T.tilt12mDeg
            : T.tilt0mDeg
/**
 * Engines kept after the flip (R11c, Q29/Q33, T09). Two sea-level Raptors at the 40 % throttle floor
 * give T/W ≈ 1.3–1.8 at 110–140 t, so a hover is impossible: the terminal is a timed deceleration
 * (hoverslam, `landing.ts`). Two engines keep the 3×2N gimbal map full rank, so roll comes from
 * differential gimbal — no cold gas. The third engine is only used while the ship is still rotating
 * (attitude authority), never after the flip.
 *
 * The engine count and contact speed are mission tuning fields (`landingEngines`, `contactSpeed`);
 * `landingEngines` is pinned to 2 by the schema (hard rule), so the search cannot buy a 3-engine burn.
 */

/** nose direction for angle of attack α with the lift vector rotated by bank σ from "up" */
export const noseFor = (vAir: V3, up: V3, alpha: number, bank: number): { x: V3; z: V3; lift: V3 } => {
  const vh = normalize(vAir)
  const upP = normalize(sub(up, scale(vh, dot(up, vh))))
  const right = cross(vh, upP)
  const lift = add(scale(upP, Math.cos(bank)), scale(right, Math.sin(bank)))
  const x = add(scale(vh, Math.cos(alpha)), scale(lift, Math.sin(alpha)))
  const z = add(scale(vh, -Math.sin(alpha)), scale(lift, Math.cos(alpha)))
  return { x, z, lift }
}

const horiz = (v: V3, up: V3): V3 => sub(v, scale(up, dot(v, up)))

/** the α-pitch axis x_b × v̂ (ECI): rotation about it changes α, which the passive trim owns */
const alphaAxis = (n: Nav): V3 => {
  const q = n.q
  const xb = v3(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y))
  return normalize(cross(xb, normalize(n.vAir)))
}

/** azimuth error to the pad (rad, + = pad to the left of the ground track) */
const azimuthError = (n: Nav): number => {
  const hdg = normalize(horiz(n.vAir, n.up))
  const tgt = normalize(horiz(sub(n.padEci, n.r), n.up))
  return Math.atan2(dot(cross(hdg, tgt), n.up), dot(hdg, tgt))
}
/** crossrange to the pad (m, + = pad to the left) and range */
const crossrange = (n: Nav): { cr: number; range: number } => {
  const range = norm(horiz(sub(n.padEci, n.r), n.up))
  return { cr: range * Math.sin(azimuthError(n)), range }
}
/** bank-reversal deadband on crossrange distance: 8 km + 2.5 % of the range to go */
const crDeadband = (range: number, floor = 8000, frac = 0.025): number => floor + frac * range

const predInput = (n: Nav, mem: Mem, phase: PredPhase): PredInput => ({
  t: n.t, r: n.r, v: n.v, mass: n.m, phase, bankEntry: mem.bank, bankHigh: mem.bankHigh, mhdDragScale: mem.mhdScale,
})

/**
 * Glide state for the damped high-α bank law (predictor.ts dampedBank), from navigation: geodetic
 * altitude rate, and the TRIM C_L, C_D of the guidance table at the current Mach (as the predictor
 * flies them; the instantaneous α carries the short-period oscillation and is not fed back).
 */
const glideOf = (ctx: GuideCtx, n: Nav): GlideState => {
  // measured α (nav) rather than the table passive trim: the guard must not under-predict the actual lift
  const co = aeroLookup(ctx.aero, n.alpha, n.mach)
  return { h: n.alt, hdot: dot(n.vAir, n.up), V: n.V, r: norm(n.r), q: n.qbar, CL: co.CL, CD: co.CD, mass: n.m }
}
/**
 * Belly-flop plan end above the catch plane (m). This is deliberately **decoupled from L's gate**.
 * `flipAltitude` tracks the gate design, so when the landing moved the gate from 2.0 km down to the
 * 1.4 km late option the plan end would follow it down and the non-convex terminal optimiser would
 * degrade badly: aiming at 1.33–1.4 km gives 1.68 km predicted / 1.85 km flown, whereas keeping the
 * end at 1.75 km gives ≈650 m at the 1.4 km trigger. G fixed this at 1750 m and it stays fixed here;
 * it is a tuning field (`bellyPlanEnd`), not a function of the gate.
 */
const hFlipPred = (ctx: GuideCtx, T: MissionTuning): number => ctx.armHeight - ctx.vehicle.pinX + T.bellyPlanEnd
/**
 * Guidance's own body-frame aerodynamic moment (R6, T08 P0-2).
 *
 * The table stores the pitching-moment coefficient for flow in the body x–z plane, i.e.
 * c_m = M_y/(q S L) with the normal force along ±z. At a banked high-α attitude the flow is not in
 * that plane: the *same* normal force (normal to the body axis, opposite the transverse flow) makes
 * both M_y and M_z with the same arm. Projecting the tabulated moment onto the actual transverse flow
 * gives, with s = |(v_y, v_z)|,
 *   M_y = −c_m q S L · v_z/s,    M_z = +c_m q S L · v_y/s,
 * which reduces to the table value for v_y = 0. Only the *static* moment is cancelled: the rate
 * damping stays in the plant (the old oracle cancelled that too, removing physical damping).
 */
export const momentProjection = (m: number, vy: number, vz: number): V3 => {
  const s = Math.hypot(vy, vz)
  if (s < 1e-9) return v3(0, 0, 0)
  return v3(0, (-m * vz) / s, (m * vy) / s)
}
const ownAeroMoment = (ctx: GuideCtx, n: Nav): V3 => {
  const vb = qrotInv(n.q, n.vAir)
  const co = aeroLookup(ctx.aero, n.alpha, n.mach)
  return momentProjection(co.Cm * n.qbar * ctx.pred.sref * ctx.vehicle.shape.p.length, vb.y, vb.z)
}

const throttleFor = (v: VehicleConfig, nEng: number, thrustTotal: number, pAmb: number): number => {
  const e = v.engines[0]
  return clamp((thrustTotal / nEng + pAmb * e.exitArea) / fVac100(e), e.minThrottle, 1)
}

const enuToEci = (n: Nav, a: V3): V3 => add(add(scale(n.east, a.x), scale(n.north, a.y)), scale(n.up, a.z))

/** limit the thrust direction to within maxTilt of vertical and above a minimum magnitude */
const shapeThrust = (a: V3, maxTilt: number, aMin: number): V3 => {
  const h = Math.hypot(a.x, a.y)
  const vz = Math.max(a.z, 0.5)
  const hMax = vz * Math.tan(maxTilt)
  const s = h > hMax ? hMax / h : 1
  const b = v3(a.x * s, a.y * s, vz)
  const m = norm(b)
  return m < aMin ? scale(b, aMin / m) : b
}

export interface GuideOut {
  readonly cmd: Cmd
  readonly mem: Mem
  readonly events: readonly string[]
}

const to = (mem: Mem, phase: Phase, t: number, extra: Partial<Mem> = {}): Mem => ({ ...mem, ...extra, phase, tPhase: t })

export const guide = (ctx: GuideCtx, n: Nav, mem0: Mem, dt: number): GuideOut => {
  const v = ctx.vehicle
  const T = ctx.tuning ?? MISSION_TUNING_DEFAULT
  const ev: string[] = []
  let mem = mem0
  const base: Omit<Cmd, "att"> = {
    gains: GAINS.vacuum, feedForward: false, ffMoment: ownAeroMoment(ctx, n), useMhd: false, useRcs: true, rcsRollOnly: false, flapsDeployed: false, engines: ENGINES_OFF,
    seeding: false, coil: "hold", armsClose: mem.armsClose, alphaCmd: n.alpha,
  }
  const coilOn = n.coilI > 0.5 * n.coilIop
  const plasma = n.t0Stag > 1800 && n.qbar > 30
  const mhdUsable = coilOn && plasma && n.mhd !== null
  const tp = n.t - mem.tPhase
  // ---------------- failure checks ----------------
  if (mem.phase !== "lost" && mem.phase !== "caught" && n.hLow < 0.05 && n.alt < 2000) {
    mem = to(mem, "lost", n.t, { reason: "ground impact" })
    ev.push(`LOST: ground impact at ${norm(n.velRel).toFixed(1)} m/s`)
  }
  switch (mem.phase) {
    case "deorbitAlign": {
      const att = { x: scale(normalize(n.v), -1), z: n.up }
      const aEst = (0.6 * 2.585e6) / n.m
      const tStart = ctx.tBurn - ctx.dvBurn / aEst / 2
      if (n.t >= tStart) {
        mem = to(mem, "deorbitBurn", n.t, { burnDv: 0 })
        ev.push(`deorbit burn ignition (Δv target ${ctx.dvBurn.toFixed(0)} m/s)`)
      }
      return { cmd: { ...base, att }, mem, events: ev }
    }
    case "deorbitBurn": {
      const att = { x: scale(normalize(n.v), -1), z: n.up }
      const dv = mem.burnDv + n.accThrust * dt
      const tail = n.accThrust * 0.45
      const on = dv + tail < ctx.dvBurn
      mem = { ...mem, burnDv: dv }
      if (!on && n.accThrust < 0.05 && tp > 3) {
        mem = to(mem, "entryAlign", n.t)
        ev.push(`deorbit burn complete, Δv ${dv.toFixed(1)} m/s; coil ramp started`)
      }
      return { cmd: { ...base, att, gains: GAINS.vacuum, engines: [{ on, throttle: 0.6, gp: 0, gy: 0 }, ENGINES_OFF[1], ENGINES_OFF[2]] }, mem, events: ev }
    }
    case "entryAlign": {
      const nf = noseFor(n.vAir, n.up, 0, 0)
      if (n.alt < 120e3) {
        mem = to(mem, "entry", n.t)
        ev.push("entry interface (120 km)")
      }
      return { cmd: { ...base, att: { x: nf.x, z: nf.z }, coil: n.t >= ctx.tRamp ? "ramp" : "hold" }, mem, events: ev }
    }
    case "entry":
    case "loft": {
      if (NOSE_FIRST) {
        // Nose-first (alpha = 0): body x along the air-relative velocity, no lift, no high-alpha
        // loft or belly lift. The coil/MHD may trim/steer. At coil-authority loss the three aft
        // flaps are commanded (the flap worker owns the aero/thermal force; until then it is a
        // neutral command) and the nose-first fall continues to the low calculated flip gate.
        const nd = normalize(n.vAir)
        const att = { x: nd, spin: 0 }
        // do not evaluate coil authority before the plasma exists (q=0 at the entry interface would
        // otherwise discharge the coil immediately); hold/ramp until the plasma phase, then Q25.
        const cc = plasma ? coilCommand(ctx, n, mem, ev, att, GAINS.aero, mhdUsable) : { coil: (n.t >= ctx.tRamp ? "ramp" : "hold") as Cmd["coil"], mem }
        mem = cc.mem
        if (mem.coilOff) {
          mem = to(mem, "flip", n.t)
          ev.push(`one-engine coil-loss flip: h ${(n.alt / 1000).toFixed(1)} km, M ${n.mach.toFixed(1)}, q ${(n.qbar / 1000).toFixed(1)} kPa, single SL gimbal, no cold gas`)
        }
        const hIgn = ctx.armHeight - ctx.vehicle.pinX + flipAltitude(n.velRel.z, T)
        if (mem.phase !== "flip" && n.hBase < hIgn) {
          mem = to(mem, "flip", n.t)
          ev.push(`nose-first descent: flip at ${n.hBase.toFixed(0)} m, v_z ${n.velRel.z.toFixed(1)} m/s (alpha = 0, low calculated gate)`)
        }
        return {
          cmd: { ...base, att, gains: GAINS.aero, useMhd: mhdUsable && !mem.coilOff, seeding: mhdUsable && n.seedLeft > 0, coil: cc.coil, alphaCmd: 0, flapsDeployed: false },
          mem, events: ev,
        }
      }
      const alpha = alphaNoseFirst(n.V)
      // online MHD drag estimate (τ = 6 s) so the predictor sees the real energy dissipation
      const tab = mhdDragLookup(ctx.pred.mhdDrag, n.alt, n.V) * n.qbar * ctx.pred.sref
      if (tab > 2e4 && n.mhdDrag > 0) mem = { ...mem, mhdScale: clamp(mem.mhdScale + (dt / 6) * (n.mhdDrag / tab - mem.mhdScale), 0.3, 6) }
      // numerical predictor–corrector on the bank magnitude (every 8 s)
      if (mem.phase === "entry" && n.t - mem.npcT > 10 && n.V > ctx.pred.vLoft + 150) {
        const sol = solveBank(ctx.pred, predInput(n, mem, "entry"), "entry", mem.bank, T.aimEntry)
        mem = { ...mem, bank: sol.bank, pred: sol.out, npcT: n.t }
      }
      // lateral: bank reversals when the crossrange leaves a deadband shrinking with range to go
      const { cr, range } = crossrange(n)
      if (mem.phase === "entry" && Math.abs(cr) > crDeadband(range) && Math.sign(cr) === mem.bankSign) {
        mem = { ...mem, bankSign: -mem.bankSign }
        ev.push(`bank reversal → ${mem.bankSign > 0 ? "right" : "left"} (crossrange ${(cr / 1000).toFixed(1)} km of ${(range / 1000).toFixed(0)} km)`)
      }
      // Descent profile (user 2026-11-21): the loft keeps the entry bank only up to the lift-up that the
      // range/heat needs. A fully monotonic descent is not available: the entry's α ≈ 10° lift exceeds
      // the centrifugal-relieved gravity, and forcing cos σ ≤ G/a_L needs |σ| ≥ 46° at 50 km, which
      // drove the transition q from 14 to 64–90 kPa (104 kPa / 21 g peak) and lost the mission. The
      // measured minimum hold is the entry's shallow lift-up (49.8 → 52.3 km); see the report.
      const bank = mem.phase === "loft" ? 0 : mem.bank * mem.bankSign
      const nf = noseFor(n.vAir, n.up, alpha, bank)
      if (mem.phase === "entry" && n.V < ctx.pred.vLoft) {
        mem = to(mem, "loft", n.t)
        ev.push(`loft: lift vector up to lower q before pitch-up (q ${(n.qbar / 1e3).toFixed(1)} kPa)`)
      }
      // running q minimum since the loft began ("q rising" = 300 Pa above it), as in the predictor
      const pitchNow = mem.phase === "loft" && pitchUpDue(n.qbar, mem.qPrev + 300, n.V, ctx.pred)
      mem = { ...mem, qPrev: mem.phase === "loft" ? Math.min(mem.qPrev, n.qbar) : 1e9 }
      if (pitchNow) {
        // At trim α the lift is ≈ 3× what an equilibrium glide needs (C_L 2.3 vs 0.7 nose-first): pitching
        // up with the lift vertical zooms the ship ≈ 13 km (T11). So the high-α |σ₀| is solved now (same
        // damped law and predictor as highAlpha, downrange only; t_rev follows in highAlpha) and the
        // pitch-up rotates the nose straight into that banked plane while the coil still has authority.
        const sol = solveBank(ctx.pred, { ...predInput(n, mem, "highAlpha"), hEnd: hFlipPred(ctx, T) }, "high", mem.bankHigh, T.aimHigh)
        mem = to(mem, "pitchUp", n.t, { bankHigh: sol.bank })
        ev.push(`pitch-up at V ${(n.V / 1000).toFixed(2)} km/s, q ${(n.qbar / 1e3).toFixed(1)} kPa, into bank |σ₀| ${(sol.bank / DEG).toFixed(0)}° (predicted downrange ${(sol.out.downrangeErr / 1e3).toFixed(1)} km)`)
      }
      // bandwidth ≈ 2.5 × the unstable aero mode ω_u = √(|∂M/∂α|/I), |∂M/∂α| ≈ 0.45·q·S·L
      const wu = Math.sqrt((0.45 * n.qbar * 63.6 * 52.1) / n.I.y)
      const gains: Gains = rollGains({ ...GAINS.mhd, wn: clamp(2.5 * wu, 0.3, GAINS.mhd.wn) }, T.rollBandEntryDeg, T.rollWnEntry)
      return {
        // Monotonic descent / tiled hull (Q23, Q26): the 4°/s barbecue roll was for bare steel. With
        // tiles it is not needed, so the roll-rate target is 0 and the existing roll deadband
        // (`rollBand` 1°/s in GAINS.mhd/aero) gives short cold-gas pulses to damp the roll, then holds
        // without continuous gas. The coil keeps pitch/yaw.
        cmd: { ...base, att: { x: nf.x, spin: 0 }, gains, feedForward: mhdUsable || n.qbar > 50, useMhd: mhdUsable, seeding: mhdUsable && n.seedLeft > 0, coil: "ramp", alphaCmd: alpha },
        mem, events: ev,
      }
    }
    case "pitchUp": {
      const at = passiveTrim(ctx.aero, n.mach)
      // rotate into the plane of the damped high-α bank (σ₀ solved at the pitch-up decision)
      const nf = noseFor(n.vAir, n.up, at, mem.bankSign * dampedBank(mem.bankHigh, glideOf(ctx, n), ctx.pred.sref, T.glideZeta))
      if ((Math.abs(n.alpha - at) < 10 * DEG && Math.hypot(n.w.y, n.w.z) < 3 * DEG) || tp > 40) {
        mem = to(mem, "highAlpha", n.t, { npcT: -1e9 })
        ev.push(`aerodynamic trim reached: α ${(n.alpha / DEG).toFixed(1)}° (trim ${(at / DEG).toFixed(1)}°)`)
      }
      return {
        cmd: { ...base, att: { x: nf.x, z: nf.z }, gains: rollGains(GAINS.pitchUp, T.rollBandPitchDeg, T.rollWnPitch), feedForward: false, useMhd: mhdUsable, seeding: mhdUsable && n.seedLeft > 0, coil: "hold", alphaCmd: at },
        mem, events: ev,
      }
    }
    case "highAlpha": {
      // passive aerodynamic trim owns α; guidance flies only the lift direction (signed bank σ)
      const pb = planBank(ctx, n, mem, dt)
      mem = pb.mem
      if (pb.event) ev.push(pb.event)
      // Q25 coil-authority hand-over: when the flight-software estimate says the plasma can no longer
      // hold/steer the ship, flip immediately so the engine gimbal takes over before a tumble. This is
      // the flip trigger now — not the low-altitude landing gate. The ship keeps falling under engine
      // control from this high altitude down to the arms; the belly-flop phase is bypassed.
      if (mem.coilOff) {
        mem = to(mem, "bellyFlop", n.t, { heading: null, tSwitch: Infinity, npcT: -1e9, liftCmd: null })
        ev.push(`belly flop at coil loss: h ${(n.alt / 1000).toFixed(1)} km, M ${n.mach.toFixed(1)}, q ${(n.qbar / 1000).toFixed(1)} kPa, coil ${(n.coilI / 1000).toFixed(1)} kA; cold-gas heading, low calculated flip gate`)
      }
      const nf = noseFor(n.vAir, n.up, n.alpha, pb.sigma)
      const att = { x: nf.x, spin: 0, free: alphaAxis(n), bankAxis: normalize(n.vAir) }
      // Q25: keep the coil until its steering authority is gone (3 s hysteresis), then ramp down
      const cc = coilCommand(ctx, n, mem, ev, att, GAINS.aero, mhdUsable)
      mem = cc.mem
      const mhdHigh = mhdUsable && !mem.coilOff
      return {
        cmd: {
          ...base, att, gains: GAINS.aero, useMhd: mhdHigh,
          seeding: mhdHigh && n.seedLeft > 0, coil: cc.coil, alphaCmd: n.alpha,
        },
        mem, events: ev,
      }
    }
    case "bellyFlop": {
      // shallow: the same planned-reversal bank law (belly-flop trim, cold-gas yaw rate);
      // subsonic / steep: the ground-fixed lift-azimuth law. Below the belly-flop start (M < 2) the
      // flow is subsonic, the drag is high and v̂ is swinging toward vertical, where a bank about v̂
      // has no well-defined "up": the lift azimuth in the ground frame is the physical control, and
      // the two-segment plan can null the full 2-D miss while the bank-about-v̂ law cannot.
      const vhB = normalize(n.vAir)
      const steep = mem.hdgCmd !== null || n.mach < 2.0 || dot(vhB, n.up) < -Math.sin(60 * DEG)
      let xCmd: V3
      let zCmd: V3 | undefined
      if (steep) {
        if (mem.hdgCmd === null) ev.push("belly flop: steep descent — lift azimuth control")
        const sg = steepGlide(ctx, n, mem, dt)
        mem = sg.mem
        xCmd = sg.x
        zCmd = sg.z
      } else {
        const pb = planBank(ctx, n, mem, dt, "belly")
        mem = pb.mem
        if (pb.event) ev.push(pb.event.replace("high-α", "belly-flop"))
        xCmd = noseFor(n.vAir, n.up, n.alpha, pb.sigma).x
      }
      const hIgn = ctx.armHeight - ctx.vehicle.pinX + flipAltitude(n.velRel.z, T)
      const vz = n.velRel.z
      // Q25: the flip follows the switch-off — no engine ignition while the coil is still the actuator.
      // The altitude gate stays because the three-engine tail cannot fly from Mach 2 (~15 km): the
      // belly flop must bring the ship to the ignition height first.
      if (n.hBase < hIgn && mem.coilOff) {
        mem = to(mem, "flip", n.t)
        ev.push(`landing flip: ${T.landingEngines} × Raptor ignition at ${n.hBase.toFixed(0)} m, v_z ${vz.toFixed(1)} m/s, predicted miss ${(mem.bellyMiss).toFixed(0)} m`)
      }
      const vh = normalize(n.vAir)
      const att = zCmd ? { x: xCmd, z: zCmd, free: alphaAxis(n), bankAxis: vh } : { x: xCmd, spin: 0, free: alphaAxis(n), bankAxis: vh }
      return { cmd: { ...base, att, gains: rollGains(GAINS.belly, T.rollBandBellyDeg, T.rollWnBelly), coil: mem.coilOff ? "discharge" : "hold" }, mem, events: ev }
    }
    case "flip":
    case "landing":
    case "terminal": {
      // R11c / Q29 / Q33 (T09): from flip ignition to the pins the sea-level Raptors are the only
      // actuators — no cold gas, no coil. Two Raptors at the 40 % floor give T/W > 1.3, so there is no
      // hover: the vertical channel is a timed deceleration to the rails and the lateral divert is an
      // MPC with the same free final time (landing.ts).
      const eng0 = v.engines[0]
      const tMaxN = fVac100(eng0) - n.pAmb * eng0.exitArea
      const tMinN = eng0.minThrottle * fVac100(eng0) - n.pAmb * eng0.exitArea
      const rel = n.pinRel
      const pz = rel.z
      const dHoriz = Math.hypot(rel.x, rel.y)
      const vHoriz = Math.hypot(n.velRel.x, n.velRel.y)
      const flipping = mem.phase === "flip"
      // Staged burn (propulsion audit + one-engine amendment): the flip keeps 2 engines for rotation
      // authority; after the flip ONE SL Raptor flies the falling/steering phase with the full
      // 40–100 % throttle (one engine at 100 % is 2.45 MN, T/W 1.83, net +8.1 m/s²). It brakes at high
      // throttle while the speed is high, then throttles down to track the two-engine vertical profile
      // as the sink is nulled. The second engine lights only for the final brake/roll near the ground.
      const stage2Alt = 150
      const nEng: number = flipping ? (NOSE_FIRST ? 1 : 2) : pz <= stage2Alt || -n.velRel.z <= 32 ? 2 : 1
      const oneEng = !flipping && nEng === 1
      // thrust-direction corridor: wide while the divert is flown, narrowing into a vertical segment
      // the *vertical* floor (2 × 40 % → 4–7 m/s² net) is above the profile deceleration near the
      // ground, so the ship must keep a tilt all the way down to shed it; the cone stays wide and
      // only closes for the last few metres (the hull lean at the pins stays ≤ ~10°)
      // tilt corridor ramps down continuously so the TVC can slew from the ≈45° divert attitude to
      // upright: ≈45° at the divert altitude, ≈28° at 40 m, ≈12° at the pins
      const tiltDeg = tiltDegFor(pz, T)
      const lim = { mass: n.m, nEng, thrustMax: tMaxN, thrustMin: tMinN, tanTilt: Math.tan(tiltDeg * DEG) }
      const terminal = mem.phase === "terminal"
      // vertical-thrust trim: close the loop on the *achieved* vertical acceleration (nav: engine
      // thrust magnitude + body attitude — both flight-software data). When the body lags the
      // commanded tilt its vertical component is too large, the descent over-decelerates and the ship
      // stalls above the pins; the trim commands less vertical thrust, i.e. a larger tilt demand.
      // Measured under-tilt feedforward: the body lags the commanded thrust direction, so the achieved
      // vertical component is larger than commanded and the descent over-decelerates (the 47 m stall).
      // The body tilt is nav data (attitude), the previous commanded tilt is our own command. Command
      // the tilt *larger* by the measured lag: u_z ← u_z·cos(θ_target)/cos(θ_target + |lag|), i.e. a
      // trim < 1. The engine floor forces the magnitude; the surplus leaves laterally (`assemble`).
      const bX = thrDirBody(n)
      const uzAch = n.accThrust * dot(bX, n.up)
      const bodyTilt = Math.acos(clamp(dot(bX, n.up), -1, 1))
      const cmdTilt = Math.atan2(Math.hypot(mem.uCmd[0], mem.uCmd[1]), Math.max(mem.uCmd[2], 0.5))
      const magMin = (nEng * tMinN) / n.m
      const tgtVert = Math.min((mem.aProf > 0.2 ? mem.aProf : 4) + 9.79, 0.98 * magMin)
      const tgtTilt = Math.acos(clamp(tgtVert / Math.max(magMin, tgtVert), 0, 1))
      const lag = bodyTilt - cmdTilt // < 0: the body is less tilted than commanded
      const ffTrim = Math.cos(clamp(tgtTilt - lag, 0.05, 1.35)) / Math.max(Math.cos(tgtTilt), 0.2)
      const fbTrim = 0.5 + 0.5 * mem.uCmd[2] / Math.max(uzAch, 2)
      const uzTrim = flipping ? 1 : clamp(0.5 * ffTrim + 0.5 * fbTrim, T.landingUzTrimMin, T.landingUzTrimMax)
      // plan at 4 Hz; inside the terminal altitude the analytic law runs every control step
      // 10 Hz receding horizon (the MPC solve is a few ms; the 0.02 s plant step gets a fresh plan
      // every 0.1 s and the same plan between)
      const replan = terminal || mem.mpcU === null || n.t - mem.mpcT >= 0.1
      let uCmd: readonly [number, number, number] = mem.uCmd
      let tgo = 0
      let feasible = true
      let isTerminal = terminal
      if (replan) {
        const bXe = thrDirBody(n)
        const dNow = [dot(bXe, n.east), dot(bXe, n.north), dot(bXe, n.up)] as const
        // the divert MPC may aim off the catch point by `landingAimE/N` to pre-compensate a measured
        // systematic tracking drift; the terminal law and the arm/catch gates always use the true pins
        const aimE = T.landingAimE, aimN = T.landingAimN
        const st = { p: [rel.x - aimE, rel.y - aimN, pz] as const, v: [n.velRel.x, n.velRel.y, n.velRel.z] as const, a: aNowEnu(n), d: dNow }
        if (terminal) {
          uCmd = terminalControl({ ...st, p: [rel.x, rel.y, pz], v: [n.velRel.x, n.velRel.y, n.velRel.z] }, lim, T.contactSpeed)
          // Real predicted terminal path: a point-mass rollout of the same terminal law, not a
          // synthetic "looks right" line. The overlay's pale line must be the plan the controller
          // actually flies; if it misses the arms, the line shows that too.
          let pp = v3(rel.x, rel.y, pz)
          let vv = v3(n.velRel.x, n.velRel.y, n.velRel.z)
          const termPath: V3[] = []
          for (let k = 0; k < 30; k++) {
            const uc = terminalControl({ p: [pp.x, pp.y, pp.z] as const, v: [vv.x, vv.y, vv.z] as const, a: [0, 0, 0] as const, d: [0, 0, 1] as const }, lim, T.contactSpeed)
            vv = add(vv, scale(sub(v3(uc[0], uc[1], uc[2]), v3(0, 0, 9.79)), 0.1))
            pp = add(pp, scale(vv, 0.1))
            termPath.push(v3(pp.x, pp.y, Math.max(pp.z, 0) + ctx.armHeight))
            if (pp.z <= 0) break
          }
          mem = { ...mem, predPath: termPath }
        } else {
          const sol = planLanding(st, lim, {
            vF: T.contactSpeed,
            tau: flipping ? T.landingTauFlip : pz > 120 ? T.landingTauHigh : T.landingTauLow,
            glide: T.landingGlide, terminalAlt: T.landingTerminalAlt, uzTrim: nEng === 2 ? uzTrim : 1, aCruise: mem.aProf, zFinal: mem.zProf,
            warm: mem.mpcU, wMag: T.landingWMag, omegaMax: T.landingOmegaMax, slewRate: T.landingSlewRate, warm6: mem.mpcU, use6: false, mode: "twoPhase",
            field: flipping ? false : { ...LATERAL_FIELD_DEFAULT, zHand: T.terminalSwitchAlt }, flipping, profileEngines: 2,
            weights: {
              qp: T.landingQp, qv: T.landingQv, wvXY: T.landingQvXY, ru: T.landingRu, rd: T.landingRd,
              Qp: T.landingTermQp, Qv: T.landingTermQv,
            },
          })
          uCmd = sol.u
          tgo = sol.tgo
          feasible = sol.feasible
          isTerminal = sol.terminal
          if (sol.u6) mem = { ...mem, mpcU: sol.u6, mpcT: n.t }
          else if (sol.plan) mem = { ...mem, mpcU: sol.plan.u, mpcT: n.t }
          // visual export: the plan's predicted positions, pad ENU from the ground (U += arm height)
          if (sol.path && sol.path.length > 1) mem = { ...mem, predPath: sol.path.map((q) => v3(q[0], q[1], q[2] + ctx.armHeight)) }
        }
        mem = { ...mem, uCmd: [uCmd[0], uCmd[1], uCmd[2]] }
        // steering-rate feed-forward: the commanded direction's rotation rate between consecutive
        // plans, led by the attitude loop's time constant (pure command feed-forward, no error feedback)
        const dirNow = normalize(enuToEci(n, shapeThrust(v3(uCmd[0], uCmd[1], uCmd[2]), tiltDeg * DEG, 0)))
        const dtp = n.t - mem.uDirT
        if (dtp > 0.02 && dtp < 0.6) {
          const leadNew = scale(sub(dirNow, mem.uDirPrev), 0.6 / dtp)
          const ln = norm(leadNew)
          mem = { ...mem, leadF: ln > 0.24 ? scale(leadNew, 0.24 / ln) : leadNew }
        }
        mem = { ...mem, uDirPrev: dirNow, uDirT: n.t }
      }
      // final safety clamp: the landing law already honours the engine magnitude floor (surplus spent
      // laterally) and the cone; do not re-impose the floor here by scaling — that would inflate the
      // vertical and stall the descent.
      const shaped = shapeThrust(v3(uCmd[0], uCmd[1], uCmd[2]), tiltDeg * DEG, 0)
      const thrDir = normalize(enuToEci(n, shaped))
      // small measured-lag lead (0.3, capped at 6°): the larger leads (0.9 and the command-rate
      // feed-forward) destabilised the attitude (73° overshoot / 139 m miss), so the tilt lag is
      // left to the vertical trim and reported rather than over-driven.
      const bXn = thrDirBody(n)
      const eDir = sub(thrDir, bXn)
      const lagMag = norm(eDir)
      const leadK = lagMag > 1e-4 ? Math.min(0.3, (6 * DEG) / lagMag) : 0
      // Command shaper: never ask the body to slew faster than it can. The plan's thrust direction
      // can jump ~38°/s at a replan; the landing loop's rate limit is ~26°/s, so the body falls
      // behind and the flown path (blue) diverges from the plan (white). Rotate the attitude target
      // toward the previous command at the achievable rate.
      const attRaw = normalize(add(thrDir, scale(eDir, leadK)))
      const dtc = clamp(n.t - mem.uDirT, 0.02, 0.25)
      const maxAng = clamp(T.landingWMaxDeg * DEG * 0.8, 4 * DEG, 26 * DEG) * dtc
      const cosA = clamp(dot(attRaw, mem.uDirPrev), -1, 1)
      const angA = Math.acos(cosA)
      const attX = angA > maxAng && angA > 1e-6
        ? normalize(add(scale(attRaw, Math.sin(maxAng) / Math.sin(angA)), scale(mem.uDirPrev, Math.sin(angA - maxAng) / Math.sin(angA))))
        : attRaw
      const throttle = throttleFor(v, nEng, n.m * norm(shaped), n.pAmb)
      // roll: the pins are symmetric (body ±y over the east/west rails), so z → north or z → south,
      // whichever is closer (≤ 90° of roll); two symmetric engines roll by differential gimbal.
      const qq = n.q
      const zb = v3(2 * (qq.x * qq.z + qq.w * qq.y), 2 * (qq.y * qq.z - qq.w * qq.x), 1 - 2 * (qq.x * qq.x + qq.y * qq.y))
      // roll is free until the last few hundred metres (the hull is axisymmetric): demanding a roll
      // angle during the divert makes the minimum-norm gimbal allocator share its authority between
      // roll and pitch/yaw, and the measured under-tilt was 10–15° (which over-decelerated the
      // vertical). Damp the roll rate instead; align the pins east–west late.
      // Roll the pins east–west early and gently. Commanding it at 250 m made the roll loop spin the
      // body at >100°/s into the terminal and flip it; 450 m leaves ~15 s at 10°/s to settle, with
      // the priority allocator keeping pitch/yaw.
      const att = pz > 450
        ? { x: attX, spin: 0 }
        : { x: attX, z: dot(zb, n.north) >= 0 ? n.north : scale(n.north, -1) }
      // Roll RCS (one-engine phase): a single gimbal has no useful roll authority; the cold-gas roll
      // couples fire only outside the roll deadband, while pitch/yaw stay on the gimbal (the finite
      // `rollCap` path in allocateGimbals keeps the gimbal from spinning the hull). No translation.
      const rollTgt = "z" in att && att.z ? att.z : null
      const bxRoll = normalize(thrDirBody(n))
      let rollErr = 0
      if (rollTgt) {
        const zp = sub(rollTgt, scale(bxRoll, dot(rollTgt, bxRoll)))
        if (norm(zp) > 1e-6) {
          const zpn = normalize(zp)
          rollErr = Math.atan2(dot(cross(zb, zpn), bxRoll), dot(zb, zpn))
        }
      }
      const rollPulse = oneEng && (Math.abs(rollErr) > 4 * DEG || Math.abs(n.w.x) > 3 * DEG)
      if (rollPulse && n.t - mem.rcsT > 2) {
        mem = { ...mem, rcsT: n.t }
        ev.push(`roll RCS pulse: roll ${(rollErr / DEG).toFixed(1)}° error, rate ${(n.w.x / DEG).toFixed(1)}°/s (pitch/yaw on the gimbal)`)
      }
      const cmdEngines = enginesOn(nEng, throttle)
      if (flipping) {
        const pointErr = Math.acos(clamp(dot(normalize(thrDirBody(n)), thrDir), -1, 1))
        // the flip ends when the thrust axis follows the command: three engines can stop any residual
        // rate; after that two (R11c) fly the hoverslam. Timeout 10 s for a stuck rotation.
        if (n.spoolDone && tp > 2 && ((pointErr < 22 * DEG && Math.hypot(n.w.y, n.w.z) < 5 * DEG) || tp > 9)) {
          // fix the vertical profile deceleration here, once: recomputing it from a state that has
          // already over-decelerated makes the required decel shrink, the tilt demand grow and the
          // descent collapse (the 47 m stall). The profile must be monotone to z = 0.
          // two-phase vertical profile: cruise (divert) deceleration + the altitude where the
          // near-vertical final descent starts. zFinal follows from the continuity of the profile:
          //   s² = vF² + 2·aBrake·zF + 2·aCruise·(pz − zF)
          // Distinct phases: a lateral-null cruise at the altitude-matched deceleration down to zProf,
          // then an upright brake at the floor's own vertical deceleration aBrake. zProf follows from
          // the profile continuity s² = vF² + 2·aBrake·zProf + 2·aCruise·(pz − zProf).
          const sHand = Math.max(-n.velRel.z, 0.7)
          const magMin = (T.landingEngines * tMinN) / n.m
          const aBrake = Math.max(magMin - 9.79, 0.6)
          const aCruise = clamp((sHand * sHand - T.contactSpeed * T.contactSpeed) / (2 * Math.max(pz, 1)), T.landingACruiseMin, T.landingACruiseMax)
          const zFin = (sHand * sHand - T.contactSpeed * T.contactSpeed - 2 * aCruise * pz) / (2 * Math.max(aBrake - aCruise, 0.2))
          const zProf = clamp(zFin, T.landingZProfMin, T.landingZProfMax)
        mem = to(mem, "landing", n.t, { nEngLand: 1, mpcU: null, aProf: aCruise, zProf })
          ev.push(`flip complete: staged burn — 1 × Raptor SL high-throttle falling/steering phase (roll-only RCS), cruise ${aCruise.toFixed(2)} m/s² to ${zProf.toFixed(0)} m, 1→2 at 150 m for the final brake/roll${feasible ? "" : " — divert INFEASIBLE with the throttle floor"}${tgo > 0 ? `, plan t_go ${tgo.toFixed(1)} s` : ""}`)
        }
      }
      // Approach direction: the terminal law is only entered from the south side and while not
      // moving away from the arms. The ship must come from above/south, never from the tower side.
      if (mem.phase === "landing" && (isTerminal || pz <= T.terminalSwitchAlt) && dHoriz < T.terminalSwitchLat && vHoriz < T.terminalSwitchV && rel.y < 5 && n.velRel.y > -0.5) {
        mem = to(mem, "terminal", n.t, { mpcU: null })
        ev.push(`terminal descent: pins ${pz.toFixed(1)} m above the rails at ${(-n.velRel.z).toFixed(1)} m/s, offset ${dHoriz.toFixed(2)} m${tgo > 0 ? `, t_go ${tgo.toFixed(1)} s` : ""}`)
      }
      let arms = mem.armsClose
      // 3 s arm travel: close before the fast last metres (the hoverslam cannot slow down to hover)
      // the ship enters the arms *sideways*: pins (body y) E–W, belly (body z) N–S, moving along the
      // arms (world y, the open south side). Close only when the across-arm offset and velocity are small.
      // Close on *time to the rails*, not a fixed altitude: the arms take 3 s, so start at
      // `armCloseLead` (> 3 s) and finish just before the pins touch. The geometry is checked at the
      // predicted closure position, so horizontal drift during the close is accounted for.
      const tRail = pz / Math.max(-n.velRel.z, 0.4)
      const lead = Math.max(TOWER.closeTime + 0.2, T.armCloseLead)
      const xClose = rel.x + n.velRel.x * lead
      const yClose = rel.y + n.velRel.y * lead
      if (mem.phase !== "flip" && !arms && tRail <= lead && Math.abs(xClose) < T.armCloseLat && Math.abs(yClose) < T.armCloseAlong && Math.abs(n.velRel.x) < T.armCloseAcrossV && vHoriz < T.armCloseV && n.velRel.y > -0.3) {
        arms = true
        mem = { ...mem, armsClose: true }
        ev.push(`chopsticks closing: pins ${pz.toFixed(1)} m above the rails (${tRail.toFixed(1)} s), predicted offset at shut ${Math.hypot(xClose, yClose).toFixed(2)} m (across ${xClose.toFixed(2)})`)
      }
      // A catch requires the arms to be *closed* around the ship: with the arms open their rails are
      // at ±13 m, and a single pin grazing one of them registers a load while the ship is nowhere near
      // the centreline. That produced false "caught" outcomes (12.8 m off, arms open, 21 m/s lateral).
      // The arms are only commanded closed once the offset is < 2 m, so gate the catch on that command.
      // A catch requires the pins to settle onto the rails with the ship moving *north along the
      // arms* (+y) and not drifting across them. A north-to-south or sideways arrival is not a catch.
      // Both pins must carry load individually: the total alone (or one pin grazing an open rail)
      // produced false catches. Each pin ≥ 15 % of weight, and the total stays above the old 45 %
      // gate so a lightly touched pair is not a catch either.
      const pinWeight = n.m * 9.81
      const bothPins = bothPinsLoaded(n.pinCount, n.pinLoadMin, 0.15 * pinWeight) && n.pinLoad > 0.45 * pinWeight
      if (mem.phase !== "caught" && mem.phase !== "lost" && mem.armsClose && Math.abs(rel.x) < 1.0 && n.velRel.y > 0 && Math.abs(n.velRel.x) < 0.8 && bothPins) {
        mem = to(mem, "caught", n.t, { caughtT: n.t })
        ev.push(`catch pins on the arms — engine shutdown at ${(-n.velRel.z).toFixed(2)} m/s, offset ${dHoriz.toFixed(2)} m. CAUGHT`)
      }
      const caught = mem.phase === "caught"
      // honest failure detection: with two Raptors at the 40 % floor the vertical cannot be re-started
      // once it over-decelerates; climbing away above the rail plane with a lateral miss is lost, and
      // reporting it now keeps the (finite) header from being burnt in a climb that cannot recover.
      let lostNow = false
      if (!caught && pz < 200 && n.velRel.z > 3) {
        lostNow = true
        mem = to(mem, "lost", n.t, { reason: "landing missed the arms (throttle floor: no hover)" })
        ev.push(`LOST: hoverslam cannot reach the pins — pins ${pz.toFixed(0)} m above the rails, ${dHoriz.toFixed(0)} m off, climbing at ${n.velRel.z.toFixed(1)} m/s (${nEng} × Raptor, ${oneEng ? "one-engine guided fall" : "2-engine floor cannot hover"})`)
      }
      if (n.propHeader < 50 && !caught && !lostNow) {
        if (mem.phase !== "lost") mem = to(mem, "lost", n.t, { reason: "landing propellant exhausted" })
        ev.push(`LOST: landing propellant exhausted at ${pz.toFixed(0)} m above the rails (${(-n.velRel.z).toFixed(0)} m/s)`)
      }
      return {
        // From flip ignition onward the engines are the thrust/pitch/yaw actuators. The one-engine
        // stage may use the cold-gas roll couples only (short roll pulses, no translation); pitch/yaw
        // remain on the gimbal. The 2-engine final uses differential-gimbal roll.
        cmd: {
          // feedForward cancels the *static* aero moment from guidance's own table (R6): at ≈ 100 m/s
          // base-first the hull is aerodynamically stable nose-up, and without the cancellation the
          // gimbals at the throttle floor cannot hold the ≈ 30–40° divert tilt (measured under-tilt
          // 10–15°, which over-decelerated the vertical and stalled the descent).
          ...base, att, feedForward: true,
          gains: flipping
            ? { ...GAINS.flip, wn: T.flipWn, zeta: T.flipZeta, wMax: T.flipWMaxDeg * DEG, wnRoll: T.flipWnRoll, rollMax: T.flipRollMaxDeg * DEG }
            : nEng === 3
              // three SL at the floor: the extra torque gives the initial divert 1.5× the attitude
              // authority of the two-engine law; the floor surplus is lateral (the vertical profile
              // is the two-engine one), so there is no 3-engine vertical stall.
              ? { ...GAINS.landing, wn: T.landingWn, zeta: T.landingZeta, wMax: T.landingWMaxDeg * DEG, wnRoll: T.landingWnRoll, rollMax: T.landingRollMaxDeg * DEG, axisAccel: [0.5, 0.44, 0.44] }
              : nEng === 1
              // one SL gimbal: pitch/yaw authority ≈ the 2-engine floor (same total thrust), roll via
              // the cold-gas couples; conservative rate caps keep the gimbal inside its box so the
              // RCS residual stays roll-only.
              ? { ...GAINS.landing, wn: T.landingWn, zeta: T.landingZeta, wMax: Math.min(T.landingWMaxDeg, 20) * DEG, wnRoll: 0.5, rollMax: 10 * DEG, axisAccel: [0.3, 0.22, 0.22] }
              : { ...GAINS.landing, wn: T.landingWn, zeta: T.landingZeta, wMax: T.landingWMaxDeg * DEG, wnRoll: T.landingWnRoll, rollMax: T.landingRollMaxDeg * DEG, axisAccel: [T.landingAxisAccelX, T.landingAxisAccelYZ, T.landingAxisAccelYZ] },
          engines: caught || lostNow ? ENGINES_OFF : cmdEngines, armsClose: arms, coil: "discharge", useRcs: oneEng, rcsRollOnly: oneEng, flapsDeployed: mem.flapsDeployed,
        },
        mem, events: ev,
      }
    }
    case "caught":
    case "lost":
      return { cmd: { ...base, att: { x: n.up, z: n.north }, useRcs: false, armsClose: mem.armsClose, coil: "discharge", flapsDeployed: mem.flapsDeployed }, mem, events: ev }
  }
}

/**
 * Flip ignition height above the catch height for a vertical speed v_z lives in `landing.ts`: the
 * gate is the altitude the two-engine hoverslam corridor needs (spool + rotation distance plus the
 * floor-thrust stopping distance), not an ad-hoc three-engine figure.
 */

/** belly flop: rotation about the near-vertical v̂ is mostly body yaw (≈ 0.0045 rad/s², scripts/authority.ts);
 *  the rate is the tuning field `bellyBankRateDeg` (default 1.8°/s). */
/**
 * No lift floor (−1): the flapless hull is axisymmetric and bare steel all round (PICA only on the
 * nose), so rolling the lift about v̂ — even past 90° — is aerodynamically and thermally equivalent.
 * A "belly must face down" rule belongs to the flapped, tiled real ship, not to this vehicle (G0).
 */
const LIFT_FLOOR = -1

/**
 * Steep-descent glide law (E21, last part of the belly flop, flight path steeper than −60°). Here the
 * bank reference (the horizontal air-velocity direction) rotates with the lift's own effect, so the
 * lift is flown by its ground-fixed azimuth ψ instead; rotating about the near-vertical v̂ is a
 * rotation about the vertical (BELLY_RATE). Candidates: ψ₁ for τ, then ψ₂ (fixed ψ ends on a ring
 * around the zero-lift point; two segments fill it). Point mass in the pad ENU frame, wind forecast
 * + measured local offset, stop at the flip altitude.
 */
/**
 * Catch alignment window above the flip altitude (m). Only the last ≈ 1.5 km needs the nose on the
 * braking azimuth for the flip/catch geometry; enforcing it any earlier throws away the lift authority
 * that is still needed to null the remaining position error (at 6 km out a quadrant snap is up to 45°
 * of steering error, which is kilometres). The window is the tuning field `bellyAlignH` (1800 m).
 */
/**
 * Pre-flip nose azimuth: the flip thrusts along the nose while it pitches up, a horizontal kick of
 * ≈ 35 m/s along the nose azimuth. Point it at the pad (nearest of N/E/S/W) so the kick starts the
 * divert; the roll carries the pin axis through the flip. This is also the hand-over the fast
 * evaluator/tests are calibrated on (a braking, east-pointing azimuth gave a larger belly miss).
 */
const quadrant = (psi: number): number => Math.round(psi / (Math.PI / 2)) * (Math.PI / 2)
const flipAzimuth = (px: number, py: number, psiNow: number): number => {
  const d = Math.hypot(px, py)
  const q = quadrant(d > 300 ? Math.atan2(-px, -py) : psiNow)
  return psiNow + Math.atan2(Math.sin(q - psiNow), Math.cos(q - psiNow))
}
/** The geometric inputs of the belly-flop point-mass law (shared with the fast evaluator). */
export interface BellyInput {
  readonly pinRel: V3
  readonly velRel: V3
  readonly hBase: number
  readonly mass: number
  /** measured wind minus forecast, ENU (fades with altitude from the start) */
  readonly windOfs: V3
  readonly aero: AeroTable
  readonly sref: number
  readonly windForecast: (h: number) => V3
}

/**
 * One point-mass belly-flop integration endpoint with the two-segment lift-azimuth law. This is the
 * flight's own `steepEnd` physics, exported so the fast evaluator runs exactly the same law (no
 * duplicated integrator). `windOfs` is zero for the predictive evaluator.
 */
export const bellyEnd = (b: BellyInput, psi1: number, tau: number, psi2: number, psi0: number, hEnd: number, rate: number, alignH: number, dtStep = 0.5): { x: number; y: number; h: number; vx: number; vy: number; vz: number; t: number } => {
  let px = b.pinRel.x, py = b.pinRel.y, h = b.hBase
  let vx = b.velRel.x, vy = b.velRel.y, vz = b.velRel.z
  const S = b.sref
  let psi = psi0, t = 0
  for (let i = 0; i < 2000 && h > hEnd; i++) {
    const w0 = b.windForecast(Math.max(h, 0))
    const kf = Math.exp(-Math.abs(h - b.hBase) / 3000)
    const w = v3(w0.x + b.windOfs.x * kf, w0.y + b.windOfs.y * kf, 0)
    const ax = vx - w.x, ay = vy - w.y, az = vz
    const V = Math.hypot(ax, ay, az)
    const dt = dtStep
    const tgt = h < hEnd + alignH ? flipAzimuth(px, py, psi) : t < tau ? psi1 : psi2
    psi += clamp(Math.atan2(Math.sin(tgt - psi), Math.cos(tgt - psi)), -rate * dt, rate * dt)
    const at = atmosphere(Math.max(h, 0))
    const mach = V / at.a
    const co = aeroLookup(b.aero, passiveTrim(b.aero, mach), mach)
    const q = 0.5 * at.rho * V * V
    const ux = ax / V, uy = ay / V, uz = az / V
    const hx = Math.sin(psi), hy = Math.cos(psi)
    const d = hx * ux + hy * uy
    let lx = hx - d * ux, ly = hy - d * uy, lz = -d * uz
    const ln = Math.hypot(lx, ly, lz) || 1
    lx /= ln; ly /= ln; lz /= ln
    const L = (q * S * co.CL) / b.mass, D = (q * S * co.CD) / b.mass
    vx += (-D * ux + L * lx) * dt
    vy += (-D * uy + L * ly) * dt
    vz += (-D * uz + L * lz - 9.79) * dt
    px += vx * dt; py += vy * dt; h += vz * dt
    t += dt
  }
  return { x: px, y: py, h, vx, vy, vz, t }
}

const steepEnd = (ctx: GuideCtx, n: Nav, mem: Mem, psi1: number, tau: number, psi2: number, psi0: number, hEnd: number, rate: number, alignH: number): { x: number; y: number } =>
  bellyEnd({ pinRel: n.pinRel, velRel: n.velRel, hBase: n.hBase, mass: n.m, windOfs: mem.windOfs, aero: ctx.aero, sref: ctx.pred.sref, windForecast: ctx.windForecast }, psi1, tau, psi2, psi0, hEnd, rate, alignH)

const steepGlide = (ctx: GuideCtx, n: Nav, mem0: Mem, dt: number): { x: V3; z: V3; mem: Mem } => {
  const T = ctx.tuning ?? MISSION_TUNING_DEFAULT
  const bellyRate = T.bellyBankRateDeg * DEG
  let mem = mem0
  // Prediction end: the *design* flip altitude, not the instantaneous-sink estimate. At the belly-flop
  // start the sink is ≈ −380 m/s, and flipAltitude(−380) is ≈ 4.6 km — but the vehicle is at that point
  // 40 s and ~1.3 km up-range of the pad, so the plan under-integrated the final descent and always
  // aimed ~1 km short. The actual gate still uses the instantaneous sink (`flipAltitude(n.velRel.z)`);
  // by the flip it is ≈ −107 m/s, i.e. within 50 m of flipAltitude(−100).
  const hFlip = hFlipPred(ctx, T)
  const vh = normalize(n.vAir)
  if (mem.hdgCmd === null) {
    // start from the azimuth of the lift actually flown
    const xb = thrDirBody(n)
    const lb = sub(xb, scale(vh, dot(xb, vh)))
    mem = { ...mem, hdgCmd: Math.atan2(dot(lb, n.east), dot(lb, n.north)), heading: null, tSwitch: Infinity, npcT: -1e9, liftCmd: null }
  }
  const hc = mem.hdgCmd ?? 0
  if (mem.heading !== null && n.t >= mem.tSwitch) mem = { ...mem, heading: mem.sigma2, tSwitch: Infinity }
  if (n.hBase > hFlip + 100 && n.t - mem.npcT > 2) {
    const D = Math.PI / 180
    interface Plan { readonly a: number; readonly tau: number; readonly b: number; readonly e: number }
    // The plan minimises the true flip miss (the T09 gate). The across-arms (E–W) component is the
    // landing's weak axis and is reported separately; weighting it in the objective traded 270 m of
    // north–south error for 100 m of east–west and made the total worse (measured 881 → 975 m).
    const miss = (a: number, tau: number, b: number): number => { const e = steepEnd(ctx, n, mem, a, tau, b, hc, hFlip, bellyRate, T.bellyAlignH); return Math.hypot(e.x - T.bellyAimE, e.y - T.bellyAimN) }
    const pick = (ps: readonly Plan[]): Plan => ps.reduce((x, y) => (y.e < x.e ? y : x))
    const heads = Array.from({ length: 12 }, (_, i) => hc + (i - 6) * 30 * D)
    let best = pick(heads.map((a) => ({ a, tau: Infinity, b: a, e: miss(a, Infinity, a) })))
    if (best.e > 40) {
      // T09 hand-over: the terminal descent is ~160 s, so a plan whose first segment is capped at 32 s
      // cannot express "brake the horizontal velocity early, then steer"; it crossed the pad at ~200 m/s
      // and stopped 4 km past it. The τ grid reaches the full descent, with ±45° steering steps.
      const taus = [8, 16, 32, 50, 70, 95, 125]
      const two = heads.flatMap((a) => [45, 90, 180, -45, -90].flatMap((db) => taus.map((tau) => ({ a, tau, b: a + db * D, e: miss(a, tau, a + db * D) }))))
      best = pick([best, ...two])
    }
    const b0 = best
    const fine = [[-8, 0, 1], [8, 0, 1], [0, -15, 1], [0, 15, 1], [0, 0, 0.7], [0, 0, 1.4]]
      .map(([da, dbb, k]) => ({ a: b0.a + da * D, b: b0.b + dbb * D, tau: b0.tau * k }))
      .map((x) => ({ ...x, e: miss(x.a, x.tau, x.b) }))
    best = pick([best, ...fine])
    const cur = mem.heading === null ? null : { a: mem.heading, tau: mem.tSwitch - n.t, b: mem.sigma2, e: miss(mem.heading, mem.tSwitch - n.t, mem.sigma2) }
    const tgt = cur && cur.e < best.e + 30 ? cur : best
    const eEnd = steepEnd(ctx, n, mem, tgt.a, tgt.tau, tgt.b, hc, hFlip, bellyRate, T.bellyAlignH)
    mem = { ...mem, heading: tgt.a, tSwitch: n.t + tgt.tau, sigma2: tgt.b, bellyMiss: Math.hypot(eEnd.x - T.bellyAimE, eEnd.y - T.bellyAimN), npcT: n.t }
    const dbg = (globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG
    if (dbg) dbg(`steep ${n.t.toFixed(0)} h ${n.hBase.toFixed(0)} ψc ${(hc / DEG).toFixed(0)} plan ψ ${(tgt.a / DEG).toFixed(0)} τ ${tgt.tau.toFixed(0)} ψ2 ${(tgt.b / DEG).toFixed(0)} miss ${tgt.e.toFixed(0)}`)
  }
  // catch alignment: the flip pitches about body y, so body z ends along the pre-flip nose azimuth.
  // In the last ≈ 1.5 km the nose swings to north or south (nearer one): after the flip the engine
  // side (+z) is N–S, the pins (±y) face the east/west arm rails, and the divert from the arms'
  // open (south) side uses the strong gimbal-pitch plane.
  const align = n.hBase < hFlip + T.bellyAlignH
  const tgtPsi = align ? flipAzimuth(n.pinRel.x, n.pinRel.y, hc) : mem.heading ?? hc
  const dPsi = Math.atan2(Math.sin(tgtPsi - hc), Math.cos(tgtPsi - hc))
  const hNew = hc + clamp(dPsi, -bellyRate * dt, bellyRate * dt)
  mem = { ...mem, hdgCmd: hNew }
  const dir = add(scale(n.east, Math.sin(hNew)), scale(n.north, Math.cos(hNew)))
  // Flare rejected on physics (T09): at the belly-flop trim (α ≈ 70°, C_L ≈ 1.26, C_D ≈ 3.6 at M 0.5)
  // the lift is ≈ 4 m/s² = 0.4 g at the terminal q ≈ 6.5 kPa, so a_L cos σ = g has no solution and the
  // lift cannot hold the descent. The sink is the drag/gravity terminal velocity (≈ 110 m/s) and the
  // sustainable horizontal drift is ≤ V·L/D ≈ 47 m/s; measured flare variant changed the miss by 4 m.
  const lift = normalize(sub(dir, scale(vh, dot(dir, vh))))
  const x = add(scale(vh, Math.cos(n.alpha)), scale(lift, Math.sin(n.alpha)))
  // roll target for the catch (the hull is axisymmetric: rolling about x is aerodynamically free).
  // The flip turns the nose up about the horizontal axis ⟂ the nose azimuth: an axis along that
  // hinge stays put, the vertical axis ends along the old nose azimuth. For body z to end N–S:
  // nose N/S → z vertical; nose E/W → z along the hinge. Sign: whichever is nearer the current z.
  const qq = n.q
  const zb = v3(2 * (qq.x * qq.z + qq.w * qq.y), 2 * (qq.y * qq.z - qq.w * qq.x), 1 - 2 * (qq.x * qq.x + qq.y * qq.y))
  // roll for the anticipated flip azimuth (from the start of the steep phase: roll is free here)
  const nsNose = Math.abs(Math.cos(flipAzimuth(n.pinRel.x, n.pinRel.y, hNew))) > 0.5
  const zRaw = nsNose ? sub(n.up, scale(x, dot(n.up, x))) : cross(n.up, x)
  const zT = normalize(zRaw)
  return { x, z: dot(zT, zb) >= 0 ? zT : scale(zT, -1), mem }
}

/**
 * Bank-slew rates about v̂ (rad/s). T08 P0-5: 1.5°/s is the *coil-era* value; once the coil is off the
 * lift is rotated by cold gas only, whose couples give 0.004 rad/s² about v̂ at high α — 2.5°/s peak,
 * ≈ 1.9°/s average over a 100° reversal (scripts/authority.ts). The bank law and the predictor must
 * slew at the rate the *current* actuator can deliver, or a reversal looks affordable when it is not.
 */
const SIGMA_RATE = (1.5 * Math.PI) / 180
const SIGMA_RATE_GAS = (1.9 * Math.PI) / 180
/**
 * High-α bank slew. The high-α descent guard needs |σ| ≈ 65–80° at the pitch-up hand-over (lift is
 * ~2–3 G); at the entry-era 1.5°/s that takes 20–30 s, during which the ship still zooms ~1 km. The
 * HTS coil has pitch/yaw authority at high-α (the attitude demand is small, see mhdAuth), so the
 * high-α bank may be slewed at 3°/s. This does not change the entry/loft bank rate.
 */
const SIGMA_RATE_HIGH = (3.0 * Math.PI) / 180
const REV_BIAS = 4000 // m, see solveRev
const REV_LEAD = 5 // s: the body starts rotating ≈ 5 s after the command (rate loop + accel limit), so reverse that early
const BANK_GRID = [0, 15, 30, 45, 60, 75, 85, 95, 110, 130, 155, 180].map((d) => (d * Math.PI) / 180)
/** crossrange bias the plan may leave for a later correction (m): 2 % of the range, capped */
const revBias = (range: number): number => clamp(0.02 * range, 50, REV_BIAS)

/**
 * High-α planned-reversal bank law (E18). Rotating the lift about v̂ is slow (cold-gas yaw
 * authority ≈ 0.005 rad/s², so a 100° reversal takes 40–50 s), so reversals are planned instead of
 * triggered by a deadband. Every few seconds, with the spherical 3-DOF predictor (signed bank that
 * slews at the current actuator rate, zero net lift below Mach 2 = belly-flop footprint centre):
 *   |σ|   nulls the predicted downrange, given one planned final reversal at t_rev;
 *   t_rev nulls the predicted crossrange (holding sign s pushes the end point to side s).
 * The ship reverses only when t_rev arrives. The commanded bank moves at the current actuator rate.
 *
 * T08 P0-5: a reversal that cannot be completed before the flip is worse than a small crossrange
 * error — the roll passes through lift-vertical at high q, and at ≈ 2 g that is a several-km range
 * excursion. So (i) the dwell between reversals is at least the time a full roll needs, and (ii) once
 * holding the current sign already puts the predicted end point inside the terminal footprint, the
 * plan is frozen (t_rev = ∞) instead of chasing a marginally better crossrange.
 */
const planBank = (ctx: GuideCtx, n: Nav, mem0: Mem, dt: number, phaseP: PredPhase = "highAlpha"): { sigma: number; mem: Mem; event?: string } => {
  const T = ctx.tuning ?? MISSION_TUNING_DEFAULT
  let mem = mem0
  const baseRate = phaseP === "belly" ? T.bellyBankRateDeg * DEG : mem.coilOff ? SIGMA_RATE_GAS : SIGMA_RATE
  const rate = phaseP === "highAlpha" ? Math.max(baseRate, SIGMA_RATE_HIGH) : baseRate
  const hFlip = hFlipPred(ctx, T)
  // measured local wind (ground − air velocity, ENU) → forecast correction, low-passed (τ 10 s), below 40 km
  if (n.hBase < 40e3) {
    const vAirEnu = v3(dot(n.vAir, n.east), dot(n.vAir, n.north), dot(n.vAir, n.up))
    const wMeas = sub(n.velRel, vAirEnu)
    const wF = ctx.windForecast(n.hBase)
    const kf = clamp(dt / 10, 0, 1)
    mem = { ...mem, windOfs: add(scale(mem.windOfs, 1 - kf), scale(v3(wMeas.x - wF.x, wMeas.y - wF.y, 0), kf)) }
  }
  let event: string | undefined
  // bank actually flown (body x component ⟂ v̂): predictions start from it, so the attitude lag
  // behind the command (≈ 5 s) is not mistaken for reversal timing (≈ 3.5 km of crossrange per s)
  const vhF = normalize(n.vAir)
  const upF = normalize(sub(n.up, scale(vhF, dot(n.up, vhF))))
  const xbF = thrDirBody(n)
  const lbF = sub(xbF, scale(vhF, dot(xbF, vhF)))
  const sFlown = Math.atan2(dot(lbF, cross(vhF, upF)), dot(lbF, upF))
  if (mem.sigmaCmd === null) {
    mem = { ...mem, sigmaCmd: sFlown, sigmaTgt: sFlown, bankSign: sFlown >= 0 ? 1 : -1, bankHigh: Math.abs(sFlown), npcT: -1e9, tRevPlan: Infinity }
  }
  const sc = mem.sigmaCmd ?? 0
  const range = crossrange(n).range
  const revSoon = Number.isFinite(mem.tRevPlan) && mem.tRevPlan - n.t < 25
  const period = revSoon ? 1 : phaseP === "belly" ? 2 : range > 500e3 ? 10 : range > 150e3 ? 6 : 4
  const uzNow = Math.sqrt(Math.max(0, 1 - dot(vhF, n.up) ** 2)) // vertical component of the in-plane "up"
  const grid = BANK_GRID.filter((m) => uzNow * Math.cos(m) >= LIFT_FLOOR)
  if (n.hBase > hFlip + 150 && n.t - mem.npcT > period) {
    const s = mem.bankSign
    const base: PredInput = { ...predInput(n, mem, phaseP), bankSign: s, sigma0: sFlown, sigmaRate: rate, hEnd: hFlip, liftFloor: LIFT_FLOOR, windOfs: mem.windOfs }
    const run = (mag: number, tRev: number): PredOut => predict(ctx.pred, { ...base, bankHigh: mag, tRev })
    let tRev = Math.max(mem.tRevPlan, n.t)
    let mag = mem.bankHigh
    // |σ| for downrange. The map is not monotone with a planned reversal (a larger |σ| is a longer
    // slew through lift-vertical): scan, take the crossing nearest the current |σ|, refine by secant.
    const solveMag = (tR: number): number => {
      const dr = (m: number): number => run(m, tR).downrangeErr - T.aimHigh
      const fg = grid.map(dr)
      let cell = -1
      for (let k = 0; k + 1 < grid.length; k++) {
        if (fg[k] >= 0 && fg[k + 1] <= 0 && (cell < 0 || Math.abs(grid[k] - mag) < Math.abs(grid[cell] - mag))) cell = k
      }
      if (cell < 0) return grid[fg.reduce((bi, x, k) => (Math.abs(x) < Math.abs(fg[bi]) ? k : bi), 0)]
      let lo = grid[cell], hi = grid[cell + 1], flo = fg[cell], fhi = fg[cell + 1]
      for (let k = 0; k < 4; k++) {
        const m = flo === fhi ? 0.5 * (lo + hi) : clamp(lo + ((hi - lo) * flo) / (flo - fhi), lo + 0.05 * (hi - lo), hi - 0.05 * (hi - lo))
        const fm = dr(m)
        if (Math.abs(fm) < 300) return m
        if (fm > 0) { lo = m; flo = fm } else { hi = m; fhi = fm }
      }
      return 0.5 * (lo + hi)
    }
    // t_rev for crossrange (holding sign s pushes the end point to side s); 10 bisections ≈ 0.4 s.
    // The plan ends REV_BIAS on the far side (−s): after the reversal the ship flies −s, so any later
    // drift is correctable by one more (small) reversal instead of being out of reach.
    const bias = revBias(range)
    const side = (o: PredOut): number => (o.crossrangeErr + s * bias) * s
    const solveRev = (m: number): number => {
      const never = run(m, Infinity)
      if (side(run(m, n.t)) > 0) return n.t // even reversing now ends on side s
      if (side(never) < 0) return Infinity // holding never reaches the target line
      let t0 = n.t, t1 = never.t
      for (let k = 0; k < 10; k++) {
        const tm = 0.5 * (t0 + t1)
        if (side(run(m, tm)) > 0) t1 = tm
        else t0 = tm
      }
      return 0.5 * (t0 + t1)
    }
    if (revSoon) tRev = solveRev(mag) // close to a planned reversal only its timing is refined, every second
    else {
      for (let pass = 0; pass < 2; pass++) {
        mag = solveMag(tRev)
        const tPrev = tRev
        tRev = solveRev(mag)
        if (Math.abs(tRev - tPrev) < 3 || (tRev === Infinity && tPrev === Infinity)) break
      }
      // crossrange not nullable with the downrange bank: trade downrange for crossrange by
      // minimising the total predicted miss over |σ| ≥ the downrange solution
      if (tRev === Infinity && Math.abs(run(mag, Infinity).crossrangeErr) > 1000) {
        const miss = (m: number): number => { const o = run(m, Infinity); return Math.hypot(o.downrangeErr - T.aimHigh, o.crossrangeErr) }
        const D = Math.PI / 180
        const cands = Array.from({ length: 37 }, (_, k) => k * 5 * D).filter((m) => uzNow * Math.cos(m) >= LIFT_FLOOR).concat([mag])
        const best = cands.map((m) => ({ m, e: miss(m) })).reduce((x, y) => (y.e < x.e ? y : x))
        const fine = [-2.5, 2.5].map((k) => clamp(best.m + k * D, 0, Math.PI)).map((m) => ({ m, e: miss(m) }))
        mag = [best, ...fine].reduce((x, y) => (y.e < x.e ? y : x)).m
      }
      // T08 P0-5 freeze: if holding the current sign already ends inside the terminal footprint, do
      // not plan a reversal for a marginally better crossrange. The reversal is a 180° roll through
      // lift-vertical; at high q its vertical impulse costs more range than the crossrange it buys.
      const hold = run(mag, Infinity)
      if (Number.isFinite(tRev) && Math.abs(hold.crossrangeErr) < crDeadband(range) && Math.abs(hold.downrangeErr - T.aimHigh) < 1500) {
        tRev = Infinity
      }
    }
    const out = run(mag, tRev)
    const dbg = (globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG
    if (dbg) {
      const nv = run(mag, Infinity), nw = run(mag, n.t)
      dbg(`${n.t.toFixed(0)} range ${(range / 1e3).toFixed(0)} s ${s} sc ${(sc / DEG).toFixed(0)} mag ${(mag / DEG).toFixed(0)} tRev ${Number.isFinite(tRev) ? (tRev - n.t).toFixed(0) : "inf"} out ${(out.downrangeErr / 1e3).toFixed(1)}/${(out.crossrangeErr / 1e3).toFixed(1)} never ${(nv.downrangeErr / 1e3).toFixed(1)}/${(nv.crossrangeErr / 1e3).toFixed(1)} now ${(nw.downrangeErr / 1e3).toFixed(1)}/${(nw.crossrangeErr / 1e3).toFixed(1)} rate ${(rate / DEG).toFixed(1)}°/s`)
    }
    // dwell: a reversal needs (bankHigh + mag)/rate to complete; never start another one before it can
    // finish (the 15 s floor let a limit cycle chatter the bank at 7–13 km range, T08 P0-5)
    const dwell = Math.max(15, (mem.bankHigh + mag) / rate + 5)
    let sign = s
    if (tRev - n.t < REV_LEAD && n.t - mem.revT > dwell) {
      sign = -s
      event = `high-α bank reversal (planned; σ ${((s * mag) / DEG).toFixed(0)}° → ${((-s * mag) / DEG).toFixed(0)}°, range ${(range / 1000).toFixed(0)} km)`
      mem = { ...mem, revT: n.t }
      tRev = Infinity
    }
    mem = {
      ...mem, bankSign: sign, bankHigh: mag, tRevPlan: tRev, sigmaTgt: sign * mag, pred: out, npcT: n.t,
      bellyMiss: Math.hypot(out.downrangeErr, out.crossrangeErr),
    }
  }
  // High-α and belly-flop: altitude-rate damping around the corrector's |σ₀| every step, with the
  // hard descent guard (predictor.ts dampedBank); the lift rotation toward it stays rate-limited below.
  if (phaseP === "highAlpha" || phaseP === "belly") mem = { ...mem, sigmaTgt: mem.bankSign * dampedBank(mem.bankHigh, glideOf(ctx, n), ctx.pred.sref, T.glideZeta, GLIDE_DAMPING.descentMargin) }
  // commanded lift dynamics, as in the predictor: carried with v̂ (passive trim), rotated about v̂
  // toward the target bank at the achievable rate (shortest arc)
  const rightF = cross(vhF, upF)
  const lc0 = mem.liftCmd ?? normalize(lbF)
  const lc1 = normalize(sub(lc0, scale(vhF, dot(lc0, vhF))))
  const lt = add(scale(upF, Math.cos(mem.sigmaTgt)), scale(rightF, Math.sin(mem.sigmaTgt)))
  const dRot = clamp(Math.atan2(dot(cross(lc1, lt), vhF), dot(lc1, lt)), -rate * dt, rate * dt)
  const lc = add(scale(lc1, Math.cos(dRot)), scale(cross(vhF, lc1), Math.sin(dRot)))
  const sNew = Math.atan2(dot(lc, rightF), dot(lc, upF))
  mem = { ...mem, sigmaCmd: sNew, liftCmd: lc }
  return { sigma: sNew, mem, event }
}

/** realised thrust acceleration in ENU (m/s²): body +x × |T|/m */
const aNowEnu = (n: Nav): [number, number, number] => {
  const x = thrDirBody(n)
  const s = n.accThrust
  return [s * dot(x, n.east), s * dot(x, n.north), s * dot(x, n.up)]
}

/** current body +x in ECI */
const thrDirBody = (n: Nav): V3 => {
  const q = n.q
  // qrot(q, ex) without importing to keep this module small
  const x = 1 - 2 * (q.y * q.y + q.z * q.z)
  const y = 2 * (q.x * q.y + q.w * q.z)
  const z = 2 * (q.x * q.z - q.w * q.y)
  return v3(x, y, z)
}
