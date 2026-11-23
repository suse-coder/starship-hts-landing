import { type Q, type V3, v3, add, clamp, cross, dot, norm, normalize, qrot, qrotInv, scale, sub, DEG } from "@ship/math"
import { type EngineCmd, fVac100, ispVac, type VehicleConfig } from "@ship/craft"
import { makeMhdModel, mhdBasis, type MhdBasis } from "@ship/mhd"
import { SHOCK_TABLE, lookupStag } from "@ship/gas"
import { type AttitudeTarget, GAINS, type Gains, torqueCommand } from "./control"
import { allocateMhd, U_BASE } from "./mhdAlloc"
import { flipAltitude } from "./landing"
export { flipAltitude }
import { dampedBank, GLIDE_DAMPING, NOSE_FIRST, type GlideState, passiveTrim, pitchUpDue, predict, type PredContext, type PredInput, type PredOut, type PredPhase, solveBank } from "./predictor"
import { aeroLookup, type AeroTable, landingAero, type LandingAero, mhdDragLookup } from "./tables"
import { atmosphere } from "@ship/earth"
import { alphaNoseFirst } from "./schedule"
import { MISSION_TUNING_DEFAULT, type MissionTuning } from "./tuning"
import { mhdLiftLookup } from "./tables"
import { TOWER, bothPinsLoaded } from "@ship/tower"
import { ENGINE_HEALTH0, type EngineHealth, guideLanding, LAND, LANDING_LAW_STATE0, type LandingLawState, monitorEngines, pinLoop, selectEngines, slamPredict, trimThrustLine } from "./landingLaw"
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
  /** measured aerodynamic force in body axes (N) — the flight-software aero estimate (R6: a
   *  measurement, not a plant oracle; used to calibrate the table's lift/drag scale) */
  readonly aeroF: V3
  /** measured MHD force in body axes (N) */
  readonly mhdF: V3
  /**
   * Total engine force in body axes (N) from the chamber pressures and the gimbal positions — the
   * real thrust vector. It is not along the body axis: the gimbals are deflected for control torque
   * and, with an asymmetric engine set (engine out), canted through the CG.
   */
  readonly engF: V3
  /** east offset of the tower's arm pair centre (tower-side tracking), m */
  readonly armX: number
  /** arm opening (1 open … 0 closed) */
  readonly arms: number
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
  /** last measured crossrange and its sample time, for the closed-loop ZEM sign law */
  readonly crPrev: number
  readonly crT: number
  /** measured / modelled bank rate (open-base bell fins are a model; the plan must not trust it) */
  readonly bankRateScale: number
  readonly sBankPrev: number
  readonly sBankT: number
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
  /** low-passed measured/table aero force ratio (combined; used by the belly model) */
  readonly aeroScale: number
  /** measured/table lift and drag scales (the combined ratio is drag-dominated, so lift needs its own) */
  readonly aeroScaleL: number
  readonly aeroScaleD: number
  /** measured / tabulated MHD lift (the drag scale does not apply to the lift) */
  readonly mhdLiftScale: number
  /** belly-flop disturbance observer: measured minus modelled aero acceleration (ENU, m/s²) */
  readonly bellyDist: V3
  /** this memory belongs to an onboard prediction rollout: plans are frozen, no nested rollouts */
  readonly predicting?: boolean
  /** belly plan: target of the fast model's end point (pad ENU, m) after the rollout correction; null before */
  readonly bellyTgt?: { readonly x: number; readonly y: number } | null
  /** belly plan: time of the last rollout correction, how many were made, and its flown-model miss (m) */
  readonly rollT?: number
  readonly rollN?: number
  readonly rollMiss?: number
  /** prediction rollout started before the belly flop: the belly phase flies the pursuit law */
  readonly bellyPursuit?: boolean
  /** high-α hand-over correction from the rollouts: shift of the fast predictor's aim (m), time, count */
  readonly haBias?: { readonly dr: number; readonly cr: number }
  readonly haRollT?: number
  /** time of the first hand-over check (a bank reversal after it ends the aim search) */
  readonly haFirstT?: number
  readonly haRollN?: number
  /** powered-descent law state (plan, disturbance observer); null before the flip completes */
  readonly land: LandingLawState | null
  /** predicted horizontal miss of the landing plan (m; 0 = the catch point is reachable) */
  readonly landMiss: number
  /** unmodelled acceleration seen by the landing observer (ENU, m/s²): the model-fidelity residual */
  readonly landResidual: V3
  /** sea-level engine health and the engines currently commanded on */
  readonly engHealth: EngineHealth
  readonly engSel: readonly number[]
  /** time of the landing-flip ignition (Infinity before it) */
  readonly tFlip: number
  /** time the engines were cut on pin contact (s); −1 before */
  readonly cutT: number
  /** flip ignition: pin offset (E, N, m) and the nose azimuth unit vector (E, N) — to measure the flip's carry */
  readonly flipP0?: readonly [number, number, number, number]
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
  /** aft flaps deployed for the nose-first drag/steering fall; the flap worker owns the aero/thermal force */
  readonly flapsDeployed: boolean
  /** per-flap deflection, rad (differential = roll/bank: the flap pair is the bank actuator) */
  readonly flapDeflections: readonly number[]
  /** the open-base bell fins are active (engines cold; the gimbals are the aero actuator) */
  readonly bellFins: boolean
  /** spend the gimbal box on roll first (last seconds before the single-engine hand-over) */
  readonly rollFirst: boolean
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
  /** landing aero database (α 60–180°, engines running); optional so older worlds still type-check */
  readonly landAero?: LandingAero
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
/**
 * Cold gas after the first flip (user decision 2026-11-22). The nitrogen system is used only for the
 * deorbit align/burn — the manoeuvre that points the ship retrograde and back. From `entryAlign` on
 * the ship flies on the coil (plasma), the aerodynamic trim and, after the flip, the Raptors. There
 * is no cold gas in the timeline after this point: if the ship cannot hold an attitude without it,
 * the mission fails honestly instead of firing thrusters the design does not have.
 */
export const COLD_GAS_AFTER_FLIP = false
/**
 * Cold gas is flown from entry until the ship is inside the 200 m radius above the tower (user
 * decision 2026-11-22): outside that radius it is the attitude actuator (the coil cannot roll); inside
 * it, the Raptors and their gimbals take everything.
 */
const CATCH_RADIUS = 200
/** flap pair as the bank actuator: maximum deflection (rad) */
const FLAP_MAX = (35 * Math.PI) / 180
/**
 * Phases in which the cold gas may fire at all (user decision: until the plasma starts, and "very
 * little"). `entryAlign` is the vacuum coast between the deorbit flip and the 120 km interface: no
 * air, no plasma, so the coil cannot act there and the gas is the only actuator. It flies on the
 * vacuum gains with their 0.6° phase-plane deadband, i.e. it corrects slowly and rarely.
 * From `entry` (the plasma) onward nothing fires.
 */

export const initMem = (bank0: number): Mem => ({
  phase: "deorbitAlign", tPhase: 0, bank: bank0, bankSign: 1, bankHigh: 30 * DEG, npcT: -1e9, revT: -1e9, pred: null,
  burnDv: 0, armsClose: false, caughtT: 0, reason: "", mhdScale: 1.0, qPrev: 1e9, heading: null, bellyMiss: 0, crPrev: 0, crT: -1e9, bankRateScale: 1, aeroScaleL: 1, aeroScaleD: 1, mhdLiftScale: 1, bellyDist: v3(0, 0, 0), sBankPrev: 0, sBankT: -1e9, sigmaCmd: null, sigmaTgt: 0, tRevPlan: Infinity, hdgCmd: null, liftCmd: null, windOfs: v3(0, 0, 0), tSwitch: Infinity, sigma2: 0, tLandEnd: Infinity, mpcU: null, mpcT: -1e9, landEng: 0, nEngLand: 1, aProf: 0, zProf: 0, uDirPrev: v3(0, 0, 1), uDirT: -1e9, leadF: v3(0, 0, 0), predPath: [], uCmd: [0, 0, 9.79],
  coilOff: false, coilLowT: -1e9, rcsT: -1e9, flapsDeployed: false, aeroScale: 1, land: null, landMiss: 0, landResidual: v3(0, 0, 0), engHealth: ENGINE_HEALTH0, engSel: [], tFlip: Infinity, cutT: -1,
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
/** pre-flip roll damping: tighten the roll-rate band and give the roll loop its own bandwidth */
const rollGains = (g: Gains, bandDeg: number, wnRoll: number): Gains => (bandDeg > 0 ? { ...g, rollBand: bandDeg * DEG, wnRoll } : { ...g, wnRoll })
/** the thrust-direction corridor height schedule (tuning fields; defaults are the flown ramp) */
/**
 * Engines kept after the flip (R11c, Q29/Q33, T09). Two sea-level Raptors at the 40 % throttle floor
 * give T/W ≈ 1.3–1.8 at 110–140 t, so a hover is impossible: the terminal is a timed deceleration
 * (hoverslam, `landing.ts`). Two engines keep the 3×2N gimbal map full rank, so roll comes from
 * differential gimbal — no cold gas. The third engine is only used while the ship is still rotating
 * (attitude authority), never after the flip.
 *
 * The engine count and contact speed are mission tuning fields (`landingEngines`, `contactSpeed`);
 * 3 engines are allowed when their extra brake/lateral authority is needed (the final gate is faster).
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
  t: n.t, r: n.r, v: n.v, mass: n.m, phase, bankEntry: mem.bank, bankHigh: mem.bankHigh, mhdDragScale: mem.mhdScale, mhdLiftScale: mem.mhdLiftScale,
  aeroScale: mem.aeroScale, aeroScaleL: mem.aeroScaleL, aeroScaleD: mem.aeroScaleD,
})

/**
 * Measured aero-force calibration (flight software, not a plant oracle): the table's CL/CD are
 * built from the static panel method at a nominal `cpMax`, but the real shock-layer pressure
 * coefficient and the rarefaction/friction terms vary with altitude and Mach. The measured body
 * force is compared with the table force at the *measured* α/Mach/q; a slow low-pass ratio feeds
 * the 3-DOF predictor. Without it the high-α predictor over-estimates the lift by ~20 % and
 * under-delivers by ~20 km at the Mach-2 hand-over (measured 2026-11-21).
 */
const AERO_SCALE_MIN = 0.35
const AERO_SCALE_MAX = 1.6
let lastAeroLog = -1e9
const measuredAeroScale = (ctx: GuideCtx, n: Nav, mem: Mem, dt: number): Mem => {
  if (n.qbar < 200 || n.V < 40) return mem
  const co = aeroLookup(ctx.aero, n.alpha, n.mach)
  const fTab = n.qbar * ctx.pred.sref * Math.hypot(co.CL, co.CD)
  const fMeas = norm(n.aeroF)
  if (!(fTab > 500) || !(fMeas > 100)) return mem
  const raw = clamp(fMeas / fTab, AERO_SCALE_MIN + 0.05, AERO_SCALE_MAX - 0.05)
  const k = clamp(dt / 2, 0, 1)
  const out = clamp(mem.aeroScale + k * (raw - mem.aeroScale), AERO_SCALE_MIN, AERO_SCALE_MAX)
  // Separate lift/drag calibration. The combined ratio above is drag-dominated at high α, so using
  // it for CL leaves the lift — which sets the range — uncorrected (the 780 km divergence). Split the
  // measured body-frame force into the component along the flow (drag) and perpendicular to it (lift)
  // and compare each with the table at the same α, M.
  const vhat = normalize(n.vAir)
  const fe = qrot(n.q, n.aeroF)
  const drag = -dot(fe, vhat)
  const liftVec = add(fe, scale(vhat, drag))
  const fL = norm(liftVec)
  const fD = Math.abs(drag)
  const Ltab = n.qbar * ctx.pred.sref * co.CL
  const Dtab = n.qbar * ctx.pred.sref * co.CD
  let L = mem.aeroScaleL
  let Ds = mem.aeroScaleD
  let mL = mem.mhdLiftScale
  if (n.mhd && norm(n.mhdF) > 1e3) {
    const me = qrot(n.q, n.mhdF)
    const mDrag = -dot(me, vhat)
    const mLift = norm(add(me, scale(vhat, mDrag)))
    const mLTab = n.qbar * ctx.pred.sref * Math.abs(mhdLiftLookup(ctx.pred.mhdDrag, Math.max(n.alt, 0), n.V))
    if (mLTab > 500 && mLift > 100) mL = clamp(mL + k * (clamp(mLift / mLTab, 0.2, 5) - mL), 0.2, 5)
  }
  if (Ltab > 500 && fL > 100) L = clamp(L + k * (clamp(fL / Ltab, 0.35, 2.2) - L), 0.35, 2.2)
  if (Dtab > 500 && fD > 100) Ds = clamp(Ds + k * (clamp(fD / Dtab, 0.35, 2.2) - Ds), 0.35, 2.2)
  const dbg = (globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG
  if (dbg && n.t - lastAeroLog > 5) {
    lastAeroLog = n.t
    dbg(`as ${n.t.toFixed(0)} h ${(n.alt / 1e3).toFixed(1)} M ${n.mach.toFixed(1)} a ${(n.alpha / DEG).toFixed(0)} meas ${(fMeas / 1e3).toFixed(0)}kN tab ${(fTab / 1e3).toFixed(0)}kN ratio ${raw.toFixed(2)} filt ${out.toFixed(2)} L ${mem.aeroScaleL.toFixed(2)} D ${mem.aeroScaleD.toFixed(2)} M ${mem.mhdLiftScale.toFixed(2)}`)
  }
  return { ...mem, aeroScale: out, aeroScaleL: L, aeroScaleD: Ds, mhdLiftScale: mL }
}

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

const enuToEci = (n: Nav, a: V3): V3 => add(add(scale(n.east, a.x), scale(n.north, a.y)), scale(n.up, a.z))

/** limit the thrust direction to within maxTilt of vertical and above a minimum magnitude */
export interface GuideOut {
  readonly cmd: Cmd
  readonly mem: Mem
  readonly events: readonly string[]
}

const to = (mem: Mem, phase: Phase, t: number, extra: Partial<Mem> = {}): Mem => ({ ...mem, ...extra, phase, tPhase: t, ...(phase === "flip" ? { tFlip: t } : {}) })

/** total thrust during the flip, in units of one engine at 100 %: two engines at the 40 % floor */
const FLIP_THRUST = 1.2

/** state at the end of an onboard prediction rollout (pad ENU relative to the catch point) */
export interface RolloutEnd {
  readonly phase: Phase
  readonly t: number
  readonly pinRel: V3
  readonly velRel: V3
  /** nose azimuth at the end state: horizontal unit vector of the body axis (pad E, N) */
  readonly nose?: { readonly e: number; readonly n: number }
}
/**
 * Onboard prediction: fly the vehicle model (the same 6-DOF dynamics and flight software, forecast
 * wind, no turbulence, no knowledge of future failures) from the current state with the given
 * guidance memory until the phase `until` is entered. Supplied by the simulation host.
 */
export type Rollout = (mem: Mem, until: Phase) => RolloutEnd | null

export const guide = (ctx: GuideCtx, n: Nav, mem0: Mem, dt: number, rollout?: Rollout): GuideOut => {
  const v = ctx.vehicle
  const T = ctx.tuning ?? MISSION_TUNING_DEFAULT
  const ev: string[] = []
  let mem = mem0
  const bellOn = v.bellFins !== null
  // inside the 200 m radius above the tower the cold gas is off: the Raptors and their gimbals own
  // the last metres (user decision)
  const nearCatch = Math.hypot(n.pinRel.x, n.pinRel.y) < CATCH_RADIUS && n.pinRel.z < 2000
  const base: Omit<Cmd, "att"> = {
    gains: GAINS.vacuum, feedForward: false, ffMoment: ownAeroMoment(ctx, n), useMhd: false, useRcs: !nearCatch, rcsRollOnly: false, flapsDeployed: false, flapDeflections: [], bellFins: false, rollFirst: false, engines: ENGINES_OFF,
    seeding: false, coil: "hold", armsClose: mem.armsClose, alphaCmd: n.alpha,
  }
  const coilOn = n.coilI > 0.5 * n.coilIop
  const plasma = n.t0Stag > 1800 && n.qbar > 30
  const mhdUsable = coilOn && plasma && n.mhd !== null
  const tp = n.t - mem.tPhase
  // measured aero calibration (all atmospheric phases; a no-op when q is negligible)
  mem = measuredAeroScale(ctx, n, mem, dt)
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
        // plasma gone: the coil has no job left, so switch it off (rampdown) instead of holding or
        // ramping it — the recorded current goes to zero from here
        const cc = plasma
          ? coilCommand(ctx, n, mem, ev, att, GAINS.aero, mhdUsable)
          : { coil: (n.t >= ctx.tRamp ? "rampdown" : "hold") as Cmd["coil"], mem }
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
      if (mem.phase === "entry" && n.t - mem.npcT > 15 && n.V > ctx.pred.vLoft + 150) {
        const sol = solveBank(ctx.pred, predInput(n, mem, "entry"), "entry", mem.bank, T.aimEntry)
        mem = { ...mem, bank: sol.bank, pred: sol.out, npcT: n.t }
      }
      // Lateral: bank reversal on the *zero-effort miss*, not the crossrange position. A position-only
      // rule lets the ship fly through the line: it banks toward the pad all the way in, crosses the
      // line and arrives with hundreds of m/s of crossrange rate (the 384 km hand-over). Reversing
      // when `crossrange + rate · time-to-go` is on the bank's side brakes the closing rate before the
      // crossing. The dwell covers a full 2|σ| reversal at the coil's bank slew (≈ 30 s at 40°).
      const { cr, range } = crossrange(n)
      const dtCr = n.t - mem.crT
      const crDot = dtCr > 0.5 && dtCr < 30 ? (cr - mem.crPrev) / dtCr : 0
      mem = { ...mem, crPrev: cr, crT: n.t }
      const tgoCr = clamp(range / Math.max(n.V, 200), 10, 400)
      const zemCr = cr + 0.85 * crDot * tgoCr
      const dbCr = crDeadband(range)
      if (mem.phase === "entry" && Math.abs(zemCr) > dbCr && Math.sign(zemCr) === mem.bankSign && n.t - mem.revT > 40) {
        mem = { ...mem, bankSign: -mem.bankSign, revT: n.t }
        ev.push(`bank reversal (ZEM) → ${mem.bankSign > 0 ? "right" : "left"}: crossrange ${(cr / 1000).toFixed(1)} km, rate ${(crDot / 1e3).toFixed(2)} km/s, ZEM ${(zemCr / 1000).toFixed(1)} km of ${(range / 1000).toFixed(0)} km`)
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
      const pb = planBank(ctx, n, mem, dt, "highAlpha", ev, rollout)
      mem = pb.mem
      if (pb.event) ev.push(pb.event)
      // Measured bank-rate calibration. The open-base bells are a *model* (wake factor, bell area,
      // Newtonian derivative); the plan may not trust it blindly — with wake 0.85 the uncalibrated
      // plan mistimed a reversal and the run lost 778 km of downrange. Measure the flown rotation
      // about v̂ while the bank is actually slewing (commanded far from flown), low-pass the ratio
      // achieved/modelled, and feed it back into every bank-rate estimate.
      {
        const vhC = normalize(n.vAir)
        const xbC = thrDirBody(n)
        const lbC = sub(xbC, scale(vhC, dot(xbC, vhC)))
        const upC = normalize(sub(n.up, scale(vhC, dot(n.up, vhC))))
        const sFl = Math.atan2(dot(lbC, cross(vhC, upC)), dot(lbC, upC))
        // sample-and-hold: the guidance runs at ~0.01 s, so accumulate the flown bank for 1 s and only
        // then form a rate. (A per-step delta is pure numerical noise.)
        if (mem.sBankT < 0) {
          // first sample: set the baseline, no rate yet
          mem = { ...mem, sBankPrev: sFl, sBankT: n.t }
        } else {
          const dtB = n.t - mem.sBankT
          if (dtB > 1 && norm(lbC) > 1e-6) {
            const achieved = Math.abs(sFl - mem.sBankPrev) / dtB
            const modelled = bankRateNow(ctx, n, SIGMA_RATE_GAS, 1)
            const err = Math.abs(((mem.sigmaTgt - sFl + Math.PI) % (2 * Math.PI)) - Math.PI)
            if (err > 3 * DEG && achieved > 0.03 * DEG && modelled > 0.1 * DEG) {
              const k = clamp(achieved / modelled, 0.35, 1.6)
              mem = { ...mem, bankRateScale: clamp(mem.bankRateScale + (Math.min(dtB, 5) / 12) * (k - mem.bankRateScale), 0.35, 1.6) }
            }
            mem = { ...mem, sBankPrev: sFl, sBankT: n.t }
          }
        }
      }
      // Dense-air hand-over (T11/M3): the high-α aero trim/bank stays the actuator until the air is
      // thick enough for the lift-azimuth law (Mach 2). A coil-authority loss at M 6/36 km (Q25) must
      // *not* force the belly flop: the bare hull is passively stable at α ≈ 28°, the aero bank steers
      // with cold gas at the gas slew rate, and the belly-flop law has no usable authority up there
      // (measured: belly flop at 35.8 km/M 6 delivered a 6.4–6.5 km miss at the flip). Keeping the
      // high-α bank to Mach 2 is the earlier 11 m baseline.
      if (n.mach < 2.0) {
        mem = to(mem, "bellyFlop", n.t, { heading: null, tSwitch: Infinity, npcT: -1e9, liftCmd: null })
        ev.push(`belly flop at Mach ${n.mach.toFixed(1)}: h ${(n.alt / 1000).toFixed(1)} km, q ${(n.qbar / 1000).toFixed(1)} kPa${mem.coilOff ? ", coil off (aero bank only)" : ""}; lift-azimuth control to the low flip gate`)
      }
      // Fly the *passive trim* α, not the instantaneous one. The predictor's lift/drag are the trim
      // values; holding the current α (which had drifted 3° high) made the flown lift differ from the
      // planned lift for the whole 600 s phase — a several-hundred-km downrange error.
      const atHigh = passiveTrim(ctx.aero, n.mach)
      const nf = noseFor(n.vAir, n.up, atHigh, pb.sigma)
      const att = { x: nf.x, spin: 0, free: alphaAxis(n), bankAxis: normalize(n.vAir) }
      // Q25: keep the coil until its steering authority is gone (3 s hysteresis), then ramp down
      const cc = coilCommand(ctx, n, mem, ev, att, GAINS.aero, mhdUsable)
      mem = cc.mem
      const mhdHigh = mhdUsable && !mem.coilOff
      // Flaps: deployed at coil-authority loss (as late as possible) and used as the bank actuator.
      // The pair is differential: the bank error is the angle between the commanded lift azimuth and
      // the one actually flown; a differential deflection rolls the hull about its axis, which at
      // high α is the bank. Rate damping on ω_x stops the overshoot.
      // deployed at coil-authority loss (measured better than deploying at plasma onset: 39.5 km
      // versus 87 km at the arms — the pair is too weak to hold the bank for the whole 380 s phase,
      // and deploying it early adds drag where the ship needs range)
      const flapDeploy = mem.coilOff
      // flown lift azimuth about v̂ (same construction as planBank's sFl)
      const vhF = normalize(n.vAir)
      const upF = normalize(sub(n.up, scale(vhF, dot(n.up, vhF))))
      const lbF = sub(thrDirBody(n), scale(vhF, dot(thrDirBody(n), vhF)))
      const sFl = Math.atan2(dot(lbF, cross(vhF, upF)), dot(lbF, upF))
      const bankErr = Math.atan2(Math.sin(mem.sigmaTgt - sFl), Math.cos(mem.sigmaTgt - sFl))
      const dFlap = clamp(1.2 * bankErr - 0.8 * n.w.x, -FLAP_MAX, FLAP_MAX)
      return {
        cmd: {
          ...base, att, gains: GAINS.aero, useMhd: mhdHigh, bellFins: (globalThis as { BELL_HA?: boolean }).BELL_HA === true, rcsRollOnly: (globalThis as { GAS_ROLL?: boolean }).GAS_ROLL === true, flapsDeployed: flapDeploy, flapDeflections: flapDeploy ? [dFlap, -dFlap] : [],
          seeding: mhdHigh && n.seedLeft > 0, coil: cc.coil, alphaCmd: atHigh,
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
        const sg = steepGlide(ctx, n, mem, dt, ev, rollout)
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
      // Flip-and-stop ignition is timed on the *stop point*, inside an altitude window around the
      // gate. The flip is a fixed manoeuvre that carries the ship ≈ `carry` metres along its nose
      // azimuth before it stands still (measured 531–559 m); the belly flop closes on the axis at
      // ≈ 35 m/s, so each second of waiting moves the stop point 35 m along the track. Igniting
      // when the stop point reaches the axis turns the belly flop's along-track error — the one
      // it cannot steer out (drag-dominated: every candidate plan ended at the same downrange) —
      // into a few hundred metres of ignition height, which the descent law does not care about.
      const xbH = thrDirBody(n)
      const nE = dot(xbH, n.east), nN = dot(xbH, n.north), nH = Math.hypot(nE, nN) || 1
      const ahead = -(n.pinRel.x * nE + n.pinRel.y * nN) / nH
      const vAlong = (n.velRel.x * nE + n.velRel.y * nN) / nH
      const F = LAND.flip
      const stopNow = ahead - vAlong * F.carryLead <= F.carry
      const ignite = F.stop && F.window > 0
        ? n.hBase < hIgn - F.window || (n.hBase < hIgn + F.window && stopNow)
        : n.hBase < hIgn
      if (ignite && mem.coilOff) {
        mem = to(mem, "flip", n.t, { flipP0: [n.pinRel.x, n.pinRel.y, nE / nH, nN / nH] })
        ev.push(`landing flip: ${T.landingEngines} × Raptor ignition at ${n.hBase.toFixed(0)} m, v_z ${vz.toFixed(1)} m/s, predicted miss ${(mem.bellyMiss).toFixed(0)} m${F.stop && F.window > 0 ? `; axis ${ahead.toFixed(0)} m ahead along the nose (stop carry ${F.carry} m), gate ${hIgn.toFixed(0)} ± ${F.window} m` : ""}`)
      }
      const vh = normalize(n.vAir)
      const att = zCmd ? { x: xCmd, z: zCmd, free: alphaAxis(n), bankAxis: vh } : { x: xCmd, spin: 0, free: alphaAxis(n), bankAxis: vh }
      return { cmd: { ...base, att, gains: rollGains(GAINS.belly, T.rollBandBellyDeg, T.rollWnBelly), bellFins: bellOn, coil: mem.coilOff ? "discharge" : "hold" }, mem, events: ev }
    }
    case "flip":
    case "landing":
    case "terminal": {
      // From flip ignition to the pins the sea-level Raptors are the only actuators (R11c): thrust,
      // pitch/yaw and — with two or more running — roll by differential gimbal. Guidance is
      // `landingLaw.ts`: one convex plan to the pins, tracked and re-solved; the throttle floor is a
      // constraint of the plan (two Raptors cannot hover, so the descent is a timed deceleration).
      const eng0 = v.engines[0]
      const tMaxN = fVac100(eng0) - n.pAmb * eng0.exitArea
      const tMinN = eng0.minThrottle * fVac100(eng0) - n.pAmb * eng0.exitArea
      const rel = n.pinRel
      const pz = rel.z
      const dHoriz = Math.hypot(rel.x, rel.y)
      const vHoriz = Math.hypot(n.velRel.x, n.velRel.y)
      const flipping = mem.phase === "flip"
      // ---- engine health and selection. The flip lights all three (redundancy while the ship is
      // broadside and falling); the descent runs `landingEngines` of the healthy ones. A failed
      // engine is replaced by the spare; with a single engine left the plan is re-made for one.
      const mon = monitorEngines(mem.engHealth, n.t, mem.engSel, n.thrusts, eng0.spoolTime, tMinN)
      mem = { ...mem, engHealth: mon.health }
      for (const i of mon.newlyFailed) ev.push(`ENGINE OUT: Raptor ${i + 1} delivers no thrust at ${pz.toFixed(0)} m — reconfiguring on the remaining engines`)
      // The flip is on two engines at the 40 % floor: three would add ~150 m/s of vertical impulse in
      // six seconds and leave the ship climbing; two at the floor give ~60 m/s, which is what the
      // hoverslam from the low gate needs. The third engine stays available as the spare.
      // After the burn the law stages to `terminalEngines` (one Raptor: throttleable through the
      // weight); the engine kept is the one nearest the pin axis' normal plane, so the trim tilt of
      // the off-centre thrust line (≈ 3°) lifts both pins almost equally.
      const stagedNow = !flipping && (mem.land?.staged ?? false)
      const wantEng = flipping ? Math.max(2, Math.min(3, T.landingEngines)) : stagedNow ? Math.min(T.terminalEngines, T.landingEngines) : T.landingEngines
      const keepPref = stagedNow
        ? [...mem.engSel].sort((a, b) => Math.abs(v.engines[a].gimbal.y) - Math.abs(v.engines[b].gimbal.y))
        : mem.engSel
      const sel = selectEngines(mon.health.failed, keepPref, wantEng)
      mem = { ...mem, engSel: sel }
      const nEng = sel.length
      const healthy = mon.health.failed.filter((f) => !f).length
      const nBurn = stagedNow ? Math.max(nEng, 1) : Math.max(Math.min(T.landingEngines, healthy), 1)
      // ---- measured thrust vector and the trimmed thrust line (through the CG for the running set)
      const toEnu = (a: V3): V3 => v3(dot(a, n.east), dot(a, n.north), dot(a, n.up))
      const bX = thrDirBody(n)
      // Trimmed thrust line: through the CG for the running set — and, when the air puts a steady
      // moment on the hull, canted by what the gimbal needs to hold it (τ = r × F = −M_aero:
      // Δd_z = −M_y/(T·L), Δd_y = M_z/(T·L)). Without this term the thrust flown in steady state
      // is not along the line the attitude command assumes: on one off-centre engine the hull
      // flies 3° to the airflow, the gimbal sat ≈ 1° off its trim angle and the pins stood 1.4 m
      // off the axis for the whole fall (measured on the isolated rig).
      const line0 = trimThrustLine(v.engines.slice(0, 3), n.thrusts, n.xcg)
      const tSum = n.thrusts.reduce((a, b) => a + b, 0)
      let lineB = line0
      if (ctx.landAero && tSum > 2e5 && !flipping) {
        const Ma = landingAero(ctx.landAero, atmosphere(Math.max(n.alt, 0)).rho, bX, n.vAir, n.xcg).M
        const Mb = qrotInv(n.q, Ma)
        const TL = tSum * Math.max(n.xcg - v.engines[0].gimbal.x, 1)
        const lim = Math.sin(6 * DEG)
        lineB = normalize(v3(line0.x, line0.y + clamp(Mb.z / TL, -lim, lim), line0.z + clamp(-Mb.y / TL, -lim, lim)))
      }
      const lineEci = qrot(n.q, lineB)
      const lineEnu = toEnu(lineEci)
      const uReal = scale(toEnu(qrot(n.q, n.engF)), 1 / n.m)
      const uAxis = scale(lineEnu, n.accThrust)
      const uAxisDot = scale(toEnu(cross(qrot(n.q, n.w), lineEci)), n.accThrust)
      // ---- environment model: forecast wind + measured offset, landing aero database
      const wf = ctx.windForecast(Math.max(n.hBase, 0))
      const wind = v3(wf.x + mem.windOfs.x, wf.y + mem.windOfs.y, 0)
      const rho = atmosphere(Math.max(n.alt, 0)).rho
      const la = ctx.landAero
      const aeroForce = (xh: V3, va: V3): V3 => (la ? landingAero(la, rho, xh, va, n.xcg).F : scale(va, -0.5 * rho * 40 * norm(va)))
      // ---- single-engine roll coupling. One off-centre gimballed engine at (y, z) makes the roll
      // torque (y·τ_y + z·τ_z)/L whenever it makes pitch/yaw torque, so with no other torque
      //   H = ω_x − κ·(y·ω_y + z·ω_z),  κ = I_yy / (I_xx·L)   (κ·r ≈ 1.4 here)
      // is conserved: the roll rate left when the ship is upright and still is H at the moment the
      // other engine shut down. The burn set (differential gimbal) therefore drives H → 0 before
      // staging, for the engine that will be kept.
      const keepEng = v.engines[[...sel].sort((a, b) => Math.abs(v.engines[a].gimbal.y) - Math.abs(v.engines[b].gimbal.y))[0] ?? 0]
      const kappa = n.I.y / (n.I.x * Math.max(n.xcg - keepEng.gimbal.x, 1))
      const spinH = kappa * (keepEng.gimbal.y * n.w.y + keepEng.gimbal.z * n.w.z)
      const hRoll = n.w.x - spinH
      // roll angle error of the pins (body z → north/south, whichever is nearer) and the roll the
      // tilt will add while the ship comes upright (φ − κ·(y·θ_y + z·θ_z) is conserved on one engine)
      const qq0 = n.q
      const zb0 = v3(2 * (qq0.x * qq0.z + qq0.w * qq0.y), 2 * (qq0.y * qq0.z - qq0.w * qq0.x), 1 - 2 * (qq0.x * qq0.x + qq0.y * qq0.y))
      const zT0 = dot(zb0, n.north) >= 0 ? n.north : scale(n.north, -1)
      const eRoll = Math.atan2(dot(cross(zb0, zT0), bX), dot(zb0, zT0))
      const upB = qrotInv(n.q, n.up)
      void kappa; void upB
      let uCmd: V3
      let uDot = v3(0, 0, 0)
      let tgo = Infinity
      let tRailLaw: number | null = null
      if (flipping) {
        // Flip: rotate straight to the attitude the brake will start from, not to vertical. The brake
        // to the staging gate begins tilted back against the horizontal velocity (beyond
        // retrograde); stopping the flip at vertical and then leaning back cost seconds at 78 m/s
        // and, at the throttle floor, could not be done at all against the crossflow moment. A
        // constant total thrust gives the gimbals their torque.
        const stagedFlip = T.terminalEngines < T.landingEngines
        const bodyUp = dot(bX, n.up)
        // full flip thrust while the nose is still coming up (gimbal torque), brake thrust after
        let aT = ((stagedFlip && bodyUp > Math.cos(55 * DEG) ? LAND.flip.thrust : FLIP_THRUST) * tMaxN) / n.m
        let dir = v3(0, 0, 1)
        if (stagedFlip && LAND.flip.stop && vHoriz > 0.05) {
          // Flip-and-stop. The burn set (two Raptors, 22 m/s² at 60 %, roll by differential gimbal)
          // leans back against the horizontal velocity and *keeps* leaning until that velocity is
          // gone: a_h = kV·v_h, i.e. the lean fades with the speed and the body arrives upright and
          // laterally at rest. The old flip stopped after 9 s at the 50° lean with 40 m/s still on
          // (measured), and left the stopping to one off-centre engine with no roll control — the
          // curved, overshooting approach. Thrust: full flip thrust while the nose comes up (gimbal
          // torque), then only what the horizontal demand needs at the maximum lean, never below
          // the two-engine floor.
          // The lean is a function of the speed left that the body can *follow*: coming back upright
          // from the lean takes lean/r seconds (r: a slew rate well inside the attitude loop's
          // limit) and removes a·(1 − cos lean)/r of speed on the way, so the lean is reduced along
          // lean(v) = acos(1 − r·v/a). A lean proportional to the speed asked for 31°/s of slew at
          // the 32°/s limit; the body lagged 40° and the speed went 3.5 → −15 → +48 m/s (measured).
          // Near rest (below `vSoft`) the curve is made linear, a_h ≈ √(2·r·a/v_s)·v_h ≈ 1.1·v_h.
          const F = LAND.flip
          const aFloor = (2 * tMinN) / n.m * 1.03
          const aFull = aT
          const thrustAt = (c: number): number => aFloor + (aFull - aFloor) * clamp(c / F.vFull, 0, 1)
          // nominal stop law: lean as a function of the speed left (1 − cos(lean) = (r/a)·v²/(v + v_s),
          // capped at the nominal lean) — the slew-limited curve, linear near rest
          const leanAt = (c: number, cap: number): number => Math.min(cap, Math.acos(clamp(1 - ((F.slew * DEG) / thrustAt(c)) * (c * c) / (c + F.vSoft), -1, 1)))
          if (bodyUp > Math.cos(55 * DEG)) aT = thrustAt(vHoriz)
          const aHnow = Math.max(0, -(uReal.x * n.velRel.x + uReal.y * n.velRel.y) / vHoriz)
          const vEff = Math.max(vHoriz - F.lag * aHnow, 0)
          const tb = leanAt(vEff, F.tiltMax * DEG)
          const ux = -n.velRel.x / vHoriz, uy = -n.velRel.y / vHoriz
          // (A closed loop on the predicted stop point was built and measured here — scale the
          // braking by D/(D − e∥), turn the lean by the cross-track zero-effort miss — and removed:
          // two engines cannot throttle below ≈ 1.5 g, so every second of gentler or re-aimed
          // braking is a second of climbing. It hunted for 8 s and the ship rose at +43 m/s.
          // The stop is the fixed manoeuvre; where it ends is the belly flop's delivery.)
          dir = v3(ux * Math.sin(tb), uy * Math.sin(tb), Math.cos(tb))
        } else if (stagedFlip && !LAND.flip.stop && vHoriz > 0.5) {
          const tb = Math.min(LAND.flip.tilt0 + LAND.flip.tiltPerV * vHoriz, LAND.flip.tiltMax) * DEG
          dir = v3((-n.velRel.x / vHoriz) * Math.sin(tb), (-n.velRel.y / vHoriz) * Math.sin(tb), Math.cos(tb))
        }
        uCmd = scale(dir, aT)
        mem = { ...mem, predPath: [] }
      } else {
        const out = guideLanding({
          t: n.t, rel, vel: n.velRel, mass: n.m, uReal, uAxis, uAxisDot: n.accThrust > 1 ? uAxisDot : null, xBody: toEnu(bX),
          nEng: stagedNow ? Math.max(Math.min(T.landingEngines, healthy), 1) : nBurn, stageTo: T.terminalEngines < T.landingEngines ? T.terminalEngines : undefined,
          propellant: n.propHeader, thrustMax: tMaxN, thrustMin: tMinN, ve: ispVac(eng0) * 9.80665, pAe: n.pAmb * eng0.exitArea,
          wind, aeroForce, contactSpeed: T.contactSpeed, kp: T.landKp, kd: T.landKd, hoverKp: T.hoverKp, hoverKd: T.hoverKd, armed: mem.armsClose, quiet: Math.abs(hRoll) < 1.2 * DEG && Math.abs(eRoll) < 12 * DEG,
          // the lateral loop is closed on the pin point: v_pin = v_cg + ω × r_pin
          velPin: add(n.velRel, toEnu(cross(qrot(n.q, n.w), qrot(n.q, v3(v.pinX - n.xcg, 0, 0))))),
          tilt: { e: Math.atan2(lineEnu.x, Math.max(lineEnu.z, 0.1)), n: Math.atan2(lineEnu.y, Math.max(lineEnu.z, 0.1)) },
          tiltRate: (() => { const d = toEnu(cross(qrot(n.q, n.w), lineEci)); return { e: d.x, n: d.y } })(),
          aeroTilt: (() => {
            // moment slope of the landing-aero model at 5° tail-first, per (m/s)², and the gimbal
            // torque of the kept engine per unit of thrust acceleration beside its trim angle
            if (!la) return undefined
            const a5 = 5 * DEG, Vr = 60
            const M5 = norm(landingAero(la, rho, v3(Math.sin(a5), 0, Math.cos(a5)), v3(0, 0, -Vr), n.xcg).M)
            const L = Math.max(n.xcg - keepEng.gimbal.x, 1)
            const avail = keepEng.gimbalMax - Math.atan(Math.hypot(keepEng.gimbal.y, keepEng.gimbal.z) / L)
            return { kM: M5 / (a5 * Vr * Vr), TLs: n.m * L * Math.sin(avail) }
          })(),
        }, mem.land ?? LANDING_LAW_STATE0)
        ev.push(...out.events)
        mem = { ...mem, land: out.state, landMiss: out.miss, landResidual: out.residual, predPath: out.path.map((q) => v3(q.x, q.y, q.z + ctx.armHeight)) }
        uCmd = out.u; uDot = out.uDot; tgo = out.tgo; tRailLaw = out.tRail
        if (mem.phase === "landing" && out.frozen) {
          mem = to(mem, "terminal", n.t)
          ev.push(`terminal: ${out.tgo.toFixed(1)} s to the pins, ${pz.toFixed(1)} m above the rails at ${(-n.velRel.z).toFixed(1)} m/s, offset ${dHoriz.toFixed(2)} m — plan frozen, tracking only`)
        }
      }
      mem = { ...mem, uCmd: [uCmd.x, uCmd.y, uCmd.z] }
      // ---- attitude command: the body axis that puts the trimmed thrust line on the commanded
      // direction, with the reference's own rotation rate fed forward
      const um = norm(uCmd)
      const dirEnu = scale(uCmd, 1 / Math.max(um, 1e-6))
      const thrDir = normalize(enuToEci(n, dirEnu))
      const attX = normalize(sub(thrDir, sub(lineEci, bX)))
      const dDot = um > 1 ? scale(sub(uDot, scale(dirEnu, dot(uDot, dirEnu))), 1 / um) : v3(0, 0, 0)
      const rateEci = enuToEci(n, cross(dirEnu, dDot))
      // ---- throttle, vertical channel first: size the thrust so that its *vertical* component is the
      // commanded one at the attitude actually flown (the descent is a timed deceleration; an attitude
      // lag must not become a vertical error). A starting engine counts by the thrust it delivers.
      const lineUp = dot(lineEci, n.up)
      const pointErr = Math.acos(clamp(dot(lineEci, thrDir), -1, 1))
      // Brake to the gate: the thrust is the commanded acceleration *projected on the thrust line*
      // actually flown. While the body is still turning to the braking attitude, full commanded
      // thrust along the wrong axis was mostly vertical: it stopped the descent and the ship
      // ballooned (measured: sink −43 → +8 m/s in 3.5 s at 53° of the commanded 78° tilt).
      const braking = false as boolean
      const thrustWant = braking
        ? n.m * Math.max(dot(uCmd, lineEnu), 0)
        : !flipping && lineUp > 0.5 && pointErr < 25 * DEG ? (n.m * uCmd.z) / lineUp : n.m * um
      const tTop = sel.reduce((mx, i) => Math.max(mx, n.thrusts[i] ?? 0), 0)
      const nEff = tTop > 0.5 * tMinN ? sel.reduce((sum, i) => sum + clamp((n.thrusts[i] ?? 0) / tTop, 0, 1), 0) : Math.max(nEng, 1)
      const throttle = clamp((thrustWant / Math.max(nEff, 0.5) + n.pAmb * eng0.exitArea) / fVac100(eng0), eng0.minThrottle, 1)
      // ---- roll: the pins (body ±y) must end over the east/west rails, so body z → north or south,
      // whichever is nearer. Free above 450 m (the hull is axisymmetric), then aligned gently.
      const flipUp = LAND.flip.rollAlign && flipping && T.terminalEngines < T.landingEngines && dot(bX, n.up) > Math.cos(55 * DEG)
      // Roll on the burn set (differential gimbal). The roll *rate* follows the single-engine
      // coupling value at all times (H = ω_x − spinH → 0), so the hand-over to one engine can happen
      // at any moment; the pin alignment is added as a slow rate on top (a 20°/s alignment slew in
      // progress at the hand-over left H = −20…−35°/s on the single engine, which has no roll
      // actuator — measured). Once the hand-over is armed only the coupling is tracked.
      const zT = zT0
      const burnSet = !flipping && !stagedNow && T.terminalEngines < T.landingEngines
      void burnSet
      const oneEng = !flipping && nEng === 1
      // Pin azimuth. The two-engine flip turns the pins over the rails by differential gimbal (it
      // hands over within ≈ 2°); on one engine the cold-gas roll couples then *hold* that azimuth
      // for the whole descent. They used to be given a roll angle only below 450 m ("roll is free
      // higher up"): a correction across the rails rolled the hull 13° through the gimbal's
      // coupling and it stayed 12° off for the next 15 s, to be turned back in the last 400 m with
      // an overshoot (measured). Seen from above, the pins were visibly not square to the arms.
      const holdPins = oneEng && LAND.rollGas
      const att = (pz > 450 || flipping) && !flipUp && !holdPins
        ? { x: attX, spin: 0, rate: rateEci }
        : { x: attX, z: zT, rate: rateEci }
      const cmdEngines: EngineCmd[] = [0, 1, 2].map((i) => ({ on: sel.includes(i), throttle, gp: 0, gy: 0 }))
      if (flipping) {
        const flipErr = Math.acos(clamp(dot(normalize(bX), thrDir), -1, 1))
        // the flip ends when the thrust axis is near vertical and the rotation is arrested (timeout 9 s)
        const stagedFlip = T.terminalEngines < T.landingEngines
        const bodyTilt = Math.acos(clamp(dot(lineEci, n.up), -1, 1))
        const done = stagedFlip && LAND.flip.stop
          // flip-and-stop ends upright and laterally at rest (or on its time limit)
          // … or as soon as the single engine can finish it (slow, nearly upright), or before the
          // two engines (≥ 1.5 g) stop the descent altogether
          ? (vHoriz < LAND.flip.endVh && bodyTilt < LAND.flip.endUpright * DEG && Math.hypot(n.w.y, n.w.z) < LAND.flip.endRate * DEG)
            || (-n.velRel.z < LAND.flip.sinkMin && bodyTilt < 25 * DEG) || tp > LAND.flip.tMax
          : stagedFlip
          ? (flipErr < LAND.flip.endTilt * DEG && Math.hypot(n.w.y, n.w.z) < 5 * DEG) || tp > LAND.flip.tMax
          : (flipErr < 22 * DEG && Math.hypot(n.w.y, n.w.z) < 5 * DEG) || tp > 9
        if (n.spoolDone && tp > 2 && done) {
          mem = to(mem, "landing", n.t, { land: null })
          const f0 = mem.flipP0
          const carried = f0 ? `; carried ${((rel.x - f0[0]) * f0[2] + (rel.y - f0[1]) * f0[3]).toFixed(0)} m along the ignition nose azimuth, ${((rel.x - f0[0]) * f0[3] - (rel.y - f0[1]) * f0[2]).toFixed(0)} m across` : ""
          ev.push(`flip complete in ${tp.toFixed(1)} s on ${nEng} engine(s): ${pz.toFixed(0)} m above the rails, ${dHoriz.toFixed(0)} m off (E ${rel.x.toFixed(1)}, N ${rel.y.toFixed(1)}), ${vHoriz.toFixed(1)} m/s horizontal, sink ${(-n.velRel.z).toFixed(0)} m/s${carried} — planned descent on ${Math.min(T.landingEngines, nEng)}`)
        }
      }
      // ---- arms: close on the time to the rails (the plan's own time-to-go), geometry checked at
      // the predicted closure position
      let arms = mem.armsClose
      const tRail = tRailLaw !== null && stagedNow ? tRailLaw : Number.isFinite(tgo) && tgo > 0 ? tgo : pz / Math.max(-n.velRel.z, 0.4)
      const lead = Math.max(TOWER.closeTime + 0.2, T.armCloseLead)
      const xClose = rel.x + n.velRel.x * Math.min(lead, tRail)
      const yClose = rel.y + n.velRel.y * Math.min(lead, tRail)
      // The arm pair tracks the ship across the rails (tower-side, ±reach): the across condition is
      // the part of the predicted offset the arms cannot reach, and the pair must already be near the
      // ship before it shuts on the hull.
      const xBeyond = Math.max(0, Math.abs(xClose) - (TOWER.armReach - 0.5))
      const armNear = Math.abs(n.armX - clamp(rel.x, -TOWER.armReach, TOWER.armReach)) < 2
      // The arms close as soon as the ship is between them: inside the corridor of the open arms
      // (|x| < 6 m, south of the carriage), above the rails and slow enough that the hull cannot
      // strike them. The time-to-rails rule stays as the fallback for a ship that is still outside
      // the corridor when the rails are close (it may still be translating in).
      // close as soon as the ship is between the arms, but only when the 3 s travel still finishes
      // before the rails (otherwise the pins arrive on open rails, which must never count)
      // "Between the arms" is literal: the aft end (pinX below the pins) has passed below the rail
      // tops. Closing earlier shut the arms under the descending ship — the hull then sat on the
      // closed pair 44 m up and burned its tanks dry (measured on the isolated rig). From there the
      // braking profile leaves ≈ 4 s to the rails, the drive needs `closeTime` (3 s): the arms are
      // shut about a second before the pins arrive.
      const inCorridor = Math.abs(rel.x) < 6 && rel.y < 8 && pz > 2 && pz < v.pinX - 2 && vHoriz < T.armCloseV && tRail > TOWER.closeTime
      if (mem.phase !== "flip" && !arms && ((inCorridor && armNear) || (tRail <= lead && xBeyond < T.armCloseLat && armNear && Math.abs(yClose) < T.armCloseAlong && Math.abs(n.velRel.x) < T.armCloseAcrossV && vHoriz < T.armCloseV))) {
        arms = true
        mem = { ...mem, armsClose: true }
        ev.push(inCorridor
          ? `chopsticks closing: ship between the arms, pins ${pz.toFixed(1)} m above the rails, offset ${Math.hypot(rel.x, rel.y).toFixed(2)} m — closing now`
          : `chopsticks closing: pins ${pz.toFixed(1)} m above the rails (${tRail.toFixed(1)} s), predicted offset at shut ${Math.hypot(xClose, yClose).toFixed(2)} m (across ${xClose.toFixed(2)})`)
      }
      // A catch needs the arms closed around the ship and both pins individually loaded (a single pin
      // grazing an open rail produced false catches): each pin ≥ 15 % of the weight, total > 45 %.
      const pinWeight = n.m * 9.81
      const bothPins = bothPinsLoaded(n.pinCount, n.pinLoadMin, 0.15 * pinWeight) && n.pinLoad > 0.45 * pinWeight
      // Engine shutdown on pin contact. With the throttleable set idling at its floor (≈ 0.75 of
      // the weight) the pins never carry the catch loads while the engine runs — a ship that sat
      // on the closed arms burned its tanks dry without being caught. Both pins bearing on the
      // rails of the closed arms, slow: cut the engine; the catch is then judged on the pin loads.
      const pinsTouch = mem.armsClose && n.arms < 0.12 && bothPinsLoaded(n.pinCount, n.pinLoadMin, 0.03 * pinWeight) && Math.abs(rel.x - n.armX) < 1.0 && vHoriz < 1.0 && Math.abs(n.velRel.z) < 1.5
      if (pinsTouch && !(mem.cutT >= 0)) {
        mem = { ...mem, cutT: n.t }
        ev.push(`both pins bearing on the rails (${(n.pinLoad / pinWeight * 100).toFixed(0)} % of the weight) — engine shutdown`)
      }
      const engineCut = mem.cutT >= 0
      // a catch needs the arms actually closed (a pin can sit on an open rail — the arms are 20 m
      // apart there and one pin lands on the inner rail; that is not a catch)
      if (mem.phase !== "caught" && mem.phase !== "lost" && mem.armsClose && n.arms < 0.12 && Math.abs(rel.x - n.armX) < 1.0 && vHoriz < 1.0 && bothPins) {
        mem = to(mem, "caught", n.t, { caughtT: n.t })
        ev.push(`catch pins on the arms — engine shutdown at ${(-n.velRel.z).toFixed(2)} m/s, offset ${dHoriz.toFixed(2)} m. CAUGHT`)
      }
      const caught = mem.phase === "caught"
      // honest failure detection
      let lostNow = false
      if (!caught && nEng === 0) {
        lostNow = true
        mem = to(mem, "lost", n.t, { reason: "no sea-level engine left" })
        ev.push(`LOST: all three sea-level engines failed at ${pz.toFixed(0)} m above the rails`)
      } else if (!caught && !flipping && !stagedNow && pz < 200 && n.velRel.z > 3) {
        lostNow = true
        mem = to(mem, "lost", n.t, { reason: "landing missed the arms (throttle floor: no hover)" })
        ev.push(`LOST: climbing away at ${n.velRel.z.toFixed(1)} m/s, pins ${pz.toFixed(0)} m above the rails, ${dHoriz.toFixed(0)} m off (${nEng} × Raptor)`)
      } else if (!caught && pz < -3) {
        lostNow = true
        mem = to(mem, "lost", n.t, { reason: `landing missed the arms by ${dHoriz.toFixed(0)} m` })
        ev.push(`LOST: pins passed the rail plane ${dHoriz.toFixed(1)} m from the catch point (across ${rel.x.toFixed(1)}, along ${rel.y.toFixed(1)}) at ${(-n.velRel.z).toFixed(1)} m/s; arms open ${n.arms.toFixed(2)} centre ${n.armX.toFixed(2)} (rel ${(rel.x - n.armX).toFixed(2)}), pins on rails ${n.pinCount}, load ${(n.pinLoad / (n.m * 9.81) * 100).toFixed(0)} % / min ${(n.pinLoadMin / (n.m * 9.81) * 100).toFixed(0)} %`)
      }
      if (n.propHeader < 50 && !caught && !lostNow) {
        if (mem.phase !== "lost") mem = to(mem, "lost", n.t, { reason: "landing propellant exhausted" })
        ev.push(`LOST: landing propellant exhausted at ${pz.toFixed(0)} m above the rails (${(-n.velRel.z).toFixed(0)} m/s)`)
      }
      // attitude feed-forward: the aerodynamic moment of the landing database at the real α (the
      // entry table stops at 95°; tail-first it would feed forward the wrong moment)
      const ffMoment = la ? qrotInv(n.q, landingAero(la, rho, bX, n.vAir, n.xcg).M) : base.ffMoment
      const gl = { ...GAINS.landing, wn: T.landingWn, zeta: T.landingZeta, wMax: T.landingWMaxDeg * DEG, wnRoll: T.landingWnRoll, rollMax: T.landingRollMaxDeg * DEG }
      return {
        cmd: {
          ...base, att, feedForward: false, ffMoment,
          gains: flipping
            ? { ...GAINS.flip, wn: T.flipWn, zeta: T.flipZeta, wMax: T.flipWMaxDeg * DEG, wnRoll: T.flipWnRoll, rollMax: T.flipRollMaxDeg * DEG }
            : nEng >= 3
              ? { ...gl, axisAccel: [0.5, 0.44, 0.44] }
              : nEng === 1
                // one gimbal: pitch/yaw only; roll on the cold-gas couples (short pulses), conservative rates
                // Bandwidth is set by the single gimbal: ≈ 8 MN·m (0.2 rad/s²) and 20°/s of slew. The
                // two-engine gains asked for the whole gimbal travel at 5°/s of rate error and the
                // loop limit-cycled stop to stop (measured: lateral speed swinging ±20 m/s, roll
                // −33°/s from the tangential gimbal component). Slow, well-damped, small rates.
                // The attitude loop is one half of the pin-position loop's pole placement
                // (`pinLoop`: ω = √6·p, ζ = 2/√6). The rate profile's braking authority is the
                // plant's own, b·δ_avail with b = T·L/I and the gimbal travel left beside the trim
                // angle (measured 0.11 rad/s² at the floor, 0.30 at full thrust).
                ? (() => {
                  const L = Math.max(n.xcg - keepEng.gimbal.x, 1)
                  const b = (n.accThrust * n.m * L) / n.I.y
                  const avail = keepEng.gimbalMax - Math.atan(Math.hypot(keepEng.gimbal.y, keepEng.gimbal.z) / L)
                  const aAx = clamp(0.7 * b * avail, 0.03, 0.3)
                  const pl = pinLoop()
                  // roll: the rate the loop may command is what the cold-gas couples can stop again
                  // (their torque over I_xx ≈ 1.3°/s²) — with the 0.3 rad/s² it assumed before, a
                  // 12° error was turned at 4°/s and overshot by 4°
                  const tRoll = Math.max(...v.couples.map((cp) => Math.abs(v.thrusters[cp.a].thrust * (v.thrusters[cp.a].pos.z * v.thrusters[cp.a].dir.y - v.thrusters[cp.a].pos.y * v.thrusters[cp.a].dir.z) * 2)))
                  const aRoll = clamp((0.8 * tRoll) / n.I.x, 0.005, 0.3)
                  return { ...gl, wn: pl.wn, zeta: pl.zeta, wMax: T.finalWMaxDeg * DEG, wnRoll: 0.5, rollMax: 10 * DEG, axisAccel: [aRoll, aAx, aAx] as const }
                })()
                : { ...gl, axisAccel: [T.landingAxisAccelX, T.landingAxisAccelYZ, T.landingAxisAccelYZ] },
          engines: caught || lostNow || engineCut ? ENGINES_OFF : cmdEngines, armsClose: arms, coil: "discharge", useRcs: oneEng && LAND.rollGas, rcsRollOnly: oneEng && LAND.rollGas, flapsDeployed: false, rollFirst: false,
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
/**
 * Continuous flip azimuth along the velocity change the landing needs: the horizontal velocity after
 * the flip should be −p/τ (toward the catch point, τ ≈ the landing time, capped), so the flip's
 * thrust kick is pointed along Δv = v_des − v_h. The old law rounded the pad bearing to the nearest
 * of N/E/S/W (up to 45° off) and ignored the velocity the ship already had: the kick added ≈ 40 m/s
 * of horizontal speed in an arbitrary direction (measured 36 → 73 m/s across the flip). The pins
 * are aligned by roll after the flip (differential gimbal), not by the flip azimuth.
 */
const FLIP_TAU = 18
const flipAzimuth = (px: number, py: number, psiNow: number, vx = 0, vy = 0): number => {
  let dx = -px / FLIP_TAU, dy = -py / FLIP_TAU
  const dn = Math.hypot(dx, dy)
  if (dn > 40) { dx *= 40 / dn; dy *= 40 / dn }
  const ex = dx - vx, ey = dy - vy
  if (Math.hypot(ex, ey) < 3) return psiNow
  const q = Math.atan2(ex, ey)
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
  /** measured/table aero force ratio (measured aero calibration); 1 = the table as built */
  readonly aeroScale?: number
  /** measured-minus-model aero acceleration (ENU, m/s²), fading out over the prediction */
  readonly dist?: V3
}

/**
 * One point-mass belly-flop integration endpoint with the two-segment lift-azimuth law. This is the
 * flight's own `steepEnd` physics, exported so the fast evaluator runs exactly the same law (no
 * duplicated integrator). `windOfs` is zero for the predictive evaluator.
 */
export const bellyEnd = (b: BellyInput, psi1: number, tau: number, psi2: number, psi0: number, hEnd: number, rate: number, alignH: number, dtStep = 0.5, out?: { x: number; y: number; h: number }[]): { x: number; y: number; h: number; vx: number; vy: number; vz: number; t: number; psi: number } => {
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
    const tgt = h < hEnd + alignH ? flipAzimuth(px, py, psi, vx, vy) : t < tau ? psi1 : psi2
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
    const L = (q * S * co.CL * (b.aeroScale ?? 1)) / b.mass, D = (q * S * co.CD * (b.aeroScale ?? 1)) / b.mass
    // residual from the observer: trusted near-term, faded with a 40 s time constant
    const kd = b.dist ? Math.exp(-t / 40) : 0
    vx += (-D * ux + L * lx + kd * (b.dist?.x ?? 0)) * dt
    vy += (-D * uy + L * ly + kd * (b.dist?.y ?? 0)) * dt
    vz += (-D * uz + L * lz - 9.79 + kd * (b.dist?.z ?? 0)) * dt
    px += vx * dt; py += vy * dt; h += vz * dt
    t += dt
    if (out) out.push({ x: px, y: py, h })
  }
  return { x: px, y: py, h, vx, vy, vz, t, psi }
}

const steepEnd = (ctx: GuideCtx, n: Nav, mem: Mem, psi1: number, tau: number, psi2: number, psi0: number, hEnd: number, rate: number, alignH: number): { x: number; y: number; h: number; vx: number; vy: number; vz: number; t: number; psi: number } =>
  bellyEnd({ pinRel: n.pinRel, velRel: n.velRel, hBase: n.hBase, mass: n.m, windOfs: mem.windOfs, aero: ctx.aero, sref: ctx.pred.sref, windForecast: ctx.windForecast, aeroScale: mem.aeroScale, dist: mem.bellyDist }, psi1, tau, psi2, psi0, hEnd, rate, alignH)

/**
 * Landing capability used by the belly planner (the "start the descent plan early" fix).
 *
 * The belly phase used to aim only at a *position* at the flip. The descent after the flip also has
 * to kill the lateral *velocity*, and that is what consumed the 1.4 km divert authority. So the
 * belly objective is now the zero-effort miss of the landing: the horizontal offset it would still
 * have to remove after `tLand` seconds of descent, `p + v·tLand`, plus a penalty for a lateral speed
 * the descent cannot null. The landing lateral authority is taken at a conservative 2-engine, 45°
 * tilt value (≈ 8 m/s²); `vMax` is the lateral speed the descent can remove while still holding
 * position authority.
 */

/**
 * End-to-end miss of a belly plan: the *descent* is flown too, not just the flip.
 *
 * The belly used to be scored on the horizontal offset it would carry into the flip, `p + v·tLand`
 * with a nominal lateral authority. This ranks candidate plans; the *decision* is taken with the
 * vehicle model plus the real terminal law (`evalPlan` in the belly correction).
 */
const LAND_E2E = { tLand: 15, aLat: 8, vMax: 30, wX: 1.4 } as const
const endToEndMiss = (T: MissionTuning, e: { readonly x: number; readonly y: number; readonly vx: number; readonly vy: number }): number => {
  const px = e.x - T.bellyAimE + e.vx * LAND_E2E.tLand
  const py = e.y - T.bellyAimN + e.vy * LAND_E2E.tLand
  const vh = Math.hypot(e.vx, e.vy)
  const excess = Math.max(0, vh - LAND_E2E.vMax)
  const need = (2 * Math.hypot(px, py)) / (LAND_E2E.tLand * LAND_E2E.tLand)
  const shortfall = Math.max(0, need - LAND_E2E.aLat) * LAND_E2E.tLand * LAND_E2E.tLand * 0.5
  return Math.hypot(px * LAND_E2E.wX, py) + 2 * excess + shortfall
}

/**
 * Delivery target of the belly phase, at flip completion: the flip-and-brake is a fixed manoeuvre of
 * the ship's own motion, and the single-engine descent then stops the remaining horizontal speed in
 * `stop` metres along the track. The ship must therefore finish the flip that far before the catch
 * axis, on the track through it. Measured capture set of the descent: ± 80 m around this point.
 */
export const DELIVERY = { epoch: 30, start: 2, tol: 120, lastAbove: 6000, gain: 0.6, trust: 3000, aStop: 4.0, flipLead: 10, fine: true, fineTol: 2, fineRollouts: 4, flipCarry: 15.3, fineBelow: 250, fineEpoch: 12, fineAbove: 2200 } as const

/** miss vector of a flip-complete state against the delivery target (pad ENU, m) */
export const deliveryMiss = (e: RolloutEnd): { x: number; y: number } => {
  const vh = Math.hypot(e.velRel.x, e.velRel.y)
  // the descent's own stopping distance along the track: v²/(2a), a ≈ 3 m/s² of lateral braking
  // flip-and-stop: the flip ends at rest, so the delivery target is the catch axis itself (plus the
  // little the residual speed still carries)
  if (LAND.flip.stop) return { x: e.pinRel.x + 1.5 * e.velRel.x, y: e.pinRel.y + 1.5 * e.velRel.y }
  const stop = clamp((vh * vh) / (2 * DELIVERY.aStop), 60, 420)
  return vh > 1 ? { x: e.pinRel.x + (stop * e.velRel.x) / vh, y: e.pinRel.y + (stop * e.velRel.y) / vh } : { x: e.pinRel.x, y: e.pinRel.y }
}

const steepGlide = (ctx: GuideCtx, n: Nav, mem0: Mem, dt: number, ev: string[] = [], rollout?: Rollout): { x: V3; z: V3; mem: Mem } => {
  const T = ctx.tuning ?? MISSION_TUNING_DEFAULT
  // One constant heading rate for the plan and the flight. The plan used to evaluate `bankRateNow`
  // once (at the belly-flop start, high q: 1.88°/s) and assume it for 130 s, while the flight
  // recomputed it every step and it fell with q (measured average 1.15°/s): the executed heading
  // lagged the plan by tens of degrees and the end point moved ≈ 2.2 km. The cold-gas belly rate is
  // available at every q of the belly flop and the attitude loop follows it within ≈ 8°.
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
  // Disturbance observer: the measured aero acceleration (nav: aero force estimate, attitude) minus
  // the predictor's model at the commanded heading. Everything the 3-DOF model does not capture —
  // the attitude loop's lag behind the heading, sideslip, bell-fin and RCS forces, table error —
  // shows up here and is fed to the plan (low-passed, τ 4 s).
  {
    const V = norm(n.vAir)
    if (V > 20) {
      const fE = qrot(n.q, n.aeroF)
      const meas = v3(dot(fE, n.east) / n.m, dot(fE, n.north) / n.m, dot(fE, n.up) / n.m)
      const va = v3(dot(n.vAir, n.east), dot(n.vAir, n.north), dot(n.vAir, n.up))
      const at0 = atmosphere(Math.max(n.alt, 0))
      const co = aeroLookup(ctx.aero, passiveTrim(ctx.aero, V / at0.a), V / at0.a)
      const qd = 0.5 * at0.rho * V * V
      const ux = va.x / V, uy = va.y / V, uz = va.z / V
      const hx = Math.sin(hc), hy = Math.cos(hc)
      const dd = hx * ux + hy * uy
      let lx = hx - dd * ux, ly = hy - dd * uy, lz = -dd * uz
      const ln = Math.hypot(lx, ly, lz) || 1
      lx /= ln; ly /= ln; lz /= ln
      const sc = mem.aeroScale
      const Lm = (qd * ctx.pred.sref * co.CL * sc) / n.m, Dm = (qd * ctx.pred.sref * co.CD * sc) / n.m
      const model = v3(-Dm * ux + Lm * lx, -Dm * uy + Lm * ly, -Dm * uz + Lm * lz)
      const k = clamp(dt / 4, 0, 1)
      const r = sub(meas, model)
      const lim = 4
      mem = { ...mem, bellyDist: v3(clamp(mem.bellyDist.x + k * (r.x - mem.bellyDist.x), -lim, lim), clamp(mem.bellyDist.y + k * (r.y - mem.bellyDist.y), -lim, lim), clamp(mem.bellyDist.z + k * (r.z - mem.bellyDist.z), -lim, lim)) }
    }
  }
  // Bank actually flown about v̂ (same construction as planBank's sFlown). The body lags the heading
  // command by 30–46° in the belly (the cold gas + bells are the actuator), so a plan evaluated from
  // the *command* is optimistic by exactly that lag: it schedules a reversal the body cannot have
  // reached. Evaluate every candidate from the flown lift azimuth instead.
  const lbF = sub(thrDirBody(n), scale(vh, dot(thrDirBody(n), vh)))
  const hcFlown = norm(lbF) > 1e-6 ? Math.atan2(dot(lbF, n.east), dot(lbF, n.north)) : hc
  if (mem.heading !== null && n.t >= mem.tSwitch) mem = { ...mem, heading: mem.sigma2, tSwitch: Infinity }
  // ---- rollout-corrected plan (flight). The fast point-mass model ranks the candidate plans; the
  // vehicle model decides where a plan really ends. At each epoch the plan being flown is rolled
  // out to flip completion, its delivery miss m is measured, and the fast model's target is moved
  // to (its own end point for that plan) − m: defect correction. The plan is re-optimised only at
  // these epochs — event-triggered, a handful of rollouts for the whole belly flop.
  // ---- the plan search on the fast point-mass model (used by the flight's first plan and by the
  // prediction rollouts that start before the belly flop)
    const D = Math.PI / 180
    const end3 = (a: number, tau: number, b: number) => steepEnd(ctx, n, mem, a, tau, b, hc, hFlip, bellyRate, T.bellyAlignH)
    const search = (tgt: { x: number; y: number } | null, seed: { a: number; tau: number; b: number } | null): { a: number; tau: number; b: number; e: number } => {
      const miss = (a: number, tau: number, b: number): number => {
        const e = end3(a, tau, b)
        if (tgt) return LAND.flip.stop ? Math.hypot(e.x + DELIVERY.flipCarry * e.vx - tgt.x, e.y + DELIVERY.flipCarry * e.vy - tgt.y) : Math.hypot(e.x - tgt.x, e.y - tgt.y)
      // flip-and-stop: the flip carries the ship ≈ `flipCarry` seconds of its drift further (the
      // kick is along the nose, and the drift follows the nose: measured 543 m at 35.6 m/s), then
      // it is at rest — that point must be the catch axis
      if (LAND.flip.stop) return Math.hypot(e.x - T.bellyAimE + DELIVERY.flipCarry * e.vx, e.y - T.bellyAimN + DELIVERY.flipCarry * e.vy)
      return endToEndMiss(T, e)
      }
      interface Plan { readonly a: number; readonly tau: number; readonly b: number; readonly e: number }
      const pick = (ps: readonly Plan[]): Plan => ps.reduce((x, y) => (y.e < x.e ? y : x))
      // A seeded correction is local: a global search every epoch re-anchors the heading command
      // and the body (1.8°/s) never settles — measured, four epochs picked ψ −139 → −138 → −6 → 5°
      // and the delivery stayed ≈ 450 m. Only the first plan is a global search.
      const heads = seed ? [seed.a] : Array.from({ length: 12 }, (_, i) => hc + (i - 6) * 30 * D)
      const taus = seed ? [seed.tau] : [8, 16, 32, 50, 70, 95, 125]
      let best = pick([
        ...heads.map((a) => ({ a, tau: seed ? seed.tau : Infinity, b: seed ? seed.b : a, e: miss(a, seed ? seed.tau : Infinity, seed ? seed.b : a) })),
        ...(seed ? [] : heads.flatMap((a) => [45, 90, 180, -45, -90].flatMap((db) => taus.map((tau) => ({ a, tau, b: a + db * D, e: miss(a, tau, a + db * D) }))))),
      ])
      // local refinement (coordinate pattern search, shrinking steps)
      for (const [sa, sb, st] of [[10, 15, 0.25], [4, 6, 0.1], [1.5, 2, 0.04], [0.5, 0.7, 0.015]] as const) {
        for (let it = 0; it < 3; it++) {
          const b0 = best
          const c = [[sa, 0, 1], [-sa, 0, 1], [0, sb, 1], [0, -sb, 1], [0, 0, 1 + st], [0, 0, 1 - st]]
            .map(([da, dbb, k]) => ({ a: b0.a + da * D, b: b0.b + dbb * D, tau: Number.isFinite(b0.tau) ? b0.tau * k : b0.tau }))
            .map((x) => ({ ...x, e: miss(x.a, x.tau, x.b) }))
          best = pick([best, ...c])
          if (best === b0) break
        }
      }
      return best
    }
  const corrected = rollout !== undefined && !mem.predicting
  if (mem.predicting) {
    // prediction rollout: a given plan is frozen. A rollout that started before the belly flop has
    // no plan: it flies the pursuit law — lift towards the catch axis from the point where the
    // horizontal motion would die out under drag (p + v_h·τ, τ = V / a_aero).
    if (mem.bellyPursuit && LAND.flip.stop) {
      // A rollout that started before the belly flop plans the belly flop the way the flight
      // will (the same fast-model search, once, at its start). It used to fly a pursuit proxy
      // instead, so the hand-over correction was steering a manoeuvre the flight never flies:
      // its "delivery miss" stayed at 0.7–2.9 km and the belly flop inherited a state it could
      // not recover from (measured: every candidate plan ended 2.1 km south of the axis).
      if (mem.heading === null) {
        const p0 = search(null, null)
        mem = { ...mem, heading: p0.a, tSwitch: n.t + p0.tau, sigma2: p0.b }
      }
    } else if (mem.bellyPursuit) {
      const aA = Math.max(norm(n.aeroF) / n.m, 3)
      const tau = clamp(norm(n.vAir) / aA, 5, 60)
      const sx = n.pinRel.x - T.bellyAimE + n.velRel.x * tau, sy = n.pinRel.y - T.bellyAimN + n.velRel.y * tau
      if (Math.hypot(sx, sy) > 100) mem = { ...mem, heading: Math.atan2(-sx, -sy), tSwitch: Infinity }
    }
  } else if (corrected) {
    const rollN = mem.rollN ?? 0
    // Epochs: every `epoch` seconds down to `lastAbove` metres above the flip; once the rolled-out
    // miss is small the plan is polished at shorter intervals and lower down (`fineEpoch`,
    // `fineAbove`): the rollouts are short there (≈ 0.05 s) and a late correction of a few metres
    // needs little heading authority.
    const fineStage = DELIVERY.fine && (mem.rollMiss ?? Infinity) < DELIVERY.fineBelow
    // a plan that already delivers inside the tolerance is left alone (no further rollouts)
    const settled = DELIVERY.fine && (mem.rollMiss ?? Infinity) < DELIVERY.fineTol
    const due = mem.heading === null || (!settled && n.t - (mem.rollT ?? -1e9) >= (fineStage ? DELIVERY.fineEpoch : DELIVERY.epoch))
    if (due && n.hBase > hFlip + (fineStage ? DELIVERY.fineAbove : DELIVERY.lastAbove)) {
      let cur = mem.heading === null ? null : { a: mem.heading, tau: mem.tSwitch - n.t, b: mem.sigma2 }
      if (!cur) { const p0 = search(mem.bellyTgt ?? null, null); cur = { a: p0.a, tau: p0.tau, b: p0.b } }
      // Choose the plan with the vehicle model: evaluate the plan and a few heading neighbours by
      // rollout (≈ 0.15 s each) and keep the best by a margin. The heading change is bounded (the
      // command slews at 1.8°/s, so a big jump spends tens of seconds pointing the lift the wrong
      // way) and a new plan must be clearly better, else the correction limit-cycles: measured
      // epochs jumping ψ −139 → −138 → −6 → 5° left the delivery at ≈ 450 m.
      let tgt = mem.bellyTgt ?? null
      let n6 = 0
      let end6: RolloutEnd | null = null
      const evalPlan = (p: { a: number; tau: number; b: number }): { x: number; y: number } | null => {
        // start the belly command where the flight actually is: it slews to the plan's heading at
        // 1.8°/s, so a rollout that starts *at* the plan's heading is optimistic by the slew time
        const e6 = rollout({ ...mem, hdgCmd: mem.hdgCmd ?? p.a, heading: p.a, tSwitch: n.t + p.tau, sigma2: p.b }, "landing")
        n6++
        if (e6 && e6.phase === "landing") end6 = e6
        if (!e6 || e6.phase !== "landing") return null
        // Score the delivery point (where the horizontal motion dies out along the track). Scoring
        // the *descent* here was tried: the terminal law's point-mass model is optimistic (it can
        // use 4 m/s² the vehicle does not have), so it ranked plans by an end point the ship could
        // not reach (measured: 193 m predicted, 570 m flown, catch lost). The belly aims at the
        // stop point; the descent is the flight's own problem and is judged in the rollouts.
        return deliveryMiss(e6)
      }
      const m0 = evalPlan(cur)
      // rollout end state of the plan that is kept (the tail of the displayed path)
      let endCur: RolloutEnd | null = m0 ? end6 : null
      let mBest: { x: number; y: number } | null = null
      let missFlown = m0 ? Math.hypot(m0.x, m0.y) : NaN
      if (m0 && missFlown > DELIVERY.tol && !DELIVERY.fine) {
        let best = cur, bestD = missFlown
        // first plan: a wide vehicle-model scan of the heading (13 rollouts, ≈ 2 s). The point-mass
        // model's ranking is not the vehicle's, so the committed heading is chosen by flying it.
        for (const da of [-30, -15, 15, 30]) {
          const p2 = { a: cur.a + da * D, tau: cur.tau, b: cur.b }
          const m2 = evalPlan(p2)
          if (m2) {
            const d = Math.hypot(m2.x, m2.y)
            if (d < bestD * 0.9) { best = p2; bestD = d; mBest = m2; endCur = end6 }
          }
        }
        if (best !== cur) { cur = best; missFlown = bestD; tgt = null }
      }
      // Fine optimisation: defect correction ("space mapping"). The fast point-mass model is wrong
      // about where a plan ends (it has no attitude lag, no flip) but right about how the end point
      // *moves* when the plan changes, and it costs microseconds; the 6-DOF rollout is exact and
      // costs 0.15 s. So: fly the current plan on the vehicle model, measure its delivery miss m,
      // ask the fast model for the nearby plan whose own stop point is shifted by −m, fly that one,
      // keep it if it is better (halve the step if not). A few rollouts per epoch instead of a
      // blind search — the scan above alone left ≈ 90 m at the flip, and with no wind the rollout
      // *is* the flight, so that residual was the optimiser's resolution, not a model error.
      // A one-heading plan is opened into two segments first: one number cannot null a 2-D miss.
      if (DELIVERY.fine && m0) {
        let mVec = mBest ?? m0
        const tRem = Math.max((n.hBase - hFlip) / Math.max(-n.velRel.z, 30), 10)
        if (!Number.isFinite(cur.tau)) cur = { a: cur.a, tau: 0.5 * tRem, b: cur.a }
        let gain = 1
        const dbg = (globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG
        for (let it = 0; it < DELIVERY.fineRollouts && missFlown > DELIVERY.fineTol && gain > 0.3; it++) {
          const ef = end3(cur.a, cur.tau, cur.b)
          const sf = { x: ef.x + DELIVERY.flipCarry * ef.vx, y: ef.y + DELIVERY.flipCarry * ef.vy }
          const pN = search({ x: sf.x - gain * mVec.x, y: sf.y - gain * mVec.y }, cur)
          const cand = { a: pN.a, tau: pN.tau, b: pN.b }
          const m2 = evalPlan(cand)
          if (!m2) break
          const d = Math.hypot(m2.x, m2.y)
          if (dbg) dbg(`defect t ${n.t.toFixed(0)} it ${it} gain ${gain}: ψ1 ${(cand.a / D).toFixed(1)}° τ ${cand.tau.toFixed(1)} ψ2 ${(cand.b / D).toFixed(1)}° fast residual ${pN.e.toFixed(0)} m → rollout miss ${d.toFixed(1)} m (was ${missFlown.toFixed(1)})`)
          if (d < missFlown) { cur = cand; mVec = m2; missFlown = d; gain = 1; endCur = end6 } else gain *= 0.5
        }
        tgt = null
      }
      // The 2-D panel's plan: the belly flop to the flip gate (the plan's own fast model, with the
      // path recorded), a straight segment for the flip itself, then the field law's descent from the
      // rolled-out flip-complete state to the rails. One continuous line from here to the tower.
      const e3p = end3(cur.a, cur.tau, cur.b)
      const bellyPath: { x: number; y: number; h: number }[] = []
      bellyEnd({ pinRel: n.pinRel, velRel: n.velRel, hBase: n.hBase, mass: n.m, windOfs: mem.windOfs, aero: ctx.aero, sref: ctx.pred.sref, windForecast: ctx.windForecast, aeroScale: mem.aeroScale, dist: mem.bellyDist }, cur.a, cur.tau, cur.b, hc, hFlip, bellyRate, T.bellyAlignH, 0.5, bellyPath)
      // (`predPath` is in the pad frame with U from the ground: the pin height above the rails plus
      // the arm height — the belly path used to be stored without it and was drawn 112 m low.)
      const zPin = (h: number): number => h - (ctx.armHeight - ctx.vehicle.pinX)
      const path: V3[] = bellyPath.map((q) => v3(q.x, q.y, Math.max(zPin(q.h), 0) + ctx.armHeight))
      // the tail is the flown plan's own rollout end (not the last candidate tried) …
      const endPlan = endCur
      if (endPlan) {
        const e6: RolloutEnd = endPlan
        const last = path[path.length - 1] ?? v3(0, 0, ctx.armHeight)
        // … a straight segment for the flip's carry, and the single-engine descent law integrated
        // forward from the flip-complete state (the same law the flight will fly)
        path.push(v3(last.x + e6.pinRel.x - e3p.x, last.y + e6.pinRel.y - e3p.y, Math.max(e6.pinRel.z, 0) + ctx.armHeight))
        const pr = slamPredict(0, {
          rel: e6.pinRel, vPin: v3(e6.velRel.x, e6.velRel.y, 0), vz: e6.velRel.z, tilt: { e: 0, n: 0 }, tiltRate: { e: 0, n: 0 },
          aNow: 7.2, mass: n.m - 9e3, ve: 3620, pAe: 1.3e5, slew: 11.8,
        }, { ez: 0, lo: 7.2, hi: 19.6, contactSpeed: T.contactSpeed }, 90, 0.25)
        for (let k = 0; k < pr.x.length; k++) path.push(v3(pr.x[k], pr.y[k], pr.z[k] + ctx.armHeight))
      }
      mem = { ...mem, heading: cur.a, tSwitch: n.t + cur.tau, sigma2: cur.b, bellyTgt: tgt, bellyMiss: missFlown, rollT: n.t, rollN: rollN + n6, rollMiss: missFlown, npcT: n.t, predPath: path }
      ev.push(`belly plan (vehicle-model rollout ${rollN + n6}): heading ${(cur.a / DEG).toFixed(0)}°${Number.isFinite(cur.tau) ? ` → ${(cur.b / DEG).toFixed(0)}° after ${cur.tau.toFixed(0)} s` : ""}; rolled-out delivery miss at flip completion ${Number.isFinite(missFlown) ? missFlown.toFixed(0) + " m" : "n/a"}`)
    }
  } else if (n.hBase > hFlip + 100 && n.t - mem.npcT > 2 && !((globalThis as { BELLY_FREEZE?: boolean }).BELLY_FREEZE && mem.heading !== null)) {
    const D = Math.PI / 180
    interface Plan { readonly a: number; readonly tau: number; readonly b: number; readonly e: number }
    // The plan minimises the true flip miss (the T09 gate). The across-arms (E–W) component is the
    // landing's weak axis and is reported separately; weighting it in the objective traded 270 m of
    // north–south error for 100 m of east–west and made the total worse (measured 881 -> 975 m).
    const miss = (a: number, tau: number, b: number): number => endToEndMiss(T, steepEnd(ctx, n, mem, a, tau, b, hc, hFlip, bellyRate, T.bellyAlignH))
    const pick = (ps: readonly Plan[]): Plan => ps.reduce((x, y) => (y.e < x.e ? y : x))
    const heads = Array.from({ length: 12 }, (_, i) => hc + (i - 6) * 30 * D)
    let best = pick(heads.map((a) => ({ a, tau: Infinity, b: a, e: miss(a, Infinity, a) })))
    if (best.e > 40) {
      // T09 hand-over: the terminal descent is ~160 s, so a plan whose first segment is capped at 32 s
      // cannot express "brake the horizontal velocity early, then steer"; it crossed the pad at ~200 m/s
      // and stopped 4 km past it. The tau grid reaches the full descent, with +/-45 deg steering steps.
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
    // Continuation: keep the plan being flown unless a new one is clearly better. Re-plans that jump
    // between near-equal local optima (measured: ψ1 −117° → −12° → −99° → −48° → −318° in 30 s)
    // mean no plan is ever executed — the heading slews toward a different target every cycle.
    const tgt = cur && cur.e < Math.max(best.e * 1.3, best.e + 150) ? cur : best
    const eEnd = steepEnd(ctx, n, mem, tgt.a, tgt.tau, tgt.b, hc, hFlip, bellyRate, T.bellyAlignH)
    mem = { ...mem, heading: tgt.a, tSwitch: n.t + tgt.tau, sigma2: tgt.b, bellyMiss: endToEndMiss(T, eEnd), npcT: n.t }
    const dbgB = (globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG
    if (dbgB) dbgB(`belly plan t ${n.t.toFixed(1)} from (${n.pinRel.x.toFixed(0)},${n.pinRel.y.toFixed(0)},h ${n.hBase.toFixed(0)}) v (${n.velRel.x.toFixed(0)},${n.velRel.y.toFixed(0)},${n.velRel.z.toFixed(0)}): end (${eEnd.x.toFixed(0)},${eEnd.y.toFixed(0)}) h ${eEnd.h.toFixed(0)} v (${eEnd.vx.toFixed(1)},${eEnd.vy.toFixed(1)},${eEnd.vz.toFixed(1)}) t ${eEnd.t.toFixed(0)} s; ψ0 ${(hcFlown / DEG).toFixed(0)} rate ${(bellyRate / DEG).toFixed(2)}°/s hEnd ${hFlip.toFixed(0)} catch ${endToEndMiss(T, eEnd).toFixed(0)} m`)
  }
  // catch alignment: the flip pitches about body y, so body z ends along the pre-flip nose azimuth.
  // In the last ≈ 1.5 km the nose swings to north or south (nearer one): after the flip the engine
  // side (+z) is N–S, the pins (±y) face the east/west arm rails, and the divert from the arms'
  // open (south) side uses the strong gimbal-pitch plane.
  const align = n.hBase < hFlip + T.bellyAlignH
  // A pursuit law (lift towards the stop point p + v_h·τ) was tried here as the flight law and
  // rejected: at the belly-flop start the ship is ~4 km downrange with 500 m/s, the stop point is
  // out of reach, and steering at it spent the whole divert authority on the unreachable range
  // (measured 10.9 km delivery versus 1.1 km for the planned heading). It remains only as the
  // proxy the pre-belly rollouts use (`bellyPursuit`), where no plan exists yet.
  // Flip-and-stop: in the alignment window the nose is pointed at the catch axis *from where the
  // flip will ignite* (p + v·t_ign): the flip carries the ship along its nose azimuth, so a nose
  // that points at the axis at ignition puts the stop point on the line through it — the
  // cross-track error is steered out here, the along-track error by the ignition timing.
  const tIgn = clamp((n.hBase - (ctx.armHeight - ctx.vehicle.pinX + flipAltitude(n.velRel.z, T))) / Math.max(-n.velRel.z, 30), 0, 60)
  const pIx = n.pinRel.x + n.velRel.x * tIgn, pIy = n.pinRel.y + n.velRel.y * tIgn
  const tgtPsi = align
    ? LAND.flip.stop && LAND.flip.window > 0
      ? (Math.hypot(pIx, pIy) > 60 ? Math.atan2(-pIx, -pIy) : hc)
      : flipAzimuth(n.pinRel.x, n.pinRel.y, hc, n.velRel.x, n.velRel.y)
    : mem.heading ?? hc
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
/** airspeed at which the coil loses steering authority in the high-α phase (measured: M 6, ≈ 1.85–1.9 km/s) */
const COIL_LOSS_V = 1900
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
/**
 * Bank rate the actuators can actually deliver (rad/s), from the moment budget:
 *   M = cold-gas couples + 3 bell fins at the gimbal limit (L·F_lat each)
 * and the inertia about the velocity vector at the current α:
 *   I_v = I_roll·cos²α + I_pitch·sin²α.
 * A torque-limited reversal of angle θ takes 2√(θ/α̈), so the equivalent linear rate is
 * ½√(M·θ/I_v); θ = 1 rad is the reference used consistently by the bank predictor.
 */
const bankRateNow = (ctx: GuideCtx, n: Nav, gasRate: number, scale = 1): number => {
  const bf = ctx.vehicle.bellFins
  if (!bf) return gasRate
  const g = ctx.vehicle.engines[0]
  const L = n.xcg - g.gimbal.x
  const V = n.V
  if (V < 1) return gasRate
  const q = n.qbar
  const beta = Math.acos(Math.min(1, Math.abs(dot(n.vAir, thrDirBody(n))) / V))
  const lat = q * bf.wakeFactor * bf.area * bf.ductFactor * 4 * Math.abs(Math.sin(beta) * Math.cos(beta))
  const mBell = 3 * L * lat * Math.sin(g.gimbalMax)
  const mGas = 0.036e6
  const ca = Math.cos(n.alpha)
  const sa = Math.sin(n.alpha)
  const Iv = n.I.x * ca * ca + n.I.y * sa * sa
  const rate = 0.5 * Math.sqrt(((mBell + mGas) * 1.0) / Math.max(Iv, 1e5))
  // Conservative plan: the bells are new hardware and the predictive bank timing is sensitive to
  // their modelled moment. Cap the rate the *plan* assumes at 1.2× the cold-gas value; the bells
  // then act as tracking margin (the attitude loop can exceed the plan) instead of making the plan
  // optimistic. This is what the pre-bell baseline did, and it is the only self-consistent setting.
  return clamp(Math.min(rate * scale, 1.2 * Math.max(gasRate, 1.2 * DEG)), 0.5 * DEG, 14 * DEG)
}

/** high-α hand-over correction: rollout epochs start inside `range` m, every `epoch` s; gain and bound of the aim shift */
/**
 * `tries`: rollouts per check (the current aim, then candidate shifts). Three were measured more
 * robust over neighbouring tuning points (5 of 12 delivered against 2 of 6 with two), but each
 * costs ≈ 1.5 s and the whole mission has 30 s; with the flown tuning both give the identical
 * flight (the third rollout never produced the accepted aim), so two are flown.
 */
export const HANDOVER = { range: 330e3, epoch: 50, tries: 2, atIgnition: false, gain: 0.85, max: 20e3, tol: 60 } as const

const planBank = (ctx: GuideCtx, n: Nav, mem0: Mem, dt: number, phaseP: PredPhase = "highAlpha", ev: string[] = [], rollout?: Rollout): { sigma: number; mem: Mem; event?: string } => {
  const T = ctx.tuning ?? MISSION_TUNING_DEFAULT
  let mem = mem0
  // ---- hand-over correction by rollout. The fast predictor ends the belly flop at its zero-lift
  // footprint centre; the vehicle does not (the lift azimuth turns at 1.8°/s, the Mach 2 →
  // transonic transition is not in the point-mass model): measured, the belly flop started 5 km
  // off with the predictor reporting 0.2 km. At each epoch the vehicle model is flown from here to
  // flip completion (this bank plan, then the belly pursuit proxy — no belly plan exists yet at
  // hand-over, so the proxy stands in for it); its delivery miss, in the
  // predictor's downrange/crossrange axes, is taken out of the predictor's aim.
  if (phaseP === "highAlpha" && rollout && !mem.predicting && norm(horiz(sub(n.padEci, n.r), n.up)) < HANDOVER.range && n.t - (mem.haRollT ?? -1e9) >= HANDOVER.epoch && n.mach > 2.3) {
    // Verified step (trust region). The aim shift is *tried on the vehicle model before it is
    // flown*: roll out the current shift, roll out the corrected one, keep the correction only if
    // its predicted delivery is better, otherwise try half of it. The unverified fixed-gain step
    // it replaces was not monotone — after the last bank reversal the downrange response changes
    // sign with the |σ| solution — and one late check moved the delivery from 1.1 km to 3.3 km
    // (measured), which the belly flop cannot recover.
    const b0 = mem.haBias ?? { dr: 0, cr: 0 }
    const hl = Math.hypot(n.pinRel.x, n.pinRel.y) || 1
    const hx = -n.pinRel.x / hl, hy = -n.pinRel.y / hl
    const tryBias = (b: { dr: number; cr: number }): { dr: number; cr: number; mm: number } | null => {
      // The hand-over is judged by flying through the flip to its completion. Judging it at flip
      // ignition with the carry model (`atIgnition`) was tried to decouple it from the landing
      // code: 2 of 9 tuning points caught against 6 of 7 flown through, so the flip stays in.
      const e6 = rollout({ ...mem, haBias: b, npcT: -1e9, bellyPursuit: true }, HANDOVER.atIgnition ? "flip" : "landing")
      if (!e6 || (e6.phase !== "landing" && e6.phase !== "flip")) return null
      const m = e6.phase === "flip" && e6.nose
        ? { x: e6.pinRel.x + LAND.flip.carry * e6.nose.e, y: e6.pinRel.y + LAND.flip.carry * e6.nose.n }
        : deliveryMiss(e6)
      return { dr: m.x * hx + m.y * hy, cr: m.x * hy - m.y * hx, mm: Math.hypot(m.x, m.y) }
    }
    const r0 = tryBias(b0)
    if (r0) {
      let bBest = b0, rBest = r0, tries = 1
      const lim = (x: number): number => clamp(x, -HANDOVER.max, HANDOVER.max)
      // Once the bank has reversed inside the hand-over range the aim is no longer searched: the
      // crossrange is then out of the bank plan's reach and the downrange response is not
      // monotone (measured: every search after that reversal ended "no better aim found" and cost
      // three rollouts). The check still flies one rollout — it is the delivery estimate.
      const monitorOnly = mem.haFirstT !== undefined && mem.revT > mem.haFirstT
      if (r0.mm > HANDOVER.tol && !monitorOnly) {
        for (const g of [HANDOVER.gain, 0.4 * HANDOVER.gain]) {
          const b = { dr: lim(bBest.dr - g * rBest.dr), cr: lim(bBest.cr - g * rBest.cr) }
          const r = tryBias(b)
          tries++
          if (r && r.mm < rBest.mm) {
            bBest = b; rBest = r
            if (rBest.mm < HANDOVER.tol || tries >= HANDOVER.tries) break
            // a second full step from the improved point, if the budget allows
            const b2 = { dr: lim(bBest.dr - HANDOVER.gain * rBest.dr), cr: lim(bBest.cr - HANDOVER.gain * rBest.cr) }
            const r2 = tryBias(b2)
            tries++
            if (r2 && r2.mm < rBest.mm) { bBest = b2; rBest = r2 }
            break
          }
          if (tries >= HANDOVER.tries) break
        }
      }
      const changed = bBest !== b0
      mem = { ...mem, haBias: bBest, haRollT: n.t, haRollN: (mem.haRollN ?? 0) + tries, haFirstT: mem.haFirstT ?? n.t, npcT: changed ? -1e9 : mem.npcT }
      ev.push(`hand-over check (vehicle-model rollout ${mem.haRollN}): delivery miss at flip completion ${r0.mm.toFixed(0)} m (downrange ${r0.dr.toFixed(0)}, crossrange ${r0.cr.toFixed(0)})${changed ? ` → ${rBest.mm.toFixed(0)} m with the predictor aim shifted to ${bBest.dr.toFixed(0)} / ${bBest.cr.toFixed(0)} m (verified by rollout)` : monitorOnly ? " — monitoring (bank already reversed)" : r0.mm > HANDOVER.tol ? " — no better aim found, kept" : ""}`)
    } else mem = { ...mem, haRollT: n.t }
  }
  const haB = phaseP === "highAlpha" ? mem.haBias ?? { dr: 0, cr: 0 } : { dr: 0, cr: 0 }
  const baseRate = phaseP === "belly" ? T.bellyBankRateDeg * DEG : mem.coilOff ? SIGMA_RATE_GAS : SIGMA_RATE
  // the 3°/s high-α slew is coil-authority pitch/yaw; once the coil is off the bank rotates on the
  // cold-gas couples and the open-base bell fins (the gimbals, engines cold). The plan must use the
  // rate the actuators can actually deliver, or a reversal looks affordable when it is not.
  const rate = phaseP === "highAlpha" && !mem.coilOff
    ? Math.max(baseRate, SIGMA_RATE_HIGH, bankRateNow(ctx, n, baseRate, mem.bankRateScale))
    : bankRateNow(ctx, n, baseRate, mem.bankRateScale)
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
  // (A prediction rollout must re-plan its bank as often as the flight does. Re-planning four
  // times less often saved 7 s of compute and made the rollouts fly open loop: every one of
  // twelve tuning points then delivered the belly flop 1–4 km short.)
  const period = revSoon ? 1.5 : phaseP === "belly" ? 3 : range > 500e3 ? 15 : range > 150e3 ? 9 : 6
  const uzNow = Math.sqrt(Math.max(0, 1 - dot(vhF, n.up) ** 2)) // vertical component of the in-plane "up"
  const grid = BANK_GRID.filter((m) => uzNow * Math.cos(m) >= LIFT_FLOOR)
  if (n.hBase > hFlip + 150 && n.t - mem.npcT > period) {
    const s = mem.bankSign
    // Coil loss in the prediction: the coil's 3°/s bank slew and its MHD force end at the measured
    // coil-loss airspeed; after that the bank turns on cold gas (+ bells). Without this the plan
    // scheduled the last high-α reversal at 3°/s for a moment when only ≈ 1.2°/s was left, the
    // reversal took ~110 s instead of ~45 s and the ship reached the belly flop 15.8 km crossrange.
    const lateRate = bankRateNow(ctx, n, SIGMA_RATE_GAS, mem.bankRateScale)
    const base: PredInput = {
      ...predInput(n, mem, phaseP), bankSign: s, sigma0: sFlown, sigmaRate: rate, hEnd: hFlip, liftFloor: LIFT_FLOOR, windOfs: mem.windOfs,
      coilLossV: mem.coilOff || phaseP !== "highAlpha" ? Infinity : COIL_LOSS_V, sigmaRateLate: lateRate,
    }
    const run = (mag: number, tRev: number): PredOut => {
      const o = predict(ctx.pred, { ...base, bankHigh: mag, tRev })
      return haB.dr === 0 && haB.cr === 0 ? o : { ...o, downrangeErr: o.downrangeErr - haB.dr, crossrangeErr: o.crossrangeErr - haB.cr }
    }
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
    //
    // The end *rate* is part of the condition. A position-only condition (measured) let the ship
    // arrive at the belly 30–60 km crossrange with 400–700 m/s of lateral speed: the bank held one
    // side until the predicted position crossed the line, spending no authority on the rate, and the
    // belly then had to kill 60 km in 166 s. Terminating the plan on `crossrange + K·crossVel`
    // makes the reversal start early enough to brake the closing rate, so the belly starts near the
    // target with little lateral speed — the "descent plan applies much earlier" fix.
    const bias = revBias(range)
    const REV_RATE_K = 25 // s: lateral speed the terminal plan is willing to brake before the gate
    const side = (o: PredOut): number => (o.crossrangeErr + s * bias + REV_RATE_K * o.crossVel) * s
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
      dbg(`${n.t.toFixed(0)} range ${(range / 1e3).toFixed(0)} s ${s} sc ${(sc / DEG).toFixed(0)} mag ${(mag / DEG).toFixed(0)} tRev ${Number.isFinite(tRev) ? (tRev - n.t).toFixed(0) : "inf"} out ${(out.downrangeErr / 1e3).toFixed(1)}/${(out.crossrangeErr / 1e3).toFixed(1)} never ${(nv.downrangeErr / 1e3).toFixed(1)}/${(nv.crossrangeErr / 1e3).toFixed(1)} now ${(nw.downrangeErr / 1e3).toFixed(1)}/${(nw.crossrangeErr / 1e3).toFixed(1)} rate ${(rate / DEG).toFixed(1)}°/s as ${mem.aeroScale.toFixed(2)}`)
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

/** current body +x in ECI */
const thrDirBody = (n: Nav): V3 => {
  const q = n.q
  // qrot(q, ex) without importing to keep this module small
  const x = 1 - 2 * (q.y * q.y + q.z * q.z)
  const y = 2 * (q.x * q.y + q.w * q.z)
  const z = 2 * (q.x * q.z - q.w * q.y)
  return v3(x, y, z)
}
