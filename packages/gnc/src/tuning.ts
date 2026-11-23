/**
 * Mission tuning — the guidance/landing knobs a parameter search may move, as one validated value.
 *
 * The vehicle/design half stays in `DesignParams` (`craft/src/config.ts`). This object is the mission
 * half: entry aim biases, belly-flop plan, flip gate, landing gains, tilt corridor, MPC weights, roll
 * damping and the deorbit Δv. Every default is the value the flown tree uses today, so
 * `MISSION_TUNING_DEFAULT` reproduces the current mission bit-for-bit when it is threaded through
 * guidance unchanged.
 *
 * The repository has no runtime schema library (no `effect` dependency), so this is a small explicit
 * schema: `TUNING_FIELDS` is the machine-readable field table (type, unit, range, description) and
 * `MissionTuning.decode` validates unknown input against it. That keeps a tuning set a first-class,
 * checkable value without adding a dependency to the simulation core.
 */

export type TuningKind = "number" | "integer"

export interface TuningField {
  readonly key: keyof TuningValues
  readonly kind: TuningKind
  /** inclusive bounds a decoded value must satisfy */
  readonly min: number
  readonly max: number
  readonly default: number
  readonly unit: string
  /** the phase/consumer that reads it — keeps the table self-documenting */
  readonly group: "entry" | "belly" | "gate" | "landing" | "roll" | "deorbit"
  readonly description: string
}

export interface TuningValues {
  // ---- entry (predictor–corrector and high-α hand-over) ----
  readonly aimEntry: number
  readonly aimHigh: number
  readonly glideZeta: number
  readonly dvBurn: number
  // ---- belly flop ----
  readonly bellyPlanEnd: number
  readonly bellyBankRateDeg: number
  readonly bellyAlignH: number
  readonly bellyAimE: number
  readonly bellyAimN: number
  // ---- flip gate (flipAltitude) ----
  readonly gateMiss: number
  readonly gateClosing: number
  readonly gateLat: number
  readonly flipDv: number
  readonly flipFall: number
  readonly flipComplete: number
  // ---- landing burn (flip → pins) ----
  readonly landingEngines: number
  readonly terminalEngines: number
  readonly finalWn: number
  readonly finalWMaxDeg: number
  readonly finalAxisAccel: number
  readonly hoverKp: number
  readonly hoverKd: number
  readonly contactSpeed: number
  readonly landKp: number
  readonly landKd: number
  readonly landingWn: number
  readonly landingZeta: number
  readonly landingWMaxDeg: number
  readonly landingWnRoll: number
  readonly landingRollMaxDeg: number
  readonly landingAxisAccelX: number
  readonly landingAxisAccelYZ: number
  readonly flipWn: number
  readonly flipZeta: number
  readonly flipWMaxDeg: number
  readonly flipWnRoll: number
  readonly flipRollMaxDeg: number
  readonly tilt300mDeg: number
  readonly tilt120mDeg: number
  readonly tilt60mDeg: number
  readonly tilt35mDeg: number
  readonly tilt12mDeg: number
  readonly tilt0mDeg: number
  readonly landingTauFlip: number
  readonly landingTauHigh: number
  readonly landingTauLow: number
  readonly landingGlide: number
  readonly landingTerminalAlt: number
  readonly landingWMag: number
  readonly landingOmegaMax: number
  readonly landingSlewRate: number
  readonly landingUzTrimMin: number
  readonly landingUzTrimMax: number
  readonly landingACruiseMin: number
  readonly landingACruiseMax: number
  readonly landingZProfMin: number
  readonly landingZProfMax: number
  readonly landingQp: number
  readonly landingQv: number
  readonly landingQvXY: number
  readonly landingRu: number
  readonly landingRd: number
  readonly landingTermQp: number
  readonly landingTermQv: number
  readonly terminalSwitchAlt: number
  readonly terminalSwitchLat: number
  readonly terminalSwitchV: number
  readonly landingAimE: number
  readonly landingAimN: number
  readonly armCloseAlt: number
  readonly armCloseLat: number
  readonly armCloseAlong: number
  readonly armCloseAcrossV: number
  readonly armCloseV: number
  /** time-to-rails at which the arms start closing; > closeTime so they finish just before contact (s) */
  readonly armCloseLead: number
  // ---- pre-flip roll cancellation (cold gas; the coil has no roll authority) ----
  readonly rollBandEntryDeg: number
  readonly rollBandAeroDeg: number
  readonly rollBandBellyDeg: number
  readonly rollBandPitchDeg: number
  readonly rollWnEntry: number
  readonly rollWnAero: number
  readonly rollWnBelly: number
  readonly rollWnPitch: number
}

const D = Math.PI / 180

/**
 * Every tunable, with the flown default. The default values are the constants of
 * `guidance.ts`/`landing.ts` as of the 2026-11-21 tree (2-engine landing, 1.4 km family gate,
 * `BELLY_PLAN_END = 1750`).
 */
export const TUNING_FIELDS: readonly TuningField[] = [
  { key: "aimEntry", kind: "number", min: 0, max: 400e3, default: 180e3, unit: "m", group: "entry", description: "Entry predictor–corrector downrange bias past the pad." },
  { key: "aimHigh", kind: "number", min: 0, max: 5e3, default: 1400, unit: "m", group: "entry", description: "High-α hand-over aim short of the pad (m). Measured: 900 m put the flip 1.1 km off; 300 m puts it 260 m off with room to brake the flip kick." },
  { key: "glideZeta", kind: "number", min: 0.05, max: 2, default: 0.3, unit: "", group: "entry", description: "High-α altitude-rate damping ratio of the phugoid law." },
  { key: "dvBurn", kind: "number", min: 80, max: 140, default: 108, unit: "m/s", group: "deorbit", description: "Deorbit retrograde Δv target (entry energy)." },
  { key: "bellyPlanEnd", kind: "number", min: 300, max: 3000, default: 2500, unit: "m", group: "belly", description: "Belly-flop plan end above the catch plane; deliberately decoupled from the flip gate." },
  { key: "bellyBankRateDeg", kind: "number", min: 0.3, max: 6, default: 1.8, unit: "deg/s", group: "belly", description: "Belly-flop lift-heading slew rate about v̂ (cold gas)." },
  { key: "bellyAlignH", kind: "number", min: 300, max: 3000, default: 1800, unit: "m", group: "belly", description: "Altitude window above the flip in which the nose snaps to the N/S catch quadrant." },
  { key: "bellyAimE", kind: "number", min: -800, max: 800, default: 0, unit: "m", group: "belly", description: "Belly-flop plan aim offset east of the catch point." },
  { key: "bellyAimN", kind: "number", min: -800, max: 800, default: 0, unit: "m", group: "belly", description: "Belly-flop plan aim offset north of the catch point." },
  { key: "gateMiss", kind: "number", min: 100, max: 1800, default: 420, unit: "m", group: "gate", description: "Design lateral miss the flip gate is sized for (flipAltitude)." },
  { key: "gateClosing", kind: "number", min: 5, max: 80, default: 35, unit: "m/s", group: "gate", description: "Design closing speed at the flip used to size the gate." },
  { key: "gateLat", kind: "number", min: 1, max: 12, default: 4.5, unit: "m/s²", group: "gate", description: "Design lateral authority used to size the gate." },
  { key: "flipDv", kind: "number", min: 5, max: 70, default: 35, unit: "m/s", group: "gate", description: "Vertical Δv the flip rotation is assumed to spend." },
  { key: "flipFall", kind: "number", min: 2, max: 10, default: 8, unit: "s", group: "gate", description: "Ballistic fall time during the flip per m/s of sink." },
  { key: "flipComplete", kind: "number", min: 0, max: 3000, default: 1500, unit: "m", group: "gate", description: "Pin height above the rails at which the flip must be complete (0: legacy lateral-stop gate)." },
  { key: "landingEngines", kind: "integer", min: 1, max: 3, default: 2, unit: "", group: "landing", description: "Sea-level Raptors from the flip to the pins (3 gives more brake/lateral authority; the final gate is faster)." },
  { key: "terminalEngines", kind: "integer", min: 1, max: 3, default: 1, unit: "", group: "landing", description: "Engines of the final approach; staged to when a plan on them is feasible (1: throttleable through the weight)." },
  { key: "finalWn", kind: "number", min: 0.3, max: 1.5, default: 0.8, unit: "rad/s", group: "landing", description: "Single-engine attitude loop natural frequency (one gimbal: ≈ 0.2 rad/s², 20°/s slew)." },
  { key: "finalWMaxDeg", kind: "number", min: 4, max: 24, default: 14, unit: "deg/s", group: "landing", description: "Single-engine attitude rate limit." },
  { key: "finalAxisAccel", kind: "number", min: 0.03, max: 0.3, default: 0.14, unit: "rad/s²", group: "landing", description: "Single-engine pitch/yaw braking acceleration assumed by the rate profile." },
  { key: "hoverKp", kind: "number", min: 0.01, max: 0.4, default: 0.3, unit: "1/s²", group: "landing", description: "Hold-until-centred lateral position gain above the rails (must sit below the attitude loop)." },
  { key: "hoverKd", kind: "number", min: 0.1, max: 2, default: 1.2, unit: "1/s", group: "landing", description: "Hold-until-centred lateral velocity gain." },
  { key: "contactSpeed", kind: "number", min: 0.2, max: 1.2, default: 0.7, unit: "m/s", group: "landing", description: "Design contact speed at the pins." },
  { key: "landKp", kind: "number", min: 0, max: 2, default: 0.3, unit: "1/s²", group: "landing", description: "Landing tracker position gain." },
  { key: "landKd", kind: "number", min: 0, max: 4, default: 1.2, unit: "1/s", group: "landing", description: "Landing tracker velocity gain." },
  { key: "landingWn", kind: "number", min: 0.5, max: 4, default: 1.7, unit: "rad/s", group: "landing", description: "Landing attitude loop natural frequency (pointing)." },
  { key: "landingZeta", kind: "number", min: 0.5, max: 1.5, default: 0.9, unit: "", group: "landing", description: "Landing attitude loop damping." },
  { key: "landingWMaxDeg", kind: "number", min: 8, max: 50, default: 26, unit: "deg/s", group: "landing", description: "Landing pointing slew-rate limit." },
  { key: "landingWnRoll", kind: "number", min: 0.3, max: 3, default: 1.0, unit: "rad/s", group: "landing", description: "Landing roll-loop bandwidth (differential gimbal)." },
  { key: "landingRollMaxDeg", kind: "number", min: 5, max: 40, default: 20, unit: "deg/s", group: "landing", description: "Landing roll-rate command limit." },
  { key: "landingAxisAccelX", kind: "number", min: 0.05, max: 1.5, default: 0.2, unit: "rad/s²", group: "landing", description: "Landing roll angular-acceleration authority (time-optimal cap)." },
  { key: "landingAxisAccelYZ", kind: "number", min: 0.1, max: 2, default: 0.7, unit: "rad/s²", group: "landing", description: "Landing pitch/yaw angular-acceleration authority." },
  { key: "flipWn", kind: "number", min: 0.5, max: 4, default: 1.3, unit: "rad/s", group: "landing", description: "Flip rotation attitude-loop bandwidth." },
  { key: "flipZeta", kind: "number", min: 0.5, max: 1.5, default: 0.9, unit: "", group: "landing", description: "Flip rotation damping." },
  { key: "flipWMaxDeg", kind: "number", min: 10, max: 60, default: 32, unit: "deg/s", group: "landing", description: "Flip pointing slew-rate limit." },
  { key: "flipWnRoll", kind: "number", min: 0.1, max: 2, default: 0.35, unit: "rad/s", group: "landing", description: "Flip roll-loop bandwidth." },
  { key: "flipRollMaxDeg", kind: "number", min: 0.5, max: 20, default: 2, unit: "deg/s", group: "landing", description: "Flip roll-rate command limit." },
  { key: "tilt300mDeg", kind: "number", min: 15, max: 70, default: 50, unit: "deg", group: "landing", description: "Thrust-corridor half-angle above 300 m of pin height." },
  { key: "tilt120mDeg", kind: "number", min: 15, max: 70, default: 48, unit: "deg", group: "landing", description: "Thrust-corridor half-angle above 120 m." },
  { key: "tilt60mDeg", kind: "number", min: 15, max: 70, default: 40, unit: "deg", group: "landing", description: "Thrust-corridor half-angle above 60 m." },
  { key: "tilt35mDeg", kind: "number", min: 8, max: 60, default: 28, unit: "deg", group: "landing", description: "Thrust-corridor half-angle above 35 m." },
  { key: "tilt12mDeg", kind: "number", min: 4, max: 45, default: 18, unit: "deg", group: "landing", description: "Thrust-corridor half-angle above 12 m." },
  { key: "tilt0mDeg", kind: "number", min: 4, max: 35, default: 12, unit: "deg", group: "landing", description: "Thrust-corridor half-angle at the pins." },
  { key: "landingTauFlip", kind: "number", min: 0.5, max: 5, default: 2.5, unit: "s", group: "landing", description: "Pointing/throttle lag handed to the MPC during the flip." },
  { key: "landingTauHigh", kind: "number", min: 0.3, max: 4, default: 1.4, unit: "s", group: "landing", description: "MPC pointing lag above 120 m." },
  { key: "landingTauLow", kind: "number", min: 0.2, max: 2, default: 0.7, unit: "s", group: "landing", description: "MPC pointing lag below 120 m." },
  { key: "landingGlide", kind: "number", min: 0, max: 0.3, default: 0.04, unit: "", group: "landing", description: "Soft glide-slope ratio z ≥ glide·|p_h| in the divert MPC." },
  { key: "landingTerminalAlt", kind: "number", min: 0.5, max: 20, default: 3, unit: "m", group: "landing", description: "Pin height where the analytic hoverslam owns the vertical channel." },
  { key: "landingWMag", kind: "number", min: 0, max: 0.5, default: 0.05, unit: "", group: "landing", description: "MPC fuel (thrust-magnitude) weight." },
  { key: "landingOmegaMax", kind: "number", min: 0.1, max: 1.2, default: 0.35, unit: "rad/s", group: "landing", description: "Plan thrust-direction slew limit (TVC)." },
  { key: "landingSlewRate", kind: "number", min: 0, max: 1.2, default: 0, unit: "rad/s", group: "landing", description: "Achievable thrust-axis slew ω of the attitude loop at the throttle floor; the two-phase lateral MPC plans increments |Δu| ≤ |u|·ω·h from the realised acceleration (0 = the old free-vector plan). EXPERIMENTAL, off: bounding the whole vector also slew-limits throttle (magnitude) changes, which are fast; the correct form is a direction-rate limit in a joint 3-D PDG (review/02)." },
  { key: "landingUzTrimMin", kind: "number", min: 0.2, max: 0.8, default: 0.4, unit: "", group: "landing", description: "Lower clamp of the measured vertical-tilt trim." },
  { key: "landingUzTrimMax", kind: "number", min: 0.6, max: 1.6, default: 1.2, unit: "", group: "landing", description: "Upper clamp of the measured vertical-tilt trim." },
  { key: "landingACruiseMin", kind: "number", min: 0.2, max: 4, default: 0.6, unit: "m/s²", group: "landing", description: "Minimum divert-cruise deceleration." },
  { key: "landingACruiseMax", kind: "number", min: 2, max: 14, default: 8, unit: "m/s²", group: "landing", description: "Maximum divert-cruise deceleration." },
  { key: "landingZProfMin", kind: "number", min: 5, max: 200, default: 30, unit: "m", group: "landing", description: "Minimum altitude where the divert cruise ends." },
  { key: "landingZProfMax", kind: "number", min: 100, max: 1200, default: 400, unit: "m", group: "landing", description: "Maximum altitude where the divert cruise ends." },
  { key: "landingQp", kind: "number", min: 0.00001, max: 0.01, default: 0.0002, unit: "", group: "landing", description: "Divert-MPC stage position weight." },
  { key: "landingQv", kind: "number", min: 0.01, max: 5, default: 0.4, unit: "", group: "landing", description: "Divert-MPC stage velocity weight." },
  { key: "landingQvXY", kind: "number", min: 0, max: 2, default: 0, unit: "", group: "landing", description: "Divert-MPC horizontal velocity weight scale." },
  { key: "landingRu", kind: "number", min: 0, max: 1, default: 0.01, unit: "", group: "landing", description: "Divert-MPC control effort weight." },
  { key: "landingRd", kind: "number", min: 0, max: 2, default: 0.1, unit: "", group: "landing", description: "Divert-MPC control-rate weight." },
  { key: "landingTermQp", kind: "number", min: 1, max: 500, default: 60, unit: "", group: "landing", description: "Divert-MPC terminal position weight." },
  { key: "landingTermQv", kind: "number", min: 10, max: 1000, default: 120, unit: "", group: "landing", description: "Divert-MPC terminal velocity weight." },
  { key: "terminalSwitchAlt", kind: "number", min: 5, max: 120, default: 45, unit: "m", group: "landing", description: "Pin height where the analytic terminal law takes over." },
  { key: "terminalSwitchLat", kind: "number", min: 1, max: 40, default: 20, unit: "m", group: "landing", description: "Maximum lateral offset at the terminal switch." },
  { key: "terminalSwitchV", kind: "number", min: 1, max: 30, default: 20, unit: "m/s", group: "landing", description: "Maximum horizontal speed at the terminal switch." },
  { key: "landingAimE", kind: "number", min: -300, max: 300, default: 0, unit: "m", group: "landing", description: "Landing divert aim east of the catch point (a pre-compensation for a systematic tracking drift; the terminal law still targets the pins)." },
  { key: "landingAimN", kind: "number", min: -300, max: 300, default: 0, unit: "m", group: "landing", description: "Landing divert aim north of the catch point." },
  { key: "armCloseAlt", kind: "number", min: 20, max: 120, default: 55, unit: "m", group: "landing", description: "Pin height where the arms are commanded closed." },
  { key: "armCloseLat", kind: "number", min: 0.5, max: 8, default: 1, unit: "m", group: "landing", description: "Lateral offset required to command the arms closed." },
  { key: "armCloseAlong", kind: "number", min: 2, max: 30, default: 12, unit: "m", group: "landing", description: "Along-rail (pad y) offset tolerated when commanding the arms closed." },
  { key: "armCloseAcrossV", kind: "number", min: 0.1, max: 3, default: 0.6, unit: "m/s", group: "landing", description: "Across-arm (pad x) speed required when commanding the arms closed." },
  { key: "armCloseV", kind: "number", min: 0.5, max: 8, default: 2.5, unit: "m/s", group: "landing", description: "Horizontal speed required to command the arms closed." },
  { key: "armCloseLead", kind: "number", min: 3.2, max: 8, default: 3.7, unit: "s", group: "landing", description: "Start the 3 s arm close when the pins are this many seconds from the rails (predicted at the closure time), so the arms are shut just before contact." },
  { key: "rollBandEntryDeg", kind: "number", min: 0.05, max: 3, default: 1.0, unit: "deg/s", group: "roll", description: "Entry roll-rate band below which no cold-gas roll torque is fired." },
  { key: "rollBandAeroDeg", kind: "number", min: 0.05, max: 3, default: 1.0, unit: "deg/s", group: "roll", description: "High-α/pitch-up roll-rate band." },
  { key: "rollBandBellyDeg", kind: "number", min: 0.05, max: 3, default: 1.5, unit: "deg/s", group: "roll", description: "Belly-flop roll-rate band." },
  { key: "rollBandPitchDeg", kind: "number", min: 0, max: 3, default: 0, unit: "deg/s", group: "roll", description: "Pitch-up roll-rate band (was unset)." },
  { key: "rollWnEntry", kind: "number", min: 0.1, max: 3, default: 0.25, unit: "rad/s", group: "roll", description: "Entry roll-loop bandwidth." },
  { key: "rollWnAero", kind: "number", min: 0.1, max: 3, default: 0.25, unit: "rad/s", group: "roll", description: "High-α roll-loop bandwidth." },
  { key: "rollWnBelly", kind: "number", min: 0.1, max: 3, default: 0.2, unit: "rad/s", group: "roll", description: "Belly-flop roll-loop bandwidth." },
  { key: "rollWnPitch", kind: "number", min: 0.1, max: 3, default: 0.25, unit: "rad/s", group: "roll", description: "Pitch-up roll-loop bandwidth." },
]

const fieldByKey = new Map<string, TuningField>(TUNING_FIELDS.map((f) => [f.key, f]))

const defaultsObject = (): Record<string, number> => {
  const out: Record<string, number> = {}
  for (const f of TUNING_FIELDS) out[f.key] = f.default
  return out
}

/** A tuning value that failed schema validation. */
export class TuningError extends Error {
  readonly issues: readonly string[]
  constructor(issues: readonly string[]) {
    super(`invalid MissionTuning: ${issues.join("; ")}`)
    this.name = "TuningError"
    this.issues = issues
  }
}

const validate = (values: Record<string, number>): readonly string[] => {
  const issues: string[] = []
  for (const key of Object.keys(values)) {
    if (!fieldByKey.has(key)) issues.push(`unknown field '${key}'`)
  }
  for (const f of TUNING_FIELDS) {
    const v = values[f.key]
    if (typeof v !== "number" || !Number.isFinite(v)) {
      issues.push(`${f.key} must be a finite number`)
      continue
    }
    if (f.kind === "integer" && !Number.isInteger(v)) issues.push(`${f.key} must be an integer`)
    if (v < f.min || v > f.max) issues.push(`${f.key} = ${v} outside [${f.min}, ${f.max}] ${f.unit}`)
  }
  return issues
}

/**
 * One immutable mission tuning. Construct with overrides of the defaults; the constructor validates
 * every field against `TUNING_FIELDS` (finite, in range, integer where required).
 */
export class MissionTuning implements TuningValues {
  readonly aimEntry: number
  readonly aimHigh: number
  readonly glideZeta: number
  readonly dvBurn: number
  readonly bellyPlanEnd: number
  readonly bellyBankRateDeg: number
  readonly bellyAlignH: number
  readonly bellyAimE: number
  readonly bellyAimN: number
  readonly gateMiss: number
  readonly gateClosing: number
  readonly gateLat: number
  readonly flipDv: number
  readonly flipFall: number
  readonly flipComplete: number
  readonly landingEngines: number
  readonly terminalEngines: number
  readonly finalWn: number
  readonly finalWMaxDeg: number
  readonly finalAxisAccel: number
  readonly hoverKp: number
  readonly hoverKd: number
  readonly contactSpeed: number
  readonly landKp: number
  readonly landKd: number
  readonly landingWn: number
  readonly landingZeta: number
  readonly landingWMaxDeg: number
  readonly landingWnRoll: number
  readonly landingRollMaxDeg: number
  readonly landingAxisAccelX: number
  readonly landingAxisAccelYZ: number
  readonly flipWn: number
  readonly flipZeta: number
  readonly flipWMaxDeg: number
  readonly flipWnRoll: number
  readonly flipRollMaxDeg: number
  readonly tilt300mDeg: number
  readonly tilt120mDeg: number
  readonly tilt60mDeg: number
  readonly tilt35mDeg: number
  readonly tilt12mDeg: number
  readonly tilt0mDeg: number
  readonly landingTauFlip: number
  readonly landingTauHigh: number
  readonly landingTauLow: number
  readonly landingGlide: number
  readonly landingTerminalAlt: number
  readonly landingWMag: number
  readonly landingOmegaMax: number
  readonly landingSlewRate: number
  readonly landingUzTrimMin: number
  readonly landingUzTrimMax: number
  readonly landingACruiseMin: number
  readonly landingACruiseMax: number
  readonly landingZProfMin: number
  readonly landingZProfMax: number
  readonly landingQp: number
  readonly landingQv: number
  readonly landingQvXY: number
  readonly landingRu: number
  readonly landingRd: number
  readonly landingTermQp: number
  readonly landingTermQv: number
  readonly terminalSwitchAlt: number
  readonly terminalSwitchLat: number
  readonly terminalSwitchV: number
  readonly landingAimE: number
  readonly landingAimN: number
  readonly armCloseAlt: number
  readonly armCloseLat: number
  readonly armCloseAlong: number
  readonly armCloseAcrossV: number
  readonly armCloseV: number
  /** time-to-rails at which the arms start closing; > closeTime so they finish just before contact (s) */
  readonly armCloseLead: number
  readonly rollBandEntryDeg: number
  readonly rollBandAeroDeg: number
  readonly rollBandBellyDeg: number
  readonly rollBandPitchDeg: number
  readonly rollWnEntry: number
  readonly rollWnAero: number
  readonly rollWnBelly: number
  readonly rollWnPitch: number

  constructor(overrides: Partial<TuningValues> = {}) {
    const values = { ...defaultsObject(), ...overrides } as Record<string, number>
    const issues = validate(values)
    if (issues.length > 0) throw new TuningError(issues)
    this.aimEntry = values.aimEntry
    this.aimHigh = values.aimHigh
    this.glideZeta = values.glideZeta
    this.dvBurn = values.dvBurn
    this.bellyPlanEnd = values.bellyPlanEnd
    this.bellyBankRateDeg = values.bellyBankRateDeg
    this.bellyAlignH = values.bellyAlignH
    this.bellyAimE = values.bellyAimE
    this.bellyAimN = values.bellyAimN
    this.gateMiss = values.gateMiss
    this.gateClosing = values.gateClosing
    this.gateLat = values.gateLat
    this.flipDv = values.flipDv
    this.flipFall = values.flipFall
    this.flipComplete = values.flipComplete
    this.landingEngines = values.landingEngines
    this.terminalEngines = values.terminalEngines
    this.finalWn = values.finalWn
    this.finalWMaxDeg = values.finalWMaxDeg
    this.finalAxisAccel = values.finalAxisAccel
    this.hoverKp = values.hoverKp
    this.hoverKd = values.hoverKd
    this.contactSpeed = values.contactSpeed
    this.landKp = values.landKp
    this.landKd = values.landKd
    this.landingWn = values.landingWn
    this.landingZeta = values.landingZeta
    this.landingWMaxDeg = values.landingWMaxDeg
    this.landingWnRoll = values.landingWnRoll
    this.landingRollMaxDeg = values.landingRollMaxDeg
    this.landingAxisAccelX = values.landingAxisAccelX
    this.landingAxisAccelYZ = values.landingAxisAccelYZ
    this.flipWn = values.flipWn
    this.flipZeta = values.flipZeta
    this.flipWMaxDeg = values.flipWMaxDeg
    this.flipWnRoll = values.flipWnRoll
    this.flipRollMaxDeg = values.flipRollMaxDeg
    this.tilt300mDeg = values.tilt300mDeg
    this.tilt120mDeg = values.tilt120mDeg
    this.tilt60mDeg = values.tilt60mDeg
    this.tilt35mDeg = values.tilt35mDeg
    this.tilt12mDeg = values.tilt12mDeg
    this.tilt0mDeg = values.tilt0mDeg
    this.landingTauFlip = values.landingTauFlip
    this.landingTauHigh = values.landingTauHigh
    this.landingTauLow = values.landingTauLow
    this.landingGlide = values.landingGlide
    this.landingTerminalAlt = values.landingTerminalAlt
    this.landingWMag = values.landingWMag
    this.landingOmegaMax = values.landingOmegaMax
    this.landingSlewRate = values.landingSlewRate
    this.landingUzTrimMin = values.landingUzTrimMin
    this.landingUzTrimMax = values.landingUzTrimMax
    this.landingACruiseMin = values.landingACruiseMin
    this.landingACruiseMax = values.landingACruiseMax
    this.landingZProfMin = values.landingZProfMin
    this.landingZProfMax = values.landingZProfMax
    this.landingQp = values.landingQp
    this.landingQv = values.landingQv
    this.landingQvXY = values.landingQvXY
    this.landingRu = values.landingRu
    this.landingRd = values.landingRd
    this.landingTermQp = values.landingTermQp
    this.landingTermQv = values.landingTermQv
    this.terminalSwitchAlt = values.terminalSwitchAlt
    this.terminalSwitchLat = values.terminalSwitchLat
    this.terminalSwitchV = values.terminalSwitchV
    this.landingAimE = values.landingAimE
    this.landingAimN = values.landingAimN
    this.armCloseAlt = values.armCloseAlt
    this.armCloseLat = values.armCloseLat
    this.armCloseAlong = values.armCloseAlong
    this.armCloseAcrossV = values.armCloseAcrossV
    this.armCloseV = values.armCloseV
    this.armCloseLead = values.armCloseLead
    this.rollBandEntryDeg = values.rollBandEntryDeg
    this.rollBandAeroDeg = values.rollBandAeroDeg
    this.rollBandBellyDeg = values.rollBandBellyDeg
    this.rollBandPitchDeg = values.rollBandPitchDeg
    this.rollWnEntry = values.rollWnEntry
    this.rollWnAero = values.rollWnAero
    this.rollWnBelly = values.rollWnBelly
    this.rollWnPitch = values.rollWnPitch
  }

  /** a new tuning with a subset of fields replaced */
  with(patch: Partial<TuningValues>): MissionTuning {
    return new MissionTuning({ ...this.toValues(), ...patch })
  }

  /** the raw validated number map (cache/hash/JSON friendly) */
  toValues(): Record<keyof TuningValues, number> {
    const out = {} as Record<keyof TuningValues, number>
    for (const f of TUNING_FIELDS) out[f.key] = this[f.key]
    return out
  }

  toJSON(): Record<string, number> {
    return this.toValues()
  }

  /**
   * Validate unknown input (e.g. JSON from a search or the app cache) into a `MissionTuning`.
   * Missing fields take their defaults; unknown fields are rejected so a typo cannot silently do
   * nothing.
   */
  static decode(input: unknown): MissionTuning {
    if (typeof input !== "object" || input === null) throw new TuningError(["input must be an object"])
    const rec = input as Record<string, unknown>
    const overrides: Record<string, number> = {}
    const issues: string[] = []
    for (const [key, value] of Object.entries(rec)) {
      const f = fieldByKey.get(key)
      if (!f) { issues.push(`unknown field '${key}'`); continue }
      if (typeof value !== "number") { issues.push(`${key} must be a number`); continue }
      overrides[key] = value
    }
    if (issues.length > 0) throw new TuningError(issues)
    return new MissionTuning(overrides)
  }
}

/** The flown defaults: identical to the constants the current tree uses. */
export const MISSION_TUNING_DEFAULT = new MissionTuning()

/** One searchable scalar: flat key, bounds, and the default it departs from. */
export interface TuningKnob {
  readonly key: keyof TuningValues
  readonly min: number
  readonly max: number
  readonly default: number
  readonly scale: "linear" | "log"
}

const specFor = (key: keyof TuningValues): TuningField => {
  const f = fieldByKey.get(key)
  if (!f) throw new TuningError([`no tuning field '${key}'`])
  return f
}

/** Build a knob from the schema table (single source of truth for bounds/defaults). */
export const knob = (key: keyof TuningValues, scale: "linear" | "log" = "linear"): TuningKnob => {
  const f = specFor(key)
  return { key, min: f.min, max: f.max, default: f.default, scale }
}

/** The default DOE vector: the coupled knobs the screen explores (gate, belly, aim, corridor). */
const bellyRateKnob: TuningKnob = { ...knob("bellyBankRateDeg", "log"), max: 2.5 } // cold-gas authority, not 6°/s

export const SCREEN_KNOBS: readonly TuningKnob[] = [
  knob("gateMiss", "log"),
  knob("bellyPlanEnd"),
  knob("bellyAimE"),
  knob("bellyAimN"),
  bellyRateKnob,
  knob("tilt300mDeg"),
  knob("tilt120mDeg"),
  knob("tilt60mDeg"),
  knob("tilt35mDeg"),
  knob("tilt12mDeg"),
  knob("tilt0mDeg"),
  knob("terminalSwitchAlt", "log"),
  knob("terminalSwitchLat", "log"),
  knob("terminalSwitchV", "log"),
  knob("armCloseAlt"),
]

/**
 * knobs the CMA-ES refinement is allowed to move. The attitude-loop gains, MPC lag/weights and roll
 * bands are deliberately **not** here: the ideal 3-DOF cannot see them, so the full 6-DOF variants in
 * `scripts/optimize.ts` sweep them instead (a fast-only optimum at `tau = 0` is a plan/plant lie).
 */
export const REFINE_KNOBS: readonly TuningKnob[] = [
  knob("gateMiss", "log"),
  knob("bellyPlanEnd"),
  bellyRateKnob,
  knob("tilt35mDeg"),
  knob("tilt12mDeg"),
  knob("tilt0mDeg"),
  knob("terminalSwitchAlt", "log"),
  knob("terminalSwitchLat", "log"),
  knob("terminalSwitchV", "log"),
  knob("armCloseAlt"),
]

/** clamping helper for a knob (linear in the stored value; log knobs are sampled in log space) */
export const clampKnob = (k: TuningKnob, value: number): number =>
  Math.min(k.max, Math.max(k.min, value))

/** normalised [0,1] → value for a knob (log knobs map geometrically) */
export const knobFromUnit = (k: TuningKnob, u: number): number => {
  const t = Math.min(1, Math.max(0, u))
  if (k.scale === "log") return k.min * Math.pow(k.max / k.min, t)
  return k.min + (k.max - k.min) * t
}

/** value → normalised [0,1] (inverse of `knobFromUnit`) */
export const knobToUnit = (k: TuningKnob, value: number): number => {
  const v = clampKnob(k, value)
  if (k.scale === "log") return Math.log(v / k.min) / Math.log(k.max / k.min)
  return (v - k.min) / (k.max - k.min)
}

/** radians helper for the degree-valued tuning fields */
export const deg = (d: number): number => d * D
