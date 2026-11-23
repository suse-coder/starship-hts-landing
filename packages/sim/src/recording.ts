/**
 * Columnar mission recording: one Float64Array per channel plus a Float32Array heat map
 * (surface temperature per thermal cell). Sampling is denser where things happen.
 * The browser scrubs it with binary search + linear interpolation.
 */
export const CHANNELS = [
  // kinematics
  ["t", "mission time", "s"], ["phase", "phase index", ""],
  ["rx", "ECI x", "m"], ["ry", "ECI y", "m"], ["rz", "ECI z", "m"],
  ["vx", "ECI vx", "m/s"], ["vy", "ECI vy", "m/s"], ["vz", "ECI vz", "m/s"],
  ["qw", "q w", ""], ["qx", "q x", ""], ["qy", "q y", ""], ["qz", "q z", ""],
  ["wx", "roll rate", "rad/s"], ["wy", "pitch rate", "rad/s"], ["wz", "yaw rate", "rad/s"],
  ["theta", "Earth rotation angle", "rad"],
  ["alt", "altitude", "m"], ["lat", "latitude", "rad"], ["lon", "longitude", "rad"],
  ["V", "airspeed", "m/s"], ["mach", "Mach", ""], ["qbar", "dynamic pressure", "Pa"],
  ["alpha", "angle of attack", "rad"], ["alphaCmd", "commanded α", "rad"], ["bank", "bank command", "rad"],
  ["g", "load factor", "g"], ["mass", "mass", "kg"], ["xcg", "CG station", "m"],
  ["attErr", "attitude error", "rad"],
  // aero
  ["CA", "axial coefficient", ""], ["CN", "normal coefficient", ""], ["xcp", "centre of pressure", "m"],
  ["aeroMy", "aero pitch moment", "N m"], ["aeroMz", "aero yaw moment", "N m"],
  // plasma / MHD
  ["T0", "stagnation temperature", "K"], ["ne", "stagnation electron density", "1/m³"],
  ["sigma", "conductivity", "S/m"], ["sigmaEff", "effective conductivity", "S/m"], ["hall", "Hall parameter", ""],
  ["stuart", "Stuart number Q", ""], ["bStag", "B at stagnation point", "T"], ["standoff", "shock standoff with field Δ", "m"], ["standoff0", "shock standoff without field Δ₀", "m"],
  ["mhdF", "MHD force", "N"], ["mhdMy", "MHD pitch moment", "N m"], ["mhdMz", "MHD yaw moment", "N m"],
  ["mhdP", "electrode power", "W"], ["mhdScale", "momentum-bound factor", ""], ["heatFactor", "MHD heat-flux factor", ""],
  ["mhdAuth", "MHD authority / demand", ""], ["seedFlow", "Cs seed flow", "kg/s"], ["seedLeft", "Cs remaining", "kg"],
  ["blackout", "comms state", ""],
  ["u0", "electrode 1 loading", ""], ["u1", "electrode 2", ""], ["u2", "electrode 3", ""], ["u3", "electrode 4", ""],
  ["u4", "electrode 5", ""], ["u5", "electrode 6", ""], ["u6", "electrode 7", ""], ["u7", "electrode 8", ""],
  // thermal
  ["qStag", "stagnation heat flux", "W/m²"], ["picaSurf", "PICA-X surface max", "K"], ["bondline", "bondline max", "K"],
  ["tileSurf", "tile surface max", "K"], ["tileBack", "steel backing max under tile", "K"],
  ["steel", "steel backing max", "K"], ["steelX", "hottest steel station", "m"], ["recession", "PICA recession max", "m"],
  ["turbFrac", "turbulent fraction", ""],
  // magnet
  ["coilI", "coil current", "A"], ["coilT", "cold-mass temperature", "K"], ["coilE", "stored energy", "J"],
  ["battery", "battery energy", "J"], ["coilMode", "coil mode", ""], ["heatLeak", "cryostat heat leak", "W"],
  // propulsion & RCS
  ["n2", "N2 remaining", "kg"], ["rcsTau", "RCS torque", "N m"], ["header", "header propellant", "kg"],
  ["thrust", "total thrust", "N"], ["T1", "Raptor 1 thrust", "N"], ["T2", "Raptor 2 thrust", "N"], ["T3", "Raptor 3 thrust", "N"],
  ["gp", "gimbal pitch", "rad"], ["gy", "gimbal yaw", "rad"],
  ["gp1", "SL1 gimbal pitch", "rad"], ["gp2", "SL2 gimbal pitch", "rad"], ["gp3", "SL3 gimbal pitch", "rad"],
  ["gy1", "SL1 gimbal yaw", "rad"], ["gy2", "SL2 gimbal yaw", "rad"], ["gy3", "SL3 gimbal yaw", "rad"],
  ["r0", "thr nose+y", ""], ["r1", "thr nose-y", ""], ["r2", "thr nose+z", ""], ["r3", "thr nose-z", ""],
  ["r4", "thr aft+y", ""], ["r5", "thr aft-y", ""], ["r6", "thr aft+z", ""], ["r7", "thr aft-z", ""],
  ["r8", "roll+a", ""], ["r9", "roll+b", ""], ["r10", "roll-a", ""], ["r11", "roll-b", ""],
  // pad frame
  ["pe", "East of pad", "m"], ["pn", "North of pad", "m"], ["pu", "Up from pad", "m"],
  ["ve", "v East", "m/s"], ["vn", "v North", "m/s"], ["vu", "v Up", "m/s"],
  ["pinX", "pin offset E", "m"], ["pinY", "pin offset N", "m"], ["pinZ", "pins above rails", "m"],
  ["hBase", "aft end height", "m"], ["arms", "arms open", ""], ["armX", "arm pair centre offset E", "m"],
  ["flapDep", "aft flap deployment", ""], ["flapD1", "aft flap 1 deflection", "rad"], ["flapD2", "aft flap 2 deflection", "rad"], ["pinLoad", "pin load", "N"],
  ["pinLoadMin", "minimum per-pin load", "N"], ["pinCount", "loaded pins", ""],
  ["bodyContact", "hull against tower structure", ""], ["hullBump", "swept hull impulse", ""],
  ["windE", "wind East", "m/s"], ["windN", "wind North", "m/s"],
  // guidance plan vs flown, per degree of freedom (catch frame: E across the rails, N along them,
  // U above the rail plane). `planOn` = 1 where the guidance holds a reference plan; the plan
  // channels are that plan sampled at the current time (descent) or height (belly flop). The tube
  // channels are the constraint band: the lateral error that is still recoverable from here, and
  // the hard limits (fastest sink that can still be stopped, tower corridor half-width).
  ["planOn", "reference plan active", ""],
  ["planX", "plan: pins E", "m"], ["planY", "plan: pins N", "m"], ["planZ", "plan: pins U", "m"],
  ["planVx", "plan: v E", "m/s"], ["planVy", "plan: v N", "m/s"], ["planVz", "plan: v U", "m/s"],
  ["planAcc", "plan: thrust acceleration", "m/s²"], ["planTiltE", "plan: tilt E", "rad"], ["planTiltN", "plan: tilt N", "rad"],
  ["tubeLat", "recoverable lateral error", "m"], ["tubeHard", "hard lateral limit (tower corridor)", "m"],
  ["vzLimit", "hard sink limit (full-thrust stop)", "m/s"],
  ["pinVx", "pin point v E", "m/s"], ["pinVy", "pin point v N", "m/s"],
  ["tiltE", "thrust-line tilt E", "rad"], ["tiltN", "thrust-line tilt N", "rad"], ["rollAng", "pin-axis azimuth error", "rad"],
  ["accT", "thrust acceleration", "m/s²"], ["accCmd", "commanded thrust acceleration", "m/s²"],
  ["cmdTiltE", "commanded tilt E", "rad"], ["cmdTiltN", "commanded tilt N", "rad"],
  ["thr1", "SL1 throttle", ""], ["thr2", "SL2 throttle", ""], ["thr3", "SL3 throttle", ""],
  ["predDR", "predicted downrange miss", "m"], ["predCR", "predicted crossrange miss", "m"], ["npcMhd", "predictor MHD drag scale", ""], ["bellyHdg", "belly-flop glide heading", "rad"], ["bellyMiss", "belly-flop predicted miss", "m"],
] as const

export type ChannelKey = (typeof CHANNELS)[number][0]
export const CH: Readonly<Record<ChannelKey, number>> = Object.fromEntries(CHANNELS.map((c, i) => [c[0], i])) as Record<ChannelKey, number>
export const N_CH = CHANNELS.length

export interface Recording {
  readonly n: number
  readonly data: Float64Array[] // per channel
  readonly heat: Float32Array // n × cells
  readonly pred: Float32Array // n × PRED_K × 3, landing-MPC path in pad ENU (E,N,U) metres
  readonly predK: number
  readonly predHas: Uint8Array // n, 1 where the sample carries a live plan
  readonly cells: number
  readonly events: readonly { t: number; msg: string }[]
  readonly meta: RecordingMeta
}

export interface RecordingMeta {
  readonly epochJd: number
  readonly theta0: number
  readonly padLat: number
  readonly padLon: number
  readonly padAlt: number
  readonly outcome: string
  readonly summary: Record<string, number | string>
  readonly phaseStarts: readonly { phase: string; t: number }[]
  readonly predPath: readonly { x: number; y: number; z: number }[] // ECEF planning path
  readonly vehicle: unknown // VehicleConfig (structured-clone friendly subset)
  readonly wallClockMs: number
  readonly steps: number
}

/** points per predicted-path sample (the landing MPC's plan to the catch) */
export const PRED_K = 24

/** append-only builder (the imperative shell of the run loop) */
export const recorder = (cells: number) => {
  const cols: number[][] = CHANNELS.map(() => [])
  const heat: number[] = []
  const preds: Float32Array[] = []
  const predHas: number[] = []
  return {
    push: (row: ArrayLike<number>, temps: ArrayLike<number>, pred?: readonly { x: number; y: number; z: number }[]): void => {
      for (let i = 0; i < N_CH; i++) cols[i].push(row[i])
      for (let i = 0; i < cells; i++) heat.push(temps[i])
      const p = new Float32Array(PRED_K * 3)
      const m = pred ? pred.length : 0
      // the whole path, resampled to PRED_K points: the belly-flop plan is hundreds of points long
      // and taking its head would show only the first seconds of the fall
      for (let k = 0; k < PRED_K && m > 0; k++) {
        const j = m === 1 ? 0 : Math.round((k * (m - 1)) / (PRED_K - 1))
        p[k * 3] = pred![j].x; p[k * 3 + 1] = pred![j].y; p[k * 3 + 2] = pred![j].z
      }
      preds.push(p)
      predHas.push(m > 1 ? 1 : 0)
    },
    size: (): number => cols[0].length,
    finish: (events: Recording["events"], meta: RecordingMeta): Recording => ({
      n: cols[0].length,
      data: cols.map((c) => Float64Array.from(c)),
      heat: Float32Array.from(heat),
      pred: (() => { const out = new Float32Array(preds.length * PRED_K * 3); preds.forEach((p, i) => out.set(p, i * PRED_K * 3)); return out })(),
      predK: PRED_K,
      predHas: Uint8Array.from(predHas),
      cells,
      events,
      meta,
    }),
  }
}

/** index i and fraction f with t_i ≤ t < t_{i+1} */
export const locate = (rec: Recording, t: number): readonly [number, number] => {
  const ts = rec.data[CH.t]
  const n = rec.n
  if (t <= ts[0]) return [0, 0]
  if (t >= ts[n - 1]) return [n - 2, 1]
  let lo = 0, hi = n - 1
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1
    if (ts[m] <= t) lo = m
    else hi = m
  }
  return [lo, (t - ts[lo]) / (ts[hi] - ts[lo])]
}

export const valueAt = (rec: Recording, key: ChannelKey, t: number): number => {
  const [i, f] = locate(rec, t)
  const c = rec.data[CH[key]]
  return c[i] + (c[i + 1] - c[i]) * f
}
