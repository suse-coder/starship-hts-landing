import { type V3, v3, qrot, norm } from "@ship/math"
import { fVac100, massProps } from "@ship/craft"
import { CH, step, type SimState, type World } from "@ship/sim"
import { aeroWrench } from "@ship/aero"
import { landingAero } from "@ship/gnc"
import { landingRig, rigWorld, trimGimbal, withActuators, type RigOptions } from "./rig"

const DEG = Math.PI / 180

/** least-squares slope of y over x */
const slope = (x: readonly number[], y: readonly number[]): number => {
  const n = x.length
  let sx = 0, sy = 0, sxx = 0, sxy = 0
  for (let i = 0; i < n; i++) { sx += x[i]; sy += y[i]; sxx += x[i] * x[i]; sxy += x[i] * y[i] }
  return (n * sxy - sx * sy) / Math.max(n * sxx - sx * sx, 1e-30)
}

export interface PlantId {
  readonly engines: number
  readonly throttle: number
  readonly mass: number
  /** thrust acceleration T/m (m/s²): measured from the vertical acceleration, and the engine model's value */
  readonly accel: number
  readonly accelModel: number
  /** gimbal step used (deg) and the pitch acceleration it produced (rad/s²): measured and T·L·sin δ / I */
  readonly stepDeg: number
  readonly alpha: number
  readonly alphaModel: number
  /** pitch acceleration per radian of gimbal, b = T·L/I (1/s²) */
  readonly b: number
  /** lateral acceleration of the centre of gravity and of the catch pins during the step (m/s²) */
  readonly aCg: number
  readonly aPin: number
  /** distance of the pins and of the centre of percussion ahead of the CG (m): equal ⇒ the pins do not feel the gimbal's side force */
  readonly pinLever: number
  readonly percussion: number
  /** roll acceleration produced by the same step (rad/s²): the off-centre engine's roll coupling */
  readonly alphaRoll: number
  /** full-gimbal pitch authority (rad/s²) and the snap it gives the pin position (m/s⁴) */
  readonly alphaMax: number
  readonly snapMax: number
}

/**
 * Open-loop plant identification: the running set holds its trim gimbal, then engine gimbals step
 * by `stepDeg` about body z (yaw gimbal → rotation about body z → motion along body y = east); the
 * angular and linear accelerations are fitted over 0.4 s after the actuator has arrived.
 */
export const identifyPlant = (engines: readonly number[], throttle: number, stepDeg = 2, header?: number): PlantId => {
  const rig = landingRig({ pin: v3(0, 0, 600), vel: v3(0, 0, 0), engines, throttle, header })
  const W0 = rig.world
  const veh = W0.vehicle
  // The bare plant: no cold gas. With the roll thrusters live the flight software cancels the
  // engine's roll coupling during the step and the identification measures the controller instead
  // (it did, once the roll couples were made strong enough to hold it). Every quantity below uses
  // the mass properties of that state.
  const cons0 = { ...rig.state.cons, n2: 0 }
  const mpBare = massProps(veh, cons0)
  const xcg = mpBare.xcg
  const trims = [0, 1, 2].map((i) => (engines.length === 1 ? trimGimbal(W0, i, xcg) : { gp: 0, gy: 0 }))
  const t0 = rig.state.t
  const tStep = t0 + 0.3
  // The override owns the whole actuator command, including which engines run: the flight
  // software stages to one engine as soon as it is in the landing phase, which silently shut down
  // the second engine of a two-engine identification (measured T/m 11.4 against a model 13.7 —
  // a 17 % "model error" that was the test, not the model).
  const W: World = withActuators(W0, (t, cmds) => cmds.map((c, i) => ({ ...c, on: engines.includes(i), throttle, gp: trims[i].gp, gy: trims[i].gy + (t >= tStep ? stepDeg * DEG : 0) })))
  let s: SimState = { ...rig.state, cons: cons0 }
  const ts: number[] = [], wz: number[] = [], wx: number[] = [], vy: number[] = [], vpin: number[] = [], vu0: number[] = [], tu: number[] = []
  const tArrive = tStep + (stepDeg * DEG) / veh.engines[0].gimbalRate + 0.02
  while (s.t < tArrive + 0.4) {
    const o = step(W, s)
    const r = o.row
    const t = r[CH.t]
    if (t < tStep - 0.02 && t > t0 + 0.05) { tu.push(t); vu0.push(r[CH.vu]) }
    if (t >= tArrive) {
      ts.push(t); wz.push(r[CH.wz]); wx.push(r[CH.wx])
      // east velocity of the CG and of the pin axis point (v + ω × r)
      const wI = qrot(s.q, s.w), off = qrot(s.q, v3(veh.pinX - xcg, 0, 0))
      const cr = v3(wI.y * off.z - wI.z * off.y, wI.z * off.x - wI.x * off.z, wI.x * off.y - wI.y * off.x)
      const crE = cr.x * rig.east.x + cr.y * rig.east.y + cr.z * rig.east.z
      vy.push(r[CH.ve]); vpin.push(r[CH.ve] + crE)
    }
    s = o.s
  }
  const m = mpBare.m, I = mpBare.I.y
  const L = xcg - veh.engines[0].gimbal.x
  const nE = engines.length
  const tModel = nE * (throttle * fVac100(veh.engines[0]) - 101325 * veh.engines[0].exitArea)
  const accel = slope(tu, vu0) + 9.80
  const alpha = slope(ts, wz)
  const alphaModel = -(tModel * L * Math.sin(stepDeg * DEG)) / I
  const b = Math.abs(alpha) / (stepDeg * DEG)
  const gmax = veh.engines[0].gimbalMax
  const trimMag = engines.length === 1 ? Math.hypot(trims[engines[0]].gp, trims[engines[0]].gy) : 0
  const alphaMax = b * (gmax - trimMag)
  return {
    engines: nE, throttle, mass: m, accel, accelModel: tModel / m, stepDeg, alpha, alphaModel, b,
    aCg: slope(ts, vy), aPin: slope(ts, vpin), pinLever: veh.pinX - xcg, percussion: I / (m * L),
    alphaRoll: slope(ts, wx), alphaMax, snapMax: (tModel / m) * alphaMax,
  }
}

/** measured throttle slew (fraction/s) and the delay to 90 % of a throttle step */
export const identifyThrottle = (from = 0.4, to = 0.9): { readonly slew: number; readonly t90: number } => {
  const rig = landingRig({ pin: v3(0, 0, 600), engines: [0], throttle: from })
  const trim = trimGimbal(rig.world, 0, rig.mass.xcg)
  const t0 = rig.state.t + 0.2
  const W = withActuators(rig.world, (t, cmds) => cmds.map((c) => ({ ...c, throttle: t >= t0 ? to : from, gp: trim.gp, gy: trim.gy })))
  let s = rig.state
  const T0 = from * fVac100(rig.world.vehicle.engines[0]), T1 = to * fVac100(rig.world.vehicle.engines[0])
  let t90 = NaN
  const ts: number[] = [], th: number[] = []
  while (s.t < t0 + 2) {
    const o = step(W, s)
    const fv = o.row[CH.T1] + 101325 * rig.world.vehicle.engines[0].exitArea
    if (s.t >= t0 && fv < T0 + 0.8 * (T1 - T0)) { ts.push(s.t); th.push(fv / fVac100(rig.world.vehicle.engines[0])) }
    if (Number.isNaN(t90) && fv >= T0 + 0.9 * (T1 - T0)) t90 = s.t - t0
    s = o.s
  }
  return { slew: slope(ts, th), t90 }
}

export interface FlightSample {
  readonly t: number
  /** pins relative to the catch point, pad ENU (m) */
  readonly pin: V3
  /** velocity of the pin point, pad ENU (m/s) */
  readonly vel: V3
  /** body-axis tilt from the vertical (deg) and roll rate (deg/s) */
  readonly tilt: number
  readonly rollRate: number
  /** azimuth error of the pin axis from east–west (deg, mod 180): 0 = pins over the rails */
  readonly rollAng: number
  /** gimbal of the first running engine (deg) and its throttle-equivalent thrust (N) */
  readonly gp: number
  readonly gy: number
  readonly thrust: number
  readonly header: number
  readonly arms: number
}

export interface FlightResult {
  readonly samples: readonly FlightSample[]
  readonly outcome: string
  readonly reason: string
  readonly events: readonly { readonly t: number; readonly msg: string }[]
  readonly end: SimState
}

/** fly the rig under the flight software until the outcome (or `tMax` seconds) */
export const flyRig = (o: RigOptions, world: World = rigWorld(), tMax = 120, every = 0.05): FlightResult => {
  const rig = landingRig(o, world)
  let s = rig.state
  const samples: FlightSample[] = []
  let last = -1e9
  const t0 = s.t
  while (!s.done && s.t - t0 < tMax) {
    const out = step(rig.world, s)
    const r = out.row
    if (r[CH.t] - last >= every) {
      last = r[CH.t]
      const q = { w: r[CH.qw], x: r[CH.qx], y: r[CH.qy], z: r[CH.qz] }
      const xb = qrot(q, v3(1, 0, 0))
      const theta = r[CH.theta]
      const upE = { x: rig.world.padEnu.u.x * Math.cos(theta) - rig.world.padEnu.u.y * Math.sin(theta), y: rig.world.padEnu.u.x * Math.sin(theta) + rig.world.padEnu.u.y * Math.cos(theta), z: rig.world.padEnu.u.z }
      const cu = xb.x * upE.x + xb.y * upE.y + xb.z * upE.z
      const k = r[CH.T1] > 1e3 ? 1 : r[CH.T2] > 1e3 ? 2 : 3
      samples.push({
        t: r[CH.t] - t0, pin: v3(r[CH.pinX], r[CH.pinY], r[CH.pinZ]), vel: v3(r[CH.ve], r[CH.vn], r[CH.vu]),
        tilt: Math.acos(Math.max(-1, Math.min(1, cu))) / DEG, rollRate: r[CH.wx] / DEG,
        rollAng: (() => {
          const yb = qrot(q, v3(0, 1, 0))
          const e = rig.world.padEnu.e, nn = rig.world.padEnu.n
          const rz = (a: { x: number; y: number; z: number }): { x: number; y: number; z: number } => ({ x: a.x * Math.cos(theta) - a.y * Math.sin(theta), y: a.x * Math.sin(theta) + a.y * Math.cos(theta), z: a.z })
          const eE = rz(e), eN = rz(nn)
          const az = Math.atan2(yb.x * eN.x + yb.y * eN.y + yb.z * eN.z, yb.x * eE.x + yb.y * eE.y + yb.z * eE.z) / DEG
          return ((az + 90) % 180 + 180) % 180 - 90
        })(),
        gp: (k === 1 ? r[CH.gp1] : k === 2 ? r[CH.gp2] : r[CH.gp3]) / DEG, gy: (k === 1 ? r[CH.gy1] : k === 2 ? r[CH.gy2] : r[CH.gy3]) / DEG,
        thrust: r[CH.thrust], header: r[CH.header], arms: r[CH.arms],
      })
    }
    s = out.s
  }
  void norm
  return { samples, outcome: s.outcome, reason: s.mem.reason, events: s.events, end: s }
}

export interface StepMetrics {
  /** initial offset (m) along the excited axis */
  readonly d0: number
  /** time to stay inside 10 % / inside 0.2 m of the axis (s) */
  readonly t10: number
  readonly settle: number
  /** overshoot past the axis (m; 0 = none) */
  readonly overshoot: number
  /** axis crossings (0 = monotonic approach) */
  readonly crossings: number
  readonly tiltMax: number
  readonly rollRateMax: number
  /** cross-axis excursion (m): the lateral axes should not couple */
  readonly cross: number
}

/** quality of a lateral step response: `axis` is the excited pad axis */
export const stepMetrics = (samples: readonly FlightSample[], axis: "x" | "y", tEnd = Infinity): StepMetrics => {
  const s = samples.filter((p) => p.t <= tEnd)
  const d0 = s[0].pin[axis]
  const sg = Math.sign(d0) || 1
  const other = axis === "x" ? "y" : "x"
  let t10 = 0, settle = 0, over = 0, crossings = 0, tilt = 0, roll = 0, cross = 0
  for (let i = 0; i < s.length; i++) {
    const x = s[i].pin[axis] * sg
    if (Math.abs(x) > 0.1 * Math.abs(d0)) t10 = s[i].t
    if (Math.abs(x) > 0.2) settle = s[i].t
    over = Math.max(over, -x)
    if (i > 0 && s[i - 1].pin[axis] * sg > 0 && x <= 0) crossings++
    tilt = Math.max(tilt, s[i].tilt); roll = Math.max(roll, Math.abs(s[i].rollRate)); cross = Math.max(cross, Math.abs(s[i].pin[other] - s[0].pin[other]))
  }
  return { d0, t10, settle, overshoot: over, crossings, tiltMax: tilt, rollRateMax: roll, cross }
}

export interface AeroCheck {
  readonly V: number
  readonly tiltDeg: number
  /** lateral force (N) and pitching moment about the CG (N·m): the guidance's landing database, and the plant */
  readonly fGuide: number
  readonly mGuide: number
  readonly fPlant: number
  readonly mPlant: number
  /** the one gimbal's holding torque at the throttle floor beside its trim angle (N·m) */
  readonly gimbalTorque: number
}

/**
 * Model check of the tail-first aerodynamics: what the guidance believes (its landing-aero
 * database) against what the plant applies (`aeroWrench`), for a hull tilted `tiltDeg` from a
 * vertical fall at V. The moment is statically destabilising and is the descent's tilt limit.
 */
export const checkTailFirstAero = (V: number, tiltDeg: number, rho = 1.16): AeroCheck => {
  const rig = landingRig({ pin: v3(0, 0, 600), engines: [0] })
  const W = rig.world, xcg = rig.mass.xcg
  const t = tiltDeg * DEG
  // a frame with x up: body axis tilted towards +z, air from below
  const xb = v3(Math.cos(t), 0, Math.sin(t)), zb = v3(-Math.sin(t), 0, Math.cos(t)), vAir = v3(-V, 0, 0)
  const g = landingAero(W.guide.landAero!, rho, xb, vAir, xcg)
  const vB = v3(vAir.x * xb.x + vAir.z * xb.z, 0, vAir.x * zb.x + vAir.z * zb.z)
  const p = aeroWrench(W.aero, { vAir: vB, w: v3(0, 0, 0), rho, a: 340, mu: 1.8e-5, xcg, cpMax: 1.84, enginesOn: true, knudsen: 1e-8, flaps: undefined as never })
  const e0 = W.vehicle.engines[0]
  const L = xcg - e0.gimbal.x
  const tFloor = e0.minThrottle * fVac100(e0) - 101325 * e0.exitArea
  return {
    V, tiltDeg, fGuide: g.F.z, mGuide: g.M.y, fPlant: p.F.x * xb.z + p.F.z * zb.z, mPlant: p.M.y,
    gimbalTorque: tFloor * L * Math.sin(e0.gimbalMax - Math.atan(Math.hypot(e0.gimbal.y, e0.gimbal.z) / L)),
  }
}

export interface CaptureCase {
  readonly start: { readonly x: number; readonly y: number; readonly vx: number; readonly vy: number }
  readonly outcome: string
  /** pin offset at 1 km above the rails and at the end (m) */
  readonly at1km: number
  readonly atRails: number
  /** largest excursion past the axis along the excited direction (m) */
  readonly overshoot: number
  readonly tiltMax: number
  readonly rollAngMax: number
  /** pin-axis azimuth error when the pins reach the rails (deg) */
  readonly rollAngRails: number
  readonly fuel: number
  /** time spent slower than 2 m/s above the last 0.3 m (s): the hover time */
  readonly hover: number
  readonly contactSpeed: number
}

/** fly the single-engine descent on the rig from `h0` m at `vz` m/s with a lateral offset and residual speed */
export const captureCase = (x: number, y: number, vx = 0, vy = 0, h0 = 1300, vz = -30): CaptureCase => {
  const f = flyRig({ pin: v3(x, y, h0), vel: v3(vx, vy, vz), engines: [0], throttle: 0.4 }, undefined, 120)
  const last = f.samples[f.samples.length - 1]
  const at1k = f.samples.find((p) => p.pin.z <= 1000)
  const mx = stepMetrics(f.samples, Math.abs(x) >= Math.abs(y) ? "x" : "y")
  const touch = [...f.samples].reverse().find((p) => p.pin.z > 0.05)
  return {
    start: { x, y, vx, vy }, outcome: f.outcome, at1km: at1k ? Math.hypot(at1k.pin.x, at1k.pin.y) : NaN, atRails: Math.hypot(last.pin.x, last.pin.y),
    overshoot: mx.overshoot, tiltMax: mx.tiltMax, rollAngMax: f.samples.reduce((m, p) => Math.max(m, Math.abs(p.rollAng)), 0),
    rollAngRails: touch ? Math.abs(touch.rollAng) : NaN,
    fuel: f.samples[0].header - last.header, hover: f.samples.filter((p) => p.pin.z > 0.3 && Math.abs(p.vel.z) < 2).length * 0.05,
    contactSpeed: touch ? Math.abs(touch.vel.z) : NaN,
  }
}
