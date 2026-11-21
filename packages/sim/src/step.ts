import {
  type Q, type V3, v3, add, sub, scale, cross, norm, normalize, dot, hadamard, qrot, qrotInv, qDot, qAddScaled, qnormalize, ZERO, clamp,
} from "@ship/math"
import {
  atmosphere, corotation, ecefToGeodetic, enuToEcef, geomagneticEcef, gravityJ2, OMEGA_EARTH, rotZ, totalWindEnu, turbStep, G0, ecefToEnu,
} from "@ship/earth"
import { lookupStag, type StagState, cyclotronFreq, criticalDensity } from "@ship/gas"
import { engineStep, engineThrust, massProps, rcsWrench, sumEngines, type EngineCmd, type EngineOut, type MassProps } from "@ship/craft"
import { aeroWrench, type AeroOut } from "@ship/aero"
import { surfaceHeating, thermalStep, thermalSummary, surfaceTemps } from "@ship/heat"
import { coilEnergy, coilNI, coilStep, dipoleTorque, mhdBasis, ZERO_MHD, type MhdOut } from "@ship/mhd"
import { armStep, TOWER, towerContact } from "@ship/tower"
import { allocateGimbals, allocateMhd, allocateRcs, guide, phaseIndex, type Nav, torqueCommand, U_BASE } from "@ship/gnc"
import { CH, N_CH } from "./recording"
import type { SimState } from "./state"
import type { World } from "./world"

/** integration/control step by flight regime */
export const dtFor = (s: SimState, alt: number): number => {
  switch (s.mem.phase) {
    case "deorbitAlign":
    case "entryAlign":
      return alt > 125e3 ? 0.5 : 0.05
    case "deorbitBurn":
      return 0.05
    case "entry":
      return alt > 100e3 ? 0.1 : 0.02
    case "loft":
    case "pitchUp":
    case "highAlpha":
    case "bellyFlop":
      return 0.02
    case "flip":
    case "landing":
      return 0.01
    case "terminal":
    case "caught":
    case "lost":
      return 0.004
  }
}

interface Rigid { readonly r: V3; readonly v: V3; readonly q: Q; readonly w: V3 }
interface DRigid { readonly dr: V3; readonly dv: V3; readonly dq: Q; readonly dw: V3 }

const addR = (a: Rigid, d: DRigid, h: number): Rigid => ({ r: add(a.r, scale(d.dr, h)), v: add(a.v, scale(d.dv, h)), q: qAddScaled(a.q, d.dq, h), w: add(a.w, scale(d.dw, h)) })

export interface StepOut {
  readonly s: SimState
  readonly row: Float64Array
  readonly temps: Float64Array
  /** landing-MPC plan from the current state to the catch, pad ENU metres (empty outside flip/landing/terminal) */
  readonly pred: readonly { x: number; y: number; z: number }[]
}

const RN_COMMS = 12e9 // Hz, Ku-band Starlink downlink

export const step = (W: World, s: SimState): StepOut => {
  const veh = W.vehicle
  const theta = W.plan.theta0 + OMEGA_EARTH * s.t
  const ecef = rotZ(s.r, -theta)
  const geo = ecefToGeodetic(ecef)
  const alt = geo.alt
  const dt = dtFor(s, alt)
  const atm = atmosphere(alt)
  const mp: MassProps = massProps(veh, s.cons)
  // ---------- local frames ----------
  const rot = (u: V3): V3 => rotZ(u, theta)
  const sl = Math.sin(geo.lat), cl = Math.cos(geo.lat), so = Math.sin(geo.lon), co = Math.cos(geo.lon)
  const east = rot(v3(-so, co, 0)), north = rot(v3(-sl * co, -sl * so, cl)), up = rot(v3(cl * co, cl * so, sl))
  const turb = turbStep(s.turb, dt, norm(sub(s.v, corotation(s.r))), alt, W.wind)
  const windEnu = totalWindEnu(alt, turb, W.wind)
  const windEci = add(add(scale(east, windEnu.x), scale(north, windEnu.y)), scale(up, windEnu.z))
  const vGround = sub(s.v, corotation(s.r))
  const vAir = sub(vGround, windEci)
  const V = norm(vAir)
  const vAirB = qrotInv(s.q, vAir)
  const qbar = 0.5 * atm.rho * V * V
  const mach = V / atm.a
  const alpha = Math.atan2(Math.hypot(vAirB.y, vAirB.z), vAirB.x)
  const stag: StagState | null = alt < 129e3 && V > 150 ? lookupStag(W.shock, Math.max(alt, 0), V) : null
  // ---------- pad frame ----------
  const padEci = rot(W.padEcef)
  const padE = rot(W.padEnu.e), padN = rot(W.padEnu.n), padU = rot(W.padEnu.u)
  const toPad = (x: V3): V3 => v3(dot(x, padE), dot(x, padN), dot(x, padU))
  const fromPad = (x: V3): V3 => add(add(scale(padE, x.x), scale(padN, x.y)), scale(padU, x.z))
  const relPad = toPad(sub(s.r, padEci))
  const velPad = toPad(vGround)
  const axisB = (x: number): V3 => qrot(s.q, v3(x - mp.xcg, 0, 0))
  const pinAxisPad = toPad(sub(add(s.r, axisB(veh.pinX)), padEci))
  const pinRel = sub(pinAxisPad, v3(0, 0, TOWER.armHeight))
  // aft-end height above ground: pad frame near the pad, geodetic far away (the pad frame's
  // "up" is tangent to the Earth at the pad, so distant points appear too low by d²/2R)
  const padDist = Math.hypot(relPad.x, relPad.y)
  const hBase = padDist < 25e3 ? toPad(sub(add(s.r, axisB(0)), padEci)).z : alt - W.plan.pad.alt + dot(up, axisB(0))
  // lowest point of the hull (aft rim, side or nose — whichever touches first), for ground contact
  const hNoseTip = hBase + dot(up, sub(axisB(veh.shape.p.length), axisB(0)))
  const axisUp = dot(up, qrot(s.q, v3(1, 0, 0)))
  const hLow = Math.min(hBase, hNoseTip) - veh.shape.p.radius * Math.sqrt(Math.max(0, 1 - axisUp * axisUp))
  // ---------- aerodynamics (held over the step) ----------
  const enginesFiring = s.eng.some((e) => e.spool > 0.3)
  const aero: AeroOut = aeroWrench(W.aero, {
    vAir: vAirB, w: s.w, rho: atm.rho, a: atm.a, mu: atm.mu, xcg: mp.xcg,
    cpMax: stag ? stag.cpMax : 1.84, enginesOn: enginesFiring, knudsen: (atm.mu / Math.max(atm.p, 1e-12)) * Math.sqrt((Math.PI * 287 * atm.T) / 2) / 9,
  })
  // ---------- MHD basis (10 Hz) ----------
  const NI = coilNI(veh.coil, s.coil)
  const vhatB = V > 1 ? scale(vAirB, 1 / V) : v3(1, 0, 0)
  const wantBasis = NI > 1e5 && stag !== null && stag.T0 > 1500 && qbar > 20 && alt < 120e3
  const basis = wantBasis
    ? s.t - s.mhdT >= 0.1 || s.mhdBasis === null
      ? mhdBasis(W.mhd, { NI, vhat: vhatB, U: V, rhoInf: atm.rho, pInf: atm.p, qInf: qbar, stag: stag!, xcg: mp.xcg, kappa: W.design.hallClosure, seedFraction: s.seeding ? W.design.seedFraction : 0 })
      : s.mhdBasis
    : null
  const mhdT = basis && basis !== s.mhdBasis ? s.t : s.mhdT
  // ---------- guidance ----------
  const thrusts = s.eng.map((e, i) => engineThrust(veh.engines[i], e, atm.p, mp.xcg).thrust)
  const accThrust = thrusts.reduce((a, b) => a + b, 0) / mp.m
  const nav: Nav = {
    t: s.t, r: s.r, v: s.v, q: s.q, w: s.w, I: mp.I, m: mp.m, xcg: mp.xcg, alt, up, east, north,
    vAir, V, mach, qbar, pAmb: atm.p, alpha, t0Stag: stag ? stag.T0 : 0, padEci,
    pinRel, velRel: velPad, hBase, hLow, mhd: basis, aeroM: aero.M, coilI: s.coil.I, coilIop: veh.coil.Iop,
    seedLeft: s.cons.seed, propHeader: s.cons.loxHeader + s.cons.ch4Header,
    thrusts, spoolDone: s.eng.slice(0, 3).filter((e) => e.on).every((e) => e.spool >= 1), pinLoad: 0, accThrust,
    mhdDrag: s.mhdOut.forceMag > 0 ? -dot(s.mhdOut.F, vhatB) : 0,
  }
  // pin load from the previous contact solution is approximated by evaluating contact now
  const pinsNow = [1, -1].map((sg) => {
    const p = toPad(sub(add(s.r, qrot(s.q, v3(veh.pinX - mp.xcg, sg * veh.pinOut, 0))), padEci))
    return { p, v: velPad }
  })
  const cNow = hBase < 300 ? towerContact(s.arms, relPad, pinsNow, []) : null
  const g = guide(W.guide, { ...nav, pinLoad: cNow ? cNow.pinLoad : 0 }, s.mem, dt)
  const cmd = g.cmd
  // ---------- control & allocation ----------
  // R6 (T08 P0-2): feed the guidance's own table moment, not the plant's exact `aero.M`.
  // The plant stays passive; the flight software no longer sees the true aerodynamic moment.
  const ff = cmd.feedForward ? cmd.ffMoment : ZERO
  const tc = torqueCommand(s.q, s.w, mp.I, cmd.att, cmd.gains, ff)
  let tauRes = tc.tau
  let mhdOut: MhdOut = ZERO_MHD
  let mhdU: readonly number[] = Array(8).fill(0)
  let mhdAuth = 0
  if (basis && cmd.useMhd) {
    const al = allocateMhd(basis, tauRes, qbar, veh.shape.p.noseRadius, U_BASE)
    mhdOut = al.out
    mhdU = al.u
    mhdAuth = al.maxMoment / Math.max(Math.hypot(tauRes.y, tauRes.z), 1)
    tauRes = sub(tauRes, mhdOut.M)
  }
  const engCmds: EngineCmd[] = s.eng.map((_, i) => cmd.engines[i] ?? { on: false, throttle: 0.4, gp: 0, gy: 0 })
  // cold-gas roll authority (strongest roll couple), with margin: the gimbal may not make more roll than this
  const rollCap = 0.7 * Math.max(...veh.couples.map((cp) => Math.abs(veh.thrusters[cp.a].thrust * (veh.thrusters[cp.a].pos.z * veh.thrusters[cp.a].dir.y - veh.thrusters[cp.a].pos.y * veh.thrusters[cp.a].dir.z) * 2)))
  // With the RCS off (flip, landing, terminal) the gimbal must take the full torque, including roll:
  // the old roll cap left that residual to cold gas, so the landing appeared to steer on thrusters.
  const gim = allocateGimbals(veh.engines.slice(0, 3), thrusts, mp.xcg, tauRes, cmd.useRcs && s.cons.n2 > 0 ? rollCap : Infinity)
  const eng = s.eng.map((e, i) => engineStep(veh.engines[i], e, { ...engCmds[i], gp: gim.gp[i] ?? 0, gy: gim.gy[i] ?? 0 }, dt))
  const feedMain = s.mem.phase === "deorbitBurn" || s.mem.phase === "deorbitAlign"
  const propAvail = feedMain ? s.cons.loxMain + s.cons.ch4Main : s.cons.loxHeader + s.cons.ch4Header
  let engOut: EngineOut = sumEngines(eng.map((e, i) => engineThrust(veh.engines[i], e, atm.p, mp.xcg)))
  // R4/R1 (T08 P1-5): an empty tank must not produce free Δv. The guidance already shuts down below
  // 50 kg, but a guidance failure must not turn the engine model into a reactionless thruster.
  if (propAvail <= 1) engOut = { F: v3(0, 0, 0), M: v3(0, 0, 0), mdot: 0, thrust: 0 }
  tauRes = sub(tauRes, engOut.M)
  // cold gas cannot help against MN·m pitch/yaw residuals of a saturated primary actuator:
  // keep roll, drop pitch/yaw unless the residual is within ~2× the couple capacity
  const coupleCap = veh.thrusters[0].thrust * (veh.thrusters[0].pos.x - veh.thrusters[4].pos.x)
  if (basis && cmd.useMhd && qbar > 1200 && mhdAuth > 1.2) tauRes = v3(tauRes.x, 0, 0) // coil has authority: cold gas only for roll
  else if (engOut.thrust > 1e5 && Math.hypot(tauRes.y, tauRes.z) > 2 * coupleCap) tauRes = v3(tauRes.x, 0, 0)
  const rcs = allocateRcs(veh.thrusters, veh.couples, mp.xcg, tauRes, cmd.useRcs && s.cons.n2 > 0)
  const rcsOut = rcsWrench(veh, rcs.duty, mp.xcg, s.cons.n2)
  // geomagnetic torque on the magnet dipole (matters in orbit)
  const bEci = rot(geomagneticEcef(ecef))
  const magT = alt > 90e3 ? dipoleTorque(NI, veh.coil.a, qrotInv(s.q, bEci)) : ZERO
  // ---------- rigid-body integration (RK4; body loads held, gravity & contact per stage) ----------
  const Fb = add(add(add(aero.F, mhdOut.F), engOut.F), rcsOut.F)
  const Mb = add(add(add(add(aero.M, mhdOut.M), engOut.M), rcsOut.M), magT)
  // Tower/arm collision must not depend on the guidance phase: a misaligned or lost ship still hits
  // Mechazilla instead of passing through it.
  const contactOn = hBase < 260 && Math.hypot(relPad.x, relPad.y) < 300
  const hullX = Array.from({ length: 20 }, (_, i) => veh.pinX - 6 + i * 1.0)
  let pinLoad = 0
  let bodyContact = false
  const deriv = (x: Rigid): DRigid => {
    let F = qrot(x.q, Fb)
    let M = Mb
    if (contactOn) {
      const rel = toPad(sub(x.r, padEci))
      const vg = toPad(sub(x.v, corotation(x.r)))
      const wI = qrot(x.q, x.w)
      const at = (b: V3): { p: V3; v: V3 } => {
        const off = qrot(x.q, b)
        return { p: add(rel, toPad(off)), v: add(vg, toPad(cross(wI, off))) }
      }
      const pins = [1, -1].map((sg) => at(v3(veh.pinX - mp.xcg, sg * veh.pinOut, 0)))
      const hull = hullX.map((xx) => ({ ...at(v3(xx - mp.xcg, 0, 0)), r: veh.shape.r(xx) }))
      const c = towerContact(s.arms, rel, pins, hull)
      pinLoad = c.pinLoad
      bodyContact = c.bodyContact
      F = add(F, fromPad(c.F))
      M = add(M, qrotInv(x.q, fromPad(c.M)))
    }
    const a = add(gravityJ2(x.r), scale(F, 1 / mp.m))
    const Iw = hadamard(mp.I, x.w)
    const wdot = v3((M.x - (x.w.y * Iw.z - x.w.z * Iw.y)) / mp.I.x, (M.y - (x.w.z * Iw.x - x.w.x * Iw.z)) / mp.I.y, (M.z - (x.w.x * Iw.y - x.w.y * Iw.x)) / mp.I.z)
    return { dr: x.v, dv: a, dq: qDot(x.q, x.w), dw: wdot }
  }
  const x0: Rigid = { r: s.r, v: s.v, q: s.q, w: s.w }
  const k1 = deriv(x0)
  const k2 = deriv(addR(x0, k1, dt / 2))
  const k3 = deriv(addR(x0, k2, dt / 2))
  const k4 = deriv(addR(x0, k3, dt))
  const comb = <T extends V3>(a: T, b: T, c: T, d: T): V3 => scale(add(add(a, scale(add(b, c), 2)), d), 1 / 6)
  const r = add(s.r, scale(comb(k1.dr, k2.dr, k3.dr, k4.dr), dt))
  const v = add(s.v, scale(comb(k1.dv, k2.dv, k3.dv, k4.dv), dt))
  const w = add(s.w, scale(comb(k1.dw, k2.dw, k3.dw, k4.dw), dt))
  const dq = { w: (k1.dq.w + 2 * k2.dq.w + 2 * k3.dq.w + k4.dq.w) / 6, x: (k1.dq.x + 2 * k2.dq.x + 2 * k3.dq.x + k4.dq.x) / 6, y: (k1.dq.y + 2 * k2.dq.y + 2 * k3.dq.y + k4.dq.y) / 6, z: (k1.dq.z + 2 * k2.dq.z + 2 * k3.dq.z + k4.dq.z) / 6 }
  const q = qnormalize(qAddScaled(s.q, dq, dt))
  // ---------- consumables ----------
  const mr = veh.mixtureRatio
  const prop = engOut.mdot * dt
  const fromMain = feedMain
  const seedFlow = s.seeding && basis ? basis.seedMassFlow : 0
  const cons = {
    ...s.cons,
    loxHeader: fromMain ? s.cons.loxHeader : Math.max(0, s.cons.loxHeader - (prop * mr) / (1 + mr)),
    ch4Header: fromMain ? s.cons.ch4Header : Math.max(0, s.cons.ch4Header - prop / (1 + mr)),
    loxMain: fromMain ? Math.max(0, s.cons.loxMain - (prop * mr) / (1 + mr)) : s.cons.loxMain,
    ch4Main: fromMain ? Math.max(0, s.cons.ch4Main - prop / (1 + mr)) : s.cons.ch4Main,
    n2: Math.max(0, s.cons.n2 - rcsOut.mdot * dt),
    seed: Math.max(0, s.cons.seed - seedFlow * dt),
  }
  // ---------- thermal (4 Hz) ----------
  const thermalDue = s.t - s.thT >= 0.25
  const hin = {
    rhoInf: atm.rho, pInf: atm.p, U: V, vhat: vhatB, Rn: veh.shape.p.noseRadius, qInf: qbar,
    mhdHeatFactor: mhdOut.heatFactor,
    stag: stag ?? { eps: 1, T2: atm.T, T0: atm.T, p0: atm.p, rho0: atm.rho, ne0: 0, n0: atm.n, nuEn0: 1, cpMax: 1.84, h0: 1004 * atm.T + 0.5 * V * V, gammaE: 1.4 },
  }
  const heat = thermalDue ? surfaceHeating(W.grid, hin) : null
  const th = heat ? thermalStep(W.grid, veh, s.th, heat, s.t - s.thT) : s.th
  const thT = thermalDue ? s.t : s.thT
  const tsum = thermalSummary(W.grid, th, veh.coil.x)
  // ---------- magnet, arms, turbulence ----------
  const coil = coilStep(veh.coil, s.coil, { command: cmd.coil, warmWallT: tsum.coilWallT, mhdPower: mhdOut.power, avionicsLoad: 2500 }, dt)
  const arms = armStep(s.arms, cmd.armsClose, dt)
  // ---------- bookkeeping ----------
  const fNonGrav = norm(add(qrot(s.q, Fb), ZERO)) / mp.m
  const gload = fNonGrav / G0
  const phase = g.mem.phase
  const events = g.events.length ? [...s.events, ...g.events.map((msg) => ({ t: s.t, msg }))] : s.events
  const caughtDone = phase === "caught" && s.t - g.mem.caughtT > W.guide.catchHold
  const lostDone = phase === "lost" && (g.mem.reason === "ground impact" || s.t - g.mem.tPhase > 3)
  const timeout = s.t > 3 * 3600
  const outcome = caughtDone ? "caught" : lostDone ? "lost" : timeout ? "timeout" : "running"
  const qStag = heat ? heat.qStagCold : s.qStag
  const peak = {
    g: Math.max(s.peak.g, phase === "caught" ? 0 : gload), qbar: Math.max(s.peak.qbar, qbar), qStag: Math.max(s.peak.qStag, qStag),
    picaSurf: Math.max(s.peak.picaSurf, tsum.picaSurfaceMax), bondline: Math.max(s.peak.bondline, tsum.bondlineMax),
    tileSurf: Math.max(s.peak.tileSurf, tsum.tileSurfaceMax), tileBack: Math.max(s.peak.tileBack, tsum.tileBackMax),
    steel: Math.max(s.peak.steel, tsum.steelMax), mhdForce: Math.max(s.peak.mhdForce, mhdOut.forceMag), coilT: Math.max(s.peak.coilT, coil.Tc),
  }
  const next: SimState = {
    t: s.t + dt, r, v, q, w, cons, eng, coil, th, thT, qStag, arms, turb, mem: g.mem,
    mhdBasis: basis, mhdT, mhdU, mhdOut, rcsDuty: rcs.duty, seeding: cmd.seeding,
    events, done: outcome !== "running", outcome, peak,
  }
  // ---------- diagnostics row ----------
  const row = new Float64Array(N_CH)
  const set = (k: keyof typeof CH, val: number): void => { row[CH[k]] = val }
  set("t", s.t); set("phase", phaseIndex(phase))
  set("rx", s.r.x); set("ry", s.r.y); set("rz", s.r.z); set("vx", s.v.x); set("vy", s.v.y); set("vz", s.v.z)
  set("qw", s.q.w); set("qx", s.q.x); set("qy", s.q.y); set("qz", s.q.z); set("wx", s.w.x); set("wy", s.w.y); set("wz", s.w.z)
  set("theta", theta); set("alt", alt); set("lat", geo.lat); set("lon", geo.lon)
  set("V", V); set("mach", mach); set("qbar", qbar); set("alpha", alpha); set("alphaCmd", cmd.alphaCmd)
  set("bank", (g.mem.phase === "highAlpha" || g.mem.phase === "bellyFlop" ? g.mem.bankHigh : g.mem.bank) * g.mem.bankSign); set("g", gload); set("mass", mp.m); set("xcg", mp.xcg); set("attErr", norm(tc.err))
  set("CA", aero.CA); set("CN", aero.CN); set("xcp", aero.xcp); set("aeroMy", aero.M.y); set("aeroMz", aero.M.z)
  if (stag) { set("T0", stag.T0); set("standoff0", 0.78 * stag.eps * veh.shape.p.noseRadius) }
  if (basis) {
    set("ne", basis.neStag); set("sigma", basis.sigmaStag); set("sigmaEff", basis.sigmaEffStag); set("hall", basis.hallStag)
    set("stuart", basis.qInteraction); set("standoff", basis.delta0)
  } else if (stag) set("standoff", 0.78 * stag.eps * veh.shape.p.noseRadius)
  set("bStag", NI * 2.0222959e-7 * (veh.coil.NI > 0 ? 1 : 0))
  set("mhdF", mhdOut.forceMag); set("mhdMy", mhdOut.M.y); set("mhdMz", mhdOut.M.z); set("mhdP", mhdOut.power)
  set("mhdScale", mhdOut.scale); set("heatFactor", mhdOut.heatFactor); set("mhdAuth", mhdAuth); set("seedFlow", seedFlow); set("seedLeft", cons.seed)
  const neAnt = basis ? basis.neStag * 0.02 : 0
  const fce = cyclotronFreq(W.mhd.antennaB * NI)
  set("blackout", neAnt > criticalDensity(RN_COMMS) ? (fce > RN_COMMS ? 2 : 1) : 0)
  mhdU.forEach((u, i) => { row[CH.u0 + i] = u })
  set("qStag", qStag); set("picaSurf", tsum.picaSurfaceMax); set("bondline", tsum.bondlineMax)
  set("tileSurf", tsum.tileSurfaceMax); set("tileBack", tsum.tileBackMax)
  set("steel", tsum.steelMax); set("steelX", tsum.steelMaxX); set("recession", tsum.recessionMax); set("turbFrac", heat ? heat.turbulentFrac : 0)
  set("coilI", coil.I); set("coilT", coil.Tc); set("coilE", coilEnergy(veh.coil, coil)); set("battery", coil.battery)
  set("coilMode", ["off", "ramp", "persistent", "rampdown", "quench", "dump", "discharge"].indexOf(coil.mode)); set("heatLeak", coil.heatLeak)
  set("n2", cons.n2); set("rcsTau", norm(rcs.achieved)); set("header", cons.loxHeader + cons.ch4Header)
  const tEach = eng.map((e, i) => engineThrust(veh.engines[i], e, atm.p, mp.xcg).thrust)
  set("thrust", engOut.thrust); set("T1", tEach[0]); set("T2", tEach[1]); set("T3", tEach[2]); set("gp", eng[0].gp); set("gy", eng[0].gy)
  rcs.duty.forEach((d, i) => { row[CH.r0 + i] = d })
  set("pe", relPad.x); set("pn", relPad.y); set("pu", relPad.z); set("ve", velPad.x); set("vn", velPad.y); set("vu", velPad.z)
  set("pinX", pinRel.x); set("pinY", pinRel.y); set("pinZ", pinRel.z); set("hBase", hBase); set("arms", arms.open)
  set("pinLoad", pinLoad); set("windE", windEnu.x); set("windN", windEnu.y)
  set("predDR", g.mem.pred ? g.mem.pred.downrangeErr : 0); set("predCR", g.mem.pred ? g.mem.pred.crossrangeErr : 0); set("npcMhd", g.mem.mhdScale); set("bellyHdg", g.mem.heading ?? 0); set("bellyMiss", g.mem.bellyMiss)
  void bodyContact
  void clamp
  void enuToEcef
  void ecefToEnu
  void normalize
  return { s: next, row, temps: surfaceTemps(W.grid, th), pred: g.mem.predPath }
}
