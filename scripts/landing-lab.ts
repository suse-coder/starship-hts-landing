/**
 * Landing lab: iterate on the flip/landing/catch from a saved state instead of from orbit (spec/12).
 *
 *   nice -n 19 bun scripts/landing-lab.ts --regen            run the mission once, save the snapshot at flip ignition
 *   nice -n 19 bun scripts/landing-lab.ts                    fly the landing from the snapshot (seconds)
 *   … --miss=300,-80        move the snapshot so the pins start 300 m east / 80 m south of the catch point
 *   … --tuning='{"k":v}'    MissionTuning overrides
 *   … --dt=1                trace interval (s);  --quiet  only the result line
 *
 * The snapshot is the full `SimState` (typed arrays and non-finite numbers preserved), so the
 * continuation is the same 6-DOF plant, not a surrogate.
 */
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs"
import { FLIGHT_DESIGN } from "@ship/craft"
import { OMEGA_EARTH, rotZ } from "@ship/earth"
import { type DescentPlan, MISSION_TUNING_DEFAULT, MissionTuning, sampleDescent } from "@ship/gnc"
import { add, cross as crossV, qFromAxisAngle, qmul, qrot, scale, sub, v3 } from "@ship/math"
import { CH, initState, makeWorld, step, type SimState, type World } from "@ship/sim"

const SNAP = ".tmp/snap/flip.json"
const SNAP_AT = (phase: string): string => `.tmp/snap/${phase}.json`
const arg = (k: string): string | undefined => process.argv.find((a) => a.startsWith(`--${k}=`))?.slice(k.length + 3)
const flag = (k: string): boolean => process.argv.includes(`--${k}`)

const encode = (s: unknown): string =>
  JSON.stringify(s, function (key, value) {
    const raw: unknown = (this as Record<string, unknown>)[key]
    if (raw instanceof Float64Array) return { __f64: Array.from(raw) }
    if (raw instanceof Float32Array) return { __f32: Array.from(raw) }
    if (raw instanceof Int32Array) return { __i32: Array.from(raw) }
    if (typeof raw === "number" && !Number.isFinite(raw)) return { __num: String(raw) }
    if (typeof raw === "function") throw new Error(`snapshot: function at '${key}'`)
    return value
  })
const decode = (text: string): unknown =>
  JSON.parse(text, (_k, v) => {
    if (v && typeof v === "object") {
      if ("__f64" in v) return Float64Array.from(v.__f64 as number[])
      if ("__f32" in v) return Float32Array.from(v.__f32 as number[])
      if ("__i32" in v) return Int32Array.from(v.__i32 as number[])
      if ("__num" in v) return Number(v.__num)
    }
    return v
  })

;(globalThis as { DBGPIN?: boolean }).DBGPIN = !!process.env.DBGPIN;(globalThis as { DBGH?: boolean }).DBGH = !!process.env.DBGH;(globalThis as { DBGBELLY?: boolean }).DBGBELLY = !!process.env.DBGBELLY
const tuningArg = arg("tuning")
const engOut = arg("engout")?.split(",").map(Number)
const base = makeWorld({
  ...FLIGHT_DESIGN,
  ...(flag("nowind") ? { windScale: 0, turbulence: 0 } : {}),
  ...(engOut && engOut.length === 2 ? { engineOut: { index: engOut[0], afterFlip: engOut[1] } } : {}),
})
const world: World = tuningArg ? { ...base, guide: { ...base.guide, tuning: MissionTuning.decode(JSON.parse(tuningArg)) } } : base

const regen = process.argv.find((a) => a === "--regen" || a.startsWith("--regen="))?.split("=")[1] ?? (flag("regen") ? "flip" : undefined)
if (regen || !existsSync(SNAP)) {
  const phase = regen ?? "flip"
  const t0 = performance.now()
  let s = initState(world)
  while (!s.done && s.mem.phase !== phase) s = step(world, s).s
  if (s.mem.phase !== phase) throw new Error(`mission ended in '${s.mem.phase}' before '${phase}': ${s.mem.reason}`)
  mkdirSync(".tmp/snap", { recursive: true })
  const path = `.tmp/snap/${phase}.json`
  writeFileSync(path, encode(s))
  console.log(`snapshot at ${phase}: t ${s.t.toFixed(1)} s, alt ${s.mem.phase}, saved ${path} (${((performance.now() - t0) / 1000).toFixed(0)} s)`)
  for (const e of s.events.slice(-6)) console.log(`  ${e.t.toFixed(1)} ${e.msg}`)
  if (phase !== "flip" || flag("stop")) process.exit(0)
}

const at = arg("at") ?? "flip"
if (flag("freeze")) (globalThis as { BELLY_FREEZE?: boolean }).BELLY_FREEZE = true
if (flag("plan")) (globalThis as { PLAN_DEBUG?: (s: string) => void }).PLAN_DEBUG = (m) => console.log(`  ${m}`)
let s = decode(readFileSync(arg("at") ? SNAP_AT(at) : SNAP, "utf8")) as SimState
// --heading=deg: rotate the whole state about the pad vertical so that the ground velocity points
// along that compass azimuth (0 = north: the approach into the open south side of the arms)
const heading = arg("heading")
if (heading !== undefined) {
  const row0 = step(world, s).row
  const cur = Math.atan2(row0[CH.ve], row0[CH.vn])
  const phi = -(Number(heading) * Math.PI / 180 - cur) // rotation about +up (counter-clockwise from above)
  const theta = world.plan.theta0 + OMEGA_EARTH * s.t
  const padEci = rotZ(world.padEcef, theta), up = rotZ(world.padEnu.u, theta)
  const qr = qFromAxisAngle(up, phi)
  const om = v3(0, 0, OMEGA_EARTH)
  const r1 = add(padEci, qrot(qr, sub(s.r, padEci)))
  const v1 = add(crossV(om, r1), qrot(qr, sub(s.v, crossV(om, s.r))))
  s = { ...s, r: r1, v: v1, q: qmul(qr, s.q) }
}
// --addprop=kg: extra landing propellant in the header tanks (sizing experiments; mixture ratio kept)
if (arg("addprop")) {
  const extra = Number(arg("addprop")), mr = world.vehicle.mixtureRatio
  s = { ...s, cons: { ...s.cons, loxHeader: s.cons.loxHeader + (extra * mr) / (1 + mr), ch4Header: s.cons.ch4Header + extra / (1 + mr) } }
}
const probe = step(world, s).row
const miss = arg("miss")?.split(",").map(Number)
if (miss && miss.length === 2) {
  const theta = world.plan.theta0 + OMEGA_EARTH * s.t
  const dE = miss[0] - probe[CH.pinX], dN = miss[1] - probe[CH.pinY]
  s = { ...s, r: add(s.r, add(scale(rotZ(world.padEnu.e, theta), dE), scale(rotZ(world.padEnu.n, theta), dN))) }
}
// ---- 6-DOF MPC search over the high-α hand-over aim -------------------------------
// The exact all-degree plant is the simulator itself: from the high-α snapshot, run the full
// `step` (high-α -> belly -> flip -> landing) for a candidate hand-over aim and score the actual
// catch. This is the full-spline model in the loop; `--mpc6=aim` searches the aim grid.
// ---- capture-set scan: fly the flip + landing from a grid of flip-ignition positions (pad ENU
// offsets of the pin axis) and report the outcome of each: --scan=centreE,centreN,span,n
if (arg("scan")) {
  const [cE, cN, span, nG] = arg("scan")!.split(",").map(Number)
  const theta = world.plan.theta0 + OMEGA_EARTH * s.t
  const p0 = step(world, s).row
  const eE = rotZ(world.padEnu.e, theta), eN = rotZ(world.padEnu.n, theta)
  let caughtN = 0
  for (let j = nG - 1; j >= 0; j--) {
    let line = ""
    for (let i = 0; i < nG; i++) {
      const tE = cE + (nG > 1 ? (i / (nG - 1) - 0.5) * span : 0), tN = cN + (nG > 1 ? (j / (nG - 1) - 0.5) * span : 0)
      let st: SimState = { ...s, r: add(s.r, add(scale(eE, tE - p0[CH.pinX]), scale(eN, tN - p0[CH.pinY]))) }
      let dMin = Infinity, vMin = 0, stg = "-"
      for (let k = 0; k < 200000 && !st.done; k++) {
        const out = step(world, st), r = out.row
        const d3 = Math.hypot(r[CH.pinX], r[CH.pinY], r[CH.pinZ])
        if (d3 < dMin) { dMin = d3; vMin = Math.hypot(r[CH.ve], r[CH.vn], r[CH.vu]) }
        if (stg === "-" && out.s.mem.land?.staged) stg = `${Math.hypot(r[CH.pinX], r[CH.pinY]).toFixed(0)}@${r[CH.pinZ].toFixed(0)}`
        st = out.s
      }
      const ok = st.mem.caughtT > 0
      if (ok) caughtN++
      const why = ok ? "" : st.mem.reason.includes("propellant") ? "fuel" : st.mem.reason.includes("missed") ? "miss" : st.mem.reason.slice(0, 6)
      // pin-axis azimuth error from east–west (mod 180°) and the roll rate at the end
      const thE = world.plan.theta0 + OMEGA_EARTH * st.t
      const byE = qrot(st.q, v3(0, 1, 0))
      const azP = Math.atan2(byE.x * rotZ(world.padEnu.n, thE).x + byE.y * rotZ(world.padEnu.n, thE).y + byE.z * rotZ(world.padEnu.n, thE).z, byE.x * rotZ(world.padEnu.e, thE).x + byE.y * rotZ(world.padEnu.e, thE).y + byE.z * rotZ(world.padEnu.e, thE).z) * 180 / Math.PI
      const azErr = ((azP + 90) % 180 + 180) % 180 - 90
      line += `  (${tE.toFixed(0)},${tN.toFixed(0)}) ${ok ? "CAUGHT" : ("lost:" + why).padEnd(10)} d ${dMin.toFixed(0).padStart(3)} v ${vMin.toFixed(0).padStart(2)} hdr ${((st.cons.loxHeader + st.cons.ch4Header) / 1e3).toFixed(1)} az ${azErr.toFixed(0).padStart(3)} wx ${(st.w.x * 57.3).toFixed(0).padStart(3)}`
    }
    console.log(line)
  }
  console.log(`scan: ${caughtN}/${nG * nG} caught`)
  process.exit(0)
}

if (arg("mpc6") === "gains") {
  const baseT = world.guide.tuning ?? MISSION_TUNING_DEFAULT
  const gains = [[0.1, 0.5], [0.3, 1.0], [0.5, 1.5], [0.2, 1.2]]
  const aims = [450, 600]
  interface GainRes { aim: number; kp: number; kd: number; d: number; v: number; caught: boolean; phase: string }
  const res: GainRes[] = []
  for (const aim of aims) for (const [kp, kd] of gains) {
    const w2 = { ...world, guide: { ...world.guide, tuning: baseT.with({ aimHigh: aim, bellyAimE: -200, bellyAimN: 200, landKp: kp, landKd: kd }) } }
    let st = structuredClone(s)
    let dMin = Infinity, vMin = Infinity
    for (let i = 0; i < 600000 && !st.done; i++) {
      const out = step(w2, st), r = out.row
      const dh = Math.hypot(r[CH.pinX], r[CH.pinY])
      if (dh < 20e3) {
        const d3 = Math.hypot(r[CH.pinX], r[CH.pinY], r[CH.pinZ])
        if (d3 < dMin) { dMin = d3; vMin = Math.hypot(r[CH.ve], r[CH.vn], r[CH.vu]) }
      }
      st = out.s
      if (st.mem.phase === "lost" || st.mem.phase === "caught") break
    }
    res.push({ aim, kp, kd, d: dMin, v: vMin, caught: st.mem.caughtT > 0, phase: st.mem.phase })
    console.log(`  mpc6 aim ${aim} kp ${kp} kd ${kd} → closest ${dMin.toFixed(1)} m at ${vMin.toFixed(1)} m/s ${st.mem.caughtT > 0 ? "CAUGHT" : st.mem.phase}`)
  }
  res.sort((x, y) => (Number(y.caught) - Number(x.caught)) || (x.d + 0.2 * x.v) - (y.d + 0.2 * y.v))
  console.log(`  mpc6 gains best aim ${res[0].aim} kp ${res[0].kp} kd ${res[0].kd} → ${res[0].d.toFixed(1)} m at ${res[0].v.toFixed(1)} m/s ${res[0].caught ? "CAUGHT" : res[0].phase}`)
  process.exit(0)
}

if (arg("mpc6") === "aim" || arg("mpc6") === "full") {
  const baseT = world.guide.tuning ?? MISSION_TUNING_DEFAULT
  const aims = arg("mpc6") === "full" ? [300, 450, 600] : [0, 150, 300, 450, 600, 900, 1200]
  const bellyAims = arg("mpc6") === "full" ? [-200, 0, 200] : [0]
  interface Xfer { d: number; h: number; v: number }
  interface AimRes { aim: number; be: number; bn: number; d: number; v: number; caught: boolean; phase: string; fi: Xfer | null; fc: Xfer | null }
  const res: AimRes[] = []
  for (const aim of aims) for (const be of bellyAims) for (const bn of bellyAims) {
    const w2 = { ...world, guide: { ...world.guide, tuning: baseT.with({ aimHigh: aim, bellyAimE: be, bellyAimN: bn }) } }
    let st = structuredClone(s)
    let dMin = Infinity, vMin = Infinity, fi: Xfer | null = null, fc: Xfer | null = null
    let prevPhase = st.mem.phase
    for (let i = 0; i < 600000 && !st.done; i++) {
      const out = step(w2, st), r = out.row
      const ph = out.s.mem.phase
      const xf: Xfer = { d: Math.hypot(r[CH.pinX], r[CH.pinY]), h: r[CH.pinZ], v: Math.hypot(r[CH.ve], r[CH.vn], r[CH.vu]) }
      if (prevPhase === "bellyFlop" && ph === "flip") fi = xf
      if (prevPhase === "flip" && (ph === "landing" || ph === "terminal")) fc = xf
      prevPhase = ph
      const dh = Math.hypot(r[CH.pinX], r[CH.pinY])
      if (dh < 20e3) {
        const d3 = Math.hypot(r[CH.pinX], r[CH.pinY], r[CH.pinZ])
        if (d3 < dMin) { dMin = d3; vMin = Math.hypot(r[CH.ve], r[CH.vn], r[CH.vu]) }
      }
      st = out.s
      if (ph === "lost" || ph === "caught") break
    }
    res.push({ aim, be, bn, d: dMin, v: vMin, caught: st.mem.caughtT > 0, phase: st.mem.phase, fi, fc })
    console.log(`  mpc6 aim ${aim} bellyAim ${be}/${bn} → flip ${fi ? fi.d.toFixed(0) : "?"} m h ${fi ? (fi.h / 1000).toFixed(1) : "?"} km v ${fi ? fi.v.toFixed(0) : "?"} | comp ${fc ? fc.d.toFixed(0) : "?"} m | closest ${dMin.toFixed(1)} m at ${vMin.toFixed(1)} m/s ${st.mem.caughtT > 0 ? "CAUGHT" : st.mem.phase}`)
  }
  res.sort((x, y) => (Number(y.caught) - Number(x.caught)) || (x.d + 0.2 * x.v) - (y.d + 0.2 * y.v))
  const bestFi = [...res].sort((x, y) => (x.fi?.d ?? 1e9) - (y.fi?.d ?? 1e9))
  for (const r of bestFi.slice(0, 6)) console.log(`  mpc6 flip-best aim ${r.aim} bellyAim ${r.be}/${r.bn} → flip ${r.fi ? r.fi.d.toFixed(0) : "?"} m h ${r.fi ? (r.fi.h / 1000).toFixed(1) : "?"} km v ${r.fi ? r.fi.v.toFixed(0) : "?"} | comp ${r.fc ? r.fc.d.toFixed(0) : "?"} m | catch ${r.d.toFixed(0)} m @ ${r.v.toFixed(0)} m/s`)
  for (const r of res.slice(0, 4)) console.log(`  mpc6 catch-best aim ${r.aim} bellyAim ${r.be}/${r.bn} → flip ${r.fi ? r.fi.d.toFixed(0) : "?"} m | comp ${r.fc ? r.fc.d.toFixed(0) : "?"} m | ${r.d.toFixed(1)} m at ${r.v.toFixed(1)} m/s ${r.caught ? "CAUGHT" : r.phase}`)
  process.exit(0)
}

// ---- 6-DOF MPC search over the belly bank schedule --------------------------------
// The exact all-degree plant is the simulator itself: clone the current SimState, force a
// candidate two-segment lift-azimuth schedule, run the same `step` (belly -> flip -> landing) to
// the catch, and score the actual closest approach and speed. This is the 6-DOF model in the loop,
// not the 3-DOF `bellyEnd` proxy. `--mpc6` searches a coarse grid and prints the best candidates.
if (flag("mpc6")) {
  (globalThis as { BELLY_FREEZE?: boolean }).BELLY_FREEZE = true
  const D = 180 / Math.PI
  const probe = step(world, s).row
  const qq = { w: probe[CH.qw], x: probe[CH.qx], y: probe[CH.qy], z: probe[CH.qz] }
  const xb = [1 - 2 * (qq.y * qq.y + qq.z * qq.z), 2 * (qq.x * qq.y + qq.w * qq.z), 2 * (qq.x * qq.z - qq.w * qq.y)]
  const Lg = probe[CH.lon] + probe[CH.theta], la = probe[CH.lat]
  const up = [Math.cos(la) * Math.cos(Lg), Math.cos(la) * Math.sin(Lg), Math.sin(la)]
  const east = [-Math.sin(Lg), Math.cos(Lg), 0]
  const north = [up[1] * east[2] - up[2] * east[1], up[2] * east[0] - up[0] * east[2], up[0] * east[1] - up[1] * east[0]]
  const dot3 = (a: number[], b: number[]): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
  const v0 = [probe[CH.ve], probe[CH.vn], probe[CH.vu]], vn0 = Math.hypot(v0[0], v0[1], v0[2]) || 1
  const u0 = v0.map((x) => x / vn0)
  const d0 = dot3(xb, u0)
  const l0 = [xb[0] - d0 * u0[0], xb[1] - d0 * u0[1], xb[2] - d0 * u0[2]]
  const hc0 = s.mem.heading ?? Math.atan2(dot3(l0, east), dot3(l0, north))
  interface MpcRes { a: number; tau: number; b: number; d: number; v: number; caught: boolean; phase: string }
  const fly = (a: number, tau: number, b: number): MpcRes => {
    let st = structuredClone(s)
    st = { ...st, mem: { ...st.mem, heading: a, tSwitch: st.t + tau, sigma2: b, npcT: st.t } }
    let dMin = Infinity, vMin = Infinity
    for (let i = 0; i < 300000 && !st.done; i++) {
      const out = step(world, st), r = out.row
      const dh = Math.hypot(r[CH.pinX], r[CH.pinY])
      if (dh < 20e3) {
        const d3 = Math.hypot(r[CH.pinX], r[CH.pinY], r[CH.pinZ])
        if (d3 < dMin) { dMin = d3; vMin = Math.hypot(r[CH.ve], r[CH.vn], r[CH.vu]) }
      }
      st = out.s
      if (st.mem.phase === "lost" || st.mem.phase === "caught") break
    }
    return { a, tau, b, d: dMin, v: vMin, caught: st.mem.caughtT > 0, phase: st.mem.phase }
  }
  const offs = [-60, -30, 0, 30, 60]
  const taus = [30, 60, 90, 120, Infinity]
  const offs2 = [-60, 0, 60]
  const res: MpcRes[] = []
  let n = 0
  for (const o1 of offs) for (const tau of taus) for (const o2 of offs2) {
    const a = hc0 + o1 / D, b = (tau === Infinity ? a : a + o2 / D)
    const rr = fly(a, tau, b)
    res.push(rr)
    n++
    if (n % 10 === 0) console.log(`  mpc6 ${n}/${offs.length * taus.length * offs2.length} best ${Math.min(...res.map((x) => x.d + 0.2 * x.v)).toFixed(0)} m`)
  }
  res.sort((x, y) => (Number(y.caught) - Number(x.caught)) || (x.d + 0.2 * x.v) - (y.d + 0.2 * y.v))
  for (const r of res.slice(0, 12)) {
    console.log(`  mpc6 ψ1 ${(r.a * D).toFixed(0)}° τ ${r.tau === Infinity ? "inf" : r.tau} ψ2 ${(r.b * D).toFixed(0)}° → closest ${r.d.toFixed(1)} m at ${r.v.toFixed(1)} m/s ${r.caught ? "CAUGHT" : r.phase}`)
  }
  process.exit(0)
}

const dtp = Number(arg("dt") ?? 1), quiet = flag("quiet")
let trackSum = 0, trackN = 0, trackMax = 0, resMax = 0
const t0 = performance.now(), tStart = s.t, nEv = s.events.length
let last = -1e9, lastBank = -1e9, lastDbg = -1e9, lastBelly = -1e9, cross: Float64Array | null = null, prevRow: Float64Array | null = null, minLat = Infinity, steps = 0
let minRow: Float64Array | null = null
const D = 180 / Math.PI
while (!s.done && s.t - tStart < 1200) {
  const out = step(world, s)
  const r = out.row
  steps++
  if (!cross && prevRow && prevRow[CH.pinZ] > 0 && r[CH.pinZ] <= 0) cross = r.slice()
  // closest approach to the catch point in 3-D, only near the pad (the tangent-plane height is
  // wrong by d²/2R far away: an earlier version reported 780 km here, which was this artifact)
  if (Math.hypot(r[CH.pinX], r[CH.pinY]) < 20e3) {
    const d3 = Math.hypot(r[CH.pinX], r[CH.pinY], r[CH.pinZ])
    if (d3 < minLat) { minLat = d3; minRow = r.slice() }
  }
  if (!quiet && (s.t - last >= dtp || out.s.done)) {
    last = s.t
    const u = out.s.mem.uCmd
    const q = { w: r[CH.qw], x: r[CH.qx], y: r[CH.qy], z: r[CH.qz] }
    const xb = [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y)]
    const Lg = r[CH.lon] + r[CH.theta], la = r[CH.lat]
    const up = xb[0] * Math.cos(la) * Math.cos(Lg) + xb[1] * Math.cos(la) * Math.sin(Lg) + xb[2] * Math.sin(la)
    console.log(
      `${s.t.toFixed(1)} ${out.s.mem.phase.padEnd(8)} pin ${r[CH.pinX].toFixed(1).padStart(7)},${r[CH.pinY].toFixed(1).padStart(7)},${r[CH.pinZ].toFixed(1).padStart(7)}` +
      ` v ${r[CH.ve].toFixed(1).padStart(6)},${r[CH.vn].toFixed(1).padStart(6)},${r[CH.vu].toFixed(1).padStart(7)}` +
      ` u ${u[0].toFixed(1).padStart(5)},${u[1].toFixed(1).padStart(5)},${u[2].toFixed(1).padStart(5)} |T/m| ${(r[CH.thrust] / r[CH.mass]).toFixed(1).padStart(5)}` +
      ` tilt cmd/body ${(Math.atan2(Math.hypot(u[0], u[1]), Math.max(u[2], 1e-3)) * D).toFixed(0).padStart(3)}/${(Math.acos(Math.max(-1, Math.min(1, up))) * D).toFixed(0).padStart(3)}` +
      ` az cmd/body ${(Math.atan2(u[0], u[1]) * D).toFixed(0).padStart(4)}/${(Math.atan2(xb[0] * -Math.sin(Lg) + xb[1] * Math.cos(Lg), -xb[0] * Math.sin(la) * Math.cos(Lg) - xb[1] * Math.sin(la) * Math.sin(Lg) + xb[2] * Math.cos(la)) * D).toFixed(0).padStart(4)} gim ${(r[CH.gp1] * D).toFixed(0)},${(r[CH.gy1] * D).toFixed(0)}/${(r[CH.gp2] * D).toFixed(0)},${(r[CH.gy2] * D).toFixed(0)} wx ${(r[CH.wx] * D).toFixed(0).padStart(3)} T ${(r[CH.T1] / 1e6).toFixed(2)},${(r[CH.T2] / 1e6).toFixed(2)},${(r[CH.T3] / 1e6).toFixed(2)} hdr ${(r[CH.header] / 1e3).toFixed(1)} arms ${r[CH.arms].toFixed(2)} pins ${r[CH.pinCount].toFixed(0)}` +
      ` m ${(r[CH.mass] / 1e3).toFixed(0)}t eng ${out.s.mem.engSel.join("")} bmiss ${out.s.mem.bellyMiss.toFixed(0)} hdg ${out.s.mem.hdgCmd === null ? "-" : ((out.s.mem.hdgCmd * 180) / Math.PI).toFixed(0)}`,
    )
  }
  if (flag("dbg") && s.mem.land?.plan && (s.mem.phase === "landing" || s.mem.phase === "terminal") && s.t - lastDbg >= 0.5) {
    lastDbg = s.t
    const pl: DescentPlan = s.mem.land.plan
    const ref = sampleDescent(pl, s.t)
    const d = out.s.mem.landResidual
    console.log(`  DBG ${s.t.toFixed(1)} tgo ${(pl.t0 + pl.tf - s.t).toFixed(1)} lvl ${s.mem.land.level}${pl.soft ? " SOFT" : ""} ref p (${ref.p.x.toFixed(1)},${ref.p.y.toFixed(1)},${ref.p.z.toFixed(1)}) v (${ref.v.x.toFixed(1)},${ref.v.y.toFixed(1)},${ref.v.z.toFixed(1)}) u (${ref.u.x.toFixed(1)},${ref.u.y.toFixed(1)},${ref.u.z.toFixed(1)}) | err p (${(r[CH.pinX] - ref.p.x).toFixed(1)},${(r[CH.pinY] - ref.p.y).toFixed(1)},${(r[CH.pinZ] - ref.p.z).toFixed(1)}) v (${(r[CH.ve] - ref.v.x).toFixed(1)},${(r[CH.vn] - ref.v.y).toFixed(1)},${(r[CH.vu] - ref.v.z).toFixed(1)}) | residual (${d.x.toFixed(2)},${d.y.toFixed(2)},${d.z.toFixed(2)})`)
  }
  if (flag("bank") && s.t - lastBank >= 5 && (s.mem.phase === "bellyFlop" || s.mem.phase === "highAlpha")) {
    lastBank = s.t
    const q = { w: r[CH.qw], x: r[CH.qx], y: r[CH.qy], z: r[CH.qz] }
    const xb = [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y)]
    const Lg = r[CH.lon] + r[CH.theta], la = r[CH.lat]
    const up = [Math.cos(la) * Math.cos(Lg), Math.cos(la) * Math.sin(Lg), Math.sin(la)]
    const v = [r[CH.vx] ?? 0, 0, 0]
    void v; void up; void Lg
    const ve = r[CH.ve], vn = r[CH.vn], vu = r[CH.vu]
    const vh = [ve, vn, vu]
    const vn2 = Math.hypot(vh[0], vh[1], vh[2]) || 1
    const vhat = vh.map((x) => x / vn2)
    const dot = (a: number[], b: number[]): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    const lb = xb.map((x, i) => x - vhat[i] * dot(xb, vhat))
    const ln = Math.hypot(lb[0], lb[1], lb[2]) || 1
    const l = lb.map((x) => x / ln)
    const upv = [0, 0, 1].map((x, i) => x - vhat[i] * dot([0, 0, 1], vhat))
    const un = Math.hypot(upv[0], upv[1], upv[2]) || 1
    const u = upv.map((x) => x / un)
    const w = [vhat[1] * u[2] - vhat[2] * u[1], vhat[2] * u[0] - vhat[0] * u[2], vhat[0] * u[1] - vhat[1] * u[0]]
    const bank = (Math.atan2(dot(l, w), dot(l, u)) * 180) / Math.PI
    const gp = (s.eng[0].gp * 180) / Math.PI, gy = (s.eng[0].gy * 180) / Math.PI
    console.log(`  BANK t ${s.t.toFixed(1)} ${s.mem.phase} flown ${bank.toFixed(1)} cmd ${s.mem.sigmaCmd === null ? "-" : ((s.mem.sigmaCmd * 180) / Math.PI).toFixed(1)} tgt ${((s.mem.sigma2 * 180) / Math.PI).toFixed(1)} hdg ${s.mem.hdgCmd === null ? "-" : ((s.mem.hdgCmd * 180) / Math.PI).toFixed(1)} gim ${gp.toFixed(1)},${gy.toFixed(1)} q ${(r[CH.qbar] / 1000).toFixed(1)} scale ${s.mem.bankRateScale.toFixed(2)} tgt ${((s.mem.sigmaTgt * 180) / Math.PI).toFixed(0)}`)
  }
  // plan-vs-flown tracking error and the model residual, over the planned descent
  const lp = s.mem.land?.plan
  if (lp && (s.mem.phase === "landing" || s.mem.phase === "terminal")) {
    const ref = sampleDescent(lp, s.t)
    const e = Math.hypot(r[CH.pinX] - ref.p.x, r[CH.pinY] - ref.p.y, r[CH.pinZ] - ref.p.z)
    trackSum += e * e; trackN++; trackMax = Math.max(trackMax, e)
    const d = out.s.mem.landResidual
    resMax = Math.max(resMax, Math.hypot(d.x, d.y, d.z))
  }
  if (flag("belly") && s.mem.phase === "bellyFlop" && prevRow && s.t - lastBelly >= 3) {
    lastBelly = s.t
    const dtb = r[CH.t] - prevRow[CH.t]
    if (dtb > 1e-4) {
      // horizontal non-gravitational acceleration = aero (engines off)
      const ae = (r[CH.ve] - prevRow[CH.ve]) / dtb, an = (r[CH.vn] - prevRow[CH.vn]) / dtb
      const vhAz = Math.atan2(r[CH.ve], r[CH.vn]) * D
      // lateral (⟂ horizontal velocity) part of it
      const vhn = Math.hypot(r[CH.ve], r[CH.vn]) || 1
      const along = (ae * r[CH.ve] + an * r[CH.vn]) / vhn
      const le = ae - along * r[CH.ve] / vhn, ln = an - along * r[CH.vn] / vhn
      console.log(`  BELLY t ${s.t.toFixed(0)} α ${(r[CH.alpha] * D).toFixed(0)}° M ${r[CH.mach].toFixed(2)} hdgCmd ${s.mem.hdgCmd === null ? "-" : (s.mem.hdgCmd * D).toFixed(0)} | lateral aero ${Math.hypot(le, ln).toFixed(2)} m/s² toward ${(Math.atan2(le, ln) * D).toFixed(0)}° | along-track ${along.toFixed(2)} | v az ${vhAz.toFixed(0)}° | plan ψ1 ${(s.mem.heading === null ? NaN : s.mem.heading * D).toFixed(0)} τ ${(s.mem.tSwitch - s.t).toFixed(0)} ψ2 ${(s.mem.sigma2 * D).toFixed(0)} miss ${s.mem.bellyMiss.toFixed(0)} | dob (${s.mem.bellyDist.x.toFixed(2)},${s.mem.bellyDist.y.toFixed(2)},${s.mem.bellyDist.z.toFixed(2)}) | pos ${r[CH.pinX].toFixed(0)},${r[CH.pinY].toFixed(0)} h ${r[CH.hBase].toFixed(0)}`)
    }
  }
  prevRow = r
  s = out.s
}
if (!quiet) for (const e of s.events.slice(nEv)) console.log(`  ${e.t.toFixed(1)} ${e.msg}`)
const c = cross
if (trackN > 0) console.log(`TRACKING plan-vs-flown: RMS ${Math.sqrt(trackSum / trackN).toFixed(2)} m, max ${trackMax.toFixed(2)} m; model residual max ${resMax.toFixed(2)} m/s²`)
console.log(
  `RESULT ${s.outcome}${s.mem.reason ? ` (${s.mem.reason})` : ""}: ` +
  (c ? `rail-plane crossing across ${c[CH.pinX].toFixed(2)} m, along ${c[CH.pinY].toFixed(2)} m, v (${c[CH.ve].toFixed(2)}, ${c[CH.vn].toFixed(2)}, ${c[CH.vu].toFixed(2)}) m/s, arms ${c[CH.arms].toFixed(2)}` : "never crossed the rail plane") +
  `; closest 3-D approach to the catch ${minRow ? `${minLat.toFixed(1)} m (E ${minRow[CH.pinX].toFixed(1)}, N ${minRow[CH.pinY].toFixed(1)}, up ${minRow[CH.pinZ].toFixed(1)}, speed ${Math.hypot(minRow[CH.ve], minRow[CH.vn], minRow[CH.vu]).toFixed(1)} m/s)` : "–"}; end state E ${prevRow ? prevRow[CH.pinX].toFixed(0) : "?"} N ${prevRow ? prevRow[CH.pinY].toFixed(0) : "?"} up ${prevRow ? prevRow[CH.pinZ].toFixed(0) : "?"} m; header ${((s.cons.loxHeader + s.cons.ch4Header) / 1e3).toFixed(2)} t; N2 ${s.cons.n2.toFixed(0)} kg; ${(s.t - tStart).toFixed(1)} s flown, ${steps} steps, ${((performance.now() - t0) / 1000).toFixed(1)} s wall`,
)
