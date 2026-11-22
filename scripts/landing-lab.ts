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
import { DEFAULT_DESIGN } from "@ship/craft"
import { OMEGA_EARTH, rotZ } from "@ship/earth"
import { type DescentPlan, MissionTuning, sampleDescent } from "@ship/gnc"
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

const tuningArg = arg("tuning")
const engOut = arg("engout")?.split(",").map(Number)
const base = makeWorld({
  ...DEFAULT_DESIGN,
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
const probe = step(world, s).row
const miss = arg("miss")?.split(",").map(Number)
if (miss && miss.length === 2) {
  const theta = world.plan.theta0 + OMEGA_EARTH * s.t
  const dE = miss[0] - probe[CH.pinX], dN = miss[1] - probe[CH.pinY]
  s = { ...s, r: add(s.r, add(scale(rotZ(world.padEnu.e, theta), dE), scale(rotZ(world.padEnu.n, theta), dN))) }
}
const dtp = Number(arg("dt") ?? 1), quiet = flag("quiet")
let trackSum = 0, trackN = 0, trackMax = 0, resMax = 0
const t0 = performance.now(), tStart = s.t, nEv = s.events.length
let last = -1e9, lastBank = -1e9, lastDbg = -1e9, cross: Float64Array | null = null, prevRow: Float64Array | null = null, minLat = Infinity, steps = 0
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
      ` att ${(r[CH.attErr] * D).toFixed(0).padStart(3)} T ${(r[CH.T1] / 1e6).toFixed(2)},${(r[CH.T2] / 1e6).toFixed(2)},${(r[CH.T3] / 1e6).toFixed(2)} hdr ${(r[CH.header] / 1e3).toFixed(1)} arms ${r[CH.arms].toFixed(2)} pins ${r[CH.pinCount].toFixed(0)}` +
      ` eng ${out.s.mem.engSel.join("")}`,
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
