/**
 * Entry altitude-profile diagnostics (task T11). Runs one mission and prints
 *   - the event log, phase starts and summary (as scripts/mission.ts),
 *   - a 10 s time series from the end of entry to the flip: h, γ, V, q, g, α, flown bank σ, steel,
 *     bondline, N₂, and the loft force balance n_L = (q S C_L / m) / (g − V²/r),
 *   - phugoid metrics of the high-α glide from the recorded altitude: period, amplitudes, log
 *     decrement δ and damping ratio ζ = δ / √(4π² + δ²),
 * and writes the same numbers as JSON to .tmp/agentE/<label>.json.
 *
 *   bun scripts/profile.ts [label]
 */
import { mkdirSync, writeFileSync } from "node:fs"
import { DEFAULT_DESIGN, rcsWrench } from "@ship/craft"
import { OMEGA_EARTH } from "@ship/earth"
import { PHASES } from "@ship/gnc"
import { CH, initState, makeWorld, recorder, step, type SimState } from "@ship/sim"
import { PHASES_OF } from "./missionHelpers"

const label = process.argv[2] ?? "run"
// optional transition study: bun scripts/profile.ts <label> <qPitch Pa> (guidance only; the plan keeps the default)
const qPitchOverride = process.argv[3] !== undefined ? Number(process.argv[3]) : null
const t0 = performance.now()
// the run loop of @ship/sim runMission (same stepping and recording intervals), plus per-phase
// actuator bookkeeping that the recording does not carry (MHD roll torque, cold-gas torque by axis)
const world0 = makeWorld(DEFAULT_DESIGN, (stage) => console.log(`[plan] ${stage}`))
const world = qPitchOverride === null ? world0 : { ...world0, guide: { ...world0.guide, pred: { ...world0.guide.pred, qPitch: qPitchOverride } } }
const recDt = (s: SimState): number => {
  switch (s.mem.phase) {
    case "deorbitAlign": case "entryAlign": return 2.0
    case "deorbitBurn": case "entry": case "loft": return 0.25
    case "pitchUp": return 0.1
    case "highAlpha": case "bellyFlop": return 0.2
    case "flip": case "landing": return 0.05
    default: return 0.04
  }
}
interface Acc { time: number; n2: number; mhdRoll: number; rcsRoll: number; mhdPY: number; rcsPY: number; coilOn: number }
const acc: Record<string, Acc> = {}
let s = initState(world)
const recB = recorder(world.grid.cellX.length)
const phaseStarts: { phase: string; t: number }[] = [{ phase: s.mem.phase, t: 0 }]
let lastRec = -1e9
let steps = 0
let coilLastOn = -1
while (!s.done) {
  const out = step(world, s)
  steps++
  const dt = out.s.t - s.t
  const a = (acc[s.mem.phase] ??= { time: 0, n2: 0, mhdRoll: 0, rcsRoll: 0, mhdPY: 0, rcsPY: 0, coilOn: 0 })
  const rcsM = rcsWrench(world.vehicle, out.s.rcsDuty, out.row[CH.xcg], s.cons.n2).M
  const mM = out.s.mhdOut.M
  a.time += dt
  a.n2 += s.cons.n2 - out.s.cons.n2
  a.mhdRoll += Math.abs(mM.x) * dt
  a.rcsRoll += Math.abs(rcsM.x) * dt
  a.mhdPY += Math.hypot(mM.y, mM.z) * dt
  a.rcsPY += Math.hypot(rcsM.y, rcsM.z) * dt
  if (Math.abs(out.row[CH.coilI]) > 50) { a.coilOn += dt; coilLastOn = out.s.t }
  if (out.s.mem.phase !== s.mem.phase) phaseStarts.push({ phase: out.s.mem.phase, t: out.s.t })
  if (s.t - lastRec >= recDt(s) || out.s.done || out.s.mem.phase !== s.mem.phase) {
    recB.push(out.row, out.temps)
    lastRec = s.t
  }
  s = out.s
}
const final = s
const rec = recB.finish(s.events, {
  epochJd: world.plan.epochJd, theta0: world.plan.theta0, padLat: world.plan.pad.lat, padLon: world.plan.pad.lon, padAlt: world.plan.pad.alt,
  outcome: s.outcome, phaseStarts, predPath: [], vehicle: { phases: PHASES }, wallClockMs: performance.now() - t0, steps,
  summary: {
    outcome: s.outcome, reason: s.mem.reason, duration: s.t, peakG: s.peak.g, peakQbar: s.peak.qbar, peakQStag: s.peak.qStag,
    peakPicaSurf: s.peak.picaSurf, peakBondline: s.peak.bondline, peakSteel: s.peak.steel, peakMhdForce: s.peak.mhdForce,
    peakCoilT: s.peak.coilT, n2Left: s.cons.n2, seedLeft: s.cons.seed,
  },
})
const wall = (performance.now() - t0) / 1000
// (consecutive duplicate events, e.g. a per-step "header tanks depleted", are printed once)
rec.events.forEach((e, k) => { if (k === 0 || e.msg !== rec.events[k - 1].msg) console.log(`${e.t.toFixed(1).padStart(8)} s  ${e.msg}`) })
console.log("\nphases:", rec.meta.phaseStarts.map((p) => `${p.phase}@${p.t.toFixed(0)}`).join("  "))
console.log("summary:", rec.meta.summary)
console.log(`samples ${rec.n}, wall ${wall.toFixed(1)} s, outcome ${final.outcome}`)

const d = rec.data
const n = rec.n
const S = world.aero.sref
const DEG = 180 / Math.PI
const ph = (i: number): string => PHASES_OF(d[CH.phase][i])
const start = (p: string): number => rec.meta.phaseStarts.find((x) => x.phase === p)?.t ?? NaN
const tLoft = start("loft"), tPitch = start("pitchUp"), tHigh = start("highAlpha"), tBelly = start("bellyFlop"), tFlip = start("flip")

interface Row { t: number; phase: string; h: number; gam: number; hdot: number; V: number; M: number; q: number; g: number; a: number; sig: number; sigCmd: number; steel: number; bond: number; n2: number; nL: number; coilT: number; dr: number; cr: number; coilI: number }
const rowAt = (i: number): Row => {
  const r = [d[CH.rx][i], d[CH.ry][i], d[CH.rz][i]]
  const v = [d[CH.vx][i], d[CH.vy][i], d[CH.vz][i]]
  const rn = Math.hypot(r[0], r[1], r[2])
  const vr = [v[0] + OMEGA_EARTH * r[1], v[1] - OMEGA_EARTH * r[0], v[2]] // v − ω × r
  const Vr = Math.hypot(vr[0], vr[1], vr[2])
  const up = r.map((x) => x / rn)
  const hdot = up[0] * vr[0] + up[1] * vr[1] + up[2] * vr[2]
  const vh = vr.map((x) => x / Vr)
  // body x from the quaternion; flown bank = angle of its component ⟂ v̂ from the in-plane "up"
  const qw = d[CH.qw][i], qx = d[CH.qx][i], qy = d[CH.qy][i], qz = d[CH.qz][i]
  const xb = [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy)]
  const dv = xb[0] * vh[0] + xb[1] * vh[1] + xb[2] * vh[2]
  const lb = xb.map((x, k) => x - dv * vh[k])
  const du = up[0] * vh[0] + up[1] * vh[1] + up[2] * vh[2]
  const upP0 = up.map((x, k) => x - du * vh[k])
  const upn = Math.hypot(upP0[0], upP0[1], upP0[2])
  const upP = upP0.map((x) => x / upn)
  const right = [vh[1] * upP[2] - vh[2] * upP[1], vh[2] * upP[0] - vh[0] * upP[2], vh[0] * upP[1] - vh[1] * upP[0]]
  const sig = Math.atan2(lb[0] * right[0] + lb[1] * right[1] + lb[2] * right[2], lb[0] * upP[0] + lb[1] * upP[1] + lb[2] * upP[2])
  const al = d[CH.alpha][i]
  const CL = d[CH.CN][i] * Math.cos(al) - d[CH.CA][i] * Math.sin(al)
  const gl = 3.986004418e14 / (rn * rn)
  const nL = (d[CH.qbar][i] * S * CL) / d[CH.mass][i] / (gl - (Vr * Vr) / rn)
  return {
    t: d[CH.t][i], phase: ph(i), h: d[CH.alt][i], gam: Math.asin(hdot / Vr) * DEG, hdot, V: d[CH.V][i], M: d[CH.mach][i], q: d[CH.qbar][i],
    g: d[CH.g][i], a: al * DEG, sig: sig * DEG, sigCmd: d[CH.bank][i] * DEG, steel: d[CH.steel][i], bond: d[CH.bondline][i], n2: d[CH.n2][i], nL, coilT: d[CH.coilT][i],
    dr: d[CH.predDR][i], cr: d[CH.predCR][i], coilI: d[CH.coilI][i],
  }
}
const rows: Row[] = []
for (let i = 0; i < n; i++) rows.push(rowAt(i))
mkdirSync(".tmp/agentE", { recursive: true })
writeFileSync(`.tmp/agentE/${label}-rows.csv`, ["t,phase,h_m,gam_deg,hdot,V,M,q_Pa,g,alpha_deg,sigFly_deg,sigCmd_deg,steel_K,bond_K,N2_kg,coilI_A,predDR_m,predCR_m"].concat(rows.map((r) => [r.t, r.phase, r.h, r.gam, r.hdot, r.V, r.M, r.q, r.g, r.a, r.sig, r.sigCmd, r.steel, r.bond, r.n2, r.coilI, r.dr, r.cr].join(","))).join("\n"))
const span = (a: number, b: number): Row[] => rows.filter((r) => r.t >= a && r.t <= b)

console.log(`\n=== 10 s series, loft − 60 s → flip (loft ${tLoft.toFixed(0)}, pitch-up ${tPitch.toFixed(0)}, high-α ${tHigh.toFixed(0)}, belly ${tBelly.toFixed(0)}, flip ${tFlip.toFixed(0)}) ===`)
console.log("    t phase         h km   γ°    ḣ m/s  V m/s    M  q kPa    g     α°  σfly°  σcmd°  steel  bond   N2kg   n_L  predDR/CR km")
let last = -1e9
for (const r of span(tLoft - 60, Number.isFinite(tFlip) ? tFlip : 1e9)) {
  if (r.t - last < 10) continue
  last = r.t
  console.log(
    `${r.t.toFixed(0).padStart(5)} ${r.phase.padEnd(11)} ${(r.h / 1e3).toFixed(1).padStart(6)} ${r.gam.toFixed(2).padStart(6)} ${r.hdot.toFixed(0).padStart(7)} ${r.V.toFixed(0).padStart(6)} ${r.M.toFixed(1).padStart(5)} ${(r.q / 1e3).toFixed(1).padStart(6)} ${r.g.toFixed(2).padStart(5)} ${r.a.toFixed(1).padStart(6)} ${r.sig.toFixed(0).padStart(6)} ${r.sigCmd.toFixed(0).padStart(6)} ${r.steel.toFixed(0).padStart(6)} ${r.bond.toFixed(0).padStart(5)} ${r.n2.toFixed(0).padStart(6)} ${r.nL.toFixed(2).padStart(5)} ${(r.dr / 1e3).toFixed(1).padStart(6)}/${(r.cr / 1e3).toFixed(1)}`,
  )
}

// ---------- loft and transition metrics ----------
const loft = span(tLoft, tPitch)
const pitch = span(tPitch, tHigh)
const high = span(tHigh, tBelly)
const max = (rs: Row[], k: keyof Row): number => rs.reduce((m, r) => Math.max(m, r[k] as number), -Infinity)
const min = (rs: Row[], k: keyof Row): number => rs.reduce((m, r) => Math.min(m, r[k] as number), Infinity)
const hLoftStart = loft[0]?.h ?? NaN
const hLoftMin = min(loft, "h")
const hLoftMax = max([...loft, ...pitch], "h")
const hHighMax = max(high, "h")
console.log("\n=== loft / transition ===")
console.log(`loft start h ${(hLoftStart / 1e3).toFixed(1)} km V ${loft[0]?.V.toFixed(0)} γ ${loft[0]?.gam.toFixed(2)}°, min h ${(hLoftMin / 1e3).toFixed(1)} km, max h (loft+pitch-up) ${(hLoftMax / 1e3).toFixed(1)} km, n_L mean ${(loft.reduce((s, r) => s + r.nL, 0) / Math.max(1, loft.length)).toFixed(2)}`)
console.log(`pitch-up at h ${(pitch[0]?.h / 1e3).toFixed(1)} km V ${pitch[0]?.V.toFixed(0)} q ${(pitch[0]?.q / 1e3).toFixed(1)} kPa; transition peak g ${max(pitch, "g").toFixed(2)}, peak q ${(max(pitch, "q") / 1e3).toFixed(1)} kPa`)

// ---------- phugoid metrics (high-α) ----------
// smooth altitude (±15 s box), find alternate extrema of h separated by > 40 s
const hs = high.map((r) => {
  const w = high.filter((x) => Math.abs(x.t - r.t) <= 15)
  return { t: r.t, h: w.reduce((s, x) => s + x.h, 0) / w.length, gam: r.gam }
})
interface Ext { t: number; h: number; kind: "max" | "min" }
const ext: Ext[] = []
for (let k = 1; k + 1 < hs.length; k++) {
  const a = hs[k - 1].h, b = hs[k].h, c = hs[k + 1].h
  const kind = b >= a && b > c ? "max" : b <= a && b < c ? "min" : null
  if (!kind) continue
  const prev = ext[ext.length - 1]
  if (prev && prev.kind === kind) { if ((kind === "max" && b > prev.h) || (kind === "min" && b < prev.h)) ext[ext.length - 1] = { t: hs[k].t, h: b, kind }; continue }
  if (prev && hs[k].t - prev.t < 40) continue
  ext.push({ t: hs[k].t, h: b, kind })
}
// peak-to-trough swings; amplitude = half swing; period = time between like extrema
const swings = ext.slice(1).map((e, k) => ({ t: e.t, A: Math.abs(e.h - ext[k].h) / 2 }))
const periods = ext.slice(2).map((e, k) => e.t - ext[k].t)
const decs = swings.slice(1).map((s, k) => Math.log(swings[k].A / s.A)) // per half period
const deltaFull = decs.length ? (2 * decs.reduce((s, x) => s + x, 0)) / decs.length : NaN
const zeta = deltaFull / Math.sqrt(4 * Math.PI * Math.PI + deltaFull * deltaFull)
const gamStd = Math.sqrt(high.reduce((s, r) => s + r.gam * r.gam, 0) / high.length - (high.reduce((s, r) => s + r.gam, 0) / high.length) ** 2)
console.log("\n=== high-α phugoid ===")
console.log("extrema:", ext.map((e) => `${e.kind}@${e.t.toFixed(0)} ${(e.h / 1e3).toFixed(1)}km`).join("  "))
console.log(`amplitudes (km): ${swings.map((s) => (s.A / 1e3).toFixed(2)).join(" ")}`)
console.log(`period (s): ${periods.map((p) => p.toFixed(0)).join(" ")}; mean ${(periods.reduce((s, x) => s + x, 0) / Math.max(1, periods.length)).toFixed(0)}`)
console.log(`log decrement per period δ ${deltaFull.toFixed(3)} → ζ ${zeta.toFixed(3)}; γ std ${gamStd.toFixed(2)}°, γ range ${min(high, "gam").toFixed(2)}…${max(high, "gam").toFixed(2)}°`)
console.log(`high-α: max h ${(hHighMax / 1e3).toFixed(1)} km, duration ${(tBelly - tHigh).toFixed(0)} s, peak g ${max(high, "g").toFixed(2)}, peak q ${(max(high, "q") / 1e3).toFixed(1)} kPa, steel max ${max(high, "steel").toFixed(0)} K`)

// ---------- per-phase peaks (all recorded samples) ----------
const phasePeaks: Record<string, { g: number; q: number; steel: number; bond: number; coilT: number; hMax: number }> = {}
for (const r of rows) {
  const p = (phasePeaks[r.phase] ??= { g: 0, q: 0, steel: 0, bond: 0, coilT: 0, hMax: 0 })
  p.g = Math.max(p.g, r.g); p.q = Math.max(p.q, r.q); p.steel = Math.max(p.steel, r.steel); p.bond = Math.max(p.bond, r.bond); p.coilT = Math.max(p.coilT, r.coilT); p.hMax = Math.max(p.hMax, r.h)
}
const preFlip = rows.filter((r) => r.t < (Number.isFinite(tFlip) ? tFlip : Infinity))
console.log("\n=== per-phase peaks ===")
for (const [k, p] of Object.entries(phasePeaks)) console.log(`${k.padEnd(12)} g ${p.g.toFixed(2).padStart(5)}  q ${(p.q / 1e3).toFixed(1).padStart(5)} kPa  steel ${p.steel.toFixed(0)} K  bond ${p.bond.toFixed(0)} K  coil ${p.coilT.toFixed(1)} K  h_max ${(p.hMax / 1e3).toFixed(1)} km`)
console.log(`before the flip: peak g ${max(preFlip, "g").toFixed(2)}, peak q ${(max(preFlip, "q") / 1e3).toFixed(1)} kPa, steel ${max(preFlip, "steel").toFixed(0)} K, bondline ${max(preFlip, "bond").toFixed(0)} K, coil ${max(preFlip, "coilT").toFixed(1)} K`)

// ---------- actuators per phase (T13/T14) ----------
console.log("\n=== actuators per phase (torque impulse ∫|M|dt in kN·m·s) ===")
console.log("phase          time s   N2 kg   MHD roll  cold-gas roll  roll MHD share   MHD p/y  cold-gas p/y  coil-on s")
for (const [k, a] of Object.entries(acc)) {
  const share = a.mhdRoll + a.rcsRoll > 0 ? (100 * a.mhdRoll) / (a.mhdRoll + a.rcsRoll) : NaN
  console.log(`${k.padEnd(13)} ${a.time.toFixed(0).padStart(7)} ${a.n2.toFixed(1).padStart(7)} ${(a.mhdRoll / 1e3).toFixed(0).padStart(10)} ${(a.rcsRoll / 1e3).toFixed(0).padStart(14)} ${share.toFixed(1).padStart(15)} % ${(a.mhdPY / 1e3).toFixed(0).padStart(9)} ${(a.rcsPY / 1e3).toFixed(0).padStart(13)} ${a.coilOn.toFixed(0).padStart(10)}`)
}
console.log(`coil current last above 50 A at t = ${coilLastOn.toFixed(1)} s (phase ${rows.find((r) => r.t >= coilLastOn)?.phase ?? "end"})`)

// ---------- delivery ----------
const flipEv = rec.events.find((e) => e.msg.startsWith("landing flip"))
const atFlip = rows.find((r) => r.t >= tFlip)
const n2Start = rows.find((r) => r.t >= tLoft)?.n2 ?? NaN
const n2Flip = atFlip?.n2 ?? final.cons.n2
const out = {
  label, outcome: final.outcome, reason: final.mem.reason, wall,
  phases: rec.meta.phaseStarts, summary: rec.meta.summary,
  loft: { hStart: hLoftStart, hMin: hLoftMin, hMax: hLoftMax, pitchH: pitch[0]?.h, pitchV: pitch[0]?.V, pitchQ: pitch[0]?.q, pitchG: max(pitch, "g"), pitchQmax: max(pitch, "q") },
  high: { hMax: hHighMax, ext, amplitudes: swings.map((s) => s.A), periods, delta: deltaFull, zeta, gamStd, gMax: max(high, "g"), qMax: max(high, "q"), steelMax: max(high, "steel"), duration: tBelly - tHigh },
  n2: { atLoft: n2Start, atFlip: n2Flip, used: n2Start - n2Flip, final: final.cons.n2 },
  actuators: acc, coilLastOn, phasePeaks, preFlip: { g: max(preFlip, "g"), q: max(preFlip, "q"), steel: max(preFlip, "steel"), bond: max(preFlip, "bond"), coilT: max(preFlip, "coilT") },
  flip: flipEv ? { t: flipEv.t, msg: flipEv.msg } : null,
  series: span(tLoft - 60, Number.isFinite(tFlip) ? tFlip : 1e9).filter((_, k) => k % 10 === 0),
  events: rec.events,
}
console.log(`\nflip: ${flipEv ? flipEv.msg : "none"}; N2 at loft ${n2Start.toFixed(0)} kg, at flip ${n2Flip.toFixed(0)} kg, final ${final.cons.n2.toFixed(0)} kg`)
mkdirSync(".tmp/agentE", { recursive: true })
writeFileSync(`.tmp/agentE/${label}.json`, JSON.stringify(out, null, 1))
console.log(`wrote .tmp/agentE/${label}.json`)
