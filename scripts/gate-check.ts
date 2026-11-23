/**
 * Full no-wind mission, key events, the state at the 1 km gate and the budgets.
 *   bun scripts/gate-check.ts ['{"aimHigh":1500}' [event-regex]]   (MissionTuning overrides as JSON)
 */
import { FLIGHT_DESIGN } from "@ship/craft"
import { MISSION_TUNING_DEFAULT } from "@ship/gnc"
import { CH, initState, makeWorld, step } from "@ship/sim"
const tun = process.argv[2] ? JSON.parse(process.argv[2]) : {}
const re = new RegExp(process.argv[3] ?? "belly plan|landing flip|flip complete|descent plan|staging|closing|catch pins|LOST|bearing")
const W0 = makeWorld({ ...FLIGHT_DESIGN, windScale: 0, turbulence: 0 })
const W = { ...W0, guide: { ...W0.guide, tuning: MISSION_TUNING_DEFAULT.with(tun) } }
let s = initState(W)
let prev: Float64Array | null = null
let at1k = "never crossed 1 km in flip/landing"
const t0 = performance.now()
let hov = 0, up = 0
while (!s.done) {
  const out = step(W, s), r = out.row
  if (prev && prev[CH.pinZ] > 1000 && r[CH.pinZ] <= 1000) at1k = `AT 1 km: phase ${out.s.mem.phase}, off E ${r[CH.pinX].toFixed(2)} N ${r[CH.pinY].toFixed(2)} (${Math.hypot(r[CH.pinX], r[CH.pinY]).toFixed(2)} m), v_h ${Math.hypot(r[CH.ve], r[CH.vn]).toFixed(2)} m/s, v_z ${r[CH.vu].toFixed(1)}`
  if ((out.s.mem.phase === "landing" || out.s.mem.phase === "terminal") && r[CH.pinZ] > 0.5) { if (Math.abs(r[CH.vu]) < 2) hov += r[CH.t] - (prev ? prev[CH.t] : r[CH.t]); if (r[CH.vu] > 0.05) up += r[CH.t] - (prev ? prev[CH.t] : r[CH.t]) }
  prev = r; s = out.s
}
for (const e of s.events.filter((e) => re.test(e.msg)).slice(-14)) console.log(`${e.t.toFixed(1)} ${e.msg.slice(0, 200)}`)
console.log(at1k)
console.log(`outcome ${s.outcome}${s.mem.reason ? " (" + s.mem.reason + ")" : ""}; header left ${((s.cons.loxHeader + s.cons.ch4Header) / 1e3).toFixed(2)} t; N2 ${s.cons.n2.toFixed(0)} kg; time <2 m/s above rails ${hov.toFixed(1)} s, climbing ${up.toFixed(1)} s; wall ${((performance.now() - t0) / 1e3).toFixed(1)} s`)
