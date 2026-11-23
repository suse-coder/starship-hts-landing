/**
 * Wall time per mission phase and the cost of the onboard prediction rollouts.
 *   bun scripts/phase-profile.ts ['{…tuning…}']
 */
import { FLIGHT_DESIGN } from "@ship/craft"
import { MISSION_TUNING_DEFAULT } from "@ship/gnc"
import { initState, makeWorld, step } from "@ship/sim"
const tun = process.argv[2] ? JSON.parse(process.argv[2]) : {}
const W0 = makeWorld({ ...FLIGHT_DESIGN, windScale: 0, turbulence: 0 })
const W = { ...W0, guide: { ...W0.guide, tuning: MISSION_TUNING_DEFAULT.with(tun) } }
let s = initState(W)
;(globalThis as { ROLLPROF?: Record<string, number> }).ROLLPROF = {}
const acc: Record<string, { wall: number; steps: number }> = {}
const T0 = performance.now(), C0 = process.cpuUsage()
while (!s.done) {
  const ph = s.mem.phase, a = performance.now()
  s = step(W, s).s
  const e = (acc[ph] ??= { wall: 0, steps: 0 }); e.wall += performance.now() - a; e.steps++
}
console.log(Object.entries(acc).map(([k, e]) => `${k} ${e.steps} steps ${(e.wall / 1e3).toFixed(2)} s`).join(" | "))
const st = (globalThis as { ROLLSTAT?: { n: number; ms: number; by: Record<string, number> } }).ROLLSTAT
console.log("rollout time by phase flown:", JSON.stringify(Object.fromEntries(Object.entries((globalThis as { ROLLPROF?: Record<string, number> }).ROLLPROF ?? {}).map(([k, v]) => [k, +(v / 1e3).toFixed(2)]))))
console.log(`cpu ${((process.cpuUsage(C0).user + process.cpuUsage(C0).system) / 1e6).toFixed(1)} s; total ${((performance.now() - T0) / 1e3).toFixed(1)} s, outcome ${s.outcome}; rollouts ${st?.n} in ${((st?.ms ?? 0) / 1e3).toFixed(1)} s by ${JSON.stringify(Object.fromEntries(Object.entries(st?.by ?? {}).map(([k, v]) => [k, +(v / 1e3).toFixed(1)])))}`)
