/**
 * Windy mission check: the full mission with the physical default wind (scale 1 + turbulence) and
 * optional tuning overrides.   bun scripts/wind-check.ts ['{"aimHigh":1500}']
 */
import { FLIGHT_DESIGN } from "@ship/craft"
import { MISSION_TUNING_DEFAULT } from "@ship/gnc"
import { initState, makeWorld, step } from "@ship/sim"
const tun = process.argv[2] ? JSON.parse(process.argv[2]) : {}
const W0 = makeWorld({ ...FLIGHT_DESIGN, windScale: 1, turbulence: 1 })
const W = { ...W0, guide: { ...W0.guide, tuning: MISSION_TUNING_DEFAULT.with(tun) } }
let s = initState(W)
const t0 = performance.now()
while (!s.done) s = step(W, s).s
const ev = (re: RegExp): string => s.events.filter((e) => re.test(e.msg)).slice(-1)[0]?.msg.slice(0, 130) ?? "-"
console.log(`belly ${ev(/belly plan/).replace(/.*miss at flip completion/, "miss")} | ${ev(/flip complete/).replace(/^.*engine\(s\): /, "")} | ${ev(/catch pins|LOST/)} | ${s.outcome}; header ${((s.cons.loxHeader + s.cons.ch4Header) / 1e3).toFixed(2)} t; wall ${((performance.now() - t0) / 1e3).toFixed(0)} s`)
