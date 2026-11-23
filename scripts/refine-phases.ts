/**
 * Phase-aware refinement on the isolated rig: how much an error at a phase boundary costs the
 * catch (the sensitivity the precision budget is built on), and the capture set of the landing.
 *
 *   bun scripts/refine-phases.ts            report plan + landing capture set
 *   bun scripts/refine-phases.ts --measure  measure the landing sensitivity (m per m/s)
 */
import { v3 } from "@ship/math"
import { flyRig, allocateBudget, captureRadius, PHASE_PRECISION } from "@ship/verify"

const flag = (k: string): boolean => process.argv.includes(`--${k}`)
const land = (off: number, vh: number): { caught: boolean; error: number; fuel: number } => {
  const f = flyRig({ pin: v3(off, 0, 360), vel: v3(vh, 0, -50), engines: [0], throttle: 0.4 }, undefined, 120)
  const last = f.samples[f.samples.length - 1]
  return { caught: f.outcome === "caught", error: Math.hypot(last.pin.x, last.pin.y), fuel: 19600 - last.header }
}

// the allocation the compute budget buys (planner seconds, not simulation seconds)
const alloc = allocateBudget(8)
console.log("precision budget (8 s of planner time, allocated from the phase sensitivities):")
for (const a of alloc) console.log(`  ${a.phase.padEnd(10)} share ${(a.share * 100).toFixed(0).padStart(3)} %  evals ${String(a.evals).padStart(4)}  ≈ ${a.cost.toFixed(1)} s`)

console.log("\nlanding capture set (1 engine, from 360 m at −50 m/s — where the flown descent starts — isolated rig):")
console.log("  offset[m]  v_lat[m/s]  outcome  off at the pins[m]  header used[t]")
for (const off of [0, 3, 10, 20, 35]) for (const vh of [0, 3, 10]) {
  const r = land(off, vh)
  console.log(`  ${String(off).padStart(8)}  ${String(vh).padStart(10)}  ${r.caught ? "CAUGHT  " : "lost    "}  ${r.error.toFixed(2).padStart(17)}  ${(r.fuel / 1e3).toFixed(2).padStart(13)}`)
}
const radius = captureRadius((m) => land(m, 0).caught, 0, 60, 5)
console.log(`  capture radius at zero lateral speed: ${radius.toFixed(0)} m`)

if (flag("measure")) {
  // sensitivity: the change of the *final* pin error for 1 m/s of lateral velocity error at the
  // phase start, sampled from the offset the terminal cannot recover (the error survives to the pins)
  const a0 = land(0, 0), a1 = land(0, 2), a2 = land(0, 5)
  const s1 = (a1.error - a0.error) / 2, s2 = (a2.error - a0.error) / 5
  console.log(`\nmeasured landing sensitivity: ${s1.toFixed(2)} m per m/s at 2 m/s, ${s2.toFixed(2)} at 5 m/s (estimate in precision.ts: ${PHASE_PRECISION.find((p) => p.phase === "landing")?.sensitivity})`)
}
