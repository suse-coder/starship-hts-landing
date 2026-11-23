/**
 * Model verification report (isolated rig, no wind): bun scripts/verify-model.ts [--plant] [--steps] [--slam]
 */
import { v3 } from "@ship/math"
import { flyRig, identifyPlant, identifyThrottle, stepMetrics } from "@ship/verify"

const flag = (k: string): boolean => process.argv.includes(`--${k}`)
const all = !flag("plant") && !flag("steps") && !flag("slam")
if (all || flag("plant")) {
  console.log("PLANT IDENTIFICATION (open-loop gimbal step on the 6-DOF plant, engines at trim)")
  console.log("eng thr  mass[t]  T/m meas/model   pitch acc meas/model [rad/s²]  b=TL/I  a_cg   a_pin  pin/percussion lever  roll acc  α_max  snap_max")
  for (const [e, th] of [[[0], 0.4], [[0], 0.7], [[0], 1.0], [[1, 2], 0.4], [[1, 2], 0.7]] as const) {
    const p = identifyPlant(e, th)
    console.log(`${p.engines}   ${th.toFixed(1)}  ${(p.mass / 1e3).toFixed(1)}   ${p.accel.toFixed(2)} / ${p.accelModel.toFixed(2)}     ${p.alpha.toFixed(4)} / ${p.alphaModel.toFixed(4)}            ${p.b.toFixed(3)}  ${p.aCg.toFixed(3)}  ${p.aPin.toFixed(3)}  ${p.pinLever.toFixed(2)} / ${p.percussion.toFixed(2)} m        ${p.alphaRoll.toFixed(4)}   ${p.alphaMax.toFixed(3)}  ${p.snapMax.toFixed(2)}`)
  }
  const th = identifyThrottle()
  console.log(`throttle step 0.4 → 0.9: slew ${th.slew.toFixed(2)} /s, 90 % after ${th.t90.toFixed(2)} s`)
}
if (all || flag("steps")) {
  console.log("\nLATERAL STEP RESPONSES (flight software, 1 engine, vertical descent from 1500 m at −40 m/s: floor thrust, then the brake)")
  console.log("axis  d0[m]  t10%[s]  settle<0.2m[s]  overshoot[m]  crossings  tilt max[°]  roll rate max[°/s]  cross-axis[m]  outcome / offset at the end")
  for (const axis of ["x", "y"] as const) for (const d of [1, 3, 10, 30]) {
    const f = flyRig({ pin: axis === "x" ? v3(d, 0, 1500) : v3(0, d, 1500), vel: v3(0, 0, -40), engines: [0], throttle: 0.4 }, undefined, 90)
    const m = stepMetrics(f.samples, axis)
    const last = f.samples[f.samples.length - 1]
    console.log(`${axis}     ${d.toString().padStart(3)}    ${m.t10.toFixed(1).padStart(5)}    ${m.settle.toFixed(1).padStart(5)}           ${m.overshoot.toFixed(2).padStart(5)}         ${m.crossings}          ${m.tiltMax.toFixed(1).padStart(4)}         ${m.rollRateMax.toFixed(1).padStart(4)}                ${m.cross.toFixed(2)}        ${f.outcome} ${Math.hypot(last.pin.x, last.pin.y).toFixed(2)} m after ${last.t.toFixed(1)} s`)
  }
}
if (all || flag("slam")) {
  console.log("\nVERTICAL DESCENT FROM 1 km (flight software, 1 engine, on the axis)")
  for (const [dx, vz] of [[0, -50], [2, -50], [0, -70]] as const) {
    const f = flyRig({ pin: v3(dx, 0, 1000), vel: v3(0, 0, vz), engines: [0], throttle: 0.4 }, undefined, 120)
    const last = f.samples[f.samples.length - 1]
    const hov = f.samples.filter((p) => p.pin.z < 60 && Math.abs(p.vel.z) < 2.0 && p.pin.z > 0.3).length * 0.05
    const ev = f.events.filter((e) => /CAUGHT|LOST|closing/.test(e.msg)).map((e) => `${(e.t - 3000).toFixed(1)} ${e.msg.slice(0, 110)}`)
    console.log(`start ${dx} m off, ${vz} m/s → ${f.outcome}${f.reason ? ` (${f.reason})` : ""} after ${last.t.toFixed(1)} s, header used ${((19600 - last.header) / 1e3).toFixed(2)} t, time below 2 m/s above the rails ${hov.toFixed(1)} s`)
    for (const e of ev) console.log(`    ${e}`)
  }
}
