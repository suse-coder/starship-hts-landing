/**
 * Model verification report (isolated rig, no wind): bun scripts/verify-model.ts [--plant] [--aero] [--capture]
 */
import { captureCase, checkTailFirstAero, identifyPlant, identifyThrottle } from "@ship/verify"

const flag = (k: string): boolean => process.argv.includes(`--${k}`)
const all = !flag("plant") && !flag("aero") && !flag("capture")
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
if (all || flag("aero")) {
  console.log("\nTAIL-FIRST AERODYNAMICS: guidance database vs plant (hull tilted from a vertical fall)")
  console.log("V[m/s] tilt   F_lat guide/plant [kN]   M guide/plant [MN·m]   share of the gimbal's holding torque at the floor")
  for (const V of [50, 70, 90]) for (const tilt of [3, 6, 10, 15]) {
    const a = checkTailFirstAero(V, tilt)
    console.log(`${String(V).padStart(4)}  ${String(tilt).padStart(3)}°   ${(a.fGuide / 1e3).toFixed(1).padStart(7)} / ${(a.fPlant / 1e3).toFixed(1).padStart(7)}       ${(a.mGuide / 1e6).toFixed(2).padStart(6)} / ${(a.mPlant / 1e6).toFixed(2).padStart(6)}          ${(Math.abs(a.mPlant) / a.gimbalTorque * 100).toFixed(0).padStart(3)} %`)
  }
}
if (all || flag("capture")) {
  console.log("\nDESCENT CAPTURE SET (flight software, 1 engine, upright from 1300 m at −30 m/s — the state the flip hands over)")
  console.log("off E,N [m]  v E,N [m/s]  outcome  off@1km  off@rails  overshoot  tilt max  pin azimuth max / at rails  fuel     hover   contact")
  for (const [x, y, vx, vy] of [[0, 0, 0, 0], [3, 0, 0, 0], [0, 3, 0, 0], [10, 0, 0, 0], [0, 10, 0, 0], [30, 0, 0, 0], [0, 30, 0, 0], [20, 20, 0, 0], [60, 0, 0, 0], [0, 60, 0, 0], [0, 0, 5, 0], [0, 0, 0, 5]] as const) {
    const c = captureCase(x, y, vx, vy)
    console.log(`${String(x).padStart(4)},${String(y).padStart(3)}     ${String(vx).padStart(3)},${String(vy).padStart(3)}     ${c.outcome.padEnd(7)}  ${c.at1km.toFixed(2).padStart(6)}   ${c.atRails.toFixed(2).padStart(6)}     ${c.overshoot.toFixed(2).padStart(6)}    ${c.tiltMax.toFixed(1).padStart(5)}°      ${c.rollAngMax.toFixed(1).padStart(5)}° / ${c.rollAngRails.toFixed(2).padStart(5)}°      ${(c.fuel / 1e3).toFixed(2)} t  ${c.hover.toFixed(1)} s   ${c.contactSpeed.toFixed(2)} m/s`)
  }
}
