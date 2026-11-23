/**
 * Check the published recording against its own constraint channels: the flown pins inside the
 * recoverable band around the plan, the sink above the full-thrust stop limit, the hull inside the
 * tower corridor.   bun scripts/recording-check.ts
 */
import { readFileSync } from "node:fs"
import { CH, N_CH } from "@ship/sim"
const buf = readFileSync("packages/app/public/recording.bin")
const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength)
const headLen = new DataView(ab).getUint32(0, true)
const head = JSON.parse(new TextDecoder().decode(new Uint8Array(ab, 4, headLen)).replace(/\0+$/, ""))
const off = 4 + headLen
const n = head.n as number
const col = (k: keyof typeof CH): Float64Array => new Float64Array(ab.slice(off + 8 * CH[k] * n, off + 8 * (CH[k] + 1) * n))
const t = col("t"), on = col("planOn"), px = col("planX"), py = col("planY"), pz = col("planZ"), x = col("pinX"), y = col("pinY"), z = col("pinZ"), tube = col("tubeLat"), hard = col("tubeHard"), vzl = col("vzLimit"), vu = col("vu"), ph = col("phase")
console.log(`n ${n}, channels ${head.ch} (code ${N_CH}), outcome ${head.outcome ?? head.meta?.outcome}`)
let nOn = 0, worstBand = 0, inBand = 0, sinkViol = 0, hardViol = 0, hardN = 0
const armX = col("armX")
for (let i = 0; i < n; i++) {
  if (on[i] > 0.5) {
    nOn++
    const e = Math.hypot(x[i] - px[i], y[i] - py[i])
    if (tube[i] > 0) { if (e <= tube[i]) inBand++; worstBand = Math.max(worstBand, e / tube[i]) }
  }
  if (vzl[i] < 0 && vu[i] < vzl[i]) sinkViol++
  if (hard[i] > 0) { hardN++; if (Math.abs(x[i] - armX[i]) > hard[i] + 0.06) hardViol++ }
}
console.log(`plan samples ${nOn}; flown inside the recoverable band in ${inBand} of ${nOn} (worst error/band ${worstBand.toFixed(2)}); sink-limit violations ${sinkViol}; tower-corridor samples ${hardN}, violations ${hardViol}`)
const pick = (zz: number): number => { let k = 0; for (let i = 0; i < n; i++) if (ph[i] >= 9 && z[i] > zz) k = i; return k }
for (const h of [1000, 500, 200, 50, 10, 1]) { const i = pick(h); console.log(`  at ${h} m: t ${t[i].toFixed(1)} flown (${x[i].toFixed(2)}, ${y[i].toFixed(2)}) plan (${px[i].toFixed(2)}, ${py[i].toFixed(2)}, z ${pz[i].toFixed(1)}) band ±${tube[i].toFixed(2)} m, sink ${vu[i].toFixed(1)} limit ${vzl[i].toFixed(1)}, corridor ±${hard[i].toFixed(2)}`) }

// ---- pin azimuth: the line through the two catch pins must be square to the arms on the way in
{
  const roll = col("rollAng")
  let hi = 0, lo = 0, rails = NaN
  for (let i = 0; i < n; i++) {
    if (ph[i] < 9 || ph[i] > 10) continue
    const d = Math.abs(roll[i]) * 180 / Math.PI
    if (z[i] > 500) hi = Math.max(hi, d); else lo = Math.max(lo, d)
    if (z[i] > 0) rails = d
  }
  console.log(`pin-axis error on one engine: up to ${hi.toFixed(1)}° above 500 m, up to ${lo.toFixed(2)}° below it, ${rails.toFixed(2)}° when the pins reach the rails`)
}

// ---- after the catch: the ship must stay where it landed, on arms that do not move, engines off
const arms = col("arms"), thrust = col("thrust"), pc = col("pinCount"), pl = col("pinLoad"), mass = col("mass")
let iCatch = -1
for (let i = 0; i < n; i++) if (ph[i] === 11) { iCatch = i; break }
if (iCatch < 0) console.log("no catch in this recording")
else {
  let iOff = n - 1
  for (let i = iCatch; i < n; i++) if (thrust[i] < 1e3) { iOff = i; break }
  let slide = 0, sag = 0, armMove = 0, armOpen = 0
  for (let i = iCatch; i < n; i++) {
    slide = Math.max(slide, Math.hypot(x[i] - x[iCatch], y[i] - y[iCatch]))
    sag = Math.max(sag, -z[i])
    armMove = Math.max(armMove, Math.abs(armX[i] - armX[iCatch]))
    armOpen = Math.max(armOpen, arms[i])
  }
  const e = n - 1
  console.log(`after the catch (t ${t[iCatch].toFixed(1)} → ${t[e].toFixed(1)} s): sink at the catch ${(-vu[iCatch - 1]).toFixed(2)} m/s; thrust zero ${(t[iOff] - t[iCatch]).toFixed(2)} s after it; pins slid ${(slide * 100).toFixed(1)} cm, sag ${(sag * 100).toFixed(1)} cm on the rails; arm pair moved ${(armMove * 1000).toFixed(0)} mm, largest opening ${armOpen.toFixed(3)}; at the end ${pc[e]} pins carry ${(pl[e] / (mass[e] * 9.81) * 100).toFixed(0)} % of the weight`)
}
