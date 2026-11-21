/** Compare the 3-DOF predictor with the 6-DOF flight from a given recorded time. */
import { runMission, CH, predContext } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { predict } from "@ship/gnc"
import { v3 } from "@ship/math"
const tq = Number(process.argv[2] ?? 2057)
const { recording: rec, world: W } = runMission(DEFAULT_DESIGN)
const d = rec.data
let i = 0
while (d[CH.t][i] < tq) i++
const r = v3(d[CH.rx][i], d[CH.ry][i], d[CH.rz][i]), v = v3(d[CH.vx][i], d[CH.vy][i], d[CH.vz][i])
const ctx = predContext(W.aeroTab, W.mhdDrag, W.aero.sref, W.plan.theta0, W.padEcef)
const ph = d[CH.phase][i]
console.log("t", d[CH.t][i], "phase idx", ph, "alt", d[CH.alt][i], "V", d[CH.V][i], "mass", d[CH.mass][i])
for (const bh of [0, 0.5]) {
  const out = predict(ctx, { t: d[CH.t][i], r, v, mass: d[CH.mass][i], phase: ph === 4 ? "loft" : ph >= 6 ? "highAlpha" : "entry", bankEntry: 0, bankHigh: bh, mhdDragScale: d[CH.npcMhd][i], record: true })
  console.log(`pred pitch=${JSON.stringify(out.pitch)} bankHigh=${bh}: tEnd=${out.t.toFixed(0)} DR=${(out.downrangeErr/1000).toFixed(1)}km CR=${(out.crossrangeErr/1000).toFixed(1)}km qmax=${(out.qMax/1000).toFixed(1)} gmax=${out.gMax.toFixed(2)}`)
}
// actual: where did we end (bellyFlop start) relative to pad
const be = rec.meta.phaseStarts.find(p => p.phase === "bellyFlop")
console.log("actual phases", rec.meta.phaseStarts.map(p=>`${p.phase}@${p.t.toFixed(0)}`).join(" "))
let j = 0; while (be && d[CH.t][j] < be.t) j++
console.log("actual at belly: pad E/N km", (d[CH.pe][j]/1000).toFixed(1), (d[CH.pn][j]/1000).toFixed(1), "alt", d[CH.alt][j].toFixed(0))
// rcs duty trace in the entry window
for (let k = 0; k < rec.n; k++) { const t = d[CH.t][k]; if (t > 1890 && t < 2000 && Math.round(t*4) % 40 === 0) console.log(t.toFixed(1), "n2", d[CH.n2][k].toFixed(0), "duty", Array.from({length:12},(_,m)=>d[CH.r0+m][k].toFixed(2)).join(" "), "wx", (d[CH.wx][k]*57.3).toFixed(2), "mhdMy", (d[CH.mhdMy][k]/1e3).toFixed(0)) }
