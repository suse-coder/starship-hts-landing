/** 3-DOF predictor from a recorded state: vertical-only vs signed bank, with/without a planned reversal. */
import { runMission, CH, predContext } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { predict } from "@ship/gnc"
import { v3 } from "@ship/math"
const tq = Number(process.argv[2] ?? 2560)
const { recording: rec, world: W } = runMission(DEFAULT_DESIGN)
const d = rec.data
let i = 0
while (d[CH.t][i] < tq) i++
const r = v3(d[CH.rx][i], d[CH.ry][i], d[CH.rz][i]), v = v3(d[CH.vx][i], d[CH.vy][i], d[CH.vz][i])
const ctx = predContext(W.aeroTab, W.mhdDrag, W.aero.sref, W.plan.theta0, W.padEcef)
const t = d[CH.t][i], base = { t, r, v, mass: d[CH.mass][i], phase: "highAlpha" as const, bankEntry: 0 }
console.log("t", t.toFixed(1), "pad", (d[CH.pe][i] / 1e3).toFixed(1), (d[CH.pn][i] / 1e3).toFixed(1), "km  V", d[CH.V][i].toFixed(0))
for (const deg of [0, 30, 50, 70, 85]) {
  const b = deg * Math.PI / 180
  const o = predict(ctx, { ...base, bankHigh: b })
  const sR = predict(ctx, { ...base, bankHigh: b, bankSign: 1, sigma0: b, sigmaRate: 0.026 })
  const sRev = predict(ctx, { ...base, bankHigh: b, bankSign: 1, sigma0: b, sigmaRate: 0.026, tRev: t + 60 })
  const f = (x: { downrangeErr: number; crossrangeErr: number }) => `${(x.downrangeErr / 1e3).toFixed(1)}/${(x.crossrangeErr / 1e3).toFixed(1)}`
  console.log(`bank ${deg}: vertical-only ${f(o)} | signed hold ${f(sR)} | reversal at +60 s ${f(sRev)}  (down/cross km)`)
}
