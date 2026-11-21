/** Predictor accuracy: propagate from a recorded state for N s with the flown bank held, compare with the 6-DOF. */
import { runMission, CH, predContext } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { predict } from "@ship/gnc"
import { v3, normalize, cross, dot, sub, scale, norm } from "@ship/math"
import { corotation } from "@ship/earth"
const { recording: rec, world: W } = runMission(DEFAULT_DESIGN)
const d = rec.data, D = 180 / Math.PI
const ctx = predContext(W.aeroTab, W.mhdDrag, W.aero.sref, W.plan.theta0, W.padEcef)
const at = (tq: number): number => { let i = 0; while (d[CH.t][i] < tq) i++; return i }
const st = (i: number) => ({ r: v3(d[CH.rx][i], d[CH.ry][i], d[CH.rz][i]), v: v3(d[CH.vx][i], d[CH.vy][i], d[CH.vz][i]) })
const sigmaAt = (i: number): number => {
  const q = { w: d[CH.qw][i], x: d[CH.qx][i], y: d[CH.qy][i], z: d[CH.qz][i] }
  const xb = v3(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y))
  const { r, v } = st(i)
  const vh = normalize(sub(v, corotation(r))), up = normalize(r)
  const upP = normalize(sub(up, scale(vh, dot(up, vh)))), lb = sub(xb, scale(vh, dot(xb, vh)))
  return Math.atan2(dot(lb, cross(vh, upP)), dot(lb, upP))
}
const N = Number(process.argv[3] ?? 60)
for (const t0 of (process.argv[2] ?? "2300,2380,2500,2560,2620").split(",").map(Number)) {
  const i = at(t0), j = at(t0 + N)
  // mean flown bank over the window
  let sm = 0, k = 0
  for (let m = i; m <= j; m += 10) { sm += sigmaAt(m); k++ }
  const sig = sm / k
  const { r, v } = st(i)
  const o = predict(ctx, { t: d[CH.t][i], r, v, mass: d[CH.mass][i], phase: "highAlpha", bankEntry: 0, bankHigh: Math.abs(sig), bankSign: Math.sign(sig) || 1, sigma0: sig, tMax: d[CH.t][j] })
  const a = st(j)
  const dV = norm(sub(o.vRel, sub(a.v, corotation(a.r)))) , Vp = norm(o.vRel), Va = norm(sub(a.v, corotation(a.r)))
  const dr = sub(o.r, a.r)
  const upA = normalize(a.r)
  console.log(`t ${t0}→${t0 + N}: flown σ̄ ${(sig * D).toFixed(0)}°  |Δr| ${(norm(dr) / 1e3).toFixed(2)} km (Δh ${(dot(dr, upA) / 1e3).toFixed(2)} km)  V pred ${Vp.toFixed(0)} actual ${Va.toFixed(0)} (Δ ${(Vp - Va).toFixed(1)} m/s)  |Δv| ${dV.toFixed(1)}`)
}
