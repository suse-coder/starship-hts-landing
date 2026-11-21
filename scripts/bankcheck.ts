/** Flown bank vs commanded bank (high-α), from the recorded attitude. */
import { runMission, CH } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { v3, normalize, cross, dot, sub, scale } from "@ship/math"
import { corotation } from "@ship/earth"
const [t0, t1] = [Number(process.argv[2] ?? 2560), Number(process.argv[3] ?? 2680)]
const { recording: rec } = runMission(DEFAULT_DESIGN)
const d = rec.data, D = 180 / Math.PI
let last = -1e9
for (let i = 0; i < rec.n; i++) {
  const t = d[CH.t][i]
  if (t < t0 || t > t1 || t - last < 5) continue
  last = t
  const q = { w: d[CH.qw][i], x: d[CH.qx][i], y: d[CH.qy][i], z: d[CH.qz][i] }
  const xb = v3(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y))
  const r = v3(d[CH.rx][i], d[CH.ry][i], d[CH.rz][i]), v = v3(d[CH.vx][i], d[CH.vy][i], d[CH.vz][i])
  const vh = normalize(sub(v, corotation(r))), up = normalize(r)
  const upP = normalize(sub(up, scale(vh, dot(up, vh)))), right = cross(vh, upP)
  const lb = sub(xb, scale(vh, dot(xb, vh)))
  const sig = Math.atan2(dot(lb, right), dot(lb, upP))
  console.log(`${t.toFixed(1)} cmd bank ${(d[CH.bank][i] * D).toFixed(0)}  flown σ ${(sig * D).toFixed(0)}  vN ${d[CH.vn][i].toFixed(0)}  vE ${d[CH.ve][i].toFixed(0)}`)
}
