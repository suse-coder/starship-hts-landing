/** Landing trace: thrust-axis tilt and azimuth vs. horizontal velocity, pad frame. */
import { runMission, CH } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { v3, normalize, cross, dot } from "@ship/math"
const t0 = Number(process.argv[2] ?? 3008), dtp = Number(process.argv[3] ?? 1)
const { recording: rec } = runMission(DEFAULT_DESIGN)
const d = rec.data, D = 180 / Math.PI
let last = -1e9
for (let i = 0; i < rec.n; i++) {
  const t = d[CH.t][i]
  if (t < t0 || t - last < dtp) continue
  last = t
  const q = { w: d[CH.qw][i], x: d[CH.qx][i], y: d[CH.qy][i], z: d[CH.qz][i] }
  const xb = v3(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y))
  const r = v3(d[CH.rx][i], d[CH.ry][i], d[CH.rz][i])
  const up = normalize(r), east = normalize(cross(v3(0, 0, 1), up)), north = cross(up, east)
  const tilt = Math.acos(Math.min(1, dot(xb, up))) * D, az = Math.atan2(dot(xb, east), dot(xb, north)) * D
  const zb = v3(2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y))
  const zAz = Math.atan2(dot(zb, east), dot(zb, north)) * D
  console.log(`${t.toFixed(1)} ${String(rec.meta.phaseStarts.filter((p) => p.t <= t).at(-1)?.phase).padEnd(8)} tilt ${tilt.toFixed(1).padStart(5)}° az ${az.toFixed(0).padStart(4)}° zAz ${zAz.toFixed(0).padStart(4)}° v=(${d[CH.ve][i].toFixed(1)},${d[CH.vn][i].toFixed(1)},${d[CH.vu][i].toFixed(1)}) pin=(${d[CH.pinX][i].toFixed(1)},${d[CH.pinY][i].toFixed(1)},${d[CH.pinZ][i].toFixed(1)}) w=(${(d[CH.wx][i] * D).toFixed(1)},${(d[CH.wy][i] * D).toFixed(1)},${(d[CH.wz][i] * D).toFixed(1)})°/s gp=${(d[CH.gp][i] * D).toFixed(1)} gy=${(d[CH.gy] ? d[CH.gy][i] * D : 0).toFixed(1)} T=${((d[CH.T1][i] + d[CH.T2][i] + d[CH.T3][i]) / 1e6).toFixed(2)}`)
}
