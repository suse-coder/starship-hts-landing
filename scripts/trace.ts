/** usage: bun scripts/trace.ts t0 t1 dt  — prints a compact state trace between t0 and t1 */
import { runMission, CH } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { PHASES } from "@ship/gnc"
const [t0, t1, dt] = process.argv.slice(2).map(Number)
const { recording: rec } = runMission(DEFAULT_DESIGN)
const d = rec.data, D = 180 / Math.PI
let last = -1e9
for (let i = 0; i < rec.n; i++) {
  const t = d[CH.t][i]
  if (t < t0 || t > t1 || t - last < dt) continue
  last = t
  const duties = Array.from({ length: 12 }, (_, k) => d[CH.r0 + k][i].toFixed(2)).join(" ")
  console.log(`${t.toFixed(1)} ${PHASES[d[CH.phase][i]].id} n2=${d[CH.n2][i].toFixed(1)} err=${(d[CH.attErr][i] * D).toFixed(2)} w=(${(d[CH.wx][i] * D).toFixed(2)},${(d[CH.wy][i] * D).toFixed(2)},${(d[CH.wz][i] * D).toFixed(2)})°/s coil=${d[CH.coilI][i].toFixed(0)}A rcsTau=${d[CH.rcsTau][i].toFixed(0)} α=${(d[CH.alpha][i]*D).toFixed(1)} duty[${duties}]`)
}
