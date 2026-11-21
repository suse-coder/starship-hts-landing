/** Final-approach trace: belly flop → catch, pad frame. */
import { runMission, CH } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { PHASES } from "@ship/gnc"
const t0 = Number(process.argv[2] ?? 0), dt = Number(process.argv[3] ?? 2)
const { recording: rec } = runMission(DEFAULT_DESIGN)
const d = rec.data, D = 180 / Math.PI
const ph0 = PHASES.findIndex((p) => p.id === "bellyFlop")
let last = -1e9
for (let i = 0; i < rec.n; i++) {
  const t = d[CH.t][i]
  if ((process.argv[4] !== "all" && d[CH.phase][i] < ph0) || t < t0 || t - last < dt) continue
  last = t
  console.log(`${t.toFixed(1)} ${PHASES[d[CH.phase][i]].id.padEnd(9)} hB=${d[CH.hBase][i].toFixed(0).padStart(5)} pad=(${d[CH.pe][i].toFixed(0)},${d[CH.pn][i].toFixed(0)}) v=(${d[CH.ve][i].toFixed(1)},${d[CH.vn][i].toFixed(1)},${d[CH.vu][i].toFixed(1)}) pin=(${d[CH.pinX][i].toFixed(1)},${d[CH.pinY][i].toFixed(1)},${d[CH.pinZ][i].toFixed(1)}) α=${(d[CH.alpha][i]*D).toFixed(0)} err=${(d[CH.attErr][i]*D).toFixed(1)} T=${(d[CH.T1][i]/1e6).toFixed(2)},${(d[CH.T2][i]/1e6).toFixed(2)},${(d[CH.T3][i]/1e6).toFixed(2)}MN gp=${(d[CH.gp][i]*D).toFixed(1)} hdg=${(d[CH.bellyHdg][i]*D).toFixed(0)} bank=${(d[CH.bank][i]*D).toFixed(0)} miss=${d[CH.bellyMiss][i].toFixed(0)} hdr=${d[CH.header][i].toFixed(0)} n2=${d[CH.n2][i].toFixed(0)} arms=${d[CH.arms][i].toFixed(2)} load=${(d[CH.pinLoad][i]/1e3).toFixed(0)}kN g=${d[CH.g][i].toFixed(2)}`)
}
