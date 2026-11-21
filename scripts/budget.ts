/** Consumable budget per phase (N2, Cs, header propellant) */
import { runMission, CH } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
const { recording: rec } = runMission(DEFAULT_DESIGN)
const d = rec.data
const at = (t: number) => { let i = 0; while (i < rec.n - 1 && d[CH.t][i] < t) i++; return i }
const ps = rec.meta.phaseStarts
ps.forEach((p, k) => {
  const i0 = at(p.t), i1 = at(k + 1 < ps.length ? ps[k + 1].t : 1e9)
  console.log(`${p.phase.padEnd(13)} ${p.t.toFixed(0).padStart(5)}–${d[CH.t][i1].toFixed(0).padStart(5)} s  N2 ${(d[CH.n2][i0] - d[CH.n2][i1]).toFixed(0).padStart(5)} kg  Cs ${(d[CH.seedLeft][i0] - d[CH.seedLeft][i1]).toFixed(0).padStart(4)} kg  prop ${(d[CH.header][i0] - d[CH.header][i1]).toFixed(0).padStart(6)} kg`)
})
