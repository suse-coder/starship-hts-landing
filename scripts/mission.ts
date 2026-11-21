/** Headless mission run: prints the event log, phase timeline and key peaks. `bun run mission [--json]` */
import { runMission, CH, PHASES_OF } from "./missionHelpers"
import { DEFAULT_DESIGN } from "@ship/craft"

const t0 = performance.now()
const { recording: rec, final } = runMission(DEFAULT_DESIGN, (p) => {
  if (p.stage !== "simulating") console.log(`[plan] ${p.stage}`)
})
const ms = performance.now() - t0
for (const e of rec.events) console.log(`${e.t.toFixed(1).padStart(8)} s  ${e.msg}`)
console.log("\nphases:", rec.meta.phaseStarts.map((p) => `${p.phase}@${p.t.toFixed(0)}`).join("  "))
console.log("summary:", rec.meta.summary)
console.log(`samples ${rec.n}, steps ${rec.meta.steps}, wall ${(ms / 1000).toFixed(1)} s, outcome ${final.outcome}`)
// condensed trace every ~30 s of mission time in the atmosphere
const d = rec.data
let last = -1e9
for (let i = 0; i < rec.n; i++) {
  const t = d[CH.t][i]
  const alt = d[CH.alt][i]
  if (alt < 130e3 && t - last >= (alt > 30e3 ? 30 : 5)) {
    last = t
    console.log(
      `${t.toFixed(0).padStart(5)} ${PHASES_OF(d[CH.phase][i]).padEnd(12)} h=${(alt / 1000).toFixed(1).padStart(6)}km V=${d[CH.V][i].toFixed(0).padStart(5)} M=${d[CH.mach][i].toFixed(1).padStart(5)} q=${(d[CH.qbar][i] / 1000).toFixed(1).padStart(5)}kPa α=${((d[CH.alpha][i] * 180) / Math.PI).toFixed(1).padStart(5)}/${((d[CH.alphaCmd][i] * 180) / Math.PI).toFixed(1)} g=${d[CH.g][i].toFixed(2)} qs=${(d[CH.qStag][i] / 1e4).toFixed(0)}W/cm² pica=${d[CH.picaSurf][i].toFixed(0)} bond=${d[CH.bondline][i].toFixed(0)} steel=${d[CH.steel][i].toFixed(0)} mhdF=${(d[CH.mhdF][i] / 1e3).toFixed(0)}kN auth=${d[CH.mhdAuth][i].toFixed(2)} σeff=${d[CH.sigmaEff][i].toFixed(0)} β=${d[CH.hall][i].toFixed(1)} err=${((d[CH.attErr][i] * 180) / Math.PI).toFixed(1)}° n2=${d[CH.n2][i].toFixed(0)} DR=${(d[CH.predDR][i] / 1000).toFixed(1)}km ms=${d[CH.npcMhd][i].toFixed(2)} bank=${((d[CH.bank][i]*180)/Math.PI).toFixed(0)} pad=(${(d[CH.pe][i] / 1000).toFixed(1)},${(d[CH.pn][i] / 1000).toFixed(1)})km`,
    )
  }
}
