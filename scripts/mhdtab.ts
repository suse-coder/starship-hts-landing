import { makeWorld } from "@ship/sim"
const W = makeWorld()
const t = W.mhdDrag
for (let i = 0; i < t.hs.length; i += 2) console.log(`h=${t.hs[i]/1000}km`, t.us.map((u, j) => j % 2 ? "" : `${(u/1000).toFixed(1)}:${t.dCD[i*t.us.length+j].toFixed(3)}/${t.dCL[i*t.us.length+j].toFixed(3)}`).filter(Boolean).join(" "))
console.log("aero CL/CD at 8°/14° M20:", W.aeroTab.CL[3*14+12].toFixed(3), W.aeroTab.CD[3*14+12].toFixed(3), W.aeroTab.CL[6*14+12].toFixed(3), W.aeroTab.CD[6*14+12].toFixed(3))
