import { makeVehicle } from "@ship/craft"
import { makeThermalGrid, surfaceHeating, N_RAYS } from "@ship/heat"
import { lookupStag, type ShockTable } from "@ship/gas"
import { SHOCK_TABLE as table } from "@ship/gas"
import { atmosphere } from "@ship/earth"
import { v3, normalize } from "@ship/math"
const v = makeVehicle(); const g = makeThermalGrid(v); const T = table as unknown as ShockTable
const SIG = 5.67e-8
for (const [hkm, U, aDeg] of [[66,6800,12],[62,6400,12],[60,5500,55],[58,5000,58],[55,4500,60],[52,3800,62],[48,3000,65],[44,2400,68],[40,1800,72],[34,1100,76]]) {
  const a = atmosphere(hkm*1000); const st = lookupStag(T, hkm*1000, U)
  const al = aDeg*Math.PI/180; const vhat = normalize(v3(Math.cos(al), 0, -Math.sin(al)))
  const h = surfaceHeating(g, { rhoInf: a.rho, pInf: a.p, U, vhat, stag: st, Rn: v.shape.p.noseRadius, mhdHeatFactor: 1, qInf: 0.5*a.rho*U*U })
  // windward ray is phi = 270° (−z) since vhat has −z component → cell normal (.., 0, -1)
  const ray = 6
  const out = g.xs.map((x, i) => { const id = i*N_RAYS+ray; const eps = g.cellPica[id] ? 0.9 : 0.85; const hw = 1000*600; const q = h.ch[id]*(h.hr[id]-hw); const Teq = Math.pow(Math.max(q,0)/(eps*SIG),0.25); return `${x.toFixed(1)}:${(q/1e4).toFixed(1)}W/cm²/${Teq.toFixed(0)}K` })
  console.log(`h=${hkm} U=${U} α=${aDeg} q=${(0.5*a.rho*U*U/1e3).toFixed(1)}kPa qstag=${(h.qStagCold/1e4).toFixed(1)}W/cm² turb=${h.turbulentFrac.toFixed(2)}\n   windward: ${out.join(" ")}`)
}
