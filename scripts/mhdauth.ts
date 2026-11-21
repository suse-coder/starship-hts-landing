import { makeVehicle } from "@ship/craft"
import { makeMhdModel, mhdBasis } from "@ship/mhd"
import { allocateMhd } from "@ship/gnc"
import { makeAeroModel, staticCoeffs } from "@ship/aero"
import { lookupStag, type ShockTable } from "@ship/gas"
import { SHOCK_TABLE as table } from "@ship/gas"
import { atmosphere } from "@ship/earth"
import { v3, normalize } from "@ship/math"
const v = makeVehicle(); const m = makeMhdModel(v); const T = table as unknown as ShockTable; const am = makeAeroModel(v)
for (const [hkm, U, aDeg] of [[85,7500,8],[80,7450,8],[75,7400,10],[70,7300,10],[65,7000,10],[60,6600,12],[55,6000,15],[50,4500,20],[48,3500,25],[46,3000,30],[45,2500,35],[42,2100,40]]) {
  const a = atmosphere(hkm*1000); const st = lookupStag(T, hkm*1000, U); const q = 0.5*a.rho*U*U
  const al = aDeg*Math.PI/180; const vhat = normalize(v3(Math.cos(al), 0, -Math.sin(al)))
  const b = mhdBasis(m, { NI: v.coil.NI, vhat, U, rhoInf: a.rho, pInf: a.p, qInf: q, stag: st, xcg: 25.6, kappa: 0.03, seedFraction: 0.01 })
  const c = staticCoeffs(am, al, U/a.a, 25.6, st.cpMax)
  const need = -c.Cm * q * am.sref * am.lref // +My needed to trim
  const up = allocateMhd(b, v3(0, 1e12, 0), q, 2.5, 0).maxMoment
  const dn = allocateMhd(b, v3(0, -1e12, 0), q, 2.5, 0).maxMoment
  const side = allocateMhd(b, v3(0, 0, 1e12), q, 2.5, 0).maxMoment
  const trim = allocateMhd(b, v3(0, need, 0), q, 2.5, 0.35)
  console.log(`h=${hkm} U=${U} α=${aDeg} q=${(q/1e3).toFixed(1)}kPa M=${(U/a.a).toFixed(1)} need +${(need/1e6).toFixed(2)} | avail +My ${(up/1e6).toFixed(2)} −My ${(dn/1e6).toFixed(2)} ±Mz ${(side/1e6).toFixed(2)} MNm | margin ${(up/need).toFixed(2)} | trim P=${(trim.out.power/1e6).toFixed(1)}MW drag=${(-trim.out.F.x/1e3).toFixed(0)}kN heat×${trim.out.heatFactor.toFixed(2)} β=${b.hallStag.toFixed(1)} σeff=${b.sigmaEffStag.toFixed(1)}`)
}
