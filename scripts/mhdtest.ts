import { makeVehicle } from "@ship/craft"
import { makeMhdModel, mhdBasis, mhdApply, coilField } from "@ship/mhd"
import { lookupStag, type ShockTable } from "@ship/gas"
import { SHOCK_TABLE as table } from "@ship/gas"
import { atmosphere } from "@ship/earth"
import { v3, normalize } from "@ship/math"
const v = makeVehicle(); const m = makeMhdModel(v); const T = table as unknown as ShockTable
const [bx0] = coilField(v, 52.1, 0); console.log("B at stagnation per A·t", bx0, "→", (bx0*v.coil.NI).toFixed(3), "T; antenna B", (m.antennaB*v.coil.NI).toFixed(2), "T")
for (const [hkm, U, aDeg, seed, kappa] of [[75,7400,10,0.01,0.03],[65,7000,10,0.01,0.03],[55,6000,15,0.01,0.03],[50,4500,20,0.01,0.03],[48,3500,25,0.01,0.03],[48,3500,25,0.01,0.03],[45,2500,30,0.01,0.03],[45,2500,30,0.01,0.15],[40,1800,30,0.01,0.03]]) {
  const a = atmosphere(hkm*1000); const st = lookupStag(T, hkm*1000, U)
  const al = aDeg*Math.PI/180; const vhat = normalize(v3(Math.cos(al), 0, -Math.sin(al)))
  const t0 = performance.now()
  const b = mhdBasis(m, { NI: v.coil.NI, vhat, U, rhoInf: a.rho, pInf: a.p, qInf: 0.5*a.rho*U*U, stag: st, xcg: 25.6, kappa, seedFraction: seed })
  const dt = performance.now()-t0
  const sym = mhdApply(b, Array(8).fill(0.5), 0.5*a.rho*U*U, 2.5)
  // max lateral moment: push +z sector side vs -z
  const best = [0,1].map(sgn => { const u = Array.from({length:8},(_,s)=>{ const ang=(s+0.5)/8*2*Math.PI; const want = (sgn?1:-1)*Math.sin(ang); return want>0?1.6:0 }); return mhdApply(b,u,0.5*a.rho*U*U,2.5) })
  const q = 0.5*a.rho*U*U
  console.log(`h=${hkm} U=${U} α=${aDeg} seed=${seed} κ=${kappa}: q=${(q/1e3).toFixed(1)}kPa T0=${st.T0.toFixed(0)} σ=${b.sigmaStag.toExponential(2)} σeff=${b.sigmaEffStag.toExponential(2)} β=${b.hallStag.toFixed(1)} Q=${b.qInteraction.toFixed(3)} | sym: F=(${(sym.F.x/1e3).toFixed(0)},${(sym.F.y/1e3).toFixed(0)},${(sym.F.z/1e3).toFixed(0)})kN M=(${(sym.M.y/1e6).toFixed(2)},${(sym.M.z/1e6).toFixed(2)})MNm P=${(sym.power/1e6).toFixed(2)}MW sc=${sym.scale.toFixed(2)} Δf=${sym.standoffFactor.toFixed(2)} | ctrl My: ${(best[0].M.y/1e6).toFixed(2)} / ${(best[1].M.y/1e6).toFixed(2)} MNm, seed ${b.seedMassFlow.toFixed(2)} kg/s (${dt.toFixed(2)}ms)`)
}
