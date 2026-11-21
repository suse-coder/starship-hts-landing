import { makeVehicle } from "@ship/craft"
import { makeMhdModel, mhdBasis } from "@ship/mhd"
import { allocateMhd } from "@ship/gnc"
import { lookupStag, SHOCK_TABLE as T } from "@ship/gas"
import { atmosphere } from "@ship/earth"
import { v3, normalize } from "@ship/math"
const v = makeVehicle(); const m = makeMhdModel(v)
for (const [hkm, U, aDeg] of [[75,7400,8.4],[70,7300,8.4],[66,7380,9.3]]) {
  const a = atmosphere(hkm*1000); const st = lookupStag(T, hkm*1000, U); const q = 0.5*a.rho*U*U
  const al = aDeg*Math.PI/180; const vhat = normalize(v3(Math.cos(al), 0, -Math.sin(al)))
  const b = mhdBasis(m, { NI: v.coil.NI, vhat, U, rhoInf: a.rho, pInf: a.p, qInf: q, stag: st, xcg: 25.6, kappa: 0.03, seedFraction: 0.01 })
  console.log("sector Mx:", b.M.map(x=>(x.x/1e3).toFixed(1)).join(" "), "kNm")
  for (const cmd of [v3(0, 3e5, 0), v3(0, 3e5, 2e5), v3(0, 0, 3e5)]) {
    const al2 = allocateMhd(b, cmd, q, 2.5, 0.35)
    console.log(`h=${hkm} cmd=(${cmd.y/1e3},${cmd.z/1e3})kNm → M=(${(al2.out.M.x/1e3).toFixed(1)}, ${(al2.out.M.y/1e3).toFixed(0)}, ${(al2.out.M.z/1e3).toFixed(0)}) kNm max=${(al2.maxMoment/1e3).toFixed(0)}`)
  }
}
