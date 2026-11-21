import { makeVehicle } from "@ship/craft"
import { makeMhdModel, mhdBasis } from "@ship/mhd"
import { lookupStag, type ShockTable } from "@ship/gas"
import { SHOCK_TABLE as table } from "@ship/gas"
import { atmosphere } from "@ship/earth"
import { v3, normalize } from "@ship/math"
const v = makeVehicle(); const m = makeMhdModel(v); const T = table as unknown as ShockTable
for (const aDeg of [0, 15]) {
  const hkm=55, U=6000; const a = atmosphere(hkm*1000); const st = lookupStag(T, hkm*1000, U)
  const al = aDeg*Math.PI/180; const vhat = normalize(v3(Math.cos(al), 0, -Math.sin(al)))
  const b = mhdBasis(m, { NI: v.coil.NI, vhat, U, rhoInf: a.rho, pInf: a.p, qInf: 0.5*a.rho*U*U, stag: st, xcg: 25.6, kappa: 0.03, seedFraction: 0.01 })
  console.log(`α=${aDeg}`)
  b.F.forEach((f,s)=>console.log(`  sector ${s} (${s*45}-${s*45+45}°): F=(${(f.x/1e3).toFixed(0)}, ${(f.y/1e3).toFixed(0)}, ${(f.z/1e3).toFixed(0)}) kN  My=${(b.M[s].y/1e6).toFixed(2)} Mz=${(b.M[s].z/1e6).toFixed(2)} MNm`))
}
