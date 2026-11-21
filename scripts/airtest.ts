import { equilibrium, normalShock, S } from "@ship/gas"
import { atmosphere } from "@ship/earth"
for (const T of [300, 2000, 3000, 4000, 6000, 8000, 10000, 15000]) {
  const rho = 101325 * 0.02897 / (8.314 * T) // ~1 atm initial guess density
  const s = equilibrium(T, rho)
  const x = (i: number) => (s.n[i] / s.ntot).toExponential(2)
  console.log(`T=${T} p=${(s.p/101325).toFixed(3)}atm h=${(s.h/1e6).toFixed(2)}MJ/kg N2=${x(S.N2)} O2=${x(S.O2)} NO=${x(S.NO)} N=${x(S.N)} O=${x(S.O)} e=${x(S.e)} ne=${s.n[S.e].toExponential(2)}`)
}
for (const [hkm, U] of [[70,7400],[60,6800],[50,5000],[40,3000],[30,1500],[20,700]]) {
  const a = atmosphere(hkm*1000); const t0 = performance.now()
  const r = normalShock(a.T, a.rho, U)
  console.log(`h=${hkm} U=${U} eps=${r.eps.toFixed(4)} T2=${r.s2.T.toFixed(0)} T0=${r.s0.T.toFixed(0)} p0=${(r.s0.p).toFixed(0)}Pa cpmax=${r.cpMax.toFixed(3)} ne0=${r.s0.n[S.e].toExponential(2)} (${(performance.now()-t0).toFixed(1)} ms)`)
}
