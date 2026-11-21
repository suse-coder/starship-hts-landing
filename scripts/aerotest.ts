import { makeVehicle } from "@ship/craft"
import { makeAeroModel, staticCoeffs, aeroWrench } from "@ship/aero"
import { v3, ZERO } from "@ship/math"
const v = makeVehicle(); const m = makeAeroModel(v)
console.log("panels", m.panels.length, "strips", m.strips.length, "wetted", m.wetted.toFixed(0))
const xcg = 25.6
for (const M of [0.5, 1.2, 3, 8]) {
  const row = [0, 5, 10, 20, 30, 45, 60, 75, 80, 90, 100].map(a => {
    const c = staticCoeffs(m, a*Math.PI/180, M, xcg, 1.9)
    const V = M*300, al = a*Math.PI/180
    const o = aeroWrench(m, { vAir: v3(V*Math.cos(al),0,-V*Math.sin(al)), w: ZERO, rho: 0.01, a: 300, mu: 1.5e-5, xcg, cpMax: 1.9, enginesOn: false, knudsen: 0 })
    return `${a}°:L/D=${(c.CL/c.CD).toFixed(2)} CL=${c.CL.toFixed(2)} CD=${c.CD.toFixed(2)} CA=${o.CA.toFixed(2)} CN=${o.CN.toFixed(2)} Cm=${c.Cm.toFixed(4)} xcp=${o.xcp.toFixed(1)}`
  })
  console.log(`M=${M}\n  ` + row.join("\n  "))
}
// damping check: pitch rate at alpha 90 M0.5
const o1 = aeroWrench(m, { vAir: v3(0,0,-80), w: v3(0,0.1,0), rho: 1, a: 340, mu: 1.8e-5, xcg, cpMax: 1.9, enginesOn: false, knudsen: 0 })
console.log("damping My for wy=0.1 at belly flop:", o1.M.y.toExponential(2), "F", o1.F)
