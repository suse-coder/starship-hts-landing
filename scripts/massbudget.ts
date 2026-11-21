/** Prints the component mass budget, CG and coil design as a markdown table. */
import { makeVehicle, massProps, initialConsumables, dryMass, planform } from "@ship/craft"
const v = makeVehicle()
console.log("| Component | Mass (t) | x (m) | Group |\n|---|---:|---:|---|")
for (const c of v.components) console.log(`| ${c.label} | ${(c.mass / 1000).toFixed(2)} | ${c.x.toFixed(1)} | ${c.group} |`)
const c0 = initialConsumables(v)
console.log(`\ndry ${(dryMass(v) / 1000).toFixed(2)} t; consumables`, Object.fromEntries(Object.entries(c0).map(([k, x]) => [k, Math.round(x)])))
const mp = massProps(v, c0); console.log(`start mass ${(mp.m / 1000).toFixed(2)} t, xcg ${mp.xcg.toFixed(2)} m, I = (${mp.I.x.toExponential(2)}, ${mp.I.y.toExponential(2)}) kg m²`)
console.log("planform", planform(v.shape), "nose cap end x", v.shape.xCapEnd.toFixed(2))
const c = v.coil
console.log(`coil: a ${c.a} m, NI ${(c.NI / 1e6).toFixed(2)} MA·t, N ${c.N.toFixed(0)}, Iop ${c.Iop} A, L ${c.L.toFixed(1)} H, E ${(c.energy / 1e6).toFixed(0)} MJ, Bcenter ${c.bCenter.toFixed(2)} T, Bpeak ${c.bPeak.toFixed(1)} T, Tcs ${c.tCs.toFixed(1)} K, battery ${(c.batteryEnergy / 3.6e6).toFixed(0)} kWh`)
console.log(`pica: area ${v.pica.area.toFixed(0)} m², mass ${(v.pica.mass / 1000).toFixed(2)} t`)
