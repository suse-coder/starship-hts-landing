/** Cold-gas angular authority about v̂ at high α (body roll + yaw mix), from the vehicle model. */
import { makeVehicle, DEFAULT_DESIGN, massProps, initialConsumables } from "@ship/craft"
import { cross, sub, v3, scale, add } from "@ship/math"
const v = makeVehicle(DEFAULT_DESIGN)
const c = initialConsumables(v)
const mp = massProps(v, { ...c, n2: c.n2 * 0.6 })
console.log("mass", mp.m.toFixed(0), "kg  xcg", mp.xcg.toFixed(1), "m  I roll/pitch/yaw", mp.I.x.toExponential(2), mp.I.y.toExponential(2), mp.I.z.toExponential(2), "kg m²")
const Ms = v.couples.map((cp) => [cp.a, cp.b].reduce((M, k) => { const t = v.thrusters[k]; return add(M, cross(sub(t.pos, v3(mp.xcg, 0, 0)), scale(t.dir, t.thrust))) }, v3(0, 0, 0)))
Ms.forEach((M, i) => console.log(`couple ${i}: M = (${(M.x / 1e3).toFixed(0)}, ${(M.y / 1e3).toFixed(0)}, ${(M.z / 1e3).toFixed(0)}) kN·m`))
const Myaw = Math.max(...Ms.map((M) => Math.abs(M.z))), Mroll = Math.max(...Ms.map((M) => Math.abs(M.x)))
for (const aDeg of [45, 65, 80]) {
  const a = aDeg * Math.PI / 180
  // unit angular acceleration about v̂ needs roll Ix cos α and yaw Iz sin α (body components)
  const acc = Math.min(Mroll / (mp.I.x * Math.cos(a)), Myaw / (mp.I.z * Math.sin(a)))
  const wMax = 2.5 * Math.PI / 180, ang = 100 * Math.PI / 180
  const tRev = 2 * wMax / acc + (ang - wMax * wMax / acc) / wMax
  console.log(`α ${aDeg}°: max angular acceleration about v̂ ${acc.toFixed(4)} rad/s² → 100° reversal at ≤ 2.5°/s takes ${tRev.toFixed(0)} s (avg ${(100 / tRev).toFixed(2)}°/s)`)
}
