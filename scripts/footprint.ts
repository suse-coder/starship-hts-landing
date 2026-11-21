/** Belly-flop reachable footprint from the recorded belly-flop start: fixed lift heading relative to the
 *  ground track, lift kept at or above the horizon (as the flight law), no wind. */
import { runMission, CH } from "@ship/sim"
import { DEFAULT_DESIGN } from "@ship/craft"
import { PHASES, aeroLookup, trimAlpha } from "@ship/gnc"
import { atmosphere } from "@ship/earth"
const { recording: rec, world: W } = runMission(DEFAULT_DESIGN)
const d = rec.data
const ph0 = PHASES.findIndex((p) => p.id === "bellyFlop")
let i = 0
const tArg = Number(process.argv[2] ?? 0)
while (i < rec.n && (d[CH.phase][i] < ph0 || d[CH.t][i] < tArg)) i++
const m = d[CH.mass][i], S = W.aero.sref
const trk = Math.atan2(d[CH.ve][i], d[CH.vn][i])
console.log("t", d[CH.t][i].toFixed(1), "pos", (d[CH.pe][i] / 1e3).toFixed(1), (d[CH.pn][i] / 1e3).toFixed(1), "km  v", d[CH.ve][i].toFixed(0), d[CH.vn][i].toFixed(0), d[CH.vu][i].toFixed(0), "h", d[CH.hBase][i].toFixed(0), "track", (trk * 180 / Math.PI).toFixed(0))
const ends: [number, number, number][] = []
for (let rel = -180; rel < 180; rel += 30) {
  let px = d[CH.pe][i], py = d[CH.pn][i], h = d[CH.hBase][i], vx = d[CH.ve][i], vy = d[CH.vn][i], vz = d[CH.vu][i]
  const psi = trk + rel * Math.PI / 180, hx = Math.sin(psi), hy = Math.cos(psi)
  for (let k = 0; k < 8000 && h > 750; k++) {
    const a = atmosphere(h), V = Math.hypot(vx, vy, vz), dt = 0.25, M = V / a.a
    const co = aeroLookup(W.aeroTab, trimAlpha(W.aeroTab, M), M), q = 0.5 * a.rho * V * V
    const ux = vx / V, uy = vy / V, uz = vz / V, dd = hx * ux + hy * uy
    let lx = hx - dd * ux, ly = hy - dd * uy, lz = -dd * uz; let ln = Math.hypot(lx, ly, lz); lx /= ln; ly /= ln; lz /= ln
    if (lz < 0) { let qx = -uz * ux, qy = -uz * uy, qz = 1 - uz * uz; const qn = Math.hypot(qx, qy, qz); qx /= qn; qy /= qn; qz /= qn; const kk = -lz / Math.max(qz, 1e-3); lx += kk * qx; ly += kk * qy; lz += kk * qz; ln = Math.hypot(lx, ly, lz); lx /= ln; ly /= ln; lz /= ln }
    const L = q * S * co.CL / m, D = q * S * co.CD / m
    vx += (-D * ux + L * lx) * dt; vy += (-D * uy + L * ly) * dt; vz += (-D * uz + L * lz - 9.79) * dt
    px += vx * dt; py += vy * dt; h += vz * dt
  }
  // along/cross track relative to the start track
  const al = px * Math.sin(trk) + py * Math.cos(trk), cr = px * Math.cos(trk) - py * Math.sin(trk)
  ends.push([rel, al, cr])
  console.log(`lift heading ${String(rel).padStart(4)}° from track → end (${(px / 1e3).toFixed(2)}, ${(py / 1e3).toFixed(2)}) km; along-track ${(al / 1e3).toFixed(2)} cross ${(cr / 1e3).toFixed(2)} km (pad frame)`)
}
