/**
 * Pre-compute the mission recording and write it to the app's public directory.
 *
 * The browser then plays that recording straight away instead of re-simulating in a Web Worker —
 * which is what a visitor to the static GitHub Pages build should get. The format is one binary file:
 *
 *   u32  little-endian header length
 *   u8[] header JSON (padded to an 8-byte boundary: { format, n, cells, predK, ch, events, meta })
 *   f64[] data: n × ch (channel-major, the same order as @ship/sim CHANNELS)
 *   f32[] heat: n × cells
 *   f32[] pred: n × predK × 3
 *   u8[]  predHas: n
 *
 * The recording is the **no-wind** mission: it is what the app plays when its "No wind" box is
 * ticked (the default), so a visitor sees the catch without waiting for a simulation. With wind on,
 * the app re-simulates.
 *
 * Run: bun scripts/snapshot.ts
 */
import { mkdirSync, statSync, writeFileSync } from "node:fs"
import { resolve } from "node:path"
import { FLIGHT_DESIGN } from "@ship/craft"
import { N_CH, runMission } from "@ship/sim"
import { physicsVersion } from "../packages/app/simVersion"

const t0 = performance.now()
let last = -1
const { recording } = runMission({ ...FLIGHT_DESIGN, windScale: 0, turbulence: 0 }, (p) => {
  if (p.stage === "simulating") {
    const pct = Math.floor(p.fraction * 20) * 5
    if (pct !== last) { last = pct; process.stdout.write(`\r  simulating ${pct}%  ${p.phase} @ ${(p.alt / 1000).toFixed(0)} km   `) }
  }
})
process.stdout.write("\n")
const r = recording
const header = {
  format: 1,
  version: physicsVersion(resolve(import.meta.dir, "..", "packages")),
  n: r.n,
  cells: r.cells,
  predK: r.predK,
  ch: N_CH,
  events: r.events,
  meta: { ...r.meta, wallClockMs: Math.round(performance.now() - t0) },
}
const enc = new TextEncoder()
const raw = enc.encode(JSON.stringify(header))
const headLen = Math.ceil((raw.length + 4) / 8) * 8 - 4
const size = 4 + headLen + 8 * N_CH * r.n + 4 * r.n * r.cells + 4 * r.n * r.predK * 3 + r.n
const buf = new ArrayBuffer(size)
new DataView(buf).setUint32(0, headLen, true)
const out = new Uint8Array(buf)
out.set(raw, 4) // the padding bytes stay zero
let off = 4 + headLen
const data = new Float64Array(buf, off, N_CH * r.n)
for (let c = 0; c < N_CH; c++) data.set(r.data[c], c * r.n)
off += 8 * N_CH * r.n
const heat = new Float32Array(buf, off, r.n * r.cells)
heat.set(r.heat)
off += 4 * r.n * r.cells
const pred = new Float32Array(buf, off, r.n * r.predK * 3)
pred.set(r.pred)
off += 4 * r.n * r.predK * 3
new Uint8Array(buf, off, r.n).set(r.predHas)

mkdirSync("packages/app/public", { recursive: true })
const path = "packages/app/public/recording.bin"
writeFileSync(path, out)
const mb = (n: number): string => (n / 1e6).toFixed(1)
console.log(`recording: n ${r.n}, cells ${r.cells}, channels ${N_CH}, predK ${r.predK}`)
console.log(`outcome: ${r.meta.outcome}${r.meta.summary.reason ? ` (${r.meta.summary.reason})` : ""}`)
console.log(`wrote ${path}: ${mb(statSync(path).size)} MB in ${((performance.now() - t0) / 1000).toFixed(0)} s`)
