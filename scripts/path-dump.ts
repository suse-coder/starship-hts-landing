/**
 * Dump the flight path, the guidance plan at each phase and the tower boxes to JSON, for
 * `scripts/path-plot.py` (Pillow) to render as a figure.
 *
 *   bun scripts/path-dump.ts [--wind] [out.json]
 */
import { FLIGHT_DESIGN } from "@ship/craft"
import { CH, initState, makeWorld, step, type SimState, type World } from "@ship/sim"
import { armsInit, TOWER, towerBoxes } from "@ship/tower"

const wind = process.argv.includes("--wind")
const out = process.argv.find((a) => a.endsWith(".json")) ?? `.tmp/path-${wind ? "wind" : "nowind"}.json`
const world: World = makeWorld({ ...FLIGHT_DESIGN, ...(wind ? {} : { windScale: 0, turbulence: 0 }) })

interface Sample { t: number; phase: string; x: number; y: number; z: number; vx: number; vy: number; vz: number; header: number; plan?: { x: number; y: number; z: number }[] }
const samples: Sample[] = []
let s: SimState = initState(world)
let last = -1e9
while (!s.done) {
  const o = step(world, s)
  const r = o.row
  if (s.t - last >= 0.5 && Math.abs(r[CH.pinX]) < 40e3 && Math.abs(r[CH.pinY]) < 40e3) {
    samples.push({
      t: s.t, phase: s.mem.phase,
      x: r[CH.pinX], y: r[CH.pinY], z: r[CH.pinZ],
      vx: r[CH.ve], vy: r[CH.vn], vz: r[CH.vu],
      header: r[CH.header],
      plan: s.mem.predPath.length > 1 ? s.mem.predPath.map((q) => ({ x: q.x, y: q.y, z: q.z })) : undefined,
    })
    last = s.t
  }
  s = o.s
}
const boxes = towerBoxes({ ...armsInit, open: 0 }).map((b) => ({ id: b.id, lo: b.lo, hi: b.hi }))
const doc = {
  wind, outcome: s.outcome, reason: s.mem.reason, armHeight: TOWER.armHeight,
  samples, boxes,
  summary: {
    t: s.t, header: s.cons.loxHeader + s.cons.ch4Header,
    caught: s.mem.caughtT > 0, offset: Math.hypot(samples[samples.length - 1]?.x ?? 0, samples[samples.length - 1]?.y ?? 0),
  },
}
await Bun.write(out, JSON.stringify(doc))
console.log(`wrote ${out}: ${samples.length} samples, ${boxes.length} boxes, outcome ${s.outcome}`)
