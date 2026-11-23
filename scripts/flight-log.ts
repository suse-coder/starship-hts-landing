/**
 * Per-phase flight log in geometry form (docs/03-spec/flight/): what the guidance planned and what
 * the vehicle flew, phase by phase, as tables and spline knots an analysis can reason about.
 *
 *   bun scripts/flight-log.ts [--out=docs/03-spec/flight/nowind] [--every=0.2]
 */
import { mkdirSync, writeFileSync } from "node:fs"
import { FLIGHT_DESIGN } from "@ship/craft"
import { qrot, v3 } from "@ship/math"
import type { Phase } from "@ship/gnc"
import { CH, initState, makeWorld, step, type SimState } from "@ship/sim"
import { phaseJson, phaseMarkdown, type LogSample, type PhaseTrace } from "@ship/verify"

const arg = (k: string): string | undefined => process.argv.find((a) => a.startsWith(`--${k}=`))?.slice(k.length + 3)
const OUT = arg("out") ?? "docs/03-spec/flight/nowind"
const EVERY = Number(arg("every") ?? 0.2)
const W = makeWorld({ ...FLIGHT_DESIGN, windScale: 0, turbulence: 0 })

/** nearest point of a plan polyline to the flown pin position (catch frame) */
const nearest = (p: { x: number; y: number; z: number }, path: readonly { x: number; y: number; z: number }[]): { x: number; y: number; z: number } | null => {
  if (path.length === 0) return null
  let best = path[0], bd = Infinity
  for (const q of path) {
    const d = (q.x - p.x) ** 2 + (q.y - p.y) ** 2 + (q.z - p.z) ** 2
    if (d < bd) { bd = d; best = q }
  }
  return best
}

let s: SimState = initState(W)
const phaseOf = (x: SimState): Phase => x.mem.phase
const traces: PhaseTrace[] = []
let samples: LogSample[] = []
let events: { t: number; msg: string }[] = []
let t0 = 0
let phase = phaseOf(s)
let last = -1e9
const flush = (t1: number): void => {
  if (samples.length > 0) traces.push({ phase, t0, t1, samples, events })
  samples = []; events = []
}
const t00 = performance.now()
while (!s.done) {
  const out = step(W, s)
  const r = out.row
  const ph = phaseOf(out.s)
  if (ph !== phase) { flush(s.t); phase = ph; t0 = s.t }
  const t = r[CH.t]
  if (t - last >= EVERY || ph !== phase) {
    last = t
    const q = { w: r[CH.qw], x: r[CH.qx], y: r[CH.qy], z: r[CH.qz] }
    const th = r[CH.theta]
    const upE = { x: W.padEnu.u.x * Math.cos(th) - W.padEnu.u.y * Math.sin(th), y: W.padEnu.u.x * Math.sin(th) + W.padEnu.u.y * Math.cos(th), z: W.padEnu.u.z }
    const xb = qrot(q, v3(1, 0, 0))
    const cu = xb.x * upE.x + xb.y * upE.y + xb.z * upE.z
    const k = r[CH.T1] > 1e3 ? 1 : r[CH.T2] > 1e3 ? 2 : 3
    const pin = { x: r[CH.pinX], y: r[CH.pinY], z: r[CH.pinZ] }
    // the guidance's plan at this step (its predicted path to the catch), compared geometrically:
    // the nearest point of that spline, so the deviation is a distance, not a phase lag
    const plan = out.pred.length > 1 ? nearest(pin, out.pred.map((p) => ({ x: p.x, y: p.y, z: p.z - W.guide.armHeight }))) : null
    samples.push({
      t, pin, vel: { x: r[CH.ve], y: r[CH.vn], z: r[CH.vu] },
      tilt: (Math.acos(Math.max(-1, Math.min(1, cu))) * 180) / Math.PI, rollRate: (r[CH.wx] * 180) / Math.PI, attErr: r[CH.attErr],
      gp: (k === 1 ? r[CH.gp1] : k === 2 ? r[CH.gp2] : r[CH.gp3]) * 180 / Math.PI,
      gy: (k === 1 ? r[CH.gy1] : k === 2 ? r[CH.gy2] : r[CH.gy3]) * 180 / Math.PI,
      thrust: r[CH.thrust], u: { x: out.s.mem.uCmd[0], y: out.s.mem.uCmd[1], z: out.s.mem.uCmd[2] },
      plan, header: r[CH.header], arms: r[CH.arms],
    })
  }
  for (const e of out.s.events) if (e.t >= s.t - 1e-9) events.push(e)
  s = out.s
}
flush(s.t)
const wall = (performance.now() - t00) / 1e3
mkdirSync(OUT, { recursive: true })
const L: string[] = []
L.push("# Flight log — no wind (Ship 42 / Flight 15)\n")
L.push(`Full 6-DOF mission, no wind, no turbulence: \`bun scripts/flight-log.ts\`. ${traces.length} phases, ${(wall).toFixed(1)} s wall, outcome **${s.outcome}**.`)
L.push(`Samples every ${EVERY} s (0.2 s below 200 m). Positions are the catch pins in the pad frame: **E** across the rails, **N** along them, **U** above the rail plane. \`plan\` is the guidance's own reference at that time (the single-engine descent plan) or the nearest point of its predicted spline.`)
L.push(`Spline knots and every sample are in \`spline.json\`; the files are written to be read by a human or a model, not replayed.\n`)
const sum: string[] = []
for (const tr of traces) {
  L.push(phaseMarkdown(tr, 1))
  L.push("")
  const km = tr.phase
  sum.push(`| ${km} | ${tr.t0.toFixed(0)}–${tr.t1.toFixed(0)} | ${(tr.t1 - tr.t0).toFixed(1)} | ${tr.samples.length} |`)
}
writeFileSync(`${OUT}/flight.md`, L.join("\n"))
writeFileSync(`${OUT}/spline.json`, JSON.stringify({ outcome: s.outcome, wall, phases: traces.map(phaseJson) }, null, 1))
const head = `# phase summary\n\n| phase | sim time (s) | duration (s) | samples |\n|---|---|---|---|\n${sum.join("\n")}\n`
writeFileSync(`${OUT}/summary.md`, head)
console.log(`wrote ${OUT}/flight.md (${L.length} lines), spline.json (${traces.length} phases), summary.md; outcome ${s.outcome}, wall ${wall.toFixed(1)} s`)
