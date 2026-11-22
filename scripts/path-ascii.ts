/**
 * ASCII flight-path report: the whole descent drawn in the terminal, so a path can be judged by eye
 * (is it heading for the tower? does it cross the tower structure? is the plan the shortest route?).
 *
 *   bun scripts/path-ascii.ts              no wind (the recorded mission)
 *   bun scripts/path-ascii.ts --wind       default wind + turbulence
 *   bun scripts/path-ascii.ts --plan       also draw the guidance's own predicted path per phase
 *
 * Two views per report:
 *   SIDE  — north (m, relative to the catch point) across, height above the rails up. The tower
 *           column, carriage and arms are drawn as solid blocks; the ship's path is the line.
 *   PLAN  — east across, north up (the ground track), with the tower footprint and the arm span.
 *
 * A collision check runs on the flown path against every tower box (with a margin) and prints the
 * first penetration, so "it looked fine" is never the only evidence.
 */
import { FLIGHT_DESIGN } from "@ship/craft"
import { CH, initState, makeWorld, step, type SimState, type World } from "@ship/sim"
import { armsInit, TOWER, towerBoxes } from "@ship/tower"

const wind = process.argv.includes("--wind")
const wantPlan = process.argv.includes("--plan")
const world: World = makeWorld({ ...FLIGHT_DESIGN, ...(wind ? {} : { windScale: 0, turbulence: 0 }) })

interface Pt {
  readonly x: number
  readonly y: number
  readonly z: number
  readonly phase: string
  readonly t: number
}
const path: Pt[] = []
const plans: { phase: string; t: number; path: readonly { x: number; y: number; z: number }[] }[] = []
let s: SimState = initState(world)
let lastSample = -1e9
let lastPlanPhase = ""
while (!s.done) {
  const out = step(world, s)
  const r = out.row
  if (s.t - lastSample >= 1) {
    path.push({ x: r[CH.pinX], y: r[CH.pinY], z: r[CH.pinZ], phase: s.mem.phase, t: s.t })
    lastSample = s.t
  }
  if (wantPlan && s.mem.predPath.length > 1 && s.mem.phase !== lastPlanPhase) {
    lastPlanPhase = s.mem.phase
    plans.push({ phase: s.mem.phase, t: s.t, path: s.mem.predPath })
  }
  s = out.s
}
const boxes = towerBoxes({ ...armsInit, open: 0 })

/** first box the path enters (with margin), or null */
const collision = (pts: readonly Pt[], margin: number): { id: string; t: number; x: number; y: number; z: number } | null => {
  for (const p of pts) {
    for (const b of boxes) {
      if (p.x > b.lo.x - margin && p.x < b.hi.x + margin && p.y > b.lo.y - margin && p.y < b.hi.y + margin && p.z > b.lo.z - margin - TOWER.armHeight && p.z < b.hi.z + margin - TOWER.armHeight) {
        return { id: b.id, t: p.t, x: p.x, y: p.y, z: p.z }
      }
    }
  }
  return null
}

/** draw a 2-D grid; `u` is the horizontal axis, `v` the vertical, `blockOf` the solid tower boxes */
const plot = (
  pts: readonly Pt[],
  horiz: (p: Pt) => number,
  vert: (p: Pt) => number,
  blockOf: (b: (typeof boxes)[number]) => { u0: number; u1: number; v0: number; v1: number } | null,
  title: string,
  uRange: [number, number],
  vRange: [number, number],
  plan?: readonly { x: number; y: number; z: number }[],
): string => {
  const W = 78, H = 22
  const g: string[][] = Array.from({ length: H }, () => Array.from({ length: W }, () => " "))
  const [u0, u1] = uRange, [v0, v1] = vRange
  const cu = (u: number): number => Math.round(((u - u0) / (u1 - u0)) * (W - 1))
  const cv = (v: number): number => Math.round(((v - v0) / (v1 - v0)) * (H - 1))
  const put = (u: number, v: number, ch: string): void => {
    const x = cu(u), y = H - 1 - cv(v)
    if (x >= 0 && x < W && y >= 0 && y < H) g[y][x] = ch
  }
  for (const b of boxes) {
    const r = blockOf(b)
    if (!r) continue
    const a = Math.max(0, Math.min(cu(r.u0), cu(r.u1))), c = Math.min(W - 1, Math.max(cu(r.u0), cu(r.u1)))
    const d = Math.max(0, Math.min(H - 1 - cv(r.v1), H - 1 - cv(r.v0))), e = Math.min(H - 1, Math.max(H - 1 - cv(r.v1), H - 1 - cv(r.v0)))
    if (a > c || d > e) continue
    for (let y = d; y <= e; y++) for (let x = a; x <= c; x++) if (g[y][x] === " ") g[y][x] = "▒"
  }
  if (plan) for (const q of plan) put(horiz({ x: q.x, y: q.y, z: q.z, phase: "", t: 0 }), vert({ x: q.x, y: q.y, z: q.z, phase: "", t: 0 }), "·")
  for (const p of pts) put(horiz(p), vert(p), "•")
  const last = pts[pts.length - 1]
  if (last) put(horiz(last), vert(last), "★")
  return [`${title}`, `  u ${u0.toFixed(0)} … ${u1.toFixed(0)} m   v ${v0.toFixed(0)} … ${v1.toFixed(0)} m`, ...g.map((r) => `  |${r.join("").replace(/\s+$/, "")}`)].join("\n")
}

const side = (p: Pt): number => p.y
const sideV = (p: Pt): number => p.z
const planU = (p: Pt): number => p.x
const planV = (p: Pt): number => p.y
const towerSide = (b: (typeof boxes)[number]): { u0: number; u1: number; v0: number; v1: number } | null =>
  b.hi.z < 1 || b.lo.z > TOWER.armHeight + 40 ? null : { u0: b.lo.y, u1: b.hi.y, v0: b.lo.z - TOWER.armHeight, v1: b.hi.z - TOWER.armHeight }
const towerPlan = (b: (typeof boxes)[number]): { u0: number; u1: number; v0: number; v1: number } | null =>
  b.hi.z < 1 ? null : { u0: b.lo.x, u1: b.hi.x, v0: b.lo.y, v1: b.hi.y }

const phases = ["bellyFlop", "flip", "landing", "terminal"]
const label = wind ? "WIND" : "NO WIND"
console.log(`\n=== ${label}: ${s.outcome}${s.mem.reason ? ` (${s.mem.reason})` : ""} — ${path.length} samples ===`)
// the pin frame is only meaningful near the pad (the pad plane is tangent), so the approach is
// summarised in words and the plots start at the belly flop
for (const ph of ["entry", "loft", "pitchUp", "highAlpha"]) {
  const pts = path.filter((p) => p.phase === ph)
  if (!pts.length) continue
  const first = pts[0], last = pts[pts.length - 1]
  console.log(`  ${ph.padEnd(9)} t ${first.t.toFixed(0)}–${last.t.toFixed(0)} s, ${pts.length} samples`)
}
for (const ph of phases) {
  const pts = path.filter((p) => p.phase === ph)
  if (pts.length < 2) continue
  const plan = plans.find((q) => q.phase === ph)?.path
  const ys = pts.map((p) => p.y), zs = pts.map((p) => p.z)
  const near = ph === "flip" || ph === "landing" || ph === "terminal"
  const uMin = near ? Math.min(...ys, -120) : Math.min(...ys), uMax = near ? Math.max(...ys, 120) : Math.max(...ys)
  const vMin = 0, vMax = near ? Math.max(...zs, 420) : Math.max(...zs, 30)
  console.log(`\n--- ${ph}: t ${pts[0].t.toFixed(0)}–${pts[pts.length - 1].t.toFixed(0)} s, north ${uMin.toFixed(0)}…${uMax.toFixed(0)} m, height ${vMin.toFixed(0)}…${vMax.toFixed(0)} m${plan ? " (· = guidance plan)" : ""}`)
  console.log(plot(pts, side, sideV, towerSide, "SIDE (north vs height above the rails)", [uMin, uMax], [vMin, vMax], plan))
}
console.log(`\n--- ground track, all phases`)
const nearPts = path.filter((p) => Math.abs(p.x) < 40e3 && Math.abs(p.y) < 40e3)
const xs = nearPts.map((p) => p.x), ys = nearPts.map((p) => p.y)
const lim = Math.max(120, Math.min(20e3, Math.max(...xs.map(Math.abs), ...ys.map(Math.abs)) * 1.1))
console.log(plot(nearPts, planU, planV, towerPlan, "PLAN (east vs north, catch point at 0,0)", [-lim, lim], [-lim, lim]))
const hit = collision(path, 1)
console.log(hit ? `\nCOLLISION: flown path enters '${hit.id}' at t ${hit.t.toFixed(1)} s (E ${hit.x.toFixed(1)}, N ${hit.y.toFixed(1)}, up ${hit.z.toFixed(1)})` : `\nno tower-box penetration on the flown path (1 m margin)`)
console.log(`final: t ${s.t.toFixed(1)} s, outcome ${s.outcome}, header ${((s.cons.loxHeader + s.cons.ch4Header) / 1e3).toFixed(2)} t`)
