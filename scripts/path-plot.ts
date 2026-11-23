/**
 * Render the flight path from `scripts/path-dump.ts` as an SVG figure (TypeScript only).
 *
 *   bun scripts/path-plot.ts .tmp/path-nowind.json docs/03-spec/images/path-nowind.svg
 *   convert docs/03-spec/images/path-nowind.svg docs/03-spec/images/path-nowind.png
 *
 * Every line is emitted as a thin polygon: the rasteriser used here (ImageMagick's built-in SVG)
 * draws polygons, rectangles, circles and text but ignores stroke on <line>/<path>, so a stroked
 * path would come out invisible or filled black.
 *
 * Panels: belly flop → catch (north vs height), the last 600 m with the guidance plan, the terminal
 * ground track, and the belly-flop ground track. Tower boxes are drawn solid.
 */
import { readFileSync, writeFileSync } from "node:fs"

interface P { x: number; y: number; z: number }
interface Sample { t: number; phase: string; x: number; y: number; z: number; vx: number; vy: number; vz: number; header: number; plan?: P[] }
interface Box { id: string; lo: P; hi: P }
interface Doc { wind: boolean; outcome: string; reason: string; armHeight: number; samples: Sample[]; boxes: Box[]; summary: { t: number; header: number; caught: boolean; offset: number } }

const src = process.argv[2] ?? ".tmp/path-nowind.json"
const dst = process.argv[3] ?? "docs/03-spec/images/path.svg"
const doc = JSON.parse(readFileSync(src, "utf8")) as Doc
const S = doc.samples
const AH = doc.armHeight

const BG = "#0c121a", PANEL = "#101720", FG = "#d2e1eb", DIM = "#5a6e7d", GRID = "#1c2834"
const FLOWN = "#5adcff", PLAN = "#ffaa3c", TOWER = "#788796", CATCH = "#5af096", BAD = "#ff5a5a"
const W = 1800, H = 1180

const out: string[] = []
const esc = (s: string): string => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;")
const text = (x: number, y: number, s: string, size = 13, fill = DIM, weight = "normal"): void => {
  out.push(`<text x="${x.toFixed(1)}" y="${y.toFixed(1)}" font-family="DejaVu Sans Mono, monospace" font-size="${size}" font-weight="${weight}" fill="${fill}">${esc(s)}</text>`)
}

type Rect = readonly [number, number, number, number]
/** Liang–Barsky segment clip to a rect; null when the segment misses it */
const clip = (x0: number, y0: number, x1: number, y1: number, r: Rect): [number, number, number, number] | null => {
  const [rx0, ry0, rx1, ry1] = r
  const dx = x1 - x0, dy = y1 - y0
  let t0 = 0, t1 = 1
  for (const [p, q] of [[-dx, x0 - rx0], [dx, rx1 - x0], [-dy, y0 - ry0], [dy, ry1 - y0]] as const) {
    if (p === 0) { if (q < 0) return null; continue }
    const t = q / p
    if (p < 0) { if (t > t1) return null; if (t > t0) t0 = t } else { if (t < t0) return null; if (t < t1) t1 = t }
  }
  return [x0 + t0 * dx, y0 + t0 * dy, x0 + t1 * dx, y0 + t1 * dy]
}

const quad = (x0: number, y0: number, x1: number, y1: number, w: number, color: string): void => {
  const dx = x1 - x0, dy = y1 - y0, n = Math.hypot(dx, dy) || 1
  const nx = (-dy / n) * (w / 2), ny = (dx / n) * (w / 2)
  out.push(`<polygon points="${(x0 + nx).toFixed(1)},${(y0 + ny).toFixed(1)} ${(x1 + nx).toFixed(1)},${(y1 + ny).toFixed(1)} ${(x1 - nx).toFixed(1)},${(y1 - ny).toFixed(1)} ${(x0 - nx).toFixed(1)},${(y0 - ny).toFixed(1)}" fill="${color}"/>`)
}

const segment = (x0: number, y0: number, x1: number, y1: number, w: number, color: string, r?: Rect): void => {
  const c = r ? clip(x0, y0, x1, y1, r) : [x0, y0, x1, y1] as [number, number, number, number]
  if (!c) return
  quad(c[0], c[1], c[2], c[3], w, color)
}

/** dashed polyline: walk the path emitting dash-length quads */
const dashed = (pts: readonly [number, number][], w: number, color: string, dash: number, gap: number, r: Rect): void => {
  let carry = 0, on = true
  for (let i = 0; i + 1 < pts.length; i++) {
    let [x0, y0] = pts[i]
    const [x1, y1] = pts[i + 1]
    let len = Math.hypot(x1 - x0, y1 - y0)
    if (len < 1e-9) continue
    let t = 0
    while (t < len) {
      const step = Math.min((on ? dash : gap) - carry, len - t)
      if (on) segment(x0 + ((x1 - x0) * t) / len, y0 + ((y1 - y0) * t) / len, x0 + ((x1 - x0) * (t + step)) / len, y0 + ((y1 - y0) * (t + step)) / len, w, color, r)
      t += step
      carry += step
      if (carry >= (on ? dash : gap) - 1e-9) { carry = 0; on = !on }
    }
  }
}

interface PanelSpec { readonly box: Rect; readonly title: string; readonly u: readonly [number, number]; readonly v: readonly [number, number]; readonly mode: "side" | "plan" }

const panel = (spec: PanelSpec): void => {
  const [x0, y0, x1, y1] = spec.box
  const [u0, u1] = spec.u, [v0, v1] = spec.v
  const px = (u: number): number => x0 + ((u - u0) / (u1 - u0)) * (x1 - x0)
  const py = (v: number): number => y1 - ((v - v0) / (v1 - v0)) * (y1 - y0)
  const rect: Rect = [x0, y0, x1, y1]
  out.push(`<rect x="${x0}" y="${y0}" width="${x1 - x0}" height="${y1 - y0}" fill="${PANEL}"/>`)
  for (let k = 1; k < 5; k++) {
    const u = u0 + ((u1 - u0) * k) / 5, v = v0 + ((v1 - v0) * k) / 5
    segment(px(u), y0, px(u), y1, 1, GRID, rect)
    segment(x0, py(v), x1, py(v), 1, GRID, rect)
  }
  for (const b of doc.boxes) {
    let a: number, c: number, e: number, f: number
    if (spec.mode === "side") {
      if (b.hi.z < 1 || b.lo.z > AH + 40) continue
      a = px(Math.min(b.lo.y, b.hi.y)); c = px(Math.max(b.lo.y, b.hi.y))
      e = py(b.lo.z - AH); f = py(b.hi.z - AH)
    } else {
      if (b.hi.z < 1) continue
      a = px(Math.min(b.lo.x, b.hi.x)); c = px(Math.max(b.lo.x, b.hi.x))
      e = py(Math.min(b.lo.y, b.hi.y)); f = py(Math.max(b.lo.y, b.hi.y))
    }
    const cx0 = Math.max(a, x0), cx1 = Math.min(c, x1), cy0 = Math.max(e, y0), cy1 = Math.min(f, y1)
    if (cx1 <= cx0 || cy1 <= cy0) continue
    out.push(`<rect x="${cx0.toFixed(1)}" y="${cy0.toFixed(1)}" width="${(cx1 - cx0).toFixed(1)}" height="${(cy1 - cy0).toFixed(1)}" fill="${TOWER}"/>`)
  }
  // guidance plans: one per phase change / 20 s, dashed
  let lastPlanT = -1e9, lastPhase = ""
  for (const s of S) {
    if (!s.plan || s.plan.length < 2) continue
    if (s.phase === lastPhase && s.t - lastPlanT < 20) continue
    lastPhase = s.phase; lastPlanT = s.t
    dashed(s.plan.map((q) => [px(spec.mode === "side" ? q.y : q.x), py(spec.mode === "side" ? q.z : q.y)] as [number, number]), 1.8, PLAN, 7, 5, rect)
  }
  // flown path
  const pts = S.map((s) => [px(spec.mode === "side" ? s.y : s.x), py(spec.mode === "side" ? s.z : s.y)] as [number, number])
  for (let i = 0; i + 1 < pts.length; i++) segment(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], 2.6, FLOWN, rect)
  const last = S[S.length - 1]
  if (last) out.push(`<circle cx="${px(spec.mode === "side" ? last.y : last.x).toFixed(1)}" cy="${py(spec.mode === "side" ? last.z : last.y).toFixed(1)}" r="4.5" fill="${FLOWN}"/>`)
  // catch point cross
  const cx = px(0), cy = py(0)
  segment(cx - 9, cy, cx + 9, cy, 2.4, CATCH, rect)
  segment(cx, cy - 9, cx, cy + 9, 2.4, CATCH, rect)
  text(x0, y0 - 24, spec.title, 17, FG, "bold")
  text(x0 + 2, y0 + 16, spec.mode === "side" ? "height above rails (m)" : "north (m)", 12, DIM)
  text(x0 + 2, y1 + 18, spec.mode === "side" ? "north (m)" : "east (m)", 12, DIM)
  text(x1 - 150, y1 + 18, `${u0.toFixed(0)} … ${u1.toFixed(0)}`, 12, DIM)
}

out.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">`)
out.push(`<rect width="${W}" height="${H}" fill="${BG}"/>`)
const terminal = process.argv.includes("--terminal")
if (terminal) {
  // the last seconds only: where the overshoot lives
  panel({ box: [70, 100, 880, 530], title: "SIDE — last 25 m of height (north vs height)", u: [-14, 14], v: [0, 26], mode: "side" })
  panel({ box: [960, 100, 1730, 530], title: "SIDE — last 3 m of height (north vs height)", u: [-2.2, 2.2], v: [0, 3.2], mode: "side" })
  panel({ box: [70, 660, 880, 1090], title: "PLAN — within ±14 m of the axis (east vs north)", u: [-14, 14], v: [-14, 14], mode: "plan" })
  panel({ box: [960, 660, 1730, 1090], title: "PLAN — within ±2.2 m of the axis", u: [-2.2, 2.2], v: [-2.2, 2.2], mode: "plan" })
} else {
  panel({ box: [70, 100, 880, 530], title: "SIDE — belly flop to catch (north vs height)", u: [-2200, 7200], v: [0, 25000], mode: "side" })
  panel({ box: [960, 100, 1730, 530], title: "SIDE — last 600 m (dashed = guidance plan)", u: [-420, 420], v: [0, 620], mode: "side" })
  panel({ box: [70, 660, 880, 1090], title: "PLAN — terminal ground track", u: [-420, 420], v: [-420, 420], mode: "plan" })
  panel({ box: [960, 660, 1730, 1090], title: "PLAN — belly flop ground track", u: [-4200, 4200], v: [-4200, 4200], mode: "plan" })
}
text(70, 40, `Ship 42 / Flight 15 — ${doc.wind ? "WIND" : "NO WIND"}`, 21, FG, "bold")
text(470, 42, `${doc.summary.caught ? "CAUGHT" : "LOST"}  offset ${doc.summary.offset.toFixed(2)} m  ${doc.reason.slice(0, 52)}`, 15, doc.summary.caught ? CATCH : BAD)
text(70, 1140, "cyan = flown path   orange dashed = guidance plan   grey = solid tower boxes   green + = catch point (arm rail top)", 13, DIM)
out.push(`</svg>`)
writeFileSync(dst, out.join("\n"))
console.log(`wrote ${dst}`)
