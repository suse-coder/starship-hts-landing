/**
 * MPC diagram workspace — the canvas/DOM half (the arithmetic is in ./model.ts).
 *
 * A grid of panels, one per degree of freedom / control of the landing guidance, drawn on 2D
 * canvases. All time panels share one time axis:
 *   wheel            zoom the time axis around the cursor
 *   drag             pan
 *   Shift + drag     select a time range and zoom into it
 *   double-click     back to the default window (flip − 20 s → end)
 *   click            seek the playback to that time
 *   click a title    maximise that panel (again, or Esc: back to the grid)
 *
 * Each panel keeps a static layer (grid, axes, phase boundaries, decimated lines) on an offscreen
 * canvas, rebuilt only when the window, the size or the recording changes. The per-frame layer on
 * top of it is cheap: playhead, cursor, crosshair, and the legend with the values at the cursor.
 */
import { CH, type Recording } from "@ship/sim"
import {
  COLORS, EMPTY_SOURCE, RANGES, buildPanels, clampWindow, decimate, defaultWindow, equalAspect, formatClock, formatValue, gridLayout,
  hasPlan, nearestSample, niceTicks, panWindow, polyRuns, rangeWindow, recordingSource, sameWindow, sampleAt, thinPath,
  tickLabel, timePanelRange, timeTicks, trackingStats, visibleRange, widen, xyPanelRange, zoomWindow,
  type IndexRange, type Panel, type PanelSet, type Poly, type RangeId, type TimePanel, type TimeWindow, type XYPanel,
} from "./model"

export interface DiagramsOptions {
  /** seek the playback (the same function the timeline scrubber uses); without it a click does nothing */
  readonly seek?: (t: number) => void
  /** display name of a phase id */
  readonly phaseLabel?: (phase: string) => string
}

export interface Diagrams {
  /** feed a finished recording (or null while a new run is simulated); resets the time window */
  readonly setRecording: (rec: Recording | null) => void
  /** current playback time: moves the playhead (and the legend values while the cursor is outside) */
  readonly setTime: (t: number) => void
  /** show / hide the workspace; nothing is drawn while it is hidden */
  readonly setActive: (on: boolean) => void
}

// ---- look (dark plotting surface, like the 2D side view and the loading sketch)
const BG = "#0b1119"
const PLOT = "#0f1722"
const GRID = "rgba(255,255,255,0.07)"
const ZERO = "rgba(255,255,255,0.22)"
const FRAME = "rgba(120,150,170,0.45)"
const TEXT = "#9aa6b2"
const PHASE = "rgba(143,182,204,0.45)"
const PHASE_TEXT = "#8fb6cc"
const CURSOR = "rgba(255,255,255,0.6)"
const FONT = "10px system-ui, sans-serif"
const SMALL = "9px system-ui, sans-serif"
/** left margin of every panel: fixed, so the time axes of a column line up */
const ML = 50, MR = 10, MB = 18
const LEGEND_ROW = 13

interface LegendItem {
  readonly label: string
  readonly color: string
  readonly swatch: "solid" | "dash" | "fill"
  /** value text at sample time `t` (null: no value, the label only) */
  readonly value: ((t: number) => string) | null
  /** the widest value text to reserve room for */
  readonly reserve: string
  x: number
  row: number
}

/** pixel geometry of the last static build of a panel */
interface Geo {
  readonly left: number; readonly top: number; readonly pw: number; readonly ph: number
  readonly x0: number; readonly x1: number; readonly y0: number; readonly y1: number
  readonly xStep: number; readonly yStep: number
}

interface PanelView {
  def: Panel
  readonly el: HTMLDivElement
  readonly plot: HTMLDivElement
  readonly canvas: HTMLCanvasElement
  readonly base: HTMLCanvasElement
  readonly stats: HTMLSpanElement
  w: number
  h: number
  dirty: boolean
  geo: Geo | null
  legend: LegendItem[]
}

const el = <K extends keyof HTMLElementTagNameMap>(tag: K, cls = "", text = ""): HTMLElementTagNameMap[K] => {
  const e = document.createElement(tag)
  if (cls) e.className = cls
  if (text) e.textContent = text
  return e
}
const withUnit = (v: string, unit: string): string => (unit && v !== "—" ? `${v} ${unit}` : v)
/** canvas pixel coordinates far outside the frame are clamped: megametre values next to a metre-scale axis */
const px = (v: number): number => (v > 1e6 ? 1e6 : v < -1e6 ? -1e6 : v)

export const createDiagrams = (root: HTMLElement, opts: DiagramsOptions = {}): Diagrams => {
  const phaseLabel = opts.phaseLabel ?? ((p: string) => p)

  // ---------------- state ----------------
  let rec: Recording | null = null
  let set: PanelSet = buildPanels(EMPTY_SOURCE)
  let bounds: TimeWindow = { t0: 0, t1: 1 }
  let home: TimeWindow = { t0: 0, t1: 1 }
  let win: TimeWindow = { t0: 0, t1: 1 }
  let playT = 0
  let active = false
  let fitLimits = true
  let maxId: string | null = null
  /** cursor: the time under it (shared by all panels) and where it is */
  let hover: { panel: PanelView; t: number; y: number } | null = null
  let drag: { panel: PanelView; mode: "pan" | "brush"; x0: number; x: number; win: TimeWindow; moved: boolean } | null = null
  /** the pointer gesture that just ended was a drag: the click event that follows it must not seek */
  let dragged = false
  /** the last click-to-seek, so a double-click (reset the window) can take its two seeks back */
  let lastSeek: { from: number; to: number } | null = null
  let frame = 0

  // ---------------- DOM ----------------
  root.classList.add("dg")
  root.innerHTML = ""
  const bar = el("div", "dg-bar")
  bar.append(el("span", "dg-title", "MPC diagrams"), el("span", "dg-sub", "plan · command · flown, per degree of freedom"))
  const rangeBox = el("div", "dg-ranges")
  const rangeBtns = new Map<RangeId, HTMLButtonElement>()
  for (const r of RANGES) {
    const b = el("button", "dg-btn", r.label)
    b.title = r.title
    b.onclick = () => { const w = ranges().get(r.id); if (w) setWindow(w) }
    rangeBtns.set(r.id, b)
    rangeBox.appendChild(b)
  }
  const fitLabel = el("label", "check dg-fit")
  fitLabel.title = "Include the hard limits (red dashed) in the y-axis autoscale"
  const fitBox = el("input")
  fitBox.type = "checkbox"; fitBox.checked = fitLimits
  fitBox.onchange = () => { fitLimits = fitBox.checked; invalidate() }
  fitLabel.append(fitBox, " Fit limits")
  const key = el("div", "dg-key")
  key.innerHTML =
    `<span><i class="k-plan"></i>plan</span><span><i class="k-flown"></i>flown</span><span><i class="k-cmd"></i>command</span>` +
    `<span><i class="k-band"></i>recoverable band</span><span><i class="k-hard"></i>hard limit</span><span><i class="k-play"></i>playhead</span>`
  const readout = el("span", "dg-read")
  bar.append(rangeBox, fitLabel, key, readout)
  const grid = el("div", "dg-grid")
  const help = el("div", "dg-help", "wheel: zoom time · drag: pan · Shift+drag: zoom to a range · double-click: reset · click: seek · click a title: maximise · Esc: grid")
  root.append(bar, grid, help)

  const views: PanelView[] = set.panels.map((def, i) => {
    const box = el("div", "dg-panel")
    const head = el("div", "dg-head")
    const name = el("span", "dg-name")
    name.tabIndex = 0
    name.setAttribute("role", "button")
    name.title = "Maximise this diagram / back to the grid (Esc)"
    name.append(el("b", "", String(i + 1)), ` ${def.title}`)
    if (def.unit) name.append(el("em", "", ` [${def.unit}]`))
    const stats = el("span", "dg-stats")
    head.append(name, stats)
    const plot = el("div", "dg-plot")
    const canvas = el("canvas")
    plot.appendChild(canvas)
    box.append(head, plot)
    grid.appendChild(box)
    const pv: PanelView = { def, el: box, plot, canvas, base: document.createElement("canvas"), stats, w: 0, h: 0, dirty: true, geo: null, legend: [] }
    const toggle = (): void => { maxId = maxId === def.id ? null : def.id; layout() }
    name.onclick = toggle
    name.onkeydown = (e) => { if (e.key === "Enter") toggle() }
    bind(pv)
    return pv
  })

  // ---------------- window ----------------
  function ranges(): Map<RangeId, TimeWindow | null> {
    const ps = rec ? rec.meta.phaseStarts : []
    return new Map(RANGES.map((r) => [r.id, rec ? rangeWindow(r.id, ps, bounds.t0, bounds.t1) : null]))
  }
  function setWindow(w: TimeWindow): void {
    const next = clampWindow(w, bounds)
    if (next.t0 === win.t0 && next.t1 === win.t1) return
    win = next
    invalidate()
  }
  /** everything static must be rebuilt (window, recording, options, size) */
  function invalidate(): void {
    for (const v of views) v.dirty = true
    const rs = ranges()
    for (const [id, b] of rangeBtns) {
      const w = rs.get(id) ?? null
      b.disabled = w === null
      b.classList.toggle("on", sameWindow(win, w))
    }
    schedule()
  }
  function schedule(): void {
    if (!active || frame) return
    frame = requestAnimationFrame(() => { frame = 0; draw() })
  }

  // ---------------- layout ----------------
  function layout(): void {
    for (const v of views) v.el.classList.toggle("max", v.def.id === maxId)
    grid.classList.toggle("has-max", maxId !== null)
    // Column widths are whole CSS pixels. With `1fr` columns an odd grid width gave every column a
    // half-pixel width, so the second column started on a half pixel: its canvas was resampled by
    // the browser and all of its text was soft while the first column's was sharp.
    const GAP = 6, PAD = 12
    if (maxId !== null) {
      grid.style.gridTemplateColumns = `${Math.max(120, Math.floor(grid.clientWidth - PAD))}px`
      grid.style.gridAutoRows = `${Math.max(120, grid.clientHeight - 12)}px`
      grid.style.overflowY = "hidden"
    } else {
      const g = gridLayout(views.length, grid.clientWidth - 12, grid.clientHeight - 12)
      const colW = Math.max(60, Math.floor((grid.clientWidth - PAD - (g.cols - 1) * GAP) / g.cols))
      grid.style.gridTemplateColumns = `repeat(${g.cols}, ${colW}px)`
      grid.style.gridAutoRows = `${Math.floor(g.cellH)}px`
      grid.style.overflowY = g.scroll ? "auto" : "hidden"
    }
    invalidate()
  }

  // ---------------- drawing: shared pieces ----------------
  const dprNow = (): number => Math.max(1, window.devicePixelRatio || 1)
  const strokePoly = (g: CanvasRenderingContext2D, p: Poly, X: (v: number) => number, Y: (v: number) => number): void => {
    g.beginPath()
    let pen = false
    for (let i = 0; i < p.n; i++) {
      if (Number.isNaN(p.y[i])) { pen = false; continue }
      if (pen) g.lineTo(px(X(p.x[i])), px(Y(p.y[i])))
      else g.moveTo(px(X(p.x[i])), px(Y(p.y[i])))
      pen = true
    }
    g.stroke()
  }
  const styleLine = (g: CanvasRenderingContext2D, role: string, color: string): void => {
    g.strokeStyle = color
    g.lineJoin = "round"
    g.lineWidth = role === "flown" ? 1.6 : role === "cmd" ? 1.3 : 1.1
    g.setLineDash(role === "plan" ? [5, 3] : role === "hard" ? [4, 4] : [])
  }
  const dot = (g: CanvasRenderingContext2D, x: number, y: number, r: number, fill: string, stroke = "#ffffff"): void => {
    g.beginPath(); g.arc(x, y, r + 1.4, 0, Math.PI * 2); g.fillStyle = "rgba(6,12,20,0.6)"; g.fill()
    g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fillStyle = fill; g.fill()
    g.strokeStyle = stroke; g.lineWidth = 1.2; g.setLineDash([]); g.stroke()
  }
  const tag = (g: CanvasRenderingContext2D, text: string, x: number, y: number, align: CanvasTextAlign, fill: string, back = "rgba(6,12,20,0.88)"): void => {
    g.font = FONT
    const w = g.measureText(text).width + 8
    const left = align === "center" ? x - w / 2 : align === "right" ? x - w : x
    g.fillStyle = back
    g.fillRect(left, y - 7, w, 14)
    g.fillStyle = fill; g.textAlign = "left"; g.textBaseline = "middle"
    g.fillText(text, left + 4, y)
  }

  /** legend rows above the plot; the widths are reserved for a value, so a moving cursor never reflows them */
  const placeLegend = (g: CanvasRenderingContext2D, items: LegendItem[], width: number): number => {
    g.font = FONT
    let x = ML, row = 0
    for (const it of items) {
      const w = 18 + g.measureText(it.value ? `${it.label} ${it.reserve}` : it.label).width + 10
      if (x > ML && x + w > width - 4) { x = ML; row++ }
      it.x = x; it.row = row
      x += w
    }
    return items.length ? row + 1 : 0
  }
  const drawLegend = (g: CanvasRenderingContext2D, items: readonly LegendItem[], t: number): void => {
    g.font = FONT
    g.textAlign = "left"; g.textBaseline = "middle"
    let row = -1, end = 0
    for (const it of items) {
      const y = 4 + it.row * LEGEND_ROW + LEGEND_ROW / 2
      // a value wider than the reserved room (megametres in the full range) pushes its neighbours right
      const x = it.row === row ? Math.max(it.x, end + 10) : it.x
      if (it.swatch === "fill") {
        g.fillStyle = COLORS.band; g.fillRect(x, y - 4, 13, 8)
        g.strokeStyle = COLORS.bandEdge; g.lineWidth = 1; g.setLineDash([]); g.strokeRect(x + 0.5, y - 3.5, 12, 7)
      } else {
        g.strokeStyle = it.color; g.lineWidth = 1.6
        g.setLineDash(it.swatch === "dash" ? [4, 2.5] : [])
        g.beginPath(); g.moveTo(x, y); g.lineTo(x + 13, y); g.stroke()
        g.setLineDash([])
      }
      const text = it.value ? `${it.label} ${it.value(t)}` : it.label
      g.fillStyle = it.color
      g.fillText(text, x + 17, y)
      row = it.row; end = x + 17 + g.measureText(text).width
    }
  }

  const sizeCanvas = (v: PanelView): boolean => {
    const dpr = dprNow()
    const w = v.plot.clientWidth, h = v.plot.clientHeight
    if (w < 40 || h < 40) return false
    const cw = Math.round(w * dpr), ch = Math.round(h * dpr)
    if (v.canvas.width !== cw || v.canvas.height !== ch) { v.canvas.width = cw; v.canvas.height = ch }
    if (v.base.width !== cw || v.base.height !== ch) { v.base.width = cw; v.base.height = ch }
    // the element is exactly as large as its backing store (one canvas pixel = one device pixel):
    // a CSS size of 100 % can differ from it by a fraction of a pixel, which the browser resamples
    const sw = `${cw / dpr}px`, sh = `${ch / dpr}px`
    if (v.canvas.style.width !== sw) v.canvas.style.width = sw
    if (v.canvas.style.height !== sh) v.canvas.style.height = sh
    v.w = w; v.h = h
    return true
  }

  /** frame, grid and tick labels of a plot area; returns the tick steps */
  const axes = (g: CanvasRenderingContext2D, v: PanelView, top: number, x: readonly [number, number], y: readonly [number, number], timeAxis: boolean): Geo => {
    const pw = v.w - ML - MR, ph = v.h - top - MB
    const X = (a: number): number => ML + ((a - x[0]) / (x[1] - x[0])) * pw
    const Y = (a: number): number => top + ph - ((a - y[0]) / (y[1] - y[0])) * ph
    g.fillStyle = PLOT; g.fillRect(ML, top, pw, ph)
    g.font = FONT; g.lineWidth = 1; g.setLineDash([])
    const yt = niceTicks(y[0], y[1], Math.max(2, Math.floor(ph / 26)))
    g.textAlign = "right"; g.textBaseline = "middle"
    for (const a of yt.ticks) {
      const yy = Math.round(Y(a)) + 0.5
      g.strokeStyle = a === 0 ? ZERO : GRID
      g.beginPath(); g.moveTo(ML, yy); g.lineTo(ML + pw, yy); g.stroke()
      g.fillStyle = TEXT
      g.fillText(tickLabel(a, yt.step), ML - 5, yy)
    }
    const xt = timeAxis ? timeTicks(x[0], x[1], Math.max(2, Math.floor(pw / 62))) : niceTicks(x[0], x[1], Math.max(2, Math.floor(pw / 62)))
    g.textAlign = "center"; g.textBaseline = "top"
    for (const a of xt.ticks) {
      const xx = Math.round(X(a)) + 0.5
      g.strokeStyle = !timeAxis && a === 0 ? ZERO : GRID
      g.beginPath(); g.moveTo(xx, top); g.lineTo(xx, top + ph); g.stroke()
      g.fillStyle = TEXT
      g.fillText(timeAxis ? formatClock(a, xt.step) : tickLabel(a, xt.step), xx, top + ph + 4)
    }
    g.strokeStyle = FRAME
    g.strokeRect(ML + 0.5, top + 0.5, pw - 1, ph - 1)
    return { left: ML, top, pw, ph, x0: x[0], x1: x[1], y0: y[0], y1: y[1], xStep: xt.step, yStep: yt.step }
  }
  const mapX = (geo: Geo) => (a: number): number => geo.left + ((a - geo.x0) / (geo.x1 - geo.x0)) * geo.pw
  const mapY = (geo: Geo) => (a: number): number => geo.top + geo.ph - ((a - geo.y0) / (geo.y1 - geo.y0)) * geo.ph
  const clip = (g: CanvasRenderingContext2D, geo: Geo): void => { g.save(); g.beginPath(); g.rect(geo.left, geo.top, geo.pw, geo.ph); g.clip() }
  const hint = (g: CanvasRenderingContext2D, geo: Geo, text: string): void => {
    g.font = FONT; g.textAlign = "center"; g.textBaseline = "middle"
    const w = g.measureText(text).width + 12
    g.fillStyle = "rgba(6,12,20,0.75)"; g.fillRect(geo.left + geo.pw / 2 - w / 2, geo.top + geo.ph - 22, w, 16)
    g.fillStyle = "#c9a35c"; g.fillText(text, geo.left + geo.pw / 2, geo.top + geo.ph - 14)
  }

  // ---------------- drawing: time panels ----------------
  const timeLegend = (p: TimePanel): LegendItem[] => {
    const at = (ys: Float64Array) => (t: number): number => sampleAt(set.t, ys, set.n, t)
    const items: LegendItem[] = []
    const add = (label: string, unit: string, color: string, swatch: LegendItem["swatch"], value: ((t: number) => number) | null): void => {
      items.push({ label, color, swatch, value: value && ((t) => withUnit(formatValue(value(t)), unit)), reserve: withUnit("−000.00", unit), x: 0, row: 0 })
    }
    // plan first, then command, flown, band, limits: the reading order of the colour convention
    for (const role of ["plan", "cmd", "flown"] as const) {
      for (const l of p.lines) if (l.role === role) add(l.label, l.unit, l.color, role === "plan" ? "dash" : "solid", at(l.y))
    }
    if (p.band) add(p.band.label, p.band.unit, COLORS.bandText, "fill", at(p.band.half))
    const hard = p.lines.filter((l) => l.role === "hard")
    if (hard.length === 2 && p.band) {
      // the lateral limit pair plan ± tubeHard: one entry, its half-width
      const hi = hard[0].y, lo = hard[1].y
      add("limit ±", p.unit, COLORS.hard, "dash", (t) => (at(hi)(t) - at(lo)(t)) / 2)
    } else for (const l of hard) add(l.label, l.unit, l.color, "dash", at(l.y))
    if (p.limits?.length) add(`limit ±${Math.max(...p.limits.map(Math.abs))}${p.unit}`, "", COLORS.hard, "dash", null)
    return items
  }

  const buildTime = (v: PanelView, p: TimePanel): void => {
    const g = v.base.getContext("2d")!
    const dpr = dprNow()
    g.setTransform(dpr, 0, 0, dpr, 0, 0)
    g.fillStyle = BG; g.fillRect(0, 0, v.w, v.h)
    v.legend = timeLegend(p)
    const top = 6 + placeLegend(g, v.legend, v.w) * LEGEND_ROW
    const vis = visibleRange(set.t, set.n, win)
    const wide = widen(vis, set.n)
    const fit: IndexRange = vis.i1 >= vis.i0 ? vis : wide
    const geo = axes(g, v, top, [win.t0, win.t1], timePanelRange(p, fit, fitLimits), true)
    v.geo = geo
    const X = mapX(geo), Y = mapY(geo)
    const cols = Math.max(1, Math.round(geo.pw))
    const dec = (ys: Float64Array): Poly => decimate(set.t, ys, wide.i0, wide.i1, win.t0, win.t1, cols)

    // phase boundaries: thin vertical lines with the phase name at the top (the phase already running
    // at the left edge is named there)
    if (rec) {
      const ps = rec.meta.phaseStarts
      g.font = SMALL; g.textBaseline = "top"; g.textAlign = "left"
      let free = geo.left
      ps.forEach((s, i) => {
        const end = i + 1 < ps.length ? ps[i + 1].t : Infinity
        if (end <= win.t0 || s.t > win.t1) return
        const inside = s.t >= win.t0
        const x = inside ? Math.round(X(s.t)) + 0.5 : geo.left
        if (inside) {
          g.strokeStyle = PHASE; g.lineWidth = 1; g.setLineDash([])
          g.beginPath(); g.moveTo(x, geo.top); g.lineTo(x, geo.top + geo.ph); g.stroke()
        }
        const name = phaseLabel(s.phase)
        const w = g.measureText(name).width
        const tx = Math.round(Math.max(x + 3, free)) // text on whole pixels (the line itself sits on x + 0.5)
        const room = Math.min(end === Infinity ? Infinity : X(end), geo.left + geo.pw) - tx
        if (w + 4 <= room) { g.fillStyle = PHASE_TEXT; g.fillText(name, tx, geo.top + 3); free = tx + w + 6 }
      })
    }

    clip(g, geo)
    if (p.band) {
      const lo = dec(p.band.lo), hi = dec(p.band.hi)
      const rl = polyRuns(lo), rh = polyRuns(hi)
      g.fillStyle = COLORS.band
      for (let k = 0; k < Math.min(rl.length, rh.length); k++) {
        g.beginPath()
        for (let i = rh[k][0]; i < rh[k][1]; i++) { if (i === rh[k][0]) g.moveTo(px(X(hi.x[i])), px(Y(hi.y[i]))); else g.lineTo(px(X(hi.x[i])), px(Y(hi.y[i]))) }
        for (let i = rl[k][1] - 1; i >= rl[k][0]; i--) g.lineTo(px(X(lo.x[i])), px(Y(lo.y[i])))
        g.closePath(); g.fill()
      }
      g.strokeStyle = COLORS.bandEdge; g.lineWidth = 1; g.setLineDash([])
      strokePoly(g, lo, X, Y); strokePoly(g, hi, X, Y)
    }
    if (p.limits) {
      styleLine(g, "hard", COLORS.hard)
      for (const lim of p.limits) { const y = Math.round(Y(lim)) + 0.5; g.beginPath(); g.moveTo(geo.left, y); g.lineTo(geo.left + geo.pw, y); g.stroke() }
    }
    // limits below, then command, flown, and the dashed plan on top (visible over a perfectly tracked line)
    for (const role of ["hard", "cmd", "flown", "plan"] as const) {
      for (const l of p.lines) {
        if (l.role !== role) continue
        styleLine(g, role, l.color)
        strokePoly(g, dec(l.y), X, Y)
      }
    }
    g.setLineDash([])
    g.restore()
    if (p.planned && !hasPlan(set.on, fit)) hint(g, geo, "no plan recorded")

    const st = p.track ? trackingStats(set.t, p.track.flown, p.track.plan, fit) : null
    v.stats.textContent = st && p.track ? `RMS ${withUnit(formatValue(st.rms), p.track.unit)} · max ${withUnit(formatValue(st.max), p.track.unit)}` : ""
    v.stats.title = st ? `flown − plan over the visible window where a plan exists (time-weighted RMS, ${st.n} samples); largest at T+${formatClock(st.tMax, 0.1)}` : ""
  }

  const overlayTime = (v: PanelView, p: TimePanel, g: CanvasRenderingContext2D): void => {
    const geo = v.geo
    if (!geo) return
    const X = mapX(geo), Y = mapY(geo)
    const inWin = (t: number): boolean => t >= geo.x0 && t <= geo.x1
    if (drag && drag.mode === "brush" && drag.moved) {
      const a = Math.max(geo.left, Math.min(drag.x0, drag.x)), b = Math.min(geo.left + geo.pw, Math.max(drag.x0, drag.x))
      g.fillStyle = "rgba(79,230,255,0.14)"; g.fillRect(a, geo.top, b - a, geo.ph)
      g.strokeStyle = "rgba(79,230,255,0.7)"; g.lineWidth = 1; g.setLineDash([]); g.strokeRect(a + 0.5, geo.top + 0.5, b - a - 1, geo.ph - 1)
    }
    if (inWin(playT)) {
      const x = Math.round(X(playT)) + 0.5
      g.strokeStyle = COLORS.playhead; g.lineWidth = 1.2; g.setLineDash([])
      g.beginPath(); g.moveTo(x, geo.top); g.lineTo(x, geo.top + geo.ph); g.stroke()
      g.fillStyle = COLORS.playhead
      g.beginPath(); g.moveTo(x - 4, geo.top); g.lineTo(x + 4, geo.top); g.lineTo(x, geo.top + 6); g.closePath(); g.fill()
    }
    if (hover && inWin(hover.t)) {
      const x = Math.round(X(hover.t)) + 0.5
      g.strokeStyle = CURSOR; g.lineWidth = 1; g.setLineDash([3, 3])
      g.beginPath(); g.moveTo(x, geo.top); g.lineTo(x, geo.top + geo.ph); g.stroke()
      g.setLineDash([])
      // a marker where the cursor line crosses each series
      clip(g, geo)
      for (const l of p.lines) {
        if (l.role === "hard") continue
        const y = sampleAt(set.t, l.y, set.n, hover.t)
        if (Number.isFinite(y)) dot(g, x, px(Y(y)), 2.2, l.color, "rgba(6,12,20,0.9)")
      }
      g.restore()
      if (hover.panel === v) {
        const hy = Math.min(geo.top + geo.ph, Math.max(geo.top, hover.y))
        g.strokeStyle = CURSOR; g.setLineDash([3, 3])
        g.beginPath(); g.moveTo(geo.left, Math.round(hy) + 0.5); g.lineTo(geo.left + geo.pw, Math.round(hy) + 0.5); g.stroke()
        g.setLineDash([])
        const val = geo.y0 + ((geo.top + geo.ph - hy) / geo.ph) * (geo.y1 - geo.y0)
        tag(g, tickLabel(val, geo.yStep / 10), geo.left - 2, hy, "right", "#ffffff", "rgba(40,56,74,0.96)")
        tag(g, formatClock(hover.t, Math.min(0.5, geo.xStep / 10)), Math.min(geo.left + geo.pw - 24, Math.max(geo.left + 24, x)), geo.top + geo.ph + 9, "center", "#ffffff", "rgba(40,56,74,0.96)")
      }
    }
    drawLegend(g, v.legend, hover ? hover.t : playT)
  }

  // ---------------- drawing: x–y panels ----------------
  const buildXY = (v: PanelView, p: XYPanel): void => {
    const g = v.base.getContext("2d")!
    const dpr = dprNow()
    g.setTransform(dpr, 0, 0, dpr, 0, 0)
    g.fillStyle = BG; g.fillRect(0, 0, v.w, v.h)
    const pair = (c: XYPanel["curves"][number]) => (t: number): string => {
      const a = sampleAt(set.t, c.x, set.n, t), b = sampleAt(set.t, c.y, set.n, t)
      return Number.isFinite(a) && Number.isFinite(b) ? `${formatValue(a)}, ${formatValue(b)} ${p.unit}` : "—"
    }
    v.legend = [...p.curves].reverse().map((c): LegendItem => ({ label: c.label, color: c.color, swatch: c.role === "plan" ? "dash" : "solid", value: pair(c), reserve: `−000.00, −000.00 ${p.unit}`, x: 0, row: 0 }))
    const top = 6 + placeLegend(g, v.legend, v.w) * LEGEND_ROW
    const vis = visibleRange(set.t, set.n, win)
    const fit: IndexRange = vis.i1 >= vis.i0 ? vis : widen(vis, set.n)
    const pw = v.w - ML - MR, ph = v.h - top - MB
    const raw = xyPanelRange(p, fit)
    const r = p.equal ? equalAspect(raw.x, raw.y, pw, ph) : raw
    const geo = axes(g, v, top, r.x, r.y, false)
    v.geo = geo
    const X = mapX(geo), Y = mapY(geo)
    const kx = geo.pw / (geo.x1 - geo.x0), ky = geo.ph / (geo.y1 - geo.y0)
    // axis names inside the plot corners (the tick rows stay free)
    g.font = SMALL; g.fillStyle = PHASE_TEXT
    g.textAlign = "right"; g.textBaseline = "bottom"; g.fillText(`${p.xLabel} [${p.unit}] →`, geo.left + geo.pw - 4, geo.top + geo.ph - 3)
    g.textAlign = "left"; g.textBaseline = "top"; g.fillText(`↑ ${p.yLabel} [${p.unit}]`, geo.left + 4, geo.top + 3)
    clip(g, geo)
    for (const role of ["flown", "plan"] as const) {
      for (const c of p.curves) {
        if (c.role !== role) continue
        styleLine(g, role, c.color)
        strokePoly(g, thinPath(c.x, c.y, fit.i0, fit.i1, kx, ky), X, Y)
      }
    }
    g.setLineDash([])
    if (p.catchPoint) {
      const cx = X(0), cy = Y(0)
      g.strokeStyle = "#5af0a0"; g.lineWidth = 1.2
      g.beginPath(); g.moveTo(cx - 7, cy); g.lineTo(cx + 7, cy); g.moveTo(cx, cy - 7); g.lineTo(cx, cy + 7); g.stroke()
      g.beginPath(); g.arc(cx, cy, 3, 0, Math.PI * 2); g.fillStyle = "#5af0a0"; g.fill()
      g.font = SMALL; g.fillStyle = COLORS.bandText; g.textAlign = "left"; g.textBaseline = "bottom"
      g.fillText("catch", cx + 6, cy - 4)
    }
    g.restore()
    if (p.planned && !hasPlan(set.on, fit)) hint(g, geo, "no plan recorded")
    v.stats.textContent = ""
  }

  const overlayXY = (v: PanelView, p: XYPanel, g: CanvasRenderingContext2D): void => {
    const geo = v.geo
    if (!geo) return
    const X = mapX(geo), Y = mapY(geo)
    const kx = geo.pw / (geo.x1 - geo.x0), ky = geo.ph / (geo.y1 - geo.y0)
    const at = (ys: Float64Array, t: number): number => sampleAt(set.t, ys, set.n, t)
    clip(g, geo)
    if (p.tube) {
      // the recoverable circle (and the hard limit) around the plan point at the playhead
      const cx = at(p.tube.x, playT), cy = at(p.tube.y, playT), r = at(p.tube.r, playT), rh = at(p.tube.rHard, playT)
      if (Number.isFinite(cx) && Number.isFinite(cy)) {
        if (r > 0) {
          g.beginPath(); g.ellipse(px(X(cx)), px(Y(cy)), Math.min(1e5, r * kx), Math.min(1e5, r * ky), 0, 0, Math.PI * 2)
          g.fillStyle = COLORS.band; g.fill()
          g.strokeStyle = COLORS.bandEdge; g.lineWidth = 1.2; g.setLineDash([]); g.stroke()
        }
        if (rh > 0) {
          // the tower's hard limit is a corridor across the rails (between the arms), not a circle:
          // two lines at the arm-pair centre ± the half-width, drawn over the plot's height
          const c0 = at(p.tube.cHard, playT)
          if (Number.isFinite(c0)) {
            styleLine(g, "hard", COLORS.hard)
            for (const sg of [-1, 1]) { g.beginPath(); g.moveTo(px(X(c0 + sg * rh)), px(geo.top)); g.lineTo(px(X(c0 + sg * rh)), px(geo.top + geo.ph)); g.stroke() }
            g.setLineDash([])
          }
        }
      }
    }
    const mark = (t: number, playhead: boolean): void => {
      for (const c of p.curves) {
        const x = at(c.x, t), y = at(c.y, t)
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue
        if (c.role === "plan") dot(g, px(X(x)), px(Y(y)), 2.4, playhead ? COLORS.plan : BG, playhead ? "rgba(6,12,20,0.9)" : COLORS.plan)
        else dot(g, px(X(x)), px(Y(y)), playhead ? 3.2 : 2.6, playhead ? COLORS.playhead : BG, playhead ? "#ffffff" : c.color)
      }
    }
    if (hover) mark(hover.t, false)
    mark(playT, true)
    g.restore()
    drawLegend(g, v.legend, hover ? hover.t : playT)
  }

  // ---------------- frame ----------------
  function draw(): void {
    if (!active) return
    for (const v of views) {
      if (maxId !== null && v.def.id !== maxId) continue
      if (!sizeCanvas(v)) continue
      const g = v.canvas.getContext("2d")!
      if (v.dirty) {
        if (v.def.kind === "time") buildTime(v, v.def)
        else buildXY(v, v.def)
        v.dirty = false
      }
      g.setTransform(1, 0, 0, 1, 0, 0)
      g.clearRect(0, 0, v.canvas.width, v.canvas.height)
      g.drawImage(v.base, 0, 0)
      const dpr = dprNow()
      g.setTransform(dpr, 0, 0, dpr, 0, 0)
      if (v.def.kind === "time") overlayTime(v, v.def, g)
      else overlayXY(v, v.def, g)
    }
    readout.textContent = rec
      ? `window T+${formatClock(win.t0, 0.1)} – T+${formatClock(win.t1, 0.1)} (${formatValue(win.t1 - win.t0)} s) · ${hover ? "cursor" : "playhead"} T+${formatClock(hover ? hover.t : playT, 0.01)}`
      : ""
  }

  // ---------------- interaction ----------------
  /** time under a canvas x for a time panel; for an x–y panel the time of the nearest flown sample */
  const timeAt = (v: PanelView, x: number, y: number): number | null => {
    const geo = v.geo
    if (!geo || set.n === 0) return null
    if (v.def.kind === "time") return geo.x0 + (Math.min(geo.pw, Math.max(0, x - geo.left)) / geo.pw) * (geo.x1 - geo.x0)
    const c = v.def.curves.find((k) => k.role === "flown")
    if (!c) return null
    const kx = geo.pw / (geo.x1 - geo.x0), ky = geo.ph / (geo.y1 - geo.y0)
    const dx = geo.x0 + (x - geo.left) / kx, dy = geo.y0 + (geo.top + geo.ph - y) / ky
    const near = nearestSample(c.x, c.y, visibleRange(set.t, set.n, win), dx, dy, kx, ky)
    return near.i >= 0 && near.dist < 40 ? set.t[near.i] : null
  }
  const local = (v: PanelView, e: MouseEvent): { x: number; y: number } => {
    const r = v.canvas.getBoundingClientRect()
    return { x: e.clientX - r.left, y: e.clientY - r.top }
  }

  function bind(v: PanelView): void {
    const c = v.canvas
    c.addEventListener("pointerdown", (e) => {
      if (e.button !== 0 || !rec) return
      const p = local(v, e)
      c.setPointerCapture(e.pointerId)
      drag = { panel: v, mode: e.shiftKey && v.def.kind === "time" ? "brush" : "pan", x0: p.x, x: p.x, win, moved: false }
    })
    c.addEventListener("pointermove", (e) => {
      if (!rec) return
      const p = local(v, e)
      if (drag && drag.panel === v) {
        drag.x = p.x
        if (Math.abs(p.x - drag.x0) > 3) drag.moved = true
        if (drag.moved && drag.mode === "pan" && v.def.kind === "time" && v.geo) {
          setWindow(panWindow(drag.win, (-(p.x - drag.x0) / v.geo.pw) * (drag.win.t1 - drag.win.t0), bounds))
          c.style.cursor = "grabbing"
        }
      }
      const t = drag && drag.mode === "pan" && drag.moved ? null : timeAt(v, p.x, p.y)
      hover = t === null ? null : { panel: v, t, y: p.y }
      schedule()
    })
    const release = (e: PointerEvent): void => {
      if (!drag || drag.panel !== v) return
      const d = drag
      drag = null
      c.style.cursor = ""
      if (c.hasPointerCapture(e.pointerId)) c.releasePointerCapture(e.pointerId)
      dragged = d.moved || e.type === "pointercancel"
      if (e.type === "pointerup" && d.mode === "brush" && d.moved && v.geo) {
        const p = local(v, e)
        const a = timeAt(v, Math.min(d.x0, p.x), p.y), b = timeAt(v, Math.max(d.x0, p.x), p.y)
        if (a !== null && b !== null && b > a) setWindow({ t0: a, t1: b })
      }
      schedule()
    }
    c.addEventListener("pointerup", release)
    c.addEventListener("pointercancel", release)
    c.addEventListener("click", (e) => {
      // a plain click (not the end of a drag, not the second click of a double-click): seek the
      // playback, exactly like scrubbing the timeline
      if (!rec || dragged || e.detail > 1 || !opts.seek) return
      const p = local(v, e)
      const t = timeAt(v, p.x, p.y)
      if (t === null) return
      lastSeek = { from: playT, to: t }
      opts.seek(t)
    })
    c.addEventListener("pointerleave", () => { if (!drag && hover?.panel === v) { hover = null; schedule() } })
    c.addEventListener("dblclick", () => {
      if (!rec) return
      // the first click of the pair already moved the playhead: put it back (playback may have run on)
      if (lastSeek && opts.seek) opts.seek(playT - (lastSeek.to - lastSeek.from))
      lastSeek = null
      setWindow(home)
    })
    c.addEventListener("wheel", (e) => {
      if (!rec || !v.geo) return
      e.preventDefault()
      const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 400 : 1
      const p = local(v, e)
      if (Math.abs(e.deltaX) > Math.abs(e.deltaY)) {
        // sideways scroll (trackpad): pan
        setWindow(panWindow(win, ((e.deltaX * unit) / v.geo.pw) * (win.t1 - win.t0), bounds))
      } else {
        const anchor = timeAt(v, p.x, p.y) ?? (hover ? hover.t : (win.t0 + win.t1) / 2)
        setWindow(zoomWindow(win, anchor, Math.exp(Math.max(-1, Math.min(1, e.deltaY * unit * 0.0016))), bounds))
      }
      const t = timeAt(v, p.x, p.y)
      hover = t === null ? null : { panel: v, t, y: p.y }
      schedule()
    }, { passive: false })
  }

  window.addEventListener("keydown", (e) => {
    if (active && e.key === "Escape" && maxId !== null) { maxId = null; layout() }
  })
  window.addEventListener("resize", () => { if (active) layout() })
  if (typeof ResizeObserver !== "undefined") {
    // the grid changes size without a window resize too (first show, maximise, results panel)
    const ro = new ResizeObserver(() => { if (active) layout() })
    ro.observe(grid)
  }

  // ---------------- public ----------------
  return {
    setRecording: (r) => {
      rec = r
      hover = null; drag = null; lastSeek = null
      set = buildPanels(r ? recordingSource(r, CH) : EMPTY_SOURCE)
      views.forEach((v, i) => { v.def = set.panels[i]; v.geo = null })
      if (r && r.n > 0) {
        bounds = { t0: set.t[0], t1: set.t[r.n - 1] }
        home = defaultWindow(r.meta.phaseStarts, bounds.t0, bounds.t1)
        win = clampWindow(home, bounds)
      }
      invalidate()
    },
    setTime: (t) => {
      if (t === playT) return
      playT = t
      schedule()
    },
    setActive: (on) => {
      active = on
      root.classList.toggle("hidden", !on)
      if (on) layout()
      else if (frame) { cancelAnimationFrame(frame); frame = 0 }
    },
  }
}
