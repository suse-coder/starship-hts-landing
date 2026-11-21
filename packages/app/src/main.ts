/**
 * Browser shell (static, no server code): run the whole mission in a Web Worker on load, show a
 * live progress sketch, then play the recording back — slider scrubbing, play/speed, phases,
 * telemetry, charts, events, exploded view, three cameras.
 */
import { DEFAULT_DESIGN } from "@ship/craft"
import { CH, valueAt, type ChannelKey, type FromWorker, type Recording } from "@ship/sim"
import { createView, marginColor, rainbow, type CameraMode, type TreeNode } from "@ship/view"
import { steelYieldRetention, THERMAL_LIMITS } from "@ship/heat"
import { renderSurvival, survival } from "./survival"
import { isSlowed, playbackRate } from "./playback"
import { SIM_VERSION } from "virtual:sim-version"
import { clearCached, fnv1a, loadCached, storeCached } from "./simCache"

const $ = <T extends HTMLElement>(id: string): T => document.getElementById(id) as T
const ICON = {
  "start": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><rect x=\"2\" y=\"2\" width=\"2\" height=\"10\"/><path d=\"M12 2v10L5 7z\"/></svg>",
  "prev": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><path d=\"M7 2v10L1 7zM13 2v10L7 7z\"/></svg>",
  "play": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><path d=\"M3 2v10l9-5z\"/></svg>",
  "pause": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><rect x=\"3\" y=\"2\" width=\"3\" height=\"10\"/><rect x=\"8\" y=\"2\" width=\"3\" height=\"10\"/></svg>",
  "stop": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><rect x=\"3\" y=\"3\" width=\"8\" height=\"8\"/></svg>",
  "next": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><path d=\"M1 2v10l6-5zM7 2v10l6-5z\"/></svg>",
  "end": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><path d=\"M2 2v10l7-5z\"/><rect x=\"10\" y=\"2\" width=\"2\" height=\"10\"/></svg>",
  "home": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\"><path d=\"M7 1l6 5h-2v6H8V8H6v4H3V6H1z\"/></svg>",
  "fit": "<svg width=\"14\" height=\"14\" viewBox=\"0 0 14 14\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"1.5\"><path d=\"M1 5V1h4M9 1h4v4M13 9v4H9M5 13H1V9\"/></svg>",
  "eye": "<svg width=\"14\" height=\"10\" viewBox=\"0 0 14 10\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"1.3\"><path d=\"M1 5c2-3 4-4 6-4s4 1 6 4c-2 3-4 4-6 4S3 8 1 5z\"/><circle cx=\"7\" cy=\"5\" r=\"1.8\" fill=\"currentColor\"/></svg>"
} as const
for (const [id, k] of [["b-start", "start"], ["b-prev", "prev"], ["b-play", "play"], ["b-stop", "stop"], ["b-next", "next"], ["b-end", "end"], ["n-home", "home"], ["n-fit", "fit"]] as const) $(id).innerHTML = ICON[k]
const altGraph = $<HTMLCanvasElement>("altgraph")
const canvas = $<HTMLCanvasElement>("view")
const view = createView(canvas, $("labels"), $<HTMLCanvasElement>("viewcube"))
const PHASE_COLORS = ["#8899aa", "#ff9a3c", "#8899aa", "#ff5a3c", "#ffd23c", "#ff3cbe", "#b07cff", "#3cc8ff", "#ffffff", "#7cff9a", "#7cffd8", "#3cff6e", "#ff3c3c"]
const PHASE_LABEL: Record<string, string> = {
  deorbitAlign: "Orbit", deorbitBurn: "Deorbit burn", entryAlign: "Coast + coil ramp", entry: "MHD glide", loft: "Loft", pitchUp: "Pitch-up",
  highAlpha: "High-α glide", bellyFlop: "Belly flop", flip: "Flip", landing: "Landing burn", terminal: "Terminal", caught: "Caught", lost: "Lost",
}
const fmtT = (t: number): string => {
  const s = Math.max(0, t), m = Math.floor(s / 60)
  return `T+${m}:${(s - m * 60).toFixed(1).padStart(4, "0")}`
}

// ---------------- loading: worker + live sketch ----------------
const sketch = $<HTMLCanvasElement>("sketch")
const sk = sketch.getContext("2d")!
const trace: { t: number; alt: number; phase: string }[] = []
const drawSketch = (pulse: number): void => {
  // plot area with margins for the axis labels (altitude in km on the left, mission time below)
  // backing store at device-pixel resolution, drawing in CSS pixels (crisp text)
  const dpr = Math.max(1, window.devicePixelRatio || 1)
  const W = sketch.clientWidth || 560, H = sketch.clientHeight || 200
  if (sketch.width !== Math.round(W * dpr) || sketch.height !== Math.round(H * dpr)) { sketch.width = Math.round(W * dpr); sketch.height = Math.round(H * dpr) }
  sk.setTransform(dpr, 0, 0, dpr, 0, 0)
  const L = 50, R = 12, T = 14, B = 24
  const tMax = 3200, aMax = 300e3
  const X = (t: number): number => L + (t / tMax) * (W - L - R)
  const Y = (alt: number): number => H - B - (alt / aMax) * (H - T - B)
  sk.clearRect(0, 0, W, H)
  sk.font = "11px system-ui, sans-serif"
  sk.lineWidth = 1
  sk.textBaseline = "middle"
  for (let km = 0; km <= 300; km += 100) {
    const y = Math.round(Y(km * 1e3)) + 0.5
    sk.strokeStyle = km === 0 ? "rgba(255,255,255,0.28)" : "rgba(255,255,255,0.09)"
    sk.beginPath(); sk.moveTo(L, y); sk.lineTo(W - R, y); sk.stroke()
    sk.fillStyle = "#9aa6b2"; sk.textAlign = "right"
    sk.fillText(`${km} km`, L - 8, y)
  }
  sk.textBaseline = "top"; sk.textAlign = "center"
  for (let m = 0; m * 60 <= tMax; m += 10) {
    const x = Math.round(X(m * 60)) + 0.5
    sk.strokeStyle = "rgba(255,255,255,0.09)"
    sk.beginPath(); sk.moveTo(x, T); sk.lineTo(x, H - B); sk.stroke()
    sk.fillStyle = "#9aa6b2"
    sk.fillText(`${m} min`, x, H - B + 6)
  }
  sk.lineWidth = 2
  for (let i = 1; i < trace.length; i++) {
    const p0 = trace[i - 1], p1 = trace[i]
    sk.strokeStyle = PHASE_COLORS[Object.keys(PHASE_LABEL).indexOf(p1.phase)] ?? "#4fb3ff"
    sk.beginPath(); sk.moveTo(X(p0.t), Y(p0.alt)); sk.lineTo(X(p1.t), Y(p1.alt)); sk.stroke()
  }
  const last = trace[trace.length - 1]
  if (last) {
    sk.fillStyle = `rgba(255,190,90,${0.5 + 0.5 * Math.sin(pulse)})`
    sk.beginPath(); sk.arc(X(last.t), Y(last.alt), 4 + 2 * Math.sin(pulse), 0, Math.PI * 2); sk.fill()
  }
}
let loading = true
let simTimer: ReturnType<typeof setTimeout> | undefined
const spin = (ts: number): void => { if (!loading) return; drawSketch(ts / 180); requestAnimationFrame(spin) }
requestAnimationFrame(spin)

/** cache key: physics-source hash (vite plugin) + design parameters */
const cacheKey = `${SIM_VERSION}:${fnv1a(JSON.stringify(DEFAULT_DESIGN))}`

const clearSimTimer = (): void => {
  if (simTimer !== undefined) {
    clearTimeout(simTimer)
    simTimer = undefined
  }
}

/** real failure state: stop the sketch, keep the card, show the reason and a retry (no silent stuck loading) */
const simError = (msg: string): void => {
  clearSimTimer()
  loading = false
  $("loading").classList.remove("hidden")
  $("loading").classList.add("error")
  $("pstatus").textContent = `simulation failed\n${msg}`
  $("load-retry").hidden = false
  $("pbar").style.width = "0%"
}

/** load the cached recording for this physics version, or simulate (?resim forces a new run) */
const boot = async (force: boolean): Promise<void> => {
  if (!force) {
    $("pstatus").textContent = "loading the stored simulation…"
    const cached = await loadCached(cacheKey).catch((e) => {
      console.warn("cache read failed; re-simulating", e)
      return null
    })
    if (cached) {
      loading = false
      $("loading").classList.add("hidden")
      $("loading").classList.remove("error")
      console.info(`simulation: loaded from cache (physics version ${SIM_VERSION})`)
      start(cached)
      return
    }
  }
  runSim()
}

const runSim = (): void => {
  loading = true
  trace.length = 0
  $("loading").classList.remove("hidden", "error")
  $("load-retry").hidden = true
  $("pstatus").textContent = "starting worker…"
  $("pbar").style.width = "0%"
  const worker = new Worker(new URL("./sim.worker.ts", import.meta.url), { type: "module" })
  clearSimTimer()
  simTimer = setTimeout(() => {
    worker.terminate() // a live worker would overwrite the error text with late progress
    simError("timed out after 10 minutes — the simulation worker may have crashed. Press Re-simulate.")
  }, 600_000)
  worker.onmessage = (ev: MessageEvent<FromWorker>) => {
    const m = ev.data
    if (m.kind === "progress") {
      const p = m.progress
      if (p.stage === "simulating") {
        trace.push({ t: p.t, alt: p.alt, phase: p.phase })
        $("pstatus").textContent = `${PHASE_LABEL[p.phase] ?? p.phase} · ${fmtT(p.t)} · ${(p.alt / 1000).toFixed(1)} km`
        $("pbar").style.width = `${(p.fraction * 100).toFixed(1)}%`
      } else $("pstatus").textContent = `building the world: ${p.stage}…`
    } else if (m.kind === "done") {
      clearSimTimer()
      worker.terminate()
      loading = false
      $("loading").classList.add("hidden")
      $("loading").classList.remove("error")
      $("load-retry").hidden = true
      start(m.recording)
      void storeCached(cacheKey, m.recording)
    } else {
      worker.terminate()
      simError(m.message)
    }
  }
  worker.onerror = (e) => {
    worker.terminate()
    simError(e.message || "the simulation worker crashed before it could start")
  }
  worker.postMessage({ kind: "run", design: DEFAULT_DESIGN })
}

$("load-retry").onclick = () => { void clearCached().then(() => runSim()).catch(() => runSim()) }

// ---------------- playback (enabled only after the full run) ----------------
let rec: Recording | null = null
let t = 0
let playing = false
let duration = 1
let resumeAfterAssemble = false

const playBtn = $<HTMLButtonElement>("b-play")
const setPlaying = (p: boolean): void => { playing = p; playBtn.innerHTML = p ? ICON.pause : ICON.play }
const seek = (tt: number): void => { t = Math.max(0, Math.min(duration, tt)) }
const phaseTimes = (): number[] => (rec ? rec.meta.phaseStarts.map((x) => x.t) : [0])
playBtn.onclick = () => { if (t >= duration) t = 0; setPlaying(!playing) }
$("b-stop").onclick = () => { setPlaying(false); seek(0) }
$("b-start").onclick = () => seek(0)
$("b-end").onclick = () => { setPlaying(false); seek(duration) }
$("b-prev").onclick = () => { const ts = phaseTimes().filter((x) => x < t - 0.5); seek(ts.length ? ts[ts.length - 1] : 0) }
$("b-next").onclick = () => { const n = phaseTimes().find((x) => x > t + 0.5); seek(n ?? duration) }
// drag anywhere on the timeline track to scrub
const track = $("track")
const scrubTo = (clientX: number): void => { const r = track.getBoundingClientRect(); seek(((clientX - r.left) / r.width) * duration) }
track.addEventListener("pointerdown", (e) => { track.setPointerCapture(e.pointerId); scrubTo(e.clientX) })
track.addEventListener("pointermove", (e) => { if (track.hasPointerCapture(e.pointerId)) scrubTo(e.clientX) })
// exploded view: pause → spring apart; uncheck → spring together, then resume if it was playing
const explodeBox = $<HTMLInputElement>("explode")
explodeBox.onchange = () => {
  if (explodeBox.checked) {
    resumeAfterAssemble = playing
    setPlaying(false)
    view.setExploded(true)
  } else view.setExploded(false)
}
$<HTMLSelectElement>("cam").onchange = (e) => view.setMode((e.target as HTMLSelectElement).value as CameraMode)
window.addEventListener("keydown", (e) => {
  if (e.target instanceof HTMLInputElement && e.target.type !== "checkbox") return
  if (e.target instanceof HTMLSelectElement) return
  if (e.code === "Space") { e.preventDefault(); playBtn.click() }
  if (e.key === "e" || e.key === "E") explodeBox.click()
  if (e.key === "Home") seek(0)
  if (e.key === "End") seek(duration)
  if (e.key === "ArrowRight") seek(t + (e.shiftKey ? 60 : 5))
  if (e.key === "ArrowLeft") seek(t - (e.shiftKey ? 60 : 5))
})
const resize = (): void => { view.resize(); if (rec) buildAltGraph(rec); drawCharts() }
window.addEventListener("resize", resize)
$("results-head").addEventListener("click", () => {
  const p = $("results")
  p.classList.toggle("collapsed")
  p.querySelector(".caret")!.textContent = p.classList.contains("collapsed") ? "▸" : "▾"
  drawCharts()
})
$("n-home").onclick = () => { if ($<HTMLSelectElement>("cam").value === "chase") view.setMode("chase"); view.home() }
$("n-fit").onclick = () => { if ($<HTMLSelectElement>("cam").value === "chase") view.setMode("chase"); view.fit() }
$<HTMLInputElement>("env-sky").onchange = (e) => view.setPhysicalSky((e.target as HTMLInputElement).checked)

// ---------------- browser tree ----------------
const buildTree = (): void => {
  const root = $("tree")
  root.innerHTML = ""
  const state = new Map<string, boolean>()
  const render = (nodes: readonly TreeNode[], depth: number, into: HTMLElement): void => {
    for (const n of nodes) {
      const row = document.createElement("div")
      row.className = "node"
      row.style.paddingLeft = `${4 + depth * 14}px`
      const tw = document.createElement("span"); tw.className = "tw"; tw.textContent = n.children ? (depth === 0 ? "▾" : "▸") : ""
      const eye = document.createElement("span"); eye.className = "eye"; eye.innerHTML = ICON.eye; eye.title = "Show / hide"
      const ico = document.createElement("span"); ico.className = `ico ${n.children ? "folder" : n.id.startsWith("c:") || ["hull", "pins", "engines", "coil", "pad", "earth"].includes(n.id) ? "body" : "fx"}`
      const lbl = document.createElement("span"); lbl.className = "lbl"; lbl.textContent = n.label; lbl.title = n.label
      row.append(tw, eye, ico, lbl)
      into.appendChild(row)
      if (n.on === false) { state.set(n.id, false); eye.classList.add("off"); row.classList.add("dim") }
      let kids: HTMLDivElement | null = null
      if (n.children) {
        kids = document.createElement("div")
        kids.className = `children${depth === 0 ? "" : " closed"}`
        into.appendChild(kids)
        render(n.children, depth + 1, kids)
        const toggle = (): void => { kids!.classList.toggle("closed"); tw.textContent = kids!.classList.contains("closed") ? "▸" : "▾" }
        tw.onclick = toggle
        lbl.ondblclick = toggle
      }
      eye.onclick = () => {
        const on = !(state.get(n.id) ?? true)
        state.set(n.id, on)
        view.setVisible(n.id, on)
        eye.classList.toggle("off", !on)
        row.classList.toggle("dim", !on)
        // children follow the parent
        kids?.querySelectorAll<HTMLElement>(".eye").forEach((ce) => ce.classList.toggle("off", !on))
        kids?.querySelectorAll<HTMLElement>(".node").forEach((cn) => cn.classList.toggle("dim", !on))
      }
    }
  }
  render(view.tree(), 0, root)
}

// ---------------- telemetry ----------------
type Row = readonly [string, (r: Recording, t: number) => string]
const V = (k: ChannelKey) => (r: Recording, tt: number): number => valueAt(r, k, tt)
const f = (k: ChannelKey, scale: number, digits: number, unit: string) => (r: Recording, tt: number): string => `${(V(k)(r, tt) * scale).toFixed(digits)} ${unit}`
const GROUPS: readonly (readonly [string, readonly Row[]])[] = [
  ["Flight", [
    ["phase", (r, tt) => PHASE_LABEL[(r.meta.vehicle as { phases: { id: string }[] }).phases[Math.round(V("phase")(r, tt))]?.id ?? ""] ?? ""],
    ["altitude", f("alt", 1e-3, 2, "km")], ["airspeed", f("V", 1, 0, "m/s")], ["Mach", f("mach", 1, 2, "")], ["dyn. pressure", f("qbar", 1e-3, 2, "kPa")],
    ["angle of attack", f("alpha", 180 / Math.PI, 1, "°")], ["load factor", f("g", 1, 2, "g")], ["mass", f("mass", 1e-3, 1, "t")],
  ]],
  ["Plasma · MHD coil", [
    ["stagnation T", f("T0", 1, 0, "K")], ["electron density", (r, tt) => `${V("ne")(r, tt).toExponential(1)} m⁻³`], ["σ_eff", f("sigmaEff", 1, 0, "S/m")],
    ["Hall β", f("hall", 1, 1, "")], ["Stuart Q", f("stuart", 1, 2, "")], ["B at stagnation", f("bStag", 1, 2, "T")],
    ["standoff Δ / Δ₀", (r, tt) => `${V("standoff")(r, tt).toFixed(2)} / ${V("standoff0")(r, tt).toFixed(2)} m`],
    ["heat-flux factor", (r, tt) => { const h = V("heatFactor")(r, tt); return h > 0 && h < 0.999 ? `${h.toFixed(2)} (−${((1 - h) * 100).toFixed(0)} %)` : "1.00" }],
    ["MHD force", f("mhdF", 1e-3, 0, "kN")], ["authority/demand", (r, tt) => { const a = V("mhdAuth")(r, tt); return a > 99 ? "≫ 1" : a.toFixed(2) }], ["electrode power", f("mhdP", 1e-3, 0, "kW")], ["Cs left", f("seedLeft", 1, 0, "kg")],
  ]],
  ["Thermal", [
    ["stag. heat flux", f("qStag", 1e-4, 1, "W/cm²")], ["PICA-X surface", f("picaSurf", 1, 0, "K")], ["bondline", f("bondline", 1, 0, "K")],
    ["steel max", (r, tt) => { const T = V("steel")(r, tt); return `${T.toFixed(0)} K · k ${(steelYieldRetention(T) * 100).toFixed(0)} %` }],
    ["hottest station", f("steelX", 1, 1, "m")],
    ["bondline margin", (r, tt) => `${(THERMAL_LIMITS.bondline - V("bondline")(r, tt)).toFixed(0)} K`], ["PICA recession", f("recession", 1e3, 1, "mm")],
  ]],
  ["Magnet (REBCO)", [
    ["coil current", f("coilI", 1, 0, "A")], ["cold mass", f("coilT", 1, 1, "K")], ["stored energy", f("coilE", 1e-6, 0, "MJ")], ["cryostat leak", f("heatLeak", 1, 1, "W")],
  ]],
  ["Propulsion · RCS", [
    ["N2 left", f("n2", 1, 0, "kg")], ["header propellant", f("header", 1, 0, "kg")], ["thrust", f("thrust", 1e-6, 2, "MN")],
    ["gimbal p/y", (r, tt) => `${(V("gp")(r, tt) * 57.3).toFixed(1)}° / ${(V("gy")(r, tt) * 57.3).toFixed(1)}°`],
  ]],
  ["Catch · Mechazilla", [
    ["range to tower", (r, tt) => { const d = Math.hypot(V("pe")(r, tt), V("pn")(r, tt)); return d > 2000 ? `${(d / 1000).toFixed(1)} km` : `${d.toFixed(0)} m` }],
    ["pins above rails", (r, tt) => (Math.hypot(V("pe")(r, tt), V("pn")(r, tt)) < 3000 ? `${V("pinZ")(r, tt).toFixed(1)} m` : "—")], ["arms open", f("arms", 100, 0, "%")], ["pin load", f("pinLoad", 1e-3, 0, "kN")],
    ["wind E/N", (r, tt) => `${V("windE")(r, tt).toFixed(1)} / ${V("windN")(r, tt).toFixed(1)} m/s`],
  ]],
]
const cells: HTMLSpanElement[] = []
const buildTelemetry = (): void => {
  const root = $("telemetry")
  root.innerHTML = ""
  cells.length = 0
  for (const [title, rows] of GROUPS) {
    const h = document.createElement("h3"); h.textContent = title; root.appendChild(h)
    for (const [label] of rows) {
      const row = document.createElement("div"); row.className = "row"
      const a = document.createElement("span"); a.textContent = label
      const b = document.createElement("span"); cells.push(b)
      row.append(a, b); root.appendChild(row)
    }
  }
}
const updateTelemetry = (): void => {
  if (!rec) return
  let k = 0
  for (const [, rows] of GROUPS) for (const [, fn] of rows) cells[k++].textContent = fn(rec, t)
}

// ---------------- charts ----------------
interface Series { readonly key: ChannelKey; readonly scale: number; readonly color: string; readonly label: string }
const CHARTS: readonly (readonly Series[])[] = [
  [{ key: "alt", scale: 1e-3, color: "#4fb3ff", label: "alt km" }, { key: "V", scale: 1e-3, color: "#ffd23c", label: "V km/s" }],
  [{ key: "qbar", scale: 1e-3, color: "#ff9a3c", label: "q kPa" }, { key: "g", scale: 1, color: "#ff5a8a", label: "g" }],
  [{ key: "steel", scale: 1, color: "#ff7a3a", label: "steel K" }, { key: "bondline", scale: 1, color: "#7cff9a", label: "bondline K" }, { key: "picaSurf", scale: 1, color: "#ffffff", label: "PICA K" }],
  [{ key: "mhdF", scale: 1e-3, color: "#3cc8ff", label: "MHD kN" }, { key: "coilI", scale: 0.1, color: "#b07cff", label: "coil A/10" }],
]
const chartEls: HTMLCanvasElement[] = []
const buildCharts = (): void => {
  const root = $("charts")
  root.innerHTML = ""
  chartEls.length = 0
  for (let i = 0; i < CHARTS.length; i++) { const c = document.createElement("canvas"); root.appendChild(c); chartEls.push(c) }
  drawCharts()
}
function drawCharts(): void {
  if (!rec) return
  const r = rec
  chartEls.forEach((c, ci) => {
    const w = (c.width = c.clientWidth * devicePixelRatio), h = (c.height = c.clientHeight * devicePixelRatio)
    const ctx = c.getContext("2d")!
    ctx.clearRect(0, 0, w, h)
    const step = Math.max(1, Math.floor(r.n / 600))
    CHARTS[ci].forEach((s, si) => {
      const d = r.data[CH[s.key]]
      let max = 1e-9
      for (let i = 0; i < r.n; i += step) max = Math.max(max, Math.abs(d[i] * s.scale))
      ctx.strokeStyle = s.color
      ctx.lineWidth = 1.5 * devicePixelRatio
      ctx.beginPath()
      for (let i = 0; i < r.n; i += step) {
        const x = (r.data[CH.t][i] / duration) * w, y = h - 6 - ((d[i] * s.scale) / max) * (h - 18)
        if (i === 0) ctx.moveTo(x, y)
        else ctx.lineTo(x, y)
      }
      ctx.stroke()
      ctx.fillStyle = s.color
      ctx.font = `${10 * devicePixelRatio}px sans-serif`
      ctx.fillText(`${s.label} (max ${max.toFixed(max < 10 ? 2 : 0)})`, 6 + si * 105 * devicePixelRatio, 12 * devicePixelRatio)
    })
  })
}
const drawCursor = (): void => {
  // light-weight: redraw a thin cursor over the static plot via a CSS gradient line
  chartEls.forEach((c) => { c.style.backgroundImage = `linear-gradient(90deg, transparent calc(${(t / duration) * 100}% - 1px), rgba(255,255,255,0.7) calc(${(t / duration) * 100}% - 1px), rgba(255,255,255,0.7) calc(${(t / duration) * 100}% + 1px), transparent calc(${(t / duration) * 100}% + 1px))` })
}

// ---------------- events & timeline track ----------------
const evEls: { t: number; el: HTMLDivElement }[] = []
const buildEvents = (): void => {
  const root = $("events")
  root.innerHTML = "<b style='color:#47515b'>Mission events</b>"
  evEls.length = 0
  for (const e of rec!.events) {
    const d = document.createElement("div")
    d.className = "ev"
    d.innerHTML = `<b>${fmtT(e.t)}</b>${e.msg}`
    d.onclick = () => seek(e.t)
    root.appendChild(d)
    evEls.push({ t: e.t, el: d })
  }
}
/** intervals where pred(i) holds, as [t0, t1] */
const intervals = (r: Recording, pred: (i: number) => boolean): [number, number][] => {
  const out: [number, number][] = []
  let s = -1
  for (let i = 0; i < r.n; i++) {
    const on = pred(i)
    if (on && s < 0) s = r.data[CH.t][i]
    if (!on && s >= 0) { out.push([s, r.data[CH.t][i]]); s = -1 }
  }
  if (s >= 0) out.push([s, r.data[CH.t][r.n - 1]])
  return out.filter(([x, y]) => y - x > 0.2)
}
const buildTracks = (): void => {
  const r = rec!
  const pct = (x: number): string => `${(x / duration) * 100}%`
  // ruler: minor tick each minute, label every 5 min
  const ruler = $("ruler")
  ruler.innerHTML = ""
  for (let m = 0; m * 60 <= duration; m++) {
    const i = document.createElement("i")
    i.style.left = pct(m * 60)
    if (m % 5 === 0) {
      i.className = "major"
      const lb = document.createElement("b"); lb.style.left = pct(m * 60); lb.textContent = `${m}m`
      ruler.appendChild(lb)
    }
    ruler.appendChild(i)
  }
  const rows = $("rows")
  rows.innerHTML = ""
  const row = (): HTMLDivElement => { const d = document.createElement("div"); d.className = "trow"; rows.appendChild(d); return d }
  const bar = (into: HTMLElement, t0: number, t1: number, label: string, light = false, title = label): void => {
    const d = document.createElement("div")
    d.className = `bar${light ? " light" : ""}`
    d.style.left = pct(t0); d.style.width = `max(2px, ${((t1 - t0) / duration) * 100}%)`
    d.textContent = label; d.title = title
    into.appendChild(d)
  }
  // phases
  const ph = row()
  const ps = r.meta.phaseStarts
  ps.forEach((x, i) => bar(ph, x.t, i + 1 < ps.length ? ps[i + 1].t : duration, PHASE_LABEL[x.phase] ?? x.phase, i % 2 === 1, `${PHASE_LABEL[x.phase]} · ${fmtT(x.t)}`))
  // events
  const ev = row()
  for (const e of r.events) {
    const d = document.createElement("div"); d.className = "ev"; d.style.left = pct(e.t); d.title = `${fmtT(e.t)} ${e.msg}`
    ev.appendChild(d)
  }
  // activity tracks
  const thr = row()
  intervals(r, (i) => r.data[CH.thrust][i] > 1e4).forEach(([x, y]) => bar(thr, x, y, "Raptor", false, `Raptor burn ${fmtT(x)} – ${fmtT(y)}`))
  const rcs = row()
  intervals(r, (i) => { let s = 0; for (let k = 0; k < 12; k++) s += r.data[CH[`r${k}` as ChannelKey]][i]; return s > 0.15 }).forEach(([x, y]) => bar(rcs, x, y, "", true, `cold gas ${fmtT(x)} – ${fmtT(y)}`))
  const coil = row()
  intervals(r, (i) => Math.abs(r.data[CH.coilI][i]) > 50).forEach(([x, y]) => bar(coil, x, y, "REBCO coil energised", true, `coil current ${fmtT(x)} – ${fmtT(y)}`))
  const pl = row()
  intervals(r, (i) => r.data[CH.ne][i] > 1e18).forEach(([x, y]) => bar(pl, x, y, "plasma", false, `shock-layer plasma n_e > 1e18 m⁻³ ${fmtT(x)} – ${fmtT(y)}`))
}
const phaseAt = (tt: number): string => {
  const ps = rec!.meta.phaseStarts
  let cur = ps[0].phase
  for (const p of ps) if (p.t <= tt) cur = p.phase
  return PHASE_LABEL[cur] ?? cur
}

// ---------------- altitude sparkline (transparent, top right) ----------------
let altBase: HTMLCanvasElement | null = null
let altMax = 1
const ALT_L = 40, ALT_R = 4, ALT_T = 12, ALT_B = 10
/** white text with a dark outline: readable over both the white sky and black space */
const altText = (g: CanvasRenderingContext2D, text: string, x: number, y: number, align: CanvasTextAlign): void => {
  g.font = "10px system-ui, sans-serif"
  g.textAlign = align
  g.textBaseline = "middle"
  g.lineJoin = "round"
  g.lineWidth = 3
  g.strokeStyle = "rgba(6,12,20,0.78)"
  g.strokeText(text, x, y)
  g.fillStyle = "#f7fbff"
  g.fillText(text, x, y)
}
const buildAltGraph = (r: Recording): void => {
  const dpr = Math.max(1, window.devicePixelRatio || 1)
  const w = Math.max(90, altGraph.clientWidth || 210), h = Math.max(56, altGraph.clientHeight || 104)
  altGraph.width = Math.round(w * dpr); altGraph.height = Math.round(h * dpr)
  const c = document.createElement("canvas")
  c.width = altGraph.width; c.height = altGraph.height
  const g = c.getContext("2d")!
  g.setTransform(dpr, 0, 0, dpr, 0, 0)
  altMax = Math.max(1000, r.data[CH.alt].reduce((m, a) => Math.max(m, a), 0))
  const alt = r.data[CH.alt]
  const X = (t: number): number => ALT_L + (t / Math.max(duration, 1e-6)) * (w - ALT_L - ALT_R)
  const Y = (a: number): number => h - ALT_B - Math.sqrt(Math.max(a, 0) / altMax) * (h - ALT_T - ALT_B)
  g.clearRect(0, 0, w, h)
  altText(g, `${(altMax / 1000).toFixed(0)} km`, ALT_L - 3, ALT_T + 1, "right")
  altText(g, "0", ALT_L - 3, h - ALT_B, "right")
  g.strokeStyle = "rgba(130,145,160,0.7)"; g.lineWidth = 1
  g.beginPath(); g.moveTo(ALT_L, h - ALT_B + 0.5); g.lineTo(w - ALT_R, h - ALT_B + 0.5); g.stroke()
  // altitude curve with a dark + light halo so it reads on white sky and black space
  const step = Math.max(1, Math.floor(r.n / 420))
  g.beginPath()
  for (let i = 0; i < r.n; i += step) {
    const x = X(r.data[CH.t][i]), y = Y(alt[i])
    if (i === 0) g.moveTo(x, y)
    else g.lineTo(x, y)
  }
  g.lineJoin = "round"; g.lineCap = "round"
  g.strokeStyle = "rgba(6,12,20,0.55)"; g.lineWidth = 4; g.stroke()
  g.strokeStyle = "rgba(248,252,255,0.85)"; g.lineWidth = 2.6; g.stroke()
  g.strokeStyle = "#0696d7"; g.lineWidth = 1.4; g.stroke()
  altBase = c
}
/** marker + current altitude label; called on the UI tick */
const drawAltGraph = (tt: number): void => {
  if (!rec || !altBase) return
  const dpr = Math.max(1, window.devicePixelRatio || 1)
  const w = Math.max(90, altGraph.clientWidth || 210), h = Math.max(56, altGraph.clientHeight || 104)
  const g = altGraph.getContext("2d")!
  g.setTransform(1, 0, 0, 1, 0, 0)
  g.clearRect(0, 0, altGraph.width, altGraph.height)
  g.drawImage(altBase, 0, 0)
  g.setTransform(dpr, 0, 0, dpr, 0, 0)
  const X = (t: number): number => ALT_L + (t / Math.max(duration, 1e-6)) * (w - ALT_L - ALT_R)
  const Y = (a: number): number => h - ALT_B - Math.sqrt(Math.max(a, 0) / altMax) * (h - ALT_T - ALT_B)
  const alt = valueAt(rec, "alt", tt)
  const x = X(Math.min(tt, duration)), y = Y(alt)
  g.strokeStyle = "rgba(110,125,140,0.85)"; g.lineWidth = 1
  g.beginPath(); g.moveTo(Math.round(x) + 0.5, ALT_T); g.lineTo(Math.round(x) + 0.5, h - ALT_B); g.stroke()
  g.beginPath(); g.arc(x, y, 4.6, 0, Math.PI * 2); g.fillStyle = "rgba(6,12,20,0.55)"; g.fill()
  g.beginPath(); g.arc(x, y, 3.2, 0, Math.PI * 2); g.fillStyle = "#ff9a3c"; g.fill()
  g.strokeStyle = "#ffffff"; g.lineWidth = 1.5; g.stroke()
  altText(g, alt > 1000 ? `${(alt / 1000).toFixed(1)} km` : `${alt.toFixed(0)} m`, w - ALT_R - 2, ALT_T + 1, "right")
}

const start = (r: Recording): void => {
  rec = r
  duration = r.data[CH.t][r.n - 1]
  t = 0
  view.setRecording(r)
  view.resize()
  document.body.classList.remove("locked")
  buildTree()
  buildTelemetry()
  buildCharts()
  buildEvents()
  buildTracks()
  buildAltGraph(r)
  const sv = $("survival")
  sv.innerHTML = ""
  const svr = survival(r)
  sv.appendChild(renderSurvival(svr, (tt) => { setPlaying(false); seek(tt) }))
  const oc = $("outcome")
  // R8/R10 (T08 P1-3): a catch with the tile surface or the PICA bondline outside its limit must not
  // read as a clean CAUGHT. The outcome stays caught/lost; the verdict is appended and shown in the tooltip.
  const outside = svr.tileSurfMargin < 0 || svr.bondMargin < 0
  oc.textContent = (r.meta.outcome === "caught" ? "CAUGHT" : `${String(r.meta.outcome).toUpperCase()}: ${r.meta.summary.reason ?? ""}`) + (outside ? " · OUTSIDE TPS LIMITS" : "")
  oc.title = outside ? `tile surface margin ${svr.tileSurfMargin.toFixed(0)} K, bondline margin ${svr.bondMargin.toFixed(0)} K` : ""
  oc.className = `badge ${r.meta.outcome}`
  console.info(`simulation: ${(r.meta.wallClockMs / 1000).toFixed(1)} s in the worker, ${r.meta.steps} steps, ${r.n} samples`)
  setPlaying(true)
}

// ---------------- ANSYS-style legends (discrete bands, value labels) ----------------
const legendHtml = (title: string, values: readonly number[], color: (v: number) => string, fmt: (v: number) => string, notes: Readonly<Record<number, string>> = {}): string =>
  `<div class="legend"><div class="lt">${title}</div>${values.slice().reverse().map((v, i, a) => {
    const next = a[i + 1]
    const mid = next === undefined ? v : (v + next) / 2
    return `<div class="lr"><i style="background:${color(mid)}"></i><span>${fmt(v)}${notes[v] ? ` <em>${notes[v]}</em>` : ""}</span></div>`
  }).join("")}</div>`
let legendKey = ""
const updateLegends = (): void => {
  const lg = view.legends()
  const key = `${lg.thermal}|${lg.wind}|${lg.windMin.toFixed(1)}|${lg.windMax.toFixed(1)}`
  if (key === legendKey) return
  legendKey = key
  const parts: string[] = []
  if (lg.thermal) {
    const tl = THERMAL_LIMITS
    parts.push(legendHtml("Steel surface T [K] · 301/30X", [300, 500, tl.steelFull, 800, tl.steelReduced, 1025, tl.steelDesign, 1275, tl.steelFail, 1600],
      (v) => `#${marginColor(v).getHexString()}`, (v) => v.toFixed(0),
      { [tl.steelFull]: "nominal", [tl.steelReduced]: "reduced", [tl.steelDesign]: "design limit", [tl.steelFail]: "failure" }))
  }
  if (lg.wind) {
    const n = 10, lo = lg.windMin, hi = lg.windMax
    parts.push(legendHtml("Wind speed [m/s]", Array.from({ length: n }, (_, i) => lo + ((hi - lo) * i) / (n - 1)), (v) => `#${rainbow((v - lo) / Math.max(hi - lo, 1e-6)).getHexString()}`, (v) => v.toFixed(2)))
  }
  $("legends").innerHTML = parts.join("")
}

/** phase id at mission time tt (the last phase that has started), for the playback-rate policy */
const phaseIdAt = (tt: number): string | undefined => rec?.meta.phaseStarts.filter((p) => p.t <= tt).at(-1)?.phase

const epochMs = (): number => (rec ? (rec.meta.epochJd - 2440587.5) * 86400000 : 0)
let last = performance.now()
let lastUi = 0
const loop = (now: number): void => {
  const dt = Math.min(0.1, (now - last) / 1000)
  last = now
  if (rec) {
    if (playing) {
      t += dt * playbackRate(phaseIdAt(t), Number($<HTMLSelectElement>("speed").value), Number($<HTMLSelectElement>("landing-speed").value))
      if (t >= duration) { t = duration; setPlaying(false) }
    }
    // resume once the exploded parts have sprung back together
    if (resumeAfterAssemble && !explodeBox.checked && view.explodeProgress() < 0.01) { resumeAfterAssemble = false; setPlaying(true) }
    $("played").style.width = `${(t / duration) * 100}%`
    $("playhead").style.left = `${(t / duration) * 100}%`
    $("tlabel").textContent = fmtT(t)
    view.render(t)
    if (now - lastUi > 90) {
      lastUi = now
      updateTelemetry()
      drawCursor()
      drawAltGraph(t)
      const slow = isSlowed(phaseIdAt(t), Number($<HTMLSelectElement>("speed").value), Number($<HTMLSelectElement>("landing-speed").value))
      $("phase-name").textContent = phaseAt(t) + (slow ? "  ·  slow motion ×" + $<HTMLSelectElement>("landing-speed").value : "")
      $("clock").textContent = `${new Date(epochMs() + t * 1000).toISOString().replace("T", " ").slice(0, 19)} UTC`
      evEls.forEach((e) => e.el.classList.toggle("past", e.t <= t))
      updateLegends()
    }
  }
  requestAnimationFrame(loop)
}
requestAnimationFrame(loop)
document.body.classList.add("locked")
view.resize()
$("b-resim").onclick = () => { setPlaying(false); rec = null; document.body.classList.add("locked"); void clearCached().then(() => runSim()) }
void boot(new URLSearchParams(location.search).has("resim"))
