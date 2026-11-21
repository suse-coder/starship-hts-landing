/**
 * Heat-survival report (spec/04 A4.8–A4.9): does the tile surface and the steel backing survive, does
 * the PICA-X bondline hold, and how much does the HTS coil push the shock and the heat flux away?
 * Pure functions over the recording; DOM rendering is separate.
 */
import { makeVehicle, TILE_MATERIAL_OPTIONS, type DesignParams } from "@ship/craft"
import { makeThermalGrid, steelVerdict, steelYieldRetention, THERMAL_LIMITS as TL } from "@ship/heat"
import { CH, type ChannelKey, type Recording } from "@ship/sim"

export interface HotSpot {
  readonly x: number // body station, m
  readonly phiDeg: number // from +y toward +z
  readonly T: number // peak surface temperature, K
  readonly tPeak: number // mission time of the peak, s
  readonly k: number // yield retention at the peak
  readonly verdict: string
}

export interface Survival {
  readonly steelPeak: number
  readonly steelPeakX: number
  readonly steelPeakT: number
  readonly kMin: number
  readonly marginDesign: number // K below the 1150 K design limit (negative = exceeded)
  readonly marginFail: number
  readonly secAboveReduced: number
  readonly secAboveDesign: number
  readonly cellsAboveLimit: number // surface cells that ever exceeded the tile limit
  readonly surfaceCells: number
  readonly bondPeak: number
  readonly bondMargin: number
  readonly tileSurfPeak: number
  readonly tileSurfMargin: number
  readonly tileBackPeak: number
  readonly tileBackMargin: number
  readonly recession: number
  readonly picaCap: number
  readonly hotSpots: readonly HotSpot[]
  // coil effect at the stagnation point (same trajectory, field on vs off)
  readonly qPeak: number // W/m² with the field
  readonly qPeakNoField: number
  readonly fAtPeak: number
  readonly fMin: number
  readonly loadWith: number // J/m² stagnation heat load
  readonly loadWithout: number
  readonly standoffAtPeak: number
  readonly standoff0AtPeak: number
  readonly stuartMax: number
}

export const survival = (r: Recording): Survival => {
  const ch = (k: ChannelKey): Float64Array => r.data[CH[k]] as Float64Array
  const t = ch("t"), steel = ch("steel"), steelX = ch("steelX"), bond = ch("bondline"), q = ch("qStag"), hf = ch("heatFactor")
  const tileSurf = ch("tileSurf"), tileBack = ch("tileBack")
  const so = ch("standoff"), so0 = ch("standoff0"), stu = ch("stuart"), rec = ch("recession")
  const design = (r.meta.vehicle as { design: DesignParams }).design
  const grid = makeThermalGrid(makeVehicle(design))
  // per-cell peak over the run
  const peak = new Float32Array(r.cells), tPk = new Float32Array(r.cells)
  for (let i = 0; i < r.n; i++) {
    const base = i * r.cells
    for (let c = 0; c < r.cells; c++) { const T = r.heat[base + c]; if (T > peak[c]) { peak[c] = T; tPk[c] = t[i] } }
  }
  let iS = 0, iQ = 0, aboveR = 0, aboveD = 0, loadW = 0, loadWo = 0, fMin = 1, stuMax = 0, bondPeak = 0, recMax = 0
  for (let i = 0; i < r.n; i++) {
    if (steel[i] > steel[iS]) iS = i
    if (q[i] > q[iQ]) iQ = i
    const dt = i > 0 ? t[i] - t[i - 1] : 0
    if (steel[i] > TL.steelReduced) aboveR += dt
    if (steel[i] > TL.steelDesign) aboveD += dt
    const f = hf[i] > 0 ? Math.min(1, hf[i]) : 1
    loadW += q[i] * dt
    loadWo += (q[i] / f) * dt
    fMin = Math.min(fMin, f)
    stuMax = Math.max(stuMax, stu[i])
    bondPeak = Math.max(bondPeak, bond[i])
    recMax = Math.max(recMax, rec[i])
  }
  const nRays = r.cells / grid.nSt
  const steelCells = Array.from({ length: r.cells }, (_, c) => c).filter((c) => !grid.cellPica[c])
  const hotSpots = [...steelCells].sort((a, b) => peak[b] - peak[a]).slice(0, 6).map((c): HotSpot => ({
    x: grid.xs[Math.floor(c / nRays)], phiDeg: (grid.cellPhi[c] * 180) / Math.PI, T: peak[c], tPeak: tPk[c],
    k: steelYieldRetention(peak[c]), verdict: steelVerdict(peak[c]),
  }))
  const fQ = hf[iQ] > 0 ? Math.min(1, hf[iQ]) : 1
  return {
    steelPeak: steel[iS], steelPeakX: steelX[iS], steelPeakT: t[iS], kMin: steelYieldRetention(steel[iS]),
    marginDesign: TL.steelDesign - steel[iS], marginFail: TL.steelFail - steel[iS],
    secAboveReduced: aboveR, secAboveDesign: aboveD,
    cellsAboveLimit: steelCells.filter((c) => peak[c] > TL.tileSurface).length, surfaceCells: steelCells.length,
    bondPeak, bondMargin: TL.bondline - bondPeak,
    tileSurfPeak: Math.max(...tileSurf), tileSurfMargin: TL.tileSurface - Math.max(...tileSurf),
    tileBackPeak: Math.max(...tileBack), tileBackMargin: TL.tileBond - Math.max(...tileBack),
    recession: recMax, picaCap: design.picaCap,
    hotSpots,
    qPeak: q[iQ], qPeakNoField: q[iQ] / fQ, fAtPeak: fQ, fMin, loadWith: loadW, loadWithout: loadWo,
    standoffAtPeak: so[iQ], standoff0AtPeak: so0[iQ], stuartMax: stuMax,
  }
}

const pass = (ok: boolean, warn = false): string => `<span class="vd ${ok ? (warn ? "warn" : "ok") : "bad"}">${ok ? (warn ? "MARGINAL" : "PASS") : "FAIL"}</span>`
const fmtT = (s: number): string => { const m = Math.floor(s / 60); return `T+${m}:${String(Math.floor(s - m * 60)).padStart(2, "0")}` }

export const renderSurvival = (s: Survival, onSeek: (t: number) => void): HTMLElement => {
  const el = document.createElement("div")
  el.className = "survival"
  const pct = (a: number, b: number): string => `${((1 - a / Math.max(b, 1e-9)) * 100).toFixed(0)} %`
  el.innerHTML = `
    <h3>Heat survival</h3>
    <div class="row"><span>301/30X steel peak</span><span>${s.steelPeak.toFixed(0)} K @ x ${s.steelPeakX.toFixed(1)} m ${pass(s.marginDesign > 0, s.marginDesign < 100)}</span></div>
    <div class="row"><span>margin to design / failure</span><span>${s.marginDesign.toFixed(0)} K / ${s.marginFail.toFixed(0)} K</span></div>
    <div class="row"><span>min. yield retention k</span><span>${(s.kMin * 100).toFixed(0)} % of RT yield</span></div>
    <div class="row"><span>time &gt; ${TL.steelReduced} K / &gt; ${TL.steelDesign} K</span><span>${s.secAboveReduced.toFixed(0)} s / ${s.secAboveDesign.toFixed(0)} s</span></div>
    <div class="row"><span>tile-surface cells over ${TL.tileSurface} K</span><span>${s.cellsAboveLimit} / ${s.surfaceCells}</span></div>
    <div class="row"><span>PICA-X bondline peak</span><span>${s.bondPeak.toFixed(0)} K (limit ${TL.bondline}) ${pass(s.bondMargin > 0, s.bondMargin < 40)}</span></div>
    <div class="row"><span>Silica tile surface peak</span><span>${s.tileSurfPeak.toFixed(0)} K (LI-900 ${TL.tileSurface} / sourced hex claim ${TILE_MATERIAL_OPTIONS.starshipHex.surfaceLimitK}) ${pass(s.tileSurfMargin > 0, s.tileSurfMargin < 100)}</span></div>
    <div class="row"><span>Tile backing (steel) peak</span><span>${s.tileBackPeak.toFixed(0)} K (adhesive ${TL.tileBond}, steel ${TL.steelDesign}) ${pass(s.tileBackMargin > 0, s.tileBackMargin < 50)}</span></div>
    <div class="row"><span>PICA-X recession</span><span>${(s.recession * 1000).toFixed(1)} of ${(s.picaCap * 1000).toFixed(0)} mm</span></div>
    <h4>Hottest tile-surface stations (the steel backing is the check above)</h4>
    <table class="hot"><tr><th>x m</th><th>φ°</th><th>T K</th><th>margin K</th><th>verdict</th></tr>
      ${s.hotSpots.map((h, i) => `<tr data-i="${i}" title="jump to ${fmtT(h.tPeak)}"><td>${h.x.toFixed(1)}</td><td>${h.phiDeg.toFixed(0)}</td><td>${h.T.toFixed(0)}</td><td>${(TL.tileSurface - h.T).toFixed(0)}</td><td>${h.T <= TL.tileSurface ? "within" : "over limit"}</td></tr>`).join("")}
    </table>
    <h4>HTS coil effect (stagnation point, same trajectory)</h4>
    <div class="row"><span>shock standoff Δ / Δ₀ at peak q</span><span>${s.standoffAtPeak.toFixed(2)} / ${s.standoff0AtPeak.toFixed(2)} m</span></div>
    <div class="row"><span>peak heat flux with / without</span><span>${(s.qPeak * 1e-4).toFixed(0)} / ${(s.qPeakNoField * 1e-4).toFixed(0)} W/cm²</span></div>
    <div class="row"><span>heat-flux factor at peak / min</span><span>${s.fAtPeak.toFixed(2)} / ${s.fMin.toFixed(2)}</span></div>
    <div class="row"><span>stag. heat load with / without</span><span>${(s.loadWith * 1e-7).toFixed(1)} / ${(s.loadWithout * 1e-7).toFixed(1)} kJ/cm² (−${pct(s.loadWith, s.loadWithout)})</span></div>
    <div class="row"><span>peak Stuart number Q</span><span>${s.stuartMax.toFixed(2)}</span></div>
    <p class="note">Heat-flux factor credited to the heating model: (1+Q_eff)^−¼ from the electrode-driven Lorentz force.
    The passive standoff growth Δ = Δ₀(1+Q)^¼ is drawn but not credited to heating (conservative).</p>`
  el.querySelectorAll<HTMLTableRowElement>("tr[data-i]").forEach((tr) => { tr.onclick = () => onSeek(s.hotSpots[Number(tr.dataset.i)].tPeak) })
  return el
}
