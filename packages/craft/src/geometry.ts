import { findRoot } from "@ship/math"
import { type V3, v3 } from "@ship/math"

/**
 * Outer mould line of the flapless ship: a cylinder (x = 0 … Lc, x measured from the aft
 * skirt plane toward the nose) and a spherically blunted tangent ogive (Lc … L).
 * The same profile feeds the aerodynamic panels, the heating stations and the 3D mesh.
 */
export interface ShapeParams {
  readonly length: number // m, total (Ship 42 / Block 3: 52.1 m)
  readonly radius: number // m (9 m diameter)
  readonly noseLength: number // m, blunted ogive length
  readonly noseRadius: number // m, spherical nose cap radius Rn
}

export interface Ogive {
  readonly Ls: number // sharp ogive length
  readonly rho: number // ogive arc radius
  readonly x0: number // sphere centre (from virtual sharp tip)
  readonly xt: number // tangency point
  readonly yt: number
  readonly xa: number // blunted tip (from virtual sharp tip)
}

const ogiveFor = (Ls: number, R: number, Rn: number): Ogive => {
  const rho = (R * R + Ls * Ls) / (2 * R)
  const x0 = Ls - Math.sqrt((rho - Rn) ** 2 - (rho - R) ** 2)
  const yt = (Rn * (rho - R)) / (rho - Rn)
  const xt = x0 - Math.sqrt(Rn * Rn - yt * yt)
  return { Ls, rho, x0, xt, yt, xa: x0 - Rn }
}

export interface Shape {
  readonly p: ShapeParams
  readonly Lc: number // cylinder length
  readonly og: Ogive
  readonly xTip: number
  readonly xCapEnd: number // where the spherical cap meets the ogive (body x)
  /** radius at body x */
  readonly r: (x: number) => number
  /** dr/dx at body x */
  readonly drdx: (x: number) => number
}

export const makeShape = (p: ShapeParams): Shape => {
  const R = p.radius
  const Rn = p.noseRadius
  const Ls = findRoot((L) => L - ogiveFor(L, R, Rn).xa - p.noseLength, p.noseLength * 0.8, p.noseLength * 2, 1e-9)
  const og = ogiveFor(Ls, R, Rn)
  const Lc = p.length - p.noseLength
  // body x  ↔  ogive coordinate s (from the virtual sharp tip toward the base): s = Ls - (x - Lc)
  const toS = (x: number): number => Ls - (x - Lc)
  const r = (x: number): number => {
    if (x <= Lc) return x < 0 ? 0 : R
    if (x >= p.length) return 0
    const s = toS(x)
    if (s <= og.xt) return Math.sqrt(Math.max(0, Rn * Rn - (s - og.x0) ** 2))
    return Math.sqrt(Math.max(0, og.rho * og.rho - (Ls - s) ** 2)) + R - og.rho
  }
  /** dr/dx = -dr/ds */
  const drdx = (x: number): number => {
    if (x <= Lc) return 0
    const s = toS(x)
    if (s <= og.xt) {
      const y = Math.sqrt(Math.max(1e-12, Rn * Rn - (s - og.x0) ** 2))
      return (s - og.x0) / y
    }
    const y = Math.sqrt(Math.max(1e-12, og.rho * og.rho - (Ls - s) ** 2))
    return -(Ls - s) / y
  }
  return { p, Lc, og, xTip: p.length, xCapEnd: p.length - (og.xt - og.xa), r, drdx }
}

/** Surface panel for Newtonian integration and heat mapping. Body frame: x nose, (y,z) = (cos φ, sin φ). */
export interface Panel {
  readonly c: V3 // centroid
  readonly n: V3 // outward unit normal
  readonly area: number
  readonly x: number
  readonly phi: number
  readonly station: number
}

/** Axial stations: dense on the nose, coarser on the barrel. Returns station edges (x). */
export const stationEdges = (s: Shape, nNose = 22, nCyl = 14): number[] => {
  const e: number[] = []
  for (let i = 0; i <= nCyl; i++) e.push((s.Lc * i) / nCyl)
  // cosine clustering toward the tip
  for (let i = 1; i <= nNose; i++) {
    const t = i / nNose
    const u = Math.sin((t * Math.PI) / 2)
    e.push(s.Lc + (s.p.length - s.Lc) * u)
  }
  return e
}

export const makePanels = (s: Shape, nPhi = 16): { panels: Panel[]; edges: number[] } => {
  const edges = stationEdges(s)
  const panels: Panel[] = []
  // base disk (aft), split in nPhi sectors; normal -x
  const R = s.p.radius
  for (let j = 0; j < nPhi; j++) {
    const phi = ((j + 0.5) / nPhi) * 2 * Math.PI
    const rc = (2 / 3) * R
    panels.push({ c: v3(0, rc * Math.cos(phi), rc * Math.sin(phi)), n: v3(-1, 0, 0), area: (Math.PI * R * R) / nPhi, x: 0, phi, station: -1 })
  }
  for (let i = 0; i < edges.length - 1; i++) {
    const xa = edges[i]
    const xb = edges[i + 1]
    const ra = s.r(xa)
    const rb = s.r(xb)
    const xm = 0.5 * (xa + xb)
    const rm = 0.5 * (ra + rb)
    const slant = Math.hypot(xb - xa, rb - ra)
    const dr = (rb - ra) / Math.max(xb - xa, 1e-9)
    const nn = Math.hypot(1, dr)
    for (let j = 0; j < nPhi; j++) {
      const phi = ((j + 0.5) / nPhi) * 2 * Math.PI
      const cy = Math.cos(phi)
      const cz = Math.sin(phi)
      panels.push({
        c: v3(xm, rm * cy, rm * cz),
        n: v3(-dr / nn, cy / nn, cz / nn),
        area: (slant * (2 * Math.PI * rm)) / nPhi,
        x: xm,
        phi,
        station: i,
      })
    }
  }
  return { panels, edges }
}

/** wetted area between x0 and x1 */
export const wettedArea = (s: Shape, x0: number, x1: number, n = 400): number => {
  let A = 0
  for (let i = 0; i < n; i++) {
    const a = x0 + ((x1 - x0) * i) / n
    const b = x0 + ((x1 - x0) * (i + 1)) / n
    const ra = s.r(a)
    const rb = s.r(b)
    A += Math.PI * (ra + rb) * Math.hypot(b - a, rb - ra)
  }
  return A
}

/** planform (side-projected) area and its centroid */
export const planform = (s: Shape, n = 600): { area: number; xc: number } => {
  let A = 0
  let M = 0
  for (let i = 0; i < n; i++) {
    const x = ((i + 0.5) / n) * s.p.length
    const dA = 2 * s.r(x) * (s.p.length / n)
    A += dA
    M += dA * x
  }
  return { area: A, xc: M / A }
}
