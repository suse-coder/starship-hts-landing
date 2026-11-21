import * as THREE from "three"
import type { Shape } from "@ship/craft"
import { Hull } from "./kit"

/**
 * PICA-X nosecone TPS appearance (spec/07 row 41): hexagonal tiles ≈ 0.30 m across flats with the
 * fine phenolic honeycomb carrier (≈ 3 cm cells) inside each tile, dark RTV-filled gaps, slight char
 * variation per tile and per cell. Procedural DataTextures (no DOM).
 *
 * One texture repeat covers PICA_TEX.w × PICA_TEX.h metres of surface and tiles seamlessly: the tile
 * lattice (pointy-top hexagons, circumradius a = 0.17 m) repeats every 4·√3·a × 6·a and the cell
 * lattice (circumradius a/10) every 40·√3·(a/10) × 60·(a/10), i.e. on the same rectangle.
 */
const A = 0.17 // tile circumradius → 0.294 m across flats
const B = A / 10 // honeycomb cell circumradius → 2.9 cm across flats
export const PICA_TEX = { w: 4 * Math.sqrt(3) * A, h: 6 * A } as const

const S3 = Math.sqrt(3)
const mod = (a: number, n: number): number => ((a % n) + n) % n
const hash = (x: number, y: number): number => {
  let h = Math.imul(x | 0, 374761393) ^ Math.imul(y | 0, 668265263)
  h = Math.imul(h ^ (h >>> 13), 1274126177)
  return ((h ^ (h >>> 16)) >>> 0) / 4294967295
}

/** nearest pointy-top hex centre for circumradius a: axial (q, r) and the distance to the hex edge */
const hexCell = (x: number, y: number, a: number): { q: number; r: number; edge: number } => {
  const qf = ((S3 / 3) * x - y / 3) / a, rf = ((2 / 3) * y) / a, sf = -qf - rf
  let q = Math.round(qf), r = Math.round(rf)
  const s = Math.round(sf)
  const dq = Math.abs(q - qf), dr = Math.abs(r - rf), ds = Math.abs(s - sf)
  if (dq > dr && dq > ds) q = -r - s
  else if (dr > ds) r = -q - s
  const cx = a * S3 * (q + r / 2), cy = a * 1.5 * r
  const px = Math.abs(x - cx), py = Math.abs(y - cy)
  return { q, r, edge: (a * S3) / 2 - Math.max(px, px * 0.5 + py * (S3 / 2)) }
}
/** lattice id reduced to the texture period (periods (n, 0) and (−n/2, n) in axial units) */
const periodicId = (q: number, r: number, n: number): number => {
  const k = Math.floor(r / n)
  return hash(mod(q + (n / 2) * k, n), r - n * k)
}

/** periodic value noise on an m × m lattice over the unit square */
const vnoise = (u: number, v: number, m: number, seed: number): number => {
  const x = u * m, y = v * m
  const i = Math.floor(x), j = Math.floor(y), fx = x - i, fy = y - j
  const g = (a: number, b: number): number => hash(mod(a, m) + seed * 977, mod(b, m) + seed * 131)
  const sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy)
  return (g(i, j) * (1 - sx) + g(i + 1, j) * sx) * (1 - sy) + (g(i, j + 1) * (1 - sx) + g(i + 1, j + 1) * sx) * sy
}

const tex = (data: Uint8Array, n: number, srgb: boolean): THREE.DataTexture => {
  const t = new THREE.DataTexture(data, n, n, THREE.RGBAFormat)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.magFilter = THREE.LinearFilter
  t.minFilter = THREE.LinearMipmapLinearFilter
  t.generateMipmaps = true
  t.anisotropy = 8
  if (srgb) t.colorSpace = THREE.SRGBColorSpace
  t.needsUpdate = true
  return t
}

/**
 * Colour (sRGB), tangent-space normal and roughness maps of the PICA-X honeycomb tiles. Use with UVs
 * in units of one texture repeat (see makePicaGeometry), or set `repeat` on all three maps.
 */
export const makePicaMaps = (size = 1024): { map: THREE.Texture; normalMap: THREE.Texture; roughnessMap: THREE.Texture } => {
  const n = size
  const H = new Float32Array(n * n)
  const col = new Uint8Array(n * n * 4), rough = new Uint8Array(n * n * 4), nrm = new Uint8Array(n * n * 4)
  const gap = 0.0035, bevel = 0.012, wall = 0.0011
  for (let j = 0; j < n; j++) {
    const v = (j + 0.5) / n, y = v * PICA_TEX.h
    for (let i = 0; i < n; i++) {
      const u = (i + 0.5) / n, x = u * PICA_TEX.w
      const t = hexCell(x, y, A), c = hexCell(x, y, B)
      const tid = periodicId(t.q, t.r, 4), cid = periodicId(c.q, c.r, 40)
      const noise = 0.65 * vnoise(u, v, 48, 1) + 0.35 * vnoise(u, v, 160, 2)
      let h: number, cr: number, cg: number, cb: number, ro: number
      if (t.edge < gap) {
        h = 0; cr = 11; cg = 10; cb = 9; ro = 0.97 // RTV-filled tile gap
      } else {
        const bev = Math.min(1, (t.edge - gap) / bevel)
        const isWall = c.edge < wall
        const ab = 1 - Math.min(1, c.edge / (B * 0.8)) // slight cupping of each cell toward its walls
        h = 0.55 + 0.45 * bev * bev * (3 - 2 * bev) + (isWall ? 0.05 : -0.02 * (1 - ab)) + 0.04 * (noise - 0.5)
        const tileK = 0.86 + 0.28 * tid // char variation between tiles
        const cellK = 0.94 + 0.12 * cid
        const k = tileK * cellK * (0.88 + 0.24 * noise) * (0.75 + 0.25 * bev)
        if (isWall) { cr = 46 * k; cg = 41 * k; cb = 36 * k; ro = 0.84 } // phenolic honeycomb carrier
        else { cr = 31 * k; cg = 28 * k; cb = 25 * k; ro = 0.9 + 0.06 * noise }
      }
      const p = j * n + i
      H[p] = h
      col[p * 4] = Math.min(255, cr); col[p * 4 + 1] = Math.min(255, cg); col[p * 4 + 2] = Math.min(255, cb); col[p * 4 + 3] = 255
      const rb = Math.round(Math.min(1, ro) * 255)
      rough[p * 4] = rough[p * 4 + 1] = rough[p * 4 + 2] = rb; rough[p * 4 + 3] = 255
    }
  }
  // normals from the periodic height field; slopes in metres (height range ≈ 6 mm)
  const px = PICA_TEX.w / n, py = PICA_TEX.h / n, depth = 0.006
  for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) {
    const hx = (H[j * n + mod(i + 1, n)] - H[j * n + mod(i - 1, n)]) * depth / (2 * px)
    const hy = (H[mod(j + 1, n) * n + i] - H[mod(j - 1, n) * n + i]) * depth / (2 * py)
    const l = Math.hypot(hx, hy, 1)
    const p = (j * n + i) * 4
    nrm[p] = Math.round((-hx / l * 0.5 + 0.5) * 255); nrm[p + 1] = Math.round((-hy / l * 0.5 + 0.5) * 255); nrm[p + 2] = Math.round((1 / l * 0.5 + 0.5) * 255); nrm[p + 3] = 255
  }
  return { map: tex(col, n, true), normalMap: tex(nrm, n, false), roughnessMap: tex(rough, n, false) }
}

/**
 * Optional PICA-X surface for ship.ts with metric UVs (1 unit = one texture repeat) so the tiles keep
 * their real size: ring bands on the ogive (integer repeats around each band, ±8 % stretch) and an
 * azimuthal-equidistant projection on the spherical cap. Offset `h` along the outward normal.
 */
export const makePicaGeometry = (shape: Shape, x0: number, h = 0.004, seg = 96): THREE.BufferGeometry => {
  const hull = new Hull(shape)
  const s0 = hull.sOf(x0), sc = hull.sOf(shape.xCapEnd), sTip = hull.length
  const pos: number[] = [], uv: number[] = [], idx: number[] = []
  const grid = (ss: readonly number[], uvAt: (s: number, phi: number) => [number, number]): void => {
    const base = pos.length / 3
    for (const s of ss) for (let i = 0; i <= seg; i++) {
      const phi = (i / seg) * Math.PI * 2
      const p = hull.point(Math.min(s, sTip - 1e-4), phi, h)
      pos.push(p.x, p.y, p.z)
      uv.push(...uvAt(s, phi))
    }
    for (let j = 0; j < ss.length - 1; j++) for (let i = 0; i < seg; i++) {
      const a = base + j * (seg + 1) + i, b = a + 1, c = a + seg + 1, d = c + 1
      idx.push(a, b, c, b, d, c)
    }
  }
  const steps = (a: number, b: number, ds: number): number[] => { const k = Math.max(1, Math.ceil((b - a) / ds)); return Array.from({ length: k + 1 }, (_, i) => a + ((b - a) * i) / k) }
  // ogive bands: boundaries where r falls by 15 %
  const bounds = [s0]
  let rB = shape.p.radius * 0.85
  for (let s = s0; s < sc; s += 0.02) if (hull.at(s).r < rB) { bounds.push(s); rB *= 0.85 }
  bounds.push(sc)
  for (let b = 0; b < bounds.length - 1; b++) {
    const rm = hull.at((bounds[b] + bounds[b + 1]) / 2).r
    const N = Math.max(1, Math.round((2 * Math.PI * rm) / PICA_TEX.w))
    grid(steps(bounds[b], bounds[b + 1], 0.15), (s, phi) => [(phi / (Math.PI * 2)) * N, (s - s0) / PICA_TEX.h])
  }
  // cap: distance from the tip along the meridian, projected azimuthally
  grid(steps(sc, sTip, 0.08), (s, phi) => { const d = sTip - s; return [(d * Math.cos(phi)) / PICA_TEX.w, (d * Math.sin(phi)) / PICA_TEX.h] })
  const g = new THREE.BufferGeometry()
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3))
  g.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2))
  g.setIndex(idx)
  g.computeVertexNormals()
  return g
}
