import * as THREE from "three"
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js"
import type { Shape } from "@ship/craft"

/**
 * Geometry kernel for the CAD-style vehicle model (docs/03-spec/07-cad-model.md).
 * Body frame: x along the axis (aft skirt plane 0 → nose tip), φ measured from +y toward +z.
 * Everything here is DOM-free (procedural textures are DataTextures) so the module can be imported
 * by tests and workers.
 */

/** profile point (radius, body x) for solids of revolution about body x */
export type RX = readonly [number, number]

export const v3 = (x: number, y: number, z: number): THREE.Vector3 => new THREE.Vector3(x, y, z)
/** point at body x, radius ρ, azimuth φ (from +y toward +z) */
export const polar = (x: number, rho: number, phi: number): THREE.Vector3 => new THREE.Vector3(x, rho * Math.cos(phi), rho * Math.sin(phi))
export const deg = (d: number): number => (d * Math.PI) / 180

// ---------------------------------------------------------------- procedural textures

const makeData = (w: number, h: number, px: (i: number, j: number, out: number[]) => void, srgb = false): THREE.DataTexture => {
  const data = new Uint8Array(w * h * 4)
  const c = [0, 0, 0, 255]
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
    c[3] = 255
    px(i, j, c)
    const k = (j * w + i) * 4
    data[k] = c[0]; data[k + 1] = c[1]; data[k + 2] = c[2]; data[k + 3] = c[3]
  }
  const t = new THREE.DataTexture(data, w, h, THREE.RGBAFormat)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.magFilter = THREE.LinearFilter
  t.minFilter = THREE.LinearMipmapLinearFilter
  t.generateMipmaps = true
  t.anisotropy = 8
  if (srgb) t.colorSpace = THREE.SRGBColorSpace
  t.needsUpdate = true
  return t
}

/** tangent-space normal map from a periodic height field h(u, v), u, v ∈ [0, 1) */
export const normalFromHeight = (w: number, h: number, height: (u: number, v: number) => number, strength: number): THREE.DataTexture => {
  const H = new Float32Array(w * h)
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) H[j * w + i] = height(i / w, j / h)
  const at = (i: number, j: number): number => H[((j + h) % h) * w + ((i + w) % w)]
  return makeData(w, h, (i, j, c) => {
    const dx = (at(i + 1, j) - at(i - 1, j)) * 0.5 * strength
    const dy = (at(i, j + 1) - at(i, j - 1)) * 0.5 * strength
    const n = new THREE.Vector3(-dx, -dy, 1).normalize()
    c[0] = Math.round((n.x * 0.5 + 0.5) * 255); c[1] = Math.round((n.y * 0.5 + 0.5) * 255); c[2] = Math.round((n.z * 0.5 + 0.5) * 255)
  })
}

/**
 * Regenerative-cooling channel ribs along the nozzle meridian: one rib per texture repeat in u
 * (lathe u runs around the axis). Raptor nozzles are channel-wall regen nozzles; ≈ 160 channels drawn.
 */
export const RIB_COUNT = 160
const ribNormal = normalFromHeight(64, 4, (u) => Math.pow(Math.abs(Math.sin(Math.PI * u)), 0.55), 3.2)
ribNormal.repeat.set(RIB_COUNT, 1)

/** filament-wound COPV over-wrap: ±55° helical tow bands (diamond pattern) */
const windNormal = normalFromHeight(64, 64, (u, v) => {
  const a = (u * 4 + v * 4) % 1, b = (u * 4 - v * 4 + 8) % 1
  const tri = (t: number): number => 1 - Math.abs(2 * t - 1)
  return 0.5 * Math.pow(tri(a), 0.4) + 0.5 * Math.pow(tri(b), 0.4)
}, 1.4)
windNormal.repeat.set(18, 10)

/** knurled / perforated louvre texture is not needed; a fine brushed-grain normal for machined parts */
const brushNormal = normalFromHeight(4, 128, (_u, v) => 0.5 + 0.5 * Math.sin(v * Math.PI * 2 * 31) * Math.sin(v * Math.PI * 2 * 7), 0.25)
brushNormal.repeat.set(1, 6)

// ---------------------------------------------------------------- materials (PBR, spec/07 §3)

const std = (p: THREE.MeshStandardMaterialParameters): THREE.MeshStandardMaterial => new THREE.MeshStandardMaterial(p)

export const MAT = {
  /** 301/30X bare brushed steel (spec/07 §3) */
  steel: std({ color: 0xb9bec4, metalness: 0.9, roughness: 0.3 }),
  /** thin steel shells seen from both sides (domes, cans, deck) */
  steelShell: std({ color: 0xaeb3b9, metalness: 0.88, roughness: 0.34, side: THREE.DoubleSide }),
  steelDark: std({ color: 0x5b6168, metalness: 0.85, roughness: 0.4 }),
  steelDarkShell: std({ color: 0x6a7077, metalness: 0.82, roughness: 0.45, side: THREE.DoubleSide }),
  /** PEZ door skin: same steel, slightly different temper/roughness so the panel reads */
  door: std({ color: 0xa8adb3, metalness: 0.88, roughness: 0.38 }),
  weld: std({ color: 0x8a8f95, metalness: 0.75, roughness: 0.55 }),
  forging: std({ color: 0x7d8288, metalness: 0.85, roughness: 0.42, normalMap: brushNormal }),
  /** Inconel / copper-alloy nozzle (legacy key, double sided) */
  inconel: std({ color: 0x6b5f55, metalness: 0.8, roughness: 0.45, side: THREE.DoubleSide }),
  /** regen-cooled nozzle outer wall: base white × vertex heat tint, channel-rib normal map */
  nozzle: std({ color: 0xffffff, vertexColors: true, metalness: 0.85, roughness: 0.36, normalMap: ribNormal, normalScale: new THREE.Vector2(0.9, 0.9) }),
  /** nozzle hot-gas wall (inside the bell): sooted, heat tinted */
  soot: std({ color: 0x2e2925, metalness: 0.35, roughness: 0.82 }),
  /** RVac radiatively cooled extension: niobium C-103 with silicide coating */
  niobium: std({ color: 0x4b4f58, metalness: 0.75, roughness: 0.42, side: THREE.DoubleSide }),
  /** machined superalloy (powerhead, pumps, preburners) */
  alloy: std({ color: 0x9b958d, metalness: 0.88, roughness: 0.33 }),
  alloyDark: std({ color: 0x6d6862, metalness: 0.85, roughness: 0.4 }),
  chrome: std({ color: 0xdfe3e7, metalness: 1.0, roughness: 0.14 }),
  black: std({ color: 0x232528, metalness: 0.45, roughness: 0.55 }),
  copper: std({ color: 0xb87333, metalness: 0.85, roughness: 0.35 }),
  aluminium: std({ color: 0xc9ccd1, metalness: 0.9, roughness: 0.4 }),
  cfrp: std({ color: 0x2b2e33, metalness: 0.3, roughness: 0.5 }),
  /** COPV over-wrap: CFRP with the helical winding normal map (spec/07 §3) */
  copv: std({ color: 0x2b2e33, metalness: 0.35, roughness: 0.42, normalMap: windNormal, normalScale: new THREE.Vector2(0.6, 0.6) }),
  lox: std({ color: 0x6fb6ff, metalness: 0.1, roughness: 0.2, transparent: true, opacity: 0.22, depthWrite: false }),
  ch4: std({ color: 0xffb36f, metalness: 0.1, roughness: 0.2, transparent: true, opacity: 0.22, depthWrite: false }),
  rebco: std({ color: 0xc58b3c, metalness: 0.7, roughness: 0.35, emissive: 0x000000 }),
  cryo: std({ color: 0xb5bcc4, metalness: 0.6, roughness: 0.35, transparent: true, opacity: 0.35, depthWrite: false }),
  /** multilayer insulation (aluminised Kapton, gold side out) */
  mli: std({ color: 0xc9a24f, metalness: 0.95, roughness: 0.3 }),
  /** G10/FR4 glass-epoxy support struts (low heat leak) */
  g10: std({ color: 0xb8b07a, metalness: 0.0, roughness: 0.6 }),
  box: std({ color: 0x3d4450, metalness: 0.4, roughness: 0.6 }),
  battery: std({ color: 0x2f6f3f, metalness: 0.3, roughness: 0.6 }),
  panel: std({ color: 0x1b1d21, metalness: 0.2, roughness: 0.7 }),
  rubber: std({ color: 0x161616, metalness: 0.0, roughness: 0.92 }),
  /** Starlink phased-array radome (PTFE/glass composite) */
  radome: std({ color: 0xd9d6cc, metalness: 0.05, roughness: 0.62 }),
  /** tungsten–copper electrode face */
  electrode: std({ color: 0x8d7a66, metalness: 0.92, roughness: 0.32 }),
  /** PICA-X edge (char) */
  pica: std({ color: 0x24201c, metalness: 0.0, roughness: 0.95 }),
  harness: std({ color: 0x34363a, metalness: 0.1, roughness: 0.7 }),
}
export type MatKey = keyof typeof MAT

// ---------------------------------------------------------------- geometry builders (body frame)

/** lathe about body x from (r, x) pairs; outward faces need the profile ordered by increasing x (outside on the right) */
export const latheGeo = (pts: readonly RX[], seg = 48, phiStart = 0, phiLength = Math.PI * 2): THREE.BufferGeometry => {
  // LatheGeometry azimuth φL maps to body φ = φL + π/2 after the rotation (+Y → +x)
  const g = new THREE.LatheGeometry(pts.map(([r, x]) => new THREE.Vector2(Math.max(r, 1e-4), x)), seg, phiStart - Math.PI / 2, phiLength)
  g.rotateZ(-Math.PI / 2)
  return g
}
/** lathe about body x from (r, x) pairs (kept for compatibility with ship.ts callers) */
export const latheX = (pts: readonly (readonly [number, number])[], mat: THREE.Material, seg = 48): THREE.Mesh => new THREE.Mesh(latheGeo(pts, seg), mat)

/** closed (r, x) loop ordered counter-clockwise in the (r, x) plane → outward normals */
export const loopCCW = (pts: RX[]): RX[] => {
  let a = 0
  for (let i = 0; i < pts.length; i++) { const [r0, x0] = pts[i], [r1, x1] = pts[(i + 1) % pts.length]; a += r0 * x1 - r1 * x0 }
  const out = a < 0 ? [...pts].reverse() : [...pts]
  out.push(out[0])
  return out
}

/** torus about the body x axis (ring in the y–z plane) at x */
export const torusX = (R: number, tube: number, x: number, radial = 8, tubular = 48, arc = Math.PI * 2): THREE.BufferGeometry => {
  const g = new THREE.TorusGeometry(R, tube, radial, tubular, arc)
  g.rotateY(Math.PI / 2)
  g.translate(x, 0, 0)
  return g
}

/** cylinder along body x from x0 (radius r0) to x1 (radius r1) */
export const cylX = (x0: number, x1: number, r0: number, r1 = r0, seg = 32, open = false): THREE.BufferGeometry => {
  const g = new THREE.CylinderGeometry(r1, r0, Math.abs(x1 - x0), seg, 1, open)
  g.rotateZ(-Math.PI / 2)
  g.translate((x0 + x1) / 2, 0, 0)
  return g
}

const Y = new THREE.Vector3(0, 1, 0)
/** quaternion rotating +Y onto d */
export const quatY = (d: THREE.Vector3): THREE.Quaternion => new THREE.Quaternion().setFromUnitVectors(Y, d.clone().normalize())

/** cylinder from a (radius ra) to b (radius rb) */
export const cylBetween = (a: THREE.Vector3, b: THREE.Vector3, ra: number, rb = ra, seg = 12, open = false): THREE.BufferGeometry => {
  const d = b.clone().sub(a)
  const g = new THREE.CylinderGeometry(rb, ra, d.length(), seg, 1, open)
  g.applyQuaternion(quatY(d))
  g.translate((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)
  return g
}

/** swept tube along a smooth (centripetal Catmull–Rom) path */
export const tubeGeo = (pts: readonly THREE.Vector3[], r: number, tubular = 24, radial = 8): THREE.BufferGeometry =>
  new THREE.TubeGeometry(new THREE.CatmullRomCurve3([...pts], false, "centripetal"), tubular, r, radial, false)

/** box of size (sx, sy, sz) placed by a matrix */
export const boxGeo = (sx: number, sy: number, sz: number, m?: THREE.Matrix4): THREE.BufferGeometry => {
  const g = new THREE.BoxGeometry(sx, sy, sz)
  if (m) g.applyMatrix4(m)
  return g
}

export const mat4 = (pos: THREE.Vector3, q?: THREE.Quaternion, s?: THREE.Vector3): THREE.Matrix4 => new THREE.Matrix4().compose(pos, q ?? new THREE.Quaternion(), s ?? new THREE.Vector3(1, 1, 1))
/** matrix with local axes X, Y, Z at p */
export const basis = (p: THREE.Vector3, X: THREE.Vector3, Yv: THREE.Vector3, Z: THREE.Vector3): THREE.Matrix4 => new THREE.Matrix4().makeBasis(X, Yv, Z).setPosition(p)
/** local frame on the body axis at x rotated to azimuth φ: X = body x, Y = radial (φ), Z = tangential */
export const axialFrame = (x: number, phi: number, rho = 0): THREE.Matrix4 =>
  basis(polar(x, rho, phi), v3(1, 0, 0), v3(0, Math.cos(phi), Math.sin(phi)), v3(0, -Math.sin(phi), Math.cos(phi)))

/** quadratic-patch grid (u, v ∈ [0, 1]) with winding so that ∂p/∂v × ∂p/∂u is the front face */
export const gridGeo = (nu: number, nv: number, fn: (u: number, v: number) => THREE.Vector3, flip = false): THREE.BufferGeometry => {
  const pos: number[] = [], uv: number[] = [], idx: number[] = []
  for (let j = 0; j <= nv; j++) for (let i = 0; i <= nu; i++) { const p = fn(i / nu, j / nv); pos.push(p.x, p.y, p.z); uv.push(i / nu, j / nv) }
  for (let j = 0; j < nv; j++) for (let i = 0; i < nu; i++) {
    const a = j * (nu + 1) + i, b = a + 1, c = a + nu + 1, d = c + 1
    if (flip) idx.push(a, b, c, b, d, c)
    else idx.push(a, c, b, b, c, d)
  }
  const g = new THREE.BufferGeometry()
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3))
  g.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2))
  g.setIndex(idx)
  g.computeVertexNormals()
  return g
}

// ---------------------------------------------------------------- merge kit

const normalise = (geo: THREE.BufferGeometry): THREE.BufferGeometry => {
  let g = geo
  for (const k of Object.keys(g.attributes)) if (k !== "position" && k !== "normal" && k !== "uv") g.deleteAttribute(k)
  if (!g.attributes.normal) g.computeVertexNormals()
  if (!g.attributes.uv) g.setAttribute("uv", new THREE.Float32BufferAttribute(new Float32Array(g.attributes.position.count * 2), 2))
  if (!g.index) {
    const n = g.attributes.position.count
    const idx = new (n > 65535 ? Uint32Array : Uint16Array)(n)
    for (let i = 0; i < n; i++) idx[i] = i
    g = g.clone()
    g.setIndex(new THREE.BufferAttribute(idx, 1))
  }
  g.morphAttributes = {}
  return g
}

/** collects geometry per material and merges it into one mesh per material (one draw call each) */
export class Kit {
  private readonly bins = new Map<THREE.Material, THREE.BufferGeometry[]>()
  private readonly extra: THREE.Object3D[] = []
  add(geo: THREE.BufferGeometry, mat: THREE.Material, m?: THREE.Matrix4): this {
    const g = m ? geo.clone().applyMatrix4(m) : geo
    const list = this.bins.get(mat) ?? []
    list.push(normalise(g))
    this.bins.set(mat, list)
    return this
  }
  /** add a ready object (e.g. a mesh with its own vertex colours or an animated actuator) */
  obj(o: THREE.Object3D): this { this.extra.push(o); return this }
  build(name = ""): THREE.Group {
    const grp = new THREE.Group()
    grp.name = name
    for (const [mat, list] of this.bins) {
      const merged = list.length === 1 ? list[0] : mergeGeometries(list, false)
      if (!merged) throw new Error(`Kit.build(${name}): incompatible geometry`)
      const m = new THREE.Mesh(merged, mat)
      m.name = `${name}:${(mat as THREE.MeshStandardMaterial).name || mat.uuid.slice(0, 4)}`
      grp.add(m)
    }
    for (const o of this.extra) grp.add(o)
    return grp
  }
}

/** instanced copies of one geometry (repeated parts: bolts, vents, COPVs) */
export const instanced = (geo: THREE.BufferGeometry, mat: THREE.Material, ms: readonly THREE.Matrix4[]): THREE.InstancedMesh => {
  const im = new THREE.InstancedMesh(geo, mat, ms.length)
  ms.forEach((m, i) => im.setMatrixAt(i, m))
  im.instanceMatrix.needsUpdate = true
  im.computeBoundingSphere()
  return im
}

export const triangles = (o: THREE.Object3D): number => {
  let n = 0
  o.traverse((c) => {
    const m = c as THREE.Mesh
    if (!m.isMesh) return
    const g = m.geometry
    const t = (g.index ? g.index.count : g.attributes.position.count) / 3
    n += t * ((c as THREE.InstancedMesh).isInstancedMesh ? (c as THREE.InstancedMesh).count : 1)
  })
  return n
}

// ---------------------------------------------------------------- hull meridian (conformal parts)

export interface MeridianPoint {
  readonly x: number
  readonly r: number
  /** unit tangent along increasing arc length (toward the nose) in (x, ρ) */
  readonly tx: number
  readonly tr: number
  /** outward unit normal in (x, ρ) */
  readonly nx: number
  readonly nr: number
}

/**
 * The OML meridian parameterised by arc length s from the aft skirt plane. Conformal parts (doors,
 * antennas, electrodes, raceway, pods) are placed by (s, φ, h): h is the offset along the outward
 * normal. The ship.ts hull is a 96-gon lathe (sagitta ≤ 2.4 mm at r 4.5 m) and the PICA-X layer sits
 * 4 mm proud, so conformal parts use h ≥ 5 mm (barrel) / ≥ 10 mm (PICA region).
 */
export class Hull {
  readonly L: number
  private readonly xs: Float64Array
  private readonly ss: Float64Array
  readonly length: number
  constructor(readonly shape: Shape) {
    const L = shape.p.length
    this.L = L
    const n = 6000
    const xs = new Float64Array(n + 1), ss = new Float64Array(n + 1)
    // dense toward the tip (cosine clustering on the nose)
    for (let i = 0; i <= n; i++) {
      const t = i / n
      xs[i] = t < 0.5 ? (t / 0.5) * shape.Lc : shape.Lc + (L - shape.Lc) * Math.sin(((t - 0.5) / 0.5) * (Math.PI / 2))
    }
    xs[n] = L
    for (let i = 1; i <= n; i++) ss[i] = ss[i - 1] + Math.hypot(xs[i] - xs[i - 1], this.r(xs[i]) - this.r(xs[i - 1]))
    this.xs = xs
    this.ss = ss
    this.length = ss[n]
  }
  r(x: number): number { return this.shape.r(Math.min(Math.max(x, 0), this.L - 1e-6)) }
  /** arc length at body x */
  sOf(x: number): number {
    const xs = this.xs
    let lo = 0, hi = xs.length - 1
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (xs[m] <= x) lo = m; else hi = m }
    const f = (x - xs[lo]) / Math.max(xs[hi] - xs[lo], 1e-12)
    return this.ss[lo] + f * (this.ss[hi] - this.ss[lo])
  }
  at(s: number): MeridianPoint {
    const ss = this.ss
    const sc = Math.min(Math.max(s, 0), this.length)
    let lo = 0, hi = ss.length - 1
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (ss[m] <= sc) lo = m; else hi = m }
    const f = (sc - ss[lo]) / Math.max(ss[hi] - ss[lo], 1e-12)
    const x = this.xs[lo] + f * (this.xs[hi] - this.xs[lo])
    const r = this.r(x)
    let tx = this.xs[hi] - this.xs[lo], tr = this.r(this.xs[hi]) - this.r(this.xs[lo])
    const tl = Math.hypot(tx, tr) || 1
    tx /= tl; tr /= tl
    return { x, r, tx, tr, nx: -tr, nr: tx }
  }
  /** point at arc length s, azimuth φ, normal offset h */
  point(s: number, phi: number, h = 0): THREE.Vector3 {
    const m = this.at(s)
    const rho = m.r + h * m.nr
    return new THREE.Vector3(m.x + h * m.nx, rho * Math.cos(phi), rho * Math.sin(phi))
  }
  /** local frame: X = meridian tangent (toward the nose), Y = outward normal, Z = circumferential (+φ) */
  frame(s: number, phi: number, h = 0): THREE.Matrix4 {
    const m = this.at(s)
    const c = Math.cos(phi), sn = Math.sin(phi)
    return basis(this.point(s, phi, h), v3(m.tx, m.tr * c, m.tr * sn), v3(m.nx, m.nr * c, m.nr * sn), v3(0, -sn, c))
  }
  /** point from a local patch coordinate: du along the meridian (m), dv circumferential (m) */
  patchPoint(s0: number, phi0: number, du: number, dv: number, h: number): THREE.Vector3 {
    const m = this.at(s0 + du)
    return this.point(s0 + du, phi0 + dv / Math.max(m.r, 0.05), h)
  }
  /**
   * conformal slab: a (lenS × widthC) patch centred at (s0, φ0) whose top lies at offset h1 with side
   * walls down to h0; corners chamfered by `bevel` (m). Used for doors, antennas, electrodes, doublers.
   */
  slab(s0: number, phi0: number, lenS: number, widthC: number, h0: number, h1: number, nu = 8, nv = 8, bevel = 0): THREE.BufferGeometry {
    const hu = lenS / 2, hv = widthC / 2
    // sample rows: uniform, plus rows exactly at the bevel offsets so the chamfer stays a narrow band
    const samples = (half: number, n: number): number[] => {
      const a = Array.from({ length: n + 1 }, (_, i) => -half + (2 * half * i) / n)
      if (bevel > 0 && bevel < half) { a.push(-half + bevel, half - bevel); a.sort((p, q) => p - q) }
      return a.filter((x, i) => i === 0 || x - a[i - 1] > 1e-6)
    }
    const us = samples(hu, nu), vs = samples(hv, nv)
    const top = gridGeo(us.length - 1, vs.length - 1, (u, v) => {
      const du = us[Math.round(u * (us.length - 1))], dv = vs[Math.round(v * (vs.length - 1))]
      // bevel: the edge ring sits lower (35 % of the slab height) for a chamfered look
      const e = Math.min(hu - Math.abs(du), hv - Math.abs(dv))
      const h = bevel > 0 && e < bevel - 1e-6 ? h0 + (h1 - h0) * 0.35 : h1
      return this.patchPoint(s0, phi0, du, dv, h)
    })
    const walls: THREE.BufferGeometry[] = []
    const edge = (n: number, f: (t: number) => [number, number]): void => {
      walls.push(gridGeo(n, 1, (t, k) => { const [du, dv] = f(t); return this.patchPoint(s0, phi0, du, dv, k === 0 ? h0 : (bevel > 0 ? h0 + (h1 - h0) * 0.35 : h1)) }))
    }
    edge(nu, (t) => [-hu + t * lenS, -hv])
    edge(nv, (t) => [hu, -hv + t * widthC])
    edge(nu, (t) => [hu - t * lenS, hv])
    edge(nv, (t) => [-hu, hv - t * widthC])
    // orient walls outward: their normal must point away from the patch centre
    const c = this.patchPoint(s0, phi0, 0, 0, (h0 + h1) / 2)
    for (const w of walls) {
      const p = w.attributes.position, n = w.attributes.normal
      const mid = new THREE.Vector3().fromBufferAttribute(p, 1).sub(c)
      if (mid.dot(new THREE.Vector3().fromBufferAttribute(n, 1)) < 0) flipGeo(w)
    }
    const g = mergeGeometries([top, ...walls].map(normalise), false)
    if (!g) throw new Error("slab merge")
    return g
  }
  /**
   * sweep a cross-section along the meridian at azimuth φ: profile points (b, n) are circumferential
   * and normal offsets (m); `scale(s)` shrinks the profile (end ramps).
   */
  sweep(stations: readonly number[], phi: number, profile: readonly (readonly [number, number])[], scale: (s: number) => number = () => 1): THREE.BufferGeometry {
    return gridGeo(profile.length - 1, stations.length - 1, (u, v) => {
      const i = Math.round(u * (profile.length - 1)), j = Math.round(v * (stations.length - 1))
      const s = stations[j], k = scale(s)
      const [b, n] = profile[i]
      const m = this.at(s)
      return this.point(s, phi + (b * Math.max(k, 0.3)) / m.r, n * k + 0.002)
    }, true)
  }
}

/** reverse triangle winding and normals */
export const flipGeo = (g: THREE.BufferGeometry): THREE.BufferGeometry => {
  const idx = g.index
  if (idx) for (let i = 0; i < idx.count; i += 3) { const a = idx.getX(i + 1); idx.setX(i + 1, idx.getX(i + 2)); idx.setX(i + 2, a) }
  const n = g.attributes.normal
  if (n) for (let i = 0; i < n.count; i++) n.setXYZ(i, -n.getX(i), -n.getY(i), -n.getZ(i))
  return g
}

/** ellipsoidal head profile (apex at xApex, rim at xApex + s·depth, rim radius R), ordered by increasing x */
export const domeProfile = (xApex: number, R: number, depth: number, s: 1 | -1, n = 20): RX[] => {
  const pts: RX[] = []
  for (let i = 0; i <= n; i++) {
    const th = (i / n) * (Math.PI / 2) // 0 at the apex
    pts.push([R * Math.sin(th), xApex + s * depth * (1 - Math.cos(th))])
  }
  // aft-pointing dome (s = +1: apex is the lowest x) is already ordered by increasing x
  return s === 1 ? pts : pts.reverse()
}
/** x of an ellipsoidal head surface at radius ρ */
export const domeX = (xApex: number, R: number, depth: number, s: 1 | -1, rho: number): number => xApex + s * depth * (1 - Math.sqrt(Math.max(0, 1 - (rho / R) ** 2)))
