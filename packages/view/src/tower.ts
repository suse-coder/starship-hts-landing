import * as THREE from "three"
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js"
import { TOWER } from "@ship/tower"

/**
 * Starbase Pad 2 ("Pad B") — procedural model of the integration tower ("Mechazilla"), its carriage and
 * chopsticks, the ship quick-disconnect arm, the Pad 2 orbital launch mount over its flame trench, the
 * tank farm, pipe racks, roads, fence and lights.
 *
 * Frame: the pad's local ENU frame mapped to three.js (x = East, y = Up, z = −North); origin at the
 * centre of the launch mount at ground level (the same frame as the contact model in @ship/tower).
 *
 * Sources (tags used in the doc comments below):
 * - [T] `TOWER` in @ship/tower — the geometry the contact model uses (rails, arms, gap, mount height).
 *   Rails and arms are drawn exactly where the physics has them.
 * - [W] Wikipedia, "SpaceX Starbase" (accessed 2026-11-21): 146 m tower (FAA determination of no
 *   hazard, 2021), stacked from nine large steel sections, dark-grey steel truss, lightning rod and a
 *   small weather station on top; the arms sit on a carriage moved by a cable over a pulley at the top
 *   from a winch at the base, with bearings at the sides of the carriage; a linear hydraulic actuator
 *   moves the arms horizontally; tracks on top of the arms; Pad 2 arms are shorter; the Pad 2 mount
 *   has a water-cooled steel plate on its top level and a steel-pipe flame diverter ("flame bucket");
 *   20 hold-down clamps and a booster quick disconnect; the tank farm stores CH₄, LOX, water, N₂,
 *   He and hydraulic fluid, with subcoolers.
 * - [E] estimated from public photos / video (proportions against the 9 m vehicles).
 *
 * Performance: repeated members are InstancedMeshes grouped by (profile, material); ≈ 1.1·10⁵
 * triangles in total. Weathering is a world-scale triplanar grime texture patched into
 * MeshStandardMaterial (no UV stretching on scaled instances).
 */
export const enu = (e: number, n: number, u: number): THREE.Vector3 => new THREE.Vector3(e, u, -n)

export interface PadModel {
  readonly root: THREE.Group
  readonly setArms: (open: number) => void
}

// ===================================================================================== dimensions

/** Lattice tower. */
const TW = {
  /** tower centre (m) [T] */
  e: TOWER.pos.x,
  n: TOWER.pos.y,
  /** half side of the square lattice, outer faces (m) [E: ≈ 12 m square, ≈ 1.3 × the 9 m booster] */
  half: 6,
  /** corner column, built-up box section (m) [E] */
  col: 1.5,
  /** stacked sections [W] */
  sections: 9,
  /** top of the pile caps = column base (m) [E] */
  base: 1.2,
  /** top of the lattice (m) [E]; crown + lightning mast reach TOWER.height = 146 m [T, W] */
  top: 134,
} as const
/** column centre-line offset from the tower centre */
const CL = TW.half - TW.col / 2
/** section height ≈ 14.8 m (9 sections) */
const HS = (TW.top - TW.base) / TW.sections

/** Carriage (rides on corner rails, cable-hoisted [W]). */
const CA = {
  /** frame bottom / top (m) [E: ≈ 15 m tall frame around the arm roots] */
  z0: TOWER.armHeight - 9,
  z1: TOWER.armHeight + 6,
  /** centre-line half width of the frame ring (m) [E: 1.6 m clearance for the bearings] */
  r: TW.half + 1.6,
  girder: 1.1,
  /** half length of the arm traverse beams (m) [E: covers the open arms, gap 26 m [T] + trolley] */
  beamHalf: 16.6,
} as const
/** south face of the carriage ring (n ≈ 14.85 m) */
const RING_S = TW.n - CA.r - CA.girder / 2
/** traverse beams centre line (1.2 m deep in n) */
const BEAM_N = RING_S - 0.6
/** upper / lower traverse beam centres (m) */
const BEAM_U = [TOWER.armHeight + 1.25, TOWER.armHeight - 5.1] as const
/** trolley front face = start of the arm truss (n ≈ 12.25 m); the rail itself starts at pos.y − 2 [T] inside the carriage */
const ROOT_N = RING_S - 2.6
/** arm tip = contact model yMin = pos.y − 2 − armLength = −17 m [T] */
const TIP_N = TOWER.pos.y - 2 - TOWER.armLength
/** actuator axis (n) */
const ACT_N = ROOT_N + 0.7
/**
 * Chopstick truss beyond [T]: depth = armDepth (2.8 m) over the catch zone (n ≤ 6 m), haunched to
 * 4.6 m at the root; chords set back from the inner face (the bumpers form the inner face) [E].
 */
const ARM = { rootDepth: 4.6, haunchN: 6, oIn: 0.32, oOut: TOWER.armWidth - 0.3, panels: 11 } as const
/**
 * Arm actuators [W: linear hydraulic actuator]: crossed cylinders (the east arm's above the upper beam,
 * the west arm's below the lower beam) so that the closed length (8.4 m) ≥ the barrel (8.2 m) and
 * the stroke covers the 8.4 m travel per arm [T: (openGap − closedGap)/2].
 */
const ACT = { anchor: 3, barrel: 8.2, uE: TOWER.armHeight + 3, uW: TOWER.armHeight - 7.1 } as const

/**
 * Pad 2 launch-mount pedestal and flame trench [W: flame bucket; E: layout]. The ground at Boca Chica
 * is at sea level, so the trench is built up: a 6.5 m concrete pedestal with the trench slot running
 * south (away from the tower), floor at grade, continuing as an open walled channel.
 */
const PED = { h: 6.5, eIn: 8.5, eOut: 23, nS: -16, nN: 13, nBack: 8, toe: 10 } as const
/** open trench beyond the pedestal (m) [E] */
const TRENCH = { end: -60, wing: 10 } as const
/** flame bucket: pipe centres on n = n0 + a·cosθ, u = top − b·sinθ (vertical at the back wall, horizontal at the floor) [E] */
const BUCKET = { n0: -4, a: 12, top: 6, b: 5.4, pipe: 0.56, pitch: 0.6 } as const
/**
 * Orbital launch mount (steel) [W: 20 clamps, QD, water-cooled top plate; T: mountHeight = 20 m; E: rest].
 * Table top + plate top = TOWER.mountHeight.
 */
const OLM = {
  leg: 3.2, legE: 12.4, legN: 8.6, legTop: 16,
  top: TOWER.mountHeight, plate: 0.3, halfE: 13.6, halfN: 13, chamfer: 3,
  /** central opening ≈ booster radius 4.5 m + 0.1 m */
  hole: 4.6,
  plateR: 11.6, clampR: 5.4, clamps: 20,
} as const
/** Ship quick-disconnect arm, stowed along the south face pointing west [E: QD level ≈ mount 20 m + booster ≈ 72 m] */
const SQD = { e: TW.e - 7, n: TW.n - 7.4, u: 95, len: 20, w: 2.4, h: 3 } as const
/** Drawworks (hoist winch) house north of the tower [W: winch at the base; E: size] */
const DW = { e: TW.e, n0: TW.n + 10, n1: TW.n + 19, w: 14, h: 7.4 } as const
/** Pipe rack from the tank farm to the tower base [E] */
const RACK = { e0: -118, e1: -9, nA: 18.8, nB: 25.2, u1: 7.2, u2: 4.4, step: 7.25 } as const
/** Rack pipes (top tier): LOX, CH₄, LN₂ (insulated, continue up the tower to the SQD), GN₂, water [E] */
const PIPES = [
  { n: 19.6, dia: 1.0, mat: "insul" }, { n: 20.9, dia: 0.9, mat: "insul" }, { n: 22.0, dia: 0.6, mat: "insul" },
  { n: 22.9, dia: 0.36, mat: "stainless" }, { n: 23.8, dia: 0.5, mat: "towerDark" },
] as const
/** Tank farm west of the pad [W: CH₄/LOX/N₂/water storage, subcoolers; E: layout, sizes] */
const TF = { e: -130, n: 12, pitchE: 14, pitchN: 15, r: 5.2, h: 26, dome: 2.8, hR: 1.9, hL: 22 } as const
/** Pad perimeter: apron, road loop, fence (m) [E] */
const SITE = { apron: [-60, 60, -95, 60], road: [-105, 105, -124, 89], fence: [-240, 140, -170, 140] } as const

// ===================================================================================== members

type V = THREE.Vector3
type Prof = "box" | "ibeam" | "tube" | "ball"
type MatKey =
  | "tower" | "towerDark" | "arm" | "rail" | "chrome" | "yellow" | "safety" | "concrete" | "apron" | "gravel"
  | "soot" | "heat" | "grate" | "tank" | "insul" | "stainless" | "asphalt" | "siding" | "roof" | "lamp"
  | "red" | "cable" | "white" | "blue" | "fins" | "fabric" | "decal"
type Mats = Readonly<Record<MatKey, THREE.MeshStandardMaterial>>

/** a straight structural member from a to b (three coords); w/d = cross-section along local x/z */
interface Member {
  readonly prof: Prof
  readonly mat: MatKey
  readonly a: V
  readonly b: V
  readonly w: number
  readonly d: number
  /** reference direction for the cross-section's local z (default: up, or north for vertical members) */
  readonly ref?: V
  /** per-instance brightness (paint batches, heat tint) */
  readonly tint?: number
}

const UP = new THREE.Vector3(0, 1, 0)
const NORTH = new THREE.Vector3(0, 0, -1)
const EAST = new THREE.Vector3(1, 0, 0)
const CORNERS = [[-1, -1], [1, -1], [1, 1], [-1, 1]] as const // SW, SE, NE, NW → faces S, E, N, W

const bar = (mat: MatKey, a: V, b: V, w: number, d = w, ref?: V, tint?: number): Member => ({ prof: "box", mat, a, b, w, d, ref, tint })
const ibeam = (mat: MatKey, a: V, b: V, w: number, d: number, tint?: number): Member => ({ prof: "ibeam", mat, a, b, w, d, tint })
const tube = (mat: MatKey, a: V, b: V, dia: number, tint?: number): Member => ({ prof: "tube", mat, a, b, w: dia, d: dia, tint })
const ball = (mat: MatKey, p: V, dia: number): Member => ({ prof: "ball", mat, a: p.clone().addScaledVector(UP, -dia / 2), b: p.clone().addScaledVector(UP, dia / 2), w: dia, d: dia })
/** axis-aligned block from ENU bounds */
const block = (mat: MatKey, e0: number, e1: number, n0: number, n1: number, u0: number, u1: number, tint?: number): Member => {
  const e = (e0 + e1) / 2, n = (n0 + n1) / 2
  return { prof: "box", mat, a: enu(e, n, Math.min(u0, u1)), b: enu(e, n, Math.max(u0, u1)), w: Math.abs(e1 - e0), d: Math.abs(n1 - n0), tint }
}
/** axis-aligned block from its centre and size */
const box = (mat: MatKey, e: number, n: number, u: number, se: number, sn: number, su: number, tint?: number): Member =>
  block(mat, e - se / 2, e + se / 2, n - sn / 2, n + sn / 2, u - su / 2, u + su / 2, tint)
/** polyline pipe: straight tubes with ball elbows at the interior vertices */
const pipeRun = (mat: MatKey, pts: readonly V[], dia: number): Member[] => {
  const out: Member[] = []
  for (let i = 1; i < pts.length; i++) out.push(tube(mat, pts[i - 1], pts[i], dia))
  for (let i = 1; i < pts.length - 1; i++) out.push(ball(mat, pts[i], dia))
  return out
}

const _x = new THREE.Vector3(), _y = new THREE.Vector3(), _z = new THREE.Vector3(), _c = new THREE.Vector3()
/** instance matrix: unit profile (length along local y, section in x/z) stretched between a and b */
const memberMatrix = (m: Member, out: THREE.Matrix4): THREE.Matrix4 => {
  _y.subVectors(m.b, m.a)
  const len = _y.length()
  _y.multiplyScalar(1 / Math.max(len, 1e-9))
  const ref = m.ref ?? (Math.abs(_y.y) > 0.95 ? NORTH : UP)
  _x.crossVectors(_y, ref)
  if (_x.lengthSq() < 1e-10) _x.crossVectors(_y, EAST)
  _x.normalize()
  _z.crossVectors(_x, _y)
  out.makeBasis(_x.multiplyScalar(m.w), _y.multiplyScalar(len), _z.multiplyScalar(m.d))
  return out.setPosition(_c.addVectors(m.a, m.b).multiplyScalar(0.5))
}

/** unit profiles: box, I-beam (flanges ⟂ local z), tube, ball */
const makeProfiles = (): Readonly<Record<Prof, THREE.BufferGeometry>> => ({
  box: new THREE.BoxGeometry(1, 1, 1),
  ibeam: mergeGeometries([
    new THREE.BoxGeometry(1, 1, 0.14).translate(0, 0, 0.43),
    new THREE.BoxGeometry(1, 1, 0.14).translate(0, 0, -0.43),
    new THREE.BoxGeometry(0.16, 1, 0.74),
  ]),
  tube: new THREE.CylinderGeometry(0.5, 0.5, 1, 14, 1),
  ball: new THREE.SphereGeometry(0.5, 12, 8),
})

/** group members by (profile, material) into InstancedMeshes */
const instance = (members: readonly Member[], mats: Mats, geos: Readonly<Record<Prof, THREE.BufferGeometry>>): THREE.InstancedMesh[] => {
  const groups = new Map<string, { readonly prof: Prof; readonly mat: MatKey; readonly list: Member[] }>()
  for (const m of members) {
    const k = `${m.prof}|${m.mat}`
    const g = groups.get(k)
    if (g) g.list.push(m)
    else groups.set(k, { prof: m.prof, mat: m.mat, list: [m] })
  }
  const out: THREE.InstancedMesh[] = []
  const mtx = new THREE.Matrix4(), col = new THREE.Color()
  for (const [k, { prof, mat, list }] of groups) {
    const im = new THREE.InstancedMesh(geos[prof], mats[mat], list.length)
    const tinted = list.some((m) => m.tint !== undefined)
    list.forEach((m, i) => {
      im.setMatrixAt(i, memberMatrix(m, mtx))
      if (tinted) im.setColorAt(i, col.setScalar(m.tint ?? 1))
    })
    im.instanceMatrix.needsUpdate = true
    im.castShadow = mat !== "fabric" && mat !== "decal" && mat !== "asphalt"
    im.receiveShadow = true
    im.name = k
    out.push(im)
  }
  return out
}

// ===================================================================================== textures & materials

/** deterministic PRNG (mulberry32) so the pad looks the same on every load */
const rng = (seed: number): (() => number) => {
  let a = seed >>> 0
  return () => {
    a = (a + 0x6d2b79f5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

const canvasTexture = (w: number, h: number, srgb: boolean, paint: (g: CanvasRenderingContext2D) => void): THREE.CanvasTexture => {
  const c = document.createElement("canvas")
  c.width = w
  c.height = h
  const g = c.getContext("2d")
  if (!g) throw new Error("pad textures: 2D canvas unavailable")
  paint(g)
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = srgb ? THREE.SRGBColorSpace : THREE.NoColorSpace
  t.anisotropy = 8
  return t
}

/** draw at the nine wrap offsets so the texture tiles seamlessly */
const wrap9 = (w: number, h: number, f: (dx: number, dy: number) => void): void => {
  for (const dx of [-w, 0, w]) for (const dy of [-h, 0, h]) f(dx, dy)
}

const speckle = (g: CanvasRenderingContext2D, w: number, h: number, rand: () => number, amp: number): void => {
  const img = g.getImageData(0, 0, w, h)
  for (let p = 0; p < img.data.length; p += 4) {
    const d = (rand() - 0.5) * amp
    img.data[p] += d
    img.data[p + 1] += d
    img.data[p + 2] += d
  }
  g.putImageData(img, 0, 0)
}

/**
 * Weathering detail (linear data, mean ≈ 0.5): soft blotches, downward rust (steel) or water (concrete)
 * streaks, speckle. Canvas top = up, so streaks run down the structure.
 */
const grimeTexture = (rand: () => number, rust: boolean): THREE.CanvasTexture => canvasTexture(512, 512, false, (g) => {
  g.fillStyle = "rgb(128,128,128)"
  g.fillRect(0, 0, 512, 512)
  for (let i = 0; i < 110; i++) {
    const x = rand() * 512, y = rand() * 512, r = 20 + rand() * 80, k = rand() < 0.5 ? 0 : 255, a = 0.05 + rand() * 0.08
    wrap9(512, 512, (dx, dy) => {
      const gr = g.createRadialGradient(x + dx, y + dy, 0, x + dx, y + dy, r)
      gr.addColorStop(0, `rgba(${k},${k},${k},${a})`)
      gr.addColorStop(1, `rgba(${k},${k},${k},0)`)
      g.fillStyle = gr
      g.fillRect(x + dx - r, y + dy - r, 2 * r, 2 * r)
    })
  }
  const col = rust ? "118,80,56" : "74,72,66"
  for (let i = 0; i < (rust ? 240 : 150); i++) {
    const x = rand() * 512, y = rand() * 512, len = 30 + rand() * 200, wd = 1 + rand() * 4, a = 0.12 + rand() * 0.28
    wrap9(512, 512, (dx, dy) => {
      const gr = g.createLinearGradient(0, y + dy, 0, y + dy + len)
      gr.addColorStop(0, `rgba(${col},${a})`)
      gr.addColorStop(1, `rgba(${col},0)`)
      g.fillStyle = gr
      g.fillRect(x + dx, y + dy, wd, len)
    })
  }
  speckle(g, 512, 512, rand, 26)
})

/** welded tank shell: ring seams (≈ 2 m), slight ring-to-ring tone variation, rain streaks */
const tankTexture = (rand: () => number): THREE.CanvasTexture => canvasTexture(256, 1024, true, (g) => {
  const rings = 14
  for (let i = 0; i < rings; i++) {
    const y0 = (i / rings) * 1024, k = 226 + rand() * 20
    g.fillStyle = `rgb(${k},${k + 1},${k + 3})`
    g.fillRect(0, y0, 256, 1024 / rings + 1)
    g.fillStyle = "rgba(120,124,128,0.75)"
    g.fillRect(0, y0, 256, 2)
  }
  for (let i = 0; i < 70; i++) {
    const x = rand() * 256, y = rand() * 1024, len = 40 + rand() * 300
    const gr = g.createLinearGradient(0, y, 0, y + len)
    gr.addColorStop(0, "rgba(140,128,108,0.16)")
    gr.addColorStop(1, "rgba(140,128,108,0)")
    g.fillStyle = gr
    g.fillRect(x, y, 1 + rand() * 3, len)
  }
})

/** yellow/black 45° safety stripes (bumpers, tip markers) */
const stripeTexture = (): THREE.CanvasTexture => canvasTexture(128, 128, true, (g) => {
  g.fillStyle = "#efbd14"
  g.fillRect(0, 0, 128, 128)
  g.fillStyle = "#1a1a1a"
  for (let k = -3; k <= 3; k++) {
    g.beginPath()
    g.moveTo(k * 64, 0)
    g.lineTo(k * 64 + 32, 0)
    g.lineTo(k * 64 + 32 + 128, 128)
    g.lineTo(k * 64 + 128, 128)
    g.closePath()
    g.fill()
  }
})

/** corrugated metal siding (vertical ribs) */
const sidingTexture = (): THREE.CanvasTexture => canvasTexture(64, 8, true, (g) => {
  for (let x = 0; x < 64; x++) {
    const k = Math.round(196 + 34 * Math.sin((x / 16) * Math.PI * 2))
    g.fillStyle = `rgb(${k},${k + 4},${k + 8})`
    g.fillRect(x, 0, 1, 8)
  }
})

/** ambient-air vaporiser fins */
const finTexture = (): THREE.CanvasTexture => canvasTexture(32, 8, true, (g) => {
  for (let x = 0; x < 32; x++) {
    const k = x % 8 < 2 ? 120 : 212 - (x % 8) * 6
    g.fillStyle = `rgb(${k},${k + 3},${k + 6})`
    g.fillRect(x, 0, 1, 8)
  }
})

/** concrete apron: 2 × 2 slabs (6 m) with tone variation, saw-cut joints, stains */
const jointTexture = (rand: () => number): THREE.CanvasTexture => canvasTexture(512, 512, true, (g) => {
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) {
    const k = 162 + rand() * 18
    g.fillStyle = `rgb(${k},${k - 4},${k - 11})`
    g.fillRect(i * 256, j * 256, 256, 256)
  }
  for (let i = 0; i < 40; i++) {
    const x = rand() * 512, y = rand() * 512, r = 10 + rand() * 60
    wrap9(512, 512, (dx, dy) => {
      const gr = g.createRadialGradient(x + dx, y + dy, 0, x + dx, y + dy, r)
      gr.addColorStop(0, `rgba(60,56,50,${0.06 + rand() * 0.1})`)
      gr.addColorStop(1, "rgba(60,56,50,0)")
      g.fillStyle = gr
      g.fillRect(x + dx - r, y + dy - r, 2 * r, 2 * r)
    })
  }
  speckle(g, 512, 512, rand, 18)
  g.fillStyle = "rgba(70,66,60,0.85)"
  for (const p of [0, 256]) {
    g.fillRect(p, 0, 3, 512)
    g.fillRect(0, p, 512, 3)
  }
})

/** soot decal along the trench (alpha): dark near the bucket, fading downstream and to the sides */
const sootTexture = (rand: () => number): THREE.CanvasTexture => {
  const t = canvasTexture(128, 512, true, (g) => {
    const img = g.createImageData(128, 512)
    for (let y = 0; y < 512; y++) for (let x = 0; x < 128; x++) {
      const u = x / 127, v = 1 - y / 511 // v = 1 at the bucket end (north)
      const side = Math.max(0, 1 - Math.pow(Math.abs(2 * u - 1) / (0.45 + 0.55 * (1 - v)), 2))
      const a = side * (0.25 + 0.65 * Math.pow(v, 0.7)) * (0.8 + 0.4 * rand())
      const p = (y * 128 + x) * 4
      img.data[p] = 30
      img.data[p + 1] = 27
      img.data[p + 2] = 24
      img.data[p + 3] = Math.min(255, a * 255)
    }
    g.putImageData(img, 0, 0)
  })
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping
  return t
}

/**
 * World-scale triplanar weathering patched into MeshStandardMaterial: albedo × grime, roughness ±.
 * Coordinates are the mesh's local (instance) space, so the pattern sticks to moving parts (arms)
 * and is unaffected by the floating origin. Standard materials keep three's log-depth chunks.
 */
const weather = (m: THREE.MeshStandardMaterial, tex: THREE.Texture, metres: number, amount: number): THREE.MeshStandardMaterial => {
  m.onBeforeCompile = (sh) => {
    sh.uniforms.uWTex = { value: tex }
    sh.uniforms.uWScale = { value: 1 / metres }
    sh.uniforms.uWAmt = { value: amount }
    sh.vertexShader = sh.vertexShader
      .replace("#include <common>", "#include <common>\nvarying vec3 vWP;\nvarying vec3 vWN;")
      .replace("#include <project_vertex>", `#include <project_vertex>
        vec4 wP = vec4( transformed, 1.0 );
        vec3 wN = objectNormal;
        #ifdef USE_INSTANCING
          wP = instanceMatrix * wP;
          wN = mat3( instanceMatrix ) * wN;
        #endif
        vWP = wP.xyz;
        vWN = wN;`)
    sh.fragmentShader = sh.fragmentShader
      .replace("#include <common>", "#include <common>\nvarying vec3 vWP;\nvarying vec3 vWN;\nuniform sampler2D uWTex;\nuniform float uWScale;\nuniform float uWAmt;")
      .replace("#include <color_fragment>", `#include <color_fragment>
        vec3 wB = pow( abs( normalize( vWN ) ), vec3( 4.0 ) );
        wB /= max( wB.x + wB.y + wB.z, 1e-4 );
        vec3 wQ = vWP * uWScale;
        vec3 wT = texture( uWTex, wQ.zy ).rgb * wB.x + texture( uWTex, wQ.xz ).rgb * wB.y + texture( uWTex, wQ.xy ).rgb * wB.z;
        diffuseColor.rgb *= mix( vec3( 1.0 ), wT * 2.0, uWAmt );`)
      .replace("#include <roughnessmap_fragment>", `#include <roughnessmap_fragment>
        roughnessFactor = clamp( roughnessFactor + ( 0.5 - wT.g ) * 0.5 * uWAmt, 0.05, 1.0 );`)
  }
  m.customProgramCacheKey = () => "pad-weather-v1"
  return m
}

/** PBR palette: dark-grey tower steel [W], primer-white arms, rails, concrete, tanks, pipes */
const makeMaterials = (rand: () => number): Mats => {
  const rust = grimeTexture(rand, true)
  const water = grimeTexture(rand, false)
  const std = (p: THREE.MeshStandardMaterialParameters): THREE.MeshStandardMaterial => new THREE.MeshStandardMaterial(p)
  const siding = sidingTexture()
  siding.repeat.set(6, 1)
  const fins = finTexture()
  fins.repeat.set(3, 1)
  const joints = jointTexture(rand)
  joints.repeat.set(10, 12.9)
  return {
    tower: weather(std({ color: 0x6f7479, metalness: 0.55, roughness: 0.6 }), rust, 7, 0.6),
    towerDark: weather(std({ color: 0x4d5258, metalness: 0.6, roughness: 0.55 }), rust, 7, 0.5),
    arm: weather(std({ color: 0xd2d5d7, metalness: 0.25, roughness: 0.5 }), rust, 6, 0.35),
    rail: weather(std({ color: 0x575c61, metalness: 0.85, roughness: 0.32 }), rust, 5, 0.3),
    chrome: std({ color: 0xe4e8ea, metalness: 1, roughness: 0.12 }),
    yellow: std({ map: stripeTexture(), metalness: 0.1, roughness: 0.55 }),
    safety: std({ color: 0xe8b81a, metalness: 0.2, roughness: 0.5 }),
    concrete: weather(std({ color: 0xaaa498, roughness: 0.93 }), water, 9, 0.6),
    apron: weather(std({ map: joints, roughness: 0.92 }), water, 14, 0.4),
    gravel: weather(std({ color: 0xa39c8c, roughness: 1 }), water, 6, 0.7),
    soot: weather(std({ color: 0x48443e, roughness: 0.95 }), water, 6, 0.7),
    heat: weather(std({ color: 0x84786c, metalness: 0.75, roughness: 0.42 }), rust, 3, 0.6),
    grate: weather(std({ color: 0x8e9397, metalness: 0.6, roughness: 0.55 }), rust, 5, 0.3),
    tank: std({ map: tankTexture(rand), metalness: 0.35, roughness: 0.38 }),
    insul: weather(std({ color: 0xeeeeea, roughness: 0.65 }), water, 5, 0.35),
    stainless: std({ color: 0xc5cbd0, metalness: 0.9, roughness: 0.25 }),
    asphalt: weather(std({ color: 0x3d3f42, roughness: 0.95 }), water, 10, 0.5),
    siding: std({ map: siding, metalness: 0.4, roughness: 0.5 }),
    roof: weather(std({ color: 0x7c8187, metalness: 0.5, roughness: 0.55 }), water, 6, 0.4),
    lamp: std({ color: 0xfff6e0, emissive: 0xfff1d0, emissiveIntensity: 0.6 }),
    red: std({ color: 0xff2a1a, emissive: 0xff2010, emissiveIntensity: 2 }),
    cable: std({ color: 0x2b2d2f, metalness: 0.8, roughness: 0.4 }),
    white: weather(std({ color: 0xe9ebec, metalness: 0.2, roughness: 0.45 }), water, 4, 0.3),
    blue: weather(std({ color: 0x35557e, metalness: 0.3, roughness: 0.5 }), water, 4, 0.3),
    fins: std({ map: fins, metalness: 0.7, roughness: 0.35 }),
    fabric: std({ color: 0x8a8f94, metalness: 0.6, roughness: 0.6, transparent: true, opacity: 0.22, side: THREE.DoubleSide, depthWrite: false }),
    decal: std({ map: sootTexture(rand), roughness: 0.95, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2 }),
  }
}

// ===================================================================================== geometry helpers

/** extruded 2-D shape (x_s, y_s) along +z_s */
const extrude = (pts: readonly (readonly [number, number])[], depth: number, holes: readonly THREE.Path[] = [], bevel = 0, curveSegments = 12): THREE.ExtrudeGeometry => {
  const s = new THREE.Shape(pts.map(([x, y]) => new THREE.Vector2(x, y)))
  s.holes.push(...holes)
  return new THREE.ExtrudeGeometry(s, { depth, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 1, curveSegments })
}

/** half cylinder with the round side up, axis along East ("e") or North ("n") */
const halfCylinder = (r: number, len: number, axis: "e" | "n"): THREE.BufferGeometry =>
  axis === "e"
    ? new THREE.CylinderGeometry(r, r, len, 16, 1, false, 0, Math.PI).rotateZ(Math.PI / 2)
    : new THREE.CylinderGeometry(r, r, len, 16, 1, false, -Math.PI / 2, Math.PI).rotateX(-Math.PI / 2)

/** vertical tank shell (lathe): base skirt, cylinder, elliptical dome; v ∝ height for the seam texture */
const tankGeometry = (r: number, h: number, dome: number, seg: number): THREE.BufferGeometry => {
  const pts = [new THREE.Vector2(r + 0.18, 0), new THREE.Vector2(r + 0.18, 0.7), new THREE.Vector2(r, 0.75), new THREE.Vector2(r, h)]
  for (let i = 1; i <= 8; i++) {
    const t = (i / 8) * (Math.PI / 2)
    pts.push(new THREE.Vector2(r * Math.cos(t), h + dome * Math.sin(t)))
  }
  const g = new THREE.LatheGeometry(pts, seg)
  const pos = g.attributes.position, uv = g.attributes.uv
  for (let i = 0; i < uv.count; i++) uv.setY(i, pos.getY(i) / (h + dome))
  return g
}

/** horizontal capsule tank (lathe, axis along North), v ∝ axial position */
const capsuleGeometry = (r: number, len: number, seg: number): THREE.BufferGeometry => {
  const pts: THREE.Vector2[] = []
  for (let i = 0; i <= 6; i++) { const t = -Math.PI / 2 + (i / 6) * (Math.PI / 2); pts.push(new THREE.Vector2(r * Math.cos(t), -len / 2 + r + r * Math.sin(t))) }
  for (let i = 0; i <= 6; i++) { const t = (i / 6) * (Math.PI / 2); pts.push(new THREE.Vector2(r * Math.cos(t), len / 2 - r + r * Math.sin(t))) }
  const g = new THREE.LatheGeometry(pts, seg)
  const pos = g.attributes.position, uv = g.attributes.uv
  for (let i = 0; i < uv.count; i++) uv.setY(i, (pos.getY(i) + len / 2) / len)
  return g.rotateX(Math.PI / 2)
}

const instancedAt = (geo: THREE.BufferGeometry, mat: THREE.Material, at: readonly V[], tints?: readonly number[]): THREE.InstancedMesh => {
  const im = new THREE.InstancedMesh(geo, mat, at.length)
  const m = new THREE.Matrix4(), c = new THREE.Color()
  at.forEach((p, i) => {
    im.setMatrixAt(i, m.makeTranslation(p.x, p.y, p.z))
    if (tints) im.setColorAt(i, c.setScalar(tints[i] ?? 1))
  })
  im.castShadow = true
  im.receiveShadow = true
  return im
}

const shadowed = <T extends THREE.Mesh>(m: T, cast = true): T => {
  m.castShadow = cast
  m.receiveShadow = true
  return m
}

// ===================================================================================== tower

/**
 * Lattice: per section four corner columns (Ø-box 1.5 m), I-beam girders at the splice (1.0 m deep)
 * and mid-height (0.75 m), X-bracing (0.42 m square tubes) in both half panels of every face with
 * gusset plates at the crossings, plan "diamond" bracing at the splices, splice collars [E]. Each
 * section gets its own paint shade (sections are fabricated and painted separately).
 */
const towerLattice = (rand: () => number): Member[] => {
  const out: Member[] = []
  const at = (de: number, dn: number, u: number): V => enu(TW.e + de, TW.n + dn, u)
  for (let k = 0; k < TW.sections; k++) {
    const z0 = TW.base + k * HS, z1 = z0 + HS, last = k === TW.sections - 1
    const tint = 0.8 + 0.22 * rand()
    const lv = [z0 + 0.5, z0 + HS / 2, last ? z1 - 0.5 : z1 + 0.5]
    for (const [cx, cy] of CORNERS) {
      out.push(bar("tower", at(cx * CL, cy * CL, z0), at(cx * CL, cy * CL, z1), TW.col, TW.col, undefined, tint))
      if (k > 0) out.push(box("towerDark", TW.e + cx * CL, TW.n + cy * CL, z0, TW.col + 0.36, TW.col + 0.36, 0.7))
    }
    for (let c = 0; c < 4; c++) {
      const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
      const A = (u: number): V => at(ax * CL, ay * CL, u), B = (u: number): V => at(bx * CL, by * CL, u)
      ;(last ? lv : lv.slice(0, 2)).forEach((z, j) => out.push(ibeam("tower", A(z), B(z), j === 1 ? 0.42 : 0.5, j === 1 ? 0.75 : 1.0, tint)))
      for (let j = 0; j < 2; j++) {
        out.push(bar("tower", A(lv[j]), B(lv[j + 1]), 0.42, 0.42, undefined, tint), bar("tower", B(lv[j]), A(lv[j + 1]), 0.42, 0.42, undefined, tint))
        const me = TW.e + ((ax + bx) / 2) * CL, mn = TW.n + ((ay + by) / 2) * CL, zm = (lv[j] + lv[j + 1]) / 2
        out.push(ay === by ? box("towerDark", me, mn, zm, 1.5, 0.62, 1.5) : box("towerDark", me, mn, zm, 0.62, 1.5, 1.5))
      }
    }
    if (k > 0) {
      const mids = [[CL, 0], [0, CL], [-CL, 0], [0, -CL]] as const
      for (let i = 0; i < 4; i++) {
        const [a1, a2] = mids[i], [b1, b2] = mids[(i + 1) % 4]
        out.push(ibeam("tower", at(a1, a2, lv[0]), at(b1, b2, lv[0]), 0.36, 0.6, tint))
      }
    }
  }
  // carriage rails on the four corners [W: bearings at the sides], cable trays on the north face [E]
  for (const [cx, cy] of CORNERS) out.push(box("rail", TW.e + cx * (TW.half + 0.1), TW.n + cy * (TW.half + 0.1), (TW.base + TW.top) / 2 + 3, 0.4, 0.4, TW.top - TW.base - 6))
  for (const de of [-1.6, 1.6]) out.push(box("grate", TW.e + de, TW.n + TW.half + 0.2, (TW.base + TW.top) / 2, 0.8, 0.25, TW.top - TW.base))
  // pile caps under the columns
  for (const [cx, cy] of CORNERS) out.push(box("concrete", TW.e + cx * CL, TW.n + cy * CL, TW.base / 2, 4, 4, TW.base))
  // floodlight brackets on the south face (lighting the mount and the catch zone) [E]
  for (const u of [TW.base + 2.5 * HS, TW.base + 5.5 * HS, TOWER.armHeight - 14]) for (const de of [-3.6, 3.6]) {
    out.push(box("towerDark", TW.e + de, TW.n - TW.half - 0.45, u, 1.6, 0.9, 0.25))
    out.push(box("lamp", TW.e + de, TW.n - TW.half - 0.95, u + 0.45, 1.3, 0.12, 0.7))
  }
  return out
}

/**
 * Internal core [E]: switch-back stair (landings every ≈ 3.7 m, 4 per section) between four core
 * columns, an elevator on guide rails west of the stair, grated service platforms at two splices.
 */
const towerCore = (): Member[] => {
  const out: Member[] = []
  const e0 = TW.e, n0 = TW.n, ce = 2, cn = 2.9, H = TW.top - TW.base, mid = (TW.base + TW.top) / 2
  for (const [sx, sy] of CORNERS) out.push(box("grate", e0 + sx * ce, n0 + sy * cn, mid, 0.3, 0.3, H))
  const hf = HS / 4
  for (let f = 0; f < TW.sections * 4; f++) {
    const z = TW.base + f * hf + 0.95
    out.push(box("grate", e0, n0 - cn + 0.6, z, 2 * ce, 1.2, 0.18), box("grate", e0, n0 + cn - 0.6, z + hf / 2, 2 * ce, 1.2, 0.18))
    out.push(bar("grate", enu(e0 - ce / 2, n0 - cn + 1.2, z), enu(e0 - ce / 2, n0 + cn - 1.2, z + hf / 2), ce - 0.15, 0.16))
    out.push(bar("grate", enu(e0 + ce / 2, n0 + cn - 1.2, z + hf / 2), enu(e0 + ce / 2, n0 - cn + 1.2, z + hf), ce - 0.15, 0.16))
  }
  for (const dn of [-1.3, 1.3]) out.push(box("rail", e0 - 3.1, n0 + dn, mid, 0.16, 0.16, H))
  out.push(box("white", e0 - 3.1, n0, 64, 1.7, 2.3, 2.7))
  for (const k of [3, 6]) {
    const z = TW.base + k * HS + 1.1
    out.push(box("grate", e0 + (ce + 4.5) / 2, n0, z, 4.5 - ce, 9, 0.22))
    out.push(box("grate", e0 + (ce - 4.5) / 2, n0 + (cn + 4.5) / 2, z, 4.5 + ce, 4.5 - cn, 0.22))
    out.push(box("grate", e0 + (ce - 4.5) / 2, n0 - (cn + 4.5) / 2, z, 4.5 + ce, 4.5 - cn, 0.22))
  }
  return out
}

/**
 * Crown [W: pulley at the top, lightning rod, weather station]: top deck, two outrigger beams over the
 * south face carrying the hoist sheaves (Ø 3.2 m) above the carriage, back sheaves at the north edge.
 * The lightning mast (separate mesh) reaches TOWER.height.
 */
const crown = (): Member[] => {
  const out: Member[] = []
  const T = TW.top, sU = T + 2.6, nF = TW.n - CA.r, nB = TW.n + CL + 0.8
  out.push(box("towerDark", TW.e, TW.n, T + 0.35, 2 * TW.half + 0.6, 2 * TW.half + 0.6, 0.7))
  for (const s of [-1, 1]) {
    const e = TW.e + 3 * s
    out.push(ibeam("towerDark", enu(e, nB + 0.6, T + 1.3), enu(e, nF - 1.2, T + 1.3), 0.6, 1.2))
    out.push(bar("towerDark", enu(e, TW.n - CL, T - HS / 2 + 0.5), enu(e, nF - 0.6, T + 0.8), 0.45))
    for (const n of [nF, nB]) {
      out.push(tube("rail", enu(e - 0.3, n, sU), enu(e + 0.3, n, sU), 3.2))
      out.push(box("towerDark", e - 0.55, n, sU - 0.6, 0.18, 1.2, 2), box("towerDark", e + 0.55, n, sU - 0.6, 0.18, 1.2, 2))
    }
  }
  out.push(box("towerDark", TW.e, TW.n, T + 1.15, 1.4, 1.4, 0.9))
  const we = TW.e + 4.6, wn = TW.n + 4.6
  out.push(tube("grate", enu(we, wn, T + 0.7), enu(we, wn, T + 5.2), 0.14), bar("grate", enu(we - 0.9, wn, T + 5), enu(we + 0.9, wn, T + 5), 0.06), box("white", we, wn, T + 5.35, 0.3, 0.3, 0.3))
  for (const s of [-1, 1]) out.push(box("white", we + s * 0.9, wn, T + 5.15, 0.22, 0.22, 0.2))
  return out
}

/** hoist cables: crown sheave → carriage sheave, over the top, down to the drawworks [W] */
const hoistCables = (): Member[] => {
  const out: Member[] = []
  const sU = TW.top + 2.6, nF = TW.n - CA.r, nB = TW.n + CL + 0.8, r = 1.6, cU = CA.z1 + 1.9
  for (const s of [-1, 1]) {
    const e = TW.e + 3 * s
    out.push(tube("cable", enu(e, nF - r, sU), enu(e, nF - r, cU), 0.16))
    out.push(tube("cable", enu(e, nF, sU + r), enu(e, nB, sU + r), 0.16))
    out.push(tube("cable", enu(e, nB + r, sU), enu(e, DW.n0 + 0.8, DW.h + 0.6), 0.16))
  }
  return out
}

/**
 * Carriage [W, E]: box frame around the tower (three ring levels, corner + mid posts, X-bracing),
 * roller bogies on the corner rails, grated top deck with the arm HPUs, hoist sheaves, the two
 * traverse beams the arm trolleys slide on, knee braces, and the fixed actuator barrels.
 */
const carriage = (): Member[] => {
  const out: Member[] = []
  const R = CA.r, g = CA.girder
  const at = (de: number, dn: number, u: number): V => enu(TW.e + de, TW.n + dn, u)
  const lv = [CA.z0 + g / 2, TOWER.armHeight - 1.5, CA.z1 - g / 2]
  for (let c = 0; c < 4; c++) {
    const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
    const mx = (ax + bx) / 2, my = (ay + by) / 2
    for (const z of lv) out.push(bar("towerDark", at(ax * R, ay * R, z), at(bx * R, by * R, z), g, g))
    out.push(bar("towerDark", at(ax * R, ay * R, CA.z0), at(ax * R, ay * R, CA.z1), g, g))
    out.push(bar("towerDark", at(mx * R, my * R, CA.z0), at(mx * R, my * R, CA.z1), 0.8, 0.8))
    const bays = [[ax, ay, mx, my], [mx, my, bx, by]] as const
    for (let t = 0; t < 2; t++) for (const [p0, p1, q0, q1] of bays) {
      out.push(bar("towerDark", at(p0 * R, p1 * R, lv[t]), at(q0 * R, q1 * R, lv[t + 1]), 0.45))
      out.push(bar("towerDark", at(q0 * R, q1 * R, lv[t]), at(p0 * R, p1 * R, lv[t + 1]), 0.45))
    }
  }
  for (const [cx, cy] of CORNERS) for (const z of [CA.z0 + 1.8, CA.z1 - 1.8]) out.push(box("rail", TW.e + cx * 6.7, TW.n + cy * 6.7, z, 1, 1, 2.4))
  const di = TW.half + 0.25, dO = R + g / 2, dz0 = CA.z1 - 0.2, dz1 = CA.z1
  out.push(
    block("grate", TW.e - dO, TW.e + dO, TW.n + di, TW.n + dO, dz0, dz1), block("grate", TW.e - dO, TW.e + dO, TW.n - dO, TW.n - di, dz0, dz1),
    block("grate", TW.e + di, TW.e + dO, TW.n - di, TW.n + di, dz0, dz1), block("grate", TW.e - dO, TW.e - di, TW.n - di, TW.n + di, dz0, dz1),
  )
  // deck handrails on the outer edge
  for (let c = 0; c < 4; c++) {
    const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
    for (const du of [0.55, 1.1]) out.push(bar("safety", at(ax * (dO - 0.1), ay * (dO - 0.1), CA.z1 + du), at(bx * (dO - 0.1), by * (dO - 0.1), CA.z1 + du), 0.06))
    for (let k = 0; k < 8; k++) {
      const t = k / 8
      out.push(box("safety", TW.e + (ax + (bx - ax) * t) * (dO - 0.1), TW.n + (ay + (by - ay) * t) * (dO - 0.1), CA.z1 + 0.55, 0.07, 0.07, 1.1))
    }
  }
  for (const s of [-1, 1]) {
    out.push(box("white", TW.e + s * 3.2, TW.n + 7.2, CA.z1 + 1.2, 3, 1.8, 2.4))
    out.push(tube("rail", at(s * 3 - 0.3, -R, CA.z1 + 1.9), at(s * 3 + 0.3, -R, CA.z1 + 1.9), 3.2))
    out.push(box("towerDark", TW.e + s * 3 - 0.55, TW.n - R, CA.z1 + 1.1, 0.18, 1.4, 2.2), box("towerDark", TW.e + s * 3 + 0.55, TW.n - R, CA.z1 + 1.1, 0.18, 1.4, 2.2))
  }
  // traverse beams, end posts, knee braces (attached at the back face, clear of the trolley shoes)
  for (const u of BEAM_U) out.push(box("towerDark", TW.e, BEAM_N, u, 2 * CA.beamHalf, 1.2, 1.3))
  for (const s of [-1, 1]) {
    out.push(block("towerDark", TW.e + s * (CA.beamHalf - 0.6), TW.e + s * CA.beamHalf, BEAM_N - 0.6, BEAM_N + 0.6, BEAM_U[1] - 0.65, BEAM_U[0] + 0.65))
    out.push(bar("towerDark", at(s * R, -R, lv[0]), enu(TW.e + s * 14.4, BEAM_N + 0.6, BEAM_U[1] - 0.65), 0.6))
    out.push(bar("towerDark", at(s * R, -R, lv[2]), enu(TW.e + s * 14.4, BEAM_N + 0.6, BEAM_U[0] + 0.65), 0.5))
  }
  // actuator anchors and barrels (rods are animated meshes)
  out.push(block("towerDark", TW.e - ACT.anchor - 0.45, TW.e - ACT.anchor + 0.45, ACT_N - 0.35, BEAM_N + 0.6, BEAM_U[0] + 0.65, ACT.uE + 0.5))
  out.push(block("towerDark", TW.e + ACT.anchor - 0.45, TW.e + ACT.anchor + 0.45, ACT_N - 0.35, BEAM_N + 0.6, ACT.uW - 0.5, BEAM_U[1] - 0.65))
  out.push(tube("arm", enu(TW.e - ACT.anchor, ACT_N, ACT.uE), enu(TW.e - ACT.anchor + ACT.barrel, ACT_N, ACT.uE), 0.66))
  out.push(tube("arm", enu(TW.e + ACT.anchor, ACT_N, ACT.uW), enu(TW.e + ACT.anchor - ACT.barrel, ACT_N, ACT.uW), 0.66))
  return out
}

/** bottom of the arm truss at station n: armDepth over the catch zone, haunched toward the root */
const armBottom = (n: number): number =>
  TOWER.armHeight - (n <= ARM.haunchN ? TOWER.armDepth : TOWER.armDepth + ((n - ARM.haunchN) / (ROOT_N - ARM.haunchN)) * (ARM.rootDepth - TOWER.armDepth))

/**
 * One chopstick (s = −1 west, +1 east) in its group frame (group x = inner face, e = s·o with o the
 * outward offset). Top of the rail = TOWER.armHeight, inner face (bumpers) at o = 0, width
 * TOWER.armWidth, depth TOWER.armDepth over the catch zone, tip at the contact model's yMin [T].
 * Rectangular Warren truss (4 chords, posts, diagonals, bottom laterals) [E], track on top [W],
 * yellow/black bumper pads on the inner top edge, root box and trolley on the traverse beams.
 */
const armMembers = (s: 1 | -1): Member[] => {
  const out: Member[] = []
  const top = TOWER.armHeight, W = TOWER.armWidth
  const at = (o: number, n: number, u: number): V => enu(s * o, n, u)
  const ab = (mat: MatKey, o0: number, o1: number, n0: number, n1: number, u0: number, u1: number): Member => block(mat, s * o0, s * o1, n0, n1, u0, u1)
  // trolley with shoes on the traverse beams, actuator lug
  out.push(ab("towerDark", -0.8, W + 0.8, ROOT_N, ROOT_N + 1.4, top - 6.2, top + 2.4))
  out.push(ab("towerDark", -0.6, W + 0.6, ROOT_N + 1.4, BEAM_N + 0.2, BEAM_U[0] + 0.65, top + 2.4))
  out.push(ab("towerDark", -0.6, W + 0.6, ROOT_N + 1.4, BEAM_N + 0.2, top - 6.2, BEAM_U[1] - 0.65))
  out.push(s > 0 ? ab("towerDark", 0.55, 1.05, ACT_N - 0.35, ACT_N + 0.35, top + 2.4, ACT.uE + 0.5) : ab("towerDark", 0.55, 1.05, ACT_N - 0.35, ACT_N + 0.35, ACT.uW - 0.5, top - 6.2))
  // root box (plated) and haunch
  const n0 = ROOT_N - 3, nEnd = TIP_N + 0.5, uT = top - 0.65
  out.push(ab("arm", 0.05, W + 0.1, n0, ROOT_N, top - ARM.rootDepth, top - 0.45))
  // chords
  for (const o of [ARM.oIn, ARM.oOut]) {
    out.push(bar("arm", at(o, n0, uT), at(o, TIP_N + 0.25, uT), 0.4))
    out.push(bar("arm", at(o, n0, armBottom(n0) + 0.2), at(o, ARM.haunchN, armBottom(ARM.haunchN) + 0.2), 0.4))
    out.push(bar("arm", at(o, ARM.haunchN, armBottom(ARM.haunchN) + 0.2), at(o, TIP_N + 0.25, armBottom(TIP_N) + 0.2), 0.4))
  }
  // panel points: posts, diagonals (Warren), bottom laterals and struts, top struts
  const dn = (n0 - nEnd) / ARM.panels
  const pn = (i: number): number => n0 - i * dn
  const bot = (n: number): number => armBottom(n) + 0.2
  for (let i = 0; i <= ARM.panels; i++) {
    const n = pn(i)
    for (const o of [ARM.oIn, ARM.oOut]) out.push(bar("arm", at(o, n, bot(n) + 0.2), at(o, n, uT - 0.2), 0.3))
    out.push(bar("arm", at(ARM.oIn, n, bot(n)), at(ARM.oOut, n, bot(n)), 0.24), bar("arm", at(ARM.oIn, n, uT), at(ARM.oOut, n, uT), 0.3))
    if (i === ARM.panels) continue
    const m = pn(i + 1)
    for (const o of [ARM.oIn, ARM.oOut]) out.push(i % 2 === 0 ? bar("arm", at(o, n, uT), at(o, m, bot(m)), 0.26) : bar("arm", at(o, n, bot(n)), at(o, m, uT), 0.26))
    out.push(bar("arm", at(ARM.oIn, n, bot(n)), at(ARM.oOut, m, bot(m)), 0.18))
  }
  // track on top (top surface = TOWER.armHeight) with joint plates
  out.push(ab("rail", 0.1, W - 0.1, TIP_N + 0.15, ROOT_N, top - 0.45, top))
  for (let n = ROOT_N - 2.4; n > TIP_N + 1; n -= 4.68) out.push(ab("towerDark", 0.06, W - 0.06, n - 0.08, n + 0.08, top - 0.5, top - 0.02))
  // bumper pads on the inner top edge over the catch zone (inner face = o 0 exactly)
  for (let k = 0; k < 9; k++) {
    const nc = 7 - k * 2.4
    out.push(ab("yellow", 0, 0.16, nc - 0.85, nc + 0.85, top - 1.35, top - 0.1))
  }
  // tip: end plate, chevron marker, conduit along the outer side
  out.push(ab("arm", 0.1, W - 0.05, TIP_N, TIP_N + 0.5, armBottom(TIP_N), top - 0.45))
  out.push(ab("yellow", 0.2, W - 0.2, TIP_N - 0.04, TIP_N + 0.02, armBottom(TIP_N) + 0.35, top - 0.8))
  out.push(tube("cable", at(W - 0.05, n0, top - 1.4), at(W - 0.05, TIP_N + 1, top - 1.4), 0.14))
  return out
}

/**
 * Ship quick-disconnect arm, stowed (pointing west along the south face): box truss 2.4 × 3 m × 20 m,
 * hinge on the SW column, swing actuator, QD housing with clamshell hood, umbilical tray with the
 * LOX/CH₄/LN₂ lines that rise inside the west face from the pipe rack [W: SQD arm; E: sizes].
 */
const sqdMembers = (): Member[] => {
  const out: Member[] = []
  const e0 = SQD.e - 0.6, e1 = e0 - SQD.len, n = SQD.n, u = SQD.u, hw = SQD.w / 2, hh = SQD.h / 2
  for (const du of [-2.2, 1.2]) out.push(bar("towerDark", enu(TW.e - CL, TW.n - CL, u + du), enu(SQD.e, SQD.n, u + du), 0.9, 0.7))
  out.push(tube("towerDark", enu(SQD.e, SQD.n, u - 3.2), enu(SQD.e, SQD.n, u + 1.7), 1.1))
  for (const sn of [-hw, hw]) for (const su of [-hh, hh]) out.push(bar("towerDark", enu(e0, n + sn, u + su), enu(e1, n + sn, u + su), 0.38))
  const P = 8, dl = SQD.len / P
  for (let i = 0; i <= P; i++) {
    const e = e0 - i * dl
    for (const sn of [-hw, hw]) out.push(bar("towerDark", enu(e, n + sn, u - hh), enu(e, n + sn, u + hh), 0.3))
    for (const su of [-hh, hh]) out.push(bar("towerDark", enu(e, n - hw, u + su), enu(e, n + hw, u + su), 0.26))
    if (i === P) continue
    const e2 = e - dl, up = i % 2 === 0
    for (const sn of [-hw, hw]) out.push(bar("towerDark", enu(e, n + sn, up ? u + hh : u - hh), enu(e2, n + sn, up ? u - hh : u + hh), 0.24))
    out.push(bar("towerDark", enu(e, n - hw, u + hh), enu(e2, n + hw, u + hh), 0.2), bar("towerDark", enu(e, n - hw, u - hh), enu(e2, n + hw, u - hh), 0.2))
  }
  out.push(box("towerDark", e1 - 1, n, u, 2, 3, 3.2), box("rail", e1 - 1, n - 1.6, u - 0.2, 1.6, 0.25, 2.2))
  out.push(block("grate", e1, e0, n - 0.9, n + 0.9, u + hh + 0.19, u + hh + 0.31))
  out.push(tube("arm", enu(TW.e - CL - 0.4, TW.n - CL + 0.6, u + 3), enu(SQD.e - 4.5, n, u + hh), 0.5))
  // feed lines: rack → inside the west face → up → out → along the outside → umbilical tray
  const lanes = [-0.5, 0.15, 0.62] as const
  for (let i = 0; i < 3; i++) {
    const p = PIPES[i], uR = RACK.u1 + 0.45 + p.dia / 2, uT = u + hh + 0.31 + p.dia / 2, eIn = TW.e - CL + 0.75
    out.push(...pipeRun(p.mat, [
      enu(RACK.e1, p.n, uR), enu(eIn, p.n, uR), enu(eIn, p.n, uT), enu(SQD.e, p.n, uT), enu(SQD.e, n + lanes[i], uT), enu(e1 - 0.2, n + lanes[i], uT),
    ], p.dia))
  }
  return out
}

// ===================================================================================== pad 2 mount

/** flame-bucket pipe centres (n, u), spaced by arc length */
const bucketPoints = (): readonly (readonly [number, number])[] => {
  const pts: [number, number][] = []
  let acc: number = BUCKET.pitch / 2, pn: number = BUCKET.n0 + BUCKET.a, pu: number = BUCKET.top
  for (let i = 1; i <= 600; i++) {
    const th = (i / 600) * (Math.PI / 2)
    const n = BUCKET.n0 + BUCKET.a * Math.cos(th), u = BUCKET.top - BUCKET.b * Math.sin(th)
    acc += Math.hypot(n - pn, u - pu)
    pn = n
    pu = u
    if (acc >= BUCKET.pitch) { pts.push([n, u]); acc -= BUCKET.pitch }
  }
  return pts
}
/** n of the bucket surface at height u (for ending wall manifolds at the bucket) */
const bucketN = (u: number): number => BUCKET.n0 + BUCKET.a * Math.cos(Math.asin(Math.min(1, Math.max(0, (BUCKET.top - u) / BUCKET.b))))

/** chamfered table outline (e, n), inset from the outer edge */
const tableOutline = (inset: number): readonly (readonly [number, number])[] => {
  const E = OLM.halfE - inset, N = OLM.halfN - inset, c = OLM.chamfer - inset * 0.414
  return [[E - c, -N], [E, -N + c], [E, N - c], [E - c, N], [-E + c, N], [-E, N - c], [-E, -N + c], [-E + c, -N]]
}

/**
 * Pedestal, trench liners and manifolds, flame bucket pipes, open trench and wing walls, mount legs,
 * bracing, portal girders, 20 hold-down clamps, booster QD, handrails, stair tower and bridge,
 * propellant and deluge feeds [W, E].
 */
const mountMembers = (rand: () => number): Member[] => {
  const out: Member[] = []
  const { h, eIn, eOut, nS, nN, nBack } = PED
  out.push(block("concrete", -eOut, -eIn, nS, nN, 0, h), block("concrete", eIn, eOut, nS, nN, 0, h), block("concrete", -eIn, eIn, nBack, nN, 0, h))
  for (const s of [-1, 1]) {
    out.push(block("heat", s * (eIn - 0.15), s * eIn, nS, nBack, 0.45, h - 0.05))
    for (const u of [5.6, 3.1]) out.push(tube("stainless", enu(s * (eIn - 0.4), nS, u), enu(s * (eIn - 0.4), bucketN(u) - 0.5, u), 0.42))
    out.push(block("concrete", s * eIn, s * (eIn + 0.9), TRENCH.end, nS, 0, 4.2))
    out.push(bar("concrete", enu(s * (eIn + 0.45), TRENCH.end, 1.4), enu(s * (eIn + 7.5), TRENCH.end - TRENCH.wing, 1.4), 0.9, 2.8))
  }
  out.push(block("soot", -eIn, eIn, TRENCH.end - TRENCH.wing, nBack, 0.3, 0.45))
  for (const [n, u] of bucketPoints()) out.push(tube("heat", enu(-eIn + 0.15, n, u), enu(eIn - 0.15, n, u), BUCKET.pipe, 0.72 + 0.4 * rand()))
  // mount steel
  const { leg, legE, legN, legTop } = OLM
  for (const [se, sn] of CORNERS) {
    out.push(box("tower", se * legE, sn * legN, (h + legTop) / 2, leg, leg, legTop - h, 0.95))
    out.push(box("towerDark", se * legE, sn * legN, h + 0.25, leg + 1, leg + 1, 0.5))
    for (const u of [h + 3.2, legTop - 3.6]) out.push(box("towerDark", se * legE, sn * legN, u, leg + 0.24, leg + 0.24, 0.45))
  }
  // vertical stiffener ribs around the table sides (every ≈ 1.6 m)
  const edge = tableOutline(0)
  for (let i = 0; i < edge.length; i++) {
    const [ae, an] = edge[i], [be, bn] = edge[(i + 1) % edge.length]
    const L = Math.hypot(be - ae, bn - an), nE = (bn - an) / L, nN = -(be - ae) / L, cnt = Math.max(1, Math.round(L / 1.6))
    for (let k = 0; k <= cnt; k++) {
      const t = k / cnt, e = ae + (be - ae) * t + nE * 0.12, n = an + (bn - an) * t + nN * 0.12
      out.push(bar("towerDark", enu(e, n, legTop + 0.15), enu(e, n, OLM.top - OLM.plate - 0.15), 0.22, 0.3, enu(nE, nN, 0)))
    }
  }
  for (const s of [-1, 1]) {
    out.push(bar("tower", enu(s * legE, -legN + 1.6, h + 0.8), enu(s * legE, legN - 1.6, legTop - 2.4), 0.9), bar("tower", enu(s * legE, legN - 1.6, h + 0.8), enu(s * legE, -legN + 1.6, legTop - 2.4), 0.9))
    out.push(box("tower", 0, s * legN, legTop - 1.1, 2 * legE, 1.6, 2.2), box("tower", s * legE, 0, legTop - 1.1, 1.6, 2 * legN, 2.2))
  }
  for (let i = 0; i < OLM.clamps; i++) {
    const a = ((i + 0.5) / OLM.clamps) * Math.PI * 2, ce = Math.cos(a), sn = Math.sin(a)
    const rad = enu(ce, sn, 0)
    const p = (r: number, u: number): V => enu(r * ce, r * sn, u)
    out.push(bar("towerDark", p(OLM.clampR, OLM.top), p(OLM.clampR, OLM.top + 0.75), 1.1, 1.4, rad))
    out.push(bar("rail", p(OLM.hole + 0.3, OLM.top + 0.3), p(OLM.hole + 0.3, OLM.top + 1.3), 0.7, 0.5, rad))
  }
  // booster QD housing and carrier on the north side (hood = separate mesh)
  out.push(box("towerDark", 0, OLM.halfN - 3.2, OLM.top + 1.6, 3.6, 3, 3.2), box("rail", 0, OLM.halfN - 5.1, OLM.top + 0.9, 1.6, 0.8, 1.6))
  for (const de of [-1, 1]) out.push(tube("insul", enu(de, OLM.halfN + 0.5, h), enu(de, OLM.halfN + 0.5, OLM.top - 0.3), 0.7))
  // handrails around the table top
  const ring = tableOutline(0.35), uT = OLM.top - OLM.plate
  for (let i = 0; i < ring.length; i++) {
    const [ae, an] = ring[i], [be, bn] = ring[(i + 1) % ring.length]
    const cnt = Math.max(1, Math.round(Math.hypot(be - ae, bn - an) / 2.2))
    for (let k = 0; k < cnt; k++) { const t = k / cnt; out.push(box("safety", ae + (be - ae) * t, an + (bn - an) * t, uT + 0.55, 0.07, 0.07, 1.1)) }
    for (const du of [0.55, 1.08]) out.push(bar("safety", enu(ae, an, uT + du), enu(be, bn, uT + du), 0.06))
  }
  // stair tower on the west pedestal + bridge to the table
  const sE = -19, sN = 0, sw = 2.4, sl = 5.2, nf = 4, hf = (uT - h) / nf
  for (const [cx, cy] of CORNERS) out.push(box("tower", sE + (cx * sw) / 2, sN + (cy * sl) / 2, (h + uT) / 2, 0.3, 0.3, uT - h))
  for (let f = 0; f < nf; f++) {
    const z = h + f * hf
    out.push(box("grate", sE, sN - sl / 2 + 0.6, z + hf, sw, 1.2, 0.18), box("grate", sE, sN + sl / 2 - 0.6, z + hf / 2, sw, 1.2, 0.18))
    out.push(bar("grate", enu(sE - sw / 4, sN - sl / 2 + 1.2, z), enu(sE - sw / 4, sN + sl / 2 - 1.2, z + hf / 2), sw / 2 - 0.1, 0.16))
    out.push(bar("grate", enu(sE + sw / 4, sN + sl / 2 - 1.2, z + hf / 2), enu(sE + sw / 4, sN - sl / 2 + 1.2, z + hf), sw / 2 - 0.1, 0.16))
  }
  out.push(block("grate", sE + sw / 2, -OLM.halfE + 0.3, -1, 1, uT - 0.2, uT))
  // propellant feeds from the rack branch over the west embankment to the mount leg
  for (const [i, dia] of [[0, 0.8], [1, 0.7]] as const) {
    const n = -2 - 1.1 * i
    out.push(...pipeRun("insul", [enu(-40, RACK.nA + 0.6, RACK.u2 - 0.9), enu(-40, n, RACK.u2 - 0.9), enu(-28, n, RACK.u2 - 0.9), enu(-23, n, h + 0.7), enu(-14.4, n, h + 0.7), enu(-14.4, n, legTop - 0.5), enu(-13.2, n, legTop - 0.5)], dia))
  }
  for (const e of [-40, -34]) out.push(box("tower", e, -2.5, (RACK.u2 - 0.9 - 0.5) / 2, 0.3, 2.6, RACK.u2 - 1.4), box("tower", e, -2.5, RACK.u2 - 1.4, 0.4, 3, 0.3))
  for (const n of [5, 12]) out.push(box("tower", -40, n, (RACK.u2 - 1.4) / 2, 0.3, 0.3, RACK.u2 - 1.4))
  // deluge water main from the deluge tanks to the trench manifolds
  out.push(...pipeRun("stainless", [enu(-43.5, -47, 1.1), enu(-36, -47, 1.1), enu(-36, -10, 1.1), enu(-33, -10, 1.1), enu(-23, -10, h + 0.7), enu(-9.2, -10, h + 0.7), enu(-9.2, -10, 5.6)], 1.2))
  return out
}

/** mount meshes: embankments, bucket fill, table (chamfered, bevelled, central opening), top plate, BQD hood */
const mountMeshes = (mats: Mats): THREE.Object3D[] => {
  const out: THREE.Object3D[] = []
  const { h, eOut, nS, nN, toe, eIn } = PED
  for (const s of [-1, 1]) {
    const m = shadowed(new THREE.Mesh(extrude([[s * eOut, 0], [s * (eOut + toe), 0], [s * eOut, h]], nN - nS), mats.gravel))
    m.position.z = -nN
    out.push(m)
  }
  // fill under the bucket pipes: the quarter ellipse offset outward by 0.25 m, extruded across the trench
  const pts: [number, number][] = []
  for (let i = 0; i <= 40; i++) {
    const th = (i / 40) * (Math.PI / 2)
    const n = BUCKET.n0 + BUCKET.a * Math.cos(th), u = BUCKET.top - BUCKET.b * Math.sin(th)
    const dn = n - BUCKET.n0, du = u - BUCKET.top, l = Math.hypot(dn, du)
    pts.push([n + (0.25 * dn) / l, Math.max(0.45, u + (0.25 * du) / l)])
  }
  pts.push([BUCKET.n0 + BUCKET.a + 0.25, 0.45])
  const fill = shadowed(new THREE.Mesh(extrude(pts, 2 * (eIn - 0.15)).rotateY(Math.PI / 2), mats.concrete))
  fill.position.x = -(eIn - 0.15)
  out.push(fill)
  // table: chamfered square with the central opening, bevelled edges
  const bev = 0.25
  const hole = new THREE.Path().absarc(0, 0, OLM.hole + bev, 0, Math.PI * 2, true)
  const tbl = shadowed(new THREE.Mesh(extrude(tableOutline(bev), OLM.top - OLM.plate - OLM.legTop - 2 * bev, [hole], bev, 32).rotateX(-Math.PI / 2), mats.tower))
  tbl.position.y = OLM.legTop + bev
  out.push(tbl)
  // water-cooled steel top plate (ring)
  const ringOuter: [number, number][] = []
  for (let i = 0; i < 64; i++) { const a = (i / 64) * Math.PI * 2; ringOuter.push([OLM.plateR * Math.cos(a), OLM.plateR * Math.sin(a)]) }
  const plate = shadowed(new THREE.Mesh(extrude(ringOuter, OLM.plate, [new THREE.Path().absarc(0, 0, OLM.hole, 0, Math.PI * 2, true)], 0, 48).rotateX(-Math.PI / 2), mats.heat))
  plate.position.y = OLM.top - OLM.plate
  out.push(plate)
  const hood = shadowed(new THREE.Mesh(halfCylinder(1.8, 3, "n"), mats.arm))
  hood.position.copy(enu(0, OLM.halfN - 3.2, OLM.top + 3.2))
  out.push(hood)
  return out
}

// ===================================================================================== site

/** pipe rack (I-beam bents every 7.25 m, two tiers) with the five service lines and cable trays [E] */
const pipeRack = (): Member[] => {
  const out: Member[] = []
  for (let e = RACK.e0 + 2; e <= RACK.e1 - 2; e += RACK.step) {
    for (const n of [RACK.nA, RACK.nB]) out.push(ibeam("tower", enu(e, n, 0.25), enu(e, n, RACK.u1 + 0.35), 0.4, 0.4))
    for (const u of [RACK.u1, RACK.u2]) out.push(ibeam("tower", enu(e, RACK.nA - 0.3, u), enu(e, RACK.nB + 0.3, u), 0.3, 0.7))
  }
  for (const n of [RACK.nA, RACK.nB]) for (const u of [RACK.u1, RACK.u2]) out.push(ibeam("tower", enu(RACK.e0, n, u + 0.1), enu(RACK.e1 - 1, n, u + 0.1), 0.25, 0.45))
  PIPES.forEach((p, i) => { if (i >= 3) out.push(...pipeRun(p.mat, [enu(RACK.e0, p.n, RACK.u1 + 0.45 + p.dia / 2), enu(RACK.e1 + 1, p.n, RACK.u1 + 0.45 + p.dia / 2), enu(RACK.e1 + 1, p.n, 0.3)], p.dia)) })
  for (let i = 0; i < 3; i++) { const p = PIPES[i]; out.push(tube(p.mat, enu(RACK.e0, p.n, RACK.u1 + 0.45 + p.dia / 2), enu(RACK.e1, p.n, RACK.u1 + 0.45 + p.dia / 2), p.dia)) }
  for (const n of [RACK.nA + 0.9, RACK.nA + 2.2, RACK.nA + 3.5]) out.push(block("grate", RACK.e0, RACK.e1, n - 0.45, n + 0.45, RACK.u2 + 0.35, RACK.u2 + 0.6))
  // header along the tank rows
  out.push(...pipeRun("insul", [enu(RACK.e0, 5, 1.4), enu(RACK.e0, 35, 1.4)], 0.8), tube("insul", enu(RACK.e0, RACK.nA + 1, 1.4), enu(RACK.e0, RACK.nA + 1, RACK.u1 + 1), 0.8))
  return out
}

/** tank farm: vertical GSE tanks (instanced shells + platforms), horizontal tanks, subcoolers, vaporisers, flare [W, E] */
const tankFarm = (mats: Mats): { members: Member[]; meshes: THREE.Object3D[] } => {
  const out: Member[] = []
  const vAt: V[] = [], vTint: number[] = []
  for (let i = 0; i < 4; i++) for (let j = 0; j < 2; j++) {
    const e = TF.e - i * TF.pitchE, n = TF.n + j * TF.pitchN
    vAt.push(enu(e, n, 0.25))
    vTint.push(i === 3 && j === 1 ? 0.82 : 0.94 + 0.06 * ((i * 7 + j * 3) % 4) / 3)
    out.push(box("concrete", e, n, 0.35, 2 * TF.r + 1.2, 2 * TF.r + 1.2, 0.5))
    out.push(box("grate", e + TF.r + 0.3, n, TF.h / 2 + 1, 0.55, 0.12, TF.h + 1.5))
    for (let k = 0; k < 6; k++) { const a = (k / 6) * Math.PI * 2; out.push(box("safety", e + 2 * Math.cos(a), n + 2 * Math.sin(a), TF.h + TF.dome + 0.35, 0.06, 0.06, 1.1)) }
    out.push(...pipeRun("insul", [enu(e + TF.r * 0.7, n - TF.r * 0.7, 1.4), enu(RACK.e0, n - TF.r * 0.7, 1.4)], 0.45))
  }
  out.push(block("concrete", TF.e - 3 * TF.pitchE - 9, TF.e + 7, TF.n - 7.5, TF.n - 7.1, 0.25, 1.4), block("concrete", TF.e - 3 * TF.pitchE - 9, TF.e + 7, TF.n + TF.pitchN + 7.1, TF.n + TF.pitchN + 7.5, 0.25, 1.4))
  out.push(block("concrete", TF.e - 3 * TF.pitchE - 9.4, TF.e - 3 * TF.pitchE - 9, TF.n - 7.5, TF.n + TF.pitchN + 7.5, 0.25, 1.4), block("concrete", TF.e + 7, TF.e + 7.4, TF.n - 7.5, TF.n + TF.pitchN + 7.5, 0.25, 1.4))
  const hAt: V[] = []
  for (let i = 0; i < 10; i++) {
    const e = -128 - 5.5 * i, n = -25
    hAt.push(enu(e, n, 1.4 + TF.hR))
    for (const dn of [-7, 7]) out.push(box("concrete", e, n + dn, 0.9, 2.8, 0.8, 1.3))
  }
  // subcoolers: steel frames with vertical vessels
  for (const n of [TF.n, TF.n + TF.pitchN]) {
    const e = TF.e - 4 * TF.pitchE - 3
    for (const [cx, cy] of CORNERS) out.push(ibeam("tower", enu(e + cx * 3.5, n + cy * 3.5, 0.25), enu(e + cx * 3.5, n + cy * 3.5, 12.5), 0.4, 0.4))
    for (const u of [4.5, 8.5, 12.3]) for (let c = 0; c < 4; c++) {
      const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
      out.push(ibeam("tower", enu(e + ax * 3.5, n + ay * 3.5, u), enu(e + bx * 3.5, n + by * 3.5, u), 0.3, 0.5))
    }
    for (const [dx, dy] of [[-1.5, -1.5], [1.5, -1.5], [0, 1.5]] as const) out.push(tube("stainless", enu(e + dx, n + dy, 0.25), enu(e + dx, n + dy, 10.5), 1.8))
    out.push(tube("insul", enu(e - 3, n, 9.8), enu(e + 3, n, 9.8), 1.2))
  }
  // ambient-air vaporisers, flare stack with guys
  for (let i = 0; i < 12; i++) out.push(box("fins", TF.e + 4 - 2.4 * i, TF.n + TF.pitchN + 12, 6.35, 1.6, 1.6, 12))
  const fe = -212, fn = -40
  out.push(tube("towerDark", enu(fe, fn, 0.25), enu(fe, fn, 32), 1), box("towerDark", fe, fn, 32.4, 1.5, 1.5, 0.8))
  for (let k = 0; k < 3; k++) { const a = (k / 3) * Math.PI * 2 + 0.3; out.push(tube("cable", enu(fe, fn, 29), enu(fe + 18 * Math.cos(a), fn + 18 * Math.sin(a), 0.3), 0.05)) }
  const meshes: THREE.Object3D[] = [
    instancedAt(tankGeometry(TF.r, TF.h, TF.dome, 48), mats.tank, vAt, vTint),
    instancedAt(capsuleGeometry(TF.hR, TF.hL, 32), mats.tank, hAt),
  ]
  return { members: out, meshes }
}

/** apron, gravel, roads, fence, light masts, drawworks, HPU containers, electrical house, deluge tanks [E] */
const site = (mats: Mats): { members: Member[]; meshes: THREE.Object3D[] } => {
  const out: Member[] = []
  const [r0, r1, r2, r3] = SITE.road
  out.push(block("asphalt", r0, r1, r2, r2 + 8, -0.05, 0.08), block("asphalt", r0, r1, r3 - 8, r3, -0.05, 0.08))
  out.push(block("asphalt", r0, r0 + 8, r2, r3, -0.05, 0.08), block("asphalt", r1 - 8, r1, r2, r3, -0.05, 0.08))
  out.push(block("asphalt", -3000, r0, -66, -57, -0.05, 0.08), block("asphalt", -205, r0, -24, -16, -0.05, 0.08))
  // fence: posts every 6 m, top rails; fabric as translucent panels (meshes)
  const [f0, f1, f2, f3] = SITE.fence
  const sides = [[f0, f2, f1, f2], [f1, f2, f1, f3], [f1, f3, f0, f3], [f0, f3, f0, f2]] as const
  for (const [ae, an, be, bn] of sides) {
    const L = Math.hypot(be - ae, bn - an), cnt = Math.round(L / 6)
    for (let k = 0; k < cnt; k++) {
      const t = k / cnt, e = ae + (be - ae) * t, n = an + (bn - an) * t
      if (e < -230 && n > -70 && n < -53) continue // gate at the access road
      out.push(box("grate", e, n, 1.3, 0.09, 0.09, 2.6))
    }
    for (const u of [2.55, 0.12]) out.push(bar("grate", enu(ae, an, u), enu(be, bn, u), 0.05))
  }
  // light masts (30 m) with lamp heads facing the mount
  const masts = [[-62, -98], [62, -98], [-62, -20], [62, -20], [-62, 62], [62, 62]] as const
  for (const [e, n] of masts) {
    const L = Math.hypot(e, n), rad = enu(-e / L, -n / L, 0), tan = enu(-n / L, e / L, 0)
    out.push(tube("grate", enu(e, n, 0.3), enu(e, n, 15), 0.8), tube("grate", enu(e, n, 15), enu(e, n, 30), 0.55), box("concrete", e, n, 0.5, 1.6, 1.6, 0.6))
    out.push(bar("towerDark", enu(e, n, 29.6), enu(e, n, 31.8), 3.6, 0.5, rad))
    for (let i = 0; i < 3; i++) for (let j = 0; j < 2; j++) {
      const p = enu(e, n, 30.1 + j * 1.1).addScaledVector(tan, (i - 1) * 1.1).addScaledVector(rad, 0.35)
      out.push(bar("lamp", p.clone().addScaledVector(UP, -0.4), p.clone().addScaledVector(UP, 0.4), 0.9, 0.2, rad))
    }
  }
  // drawworks house, HPU containers, electrical house
  out.push(block("siding", DW.e - DW.w / 2, DW.e + DW.w / 2, DW.n0, DW.n1, 0.3, DW.h), block("roof", DW.e - DW.w / 2 - 0.3, DW.e + DW.w / 2 + 0.3, DW.n0 - 0.3, DW.n1 + 0.3, DW.h, DW.h + 0.35))
  out.push(box("white", 12.2, TW.n + 1, 1.65, 2.44, 12.2, 2.6), box("blue", 15.2, TW.n + 1, 1.65, 2.44, 12.2, 2.6), box("white", 18.2, TW.n + 1, 1.65, 2.44, 12.2, 2.6))
  out.push(block("siding", 22, 32, 36, 42, 0.3, 4.8), block("roof", 21.8, 32.2, 35.8, 42.2, 4.8, 5.1))
  const meshes: THREE.Object3D[] = []
  const [a0, a1, a2, a3] = SITE.apron
  const apron = shadowed(new THREE.Mesh(new THREE.BoxGeometry(a1 - a0, 0.4, a3 - a2), mats.apron), false)
  apron.position.copy(enu((a0 + a1) / 2, (a2 + a3) / 2, 0.15))
  const gravel = shadowed(new THREE.Mesh(new THREE.BoxGeometry(95, 0.3, 105), mats.gravel), false)
  gravel.position.copy(enu(-157.5, 3, 0.1))
  const decal = new THREE.Mesh(new THREE.PlaneGeometry(26, 92).rotateX(-Math.PI / 2), mats.decal)
  decal.position.copy(enu(0, -4 - 46, 0.47))
  decal.receiveShadow = true
  meshes.push(apron, gravel, decal)
  for (const [ae, an, be, bn] of sides) {
    const L = Math.hypot(be - ae, bn - an)
    const fab = new THREE.Mesh(new THREE.PlaneGeometry(L, 2.4), mats.fabric)
    fab.position.copy(enu((ae + be) / 2, (an + bn) / 2, 1.3))
    fab.rotation.y = Math.atan2(-(bn - an), be - ae)
    meshes.push(fab)
  }
  meshes.push(instancedAt(tankGeometry(4, 20, 2, 40), mats.tank, [enu(-48, -40, 0.35), enu(-48, -54, 0.35)]))
  return { members: out, meshes }
}

// ===================================================================================== assembly

export const makePad = (): PadModel => {
  const root = new THREE.Group()
  root.name = "pad2"
  const rand = rng(20261114)
  const mats = makeMaterials(rand)
  const geos = makeProfiles()
  const farm = tankFarm(mats)
  const env = site(mats)
  const fixed: Member[] = [
    ...towerLattice(rand), ...towerCore(), ...crown(), ...hoistCables(), ...carriage(), ...sqdMembers(),
    ...mountMembers(rand), ...pipeRack(), ...farm.members, ...env.members,
  ]
  root.add(...instance(fixed, mats, geos), ...mountMeshes(mats), ...farm.meshes, ...env.meshes)
  // lightning mast to TOWER.height [T, W], SQD hood
  const mastBase = TW.top + 1.6
  const mast = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.3, TOWER.height - mastBase, 10), mats.towerDark))
  mast.position.copy(enu(TW.e, TW.n, (mastBase + TOWER.height) / 2))
  const hood = shadowed(new THREE.Mesh(halfCylinder(1.6, 2.2, "e"), mats.arm))
  hood.position.copy(enu(SQD.e - 0.6 - SQD.len - 1, SQD.n, SQD.u + 1.6))
  root.add(mast, hood)
  // aviation obstruction lights: crown corners, mast tip, mid-height corners
  const beacons: V[] = [enu(TW.e, TW.n, TOWER.height + 0.2)]
  for (const [cx, cy] of CORNERS) beacons.push(enu(TW.e + cx * 6.2, TW.n + cy * 6.2, TW.top + 0.9), enu(TW.e + cx * 6.3, TW.n + cy * 6.3, TW.base + 4.5 * HS))
  const lights = instancedAt(new THREE.SphereGeometry(0.28, 10, 8), mats.red, beacons)
  lights.castShadow = false
  root.add(lights)
  // chopsticks: one group per arm, translated by setArms (inner faces at ±gap/2 [T])
  const arms = ([-1, 1] as const).map((s) => {
    const g = new THREE.Group()
    g.name = s < 0 ? "chopstick-west" : "chopstick-east"
    g.add(...instance(armMembers(s), mats, geos))
    root.add(g)
    return g
  })
  // actuator rods (chrome), stretched between the fixed barrel end and the trolley lug
  const rodGeo = new THREE.CylinderGeometry(0.5, 0.5, 1, 14).rotateZ(Math.PI / 2)
  const rods = ([-1, 1] as const).map((s) => {
    const m = shadowed(new THREE.Mesh(rodGeo, mats.chrome))
    m.position.copy(enu(0, ACT_N, s > 0 ? ACT.uE : ACT.uW))
    m.scale.set(1, 0.38, 0.38)
    root.add(m)
    return m
  })
  const setArms = (open: number): void => {
    const k = Math.min(1, Math.max(0, Number.isFinite(open) ? open : 1))
    const gap = TOWER.closedGap + (TOWER.openGap - TOWER.closedGap) * k
    arms.forEach((g, i) => {
      const s = i === 0 ? -1 : 1
      g.position.set(TW.e + (s * gap) / 2, 0, 0)
      const xb = TW.e - s * ACT.anchor + s * ACT.barrel, xl = TW.e + s * (gap / 2 + 0.8)
      const rod = rods[i]
      rod.position.x = (xb + xl) / 2
      rod.scale.x = Math.max(0.02, Math.abs(xl - xb))
      rod.visible = Math.abs(xl - xb) > 0.05
    })
  }
  setArms(1)
  return { root, setArms }
}
