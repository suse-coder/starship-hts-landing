import * as THREE from "three"
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js"
import { TOWER, armsInit, armInnerAt, armDepthAt } from "@ship/tower"

/**
 * Starbase Pad 2 ("Pad B") — procedural model of the integration tower ("Mechazilla"), its carriage and
 * chopsticks, the ship quick-disconnect arm, the Pad 2 orbital launch mount over its flame trench, the
 * tank farm, pipe racks, roads, fence and lights.
 *
 * Frame: the pad's local ENU frame mapped to three.js (x = East, y = Up, z = −North); origin at the
 * centre of the launch mount at ground level (the same frame as the contact model in @ship/tower).
 *
 * Sources (tags used in the doc comments below):
 * - [T] `TOWER`, `armInnerAt`, `armDepthAt` and the `towerBoxes` envelopes in @ship/tower — the
 *   geometry the contact model uses. The picture is built from those numbers: rail top, inner faces,
 *   arm length, wedge depth and tip station are read from the module, never restated (the private
 *   bounds of a `towerBoxes` envelope are quoted where a part has to stay inside one).
 * - [R] the annotated reference render of the structure (Owe BL): square four-legged lattice mast in
 *   light steel with girts and diagonal bracing in short repeating modules, two red-striped carriage
 *   tracks on the corner columns of the arm face, a dark portal-frame carriage, two dark tapered
 *   truss chopsticks with a ladder-like top, a dark articulated QD arm with an elbow and an end plate,
 *   a lattice crown.
 * - [W] Wikipedia, "SpaceX Starbase" (accessed 2026-11-21): 146 m tower (FAA determination of no
 *   hazard, 2021), lightning rod and a small weather station on top; the arms sit on a carriage moved
 *   by a cable over a pulley at the top from a winch at the base, with bearings at the sides of the
 *   carriage; a linear hydraulic actuator moves the arms; tracks on top of the arms; the Pad 2 mount
 *   has a water-cooled steel plate on its top level and a steel-pipe flame diverter ("flame bucket");
 *   20 hold-down clamps and a booster quick disconnect; the tank farm stores CH₄, LOX, water, N₂, He
 *   and hydraulic fluid, with subcoolers.
 * - [E] estimated (proportions against the 9 m vehicles and the reference render).
 *
 * Kinematics: the real chopsticks swing about vertical hinges on the carriage; the contact model
 * translates the pair across x (gap `closedGap…openGap`, centre `cx`). The picture keeps the model's
 * kinematics — each arm rides a trolley on a traverse girder of the carriage — and draws the hinge
 * boss at each arm root so it still reads as a hinged chopstick.
 *
 * Scene graph: `root` ("pad2") → "site" (mount, trench, tank farm, roads), "mechazilla" (mast, crown,
 * carriage, traverse girder, QD arm, actuator rods, beacons), "chopstick-west", "chopstick-east".
 *
 * Performance: every straight member is an instance of one of five unit profiles, grouped by
 * (profile, material) into InstancedMeshes; the tower and the site use separate pools so the tower
 * can be culled and counted on its own. Measured (packages/view/test/tower.test.ts keeps the budget):
 * the tower — "mechazilla" + both chopsticks — is 1150 instances in 23 draw calls, ≈ 2.15·10⁴
 * triangles; the whole pad is 64 draw calls, ≈ 6.1·10⁴ triangles. Weathering is a world-scale
 * triplanar grime texture patched into MeshStandardMaterial (no UV stretching on scaled instances).
 * `setArms` only writes two group positions and two rod transforms (no allocation).
 */
export const enu = (e: number, n: number, u: number): THREE.Vector3 => new THREE.Vector3(e, u, -n)

export interface PadModel {
  readonly root: THREE.Group
  readonly setArms: (open: number, cx?: number) => void
}

// ===================================================================================== dimensions

/** Lattice mast. */
const TW = {
  /** tower centre (m) [T: TOWER.pos] */
  e: TOWER.pos.x,
  n: TOWER.pos.y,
  /** half side of the square mast, outer column faces (m) [T: "tower-lattice" box is ±6.1 m] */
  half: 6,
  /** corner column, square box section (m) [T: box comment "corner columns at ±5.25 m (1.5 m boxes)"] */
  col: 1.5,
  /** bracing modules (girt to girt) [R: repeating modules of ≈ 8–10 m; E: 15 × 8.85 m] */
  modules: 15,
  /** top of the pile caps = column base (m) [T: "tower-pile-caps" box is 1.2 m tall] */
  base: 1.2,
  /** top of the lattice (m) [T: "tower-crown" box starts at 133.5 m]; crown + lightning rod reach TOWER.height */
  top: 134,
} as const
/** column centre-line offset from the tower centre (5.25 m) */
const CL = TW.half - TW.col / 2
/** module height ≈ 8.85 m */
const HM = (TW.top - TW.base) / TW.modules
/** floodlight levels on the arm face (m) [T: "tower-floodlights-low/mid/high" boxes, 37.9–39.2, 82.2–83.5, 97.6–98.9] */
const FLOOD_U = [38.1, 82.4, TOWER.armHeight - 14] as const
/** crown [T: "tower-crown" box, 133.5–139.5 m, e ±6.4, n −9.3…+7.7 about the tower centre; R: lattice crown; E: layout] */
const CROWN = { apex: 139, foot: 4.2, head: 1.1, jibE: 3.8, sheaveE: 3, sheaveU: TW.top + 2.6, sheaveR: 0.8 } as const
/**
 * Hoist cable drops (n): 0.15 m outside the south and north column faces, i.e. on the faces of the
 * "tower-lattice" box [T], so no cable hangs in the catch corridor.
 */
const DROP_S = TW.n - TW.half - 0.15
const DROP_N = TW.n + TW.half + 0.15

/** Carriage (portal frame around the mast, rides the two tracks, cable-hoisted [W]). */
const CA = {
  /** frame bottom / top (m) [E: 15 m deep frame; T: "carriage" box is armHeight −9.1…+9.6] */
  z0: TOWER.armHeight - 9,
  z1: TOWER.armHeight + 6,
  /** centre-line half width of the frame (m) [E: 1.6 m clearance for the bogies; T: box ±8.2 m] */
  r: TW.half + 1.6,
  girder: 1.1,
  /** half length of the traverse girder (m) [T: "traverse-beams" box ±16.6 m] */
  beamHalf: 16.6,
} as const
/** south face of the carriage frame (n = 14.85 m) */
const RING_S = TW.n - CA.r - CA.girder / 2
/** traverse girder chords, centre line (1.2 m deep in n: 13.65…14.85) [T: "traverse-beams" box n 13.6…14.9] */
const BEAM_N = RING_S - 0.6
/** upper / lower traverse chord centres, 1.3 m deep (m) [T: box armHeight −5.8…+1.9] */
const BEAM_U = [TOWER.armHeight + 1.25, TOWER.armHeight - 5.1] as const
/** web (posts and diagonals) of the traverse girder, behind the trolley lugs */
const WEB_N = BEAM_N + 0.43
/**
 * Arm actuators [W: linear hydraulic actuator], between the two traverse chords: crossed cylinders
 * (the east arm's above the west arm's), barrel fixed to the girder, rod to the trolley lug `lug`
 * metres outboard of the arm's inner edge. Closed length (8.4 m) ≥ barrel (8.2 m); the stroke covers
 * the 8.4 m travel per arm [T: (openGap − closedGap)/2].
 */
const ACT = { anchor: 3, barrel: 8.2, uE: TOWER.armHeight - 0.6, uW: TOWER.armHeight - 3.2, n: BEAM_N - 0.25, lug: 0.8 } as const

/** trolley front face (n = 12.25 m) [T: "arm-*-trolley" boxes start at n = 12.25] */
const ROOT_N = RING_S - 2.6
/** arm tip = contact model yMin = pos.y − 2 − armLength [T] */
const TIP_N = TOWER.pos.y - 2 - TOWER.armLength
/**
 * Chopstick truss [T for every contact dimension; R/E for the member layout]:
 * - rail top = TOWER.armHeight, rail inner edge = `armInnerAt(·, 0)` (group x = 0), width TOWER.armWidth;
 * - underside at `armDepthAt(n)` below the rail (wedge, armRootDepth → armTipDepth);
 * - below the rail the inner face is the plane `armInnerAt(·, dz)`: it leans outward with depth
 *   (TOWER.armFlare over TOWER.armRootDepth), so the lower chord sits outboard of the rail. That
 *   lean leaves no room for a fourth chord inside the arm width, hence a three-chord taper truss
 *   (two top chords under the rail and walkway, one lower chord), which is also what [R] shows.
 */
const ARM = {
  /** landing rail: thickness and width (m) [E]; its top and inner edge are [T] */
  rail: 0.3, railW: 0.9,
  /** hazard-painted fascia on the rail's inner edge (m) [E] */
  edge: 0.05,
  /** round members Ø (m) [E] */
  chord: 0.4, spine: 0.44, post: 0.26, diag: 0.2, rung: 0.2,
  /** rub plate on the inner face below the rail: thickness, depth range below the rail top (m) [E] */
  pad: 0.1, padTop: 0.4, padBot: 1.24,
  /** truss panels between the root frame and the tip [E: ≈ 2.5 m pitch] */
  panels: 10,
  /** the top narrows over the last 6 m to this outer chord offset (m) [R: narrow tip; E] */
  taper: 6, tipOut: 1.0,
  /** hinge boss: axis offset from the inner edge, distance in front of the trolley (m) [E] */
  hingeO: 1.1, hingeN: 0.8,
} as const
/** root frame of the truss (1.6 m in front of the trolley: the hinge bay) and its last frame before the tip shoe */
const TRUSS_N = ROOT_N - 1.6
const TRUSS_END = TIP_N + 0.9
/** station where the top starts to narrow */
const TAPER_N = TIP_N + ARM.taper
/** outward lean of an arm's inner face at `dz` below the rail top (m) [T: armInnerAt] */
const innerLean = (dz: number): number => armInnerAt(armsInit, 1, dz) - armInnerAt(armsInit, 1, 0)
/** lean per metre of depth and the secant of the lean angle */
const LEAN_K = innerLean(1)
const LEAN_SEC = Math.hypot(1, LEAN_K)
/** outward offset of the axis of a round member (radius r) whose surface stays `gap` behind the inner face at depth dz */
const behind = (dz: number, r: number, gap = 0.01): number => innerLean(dz) + (r + gap) * LEAN_SEC

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
/**
 * Ship quick-disconnect arm [R: dark lattice arm with an elbow and an end plate, below the chopsticks;
 * E: sizes], parked swung away from the catch corridor: hinged on the SW column, pointing west along
 * the south face. Every part stays inside the contact model's envelope
 * [T: "ship-qd-arm" box, e −29.8…−5, n 14…17.2, u 93.2…98.3].
 */
const QD = {
  /** hinge axis and arm centre line (m) */
  hingeE: TW.e - 7, n: TW.n - 7.4,
  /** upper arm: axis height, east/west ends, half width (n) and half height, panels */
  u: 97, e0: TW.e - 7.7, e1: TW.e - 18, hw: 1.05, hh: 1, panels: 4,
  /** elbow pin (axis along n) */
  elbowE: TW.e - 18.5,
  /** forearm: ends, axis drop over its length, half width / height, panels */
  f0: TW.e - 19, f1: TW.e - 26.6, drop: 1.2, fhw: 0.95, fhh: 0.85, fpanels: 3,
  /** hose bundle under the north lower chord: lane (n), top hose height, spacing, Ø */
  hoseN: TW.n - 6.35, hoseU: 95.45, hoseStep: 0.36, hose: 0.3,
} as const
/** QD propellant risers inside the west side of the mast (e) [E] */
const RISER_E = TW.e - CL + 0.75
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
type Prof = "box" | "ibeam" | "tube" | "pipe" | "ball"
type MatKey =
  | "mast" | "black" | "stripe" | "tower" | "towerDark" | "arm" | "rail" | "chrome" | "yellow" | "safety" | "concrete" | "apron" | "gravel"
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
/** round truss member (8-sided) */
const pipe = (mat: MatKey, a: V, b: V, dia: number): Member => ({ prof: "pipe", mat, a, b, w: dia, d: dia })
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

/** polyline hose without elbow fittings: every run is extended by half a diameter so the corners close */
const hoseRun = (mat: MatKey, pts: readonly V[], dia: number): Member[] => {
  const out: Member[] = []
  for (let i = 1; i < pts.length; i++) {
    const dir = pts[i].clone().sub(pts[i - 1]).normalize()
    const a = i > 1 ? pts[i - 1].clone().addScaledVector(dir, -dia / 2) : pts[i - 1]
    const b = i < pts.length - 1 ? pts[i].clone().addScaledVector(dir, dia / 2) : pts[i]
    out.push(pipe(mat, a, b, dia))
  }
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

/** unit profiles: box, I-beam (flanges ⟂ local z), tube (14-sided), pipe (8-sided truss member), ball */
const makeProfiles = (): Readonly<Record<Prof, THREE.BufferGeometry>> => ({
  box: new THREE.BoxGeometry(1, 1, 1),
  ibeam: mergeGeometries([
    new THREE.BoxGeometry(1, 1, 0.14).translate(0, 0, 0.43),
    new THREE.BoxGeometry(1, 1, 0.14).translate(0, 0, -0.43),
    new THREE.BoxGeometry(0.16, 1, 0.74),
  ]),
  tube: new THREE.CylinderGeometry(0.5, 0.5, 1, 14, 1),
  pipe: new THREE.CylinderGeometry(0.5, 0.5, 1, 8, 1),
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

/**
 * PBR palette. Mechazilla [R]: light-grey painted steel for the mast and crown ("mast"), near-black
 * steel for the carriage, chopsticks and QD arm ("black"), red paint for the track stripes ("stripe"),
 * bare machined steel for rails, pins and sheaves ("rail"). The mount and site keep their mid-grey
 * structural steel ("tower"/"towerDark"), concrete, tanks and pipes.
 */
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
    mast: weather(std({ color: 0xc6cacd, metalness: 0.3, roughness: 0.55 }), rust, 7, 0.45),
    black: weather(std({ color: 0x24272b, metalness: 0.65, roughness: 0.5 }), rust, 6, 0.35),
    stripe: weather(std({ color: 0xc21f1f, metalness: 0.15, roughness: 0.5 }), water, 5, 0.25),
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
 * Lattice mast [R, E; T for the envelope]: four box corner columns (1.5 m, centre lines ±5.25 m),
 * and per 8.85 m module on every face a girt (0.5 × 0.8 m), X-bracing (0.4 m square tubes) with a
 * gusset at the crossing and a light strut through it; corner knee braces in plan at every third
 * girt. Each module gets its own paint shade. The two carriage tracks run the full height on the
 * south corner columns (the arm face): a dark rail plate with a red stripe on the south face,
 * returned onto the column's side face [R: red-striped tracks]. Everything stays inside the
 * "tower-lattice" box (±6.1 m) [T].
 */
const towerLattice = (rand: () => number): Member[] => {
  const out: Member[] = []
  const at = (de: number, dn: number, u: number): V => enu(TW.e + de, TW.n + dn, u)
  /** girt level k = 0 … modules */
  const zg = (k: number): number => (k === 0 ? TW.base + 0.6 : k === TW.modules ? TW.top - 0.4 : TW.base + k * HM)
  for (let k = 0; k < TW.modules; k++) {
    const z0 = TW.base + k * HM, za = zg(k), zb = zg(k + 1)
    const tint = 0.88 + 0.12 * rand()
    for (const [cx, cy] of CORNERS) out.push(bar("mast", at(cx * CL, cy * CL, z0), at(cx * CL, cy * CL, z0 + HM), TW.col, TW.col, undefined, tint))
    for (let c = 0; c < 4; c++) {
      const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
      const A = (u: number): V => at(ax * CL, ay * CL, u), B = (u: number): V => at(bx * CL, by * CL, u)
      out.push(bar("mast", A(za), B(za), 0.5, 0.8, undefined, tint))
      if (k === TW.modules - 1) out.push(bar("mast", A(zb), B(zb), 0.5, 0.8, undefined, tint))
      out.push(bar("mast", A(za + 0.4), B(zb - 0.4), 0.4, 0.4, undefined, tint), bar("mast", B(za + 0.4), A(zb - 0.4), 0.4, 0.4, undefined, tint))
      const me = TW.e + ((ax + bx) / 2) * CL, mn = TW.n + ((ay + by) / 2) * CL, zm = (za + zb) / 2
      out.push(ay === by ? box("mast", me, mn, zm, 1.3, 0.5, 1.1, 0.8 * tint) : box("mast", me, mn, zm, 0.5, 1.3, 1.1, 0.8 * tint))
      out.push(bar("mast", A(zm), B(zm), 0.3, 0.36, undefined, tint))
    }
    if (k > 0 && k % 3 === 0) for (const [cx, cy] of CORNERS) out.push(bar("mast", at(cx * CL, cy * (CL - 1.6), za), at(cx * (CL - 1.6), cy * CL, za), 0.3, 0.4, undefined, tint))
  }
  // carriage tracks on the two south columns: rail plate + red stripe, south face and side face
  const zt0 = TW.base + 0.3, zt1 = TW.top - 0.2, nS = TW.n - TW.half
  for (const sx of [-1, 1]) {
    const e = TW.e + sx * CL, eO = TW.e + sx * TW.half
    out.push(block("rail", e - 0.5, e + 0.5, nS - 0.06, nS, zt0, zt1), block("stripe", e - 0.2, e + 0.2, nS - 0.1, nS - 0.06, zt0, zt1))
    out.push(block("rail", eO, eO + sx * 0.06, nS + 0.25, nS + 1.25, zt0, zt1), block("stripe", eO + sx * 0.06, eO + sx * 0.1, nS + 0.55, nS + 0.95, zt0, zt1))
  }
  // cable trays up the north face (hoist and power), inside the column line
  for (const de of [-1.6, 1.6]) out.push(box("grate", TW.e + de, TW.n + CL + 0.4, (TW.base + TW.top) / 2, 0.8, 0.2, TW.top - TW.base))
  // pile caps under the columns [T: "tower-pile-caps"]
  for (const [cx, cy] of CORNERS) out.push(box("concrete", TW.e + cx * CL, TW.n + cy * CL, TW.base / 2, 4, 4, TW.base))
  // floodlights on the south face (lighting the mount and the catch zone) on a light rail between the columns [T: floodlight boxes]
  for (const u of FLOOD_U) {
    out.push(box("mast", TW.e, TW.n - CL, u - 0.1, 2 * CL, 0.3, 0.3))
    for (const de of [-3.6, 3.6]) {
      out.push(block("grate", TW.e + de - 0.8, TW.e + de + 0.8, nS - 0.9, TW.n - CL + 0.1, u - 0.125, u + 0.125))
      out.push(box("lamp", TW.e + de, nS - 0.95, u + 0.45, 1.3, 0.12, 0.7))
    }
  }
  return out
}

/**
 * Internal core [E]: switch-back stair (two flights per module) between four core columns, an
 * elevator on guide rails west of the stair, grated service platforms at two girt levels.
 */
const towerCore = (): Member[] => {
  const out: Member[] = []
  const e0 = TW.e, n0 = TW.n, ce = 2, cn = 2.9, H = TW.top - TW.base, mid = (TW.base + TW.top) / 2
  for (const [sx, sy] of CORNERS) out.push(box("grate", e0 + sx * ce, n0 + sy * cn, mid, 0.3, 0.3, H))
  const hf = HM / 2, flights = TW.modules * 2
  for (let f = 0; f < flights; f++) {
    const z = TW.base + f * hf + 0.95
    out.push(box("grate", e0, n0 - cn + 0.6, z, 2 * ce, 1.2, 0.18), box("grate", e0, n0 + cn - 0.6, z + hf / 2, 2 * ce, 1.2, 0.18))
    out.push(bar("grate", enu(e0 - ce / 2, n0 - cn + 1.2, z), enu(e0 - ce / 2, n0 + cn - 1.2, z + hf / 2), ce - 0.15, 0.16))
    // the last half flight would come out through the crown deck
    if (f < flights - 1) out.push(bar("grate", enu(e0 + ce / 2, n0 + cn - 1.2, z + hf / 2), enu(e0 + ce / 2, n0 - cn + 1.2, z + hf), ce - 0.15, 0.16))
  }
  for (const dn of [-1.3, 1.3]) out.push(box("rail", e0 - 3.1, n0 + dn, mid, 0.16, 0.16, H))
  out.push(box("mast", e0 - 3.1, n0, 64, 1.7, 2.5, 2.7))
  for (const k of [5, 10]) {
    const z = TW.base + k * HM + 0.5
    out.push(box("grate", e0 + (ce + 4.5) / 2, n0, z, 4.5 - ce, 9, 0.22))
    out.push(box("grate", e0 + (ce - 3.8) / 2, n0 + (cn + 4.5) / 2, z, 3.8 + ce, 4.5 - cn, 0.22))
    out.push(box("grate", e0 + (ce - 3.8) / 2, n0 - (cn + 4.5) / 2, z, 3.8 + ce, 4.5 - cn, 0.22))
  }
  return out
}

/**
 * Crown [R: lattice crown; W: pulley at the top, lightning rod, weather station; T: "tower-crown"
 * box]: top deck, a lattice king-post pyramid carrying the lightning rod to TOWER.height, and two
 * lattice jib girders over the south and north faces with the hoist sheaves (Ø 1.6 m) on common axles.
 */
const crown = (): Member[] => {
  const out: Member[] = []
  const T = TW.top, zD = T + 0.6, zA = CROWN.apex, zm = (zD + zA) / 2, { foot, head } = CROWN, mid = (foot + head) / 2
  const at = (de: number, dn: number, u: number): V => enu(TW.e + de, TW.n + dn, u)
  out.push(box("mast", TW.e, TW.n, T + 0.3, 2 * TW.half, 2 * TW.half, 0.6))
  for (let c = 0; c < 4; c++) {
    const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
    out.push(bar("mast", at(ax * foot, ay * foot, zD), at(ax * head, ay * head, zA), 0.45))
    out.push(bar("mast", at(ax * mid, ay * mid, zm), at(bx * mid, by * mid, zm), 0.3), bar("mast", at(ax * head, ay * head, zA), at(bx * head, by * head, zA), 0.35))
    out.push(bar("mast", at(ax * foot, ay * foot, zD), at(bx * mid, by * mid, zm), 0.22), bar("mast", at(bx * foot, by * foot, zD), at(ax * mid, ay * mid, zm), 0.22))
    out.push(bar("mast", at(ax * mid, ay * mid, zm), at(bx * head, by * head, zA), 0.22), bar("mast", at(bx * mid, by * mid, zm), at(ax * head, ay * head, zA), 0.22))
  }
  out.push(box("mast", TW.e, TW.n, zA + 0.2, 2 * head + 0.5, 2 * head + 0.5, 0.4))
  // lightning rod to TOWER.height [T, W]
  out.push(bar("rail", at(0, 0, zA + 0.4), at(0, 0, zA + 4), 0.28), bar("rail", at(0, 0, zA + 4), at(0, 0, TOWER.height), 0.12))
  // jib girders and sheaves
  const zB = T + 1.25, zT = T + 4.3, nS = TW.n - 8.6, nN = TW.n + 7.5, nF = DROP_S + CROWN.sheaveR, nB = DROP_N - CROWN.sheaveR, half = 3.2
  for (const s of [-1, 1]) {
    const e = TW.e + s * CROWN.jibE
    out.push(bar("mast", enu(e, nS, zB), enu(e, nN, zB), 0.5, 0.9), bar("mast", enu(e, TW.n - half, zT), enu(e, TW.n + half, zT), 0.4))
    out.push(bar("mast", enu(e, TW.n - half, zT), enu(e, nS + 0.3, zB + 0.4), 0.34), bar("mast", enu(e, TW.n + half, zT), enu(e, nN - 0.3, zB + 0.4), 0.34))
    for (const dn of [-half, half]) out.push(bar("mast", enu(e, TW.n + dn, zB), enu(e, TW.n + dn, zT), 0.3))
    out.push(bar("mast", enu(e, TW.n - half, zB), enu(e, TW.n + half, zT), 0.24), bar("mast", enu(e, TW.n + half, zB), enu(e, TW.n - half, zT), 0.24))
    const es = TW.e + s * CROWN.sheaveE
    for (const n of [nF, nB]) {
      out.push(box("mast", e, n, zB + 1.1, 0.3, 1, 1.6))
      out.push(tube("rail", enu(es - 0.25, n, CROWN.sheaveU), enu(es + 0.25, n, CROWN.sheaveU), 2 * CROWN.sheaveR))
    }
  }
  for (const n of [nF, nB]) out.push(tube("rail", enu(TW.e - CROWN.jibE, n, CROWN.sheaveU), enu(TW.e + CROWN.jibE, n, CROWN.sheaveU), 0.36))
  // weather station on the NE corner [W]
  const we = TW.e + 5.2, wn = TW.n + 5.2
  out.push(bar("grate", enu(we, wn, zD), enu(we, wn, T + 5), 0.14), bar("grate", enu(we - 0.9, wn, T + 4.8), enu(we + 0.9, wn, T + 4.8), 0.06), box("grate", we, wn, T + 5.15, 0.3, 0.3, 0.3))
  return out
}

/**
 * Hoist cables [W: winch at the base, pulley at the top]: dead-ended on the carriage's south girder,
 * up the south face, over the two crown sheaves, down the north face into a covered duct that runs
 * to the drawworks house.
 */
const hoistCables = (): Member[] => {
  const out: Member[] = []
  const sU = CROWN.sheaveU, r = CROWN.sheaveR
  for (const s of [-1, 1]) {
    const e = TW.e + s * CROWN.sheaveE
    out.push(tube("cable", enu(e, DROP_S, sU), enu(e, DROP_S, CA.z1 + 0.6), 0.16))
    out.push(tube("cable", enu(e, DROP_S + r, sU + r), enu(e, DROP_N - r, sU + r), 0.16))
    out.push(tube("cable", enu(e, DROP_N, sU), enu(e, DROP_N, 0.6), 0.16))
    out.push(block("concrete", e - 0.45, e + 0.45, DROP_N - 0.35, DW.n0, 0.25, 0.9))
  }
  return out
}

/**
 * Carriage [R: dark portal frame on the tracks; W: cable-hoisted, bearings at the sides; E: sizes;
 * T: "carriage" and "traverse-beams" boxes]: a 15 m deep box frame wrapping the south, east and west
 * faces (three ring levels, corner and mid posts, X-bracing) tied across the north face, bogies on
 * the two tracks and side rollers on all four columns, a grated top deck with handrail, hydraulic
 * power units and accumulators, the hoist cable dead ends, and — the model's arm kinematics — the traverse
 * girder (two chords, posts and diagonals) the arm trolleys slide on, with the two actuator barrels.
 */
const carriage = (): Member[] => {
  const out: Member[] = []
  const R = CA.r, g = CA.girder
  const at = (de: number, dn: number, u: number): V => enu(TW.e + de, TW.n + dn, u)
  const lv = [CA.z0 + g / 2, TOWER.armHeight - 1.5, CA.z1 - g / 2]
  for (let c = 0; c < 4; c++) {
    const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4]
    const mx = (ax + bx) / 2, my = (ay + by) / 2
    out.push(bar("black", at(ax * R, ay * R, CA.z0), at(ax * R, ay * R, CA.z1), g, g))
    if (c === 2) {
      // north face: open, two tie girders only
      for (const z of [lv[0], lv[2]]) out.push(bar("black", at(ax * R, ay * R, z), at(bx * R, by * R, z), 0.7, 0.7))
      continue
    }
    for (const z of lv) out.push(bar("black", at(ax * R, ay * R, z), at(bx * R, by * R, z), g, g))
    out.push(bar("black", at(mx * R, my * R, CA.z0), at(mx * R, my * R, CA.z1), 0.8, 0.8))
    const bays = [[ax, ay, mx, my], [mx, my, bx, by]] as const
    for (let t = 0; t < 2; t++) for (const [p0, p1, q0, q1] of bays) {
      out.push(bar("black", at(p0 * R, p1 * R, lv[t]), at(q0 * R, q1 * R, lv[t + 1]), 0.45))
      out.push(bar("black", at(q0 * R, q1 * R, lv[t]), at(p0 * R, p1 * R, lv[t + 1]), 0.45))
    }
  }
  // bogies on the two tracks (south face) and side rollers on the four columns
  for (const z of [CA.z0 + 1.8, CA.z1 - 1.8]) for (const sx of [-1, 1]) {
    out.push(block("rail", TW.e + sx * CL - 0.65, TW.e + sx * CL + 0.65, TW.n - R + g / 2, TW.n - TW.half - 0.1, z - 1.2, z + 1.2))
    for (const sy of [-1, 1]) out.push(block("rail", TW.e + sx * TW.half, TW.e + sx * (R - g / 2), TW.n + sy * CL - 0.6, TW.n + sy * CL + 0.6, z - 1, z + 1))
  }
  // top deck (grating ring) with handrail
  const di = TW.half + 0.25, dO = R + g / 2, dz0 = CA.z1 - 0.2, dz1 = CA.z1
  out.push(
    block("grate", TW.e - dO, TW.e + dO, TW.n + di, TW.n + dO, dz0, dz1), block("grate", TW.e - dO, TW.e + dO, TW.n - dO, TW.n - di, dz0, dz1),
    block("grate", TW.e + di, TW.e + dO, TW.n - di, TW.n + di, dz0, dz1), block("grate", TW.e - dO, TW.e - di, TW.n - di, TW.n + di, dz0, dz1),
  )
  for (let c = 0; c < 4; c++) {
    const [ax, ay] = CORNERS[c], [bx, by] = CORNERS[(c + 1) % 4], h = dO - 0.1
    for (const du of [0.55, 1.1]) out.push(bar("grate", at(ax * h, ay * h, CA.z1 + du), at(bx * h, by * h, CA.z1 + du), 0.06))
    for (let k = 0; k < 8; k++) {
      const t = k / 8
      out.push(box("grate", TW.e + (ax + (bx - ax) * t) * h, TW.n + (ay + (by - ay) * t) * h, CA.z1 + 0.55, 0.07, 0.07, 1.1))
    }
  }
  // hydraulic power units on the north deck, accumulator bottles on the side decks, hoist cable dead ends on the south girder
  for (const s of [-1, 1]) {
    out.push(box("mast", TW.e + s * 5.2, TW.n + 7.2, CA.z1 + 1.2, 2.6, 1.8, 2.4))
    for (let j = 0; j < 3; j++) out.push(tube("rail", at(s * 7.2, 2.4 + j * 0.95, CA.z1), at(s * 7.2, 2.4 + j * 0.95, CA.z1 + 2.4), 0.7))
    const es = TW.e + s * CROWN.sheaveE
    out.push(block("black", es - 0.4, es + 0.4, TW.n - R + g / 2, DROP_S + 0.05, CA.z1 - 0.6, CA.z1 + 0.7))
  }
  // traverse girder: two chords, end posts, web posts and diagonals behind the trolley lugs
  const w0 = BEAM_U[1] + 0.65, w1 = BEAM_U[0] - 0.65, bays = 8, dx = (2 * CA.beamHalf) / bays
  for (const u of BEAM_U) out.push(box("black", TW.e, BEAM_N, u, 2 * CA.beamHalf, 1.2, 1.3))
  for (const s of [-1, 1]) out.push(block("black", TW.e + s * (CA.beamHalf - 0.6), TW.e + s * CA.beamHalf, BEAM_N - 0.6, BEAM_N + 0.6, w0, w1))
  for (let j = 0; j < bays; j++) {
    const x0 = TW.e - CA.beamHalf + j * dx, up = j % 2 === 0
    if (j > 0) out.push(box("black", x0, WEB_N, (w0 + w1) / 2, 0.3, 0.3, w1 - w0))
    out.push(bar("black", enu(x0 + 0.2, WEB_N, up ? w0 : w1), enu(x0 + dx - 0.2, WEB_N, up ? w1 : w0), 0.26))
  }
  // actuators: anchor post between the chords, clevis, fixed barrel (the rods are animated meshes)
  for (const s of [-1, 1]) {
    const u = s > 0 ? ACT.uE : ACT.uW, xa = TW.e - s * ACT.anchor
    out.push(block("black", xa - 0.35, xa + 0.35, ACT.n + 0.3, BEAM_N + 0.6, w0, w1), block("black", xa - 0.35, xa + 0.35, ACT.n - 0.3, ACT.n + 0.3, u - 0.4, u + 0.4))
    out.push(tube("rail", enu(xa, ACT.n, u), enu(xa + s * ACT.barrel, ACT.n, u), 0.56))
  }
  return out
}

/**
 * One chopstick (s = −1 west, +1 east) in its group frame: group x = the rail's inner edge
 * (`armInnerAt(·, 0)`), e = s·o with o the outward offset, dz the depth below the rail top.
 *
 * Contact contract [T]: rail top at TOWER.armHeight; rail inner edge at o = 0; tip at the contact
 * model's yMin; nothing forward of the hinge bay is above the rail, below `armDepthAt(n)`, or inside
 * the inner face — o ≥ 0 over the rail thickness and o ≥ `armInnerAt` lean below it.
 *
 * Structure [R, E]: a three-chord taper truss of round tubes — two top chords (Ø 0.4 m) under the
 * rail and the walkway, one lower chord (Ø 0.44 m) following the wedge and the inner-face lean —
 * with a triangular frame at every panel point and Warren diagonals in both side planes and the top
 * plane; the landing rail (0.85 × 0.3 m) with a hazard-painted inner fascia; a rub plate lying in the
 * inner-face plane below the rail; a ladder walkway on the outboard half of the top; a narrow tip
 * with a round wear shoe; at the root the hinge boss (pin + two knuckles) and the lower-chord
 * bearing, then the trolley that hooks over the traverse chords and carries the actuator lug.
 */
const armMembers = (s: 1 | -1): Member[] => {
  const out: Member[] = []
  const top = TOWER.armHeight, W = TOWER.armWidth
  const at = (o: number, n: number, dz: number): V => enu(s * o, n, top - dz)
  const ab = (mat: MatKey, o0: number, o1: number, n0: number, n1: number, dz0: number, dz1: number): Member => block(mat, s * o0, s * o1, n0, n1, top - dz0, top - dz1)
  // ---- trolley: body in front of the girder, shoes over the upper and under the lower chord, actuator lug
  const actU = s > 0 ? ACT.uE : ACT.uW, back = ROOT_N + 1.4
  out.push(ab("black", -0.55, W + 0.55, ROOT_N, back, -2.4, 6.2))
  out.push(ab("black", -0.4, W + 0.4, back, BEAM_N + 0.2, -2.4, top - (BEAM_U[0] + 0.65)))
  out.push(ab("black", -0.4, W + 0.4, back, BEAM_N + 0.2, top - (BEAM_U[1] - 0.65), 6.2))
  out.push(ab("black", ACT.lug - 0.35, ACT.lug + 0.35, back, ACT.n + 0.3, top - actU - 0.4, top - actU + 0.4))
  // ---- truss
  const rC = ARM.chord / 2, rS = ARM.spine / 2
  /** top chord axes: under the rail; the inner one stands behind the rub plate */
  const dzT = ARM.rail + rC, oIn = behind(dzT, rC, ARM.pad + 0.02)
  const oOut = (n: number): number => (n < TAPER_N ? ARM.tipOut + ((W - rC - ARM.tipOut) * (n - TIP_N)) / (TAPER_N - TIP_N) : W - rC)
  /** lower chord axis: its underside on the wedge, its inner side on the inner face */
  const dzS = (n: number): number => armDepthAt(n) - rS - 0.03
  const tI = (n: number): V => at(oIn, n, dzT), tO = (n: number): V => at(oOut(n), n, dzT), sp = (n: number): V => at(behind(dzS(n), rS), n, dzS(n))
  const nTip = TIP_N + 0.4
  out.push(pipe("black", tI(ROOT_N), tI(nTip), ARM.chord), pipe("black", tO(ROOT_N), tO(TAPER_N), ARM.chord), pipe("black", tO(TAPER_N), tO(nTip), ARM.chord))
  const st = [ROOT_N, ...Array.from({ length: ARM.panels + 1 }, (_, i) => TRUSS_N - (i * (TRUSS_N - TRUSS_END)) / ARM.panels)]
  for (let i = 0; i < st.length; i++) {
    const n = st[i]
    out.push(pipe("black", tI(n), sp(n), ARM.post), pipe("black", tO(n), sp(n), ARM.post), pipe("black", tI(n), tO(n), ARM.rung))
    if (i === st.length - 1) continue
    const m = st[i + 1], even = i % 2 === 0
    out.push(pipe("black", sp(n), sp(m), ARM.spine))
    out.push(even ? pipe("black", tI(n), sp(m), ARM.diag) : pipe("black", sp(n), tI(m), ARM.diag))
    out.push(even ? pipe("black", tO(n), sp(m), ARM.diag) : pipe("black", sp(n), tO(m), ARM.diag))
    out.push(even ? pipe("black", tI(n), tO(m), ARM.diag) : pipe("black", tO(n), tI(m), ARM.diag))
  }
  // ---- landing rail (top = TOWER.armHeight, inner edge o = 0) with its hazard fascia
  out.push(ab("rail", ARM.edge, ARM.railW, TIP_N, ROOT_N, 0, ARM.rail))
  const segs = 20, dn = (ROOT_N - TIP_N) / segs
  for (let k = 0; k < segs; k++) out.push(ab("yellow", 0, ARM.edge, TIP_N + k * dn, TIP_N + (k + 1) * dn, 0, ARM.rail))
  // ---- rub plate in the inner-face plane (its inner surface 1 cm behind `armInnerAt`)
  const dzm = (ARM.padTop + ARM.padBot) / 2, q = ARM.pad / 2 + 0.01
  const po = innerLean(dzm) + q / LEAN_SEC, pz = dzm - (q * LEAN_K) / LEAN_SEC
  out.push(bar("rail", at(po, TIP_N + 3, pz), at(po, TRUSS_N, pz), (ARM.padBot - ARM.padTop) * LEAN_SEC, ARM.pad, enu(s, 0, LEAN_K)))
  // ---- ladder walkway on the outboard half of the top (stringers on the frame rungs, below the rail top)
  const wIn = ARM.railW + 0.1, wOut = W - 0.1
  for (const o of [wIn, wOut]) out.push(bar("rail", at(o, ROOT_N, ARM.rail + 0.05), at(o, TAPER_N, ARM.rail + 0.05), 0.1))
  for (let n = ROOT_N - 0.4; n > TAPER_N; n -= 0.8) out.push(ab("rail", wIn - 0.05, wOut + 0.05, n - 0.15, n + 0.15, ARM.rail - 0.05, ARM.rail))
  // ---- tip: solid shoe under the rail with a round nose ending exactly at the tip station
  const dTip = armDepthAt(TIP_N) - 0.02, sIn = innerLean(dTip) + 0.02, sOut = ARM.tipOut + rC, rN = (sOut - sIn) / 2
  out.push(ab("black", sIn, sOut, TIP_N + rN, TIP_N + 1.1, ARM.rail, dTip))
  out.push(tube("rail", at((sIn + sOut) / 2, TIP_N + rN, dTip), at((sIn + sOut) / 2, TIP_N + rN, ARM.rail), 2 * rN))
  // ---- hinge boss between the top chords (vertical pin, two knuckles, clevis plate from the trolley)
  const hN = ROOT_N - ARM.hingeN, hO = ARM.hingeO
  out.push(tube("rail", at(hO, hN, 1.75), at(hO, hN, ARM.rail), 0.76))
  out.push(tube("rail", at(hO, hN, 0.62), at(hO, hN, 0.32), 1), tube("rail", at(hO, hN, 1.45), at(hO, hN, 1.15), 1))
  out.push(ab("black", hO - 0.45, W, hN, ROOT_N, 0.66, 1.1))
  // ---- lower-chord bearing at the trolley
  const rB = 0.32, bz = (n: number): number => armDepthAt(n) - rB - 0.03
  out.push(tube("rail", at(behind(bz(ROOT_N - 0.7), rB), ROOT_N - 0.7, bz(ROOT_N - 0.7)), at(behind(bz(ROOT_N), rB), ROOT_N, bz(ROOT_N)), 2 * rB))
  return out
}

/** box truss of round tubes between a and b: four chords, a frame at every panel point, Warren diagonals on the four faces */
const boxTruss = (mat: MatKey, a: V, b: V, side: V, hw: number, hh: number, panels: number, chord: number, web: number): Member[] => {
  const out: Member[] = []
  const up = new THREE.Vector3().crossVectors(side, b.clone().sub(a).normalize()).normalize()
  const p = (t: number, i: number, j: number): V => a.clone().lerp(b, t).addScaledVector(side, i * hw).addScaledVector(up, j * hh)
  for (const i of [-1, 1]) for (const j of [-1, 1]) out.push(pipe(mat, p(0, i, j), p(1, i, j), chord))
  for (let k = 0; k <= panels; k++) {
    const t = k / panels
    for (const i of [-1, 1]) out.push(pipe(mat, p(t, i, -1), p(t, i, 1), web))
    for (const j of [-1, 1]) out.push(pipe(mat, p(t, -1, j), p(t, 1, j), web))
    if (k === panels) continue
    const t2 = (k + 1) / panels, f = k % 2 === 0 ? 1 : -1
    for (const i of [-1, 1]) out.push(pipe(mat, p(t, i, -f), p(t2, i, f), web * 0.85))
    for (const j of [-1, 1]) out.push(pipe(mat, p(t, -f, j), p(t2, f, j), web * 0.85))
  }
  return out
}

/**
 * Ship QD arm, parked [R, E; T: "ship-qd-arm" box]: a shoe on the west track of the SW column, two
 * brackets to the hinge post, the upper arm (box truss 2.1 × 2 m × 10.3 m), the elbow pin with cheek
 * plates, the forearm (box truss, dropping 1.2 m over 7.6 m), the QD head with its plate facing the
 * vehicle side, a swing cylinder under the arm, and the three propellant hoses that leave the tower
 * through the south face and hang under the north lower chord.
 */
const qdMembers = (): Member[] => {
  const out: Member[] = []
  const { n, u } = QD
  // shoe on the track: upper block, back plate, lower block (the hoses pass through the slot)
  const sE0 = TW.e - TW.half - 0.05, sE1 = TW.e - 5, sN1 = TW.n - TW.half - 0.1
  out.push(block("black", sE0, sE1, sN1 - 0.6, sN1, u - 1.3, u + 1.25), block("black", sE0, sE1, sN1 - 0.08, sN1, u - 2.6, u - 1.3), block("black", sE0, sE1, sN1 - 0.6, sN1, u - 3.5, u - 2.6))
  for (const du of [-0.7, 0.7]) out.push(bar("black", enu(TW.e - 5.5, sN1 - 0.3, u + du), enu(QD.hingeE, n, u + du), 0.7, 0.5))
  out.push(tube("rail", enu(QD.hingeE, n, u - 1.25), enu(QD.hingeE, n, u + 1.25), 1))
  // upper arm, elbow, forearm
  out.push(...boxTruss("black", enu(QD.e0, n, u), enu(QD.e1, n, u), NORTH, QD.hw, QD.hh, QD.panels, 0.3, 0.2))
  out.push(tube("rail", enu(QD.elbowE, n - QD.hw - 0.25, u), enu(QD.elbowE, n + QD.hw + 0.25, u), 0.9))
  for (const sn of [-1, 1]) out.push(box("black", QD.elbowE, n + sn * QD.hw, u, 1.5, 0.16, 1.5))
  const fU = u - 0.1
  out.push(...boxTruss("black", enu(QD.f0, n, fU), enu(QD.f1, n, fU - QD.drop), NORTH, QD.fhw, QD.fhh, QD.fpanels, 0.26, 0.18))
  // QD head and plate
  out.push(block("black", QD.f1 - 2.4, QD.f1, n - 1.3, n + 1.3, 93.3, u))
  out.push(block("rail", QD.f1 - 2.9, QD.f1 + 0.3, n - 1.52, n - 1.3, 93.3, u + 0.2))
  // swing cylinder under the arm: lower shoe → lug under the south lower chord
  const lugE = QD.e0 - 5.2, lugN = n - QD.hw
  out.push(tube("rail", enu(TW.e - 5.6, sN1 - 0.35, u - 3.05), enu(lugE, lugN, u - 2.4), 0.4))
  out.push(bar("black", enu(lugE, lugN, u - QD.hh), enu(lugE, lugN, u - 2.55), 0.3))
  // hoses: tower riser → south face → along the arm under the north lower chord → down the forearm
  for (let i = 0; i < 3; i++) {
    const z = QD.hoseU - i * QD.hoseStep, eS = TW.e - 4.25 + 0.35 * i
    out.push(...hoseRun("black", [
      enu(RISER_E, PIPES[i].n, z), enu(eS, PIPES[i].n, z), enu(eS, QD.hoseN, z), enu(QD.e1 - 0.4, QD.hoseN, z), enu(QD.f1 + 0.2, QD.hoseN, z - QD.drop),
    ], QD.hose))
  }
  // hose clips under the north lower chords
  for (let k = 0; k <= QD.panels; k++) {
    const e = QD.e0 + ((QD.e1 - QD.e0) * k) / QD.panels
    out.push(block("black", e - 0.04, e + 0.04, QD.hoseN - 0.25, n + QD.hw, QD.hoseU - 2 * QD.hoseStep - 0.25, u - QD.hh))
  }
  for (let k = 0; k <= QD.fpanels; k++) {
    const t = k / QD.fpanels, e = QD.f0 + (QD.f1 - QD.f0) * t, dz = QD.drop * t
    out.push(block("black", e - 0.04, e + 0.04, QD.hoseN - 0.25, n + QD.fhw, QD.hoseU - 2 * QD.hoseStep - 0.25 - dz, fU - QD.fhh - dz))
  }
  return out
}

/**
 * Propellant risers for the ship QD (LOX, CH₄, LN₂): from the pipe rack into the west side of the
 * mast and up inside it to the QD level, where the hoses take over [E]. Site piping (site pool).
 */
const qdRisers = (): Member[] => {
  const out: Member[] = []
  for (let i = 0; i < 3; i++) {
    const p = PIPES[i], uR = RACK.u1 + 0.45 + p.dia / 2
    out.push(...pipeRun(p.mat, [enu(RACK.e1, p.n, uR), enu(RISER_E, p.n, uR), enu(RISER_E, p.n, QD.hoseU + 0.3)], p.dia))
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
  // two instance pools: the tower (mast, crown, carriage, traverse girder, QD arm) and the site
  const towerMembers: Member[] = [...towerLattice(rand), ...towerCore(), ...crown(), ...hoistCables(), ...carriage(), ...qdMembers()]
  const siteMembers: Member[] = [...qdRisers(), ...mountMembers(rand), ...pipeRack(), ...farm.members, ...env.members]
  const siteGroup = new THREE.Group()
  siteGroup.name = "site"
  siteGroup.add(...instance(siteMembers, mats, geos), ...mountMeshes(mats), ...farm.meshes, ...env.meshes)
  const tower = new THREE.Group()
  tower.name = "mechazilla"
  tower.add(...instance(towerMembers, mats, geos))
  // aviation obstruction lights: rod tip, crown corners, mid-height corners
  const beacons: V[] = [enu(TW.e, TW.n, TOWER.height - 0.28)]
  for (const [cx, cy] of CORNERS) beacons.push(enu(TW.e + cx * (TW.half - 0.2), TW.n + cy * (TW.half - 0.2), TW.top + 0.9), enu(TW.e + cx * (TW.half - 0.2), TW.n + cy * (TW.half - 0.2), TW.base + 7 * HM + 0.7))
  const lights = instancedAt(new THREE.SphereGeometry(0.28, 10, 8), mats.red, beacons)
  lights.castShadow = false
  lights.name = "beacons"
  tower.add(lights)
  root.add(siteGroup, tower)
  // chopsticks: one group per arm (direct children of the root), translated by setArms so that the
  // rail's inner edge sits at ±gap/2 about the pair centre [T: armInnerX]
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
    m.name = s < 0 ? "rod-west" : "rod-east"
    m.position.copy(enu(0, ACT.n, s > 0 ? ACT.uE : ACT.uW))
    m.scale.set(1, 0.38, 0.38)
    tower.add(m)
    return m
  })
  // `open`: 1 open … 0 closed (clamped; non-finite → open). `cx`: east offset of the arm pair's centre
  // (the tower-side tracking of the ship, [T] ArmState.cx; non-finite → 0)
  const setArms = (open: number, cx = 0): void => {
    const k = Math.min(1, Math.max(0, Number.isFinite(open) ? open : 1))
    const c = Number.isFinite(cx) ? cx : 0
    const gap = TOWER.closedGap + (TOWER.openGap - TOWER.closedGap) * k
    for (let i = 0; i < 2; i++) {
      const s = i === 0 ? -1 : 1
      arms[i].position.set(TW.e + c + (s * gap) / 2, 0, 0)
      const xb = TW.e - s * ACT.anchor + s * ACT.barrel, xl = TW.e + c + s * (gap / 2 + ACT.lug)
      const rod = rods[i]
      rod.position.x = (xb + xl) / 2
      rod.scale.x = Math.max(0.02, Math.abs(xl - xb))
      rod.visible = Math.abs(xl - xb) > 0.05
    }
  }
  setArms(1)
  return { root, setArms }
}
