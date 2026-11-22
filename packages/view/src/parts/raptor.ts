import * as THREE from "three"
import { boxGeo, cylBetween, Kit, latheGeo, MAT, quatY, type RX, torusX, tubeGeo, v3 } from "./kit"

/**
 * Raptor 3 (full-flow staged combustion), engine frame: gimbal point at the origin, nozzle along −x,
 * y/z parallel to the body axes (the engine group is only translated and gimballed by ship.ts).
 *
 * Dimensions (Wikipedia "SpaceX Raptor", research/05, public photos; computed where noted):
 * - SL: 3.1 m gimbal → exit, exit Ø 1.30 m; RVac: 4.3 m, exit Ø 2.30 m (research/05).
 * - throat r 0.13 m: A* = F/(C_F·p_c) = 2.75 MN/(1.7·35 MPa) = 0.046 m² → r* ≈ 0.12–0.13 m;
 *   ε_SL = (0.65/0.13)² = 25, ε_Vac = (1.15/0.13)² = 78.
 * - chamber r 0.27 m (contraction ≈ 4), powerhead envelope Ø ≈ 1.6 m incl. the two turbopumps.
 * - LOX turbopump + ox-rich preburner on +z, CH₄ turbopump + fuel-rich preburner on −z, each a
 *   vertical stack (preburner → turbine → pump), hot-gas elbows into the main injector manifold.
 * - SL: two electric TVC actuators (Ø 0.14 × ≈ 1.0 m) at 45° either side of +y, from a chamber
 *   clevis to the thrust structure; their pose is solved every frame so they stay attached to the
 *   fixed structure while the engine gimbals. RVac: two rigid struts (fixed engine).
 */
export type EngineKind = "SL" | "RVac"

export interface RaptorDims {
  readonly length: number // gimbal point → exit plane, m
  readonly exitR: number
  readonly throatX: number
  readonly throatR: number
  readonly thetaN: number // bell initial angle, rad
  readonly thetaE: number // exit angle, rad
  /** wall + channel thickness of the regen jacket, m */
  readonly wall: number
  /** fraction of the bell (from the throat) that is regen-cooled; the rest is the Nb extension */
  readonly regen: number
}

export const RAPTOR: Record<EngineKind, RaptorDims> = {
  SL: { length: 3.1, exitR: 0.65, throatX: -1.42, throatR: 0.13, thetaN: (32 * Math.PI) / 180, thetaE: (8 * Math.PI) / 180, wall: 0.014, regen: 1 },
  RVac: { length: 4.3, exitR: 1.15, throatX: -1.42, throatR: 0.13, thetaN: (36 * Math.PI) / 180, thetaE: (5 * Math.PI) / 180, wall: 0.018, regen: 0.3 },
}

/** TVC actuator (SL) / strut (RVac) directions in the engine y–z plane: 45° either side of +y */
export const TVC_DIRS: readonly THREE.Vector3[] = [v3(0, Math.SQRT1_2, Math.SQRT1_2), v3(0, Math.SQRT1_2, -Math.SQRT1_2)]
/** clevis on the engine (x, radial offset) and anchor on the fixed thrust structure (x, radial offset) */
export const MOUNT: Record<EngineKind, { readonly clevis: readonly [number, number]; readonly anchor: readonly [number, number] }> = {
  SL: { clevis: [-0.95, 0.3], anchor: [-0.08, 0.9] },
  RVac: { clevis: [-1.05, 0.3], anchor: [-0.55, 0.85] },
}
/** anchor point of actuator k in the (unrotated) engine frame */
export const tvcAnchor = (kind: EngineKind, k: number): THREE.Vector3 => {
  const [x, r] = MOUNT[kind].anchor
  return TVC_DIRS[k].clone().multiplyScalar(r).setX(x)
}
/** propellant inlet flanges (top of the inlet ducts) in the engine frame: LOX on +z, CH₄ on −z */
export const INLETS = { lox: v3(0.02, 0, 0.8), ch4: v3(0.02, 0, -0.78) } as const

/** turbopump stack lateral offsets and radius scale */
const STACK = { lox: { z: 0.6, k: 1.0 }, ch4: { z: -0.58, k: 0.86 } } as const

/** Rao-style bell as a quadratic Bézier from the throat to the exit, (r, x) ordered by increasing x */
export const bellProfile = (d: RaptorDims, n = 22, r0 = 0): RX[] => {
  const Ln = d.throatX + d.length
  const tn = Math.tan(d.thetaN), te = Math.tan(d.thetaE)
  const ac = (d.exitR - d.throatR - Ln * te) / (tn - te)
  const rc = d.throatR + ac * tn
  const pts: RX[] = []
  for (let i = 0; i <= n; i++) {
    const t = Math.pow(i / n, 1.25)
    const a = 2 * (1 - t) * t * ac + t * t * Ln
    const r = (1 - t) * (1 - t) * d.throatR + 2 * (1 - t) * t * rc + t * t * d.exitR
    pts.push([r + r0, d.throatX - a])
  }
  return pts.reverse()
}
/** bell radius at axial fraction f ∈ [0, 1] from the throat */
export const bellR = (d: RaptorDims, f: number): number => {
  const p = bellProfile(d, 200)
  const x = d.throatX - f * (d.throatX + d.length)
  for (let i = 0; i < p.length - 1; i++) if (p[i][1] <= x && p[i + 1][1] >= x) {
    const u = (x - p[i][1]) / (p[i + 1][1] - p[i][1])
    return p[i][0] + u * (p[i + 1][0] - p[i][0])
  }
  return d.exitR
}

/** heat tint along the regen jacket: bronze/straw near the throat → bare superalloy toward the exit */
const tint = (f: number, out: THREE.Color): THREE.Color => {
  const a = new THREE.Color(0.62, 0.44, 0.27), b = new THREE.Color(0.5, 0.48, 0.47), c = new THREE.Color(0.4, 0.39, 0.385)
  return f < 0.35 ? out.copy(a).lerp(b, f / 0.35) : out.copy(b).lerp(c, (f - 0.35) / 0.65)
}

const turbopump = (kit: Kit, z: number, k: number): void => {
  // one vertical stack about an axis parallel to x at (y 0, z): pump → turbine → preburner (bottom → top)
  const prof: RX[] = [
    [0.0, -1.2], [0.07, -1.19], [0.11 * k, -1.16], [0.135 * k, -1.11], [0.175 * k, -1.03], [0.195 * k, -0.94], [0.19 * k, -0.86], [0.16 * k, -0.78],
    [0.14 * k, -0.73], [0.2 * k, -0.695], [0.208 * k, -0.62], [0.172 * k, -0.555], [0.122 * k, -0.52], [0.115 * k, -0.49], [0.115 * k, -0.25],
    [0.1 * k, -0.21], [0.06 * k, -0.182], [0.0, -0.172],
  ]
  const at = new THREE.Matrix4().makeTranslation(0, 0, z)
  kit.add(latheGeo(prof, 18), MAT.alloy, at)
  kit.add(torusX(0.19 * k, 0.062 * k, -0.915, 6, 20), MAT.alloy, at) // pump discharge volute
  kit.add(torusX(0.19 * k, 0.05 * k, -0.625, 6, 20), MAT.alloyDark, at) // turbine exhaust collector
  kit.add(torusX(0.118 * k, 0.018, -0.255, 4, 16), MAT.steelDark, at) // preburner injector flange
  kit.add(torusX(0.118 * k, 0.014, -0.47, 4, 16), MAT.steelDark, at) // preburner/turbine flange
  kit.add(cylBetween(v3(-0.19, 0, z), v3(-0.1, 0, z), 0.022, 0.022, 8), MAT.steelDark) // igniter
}

/** telescoping electric actuator whose ends are re-solved every frame (engine clevis ↔ fixed anchor) */
const actuator = (engine: THREE.Group, clevis: THREE.Vector3, anchor: THREE.Vector3): THREE.Mesh[] => {
  const bk = new Kit(), rk = new Kit()
  // barrel (on the structure side): eye, motor housing, gearbox, side motor — built along +Y from the anchor
  bk.add(new THREE.SphereGeometry(0.045, 8, 6), MAT.steelDark)
  bk.add(cylBetween(v3(0, 0.03, 0), v3(0, 0.58, 0), 0.07, 0.066, 12), MAT.steelDark)
  bk.add(cylBetween(v3(0, 0.1, 0), v3(0, 0.22, 0), 0.085, 0.085, 12), MAT.black)
  bk.add(cylBetween(v3(0.12, 0.06, 0), v3(0.12, 0.34, 0), 0.05, 0.05, 12), MAT.black)
  bk.add(boxGeo(0.12, 0.08, 0.08, new THREE.Matrix4().makeTranslation(0.07, 0.16, 0)), MAT.black)
  // rod (on the engine side): rod-end eye and chrome rod along +Y from the clevis
  rk.add(new THREE.SphereGeometry(0.04, 8, 6), MAT.steelDark)
  rk.add(cylBetween(v3(0, 0.02, 0), v3(0, 0.64, 0), 0.032, 0.032, 10), MAT.chrome)
  const barrel = bk.build("tvc-barrel").children as THREE.Mesh[]
  const rod = rk.build("tvc-rod").children as THREE.Mesh[]
  const fixed = new THREE.Matrix4(), tmp = new THREE.Matrix4(), q0 = new THREE.Quaternion()
  const up = new THREE.Vector3(), lo = new THREE.Vector3(), d = new THREE.Vector3(), sc = new THREE.Vector3()
  const pose = (): void => {
    // fixed frame = grandparent world × translation(gimbal group) × engine local: the gimbal rotation removed
    fixed.copy(engine.matrix)
    const g = engine.parent
    if (g) { tmp.compose(g.position, q0, g.scale); fixed.premultiply(tmp); if (g.parent) fixed.premultiply(g.parent.matrixWorld) }
    up.copy(anchor).applyMatrix4(fixed)
    lo.copy(clevis).applyMatrix4(engine.matrixWorld)
    const s = engine.matrixWorld.getMaxScaleOnAxis()
    sc.set(s, s, s)
    d.subVectors(lo, up)
    for (const m of barrel) m.matrixWorld.compose(up, quatY(d), sc)
    d.negate()
    for (const m of rod) m.matrixWorld.compose(lo, quatY(d), sc)
  }
  const all = [...barrel, ...rod]
  for (const m of all) {
    m.matrixAutoUpdate = false
    m.matrixWorldAutoUpdate = false
    m.frustumCulled = false
    m.onBeforeRender = () => pose()
    m.onBeforeShadow = (_r, obj, _c, shadowCamera) => { pose(); obj.modelViewMatrix.multiplyMatrices(shadowCamera.matrixWorldInverse, obj.matrixWorld) }
  }
  return all
}

/**
 * Raptor 3 SL or Vac. Returns the group (gimbal point at the origin, nozzle along −x), the length
 * gimbal → exit plane and the exit radius (used by ship.ts for the plumes).
 */
export const makeRaptor = (kind: EngineKind): { group: THREE.Group; length: number; exitR: number } => {
  const d = RAPTOR[kind]
  const group = new THREE.Group()
  group.name = kind === "SL" ? "21 Raptor 3 SL" : "22 Raptor 3 Vac"
  const kit = new Kit()
  const tx = d.throatX, w = d.wall
  // ---- gimbal block (universal joint), upper yoke flange
  kit.add(boxGeo(0.2, 0.2, 0.2), MAT.steelDark)
  kit.add(cylBetween(v3(0, -0.17, 0), v3(0, 0.17, 0), 0.045, 0.045, 8), MAT.chrome)
  kit.add(cylBetween(v3(0, 0, -0.17), v3(0, 0, 0.17), 0.045, 0.045, 8), MAT.chrome)
  kit.add(latheGeo([[0.0, 0.06], [0.2, 0.06], [0.215, 0.085], [0.2, 0.11], [0.0, 0.11]], 20), MAT.steelDark)
  // ---- thrust post → main injector / hot-gas manifold dome → MCC → convergent section (one lathe, bottom → top)
  const body: RX[] = [
    [d.throatR + w, tx], [0.152, tx + 0.05], [0.19, tx + 0.13], [0.245, tx + 0.24], [0.27, tx + 0.33], [0.27, -0.72],
    [0.33, -0.68], [0.4, -0.61], [0.408, -0.53], [0.4, -0.46], [0.36, -0.385], [0.3, -0.345], [0.22, -0.3], [0.15, -0.22], [0.11, -0.12],
  ]
  kit.add(latheGeo(body, 28), MAT.alloy)
  kit.add(torusX(0.41, 0.055, -0.53, 6, 32), MAT.alloyDark) // fuel-rich gas manifold
  kit.add(torusX(0.405, 0.02, -0.62, 4, 32), MAT.steelDark) // injector flange
  kit.add(torusX(0.273, 0.013, -0.8, 4, 28), MAT.steelDark) // chamber jacket ribs
  kit.add(torusX(0.273, 0.013, -0.97, 4, 28), MAT.steelDark)
  // ---- turbopump stacks and their plumbing
  for (const prop of ["lox", "ch4"] as const) {
    const { z, k } = STACK[prop]
    const s = Math.sign(z)
    turbopump(kit, z, k)
    // inlet duct: from the flange at the top, down the outside of the stack, J-turn into the inducer from below
    const zi = INLETS[prop].z
    const duct = [v3(0.02, 0, zi), v3(-0.3, 0, zi + s * 0.06), v3(-0.75, 0, zi + s * 0.08), v3(-1.1, 0, zi + s * 0.04), v3(-1.27, 0, z + s * 0.12), v3(-1.27, 0, z), v3(-1.19, 0, z)]
    kit.add(tubeGeo(duct, 0.1 * k, 20, 8), MAT.alloy)
    kit.add(torusX(0.1 * k + 0.02, 0.022, 0.02, 4, 14).translate(0, 0, zi), MAT.steelDark) // inlet flange
    for (const xb of [-0.05, -0.09, -0.13]) kit.add(torusX(0.1 * k + 0.008, 0.012, xb, 3, 12).translate(0, 0, zi + s * 0.01), MAT.steelDark) // gimbal bellows
    // hot-gas elbow: turbine exhaust → main injector manifold
    kit.add(tubeGeo([v3(-0.6, 0, z - s * 0.12), v3(-0.53, 0, z - s * 0.22), v3(-0.44, 0, s * 0.35), v3(-0.38, 0, s * 0.27)], 0.085 * k, 12, 8), MAT.alloyDark)
  }
  const zl = STACK.lox.z, zc = STACK.ch4.z
  // LOX: pump discharge → main oxidiser valve → ox-rich preburner (all LOX passes the ORPB in FFSC)
  kit.add(tubeGeo([v3(-0.91, -0.16, zl + 0.2), v3(-0.72, -0.24, zl + 0.25), v3(-0.38, -0.22, zl + 0.2), v3(-0.13, -0.12, zl + 0.06), v3(-0.15, 0, zl)], 0.05, 16, 6), MAT.alloy)
  kit.add(boxGeo(0.15, 0.13, 0.13, new THREE.Matrix4().makeTranslation(-0.6, -0.27, zl + 0.25)), MAT.black) // MOV actuator
  // CH₄: pump discharge → main fuel valve → regen inlet manifold on the nozzle; jacket outlet → fuel-rich preburner
  const fMan = d.regen < 1 ? 0.12 : 0.42
  const xMan = tx - fMan * (tx + d.length), rMan = bellR(d, fMan) + w
  kit.add(tubeGeo([v3(-0.91, -0.16, zc - 0.18), v3(-1.25, -0.22, zc - 0.14), v3(-1.7, -0.2, -rMan - 0.12), v3(xMan + 0.04, -0.12, -rMan - 0.04)], 0.048, 16, 6), MAT.alloy)
  kit.add(boxGeo(0.14, 0.12, 0.12, new THREE.Matrix4().makeTranslation(-1.25, -0.3, zc - 0.14)), MAT.black) // MFV actuator
  kit.add(tubeGeo([v3(-0.74, -0.12, -0.26), v3(-0.58, -0.2, -0.42), v3(-0.3, -0.16, zc + 0.02), v3(-0.14, -0.06, zc), v3(-0.16, 0, zc)], 0.045, 14, 6), MAT.alloy)
  // engine controller + harness
  kit.add(boxGeo(0.26, 0.12, 0.2, new THREE.Matrix4().makeTranslation(-0.3, -0.46, 0)), MAT.black)
  kit.add(tubeGeo([v3(-0.3, -0.52, 0.08), v3(-0.45, -0.45, 0.45), v3(-0.6, -0.36, zl + 0.2)], 0.016, 10, 4), MAT.harness)
  kit.add(tubeGeo([v3(-0.3, -0.52, -0.08), v3(-0.8, -0.45, -0.5), v3(-1.2, -0.36, zc - 0.14)], 0.016, 10, 4), MAT.harness)
  // ---- nozzle: regen jacket (vertex heat tint + channel ribs), hot-gas wall, exit lip
  const Ln = tx + d.length
  const full = bellProfile(d, kind === "SL" ? 18 : 24)
  const xSplit = tx - d.regen * Ln
  const outerRegen = bellProfile(d, kind === "SL" ? 18 : 24, w).filter(([, x]) => x >= xSplit - 1e-6)
  if (d.regen < 1) outerRegen.unshift([bellR(d, d.regen) + w, xSplit])
  const jacket = latheGeo(outerRegen, kind === "SL" ? 36 : 40)
  const col = new Float32Array(jacket.attributes.position.count * 3)
  const c = new THREE.Color()
  for (let i = 0; i < jacket.attributes.position.count; i++) {
    tint((tx - jacket.attributes.position.getX(i)) / Ln, c)
    col[i * 3] = c.r; col[i * 3 + 1] = c.g; col[i * 3 + 2] = c.b
  }
  jacket.setAttribute("color", new THREE.BufferAttribute(col, 3))
  const jm = new THREE.Mesh(jacket, MAT.nozzle)
  jm.name = `${group.name} nozzle (regen jacket)`
  kit.obj(jm)
  kit.add(latheGeo(full.filter(([, x]) => x >= xSplit - 1e-6).reverse(), kind === "SL" ? 28 : 36), MAT.soot) // hot-gas wall, normals toward the axis
  if (d.regen < 1) {
    // Nb extension with four hat-section stiffener bands (spec/07 row 22)
    kit.add(latheGeo(full.filter(([, x]) => x <= xSplit + 1e-6), 48), MAT.niobium)
  }
  // ---- TVC actuators (SL, live) or rigid struts (RVac)
  for (let k = 0; k < 2; k++) {
    const [cx, cr] = MOUNT[kind].clevis
    const clevis = TVC_DIRS[k].clone().multiplyScalar(cr).setX(cx)
    const anchor = tvcAnchor(kind, k)
    kit.add(boxGeo(0.1, 0.09, 0.09, new THREE.Matrix4().compose(TVC_DIRS[k].clone().multiplyScalar(cr - 0.03).setX(cx), quatY(TVC_DIRS[k]), new THREE.Vector3(1, 1, 1))), MAT.steelDark) // clevis bracket
    if (kind === "SL") for (const m of actuator(group, clevis, anchor)) kit.obj(m)
    else {
      kit.add(cylBetween(clevis, anchor, 0.045, 0.045, 10), MAT.steelDark)
      kit.add(new THREE.SphereGeometry(0.05, 8, 6).translate(anchor.x, anchor.y, anchor.z), MAT.steelDark)
    }
  }
  const built = kit.build(group.name)
  for (const ch of [...built.children]) group.add(ch)
  return { group, length: d.length, exitR: d.exitR }
}
