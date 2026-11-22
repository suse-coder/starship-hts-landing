import * as THREE from "three"
import { axialFrame, boxGeo, cylBetween, cylX, deg, domeProfile, Kit, latheGeo, loopCCW, MAT, polar, type RX, torusX, tubeGeo, v3 } from "./kit"
import { bayCeiling, commonDomeX, type Ctx, fwdDomeX, LAYOUT, type PartSpec } from "./layout"
import { tvcAnchor } from "./raptor"

/**
 * 10 STRUCTURE (spec/07 rows 11–18) and the 41 PICA-X closeout. Dimensions: research/05 (21 rings ×
 * 1.83 m, 3.97 mm 301 steel, Ø 9 m), the spec/07 part table, and the shared LAYOUT.
 */

const R = 4.5

/** 11 weld seams: 20 circumferential beads (x = 1.83 k, 32 mm wide, 6 mm proud) + one longitudinal seam per rolled ring */
const seams = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  for (let k = 1; k < LAYOUT.rings; k++) {
    const x = k * LAYOUT.ringH
    kit.add(latheGeo([[R - 0.001, x - 0.016], [R + 0.0058, x], [R - 0.001, x + 0.016]], 96), MAT.weld)
  }
  const xPica = c.v.pica.x0 - 0.02
  for (let k = 0; k < LAYOUT.rings; k++) {
    const x0 = k * LAYOUT.ringH + 0.02, x1 = Math.min((k + 1) * LAYOUT.ringH - 0.02, xPica)
    const phi = deg((k * 137.508 + 20) % 360) // rolled-ring seams clocked from ring to ring
    kit.add(boxGeo(x1 - x0, 0.011, 0.022), MAT.weld, axialFrame((x0 + x1) / 2, phi, R + 0.0025))
  }
  return kit.build("seams")
}

/** 12 shield deck closing the six engine compartments at x 1.8 (seen from below) + aft skirt edge ring */
const shieldDeck = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const shape = new THREE.Shape().absarc(0, 0, LAYOUT.rIn - 0.005, 0, Math.PI * 2, false)
  // SL holes r 0.85: nozzle r 0.33 at the deck + 15° gimbal swing 0.47 m; RVac r 0.88: bell r 0.78 there
  const holes = c.v.engines.map((e) => ({ y: e.gimbal.y, z: e.gimbal.z, r: e.kind === "SL" ? 0.85 : 0.88 }))
  for (const h of holes) shape.holes.push(new THREE.Path().absarc(h.y, h.z, h.r, 0, Math.PI * 2, true))
  const deck = new THREE.ExtrudeGeometry(shape, { depth: 0.04, bevelEnabled: false, curveSegments: 18 })
  // shape (x, y) → body (y, z); extrusion z → body x
  deck.applyMatrix4(new THREE.Matrix4().set(0, 0, 1, LAYOUT.shieldDeck - 0.02, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1))
  kit.add(deck, MAT.steelDarkShell)
  // underside stiffeners between the holes (outboard of each SL hole, inboard of each RVac hole)
  for (const e of c.v.engines) {
    const phi = Math.atan2(e.gimbal.z, e.gimbal.y)
    const [r0, r1] = e.kind === "SL" ? [2.2, 4.42] : [0.12, 2.26]
    kit.add(boxGeo(0.07, r1 - r0, 0.05), MAT.steelDark, axialFrame(LAYOUT.shieldDeck - 0.055, phi, (r0 + r1) / 2))
  }
  return kit.build("shieldDeck")
}

/** wall of a shield can (radius rad about the engine axis) from xBot up to the bay ceiling */
const canGeo = (cy: number, cz: number, rad: number, xBot: number, seg = 40, xMinTop = -Infinity): THREE.BufferGeometry => {
  const pos: number[] = [], uv: number[] = [], idx: number[] = []
  for (let i = 0; i <= seg; i++) {
    const a = (i / seg) * Math.PI * 2
    const y = cy + rad * Math.cos(a), z = cz + rad * Math.sin(a)
    const top = Math.max(bayCeiling(Math.hypot(y, z)) - 0.015, xMinTop)
    pos.push(xBot, y, z, top, y, z)
    uv.push(i / seg, 0, i / seg, 1)
  }
  for (let i = 0; i < seg; i++) { const a = i * 2; idx.push(a, a + 2, a + 1, a + 1, a + 2, a + 3) }
  const g = new THREE.BufferGeometry()
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3))
  g.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2))
  g.setIndex(idx)
  g.computeVertexNormals()
  return g
}



/** 12 purge vents ×24: louvred grilles 0.40 × 0.25 on the skirt at x 2.2, φ 15° steps (local X axial, Y out, Z tangential) */
const vents = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const s = c.hull.sOf(2.2)
  for (let k = 0; k < 24; k++) {
    const F = c.hull.frame(s, deg(15 * k), 0)
    const at = (x: number, y: number, z: number, rx = 0): THREE.Matrix4 => F.clone().multiply(new THREE.Matrix4().compose(v3(x, y, z), new THREE.Quaternion().setFromAxisAngle(v3(1, 0, 0), rx), v3(1, 1, 1)))
    kit.add(boxGeo(0.4, 0.006, 0.25), MAT.panel, at(0, 0.002, 0))
    for (const z of [-0.1375, 0.1375]) kit.add(boxGeo(0.45, 0.014, 0.025), MAT.steelDark, at(0, 0.006, z))
    for (const x of [-0.2125, 0.2125]) kit.add(boxGeo(0.025, 0.014, 0.25), MAT.steelDark, at(x, 0.006, 0))
    for (const z of [-0.09, -0.03, 0.03, 0.09]) kit.add(boxGeo(0.38, 0.004, 0.052), MAT.steel, at(0, 0.012, z, deg(35)))
  }
  return kit.build("vents")
}

/** 13 thrust puck (forging Ø 3.6 × 0.6, x 3.6–4.2) + thrust cone + SL mounting bosses + RVac thrust pads + TVC/strut posts */
const thrustStructure = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const { x0, x1, r } = LAYOUT.puck
  kit.add(latheGeo(loopCCW([[0.42, x0], [r - 0.15, x0], [r, x0 + 0.08], [r, x1 - 0.08], [r - 0.15, x1], [0.42, x1]]), 64), MAT.forging)
  kit.add(latheGeo([[r - 0.06, x1], [0.55, 5.7]], 48), MAT.steelShell)
  kit.add(torusX(0.55, 0.05, 5.7, 8, 32), MAT.steelDark)
  for (let k = 0; k < 12; k++) { const p = deg(15 + 30 * k); kit.add(cylBetween(polar(x1, r - 0.05, p), polar(5.7, 0.58, p), 0.035, 0.035, 5), MAT.steelDark) }
  const bolt = cylX(-0.015, 0, 0.022, 0.022, 6)
  for (const e of c.v.engines) {
    const g = e.gimbal
    if (e.kind === "SL") {
      kit.add(cylX(x0 - 0.1, x0, 0.26, 0.26, 28).translate(0, g.y, g.z), MAT.forging)
      for (let k = 0; k < 12; k++) { const a = (k / 12) * Math.PI * 2; kit.add(bolt, MAT.steelDark, new THREE.Matrix4().makeTranslation(x0 - 0.1, g.y + 0.215 * Math.cos(a), g.z + 0.215 * Math.sin(a))) }
    } else {
      // RVac thrust pad from the mount flange up into the dome, tied to the skirt wall by two struts
      kit.add(canGeo(g.y, g.z, 0.32, g.x + 0.11, 24, g.x + 0.2), MAT.steelDarkShell)
      kit.add(cylX(g.x + 0.11, g.x + 0.14, 0.33, 0.33, 24).translate(0, g.y, g.z), MAT.steelDark)
      const phi = Math.atan2(g.z, g.y)
      for (const dp of [deg(-12), deg(12)]) kit.add(cylBetween(polar(g.x + 0.15, 3.5, phi + dp * 0.3), polar(5.9, 4.42, phi + dp), 0.06, 0.06, 8), MAT.steelDark)
    }
    // TVC actuator (SL) / strut (RVac) reaction posts: from the anchor up to the bay ceiling
    for (let k = 0; k < 2; k++) {
      const a = tvcAnchor(e.kind, k).add(v3(g.x, g.y, g.z))
      const top = bayCeiling(Math.hypot(a.y, a.z)) + 0.02
      kit.add(boxGeo(0.16, 0.07, 0.16, new THREE.Matrix4().makeTranslation(a.x - 0.01, a.y, a.z)), MAT.steelDark)
      if (top > a.x + 0.05) kit.add(cylBetween(v3(a.x, a.y, a.z), v3(top, a.y, a.z), 0.055, 0.055, 8), MAT.steelDark)
    }
  }
  return kit.build("thrustStructure")
}

/** ellipsoidal head with 12 gore welds, polar-cap ring and rim joint ring */
const domeShell = (apex: number, depth: number, s: 1 | -1, name: string, extra?: (k: Kit) => void): THREE.Object3D => {
  const kit = new Kit()
  const Rd = LAYOUT.rIn
  kit.add(latheGeo(domeProfile(apex, Rd, depth, s, 18), 48), MAT.steelShell)
  // gore seams on the convex side (aft for s = +1 domes, forward for s = −1)
  const off = -s * 0.012
  const xs = (rho: number): number => apex + s * depth * (1 - Math.sqrt(Math.max(0, 1 - (rho / Rd) ** 2)))
  for (let k = 0; k < 12; k++) {
    const phi = (k / 12) * Math.PI * 2 + 0.13
    const pts: THREE.Vector3[] = []
    for (let i = 0; i <= 10; i++) { const rho = 1.0 + (Rd - 0.02 - 1.0) * (i / 10) ** 0.8; pts.push(polar(xs(rho) + off, rho, phi)) }
    kit.add(tubeGeo(pts, 0.011, 14, 3), MAT.weld)
  }
  kit.add(torusX(1.0, 0.014, xs(1.0) + off, 4, 48), MAT.weld)
  kit.add(torusX(Rd - 0.01, 0.03, apex + s * depth, 4, 80), MAT.steelDark)
  extra?.(kit)
  return kit.build(name)
}

/** 16 payload-bay deck (annular, 12 radial beams between the COPVs) + dispenser rails */
const payloadDeck = (): THREE.Object3D => {
  const kit = new Kit()
  const x = LAYOUT.payloadDeck
  kit.add(latheGeo(loopCCW([[1.0, x - 0.04], [4.42, x - 0.04], [4.42, x + 0.04], [1.0, x + 0.04]]), 64), MAT.steelDarkShell)
  kit.add(torusX(1.0, 0.05, x, 6, 32), MAT.steelDark)
  for (let k = 0; k < 12; k++) kit.add(boxGeo(0.22, 3.4, 0.08), MAT.steelDark, axialFrame(x - 0.15, deg(15 + 30 * k), 2.7))
  // dispenser rails either side of the PEZ door opening (C-channels, x 34.0–38.3)
  for (const dp of [-13, 13]) {
    const F = axialFrame(36.15, LAYOUT.pez.phi + deg(dp), 4.3)
    kit.add(boxGeo(4.3, 0.12, 0.04), MAT.aluminium, F)
    for (const yy of [-0.05, 0.05]) kit.add(boxGeo(4.3, 0.035, 0.12), MAT.aluminium, F.clone().multiply(new THREE.Matrix4().makeTranslation(0, yy, 0)))
  }
  return kit.build("payloadDeck")
}

/** 16 PEZ door: skin panel 3.0 × 6.0 m with gap line, frame, 3 hinges at the forward edge, 4 latches aft */
const pezDoor = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const { x0, x1, phi, width } = LAYOUT.pez
  const h = c.hull
  const s0 = h.sOf((x0 + x1) / 2), len = x1 - x0
  kit.add(h.slab(s0, phi, len + 0.05, width + 0.05, -0.002, 0.006, 4, 12), MAT.panel) // gap line
  kit.add(h.slab(s0, phi, len - 0.02, width - 0.02, -0.002, 0.011, 4, 16, 0.02), MAT.door)
  const fw = 0.08, fh = 0.016
  kit.add(h.slab(h.sOf(x0 - fw / 2 - 0.025), phi, fw, width + 0.2, -0.002, fh, 1, 12), MAT.steelDark)
  kit.add(h.slab(h.sOf(x1 + fw / 2 + 0.025), phi, fw, width + 0.2, -0.002, fh, 1, 12), MAT.steelDark)
  for (const sd of [-1, 1]) kit.add(h.slab(s0, phi + (sd * (width / 2 + fw / 2 + 0.025)) / R, len + 0.05, fw, -0.002, fh, 6, 1), MAT.steelDark)
  // hinges (knuckle along the circumferential direction) and latches
  for (const dv of [-1.0, 0, 1.0]) {
    const F = h.frame(h.sOf(x1 + 0.02), phi + dv / R, 0.03)
    kit.add(cylBetween(v3(0, 0, -0.15), v3(0, 0, 0.15), 0.04, 0.04, 10), MAT.steelDark, F)
    kit.add(boxGeo(0.22, 0.012, 0.26), MAT.steelDark, F.clone().multiply(new THREE.Matrix4().makeTranslation(-0.1, -0.012, 0)))
  }
  for (const dv of [-1.1, -0.37, 0.37, 1.1]) kit.add(boxGeo(0.1, 0.03, 0.16), MAT.black, h.frame(h.sOf(x0 + 0.06), phi + dv / R, 0.02))
  return kit.build("pez")
}

/** 17 nosecone frames ×8 (I-section 0.17 deep following the ogive), segment joint ring, hat stringers ×24 */
const noseStructure = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const h = c.hull
  const ring = (x: number, depth: number, wid: number): void => {
    const m = h.at(h.sOf(x))
    const a0 = 0.045 // inner face of the steel skin under the PICA-X
    // T-section: outer flange on the skin, web, inner cap
    const sec: [number, number][] = [[a0, -wid], [a0, wid], [a0 + 0.02, wid], [a0 + 0.02, 0.008], [a0 + depth - 0.02, 0.008], [a0 + depth - 0.02, wid * 0.7], [a0 + depth, wid * 0.7], [a0 + depth, -wid * 0.7], [a0 + 0.02, -wid * 0.7 + 0.0], [a0 + 0.02, -wid]]
    const loop: RX[] = sec.map(([a, b]) => [m.r - a * m.nr + b * m.tr, m.x - a * m.nx + b * m.tx])
    kit.add(latheGeo(loopCCW(loop), 44), MAT.steelDark)
  }
  for (const x of LAYOUT.noseFrames) ring(x, 0.17, 0.05)
  ring(LAYOUT.noseJoint, 0.22, 0.09)
  const st: number[] = []
  const sA = h.sOf(38.5), sB = h.sOf(49.6)
  for (let i = 0; i <= 14; i++) st.push(sA + (sB - sA) * (i / 14))
  const hat: (readonly [number, number])[] = [[-0.045, -0.045], [-0.028, -0.105], [0.028, -0.105], [0.045, -0.045]]
  for (let k = 0; k < 24; k++) kit.add(h.sweep(st, deg(7.5 + 15 * k), hat), MAT.steelDarkShell)
  return kit.build("noseStructure")
}

/** frustum box: root (x0 × z0) at y0 → tip (x1 × z1) at y1 */
const frustumBox = (x0: number, z0: number, x1: number, z1: number, y0: number, y1: number): THREE.BufferGeometry => {
  const g = new THREE.BoxGeometry(1, 1, 1)
  const p = g.attributes.position
  for (let i = 0; i < p.count; i++) {
    const t = p.getY(i) + 0.5
    p.setXYZ(i, p.getX(i) * (x0 + (x1 - x0) * t), y0 + (y1 - y0) * t, p.getZ(i) * (z0 + (z1 - z0) * t))
  }
  g.computeVertexNormals()
  return g
}

/** 18 catch pins ×2 (1.0 × 0.9 root, tip at r = pinOut) with doubler plates, bolts and aft wear pads; bumpers ×2 */
const catchPins = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const h = c.hull
  const sp = h.sOf(c.v.pinX)
  const yTip = c.v.pinOut - h.r(c.v.pinX)
  const bolt = new THREE.CylinderGeometry(0.024, 0.024, 0.022, 6).translate(0, 0.011, 0)
  for (const phi of [0, Math.PI]) {
    const F = h.frame(sp, phi, 0)
    const loc = (x: number, y: number, z: number): THREE.Matrix4 => F.clone().multiply(new THREE.Matrix4().makeTranslation(x, y, z))
    kit.add(frustumBox(1.0, 0.9, 0.86, 0.76, -0.25, yTip - 0.05), MAT.steelDark, F)
    kit.add(frustumBox(0.86, 0.76, 0.74, 0.64, yTip - 0.05, yTip), MAT.steelDark, F)
    kit.add(boxGeo(0.05, 0.42, 0.6), MAT.black, loc(-0.47, yTip - 0.3, 0)) // wear pad on the aft (bearing) face
    kit.add(h.slab(sp, phi, 1.7, 1.5, -0.004, 0.035, 6, 6, 0.06), MAT.steelDark) // doubler
    for (let i = 0; i < 20; i++) {
      const per = (i / 20) * 4, side = Math.floor(per), f = per - side
      const du = side === 0 ? -0.77 + 1.54 * f : side === 1 ? 0.77 : side === 2 ? 0.77 - 1.54 * f : -0.77
      const dv = side === 0 ? -0.67 : side === 1 ? -0.67 + 1.34 * f : side === 2 ? 0.67 : 0.67 - 1.34 * f
      kit.add(bolt, MAT.steel, h.frame(sp + du, phi + dv / h.r(c.v.pinX), 0.035))
    }
    kit.add(h.slab(h.sOf(c.v.pinX - 1.25), phi, 0.5, 0.4, -0.004, 0.11, 3, 3, 0.04), MAT.rubber) // bumper
  }
  return kit.build("catchPins")
}

/** 18 lift beam across the nosecone base between the pin roots (internal load path) */
const liftBeam = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const x = c.v.pinX, rw = c.hull.r(x) - 0.06
  kit.add(boxGeo(0.45, 2 * rw - 0.4, 0.3, new THREE.Matrix4().makeTranslation(x, 0, 0)), MAT.steelDark)
  for (const s of [1, -1]) {
    kit.add(boxGeo(1.0, 0.5, 0.9, new THREE.Matrix4().makeTranslation(x, s * (rw - 0.25), 0)), MAT.forging)
    kit.add(cylBetween(v3(x - 0.2, s * (rw - 0.5), 0.1), v3(x - 1.4, s * (rw - 0.05), 0.1), 0.05, 0.05, 8), MAT.steelDark)
    kit.add(cylBetween(v3(x + 0.2, s * (rw - 0.5), -0.1), v3(x + 1.2, s * (rw - 0.1), -0.1), 0.05, 0.05, 8), MAT.steelDark)
  }
  return kit.build("liftBeam")
}

/** 41 PICA-X aft closeout: RTV-sealed edge band of the nosecone TPS at x0 */
const picaCloseout = (c: Ctx): THREE.Object3D => {
  const x0 = c.v.pica.x0
  return new Kit().add(latheGeo([[R - 0.0015, x0 - 0.035], [R + 0.012, x0 - 0.012], [R + 0.012, x0 + 0.02], [R + 0.006, x0 + 0.04]], 128), MAT.pica).build("picaCloseout")
}

export const structureParts = (c: Ctx): PartSpec[] => [
  { id: "seams", label: "11 Weld seams W01–W20 + longitudinal ring seams ×21", asm: "structure", obj: seams(c), internal: false },
  { id: "shieldDeck", label: "12 Engine-bay shield deck (open nozzle holes)", asm: "structure", obj: shieldDeck(c), internal: false },
  { id: "vents", label: "12 Engine-bay purge vents ×24 (louvred, x 2.2)", asm: "structure", obj: vents(c), internal: false },
  { id: "puck", label: "13 Thrust puck Ø 3.6 + thrust cone + TVC posts + RVac pads", asm: "structure", obj: thrustStructure(c), internal: true, dist: 6, axial: -6 },
  { id: "aftDome", label: "13 Aft dome (LOX tank bottom, depth 3.8)", asm: "structure", obj: domeShell(LAYOUT.aftDome.apex, LAYOUT.aftDome.depth, 1, "aftDome"), internal: true, dist: 0, axial: -3 },
  {
    id: "commonDome", label: "14 Common dome (LOX | CH₄) + CH₄ sump Ø 1.4 → 0.8", asm: "structure", internal: true, dist: 0, axial: 0,
    obj: domeShell(LAYOUT.commonDome.apex, LAYOUT.commonDome.depth, 1, "commonDome", (k) => {
      const xs = LAYOUT.commonDome.apex
      k.add(latheGeo([[0.4, xs - 1.2], [0.48, xs - 0.95], [0.7, commonDomeX(0.7) + 0.01]], 32), MAT.steel)
      k.add(torusX(0.42, 0.03, xs - 1.2, 6, 24), MAT.steelDark)
      for (let i = 0; i < 4; i++) k.add(boxGeo(0.9, 0.02, 0.5), MAT.steelDark, axialFrame(xs - 0.6, deg(45 + 90 * i), 0.0)) // anti-vortex baffles
    }),
  },
  {
    id: "fwdDome", label: "15 Forward dome (top of the CH₄ tank) + manhole", asm: "structure", internal: true, dist: 0, axial: 3,
    obj: domeShell(LAYOUT.fwdDome.apex, LAYOUT.fwdDome.depth, -1, "fwdDome", (k) => {
      const xa = fwdDomeX(0)
      k.add(cylX(xa - 0.01, xa + 0.06, 0.45, 0.45, 32), MAT.steelDark)
      for (let i = 0; i < 16; i++) { const a = (i / 16) * Math.PI * 2; k.add(cylX(xa + 0.06, xa + 0.085, 0.025, 0.025, 6).translate(0, 0.39 * Math.cos(a), 0.39 * Math.sin(a)), MAT.steel) }
    }),
  },
  { id: "deck", label: "16 Payload-bay deck + dispenser rails", asm: "structure", obj: payloadDeck(), internal: true, dist: 6, axial: 4 },
  { id: "pez", label: "16 PEZ door 3.0 × 6.0 + frame, hinges, latches (φ 90°)", asm: "structure", obj: pezDoor(c), internal: false },
  { id: "frames", label: "17 Nosecone frames ×8, segment joint, stringers ×24", asm: "structure", obj: noseStructure(c), internal: true, dist: 0, axial: 6 },
  { id: "catchPins", label: "18 Catch pins ×2 (tip r 5.3) + doublers + bumpers ×2", asm: "structure", obj: catchPins(c), internal: false },
  { id: "liftBeam", label: "18 Lift beam + pin root fittings", asm: "structure", obj: liftBeam(c), internal: true, dist: 10, axial: 4 },
  { id: "picaEdge", label: "41 PICA-X aft closeout band (x 37.93)", asm: "tps", obj: picaCloseout(c), internal: false },
]
