import * as THREE from "three"
import { axialFrame, basis, boxGeo, cylBetween, cylX, deg, Kit, MAT, polar, torusX, tubeGeo, v3 } from "./kit"
import { type Ctx, LAYOUT, type PartSpec } from "./layout"

/**
 * 50 HTS-MHD SYSTEM ◆ (spec/07 rows 51–57). The REBCO coil torus itself and the electrode glow
 * overlays are drawn by ship.ts (coil tube radius max(wpRadial, 0.12) + 0.13 m); these parts surround
 * them. The R_n 2.5 m cap (sphere centre x 49.6) leaves 0.32 m from the coil centre line (x 50.9,
 * r 1.75) to the OML, which sizes the cryostat (tube Ø 0.64 m, centre 4 cm inboard).
 */

const coilGeom = (c: Ctx): { x: number; a: number; tube: number; cx: number; ca: number; ct: number } => {
  const k = c.v.coil
  const tube = Math.max(k.wpRadial, 0.12) + 0.13
  const xs = c.v.shape.p.length - c.v.shape.p.noseRadius // cap sphere centre
  const d = v3(xs - k.x, -k.a, 0).normalize()
  return { x: k.x, a: k.a, tube, cx: k.x + 0.04 * d.x, ca: k.a + 0.04 * d.y, ct: tube + 0.07 }
}

/** 52 CF hoop structure: 16 clamp bands around the winding + continuous outer over-wrap band */
const hoop = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const g = coilGeom(c)
  const band = new THREE.TorusGeometry(g.tube + 0.012, 0.016, 4, 18)
  band.scale(1, 1, 3)
  for (let k = 0; k < 16; k++) {
    const psi = ((k + 0.5) / 16) * Math.PI * 2
    kit.add(band, MAT.cfrp, basis(polar(g.x, g.a, psi), v3(1, 0, 0), v3(0, Math.cos(psi), Math.sin(psi)), v3(0, -Math.sin(psi), Math.cos(psi))))
  }
  kit.add(torusX(g.a + g.tube + 0.01, 0.012, 0, 6, 128).scale(6, 1, 1).translate(g.x, 0, 0), MAT.cfrp)
  return kit.build("hoop")
}

/** 53 cryostat (transparent Al vacuum vessel) + G10 support struts ×8 + warm support ring at x 50.2 */
const cryostat = (c: Ctx): THREE.Object3D => {
  const g = coilGeom(c)
  const grp = new THREE.Group()
  const shell = new THREE.Mesh(torusX(g.ca, g.ct, g.cx, 16, 80), MAT.cryo)
  shell.renderOrder = 3
  grp.add(shell)
  const kit = new Kit()
  const xr = LAYOUT.csTankX, rr = c.hull.r(xr) - 0.14
  kit.add(torusX(rr, 0.045, xr, 6, 96), MAT.steelDark)
  for (let k = 0; k < 8; k++) {
    const psi = deg(11.25 + 45 * k)
    kit.add(cylBetween(polar(g.cx - 0.2, g.ca + 0.12, psi), polar(xr + 0.02, rr - 0.03, psi + deg(6)), 0.025, 0.025, 6), MAT.g10)
  }
  kit.add(cylBetween(polar(g.cx - 0.25, g.ca - 0.1, deg(90)), polar(g.cx - 0.5, g.ca - 0.2, deg(90)), 0.05, 0.05, 10), MAT.aluminium) // pump-out port
  grp.add(kit.build("cryostatStruts"))
  grp.name = "cryostat"
  return grp
}

/** 54 cryocoolers ×2 (compressor 0.8 × 0.6 × 0.6 with fins, cold head, transfer line to the cryostat) */
const cryocoolers = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const g = coilGeom(c)
  const { x, r } = LAYOUT.cryocooler
  for (const phi of [0, Math.PI]) {
    const F = axialFrame(x, phi, r)
    const loc = (px: number, py: number, pz: number): THREE.Matrix4 => F.clone().multiply(new THREE.Matrix4().makeTranslation(px, py, pz))
    kit.add(boxGeo(0.8, 0.6, 0.6), MAT.box, F)
    for (let i = 0; i < 7; i++) kit.add(boxGeo(0.7, 0.1, 0.012), MAT.aluminium, loc(0, 0.35, -0.24 + i * 0.08))
    kit.add(boxGeo(0.06, 0.66, 0.66), MAT.steelDark, loc(-0.43, 0, 0))
    const p = (xx: number, rr: number): THREE.Vector3 => polar(xx, rr, phi)
    kit.add(cylX(x + 0.4, x + 0.9, 0.11, 0.11, 16).translate(0, p(0, r).y, p(0, r).z), MAT.aluminium)
    kit.add(cylX(x + 0.9, x + 1.15, 0.07, 0.07, 14).translate(0, p(0, r).y, p(0, r).z), MAT.copper)
    kit.add(torusX(0.12, 0.02, x + 0.42, 6, 18).translate(0, p(0, r).y, p(0, r).z), MAT.steelDark)
    kit.add(tubeGeo([p(x + 1.15, r), p(x + 1.4, r + 0.04), p(g.cx - g.ct + 0.02, g.ca - 0.1)], 0.035, 16, 6), MAT.steel)
    for (const dp of [-8, 8]) kit.add(cylBetween(polar(x + 0.3, r + 0.3, phi + deg(dp)), polar(49.35, c.hull.r(49.35) - 0.2, phi + deg(dp * 1.4)), 0.03, 0.03, 6), MAT.steelDark)
  }
  return kit.build("cryocoolers")
}

/** 55 power converter ring (8 modules at x 49.6), dump resistor, copper busbars ×8 to the electrodes */
const converter = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const x = LAYOUT.converterX
  kit.add(torusX(1.9, 0.04, x, 6, 96), MAT.steelDark)
  for (let k = 0; k < 8; k++) {
    const phi = deg(22.5 + 45 * k)
    const F = axialFrame(x, phi, 1.9)
    kit.add(boxGeo(0.35, 0.25, 0.55), MAT.box, F)
    kit.add(boxGeo(0.3, 0.02, 0.5), MAT.copper, F.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0.135, 0)))
    const sE = c.hull.sOf(51.2), e = c.hull.point(sE + 0.35, phi, -0.08)
    kit.add(tubeGeo([polar(x + 0.1, 1.77, phi), polar(50.0, 1.45, phi), polar(50.35, 1.3, phi), polar(51.2, 1.3, phi), e], 0.025, 20, 5), MAT.copper)
  }
  const d = LAYOUT.dumpResistor
  const F = axialFrame(d.x, d.phi, d.r)
  kit.add(boxGeo(0.5, 0.3, 0.4), MAT.steelDark, F)
  for (let i = 0; i < 6; i++) kit.add(boxGeo(0.46, 0.08, 0.012), MAT.alloyDark, F.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0.19, -0.16 + i * 0.064)))
  kit.add(tubeGeo([polar(d.x + 0.25, d.r, d.phi), polar(x - 0.1, d.r + 0.3, d.phi - deg(10)), polar(x, 1.78, deg(67.5))], 0.03, 16, 6), MAT.copper)
  return kit.build("converter")
}

/** 55 electrode sectors ×8 (W/Cu, 0.9 × 0.9, flush on the cap at x ≈ 51.2, φ 22.5° + 45° k) with ceramic insulator frames */
const electrodePads = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const s = c.hull.sOf(c.v.shape.p.length - 0.9)
  for (let k = 0; k < 8; k++) {
    const phi = deg(22.5 + 45 * k)
    kit.add(c.hull.slab(s, phi, 1.0, 1.0, -0.004, 0.009, 4, 4), MAT.radome)
    kit.add(c.hull.slab(s, phi, 0.9, 0.9, -0.004, 0.016, 6, 6, 0.03), MAT.electrode)
  }
  return kit.build("electrodePads")
}

/** 56 coil charge battery Ø 2.0 × 0.6 (12 sector modules, end plates, terminals, four struts) */
const battery = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const x = LAYOUT.batteryX
  for (let k = 0; k < 12; k++) {
    const g = new THREE.CylinderGeometry(0.97, 0.97, 0.5, 4, 1, false, deg(30 * k + 1), deg(28))
    g.rotateZ(-Math.PI / 2)
    g.translate(x, 0, 0)
    kit.add(g, MAT.battery)
  }
  kit.add(cylX(x - 0.3, x - 0.25, 1.0, 1.0, 48), MAT.aluminium)
  kit.add(cylX(x + 0.25, x + 0.3, 1.0, 1.0, 48), MAT.aluminium)
  kit.add(cylX(x - 0.25, x + 0.25, 0.16, 0.16, 16), MAT.aluminium)
  for (const p of [0.3, 0.3 + Math.PI]) kit.add(cylX(x + 0.3, x + 0.4, 0.05, 0.05, 10).translate(0, 0.5 * Math.cos(p), 0.5 * Math.sin(p)), MAT.copper)
  const xf = LAYOUT.noseFrames[6]
  for (let k = 0; k < 4; k++) { const p = deg(45 + 90 * k); kit.add(cylBetween(polar(x, 0.98, p), polar(xf + 0.03, c.hull.r(xf) - 0.22, p), 0.035, 0.035, 6), MAT.steelDark) }
  return kit.build("coilBattery")
}

/** 57 caesium seed tank Ø 0.80 (heated, Cs melts at 301.6 K) + feed line + injector ring with 16 stubs */
const csTank = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const x = LAYOUT.csTankX, xi = LAYOUT.injectorX, xr = xi - 0.15
  kit.add(new THREE.SphereGeometry(0.4, 28, 18).translate(x, 0, 0), MAT.steel)
  kit.add(torusX(0.402, 0.03, 0, 6, 40).scale(3, 1, 1).translate(x, 0, 0), MAT.copper)
  const rr = c.hull.r(x) - 0.14
  for (let k = 0; k < 4; k++) { const p = deg(45 + 90 * k); kit.add(cylBetween(polar(x, 0.38, p), polar(x, rr - 0.04, p), 0.03, 0.03, 6), MAT.steelDark) }
  kit.add(cylX(x + 0.38, xr - 0.15, 0.03, 0.03, 8), MAT.steel)
  kit.add(cylX(x + 0.5, x + 0.64, 0.07, 0.07, 12), MAT.black)
  for (let k = 0; k < 4; k++) kit.add(cylBetween(v3(xr - 0.15, 0, 0), polar(xr, 0.95, deg(90 * k)), 0.022, 0.022, 6), MAT.steel)
  kit.add(torusX(0.95, 0.035, xr, 6, 64), MAT.steel)
  const s = c.hull.sOf(xi)
  for (let j = 0; j < 16; j++) { const p = deg(11.25 + 22.5 * j); kit.add(cylBetween(polar(xr, 0.95, p), c.hull.point(s, p, -0.02), 0.014, 0.014, 5), MAT.steel) }
  return kit.build("csTank")
}

/** 57 Cs injector ports ×16 on the cap (x 51.85) */
const csPorts = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const s = c.hull.sOf(LAYOUT.injectorX)
  for (let j = 0; j < 16; j++) {
    const F = c.hull.frame(s, deg(11.25 + 22.5 * j), 0.004)
    kit.add(new THREE.CylinderGeometry(0.04, 0.045, 0.012, 12).translate(0, 0.006, 0), MAT.steelDark, F)
    kit.add(new THREE.CylinderGeometry(0.016, 0.016, 0.004, 8).translate(0, 0.013, 0), MAT.black, F)
  }
  return kit.build("csPorts")
}

export const mhdParts = (c: Ctx): PartSpec[] => [
  { id: "hoop", label: "52 CF hoop structure: clamp bands ×16 + over-wrap band", asm: "mhd", obj: hoop(c), internal: true, dist: 0, axial: 6 },
  { id: "cryostat", label: "53 Cryostat (tube Ø 0.64) + G10 struts ×8 + support ring", asm: "mhd", obj: cryostat(c), internal: true, dist: 3, axial: 8 },
  { id: "cryocoolers", label: "54 Cryocoolers ×2 (25 K, 180 W) + transfer lines", asm: "mhd", obj: cryocoolers(c), internal: true, dist: 7, axial: 4 },
  { id: "converter", label: "55 Power converter ring ×8 + dump resistor + busbars", asm: "mhd", obj: converter(c), internal: true, dist: 11, axial: 6 },
  { id: "electrodePads", label: "55 Electrode sectors ×8 (W/Cu 0.9 × 0.9) on the cap", asm: "mhd", obj: electrodePads(c), internal: false },
  { id: "coilBattery", label: "56 Coil charge battery Ø 2.0 × 0.6 (100 kWh)", asm: "mhd", obj: battery(c), internal: true, dist: 15, axial: 3 },
  { id: "csTank", label: "57 Caesium seed tank Ø 0.8 + injector ring", asm: "mhd", obj: csTank(c), internal: true, dist: 19, axial: 8 },
  { id: "csPorts", label: "57 Cs injector ports ×16 (cap, x 51.85)", asm: "mhd", obj: csPorts(c), internal: false },
]
