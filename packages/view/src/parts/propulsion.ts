import * as THREE from "three"
import { cylBetween, cylX, deg, domeProfile, Kit, latheGeo, MAT, polar, type RX, torusX, tubeGeo, v3 } from "./kit"
import { aftDomeX, bayCeiling, commonDomeX, type Ctx, LAYOUT, type PartSpec } from "./layout"
import { INLETS } from "./raptor"

/**
 * 20 PROPULSION (spec/07 rows 23–26): propellant volumes, downcomers, engine feed ducts, header
 * transfer lines, header tanks with sumps and vents. Sizes from the consumable masses: LOX header
 * 15.65 t / 1141 kg m⁻³ = 13.7 m³ → Ø 2.97 m sphere; CH₄ header 4.35 t / 422 kg m⁻³ = 10.3 m³ → Ø 2.70 m.
 */

/** 23 propellant volumes (translucent), inset 3 cm from the steel so they never z-fight the domes */
const tanks = (): { lox: THREE.Object3D; ch4: THREE.Object3D } => {
  const ri = 4.42, a = LAYOUT.aftDome, cd = LAYOUT.commonDome, fd = LAYOUT.fwdDome
  const loxP: RX[] = [...domeProfile(a.apex + 0.03, ri, a.depth - 0.06, 1, 20), ...domeProfile(cd.apex - 0.03, ri, cd.depth, 1, 20).reverse()]
  const ch4P: RX[] = [...domeProfile(cd.apex + 0.03, ri, cd.depth, 1, 20), ...domeProfile(fd.apex - 0.03, ri, fd.depth - 0.06, -1, 20)]
  const lox = new THREE.Mesh(latheGeo(loxP, 64), MAT.lox)
  const ch4 = new THREE.Mesh(latheGeo(ch4P, 64), MAT.ch4)
  lox.renderOrder = ch4.renderOrder = 2
  return { lox, ch4 }
}

/** 24 downcomers: centre CH₄ Ø 0.80 (puck → sump), RVac CH₄ ×3 Ø 0.30, engine feed ducts with gimbal bellows */
const downcomers = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const xs = LAYOUT.commonDome.apex - 1.2
  kit.add(cylX(LAYOUT.puck.x1, xs, 0.4, 0.4, 20, true), MAT.steel)
  for (const xb of [8.0, 12.5, 17.0]) for (const d of [-0.06, 0, 0.06]) kit.add(torusX(0.41, 0.02, xb + d, 3, 24), MAT.steelDark) // expansion bellows
  for (const xb of [9.5, 15.5]) {
    kit.add(torusX(0.43, 0.04, xb, 4, 24), MAT.steelDark)
    for (const p of [90, 210, 330]) kit.add(cylBetween(polar(xb, 0.43, deg(p)), polar(xb + 0.4, 4.4, deg(p)), 0.035, 0.035, 6), MAT.steelDark) // slosh/support struts
  }
  for (const e of c.v.engines) {
    const g = v3(e.gimbal.x, e.gimbal.y, e.gimbal.z)
    for (const prop of ["lox", "ch4"] as const) {
      const f = g.clone().add(INLETS[prop])
      const rho = Math.hypot(f.y, f.z), top = bayCeiling(rho) + 0.03
      const r = prop === "lox" ? 0.11 : 0.095
      kit.add(cylX(f.x + 0.02, top, r, r, 12, true).translate(0, f.y, f.z), MAT.steel)
      kit.add(torusX(r + 0.02, 0.022, f.x + 0.03, 3, 12).translate(0, f.y, f.z), MAT.steelDark)
      kit.add(torusX(r + 0.03, 0.025, top - 0.03, 3, 12).translate(0, f.y, f.z), MAT.steelDark)
      for (let i = 0; i < 3; i++) kit.add(torusX(r + 0.012, 0.014, f.x + 0.1 + i * 0.05, 3, 12).translate(0, f.y, f.z), MAT.steelDark)
    }
    if (e.kind === "RVac") {
      // RVac CH₄ downcomer: common dome at r 2.4 → down the LOX tank → over the CH₄ inlet, through the aft dome
      const phi = Math.atan2(e.gimbal.z, e.gimbal.y)
      const f = g.clone().add(INLETS.ch4)
      const xi = aftDomeX(Math.hypot(f.y, f.z))
      const path = [polar(commonDomeX(2.4) + 0.05, 2.4, phi), polar(15, 2.4, phi), polar(8.5, 2.4, phi), v3(xi + 0.9, (f.y + 2.4 * Math.cos(phi)) / 2, (f.z + 2.4 * Math.sin(phi)) / 2), v3(xi + 0.25, f.y, f.z), v3(xi - 0.02, f.y, f.z)]
      kit.add(tubeGeo(path, 0.15, 40, 8), MAT.steel)
      kit.add(torusX(0.17, 0.025, commonDomeX(2.4) + 0.03, 6, 20).translate(0, 2.4 * Math.cos(phi), 2.4 * Math.sin(phi)), MAT.steelDark)
    }
  }
  return kit.build("downcomers")
}

/** LOX header sump outlet (polar 140° from +x on the sphere, toward the LOX line azimuth) */
const loxSump = (): { base: THREE.Vector3; out: THREE.Vector3; dir: THREE.Vector3 } => {
  const R = 1.485, th = deg(140), phi = LAYOUT.lineLoxPhi, x0 = LAYOUT.headers.lox
  const dir = new THREE.Vector3(Math.cos(th), Math.sin(th) * Math.cos(phi), Math.sin(th) * Math.sin(phi))
  return { base: v3(x0, 0, 0).addScaledVector(dir, R - 0.05), out: v3(x0, 0, 0).addScaledVector(dir, R + 0.32), dir }
}

/** 24 header transfer lines ×2 (Ø 0.20, r 4.1) from the engine bay to the header sumps */
const headerLines = (): THREE.Object3D => {
  const kit = new Kit()
  const r = LAYOUT.lineR, pc = LAYOUT.lineCh4Phi, pl = LAYOUT.lineLoxPhi
  const ch4 = [polar(4.6, r, pc), polar(20, r, pc), polar(38.4, r, pc), polar(39.6, 3.85, pc), polar(40.7, 2.9, pc), polar(41.25, 1.0, pc), polar(41.55, 0.12, pc), v3(41.75, 0, 0), v3(41.92, 0, 0)]
  kit.add(tubeGeo(ch4, 0.1, 90, 6), MAT.steel)
  const s = loxSump()
  const lox = [polar(4.6, r, pl), polar(20, r, pl), polar(38.4, r, pl), polar(40.5, 3.6, pl), polar(42.4, 2.5, pl), polar(44.2, 2.05, pl), s.out.clone().addScaledVector(s.dir, 0.35), s.out]
  kit.add(tubeGeo(lox, 0.1, 90, 6), MAT.steel)
  // line clamps to the tank wall every ~3.7 m
  for (let x = 6; x < 38; x += 3.66) for (const p of [pc, pl]) kit.add(cylBetween(polar(x, r + 0.1, p), polar(x, 4.44, p), 0.03, 0.03, 6), MAT.steelDark)
  return kit.build("headerLines")
}

/** 25 header tanks: steel spheres with equator weld, sump cone + outlet valve, six struts to the nose frames */
const headers = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const { lox: xl, ch4: xc } = LAYOUT.headers
  const frameAt = (x: number): number => LAYOUT.noseFrames.reduce((b, f) => (Math.abs(f - x) < Math.abs(b - x) ? f : b), LAYOUT.noseFrames[0])
  for (const [x, R] of [[xl, 1.485], [xc, 1.35]] as const) {
    kit.add(new THREE.SphereGeometry(R, 40, 22).translate(x, 0, 0), MAT.steel)
    kit.add(torusX(R + 0.004, 0.018, x, 4, 56), MAT.weld)
    const fx = frameAt(x), rf = c.hull.r(fx) - 0.2
    for (let k = 0; k < 6; k++) {
      const p = deg(30 + 60 * k)
      kit.add(cylBetween(polar(x, R - 0.02, p), polar(fx, rf, p + deg(4)), 0.04, 0.04, 6), MAT.steelDark)
      kit.add(cylX(x - 0.12, x + 0.12, 0.07, 0.07, 8).translate(0, (R + 0.02) * Math.cos(p), (R + 0.02) * Math.sin(p)), MAT.steelDark) // strut lug
    }
  }
  // CH₄ sump: cone on the aft pole, outlet valve
  kit.add(latheGeo([[0.13, xc - 1.35 - 0.38], [0.2, xc - 1.35 - 0.2], [0.5, xc - 1.25]], 24), MAT.steel)
  kit.add(cylX(xc - 1.35 - 0.48, xc - 1.35 - 0.36, 0.17, 0.17, 16), MAT.black)
  // LOX sump: cone off-axis toward the LOX line
  const s = loxSump()
  const g = new THREE.LatheGeometry([new THREE.Vector2(0.45, 0), new THREE.Vector2(0.22, 0.2), new THREE.Vector2(0.13, 0.37)], 24)
  g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(v3(0, 1, 0), s.dir))
  g.translate(s.base.x, s.base.y, s.base.z)
  kit.add(g, MAT.steel)
  kit.add(cylBetween(s.out.clone().addScaledVector(s.dir, -0.05), s.out.clone().addScaledVector(s.dir, 0.08), 0.17, 0.17, 16), MAT.black)
  return kit.build("headers")
}

/** 26 header vents ×2: internal lines from the header tops to flush vent ports on the nosecone */
const ventLines = (): THREE.Object3D => {
  const kit = new Kit()
  const { lox: xl, ch4: xc } = LAYOUT.headers
  const [pl, pc] = LAYOUT.headerVentPhi
  const xv = LAYOUT.headerVentX
  const sp = (x0: number, R: number, th: number, phi: number): THREE.Vector3 => polar(x0 + R * Math.cos(th), R * Math.sin(th), phi)
  kit.add(tubeGeo([sp(xl, 1.485, deg(30), pl), polar(47.98, 1.3, pl), polar(47.85, 2.4, pl), polar(xv, 3.18, pl)], 0.04, 32, 6), MAT.steel)
  kit.add(tubeGeo([sp(xc, 1.35, deg(40), pc), polar(44.95, 1.7, pc), polar(45.9, 2.7, pc), polar(47.2, 3.08, pc), polar(xv, 3.18, pc)], 0.04, 40, 6), MAT.steel)
  for (const p of [pl, pc]) kit.add(cylBetween(polar(xv, 3.12, p), polar(xv, 3.27, p), 0.075, 0.075, 12), MAT.black) // vent valve
  return kit.build("ventLines")
}

/** 26 vent ports ×2 on the nosecone (external, flush on the PICA-X) */
const ventPorts = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const s = c.hull.sOf(LAYOUT.headerVentX)
  for (const p of LAYOUT.headerVentPhi) {
    const F = c.hull.frame(s, p, 0.004)
    kit.add(new THREE.CylinderGeometry(0.11, 0.12, 0.016, 20).translate(0, 0.008, 0), MAT.steelDark, F)
    kit.add(new THREE.CylinderGeometry(0.065, 0.065, 0.006, 16).translate(0, 0.017, 0), MAT.black, F)
  }
  return kit.build("ventPorts")
}

export const propulsionParts = (c: Ctx): PartSpec[] => {
  const t = tanks()
  return [
    { id: "loxTank", label: "23 LOX main tank volume (x 3.6–21.5)", asm: "propulsion", obj: t.lox, internal: true, dist: 4, axial: 0 },
    { id: "ch4Tank", label: "23 CH₄ main tank volume (x 21.5–31.6)", asm: "propulsion", obj: t.ch4, internal: true, dist: 4, axial: 2 },
    { id: "downcomers", label: "24 Downcomers: centre Ø 0.8, RVac ×3 Ø 0.3, engine feed ducts ×12", asm: "propulsion", obj: downcomers(c), internal: true, dist: 14, axial: -2 },
    { id: "headerLines", label: "24 Header transfer lines ×2 (Ø 0.2, r 4.1)", asm: "propulsion", obj: headerLines(), internal: true, dist: 19, axial: 0 },
    { id: "headers", label: "25 Header tanks: LOX Ø 2.97 (15.65 t), CH₄ Ø 2.70 (4.35 t) + sumps", asm: "propulsion", obj: headers(c), internal: true, dist: 10, axial: 6 },
    { id: "ventLines", label: "26 Header vent lines ×2", asm: "propulsion", obj: ventLines(), internal: true, dist: 14, axial: 8 },
    { id: "ventPorts", label: "26 Header vent ports ×2 (nosecone)", asm: "propulsion", obj: ventPorts(c), internal: false },
  ]
}
