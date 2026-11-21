import * as THREE from "three"
import type { Thruster } from "@ship/craft"
import { axialFrame, cylBetween, cylX, deg, instanced, Kit, MAT, polar, torusX, tubeGeo, v3 } from "./kit"
import { type Ctx, LAYOUT, type PartSpec } from "./layout"

/**
 * 30 REACTION CONTROL ◆ (concept, 15× the real ship's ullage-gas RCS). COPVs ×12 Ø 0.90 × 2.40 m
 * (12 × 1.3 m³ = 15.6 m³ of N₂ at 300 bar ≈ 5 t), ring at x 32.5, r 3.4; 12 cold-gas thrusters of
 * 4 kN (I_sp 73 s) in 8 pods. Nozzle: throat Ø 33 mm (A* = F/(C_F p_c) = 4 kN/(1.6·3 MPa)), exit Ø 0.25 m,
 * 0.30 m long (spec/07 row 32/33).
 */

/** 31 COPVs ×12 (instanced: over-wrap, polar bosses, straps, wall brackets) + N₂ manifold + trunk lines */
const copvs = (c: Ctx): THREE.Object3D => {
  const g = new THREE.Group()
  const x0 = c.v.xN2, rr = 3.4
  const ms = Array.from({ length: 12 }, (_, k) => axialFrame(x0, deg(30 * k), rr))
  // local frame: X = body x (vessel axis), Y = radial outward, Z = tangential
  const body = new THREE.CapsuleGeometry(0.45, 1.5, 5, 16)
  body.rotateZ(-Math.PI / 2)
  g.add(instanced(body, MAT.copv, ms))
  const boss = new Kit()
  for (const s of [-1, 1]) boss.add(cylX(s * 1.18, s * 1.27, 0.075, 0.075, 10), MAT.aluminium)
  for (const xs of [-0.45, 0.45]) boss.add(torusX(0.457, 0.014, xs, 3, 24), MAT.steelDark)
  for (const xs of [-0.45, 0.45]) boss.add(new THREE.BoxGeometry(0.06, 0.6, 0.14).translate(xs, 0.72, 0), MAT.steelDark) // strap → wall bracket
  for (const m of boss.build("copvHardware").children as THREE.Mesh[]) g.add(instanced(m.geometry, m.material as THREE.Material, ms))
  const kit = new Kit()
  const xm = x0 - 1.38
  kit.add(torusX(rr, 0.035, xm, 4, 80), MAT.steel) // N₂ ring manifold
  for (let k = 0; k < 12; k++) kit.add(cylBetween(polar(xm, rr, deg(30 * k)), polar(x0 - 1.26, rr, deg(30 * k)), 0.022, 0.022, 6), MAT.steel)
  const p = LAYOUT.n2Phi
  // aft trunk along the tank wall to the aft pod ring (x 5.5), forward trunk to the nose pod ring (x 44.6)
  kit.add(tubeGeo([polar(xm, rr + 0.03, p), polar(xm - 0.15, 3.9, p), polar(xm - 0.6, 4.3, p), polar(18, 4.3, p), polar(6.2, 4.3, p), polar(5.55, 4.4, p)], 0.03, 60, 5), MAT.steel)
  kit.add(torusX(4.4, 0.025, 5.5, 4, 96), MAT.steel)
  const xn = 44.6, rn = c.hull.r(xn) - 0.25
  kit.add(tubeGeo([polar(xm, rr + 0.03, p), polar(xm + 0.2, 3.9, p), polar(xm + 0.8, 4.3, p), polar(38.4, 4.3, p), polar(40.0, 4.08, p), polar(42.0, 3.93, p), polar(xn - 0.2, rn, p), polar(xn, rn, p - deg(3))], 0.03, 60, 5), MAT.steel)
  kit.add(torusX(rn, 0.025, xn, 4, 96), MAT.steel)
  // valve stubs from the ring lines to each pod
  for (const t of c.v.thrusters) {
    const phi = Math.atan2(t.pos.z, t.pos.y)
    const ring = t.pos.x < 20 ? 4.4 : rn
    kit.add(cylBetween(polar(t.pos.x, ring, phi), polar(t.pos.x, c.hull.r(t.pos.x) - 0.03, phi), 0.03, 0.03, 6), MAT.steel)
  }
  g.add(kit.build("n2Lines"))
  g.name = "copv"
  return g
}

/** bell nozzle along +Y: throat at y 0, exit Ø 0.25 at y 0.30 */
const rcsNozzle = (): THREE.BufferGeometry => {
  const pts = [[0.03, 0], [0.034, 0.03], [0.052, 0.08], [0.08, 0.16], [0.106, 0.24], [0.125, 0.3]].map(([r, y]) => new THREE.Vector2(r, y))
  return new THREE.LatheGeometry(pts, 16)
}

/** 32/33 RCS pods: streamlined blister + doubler, one nozzle per thruster (radial or tangential roll) */
const pods = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const h = c.hull
  const groups = new Map<string, Thruster[]>()
  for (const t of c.v.thrusters) {
    const k = `${t.pos.x.toFixed(2)}:${t.pos.y.toFixed(2)}:${t.pos.z.toFixed(2)}`
    groups.set(k, [...(groups.get(k) ?? []), t])
  }
  const noz = rcsNozzle()
  const valve = new THREE.CylinderGeometry(0.06, 0.06, 0.1, 12).translate(0, -0.05, 0)
  for (const ts of groups.values()) {
    const p0 = ts[0].pos
    const phi = Math.atan2(p0.z, p0.y)
    const s = h.sOf(p0.x)
    const hOff = p0.x >= c.v.pica.x0 ? 0.006 : 0.0
    const F = h.frame(s, phi, hOff)
    const inv = new THREE.Matrix4().extractRotation(F).invert()
    const multi = ts.length > 1
    kit.add(h.slab(s, phi, multi ? 1.0 : 0.8, multi ? 0.75 : 0.6, hOff - 0.004, hOff + 0.012, 4, 4, 0.03), MAT.steelDark)
    const blister = new THREE.SphereGeometry(1, 16, 7, 0, Math.PI * 2, 0, Math.PI / 2)
    blister.scale(multi ? 0.42 : 0.34, 0.22, multi ? 0.3 : 0.25)
    kit.add(blister, MAT.steel, F)
    for (const t of ts) {
      const e = v3(-t.dir.x, -t.dir.y, -t.dir.z).normalize().applyMatrix4(inv)
      const radial = Math.abs(e.y) > 0.7
      const base = radial ? v3(multi ? -0.14 : 0, 0.0, 0) : v3(0.12, 0.1, 0)
      const L = F.clone().multiply(new THREE.Matrix4().compose(base, new THREE.Quaternion().setFromUnitVectors(v3(0, 1, 0), e), v3(1, 1, 1)))
      kit.add(noz, MAT.inconel, L)
      kit.add(valve, MAT.black, L)
    }
  }
  return kit.build("rcsPods")
}

export const rcsParts = (c: Ctx): PartSpec[] => [
  { id: "copv", label: "31 N₂ COPVs ×12 (Ø 0.9 × 2.4, 300 bar) + manifold, trunks, pod rings", asm: "rcs", obj: copvs(c), internal: true, dist: 8, axial: 4 },
  { id: "rcsPods", label: "32/33 Cold-gas thruster pods ×8 (12 nozzles, exit Ø 0.25)", asm: "rcs", obj: pods(c), internal: false },
]
