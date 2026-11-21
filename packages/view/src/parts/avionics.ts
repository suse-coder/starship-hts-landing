import * as THREE from "three"
import { axialFrame, boxGeo, cylBetween, deg, Kit, MAT, v3 } from "./kit"
import { type Ctx, LAYOUT, type PartSpec } from "./layout"

/** 60 AVIONICS · POWER · COMMS (spec/07 rows 19, 61–64) */

/** 61 flight computers ×3, IMU on isolators, GNSS receivers ×2 on the payload-bay deck */
const bay = (): THREE.Object3D => {
  const kit = new Kit()
  const x = LAYOUT.payloadDeck + 0.04
  const unit = (phi: number, sx: number, sy: number, sz: number, fins: boolean): void => {
    const F = axialFrame(x + sx / 2, phi, 2.6)
    const loc = (px: number, py: number, pz: number): THREE.Matrix4 => F.clone().multiply(new THREE.Matrix4().makeTranslation(px, py, pz))
    kit.add(boxGeo(sx, sy, sz), MAT.box, F)
    for (let i = 0; i < 4; i++) kit.add(cylBetween(v3(0, 0, 0), v3(0, -0.06, 0), 0.028, 0.028, 8), MAT.black, loc(-sx / 4 + (i % 2) * (sx / 2), -sy / 2, -sz / 4 + Math.floor(i / 2) * (sz / 2)))
    if (fins) for (let i = 0; i < 7; i++) kit.add(boxGeo(0.05, sy - 0.06, 0.012), MAT.aluminium, loc(sx / 2 + 0.025, 0, -sz / 2 + 0.08 + i * ((sz - 0.16) / 6)))
  }
  for (const p of [30, 150, 270]) unit(deg(p), 0.6, 0.7, 0.9, true)
  for (const p of [210, 330]) unit(deg(p), 0.3, 0.4, 0.5, false)
  // IMU on four elastomer isolators
  const F = axialFrame(x + 0.28, deg(90), 2.6)
  kit.add(boxGeo(0.4, 0.4, 0.4), MAT.black, F)
  for (const [a, b] of [[-0.15, -0.15], [-0.15, 0.15], [0.15, -0.15], [0.15, 0.15]]) kit.add(cylBetween(v3(-0.26, a, b), v3(-0.2, a, b), 0.04, 0.04, 8), MAT.rubber, F)
  return kit.build("avionics")
}

/** 62 vehicle batteries ×4: two in the engine bay (on the shield deck), two on the payload-bay deck */
const batteries = (): THREE.Object3D => {
  const kit = new Kit()
  const one = (x: number, phi: number, rho: number): void => {
    const F = axialFrame(x + 0.4, phi, rho)
    kit.add(boxGeo(0.8, 0.45, 0.7), MAT.battery, F)
    kit.add(boxGeo(0.06, 0.47, 0.72), MAT.box, F.clone().multiply(new THREE.Matrix4().makeTranslation(0.4, 0, 0)))
    for (const z of [-0.2, 0.2]) kit.add(cylBetween(v3(0.43, 0, z), v3(0.5, 0, z), 0.03, 0.03, 8), MAT.copper, F)
  }
  for (const p of [90, 210]) one(LAYOUT.shieldDeck + 0.04, deg(p), 3.85)
  for (const p of [0, 180]) one(LAYOUT.payloadDeck + 0.04, deg(p), 3.7)
  return kit.build("batteries")
}

/** 19 external cable raceway 0.30 × 0.20 (x 2 → 41) with clamp brackets every 1.0 m; carries the 63 harness trunks */
const raceway = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const h = c.hull, phi = LAYOUT.racewayPhi
  const sA = h.sOf(2.0), sB = h.sOf(41.0)
  const st = [sA, sA + 0.08, sA + 0.18, sA + 0.32, h.sOf(37.9), h.sOf(38.43)]
  for (let s = h.sOf(38.43) + 0.3; s < sB - 0.32; s += 0.3) st.push(s)
  st.push(sB - 0.32, sB - 0.18, sB - 0.08, sB)
  const ramp = (s: number): number => Math.min(1, 0.15 + 0.85 * Math.min(s - sA, sB - s) / 0.32)
  const prof: (readonly [number, number])[] = [[-0.15, 0], [-0.135, 0.11], [-0.1, 0.18], [-0.05, 0.2], [0, 0.205], [0.05, 0.2], [0.1, 0.18], [0.135, 0.11], [0.15, 0]]
  kit.add(h.sweep(st, phi, prof, ramp), MAT.steel)
  const band = prof.map(([b, n]) => [b * 1.08, n + 0.008] as const)
  for (let s = sA + 0.6; s < sB - 0.5; s += 1.0) {
    kit.add(h.sweep([s, s + 0.06], phi, band), MAT.steelDark)
    for (const sd of [-1, 1]) kit.add(h.slab(s + 0.03, phi + (sd * 0.2) / h.at(s).r, 0.06, 0.08, -0.002, 0.012, 1, 1), MAT.steelDark)
  }
  return kit.build("raceway")
}

/** 64 Starlink phased arrays ×4: flush radome panels 1.2 × 0.8 on the nosecone, x 41.5, φ 45° + 90° k */
const starlink = (c: Ctx): THREE.Object3D => {
  const kit = new Kit()
  const s = c.hull.sOf(LAYOUT.starlinkX)
  for (let k = 0; k < 4; k++) {
    const phi = deg(45 + 90 * k)
    kit.add(c.hull.slab(s, phi, 1.28, 0.88, -0.002, 0.012, 4, 4), MAT.panel)
    kit.add(c.hull.slab(s, phi, 1.2, 0.8, -0.002, 0.02, 8, 6, 0.03), MAT.radome)
  }
  return kit.build("starlink")
}

export const avionicsParts = (c: Ctx): PartSpec[] => [
  { id: "avionics", label: "61 Flight computers ×3, IMU, GNSS ×2 (payload deck)", asm: "avionics", obj: bay(), internal: true, dist: 6, axial: 4 },
  { id: "batteries", label: "62 Vehicle batteries ×4 (engine bay, payload deck)", asm: "avionics", obj: batteries(), internal: true, dist: 10, axial: 0 },
  { id: "raceway", label: "19 Cable raceway 0.30 × 0.20 (x 2–41) + brackets; 63 harness trunks", asm: "avionics", obj: raceway(c), internal: false },
  { id: "starlink", label: "64 Starlink antennas ×4 (flush, nosecone x 41.5)", asm: "avionics", obj: starlink(c), internal: false },
]
