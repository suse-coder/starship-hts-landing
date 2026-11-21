import * as THREE from "three"
import type { VehicleConfig } from "@ship/craft"
import { avionicsParts } from "./parts/avionics"
import { type Assembly, ctxOf, type PartSpec } from "./parts/layout"
import { mhdParts } from "./parts/mhd"
import { propulsionParts } from "./parts/propulsion"
import { rcsParts } from "./parts/rcs"
import { structureParts } from "./parts/structure"

/**
 * Detailed vehicle parts per docs/03-spec/07-cad-model.md (assembly tree 10–60), built from
 * components in `./parts/*` (kit: geometry kernel + PBR materials; raptor: Raptor 3 SL/Vac; structure,
 * propulsion, rcs, mhd, avionics; pica: PICA-X honeycomb maps). Body frame: x from the aft skirt plane
 * to the nose tip, φ from +y toward +z. External parts are always visible; internal parts appear when
 * the hull is hidden, transparent, or in the exploded view (offset by `away`, grouped by assembly).
 */
export type { Assembly } from "./parts/layout"
export { latheX, MAT } from "./parts/kit"
export { makeRaptor } from "./parts/raptor"
export { makePicaGeometry, makePicaMaps, PICA_TEX } from "./parts/pica"

export interface Part {
  readonly id: string
  readonly label: string
  readonly asm: Assembly
  readonly obj: THREE.Object3D
  readonly internal: boolean
  readonly away: THREE.Vector3 // exploded position offset
}

export const ASM_LABEL: Record<Assembly, string> = {
  structure: "10 Structure", propulsion: "20 Propulsion", rcs: "30 Reaction control", tps: "40 Thermal protection", mhd: "50 HTS-MHD system", avionics: "60 Avionics · power · comms",
}
/** exploded-view direction per assembly (azimuth in the body y–z plane) */
const ASM_ANGLE: Record<Assembly, number> = { structure: 0, propulsion: 1, rcs: 2, tps: 3, mhd: 4, avionics: 5 }

const toPart = (p: PartSpec): Part => {
  const ang = (ASM_ANGLE[p.asm] / 6) * Math.PI * 2 + 0.3
  const d = p.internal ? 9 + (p.dist ?? 0) : 0
  const away = new THREE.Vector3(p.internal ? p.axial ?? 0 : 0, d * Math.cos(ang), d * Math.sin(ang))
  p.obj.name = p.id
  return { id: p.id, label: p.label, asm: p.asm, obj: p.obj, internal: p.internal, away }
}

export const makeParts = (v: VehicleConfig): Part[] => {
  const c = ctxOf(v)
  return [...structureParts(c), ...propulsionParts(c), ...rcsParts(c), ...mhdParts(c), ...avionicsParts(c)].map(toPart)
}
