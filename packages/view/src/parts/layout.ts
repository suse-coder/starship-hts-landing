import type * as THREE from "three"
import type { VehicleConfig } from "@ship/craft"
import { deg, domeX, Hull } from "./kit"

/**
 * One consistent internal arrangement shared by all part builders (body x, metres). Values follow
 * docs/03-spec/07-cad-model.md; dome "x" values in spec/07 are apex stations (LOX tank 3.6–21.5 and
 * CH₄ tank 21.5–31.6 run apex to apex). Deviations, each forced by a clash, are listed in DEVIATIONS.
 */
export type Assembly = "structure" | "propulsion" | "rcs" | "tps" | "mhd" | "avionics"

export interface PartSpec {
  readonly id: string
  readonly label: string
  readonly asm: Assembly
  readonly obj: THREE.Object3D
  readonly internal: boolean
  /** exploded view: extra radial distance (m) beyond the 9 m base, and axial shift (m) */
  readonly dist?: number
  readonly axial?: number
}

export interface Ctx {
  readonly v: VehicleConfig
  readonly hull: Hull
}

export const LAYOUT = {
  /** inner wall radius for internal parts (hull 96-gon chord minimum is 4.4976 m) */
  rIn: 4.48,
  ringH: 1.83,
  rings: 21,
  /** aft dome: apex at the thrust-puck face (SL gimbal plane), depth 3.8 m so the RVac mounts (x 4.6, r 3.2) sit under it */
  aftDome: { apex: 3.6, depth: 3.8 },
  /** common dome bulges aft into the LOX tank (apex 21.5, rim 23.7) */
  commonDome: { apex: 21.5, depth: 2.2 },
  /** forward dome bulges forward (apex 31.6, rim 29.4) */
  fwdDome: { apex: 31.6, depth: 2.2 },
  puck: { x0: 3.6, x1: 4.2, r: 1.8 },
  /** engine-bay shield deck (bottom of the six shield compartments) */
  shieldDeck: 1.8,
  /** payload-bay deck above the COPV ring */
  payloadDeck: 33.9,
  /** external raceway azimuth: clears the PEZ door (φ 90° ± 19°) and the +z RCS pods */
  racewayPhi: deg(112.5),
  /** header feed lines (CH₄ line, LOX line) azimuths, inside the tanks at r 4.1 */
  lineCh4Phi: deg(45),
  lineLoxPhi: deg(135),
  lineR: 4.1,
  /** RCS N₂ trunk line azimuth */
  n2Phi: deg(345),
  pez: { x0: 32.0, x1: 38.0, phi: deg(90), width: 3.0 },
  /** header tank centres (config xLoxHeader / xCh4Header) */
  headers: { lox: 46.6, ch4: 43.6 },
  noseFrames: [39.2, 40.65, 42.1, 43.55, 45.0, 46.45, 47.9, 49.35],
  noseJoint: 45.3,
  starlinkX: 41.5,
  headerVentX: 47.5,
  headerVentPhi: [deg(60), deg(240)],
  cryocooler: { x: 48.9, r: 1.45 },
  converterX: 49.6,
  dumpResistor: { x: 49.0, r: 1.35, phi: deg(90) },
  batteryX: 48.4,
  csTankX: 50.2,
  injectorX: 51.85,
} as const

/** x of the aft dome surface at radius ρ */
export const aftDomeX = (rho: number): number => domeX(LAYOUT.aftDome.apex, LAYOUT.rIn, LAYOUT.aftDome.depth, 1, Math.min(rho, LAYOUT.rIn - 1e-3))
export const commonDomeX = (rho: number): number => domeX(LAYOUT.commonDome.apex, LAYOUT.rIn, LAYOUT.commonDome.depth, 1, Math.min(rho, LAYOUT.rIn - 1e-3))
export const fwdDomeX = (rho: number): number => domeX(LAYOUT.fwdDome.apex, LAYOUT.rIn, LAYOUT.fwdDome.depth, -1, Math.min(rho, LAYOUT.rIn - 1e-3))
/** ceiling of the engine bay at radius ρ: thrust-puck face inside its radius, else the aft dome */
export const bayCeiling = (rho: number): number => (rho < LAYOUT.puck.r ? LAYOUT.puck.x0 : aftDomeX(rho))

/** documented departures from the spec/07 table (reported to the orchestrator) */
export const DEVIATIONS: readonly string[] = [
  "13 aft dome depth 3.8 m (spec 2.2): the RVac mounts at x 4.6, r 3.2 (config) must sit under the dome; an ellipsoidal head of depth 2.2 reaches only x 4.25 there",
  "12 purge vents at x 2.2 sit just above the shield deck (x 1.8), which closes the six shield compartments at the bottom",
  "19 raceway at φ 112.5° (spec 90°): φ 90° runs through the PEZ door (φ 90° ± 19°) and the +z RCS pods at x 5.5 / 44.6",
  "53 cryostat tube Ø 0.64 m with its centre 4 cm inboard of the coil centre line (spec Ø 0.70): the R_n 2.5 m cap leaves 0.32 m from the coil centre line to the OML",
  "54 cryocoolers at r 1.45 m (spec 1.0): the coil battery (Ø 2.0 m, x 48.1–48.7) occupies r ≤ 1.0 at the same station",
  "18 pin tip at r = pinOut 5.3 m (spec); ship.ts draws its box tip at pinOut + 0.4 = 5.7 m",
]

export const ctxOf = (v: VehicleConfig): Ctx => ({ v, hull: new Hull(v.shape) })
