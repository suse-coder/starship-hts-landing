import { type V3, v3, normalize } from "@ship/math"
import { MU0 } from "@ship/earth"
import { type Shape, makeShape, wettedArea } from "./geometry"
import { TILE, tileCentroidX, tileLayerMass } from "./tiles"

/**
 * Design parameters a user may change in the UI ("Design" panel → re-simulate).
 * Defaults are the answers proposed in docs/02-plan/05-open-questions.md.
 */
export interface DesignParams {
  readonly bStag: number // T, coil field at the stagnation point (MEESST target 1–2 T; Q3)
  readonly hallClosure: number // κ: fraction of Hall-current closure in the shock layer (0 = ideal blocking; Q4)
  readonly seedFraction: number // Cs mole fraction in the seeded shock layer (Q5)
  readonly jeTape: number // A/mm², REBCO tape engineering current density at 25 K, ~10 T (Q3)
  readonly noseRadius: number // m (Q6)
  readonly picaCap: number // m, PICA-X thickness on the spherical cap
  readonly picaFlank: number // m, PICA-X thickness at the aft edge of the coverage
  readonly landingProp: number // kg in header tanks at entry
  readonly rcsThrust: number // N per cold-gas thruster
  readonly n2Mass: number // kg of nitrogen
  readonly windScale: number
  readonly turbulence: number
  readonly seed: number
}

export const DEFAULT_DESIGN: DesignParams = {
  bStag: 2.0,
  hallClosure: 0.03,
  seedFraction: 0.01,
  jeTape: 700,
  noseRadius: 2.5,
  picaCap: 0.065,
  picaFlank: 0.025,
  // Q24 (2026-11-21): 26 t = 1.3 × the 20 t the flip and landing burn used (sim: header exhausted at T+3086 s)
  landingProp: 26000,
  rcsThrust: 4000,
  n2Mass: 5000,
  windScale: 1,
  turbulence: 1,
  seed: 42,
}

/** Inertia primitive for the mass model. */
export type Body =
  | { readonly kind: "point"; readonly r: number }
  | { readonly kind: "shell"; readonly r: number; readonly l: number }
  | { readonly kind: "solid"; readonly r: number; readonly l: number }
  | { readonly kind: "ring"; readonly r: number }
  | { readonly kind: "disk"; readonly r: number }
  | { readonly kind: "sphere"; readonly r: number }

export interface Component {
  readonly id: string
  readonly label: string
  readonly mass: number // kg (dry or nominal)
  readonly x: number // m, centre along body axis
  readonly body: Body
  readonly group: "structure" | "propulsion" | "tps" | "mhd" | "rcs" | "avionics" | "propellant"
  readonly note?: string
}

export interface Engine {
  readonly id: string
  readonly kind: "SL" | "RVac"
  readonly gimbal: V3 // gimbal point, body frame
  readonly thrustSL: number // N at sea level, 100 %
  readonly isp0: number // s at sea level
  readonly exitArea: number // m²
  readonly minThrottle: number
  readonly gimbalMax: number // rad
  readonly gimbalRate: number // rad/s
  readonly spoolTime: number // s ignition → full thrust
}

export interface Thruster {
  readonly id: string
  readonly pos: V3
  readonly dir: V3 // force direction on the vehicle
  readonly thrust: number
}

/** two thrusters fired together as a pure couple */
export interface Couple {
  readonly a: number
  readonly b: number
}

export interface CoilDesign {
  readonly a: number // loop radius, m
  readonly x: number // body x of the loop plane
  readonly zStag: number // loop plane → stagnation point distance
  readonly NI: number // ampere-turns
  readonly Iop: number // A
  readonly N: number // turns
  readonly L: number // H
  readonly energy: number // J
  readonly bCenter: number // T
  readonly bPeak: number // T on the winding (estimate)
  readonly wpAxial: number // winding pack axial width, m
  readonly wpRadial: number // m
  readonly massTape: number
  readonly massStructure: number
  readonly massCryostat: number
  readonly massCryocooler: number
  readonly massBattery: number
  readonly massPower: number
  readonly massSeed: number
  readonly seedMass: number // kg of Cs carried
  readonly tOp: number // K
  readonly tCs: number // current-sharing temperature at Iop, K
  readonly heatCapacity: number // J/K of the cold mass incl. thermal buffer
  readonly cryoLift: number // W at T_op
  readonly rDump: number // ohm, quench-protection tap (τ = L/R_d ≈ 2 s)
  readonly rDischarge: number // ohm, planned-discharge tap: V = I_op·R ≤ insulation (τ = L/R ≈ 65 s)
  readonly dumpMass: number // kg of dump-resistor plates (AISI 304), included in massPower
  readonly dumpT0: number // K, dump-resistor temperature at the start of the mission
  readonly vInsulation: number // V, coil terminal/ground insulation limit the dump voltage I·R_d is judged against
  readonly batteryEnergy: number // J
}

export interface PicaDesign {
  readonly x0: number // aft edge of the coverage
  readonly capThickness: number
  readonly flankThickness: number
  readonly density: number
  readonly area: number
  readonly mass: number
}

export interface VehicleConfig {
  readonly design: DesignParams
  readonly shape: Shape
  readonly components: readonly Component[]
  readonly engines: readonly Engine[]
  readonly thrusters: readonly Thruster[]
  readonly couples: readonly Couple[]
  readonly rcsIsp: number
  readonly coil: CoilDesign
  readonly pica: PicaDesign
  readonly steelThickness: number
  readonly pinX: number // catch pin body x
  readonly pinOut: number // pin tip radius (from axis)
  readonly xLoxHeader: number
  readonly xCh4Header: number
  readonly xLoxMain: number
  readonly xCh4Main: number
  readonly xN2: number
  readonly mixtureRatio: number
}

const R_HULL = 4.5
const LENGTH = 52.1
/** 21 barrel rings × 1.83 m = 38.43 m of cylinder; the nosecone is the rest (research/05) */
const RING_H = 1.83
const N_RINGS = 21
const NOSE_LEN = LENGTH - N_RINGS * RING_H // 13.67 m
/** 304L density (Wikipedia Starship footnote: 7.93 g/cm³); the ring figure ≈ 1.6 t is that footnote's estimate */
const STEEL_RHO = 7930
/** Mass-decision 2026-11-21 (Q23): all real Starship mass except the flaps is in the model. */
const RING_MASS = Math.PI * 2 * R_HULL * RING_H * 0.00397 * STEEL_RHO // 1.63 t per ring (computed)
const COPV_START = 6 * 150 // kg, engine-start COPVs around the CH₄ header (assumed, optimistic; conservative 300 kg each)
/** vacuum-jacketed header lines ×2: Ø 0.20 m, 2 mm wall + 1 mm jacket of Ø 0.25 m, 39 m long (computed) */
const HEADER_LINES = 2 * (Math.PI * 0.2 * 0.002 + Math.PI * 0.25 * 0.001) * 39 * STEEL_RHO
const PLUMBING_EXTRA = 400 // kg, chill lines ×4, vent lines, bellows (assumed, optimistic; conservative 1 t)

/** Physics-based sizing of the HTS coil system (see docs/03-spec/01-vehicle.md §4). */
export const designCoil = (d: DesignParams, xTip: number): CoilDesign => {
  const a = 1.75
  const zStag = 1.2
  const NI = (2 * d.bStag * (a * a + zStag * zStag) ** 1.5) / (MU0 * a * a)
  const fill = 0.35 // tape fraction of the winding pack (rest: insulation, CF structure, cooling)
  const Jwp = fill * d.jeTape * 1e6
  const wpAxial = 0.5
  const wpRadial = NI / Jwp / wpAxial
  const gmd = 0.2235 * (wpAxial + wpRadial) // geometric mean distance of a rectangle
  const LperN2 = MU0 * a * (Math.log((8 * a) / gmd) - 2)
  const Iop = 3000
  const N = NI / Iop
  const L = LperN2 * N * N
  const energy = 0.5 * LperN2 * NI * NI
  const bCenter = (MU0 * NI) / (2 * a)
  const bPeak = bCenter + (MU0 * NI) / (2 * (wpAxial + wpRadial))
  const massTape = ((NI * 2 * Math.PI * a) / (d.jeTape * 1e6)) * 8900
  // virial theorem: M_struct ≥ (ρ/σ)·E, CF composite σ_allow 1.2 GPa, ρ 1600, safety 2
  const massStructure = 2 * (1600 / 1.2e9) * energy
  const cryoArea = 2 * Math.PI * a * 2 * (wpAxial + wpRadial + 0.3)
  const massCryostat = 15 * cryoArea
  const cryoLift = 180 // W at 25 K (two Stirling/GM-class cold heads)
  const massCryocooler = 1.4 * cryoLift
  const batteryEnergy = energy / 0.95 / 0.85
  const massBattery = batteryEnergy / 720e3 // 200 Wh/kg pack level
  // dump resistor: AISI 304 plates that absorb the FULL stored energy (controlled "dump") with a bulk rise
  // ΔT ≤ 400 K from 290 K; mean c_p(290…690 K) ≈ 530 J/kg K (NIST 304 fit + Incropera Table A.1).
  // 290.6 MJ → ≈ 1.37 t (the former 50 kg allowance would have reached > 3000 K, i.e. melted)
  const dumpMass = energy / (530 * 400)
  const massPower = 200 + 150 + dumpMass // converter, electrodes+busbars, dump resistor
  // R5/T08 P1-4: the 400 kg of caesium is a consumable, not dry mass. The dry item is the tank and
  // injectors only (90 kg); mass.ts adds `seedMass` once as the consumable.
  const seedMass = 400
  const massSeed = 90
  // REBCO: Ic(T) ∝ (1 - T/Tc*)^1.3 with Tc*(10 T) ≈ 70 K; operate at Iop/Ic(25 K) ≈ 0.55
  const tOp = 25
  const Tcs = 70 - (70 - tOp) * 0.55 ** (1 / 1.3)
  const heatCapacity = (massTape + massStructure) * 12 + 60 * 900 // cold mass + 60 kg solid-N2 buffer
  // dump resistor sized for an L/R ≈ 2 s energy extraction on quench
  return {
    a, x: xTip - zStag, zStag, NI, Iop, N, L, energy, bCenter, bPeak, wpAxial, wpRadial,
    massTape, massStructure, massCryostat, massCryocooler, massBattery, massPower, massSeed, seedMass,
    tOp, tCs: Tcs, heatCapacity, cryoLift, rDump: L / 2.0, batteryEnergy,
    // insulation: assumed 3 kV (fast-discharge voltages of large superconducting magnets are a few kV,
    // e.g. ITER TF ≈ 3.5 kV); the τ = 2 s dump gives I_op·R_d ≈ 97 kV — reported, not hidden (R8)
    dumpMass, dumpT0: 290, vInsulation: 3000,
    // planned discharge after the steering switch-off (Q31, 2026-11-21): the same resistor bank on a 1 Ω
    // tap, so the terminal voltage I_op·R stays at the 3 kV insulation limit; τ = L/R ≈ 65 s, < 1 % current
    // after ≈ 300 s. The battery ramp-down (450 kW) would need ≈ 650 s and left ≈ 2 kA at touchdown.
    rDischarge: 3000 / Iop,
  }
}

export const designPica = (d: DesignParams, s: Shape): PicaDesign => {
  const x0 = s.Lc - 0.5
  const density = 270 // kg/m³ (PICA, Tran et al. 1997; PICA-X proxy)
  let area = 0
  let vol = 0
  const n = 300
  for (let i = 0; i < n; i++) {
    const xa = x0 + ((s.p.length - x0) * i) / n
    const xb = x0 + ((s.p.length - x0) * (i + 1)) / n
    const dA = Math.PI * (s.r(xa) + s.r(xb)) * Math.hypot(xb - xa, s.r(xb) - s.r(xa))
    const t = picaThicknessAt(d, s, 0.5 * (xa + xb), x0)
    area += dA
    vol += dA * t
  }
  const carrier = 3.0 // kg/m² RTV bond + carrier/honeycomb
  return { x0, capThickness: d.picaCap, flankThickness: d.picaFlank, density, area, mass: vol * density + area * carrier }
}

export const picaThicknessAt = (d: DesignParams, s: Shape, x: number, x0: number): number => {
  const xc = s.xCapEnd - 1.0
  if (x >= xc) return d.picaCap
  if (x <= x0) return d.picaFlank
  const t = (x - x0) / (xc - x0)
  return d.picaFlank + (d.picaCap - d.picaFlank) * t * t
}

export const makeVehicle = (d: DesignParams = DEFAULT_DESIGN): VehicleConfig => {
  const shape = makeShape({ length: LENGTH, radius: R_HULL, noseLength: NOSE_LEN, noseRadius: d.noseRadius })
  const coil = designCoil(d, LENGTH)
  const pica = designPica(d, shape)
  const steelT = 0.00397
  const noseSkin = wettedArea(shape, shape.Lc, LENGTH) * steelT * 7900
  const xLoxHeader = 46.6
  const xCh4Header = 43.6
  const components: Component[] = [
    { id: "barrel", label: `Steel barrel (301/30X, 3.97 mm, ${N_RINGS} rings × ${RING_H} m, ${(RING_MASS / 1000).toFixed(2)} t each)`, mass: N_RINGS * RING_MASS, x: (N_RINGS * RING_H) / 2, body: { kind: "shell", r: R_HULL, l: N_RINGS * RING_H }, group: "structure" },
    { id: "noseShell", label: "Nosecone steel (2 segments) + frames, chopstick lift reinforcement (no flap hinges)", mass: noseSkin + 2000, x: LENGTH - NOSE_LEN / 2, body: { kind: "shell", r: 3.4, l: NOSE_LEN }, group: "structure" },
    { id: "aftDome", label: "Aft dome, thrust puck, thrust cone", mass: 3500, x: 4.0, body: { kind: "disk", r: R_HULL }, group: "structure" },
    { id: "commonDome", label: "Common dome", mass: 2000, x: 21.5, body: { kind: "disk", r: R_HULL }, group: "structure" },
    { id: "fwdDome", label: "Forward dome", mass: 1800, x: 31.6, body: { kind: "disk", r: R_HULL }, group: "structure" },
    { id: "stringers", label: "Tank stringers + internal structure", mass: 3500, x: 18, body: { kind: "shell", r: 4.4, l: 27 }, group: "structure" },
    { id: "skirt", label: "Aft skirt / engine bay", mass: 4500, x: 2.8, body: { kind: "shell", r: R_HULL, l: 5.5 }, group: "structure" },
    { id: "rsl", label: "3 × Raptor 3 (sea level), 1525 kg each", mass: 3 * 1525, x: 2.2, body: { kind: "ring", r: 1.3 }, group: "propulsion" },
    { id: "rvac", label: "3 × Raptor 3 Vacuum, ~2100 kg each", mass: 3 * 2100, x: 2.6, body: { kind: "ring", r: 3.2 }, group: "propulsion" },
    { id: "feed", label: "Downcomers, valves, electric TVC", mass: 3000, x: 10, body: { kind: "solid", r: 1.5, l: 12 }, group: "propulsion" },
    { id: "payloadBay", label: "Payload bay (PEZ door, dispenser, deck)", mass: 4000, x: 35.0, body: { kind: "shell", r: 4.3, l: 6.8 }, group: "structure" },
    { id: "avionics", label: "Avionics, batteries, harness, Starlink antennas", mass: 3200, x: 28, body: { kind: "shell", r: 4.2, l: 40 }, group: "avionics" },
    // Q24: 26 t of header propellant (31.2 m³ vs 24.0 m³ at 20 t) costs tank structure: mass ∝ area (V^2/3)
    // 1.60 t × (31.2/24.0)^(2/3) = 1.91 t (computed; conservative alternative 2.2 t with thicker walls + struts)
    { id: "headers", label: "Header tanks (LOX Ø 3.24, CH₄ Ø 2.94) + sumps", mass: 1905, x: 45.5, body: { kind: "solid", r: 1.4, l: 5 }, group: "propulsion" },
    { id: "copvStart", label: `Engine-start COPVs ×6 at the CH₄ header (assumed, 150 kg each)`, mass: COPV_START, x: 43.6, body: { kind: "ring", r: 1.5 }, group: "propulsion" },
    { id: "plumbing", label: "Vacuum-jacketed header lines ×2 (Ø 0.20 m, computed) + chill, vent lines, bellows (assumed)", mass: HEADER_LINES + PLUMBING_EXTRA, x: 24.1, body: { kind: "shell", r: 4.1, l: 39 }, group: "propulsion" },
    { id: "misc", label: "Purge / fire suppression / misc", mass: 800, x: 4.0, body: { kind: "shell", r: 4.0, l: 3 }, group: "structure" },
    { id: "tiles", label: `Silica tile heat shield (LI-900 class, ${(TILE.thickness * 100).toFixed(1)} cm, barrel; Q23)`, mass: tileLayerMass(shape, d), x: tileCentroidX(shape, d), body: { kind: "shell", r: R_HULL, l: 40 }, group: "tps" },
    { id: "coating", label: "High-emissivity coating on bare steel (ε 0.85)", mass: 250, x: (N_RINGS * RING_H) / 2, body: { kind: "shell", r: R_HULL, l: N_RINGS * RING_H }, group: "tps" },
    { id: "pica", label: `PICA-X nosecone TPS (${pica.area.toFixed(0)} m², ${(d.picaCap * 100).toFixed(1)}→${(d.picaFlank * 100).toFixed(1)} cm)`, mass: pica.mass, x: (pica.x0 + LENGTH) / 2, body: { kind: "shell", r: 3.0, l: LENGTH - pica.x0 }, group: "tps" },
    { id: "coilTape", label: `HTS coil: REBCO winding (NI ${(coil.NI / 1e6).toFixed(2)} MA·t)`, mass: coil.massTape, x: coil.x, body: { kind: "ring", r: coil.a }, group: "mhd" },
    { id: "coilStruct", label: "Coil CF hoop structure (virial-sized)", mass: coil.massStructure, x: coil.x, body: { kind: "ring", r: coil.a }, group: "mhd" },
    { id: "cryostat", label: "Cryostat, MLI, thermal shield, SN2 buffer", mass: coil.massCryostat + 60, x: coil.x, body: { kind: "ring", r: coil.a }, group: "mhd" },
    { id: "cryocooler", label: "Cryocoolers (2 cold heads, 25 K)", mass: coil.massCryocooler, x: 48.9, body: { kind: "solid", r: 0.6, l: 0.8 }, group: "mhd" },
    { id: "mhdPower", label: "Electrode segments, converter, dump resistor", mass: coil.massPower, x: 49.6, body: { kind: "ring", r: 1.9 }, group: "mhd" },
    { id: "battery", label: `Coil charge battery (${(coil.batteryEnergy / 3.6e6).toFixed(0)} kWh)`, mass: coil.massBattery, x: 48.4, body: { kind: "solid", r: 1.0, l: 0.6 }, group: "mhd" },
    { id: "seed", label: "Caesium seed tank + injectors (dry; 400 kg Cs in consumables)", mass: coil.massSeed, x: 50.2, body: { kind: "solid", r: 0.4, l: 0.5 }, group: "mhd" },
    { id: "rcsDry", label: `Cold-gas RCS: N2 COPVs (${(d.n2Mass / 1000).toFixed(1)} t gas at 300 bar), 12 thrusters`, mass: 400 + 0.6 * d.n2Mass, x: 32.5, body: { kind: "shell", r: 3.8, l: 2 }, group: "rcs" },
  ]
  const thrusters: Thruster[] = []
  const T = d.rcsThrust
  const xN = 44.6
  const rN = shape.r(xN)
  const xA = 5.5
  for (const [x, r, tag] of [[xN, rN, "nose"], [xA, R_HULL, "aft"]] as const) {
    thrusters.push({ id: `${tag}+y`, pos: v3(x, r, 0), dir: v3(0, -1, 0), thrust: T })
    thrusters.push({ id: `${tag}-y`, pos: v3(x, -r, 0), dir: v3(0, 1, 0), thrust: T })
    thrusters.push({ id: `${tag}+z`, pos: v3(x, 0, r), dir: v3(0, 0, -1), thrust: T })
    thrusters.push({ id: `${tag}-z`, pos: v3(x, 0, -r), dir: v3(0, 0, 1), thrust: T })
  }
  // roll couples at the aft skirt
  thrusters.push({ id: "roll+a", pos: v3(xA, 0, R_HULL), dir: v3(0, -1, 0), thrust: T })
  thrusters.push({ id: "roll+b", pos: v3(xA, 0, -R_HULL), dir: v3(0, 1, 0), thrust: T })
  thrusters.push({ id: "roll-a", pos: v3(xA, 0, R_HULL), dir: v3(0, 1, 0), thrust: T })
  thrusters.push({ id: "roll-b", pos: v3(xA, 0, -R_HULL), dir: v3(0, -1, 0), thrust: T })
  // pure-torque couples (thruster index pairs fired with equal duty): no net force, no cross-axis torque
  const ix = (id: string): number => thrusters.findIndex((t) => t.id === id)
  const couples: Couple[] = [
    ["nose+z", "aft-z"], ["nose-z", "aft+z"], ["nose+y", "aft-y"], ["nose-y", "aft+y"], ["roll+a", "roll+b"], ["roll-a", "roll-b"],
  ].map(([a, b]) => ({ a: ix(a), b: ix(b) }))
  const engines: Engine[] = []
  for (let k = 0; k < 3; k++) {
    const a = (k * 2 * Math.PI) / 3 + Math.PI / 2
    engines.push({
      id: `RSL${k + 1}`, kind: "SL", gimbal: v3(3.6, 1.3 * Math.cos(a), 1.3 * Math.sin(a)),
      thrustSL: 2.45e6, isp0: 350, exitArea: 1.33, minThrottle: 0.4, gimbalMax: (15 * Math.PI) / 180, gimbalRate: (20 * Math.PI) / 180, spoolTime: 2.5,
    })
  }
  for (let k = 0; k < 3; k++) {
    const a = (k * 2 * Math.PI) / 3 + Math.PI / 2 + Math.PI / 3
    engines.push({
      id: `RVAC${k + 1}`, kind: "RVac", gimbal: v3(4.6, 3.2 * Math.cos(a), 3.2 * Math.sin(a)),
      thrustSL: 0, isp0: 0, exitArea: 4.15, minThrottle: 0.6, gimbalMax: 0, gimbalRate: 0, spoolTime: 3,
    })
  }
  return {
    design: d, shape, components, engines, thrusters, couples, rcsIsp: 73, coil, pica, steelThickness: steelT,
    // pins at the nosecone base (chopstick lift points), research/05
    pinX: N_RINGS * RING_H + 1.2, pinOut: R_HULL + 0.8, xLoxHeader, xCh4Header, xLoxMain: 5.2, xCh4Main: 22.3, xN2: 32.5, mixtureRatio: 3.6,
  }
}

export const thrusterDir = (t: Thruster): V3 => normalize(t.dir)
