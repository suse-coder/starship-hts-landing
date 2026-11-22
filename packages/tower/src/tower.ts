import { clamp } from "@ship/math"
import { type V3, v3, ZERO, add, cross, dot, norm, sub, scale } from "@ship/math"

/**
 * Mechazilla (Pad 2 integration tower) in the pad ENU frame (origin: launch-mount centre,
 * x East, y North, z Up). Two "chopstick" arms extend south from the tower carriage; their top
 * rails carry the ship's two catch pins. Arms close by sliding along their traverse beams toward each other; the inner faces stay parallel (contact model).
 * Contact: penalty springs with critically-damped shock absorbers, Coulomb friction.
 */
export const TOWER = {
  height: 146, // m
  pos: v3(0, 23, 0), // tower centre (north of the mount)
  armHeight: 112, // m, top of the arm rails
  armLength: 44, // m along −y from the carriage
  armWidth: 1.6, // m
  /** depth of the arm truss at the carriage (root) and at the tip: the real chopsticks are a wedge */
  armRootDepth: 4.6,
  armTipDepth: 1.0,
  armDepth: 2.8,
  closedGap: 9.2, // m between inner faces when closed (ship Ø 9 m)
  openGap: 26, // m
  closeTime: 3.0, // s
  mountHeight: 20, // m launch mount top
  /**
   * Lateral tracking of the arm pair (m, m/s). The real arms ride trolleys on the traverse beams and
   * are driven independently, so the closed pair can be centred on an incoming ship that is a few
   * metres off the tower centreline. Reach is limited by the beams (±16.6 m, closed pair half-width
   * 6.8 m incl. trolley) with margin; the rate is the closing actuator's (8.4 m in 3 s).
   */
  armReach: 7.5, // m, centre offset of the pair
  armTrackRate: 2.5, // m/s
  armStandoff: 0.04, // m, clearance the compliant closing drive keeps to the hull
  /**
   * Outward flare of each arm's inner face from the rail top down to the underside (m). The real
   * chopsticks are V trusses: the rails that carry the pins are the narrowest point and the structure
   * opens downward, so the ship's aft section passes between the arms. With vertical inner faces the
   * engine section had 0.1 m of clearance and struck the arm underside on a slightly tilted arrival.
   */
  /**
   * Outward lean of the inner face from the rail down to the bottom spine (m). The real truss is a
   * ▽: two top chords under the track and one bottom spine at mid-width, so the inner face leans out
   * by about half the arm width — not a wide funnel.
   */
  armFlare: 1.8,
  kPin: 2.4e6, // N/m per pin (shock absorber spring)
  cPin: 9.0e5, // N s/m per pin
  mu: 0.35,
} as const

/** N s/m, slope of the regularised friction law at zero slip */
const FRICTION_C = 4e5

export interface ArmState {
  readonly open: number // 1 open … 0 closed
  readonly cmdClose: boolean
  /** east offset of the arm pair's centre from the tower centreline (m), |cx| ≤ TOWER.armReach */
  readonly cx: number
}
export const armsInit: ArmState = { open: 1, cmdClose: false, cx: 0 }

/**
 * `cxCmd`: commanded centre of the pair (tower-side tracking); rate- and reach-limited.
 * `hull`: the hull between the arms (axis x and radius at arm height), when there is one. The
 * closing drive is compliant: the arms stop `armStandoff` short of the hull instead of driving into
 * it. A rigid 2.8 m/s actuator with 0.1 m of clearance shoved a hull that was a few decimetres off
 * the pair's centre sideways at 3–4 m/s and it rattled between the arms (measured); the real arms
 * close until they bear on the hull.
 */
export const armStep = (s: ArmState, cmdClose: boolean, dt: number, cxCmd = 0, hull?: { readonly x: number; readonly r: number }): ArmState => {
  const tgt = clamp(cxCmd, -TOWER.armReach, TOWER.armReach)
  const cx0 = s.cx ?? 0
  const cx = cx0 + clamp(tgt - cx0, -TOWER.armTrackRate * dt, TOWER.armTrackRate * dt)
  let open = clamp(s.open + (cmdClose ? -dt / TOWER.closeTime : dt / TOWER.closeTime), 0, 1)
  if (hull && cmdClose) {
    const gapMin = 2 * (hull.r + Math.abs(hull.x - cx) + TOWER.armStandoff)
    const openMin = clamp((gapMin - TOWER.closedGap) / (TOWER.openGap - TOWER.closedGap), 0, 1)
    // never closer than the hull allows; if the hull moved in, the arm yields at the closing rate
    open = Math.max(open, Math.min(openMin, s.open + dt / TOWER.closeTime))
  }
  return { cmdClose, open, cx }
}

/**
 * Tower-side tracking law: centre the arm pair on the ship's catch-pin axis while it descends into
 * the catch window (within 150 m above the rails, inside the arms' y-span, within twice the reach
 * across). A short velocity lead covers the actuator lag. Outside the window the pair returns to
 * the centreline. The ship's own guidance still aims at the centreline; the arms only absorb its
 * residual across-rail error — a ship farther off than the reach, or moving faster than the arms,
 * is still a miss.
 */
export const armTrackCommand = (pinAxis: V3, vel: V3): number => {
  const dz = pinAxis.z - TOWER.armHeight
  const yMin = TOWER.pos.y - 2 - TOWER.armLength, yMax = TOWER.pos.y - 2
  const inWindow = dz > -3 && dz < 150 && pinAxis.y > yMin - 10 && pinAxis.y < yMax && Math.abs(pinAxis.x) < 2 * TOWER.armReach
  return inWindow ? clamp(pinAxis.x + 0.4 * vel.x, -TOWER.armReach, TOWER.armReach) : 0
}

export const armGap = (s: ArmState): number => TOWER.closedGap + (TOWER.openGap - TOWER.closedGap) * s.open
/** inner-face x positions of the two arms (east/west) */
export const armInnerX = (s: ArmState): readonly [number, number] => [(s.cx ?? 0) - armGap(s) / 2, (s.cx ?? 0) + armGap(s) / 2]

/**
 * Inner-face x of the west (`side` −1) or east (+1) arm at `dz` metres below the rail top. The arms
 * flare outward with depth (`TOWER.armFlare` over `TOWER.armDepth`), so the opening is widest at the
 * arm underside and narrowest at the pin rails.
 */
export const armInnerAt = (s: ArmState, side: 1 | -1, dz: number): number => {
  const [xl, xr] = armInnerX(s)
  const k = clamp(dz / TOWER.armRootDepth, 0, 1) * TOWER.armFlare
  return side < 0 ? xl - k : xr + k
}

/** truss depth at station `y` (pad ENU): linear wedge from the carriage to the tip, as built */
export const armDepthAt = (y: number): number => {
  const yRoot = TOWER.pos.y - 2, yTip = yRoot - TOWER.armLength
  const f = clamp((y - yTip) / (yRoot - yTip), 0, 1)
  return TOWER.armTipDepth + (TOWER.armRootDepth - TOWER.armTipDepth) * f
}

export interface ContactOut {
  readonly F: V3 // ENU, on the ship
  readonly M: V3 // ENU, about the ship CG
  readonly pinLoad: number // N vertical on pins (sum over every pin)
  /** per-pin vertical load in input order, N; 0 for a pin that is not on a rail */
  readonly pinLoads: readonly number[]
  /** pins actually carrying load (over a rail, in the arm y-span, in the contact band) */
  readonly pinCount: number
  /** smallest per-pin load, N (0 when there is no pin) */
  readonly pinLoadMin: number
  readonly bodyContact: boolean
}

/** maximum penetration treated as a pin/rail contact, m */
const PIN_CONTACT_BAND = 1.5

/**
 * Both catch pins individually carry at least `minLoadEach` newtons. A catch must not be inferred
 * from the total pin load or from one pin grazing a rail: with the arms open the rails are ≈ ±13 m
 * and a single pin can register a large load while the ship is nowhere near the centreline.
 */
export const bothPinsLoaded = (pinCount: number, pinLoadMin: number, minLoadEach: number): boolean =>
  pinCount === 2 && pinLoadMin > 0 && pinLoadMin >= minLoadEach

/** axis-aligned box in pad ENU, labelled for diagnostics */
export interface TowerBox {
  readonly id: string
  readonly lo: V3
  readonly hi: V3
}

/** trolley / actuator envelope above the arm root (view/tower.ts ARM + ACT) */
const ARM_TROLLEY_DEPTH = 6.2
const ARM_TROLLEY_TOP = 3.6
/** ship quick-disconnect arm, stowed west along the south face at 95 m */
const SQD_Y0 = 14.0
const SQD_Y1 = 17.2
const SQD_Z0 = 93.2
const SQD_Z1 = 98.3
const SQD_X0 = -29.8
const SQD_X1 = -5.0

/**
 * Solid structure boxes the hull can hit (pad ENU). Every visible component of `packages/view/src/tower.ts`
 * that the ship can reach is represented by a conservative AABB here: the lattice column (corner
 * columns and X-bracing), crown/outriggers, carriage ring + sheaves + traverse beams, both V-arms
 * (top rail, inner bumper, underside, root haunch, tip, trolley/actuator), the stowed ship QD arm,
 * launch-mount table/legs/pedestals and trench walls, and the near-tower site buildings. The arm
 * inner faces stay at ±armGap/2 and the rail top stays at TOWER.armHeight (the V-arm visual contract).
 *
 * Sizes are conservative (they may include air inside an open truss) but they must contain the visible
 * steel: a hull that reaches one of them must be deflected, never pass through.
 */
export const towerBoxes = (arms: ArmState): readonly TowerBox[] => {
  const [xl, xr] = armInnerX(arms)
  const yTip = TOWER.pos.y - 2 - TOWER.armLength
  const yRoot = TOWER.pos.y - 2
  const y = TOWER.pos.y
  return [
    // lattice: corner columns at ±5.25 m (1.5 m boxes) and bracing inside a ±6 m envelope; mast to 146 m
    { id: "tower-lattice", lo: v3(-6.1, y - 6.1, 0), hi: v3(6.1, y + 6.1, TOWER.height) },
    // floodlight brackets/luminaires protruding south of the lattice at three service levels
    // (view/tower.ts: u = base + 2.5·HS, base + 5.5·HS and armHeight − 14 m)
    { id: "tower-floodlights-low", lo: v3(-4.5, y - 7.2, 37.9), hi: v3(4.5, y - 6.1, 39.2) },
    { id: "tower-floodlights-mid", lo: v3(-4.5, y - 7.2, 82.2), hi: v3(4.5, y - 6.1, 83.5) },
    { id: "tower-floodlights-high", lo: v3(-4.5, y - 7.2, 97.6), hi: v3(4.5, y - 6.1, 98.9) },
    // concrete pile caps under the four corner columns (4 × 4 m each, 1.2 m tall)
    { id: "tower-pile-caps", lo: v3(-7.3, y - 7.3, 0), hi: v3(7.3, y + 7.3, 1.2) },
    // crown: top deck, south/north outriggers and sheaves (z 134–138), weather station to 139.4 m
    { id: "tower-crown", lo: v3(-6.4, y - 9.3, 133.5), hi: v3(6.4, y + 7.7, 139.5) },
    // carriage: ring/girders ±8.15 m, deck handrails, hoist sheaves at the south face
    { id: "carriage", lo: v3(-8.2, y - 9.3, TOWER.armHeight - 9.1), hi: v3(8.2, y + 8.2, TOWER.armHeight + 9.6) },
    // traverse beams the arm trolleys ride on (x ±16.6 m, two levels around 107–114 m)
    { id: "traverse-beams", lo: v3(-16.6, 13.6, TOWER.armHeight - 5.8), hi: v3(16.6, 14.9, TOWER.armHeight + 1.9) },
    // the two arms: the real wedge — a horizontal track on top, a single lower chord, and the
    // underside rising from `armRootDepth` at the carriage to `armTipDepth` at the tip. Sliced along
    // the length (each slice's inner face is the narrowest x inside it, so the boxes contain the
    // steel); the inner face leans out by `armFlare` from the rail to the bottom spine.
    ...Array.from({ length: 10 }, (_, k) => {
      const y0 = yTip + ((yRoot - yTip) * k) / 10, y1 = yTip + ((yRoot - yTip) * (k + 1)) / 10
      const d = armDepthAt(y1)
      // the box's inner edge is the rail face (the narrowest point of the slice), so the box contains
      // the whole truss including its top; the V flare is the visible inner face, not extra envelope
      const w = armInnerAt(arms, -1, 0), e = armInnerAt(arms, 1, 0)
      const top = TOWER.armHeight
      return [
        { id: k === 0 ? "arm-west" : `arm-west-${k}`, lo: v3(w - TOWER.armWidth, y0, top - d), hi: v3(w, y1, top) },
        { id: k === 0 ? "arm-east" : `arm-east-${k}`, lo: v3(e, y0, top - d), hi: v3(e + TOWER.armWidth, y1, top) },
      ]
    }).flat(),
    // arm trolleys, shoes, clevis and actuator anchors at the carriage end
    { id: "arm-west-trolley", lo: v3(xl - 0.6, 12.25, TOWER.armHeight - ARM_TROLLEY_DEPTH), hi: v3(xl + TOWER.armWidth + 0.6, 14.9, TOWER.armHeight + ARM_TROLLEY_TOP) },
    { id: "arm-east-trolley", lo: v3(xr - TOWER.armWidth - 0.6, 12.25, TOWER.armHeight - ARM_TROLLEY_DEPTH), hi: v3(xr + 0.6, 14.9, TOWER.armHeight + ARM_TROLLEY_TOP) },
    // stowed ship quick-disconnect arm, west of the tower at the QD level (≈ 95 m)
    { id: "ship-qd-arm", lo: v3(SQD_X0, SQD_Y0, SQD_Z0), hi: v3(SQD_X1, SQD_Y1, SQD_Z1) },
    // orbital launch mount: table + top plate + legs + clamps (top at TOWER.mountHeight)
    { id: "launch-mount", lo: v3(-14.1, -13.1, 0), hi: v3(14.1, 13.1, TOWER.mountHeight + 1.4) },
    // booster QD housing/hood on the north side of the mount table
    { id: "bqd-hood", lo: v3(-2, 7.6, 21), hi: v3(2, 12, 25.4) },
    // stair tower on the west pedestal and its bridge to the mount table
    { id: "stair-tower", lo: v3(-22.5, -3, 0), hi: v3(-15.5, 3, TOWER.mountHeight + 1) },
    // concrete pedestal blocks either side of the flame trench (visible to ±23 m, 6.5 m tall)
    { id: "pedestal-west", lo: v3(-23, -16, 0), hi: v3(-8.5, 13, 6.5) },
    { id: "pedestal-east", lo: v3(8.5, -16, 0), hi: v3(23, 13, 6.5) },
    // open trench beyond the pedestal and its end wing walls
    { id: "trench-west", lo: v3(-9.4, -60, 0), hi: v3(-8.5, -16, 4.2) },
    { id: "trench-east", lo: v3(8.5, -60, 0), hi: v3(9.4, -16, 4.2) },
    { id: "trench-wing-west", lo: v3(-16.9, -70, 0), hi: v3(-8.5, -60, 2.8) },
    { id: "trench-wing-east", lo: v3(8.5, -70, 0), hi: v3(16.9, -60, 2.8) },
    // near-tower site structures within the ship's reach: drawworks house, HPU containers, electrical house
    { id: "drawworks", lo: v3(-7.3, 33, 0), hi: v3(7.3, 42.3, 7.8) },
    { id: "hpu-containers", lo: v3(10.9, 17.9, 0), hi: v3(19.5, 30.1, 3) },
    { id: "electrical-house", lo: v3(21.8, 35.8, 0), hi: v3(32.2, 42.2, 5.1) },
    // pipe rack west of the tower (two low tiers, 7.6 m tall) and the tank farm/deluge tanks
    { id: "pipe-rack", lo: v3(-118, 18.5, 0), hi: v3(-9, 25.5, 7.6) },
    { id: "tank-farm", lo: v3(-175, -40, 0), hi: v3(-100, 60, 29) },
    { id: "subcoolers", lo: v3(-193, 7, 0), hi: v3(-185, 38, 13) },
    { id: "flare-stack", lo: v3(-213, -41, 0), hi: v3(-211, -39, 33) },
    { id: "deluge-tanks", lo: v3(-52, -58, 0), hi: v3(-44, -36, 22) },
    // 30 m light masts around the pad
    { id: "light-mast-sw", lo: v3(-63.8, -99.8, 0), hi: v3(-60.2, -96.2, 31.8) },
    { id: "light-mast-se", lo: v3(60.2, -99.8, 0), hi: v3(63.8, -96.2, 31.8) },
    { id: "light-mast-wnw", lo: v3(-63.8, -21.8, 0), hi: v3(-60.2, -18.2, 31.8) },
    { id: "light-mast-ene", lo: v3(60.2, -21.8, 0), hi: v3(63.8, -18.2, 31.8) },
    { id: "light-mast-nw", lo: v3(-63.8, 60.2, 0), hi: v3(-60.2, 63.8, 31.8) },
    { id: "light-mast-ne", lo: v3(60.2, 60.2, 0), hi: v3(63.8, 63.8, 31.8) },
  ]
}

/** N/m and N s/m for the hull against a solid structure (softer than the rail bumpers) */
const K_BODY = 3.0e6
const C_BODY = 4.0e5

/**
 * Sphere (hull station, radius r) against an AABB: closest-point normal and penetration depth.
 * A centre inside the box is pushed out through the nearest face (the only ambiguous case);
 * otherwise the normal is the exact surface normal.
 */
const sphereBoxHit = (p: V3, r: number, b: TowerBox): { n: V3; pen: number } | null => {
  const qx = clamp(p.x, b.lo.x, b.hi.x), qy = clamp(p.y, b.lo.y, b.hi.y), qz = clamp(p.z, b.lo.z, b.hi.z)
  let nx = p.x - qx, ny = p.y - qy, nz = p.z - qz
  const d2 = nx * nx + ny * ny + nz * nz
  if (d2 > r * r) return null
  let pen: number
  if (d2 > 1e-12) {
    const d = Math.sqrt(d2)
    nx /= d; ny /= d; nz /= d
    pen = r - d
  } else {
    // centre inside: nearest face, push along its outward normal
    const faces = [
      { d: p.x - b.lo.x, nx: -1, ny: 0, nz: 0 },
      { d: b.hi.x - p.x, nx: 1, ny: 0, nz: 0 },
      { d: p.y - b.lo.y, nx: 0, ny: -1, nz: 0 },
      { d: b.hi.y - p.y, nx: 0, ny: 1, nz: 0 },
      { d: p.z - b.lo.z, nx: 0, ny: 0, nz: -1 },
      { d: b.hi.z - p.z, nx: 0, ny: 0, nz: 1 },
    ]
    const f = faces.reduce((a, c) => (c.d < a.d ? c : a))
    nx = f.nx; ny = f.ny; nz = f.nz
    pen = r + f.d
  }
  return { n: v3(nx, ny, nz), pen }
}

/**
 * Sphere (hull station, radius r) against an AABB: normal + a critically-damped penalty force
 * (no adhesion: the normal force is never negative).
 */
const sphereBox = (p: V3, v: V3, r: number, b: TowerBox): { F: V3; n: V3; pen: number } | null => {
  const hit = sphereBoxHit(p, r, b)
  if (!hit) return null
  const vn = dot(v, hit.n)
  const mag = Math.max(0, K_BODY * hit.pen - C_BODY * vn)
  if (mag <= 0) return null
  return { F: scale(hit.n, mag), n: hit.n, pen: hit.pen }
}

/** a swept sphere hit: outward normal, penetration at the end, and first-touch fraction t ∈ [0, 1] */
interface SweptHit {
  readonly n: V3
  readonly pen: number
  readonly t: number
}

/**
 * Continuous sphere-vs-AABB test over the segment p0 → p. The box is expanded by the sphere radius
 * (a conservative Minkowski sum; the rounded corners are over-approximated by the square ones) and
 * the segment is clipped against its slabs. This catches a fast hull station crossing a thin arm in
 * a single RK4 step; the normal is the face normal at first touch.
 */
const sweptSphereBox = (p0: V3, p: V3, r: number, b: TowerBox): SweptHit | null => {
  const start = sphereBoxHit(p0, r, b)
  const end = sphereBoxHit(p, r, b)
  const qx = clamp(p0.x, b.lo.x, b.hi.x), qy = clamp(p0.y, b.lo.y, b.hi.y), qz = clamp(p0.z, b.lo.z, b.hi.z)
  const dx0 = p0.x - qx, dy0 = p0.y - qy, dz0 = p0.z - qz
  const d0 = Math.hypot(dx0, dy0, dz0)
  if (d0 < r) {
    // already in contact at the start; use the static normal so the solver never waits for a face
    const n = d0 > 1e-9 ? v3(dx0 / d0, dy0 / d0, dz0 / d0) : (start ? start.n : v3(0, 1, 0))
    return { n, pen: end ? end.pen : r - d0, t: 0 }
  }
  const p0a = [p0.x, p0.y, p0.z]
  const pa = [p.x, p.y, p.z]
  const lo = [b.lo.x, b.lo.y, b.lo.z]
  const hi = [b.hi.x, b.hi.y, b.hi.z]
  let tEnter = 0
  let tExit = 1
  let nx = 0, ny = 0, nz = 0
  for (let a = 0; a < 3; a++) {
    const d = pa[a] - p0a[a]
    const l = lo[a] - r
    const h = hi[a] + r
    if (Math.abs(d) < 1e-12) {
      if (p0a[a] < l || p0a[a] > h) return null
      continue
    }
    let t1 = (l - p0a[a]) / d
    let t2 = (h - p0a[a]) / d
    let sign = -1 // entering through the low face: normal points along −axis
    if (t1 > t2) {
      const tmp = t1
      t1 = t2
      t2 = tmp
      sign = 1
    }
    if (t1 > tEnter) {
      tEnter = t1
      nx = 0; ny = 0; nz = 0
      if (a === 0) nx = sign
      else if (a === 1) ny = sign
      else nz = sign
    }
    if (t2 < tExit) tExit = t2
    if (tEnter > tExit) return null
  }
  if (tEnter < 0 || tEnter > 1 || tExit < 0) return null
  if (nx === 0 && ny === 0 && nz === 0) {
    if (end) return { n: end.n, pen: end.pen, t: 0 }
    // started inside the expanded box but outside the sphere overlap: nearest face from p0
    const qx0 = clamp(p0.x, b.lo.x, b.hi.x), qy0 = clamp(p0.y, b.lo.y, b.hi.y), qz0 = clamp(p0.z, b.lo.z, b.hi.z)
    const ddx = p0.x - qx0, ddy = p0.y - qy0, ddz = p0.z - qz0
    const dd = Math.hypot(ddx, ddy, ddz)
    return dd > 1e-9 ? { n: v3(ddx / dd, ddy / dd, ddz / dd), pen: 0, t: 0 } : null
  }
  return { n: v3(nx, ny, nz), pen: end ? end.pen : 0, t: tEnter }
}

/** one hull station's swept path (ENU pad frame) */
export interface HullSphere {
  readonly p: V3 // centre at the end of the step
  readonly p0: V3 // centre at the start of the step
  readonly r: number
}

/** the first structure contact along a swept hull station, ENU pad frame */
export interface HullImpact {
  readonly index: number // hull station index
  readonly box: string // component id from towerBoxes
  readonly n: V3 // outward normal, from the structure to the ship
  readonly pen: number // penetration at the end of the step, m
  readonly t: number // first-touch fraction along p0 → p
}

/** earliest swept hull/structure contact over all stations and boxes (null when the hull is clear) */
export const firstHullImpact = (arms: ArmState, hull: readonly HullSphere[]): HullImpact | null => {
  let best: HullImpact | null = null
  const boxes = towerBoxes(arms)
  for (let i = 0; i < hull.length; i++) {
    const h = hull[i]
    for (const b of boxes) {
      const hit = sweptSphereBox(h.p0, h.p, h.r, b)
      if (!hit) continue
      if (!best || hit.t < best.t || (hit.t === best.t && hit.pen > best.pen)) {
        best = { index: i, box: b.id, n: hit.n, pen: hit.pen, t: hit.t }
      }
    }
  }
  return best
}

/** restitution and minimum inward speed of a hull strike (a real bump, not adhesion) */
const HULL_RESTITUTION = 0.15
const HULL_MIN_IMPACT_SPEED = 0.25

export interface HullResponse {
  readonly impact: HullImpact
  /** ENU velocity change of the CG (normal restitution + Coulomb friction) */
  readonly dv: V3
  /** ENU position correction of the CG that puts the first-touched station back outside */
  readonly push: V3
}

/**
 * Resolve a swept hull strike as a rigid-body impulse at the CG: remove the inward normal velocity,
 * add a small restitution and a Coulomb friction impulse, then translate the ship back out to the
 * first-touch plane. Applied once per control step after RK4 (the continuous penalty still handles
 * sustained contact); this is what stops a 20–50 m/s hull crossing a thin member in one step.
 */
export const resolveHullImpact = (
  arms: ArmState,
  hull: readonly HullSphere[],
  mass: number,
  v: V3,
  restitution = HULL_RESTITUTION,
): HullResponse | null => {
  const impact = firstHullImpact(arms, hull)
  if (!impact) return null
  const vn = dot(v, impact.n)
  if (vn >= -HULL_MIN_IMPACT_SPEED) return null
  const station = hull[impact.index]
  const pEntry = add(station.p0, scale(sub(station.p, station.p0), impact.t))
  const overshoot = Math.max(0, -dot(sub(station.p, pEntry), impact.n))
  const push = scale(impact.n, overshoot + 0.01)
  const jn = mass * (1 + restitution) * -vn
  let dv = scale(impact.n, jn / mass)
  const vt = sub(v, scale(impact.n, vn))
  const vtMag = norm(vt)
  if (vtMag > 1e-9) {
    const jt = Math.min(TOWER.mu * jn, mass * vtMag)
    dv = sub(dv, scale(vt, jt / (mass * vtMag)))
  }
  return { impact, dv, push }
}

/**
 * Contact forces given pin positions/velocities and the ship axis line (all ENU, pad frame).
 * pins: tip positions of the two pins; body: points sampled along the hull axis with radius R.
 */
export const towerContact = (
  arms: ArmState,
  cg: V3,
  pins: readonly { p: V3; v: V3 }[],
  hull: readonly { p: V3; v: V3; r: number }[],
): ContactOut => {
  let F = ZERO
  let M = ZERO
  let load = 0
  let body = false
  const [xl, xr] = armInnerX(arms)
  const yMin = TOWER.pos.y - 2 - TOWER.armLength
  const yMax = TOWER.pos.y - 2
  const loads: number[] = []
  for (const { p, v } of pins) {
    // a pin counts only with its tip over a rail, inside the arm y-span and in the contact band
    const overArm = (p.x <= xl && p.x >= xl - TOWER.armWidth - 0.6) || (p.x >= xr && p.x <= xr + TOWER.armWidth + 0.6)
    const pen = TOWER.armHeight - p.z
    const onRail = overArm && p.y >= yMin && p.y <= yMax && pen > 0 && pen <= PIN_CONTACT_BAND
    const fz = onRail ? Math.max(0, TOWER.kPin * pen - TOWER.cPin * v.z) : 0
    loads.push(fz)
    if (fz <= 0) continue
    // Coulomb friction cone: F_t = −μF_n·tanh(c|v_t|/(μF_n))·v̂_t — antiparallel to the slip velocity,
    // |F_t| < μF_n for every direction, smooth through v_t = 0 (slope c = 4e5 N s/m: viscous
    // regularisation of stiction at small slip speeds)
    const vt = Math.hypot(v.x, v.y)
    const fMax = TOWER.mu * fz
    const ft = vt > 0 && fMax > 0 ? fMax * Math.tanh((FRICTION_C * vt) / fMax) : 0
    const f = v3(vt > 0 ? (-ft * v.x) / vt : 0, vt > 0 ? (-ft * v.y) / vt : 0, fz)
    F = add(F, f)
    M = add(M, cross(sub(p, cg), f))
    load += fz
  }
  // hull against the solid tower/arm structure (the arms' inner faces are the sides of the arm boxes,
  // so closing arms squeeze an off-centre hull; the top/tip/side faces are real too, and the tower
  // column and mount are no longer transparent)
  const boxes = towerBoxes(arms)
  for (const { p, v, r } of hull) {
    for (const b of boxes) {
      const c = sphereBox(p, v, r, b)
      if (!c) continue
      body = true
      // Coulomb friction on the hull/structure contact (antiparallel to the slip, ≤ μF_n)
      let f = c.F
      const vt = sub(v, scale(c.n, dot(v, c.n)))
      const vtMag = norm(vt)
      if (vtMag > 1e-9) {
        const fMax = TOWER.mu * norm(c.F)
        const ft = fMax * Math.tanh((FRICTION_C * vtMag) / Math.max(fMax, 1))
        f = sub(f, scale(vt, ft / vtMag))
      }
      F = add(F, f)
      M = add(M, cross(sub(p, cg), f))
    }
  }
  const pinCount = loads.reduce((n, fz) => n + (fz > 0 ? 1 : 0), 0)
  const pinLoadMin = loads.length === 0 ? 0 : Math.min(...loads)
  return { F, M, pinLoad: load, pinLoads: loads, pinCount, pinLoadMin, bodyContact: body }
}

export const scaleContact = (c: ContactOut, s: number): ContactOut => ({ ...c, F: scale(c.F, s), M: scale(c.M, s) })
