import { clamp } from "@ship/math"
import { type V3, v3, ZERO, add, cross, sub, scale } from "@ship/math"

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
  armLength: 38, // m along −y from the carriage
  armWidth: 1.6, // m
  armDepth: 2.8,
  closedGap: 9.2, // m between inner faces when closed (ship Ø 9 m)
  openGap: 26, // m
  closeTime: 3.0, // s
  mountHeight: 20, // m launch mount top
  kPin: 2.4e6, // N/m per pin (shock absorber spring)
  cPin: 9.0e5, // N s/m per pin
  mu: 0.35,
} as const

/** N s/m, slope of the regularised friction law at zero slip */
const FRICTION_C = 4e5

export interface ArmState {
  readonly open: number // 1 open … 0 closed
  readonly cmdClose: boolean
}
export const armsInit: ArmState = { open: 1, cmdClose: false }

export const armStep = (s: ArmState, cmdClose: boolean, dt: number): ArmState => ({
  cmdClose,
  open: clamp(s.open + (cmdClose ? -dt / TOWER.closeTime : dt / TOWER.closeTime), 0, 1),
})

export const armGap = (s: ArmState): number => TOWER.closedGap + (TOWER.openGap - TOWER.closedGap) * s.open
/** inner-face x positions of the two arms (east/west) */
export const armInnerX = (s: ArmState): readonly [number, number] => [-armGap(s) / 2, armGap(s) / 2]

export interface ContactOut {
  readonly F: V3 // ENU, on the ship
  readonly M: V3 // ENU, about the ship CG
  readonly pinLoad: number // N vertical on pins
  readonly bodyContact: boolean
}

/** axis-aligned box in pad ENU */
interface Box {
  readonly lo: V3
  readonly hi: V3
}

/**
 * Solid structure boxes the hull can hit (pad ENU). The contact model used to know only the pins and
 * the arms' inner faces, so a misaligned ship flew straight through the tower. Sizes are conservative
 * (the visual lattice is inside these boxes), and the arm boxes move with the arm gap.
 */
const towerBoxes = (arms: ArmState): readonly Box[] => {
  const [xl, xr] = armInnerX(arms)
  const yTip = TOWER.pos.y - 2 - TOWER.armLength
  const yRoot = TOWER.pos.y - 2
  const zArm = TOWER.armHeight - TOWER.armDepth
  return [
    // main tower column (north of the mount) and the carriage that carries the arms
    { lo: v3(-5, TOWER.pos.y - 5, 0), hi: v3(5, TOWER.pos.y + 5, TOWER.height) },
    { lo: v3(-7, TOWER.pos.y - 7, TOWER.armHeight - 20), hi: v3(7, TOWER.pos.y + 7, TOWER.armHeight + 8) },
    // the two arms: solid from the rail top down by armDepth, tip to root
    { lo: v3(xl - TOWER.armWidth, yTip, zArm), hi: v3(xl, yRoot, TOWER.armHeight) },
    { lo: v3(xr, yTip, zArm), hi: v3(xr + TOWER.armWidth, yRoot, TOWER.armHeight) },
    // launch-mount table (octagon approximated by its inscribed box)
    { lo: v3(-13.6, -13, 0), hi: v3(13.6, 13, TOWER.mountHeight) },
  ]
}

/** N/m and N s/m for the hull against a solid structure (softer than the rail bumpers) */
const K_BODY = 3.0e6
const C_BODY = 4.0e5

/**
 * Sphere (hull station, radius r) against an AABB: the closest-point normal, penetration depth and
 * a critically-damped penalty force. A centre inside the box is pushed out through the nearest face
 * (the only ambiguous case); otherwise the normal is the exact surface normal.
 */
const sphereBox = (p: V3, v: V3, r: number, b: Box): { F: V3; pen: number } | null => {
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
  const vn = v.x * nx + v.y * ny + v.z * nz
  const mag = Math.max(0, K_BODY * pen - C_BODY * vn)
  if (mag <= 0) return null
  return { F: v3(mag * nx, mag * ny, mag * nz), pen }
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
  for (const { p, v } of pins) {
    // pin rests on the arm top if it is laterally over an arm rail
    const overArm = (p.x <= xl && p.x >= xl - TOWER.armWidth - 0.6) || (p.x >= xr && p.x <= xr + TOWER.armWidth + 0.6)
    if (!overArm || p.y < yMin || p.y > yMax) continue
    const pen = TOWER.armHeight - p.z
    if (pen <= 0 || pen > 1.5) continue
    const fz = Math.max(0, TOWER.kPin * pen - TOWER.cPin * v.z)
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
      F = add(F, c.F)
      M = add(M, cross(sub(p, cg), c.F))
    }
  }
  return { F, M, pinLoad: load, bodyContact: body }
}

export const scaleContact = (c: ContactOut, s: number): ContactOut => ({ ...c, F: scale(c.F, s), M: scale(c.M, s) })
