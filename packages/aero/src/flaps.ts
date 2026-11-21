import { type V3, v3, add, cross, dot, norm, scale, sign, ZERO } from "@ship/math"
import type { AftFlapSet } from "@ship/craft"

/**
 * Deployable aft flap / grid-fin loads.
 *
 * Model (deliberately simple, one strip per flap):
 *  - each flap is a flat plate of planform area A on a circumferential hinge at radius r_h;
 *  - deployed, its neutral normal is radial and the plate is broadside to a belly-first crossflow;
 *  - a deflection δ (±maxDeflection) rotates the normal about the hinge line toward the body axis,
 *    so an axial flow component produces a drag force ∝ sin²δ (Newtonian flat plate, C_N = 2 sin²δ);
 *  - the local dynamic pressure is q·wakeFactor. The single wake factor represents base-flow
 *    shadowing of an aft-mounted surface; 0.3 is a fully shadowed lee flap, 1.0 is no shadowing;
 *  - the exposed area scales with the deployment fraction, so a stowed flap produces exactly zero
 *    force and moment (not a small residual).
 *
 * The normal-force coefficient is C_N = 2 sin²α_attack, capped at the Newtonian flat-plate value
 * C_N,max = 2. Viscous/profile drag of the lattice is neglected; it is small compared with the
 * normal force at the deflections and broadside angles of interest.
 */
export interface AftFlapState {
  /** 0 = stowed flush, 1 = fully folded out */
  readonly deployment: number
  /** commanded actual deflection per flap, rad (positive tilts the normal toward the aft end) */
  readonly deflections: readonly number[]
}

/** Guidance/MPC interface: one deploy flag plus optional one deflection per flap. */
export interface AftFlapCommand {
  readonly deploy: boolean
  /** commanded deflection per flap, rad; omitted = zero; clamped to ±maxDeflection by the actuator */
  readonly deflections?: readonly number[]
}

export const stowedAftFlapState = (count: number): AftFlapState => ({
  deployment: 0,
  deflections: Array.from({ length: count }, () => 0),
})

export const stowedAftFlapCommand = (count: number): AftFlapCommand => ({
  deploy: false,
  deflections: Array.from({ length: count }, () => 0),
})

export interface AftFlapInput {
  readonly vAir: V3 // body-frame velocity relative to the air
  readonly w?: V3 // body rates, rad/s (default zero)
  readonly rho: number
  readonly xcg: number
}

export interface AftFlapOut {
  readonly F: V3
  readonly M: V3
  readonly forceMag: number
  readonly momentMag: number
  /** freestream dynamic pressure at the flap centroid, Pa (before the wake factor) */
  readonly q: number
}

const clamp = (x: number, lo: number, hi: number): number => (x < lo ? lo : x > hi ? hi : x)

/** Rate-limited deployment and deflection actuator. Pure; returns the next state. */
export const stepAftFlaps = (
  set: AftFlapSet | null,
  state: AftFlapState,
  cmd: AftFlapCommand,
  dt: number,
): AftFlapState => {
  if (!set) return state
  const target = cmd.deploy ? 1 : 0
  const cmdDeflections = cmd.deflections ?? []
  const dDeploy = clamp(target - state.deployment, -(dt / set.deployTime), dt / set.deployTime)
  const deflections = Array.from({ length: set.count }, (_, k) => {
    const prev = state.deflections[k] ?? 0
    const wanted = clamp(cmdDeflections[k] ?? 0, -set.maxDeflection, set.maxDeflection)
    return prev + clamp(wanted - prev, -set.deflectionRate * dt, set.deflectionRate * dt)
  })
  return { deployment: clamp(state.deployment + dDeploy, 0, 1), deflections }
}

/**
 * Force and moment (about the CG, body frame) of the deployed aft flaps. Folded flaps return the
 * zero vector exactly.
 */
export const aftFlapWrench = (set: AftFlapSet | null, state: AftFlapState, inp: AftFlapInput): AftFlapOut => {
  if (!set || state.deployment <= 0 || set.area <= 0) {
    return { F: v3(0, 0, 0), M: v3(0, 0, 0), forceMag: 0, momentMag: 0, q: 0 }
  }
  const deployment = clamp(state.deployment, 0, 1)
  const w = inp.w ?? ZERO
  const qRef = 0.5 * inp.rho * norm(inp.vAir) ** 2
  let fx = 0, fy = 0, fz = 0, mx = 0, my = 0, mz = 0
  for (let k = 0; k < set.count; k++) {
    const phi = set.azimuths[k]
    const c = Math.cos(phi)
    const s = Math.sin(phi)
    // radial neutral normal and circumferential hinge tangent
    const n0 = v3(0, c, s)
    const hinge = v3(0, -s, c)
    const delta = clamp(state.deflections[k] ?? 0, -set.maxDeflection, set.maxDeflection)
    // Rodrigues about the unit hinge: n = n0 cosδ + (hinge × n0) sinδ; hinge × n0 = -x̂
    const n = add(scale(n0, Math.cos(delta)), scale(cross(hinge, n0), Math.sin(delta)))
    const r = v3(set.xCentroid - inp.xcg, set.rCentroid * c, set.rCentroid * s)
    const vLoc = add(inp.vAir, cross(w, r))
    const speed = norm(vLoc)
    if (speed < 1e-6) continue
    const vn = dot(vLoc, n)
    const sinAttack = Math.abs(vn) / speed
    const cn = 2 * sinAttack * sinAttack // Newtonian flat plate, C_N,max = 2
    const qLocal = 0.5 * inp.rho * speed * speed * set.wakeFactor
    const force = scale(n, -sign(vn) * cn * qLocal * set.area * deployment)
    fx += force.x
    fy += force.y
    fz += force.z
    mx += r.y * force.z - r.z * force.y
    my += r.z * force.x - r.x * force.z
    mz += r.x * force.y - r.y * force.x
  }
  const F = v3(fx, fy, fz)
  const M = v3(mx, my, mz)
  return { F, M, forceMag: norm(F), momentMag: norm(M), q: qRef }
}

/**
 * Authority reference used by the report/tests: symmetric deflection of every flap at a given
 * flight condition, returning force, moment, axial drag and the 10 s Δv at the CG.
 */
export interface AftFlapAuthority {
  readonly q: number
  readonly force: V3
  readonly moment: V3
  readonly forceMag: number
  readonly momentMag: number
  /** axial force along +x (negative = drag) */
  readonly axialForce: number
  /** |force|·10 s / mass, m/s */
  readonly decel10s: number
}

export const aftFlapAuthority = (
  set: AftFlapSet,
  inp: AftFlapInput & { readonly V: number; readonly alpha: number; readonly mass: number; readonly deflection: number },
): AftFlapAuthority => {
  const vAir = v3(inp.V * Math.cos(inp.alpha), 0, -inp.V * Math.sin(inp.alpha))
  const state: AftFlapState = {
    deployment: 1,
    deflections: Array.from({ length: set.count }, () => inp.deflection),
  }
  const out = aftFlapWrench(set, state, { ...inp, vAir })
  return {
    q: out.q,
    force: out.F,
    moment: out.M,
    forceMag: out.forceMag,
    momentMag: out.momentMag,
    axialForce: out.F.x,
    decel10s: (out.forceMag * 10) / inp.mass,
  }
}
