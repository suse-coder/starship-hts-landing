import { type V3, v3, cross, dot, norm, normalize, scale, add, anyPerp } from "./vec3"

/**
 * Unit quaternion q = w + xi + yj + zk, Hamilton convention.
 * Attitude q maps body-frame vectors to the inertial frame: v_I = q ⊗ v_B ⊗ q*.
 */
export interface Q {
  readonly w: number
  readonly x: number
  readonly y: number
  readonly z: number
}

export const q4 = (w: number, x: number, y: number, z: number): Q => ({ w, x, y, z })
export const QI: Q = q4(1, 0, 0, 0)

export const qmul = (a: Q, b: Q): Q =>
  q4(
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  )
export const qconj = (a: Q): Q => q4(a.w, -a.x, -a.y, -a.z)
export const qnorm = (a: Q): number => Math.hypot(a.w, a.x, a.y, a.z)
export const qnormalize = (a: Q): Q => {
  const n = qnorm(a)
  return q4(a.w / n, a.x / n, a.y / n, a.z / n)
}

/** rotate body vector into inertial frame */
export const qrot = (q: Q, v: V3): V3 => {
  // v' = v + 2w(u×v) + 2u×(u×v), u = (x,y,z)
  const u = v3(q.x, q.y, q.z)
  const t = scale(cross(u, v), 2)
  return add(add(v, scale(t, q.w)), cross(u, t))
}
/** rotate inertial vector into body frame */
export const qrotInv = (q: Q, v: V3): V3 => qrot(qconj(q), v)

export const qFromAxisAngle = (axis: V3, angle: number): Q => {
  const n = normalize(axis)
  const s = Math.sin(angle / 2)
  return q4(Math.cos(angle / 2), n.x * s, n.y * s, n.z * s)
}

/** dq/dt for body rates w (rad/s, body frame) */
export const qDot = (q: Q, w: V3): Q => {
  const p = qmul(q, q4(0, w.x, w.y, w.z))
  return q4(0.5 * p.w, 0.5 * p.x, 0.5 * p.y, 0.5 * p.z)
}

export const qAddScaled = (a: Q, b: Q, s: number): Q => q4(a.w + b.w * s, a.x + b.x * s, a.y + b.y * s, a.z + b.z * s)

/** quaternion whose body axes (ex,ey,ez) map onto the given orthonormal inertial vectors */
export const qFromBasis = (bx: V3, by: V3, bz: V3): Q => {
  const m00 = bx.x, m01 = by.x, m02 = bz.x
  const m10 = bx.y, m11 = by.y, m12 = bz.y
  const m20 = bx.z, m21 = by.z, m22 = bz.z
  const tr = m00 + m11 + m22
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2
    return qnormalize(q4(0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s))
  }
  if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2
    return qnormalize(q4((m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s))
  }
  if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2
    return qnormalize(q4((m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s))
  }
  const s = Math.sqrt(1 + m22 - m00 - m11) * 2
  return qnormalize(q4((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s))
}

/**
 * Attitude whose body x axis is `xAxis` and whose body z axis is as close as possible to `zHint`.
 */
export const qLookX = (xAxis: V3, zHint: V3): Q => {
  const bx = normalize(xAxis)
  let bz = add(zHint, scale(bx, -dot(zHint, bx)))
  if (norm(bz) < 1e-9) bz = anyPerp(bx)
  bz = normalize(bz)
  const by = cross(bz, bx)
  return qFromBasis(bx, by, bz)
}

/** rotation vector (axis*angle) of the shortest rotation taking a to b (both unit quats); expressed in a's body frame */
export const qErrorBody = (current: Q, target: Q): V3 => {
  let e = qmul(qconj(current), target)
  if (e.w < 0) e = q4(-e.w, -e.x, -e.y, -e.z)
  const s = Math.hypot(e.x, e.y, e.z)
  if (s < 1e-12) return v3(2 * e.x, 2 * e.y, 2 * e.z)
  const ang = 2 * Math.atan2(s, e.w)
  return v3((e.x / s) * ang, (e.y / s) * ang, (e.z / s) * ang)
}

export const qSlerp = (a: Q, b0: Q, t: number): Q => {
  let b = b0
  let c = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
  if (c < 0) {
    b = q4(-b.w, -b.x, -b.y, -b.z)
    c = -c
  }
  if (c > 0.9995) return qnormalize(q4(a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t))
  const th = Math.acos(c)
  const s = Math.sin(th)
  const wa = Math.sin((1 - t) * th) / s
  const wb = Math.sin(t * th) / s
  return q4(a.w * wa + b.w * wb, a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb)
}

export const qDotProduct = (a: Q, b: Q): number => a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
