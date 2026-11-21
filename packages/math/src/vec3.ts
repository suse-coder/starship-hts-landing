/**
 * Immutable 3-vectors. All functions are pure; nothing mutates its arguments.
 * Plain objects are used (not tuples) because V8 allocates and inlines them well.
 */
export interface V3 {
  readonly x: number
  readonly y: number
  readonly z: number
}

export const v3 = (x: number, y: number, z: number): V3 => ({ x, y, z })
export const ZERO: V3 = v3(0, 0, 0)
export const EX: V3 = v3(1, 0, 0)
export const EY: V3 = v3(0, 1, 0)
export const EZ: V3 = v3(0, 0, 1)

export const add = (a: V3, b: V3): V3 => v3(a.x + b.x, a.y + b.y, a.z + b.z)
export const sub = (a: V3, b: V3): V3 => v3(a.x - b.x, a.y - b.y, a.z - b.z)
export const scale = (a: V3, s: number): V3 => v3(a.x * s, a.y * s, a.z * s)
export const neg = (a: V3): V3 => v3(-a.x, -a.y, -a.z)
/** a + b*s */
export const addScaled = (a: V3, b: V3, s: number): V3 => v3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s)
export const dot = (a: V3, b: V3): number => a.x * b.x + a.y * b.y + a.z * b.z
export const cross = (a: V3, b: V3): V3 =>
  v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
export const norm2 = (a: V3): number => a.x * a.x + a.y * a.y + a.z * a.z
export const norm = (a: V3): number => Math.sqrt(norm2(a))
export const normalize = (a: V3): V3 => {
  const n = norm(a)
  return n > 0 ? scale(a, 1 / n) : ZERO
}
export const lerpV = (a: V3, b: V3, t: number): V3 =>
  v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t)
export const hadamard = (a: V3, b: V3): V3 => v3(a.x * b.x, a.y * b.y, a.z * b.z)
export const sum = (vs: readonly V3[]): V3 => vs.reduce(add, ZERO)
/** component of a perpendicular to unit vector n */
export const reject = (a: V3, n: V3): V3 => addScaled(a, n, -dot(a, n))
export const angleBetween = (a: V3, b: V3): number => {
  const c = dot(a, b) / Math.max(norm(a) * norm(b), 1e-300)
  return Math.acos(Math.max(-1, Math.min(1, c)))
}
/** any unit vector perpendicular to unit vector n */
export const anyPerp = (n: V3): V3 =>
  Math.abs(n.x) < 0.9 ? normalize(cross(n, EX)) : normalize(cross(n, EY))
export const isFiniteV = (a: V3): boolean => Number.isFinite(a.x) && Number.isFinite(a.y) && Number.isFinite(a.z)

/** 3x3 matrix, row-major, immutable */
export interface M3 {
  readonly m: readonly [number, number, number, number, number, number, number, number, number]
}
export const m3 = (...m: [number, number, number, number, number, number, number, number, number]): M3 => ({ m })
export const diag = (a: number, b: number, c: number): M3 => m3(a, 0, 0, 0, b, 0, 0, 0, c)
export const mulMV = (A: M3, v: V3): V3 => {
  const m = A.m
  return v3(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z)
}
export const invM3 = (A: M3): M3 => {
  const [a, b, c, d, e, f, g, h, i] = A.m
  const A0 = e * i - f * h
  const B0 = -(d * i - f * g)
  const C0 = d * h - e * g
  const det = a * A0 + b * B0 + c * C0
  const s = 1 / det
  return m3(
    A0 * s, -(b * i - c * h) * s, (b * f - c * e) * s,
    B0 * s, (a * i - c * g) * s, -(a * f - c * d) * s,
    C0 * s, -(a * h - b * g) * s, (a * e - b * d) * s,
  )
}
