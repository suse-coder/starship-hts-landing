/**
 * @ship/math — linear algebra, quaternion kinematics, numerics and the counter-based RNG.
 * References: Hamilton quaternion algebra (q̇ = ½ q ⊗ (0, ω) for body rates, e.g. Markley & Crassidis,
 * "Fundamentals of Spacecraft Attitude Determination and Control", 2014, eq. 3.20); Euler's equations
 * of a torque-free rigid body conserve the inertial angular momentum and the rotational kinetic energy
 * (Goldstein, Classical Mechanics, §5.5); RK4 local error O(h⁵).
 */
import { describe, expect, test } from "bun:test"
import {
  type Q, type V3, v3, add, scale, sub, dot, cross, norm, normalize, EX, EY, EZ, angleBetween, m3, invM3, mulMV,
  q4, qmul, qconj, qnorm, qrot, qrotInv, qFromAxisAngle, qDot, qAddScaled, qFromBasis, qLookX, qErrorBody, qSlerp,
  interp1, interp2, bracket, solveTridiag, findRoot, smoothstep, wrapPi, clamp, uniform, gauss, hadamard,
} from "../src/index"

const close = (a: V3, b: V3, tol: number): void => {
  expect(Math.abs(a.x - b.x)).toBeLessThan(tol)
  expect(Math.abs(a.y - b.y)).toBeLessThan(tol)
  expect(Math.abs(a.z - b.z)).toBeLessThan(tol)
}

/** classic RK4 for the attitude ODE (q, ω) with a body-torque function */
const rk4Attitude = (q0: Q, w0: V3, I: V3, torque: (q: Q, w: V3) => V3, dt: number, n: number, renorm: boolean): { q: Q; w: V3 } => {
  const f = (q: Q, w: V3): { dq: Q; dw: V3 } => {
    const tau = torque(q, w)
    const c = cross(w, hadamard(I, w))
    return { dq: qDot(q, w), dw: v3((tau.x - c.x) / I.x, (tau.y - c.y) / I.y, (tau.z - c.z) / I.z) }
  }
  let q = q0
  let w = w0
  for (let i = 0; i < n; i++) {
    const k1 = f(q, w)
    const k2 = f(qAddScaled(q, k1.dq, dt / 2), add(w, scale(k1.dw, dt / 2)))
    const k3 = f(qAddScaled(q, k2.dq, dt / 2), add(w, scale(k2.dw, dt / 2)))
    const k4 = f(qAddScaled(q, k3.dq, dt), add(w, scale(k3.dw, dt)))
    const dq = q4(
      (k1.dq.w + 2 * k2.dq.w + 2 * k3.dq.w + k4.dq.w) / 6, (k1.dq.x + 2 * k2.dq.x + 2 * k3.dq.x + k4.dq.x) / 6,
      (k1.dq.y + 2 * k2.dq.y + 2 * k3.dq.y + k4.dq.y) / 6, (k1.dq.z + 2 * k2.dq.z + 2 * k3.dq.z + k4.dq.z) / 6,
    )
    q = qAddScaled(q, dq, dt)
    if (renorm) q = scaleQ(q, 1 / qnorm(q))
    w = add(w, scale(add(add(k1.dw, scale(add(k2.dw, k3.dw), 2)), k4.dw), dt / 6))
  }
  return { q, w }
}
const scaleQ = (q: Q, s: number): Q => q4(q.w * s, q.x * s, q.y * s, q.z * s)

describe("vec3 / M3", () => {
  test("right-handed cross product and Lagrange identity |a×b|² + (a·b)² = |a|²|b|²", () => {
    close(cross(EX, EY), EZ, 1e-300)
    close(cross(EY, EZ), EX, 1e-300)
    const a = v3(1.3, -2.1, 0.7), b = v3(-0.4, 0.9, 3.3)
    const c = cross(a, b)
    expect(Math.abs(dot(c, a))).toBeLessThan(1e-12)
    expect(Math.abs(dot(c, b))).toBeLessThan(1e-12)
    expect(norm(c) ** 2 + dot(a, b) ** 2).toBeCloseTo(norm(a) ** 2 * norm(b) ** 2, 10)
  })
  test("invM3 is the matrix inverse", () => {
    const A = m3(4, 1, -2, 0.5, 3, 1, -1, 2, 5)
    const x = v3(0.3, -1.7, 2.2)
    close(mulMV(invM3(A), mulMV(A, x)), x, 1e-12)
  })
  test("angleBetween / normalize", () => {
    expect(angleBetween(EX, EY)).toBeCloseTo(Math.PI / 2, 14)
    expect(norm(normalize(v3(3, 4, 12)))).toBeCloseTo(1, 15)
  })
})

describe("quaternions (Hamilton, body → inertial)", () => {
  test("rotation by +90° about z maps x → y (right-hand rule) and preserves length", () => {
    const q = qFromAxisAngle(EZ, Math.PI / 2)
    close(qrot(q, EX), EY, 1e-15)
    const v = v3(0.3, -2, 5)
    expect(norm(qrot(q, v))).toBeCloseTo(norm(v), 13)
    close(qrotInv(q, qrot(q, v)), v, 1e-13)
  })
  test("composition: qrot(a⊗b, v) = qrot(a, qrot(b, v)); a⊗a* = 1", () => {
    const a = qFromAxisAngle(v3(1, 2, -1), 0.7), b = qFromAxisAngle(v3(-0.3, 0.2, 1), -1.9)
    const v = v3(1, -2, 0.5)
    close(qrot(qmul(a, b), v), qrot(a, qrot(b, v)), 1e-13)
    const e = qmul(a, qconj(a))
    expect(e.w).toBeCloseTo(1, 14)
  })
  test("qFromBasis / qLookX build the requested body axes", () => {
    const bx = normalize(v3(1, 1, 0)), bz = v3(0, 0, 1), by = cross(bz, bx)
    const q = qFromBasis(bx, by, bz)
    close(qrot(q, EX), bx, 1e-14)
    close(qrot(q, EZ), bz, 1e-14)
    const q2 = qLookX(v3(0, -3, 0), v3(1, 0, 0.2))
    close(qrot(q2, EX), v3(0, -1, 0), 1e-14)
    // body z = the hint projected ⟂ to body x: (1, 0, 0.2)/|·|
    close(qrot(q2, EZ), normalize(v3(1, 0, 0.2)), 1e-14)
  })
  test("qErrorBody returns the rotation vector (angle·axis) in the current body frame", () => {
    const cur = qFromAxisAngle(v3(0.2, 1, 0.1), 0.4)
    const rel = qFromAxisAngle(EY, 0.25)
    const e = qErrorBody(cur, qmul(cur, rel))
    close(e, v3(0, 0.25, 0), 1e-12)
  })
  test("slerp halfway is the half-angle rotation", () => {
    const s = qSlerp(qFromAxisAngle(EZ, 0), qFromAxisAngle(EZ, 1.2), 0.5)
    close(qrot(s, EX), v3(Math.cos(0.6), Math.sin(0.6), 0), 1e-12)
  })
  test("kinematics q̇ = ½ q⊗(0,ω): constant body rate integrates to the axis–angle rotation", () => {
    // spin at 0.3 rad/s about body (1,2,2)/3 for 10 s → rotation of 3 rad about that axis
    const axis = normalize(v3(1, 2, 2))
    const w = scale(axis, 0.3)
    const q0 = qFromAxisAngle(v3(0.4, -1, 0.3), 0.8)
    const { q } = rk4Attitude(q0, w, v3(1, 1, 1), () => v3(0, 0, 0), 0.01, 1000, false)
    const expected = qmul(q0, qFromAxisAngle(axis, 3.0))
    close(qrot(q, EX), qrot(expected, EX), 1e-9)
    close(qrot(q, EY), qrot(expected, EY), 1e-9)
    // the continuous flow preserves |q| exactly (q · (q⊗ω) = 0); RK4 drifts only at O(h⁴)
    expect(Math.abs(qnorm(q) - 1)).toBeLessThan(1e-9)
  })
  test("torque-free asymmetric rigid body: RK4 conserves inertial angular momentum and kinetic energy", () => {
    const I = v3(1.4e6, 3.7e7, 3.9e7) // Starship-like principal moments, kg m²
    const w0 = v3(0.05, 0.01, -0.02)
    const q0 = qFromAxisAngle(v3(1, 1, 0), 0.3)
    const H = (q: Q, w: V3): V3 => qrot(q, hadamard(I, w))
    const T = (w: V3): number => 0.5 * dot(w, hadamard(I, w))
    const { q, w } = rk4Attitude(q0, w0, I, () => v3(0, 0, 0), 0.01, 20000, true) // 200 s
    const h0 = H(q0, w0), h1 = H(q, w)
    expect(norm(sub(h1, h0)) / norm(h0)).toBeLessThan(1e-8)
    expect(Math.abs(T(w) / T(w0) - 1)).toBeLessThan(1e-8)
  })
})

describe("numerics", () => {
  test("interp1 / bracket: exact on linear data, clamped at the ends", () => {
    const xs = [0, 1, 3, 7], ys = xs.map((x) => 2 * x - 1)
    expect(interp1(xs, ys, 2.2)).toBeCloseTo(3.4, 14)
    expect(interp1(xs, ys, -5)).toBe(-1)
    expect(interp1(xs, ys, 99)).toBe(13)
    const [i, f] = bracket(xs, 5)
    expect(i).toBe(2)
    expect(f).toBeCloseTo(0.5, 14)
  })
  test("interp2 reproduces a bilinear function exactly", () => {
    const xs = [0, 1, 2.5], ys = [-1, 0, 4]
    const fn = (x: number, y: number): number => 1 + 2 * x - 3 * y + 0.5 * x * y
    const z = xs.flatMap((x) => ys.map((y) => fn(x, y)))
    expect(interp2(xs, ys, z, 1.7, 2.3)).toBeCloseTo(fn(1.7, 2.3), 12)
  })
  test("solveTridiag (Thomas) solves A x = d", () => {
    const n = 6
    const a = new Float64Array(n).fill(-1), b = new Float64Array(n).fill(4), c = new Float64Array(n).fill(-1)
    const x = Float64Array.from([1, -2, 3, 0.5, 2, -1])
    const d = new Float64Array(n)
    for (let i = 0; i < n; i++) d[i] = b[i] * x[i] + (i > 0 ? a[i] * x[i - 1] : 0) + (i < n - 1 ? c[i] * x[i + 1] : 0)
    const s = solveTridiag(a, b, c, d)
    for (let i = 0; i < n; i++) expect(s[i]).toBeCloseTo(x[i], 12)
  })
  test("findRoot, smoothstep, wrapPi, clamp", () => {
    expect(findRoot((x) => x * x - 2, 0, 2, 1e-14)).toBeCloseTo(Math.SQRT2, 12)
    expect(smoothstep(0, 1, -1)).toBe(0)
    expect(smoothstep(0, 1, 2)).toBe(1)
    expect(smoothstep(0, 1, 0.5)).toBe(0.5)
    expect(wrapPi(3 * Math.PI + 0.1)).toBeCloseTo(-Math.PI + 0.1, 12)
    expect(clamp(5, 0, 1)).toBe(1)
  })
})

describe("counter-based RNG", () => {
  test("uniform: deterministic, in (0,1), mean 1/2, variance 1/12", () => {
    expect(uniform(42, 7, 3)).toBe(uniform(42, 7, 3))
    let s = 0, s2 = 0
    const n = 200000
    for (let k = 0; k < n; k++) {
      const u = uniform(7, k)
      expect(u > 0 && u < 1).toBe(true)
      s += u
      s2 += u * u
    }
    const m = s / n
    expect(Math.abs(m - 0.5)).toBeLessThan(5 * Math.sqrt(1 / 12 / n))
    expect(Math.abs(s2 / n - m * m - 1 / 12)).toBeLessThan(0.002)
  })
  test("gauss (Box–Muller): mean 0, variance 1, ~68.3 % within ±1σ", () => {
    let s = 0, s2 = 0, in1 = 0
    const n = 200000
    for (let k = 0; k < n; k++) {
      const g = gauss(11, k, 2)
      s += g
      s2 += g * g
      if (Math.abs(g) < 1) in1++
    }
    expect(Math.abs(s / n)).toBeLessThan(5 / Math.sqrt(n))
    expect(Math.abs(s2 / n - 1)).toBeLessThan(5 * Math.sqrt(2 / n))
    expect(Math.abs(in1 / n - 0.682689)).toBeLessThan(0.005)
  })
})
