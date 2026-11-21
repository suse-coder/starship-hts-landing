/**
 * @ship/craft — outer mould line, panels, mass properties, coil sizing, Raptor and cold-gas RCS models.
 * References:
 *  - research/05 + spec/01: 9 m diameter, 52.1 m long, 21 barrel rings × 1.83 m = 38.43 m, nosecone
 *    13.67 m (tangent ogive with a spherical cap R_n = 2.5 m).
 *  - Divergence theorem: for a closed surface ∮ n dA = 0 → a uniform pressure exerts no net force.
 *  - Mass properties: m = Σ m_i, x_cg = Σ m_i x_i / m, parallel-axis theorem I = Σ (I_i + m_i d_i²);
 *    principal moments of any real body satisfy I_a ≤ I_b + I_c (triangle inequality).
 *  - Thin shell / solid cylinder / ring / disk moments (e.g. Meriam & Kraige, Appendix B).
 *  - On-axis loop field B = μ0 N I a² / (2 (a² + z²)^{3/2}); loop inductance L = μ0 a N² (ln(8a/gmd) − 2)
 *    (Grover); stored energy ½ L I²; dump time constant τ = L/R.
 *  - Rocket thrust F = ṁ Isp g0, F_SL = F_vac − p_SL A_e (Sutton & Biblarz, Rocket Propulsion Elements);
 *    Raptor 3 sea level 2.45 MN, Isp 350 s; N₂ cold gas Isp ≈ 73 s (Wertz & Larson, SMAD).
 */
import { describe, expect, test } from "bun:test"
import { v3, cross, sub, norm, type V3 } from "@ship/math"
import { MU0, G0 } from "@ship/earth"
import {
  makeVehicle, makePanels, wettedArea, planform, massProps, dryMass, initialConsumables, designCoil, DEFAULT_DESIGN,
  fVac100, ispVac, engineThrust, engineStep, engineOff, rcsWrench, type Consumables,
} from "../src/index"

const v = makeVehicle()
const s = v.shape
const rel = (a: number, b: number): number => Math.abs(a / b - 1)

describe("outer mould line (research/05)", () => {
  test("21 rings × 1.83 m barrel, 13.67 m nosecone, 52.1 m overall, 9 m diameter", () => {
    expect(s.Lc).toBeCloseTo(21 * 1.83, 10)
    expect(s.p.length - s.Lc).toBeCloseTo(13.67, 10)
    expect(s.p.length).toBe(52.1)
    expect(2 * s.p.radius).toBe(9)
    expect(s.r(10)).toBe(4.5)
    expect(s.r(s.Lc)).toBe(4.5)
  })

  test("tangent ogive: zero slope at the barrel joint, C¹ joint to the spherical cap, closes at the tip", () => {
    expect(Math.abs(s.drdx(s.Lc + 1e-7))).toBeLessThan(1e-5)
    const xe = s.xCapEnd, d = 1e-7
    expect(Math.abs(s.r(xe - d) - s.r(xe + d))).toBeLessThan(1e-5)
    expect(Math.abs(s.drdx(xe - d) - s.drdx(xe + d))).toBeLessThan(1e-5)
    expect(s.r(s.p.length - 1e-9)).toBeLessThan(1e-3)
    // nose radius non-increasing toward the tip
    let prev = Infinity
    for (let x = s.Lc; x < s.p.length; x += 0.05) {
      expect(s.r(x)).toBeLessThanOrEqual(prev + 1e-12)
      prev = s.r(x)
    }
  })

  test("spherical cap of radius R_n = 2.5 m: (x − x_c)² + r² = R_n²", () => {
    const Rn = s.p.noseRadius, xc = s.p.length - Rn
    for (let x = s.xCapEnd + 0.01; x < s.p.length; x += 0.1) expect(Math.hypot(x - xc, s.r(x))).toBeCloseTo(Rn, 8)
  })

  test("wetted barrel area = 2πR L_c; planform lies between the barrel and a full cylinder", () => {
    expect(rel(wettedArea(s, 0, s.Lc), 2 * Math.PI * 4.5 * s.Lc)).toBeLessThan(1e-12)
    const pf = planform(s)
    expect(pf.area).toBeGreaterThan(9 * s.Lc)
    expect(pf.area).toBeLessThan(9 * s.p.length)
  })

  test("closed panel hull: Σ n dA = 0 (uniform pressure gives no net force); unit normals", () => {
    const { panels } = makePanels(s, 16)
    let sx = 0, sy = 0, sz = 0, area = 0
    for (const p of panels) {
      expect(norm(p.n)).toBeCloseTo(1, 12)
      sx += p.n.x * p.area
      sy += p.n.y * p.area
      sz += p.n.z * p.area
      area += p.area
    }
    expect(Math.hypot(sx, sy, sz) / area).toBeLessThan(1e-12)
  })
})

describe("mass properties", () => {
  const c0 = initialConsumables(v)
  const consumed = (c: Consumables): number => c.loxHeader + c.ch4Header + c.loxMain + c.ch4Main + c.n2 + c.seed

  test("total mass and CG are the mass-weighted sums of the components", () => {
    const mp = massProps(v, c0)
    expect(mp.m).toBeCloseTo(dryMass(v) + consumed(c0), 6)
    const items = [
      ...v.components.map((c) => [c.mass, c.x] as const),
      [c0.loxHeader, v.xLoxHeader], [c0.ch4Header, v.xCh4Header], [c0.loxMain, v.xLoxMain], [c0.ch4Main, v.xCh4Main], [c0.n2, v.xN2], [c0.seed, 50.2],
    ] as const
    const m = items.reduce((a, [mi]) => a + mi, 0)
    const xcg = items.reduce((a, [mi, xi]) => a + mi * xi, 0) / m
    expect(mp.xcg).toBeCloseTo(xcg, 9)
    expect(mp.xcg).toBeGreaterThan(0)
    expect(mp.xcg).toBeLessThan(s.p.length)
  })

  test("inertia: positive definite, axisymmetric (Iyy = Izz), triangle inequality Ixx ≤ Iyy + Izz", () => {
    for (const c of [c0, { ...c0, loxMain: 0, ch4Main: 0 }, { ...c0, loxHeader: 0, ch4Header: 0, n2: 0 }]) {
      const { I } = massProps(v, c)
      expect(I.x).toBeGreaterThan(0)
      expect(I.y).toBeGreaterThan(0)
      expect(I.y).toBe(I.z)
      expect(I.x).toBeLessThanOrEqual(I.y + I.z)
      // a 52 m slender body: pitch inertia far above roll inertia
      expect(I.y / I.x).toBeGreaterThan(10)
    }
  })

  test("parallel-axis theorem: adding δm at distance d from the CG raises Iyy by its own inertia + δm·m/(m+δm)·d²", () => {
    const base = massProps(v, c0)
    const dm = 1000
    // the seed (a 0.3 m sphere, 0.4 m r²) sits at x = 50.2; add the mass through the seed consumable
    const mp = massProps(v, { ...c0, seed: c0.seed + dm })
    const d = 50.2 - base.xcg
    const expected = base.I.y + 0.4 * dm * 0.3 ** 2 + (dm * base.m * d * d) / (base.m + dm)
    expect(rel(mp.I.y, expected)).toBeLessThan(1e-9)
    expect(mp.xcg).toBeCloseTo((base.m * base.xcg + dm * 50.2) / (base.m + dm), 9)
  })

  test("burning header propellant (forward, x ≈ 44–47 m) moves the CG aft", () => {
    const full = massProps(v, c0)
    const empty = massProps(v, { ...c0, loxHeader: 0, ch4Header: 0 })
    expect(empty.xcg).toBeLessThan(full.xcg)
    expect(full.m - empty.m).toBeCloseTo(DEFAULT_DESIGN.landingProp, 6)
  })

  test("header load split by the methalox mixture ratio O/F = 3.6", () => {
    expect(c0.loxHeader / c0.ch4Header).toBeCloseTo(3.6, 12)
    expect(c0.loxHeader + c0.ch4Header).toBeCloseTo(DEFAULT_DESIGN.landingProp, 9)
  })
})

describe("HTS coil sizing (spec/01 §4)", () => {
  const c = designCoil(DEFAULT_DESIGN, 52.1)
  test("NI gives the design field B_stag on the axis at z_stag: μ0 NI a²/(2(a²+z²)^{3/2}) = 2 T", () => {
    const B = (MU0 * c.NI * c.a ** 2) / (2 * (c.a ** 2 + c.zStag ** 2) ** 1.5)
    expect(B).toBeCloseTo(DEFAULT_DESIGN.bStag, 12)
    expect(c.bCenter).toBeCloseTo((MU0 * c.NI) / (2 * c.a), 9)
  })
  test("circuit consistency: E = ½ L I_op², N = NI/I_op, dump τ = L/R = 2 s", () => {
    expect(rel(c.energy, 0.5 * c.L * c.Iop ** 2)).toBeLessThan(1e-12)
    expect(c.N * c.Iop).toBeCloseTo(c.NI, 6)
    expect(c.L / c.rDump).toBeCloseTo(2, 12)
    // Grover: L = μ0 a N² (ln(8a/gmd) − 2), gmd = 0.2235 (w + h) for a rectangular winding pack
    const gmd = 0.2235 * (c.wpAxial + c.wpRadial)
    expect(rel(c.L, MU0 * c.a * c.N ** 2 * (Math.log((8 * c.a) / gmd) - 2))).toBeLessThan(1e-12)
  })
  test("REBCO current-sharing temperature at I_op/I_c = 0.55 is between T_op and T_c* = 70 K", () => {
    expect(c.tCs).toBeGreaterThan(c.tOp + 10)
    expect(c.tCs).toBeLessThan(70)
    expect(c.tCs).toBeCloseTo(70 - (70 - c.tOp) * 0.55 ** (1 / 1.3), 12)
  })
})

describe("PICA-X nosecone TPS", () => {
  test("area = wetted area of the covered nose; mass between flank and cap areal densities", () => {
    const p = v.pica
    expect(rel(p.area, wettedArea(s, p.x0, s.p.length))).toBeLessThan(0.01)
    expect(p.mass).toBeGreaterThan(p.area * (p.flankThickness * p.density + 3))
    expect(p.mass).toBeLessThan(p.area * (p.capThickness * p.density + 3))
  })
})

describe("Raptor 3 (sea level) and cold-gas RCS", () => {
  const e = v.engines[0]
  test("F_vac = F_SL + p_SL A_e, Isp_vac = Isp_SL F_vac/F_SL, ṁ identical at sea level and in vacuum", () => {
    expect(fVac100(e)).toBeCloseTo(2.45e6 + 101325 * 1.33, 6)
    expect(ispVac(e)).toBeCloseTo((350 * fVac100(e)) / 2.45e6, 9)
    const full = { on: true, spool: 1, throttle: 1, gp: 0, gy: 0 }
    const sl = engineThrust(e, full, 101325, 25)
    const vac = engineThrust(e, full, 0, 25)
    expect(sl.thrust).toBeCloseTo(2.45e6, 3)
    expect(vac.thrust).toBeCloseTo(fVac100(e), 3)
    expect(sl.mdot).toBeCloseTo(vac.mdot, 9)
    expect(sl.thrust / (sl.mdot * G0)).toBeCloseTo(350, 6) // F = ṁ Isp g0
  })

  test("gimballed thrust: |F| = thrust, moment M = r × F about the CG", () => {
    const st = { on: true, spool: 1, throttle: 0.8, gp: 0.1, gy: -0.07 }
    const o = engineThrust(e, st, 0, 27)
    expect(norm(o.F)).toBeCloseTo(o.thrust, 6)
    const M = cross(sub(e.gimbal, v3(27, 0, 0)), o.F)
    expect(norm(sub(o.M, M))).toBeLessThan(1e-6 * norm(M))
  })

  test("actuator limits: throttle ≥ 40 %, gimbal ≤ 15° at ≤ 20°/s, ignition delay before thrust", () => {
    const s1 = engineStep(e, { ...engineOff, throttle: 0.4 }, { on: true, throttle: 0.1, gp: 1, gy: -1 }, 0.1)
    expect(s1.throttle).toBeGreaterThanOrEqual(0.4)
    expect(s1.gp).toBeCloseTo((2 * Math.PI) / 180, 12) // 20°/s × 0.1 s
    expect(s1.gy).toBeCloseTo((-2 * Math.PI) / 180, 12)
    let st = s1
    for (let k = 0; k < 100; k++) st = engineStep(e, st, { on: true, throttle: 1, gp: 1, gy: 0 }, 0.1)
    expect(st.gp).toBeCloseTo((15 * Math.PI) / 180, 12)
    // spool 0.2 (0.5 s after ignition) is still inside the ignition delay: no thrust
    expect(engineThrust(e, { ...st, spool: 0.2 }, 0, 25).thrust).toBe(0)
  })

  test("every RCS couple is a pure torque about one body axis (zero net force), independent of the CG", () => {
    for (const cp of v.couples) {
      const duty = v.thrusters.map((_, i) => (i === cp.a || i === cp.b ? 1 : 0))
      const w1 = rcsWrench(v, duty, 20, 100)
      const w2 = rcsWrench(v, duty, 30, 100)
      expect(norm(w1.F)).toBeLessThan(1e-9)
      expect(norm(sub(w1.M, w2.M))).toBeLessThan(1e-6)
      const m = [Math.abs(w1.M.x), Math.abs(w1.M.y), Math.abs(w1.M.z)].sort((a, b) => b - a)
      expect(m[1]).toBeLessThan(1e-9 * m[0])
      // N₂ cold gas: ṁ = F/(Isp g0), Isp 73 s
      expect(w1.mdot).toBeCloseTo((2 * DEFAULT_DESIGN.rcsThrust) / (73 * G0), 9)
    }
    expect(norm(rcsWrench(v, v.thrusters.map(() => 1), 25, 0).M as V3)).toBe(0) // no gas, no thrust
  })
})
