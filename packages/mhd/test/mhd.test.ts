/**
 * spec/05 T-coil, T-mhd-phys — HTS coil field, plasma Lorentz interaction, magnet circuit.
 * References:
 *  - Biot–Savart, circular loop: on axis B = μ0 I a² / (2 (a² + z²)^{3/2}), centre μ0 I/(2a); off axis via
 *    complete elliptic integrals (Simpson et al., NASA/TM-2013-217919); far field → dipole
 *    B = μ0 m (3(m̂·r̂)r̂ − m̂)/(4π r³), m = I π a²; ∇·B = 0.
 *  - MHD Ohm's law j = σ_eff (E + u × B), E = −K u × B (electrode loading); Lorentz force on the gas
 *    f = j × B = −σ_eff (1−K) B² u⊥ opposes the flow → the reaction on the vehicle is extra drag
 *    (Bisek, Boyd & Poggie, J. Spacecraft Rockets 2010; Fujino et al. 2006 "MHD heat shield").
 *    Generator power K(1−K)σ|u×B|² > 0 for 0 < K < 1, < 0 for K < 0 (driven).
 *  - Interaction (Stuart) parameter Q = σ_eff B² R_n/(ρ∞ U∞) ∝ B² when the Hall current is blocked (κ = 0);
 *    with Hall leakage σ_eff = σ/(1 + κβ²), β ∝ B → Q saturates at large B.
 *  - spec/05 T-mhd-phys: moment = −Σ r × (j × B) dV; zero with the coil off, K_s = 1, or V < 2 km/s;
 *    force ≤ momentum flux (F_cap = 1.2 q∞ π (R_n+Δ)²); heat factor (Δ0/Δ)^½ ≤ 1.
 *  - Magnet circuit (spec/02 §6): dI/dt = V/L, V = min(1200 V, 450 kW/I); quench dump τ = L/R = 2 s;
 *    REBCO I_c(T) ∝ (1 − T/70 K)^1.3, I_op/I_c(25 K) = 0.55 → T_cs = 70 − 45·0.55^{1/1.3} = 41.6 K.
 */
import { describe, expect, test } from "bun:test"
import { type V3, v3, normalize, norm, sub, cross, scale, add, dot } from "@ship/math"
import { atmosphere, MU0 } from "@ship/earth"
import { lookupStag, SHOCK_TABLE } from "@ship/gas"
import { makeVehicle, massProps, initialConsumables, DEFAULT_DESIGN } from "@ship/craft"
import {
  loopField, coilField, makeMhdModel, mhdBasis, mhdApply, dipoleTorque, N_SECT, initCoil, coilStep, coilEnergy, coilNI,
  currentSharingT, coldEnthalpy, coldTemperature, coldHeatCapacity, dumpEnthalpy,
  type MhdInput, type CoilState,
} from "../src/index"

const v = makeVehicle()
const model = makeMhdModel(v)
const xcg = massProps(v, initialConsumables(v)).xcg
const rel = (a: number, b: number): number => Math.abs(a / b - 1)
const DEG = Math.PI / 180

const flight = (hkm: number, U: number, alphaDeg: number, over: Partial<MhdInput> = {}): MhdInput => {
  const a = atmosphere(hkm * 1000)
  const al = alphaDeg * DEG
  return {
    NI: v.coil.NI, vhat: normalize(v3(Math.cos(al), 0, -Math.sin(al))), U, rhoInf: a.rho, pInf: a.p, qInf: 0.5 * a.rho * U * U,
    stag: lookupStag(SHOCK_TABLE, hkm * 1000, U), xcg, kappa: DEFAULT_DESIGN.hallClosure, seedFraction: DEFAULT_DESIGN.seedFraction, ...over,
  }
}
const uniformLoad = (u: number): number[] => Array(N_SECT).fill(u)

/** direct numerical Biot–Savart of a loop of radius a in the plane x = 0 (per ampere) at (x, ρ, 0) */
const biotSavart = (a: number, x: number, rho: number, nSeg = 4000): V3 => {
  let B = v3(0, 0, 0)
  for (let i = 0; i < nSeg; i++) {
    const p0 = (i / nSeg) * 2 * Math.PI, p1 = ((i + 1) / nSeg) * 2 * Math.PI, pm = 0.5 * (p0 + p1)
    const dl = v3(0, a * (Math.cos(p1) - Math.cos(p0)), a * (Math.sin(p1) - Math.sin(p0)))
    const R = sub(v3(x, rho, 0), v3(0, a * Math.cos(pm), a * Math.sin(pm)))
    B = add(B, scale(cross(dl, R), MU0 / (4 * Math.PI) / norm(R) ** 3))
  }
  return B
}

describe("coil field (T-coil)", () => {
  test("loop on axis = μ0 a²/(2(a²+z²)^{3/2}) and μ0/(2a) at the centre", () => {
    const a = 1.75
    for (const z of [0, 0.3, 1.2, 4, -2.5]) {
      const [bx, br] = loopField(a, 0, z, 0)
      expect(rel(bx, (MU0 * a * a) / (2 * (a * a + z * z) ** 1.5))).toBeLessThan(1e-12)
      expect(br).toBe(0)
    }
    expect(rel(loopField(a, 0, 0, 0)[0], MU0 / (2 * a))).toBeLessThan(1e-12)
  })

  test("off-axis elliptic-integral field = direct Biot–Savart summation (1e-5)", () => {
    for (const [x, rho] of [[0.9, 1.2], [0.2, 0.5], [-1.5, 2.6], [0.05, 1.9]]) {
      const [bx, br] = loopField(1.75, 0, x, rho)
      const B = biotSavart(1.75, x, rho)
      expect(Math.hypot(bx - B.x, br - B.y) / norm(B)).toBeLessThan(1e-5)
    }
  })

  test("far field → magnetic dipole μ0 m (3(m̂·r̂)r̂ − m̂)/(4π r³) (0.1 % at r = 100 a)", () => {
    const a = 1.75, r = 100 * a
    for (const th of [0, 30 * DEG, 60 * DEG, 90 * DEG]) {
      const x = r * Math.cos(th), rho = r * Math.sin(th)
      const [bx, br] = loopField(a, 0, x, rho)
      const k = (MU0 * Math.PI * a * a) / (4 * Math.PI * r ** 3)
      const dx = k * (3 * Math.cos(th) ** 2 - 1), dr = k * 3 * Math.cos(th) * Math.sin(th)
      expect(Math.hypot(bx - dx, br - dr) / Math.hypot(dx, dr)).toBeLessThan(1e-3)
    }
  })

  test("∇·B = ∂B_x/∂x + (1/ρ) ∂(ρ B_ρ)/∂ρ = 0", () => {
    const a = 1.75, h = 1e-4
    for (const [x, rho] of [[0.6, 0.9], [1.3, 2.2], [-0.4, 3.0]]) {
      const dBx = (loopField(a, 0, x + h, rho)[0] - loopField(a, 0, x - h, rho)[0]) / (2 * h)
      const dRBr = ((rho + h) * loopField(a, 0, x, rho + h)[1] - (rho - h) * loopField(a, 0, x, rho - h)[1]) / (2 * h * rho)
      expect(Math.abs(dBx + dRBr) * a / Math.hypot(...loopField(a, 0, x, rho))).toBeLessThan(1e-6)
    }
  })

  test("winding pack at the design NI gives B ≈ B_stag = 2 T at the stagnation point (1 %)", () => {
    const [bx, br] = coilField(v, v.shape.p.length, 0)
    expect(br).toBe(0)
    expect(rel(bx * v.coil.NI, DEFAULT_DESIGN.bStag)).toBeLessThan(0.01)
    expect(rel(Math.hypot(...[model.gBx[0], model.gBr[0]]) * v.coil.NI, DEFAULT_DESIGN.bStag)).toBeLessThan(0.01)
  })
})

describe("plasma interaction (T-mhd-phys)", () => {
  test("no authority with the coil off, with open-circuit electrodes (K_s = 1), or a cold shock layer", () => {
    const off = mhdBasis(model, flight(65, 7000, 10, { NI: 0 }))
    expect(off.fCap).toBe(0)
    expect(mhdApply(off, uniformLoad(1), 1e4, 2.5).F).toEqual(v3(0, 0, 0))
    const b = mhdBasis(model, flight(65, 7000, 10))
    const open = mhdApply(b, uniformLoad(0), 1e4, 2.5)
    expect(norm(open.F)).toBe(0)
    expect(norm(open.M)).toBe(0)
    expect(open.power).toBe(0)
    // 30 km, 1.2 km/s: T0 < 1500 K → no conducting layer
    const cold = mhdBasis(model, flight(30, 1200, 10))
    expect(cold.fCap).toBe(0)
  })

  test("authority fades out below ≈ 2 km/s: |M| < 1e-3 of the aerodynamic moment scale q S L at 1.8 km/s", () => {
    const inp = flight(40, 1800, 10)
    const o = mhdApply(mhdBasis(model, inp), uniformLoad(1.6), inp.qInf, 2.5)
    expect(norm(o.M)).toBeLessThan(1e-3 * inp.qInf * Math.PI * 4.5 ** 2 * 52.1)
  })

  test("the reaction force is always drag (F·v̂ < 0) — the Lorentz force decelerates the shock layer", () => {
    for (const [h, U, al] of [[75, 7400, 0], [65, 7000, 10], [55, 6000, 15], [50, 4500, 20]]) {
      const inp = flight(h, U, al)
      const b = mhdBasis(model, inp)
      for (const u of [0.3, 1, 1.6]) {
        const o = mhdApply(b, uniformLoad(u), inp.qInf, 2.5)
        expect(dot(o.F, inp.vhat)).toBeLessThan(0)
      }
    }
  })

  test("axisymmetric flow (α = 0) with uniform loading: no side force, no moment", () => {
    const inp = flight(65, 7000, 0)
    const o = mhdApply(mhdBasis(model, inp), uniformLoad(1), inp.qInf, 2.5)
    expect(Math.hypot(o.F.y, o.F.z)).toBeLessThan(1e-9 * Math.abs(o.F.x))
    expect(norm(o.M)).toBeLessThan(1e-9 * Math.abs(o.F.x) * 52)
  })

  test("momentum bound: |F| < F_cap = 1.2 q∞ π (R_n + Δ)² even when the linear force is 10× larger", () => {
    const inp = flight(65, 7000, 10)
    const b = mhdBasis(model, inp)
    expect(rel(b.fCap, 1.2 * inp.qInf * Math.PI * (2.5 + b.delta0) ** 2)).toBeLessThan(1e-12)
    for (const u of [0.1, 1, 1.6]) {
      const o = mhdApply(b, uniformLoad(u), inp.qInf, 2.5)
      expect(o.forceMag).toBeLessThan(b.fCap)
      expect(o.scale).toBeGreaterThan(0)
      expect(o.scale).toBeLessThanOrEqual(1)
    }
  })

  test("heat factor (Δ0/Δ)^½ ≤ 1, = 1 without force, decreasing with |F|; standoff Δ/Δ0 = √(1+Q_eff) ≥ 1", () => {
    const inp = flight(65, 7000, 10)
    const b = mhdBasis(model, inp)
    let prevF = -1, prevH = 1.0000001
    for (const u of [0, 0.05, 0.2, 0.6, 1.2]) {
      const o = mhdApply(b, uniformLoad(u), inp.qInf, 2.5)
      expect(o.heatFactor).toBeLessThanOrEqual(1)
      expect(o.standoffFactor).toBeGreaterThanOrEqual(1)
      expect(o.heatFactor).toBeCloseTo(1 / Math.sqrt(o.standoffFactor), 14)
      expect(o.forceMag).toBeGreaterThan(prevF)
      expect(o.heatFactor).toBeLessThan(prevH)
      prevF = o.forceMag
      prevH = o.heatFactor
    }
  })

  test("Stuart number Q ∝ B² with Hall current blocked (κ = 0); saturates with Hall leakage (κ > 0)", () => {
    const q1 = mhdBasis(model, flight(65, 7000, 10, { kappa: 0, seedFraction: 0 })).qInteraction
    const q2 = mhdBasis(model, flight(65, 7000, 10, { kappa: 0, seedFraction: 0, NI: 2 * v.coil.NI })).qInteraction
    expect(q2 / q1).toBeCloseTo(4, 9)
    const k1 = mhdBasis(model, flight(65, 7000, 10)).qInteraction
    const k2 = mhdBasis(model, flight(65, 7000, 10, { NI: 2 * v.coil.NI })).qInteraction
    expect(k2 / k1).toBeLessThan(4)
    expect(k2 / k1).toBeGreaterThan(1)
  })

  test("Q rises steeply with entry speed at fixed altitude (Saha: hotter layer → far more electrons)", () => {
    const q4 = mhdBasis(model, flight(65, 4000, 10, { kappa: 0, seedFraction: 0 })).qInteraction
    const q7 = mhdBasis(model, flight(65, 7000, 10, { kappa: 0, seedFraction: 0 })).qInteraction
    expect(q7 / q4).toBeGreaterThan(10)
  })

  test("electrode power = Σ u(1−u) P_s: generator for 0 < u < 1, zero at short circuit, consumed when driven", () => {
    const inp = flight(65, 7000, 10)
    const b = mhdBasis(model, inp)
    const P = b.P.reduce((a, x) => a + x, 0)
    const half = mhdApply(b, uniformLoad(0.5), inp.qInf, 2.5)
    expect(rel(half.power, 0.25 * P * half.scale)).toBeLessThan(1e-12)
    expect(half.power).toBeGreaterThan(0)
    expect(mhdApply(b, uniformLoad(1), inp.qInf, 2.5).power).toBe(0)
    expect(mhdApply(b, uniformLoad(1.6), inp.qInf, 2.5).power).toBeLessThan(0)
  })

  test("moment is −Σ r × f about the CG: M(x_cg + d) = M(x_cg) − d x̂ × F", () => {
    const a = flight(60, 6600, 12)
    const ba = mhdBasis(model, a)
    const bb = mhdBasis(model, { ...a, xcg: a.xcg + 3 })
    for (let s = 0; s < N_SECT; s++) {
      const expected = sub(ba.M[s], cross(v3(3, 0, 0), ba.F[s]))
      expect(norm(sub(bb.M[s], expected))).toBeLessThan(1e-9 * (norm(ba.M[s]) + 1))
    }
  })

  test("geomagnetic torque on the coil dipole τ = m × B, m = NI π a² x̂", () => {
    const B = v3(2e-5, -3e-5, 1e-5)
    const t = dipoleTorque(1e7, 1.75, B)
    const m = v3(1e7 * Math.PI * 1.75 ** 2, 0, 0)
    expect(norm(sub(t, cross(m, B)))).toBeLessThan(1e-12 * norm(t))
  })
})

describe("magnet circuit and cryogenics", () => {
  const c = v.coil
  const hold = { command: "hold" as const, warmWallT: 290, mhdPower: 0, avionicsLoad: 0 }
  const persistent = (Tc: number): CoilState => ({ ...initCoil(c), mode: "persistent", I: c.Iop, Tc })

  test("ramp time = L·I₁/1200 V + ½L(I_op² − I₁²)/450 kW (voltage- then power-limited), E = ½ L I_op²", () => {
    let s = initCoil(c)
    let t = 0
    const dt = 0.05
    while (s.I < c.Iop && t < 5000) {
      s = coilStep(c, s, { ...hold, command: "ramp" }, dt)
      t += dt
    }
    const I1 = 450e3 / 1200
    const tRef = (c.L * I1) / 1200 + (0.5 * c.L * (c.Iop ** 2 - I1 ** 2)) / 450e3
    expect(rel(t, tRef)).toBeLessThan(1e-3)
    expect(s.mode).toBe("persistent")
    expect(rel(coilEnergy(c, s), c.energy)).toBeLessThan(1e-12)
    expect(coilNI(c, s)).toBeCloseTo(c.NI, 3)
  })

  test("persistent mode holds the current (no joint resistance modelled)", () => {
    let s = persistent(24.5)
    for (let k = 0; k < 1000; k++) s = coilStep(c, s, hold, 0.1)
    expect(s.I).toBe(c.Iop)
    expect(s.mode).toBe("persistent")
  })

  test("quench dump: I(t) = I0 e^{−t/τ}, τ = L/R = 2 s; 70 % of the released energy in the dump resistor", () => {
    let s: CoilState = { ...persistent(24.5), mode: "quench" }
    const E0 = coilEnergy(c, s)
    const dt = 1e-3
    for (let k = 0; k < 2000; k++) s = coilStep(c, s, hold, dt)
    expect(rel(s.I / c.Iop, Math.exp(-1))).toBeLessThan(1e-3)
    expect(rel(s.dumped, 0.7 * (E0 - coilEnergy(c, s)))).toBeLessThan(1e-9)
  })

  // Fixed (T12): T_cs reduced to 70 − 45·(I/I_op)^{1/1.3} (the 0.55 cancelled → zero margin at I_op).
  // Now T_cs = 70 − 45·(0.55·I/I_op)^{1/1.3}, the 0.55 recovered from the craft design value tCs.
  test("current-sharing temperature at I_op = design T_cs (41.6 K): a coil at 30 K must not quench", () => {
    expect(c.tCs).toBeCloseTo(41.59, 1)
    expect(currentSharingT(c, c.Iop)).toBeCloseTo(c.tCs, 10)
    expect(currentSharingT(c, c.Iop)).toBeCloseTo(70 - 45 * 0.55 ** (1 / 1.3), 10)
    expect(currentSharingT(c, c.Iop / 2)).toBeGreaterThan(c.tCs) // margin grows as the current falls
    const s = coilStep(c, persistent(30), hold, 0.01)
    expect(s.mode).toBe("persistent")
    expect(coilStep(c, persistent(41.0), hold, 0.01).mode).toBe("persistent")
    expect(coilStep(c, persistent(42.0), hold, 0.01).mode).toBe("quench")
  })

  test("cold-mass heat capacity C(T) from NIST/Debye data: order of the design value at 25 K, H(25→300 K) ≈ 2.4e8 J", () => {
    // design value (craft designCoil) ≈ 80 kJ/K near 25 K; the material model gives the same order
    expect(coldHeatCapacity(c, 25) / c.heatCapacity).toBeGreaterThan(0.7)
    expect(coldHeatCapacity(c, 25) / c.heatCapacity).toBeLessThan(2)
    // C(T) rises steeply (Debye T³ → Dulong–Petit)
    expect(coldHeatCapacity(c, 100)).toBeGreaterThan(3 * coldHeatCapacity(c, 25))
    const h = coldEnthalpy(c, 300) - coldEnthalpy(c, 25)
    expect(h).toBeGreaterThan(2.0e8)
    expect(h).toBeLessThan(3.0e8)
    // inverse is exact
    for (const T of [24.3, 35.61, 63.15, 77.0, 150, 299]) expect(coldTemperature(c, coldEnthalpy(c, T))).toBeCloseTo(T, 9)
  })

  // Fixed (T12): the quench heated a constant 80 kJ/K (→ ≈ 1100 K). Now the enthalpy H(T) of the winding
  // pack (Hastelloy/Cu/Ag tape, CFRP structure, SN2 buffer) is inverted: 0.3·E0 ≈ 87 MJ → ≈ 160 K.
  test("full quench at I_op leaves the cold mass below room temperature (enthalpy H(25→300 K) ≫ 0.3 E0)", () => {
    let s: CoilState = { ...persistent(24.5), mode: "quench" }
    const E0 = coilEnergy(c, s)
    for (let k = 0; k < 2000; k++) s = coilStep(c, s, hold, 0.01)
    expect(s.mode).toBe("off")
    expect(s.Tc).toBeLessThan(300)
    expect(s.Tc).toBeGreaterThan(100)
    // lumped balance: within 1 K of H⁻¹(H(24.5 K) + 0.3 E0) (the 20 s of cryostat leak adds < 1 kJ … 2 kJ)
    expect(Math.abs(s.Tc - coldTemperature(c, coldEnthalpy(c, 24.5) + 0.3 * E0))).toBeLessThan(1)
    expect(s.dumped).toBeCloseTo(0.7 * E0, -1)
    expect(s.tDump).toBeGreaterThan(c.dumpT0)
  })
})

describe("controlled fast discharge (\"dump\")", () => {
  const c = v.coil
  const cmd = (command: "hold" | "dump" | "rampdown") => ({ command, warmWallT: 290, mhdPower: 0, avionicsLoad: 0 })
  const persistent: CoilState = { ...initCoil(c), mode: "persistent", I: c.Iop, Tc: 24.5 }

  test("time constant: I(t) = I₀ e^{−t R_d/L}, τ = L/R_d = 2 s (exact exponential per step)", () => {
    let s = coilStep(c, persistent, cmd("dump"), 1e-3)
    expect(s.mode).toBe("dump")
    for (let k = 1; k < 2000; k++) s = coilStep(c, s, cmd("hold"), 1e-3) // switch stays open: command ignored
    expect(s.mode).toBe("dump")
    expect(rel(s.I / c.Iop, Math.exp(-1))).toBeLessThan(1e-9)
    // coarse steps give the same decay (sim uses 0.02–0.1 s in entry)
    let q = persistent
    for (let k = 0; k < 20; k++) q = coilStep(c, q, cmd("dump"), 0.1)
    expect(rel(q.I / c.Iop, Math.exp(-1))).toBeLessThan(1e-9)
  })

  test("energy balance: ALL of ½ L I₀² ends in the dump resistor; nothing to the battery, no quench", () => {
    let s = persistent
    const E0 = coilEnergy(c, s)
    let k = 0
    for (; k < 100000 && s.mode !== "off"; k++) s = coilStep(c, s, cmd("dump"), 0.01)
    expect(s.mode).toBe("off")
    expect(s.I).toBe(0)
    expect(k * 0.01).toBeGreaterThan(2 * Math.log(c.Iop)) // ≈ 16 s until I < 1 A
    expect(rel(s.dumped, E0)).toBeLessThan(1e-12)
    expect(rel(dumpEnthalpy(c, s.tDump) - dumpEnthalpy(c, c.dumpT0), E0)).toBeLessThan(1e-9)
    // no normal zone: the cold mass only sees the hysteretic tape loss (≈ 6.5 kJ) + cryostat leak
    expect(s.Tc - 24.5).toBeLessThan(0.2)
    // the battery never receives dump energy (only the cryocooler draws from it)
    expect(s.battery).toBeLessThanOrEqual(persistent.battery)
  })

  test("dump-resistor temperature: AISI 304 plates sized for ΔT ≈ 400 K at the full stored energy", () => {
    let s = persistent
    for (let k = 0; k < 3000 && s.mode !== "off"; k++) s = coilStep(c, s, cmd("dump"), 0.01)
    const dT = s.tDump - c.dumpT0
    // design: m = E0 / (530 J/kg K · 400 K); the c_p(T) integral gives the same rise within 5 %
    expect(c.dumpMass).toBeCloseTo(c.energy / (530 * 400), 6)
    expect(dT).toBeGreaterThan(380)
    expect(dT).toBeLessThan(420)
    expect(s.tDump).toBeLessThan(1000) // far below the 304 solidus (≈ 1670 K)
  })

  test("dump voltage V = I·R_d is recorded against the insulation limit (≈ 97 kV ≫ 3 kV: reported, not hidden)", () => {
    const s = coilStep(c, persistent, cmd("dump"), 0.01)
    expect(s.vDump).toBeCloseTo(c.Iop * c.rDump, 6)
    expect(s.vDumpMax).toBe(s.vDump)
    expect(s.vDumpMax / c.vInsulation).toBeGreaterThan(30) // the τ = 2 s design violates a 3 kV insulation limit
    expect(c.vInsulation / c.Iop).toBeLessThan(c.rDump) // R_d ≤ 1 Ω would be needed → τ ≥ 65 s
  })

  test("rampdown (normal switch-off) returns ≈ 0.9 E₀ to the battery and dumps nothing; dump on an idle coil does nothing", () => {
    let s: CoilState = { ...persistent, battery: 0.1 * c.batteryEnergy }
    const E0 = coilEnergy(c, s), b0 = s.battery
    let busE = 0 // ∫ bus power dt (converter return − cryocooler electrical)
    for (let k = 0; k < 200000 && s.mode !== "off"; k++) {
      s = coilStep(c, s, cmd("rampdown"), 0.05)
      busE += s.busPower * 0.05
    }
    expect(s.mode).toBe("off")
    expect(s.dumped).toBe(0)
    expect(rel(s.battery - b0, busE)).toBeLessThan(1e-9) // battery = ∫ bus power
    expect((s.battery - b0) / E0).toBeGreaterThan(0.85)
    expect((s.battery - b0) / E0).toBeLessThan(0.9)
    const idle = coilStep(c, initCoil(c), cmd("dump"), 0.01)
    expect(idle.mode).toBe("off")
    expect(idle.dumped).toBe(0)
  })
})

describe("planned discharge after the steering switch-off (Q31)", () => {
  const c = v.coil
  const cmd = (command: "hold" | "discharge") => ({ command, warmWallT: 290, mhdPower: 0, avionicsLoad: 0 })
  const persistent: CoilState = { ...initCoil(c), mode: "persistent", I: c.Iop, Tc: 24.5 }

  test("terminal voltage I·R_dis stays at or below the 3 kV insulation rating", () => {
    const s = coilStep(c, persistent, cmd("discharge"), 0.01)
    expect(s.mode).toBe("discharge")
    expect(s.vDumpMax).toBeLessThanOrEqual(c.vInsulation + 1e-6)
  })

  test("τ = L/R_dis ≈ 65 s, and < 1 % of the current is left after 300 s (before the landing burn)", () => {
    expect(c.L / c.rDischarge).toBeGreaterThan(60)
    expect(c.L / c.rDischarge).toBeLessThan(70)
    let s = persistent
    for (let k = 0; k < 3000; k++) s = coilStep(c, s, cmd("hold"), k === 0 ? 0.1 : 0.1) // switch stays open after the first command
    let q = coilStep(c, persistent, cmd("discharge"), 0.1)
    for (let k = 1; k < 3000; k++) q = coilStep(c, q, cmd("hold"), 0.1)
    expect(s.mode).toBe("persistent") // a hold never discharges
    expect(q.I / c.Iop).toBeLessThan(0.01)
  })

  test("energy balance: all of ½ L I₀² ends in the resistor bank, no quench, no battery return", () => {
    const E0 = coilEnergy(c, persistent)
    let s = persistent
    for (let k = 0; k < 200000 && s.mode !== "off"; k++) s = coilStep(c, s, cmd("discharge"), 0.05)
    expect(s.mode).toBe("off")
    expect(rel(s.dumped, E0)).toBeLessThan(1e-9)
    expect(s.Tc - 24.5).toBeLessThan(0.5)
    expect(s.battery).toBeLessThanOrEqual(persistent.battery)
  })
})
