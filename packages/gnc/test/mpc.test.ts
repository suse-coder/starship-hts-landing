/**
 * spec/05 T-GNC-MPC: landing MPC.
 *  1. the plan respects the thrust/tilt constraints;
 *  2. lag-aware model: closed loop on a plant whose thrust really lags (τ = 2 s) lands the 1.2 km
 *     divert with less overshoot than the instantaneous model;
 *  3. solve time budget (the browser worker re-plans at 20 Hz).
 */
import { describe, expect, test } from "bun:test"
import { solveLandingMpc, type MpcProblem } from "../src/mpc"

const G = 9.79
const base = (p: [number, number, number], v: [number, number, number], a: [number, number, number], tgo: number, tau: number): MpcProblem => ({
  p0: p, v0: v, a0: a, tau, n: 25, h: tgo / 25, vRef: [0, 0, -1], vF: [0, 0, -1],
  aZMin: 8.5, aZMax: 21, aMagMin: 8.6, aMagMax: 21.5, tanTilt: Math.tan((25 * Math.PI) / 180),
  weak: null, aWeak: Infinity, uPrev: a, glide: 0.05,
})

/** closed loop: plant with a first-order thrust lag τ_plant, controller with model lag τ_model */
const closedLoop = (tauPlant: number, tauModel: number): { miss: number; overshoot: number; vEnd: number } => {
  const p: [number, number, number] = [30, -1200, 150], v: [number, number, number] = [-10, 60, -20]
  const a: [number, number, number] = [0, 0, G]
  let warm: Float64Array | null = null
  let u: [number, number, number] = [0, 0, G]
  const tEnd = 34, dt = 0.05
  let t = 0, overshoot = 0
  for (let step = 0; step < 2000 && t < tEnd; step++) {
    if (step % 2 === 0) {
      const plan = solveLandingMpc(base([...p], [...v], [...a], Math.max(tEnd - t, 5), tauModel), warm, 60)
      warm = plan.u
      u = [plan.u[0], plan.u[1], plan.u[2]]
    }
    const k = 1 - Math.exp(-dt / tauPlant)
    for (let j = 0; j < 3; j++) a[j] += (u[j] - a[j]) * k
    for (let j = 0; j < 3; j++) { const acc = a[j] - (j === 2 ? G : 0); p[j] += v[j] * dt + 0.5 * acc * dt * dt; v[j] += acc * dt }
    overshoot = Math.max(overshoot, p[1]) // started south (−y), any +y is overshoot past the aim
    t += dt
  }
  return { miss: Math.hypot(p[0], p[1], p[2]), overshoot, vEnd: Math.hypot(...v) }
}

describe("landing MPC", () => {
  test("plan satisfies thrust magnitude and tilt limits", () => {
    const pr = base([200, -600, 120], [0, 40, -15], [0, 0, G], 25, 2)
    const plan = solveLandingMpc(pr, null, 80)
    for (let k = 0; k < pr.n; k++) {
      const [x, y, z] = [plan.u[3 * k], plan.u[3 * k + 1], plan.u[3 * k + 2]]
      const m = Math.hypot(x, y, z)
      expect(m).toBeGreaterThanOrEqual(8.6 - 1e-6)
      expect(m).toBeLessThanOrEqual(21.5 + 1e-6)
      expect(Math.hypot(x, y)).toBeLessThanOrEqual(Math.tan((25 * Math.PI) / 180) * z * 1.0001 + 1e-6)
    }
  })

  test("lag-aware model lands a lagging plant with less overshoot", () => {
    const naive = closedLoop(2.0, 0)
    const aware = closedLoop(2.0, 2.0)
    console.log(`τ-blind: miss ${naive.miss.toFixed(1)} m, overshoot ${naive.overshoot.toFixed(1)} m · τ-aware: miss ${aware.miss.toFixed(1)} m, overshoot ${aware.overshoot.toFixed(1)} m`)
    expect(aware.miss).toBeLessThan(5)
    expect(aware.overshoot).toBeLessThanOrEqual(naive.overshoot + 1e-6)
  })

  test("solve time fits the 20 Hz re-plan budget", () => {
    const pr = base([200, -600, 120], [0, 40, -15], [0, 0, G], 25, 2)
    const t0 = performance.now()
    for (let i = 0; i < 20; i++) solveLandingMpc(pr, null, 60)
    expect((performance.now() - t0) / 20).toBeLessThan(10)
  })
})
