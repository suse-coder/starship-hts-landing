/**
 * Guidance's own aerodynamic-moment model (R6, T08 P0-2).
 *
 * The plant now cancels `cmd.ffMoment`, which must come from the guidance's *own* aero table, not from
 * the plant's exact `aero.M`. The table stores the pitching-moment coefficient for flow in the body
 * x–z plane; at a banked attitude the same normal force (normal to the body axis, opposite the
 * transverse flow) produces both M_y and M_z with the same arm, so the tabulated moment is projected
 * onto the actual transverse flow direction.
 */
import { describe, expect, test } from "bun:test"
import { momentProjection } from "../src/index"

const D = Math.PI / 180

describe("guidance aero moment (R6 P0-2)", () => {
  test("reduces to the table pitch moment for flow in the body x–z plane (v_y = 0)", () => {
    const m = 1.23e6
    const M = momentProjection(m, 0, -1)
    expect(M.x).toBe(0)
    expect(M.y).toBeCloseTo(m, 6)
    expect(M.z).toBeCloseTo(0, 6)
    // and the opposite sign for flow from the other side of the plane
    const M2 = momentProjection(m, 0, 1)
    expect(M2.y).toBeCloseTo(-m, 6)
  })

  test("banking the flow rotates the moment with it, preserving magnitude and ⟂ to the flow", () => {
    const m = 5e5
    for (const deg of [0, 20, 45, 60, 90, 135, 180, -30]) {
      const phi = deg * D
      const vy = Math.sin(phi)
      const vz = -Math.cos(phi)
      const M = momentProjection(m, vy, vz)
      expect(Math.hypot(M.y, M.z)).toBeCloseTo(m, 6)
      // the moment (r × F, r along body x) is perpendicular to the transverse flow direction
      expect(M.y * vy + M.z * vz).toBeCloseTo(0, 6)
    }
  })

  test("no transverse flow → no static moment (and no divide-by-zero)", () => {
    const M = momentProjection(1e7, 0, 0)
    expect(Math.hypot(M.x, M.y, M.z)).toBe(0)
  })
})
