/**
 * The drawn tower against the contact model (@ship/tower): the meshes of packages/view/src/tower.ts
 * are measured vertex by vertex and compared with `armInnerX` / `armInnerAt` / `armDepthAt` and the
 * `towerBoxes` envelopes, so the picture and the physics stay one thing. Geometry only — nothing
 * here renders (no WebGL): what the structure looks like is not covered by these tests.
 */
import { afterAll, beforeAll, describe, expect, test } from "bun:test"
import * as THREE from "three"
import { TOWER, armDepthAt, armInnerAt, armInnerX, towerBoxes, type ArmState, type TowerBox } from "@ship/tower"
import { makePad, type PadModel } from "../src/tower"

/** canvas stand-in: the pad paints its weathering textures on 2-D canvases, which bun does not have */
const fakeContext: unknown = new Proxy({}, {
  get: (_t, k) => {
    if (k === "getImageData" || k === "createImageData") return (a: number, b: number, w?: number, h?: number) => ({ data: new Uint8ClampedArray(4 * (w ?? a) * (h ?? b)) })
    if (k === "createRadialGradient" || k === "createLinearGradient") return () => ({ addColorStop: () => {} })
    return () => {}
  },
  set: () => true,
})
const host = globalThis as { document?: unknown }
const hadDocument = "document" in host
const realDocument = host.document

let pad: PadModel
beforeAll(() => {
  host.document = { createElement: () => ({ width: 0, height: 0, getContext: () => fakeContext }) }
  pad = makePad()
})
afterAll(() => {
  if (hadDocument) host.document = realDocument
  else delete host.document
})

interface P { readonly e: number; readonly n: number; readonly u: number }

/** every vertex of a mesh (all instances) in the pad ENU frame */
const vertices = (m: THREE.Mesh): P[] => {
  const out: P[] = []
  const pos = m.geometry.attributes.position, v = new THREE.Vector3(), im = new THREE.Matrix4()
  const count = m instanceof THREE.InstancedMesh ? m.count : 1
  for (let i = 0; i < count; i++) {
    if (m instanceof THREE.InstancedMesh) m.getMatrixAt(i, im)
    else im.identity()
    for (let k = 0; k < pos.count; k++) {
      v.fromBufferAttribute(pos, k).applyMatrix4(im).applyMatrix4(m.matrixWorld)
      out.push({ e: v.x, n: -v.z, u: v.y })
    }
  }
  return out
}
const meshes = (o: THREE.Object3D): THREE.Mesh[] => {
  const out: THREE.Mesh[] = []
  o.traverse((c) => { if (c instanceof THREE.Mesh) out.push(c) })
  return out
}
const group = (name: string): THREE.Object3D => {
  const g = pad.root.children.find((c) => c.name === name)
  if (!g) throw new Error(`pad: no child "${name}"`)
  return g
}
const child = (g: THREE.Object3D, name: string): THREE.Mesh => {
  const m = g.children.find((c) => c.name === name)
  if (!(m instanceof THREE.Mesh)) throw new Error(`${g.name}: no mesh "${name}"`)
  return m
}
const place = (s: ArmState): void => {
  pad.setArms(s.open, s.cx)
  pad.root.updateMatrixWorld(true)
}
/** distance from a point to the union of the contact model's boxes (0 inside) */
const outside = (p: P, boxes: readonly TowerBox[]): number => {
  let best = Infinity
  for (const b of boxes) {
    const dx = Math.max(b.lo.x - p.e, 0, p.e - b.hi.x), dy = Math.max(b.lo.y - p.n, 0, p.n - b.hi.y), dz = Math.max(b.lo.z - p.u, 0, p.u - b.hi.z)
    best = Math.min(best, Math.hypot(dx, dy, dz))
  }
  return best
}
const triangles = (m: THREE.Mesh): number =>
  ((m.geometry.index ? m.geometry.index.count : m.geometry.attributes.position.count) / 3) * (m instanceof THREE.InstancedMesh ? m.count : 1)

const state = (open: number, cx = 0): ArmState => ({ open, cmdClose: open < 1, cx })
const STATES = [state(1), state(0), state(0.3, 2.5), state(0, -TOWER.armReach), state(0.6, TOWER.armReach)] as const
const SIDES = [[-1, "chopstick-west"], [1, "chopstick-east"]] as const
/** instance matrices are 32-bit floats: ≈ 1e-5 m at these coordinates */
const F32 = 1e-4
/** thickness of the landing rail (view/tower.ts ARM.rail): over it the inner face is the vertical rail edge */
const RAIL = 0.3
const yTip = TOWER.pos.y - 2 - TOWER.armLength

describe("pad model: public interface", () => {
  test("root group, arm groups and setArms", () => {
    expect(pad.root).toBeInstanceOf(THREE.Group)
    expect(pad.root.name).toBe("pad2")
    expect(typeof pad.setArms).toBe("function")
    expect(pad.root.children.map((c) => c.name)).toEqual(["site", "mechazilla", "chopstick-west", "chopstick-east"])
  })

  test("setArms puts the rails' inner edges where armInnerX has them, for any opening and centre offset", () => {
    for (const s of STATES) {
      place(s)
      const [xl, xr] = armInnerX(s)
      expect(group("chopstick-west").position.toArray()).toEqual([xl, 0, 0])
      expect(group("chopstick-east").position.toArray()).toEqual([xr, 0, 0])
    }
  })

  test("setArms clamps the opening and ignores non-finite input (open → 1, cx → 0)", () => {
    pad.setArms(-3)
    expect(group("chopstick-east").position.x).toBe(armInnerX(state(0))[1])
    pad.setArms(7, 1)
    expect(group("chopstick-east").position.x).toBe(armInnerX(state(1, 1))[1])
    pad.setArms(Number.NaN, Number.NaN)
    expect(group("chopstick-west").position.x).toBe(armInnerX(state(1))[0])
    pad.setArms(0.5)
    expect(group("chopstick-west").position.x).toBe(armInnerX(state(0.5))[0])
  })

  test("the actuator rods end on the trolley lugs", () => {
    const tower = group("mechazilla")
    for (const s of STATES) {
      place(s)
      for (const [side, name] of SIDES) {
        const rod = child(tower, side < 0 ? "rod-west" : "rod-east")
        const arm = group(name)
        const lugs = vertices(child(arm, "box|black")).filter((p) => p.n > 13.7 && p.u < TOWER.armHeight + 1 && p.u > TOWER.armHeight - 5)
        const lugE = (Math.min(...lugs.map((p) => p.e)) + Math.max(...lugs.map((p) => p.e))) / 2
        const ends = [rod.position.x - rod.scale.x / 2, rod.position.x + rod.scale.x / 2]
        if (rod.visible) expect(Math.min(...ends.map((x) => Math.abs(x - lugE)))).toBeLessThan(1e-3)
      }
    }
  })
})

describe("chopsticks against the contact model", () => {
  test("rail top, rail inner edge, tip station and rail span", () => {
    for (const s of STATES) {
      place(s)
      const trolleyN = towerBoxes(s).find((b) => b.id === "arm-west-trolley")?.lo.y ?? Number.NaN
      for (const [side, name] of SIDES) {
        const arm = group(name)
        const rail = vertices(child(arm, "box|rail")), fascia = vertices(child(arm, "box|yellow"))
        const all = meshes(arm).flatMap(vertices)
        // the pins land on the rail: its top is the catch line
        expect(Math.max(...rail.map((p) => p.u))).toBeCloseTo(TOWER.armHeight, 4)
        expect(Math.max(...fascia.map((p) => p.u))).toBeCloseTo(TOWER.armHeight, 4)
        // the rail's inner edge (hazard fascia) is the contact model's inner face at the rail
        const inner = side < 0 ? Math.max(...fascia.map((p) => p.e)) : Math.min(...fascia.map((p) => p.e))
        expect(inner).toBeCloseTo(armInnerAt(s, side, 0), 4)
        // the arm ends at the contact model's tip; the rail runs from there to the trolley
        expect(Math.min(...all.map((p) => p.n))).toBeCloseTo(yTip, 4)
        expect(Math.min(...rail.map((p) => p.n))).toBeCloseTo(yTip, 4)
        expect(Math.max(...rail.map((p) => p.n))).toBeCloseTo(trolleyN, 4)
      }
    }
  })

  test("forward of the trolley nothing is above the rail, below the wedge or inside the inner face", () => {
    for (const s of STATES) {
      place(s)
      const trolleyN = towerBoxes(s).find((b) => b.id === "arm-west-trolley")?.lo.y ?? Number.NaN
      for (const [side, name] of SIDES) {
        const x0 = armInnerAt(s, side, 0)
        let above = -Infinity, below = -Infinity, inside = -Infinity, outboard = -Infinity, deepest = 0
        for (const p of meshes(group(name)).flatMap(vertices)) {
          if (p.n > trolleyN - 1e-3) continue
          const dz = TOWER.armHeight - p.u, o = side * (p.e - x0)
          above = Math.max(above, -dz)
          below = Math.max(below, dz - armDepthAt(p.n))
          // over the rail's thickness the face is the vertical rail edge; below it, the leaning face
          const face = dz <= RAIL + F32 ? 0 : side * (armInnerAt(s, side, dz) - x0)
          inside = Math.max(inside, face - o)
          outboard = Math.max(outboard, o - TOWER.armWidth)
          deepest = Math.max(deepest, dz)
        }
        expect(above).toBeLessThan(F32)
        expect(below).toBeLessThan(F32)
        expect(inside).toBeLessThan(F32)
        // the lower chord follows the leaning inner face, which carries it past the 1.6 m rail width at the root
        expect(outboard).toBeLessThan(0.45)
        // the truss really is a wedge: close to armDepthAt at the trolley, close to armTipDepth at the tip
        expect(deepest).toBeGreaterThan(armDepthAt(trolleyN) - 0.1)
      }
    }
  })

  test("the underside rises to the tip depth", () => {
    place(state(1))
    const tip = meshes(group("chopstick-east")).flatMap(vertices).filter((p) => p.n < yTip + 2)
    const deepest = Math.max(...tip.map((p) => TOWER.armHeight - p.u))
    expect(deepest).toBeLessThan(armDepthAt(yTip + 2) + F32)
    expect(deepest).toBeGreaterThan(TOWER.armTipDepth - 0.1)
  })
})

describe("fixed tower against the contact model's envelopes", () => {
  test("mast, crown, carriage, traverse girder and QD arm lie inside towerBoxes", () => {
    place(state(1))
    const boxes = towerBoxes(state(1))
    for (const m of meshes(group("mechazilla"))) {
      const worst = Math.max(...vertices(m).map((p) => outside(p, boxes)))
      // the QD hoses cross 0.4 m of open air between the mast face and the arm's envelope
      expect(worst).toBeLessThan(m.name === "pipe|black" ? 0.45 : 1e-3)
    }
  })

  test("the QD arm is parked out of the arm span and below the carriage", () => {
    place(state(0))
    const qd = towerBoxes(state(0)).find((b) => b.id === "ship-qd-arm")
    if (!qd) throw new Error("no ship-qd-arm box")
    const steel = vertices(child(group("mechazilla"), "pipe|black")).filter((p) => p.e < qd.hi.x)
    expect(steel.length).toBeGreaterThan(1000)
    for (const p of steel) {
      expect(outside(p, [qd])).toBeLessThan(1e-3)
      expect(p.u).toBeLessThan(TOWER.armHeight - TOWER.armRootDepth)
    }
  })

  test("nothing fixed reaches south of the traverse girder into the catch corridor", () => {
    const beams = towerBoxes(state(1)).find((b) => b.id === "traverse-beams")
    if (!beams) throw new Error("no traverse-beams box")
    for (const m of meshes(group("mechazilla"))) expect(Math.min(...vertices(m).map((p) => p.n))).toBeGreaterThan(beams.lo.y - 1e-3)
  })

  test("the mast stands on its pile caps and tops out at TOWER.height", () => {
    const all = meshes(group("mechazilla")).flatMap(vertices)
    expect(Math.max(...all.map((p) => p.u))).toBeCloseTo(TOWER.height, 3)
    const mast = vertices(child(group("mechazilla"), "box|mast")), caps = vertices(child(group("mechazilla"), "box|concrete"))
    const capTop = Math.max(...caps.filter((p) => Math.abs(p.e) > 3).map((p) => p.u))
    expect(Math.min(...mast.map((p) => p.u))).toBeCloseTo(capTop, 4)
  })
})

describe("real-time budget", () => {
  test("the whole tower is under 40 draw calls and 60 000 triangles", () => {
    const parts = [group("mechazilla"), group("chopstick-west"), group("chopstick-east")].flatMap(meshes)
    expect(parts.length).toBeLessThan(40)
    expect(parts.reduce((t, m) => t + triangles(m), 0)).toBeLessThan(60_000)
  })
})
