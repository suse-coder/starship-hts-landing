import * as THREE from "three"
import { CH, type Recording } from "@ship/sim"

/**
 * Predicted-path overlay in the ship-centred local frame.
 *
 * The cyan/blue line is the recorded future trajectory (the simulation's actual `pe/pn/pu` ahead of
 * the playhead), resampled to a fixed 32 control points and smoothed with CatmullRomCurve3. The pale
 * line is the landing MPC's per-sample rollout (`rec.pred`, see `PlanBuffer` below) — a real planned
 * trajectory, not a synthetic target — and is optional: the overlay silently falls back to the
 * recorded path until the orchestrator attaches one.
 *
 * Fixed allocation: 2 × 96 polyline vertices (Float32Array) and 2 × 32 Vector3 control points are
 * created once; `update` only writes into those buffers and sets `needsUpdate`. No per-frame object
 * allocation in the hot path.
 */

/** control points used for each spline (including the current ship position) */
const MAX_CONTROL = 32
/** vertices in each rendered polyline */
const RES = 96

/**
 * Optional future landing-plan buffer attached to a recording:
 * `n` trace samples, each with `k` interleaved pad-ENU points (E,N,U; length `n*k*3`), matching the
 * recorded sample index. `has[i]` is 0 where the sample carries no live plan (entry, before the flip).
 * Unknown shapes are ignored and the recorded path is used.
 */
interface PlanBuffer {
  readonly n: number
  readonly k: number
  readonly has: ArrayLike<number> | null
  readonly xyz: ArrayLike<number>
}

const arrayLike = (v: unknown): v is ArrayLike<number> =>
  typeof v === "object" && v !== null && typeof (v as { length?: unknown }).length === "number"

/** read the optional `rec.pred` plan buffer without assuming it exists or has a fixed layout */
const readPlan = (rec: Recording, xyz: unknown): PlanBuffer | null => {
  const k: unknown = Reflect.get(rec, "predK")
  const has: unknown = Reflect.get(rec, "predHas")
  if (!arrayLike(xyz) || typeof k !== "number" || k < 2 || xyz.length < rec.n * k * 3) return null
  return { n: rec.n, k, has: arrayLike(has) ? has : null, xyz }
}

export interface PathOverlay {
  readonly group: THREE.Group
  /** hide/show the overlay (tree toggle) */
  readonly setVisible: (on: boolean) => void
  /** recompute both splines for the current sample and interpolation fraction */
  readonly update: (rec: Recording, i: number, f: number) => void
}

export const makePathOverlay = (): PathOverlay => {
  const group = new THREE.Group()
  group.name = "predicted-path"
  group.renderOrder = 8

  const makeLine = (color: number, opacity: number): { line: THREE.Line; pos: Float32Array; ctrl: THREE.Vector3[]; curve: THREE.CatmullRomCurve3 } => {
    const pos = new Float32Array(RES * 3)
    const geo = new THREE.BufferGeometry()
    const attr = new THREE.BufferAttribute(pos, 3)
    attr.setUsage(THREE.DynamicDrawUsage)
    geo.setAttribute("position", attr)
    geo.setDrawRange(0, RES)
    const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthWrite: false, depthTest: true })
    const line = new THREE.Line(geo, mat)
    line.frustumCulled = false
    line.renderOrder = 8
    line.visible = false
    const ctrl = Array.from({ length: MAX_CONTROL }, () => new THREE.Vector3())
    const curve = new THREE.CatmullRomCurve3(ctrl, false, "centripetal", 0.5)
    group.add(line)
    return { line, pos, ctrl, curve }
  }

  const actual = makeLine(0x4fe6ff, 0.9)
  // pale ice-white (not additive: additive white disappears over the studio/physical-sky white)
  const plan = makeLine(0xdcf4ff, 1.0)
  const tmp = new THREE.Vector3()
  let on = true
  let hasPlan = false
  let planRaw: unknown = null
  let planCache: PlanBuffer | null = null

  const writeCurve = (ln: { pos: Float32Array; curve: THREE.CatmullRomCurve3; line: THREE.Line }): void => {
    for (let s = 0; s < RES; s++) {
      ln.curve.getPoint(s / (RES - 1), tmp)
      const o = s * 3
      ln.pos[o] = tmp.x
      ln.pos[o + 1] = tmp.y
      ln.pos[o + 2] = tmp.z
    }
    const attr = ln.line.geometry.getAttribute("position") as THREE.BufferAttribute
    attr.needsUpdate = true
    ln.line.visible = on
  }

  const update = (rec: Recording, i: number, f: number): void => {
    const n = rec.n
    if (n < 2) {
      actual.line.visible = false
      plan.line.visible = false
      return
    }
    const pe = rec.data[CH.pe], pn = rec.data[CH.pn], pu = rec.data[CH.pu]
    const j = Math.min(n - 1, i + 1)
    const curE = pe[i] + (pe[j] - pe[i]) * f
    const curN = pn[i] + (pn[j] - pn[i]) * f
    const curU = pu[i] + (pu[j] - pu[i]) * f

    // ---- recorded future: the actual trajectory ahead, resampled to MAX_CONTROL points
    const span = n - 1 - i
    const lastK = i + Math.max(0, span)
    actual.ctrl[0].set(0, 0, 0)
    for (let c = 1; c < MAX_CONTROL; c++) {
      const k = Math.min(lastK, i + Math.max(1, Math.round((span * c) / (MAX_CONTROL - 1))))
      actual.ctrl[c].set(pe[k] - curE, pu[k] - curU, -(pn[k] - curN))
    }
    actual.line.visible = on && span >= 2
    if (actual.line.visible) writeCurve(actual)

    // ---- optional MPC plan: a brighter line from the same current position into the arms
    const predRaw: unknown = Reflect.get(rec, "pred")
    if (predRaw !== planRaw) { planRaw = predRaw; planCache = readPlan(rec, predRaw) }
    const pb = planCache
    hasPlan = false
    if (pb && pb.n > 0) {
      const s = Math.min(pb.n - 1, Math.max(0, i))
      const base = s * pb.k * 3
      if ((!pb.has || pb.has[s] !== 0) && pb.k >= 2) {
        plan.ctrl[0].set(0, 0, 0)
        for (let c = 1; c < MAX_CONTROL; c++) {
          const q = Math.min(pb.k - 1, Math.round((c * (pb.k - 1)) / (MAX_CONTROL - 1)))
          const o = base + q * 3
          plan.ctrl[c].set(pb.xyz[o] - curE, pb.xyz[o + 2] - curU, -(pb.xyz[o + 1] - curN))
        }
        hasPlan = true
      }
    }
    plan.line.visible = on && hasPlan
    if (plan.line.visible) writeCurve(plan)
  }

  return {
    group,
    setVisible: (v: boolean): void => {
      on = v
      group.visible = v
    },
    update,
  }
}
