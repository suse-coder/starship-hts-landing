/**
 * Fitted, texel-stable shadow frustum for the Sun (one orthographic shadow map).
 *
 * The world origin rides with the ship, so the ground and the tower slide through the world every
 * frame. A shadow camera that simply follows the ship therefore resamples every static edge each
 * frame and the shadows crawl. Two measures keep them still:
 *  - the frustum is a square of a quantised half-size (steps of √2) around the bounding sphere of
 *    what must be shadowed: the ship alone, or ship + tower once the ship is within reach of the pad;
 *  - its centre is snapped to whole shadow-map texels measured from the PAD origin along the light's
 *    own right/up axes, so a point fixed to the pad always falls on the same texel.
 * Pure math (no three.js): unit-tested, no allocation per call.
 */

export interface ShadowFitOptions {
  /** radius around the ship (world origin) that must be shadowed, m */
  readonly shipRadius: number
  /** tower sphere: centre height above the pad origin and radius, m */
  readonly towerHeight: number
  readonly towerRadius: number
  /** the tower joins the frustum when the ship is closer to it than this, m */
  readonly mergeDistance: number
  /** smallest / largest frustum half-size, m */
  readonly minHalf: number
  readonly maxHalf: number
  /** distance of the shadow camera from the frustum centre, m */
  readonly distance: number
  /** casters up to this far outside the frustum sphere towards the Sun still cast into it, m */
  readonly casterMargin: number
}

export const SHADOW_FIT: ShadowFitOptions = {
  shipRadius: 45, towerHeight: 72, towerRadius: 105, mergeDistance: 400, minHalf: 64, maxHalf: 362.04, distance: 1500, casterMargin: 320,
}

export interface ShadowFit {
  /** frustum centre (shadow camera target), world */
  cx: number; cy: number; cz: number
  /** shadow camera position = centre + sun × distance */
  px: number; py: number; pz: number
  /** orthographic half-size, m */
  half: number
  near: number
  far: number
  /** world size of one shadow-map texel, m */
  texel: number
}

export const makeShadowFit = (): ShadowFit => ({ cx: 0, cy: 0, cz: 0, px: 0, py: 1, pz: 0, half: 64, near: 1, far: 2, texel: 1 })

/** smallest half-size of the √2 ladder min·2^(k/2) that is ≥ r (clamped to max) */
export const quantiseHalf = (r: number, min: number, max: number): number => {
  if (!(r > min)) return min
  const k = Math.ceil(2 * Math.log2(r / min) - 1e-9)
  return Math.min(max, min * Math.pow(2, k / 2))
}

/**
 * Light-space axes for a directional light looking along −sun with world up (0, 1, 0), as
 * three.js `Matrix4.lookAt` builds them: right = normalize(up × sun), up' = sun × right. A Sun at
 * the zenith uses the +z fallback of lookAt's degenerate case.
 */
export const lightAxes = (sx: number, sy: number, sz: number, out: Float64Array): void => {
  // z = direction to the light; right = up × z = (0,1,0) × (zx,zy,zz) = (zz, 0, −zx)
  let zx = sx, zy = sy, zz = sz
  let rx = zz, rz = -zx
  let l = Math.hypot(rx, rz)
  if (l === 0) {
    // lookAt nudges the view direction when it is exactly parallel to up: z += 1e-4 (|up.z| ≠ 1 here)
    zz += 1e-4
    const zl = Math.hypot(zx, zy, zz)
    zx /= zl; zy /= zl; zz /= zl
    rx = zz; rz = -zx
    l = Math.hypot(rx, rz)
  }
  rx /= l; rz /= l
  // up' = z × right (right has no y component)
  out[0] = rx; out[1] = 0; out[2] = rz
  out[3] = zy * rz; out[4] = zz * rx - zx * rz; out[5] = -zy * rx
}

const _ax = new Float64Array(6)

/**
 * Fit the shadow frustum. `sun` = unit direction to the Sun (world), `pad` = world position of the
 * pad origin (the snapping reference), `padNear` = whether the tower is drawn at all, `mapSize` =
 * shadow-map width in texels.
 */
export const fitShadow = (
  out: ShadowFit, sx: number, sy: number, sz: number, padX: number, padY: number, padZ: number, padNear: boolean, mapSize: number,
  o: ShadowFitOptions = SHADOW_FIT,
): ShadowFit => {
  // bounding sphere of the ship sphere (origin) and, within reach, the tower sphere
  let cx = 0, cy = 0, cz = 0, r = o.shipRadius
  if (padNear) {
    const tx = padX, ty = padY + o.towerHeight, tz = padZ
    const d = Math.hypot(tx, ty, tz)
    if (d < o.mergeDistance) {
      if (d + o.shipRadius <= o.towerRadius) { cx = tx; cy = ty; cz = tz; r = o.towerRadius } else {
        r = 0.5 * (d + o.shipRadius + o.towerRadius)
        const k = (r - o.shipRadius) / d
        cx = tx * k; cy = ty * k; cz = tz * k
      }
    }
  }
  const half = quantiseHalf(r, o.minHalf, o.maxHalf)
  const texel = (2 * half) / Math.max(mapSize, 1)
  // snap the centre to the texel grid anchored at the pad origin
  lightAxes(sx, sy, sz, _ax)
  const dx = cx - padX, dy = cy - padY, dz = cz - padZ
  const u = dx * _ax[0] + dy * _ax[1] + dz * _ax[2], v = dx * _ax[3] + dy * _ax[4] + dz * _ax[5]
  const du = Math.round(u / texel) * texel - u, dv = Math.round(v / texel) * texel - v
  cx += _ax[0] * du + _ax[3] * dv; cy += _ax[1] * du + _ax[4] * dv; cz += _ax[2] * du + _ax[5] * dv
  out.cx = cx; out.cy = cy; out.cz = cz
  out.px = cx + sx * o.distance; out.py = cy + sy * o.distance; out.pz = cz + sz * o.distance
  out.half = half
  out.texel = texel
  // depth range: the frustum sphere (its square's half-diagonal) plus casters towards the Sun
  const reach = half * Math.SQRT2
  out.near = Math.max(1, o.distance - reach - o.casterMargin)
  out.far = o.distance + reach + 20
  return out
}
