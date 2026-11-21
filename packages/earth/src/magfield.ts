import { type V3, v3, dot, scale, sub, norm } from "@ship/math"
import { R_EQ } from "./constants"

/**
 * Geomagnetic field as the IGRF degree-1 (tilted dipole) term, epoch 2025:
 * g10 = -29350 nT, g11 = -1410 nT, h11 = 4545 nT (IGRF-14, rounded). Input/output in ECEF.
 */
const G = v3(-1410e-9, 4545e-9, -29350e-9)

export const geomagneticEcef = (r: V3): V3 => {
  const rn = norm(r)
  const rh = scale(r, 1 / rn)
  const k = (R_EQ / rn) ** 3
  return scale(sub(scale(rh, 3 * dot(G, rh)), G), k)
}
