/** Frame helpers re-exported for the guidance layer. */
import type { V3 } from "@ship/math"
import { rotZ } from "@ship/earth"
export { corotation, ecefToGeodetic, eciToEcef, gravityJ2 } from "@ship/earth"
/** rotate an ECI-axes vector into ECEF axes (no transport term) */
export const ecefToEcef0 = (v: V3, theta: number): V3 => rotZ(v, -theta)
