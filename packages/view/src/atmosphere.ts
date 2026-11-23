/**
 * One atmosphere model for the whole viewer: the sky dome, the aerial perspective over the ground,
 * the colour of direct sunlight, the ambient sky light, the fog on distant solid objects and the
 * exposure all come from the same single-scattering integral, so they stay consistent with each
 * other at every Sun elevation and altitude.
 *
 * Model (Nishita et al. 1993; Bruneton & Neyret 2008; Hillaire 2020), lengths in km:
 *  - Rayleigh scattering β_R = (5.8, 13.5, 33.1)·10⁻³ km⁻¹ at sea level, scale height 8 km;
 *  - Mie (aerosol) scattering β_M = 21·10⁻³ km⁻¹, extinction 1.1 β_M, scale height 1.2 km,
 *    Cornette–Shanks phase with g = 0.76;
 *  - ozone absorption β_O = (0.650, 1.881, 0.085)·10⁻³ km⁻¹ at the layer peak, tent profile centred
 *    at 25 km with a 15 km half-width (no scattering: it only tints long slant paths, which is what
 *    keeps the twilight zenith blue);
 *  - spherical Earth R = 6371 km, atmosphere top at 6471 km, analytic Earth shadow on light paths.
 * Integration: 16 view samples × 6 light samples. Each view step is integrated analytically
 * (∫ S e^{−σs} ds = S (1 − e^{−σΔ})/σ, energy conserving for any step length) and the samples are
 * spaced quadratically from the camera when it is inside the atmosphere, so a 1000 km horizon path
 * still resolves the first few kilometres where most of the light is scattered.
 *
 * `ATMOSPHERE_GLSL` is the shader implementation; the functions below are the same code in
 * TypeScript (identical constants, sample positions and weights), used on the CPU for a handful of
 * rays per frame. Radiance is per unit solar irradiance until multiplied by `ATM.SUN_I`.
 */

export const ATM = {
  /** Earth and atmosphere-top radii, km */
  RE: 6371, RA: 6471,
  /** Rayleigh / Mie density scale heights, km */
  HR: 8, HM: 1.2,
  /** Rayleigh scattering coefficient at sea level, km⁻¹ (RGB ≈ 680 / 550 / 440 nm) */
  BR: [5.8e-3, 13.5e-3, 33.1e-3] as readonly number[],
  /** Mie scattering coefficient at sea level, km⁻¹, and extinction / scattering ratio */
  BM: 21e-3, BM_EXT: 1.1,
  /** Mie asymmetry */
  G: 0.76,
  /** ozone absorption at the layer peak, km⁻¹; layer centre and half-width, km */
  BO: [0.65e-3, 1.881e-3, 0.085e-3] as readonly number[], OZ_C: 25, OZ_W: 15,
  /** solar irradiance scale of the viewer's radiometry (scene-referred units) */
  SUN_I: 18,
  /** radiance clamp of the sky dome (keeps the horizon aureole below the bloom threshold) */
  SKY_MAX: 2.2,
  N_VIEW: 16, N_LIGHT: 6,
} as const

const f = (x: number): string => (Number.isInteger(x) ? x.toFixed(1) : String(x))
const v3 = (a: readonly number[]): string => `vec3(${a.map(f).join(", ")})`

/**
 * GLSL (ES 1.0 / 3.0 compatible) implementation. Needs `PI` (three's <common> chunk). Positions are
 * in km relative to the Earth centre, `sun` is the unit direction to the Sun.
 */
export const ATMOSPHERE_GLSL = /* glsl */ `
  const float ATM_RE = ${f(ATM.RE)}, ATM_RA = ${f(ATM.RA)}, ATM_HR = ${f(ATM.HR)}, ATM_HM = ${f(ATM.HM)};
  const vec3 ATM_BR = ${v3(ATM.BR)};
  const float ATM_BM = ${f(ATM.BM)}, ATM_BME = ${f(ATM.BM * ATM.BM_EXT)};
  const vec3 ATM_BO = ${v3(ATM.BO)};
  // ray / sphere (centre at the origin): entry and exit distances, (1e9, -1e9) on a miss
  vec2 atmRS(vec3 o, vec3 d, float R){
    float b = dot(o, d); float c = dot(o, o) - R * R; float h = b * b - c;
    if (h < 0.0) return vec2(1e9, -1e9);
    h = sqrt(h); return vec2(-b - h, -b + h);
  }
  // relative densities at height h (km): Rayleigh, Mie, ozone
  vec3 atmDensity(float h){
    return vec3(exp(-h / ATM_HR), exp(-h / ATM_HM), max(0.0, 1.0 - abs(h - ${f(ATM.OZ_C)}) / ${f(ATM.OZ_W)}));
  }
  // column densities (km) from p towards the Sun up to the top of the atmosphere
  vec3 atmSunDepth(vec3 p, vec3 sun){
    float len = atmRS(p, sun, ATM_RA).y;
    vec3 od = vec3(0.0);
    for (int j = 0; j < ${ATM.N_LIGHT}; j++) {
      float u0 = float(j) / ${f(ATM.N_LIGHT)}, u1 = float(j + 1) / ${f(ATM.N_LIGHT)};
      float ta = len * u0 * u0, tb = len * u1 * u1;
      vec3 q = p + sun * (0.5 * (ta + tb));
      od += atmDensity(max(length(q) - ATM_RE, 0.0)) * (tb - ta);
    }
    return od;
  }
  // single scattering along o + d t, t in [t0, t1]: radiance per unit solar irradiance, and the
  // transmittance of the segment. quad = 1: samples spaced quadratically from t0 (camera inside
  // the atmosphere), 0: uniform (camera in space, the dense air is in the middle of the chord)
  vec3 atmScatter(vec3 o, vec3 d, float t0, float t1, vec3 sun, float quad, out vec3 trans){
    float mu = dot(d, sun);
    float pR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    const float g = ${f(ATM.G)};
    float pM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) / ((2.0 + g * g) * pow(max(1.0 + g * g - 2.0 * g * mu, 1e-4), 1.5));
    vec3 L = vec3(0.0);
    trans = vec3(1.0);
    float len = t1 - t0;
    for (int i = 0; i < ${ATM.N_VIEW}; i++) {
      float u0 = float(i) / ${f(ATM.N_VIEW)}, u1 = float(i + 1) / ${f(ATM.N_VIEW)};
      float ta = len * mix(u0, u0 * u0, quad), tb = len * mix(u1, u1 * u1, quad);
      float ds = tb - ta;
      vec3 p = o + d * (t0 + 0.5 * (ta + tb));
      vec3 rho = atmDensity(max(length(p) - ATM_RE, 0.0));
      vec3 x = (ATM_BR * rho.x + ATM_BME * rho.y + ATM_BO * rho.z) * ds;
      vec3 tStep = exp(-x);
      vec2 le = atmRS(p, sun, ATM_RE);
      if (!(le.x > 0.0 && le.x < le.y)) { // not in the Earth's shadow
        vec3 od = atmSunDepth(p, sun);
        vec3 tSun = exp(-(ATM_BR * od.x + ATM_BME * od.y + ATM_BO * od.z));
        vec3 S = (ATM_BR * (rho.x * pR) + ATM_BM * (rho.y * pM)) * tSun;
        // ∫ e^{-σ s} ds over the step = Δ (1 − e^{−x})/x; series for small x (float precision)
        vec3 w = mix((1.0 - tStep) / max(x, vec3(1e-6)), 1.0 - 0.5 * x, step(x, vec3(1e-3)));
        L += trans * S * w * ds;
      }
      trans *= tStep;
    }
    return L;
  }
`

// ---------------------------------------------------------------------------------------------------
// CPU port (same samples and weights as the shader). Scratch storage: no allocation per call.

const _rs = new Float64Array(2)
const _od = new Float64Array(3)
const _L = new Float64Array(3)
const _T = new Float64Array(3)
const _a = new Float64Array(3)
const _b = new Float64Array(3)

/** ray / sphere about the origin: writes entry/exit distances into out, (1e9, −1e9) on a miss */
export const raySphere = (ox: number, oy: number, oz: number, dx: number, dy: number, dz: number, R: number, out: Float64Array): void => {
  const b = ox * dx + oy * dy + oz * dz, c = ox * ox + oy * oy + oz * oz - R * R
  const h = b * b - c
  if (h < 0) { out[0] = 1e9; out[1] = -1e9; return }
  const s = Math.sqrt(h)
  out[0] = -b - s; out[1] = -b + s
}

const rhoR = (h: number): number => Math.exp(-h / ATM.HR)
const rhoM = (h: number): number => Math.exp(-h / ATM.HM)
const rhoO = (h: number): number => Math.max(0, 1 - Math.abs(h - ATM.OZ_C) / ATM.OZ_W)

/** column densities (km) of Rayleigh / Mie / ozone from p towards the Sun, `n` quadratic samples */
export const sunDepth = (px: number, py: number, pz: number, sx: number, sy: number, sz: number, out: Float64Array, n: number = ATM.N_LIGHT): void => {
  raySphere(px, py, pz, sx, sy, sz, ATM.RA, _rs)
  const len = _rs[1]
  let r = 0, m = 0, o = 0
  for (let j = 0; j < n; j++) {
    const u0 = j / n, u1 = (j + 1) / n
    const ta = len * u0 * u0, tb = len * u1 * u1, t = 0.5 * (ta + tb), d = tb - ta
    const h = Math.max(Math.hypot(px + sx * t, py + sy * t, pz + sz * t) - ATM.RE, 0)
    r += rhoR(h) * d; m += rhoM(h) * d; o += rhoO(h) * d
  }
  out[0] = r; out[1] = m; out[2] = o
}

/**
 * Single scattering along o + d·t, t ∈ [t0, t1] (km, Earth-centred). Writes the in-scattered
 * radiance per unit solar irradiance into outL and the segment transmittance into outT (RGB).
 */
export const scatter = (
  ox: number, oy: number, oz: number, dx: number, dy: number, dz: number, t0: number, t1: number,
  sx: number, sy: number, sz: number, quad: boolean, outL: Float64Array, outT: Float64Array,
): void => {
  const mu = dx * sx + dy * sy + dz * sz
  const pR = (3 / (16 * Math.PI)) * (1 + mu * mu)
  const g = ATM.G
  const pM = ((3 / (8 * Math.PI)) * ((1 - g * g) * (1 + mu * mu))) / ((2 + g * g) * Math.pow(Math.max(1 + g * g - 2 * g * mu, 1e-4), 1.5))
  let l0 = 0, l1 = 0, l2 = 0, tr0 = 1, tr1 = 1, tr2 = 1
  const len = t1 - t0, N = ATM.N_VIEW, bme = ATM.BM * ATM.BM_EXT
  for (let i = 0; i < N; i++) {
    const u0 = i / N, u1 = (i + 1) / N
    const ta = len * (quad ? u0 * u0 : u0), tb = len * (quad ? u1 * u1 : u1), ds = tb - ta, t = t0 + 0.5 * (ta + tb)
    const px = ox + dx * t, py = oy + dy * t, pz = oz + dz * t
    const h = Math.max(Math.hypot(px, py, pz) - ATM.RE, 0)
    const r = rhoR(h), m = rhoM(h), o = rhoO(h)
    raySphere(px, py, pz, sx, sy, sz, ATM.RE, _rs)
    const lit = !(_rs[0] > 0 && _rs[0] < _rs[1])
    if (lit) sunDepth(px, py, pz, sx, sy, sz, _od)
    for (let c = 0; c < 3; c++) {
      const x = (ATM.BR[c] * r + bme * m + ATM.BO[c] * o) * ds
      const ts = Math.exp(-x)
      const tr = c === 0 ? tr0 : c === 1 ? tr1 : tr2
      if (lit) {
        const tSun = Math.exp(-(ATM.BR[c] * _od[0] + bme * _od[1] + ATM.BO[c] * _od[2]))
        const S = (ATM.BR[c] * r * pR + ATM.BM * m * pM) * tSun
        const w = x <= 1e-3 ? 1 - 0.5 * x : (1 - ts) / Math.max(x, 1e-6)
        const add = tr * S * w * ds
        if (c === 0) l0 += add; else if (c === 1) l1 += add; else l2 += add
      }
      if (c === 0) tr0 *= ts; else if (c === 1) tr1 *= ts; else tr2 *= ts
    }
  }
  outL[0] = l0; outL[1] = l1; outL[2] = l2
  outT[0] = tr0; outT[1] = tr1; outT[2] = tr2
}

/**
 * Sky radiance (scene units, ATM.SUN_I included, not clamped) seen from altitude `altKm` in the unit
 * direction d, local frame with +y up; `s` is the unit Sun direction in the same frame. Rays that
 * hit the ground return the in-scatter up to the surface (no surface term). Optional outT receives
 * the transmittance of the path.
 */
export const skyRadiance = (altKm: number, dx: number, dy: number, dz: number, sx: number, sy: number, sz: number, out: Float64Array, outT: Float64Array = _T): void => {
  const oy = ATM.RE + Math.max(altKm, 1e-3)
  raySphere(0, oy, 0, dx, dy, dz, ATM.RA, _rs)
  const a0 = _rs[0], a1 = _rs[1]
  if (a0 > a1 || a1 < 0) { out[0] = out[1] = out[2] = 0; outT[0] = outT[1] = outT[2] = 1; return }
  const t0 = Math.max(a0, 0)
  let t1 = a1
  raySphere(0, oy, 0, dx, dy, dz, ATM.RE, _rs)
  if (_rs[0] > 0 && _rs[0] < _rs[1]) t1 = Math.min(t1, _rs[0])
  if (t1 <= t0) { out[0] = out[1] = out[2] = 0; outT[0] = outT[1] = outT[2] = 1; return }
  scatter(0, oy, 0, dx, dy, dz, t0, t1, sx, sy, sz, a0 < 0, _L, outT)
  out[0] = _L[0] * ATM.SUN_I; out[1] = _L[1] * ATM.SUN_I; out[2] = _L[2] * ATM.SUN_I
}

/**
 * Transmittance (RGB) of direct sunlight arriving at altitude `altKm` from elevation asin(sinElev)
 * above the local horizontal; zero where the solid Earth blocks the ray. 24 samples.
 */
export const sunTransmittance = (altKm: number, sinElev: number, out: Float64Array): void => {
  const se = Math.max(-1, Math.min(1, sinElev)), ce = Math.sqrt(1 - se * se)
  const oy = ATM.RE + Math.max(altKm, 1e-3)
  raySphere(0, oy, 0, ce, se, 0, ATM.RE, _rs)
  if (_rs[0] > 0 && _rs[0] < _rs[1]) { out[0] = out[1] = out[2] = 0; return }
  if (oy >= ATM.RA && se >= 0) { out[0] = out[1] = out[2] = 1; return }
  sunDepth(0, oy, 0, ce, se, 0, _od, 24)
  const bme = ATM.BM * ATM.BM_EXT
  for (let c = 0; c < 3; c++) out[c] = Math.exp(-(ATM.BR[c] * _od[0] + bme * _od[1] + ATM.BO[c] * _od[2]))
}

/**
 * Cosine-weighted mean radiance of the upper hemisphere at altitude `altKm` (the sky's diffuse
 * irradiance is π × this), scene units. 4 zenith rings × 8 azimuths, equal weights (cos-weighted
 * stratification: sin²θ uniform). The Sun direction is (sx, sy, sz), +y up.
 */
export const skyAmbient = (altKm: number, sx: number, sy: number, sz: number, out: Float64Array): void => {
  let r = 0, g = 0, b = 0
  const NT = 4, NP = 8
  for (let i = 0; i < NT; i++) {
    const s2 = (i + 0.5) / NT, st = Math.sqrt(s2), ct = Math.sqrt(1 - s2)
    for (let j = 0; j < NP; j++) {
      const ph = ((j + 0.5) / NP) * Math.PI * 2
      skyRadiance(altKm, st * Math.cos(ph), ct, st * Math.sin(ph), sx, sy, sz, _a, _b)
      r += Math.min(_a[0], ATM.SKY_MAX); g += Math.min(_a[1], ATM.SKY_MAX); b += Math.min(_a[2], ATM.SKY_MAX)
    }
  }
  const k = 1 / (NT * NP)
  out[0] = r * k; out[1] = g * k; out[2] = b * k
}

/**
 * Aerial perspective of the straight segment camera → target (both km, Earth-centred): in-scattered
 * radiance (scene units, clamped like the sky) into outL, transmittance (RGB) into outT.
 */
export const aerialPerspective = (
  cx: number, cy: number, cz: number, tx: number, ty: number, tz: number,
  sx: number, sy: number, sz: number, outL: Float64Array, outT: Float64Array,
): void => {
  let dx = tx - cx, dy = ty - cy, dz = tz - cz
  const dist = Math.hypot(dx, dy, dz)
  outL[0] = outL[1] = outL[2] = 0; outT[0] = outT[1] = outT[2] = 1
  if (!(dist > 1e-6)) return
  dx /= dist; dy /= dist; dz /= dist
  raySphere(cx, cy, cz, dx, dy, dz, ATM.RA, _rs)
  if (_rs[0] > _rs[1] || _rs[1] < 0) return
  const t0 = Math.max(_rs[0], 0), t1 = Math.min(_rs[1], dist)
  if (t1 <= t0) return
  scatter(cx, cy, cz, dx, dy, dz, t0, t1, sx, sy, sz, _rs[0] < 0, _L, outT)
  for (let c = 0; c < 3; c++) outL[c] = Math.min(_L[c] * ATM.SUN_I, ATM.SKY_MAX)
}

/** Rec. 709 luminance of a linear RGB triple */
export const luminance = (c: ArrayLike<number>): number => 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]

/** Henyey–Greenstein phase function, sr⁻¹ (∫ over the sphere = 1) */
export const henyeyGreenstein = (mu: number, g: number): number => {
  const g2 = g * g
  return (1 - g2) / (4 * Math.PI * Math.pow(Math.max(1 + g2 - 2 * g * mu, 1e-4), 1.5))
}

const smooth = (a: number, b: number, x: number): number => {
  const t = Math.max(0, Math.min(1, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)
}

/** reference daylight for the exposure: Sun at 35° elevation over sea level */
const EXPOSURE_BASE = 0.6
const _e = new Float64Array(3)
const daylight = (sinElev: number): number => {
  sunTransmittance(0, Math.max(sinElev, 0), _e)
  // relative illuminance of a horizontal surface: direct beam + sky (≈ 12 % of the beam at mid
  // elevations, a twilight floor below the horizon)
  return Math.max(sinElev, 0) * luminance(_e) + 0.12 * smooth(-0.12, 0.25, sinElev) + 0.005
}
let dayRef = -1

/**
 * Tone-mapping exposure for a Sun at elevation asin(sinElev) seen from altitude `altM` (metres).
 * A camera's auto-exposure follows the scene illuminance; here half of the change (in stops) is
 * compensated (and at most +0.85 stop), so a low Sun still reads as evening: 0.54 at a high Sun,
 * 0.6 at 35°, 1.08 from a Sun ≈ 8° above the horizon down into the night. Above the weather (20 → 60 km) the vehicle is in direct, unattenuated sunlight whatever the
 * local elevation, and the exposure returns to the base value.
 */
export const exposureForSun = (sinElev: number, altM: number): number => {
  if (dayRef < 0) dayRef = daylight(Math.sin((35 * Math.PI) / 180))
  const s = Number.isFinite(sinElev) ? Math.max(-1, Math.min(1, sinElev)) : 1
  const boost = Math.max(0.9, Math.min(1.8, Math.sqrt(dayRef / daylight(s))))
  const w = 1 - smooth(20e3, 60e3, Number.isFinite(altM) ? altM : 0)
  return EXPOSURE_BASE * (1 + (boost - 1) * w)
}
