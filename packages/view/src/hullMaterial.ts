import * as THREE from "three"

/**
 * Bare stainless-steel skin of the vehicle, patched into a MeshStandardMaterial (lights, shadows,
 * image-based lighting, fog and the logarithmic depth buffer stay three's own). The mesh must be a
 * body of revolution about its local +x axis (the body frame: x from the aft skirt plane to the nose).
 *
 * What the patch adds, all procedural in the body frame (no textures):
 *  - ring-to-ring variation: every rolled ring (`ringH` metres tall) comes from a different coil —
 *    ±2.5 % reflectance, a slight warm/cool cast, ±0.035 roughness;
 *  - weld heat-affected zones: a straw-coloured, rougher band ≈ 3 cm either side of each
 *    circumferential seam (the seam beads themselves are geometry, parts/structure.ts);
 *  - oil-canning: thin plate dishes between the welds. Surface slopes of ±2° decaying over 0.15 m
 *    from each seam (weld shrinkage) plus ±0.8° metre-scale dents perturb the shading normal, which
 *    is what makes the reflections of horizon and tower wobble on the real vehicle;
 *  - temper colours on the engine section: straw → blue oxide tint over the first 4 m above `aftX`;
 *  - soot streaks running aft from the heat-shield edge at `tpsX` (optional).
 * Normal detail fades out once a pixel covers more than ≈ 0.6 m of hull, and the weld bands widen
 * to a pixel at constant integrated strength (no aliasing or shimmer at range).
 *
 * Returns the uniform that switches the detail off (0) for false-colour analysis views.
 */
export interface SteelHullOptions {
  /** height of one rolled ring, m (circumferential welds at integer multiples along body x) */
  readonly ringH: number
  /** body x of the aft end of the heat-tinted engine section, m */
  readonly aftX: number
  /** body x of the heat-shield edge, m; omit for no soot streaks */
  readonly tpsX?: number
}

const PARS = /* glsl */ `
  varying vec3 vHullP;
  varying vec3 vHullAx;
  uniform float uHullDetail;
  float hullRing, hullSeamD, hullSeamSide, hullU, hullFade, hullHaz;
  // hash without sine (Hoskins): uniform in 0 … 1
  float hullHash(vec2 p){
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
  }
  // value noise, quintic fade, periodic in y with period py (lattice cells around the barrel)
  float hullNoise(vec2 q, float py){
    vec2 i = floor(q), f = fract(q);
    vec2 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float y0 = mod(i.y, py), y1 = mod(i.y + 1.0, py);
    return mix(mix(hullHash(vec2(i.x, y0)), hullHash(vec2(i.x + 1.0, y0)), u.x),
               mix(hullHash(vec2(i.x, y1)), hullHash(vec2(i.x + 1.0, y1)), u.x), u.y);
  }
`

const colour = (o: SteelHullOptions): string => /* glsl */ `
  {
    float xr = vHullP.x / ${o.ringH.toFixed(4)};
    hullRing = floor(xr);
    float rf = xr - hullRing;
    hullSeamD = min(rf, 1.0 - rf) * ${o.ringH.toFixed(4)};        // m to the nearest circumferential weld
    hullSeamSide = rf < 0.5 ? 1.0 : -1.0;
    hullU = (atan(vHullP.z, vHullP.y) + PI) * RECIPROCAL_PI2;     // 0 … 1 around the barrel
    float px = length(fwidth(vHullP));                            // m of hull per pixel
    hullFade = uHullDetail * (1.0 - smoothstep(0.15, 0.6, px));
    // heat-affected band beside the weld, pre-filtered: never narrower than a pixel, same integral
    float hazW = max(0.03, 0.8 * px);
    hullHaz = uHullDetail * exp(-hullSeamD / hazW) * (0.03 / hazW);
    float tone = hullHash(vec2(hullRing, 3.0)) - 0.5;
    vec3 tint = vec3(1.0 + 0.05 * tone) * mix(vec3(0.985, 1.0, 1.02), vec3(1.02, 1.0, 0.975), hullHash(vec2(hullRing, 11.0)));
    tint *= mix(vec3(1.0), vec3(0.90, 0.84, 0.74), 0.55 * hullHaz);
    float aft = 1.0 - smoothstep(${o.aftX.toFixed(3)}, ${(o.aftX + 4).toFixed(3)}, vHullP.x);
    float tn = hullNoise(vec2(vHullP.x * 0.7, hullU * 20.0), 20.0);
    tint *= mix(vec3(1.0), mix(vec3(0.86, 0.74, 0.56), vec3(0.62, 0.66, 0.84), smoothstep(0.35, 0.75, tn)), 0.30 * aft);
    ${o.tpsX === undefined ? "" : `
    float behind = ${o.tpsX.toFixed(3)} - vHullP.x;
    float soot = step(0.0, behind) * (1.0 - smoothstep(0.0, 3.5, behind));
    tint *= 1.0 - 0.22 * soot * hullNoise(vec2(vHullP.x * 0.25, hullU * 90.0), 90.0);`}
    diffuseColor.rgb *= mix(vec3(1.0), tint, uHullDetail);
  }`

const ROUGHNESS = /* glsl */ `
  roughnessFactor = clamp(roughnessFactor + uHullDetail * (0.07 * (hullHash(vec2(hullRing, 7.0)) - 0.5) + 0.10 * hullHaz), 0.08, 1.0);`

const NORMAL = /* glsl */ `
  if (hullFade > 0.0) {
    // slopes (rad) along the axis and around the circumference; 40 lattice cells around ≈ 0.7 m at r 4.5 m
    vec2 q = vec2(vHullP.x * 1.4 + hullRing * 13.7, hullU * 40.0);
    float sAx = hullSeamSide * 0.035 * exp(-hullSeamD / 0.15) + 0.028 * (hullNoise(q, 40.0) - 0.5);
    float sTan = 0.028 * (hullNoise(q + vec2(57.3, 0.0), 40.0) - 0.5);
    vec3 axV = normalize(vHullAx);
    vec3 tanV = normalize(cross(normal, axV) + vec3(1e-5));
    normal = normalize(normal + (axV * sAx + tanV * sTan) * hullFade);
  }`

export const applySteelHull = (mat: THREE.MeshStandardMaterial, o: SteelHullOptions): { value: number } => {
  const detail = { value: 1 }
  mat.onBeforeCompile = (sh) => {
    sh.uniforms.uHullDetail = detail
    sh.vertexShader = sh.vertexShader
      .replace("#include <common>", "#include <common>\nvarying vec3 vHullP;\nvarying vec3 vHullAx;")
      .replace("#include <begin_vertex>", "#include <begin_vertex>\nvHullP = position;\nvHullAx = mat3(modelViewMatrix) * vec3(1.0, 0.0, 0.0);")
    sh.fragmentShader = sh.fragmentShader
      .replace("#include <common>", `#include <common>\n${PARS}`)
      .replace("#include <color_fragment>", `#include <color_fragment>\n${colour(o)}`)
      .replace("#include <roughnessmap_fragment>", `#include <roughnessmap_fragment>\n${ROUGHNESS}`)
      .replace("#include <normal_fragment_maps>", `#include <normal_fragment_maps>\n${NORMAL}`)
  }
  mat.customProgramCacheKey = () => `steel-hull-v1:${o.ringH}:${o.aftX}:${o.tpsX ?? "-"}`
  return detail
}
