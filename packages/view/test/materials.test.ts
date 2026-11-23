/**
 * Shader plumbing of the environment and hull materials, without a GPU: every patch into three.js'
 * standard material must still find its anchor chunk (a three upgrade that renames a chunk would
 * otherwise silently drop the effect), the hand-written shaders must be structurally sound, and the
 * per-frame setters must leave finite uniforms. Whether the shaders compile on a GPU and what they
 * look like is NOT covered here.
 */
import { afterAll, beforeAll, describe, expect, test } from "bun:test"
import * as THREE from "three"
import { ATM } from "../src/atmosphere"
import { makeGlobalClouds } from "../src/cloudsGlobal"
import { makeAtmosphere, makeCoast, makeEarth, makeStars, type Coast, type EarthModel } from "../src/env"
import { applySteelHull } from "../src/hullMaterial"

/** canvas / image stand-ins: the coast paints one detail texture on a 2-D canvas, the Earth loads images */
const fakeContext: unknown = new Proxy({}, { get: () => () => {}, set: () => true })
const fakeElement = (): unknown => ({ width: 0, height: 0, style: {}, getContext: () => fakeContext, addEventListener: () => {}, removeEventListener: () => {} })
const host = globalThis as { document?: unknown }
const hadDocument = "document" in host
const realDocument = host.document

let coast: Coast
let earth: EarthModel
beforeAll(() => {
  host.document = { createElement: fakeElement, createElementNS: fakeElement }
  coast = makeCoast()
  earth = makeEarth(6371e3, "day.jpg", "night.jpg", 1e-3)
})
afterAll(() => {
  if (hadDocument) host.document = realDocument
  else delete host.document
})

interface Stage { uniforms: Record<string, { value: unknown }>; vertexShader: string; fragmentShader: string }
/** run a material's onBeforeCompile against three's stock standard shader */
const compileStandard = (m: THREE.Material): Stage => {
  const sh: Stage = { uniforms: {}, vertexShader: THREE.ShaderLib.standard.vertexShader, fragmentShader: THREE.ShaderLib.standard.fragmentShader }
  m.onBeforeCompile(sh as unknown as THREE.WebGLProgramParametersWithUniforms, null as unknown as THREE.WebGLRenderer)
  return sh
}
const balanced = (src: string): void => {
  for (const [a, b] of [["{", "}"], ["(", ")"], ["[", "]"]]) expect(src.split(a).length).toBe(src.split(b).length)
}
const standardMats = (root: THREE.Object3D): THREE.MeshStandardMaterial[] => {
  const out: THREE.MeshStandardMaterial[] = []
  root.traverse((o) => { const m = (o as THREE.Mesh).material as THREE.Material | undefined; if (m && (m as THREE.MeshStandardMaterial).isMeshStandardMaterial) out.push(m as THREE.MeshStandardMaterial) })
  return out
}
const finite = (v: unknown): boolean => {
  if (typeof v === "number") return Number.isFinite(v)
  if (Array.isArray(v)) return v.every(finite)
  const o = v as { isColor?: boolean; isVector2?: boolean; isVector3?: boolean; r?: number; g?: number; b?: number; x?: number; y?: number; z?: number }
  if (o?.isColor) return finite(o.r) && finite(o.g) && finite(o.b)
  if (o?.isVector3) return finite(o.x) && finite(o.y) && finite(o.z)
  if (o?.isVector2) return finite(o.x) && finite(o.y)
  return true
}

describe("stainless hull material", () => {
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.96, roughness: 0.3 })
  const detail = applySteelHull(mat, { ringH: 1.83, aftX: 5.5, tpsX: 36 })
  const sh = compileStandard(mat)
  test("all four hooks land in the standard shader", () => {
    expect(sh.vertexShader).toContain("vHullP = position;")
    expect(sh.vertexShader).toContain("varying vec3 vHullAx;")
    expect(sh.fragmentShader).toContain("float hullNoise(vec2 q, float py)")
    expect(sh.fragmentShader).toContain("diffuseColor.rgb *= mix(vec3(1.0), tint, uHullDetail);")
    expect(sh.fragmentShader).toContain("roughnessFactor = clamp(roughnessFactor + uHullDetail")
    expect(sh.fragmentShader).toContain("normal = normalize(normal + (axV * sAx + tanV * sTan) * hullFade);")
    // colour hook before roughness before normal: the shared ring/seam variables are set before use
    const f = sh.fragmentShader
    expect(f.indexOf("hullRing = floor(xr);")).toBeLessThan(f.indexOf("roughnessFactor = clamp(roughnessFactor + uHullDetail"))
    expect(f.indexOf("roughnessFactor = clamp(roughnessFactor + uHullDetail")).toBeLessThan(f.indexOf("normal = normalize(normal + (axV"))
    balanced(sh.fragmentShader)
  })
  test("ring height, aft station and heat-shield edge are baked in; the detail switch is a live uniform", () => {
    expect(sh.fragmentShader).toContain("vHullP.x / 1.8300")
    expect(sh.fragmentShader).toContain("smoothstep(5.500, 9.500, vHullP.x)")
    expect(sh.fragmentShader).toContain("36.000 - vHullP.x")
    expect(sh.uniforms.uHullDetail).toBe(detail)
    expect(detail.value).toBe(1)
  })
  test("no soot term without a heat-shield edge, and a distinct program per option set", () => {
    const m2 = new THREE.MeshStandardMaterial()
    applySteelHull(m2, { ringH: 1.83, aftX: 3.7 })
    expect(compileStandard(m2).fragmentShader).not.toContain("float soot")
    expect(m2.customProgramCacheKey()).not.toBe(mat.customProgramCacheKey())
  })
})

describe("coast", () => {
  test("ground, beach and pad: procedural cover, wetness → roughness, radial fade; no depth write, no scene fog", () => {
    const mats = standardMats(coast.root)
    expect(mats.length).toBe(3)
    const keys = new Set<string>()
    for (const m of mats) {
      const sh = compileStandard(m)
      expect(sh.vertexShader).toContain("vCoastW = (modelMatrix * vec4(transformed, 1.0)).xyz;")
      expect(sh.fragmentShader).toContain("uniform sampler2D uTerr;")
      expect(sh.fragmentShader).toContain("coastWet =")
      expect(sh.fragmentShader).toContain("roughnessFactor = mix(roughnessFactor, 0.3, coastWet);")
      expect(sh.fragmentShader).toContain("gl_FragColor.a *= 1.0 - smoothstep(uCoastFade.x, uCoastFade.y")
      expect(sh.uniforms.uTerr.value).toBeInstanceOf(THREE.DataTexture)
      balanced(sh.fragmentShader)
      expect(m.depthWrite).toBe(false)
      expect(m.transparent).toBe(true)
      expect(m.fog).toBe(false)
      keys.add(m.customProgramCacheKey())
    }
    expect(keys.size).toBe(3)
    expect(mats.some((m) => compileStandard(m).fragmentShader.includes("float elev = "))).toBe(true)
  })
  test("planes are painted in a fixed order before the cloud shell and the atmosphere", () => {
    const orders = coast.root.children.map((c) => c.renderOrder)
    expect(new Set(orders).size).toBe(orders.length)
    for (const o of orders) expect(o).toBeLessThanOrEqual(-6)
  })
  test("water shader: structure, the mip-variance table and finite uniforms after setLight / setFade", () => {
    const w = coast.water
    expect(w.fragmentShader).toContain("vec3 waveLayer(vec2 p, vec2 dir, float tile, float off, float w2)")
    expect(w.fragmentShader).toContain("uniform float uRet[9];")
    expect(w.depthWrite).toBe(false)
    balanced(w.fragmentShader)
    balanced(w.vertexShader)
    const ret = w.uniforms.uRet.value as number[]
    expect(ret.length).toBe(9)
    expect(ret[0]).toBe(1)
    coast.root.position.set(120, -900, 40)
    coast.setFade(0.5)
    coast.setLight({
      sunDir: new THREE.Vector3(0.6, 0.7, 0.39).normalize(), sunColor: new THREE.Color(3.9, 3.4, 2.7), skyZenith: new THREE.Color(0.08, 0.14, 0.27),
      skyHorizon: new THREE.Color(0.4, 0.55, 0.58), skySun: new THREE.Color(0.6, 0.6, 0.5), skyAmbient: new THREE.Color(0.18, 0.27, 0.44), time: 12345.678,
    })
    for (const [k, u] of Object.entries(w.uniforms)) expect([k, finite(u.value)]).toEqual([k, true])
    expect(w.uniforms.uOpacity.value).toBe(0.5)
    const off = w.uniforms.uWaveOff.value as THREE.Vector3
    for (const c of [off.x, off.y, off.z, w.uniforms.uSurf.value as number]) { expect(c).toBeGreaterThanOrEqual(0); expect(c).toBeLessThan(1) }
    // irradiance = direct beam on the horizontal + π × mean sky radiance
    expect((w.uniforms.uIrr.value as THREE.Color).r).toBeCloseTo(3.9 * 0.7 / Math.hypot(0.6, 0.7, 0.39) + Math.PI * 0.18, 5)
    expect((w.uniforms.uCoastPad.value as THREE.Vector2).x).toBe(120)
  })
})

describe("atmosphere shell", () => {
  const a = makeAtmosphere(6371e3, 1e-3, true)
  const m = a.mesh.material as THREE.ShaderMaterial
  test("shader carries the shared model; blended as extinction + in-scatter (premultiplied over)", () => {
    expect(m.fragmentShader).toContain("vec3 atmScatter(")
    expect(m.fragmentShader).toContain("float alpha = 1.0 - dot(trans")
    balanced(m.fragmentShader)
    expect(m.blending).toBe(THREE.CustomBlending)
    expect(m.blendSrc).toBe(THREE.OneFactor)
    expect(m.blendDst).toBe(THREE.OneMinusSrcAlphaFactor)
    expect(m.depthWrite).toBe(false)
    expect(m.uniforms.uSunI.value).toBe(ATM.SUN_I)
  })
  test("drawn after the cloud shell and the cloud shadows, before the local deck", () => {
    expect(a.mesh.renderOrder).toBeGreaterThan(-2)
    expect(a.mesh.renderOrder).toBeLessThan(-1)
  })
  test("back faces from inside the shell, front faces from space", () => {
    const sun = new THREE.Vector3(0, 1, 0), c = new THREE.Vector3(0, -6371e3, 0)
    a.update(sun, c, new THREE.Vector3(0, 100, 0), 1)
    expect(m.side).toBe(THREE.BackSide)
    a.update(sun, c, new THREE.Vector3(0, 400e3, 0), 1)
    expect(m.side).toBe(THREE.FrontSide)
  })
})

describe("earth, stars, cloud shell", () => {
  test("earth: ocean-glint and city-light hooks land; no scene fog on the planet", () => {
    const m = earth.earth.material as THREE.MeshStandardMaterial
    const sh = compileStandard(m)
    expect(sh.fragmentShader).toContain("earthOcean = smoothstep(")
    expect(sh.fragmentShader).toContain("roughnessFactor = mix(roughnessFactor, 0.48, earthOcean);")
    expect(sh.fragmentShader).toContain("totalEmissiveRadiance *= 1.0 - smoothstep(-0.14, 0.06, dot(normal, uSunView));")
    balanced(sh.fragmentShader)
    expect(m.fog).toBe(false)
  })
  test("the atmosphere can be integrated about a different centre than the globe (pad-tangent sphere)", () => {
    const cam = new THREE.PerspectiveCamera()
    cam.position.set(0, 50, 0)
    cam.updateMatrixWorld()
    const atm = earth.atmosphere.material as THREE.ShaderMaterial
    earth.update(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, -6374e3, 0), cam, 1, new THREE.Vector3(10, -6371e3, 5))
    expect((atm.uniforms.uCenter.value as THREE.Vector3).y).toBe(-6371e3)
    earth.update(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, -6374e3, 0), cam, 1)
    expect((atm.uniforms.uCenter.value as THREE.Vector3).y).toBe(-6374e3)
  })
  test("stars ignore the scene fog", () => {
    expect((makeStars().material as THREE.PointsMaterial).fog).toBe(false)
  })
  test("global cloud shell: relief tap and solid-Earth occlusion, Earth radius in the shell's units", () => {
    const g = makeGlobalClouds({ radius: 6371, height: 4.2, texture: new THREE.Texture(), detailTexture: new THREE.Texture() })
    const m = g.mesh.material as THREE.ShaderMaterial
    expect(m.fragmentShader).toContain("float relief = clamp(")
    expect(m.fragmentShader).toContain("if (tE > 0.0 && tE < distance(vW, cameraPosition)) discard;")
    expect(m.vertexShader).toContain("vSunL = normalize(transpose(mat3(modelMatrix)) * uSun);")
    balanced(m.fragmentShader)
    expect(m.uniforms.uEarthR.value).toBe(6371)
    expect(g.shellRadius).toBeCloseTo(6375.2, 9)
    g.update({ sunDir: new THREE.Vector3(1, 0, 0), sunColor: new THREE.Color(3, 3, 3), skyColor: new THREE.Color(0.2, 0.3, 0.4), centre: new THREE.Vector3(), camera: new THREE.Vector3(0, 9000, 0), time: 10, wind: [5, 2], eclipse: 1 })
    for (const [k, u] of Object.entries(m.uniforms)) expect([k, finite(u.value)]).toEqual([k, true])
  })
})
