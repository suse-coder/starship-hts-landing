/**
 * Dev harness for the volumetric cloud deck (packages/view/src/clouds.ts). Same renderer settings as
 * the app (log depth, ACES 0.6, bloom 0.35/0.3/2.5, sRGB output), the app's single-scattering
 * atmosphere shell (env.ts makeAtmosphere) as the sky, Blue Marble Earth above 8 km, Starbase coast.
 *
 * Query: view=below|inside|above|high · sun=<elev°> · az=<Sun azimuth° from North> · yaw=<view azimuth°
 * relative to the Sun> · pitch=<°> · alt=<m> · cov=<0..1> · t=<s> · wind=<E,N m/s> · ship=1 · still=1
 * · nocloud=1 · fov=<°>
 * The "ship" (camera target) sits at the world origin as in the app; the pad origin moves.
 */
import * as THREE from "three"
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js"
import { RenderPass } from "three/addons/postprocessing/RenderPass.js"
import { UnrealBloomPass } from "three/addons/postprocessing/UnrealBloomPass.js"
import { OutputPass } from "three/addons/postprocessing/OutputPass.js"
import { makeAtmosphere, makeCoast, makeEarth } from "../../view/src/env"
import { makeClouds } from "../../view/src/clouds"

const qs = new URLSearchParams(location.search)
const num = (k: string, d: number): number => (qs.has(k) ? Number(qs.get(k)) : d)
const view = qs.get("view") ?? "below"
const presets: Record<string, { alt: number; e: number; n: number; yaw: number; pitch: number }> = {
  below: { alt: 300, e: -1500, n: -900, yaw: 115, pitch: 14 },
  inside: { alt: 2200, e: 2600, n: 1800, yaw: 95, pitch: 2 },
  above: { alt: 9000, e: -3000, n: -6000, yaw: 100, pitch: -30 },
  high: { alt: 25000, e: -8000, n: -20000, yaw: 125, pitch: -38 },
}
const pre = presets[view] ?? presets.below
const sunEl = THREE.MathUtils.degToRad(num("sun", 35))
const sunAz = THREE.MathUtils.degToRad(num("az", 120))
const yaw = sunAz + THREE.MathUtils.degToRad(num("yaw", pre.yaw))
const pitch = THREE.MathUtils.degToRad(num("pitch", pre.pitch))
const alt = num("alt", pre.alt)
const simTime = num("t", 0)
const wind = (qs.get("wind") ?? "4,2").split(",").map(Number) as [number, number]
const still = qs.has("still")

/** three.js direction (x East, y Up, z −North) from elevation / azimuth (from North, clockwise) */
const dirFrom = (el: number, az: number): THREE.Vector3 => new THREE.Vector3(Math.sin(az) * Math.cos(el), Math.sin(el), -Math.cos(az) * Math.cos(el))
const sunDir = dirFrom(sunEl, sunAz)

/**
 * Direct-Sun colour at the deck relative to a zenith Sun: Kasten–Young air mass, Rayleigh + aerosol
 * vertical optical depths τ(R,G,B) = (0.11, 0.17, 0.33) for 680/550/450 nm (aerosol τ ≈ 0.06).
 */
const sunTint = (el: number): THREE.Color => {
  const deg = THREE.MathUtils.radToDeg(el)
  if (deg < -1) return new THREE.Color(0, 0, 0)
  const m = 1 / (Math.sin(Math.max(el, 0)) + 0.50572 * Math.pow(Math.max(deg, 0) + 6.07995, -1.6364))
  const tau = [0.11, 0.17, 0.33]
  const k = tau.map((t) => Math.exp(-t * (m - 1)))
  const horizon = THREE.MathUtils.smoothstep(deg, -1, 1)
  return new THREE.Color(k[0] * horizon, k[1] * horizon, k[2] * horizon)
}

const canvas = document.getElementById("c") as HTMLCanvasElement
const hud = document.getElementById("hud") as HTMLDivElement
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, logarithmicDepthBuffer: true })
renderer.setPixelRatio(1)
renderer.setSize(window.innerWidth, window.innerHeight, false)
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 0.6
renderer.outputColorSpace = THREE.SRGBColorSpace
const scene = new THREE.Scene()
scene.background = new THREE.Color(0)
const camera = new THREE.PerspectiveCamera(num("fov", 45), window.innerWidth / window.innerHeight, 0.5, 5e8)
const composer = new EffectComposer(renderer)
composer.addPass(new RenderPass(scene, camera))
composer.addPass(new UnrealBloomPass(new THREE.Vector2(512, 512), 0.35, 0.3, 2.5))
composer.addPass(new OutputPass())

// ---- sky (as in the app: atmosphere shell around the Earth centre, ship at the world origin) and the
// calibration of the ambient sky colour (cos-weighted mean radiance of the upper sky at sea level)
const R_E = 6371e3
const atm = makeAtmosphere(R_E, 1e-3, true)
scene.add(atm.mesh)
const measureSky = (dir: THREE.Vector3): { mean: THREE.Color; horizon: THREE.Color } => {
  const a = makeAtmosphere(R_E, 1e-3, false)
  a.mesh.position.set(0, -R_E - 2, 0)
  a.update(dir, a.mesh.position, new THREE.Vector3(), 1)
  const sc = new THREE.Scene()
  sc.add(a.mesh)
  const S = 32
  const rt = new THREE.WebGLCubeRenderTarget(S, { type: THREE.FloatType })
  const cc = new THREE.CubeCamera(1, 2e7, rt)
  cc.update(renderer, sc)
  const buf = new Float32Array(S * S * 4)
  const mean = [0, 0, 0], hor = [0, 0, 0]
  let wSum = 0, hSum = 0
  for (let face = 0; face < 6; face++) {
    if (face === 3) continue // −y
    renderer.readRenderTargetPixels(rt, 0, 0, S, S, buf, face)
    for (let j = 0; j < S; j++) for (let i = 0; i < S; i++) {
      const nx = ((i + 0.5) / S) * 2 - 1, ny = ((j + 0.5) / S) * 2 - 1
      const len = Math.hypot(nx, ny, 1)
      const y = face === 2 ? 1 / len : -ny / len // side faces: camera up = −y world
      if (y <= 0) continue
      const w = (y / (len * len * len))
      const o = (j * S + i) * 4
      for (let c = 0; c < 3; c++) mean[c] += buf[o + c] * w
      wSum += w
      if (face !== 2 && y < 0.12) { for (let c = 0; c < 3; c++) hor[c] += buf[o + c]; hSum++ }
    }
  }
  rt.dispose()
  return { mean: new THREE.Color(mean[0] / wSum, mean[1] / wSum, mean[2] / wSum), horizon: new THREE.Color(hor[0] / hSum, hor[1] / hSum, hor[2] / hSum) }
}
const skyM = measureSky(sunDir)

// ---- lights + environment (as in the app)
const tint = sunTint(sunEl)
const sunI = 3.0
const sunLight = new THREE.DirectionalLight(new THREE.Color(1, 0.96, 0.9).multiply(tint), sunI)
sunLight.position.copy(sunDir).multiplyScalar(800)
const hemi = new THREE.HemisphereLight(0xbcd7ff, 0x5a5040, 0.2)
scene.add(sunLight, sunLight.target, hemi)
const pmrem = new THREE.PMREMGenerator(renderer)
{
  const envScene = new THREE.Scene()
  const ea = makeAtmosphere(R_E, 1e-3, false)
  ea.mesh.position.set(0, -R_E - Math.max(alt, 2), 0)
  ea.update(sunDir, ea.mesh.position, new THREE.Vector3(), 1)
  envScene.add(ea.mesh, new THREE.Mesh(new THREE.SphereGeometry(5e4, 32, 12, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2), new THREE.MeshBasicMaterial({ color: new THREE.Color(0.2, 0.19, 0.14), side: THREE.BackSide })))
  scene.environment = pmrem.fromScene(envScene, 0, 0.1, 2e7).texture
}

// ---- pad frame: coast, far ocean, clouds
const origin = new THREE.Vector3(-pre.e, -alt, pre.n) // pad origin relative to the ship (world origin)
const coast = makeCoast()
coast.root.position.copy(origin)
scene.add(coast.root)
coast.water.uniforms.uSun.value.copy(sunDir)
coast.water.uniforms.uSky.value.copy(skyM.horizon)
// Earth sphere (no depth inside the atmosphere, as in the app), shown above 8 km like the app
const earthCentre = new THREE.Vector3(0, -R_E, 0).add(origin)
const earth = makeEarth(R_E, "../textures/earth_bluemarble_4096.jpg", "../textures/earth_night_blackmarble2016_3840.jpg", 1e-3)
earth.earth.position.copy(earthCentre)
earth.earth.rotation.set(0.3, 1.9, 0) // any orientation: ocean/land texture under the deck
earth.earth.visible = alt > 8000
atm.mesh.position.copy(earthCentre)
scene.add(earth.earth)
if (qs.has("ship")) {
  const ship = new THREE.Mesh(new THREE.CylinderGeometry(4.5, 4.5, 50, 48), new THREE.MeshStandardMaterial({ color: 0xc8ccd0, metalness: 1, roughness: 0.3 }))
  scene.add(ship)
}

const tb0 = performance.now()
const deck = makeClouds({ coverage: num("cov", 0.35) })
const bakeMs = performance.now() - tb0
if (!qs.has("nocloud")) scene.add(deck.mesh)

// camera: at the ship (or 150 m behind it with ship=1), looking along yaw/pitch
const look = dirFrom(pitch, yaw)
camera.position.copy(qs.has("ship") ? look.clone().multiplyScalar(-150) : new THREE.Vector3())
camera.lookAt(camera.position.clone().add(look))

const skyFade = Math.exp(-Math.max(alt, 0) / 8000)
const frame = (time: number): void => {
  atm.update(sunDir, earthCentre, camera.position, 1)
  earth.update(sunDir, earthCentre, camera, 1)
  deck.update({
    sunDir, sunColor: new THREE.Color(1, 0.96, 0.9).multiply(tint).multiplyScalar(sunI), skyColor: skyM.mean,
    origin, time, wind, eclipse: 1,
  })
  coast.water.uniforms.uTime.value = time
  composer.render()
}

const fmt = (c: THREE.Color): string => `(${c.r.toFixed(3)}, ${c.g.toFixed(3)}, ${c.b.toFixed(3)})`
const stats = { view, alt, sunEl: num("sun", 35), bakeMs, skyMean: skyM.mean.toArray(), skyHorizon: skyM.horizon.toArray(), skyFade, frameMs: 0, frames: 0, sunTransmittance: 1 }
const w = window as unknown as { __stats: typeof stats; __ready: boolean; __timing: (n: number) => number }
w.__stats = stats
const gl = renderer.getContext()
w.__timing = (n: number): number => {
  frame(simTime)
  gl.finish()
  const t0 = performance.now()
  for (let i = 0; i < n; i++) frame(simTime + i * 0.016)
  gl.finish()
  const ms = (performance.now() - t0) / n
  stats.frameMs = ms
  return ms
}
/** HDR radiance statistics of the cloud pixels (linear, before tone mapping): luminance percentiles */
;(w as unknown as { __hdr: () => unknown }).__hdr = () => {
  const W = 320, H = Math.round((320 * window.innerHeight) / window.innerWidth)
  const rt = new THREE.WebGLRenderTarget(W, H, { type: THREE.FloatType })
  const px = (withClouds: boolean): Float32Array => {
    deck.mesh.visible = withClouds
    renderer.setRenderTarget(rt)
    renderer.render(scene, camera)
    renderer.setRenderTarget(null)
    const b = new Float32Array(W * H * 4)
    renderer.readRenderTargetPixels(rt, 0, 0, W, H, b)
    return b
  }
  const a = px(false), b = px(true)
  deck.mesh.visible = true
  const lc: number[] = [], ls: number[] = []
  for (let i = 0; i < W * H; i++) {
    const o = i * 4
    const la = 0.2126 * a[o] + 0.7152 * a[o + 1] + 0.0722 * a[o + 2], lb = 0.2126 * b[o] + 0.7152 * b[o + 1] + 0.0722 * b[o + 2]
    if (Math.abs(la - lb) > 0.02) lc.push(lb); else ls.push(lb)
  }
  lc.sort((x, y) => x - y); ls.sort((x, y) => x - y)
  const pc = (v: number[], q: number): number => (v.length ? v[Math.min(v.length - 1, Math.floor(q * v.length))] : NaN)
  rt.dispose()
  return { cloudFrac: lc.length / (W * H), cloud: [0.05, 0.25, 0.5, 0.75, 0.95, 0.995].map((q) => +pc(lc, q).toFixed(3)), rest: [0.05, 0.5, 0.95].map((q) => +pc(ls, q).toFixed(3)) }
}
const drawHud = (): void => {
  hud.textContent = `view=${view} alt=${alt} m sun=${num("sun", 35)}° cov=${num("cov", 0.35)}\nbake ${bakeMs.toFixed(0)} ms  frame ${stats.frameMs.toFixed(1)} ms\nsky ${fmt(skyM.mean)} horizon ${fmt(skyM.horizon)}  T_sun(ship) ${stats.sunTransmittance.toFixed(2)}`
}
let n = 0
const t0 = performance.now()
const loop = (): void => {
  const time = simTime + (still ? 0 : (performance.now() - t0) / 1000)
  frame(time)
  stats.frames = ++n
  stats.sunTransmittance = deck.mesh.userData.sunTransmittance as number
  drawHud()
  if (n === 2) w.__ready = true
  if (!still || n < 2) requestAnimationFrame(loop)
}
loop()
window.addEventListener("resize", () => {
  renderer.setSize(window.innerWidth, window.innerHeight, false)
  composer.setSize(window.innerWidth, window.innerHeight)
  camera.aspect = window.innerWidth / window.innerHeight
  camera.updateProjectionMatrix()
})
