/**
 * Dev harness for the global cloud shell + the local volumetric deck hand-off (cloudsGlobal.ts and
 * clouds.ts), with the same renderer settings as the app (log depth, ACES 0.6, bloom 0.35/0.3/2.5,
 * sRGB output) and the app's single-scattering atmosphere.
 *
 * Query: alt=<m> · e=<m> · n=<m> · yaw=<° from North> · pitch=<°> · sun=<elev°> · az=<°> ·
 *        cov=<0..1> · t=<s> · wind=<E,N> · old=1 (old cloud-painted day map, no global shell) ·
 *        noglobal=1 · nolocal=1 · still=1 · fov=<°>
 */
import * as THREE from "three"
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js"
import { RenderPass } from "three/addons/postprocessing/RenderPass.js"
import { UnrealBloomPass } from "three/addons/postprocessing/UnrealBloomPass.js"
import { OutputPass } from "three/addons/postprocessing/OutputPass.js"
import { makeAtmosphere, makeCoast, makeEarth } from "../../view/src/env"
import { makeClouds } from "../../view/src/clouds"
import { loadCloudDetail, loadCloudMask, makeGlobalClouds } from "../../view/src/cloudsGlobal"

const qs = new URLSearchParams(location.search)
const num = (k: string, d: number): number => (qs.has(k) ? Number(qs.get(k)) : d)
const alt = num("alt", 400e3)
const e = num("e", -120e3), n = num("n", -80e3)
const sunEl = THREE.MathUtils.degToRad(num("sun", 35))
const sunAz = THREE.MathUtils.degToRad(num("az", 120))
const yaw = THREE.MathUtils.degToRad(num("yaw", 120))
const pitch = THREE.MathUtils.degToRad(num("pitch", -35))
const simTime = num("t", 0)
const wind = (qs.get("wind") ?? "4,2").split(",").map(Number) as [number, number]
const still = qs.has("still")
const oldLook = qs.has("old")
const smoothstep = (a: number, b: number, x: number): number => { const t = Math.max(0, Math.min(1, (x - a) / (b - a))); return t * t * (3 - 2 * t) }
const dirFrom = (el: number, az: number): THREE.Vector3 => new THREE.Vector3(Math.sin(az) * Math.cos(el), Math.sin(el), -Math.cos(az) * Math.cos(el))
const sunDir = dirFrom(sunEl, sunAz)
const R_E = 6371e3
const localK = Math.pow(smoothstep(60e3, 15e3, alt), 3)
const localExtent = 60e3 + 3 * Math.max(0, Math.min(60e3, alt))

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

const padOrigin = new THREE.Vector3(-e, -alt, n)
const earthCentre = new THREE.Vector3(0, -R_E, 0).add(padOrigin)
const atm = makeAtmosphere(R_E, 1e-3, true)
atm.mesh.position.copy(earthCentre)
scene.add(atm.mesh)
atm.mesh.visible = !qs.has("noatm")
const sunColor = new THREE.Color(1, 0.96, 0.9).multiply(sunTint(sunEl)).multiplyScalar(3.0)
const skyColor = new THREE.Color(0.163, 0.274, 0.423).multiplyScalar(Math.max(0, Math.min(1, sunDir.y * 4 + 0.3)))
const sunLight = new THREE.DirectionalLight(new THREE.Color(1, 0.96, 0.9).multiply(sunTint(sunEl)), 3.0)
sunLight.position.copy(sunDir).multiplyScalar(800)
scene.add(sunLight, sunLight.target, new THREE.HemisphereLight(0xbcd7ff, 0x5a5040, 0.2))

const coast = makeCoast()
coast.root.position.copy(padOrigin)
scene.add(coast.root)
coast.water.uniforms.uSun.value.copy(sunDir)
const cloudTex = loadCloudMask("../textures/earth_clouds_4096.jpg")
const cloudDetail = loadCloudDetail("../textures/earth_clouds_detail_512.png")
const dayUrl = oldLook ? "../textures/earth_bluemarble_4096.jpg" : "../textures/earth_day_cloudfree_4096.jpg"
const earth = makeEarth(R_E, dayUrl, "../textures/earth_night_blackmarble2016_3840.jpg", 1e-3)
earth.earth.position.copy(earthCentre)
// orient the Earth so the pad site (Starbase: 26° N, 97° W by default) sits at the centre of the
// view: this is the region the ship descends over, not an arbitrary pole
const latR = THREE.MathUtils.degToRad(num("lat", 26)), lonR = THREE.MathUtils.degToRad(num("lon", -97))
const siteGeom = new THREE.Vector3(-Math.cos(latR) * Math.cos(lonR), Math.sin(latR), -Math.cos(latR) * Math.sin(lonR)).normalize()
earth.earth.quaternion.setFromUnitVectors(siteGeom, dirFrom(pitch, yaw).normalize().negate())
if (qs.has("rot")) earth.earth.rotation.set(num("rx", 0.3), num("rot", 1.9), 0)
earth.earth.visible = alt > 8000
if (qs.has("basic")) earth.earth.material = new THREE.MeshBasicMaterial({ map: (earth.earth.material as THREE.MeshStandardMaterial).map })
earth.earth.updateMatrix()
scene.add(earth.earth)

const globalShell = qs.has("noglobal") || oldLook ? null : makeGlobalClouds({ radius: R_E, height: 4200, texture: cloudTex, detailTexture: cloudDetail })
if (globalShell) {
  globalShell.mesh.position.copy(earthCentre)
  globalShell.mesh.quaternion.copy(earth.earth.quaternion)
  globalShell.mesh.updateMatrix()
  globalShell.mesh.matrixAutoUpdate = false
  scene.add(globalShell.mesh)
}
const deck = qs.has("nolocal") ? null : makeClouds({ coverage: num("cov", 0.35), maxExtent: 240e3 })
if (deck) scene.add(deck.mesh)

;(window as unknown as { __THREE: typeof THREE }).__THREE = THREE
;(window as unknown as { __global?: THREE.Mesh }).__global = globalShell?.mesh
;(window as unknown as { __getCloud: () => HTMLImageElement }).__getCloud = () => cloudTex.image as HTMLImageElement
;(window as unknown as { __renderer: THREE.WebGLRenderer }).__renderer = renderer
;(window as unknown as { __earth: THREE.Mesh; __scene: THREE.Scene; __camera: THREE.Camera }).__earth = earth.earth
;(window as unknown as { __scene: THREE.Scene }).__scene = scene
;(window as unknown as { __camera: THREE.Camera }).__camera = camera
const look = dirFrom(pitch, yaw)
camera.position.set(0, 0, 0)
camera.lookAt(look)

;(window as unknown as { __diag: () => unknown }).__diag = () => {
  const map = (earth.earth.material as THREE.MeshStandardMaterial).map
  return {
    earthMap: map ? { w: map.image?.width, h: map.image?.height, loaded: !!(map.image as HTMLImageElement)?.complete } : null,
    earthVisible: earth.earth.visible,
    earthCentre: earthCentre.toArray(), camDist: camera.position.distanceTo(earthCentre),
    cloudMap: { w: cloudTex.image?.width, h: cloudTex.image?.height, loaded: !!(cloudTex.image as HTMLImageElement)?.complete },
    globalVisible: globalShell?.mesh.visible ?? null,
    globalScale: globalShell ? globalShell.mesh.scale.toArray() : null,
    globalMatrix: globalShell ? globalShell.mesh.matrixWorld.elements.slice(12, 15) : null,
    atmVisible: atm.mesh.visible,
  }
}

const padDir = padOrigin.clone().sub(earthCentre).normalize()
const frame = (time: number): void => {
  atm.update(sunDir, earthCentre, camera.position, 1)
  earth.update(sunDir, earthCentre, camera, 1)
  // the app fades the local coast/sea planes out between 25 and 45 km; at altitude only the Earth
  // sphere and the cloud shell remain (otherwise a 300 km plane is a giant floating triangle)
  const coastK = 1 - Math.max(0, Math.min(1, (alt - 25e3) / 20e3))
  coast.root.visible = coastK > 0.002
  coast.setFade(coastK)
  globalShell?.update({
    sunDir, sunColor, skyColor, centre: earthCentre, camera: camera.position, time, wind, eclipse: 1,
    holeDir: padDir, holeRadius: localExtent / R_E, holeAmount: localK,
  })
  deck?.update({
    sunDir, sunColor, skyColor, origin: padOrigin, time, wind, eclipse: 1, extent: localExtent, fade: localK,
  })
  if (deck) deck.mesh.visible = localK > 0.005
  coast.water.uniforms.uTime.value = time
  composer.render()
}

const stats = { alt, localK, localExtent, sunEl: num("sun", 35), frameMs: 0, global: !!globalShell, old: oldLook }
const w = window as unknown as { __stats: typeof stats; __ready: boolean; __timing: (n: number) => number; __render: () => void }
w.__stats = stats
w.__render = (): void => { frame(simTime) }
const gl = renderer.getContext()
w.__timing = (n: number): number => {
  frame(simTime); gl.finish()
  const t0 = performance.now()
  for (let i = 0; i < n; i++) frame(simTime + i * 0.016)
  gl.finish()
  const ms = (performance.now() - t0) / n
  stats.frameMs = ms
  return ms
}
const fmt = (c: THREE.Color): string => `(${c.r.toFixed(3)}, ${c.g.toFixed(3)}, ${c.b.toFixed(3)})`
const drawHud = (): void => {
  hud.textContent = `alt=${(alt / 1000).toFixed(1)} km  localK=${localK.toFixed(2)} extent=${(localExtent / 1000).toFixed(0)} km  global=${!!globalShell}\nsun=${num("sun", 35)}°  frame ${stats.frameMs.toFixed(1)} ms  sky ${fmt(skyColor)}`
}
let count = 0
const t0 = performance.now()
const loop = (): void => {
  const time = simTime + (still ? 0 : (performance.now() - t0) / 1000)
  frame(time)
  stats.frameMs = 0
  drawHud()
  if (++count === 2) (w as unknown as { __ready: boolean }).__ready = true
  if (!still || count < 2) requestAnimationFrame(loop)
}
loop()
window.addEventListener("resize", () => {
  renderer.setSize(window.innerWidth, window.innerHeight, false)
  composer.setSize(window.innerWidth, window.innerHeight)
  camera.aspect = window.innerWidth / window.innerHeight
  camera.updateProjectionMatrix()
})
