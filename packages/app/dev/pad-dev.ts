/**
 * Dev harness for the Pad 2 / Mechazilla model (packages/view/src/tower.ts).
 * URL: /dev/pad.html?view=wide|arms|base|ship [&open=0..1] [&env=room|sky] [&az=225&el=35]
 * Renderer settings follow packages/view/src/index.ts (ACES 0.6, sRGB, PCF soft shadows, log depth, bloom).
 */
import * as THREE from "three"
import { OrbitControls } from "three/addons/controls/OrbitControls.js"
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js"
import { RenderPass } from "three/addons/postprocessing/RenderPass.js"
import { UnrealBloomPass } from "three/addons/postprocessing/UnrealBloomPass.js"
import { OutputPass } from "three/addons/postprocessing/OutputPass.js"
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js"
import { TOWER } from "@ship/tower"
import { makePad, enu } from "../../view/src/tower"
import { makeCoast, makeSky } from "../../view/src/env"

interface Preset {
  readonly cam: THREE.Vector3
  readonly target: THREE.Vector3
  readonly fov: number
  readonly open: number
  readonly ship: boolean
  /** shadow frustum centre and half-size (m) */
  readonly shadowAt: THREE.Vector3
  readonly shadowR: number
}

/** ship placeholder: Ø 9 m × 52 m, pins (nosecone base, 38.4 m above the aft end) resting on the rails */
const SHIP = { r: 4.5, len: 52, pinX: 21 * 1.83 } as const

const PRESETS: Readonly<Record<string, Preset>> = {
  wide: { cam: enu(-150, -370, 55), target: enu(-25, 0, 58), fov: 36, open: 1, ship: false, shadowAt: enu(-60, 0, 0), shadowR: 240 },
  arms: { cam: enu(-30, -34, 122), target: enu(0, 3, 109), fov: 50, open: 0.55, ship: false, shadowAt: enu(0, 5, 105), shadowR: 60 },
  base: { cam: enu(-40, -70, 24), target: enu(0, -6, 9), fov: 50, open: 1, ship: false, shadowAt: enu(0, -10, 0), shadowR: 90 },
  ship: { cam: enu(-95, -125, 78), target: enu(0, 4, 92), fov: 42, open: 0, ship: true, shadowAt: enu(0, 5, 60), shadowR: 110 },
}

const q = new URLSearchParams(location.search)
const preset = PRESETS[q.get("view") ?? "wide"] ?? PRESETS.wide
const openQ = q.get("open")
const open = openQ !== null ? Number(openQ) : preset.open
const envMode = q.get("env") === "sky" ? "sky" : "room"
const az = (Number(q.get("az") ?? 225) * Math.PI) / 180
const el = (Number(q.get("el") ?? 35) * Math.PI) / 180

const canvas = document.getElementById("c")
const hud = document.getElementById("hud")
if (!(canvas instanceof HTMLCanvasElement) || !hud) throw new Error("pad harness: missing #c / #hud")

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, logarithmicDepthBuffer: true })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 0.6
renderer.outputColorSpace = THREE.SRGBColorSpace
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFShadowMap // r0.187 removed PCFSoftShadowMap (falls back to this)

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x000000)
const camera = new THREE.PerspectiveCamera(preset.fov, 1, 0.5, 5e8)
camera.position.copy(preset.cam)
const controls = new OrbitControls(camera, canvas)
controls.target.copy(preset.target)
controls.enableDamping = true
controls.update()

// Sun from the SW at 35° (default) + sky fill
const sunDir = enu(Math.sin(az) * Math.cos(el), Math.cos(az) * Math.cos(el), Math.sin(el)).normalize()
const sun = new THREE.DirectionalLight(0xfff4e0, 3.0)
sun.castShadow = true
sun.shadow.mapSize.set(4096, 4096)
const sc = sun.shadow.camera
sc.left = -preset.shadowR; sc.right = preset.shadowR; sc.top = preset.shadowR; sc.bottom = -preset.shadowR
sc.near = 1; sc.far = 2000
sun.shadow.bias = -0.0002
sun.shadow.normalBias = 0.06
sun.target.position.copy(preset.shadowAt)
sun.position.copy(preset.shadowAt).addScaledVector(sunDir, 800)
const hemi = new THREE.HemisphereLight(0xbcd7ff, 0x5a5040, 0.2)
scene.add(sun, sun.target, hemi)

// environment for reflections: RoomEnvironment (default) or the app's sky capture
const pmrem = new THREE.PMREMGenerator(renderer)
if (envMode === "room") {
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture
} else {
  const envScene = new THREE.Scene()
  const envSky = makeSky()
  envSky.setSun(sunDir, 0, 0)
  const ground = new THREE.Mesh(new THREE.SphereGeometry(5e4, 32, 12, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2), new THREE.MeshBasicMaterial({ color: new THREE.Color(0.2, 0.19, 0.14), side: THREE.BackSide }))
  envScene.add(envSky.sky, ground)
  scene.environment = pmrem.fromScene(envScene, 0, 0.1, 1e6).texture
}

const { sky, setSun } = makeSky()
setSun(sunDir, 0, 0)
scene.add(sky)
const coast = makeCoast()
coast.water.uniforms.uSun.value.copy(sunDir)
scene.add(coast.root)
// diagnostic (?ground=dense): swap the coast's single-quad 300 km planes for subdivided ones to test
// the depth-precision artefact of huge triangles (scrub drawn over objects < ≈ 4 m tall at close range)
if (q.get("ground") === "dense") {
  coast.root.children.forEach((o) => {
    if (!(o instanceof THREE.Mesh) || !(o.geometry instanceof THREE.PlaneGeometry)) return
    const { width, height } = o.geometry.parameters
    o.geometry = new THREE.PlaneGeometry(width, height, Math.min(256, Math.ceil(width / 1000)), Math.min(256, Math.ceil(height / 1000)))
  })
}

const t0 = performance.now()
const pad = makePad()
pad.setArms(open)
scene.add(pad.root)
const buildMs = performance.now() - t0

if (preset.ship) {
  const steel = new THREE.MeshStandardMaterial({ color: 0xc9ced3, metalness: 0.9, roughness: 0.3 })
  const base = TOWER.armHeight - SHIP.pinX
  const hull = new THREE.Mesh(new THREE.CylinderGeometry(SHIP.r, SHIP.r, SHIP.len, 64), steel)
  hull.position.copy(enu(0, 0, base + SHIP.len / 2))
  hull.castShadow = hull.receiveShadow = true
  scene.add(hull)
  for (const s of [-1, 1]) {
    const pin = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.5, 0.8), steel)
    pin.position.copy(enu(s * (SHIP.r + 0.9), 0, TOWER.armHeight + 0.25))
    pin.castShadow = true
    scene.add(pin)
  }
}

const composer = new EffectComposer(renderer)
composer.addPass(new RenderPass(scene, camera))
composer.addPass(new UnrealBloomPass(new THREE.Vector2(512, 512), 0.35, 0.3, 2.5))
composer.addPass(new OutputPass())

const countTris = (root: THREE.Object3D): number => {
  let tris = 0
  root.traverse((o) => {
    if (!(o instanceof THREE.Mesh)) return
    const g = o.geometry
    const n = (g.index ? g.index.count : g.attributes.position.count) / 3
    tris += n * (o instanceof THREE.InstancedMesh ? o.count : 1)
  })
  return tris
}
let draws = 0
pad.root.traverse((o) => { if (o instanceof THREE.Mesh) draws++ })
const tris = countTris(pad.root)

const resize = (): void => {
  const w = canvas.clientWidth, h = canvas.clientHeight
  renderer.setSize(w, h, false)
  composer.setSize(w, h)
  camera.aspect = w / Math.max(h, 1)
  camera.updateProjectionMatrix()
}
window.addEventListener("resize", resize)
resize()

/**
 * geometry check against the contact model: for open = 0 and 1, the bumper inner faces must sit at
 * ±gap/2, the rail tops at armHeight, the rails/arm within n ∈ [pos.y − 2 − armLength, pos.y − 2]
 */
const armCheck = (): readonly object[] => [0, 1].map((k) => {
  pad.setArms(k)
  pad.root.updateMatrixWorld(true)
  const gap = TOWER.closedGap + (TOWER.openGap - TOWER.closedGap) * k
  const arms = pad.root.children.filter((o) => o.name.startsWith("chopstick-"))
  const r = arms.map((g) => {
    const part = (name: string): THREE.Box3 => {
      const m = g.children.find((o) => o.name === name)
      return m ? new THREE.Box3().setFromObject(m) : new THREE.Box3()
    }
    const bump = part("box|yellow"), rail = part("box|rail"), all = new THREE.Box3().setFromObject(g)
    return {
      arm: g.name, innerFaceE: g.name.endsWith("east") ? bump.min.x : bump.max.x, railTopU: rail.max.y,
      railN: [-rail.max.z, -rail.min.z], tipN: -all.max.z,
    }
  })
  return { open: k, gap, expectInnerE: [-gap / 2, gap / 2], expectTopU: TOWER.armHeight, expectTipN: TOWER.pos.y - 2 - TOWER.armLength, arms: r }
})
const check = armCheck()
pad.setArms(open)

declare global {
  interface Window { __ready?: boolean; __tris?: number; __draws?: number; __buildMs?: number; __check?: readonly object[] }
}
const w = window
w.__tris = tris
w.__draws = draws
w.__buildMs = buildMs
w.__check = check
let frames = 0
const loop = (): void => {
  controls.update()
  coast.water.uniforms.uTime.value = (performance.now() - t0) / 1000
  composer.render()
  frames++
  if (frames === 3) w.__ready = true
  hud.textContent = `Pad 2 · ${q.get("view") ?? "wide"} · open ${open.toFixed(2)} · env ${envMode} · ${Math.round(tris).toLocaleString()} triangles · ${draws} meshes · build ${buildMs.toFixed(0)} ms`
  requestAnimationFrame(loop)
}
loop()
