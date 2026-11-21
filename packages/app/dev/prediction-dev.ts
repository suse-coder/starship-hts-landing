/**
 * Dev harness for the predicted-path overlay (view/src/prediction.ts), independent of the full app
 * simulation: a synthetic pad-ENU approach with the same sample layout, so the spline, the optional
 * `rec.pred` plan line and the visibility toggle can be checked in isolation.
 *
 * Query: i=<0..1 current sample> · plan=0|1 · path=0|1 · still=1
 */
import * as THREE from "three"
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js"
import { RenderPass } from "three/addons/postprocessing/RenderPass.js"
import { OutputPass } from "three/addons/postprocessing/OutputPass.js"
import { CH, N_CH, type Recording } from "@ship/sim"
import { makePathOverlay } from "../../view/src/prediction"

const qs = new URLSearchParams(location.search)
const frac = Math.max(0, Math.min(1, Number(qs.get("i") ?? 0.85)))
const planOn = qs.get("plan") !== "0"
const pathOn = qs.get("path") !== "0"

const canvas = document.getElementById("c") as HTMLCanvasElement
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, logarithmicDepthBuffer: true })
renderer.setPixelRatio(1)
renderer.setSize(window.innerWidth, window.innerHeight, false)
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 0.8
renderer.outputColorSpace = THREE.SRGBColorSpace
const scene = new THREE.Scene()
scene.background = new THREE.Color(0x0a1018)
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.3, 1e6)

scene.add(new THREE.HemisphereLight(0xbcd7ff, 0x40382a, 0.9))
const sun = new THREE.DirectionalLight(0xfff4e0, 2.2)
sun.position.set(-300, 500, 200)
scene.add(sun)
const ground = new THREE.Mesh(new THREE.CircleGeometry(4000, 64), new THREE.MeshStandardMaterial({ color: 0x2a3038, roughness: 1 }))
ground.rotation.x = -Math.PI / 2
ground.position.y = -120
scene.add(ground)
const grid = new THREE.GridHelper(4000, 40, 0x1c3848, 0x162630)
grid.position.y = -119
scene.add(grid)

const n = 240
const data: Float64Array[] = Array.from({ length: N_CH }, () => new Float64Array(n))
const shape = (u: number): [number, number, number] => {
  // a plausible belly-flop → flip → landing approach in pad ENU
  const s = 1 - u
  const pe = 2200 * s * s + 620 * Math.sin(u * Math.PI * 0.9) * s
  const pn = 1500 * s * s - 300 * Math.sin(u * Math.PI * 1.3) * s
  const pu = 90 + 3600 * Math.pow(s, 1.35)
  return [pe, pn, pu]
}
for (let i = 0; i < n; i++) {
  const u = i / (n - 1)
  const [e, nn, uu] = shape(u)
  data[CH.pe][i] = e; data[CH.pn][i] = nn; data[CH.pu][i] = uu
}
const k = 24
const pred = new Float32Array(n * k * 3)
const predHas = new Uint8Array(n)
for (let s = 0; s < n; s++) {
  predHas[s] = s > n * 0.55 ? 1 : 0
  const [e0, n0, u0] = shape(s / (n - 1))
  for (let q = 0; q < k; q++) {
    const u = q / (k - 1)
    // the plan is a slightly east-offset copy of the approach, ending in the arms
    const [e, nn, uu] = shape((1 - u) * 0 + (s / (n - 1)) * (1 - u) + (1 - u) * 0 + u * 1)
    const o = (s * k + q) * 3
    pred[o] = e + 25
    pred[o + 1] = nn
    pred[o + 2] = uu
  }
  void e0; void n0; void u0
}
const rec = { n, data, heat: new Float32Array(0), cells: 0, events: [], meta: {} } as unknown as Recording
;(rec as unknown as { pred: Float32Array }).pred = pred
;(rec as unknown as { predK: number }).predK = k
;(rec as unknown as { predHas: Uint8Array }).predHas = predHas

const overlay = makePathOverlay()
scene.add(overlay.group)
overlay.setVisible(pathOn)

const i = Math.round(frac * (n - 1))
const [ce, cn, cu] = shape(i / (n - 1))
const [te, tn, tu] = shape(1)
// catch tower at the future catch point, relative to the current ship
const tower = new THREE.Group()
const post = new THREE.Mesh(new THREE.BoxGeometry(16, 160, 16), new THREE.MeshStandardMaterial({ color: 0x8894a2, metalness: 0.5, roughness: 0.5, emissive: 0x1a2630, emissiveIntensity: 0.8 }))
post.position.y = -40
const armL = new THREE.Mesh(new THREE.BoxGeometry(90, 7, 7), new THREE.MeshStandardMaterial({ color: 0xe0b85a, metalness: 0.4, roughness: 0.5, emissive: 0x5a4310, emissiveIntensity: 0.6 }))
const armR = armL.clone()
armL.position.set(-35, 42, 18); armR.position.set(-35, 42, -18)
tower.add(post, armL, armR)
tower.position.set(te - ce, tu - cu - 42, -(tn - cn))
scene.add(tower)
const ship = new THREE.Mesh(new THREE.CylinderGeometry(4.5, 4.5, 40, 24), new THREE.MeshStandardMaterial({ color: 0xcfd6dd, metalness: 1, roughness: 0.3 }))
ship.rotation.z = Math.PI / 2
scene.add(ship)

// camera: a three-quarter view of the ship and the path from here into the arms
const target = new THREE.Vector3((te - ce) * 0.35, Math.max(-60, (tu - cu) * 0.25), -(tn - cn) * 0.35)
const dist = Math.max(150, Math.min(620, Math.hypot(te - ce, tn - cn, tu - cu) * 0.48))
camera.position.set(target.x + dist * 0.62, target.y + dist * 0.34, target.z + dist * 0.72)
camera.lookAt(target)

const frame = (): void => {
  overlay.update(rec, i, 0)
  renderer.render(scene, camera)
}
const w = window as unknown as { __ready: boolean; __frame: () => void; __overlay: typeof overlay }
w.__overlay = overlay
w.__frame = frame
frame()
w.__ready = true
if (!qs.has("still")) {
  const loop = (): void => { frame(); requestAnimationFrame(loop) }
  requestAnimationFrame(loop)
}
window.addEventListener("resize", () => {
  renderer.setSize(window.innerWidth, window.innerHeight, false)
  camera.aspect = window.innerWidth / window.innerHeight
  camera.updateProjectionMatrix()
})
