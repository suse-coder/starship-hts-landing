/**
 * Dev harness for the detailed vehicle (packages/view/src/ship.ts + parts.ts + parts/*).
 * URL: /dev/ship.html?view=full|aft|engines|nose|exploded|internal|pez|cap [&patch=0] [&gp=0.1&gy=-0.1]
 *      [&az=35&el=10&dist=75] [&x=26] (target body x) [&hide=id,id] [&show=id]
 * patch=1 (default) previews the proposed ship.ts changes: open aft skirt (no base disk), ship.ts box
 * pins hidden (detailed pins come from parts), PICA-X maps + metric-UV geometry from parts/pica.ts.
 * Renderer settings follow packages/view/src/index.ts (ACES 0.6, sRGB, PCF soft shadows, log depth).
 */
import * as THREE from "three"
import { OrbitControls } from "three/addons/controls/OrbitControls.js"
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js"
import { DEFAULT_DESIGN, makeVehicle } from "@ship/craft"
import { makeThermalGrid } from "@ship/heat"
import { makeShip, type ShipFrame } from "../../view/src/ship"
import { makePicaGeometry, makePicaMaps } from "../../view/src/parts"

interface Preset {
  readonly x: number // target body x
  readonly az: number // deg, about the vehicle axis (world up)
  readonly el: number // deg
  readonly dist: number
  readonly fov: number
  readonly exploded?: number
  readonly hide?: readonly string[]
}

const ENGINE_HIDE = ["hull", "seams", "vents", "shieldDeck", "p:skirt", "raceway", "rcsPods", "p:loxTank", "p:ch4Tank", "p:aftDome", "p:headerLines", "p:copv", "p:batteries", "picaEdge", "pez", "catchPins", "starlink", "ventPorts", "electrodePads", "csPorts"]
const PRESETS: Readonly<Record<string, Preset>> = {
  full: { x: 26, az: 35, el: 8, dist: 88, fov: 40 },
  aft: { x: 1.2, az: 25, el: -38, dist: 17, fov: 45 },
  engines: { x: 2.6, az: 40, el: -6, dist: 10.5, fov: 45, hide: ENGINE_HIDE },
  nose: { x: 46.5, az: 55, el: 12, dist: 20, fov: 45 },
  cap: { x: 50.5, az: 20, el: 25, dist: 8, fov: 45 },
  pez: { x: 35, az: 85, el: 5, dist: 13, fov: 45 },
  exploded: { x: 28, az: 25, el: 18, dist: 118, fov: 42, exploded: 1 },
  internal: { x: 26, az: 35, el: 10, dist: 88, fov: 40, hide: ["hull"] },
}

const q = new URLSearchParams(location.search)
const name = q.get("view") ?? "full"
const base = PRESETS[name] ?? PRESETS.full
const num = (k: string, d: number): number => (q.get(k) !== null ? Number(q.get(k)) : d)
const preset: Preset = { ...base, x: num("x", base.x), az: num("az", base.az), el: num("el", base.el), dist: num("dist", base.dist), exploded: num("exploded", base.exploded ?? 0) }
const patch = q.get("patch") !== "0"

const canvas = document.getElementById("c")
const hud = document.getElementById("hud")
if (!(canvas instanceof HTMLCanvasElement) || !hud) throw new Error("ship harness: missing #c / #hud")

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, logarithmicDepthBuffer: true })
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
renderer.toneMapping = THREE.ACESFilmicToneMapping
renderer.toneMappingExposure = 0.6
renderer.outputColorSpace = THREE.SRGBColorSpace
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFSoftShadowMap
renderer.info.autoReset = true

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x2a2e34)
const pmrem = new THREE.PMREMGenerator(renderer)
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture

// ---- vehicle: body x → world +Y (body (x, y, z) → world (−y, x, z))
const vehicle = makeVehicle(DEFAULT_DESIGN)
const grid = makeThermalGrid(vehicle)
const t0 = performance.now()
const ship = makeShip(vehicle, grid)
const buildMs = performance.now() - t0
ship.root.rotation.z = Math.PI / 2
scene.add(ship.root)
const bodyToWorld = (x: number, y = 0, z = 0): THREE.Vector3 => new THREE.Vector3(-y, x, z)

if (patch) {
  // (a) open aft skirt: drop the hull base-disk triangles (all three vertices on x = 0)
  ship.root.traverse((o) => {
    const m = o as THREE.Mesh
    if (!m.isMesh || !(m.parent?.name === "hull" || m.parent?.name === "heatGlow")) return
    const g = m.geometry, idx = g.index, p = g.attributes.position
    if (!idx || !(g instanceof THREE.LatheGeometry || g.attributes.color)) return
    const keep: number[] = []
    for (let i = 0; i < idx.count; i += 3) {
      const a = idx.getX(i), b = idx.getX(i + 1), c = idx.getX(i + 2)
      if (Math.abs(p.getX(a)) < 1e-3 && Math.abs(p.getX(b)) < 1e-3 && Math.abs(p.getX(c)) < 1e-3) continue
      keep.push(a, b, c)
    }
    g.setIndex(keep)
  })
  // (b) ship.ts box pins replaced by parts "catchPins"
  ship.setVisible("pins", false)
  // (c) PICA-X maps + metric UVs
  const hullLayer = ship.root.getObjectByName("hull")
  const pica = hullLayer?.children[1] as THREE.Mesh | undefined
  if (pica) {
    const maps = makePicaMaps()
    pica.geometry = makePicaGeometry(vehicle.shape, vehicle.pica.x0)
    const mat = pica.material as THREE.MeshStandardMaterial
    mat.map = maps.map
    mat.normalMap = maps.normalMap
    mat.roughnessMap = maps.roughnessMap
    mat.roughness = 1
    mat.metalness = 0
    mat.needsUpdate = true
  }
}
for (const id of [...(preset.hide ?? []), ...(q.get("hide") ?? "").split(",").filter(Boolean)]) ship.setVisible(id, false)
for (const id of (q.get("show") ?? "").split(",").filter(Boolean)) ship.setVisible(id, true)

const frame: ShipFrame = {
  heat: new Float32Array(grid.cellPhi.length).fill(300),
  thrust: [0, 0, 0], gp: num("gp", 0), gy: num("gy", 0), rcsDuty: new Array(12).fill(0), coilFrac: 0, electrodes: new Array(8).fill(0),
  qStag: 0, ne: 0, t0: 300, standoff: 1, standoff0: 1, heatFactor: 1, mach: 0, qbar: 0, vAirBody: new THREE.Vector3(1, 0, 0),
  exploded: preset.exploded ?? 0, pAmb: 101325, studio: true, thermalMode: false, time: 0,
}
ship.update(frame)

// ---- camera + light
const target = bodyToWorld(preset.x)
const camera = new THREE.PerspectiveCamera(preset.fov, 1, 0.05, 5e4)
const az = (preset.az * Math.PI) / 180, el = (preset.el * Math.PI) / 180
camera.position.copy(target).add(new THREE.Vector3(Math.cos(el) * Math.sin(az), Math.sin(el), Math.cos(el) * Math.cos(az)).multiplyScalar(preset.dist))
const controls = new OrbitControls(camera, canvas)
controls.target.copy(target)
controls.update()
const sun = new THREE.DirectionalLight(0xfff4e0, 2.6)
sun.position.copy(target).add(new THREE.Vector3(-40, 60, 70))
sun.target.position.copy(target)
sun.castShadow = true
sun.shadow.mapSize.set(4096, 4096)
const sr = Math.max(12, preset.dist * 0.55)
Object.assign(sun.shadow.camera, { left: -sr, right: sr, top: sr, bottom: -sr, near: 1, far: 400 })
sun.shadow.bias = -0.0002
sun.shadow.normalBias = 0.03
scene.add(sun, sun.target, new THREE.HemisphereLight(0xbcd7ff, 0x5a5040, 0.35))

const resize = (): void => {
  const w = canvas.clientWidth, h = canvas.clientHeight
  renderer.setSize(w, h, false)
  camera.aspect = w / h
  camera.updateProjectionMatrix()
}
window.addEventListener("resize", resize)
resize()

const visibleTris = (): number => {
  let n = 0
  scene.traverseVisible((o) => {
    const m = o as THREE.Mesh
    if (!m.isMesh) return
    const g = m.geometry
    n += ((g.index ? g.index.count : g.attributes.position.count) / 3) * ((o as THREE.InstancedMesh).isInstancedMesh ? (o as THREE.InstancedMesh).count : 1)
  })
  return n
}
let frames = 0
const loop = (): void => {
  controls.update()
  renderer.render(scene, camera)
  if (++frames === 3) {
    const info = renderer.info.render
    const stats = { view: name, drawnTriangles: info.triangles, drawCalls: info.calls, visibleGeometryTriangles: Math.round(visibleTris()), buildMs: Math.round(buildMs) }
    ;(window as unknown as { __stats: unknown }).__stats = stats
    hud.textContent = `Ship 42 — ${name}${patch ? " (ship.ts patch preview)" : ""}\n` + Object.entries(stats).map(([k, v]) => `${k}: ${v}`).join("\n")
    console.log("[ship-dev]", JSON.stringify(stats))
  }
  requestAnimationFrame(loop)
}
loop()
