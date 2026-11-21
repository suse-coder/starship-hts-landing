/** @ship/view — three.js playback of a recorded mission (no simulation here: pure rendering). */
import * as THREE from "three"
import { OrbitControls } from "three/addons/controls/OrbitControls.js"
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js"
import { RenderPass } from "three/addons/postprocessing/RenderPass.js"
import { UnrealBloomPass } from "three/addons/postprocessing/UnrealBloomPass.js"
import { ShaderPass } from "three/addons/postprocessing/ShaderPass.js"
import { OutputPass } from "three/addons/postprocessing/OutputPass.js"
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js"
import { CH, locate, type Recording, type ChannelKey } from "@ship/sim"
import { makeVehicle, type DesignParams } from "@ship/craft"
import { makeThermalGrid } from "@ship/heat"
import { meanWindEnu, sunDirectionEci, type WindParams } from "@ship/earth"
import { makeShip, type ShipModel, type TreeNode } from "./ship"
import { makePad, enu, type PadModel } from "./tower"
import { EARTH_GEOM_TO_ECEF, makeAtmosphere, makeCoast, makeEarth, makeStars } from "./env"
import { makeWindField } from "./fx"
import { makeClouds } from "./clouds"
import { makePathOverlay } from "./prediction"
import { loadCloudDetail, loadCloudMask, makeGlobalClouds } from "./cloudsGlobal"

export type { TreeNode } from "./ship"
export { glowColor, marginColor } from "./ship"
export { rainbow } from "./fx"

export type CameraMode = "chase" | "tower" | "globe"

export interface ViewTreeNode extends TreeNode { readonly on?: boolean }

export interface View {
  readonly setRecording: (rec: Recording) => void
  readonly render: (t: number) => void
  readonly setMode: (m: CameraMode) => void
  readonly setExploded: (on: boolean) => void
  readonly resize: () => void
  /** browser tree (ship, physics overlays, environment) — valid after setRecording */
  readonly tree: () => readonly ViewTreeNode[]
  readonly setVisible: (id: string, on: boolean) => void
  /** current exploded-view displacement 0..1 (spring-animated) */
  readonly explodeProgress: () => number
  /** studio (Fusion-like white) or physical sky background */
  readonly setPhysicalSky: (on: boolean) => void
  readonly home: () => void
  readonly fit: () => void
  /** legend state for the app overlays */
  readonly legends: () => { readonly thermal: boolean; readonly wind: boolean; readonly windMin: number; readonly windMax: number }
}

const R_E = 6371e3
const OM = 7.292115e-5
const PHASE_COLORS = [0x8899aa, 0xff9a3c, 0x8899aa, 0xff5a3c, 0xffd23c, 0xff3cbe, 0xb07cff, 0x3cc8ff, 0xffffff, 0x7cff9a, 0x7cffd8, 0x3cff6e, 0xff3c3c]
// Cloud-free Blue Marble day map + the NASA fair-weather cloud mask as a separate shell: the clouds
// are no longer painted flat on the surface, so there is no double-cloud ghost during the hand-off.
const EARTH_TEX = `${import.meta.env?.BASE_URL ?? "./"}textures/earth_day_cloudfree_4096.jpg`
const NIGHT_TEX = `${import.meta.env?.BASE_URL ?? "./"}textures/earth_night_blackmarble2016_3840.jpg`
const CLOUD_TEX = `${import.meta.env?.BASE_URL ?? "./"}textures/earth_clouds_4096.jpg`
const CLOUD_DETAIL_TEX = `${import.meta.env?.BASE_URL ?? "./"}textures/earth_clouds_detail_512.png`
const SUN_ANG_R = 4.65e-3 // rad, apparent solar radius
const smoothstep = (a: number, b: number, x: number): number => {
  const t = Math.max(0, Math.min(1, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)
}
/** hand-off: global shell at altitude, local volumetric deck on approach (0 = global, 1 = local) */
const LOCAL_CLOUD_LO = 15e3
const LOCAL_CLOUD_HI = 60e3
/** local deck's regional half-width grows from 60 km at the ground to 240 km at 60 km altitude */
const localCloudExtent = (alt: number): number => 60e3 + 3 * Math.max(0, Math.min(60e3, alt))
/** fraction of the solar disc visible past the Earth's limb (linear penumbra model) */
const sunVisible = (sunDir: THREE.Vector3, toCentre: THREE.Vector3, rE: number): number => {
  const d = toCentre.length()
  const rho = Math.asin(Math.min(1, rE / d)) // angular radius of the Earth
  const theta = Math.acos(Math.max(-1, Math.min(1, sunDir.dot(toCentre) / d))) // Sun–Earth-centre separation
  return Math.max(0, Math.min(1, (theta - (rho - SUN_ANG_R)) / (2 * SUN_ANG_R)))
}

/** vertical white → light-grey gradient like Fusion 360's default environment */
const studioBackground = (): THREE.Texture => {
  const c = document.createElement("canvas")
  c.width = 2; c.height = 256
  const g = c.getContext("2d")!
  const gr = g.createLinearGradient(0, 0, 0, 256)
  gr.addColorStop(0, "#ffffff"); gr.addColorStop(0.65, "#f4f6f8"); gr.addColorStop(1, "#dde2e7")
  g.fillStyle = gr; g.fillRect(0, 0, 2, 256)
  const tex = new THREE.CanvasTexture(c)
  tex.colorSpace = THREE.SRGBColorSpace
  return tex
}

/** Fusion-style ViewCube in its own small canvas */
const makeViewCube = (cv: HTMLCanvasElement, onPick: (dir: THREE.Vector3) => void): ((cam: THREE.Camera, target: THREE.Vector3) => void) => {
  const r = new THREE.WebGLRenderer({ canvas: cv, antialias: true, alpha: true })
  r.setPixelRatio(Math.max(2, Math.min(window.devicePixelRatio * 2, 4)))
  const sc = new THREE.Scene()
  const cam = new THREE.PerspectiveCamera(30, 1, 0.1, 20)
  const face = (txt: string): THREE.MeshBasicMaterial => {
    const c = document.createElement("canvas"); c.width = c.height = 512
    const g = c.getContext("2d")!
    g.fillStyle = "#f3f5f7"; g.fillRect(0, 0, 512, 512)
    g.strokeStyle = "#9aa1a8"; g.lineWidth = 14; g.strokeRect(0, 0, 512, 512)
    g.fillStyle = "#4a525a"; g.font = "600 92px system-ui, sans-serif"; g.textAlign = "center"; g.textBaseline = "middle"; g.fillText(txt, 256, 262)
    const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8
    return new THREE.MeshBasicMaterial({ map: t })
  }
  const cube = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), ["EAST", "WEST", "UP", "DOWN", "SOUTH", "NORTH"].map(face))
  sc.add(cube)
  const axes = new THREE.AxesHelper(0.9); axes.position.set(-0.5, -0.5, 0.5); sc.add(axes)
  const ray = new THREE.Raycaster()
  cv.addEventListener("click", (e) => {
    const b = cv.getBoundingClientRect()
    ray.setFromCamera(new THREE.Vector2(((e.clientX - b.left) / b.width) * 2 - 1, -((e.clientY - b.top) / b.height) * 2 + 1), cam)
    const hit = ray.intersectObject(cube)[0]
    if (hit?.face) onPick(hit.face.normal.clone())
  })
  return (main, target) => {
    const d = main.position.clone().sub(target).normalize().multiplyScalar(3.4)
    cam.position.copy(d); cam.up.copy(main.up); cam.lookAt(0, 0, 0)
    r.setSize(cv.clientWidth, cv.clientHeight, false)
    r.render(sc, cam)
  }
}

/** colour of direct sunlight after Rayleigh extinction at air mass m (Kasten–Young air mass; τ_R ∝ λ⁻⁴·⁰⁵) */
const sunTintRGB = (sinElev: number): THREE.Color => {
  const h = Math.max(0.02, Math.asin(Math.max(-1, Math.min(1, sinElev))))
  const m = 1 / (Math.sin(h) + 0.50572 * Math.pow((h * 180) / Math.PI + 6.07995, -1.6364))
  const tau = (lambdaUm: number): number => 0.00865 * Math.pow(lambdaUm, -4.05)
  return new THREE.Color(Math.exp(-tau(0.65) * m), Math.exp(-tau(0.55) * m), Math.exp(-tau(0.45) * m))
}

export const createView = (canvas: HTMLCanvasElement, labelLayer: HTMLElement, cubeCanvas?: HTMLCanvasElement): View => {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, logarithmicDepthBuffer: true })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = 0.6
  renderer.outputColorSpace = THREE.SRGBColorSpace
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = THREE.PCFShadowMap
  const scene = new THREE.Scene()
  const camera = new THREE.PerspectiveCamera(45, 1, 0.5, 5e8)
  camera.position.set(-90, 40, 110)
  const controls = new OrbitControls(camera, canvas)
  controls.enableDamping = true
  controls.dampingFactor = 0.08
  // right-drag pans the view (move freely around the scene); left-drag still orbits the target.
  // The target is *not* re-centred on the ship every frame, so the pan persists until Fit/Home.
  controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN }
  controls.screenSpacePanning = true
  canvas.addEventListener("contextmenu", (e) => e.preventDefault())
  const composer = new EffectComposer(renderer)
  composer.addPass(new RenderPass(scene, camera))
  // NaN/Inf guard before bloom: one bad fragment (plasma, heat glow, cloud raymarch) would otherwise be
  // smeared over the whole screen by the bloom mip chain — a view-dependent white flash. Clamp to [0, 6e4].
  const sanitize = new ShaderPass({
    uniforms: { tDiffuse: { value: null } },
    vertexShader: `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
    fragmentShader: `uniform sampler2D tDiffuse; varying vec2 vUv;
      void main(){
        vec4 c = texture2D(tDiffuse, vUv);
        c.r = (c.r >= 0.0 && c.r < 60000.0) ? c.r : 0.0;
        c.g = (c.g >= 0.0 && c.g < 60000.0) ? c.g : 0.0;
        c.b = (c.b >= 0.0 && c.b < 60000.0) ? c.b : 0.0;
        c.a = 1.0;
        gl_FragColor = c;
      }`,
  })
  composer.addPass(sanitize)
  // bloom only for emitters (plasma, plumes, incandescent steel, the Sun disc): HDR threshold above lit surfaces
  const bloom = new UnrealBloomPass(new THREE.Vector2(512, 512), 0.35, 0.3, 2.5)
  composer.addPass(bloom)
  composer.addPass(new OutputPass())
  // light: Sun (shadows near the ship) + sky fill
  const sun = new THREE.DirectionalLight(0xfff4e0, 3.0)
  sun.castShadow = true
  sun.shadow.mapSize.set(2048, 2048)
  const sc = sun.shadow.camera
  sc.left = -120; sc.right = 120; sc.top = 120; sc.bottom = -120; sc.near = 1; sc.far = 2000
  sun.shadow.bias = -0.0002
  sun.shadow.normalBias = 0.06
  const hemi = new THREE.HemisphereLight(0xbcd7ff, 0x5a5040, 0.25)
  scene.add(sun, sun.target, hemi)
  // image-based lighting: PBR steel needs an environment to reflect. Physical mode captures the
  // same single-scattering sky (+ a ground/Earth hemisphere) into a PMREM cube; studio: a soft room.
  const pmrem = new THREE.PMREMGenerator(renderer)
  const roomEnv = pmrem.fromScene(new RoomEnvironment(), 0.04).texture
  const envScene = new THREE.Scene()
  const envAtm = makeAtmosphere(R_E, 1e-3, false)
  const envGround = new THREE.Mesh(new THREE.SphereGeometry(5e4, 32, 12, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2), new THREE.MeshBasicMaterial({ color: 0x40382a, side: THREE.BackSide, depthWrite: false }))
  envGround.renderOrder = -6
  envScene.add(envAtm.mesh, envGround)
  let skyEnv: THREE.WebGLRenderTarget | null = null
  let envKey = ""
  const envOrigin = new THREE.Vector3()
  const updateEnv = (sunDir: THREE.Vector3, alt: number, dayK: number, rn: number): void => {
    // fine quantisation (altitude ≈ 5 % in log, Sun ≈ 1°, day factor 5 %): consecutive captures differ imperceptibly
    const key = `${Math.round(Math.log1p(Math.max(alt, 0) / 200) * 20)}|${Math.round(dayK * 20)}|${Math.round(sunDir.x * 57)},${Math.round(sunDir.y * 57)},${Math.round(sunDir.z * 57)}`
    if (key === envKey) return
    envKey = key
    envAtm.mesh.position.set(0, -rn, 0)
    envAtm.update(sunDir, envAtm.mesh.position, envOrigin, 1)
    // ground below the horizon: coastal scrub near the pad, ocean/cloud Earth from altitude
    const kHigh = Math.max(0, Math.min(1, alt / 20e3))
    const c = new THREE.Color(0.2, 0.19, 0.14).lerp(new THREE.Color(0.13, 0.2, 0.32), kHigh).multiplyScalar(Math.max(0.03, dayK))
    ;(envGround.material as THREE.MeshBasicMaterial).color.copy(c)
    skyEnv?.dispose()
    skyEnv = pmrem.fromScene(envScene, 0, 0.1, 1e6)
  }
  // ---- local frame (metres, ship at the origin)
  const local = new THREE.Group()
  scene.add(local)
  // the sky is the single-scattering atmosphere of the local Earth (earthL.atmosphere) at every altitude
  const stars = makeStars()
  scene.add(stars)
  const coast = makeCoast()
  local.add(coast.root)
  const pad: PadModel = makePad()
  local.add(pad.root)
  const earthL = makeEarth(R_E, EARTH_TEX, NIGHT_TEX, 1e-3)
  earthL.earth.matrixAutoUpdate = false
  earthL.atmosphere.matrixAutoUpdate = false
  local.add(earthL.earth, earthL.atmosphere)
  // planet-wide cloud shell (visible from orbit down to the ground); the local volumetric deck takes
  // over around the pad through the shader's hole/hand-off uniforms
  const cloudMaskTex = loadCloudMask(CLOUD_TEX)
  const cloudDetailTex = loadCloudDetail(CLOUD_DETAIL_TEX)
  const cloudsGlobalL = makeGlobalClouds({ radius: R_E, height: 4200, texture: cloudMaskTex, detailTexture: cloudDetailTex })
  const cloudScaleL = new THREE.Matrix4().makeScale(cloudsGlobalL.shellRadius, cloudsGlobalL.shellRadius, cloudsGlobalL.shellRadius)
  local.add(cloudsGlobalL.mesh)
  let ship: ShipModel | null = null
  const shipFrame = new THREE.Group()
  local.add(shipFrame)
  const windField = makeWindField()
  local.add(windField.mesh)
  const clouds = makeClouds()
  local.add(clouds.mesh)
  const pathOverlay = makePathOverlay()
  local.add(pathOverlay.group)
  // ---- globe frame (km; ECEF → three: x, z, −y; the Earth geometry needs no rotation there)
  const globe = new THREE.Group()
  scene.add(globe)
  const g = (x: number, y: number, z: number): THREE.Vector3 => new THREE.Vector3(x / 1000, z / 1000, -y / 1000)
  const earthG = makeEarth(6371, EARTH_TEX, NIGHT_TEX, 1)
  globe.add(earthG.earth, earthG.atmosphere)
  const cloudsGlobalG = makeGlobalClouds({ radius: 6371, height: 4.2, texture: cloudMaskTex, detailTexture: cloudDetailTex })
  globe.add(cloudsGlobalG.mesh)
  const marker = new THREE.Mesh(new THREE.SphereGeometry(18, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }))
  const padMark = new THREE.Mesh(new THREE.SphereGeometry(14, 12, 10), new THREE.MeshBasicMaterial({ color: 0xff4d4d }))
  globe.add(marker, padMark)
  let trackDone: THREE.Line | null = null
  let predLine: THREE.Line | null = null
  let rec: Recording | null = null
  let mode: CameraMode = "chase"
  let explodeTarget = 0, explode = 0, explodeVel = 0
  let lastFrame = performance.now()
  const t0 = performance.now()
  const envHidden = new Set<string>()
  let physicalSky = true
  let thermalMode = false
  let windOn = true
  let pathOn = true
  let windMax = 1
  let windMin = 0
  let windP: WindParams = { scaleMean: 1, turbulence: 1, seed: 1 }
  const studio = studioBackground()
  const black = new THREE.Color(0x000000)
  const HOME = new THREE.Vector3(-0.62, 0.36, 0.7)
  let fitDist = 140
  const mouse = new THREE.Vector2(9, 9)
  const picker = new THREE.Raycaster()
  let hovered: THREE.Object3D | null = null
  canvas.addEventListener("pointermove", (e) => { const b = canvas.getBoundingClientRect(); mouse.set(((e.clientX - b.left) / b.width) * 2 - 1, -((e.clientY - b.top) / b.height) * 2 + 1) })
  canvas.addEventListener("pointerleave", () => mouse.set(9, 9))
  const updateCube = cubeCanvas ? makeViewCube(cubeCanvas, (dir) => {
    if (mode === "tower") return
    const tgt = controls.target.clone()
    camera.position.copy(tgt.clone().addScaledVector(dir, camera.position.distanceTo(tgt)))
    controls.update()
  }) : null
  let labelEls: HTMLDivElement[] = []
  let duration = 1

  const setRecording = (r: Recording): void => {
    rec = r
    duration = r.data[CH.t][r.n - 1]
    const meta = r.meta.vehicle as { design: DesignParams }
    windP = { scaleMean: meta.design.windScale, turbulence: meta.design.turbulence, seed: meta.design.seed }
    const vehicle = makeVehicle(meta.design)
    if (ship) shipFrame.remove(ship.root)
    ship = makeShip(vehicle, makeThermalGrid(vehicle))
    shipFrame.add(ship.root)
    if (trackDone) globe.remove(trackDone)
    if (predLine) globe.remove(predLine)
    const pts: number[] = [], cols: number[] = []
    const c = new THREE.Color()
    for (let i = 0; i < r.n; i += 2) {
      const th = r.data[CH.theta][i]
      const x = r.data[CH.rx][i], y = r.data[CH.ry][i], z = r.data[CH.rz][i]
      const p = g(x * Math.cos(th) + y * Math.sin(th), -x * Math.sin(th) + y * Math.cos(th), z)
      pts.push(p.x, p.y, p.z)
      c.setHex(PHASE_COLORS[r.data[CH.phase][i]] ?? 0xffffff)
      cols.push(c.r, c.g, c.b)
    }
    const geo = new THREE.BufferGeometry()
    geo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3))
    geo.setAttribute("color", new THREE.Float32BufferAttribute(cols, 3))
    trackDone = new THREE.Line(geo, new THREE.LineBasicMaterial({ vertexColors: true }))
    globe.add(trackDone)
    if (r.meta.predPath.length > 1) {
      predLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(r.meta.predPath.map((p) => g(p.x, p.y, p.z))), new THREE.LineDashedMaterial({ color: 0x888888, dashSize: 40, gapSize: 30 }))
      predLine.computeLineDistances()
      globe.add(predLine)
    }
    const la = r.meta.padLat, lo = r.meta.padLon
    padMark.position.copy(g(6371e3 * Math.cos(la) * Math.cos(lo), 6371e3 * Math.cos(la) * Math.sin(lo), 6371e3 * Math.sin(la)))
    labelLayer.innerHTML = ""
    labelEls = ship.labels.map((l) => {
      const d = document.createElement("div")
      d.className = "part-label"
      d.textContent = l.text
      labelLayer.appendChild(d)
      return d
    })
  }

  const v = (key: ChannelKey, i: number, f: number): number => {
    const ch = rec!.data[CH[key]]
    return ch[i] + (ch[i + 1] - ch[i]) * f
  }
  const basis = new THREE.Matrix4()
  const render = (t: number): void => {
    if (!rec || !ship) { renderer.render(scene, camera); return }
    const [i, f] = locate(rec, Math.min(t, duration))
    // predicted path: recorded future (or the MPC plan if the orchestrator attached rec.pred)
    if (pathOn) pathOverlay.update(rec, i, f)
    const rx = v("rx", i, f), ry = v("ry", i, f), rz = v("rz", i, f)
    const rn = Math.hypot(rx, ry, rz)
    const up = new THREE.Vector3(rx / rn, ry / rn, rz / rn)
    const east = new THREE.Vector3(0, 0, 1).cross(up).normalize()
    const north = up.clone().cross(east)
    const toThree = (a: THREE.Vector3): THREE.Vector3 => new THREE.Vector3(a.dot(east), a.dot(up), -a.dot(north))
    const qw = v("qw", i, f), qx = v("qx", i, f), qy = v("qy", i, f), qz = v("qz", i, f)
    const qn = Math.hypot(qw, qx, qy, qz)
    const q = new THREE.Quaternion(qx / qn, qy / qn, qz / qn, qw / qn) // body → ECI
    basis.makeBasis(toThree(new THREE.Vector3(1, 0, 0).applyQuaternion(q)), toThree(new THREE.Vector3(0, 1, 0).applyQuaternion(q)), toThree(new THREE.Vector3(0, 0, 1).applyQuaternion(q)))
    const xcg = v("xcg", i, f)
    ship.root.quaternion.setFromRotationMatrix(basis)
    ship.root.children.forEach((ch) => ch.position.set(-xcg, 0, 0))
    const vel = new THREE.Vector3(v("vx", i, f) + OM * ry, v("vy", i, f) - OM * rx, v("vz", i, f))
    // air velocity relative to the vehicle, body axes (the flow arrives from −vAir = direction of motion)
    const vb = vel.clone().applyQuaternion(q.clone().invert()).normalize().negate()
    const k = f < 0.5 ? i : i + 1
    const heat = rec.heat.subarray(k * rec.cells, (k + 1) * rec.cells)
    // exploded-view spring: soft, slightly bouncy outward; stiff and fast back together
    const nowMs = performance.now()
    const dtS = Math.min(0.05, (nowMs - lastFrame) / 1000)
    lastFrame = nowMs
    const kSp = explodeTarget > 0.5 ? 55 : 160
    const cSp = 2 * Math.sqrt(kSp) * (explodeTarget > 0.5 ? 0.6 : 0.9)
    explodeVel += (kSp * (explodeTarget - explode) - cSp * explodeVel) * dtS
    explode = Math.max(0, explode + explodeVel * dtS)
    if (explodeTarget === 0 && explode < 0.002 && Math.abs(explodeVel) < 0.01) { explode = 0; explodeVel = 0 }
    const alt = v("alt", i, f)
    ship.update({
      heat, thrust: [v("T1", i, f), v("T2", i, f), v("T3", i, f)], gp: v("gp", i, f), gy: v("gy", i, f),
      rcsDuty: Array.from({ length: 12 }, (_, m) => v(`r${m}` as ChannelKey, i, f)),
      coilFrac: Math.min(1, Math.abs(v("coilI", i, f)) / 2600), electrodes: Array.from({ length: 8 }, (_, m) => v(`u${m}` as ChannelKey, i, f)),
      qStag: v("qStag", i, f), ne: v("ne", i, f), t0: v("T0", i, f), standoff: v("standoff", i, f), standoff0: v("standoff0", i, f), heatFactor: Math.min(1, Math.max(0.05, v("heatFactor", i, f) || 1)),
      mach: v("mach", i, f), qbar: v("qbar", i, f), vAirBody: vb, exploded: explode, studio: !physicalSky, thermalMode,
      pAmb: Math.max(0, 101325 * Math.exp(-alt / 7200)), time: (nowMs - t0) / 1000,
    })
    // pad placement relative to the ship CG
    const pe = v("pe", i, f), pn = v("pn", i, f), pu = v("pu", i, f)
    const dhPad = Math.hypot(pe, pn)
    const near = dhPad < 100e3 && pu < 45e3
    pad.root.visible = near && pu < 30e3 && !envHidden.has("pad")
    // local coast/sea planes fade out over the Earth sphere between 25 and 45 km (no switch, no z-fight:
    // inside the atmosphere the Earth writes no depth and the planes draw over it)
    const coastK = (1 - Math.max(0, Math.min(1, (pu - 25e3) / 20e3))) * (1 - Math.max(0, Math.min(1, (dhPad - 60e3) / 40e3)))
    coast.root.visible = near && coastK > 0.002 && !envHidden.has("pad")
    coast.setFade(coastK)
    pad.root.position.copy(enu(-pe, -pn, -pu))
    coast.root.position.copy(pad.root.position)
    pad.setArms(v("arms", i, f))
    // Earth in the local frame: three ← ENU ← ECI ← ECEF(θ) ← geometry
    const th = v("theta", i, f)
    const eciFromEcef = new THREE.Matrix4().makeRotationZ(th)
    const threeFromEci = new THREE.Matrix4().makeBasis(new THREE.Vector3(east.x, up.x, -north.x), new THREE.Vector3(east.y, up.y, -north.y), new THREE.Vector3(east.z, up.z, -north.z))
    const mEarth = new THREE.Matrix4().multiplyMatrices(threeFromEci, eciFromEcef).multiply(EARTH_GEOM_TO_ECEF)
    mEarth.setPosition(new THREE.Vector3(0, -rn, 0))
    earthL.earth.matrix.copy(mEarth)
    earthL.atmosphere.matrix.copy(mEarth)
    const earthVis = (!near || pu > 8e3) && !envHidden.has("earth")
    earthL.earth.visible = earthVis
    earthL.atmosphere.visible = physicalSky && !envHidden.has("earth")
    earthG.earth.visible = !envHidden.has("earth")
    earthG.atmosphere.visible = !envHidden.has("earth")
    if (trackDone) trackDone.visible = !envHidden.has("track")
    // Sun: true direction at the recorded epoch (ECI → local three)
    const jd = rec.meta.epochJd + t / 86400
    const s = sunDirectionEci(jd)
    const sunThree = toThree(new THREE.Vector3(s.x, s.y, s.z)).normalize()
    sun.position.copy(sunThree).multiplyScalar(800)
    sun.target.position.set(0, 0, 0)
    const skyFade = Math.exp(-Math.max(alt, 0) / 8000) // overhead air column (star visibility)
    const sunEcef = new THREE.Vector3(s.x, s.y, s.z).applyMatrix4(new THREE.Matrix4().makeRotationZ(-th))
    const centreL = new THREE.Vector3(0, -rn, 0)
    const sunGlobe = g(sunEcef.x, sunEcef.y, sunEcef.z).normalize()
    earthL.update(sunThree, centreL, camera, 1)
    earthG.update(sunGlobe, new THREE.Vector3(), camera, 1)
    // eclipse: the Sun sets behind the Earth's limb (penumbra over the solar disc)
    const sunFrac = sunVisible(sunThree, centreL, R_E)
    const dayK = Math.max(0, Math.min(1, sunThree.y * 4 + 0.3))
    const cloudSunColor = new THREE.Color(1, 0.96, 0.9).multiply(sunTintRGB(sunThree.y)).multiplyScalar(3.0)
    const cloudSkyColor = new THREE.Color(0.163, 0.274, 0.423).multiplyScalar(dayK)
    // hand-off: the planet-wide shell from orbit down, the pad-frame volumetric deck on approach;
    // local deck's region grows with altitude and the global shell is holed out underneath it
    // the hand-off starts at 60 km but is strongly weighted toward the ground: above ~30 km the
    // planet-wide texture is the better-looking layer, and the volumetric deck only needs to carry
    // the pad-scale puffs once the camera is close
    const localK = Math.pow(smoothstep(LOCAL_CLOUD_HI, LOCAL_CLOUD_LO, alt), 3)
    const localExtent = localCloudExtent(alt)
    const nowS = (nowMs - t0) / 1000
    const cloudWind = meanWindEnu(2000, windP)
    const padDir = pad.root.position.clone().sub(centreL).normalize()
    // global shell, local frame: same Earth matrix, a few km larger
    cloudsGlobalL.mesh.matrix.copy(mEarth).multiply(cloudScaleL)
    cloudsGlobalL.mesh.matrixWorldNeedsUpdate = true
    cloudsGlobalL.update({
      sunDir: sunThree, sunColor: cloudSunColor, skyColor: cloudSkyColor, centre: centreL,
      camera: camera.position, time: nowS, wind: [cloudWind.x, cloudWind.y] as const, eclipse: sunFrac,
      holeDir: padDir, holeRadius: localExtent / R_E, holeAmount: localK,
    })
    cloudsGlobalL.mesh.visible = physicalSky && !envHidden.has("earth") && !envHidden.has("clouds")
    // global shell, globe frame (ECEF, km): the same mask and Sun, no local hole
    cloudsGlobalG.mesh.matrix.makeScale(cloudsGlobalG.shellRadius, cloudsGlobalG.shellRadius, cloudsGlobalG.shellRadius)
    cloudsGlobalG.mesh.matrixWorldNeedsUpdate = true
    cloudsGlobalG.update({
      sunDir: sunGlobe, sunColor: cloudSunColor, skyColor: cloudSkyColor, centre: new THREE.Vector3(),
      camera: camera.position, time: nowS, wind: [cloudWind.x, cloudWind.y] as const, eclipse: 1, holeAmount: 0,
    })
    cloudsGlobalG.mesh.visible = !envHidden.has("earth") && !envHidden.has("clouds")
    // volumetric cumulus deck (clouds.ts): fixed in the pad frame, lit by the true Sun, dims the Sun
    // behind cloud; faded against the global shell by altitude and grown to a regional extent
    const cloudLocal = physicalSky && localK > 0.005 && !envHidden.has("clouds")
    if (cloudLocal) {
      clouds.update({
        sunDir: sunThree, sunColor: cloudSunColor, skyColor: cloudSkyColor,
        origin: pad.root.position, time: nowS, wind: [cloudWind.x, cloudWind.y] as const, eclipse: sunFrac,
        extent: localExtent, fade: localK,
      })
    } else {
      clouds.mesh.userData.sunTransmittance = 1
    }
    clouds.mesh.visible = cloudLocal
    coast.water.uniforms.uTime.value = nowS
    coast.water.uniforms.uSun.value.copy(sunThree)
    // background: studio, or sky (Preetham) fading to black space with stars
    if (physicalSky) {
      scene.background = black
      // stars fade in as the sky darkens (altitude or night) instead of switching at a threshold
      const starK = Math.max(0, Math.min(1, (0.12 - skyFade * dayK) / 0.1))
      ;(stars.material as THREE.PointsMaterial).color.setScalar(starK)
      stars.visible = starK > 0.01
      updateEnv(sunThree, alt, dayK, rn)
      scene.environment = skyEnv ? skyEnv.texture : null
      scene.environmentIntensity = 0.6
      renderer.toneMappingExposure = 0.6
    } else {
      scene.background = studio; stars.visible = false
      scene.environment = roomEnv
      scene.environmentIntensity = 1
      renderer.toneMappingExposure = 0.85
    }
    // direct sunlight: eclipse fraction × low-sun dimming near the ground (long slant path)
    sun.intensity = 3.0 * sunFrac * (alt > 30e3 ? 1 : Math.max(0.05, Math.min(1, sunThree.y * 3 + 0.2)))
    sun.intensity *= Number(clouds.mesh.userData.sunTransmittance ?? 1)
    hemi.intensity = (physicalSky ? 0.2 : 0.5) * Math.max(0.15, dayK) * (alt > 60e3 ? 0.4 : 1)
    // wind field: forecast profile + measured local offset (as the guidance uses it)
    const wE = v("windE", i, f), wN = v("windN", i, f)
    const m0 = meanWindEnu(alt, windP)
    // wind arrows fade in below 40 km (fully shown below 32 km) instead of popping in
    const windK = Math.max(0, Math.min(1, (40e3 - alt) / 8e3)) * Math.max(0, 1 - explode * 20)
    windField.setOpacity(windK)
    windField.mesh.visible = windOn && mode !== "globe" && windK > 0.01
    if (windField.mesh.visible) {
      const range = windField.update((_de, _dn, du) => {
        const m = meanWindEnu(Math.max(0, alt + du), windP)
        const kOfs = Math.exp(-Math.abs(du) / 3000)
        return [m.x + (wE - m0.x) * kOfs, m.y + (wN - m0.y) * kOfs, 0]
      })
      windMin = range.min
      windMax = range.max
    }
    // globe marker
    marker.position.copy(g(rx * Math.cos(th) + ry * Math.sin(th), -rx * Math.sin(th) + ry * Math.cos(th), rz))
    local.visible = mode !== "globe"
    globe.visible = mode === "globe"
    if (mode === "tower") {
      const camPos = enu(-pe - 350, -pn - 350, -pu + 60)
      camera.position.copy(camPos)
      camera.lookAt(0, 0, 0)
      camera.fov = Math.max(4, Math.min(60, (2 * Math.atan(80 / Math.max(camPos.length(), 1)) * 180) / Math.PI))
      camera.updateProjectionMatrix()
    } else if (mode === "globe") {
      controls.target.set(0, 0, 0)
      controls.update()
    } else {
      // chase/local: the target is wherever the user panned it (right-drag); do not re-centre on the
      // ship, or panning would snap back every frame. `setMode("chase")`/Home/Fit reset it.
      controls.update()
    }
    // hover pick among visible internal parts (label only what is under the cursor)
    const cands = ship.labels.map((l) => l.obj).filter((o) => o.visible)
    picker.setFromCamera(mouse, camera)
    const hit = mode === "chase" && cands.length ? picker.intersectObjects(cands, true)[0] : undefined
    let nh: THREE.Object3D | null = null
    if (hit) { let o: THREE.Object3D | null = hit.object; while (o && !cands.includes(o)) o = o.parent; nh = o }
    hovered = nh
    composer.render()
    updateCube?.(camera, controls.target)
    const w = canvas.clientWidth, h = canvas.clientHeight
    ship.labels.forEach((l, m) => {
      const el = labelEls[m]
      if (!el) return
      if (mode !== "chase" || !l.obj.visible || l.obj !== hovered) { el.style.display = "none"; return }
      el.style.display = "block"
      el.style.transform = `translate(${((mouse.x + 1) / 2) * w + 14}px, ${((1 - mouse.y) / 2) * h + 10}px)`
    })
  }
  const setMode = (m: CameraMode): void => {
    mode = m
    if (m === "chase") { camera.fov = 45; camera.position.set(-90, 40, 110); controls.target.set(0, 0, 0); controls.enabled = true }
    if (m === "globe") { camera.fov = 45; camera.position.copy(marker.position.clone().normalize().multiplyScalar(6371 + 17000)); controls.enabled = true }
    if (m === "tower") controls.enabled = false
    camera.updateProjectionMatrix()
  }
  const setExploded = (on: boolean): void => { explodeTarget = on ? 1 : 0 }
  const resize = (): void => {
    const w = canvas.clientWidth, h = canvas.clientHeight
    renderer.setSize(w, h, false)
    composer.setSize(w, h)
    camera.aspect = w / Math.max(h, 1)
    camera.updateProjectionMatrix()
  }
  const tree = (): readonly ViewTreeNode[] => [
    ...(ship ? ship.tree : []),
    { id: "path", label: "Paths: actual (cyan) · MPC plan (pale)", on: true },
    {
      id: "analysis", label: "Analysis views", children: [
        { id: "thermal", label: "Steel strength margin (heat map)", on: false },
        { id: "wind", label: "Wind field (ANSYS-style arrows)", on: true },
      ],
    },
    { id: "env", label: "Environment", children: [{ id: "pad", label: "Mechazilla · Pad 2 · Starbase coast" }, { id: "earth", label: "Earth (Blue Marble) & atmosphere" }, { id: "clouds", label: "Clouds (global shell + cumulus deck)" }, { id: "track", label: "Trajectory (globe)" }] },
  ]
  const setVisible = (id: string, on: boolean): void => {
    if (id === "path") { pathOn = on; pathOverlay.setVisible(on); return }
    if (id === "thermal") { thermalMode = on; return }
    if (id === "wind") { windOn = on; return }
    if (id === "analysis") { thermalMode = on; windOn = on; return }
    if (id === "env") { ["pad", "earth", "clouds", "track"].forEach((k) => setVisible(k, on)); return }
    if (id === "pad" || id === "earth" || id === "clouds" || id === "track") { if (on) envHidden.delete(id); else envHidden.add(id); return }
    ship?.setVisible(id, on)
  }
  const home = (): void => {
    if (mode === "globe") { camera.position.copy(marker.position.clone().normalize().multiplyScalar(6371 + 17000)); return }
    camera.position.copy(controls.target.clone().addScaledVector(HOME, fitDist))
    controls.update()
  }
  const fit = (): void => {
    fitDist = explode > 0.3 ? 210 : 120
    const dir = camera.position.clone().sub(controls.target).normalize()
    camera.position.copy(controls.target.clone().addScaledVector(dir, fitDist))
    controls.update()
  }
  return {
    setRecording, render, setMode, setExploded, resize, tree, setVisible, explodeProgress: () => explode,
    setPhysicalSky: (on) => { physicalSky = on }, home, fit,
    legends: () => ({ thermal: thermalMode, wind: windOn && windField.mesh.visible, windMin, windMax }),
  }
}
