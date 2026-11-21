import * as THREE from "three"
import type { VehicleConfig } from "@ship/craft"
import { THERMAL_LIMITS as TL, type ThermalGrid } from "@ship/heat"
import { blackbody, makePlasmaMaterial, makeShockLayerMaterial, makePlumeMaterial, makeShockMaterial, makeWakeMaterial } from "./fx"
import { ASM_LABEL, type Assembly, makeParts, makePicaGeometry, makePicaMaps, makeRaptor, MAT } from "./parts"

/**
 * The vehicle in BODY axes (x along the axis toward the nose, origin at the aft skirt plane).
 * Hull = the physics outer mould line (what is drawn is what is simulated). Detailed parts per
 * docs/03-spec/07-cad-model.md. Everything visual is driven by recorded channels via `update`.
 */
export interface ShipFrame {
  readonly heat: Float32Array | null // surface temperature per thermal cell (K)
  readonly thrust: readonly number[] // N per sea-level engine
  readonly gp: number
  readonly gy: number
  readonly rcsDuty: readonly number[] // 12 thrusters
  readonly coilFrac: number // I / Iop
  readonly electrodes: readonly number[] // 8 loadings
  readonly qStag: number // W/m²
  readonly ne: number // stagnation electron density, 1/m³
  readonly t0: number // stagnation temperature K
  readonly standoff: number // m (with the field, Δ = Δ₀(1+Q)^¼)
  readonly standoff0: number // m (no field, Δ₀ = 0.78 ε R_n)
  readonly heatFactor: number // (Δ0/Δ)^½ — MHD heat-flux factor
  readonly mach: number
  readonly qbar: number // Pa (continuum flow: shock layer exists)
  readonly vAirBody: THREE.Vector3 // unit air-velocity direction in body axes (flow comes from −vAir)
  readonly exploded: number // 0..1
  readonly pAmb: number // Pa
  readonly studio: boolean
  readonly thermalMode: boolean // false-colour steel strength margin instead of incandescence
  readonly time: number // wall-clock seconds (shader animation)
}

export interface TreeNode {
  readonly id: string
  readonly label: string
  readonly children?: readonly TreeNode[]
  /** initial visibility (default on) */
  readonly on?: boolean
}

export interface ShipModel {
  readonly root: THREE.Group
  readonly update: (f: ShipFrame) => void
  readonly labels: readonly { readonly text: string; readonly obj: THREE.Object3D; readonly id: string }[]
  readonly tree: readonly TreeNode[]
  readonly setVisible: (id: string, on: boolean) => void
  readonly plasmaLight: THREE.PointLight
}

/** incandescence colour for a surface temperature (black-body appearance, dim below ≈ 750 K) */
export const glowColor = (T: number, out = new THREE.Color()): THREE.Color => {
  if (T < 750) return out.setRGB(0, 0, 0)
  blackbody(T, out)
  const k = Math.min(1.6, ((T - 750) / 900) ** 2.2)
  return out.multiplyScalar(k)
}

/** 301/30X steel strength margin (spec/04 A4.8): blue cold → green nominal → yellow reduced → orange design limit → red strength loss → magenta failure */
const MARGIN_STOPS: readonly [number, number, number, number][] = [[300, 0.12, 0.55, 0.95], [TL.steelFull, 0.18, 0.8, 0.44], [TL.steelReduced, 0.95, 0.85, 0.2], [TL.steelDesign, 0.98, 0.5, 0.1], [TL.steelFail, 0.9, 0.12, 0.1], [1600, 0.85, 0.2, 0.85]]
export const marginColor = (T: number, out = new THREE.Color()): THREE.Color => {
  const stops = MARGIN_STOPS
  if (T <= stops[0][0]) return out.setRGB(stops[0][1], stops[0][2], stops[0][3])
  for (let i = 1; i < stops.length; i++) {
    const [t1, r1, g1, b1] = stops[i]
    if (T <= t1) {
      const [t0, r0, g0, b0] = stops[i - 1]
      const f = (T - t0) / (t1 - t0)
      return out.setRGB(r0 + (r1 - r0) * f, g0 + (g1 - g0) * f, b0 + (b1 - b0) * f)
    }
  }
  const s = stops[stops.length - 1]
  return out.setRGB(s[1], s[2], s[3])
}

const lathe = (pts: THREE.Vector2[], segs: number): THREE.BufferGeometry => {
  const g = new THREE.LatheGeometry(pts, segs)
  g.rotateZ(-Math.PI / 2) // lathe axis +Y → body +x
  return g
}

export const makeShip = (v: VehicleConfig, grid: ThermalGrid): ShipModel => {
  const root = new THREE.Group()
  const inner = new THREE.Group() // internal + exploded parts
  const shell = new THREE.Group() // hull, external parts, effects
  root.add(shell, inner)
  const L = v.shape.p.length
  const layers = new Map<string, THREE.Group>()
  const layer = (id: string): THREE.Group => {
    let g = layers.get(id)
    if (!g) { g = new THREE.Group(); g.name = id; shell.add(g); layers.set(id, g) }
    return g
  }
  const hidden = new Set<string>()
  // ---- hull profile (dense near the nose)
  const xs: number[] = []
  for (let i = 0; i <= 80; i++) xs.push((i / 80) * (L - 14))
  for (let i = 1; i <= 100; i++) xs.push(L - 14 + 14 * (1 - Math.cos((i / 100) * (Math.PI / 2))))
  const prof = xs.map((x) => new THREE.Vector2(Math.max(v.shape.r(Math.min(x, L - 1e-4)), 1e-3), x))
  prof.push(new THREE.Vector2(0, L))
  const SEG = 96
  const hullGeo = lathe(prof, SEG)
  const nV = hullGeo.attributes.position.count
  // map every vertex to its thermal cell
  const nRays = grid.cellPhi.length / grid.nSt
  const cellOf = new Int32Array(nV)
  const isPica = new Uint8Array(nV)
  const pos = hullGeo.attributes.position
  for (let i = 0; i < nV; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i)
    let st = 0, best = Infinity
    for (let s = 0; s < grid.nSt; s++) { const d = Math.abs(grid.xs[s] - x); if (d < best) { best = d; st = s } }
    const phi = (Math.atan2(z, y) + 2 * Math.PI) % (2 * Math.PI)
    cellOf[i] = st * nRays + (Math.round((phi / (2 * Math.PI)) * nRays) % nRays)
    isPica[i] = x >= v.pica.x0 ? 1 : 0
  }
  const baseCol = new Float32Array(nV * 3)
  hullGeo.setAttribute("color", new THREE.BufferAttribute(baseCol, 3))
  const hullMat = new THREE.MeshStandardMaterial({ vertexColors: true, metalness: 0.88, roughness: 0.3, transparent: true, side: THREE.DoubleSide })
  const hull = new THREE.Mesh(hullGeo, hullMat)
  hull.castShadow = true
  hull.receiveShadow = true
  layer("hull").add(hull)
  // PICA-X honeycomb cover on the nosecone only
  const picaGeo = makePicaGeometry(v.shape, v.pica.x0)
  const pm = makePicaMaps()
  const picaMat = new THREE.MeshStandardMaterial({ map: pm.map, normalMap: pm.normalMap, roughnessMap: pm.roughnessMap, roughness: 1, metalness: 0, transparent: true })
  const pica = new THREE.Mesh(picaGeo, picaMat)
  pica.castShadow = true
  layer("hull").add(pica)
  // incandescent glow layer (photoreal mode)
  const glowGeo = hullGeo.clone()
  const glowCol = new Float32Array(nV * 3)
  glowGeo.setAttribute("color", new THREE.BufferAttribute(glowCol, 3))
  const glowMat = new THREE.MeshBasicMaterial({ vertexColors: true, blending: THREE.AdditiveBlending, transparent: true, depthWrite: false })
  const glow = new THREE.Mesh(glowGeo, glowMat)
  glow.scale.setScalar(1.006)
  layer("heatGlow").add(glow)
  // catch pins are parts (structure › catchPins) at pinX / pinOut
  // ---- engines (detailed) + plumes
  const plumes: { mesh: THREE.Mesh; mat: THREE.ShaderMaterial; len: number; exitR: number }[] = []
  const slGroups: THREE.Group[] = []
  for (const e of v.engines) {
    const rp = makeRaptor(e.kind === "SL" ? "SL" : "RVac")
    const g = new THREE.Group()
    g.position.set(e.gimbal.x, e.gimbal.y, e.gimbal.z)
    g.add(rp.group)
    rp.group.traverse((o) => { if ((o as THREE.Mesh).isMesh) o.castShadow = true })
    if (e.kind === "SL") {
      const mat = makePlumeMaterial()
      // +Y of the cylinder is the far tail (the cone is rotated +90° about z, so +Y → −X): wide at
      // the tail and narrow at the nozzle exit, i.e. the jet expands downstream. The old radii were
      // reversed and the shader's axial coordinate ran from tail to nozzle, so the bright core and
      // Mach cells sat at the tip.
      const cone = new THREE.Mesh(new THREE.CylinderGeometry(rp.exitR * 2.2, rp.exitR * 0.9, 1, 32, 1, true), mat)
      cone.rotation.z = Math.PI / 2 // cylinder +Y → body −x (top at the exit)
      cone.renderOrder = 5
      g.add(cone)
      plumes.push({ mesh: cone, mat, len: rp.length, exitR: rp.exitR })
      slGroups.push(g)
    }
    layer("engines").add(g)
  }
  // ---- cold-gas puffs
  const puffs = v.thrusters.map((t) => {
    const m = new THREE.Mesh(new THREE.ConeGeometry(0.5, 3.5, 12, 1, true), new THREE.MeshBasicMaterial({ color: 0xcfe6ff, transparent: true, opacity: 0, depthWrite: false }))
    const d = new THREE.Vector3(-t.dir.x, -t.dir.y, -t.dir.z).normalize()
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, -1, 0), d)
    m.position.set(t.pos.x, t.pos.y, t.pos.z).addScaledVector(d, 1.75)
    layer("rcsJets").add(m)
    return m
  })
  // the recording stores the RCS duty at 25 Hz; applying it raw makes a pulsing thruster blink white
  // at frame rate. Low-pass it (~80 ms) so a jet reads as a steady plume, not a flicker.
  const rcsSm = new Float64Array(puffs.length)
  // ---- HTS coil + field lines
  const coil = v.coil
  const coilMat = MAT.rebco.clone()
  const coilMesh = new THREE.Mesh(new THREE.TorusGeometry(coil.a, Math.max(coil.wpRadial, 0.12) + 0.13, 16, 64), coilMat)
  coilMesh.rotation.y = Math.PI / 2
  coilMesh.position.x = coil.x
  layer("coil").add(coilMesh)
  const fieldMat = new THREE.LineBasicMaterial({ color: 0x59b8ff, transparent: true, opacity: 0, depthWrite: false })
  for (const Lr of [1.6, 2.6, 4.0, 6.0]) {
    for (let k = 0; k < 8; k++) {
      const phi = (k / 8) * Math.PI * 2
      const pts: THREE.Vector3[] = []
      for (let i = 1; i < 80; i++) {
        const th = (i / 80) * Math.PI
        const r = Lr * coil.a * Math.sin(th) ** 2
        pts.push(new THREE.Vector3(coil.x + r * Math.cos(th), r * Math.sin(th) * Math.cos(phi), r * Math.sin(th) * Math.sin(phi)))
      }
      layer("field").add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), fieldMat))
    }
  }
  // ---- electrode sectors on the cap
  const xEl = L - 0.9
  const rEl = v.shape.r(xEl)
  const electrodes = Array.from({ length: 8 }, (_, i) => {
    const phi = ((i + 0.5) / 8) * Math.PI * 2
    const m = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.9, 0.9), new THREE.MeshBasicMaterial({ color: 0x7fd0ff, transparent: true, opacity: 0, blending: THREE.AdditiveBlending, depthWrite: false }))
    m.position.set(xEl, (rEl + 0.05) * Math.cos(phi), (rEl + 0.05) * Math.sin(phi))
    m.rotation.x = phi
    layer("electrodes").add(m)
    return m
  })
  // ---- plasma: shock-layer sheath (shader), wake, light
  // shock-layer sheath: the OML offset outward (thickness scaled by the standoff below)
  // only the nose (last ≈ 8 m: cap + first ogive metres) carries a visible sheath; the barrel's thin boundary-layer
  // glow is invisible next to the shock layer and, drawn over 50 m of hull, it washed out the frame (2026-11-21)
  const sheathPts: THREE.Vector2[] = prof.slice(0, -1).filter((p) => p.y >= L - 8).map((p) => new THREE.Vector2(p.x + 0.3, p.y))
  sheathPts.push(new THREE.Vector2(0, L + 0.3))
  const plasmaMat = makePlasmaMaterial()
  const sheath = new THREE.Mesh(lathe(sheathPts, 64), plasmaMat)
  sheath.renderOrder = 6
  layer("plasma").add(sheath)
  const wakeMat = makeWakeMaterial()
  const wakeGeo = new THREE.CylinderGeometry(3.2, 9, 1, 48, 1, true).translate(0, -0.5, 0) // top at 0, extends to −1 (narrow trail)
  const wake = new THREE.Mesh(wakeGeo, wakeMat)
  wake.renderOrder = 6
  layer("plasma").add(wake)
  // shock-layer plasma ahead of the nose (the main visible plasma), on the bow-shock cap geometry
  const shockLayerMat = makeShockLayerMaterial()
  const shockLayer = new THREE.Mesh(new THREE.LatheGeometry((() => {
    const RnL = v.shape.p.noseRadius
    const pts: THREE.Vector2[] = []
    for (let i = 0; i <= 40; i++) { const r = (i / 40) * 1.6 * RnL; pts.push(new THREE.Vector2(Math.max(r, 1e-3), -(r * r) / (2 * 1.3 * RnL))) }
    return pts
  })(), 64), shockLayerMat)
  shockLayer.renderOrder = 6
  layer("plasma").add(shockLayer)
  // soft glow around the nose (what a camera sees of the shock-layer emission at a distance): radial sprite,
  // additive, HDR peak ≤ 1.6 (below the bloom threshold 2.5), size ∝ shock-layer brightness
  const haloTex = (() => {
    const c = document.createElement("canvas"); c.width = c.height = 128
    const g = c.getContext("2d")!
    const grd = g.createRadialGradient(64, 64, 0, 64, 64, 64)
    grd.addColorStop(0, "rgba(255,255,255,1)"); grd.addColorStop(0.25, "rgba(255,255,255,0.55)")
    grd.addColorStop(0.6, "rgba(255,255,255,0.12)"); grd.addColorStop(1, "rgba(255,255,255,0)")
    g.fillStyle = grd; g.fillRect(0, 0, 128, 128)
    return new THREE.CanvasTexture(c)
  })()
  const haloMat = new THREE.SpriteMaterial({ map: haloTex, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending })
  const halo = new THREE.Sprite(haloMat)
  halo.renderOrder = 8
  layer("plasma").add(halo)
  // air-plasma line emission (N₂⁺ first negative 391 nm violet + N₂ first positive red) reads pink-violet on
  // cameras; PICA-X ablation products (Na, CN) add orange. Blended with the black body of T0.
  const AIR_LINES = new THREE.Color(1.0, 0.42, 0.78)
  const plasmaCol = new THREE.Color()
  let plSm = 0, t0Sm = 0, shSm = 0, soSm = 0, so0Sm = 0, upInit = false // frame-smoothed plasma drivers
  const upSm = new THREE.Vector3(1, 0, 0), flowSm = new THREE.Vector3(1, 0, 0) // frame-smoothed plasma brightness
  const plasmaLight = new THREE.PointLight(0xff8040, 0, 60, 2)
  plasmaLight.position.set(L + 2, 0, 0)
  layer("plasma").add(plasmaLight)
  // ---- bow shock with the field (Δ) and the no-field ghost (Δ0 = Δ·f²)
  // local bow-shock cap (Billig 1967, sphere-cone): shock nose radius R_s = 1.143 R_n e^{0.54/(M-1)^1.2}
  // ≈ 1.16 R_n at M 20; drawn as a cap of lateral extent 2.6 R_n around the stagnation line only,
  // because the standoff is computed only there (heat/mhd stagnation model).
  const Rn = v.shape.p.noseRadius
  const Rs = 1.16 * Rn
  const shockPts: THREE.Vector2[] = []
  for (let i = 0; i <= 30; i++) { const r = (i / 30) * 2.6 * Rn; shockPts.push(new THREE.Vector2(r, -(r * r) / (2 * Rs))) }
  const shockMat = makeShockMaterial()
  const shock = new THREE.Mesh(new THREE.LatheGeometry(shockPts, 64), shockMat)
  shock.renderOrder = 7
  layer("shock").add(shock)
  // no-field shock Δ₀: dashed engineering outline (12 meridians + 3 rings) so both surfaces read at once
  const ghostPts: THREE.Vector3[] = []
  const par = (r: number, a: number): THREE.Vector3 => new THREE.Vector3(r * Math.cos(a), -(r * r) / (2 * Rs), r * Math.sin(a))
  const rMax = 2.6 * Rn
  for (let k = 0; k < 12; k++) { const a = (k / 12) * Math.PI * 2; for (let i = 0; i < 30; i++) ghostPts.push(par((i / 30) * rMax, a), par(((i + 1) / 30) * rMax, a)) }
  for (const r of [rMax / 3, (2 * rMax) / 3, rMax]) for (let k = 0; k < 72; k++) ghostPts.push(par(r, (k / 72) * Math.PI * 2), par(r, ((k + 1) / 72) * Math.PI * 2))
  const ghostMat = new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.25, gapSize: 0.18, transparent: true, opacity: 0, depthWrite: false })
  const ghost = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(ghostPts), ghostMat)
  ghost.computeLineDistances()
  ghost.renderOrder = 8
  layer("mhdGhost").add(ghost)
  // ---- detailed parts (spec/07)
  const parts = makeParts(v)
  const placed = parts.map((p) => {
    const home = p.obj.position.clone()
    if (p.internal) inner.add(p.obj)
    else layer(p.id).add(p.obj)
    p.obj.traverse((o) => { if ((o as THREE.Mesh).isMesh) { o.castShadow = true; o.receiveShadow = true } })
    return { ...p, home, target: home.clone().add(p.away) }
  })
  const labels = placed.filter((p) => p.internal).map((p) => ({ text: p.label, obj: p.obj, id: `p:${p.id}` }))
  const tmp = new THREE.Color()
  let lastHeat: Float32Array | null = null
  let lastMode: boolean | null = null
  let lastStudio: boolean | null = null
  const fxMats = (): THREE.Material[] => [...puffs.map((m) => m.material as THREE.Material), fieldMat]
  const update = (f: ShipFrame): void => {
    if (f.studio !== lastStudio) {
      lastStudio = f.studio
      for (const m of fxMats()) { m.blending = f.studio ? THREE.NormalBlending : THREE.AdditiveBlending; m.needsUpdate = true }
      shockMat.uniforms.uColor.value.set(f.studio ? 0x1f6fc0 : 0xd8ecff)
      ghostMat.color.set(f.studio ? 0x30363c : 0xffffff)
      puffs.forEach((m) => (m.material as THREE.MeshBasicMaterial).color.set(f.studio ? 0x9fb8cc : 0xe8f4ff))
      fieldMat.color.set(f.studio ? 0x1f7fd0 : 0x59b8ff)
    }
    // hull colour: steel / PICA base, or false-colour strength margin
    if (f.heat && (f.heat !== lastHeat || f.thermalMode !== lastMode)) {
      for (let i = 0; i < nV; i++) {
        const Th = f.heat[cellOf[i]]
        const T = Number.isFinite(Th) ? Th : 300
        if (f.thermalMode) marginColor(T, tmp)
        else if (isPica[i]) tmp.setRGB(0.07, 0.065, 0.06)
        else tmp.setRGB(0.72, 0.74, 0.77)
        baseCol[i * 3] = tmp.r; baseCol[i * 3 + 1] = tmp.g; baseCol[i * 3 + 2] = tmp.b
        glowColor(T, tmp)
        glowCol[i * 3] = tmp.r; glowCol[i * 3 + 1] = tmp.g; glowCol[i * 3 + 2] = tmp.b
      }
      hullGeo.attributes.color.needsUpdate = true
      glowGeo.attributes.color.needsUpdate = true
      hullMat.metalness = f.thermalMode ? 0.1 : 0.88
      hullMat.roughness = f.thermalMode ? 0.7 : 0.3
      pica.visible = !f.thermalMode
      glow.visible = !f.thermalMode && layer("hull").visible
      lastHeat = f.heat
      lastMode = f.thermalMode
    }
    // plumes (length grows as the ambient pressure falls: under-expanded at altitude)
    const pRatio = Math.min(f.pAmb / 101325, 1)
    const expand = 1 + 2.5 * (1 - pRatio)
    slGroups.forEach((g, i) => {
      g.rotation.set(0, f.gp, f.gy)
      const T = f.thrust[i] ?? 0
      const p = plumes[i]
      const len = (T / 2.45e6) * 32 * expand
      p.mesh.visible = T > 1e4 && !p.mesh.userData.off
      p.mesh.scale.set(1 + 0.5 * (expand - 1), len, 1 + 0.5 * (expand - 1))
      p.mesh.position.x = -p.len - len / 2
      p.mat.uniforms.uTime.value = f.time
      p.mat.uniforms.uPamb.value = pRatio
      p.mat.uniforms.uThrottle.value = Math.min(1, T / 1.5e6)
    })
    puffs.forEach((m, i) => {
      const target = Math.min(1, f.rcsDuty[i] ?? 0)
      rcsSm[i] += (target - rcsSm[i]) * 0.22
      ;(m.material as THREE.MeshBasicMaterial).opacity = Math.min(0.5, rcsSm[i] * 0.7)
    })
    coilMat.emissive.setRGB(0.15 * f.coilFrac, 0.45 * f.coilFrac, 1.0 * f.coilFrac)
    fieldMat.opacity = 0.35 * f.coilFrac * (1 - 0.7 * f.exploded)
    // plasma: emission ∝ n_e (compressed), colour = black body at T0, only in hypersonic flow
    // every driver is guarded: a single NaN from a channel would propagate through blackbody → additive
    // plasma → bloom and flash the whole screen white (the user's symptom)
    const ne = Number.isFinite(f.ne) ? f.ne : 0
    const mach = Number.isFinite(f.mach) ? f.mach : 0
    const plRaw = Math.max(0, Math.min(1, (Math.log10(Math.max(ne, 1)) - 17.0) / 4.0)) * Math.min(1, Math.max(0, (mach - 4) / 8))
    // the recording is 25 Hz; following its raw ne/mach/T0 per frame aliases into a 5–15 Hz white flicker.
    // Low-pass every driver (~0.15 s) and never snap: on a seek the glow fades in over ≈0.5 s.
    plSm += (plRaw - plSm) * 0.10
    const t0Raw = Number.isFinite(f.t0) ? f.t0 : 1500
    if (t0Sm === 0 || !Number.isFinite(t0Sm)) t0Sm = t0Raw
    t0Sm += (t0Raw - t0Sm) * 0.08
    const pl = plSm
    // the shock standoff and the flow direction also arrive at 25 Hz; using them raw slides the bright cap
    // back and forth every sample (visible as a flicker). Low-pass both.
    const soRaw = Number.isFinite(f.standoff) ? f.standoff : soSm
    const so0Raw = Number.isFinite(f.standoff0) ? f.standoff0 : so0Sm
    soSm += (soRaw - soSm) * 0.10
    so0Sm += (so0Raw - so0Sm) * 0.10
    const vb = f.vAirBody
    if (Number.isFinite(vb.x) && Number.isFinite(vb.y) && Number.isFinite(vb.z) && vb.lengthSq() > 1e-12) {
      flowSm.lerp(vb, 0.10).normalize()
      const upRaw = vb.clone().negate().normalize()
      if (!upInit) { upSm.copy(upRaw); upInit = true } else upSm.lerp(upRaw, 0.10).normalize()
    }
    electrodes.forEach((m, i) => { (m.material as THREE.MeshBasicMaterial).opacity = Math.min(0.9, (f.electrodes[i] ?? 0) * 0.55) * pl })
    blackbody(Math.max(t0Sm, 1500), plasmaCol)
    plasmaCol.lerp(AIR_LINES, 0.55)
    plasmaMat.uniforms.uColor.value.copy(plasmaCol)
    plasmaMat.uniforms.uIntensity.value = 0.5 * pl
    plasmaMat.uniforms.uTime.value = f.time
    plasmaMat.uniforms.uFlow.value.copy(flowSm)
    sheath.visible = pl > 0.01
    sheath.scale.set(1, 1 + soSm * 0.02, 1 + soSm * 0.02)
    const upstream = upSm
    wake.visible = pl > 0.02
    if (wake.visible) {
      // the wake streams downstream along +v̂_air … from the nose region backward past the vehicle
      wake.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), upstream)
      wake.position.set(L - 4, 0, 0)
      wake.scale.set(1, 120 * pl + 30, 1)
      wakeMat.uniforms.uIntensity.value = 0.18 * pl
      blackbody(Math.max(t0Sm * 0.8, 1500), wakeMat.uniforms.uColor.value)
      wakeMat.uniforms.uTime.value = f.time
    }
    plasmaLight.color.copy(plasmaMat.uniforms.uColor.value)
    plasmaLight.intensity = 450 * pl * pl // was 6000: lit the metallic hull above the bloom threshold
    // shocks: actual standoff Δ and the no-field Δ0 = Δ·f²
    // a bow shock needs continuum flow: fade in between q̄ ≈ 10 Pa (≈ 95 km) and ≈ 300 Pa (≈ 80 km)
    const cont = Math.max(0, Math.min(1, (Math.log10(Math.max(f.qbar, 1)) - 1) / 1.5))
    const shRaw = Math.max(0, Math.min(1, (mach - 2) / 6)) * cont
    shSm += (shRaw - shSm) * 0.10
    const sh = shSm
    shock.visible = sh > 0.01
    ghost.visible = sh > 0.01 && soSm > 1.01 * so0Sm
    const place = (m: THREE.Object3D, d: number): void => {
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), upstream)
      m.position.set(L, 0, 0).addScaledVector(upstream, Math.max(d, 0.3))
    }
    if (shock.visible) { place(shock, soSm); shockMat.uniforms.uOpacity.value = 0.6 * sh }
    shockLayer.visible = sh > 0.01 && pl > 0.01
    if (shockLayer.visible) {
      place(shockLayer, soSm)
      shockLayerMat.uniforms.uColor.value.copy(plasmaMat.uniforms.uColor.value)
      shockLayerMat.uniforms.uIntensity.value = 0.9 * pl * sh
      shockLayerMat.uniforms.uTime.value = f.time
    }
    halo.visible = shockLayer.visible
    if (halo.visible) {
      halo.position.set(L, 0, 0).addScaledVector(upstream, Math.max(soSm, 0.3) * 0.5)
      const sz = 6 + 12 * pl
      halo.scale.set(sz, sz, 1)
      haloMat.color.copy(plasmaCol).multiplyScalar(0.9 * pl * sh)
    }
    if (ghost.visible) { place(ghost, so0Sm); ghostMat.opacity = 0.45 * sh }
    // exploded view and internal visibility
    const e = f.exploded
    const hullOn = layer("hull").visible
    hullMat.opacity = 1 - 0.85 * e
    picaMat.opacity = 1 - 0.85 * e
    hullMat.depthWrite = e < 0.5
    glowMat.opacity = 1 - e
    glow.visible = !f.thermalMode && hullOn && e < 0.98
    placed.forEach((p) => {
      if (p.internal) p.obj.visible = !hidden.has(`p:${p.id}`) && (e > 0.02 || !hullOn)
      p.obj.position.lerpVectors(p.home, p.target, p.internal ? e : 0)
    })
  }
  const asms: Assembly[] = ["structure", "propulsion", "rcs", "tps", "mhd", "avionics"]
  const tree: TreeNode[] = [
    {
      id: "ship", label: "Ship 42 (flapless HTS-MHD)", children: [
        { id: "hull", label: "Outer mould line — 301/30X steel + PICA-X honeycomb nosecone" },
        { id: "engines", label: "21/22 Raptor 3 ×3 SL + ×3 Vac" },
        { id: "coil", label: "51 HTS coil (REBCO)" },
        ...asms.map((a) => ({ id: `a:${a}`, label: ASM_LABEL[a], children: placed.filter((p) => p.asm === a).map((p) => ({ id: p.internal ? `p:${p.id}` : p.id, label: p.label })) })),
      ],
    },
    {
      id: "overlays", label: "Physics overlays", children: [
        { id: "plasma", label: "Plasma (sheath, wake, light)" }, { id: "shock", label: "Bow shock (with field, Δ)" }, { id: "mhdGhost", label: "Bow shock without field (Δ₀)" },
        { id: "field", label: "Magnetic field lines" }, { id: "electrodes", label: "Electrode sectors (loading)" }, { id: "heatGlow", label: "Steel incandescence" },
        { id: "plumes", label: "Engine plumes" }, { id: "rcsJets", label: "Cold-gas jets" },
      ],
    },
  ]
  const setVisible = (id: string, on: boolean): void => {
    if (id === "plumes") { plumes.forEach((p) => (p.mesh.userData.off = !on)); return }
    if (id.startsWith("p:")) { if (on) hidden.delete(id); else hidden.add(id); return }
    if (id.startsWith("a:")) { placed.filter((p) => `a:${p.asm}` === id).forEach((p) => setVisible(p.internal ? `p:${p.id}` : p.id, on)); return }
    if (id === "ship") { ["hull", "engines", "coil", ...asms.map((a) => `a:${a}`)].forEach((k) => setVisible(k, on)); return }
    if (id === "overlays") { ["plasma", "shock", "mhdGhost", "field", "electrodes", "heatGlow", "plumes", "rcsJets"].forEach((k) => setVisible(k, on)); return }
    const g = layers.get(id)
    if (g) g.visible = on
  }
  return { root, update, labels, tree, setVisible, plasmaLight }
}
