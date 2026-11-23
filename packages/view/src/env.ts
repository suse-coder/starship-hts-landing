import * as THREE from "three"
import { Sky } from "three/addons/objects/Sky.js"
import { ATM, ATMOSPHERE_GLSL } from "./atmosphere"
import { bakeTerrainNoise, bakeWaves } from "./procedural"

/**
 * Environment: single-scattering atmosphere (sky, aerial perspective, limb) with the Sun at its true
 * position, Blue Marble Earth (NASA Visible Earth, public domain, bundled), starfield for space, and
 * the Starbase coast: wind-tidal flats, scrub and lagoons west of a straight barrier beach, Gulf
 * water to the east. All shaders include the logarithmic-depth chunks used by the renderer.
 */

/**
 * Preetham/Hosek-style analytic sky (three/addons Sky) seen from the vehicle. Above the ground the
 * scattered radiance scales with the air column overhead, ∝ exp(−h/H) with the Rayleigh scale height
 * H ≈ 8 km: the sky fades through deep blue to black space (≈ 3 % at 30 km, < 0.1 % at 60 km).
 * Clouds (shader cloud deck) exist only below the deck (≈ 2–4 km) and fade out above it.
 */
export const makeSky = (): { sky: Sky; setSun: (dirThree: THREE.Vector3, altitude: number, time: number) => number } => {
  const sky = new Sky()
  sky.scale.setScalar(4.0e5)
  sky.renderOrder = -10
  const m = sky.material as THREE.ShaderMaterial
  m.depthTest = false
  m.depthWrite = false
  m.uniforms.uFade = { value: 1 }
  m.fragmentShader = m.fragmentShader
    .replace("uniform float time;", "uniform float time;\n\t\tuniform float uFade;")
    // clamp the HDR sky (the Sun disc is ≈ 10³ × the sky) so bloom gives a tight glow, not a halo
    .replace("gl_FragColor = vec4( texColor, 1.0 );", "gl_FragColor = vec4( min( texColor, vec3( 3.0 ) ) * uFade, 1.0 );")
  const u = m.uniforms
  u.turbidity.value = 2.6
  u.rayleigh.value = 1.2
  u.mieCoefficient.value = 0.003
  u.mieDirectionalG.value = 0.74
  u.cloudCoverage.value = 0 // the flat sky-dome cloud layer is replaced by the volumetric deck (clouds.ts)
  const setSun = (dir: THREE.Vector3, altitude: number, time: number): number => {
    u.sunPosition.value.copy(dir)
    const fade = Math.exp(-Math.max(altitude, 0) / 8000)
    u.uFade.value = fade
    u.time.value = time
    sky.visible = fade > 2e-4
    return fade
  }
  return { sky, setSun }
}

export const makeStars = (): THREE.Points => {
  const n = 4000, pos = new Float32Array(n * 3), col = new Float32Array(n * 3)
  // Far outside both the local Earth (R = 6371 km) and the 17 000 km globe camera, but inside the
  // camera far plane (5e8 m): the Earth's opaque disc then covers every star line of sight at every
  // altitude, in the chase view and in globe view. At the old 300 km radius the whole star sphere
  // sat inside the globe model, so the globe view had no stars above the limb.
  for (let i = 0; i < n; i++) {
    const u = Math.random() * 2 - 1, th = Math.random() * Math.PI * 2, r = 2.0e8
    const s = Math.sqrt(1 - u * u)
    pos.set([r * s * Math.cos(th), r * u, r * s * Math.sin(th)], i * 3)
    const b = 0.5 + Math.random() * 0.5
    col.set([b, b, b * (0.9 + Math.random() * 0.2)], i * 3)
  }
  const g = new THREE.BufferGeometry()
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3))
  g.setAttribute("color", new THREE.BufferAttribute(col, 3))
  // Opaque, not transparent: three.js draws every transparent object after the opaque Earth, so a
  // transparent starfield paints over the planet whenever the Earth writes no depth (near the surface).
  // In the opaque pass the stars draw first (renderOrder −9) and the Earth covers them. The altitude
  // fade is done by scaling the material colour, which still multiplies the per-star vertex colours.
  // fog: false — the scene fog carries the aerial perspective of solid objects near the pad; the
  // stars are dimmed by the atmosphere shell itself (its alpha is 1 − path transmittance)
  const p = new THREE.Points(g, new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true, depthWrite: false, fog: false }))
  p.renderOrder = -9
  return p
}

export interface Atmosphere {
  readonly mesh: THREE.Mesh
  /** sun direction (world, unit), Earth centre (world), camera position (world), in-scatter weight 0..1 */
  readonly update: (sunWorld: THREE.Vector3, centre: THREE.Vector3, cameraPos: THREE.Vector3, weight: number) => void
}

const RE_KM = ATM.RE, RA_KM = ATM.RA

/**
 * Single-scattering atmosphere (model and constants: atmosphere.ts). This is THE sky at every
 * altitude (ground, inside, orbit) — one model, no handover — and also the aerial perspective of
 * whatever lies behind it without writing depth (the Earth, the local coast planes, the planet-wide
 * cloud shell): the shell is blended as  dst·T̄ + L  with T̄ the luminance-weighted transmittance of
 * the view path and L the in-scattered radiance, so distant ground fades into exactly the colour the
 * sky has at the horizon. Optional Sun disc (0.267° radius, limb darkened) attenuated by the
 * view-path transmittance. `kmPerUnit`: world units → km (1e-3 metre frame, 1 globe).
 */
export const makeAtmosphere = (radius: number, kmPerUnit: number, sunDisc: boolean): Atmosphere => {
  const mat = new THREE.ShaderMaterial({
    uniforms: {
      uSun: { value: new THREE.Vector3(1, 0, 0) }, uCenter: { value: new THREE.Vector3() }, uKm: { value: kmPerUnit }, uWeight: { value: 1 },
      uSunI: { value: ATM.SUN_I }, uDisc: { value: sunDisc ? 1 : 0 }, uDiscI: { value: 8 },
    },
    vertexShader: /* glsl */ `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      varying vec3 vW;
      void main(){ vec4 w = modelMatrix*vec4(position,1.0); vW = w.xyz; gl_Position = projectionMatrix*viewMatrix*w;
        #include <logdepthbuf_vertex>
      }`,
    fragmentShader: /* glsl */ `
      #include <common>
      #include <logdepthbuf_pars_fragment>
      uniform vec3 uSun; uniform vec3 uCenter; uniform float uKm; uniform float uWeight; uniform float uSunI; uniform float uDisc; uniform float uDiscI;
      varying vec3 vW;
      ${ATMOSPHERE_GLSL}
      void main(){
        #include <logdepthbuf_fragment>
        vec3 o = (cameraPosition - uCenter) * uKm;
        vec3 d = normalize(vW - cameraPosition);
        vec2 a = atmRS(o, d, ATM_RA);
        if (a.x > a.y || a.y < 0.0) discard;
        float t0 = max(a.x, 0.0), t1 = a.y;
        vec2 e = atmRS(o, d, ATM_RE);
        bool hitsEarth = e.x > 0.0 && e.x < e.y;
        if (hitsEarth) t1 = min(t1, e.x);
        if (t1 <= t0) discard;
        vec3 trans;
        vec3 col = uSunI * atmScatter(o, d, t0, t1, uSun, a.x < 0.0 ? 1.0 : 0.0, trans);
        vec3 disc = vec3(0.0);
        if (uDisc > 0.5 && !hitsEarth) {
          // solar disc, angular radius 0.267° (cos = 0.9999891); x = (θ/θ☉)² for small angles;
          // limb darkening I(μ)/I(1) = 0.4 + 0.6 μ with μ = √(1 − x); seen through the path transmittance
          float x = clamp((1.0 - dot(d, uSun)) / 1.09e-5, 0.0, 1.0);
          disc = trans * ((1.0 - smoothstep(0.8, 1.0, x)) * (0.4 + 0.6 * sqrt(1.0 - x)) * uDiscI);
        }
        float alpha = 1.0 - dot(trans, vec3(0.2126, 0.7152, 0.0722));
        gl_FragColor = vec4((min(col, vec3(${ATM.SKY_MAX.toFixed(1)})) + disc) * uWeight, clamp(alpha, 0.0, 1.0) * uWeight);
      }`,
    // premultiplied "over": dst·(1 − α) + src with α = 1 − T̄ (extinction of what is behind + in-scatter)
    transparent: true, depthWrite: false,
    blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor, blendDst: THREE.OneMinusSrcAlphaFactor,
  })
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius * (RA_KM / RE_KM), 160, 80), mat)
  // after the global cloud shell (−4) and the local cloud shadows (−2): its in-scatter is in front of
  // the cloud tops and of the shadowed ground; before the local cloud deck (−1), which carries its
  // own aerial perspective
  mesh.renderOrder = -1.5
  mesh.frustumCulled = false
  const update: Atmosphere["update"] = (sunWorld, centre, cameraPos, weight) => {
    mat.uniforms.uSun.value.copy(sunWorld).normalize()
    mat.uniforms.uCenter.value.copy(centre)
    mat.uniforms.uWeight.value = weight
    // outside the shell: front faces; inside: back faces (every view ray then has a fragment)
    const inside = cameraPos.distanceTo(centre) * kmPerUnit < RA_KM
    const side = inside ? THREE.BackSide : THREE.FrontSide
    if (mat.side !== side) { mat.side = side; mat.needsUpdate = true }
  }
  return { mesh, update }
}

export interface EarthModel {
  readonly earth: THREE.Mesh
  readonly atmosphere: THREE.Mesh
  /**
   * per frame: sun direction (world), Earth centre (world), camera, in-scatter weight. `atmCentre`
   * (default: `centre`) is the centre of the sphere the atmosphere is integrated over: near the pad
   * the caller passes the sphere tangent to the local ground, so the aerial perspective of the coast
   * planes ends on the ground that is actually drawn (the WGS84 pad lies ≈ 3 km above the mean sphere).
   */
  readonly update: (sunWorld: THREE.Vector3, centre: THREE.Vector3, camera: THREE.Camera, weight: number, atmCentre?: THREE.Vector3) => void
}

/**
 * Earth: Blue Marble day map, Black Marble 2016 city lights on the night side (soft terminator), and the
 * single-scattering atmosphere above. Open water (dark, blue-dominant texels of the day map) is given a
 * glossier microsurface (roughness 0.48, i.e. GGX α ≈ 0.23: the Cox–Munk slope variance of a breezy sea
 * seen from far away), which produces the broad sun glint on the ocean from orbit; its peak stays below
 * the bloom threshold. Local axes are ECEF-aligned through
 * EARTH_GEOM_TO_ECEF. Inside the atmosphere the surface writes no depth (local ground, ship and clouds
 * always draw over it).
 */
export const makeEarth = (radius: number, dayUrl: string, nightUrl: string, kmPerUnit: number): EarthModel => {
  const loader = new THREE.TextureLoader()
  const day = loader.load(dayUrl)
  day.colorSpace = THREE.SRGBColorSpace
  day.anisotropy = 8
  const night = loader.load(nightUrl)
  night.colorSpace = THREE.SRGBColorSpace
  // fog: false — the atmosphere shell is the aerial perspective of the planet
  const mat = new THREE.MeshStandardMaterial({ map: day, roughness: 0.92, metalness: 0, emissive: 0xffd9a0, emissiveMap: night, emissiveIntensity: 1.6, fog: false })
  const uSunView = { value: new THREE.Vector3(1, 0, 0) }
  mat.onBeforeCompile = (sh) => {
    sh.uniforms.uSunView = uSunView
    sh.fragmentShader = sh.fragmentShader
      .replace("#include <common>", "#include <common>\nuniform vec3 uSunView;\nfloat earthOcean;")
      .replace("#include <map_fragment>", `#include <map_fragment>
        earthOcean = smoothstep(1.3, 2.2, diffuseColor.b / max(diffuseColor.r, 1e-4))
          * (1.0 - smoothstep(0.06, 0.14, dot(diffuseColor.rgb, vec3(0.2126, 0.7152, 0.0722))));`)
      .replace("#include <roughnessmap_fragment>", "#include <roughnessmap_fragment>\n  roughnessFactor = mix(roughnessFactor, 0.48, earthOcean);")
      .replace("#include <emissivemap_fragment>", "#include <emissivemap_fragment>\n  totalEmissiveRadiance *= 1.0 - smoothstep(-0.14, 0.06, dot(normal, uSunView));")
  }
  mat.customProgramCacheKey = () => "earth-v2"
  const earth = new THREE.Mesh(new THREE.SphereGeometry(radius, 256, 128), mat)
  earth.renderOrder = -5
  const atm = makeAtmosphere(radius, kmPerUnit, true)
  const update: EarthModel["update"] = (sunWorld, centre, camera, weight, atmCentre) => {
    uSunView.value.copy(sunWorld).transformDirection(camera.matrixWorldInverse)
    atm.update(sunWorld, atmCentre ?? centre, camera.position, weight)
    mat.depthWrite = camera.position.distanceTo(centre) * kmPerUnit >= RA_KM
  }
  return { earth, atmosphere: atm.mesh, update }
}

/** geometry→ECEF basis of the Earth mesh (SphereGeometry: poles on +y, u = 0 at lon −180°) */
export const EARTH_GEOM_TO_ECEF = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1)

const noiseCanvas = (w: number, h: number, paint: (g: CanvasRenderingContext2D) => void): THREE.CanvasTexture => {
  const c = document.createElement("canvas"); c.width = w; c.height = h
  paint(c.getContext("2d")!)
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = THREE.SRGBColorSpace
  t.anisotropy = 8
  return t
}

/** metre-scale ground detail (grass tufts, pebbles): modulates the procedural ground cover near the camera */
const groundTexture = (): THREE.CanvasTexture => noiseCanvas(512, 512, (g) => {
  g.fillStyle = "#9b9168"; g.fillRect(0, 0, 512, 512)
  for (let i = 0; i < 9000; i++) {
    const x = Math.random() * 512, y = Math.random() * 512, r = 1 + Math.random() * 5
    const k = Math.random()
    g.fillStyle = k < 0.45 ? `rgba(92,104,64,${0.25 + Math.random() * 0.35})` : k < 0.8 ? `rgba(176,164,124,${0.2 + Math.random() * 0.3})` : `rgba(70,74,52,${0.2 + Math.random() * 0.3})`
    g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill()
  }
})

/** lighting of the coast for one frame (all colours linear RGB in scene radiometric units) */
export interface CoastLight {
  /** unit direction to the Sun, world frame */
  readonly sunDir: THREE.Vector3
  /** direct solar irradiance at sea level on a surface facing the Sun (light colour × intensity) */
  readonly sunColor: THREE.Color
  /** sky radiance at the zenith */
  readonly skyZenith: THREE.Color
  /** sky radiance just above the horizon, 90° in azimuth from the Sun */
  readonly skyHorizon: THREE.Color
  /** extra horizon radiance towards the Sun's azimuth (aerosol aureole), added to skyHorizon there */
  readonly skySun: THREE.Color
  /** cosine-weighted mean radiance of the sky (diffuse irradiance = π × this) */
  readonly skyAmbient: THREE.Color
  /** seconds (wave animation) */
  readonly time: number
}

export interface Coast {
  readonly root: THREE.Group
  readonly water: THREE.ShaderMaterial
  /** 1 = opaque, → 0 fades the local planes out over the Earth sphere (25–45 km) */
  readonly setFade: (k: number) => void
  /** Sun, sky and time for the water shader */
  readonly setLight: (l: CoastLight) => void
}

const WAVE_N = 256
const TERR_N = 256

const halfFloats = (a: Float32Array): Uint16Array => {
  const h = new Uint16Array(a.length)
  for (let i = 0; i < a.length; i++) h[i] = THREE.DataUtils.toHalfFloat(a[i])
  return h
}

/**
 * Ground cover of the Rio Grande delta around Starbase, evaluated per fragment in the pad frame
 * (+x east, +z south, metres). Four octaves of baked noise (47 m, 410 m, 3.7 km, 29 km, mutually
 * rotated, so nothing repeats visibly) give a relative-elevation proxy; its thresholds place
 *   lagoons and tidal ponds (lowest ≈ 11 % of the area: dark, smooth water that glints in the Sun),
 *   wind-tidal flats (bare mud, algal mats, salt crust on the dry parts),
 *   coastal prairie and thorn scrub on the higher clay "lomas",
 *   foredune sand behind the beach, and graded fill around the launch site.
 * Albedos are linear reflectances (dry sand ≈ 0.4, mud ≈ 0.13, scrub ≈ 0.09, water ≈ 0.05).
 */
const groundAlbedo = (shoreE: number): string => /* glsl */ `
  {
    vec2 gp = vCoastW.xz - uCoastPad;
    float dSh = ${shoreE.toFixed(1)} - gp.x;              // m inland of the dune line
    float rPad = length(gp);
    float n0 = texture2D(uTerr, gp * (1.0 / 47.0)).r;
    float n1 = texture2D(uTerr, mat2(0.8, 0.6, -0.6, 0.8) * gp * (1.0 / 410.0) + 0.37).g;
    float n2 = texture2D(uTerr, mat2(0.6, -0.8, 0.8, 0.6) * gp * (1.0 / 3700.0) + 0.11).b;
    float n3 = texture2D(uTerr, mat2(0.96, 0.28, -0.28, 0.96) * gp * (1.0 / 29000.0) + 0.71).a;
    float elev = 0.50 * n3 + 0.32 * n2 + 0.18 * n1;
    float bare = 1.0 - smoothstep(160.0, 520.0, rPad);
    float dune = 1.0 - smoothstep(40.0, 320.0, dSh);
    float water = (1.0 - smoothstep(0.275, 0.31, elev)) * (1.0 - bare) * smoothstep(250.0, 700.0, dSh) * smoothstep(700.0, 1300.0, rPad);
    float wet = 1.0 - smoothstep(0.31, 0.40, elev);
    float veg = smoothstep(0.50, 0.62, elev + 0.22 * (n1 - 0.5) + 0.10 * (n0 - 0.5)) * (1.0 - bare);
    vec3 flatC = mix(vec3(0.34, 0.31, 0.25), vec3(0.52, 0.50, 0.45), smoothstep(0.55, 0.80, n1) * (1.0 - wet));
    flatC = mix(flatC, vec3(0.15, 0.135, 0.105), 0.85 * wet);
    vec3 vegC = mix(vec3(0.23, 0.215, 0.12), vec3(0.085, 0.115, 0.05), smoothstep(0.35, 0.70, 0.6 * n1 + 0.4 * n0));
    vec3 alb = mix(flatC, vegC, veg);
    alb = mix(alb, vec3(0.50, 0.43, 0.31), dune * (1.0 - 0.6 * veg * step(0.5, n0)));
    alb = mix(alb, vec3(0.40, 0.36, 0.28), bare);
    alb *= 0.88 + 0.24 * n0;
    alb = mix(alb, vec3(0.035, 0.065, 0.06), water);
    // the painted detail map modulates the cover around its mean luminance; it fades out with range,
    // so its 250 m repeat never shows from the air
    float kDet = (1.0 - smoothstep(120.0, 900.0, distance(vCoastW, cameraPosition))) * (1.0 - water);
    float det = clamp(dot(diffuseColor.rgb, vec3(0.2126, 0.7152, 0.0722)) / 0.24, 0.7, 1.35);
    diffuseColor.rgb = alb * mix(1.0, det, kDet);
    coastWet = max(water, 0.6 * wet * (1.0 - veg) * (1.0 - bare));
  }`

/** beach: dry sand at the dune line, darker and smoother in the swash zone at the waterline */
const beachAlbedo = (shoreE: number): string => /* glsl */ `
  {
    vec2 gp = vCoastW.xz - uCoastPad;
    float bx = gp.x - ${shoreE.toFixed(1)};               // 0 at the dune line → 90 m at the waterline
    float n0 = texture2D(uTerr, gp * (1.0 / 31.0)).r;
    float n1 = texture2D(uTerr, gp.yx * (1.0 / 260.0) + 0.43).g;
    float wetS = smoothstep(58.0 + 10.0 * (n1 - 0.5), 86.0, bx);
    diffuseColor.rgb *= (0.90 + 0.20 * n0) * mix(1.0, 0.52, wetS);
    coastWet = 0.75 * wetS;
  }`

/** concrete landing-pad disc: metre-scale staining only */
const padAlbedo = /* glsl */ `
  {
    diffuseColor.rgb *= 0.90 + 0.20 * texture2D(uTerr, (vCoastW.xz - uCoastPad) * (1.0 / 23.0)).r;
    coastWet = 0.0;
  }`

/**
 * Ground (west of the shoreline), beach strip, and Gulf water.
 *
 * The planes never write depth: they are painted back to front before everything else that is
 * transparent, and the atmosphere shell (makeAtmosphere) then lays its in-scatter and extinction
 * over them. The far ground therefore dissolves into the horizon haze of the same sky model instead
 * of ending in a hard edge, and from altitude the coast is seen through the air column below.
 *
 * Water (per fragment, pad frame): three layers of one baked slope patch (19 m, 83 m, 470 m tiles,
 * travelling shoreward at their deep-water phase speeds c = √(gλ/2π)); the slope variance that the
 * mip chain filters out at range is returned to the BRDF as GGX roughness α² (Cox & Munk 1954: total
 * mean-square slope 0.003 + 0.00512·U ≈ 0.032 at U = 5.5 m/s), so the sun glitter keeps its energy
 * and widens with distance instead of aliasing. Reflection: mean Fresnel of a rough sea surface
 * (Bruneton et al. 2010) × the sea-level sky (zenith / horizon / solar aureole from the atmosphere
 * model); refraction: Lambertian water-leaving radiance, albedo from deep Gulf blue to the turbid
 * surf zone, × the surface irradiance. Surf: breaking lines running up the beach and a swash edge.
 */
export const makeCoast = (shoreE = 950): Coast => {
  const root = new THREE.Group()
  const gtex = groundTexture()
  // 300 km planes: from 10 km altitude the horizon is ≈ 360 km away, the flat-plane sag at 150 km is
  // ≈ 1.8 km (hidden by haze); beyond that the Earth sphere takes over. The planes are alpha-faded to
  // the Earth before their 150 km north/south ends (the old hard edge read as a straight seam at
  // 20–30 km) and the far east/west ends; see `coastPatch` below.
  const W = 300e3
  const GRID = 256 // subdivisions: one 300 km quad fails the depth test against objects a few metres tall
  gtex.repeat.set(W / 250, W / 250)
  const terrTex = new THREE.DataTexture(bakeTerrainNoise(TERR_N, 11), TERR_N, TERR_N, THREE.RGBAFormat)
  terrTex.wrapS = terrTex.wrapT = THREE.RepeatWrapping
  terrTex.magFilter = THREE.LinearFilter
  terrTex.minFilter = THREE.LinearMipmapLinearFilter
  terrTex.generateMipmaps = true
  terrTex.anisotropy = 8
  terrTex.needsUpdate = true
  const groundMat = new THREE.MeshStandardMaterial({ map: gtex, roughness: 0.96 })
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(W, W, GRID, GRID), groundMat)
  ground.rotation.x = -Math.PI / 2
  ground.position.set(shoreE - W / 2, -0.05, 0)
  ground.receiveShadow = true
  const beachMat = new THREE.MeshStandardMaterial({ color: 0xc2b592, roughness: 0.95 })
  const beach = new THREE.Mesh(new THREE.PlaneGeometry(90, W, 2, GRID), beachMat)
  beach.rotation.x = -Math.PI / 2
  beach.position.set(shoreE + 45, -0.04, 0)
  beach.receiveShadow = true
  const padMat = new THREE.MeshStandardMaterial({ color: 0xbab6aa, roughness: 0.95 })
  const pad = new THREE.Mesh(new THREE.CylinderGeometry(75, 75, 0.4, 64), padMat)
  pad.position.y = 0.1
  pad.receiveShadow = true
  // Pad-frame point the local planes fade around (updated from the root each frame), and the fade
  // band: pad-scale detail survives, the hard 150 km north/south ends and 300 km east/west ends
  // dissolve into the Earth sphere before they can be seen as a straight seam at altitude.
  const coastPad = { value: new THREE.Vector2() }
  const coastFadeRange = { value: new THREE.Vector2(95e3, 140e3) }
  /**
   * Patch a standard material into a coast plane: procedural ground cover (albedo + wetness →
   * roughness; standing water gets roughness 0.3 — wind-ruffled, so its sun glint is a broad sheen
   * that stays under the bloom threshold instead of a mirror flash), radial alpha-fade towards the Earth sphere, no depth write, no scene fog (the
   * atmosphere shell is its aerial perspective).
   */
  const coastPatch = (m: THREE.MeshStandardMaterial, kind: "ground" | "beach" | "pad"): void => {
    const albedo = kind === "ground" ? groundAlbedo(shoreE) : kind === "beach" ? beachAlbedo(shoreE) : padAlbedo
    m.transparent = true
    m.depthWrite = false
    m.fog = false
    m.onBeforeCompile = (sh) => {
      sh.uniforms.uCoastPad = coastPad
      sh.uniforms.uCoastFade = coastFadeRange
      sh.uniforms.uTerr = { value: terrTex }
      sh.vertexShader = sh.vertexShader
        .replace("#include <common>", "#include <common>\nvarying vec3 vCoastW;")
        .replace("#include <begin_vertex>", "#include <begin_vertex>\nvCoastW = (modelMatrix * vec4(transformed, 1.0)).xyz;")
      sh.fragmentShader = sh.fragmentShader
        .replace("#include <common>", "#include <common>\nvarying vec3 vCoastW;\nuniform vec2 uCoastPad;\nuniform vec2 uCoastFade;\nuniform sampler2D uTerr;\nfloat coastWet;")
        .replace("#include <map_fragment>", `#include <map_fragment>\n${albedo}`)
        .replace("#include <roughnessmap_fragment>", "#include <roughnessmap_fragment>\n  roughnessFactor = mix(roughnessFactor, 0.3, coastWet);")
        .replace("#include <opaque_fragment>", `#include <opaque_fragment>
          gl_FragColor.a *= 1.0 - smoothstep(uCoastFade.x, uCoastFade.y, distance(vCoastW.xz, uCoastPad));`)
    }
    m.customProgramCacheKey = () => `coast-${kind}-v2`
  }
  coastPatch(groundMat, "ground")
  coastPatch(beachMat, "beach")
  coastPatch(padMat, "pad")
  const waves = bakeWaves(WAVE_N, 7)
  const waveTex = new THREE.DataTexture(halfFloats(waves.data), WAVE_N, WAVE_N, THREE.RGBAFormat, THREE.HalfFloatType)
  waveTex.wrapS = waveTex.wrapT = THREE.RepeatWrapping
  waveTex.magFilter = THREE.LinearFilter
  waveTex.minFilter = THREE.LinearMipmapLinearFilter
  waveTex.generateMipmaps = true
  waveTex.needsUpdate = true
  const NRET = waves.retained.length
  const water = new THREE.ShaderMaterial({
    uniforms: {
      uTime: { value: 0 }, uSun: { value: new THREE.Vector3(0, 1, 0) }, uOpacity: { value: 1 },
      uSunColor: { value: new THREE.Color(3, 2.8, 2.4) }, uSkyZenith: { value: new THREE.Color(0.08, 0.14, 0.27) },
      uSkyHorizon: { value: new THREE.Color(0.4, 0.55, 0.58) }, uSkySun: { value: new THREE.Color(0.6, 0.6, 0.5) },
      uIrr: { value: new THREE.Color(2.3, 2.4, 2.7) },
      uWaves: { value: waveTex }, uRet: { value: Array.from(waves.retained) }, uWaveOff: { value: new THREE.Vector3() }, uSurf: { value: 0 },
      uCoastPad: coastPad, uCoastFade: coastFadeRange,
    },
    vertexShader: /* glsl */ `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      varying vec3 vW;
      void main(){ vec4 w = modelMatrix*vec4(position,1.0); vW = w.xyz; gl_Position = projectionMatrix*viewMatrix*w;
        #include <logdepthbuf_vertex>
      }`,
    fragmentShader: /* glsl */ `
      #include <common>
      #include <logdepthbuf_pars_fragment>
      uniform sampler2D uWaves;
      uniform float uRet[${NRET}];
      uniform vec3 uWaveOff;
      uniform float uSurf; uniform float uOpacity; uniform float uTime;
      uniform vec3 uSun; uniform vec3 uSunColor; uniform vec3 uSkyZenith; uniform vec3 uSkyHorizon; uniform vec3 uSkySun; uniform vec3 uIrr;
      uniform vec2 uCoastPad; uniform vec2 uCoastFade;
      varying vec3 vW;
      const float WAVE_TEXELS = ${WAVE_N.toFixed(1)};
      const float SHORE = ${(shoreE + 90).toFixed(1)};      // pad-frame x of the waterline, m

      // one layer of the baked slope patch: tile (m) along the travel direction dir, phase offset off
      // (tiles), slope variance w2. Returns the world-frame slope (xy) and the variance lost to the
      // mip filter at this pixel footprint (z).
      vec3 waveLayer(vec2 p, vec2 dir, float tile, float off, float w2){
        vec2 q = vec2(dot(p, dir), dot(p, vec2(-dir.y, dir.x))) / tile;
        float rho = max(length(dFdx(q)), length(dFdy(q))) * WAVE_TEXELS;
        float lod = clamp(log2(max(rho, 1e-6)), 0.0, ${(NRET - 1).toFixed(1)});
        float fl = floor(lod);
        int i = int(fl);
        float ret = mix(uRet[i], uRet[min(i + 1, ${NRET - 1})], lod - fl);
        vec2 s = texture2D(uWaves, q - vec2(off, 0.0)).rg * sqrt(w2);
        return vec3(s.x * dir + s.y * vec2(-dir.y, dir.x), w2 * (1.0 - ret));
      }

      void main(){
        #include <logdepthbuf_fragment>
        vec2 p = vW.xz - uCoastPad;                      // pad frame: +x east (seaward), +z south
        float dShore = p.x - SHORE;                      // m seaward of the waterline
        vec3 l1 = waveLayer(p, vec2(-0.8525, -0.5227), 19.0, uWaveOff.x, 0.0120);
        vec3 l2 = waveLayer(p, vec2(-0.9553, -0.2955), 83.0, uWaveOff.y, 0.0100);
        vec3 l3 = waveLayer(p, vec2(-0.9928, -0.1197), 470.0, uWaveOff.z, 0.0035);
        vec2 slope = l1.xy + l2.xy + l3.xy;
        // unresolved mean-square slope: capillary waves below the texel (0.006) + what the mips removed
        float sig2 = 0.006 + l1.z + l2.z + l3.z;
        vec3 N = normalize(vec3(-slope.x, 1.0, -slope.y));
        vec3 V = normalize(cameraPosition - vW);
        vec3 L = normalize(uSun);
        float nv = clamp(dot(N, V), 1e-3, 1.0);
        // mean Fresnel reflectance of a Gaussian-sloped surface (F0 = 0.02, water n = 1.34)
        float sigV = sqrt(0.5 * sig2);
        float fres = 0.02 + 0.98 * pow(1.0 - nv, 5.0 * exp(-2.69 * sigV)) / (1.0 + 22.7 * pow(sigV, 1.5));
        // sky seen in the mirror direction; unresolved facets tilt towards the viewer, which lifts the
        // mean reflected elevation by ≈ 1.5 σ (a rough sea is darker than the sky at the horizon)
        vec3 R = reflect(-V, N);
        float el = clamp(abs(R.y) + 1.5 * sigV, 0.0, 1.0);
        vec3 sky = mix(uSkyHorizon, uSkyZenith, pow(el, 0.42));
        float az = max(dot(normalize(R.xz + vec2(1e-5)), normalize(uSun.xz + vec2(1e-5))), 0.0);
        sky += uSkySun * (pow(az, 6.0) * (1.0 - el) * (1.0 - el));
        // sun glitter: GGX microfacet lobe with α² = unresolved slope variance, Smith visibility
        float a2 = clamp(sig2, 1e-4, 0.5);
        vec3 H = normalize(L + V);
        float nh = max(dot(N, H), 0.0), nl = max(dot(N, L), 0.0), vh = clamp(dot(V, H), 0.0, 1.0);
        float dd = nh * nh * (a2 - 1.0) + 1.0;
        float ggx = a2 / (PI * dd * dd);
        float vis = 0.5 / max(nl * sqrt(nv * nv * (1.0 - a2) + a2) + nv * sqrt(nl * nl * (1.0 - a2) + a2), 1e-4);
        vec3 glint = uSunColor * (ggx * vis * (0.02 + 0.98 * pow(1.0 - vh, 5.0)) * nl);
        // water-leaving radiance (Lambertian): deep Gulf → shelf → turbid nearshore → surf zone
        float shelf = smoothstep(0.0, 25000.0, dShore);
        float nearK = 1.0 - smoothstep(0.0, 900.0, dShore);
        float surfK = 1.0 - smoothstep(0.0, 90.0, dShore);
        vec3 alb = mix(vec3(0.020, 0.070, 0.085), vec3(0.010, 0.040, 0.075), shelf);
        alb = mix(alb, vec3(0.07, 0.12, 0.10), nearK);
        alb = mix(alb, vec3(0.16, 0.17, 0.13), surfK * surfK);
        vec3 col = alb * uIrr * (RECIPROCAL_PI * (1.0 - fres)) + sky * fres + glint;
        // surf: breaking lines parallel to the shore (26 m apart, 9 s period) running up the beach,
        // broken up alongshore by the swell height field; swash edge at the waterline
        float hz = texture2D(uWaves, p.yx * (1.0 / 310.0) + vec2(0.0, 0.3 * uWaveOff.z)).b;
        float line = pow(0.5 + 0.5 * sin(6.2831853 * (dShore / 26.0 + uSurf + 0.35 * hz)), 6.0);
        float zone = 1.0 - smoothstep(20.0, 75.0, dShore);
        float swash = 1.0 - smoothstep(0.0, max(7.0 + 3.0 * hz, 1.0), dShore);
        float foam = clamp(zone * line * (0.55 + 0.25 * hz) + 0.8 * swash, 0.0, 1.0);
        col = mix(col, 0.75 * uIrr * RECIPROCAL_PI, foam);
        // soft knee above luminance 1.6 (→ ≤ 2.3): the glitter stays below the bloom threshold 2.5
        float Y = dot(col, vec3(0.2126, 0.7152, 0.0722));
        if (Y > 1.6) col *= (1.6 + 0.7 * (1.0 - exp(-(Y - 1.6) / 0.7))) / Y;
        float fade = 1.0 - smoothstep(uCoastFade.x, uCoastFade.y, length(p));
        gl_FragColor = vec4(col, uOpacity * fade);
      }`,
    transparent: true, depthWrite: false,
  })
  const sea = new THREE.Mesh(new THREE.PlaneGeometry(W, W, GRID, GRID), water)
  sea.rotation.x = -Math.PI / 2
  sea.position.set(shoreE + 90 + W / 2, -0.3, 0)
  root.add(ground, beach, pad, sea)
  // Coast planes draw first in the transparent pass (no depth write: explicit back-to-front order),
  // before the global cloud shell (−4), the cloud shadow (−2) and the atmosphere (−1.5), so the local
  // cloud shadow still multiplies the ground and the haze lies over all of it.
  ground.renderOrder = -6.3
  sea.renderOrder = -6.2
  beach.renderOrder = -6.1
  pad.renderOrder = -6
  const fadeMats = [groundMat, beachMat, padMat]
  let lastK = -1
  const setFade = (k: number): void => {
    coastPad.value.set(root.position.x, root.position.z)
    if (Math.abs(k - lastK) < 1e-3) return
    lastK = k
    for (const m of fadeMats) m.opacity = k
    water.uniforms.uOpacity.value = k
  }
  const frac = (x: number): number => x - Math.floor(x)
  const wu = water.uniforms
  const setLight = (l: CoastLight): void => {
    wu.uSun.value.copy(l.sunDir)
    wu.uSunColor.value.copy(l.sunColor)
    wu.uSkyZenith.value.copy(l.skyZenith)
    wu.uSkyHorizon.value.copy(l.skyHorizon)
    wu.uSkySun.value.copy(l.skySun)
    // irradiance of the sea surface: direct beam on the horizontal + diffuse sky (π × mean radiance)
    const sy = Math.max(l.sunDir.y, 0)
    wu.uIrr.value.setRGB(l.sunColor.r * sy + Math.PI * l.skyAmbient.r, l.sunColor.g * sy + Math.PI * l.skyAmbient.g, l.sunColor.b * sy + Math.PI * l.skyAmbient.b)
    wu.uTime.value = l.time
    // phase offsets in tiles, wrapped on the CPU (double precision): deep-water phase speeds
    // c = √(g λ / 2π) of the dominant wavelength of each layer (3 m, 15 m, 80 m)
    wu.uWaveOff.value.set(frac((2.2 * l.time) / 19), frac((4.8 * l.time) / 83), frac((11.2 * l.time) / 470))
    wu.uSurf.value = frac(l.time / 9)
  }
  return { root, water, setFade, setLight }
}

