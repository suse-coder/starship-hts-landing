import * as THREE from "three"
import { Sky } from "three/addons/objects/Sky.js"

/**
 * Environment: physically based sky (Preetham) with the Sun at its true position, Blue Marble Earth
 * (NASA Visible Earth, public domain, bundled) with an atmosphere rim, starfield for space, Starbase
 * coast with Gulf water. All shaders include the logarithmic-depth chunks used by the renderer.
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
  for (let i = 0; i < n; i++) {
    const u = Math.random() * 2 - 1, th = Math.random() * Math.PI * 2, r = 3.0e5
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
  const p = new THREE.Points(g, new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true, depthWrite: false }))
  p.renderOrder = -9
  return p
}

export interface Atmosphere {
  readonly mesh: THREE.Mesh
  /** sun direction (world, unit), Earth centre (world), camera position (world), in-scatter weight 0..1 */
  readonly update: (sunWorld: THREE.Vector3, centre: THREE.Vector3, cameraPos: THREE.Vector3, weight: number) => void
}

const RE_KM = 6371, RA_KM = 6471

/**
 * Single-scattering atmosphere (Nishita 1993 / O'Neil, GPU Gems 2 ch. 16): Rayleigh β = 5.8/13.5/33.1·10⁻⁶
 * m⁻¹, H_R = 8 km; Mie β = 21·10⁻⁶ m⁻¹, H_M = 1.2 km, g = 0.76; 16 view × 6 light samples; analytic
 * Earth shadow on the light paths. This is THE sky at every altitude (ground, inside, orbit) — one
 * model, no handover. Optional Sun disc (0.267° radius) attenuated by the view-path transmittance and
 * clamped so bloom gives a tight glow. `kmPerUnit`: world units → km (1e-3 metre frame, 1 globe).
 */
export const makeAtmosphere = (radius: number, kmPerUnit: number, sunDisc: boolean): Atmosphere => {
  const mat = new THREE.ShaderMaterial({
    uniforms: { uSun: { value: new THREE.Vector3(1, 0, 0) }, uCenter: { value: new THREE.Vector3() }, uKm: { value: kmPerUnit }, uWeight: { value: 1 }, uSunI: { value: 18 }, uDisc: { value: sunDisc ? 1 : 0 } },
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
      uniform vec3 uSun; uniform vec3 uCenter; uniform float uKm; uniform float uWeight; uniform float uSunI; uniform float uDisc;
      varying vec3 vW;
      const float RE = ${RE_KM.toFixed(1)}, RA = ${RA_KM.toFixed(1)}, HR = 8.0, HM = 1.2;
      const vec3 BR = vec3(5.8e-3, 13.5e-3, 33.1e-3); // 1/km
      const float BM = 21e-3;
      vec2 rs(vec3 o, vec3 d, float R){ float b = dot(o,d); float c = dot(o,o) - R*R; float h = b*b - c; if (h < 0.0) return vec2(1e9, -1e9); h = sqrt(h); return vec2(-b - h, -b + h); }
      void main(){
        #include <logdepthbuf_fragment>
        vec3 o = (cameraPosition - uCenter) * uKm;
        vec3 d = normalize(vW - cameraPosition);
        vec2 a = rs(o, d, RA);
        if (a.x > a.y || a.y < 0.0) discard;
        float t0 = max(a.x, 0.0), t1 = a.y;
        vec2 e = rs(o, d, RE);
        bool hitsEarth = e.x > 0.0 && e.x < e.y;
        if (hitsEarth) t1 = min(t1, e.x);
        if (t1 <= t0) discard;
        const int N = 16; const int M = 6;
        float ds = (t1 - t0) / float(N);
        vec3 sumR = vec3(0.0), sumM = vec3(0.0); float odR = 0.0, odM = 0.0;
        for (int i = 0; i < N; i++) {
          vec3 p = o + d * (t0 + ds * (float(i) + 0.5));
          float h = max(length(p) - RE, 0.0);
          float dr = exp(-h / HR) * ds, dm = exp(-h / HM) * ds;
          odR += dr; odM += dm;
          vec2 le = rs(p, uSun, RE);
          if (le.x > 0.0 && le.x < le.y) continue; // sample in the Earth's shadow
          float dl = rs(p, uSun, RA).y / float(M);
          float lR = 0.0, lM = 0.0;
          for (int j = 0; j < M; j++) { vec3 q = p + uSun * (dl * (float(j) + 0.5)); float hq = max(length(q) - RE, 0.0); lR += exp(-hq / HR) * dl; lM += exp(-hq / HM) * dl; }
          vec3 att = exp(-(BR * (odR + lR) + BM * 1.1 * (odM + lM)));
          sumR += att * dr; sumM += att * dm;
        }
        float mu = dot(d, uSun);
        float pR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
        float g = 0.76;
        float pM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) / ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));
        vec3 col = uSunI * (sumR * BR * pR + sumM * BM * pM);
        vec3 disc = vec3(0.0);
        if (uDisc > 0.5 && !hitsEarth) {
          // solar disc, radius 0.267° (cos = 0.9999891), limb-darkened, through the view-path transmittance
          float c = smoothstep(0.9999891, 0.9999950, mu);
          vec3 trans = exp(-(BR * odR + BM * 1.1 * odM));
          disc = trans * c * 3.0;
        }
        gl_FragColor = vec4((min(col, vec3(2.2)) + disc) * uWeight, 1.0);
      }`,
    transparent: true, blending: THREE.AdditiveBlending, depthWrite: false,
  })
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius * (RA_KM / RE_KM), 160, 80), mat)
  // after the global cloud shell (−4): its in-scatter is in front of the cloud tops
  mesh.renderOrder = -3
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
  /** per frame: sun direction (world), Earth centre (world), camera, in-scatter weight */
  readonly update: (sunWorld: THREE.Vector3, centre: THREE.Vector3, camera: THREE.Camera, weight: number) => void
}

/**
 * Earth: Blue Marble day map, Black Marble 2016 city lights on the night side (soft terminator), and the
 * single-scattering atmosphere above. Local axes are ECEF-aligned through EARTH_GEOM_TO_ECEF. Inside
 * the atmosphere the surface writes no depth (local ground, ship and clouds always draw over it).
 */
export const makeEarth = (radius: number, dayUrl: string, nightUrl: string, kmPerUnit: number): EarthModel => {
  const loader = new THREE.TextureLoader()
  const day = loader.load(dayUrl)
  day.colorSpace = THREE.SRGBColorSpace
  day.anisotropy = 8
  const night = loader.load(nightUrl)
  night.colorSpace = THREE.SRGBColorSpace
  const mat = new THREE.MeshStandardMaterial({ map: day, roughness: 0.92, metalness: 0, emissive: 0xffd9a0, emissiveMap: night, emissiveIntensity: 1.6 })
  const uSunView = { value: new THREE.Vector3(1, 0, 0) }
  mat.onBeforeCompile = (sh) => {
    sh.uniforms.uSunView = uSunView
    sh.fragmentShader = sh.fragmentShader
      .replace("#include <common>", "#include <common>\nuniform vec3 uSunView;")
      .replace("#include <emissivemap_fragment>", "#include <emissivemap_fragment>\n  totalEmissiveRadiance *= smoothstep(0.06, -0.14, dot(normal, uSunView));")
  }
  const earth = new THREE.Mesh(new THREE.SphereGeometry(radius, 256, 128), mat)
  earth.renderOrder = -5
  const atm = makeAtmosphere(radius, kmPerUnit, true)
  const update: EarthModel["update"] = (sunWorld, centre, camera, weight) => {
    uSunView.value.copy(sunWorld).transformDirection(camera.matrixWorldInverse)
    atm.update(sunWorld, centre, camera.position, weight)
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

/** Starbase coastal scrub / sand ground texture (procedural) */
const groundTexture = (): THREE.CanvasTexture => noiseCanvas(512, 512, (g) => {
  g.fillStyle = "#9b9168"; g.fillRect(0, 0, 512, 512)
  for (let i = 0; i < 9000; i++) {
    const x = Math.random() * 512, y = Math.random() * 512, r = 1 + Math.random() * 5
    const k = Math.random()
    g.fillStyle = k < 0.45 ? `rgba(92,104,64,${0.25 + Math.random() * 0.35})` : k < 0.8 ? `rgba(176,164,124,${0.2 + Math.random() * 0.3})` : `rgba(70,74,52,${0.2 + Math.random() * 0.3})`
    g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill()
  }
})

export interface Coast {
  readonly root: THREE.Group
  readonly water: THREE.ShaderMaterial
  /** 1 = opaque, → 0 fades the local planes out over the Earth sphere (25–45 km) */
  readonly setFade: (k: number) => void
}

/** ground (west of the shoreline), beach strip, and Gulf water with procedural waves and sun glint */
export const makeCoast = (shoreE = 950): Coast => {
  const root = new THREE.Group()
  const gtex = groundTexture()
  // 300 km planes: from 10 km altitude the horizon is ≈ 360 km away, the flat-plane sag at 150 km is
  // ≈ 1.8 km (hidden by haze); beyond that the Earth sphere takes over
  const W = 300e3
  const GRID = 256 // subdivisions: one 300 km quad fails the depth test against objects a few metres tall
  gtex.repeat.set(W / 250, W / 250)
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(W, W, GRID, GRID), new THREE.MeshStandardMaterial({ map: gtex, roughness: 1 }))
  ground.rotation.x = -Math.PI / 2
  ground.position.set(shoreE - W / 2, -0.05, 0)
  ground.receiveShadow = true
  const beach = new THREE.Mesh(new THREE.PlaneGeometry(90, W, 2, GRID), new THREE.MeshStandardMaterial({ color: 0xd8c9a0, roughness: 1 }))
  beach.rotation.x = -Math.PI / 2
  beach.position.set(shoreE + 45, -0.04, 0)
  beach.receiveShadow = true
  const pad = new THREE.Mesh(new THREE.CylinderGeometry(75, 75, 0.4, 64), new THREE.MeshStandardMaterial({ color: 0xbab6aa, roughness: 0.95 }))
  pad.position.y = 0.1
  pad.receiveShadow = true
  const water = new THREE.ShaderMaterial({
    uniforms: { uTime: { value: 0 }, uSun: { value: new THREE.Vector3(0, 1, 0) }, uSky: { value: new THREE.Color(0.55, 0.7, 0.9) }, uOpacity: { value: 1 } },
    vertexShader: /* glsl */ `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      varying vec3 vW;
      void main(){ vec4 w = modelMatrix*vec4(position,1.0); vW = w.xyz; gl_Position = projectionMatrix*viewMatrix*w;
        #include <logdepthbuf_vertex>
      }`,
    fragmentShader: /* glsl */ `
      #include <logdepthbuf_pars_fragment>
      uniform float uTime; uniform vec3 uSun; uniform vec3 uSky; uniform float uOpacity; varying vec3 vW;
      // each wave fades out once its phase changes by more than ~1 rad per pixel (no moiré at range)
      vec2 wave(vec2 p, vec2 d, float k, float a, float s){ float x = dot(p,d)*k; float aa = clamp(1.5 - fwidth(x), 0.0, 1.0); return d*(a*k*cos(x + uTime*s))*aa; }
      void main(){
        #include <logdepthbuf_fragment>
        vec2 p = vW.xz;
        vec2 g = wave(p, normalize(vec2(-0.8,0.6)), 0.08, 0.6, 1.3) + wave(p, normalize(vec2(-0.3,1.0)), 0.21, 0.18, 2.1)
               + wave(p, normalize(vec2(-1.0,-0.2)), 0.55, 0.05, 3.3) + wave(p, normalize(vec2(0.4,0.9)), 1.3, 0.015, 5.0);
        vec3 N = normalize(vec3(-g.x, 1.0, -g.y));
        vec3 V = normalize(cameraPosition - vW);
        vec3 L = normalize(uSun);
        float fres = 0.02 + 0.98*pow(1.0 - max(dot(N,V),0.0), 5.0);
        vec3 deep = vec3(0.03,0.16,0.22), shallow = vec3(0.07,0.32,0.36);
        float shore = smoothstep(0.0, 900.0, vW.x - ${shoreE.toFixed(1)});
        vec3 base = mix(shallow, deep, shore);
        vec3 H = normalize(L + V);
        float spec = pow(max(dot(N,H),0.0), 420.0)*6.0 + pow(max(dot(N,H),0.0), 40.0)*0.25;
        float foam = smoothstep(60.0, 0.0, vW.x - ${shoreE.toFixed(1)} - 90.0) * (0.5 + 0.5*sin(vW.z*0.05 + uTime*1.7));
        vec3 col = mix(base, uSky, fres) + vec3(1.0,0.95,0.85)*spec*max(L.y,0.0) + vec3(0.9)*foam*0.35;
        gl_FragColor = vec4(col, uOpacity);
      }`,
    transparent: true,
  })
  const sea = new THREE.Mesh(new THREE.PlaneGeometry(W, W, GRID, GRID), water)
  sea.rotation.x = -Math.PI / 2
  sea.position.set(shoreE + 90 + W / 2, -0.3, 0)
  root.add(ground, beach, pad, sea)
  const fadeMats = [ground.material, beach.material, pad.material] as THREE.MeshStandardMaterial[]
  let lastK = -1
  const setFade = (k: number): void => {
    if (Math.abs(k - lastK) < 1e-3) return
    lastK = k
    const tr = k < 0.999
    for (const m of fadeMats) { if (m.transparent !== tr) { m.transparent = tr; m.needsUpdate = true } m.opacity = k; m.depthWrite = !tr }
    water.uniforms.uOpacity.value = k
    water.depthWrite = !tr
  }
  return { root, water, setFade }
}
