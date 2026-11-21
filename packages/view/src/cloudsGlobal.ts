import * as THREE from "three"

/**
 * Global cloud shell at Earth scale.
 *
 * The volumetric deck in `clouds.ts` is a local, pad-frame cumulus field: from orbit it would only
 * ever be a small patch, while the Earth day texture carries the clouds painted flat on the surface.
 * This module adds the planet-wide layer that is visible from space:
 *
 *  - a sphere a few km above the surface (a real shell, so cloud tops parallax correctly against the
 *    surface and protrude past the limb);
 *  - the NASA "fair weather clouds" map (public domain, `earth_clouds_4096.jpg`, credits in
 *    `textures/CREDITS.md`), sampled with a soft threshold so the cloud edges dissolve instead of
 *    cutting out;
 *  - lighting from the true Sun: a smooth terminator, the low-Sun reddening, a cool ambient on the
 *    day side, and a nearly black night side so the Black Marble city lights read through clear gaps;
 *  - a shallow forward-scattering lobe and a soft luminance knee, so sunlit tops stay white without
 *    blooming and the limb does not look like a decal;
 *  - a circular "local hole" that can be faded in around the pad frame: the volumetric deck takes
 *    over there during the hand-off, and the global layer only shows the distant clouds beyond it.
 *
 * The shell is five texture taps per fragment (three mask taps for the soft edge and two fractal
 * detail taps), no raymarch, and no additional render target: a fraction of the cost of the local
 * deck.
 */

export interface GlobalCloudFrame {
  /** unit Sun direction in the same world frame as `centre` */
  readonly sunDir: THREE.Vector3
  /** linear direct-Sun colour at the cloud level (already includes the low-Sun tint) */
  readonly sunColor: THREE.Color
  /** linear mean sky colour above the clouds */
  readonly skyColor: THREE.Color
  /** Earth centre in world coordinates */
  readonly centre: THREE.Vector3
  /** world camera position (decides whether the camera is inside the shell) */
  readonly camera: THREE.Vector3
  /** seconds (cloud drift) */
  readonly time: number
  /** m/s East, North of the global cloud drift */
  readonly wind: readonly [number, number]
  /** 0..1 fraction of the Sun visible (orbital night → 0) */
  readonly eclipse: number
  /** overall opacity multiplier (1 = fully on) */
  readonly opacity?: number
  /** unit direction from the Earth centre to the pad (local hole) */
  readonly holeDir?: THREE.Vector3
  /** angular radius of the hole (radians) */
  readonly holeRadius?: number
  /** 0 = no hole, 1 = the global layer is absent inside `holeRadius` */
  readonly holeAmount?: number
}

export interface GlobalClouds {
  readonly mesh: THREE.Mesh
  /** Earth radius + shell height in world units */
  readonly shellRadius: number
  readonly update: (f: GlobalCloudFrame) => void
}

export interface GlobalCloudOptions {
  /** Earth radius in the world units of the frame the mesh is placed in */
  readonly radius: number
  /** shell height above the surface in the same units */
  readonly height?: number
  /** cloud mask URL (equirectangular, white = dense cloud) */
  readonly url?: string
  /** already-loaded mask (shared by several shells / Earth frames); overrides `url` */
  readonly texture?: THREE.Texture
  /** fractal detail map URL (tileable grayscale) */
  readonly detailUrl?: string
  /** already-loaded detail map, shared by every shell */
  readonly detailTexture?: THREE.Texture
  /** shell resolution; 64×32 is enough for a smooth silhouette */
  readonly segments?: number
}

/** Tileable grayscale fractal map used to break the coarse cloud composite into cloud-sized detail. */
export const loadCloudDetail = (url: string): THREE.Texture => {
  const tex = new THREE.TextureLoader().load(url)
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping
  tex.minFilter = THREE.LinearMipmapLinearFilter
  tex.magFilter = THREE.LinearFilter
  tex.generateMipmaps = true
  tex.anisotropy = 4
  tex.colorSpace = THREE.NoColorSpace
  return tex
}

/** Linear grayscale cloud density map (equirectangular), shared by every Earth frame. */
export const loadCloudMask = (url: string): THREE.Texture => {
  const tex = new THREE.TextureLoader().load(url)
  tex.wrapS = THREE.RepeatWrapping
  tex.wrapT = THREE.ClampToEdgeWrapping
  tex.minFilter = THREE.LinearMipmapLinearFilter
  tex.magFilter = THREE.LinearFilter
  tex.generateMipmaps = true
  tex.anisotropy = 8
  tex.colorSpace = THREE.NoColorSpace
  return tex
}

const VERT = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_vertex>
  varying vec2 vUv;
  varying vec3 vW;
  void main(){
    vUv = uv;
    vec4 w = modelMatrix * vec4(position, 1.0);
    vW = w.xyz;
    gl_Position = projectionMatrix * viewMatrix * w;
    #include <logdepthbuf_vertex>
  }`

const FRAG = /* glsl */ `
  #include <common>
  #include <logdepthbuf_pars_fragment>
  uniform sampler2D uMask, uDetail;
  uniform vec3 uSun, uSunColor, uSkyColor, uCentre, uHoleDir;
  uniform vec2 uDrift;
  uniform float uOpacity, uHoleRadius, uHoleAmount, uEclipse, uTime;
  varying vec2 vUv;
  varying vec3 vW;

  // soft cloud mask: the source is a grayscale density map; two thresholds give a soft edge and
  // keep the thin wisps from disappearing at grazing angles
  float cloudMask(vec2 uv){
    return texture2D(uMask, uv).r;
  }

  void main(){
    #include <logdepthbuf_fragment>
    vec3 N = normalize(vW - uCentre);
    vec3 V = normalize(cameraPosition - vW);
    // the map is equirectangular: longitude wraps, latitude clamps (no seam at the poles)
    vec2 uv = vec2(fract(vUv.x + uDrift.x), clamp(vUv.y + uDrift.y, 0.001, 0.999));
    // the 10 km cloud composite magnifies to coarse blocks at ISS range; a fractal detail map
    // domain-warps the edges and erodes the density, giving cloud-sized structure below the texel
    float d1 = texture2D(uDetail, uv * 256.0).r;
    float d2 = texture2D(uDetail, uv * 1024.0 + vec2(0.37, 0.11)).r;
    vec2 warp = (vec2(d1, d2) - 0.5) * vec2(0.0032, 0.0016);
    float m = cloudMask(uv + warp);
    float mEdge = cloudMask(uv + warp + vec2(0.0007, 0.0));
    float mEdge2 = cloudMask(uv + warp - vec2(0.0007, 0.0));
    m = m * 0.6 + max(m, max(mEdge, mEdge2)) * 0.4; // conservative edge: no hard texture stair-steps
    float grain = d1 * 0.65 + d2 * 0.35;
    float a = smoothstep(0.13, 0.62, m);
    // the thin tail of the distribution gives cirrus-like translucency
    float thin = smoothstep(0.05, 0.30, m);
    a = max(a, thin * 0.16);
    a *= smoothstep(0.06, 0.92, 0.30 + 0.95 * grain);
    a *= uOpacity;

    if (uHoleAmount > 0.0) {
      float ang = acos(clamp(dot(N, uHoleDir), -1.0, 1.0));
      float hole = smoothstep(uHoleRadius * 0.65, uHoleRadius * 1.35, ang);
      a *= mix(1.0, hole, clamp(uHoleAmount, 0.0, 1.0));
    }
    if (a < 0.002) discard;

    float ndl = dot(N, uSun);
    float day = smoothstep(-0.12, 0.14, ndl);
    float sunUp = max(ndl, 0.0);
    // Kasten–Young-like air mass at this latitude, for the sunrise/sunset tint
    float sinEl = max(ndl, 0.0);
    float am = 1.0 / (sinEl + 0.50572 * pow(degrees(asin(sinEl)) + 6.07995, -1.6364));
    vec3 tint = mix(vec3(1.0, 0.52, 0.30), vec3(1.0, 0.98, 0.94), smoothstep(0.0, 0.32, ndl));
    tint *= exp(-vec3(0.11, 0.17, 0.33) * max(am - 1.0, 0.0));

    // lit cloud: sun on a horizontal sheet plus the sky's multiple scattering. The night side keeps
    // only a trace of airglow so it reads as cloud cover, not as another black sky.
    vec3 lit = uSunColor * tint * (0.32 + 0.62 * pow(sunUp, 0.75));
    vec3 amb = uSkyColor * (0.24 + 0.76 * day) * 0.55;
    vec3 col = lit * day + amb;
    // shaded undersides / bases: cooler and a little darker where the sheet is optically thick;
    // the fractal grain breaks the flat decal into billows without another texture lookup
    col *= mix(0.78, 1.02, smoothstep(0.10, 0.75, a));
    col *= 0.84 + 0.30 * grain;
    // forward-scattered silver lining when looking roughly toward the Sun
    float fwd = pow(max(dot(-V, uSun), 0.0), 5.0);
    col += uSunColor * tint * fwd * day * 0.10;
    // limb turn: a slightly cooler, hazier cloud at grazing angles
    float limb = smoothstep(0.0, 0.22, dot(N, V));
    col = mix(col * vec3(0.86, 0.92, 1.0), col, limb);
    col *= uEclipse;
    // very dark on the night side, but not a hole
    col += vec3(0.006, 0.008, 0.013) * (1.0 - day);

    // soft knee above luminance 1.6 keeps sunlit tops below the bloom threshold 2.5
    float Y = dot(col, vec3(0.2126, 0.7152, 0.0722));
    if (Y > 1.6) col *= (1.6 + 0.7 * (1.0 - exp(-(Y - 1.6) / 0.7))) / Y;
    gl_FragColor = vec4(col * a, a);
  }`

/**
 * Planet-wide cloud shell. Add `mesh` to the frame that contains the Earth (set its matrix to the
 * same Earth transform each frame), then call `update` every frame.
 */
export const makeGlobalClouds = (opts: GlobalCloudOptions): GlobalClouds => {
  const height = opts.height ?? 4500
  const shellRadius = opts.radius + height
  const tex = opts.texture ?? loadCloudMask(opts.url ?? "")
  const neutralDetail = new THREE.DataTexture(new Uint8Array([200]), 1, 1, THREE.RedFormat)
  neutralDetail.needsUpdate = true
  const detail = opts.detailTexture ?? (opts.detailUrl ? loadCloudDetail(opts.detailUrl) : neutralDetail)
  const uniforms = {
    uMask: { value: tex },
    uDetail: { value: detail },
    uSun: { value: new THREE.Vector3(1, 0, 0) },
    uSunColor: { value: new THREE.Color(1, 1, 1) },
    uSkyColor: { value: new THREE.Color(0.16, 0.27, 0.42) },
    uCentre: { value: new THREE.Vector3() },
    uHoleDir: { value: new THREE.Vector3(0, 1, 0) },
    uDrift: { value: new THREE.Vector2() },
    uOpacity: { value: 1 },
    uHoleRadius: { value: 0.1 },
    uHoleAmount: { value: 0 },
    uEclipse: { value: 1 },
    uTime: { value: 0 },
  }
  const mat = new THREE.ShaderMaterial({
    uniforms,
    vertexShader: VERT,
    fragmentShader: FRAG,
    transparent: true,
    depthWrite: false,
    depthTest: true,
    side: THREE.FrontSide,
    blending: THREE.CustomBlending,
    blendEquation: THREE.AddEquation,
    blendSrc: THREE.OneFactor,
    blendDst: THREE.OneMinusSrcAlphaFactor,
  })
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(1, opts.segments ?? 128, (opts.segments ?? 128) / 2), mat)
  mesh.scale.setScalar(shellRadius)
  mesh.frustumCulled = false
  mesh.updateMatrix()
  mesh.matrixAutoUpdate = false
  mesh.name = "cloud-global"
  mesh.renderOrder = -4
  mesh.userData.shellRadius = shellRadius
  let side: THREE.Side = THREE.FrontSide
  const update = (f: GlobalCloudFrame): void => {
    uniforms.uSun.value.copy(f.sunDir).normalize()
    uniforms.uSunColor.value.copy(f.sunColor)
    uniforms.uSkyColor.value.copy(f.skyColor)
    uniforms.uCentre.value.copy(f.centre)
    uniforms.uOpacity.value = f.opacity ?? 1
    uniforms.uEclipse.value = Math.max(0, Math.min(1, f.eclipse))
    uniforms.uTime.value = f.time
    // one map width per ~9 hours at 5 m/s: the global field is extremely slow; the local deck
    // carries the fast evolution
    uniforms.uDrift.value.set((f.wind[0] * f.time) / (2 * Math.PI * 6371e3), -(f.wind[1] * f.time) / (2 * Math.PI * 6371e3))
    uniforms.uHoleDir.value.copy(f.holeDir ?? new THREE.Vector3(0, 1, 0)).normalize()
    uniforms.uHoleRadius.value = f.holeRadius ?? 0
    uniforms.uHoleAmount.value = Math.max(0, Math.min(1, f.holeAmount ?? 0))
    // outside the shell front faces are visible; under it (ground, low altitude) back faces are
    const inside = f.camera.distanceTo(f.centre) < shellRadius
    const want = inside ? THREE.BackSide : THREE.FrontSide
    if (want !== side) { side = want; mat.side = side; mat.needsUpdate = true }
  }
  return { mesh, shellRadius, update }
}
