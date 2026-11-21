import * as THREE from "three"

/**
 * Physically driven visual effects. Every parameter comes from recorded channels:
 * plasma colour = black-body colour of the stagnation temperature T₀, brightness from the electron
 * density n_e (emission ∝ n_e²-ish, compressed), plume length/expansion from thrust and ambient
 * pressure, wind arrows from the forecast profile corrected by the measured local wind.
 */

/** sRGB colour of a black body at T (K) — Tanner Helland fit (1000–40000 K), clamped */
export const blackbody = (T: number, out = new THREE.Color()): THREE.Color => {
  const t = Math.max(1000, Math.min(40000, T)) / 100
  const r = t <= 66 ? 255 : 329.698727446 * (t - 60) ** -0.1332047592
  const g = t <= 66 ? 99.4708025861 * Math.log(t) - 161.1195681661 : 288.1221695283 * (t - 60) ** -0.0755148492
  const b = t >= 66 ? 255 : t <= 19 ? 0 : 138.5177312231 * Math.log(t - 10) - 305.0447927307
  const c = (v: number): number => Math.max(0, Math.min(255, v)) / 255
  return out.setRGB(c(r), c(g), c(b), THREE.SRGBColorSpace)
}

const NOISE = /* glsl */ `
  vec3 hash3(vec3 p){ p = vec3(dot(p,vec3(127.1,311.7,74.7)), dot(p,vec3(269.5,183.3,246.1)), dot(p,vec3(113.5,271.9,124.6))); return -1.0 + 2.0*fract(sin(p)*43758.5453123); }
  float noise(vec3 p){ vec3 i=floor(p), f=fract(p); vec3 u=f*f*(3.0-2.0*f);
    return mix(mix(mix(dot(hash3(i),f), dot(hash3(i+vec3(1,0,0)),f-vec3(1,0,0)),u.x), mix(dot(hash3(i+vec3(0,1,0)),f-vec3(0,1,0)), dot(hash3(i+vec3(1,1,0)),f-vec3(1,1,0)),u.x),u.y),
               mix(mix(dot(hash3(i+vec3(0,0,1)),f-vec3(0,0,1)), dot(hash3(i+vec3(1,0,1)),f-vec3(1,0,1)),u.x), mix(dot(hash3(i+vec3(0,1,1)),f-vec3(0,1,1)), dot(hash3(i+vec3(1,1,1)),f-vec3(1,1,1)),u.x),u.y),u.z); }
  float fbm(vec3 p){ float a=0.5, s=0.0; for(int k=0;k<5;k++){ s+=a*noise(p); p*=2.03; a*=0.5; } return s; }
`

/**
 * shock-layer plasma over the whole OML. Emission is weighted by the Newtonian inclination of the
 * surface to the flow (post-shock pressure/density ∝ sin²θ, θ = angle between surface and flow):
 * windward faces glow, the leeward expansion stays dark. Flow noise streams downstream.
 */
export const makePlasmaMaterial = (): THREE.ShaderMaterial => new THREE.ShaderMaterial({
  uniforms: { uTime: { value: 0 }, uColor: { value: new THREE.Color(1, 0.5, 0.2) }, uIntensity: { value: 0 }, uFlow: { value: new THREE.Vector3(1, 0, 0) } },
  vertexShader: /* glsl */ `
    #include <common>
    #include <logdepthbuf_pars_vertex>

    varying vec3 vN; varying vec3 vV; varying vec3 vP; varying vec3 vNo; varying vec2 vUv;
    void main(){ vUv = uv; vec4 wp = modelMatrix*vec4(position,1.0); vP = position; vNo = normal; vN = normalize(normalMatrix*normal);
      vec4 mv = viewMatrix*wp; vV = normalize(-mv.xyz); gl_Position = projectionMatrix*mv;
      #include <logdepthbuf_vertex>
    }`,
  fragmentShader: /* glsl */ `
    #include <logdepthbuf_pars_fragment>

    uniform float uTime; uniform vec3 uColor; uniform float uIntensity; uniform vec3 uFlow;
    varying vec3 vN; varying vec3 vV; varying vec3 vP; varying vec3 vNo; varying vec2 vUv;
    ${NOISE}
    // 3 octaves, not 5: the highest octave of a fast-scrolling fBm moves about one cycle per frame at
    // 60 fps, so it aliases into white sparkle where the additive plasma crosses the bloom threshold.
    float fbm3(vec3 p){ float a=0.5, s=0.0; for(int k=0;k<3;k++){ s+=a*noise(p); p*=2.03; a*=0.5; } return s; }
    void main(){
      #include <logdepthbuf_fragment>
      float sinT = max(dot(normalize(vNo), -normalize(uFlow)), 0.0); // sin of the local flow inclination
      float newton = sinT * sinT;
      if (newton < 0.003) discard;
      float fres = pow(max(0.0, 1.0 - abs(dot(normalize(vN), normalize(vV)))), 1.6);
      // slow, large-scale flow structure: the plasma still streams, but no sub-pixel motion per frame
      float n = fbm3(vP*0.35 - uFlow*uTime*1.2);
      float streak = fbm3(vec3(vP.x*0.12 - uTime*0.8, vP.y*1.4, vP.z*1.4));
      // the sheath geometry covers only the nose (cap + first ogive metres); fade it out towards the aft cut
      float noseFade = smoothstep(0.0, 0.7, vUv.y);
      float a = uIntensity * newton * noseFade * (0.35 + 0.65*fres) * (0.65 + 0.35*n + 0.2*streak);
      vec3 col = uColor * (1.25 + 0.55*n) + vec3(0.25,0.15,0.35)*fres*uIntensity;
      // soft knee below the bloom threshold 2.5: brightness varies smoothly instead of popping white
      vec3 rad = col*a;
      float m = max(rad.r, max(rad.g, rad.b));
      rad *= 2.2 * (1.0 - exp(-m/2.2)) / max(m, 1e-4);
      gl_FragColor = vec4(rad, a);
    }`,
  transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
})

/** bow-shock surface overlay: Fresnel rim (the shock reads as a thin shell seen edge-on) */
export const makeShockMaterial = (): THREE.ShaderMaterial => new THREE.ShaderMaterial({
  uniforms: { uColor: { value: new THREE.Color(0xd8ecff) }, uOpacity: { value: 0 } },
  vertexShader: /* glsl */ `
    #include <common>
    #include <logdepthbuf_pars_vertex>
    varying vec3 vN; varying vec3 vV;
    void main(){ vN = normalize(normalMatrix*normal); vec4 mv = modelViewMatrix*vec4(position,1.0); vV = normalize(-mv.xyz); gl_Position = projectionMatrix*mv;
      #include <logdepthbuf_vertex>
    }`,
  fragmentShader: /* glsl */ `
    #include <logdepthbuf_pars_fragment>
    uniform vec3 uColor; uniform float uOpacity; varying vec3 vN; varying vec3 vV;
    void main(){
      #include <logdepthbuf_fragment>
      float fres = pow(max(0.0, 1.0 - abs(dot(normalize(vN), normalize(vV)))), 2.2);
      gl_FragColor = vec4(uColor, (0.06 + 0.7*fres) * uOpacity);
    }`,
  transparent: true, depthWrite: false, side: THREE.DoubleSide,
})

/**
 * plasma wake: a faint, narrow incandescent trail along −v̂ (meteor-like), fading with distance from the vehicle.
 * Display rules (2026-11-21, "whole screen flickers white"): the chase camera often sits inside the trail, so
 * (1) fragments within 30–90 m of the camera fade to zero, (2) the turbulence scrolls slowly (no sub-frame motion),
 * (3) the radiance is soft-capped at 1.2, far below the bloom threshold 2.5, so the trail never blooms the screen.
 */
export const makeWakeMaterial = (): THREE.ShaderMaterial => new THREE.ShaderMaterial({
  uniforms: { uTime: { value: 0 }, uColor: { value: new THREE.Color(1, 0.5, 0.2) }, uIntensity: { value: 0 } },
  vertexShader: /* glsl */ `
    #include <common>
    #include <logdepthbuf_pars_vertex>
    varying vec2 vUv; varying vec3 vP; varying float vDepth;
    void main(){ vUv = uv; vP = position; vec4 mv = modelViewMatrix*vec4(position,1.0); vDepth = -mv.z; gl_Position = projectionMatrix*mv;
      #include <logdepthbuf_vertex>
    }`,
  fragmentShader: /* glsl */ `
    #include <logdepthbuf_pars_fragment>
    uniform float uTime; uniform vec3 uColor; uniform float uIntensity; varying vec2 vUv; varying vec3 vP; varying float vDepth;
    ${NOISE}
    float fbm3(vec3 p){ float a=0.5, s=0.0; for(int k=0;k<3;k++){ s+=a*noise(p); p*=2.03; a*=0.5; } return s; }
    void main(){
      #include <logdepthbuf_fragment>
      float along = vUv.y;                      // 0 at the vehicle, 1 at the tail end
      float n = fbm3(vec3(vP.x*0.05, vP.y*0.02 + uTime*0.6, vP.z*0.05));
      float edge = smoothstep(0.0, 0.35, vUv.x) * smoothstep(1.0, 0.65, vUv.x);
      float nearFade = smoothstep(30.0, 90.0, vDepth);
      float a = uIntensity * pow(1.0 - along, 2.6) * (0.7 + 0.3*n) * (0.5 + 0.5*edge) * nearFade;
      vec3 rad = uColor * a;
      float m = max(rad.r, max(rad.g, rad.b));
      rad *= 1.2 * (1.0 - exp(-m/1.2)) / max(m, 1e-4);
      gl_FragColor = vec4(rad, a);
    }`,
  transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
})

/**
 * Shock-layer plasma ahead of the nose: drawn on the bow-shock cap (the paraboloid placed at the standoff Δ in
 * front of the stagnation point). The gas between the shock and the PICA cap is hottest and densest on the
 * stagnation streamline, so the emission is brightest on the axis and falls off with the cap radius r/R
 * (uv.y of the lathe: 0 at the apex). Limb brightening (∝ path length through a thin shell seen edge-on) gives
 * the glowing rim; the core goes white-hot (black body at T0 blended towards white). Radiance is soft-capped at
 * 2.4, just below the bloom threshold, so only a small halo can bloom and the screen never washes out.
 */
export const makeShockLayerMaterial = (): THREE.ShaderMaterial => new THREE.ShaderMaterial({
  uniforms: { uTime: { value: 0 }, uColor: { value: new THREE.Color(1, 0.5, 0.2) }, uIntensity: { value: 0 } },
  vertexShader: /* glsl */ `
    #include <common>
    #include <logdepthbuf_pars_vertex>
    varying vec2 vUv; varying vec3 vN; varying vec3 vV; varying vec3 vP;
    void main(){ vUv = uv; vP = position; vN = normalize(normalMatrix*normal); vec4 mv = modelViewMatrix*vec4(position,1.0); vV = normalize(-mv.xyz); gl_Position = projectionMatrix*mv;
      #include <logdepthbuf_vertex>
    }`,
  fragmentShader: /* glsl */ `
    #include <logdepthbuf_pars_fragment>
    uniform float uTime; uniform vec3 uColor; uniform float uIntensity;
    varying vec2 vUv; varying vec3 vN; varying vec3 vV; varying vec3 vP;
    ${NOISE}
    void main(){
      #include <logdepthbuf_fragment>
      float r = vUv.y;                                   // 0 on the stagnation axis → 1 at the cap edge
      float core = exp(-pow(r / 0.30, 2.0));             // stagnation region
      float body = pow(max(1.0 - r, 0.0), 1.6);          // shock layer thinning towards the shoulder
      float limb = pow(max(0.0, 1.0 - abs(dot(normalize(vN), normalize(vV)))), 1.8);
      float n = noise(vec3(vP.x*0.6, vP.y*0.6 - uTime*0.5, vP.z*0.6)); // slow, large-scale shimmer (±10 %)
      float e = uIntensity * (1.4*core + 0.55*body + 0.45*limb*body) * (0.92 + 0.08*n);
      vec3 col = mix(uColor, vec3(1.0, 0.97, 0.92), 0.65*core);
      vec3 rad = col * e * 2.0;
      float m = max(rad.r, max(rad.g, rad.b));
      rad *= 2.4 * (1.0 - exp(-m/2.4)) / max(m, 1e-4);
      float a = clamp(e, 0.0, 1.0) * smoothstep(1.0, 0.75, r);
      gl_FragColor = vec4(rad * smoothstep(1.0, 0.75, r), a);
    }`,
  transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
})

/**
 * methalox plume (emission only, additive). Brightness ∝ projected column depth through the jet
 * (|n·v| on the cone surface), faint blue core (CH* and C₂ band emission of near-stoichiometric CH₄/O₂),
 * sharp periodic shock cells near the exit at sea level (over-expanded nozzle), orange afterburning of
 * the fuel-rich film in air only. No high-frequency flicker (turbulence is a slow ±8 % modulation).
 */
export const makePlumeMaterial = (): THREE.ShaderMaterial => new THREE.ShaderMaterial({
  uniforms: { uTime: { value: 0 }, uPamb: { value: 1 }, uThrottle: { value: 0 } },
  vertexShader: /* glsl */ `
    #include <common>
    #include <logdepthbuf_pars_vertex>
    varying vec2 vUv; varying vec3 vN; varying vec3 vV;
    void main(){ vUv = uv; vN = normalize(normalMatrix*normal); vec4 mv = modelViewMatrix*vec4(position,1.0); vV = normalize(-mv.xyz); gl_Position = projectionMatrix*mv;
      #include <logdepthbuf_vertex>
    }`,
  fragmentShader: /* glsl */ `
    #include <logdepthbuf_pars_fragment>

    uniform float uTime; uniform float uPamb; uniform float uThrottle; varying vec2 vUv; varying vec3 vN; varying vec3 vV;
    ${NOISE}
    void main(){
      #include <logdepthbuf_fragment>
      float s = 1.0 - vUv.y;                    // 0 at the nozzle exit, 1 at the tail
      float chord = pow(abs(dot(normalize(vN), normalize(vV))), 0.8);
      float core = exp(-s * 4.0);
      float cells = uPamb * pow(0.5 * (1.0 + cos(s * 34.0)), 6.0) * exp(-s * 3.5);
      float turb = 0.92 + 0.08 * noise(vec3(s * 6.0 - uTime * 4.0, vUv.x * 6.0, uTime * 0.7));
      vec3 col = vec3(0.62, 0.7, 1.0) * core * 0.8 + vec3(1.0, 0.86, 0.62) * cells * 1.1 + vec3(1.0, 0.5, 0.24) * (1.0 - core) * 0.4 * uPamb;
      float a = chord * turb * uThrottle * (1.0 - s) * 0.8;
      gl_FragColor = vec4(col * a, a);
    }`,
  transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide,
})

/** ANSYS-style rainbow colour map (blue → cyan → green → yellow → red) for t ∈ [0, 1] */
export const rainbow = (t: number, out = new THREE.Color()): THREE.Color => {
  const x = Math.max(0, Math.min(1, t))
  return out.setHSL(0.66 * (1 - x), 1, 0.5)
}

export interface WindField {
  readonly mesh: THREE.InstancedMesh
  /** wind(e, n, u) → ENU wind (m/s) at a point relative to the ship; colours span the field's min..max (ANSYS default) */
  readonly update: (wind: (de: number, dn: number, du: number) => readonly [number, number, number]) => { readonly min: number; readonly max: number }
  readonly setOpacity: (k: number) => void
}

/** a 3-D grid of arrow glyphs around the vehicle (local ENU, ship at the origin) */
export const makeWindField = (nx = 7, ny = 7, nz = 5, spacing = 28): WindField => {
  // ANSYS vector plot: slim shaft + cone head, saturated colours (not tone mapped)
  const shaft = new THREE.CylinderGeometry(0.22, 0.22, 0.78, 6).translate(0, 0.39, 0)
  const head = new THREE.ConeGeometry(0.75, 0.22, 10).translate(0, 0.89, 0)
  const geo = mergeTwo(shaft, head)
  const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, toneMapped: false, transparent: true, opacity: 1 })
  const count = nx * ny * nz
  const mesh = new THREE.InstancedMesh(geo, mat, count)
  mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(count * 3), 3)
  mesh.frustumCulled = false
  const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), up = new THREE.Vector3(0, 1, 0), d = new THREE.Vector3(), c = new THREE.Color()
  const samples = new Float64Array(count * 7)
  const update: WindField["update"] = (wind) => {
    let i = 0, lo = Infinity, hi = 0
    for (let a = 0; a < nx; a++) for (let b = 0; b < ny; b++) for (let k = 0; k < nz; k++) {
      const de = (a - (nx - 1) / 2) * spacing, dn = (b - (ny - 1) / 2) * spacing, du = (k - (nz - 1) / 2) * spacing
      const [we, wn, wu] = wind(de, dn, du)
      const sp = Math.hypot(we, wn, wu)
      samples.set([de, dn, du, we, wn, wu, sp], i * 7)
      lo = Math.min(lo, sp); hi = Math.max(hi, sp)
      i++
    }
    // readable range: at least 4 m/s wide, snapped to 0.5 m/s (a 0.3 m/s spread would paint the full rainbow)
    const span = Math.max(hi - lo, 4)
    lo = Math.max(0, Math.floor(((lo + hi) / 2 - span / 2) * 2) / 2)
    hi = Math.ceil((lo + span) * 2) / 2
    for (i = 0; i < count; i++) {
      const [de, dn, du, we, wn, wu, sp] = samples.subarray(i * 7, i * 7 + 7)
      d.set(we, wu, -wn)
      const len = 4 + 1.0 * sp
      if (sp > 1e-3) q.setFromUnitVectors(up, d.normalize()); else q.identity()
      m4.compose(new THREE.Vector3(de, du, -dn), q, new THREE.Vector3(1, len, 1))
      mesh.setMatrixAt(i, m4)
      mesh.setColorAt(i, rainbow((sp - lo) / (hi - lo), c))
    }
    mesh.instanceMatrix.needsUpdate = true
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
    return { min: lo, max: hi }
  }
  const setOpacity = (k: number): void => { mat.opacity = k; mat.depthWrite = k > 0.99 }
  return { mesh, update, setOpacity }
}

/** merge two non-indexed-compatible geometries (positions + normals) */
const mergeTwo = (a: THREE.BufferGeometry, b: THREE.BufferGeometry): THREE.BufferGeometry => {
  const ga = a.toNonIndexed(), gb = b.toNonIndexed()
  const pos = new Float32Array([...(ga.attributes.position.array as Float32Array), ...(gb.attributes.position.array as Float32Array)])
  const g = new THREE.BufferGeometry()
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3))
  g.computeVertexNormals()
  return g
}
