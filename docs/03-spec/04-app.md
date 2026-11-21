# spec/04 — Browser application

**A0 Static, browser-only, public (user, 2026-11-20).** The application is a static site: HTML,
JS, workers and assets only, with **no server-side code at runtime**. Everything, including the
real-gas tables, mission targeting and the full 6-DOF mission, is computed in the visitor's browser.
It is published publicly on **GitHub Pages**:

- Repository: public GitHub repo created with the `gh` CLI **after the mission works end to end**
  (catch reached in the browser and verified with screenshots).
- Deployment: a GitHub Actions workflow (`.github/workflows/pages.yml`) on push to `main`:
  checkout → setup Bun → `bun install` → typecheck + tests → `vite build` of `packages/app` →
  `actions/upload-pages-artifact` → `actions/deploy-pages`. Chosen over a `gh-pages` branch
  because no build output is committed and every deploy is tied to a tested commit.
- Vite `base: "./"` (relative URLs) so the site works under `https://<user>.github.io/<repo>/`;
  workers are bundled as ES modules; no CDN imports at runtime.

**A1 Load sequence.** On page load the Web Worker runs the **entire mission first**. The main
thread shows a loading screen with a live animation and progress (phase name, simulated mission time,
percentage, a small trajectory sketch growing as progress messages arrive). When the recording
arrives (transferred, not copied), the 3D view and the controls appear. Target: ≤ 30 s on a typical
laptop (G4). Re-simulation from the Design panel uses the same worker and loading overlay.

**A2 Playback.** Pure playback of the recording, nothing re-simulated:
a timeline **slider** to scrub back and forth instantly (any time, either direction, 60 fps); a
**play/pause** button; playback speed (×1 … ×1000, default fast enough to see the whole return in
about a minute); phase jump buttons. State at any time comes from interpolating the recording
(`locate`/`valueAt`), never from integrating.

**A3 Look and feel: Fusion 360 workspace, ANSYS-style results (user, 2026-11-21).** No ribbon and
no toolbars. The screen is the 3D view, with overlays floating on it:

- **Browser tree (left overlay)**, like Fusion 360's browser: a collapsible tree
  `Ship 42 → hull, groups (structure, propulsion, TPS, HTS coil, RCS, avionics) → components`,
  plus `Physics overlays` (plasma sheath, bow shock, field lines, electrodes, plumes, cold-gas jets,
  heat glow) and `Environment` (Mechazilla and pad, Earth, trajectory). Every node has an eye toggle
  that shows or hides it in the 3D view. Hiding the hull reveals the internal components in place.
- **Timeline (bottom overlay)**, like Fusion's timeline: phase-coloured track with event ticks and
  a draggable playhead. Dragging anywhere on the track, or the slider, scrubs instantly in either
  direction. This is only enabled after the full simulation has finished (A1). Buttons: start,
  previous phase, play/pause, **stop** (pause and return to T+0), next phase, end, plus a playback
  speed selector.
- **Exploded view checkbox**: checking it pauses playback and animates the parts outward with a
  spring. Unchecking it springs the parts back together quickly (stiffer spring), and playback
  resumes if it was running before.
- **Results inspector (right overlay, collapsible)**, like an ANSYS results panel: grouped live
  readouts (flight, plasma/MHD, thermal, magnet, propulsion, catch) and charts with a time cursor.
- Light, translucent panels with a blue accent (Fusion style) over the physically-coloured scene
  (sky by altitude, incandescent steel, plasma glow).

**A4 Realistic rendering (user, 2026-11-21).** The world is rendered, not just the ship. It must look
beautiful at landing, and the plasma must be photorealistic and clearly visible:
- **Environment "Realistic"** (default; "Studio" = Fusion white is the alternative): a physically
  based sky (Rayleigh/Mie) with the Sun at its true position for the recorded UTC time and the pad
  location; from orbit, Earth with an atmosphere rim and the terminator; Starbase coast (scrub,
  beach, Gulf of Mexico with a moving water surface); a detailed Mechazilla (lattice tower,
  chopsticks with recorded opening, carriage, launch mount); real-time shadows.
- **Plasma**: emission shader on the shock layer around the nose, colour from the black-body
  temperature T₀ and brightness from the electron density n_e (both recorded), turbulent flowing
  noise, a fading wake trail along −v̂, a coloured point light that lights the steel, and bloom.
- **Engine plumes**: methalox plume shader (blue core, Mach diamonds at sea level, expanding with
  altitude) with bloom; cold-gas jets as faint puffs.
- **Wind (ANSYS-style)**: an arrow glyph field around the vehicle showing the wind vector (forecast
  profile + recorded local wind), coloured with a rainbow map (blue → red) by speed, with a colour
  legend (m/s). There is a tree toggle ("Wind field").
- Post-processing: ACES tone mapping and bloom. All assets are generated in code or bundled
  (no runtime CDN).

- **Run**: on load (and on "Re-simulate"), a Web Worker builds the World (tables, targeting) and
  runs the mission; progress is streamed; the `Recording` (Float64 channels + Float32 heat map) is
  transferred to the main thread.
- **Timeline**: a slider over mission time with phase-coloured segments and phase jump buttons;
  play/pause; speed ×0.25 … ×1000; mission elapsed time and UTC.
- **3D** (`@ship/view`, three.js): floating origin at the ship; Earth with atmosphere glow and the
  planned ground track; the ship built from the same outer mould line (lathe) with bare-steel
  heat colouring per thermal cell, PICA-X cap, engines; plasma sheath and bow shock sized from the
  recorded standoff and electron density; coil field lines; electrode sector glow by loading;
  Raptor plumes and cold-gas puffs from the recorded duties; Mechazilla tower, chopsticks (recorded
  arm opening), launch mount and pad.
- **Exploded view** button: every part (nose cap, coil, cryostat, electrodes, header tanks,
  tanks/domes, payload bay, engines, RCS, pins) slides along its explode vector with labels.
- **Cameras**: chase, orbit (drag), tower, globe.
- **Telemetry**: grouped readouts (flight, plasma/MHD, thermal, magnet, propulsion, catch);
  charts with a time cursor (altitude/velocity, q & g, heat flux & temperatures, MHD force &
  authority).
- **Design panel**: B, κ, seed fraction, nose radius, PICA thickness, N2, wind/turbulence → re-run.

## Requirement status

| ID | Requirement | Status |
|---|---|---|
| A0.1 | Static site, no server code, relative base, bundled assets | [x] dev · [ ] production build verified |
| A0.2 | Public repo + GitHub Actions Pages deploy | [ ] (M8, after user approval) |
| A1.1 | Whole mission simulated in a worker on load, progress sketch (crisp axes) | [x] |
| A1.2 | Timeline locked until the run completes | [x] |
| A2.1 | Drag-to-scrub playhead/track, both directions | [x] |
| A2.2 | Start / previous / play-pause / stop / next / end, speed ×1…×1000 | [x] |
| A3.1 | Fusion-style browser tree with eye toggles (parts, overlays, environment) | [x] |
| A3.2 | ViewCube (crisp), navigation bar (home, fit, camera, environment) | [x] |
| A3.3 | Exploded-view checkbox: pause → spring apart → spring back → resume | [x] |
| A3.4 | Results inspector (telemetry, charts, events) | [x] |
| A3.5 | **Right-mouse pan** (2026-11-21): left-drag orbits, right-drag pans the target in the screen plane and persists across frames; context menu suppressed; Fit/Home resets the target | [x] `view/src/index.ts` (`RIGHT: PAN`, `screenSpacePanning`, `contextmenu`); review/02 §5 |
| A3.6 | **Altitude graph** (2026-11-21): small transparent altitude-vs-mission-time graph top-right, left of the ViewCube; phase/altitude scale, live marker and current altitude during playback | [x] `app/index.html#altgraph`, `app/src/main.ts` `buildAltGraph`/`drawAltGraph`; review/02 §5 |
| A4.1 | Physical sky with the true Sun, default on (TS-13) | [x] `r-0.62.png` |
| A4.2 | Photorealistic Earth: Blue Marble day, Black Marble night lights, single-scattering atmosphere, eclipse; stars only in the background (Earth occludes them, 2026-11-21) | [x] V: Earth region 0.000 % bright with Earth shown, 0.033 % with Earth hidden; stars only above the limb |
| A4.3 | Starbase coast, Gulf water, detailed Mechazilla (T03), volumetric clouds (T02, TS-14), shadows | [x] coasts/tower verified earlier; global cloud shell + local deck verified 2026-11-21 (`final.log`, `final2.log`, C4 screenshots) |
| A4.4 | Photorealistic plasma (windward shock-layer emission, wake, light), visible in descent | [~] |
| A4.5 | Methalox plume shader (no flicker), cold-gas puffs with a low-passed opacity (25 Hz duty → steady plume, 2026-11-21) | [~] N₂ was constant after the flip in the verified run; landing visual frames invalid (Vite reload), so the puffs are not visually confirmed |
| A4.6 | ANSYS-style wind arrows with legend (≥ 4 m/s range) | [~] |
| A4.7 | Detailed vehicle (spec/07) with PICA-X honeycomb (T04) | [x] 41/41 part rows built; PICA-X maps and geometry at real scale; exploded-view acceptance pending |
| A4.10 | **No flicker or popping** during playback (measured: `.tmp/pw/flicker.cjs` luminance series) | [x] 2026-11-21: at T+30:29.9 white pixels 0.15 %, mean 116.8 → 119.4 smoothly; V: 12 frames, max mean jump 0.08, zero blink, scene white 0.000 % (the 2.3 % full-frame white is UI chrome) |
| A4.8 | **Thermal-margin view** (user, 2026-11-21): hull false-colour map of steel temperature vs 301/30X strength (TS-16: ≤ 700 K nominal, ≤ 900 K reduced, ≤ 1150 K design limit, < 1400 K strength loss, ≥ 1400 K failure; k(T) yield retention) with an ANSYS legend; PICA bondline vs 560 K; silica tile surface vs 1530 K and tile backing vs 600 K (Q23); results show the hottest station, the margins, the time over limits and a hot-spot table | [x] V (earlier trajectory): tile surface 1761 K FAIL, backing 328 K PASS, bondline 615 K FAIL; tile-surface table (37.4 m → 1761 K, −231 K); 25/56 cells over 1530 K; outcome LOST. Current measured values are in spec/05/spec/11. |
| A4.9 | **MHD effect view**: shock standoff with the field Δ (Fresnel shell) vs without Δ₀ (recorded channel `standoff0`, dashed); heat-flux factor, q with/without, stagnation heat load with/without, Stuart number in the results | [~] implemented, screenshot pending |
| A4.11 | **Recording cache** (2026-11-21): the stored run is reused on reload while the physics sources are unchanged (hash in `app/vite.config.ts`, IndexedDB in `app/src/simCache.ts`); `?resim` and the Re-simulate button force a new run | [~] implemented, reload test pending |
| A4.12 | **Landing slow motion** (user, 2026-11-21): the flip, landing, terminal and catch phases never play faster than the landing rate (default ×0.5; ×1, ×0.5, ×0.2 or ×0.1 next to the speed selector); the timeline header shows the slow-motion rate | [x] V: `#landing-speed` default 0.5; landing burn header reads `Landing burn · slow motion ×0.5` |
| A4.13 | **Tab icon** (user, 2026-11-21): SVG favicon (silver ship, HTS coil ring, plume, Earth limb) served from `app/public/favicon.svg` | [x] V: HTTP 200, `image/svg+xml`, 980 bytes; DOM `<link rel="icon">`; main-thread 32 px render is a clear rocket |
| A4.14 | **Global cloud shell + local volumetric deck** (2026-11-21): a planet-wide cloud shell with a regional volumetric cumulus deck that grows with altitude and fades into the shell; no flat painted clouds and no hand-off ghost | [x] `view/src/cloudsGlobal.ts`, `view/src/clouds.ts`, `view/src/index.ts`; `final.log` (400/50/30 km), `final2.log` (5/2/0.3 km); screenshots `.tmp/shots/agentC4/final-limb.png`, `horizon-h2.png`, `horizon-h5.png` |
| A4.15 | **Predicted-path overlay** (2026-11-21): a tree toggle shows the recorded future path (cyan) and, when present, the landing MPC plan (pale); the two are clearly distinguished so the plan is not mistaken for the flown path | [x] `view/src/prediction.ts`, tree "Predicted path (MPC)"; `app4-path-off.png` / `app4-path-on.png`, `app4.log`; review/02 §"Why the two overlay lines differ" |
