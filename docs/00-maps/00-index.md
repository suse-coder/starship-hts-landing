# Map of maps

| Map | Question it answers |
|---|---|
| [10-knowledge](10-knowledge.md) | Which sources exist (Flight 15, Starship V3, PICA, steel, MEESST/HTS, aero/heating methods), what they contain, how far to trust them |
| [20-physics](20-physics.md) | Which physical model is used for what, in which regime, with which equation, in which file — and what it neglects |
| [30-prior-art](30-prior-art.md) | Which papers and open-source projects already solve each sub-problem (powered descent, SCvx/G-FOLD, entry NPC, INDI, allocation, MHD), their licences, and what we adopted |
| [50-code](50-code.md) | Which package/file implements what, the dependency graph, the data flow of one step |
| [60-glossary](60-glossary.md) | What each term means here (Stuart number, Hall closure κ, loading factor, trim lift loss, barbecue roll, …) |
| [plan/05 open questions](../02-plan/05-open-questions.md) | What I would ask you, and the default the engine runs with until you answer |

**Rule zero (spec/00 G0):** everything happens through real, simulated physics: no fake rules,
forces or overrides to make it work. The HTS coil, not flaps, stabilises and steers in the plasma
phase. Audit: review/00.

## The one-paragraph picture

Ship 42 (Flight 15, 13–14 Nov 2026) re-entered from a ~275 km orbit and was caught by the Pad 2
tower. This project re-flies that return with a **modified ship**: no flaps, a **PICA-X nose cap**,
the **real silica tile heat shield** on the barrel over 301/30X stainless-steel backing (Q23), and
a **superconducting (REBCO) coil in the nose** that interacts with the shock-layer plasma. The
engine is a modular TypeScript physics simulation (real-gas thermochemistry from partition
functions, Newtonian/Jorgensen aerodynamics integrated over the hull, Sutton–Graves/Lees/Eckert
heating, PICA-X ablation, tile/steel thermal response, a Biot–Savart coil field and a Lorentz-force
shock-layer model with Hall effect and caesium seeding, magnet circuit and cryogenics, cold-gas RCS,
Raptor 3 propulsion, J2 gravity, US-1976 atmosphere, winds/Dryden turbulence, Mechazilla contact)
driven by flight software (3-DOF predictor–corrector guidance, rate-command attitude control,
electrode/RCS/gimbal allocation). It runs the whole return — deorbit, coil ramp, entry, MHD-trimmed
nose-first glide, pitch-up, high-α glide with barbecue roll, belly flop, flip, landing burn, catch —
in a Web Worker; the measured headless worker run is ≈ 26–28 s (G4 ≤ 30 s met on that evidence, the
10–15 s target not met). The browser scrubs the recorded mission in 3D. **The catch is not yet
achieved; the current mission ends `lost`.**

**What physics forced (research/04):** the literal concept "nose-first all the way, coil
prevents tumbling, flip near the ground" cannot work: nose-first Starship has a ballistic
coefficient ≈ 2000 kg/m², so dynamic pressure at the end of the plasma phase is 20–150 kPa,
and any rotation to tail-down at that q means 10–30 g; the coil has no plasma (no authority)
below ≈ 2.5 km/s. The engine therefore flies the concept where it is physical — an
**MHD-trimmed nose-first lifting glide** (α 10–14°, the coil is the only pitch/yaw actuator) —
then a **controlled pitch-up at the loft's dynamic-pressure minimum** into the hull's own passive
aerodynamic trim (α ≈ 51° hypersonic → 84° subsonic), with a **barbecue roll** to spread the heat,
and finally the belly flop, flip and catch.

## Status board

| Track | State | Evidence |
|---|---|---|
| Research: Flight 15, Starship V3, materials, MHD/HTS | ✅ | research/01–03 |
| Feasibility analysis of the flapless concept | ✅ (the literal concept is infeasible; physical variant chosen) | research/04, journal/2026-11-20 |
| Engine packages (math, earth, gas, craft, aero, heat, mhd, tower, gnc, sim) | ✅ typecheck clean | map/50 |
| Real-gas air + shock table | ✅ checked vs. known equilibrium-air values | spec/02 §2, tests |
| Deorbit → entry → MHD glide → pitch-up → high-α → belly flop | ✅ simulated, closed loop | `bun run mission` |
| Flip → landing → catch | 🔴 **open — `lost`**: pins at the rail plane 11 m off, climbing 3.0 m/s; best 6-DOF optimiser candidate 6.8 m / 11.7 m/s, lost | `mission_verify.log`, bench/02, review/02 |
| TPS (tile / bondline) | 🔴 **over the LI-900 limits**: 1650–1684 K vs 1530 K, 695–701 K vs 560 K; backing passes | spec/11 §3.2, spec/05 |
| T‑mc (wind/κ/B/mass) | ⬜ **not run** | spec/11 §3.3 |
| Browser app (Fusion-360 look: browser tree, ViewCube, animation timeline, exploded spring, results inspector) | ✅ first version: whole mission simulated in a worker on load (≈ 26–28 s headless), playback verified in Chromium | spec/04 A0–A3 |
| Rendering: global/local clouds, predicted path, right-mouse pan, altitude graph | ✅ implemented; clouds and predicted path visually verified 2026-11-21 | `.tmp/pw/final.log`, `final2.log`, C4 screenshots; review/02 |
| Public GitHub repo + Pages deploy (Actions) | ⬜ **blocked** until M3, TPS, T‑mc and user approval | [spec/11](../03-spec/11-release.md) |
| Physical-fidelity audit (G0, R1–R10) | ✅ plant clean; idealisations I1–I8 listed | review/00 |
| Review, benchmarks | 🔄 review/01 NO-GO, review/02 written; bench/01 and bench/02 run | review/, bench/ |
