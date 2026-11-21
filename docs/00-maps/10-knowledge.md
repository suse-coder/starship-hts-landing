# Knowledge map — sources and trust

| Topic | Source (read 2026-11-20) | What we take | Trust |
|---|---|---|---|
| Flight 15 timeline | Wikipedia "Starship flight 15" (extract via API) | launch 2026-11-13 22:47:36 UTC, Pad 2, Ship 42 (Block 3), ~275 km orbit, ~14 orbits, caught at T+22:32:30 (2026-11-14 21:20:06 UTC), deorbit toward the Pacific then glide to Starbase | high for dates/places; orbit inclination **not** given → assumption A-INC |
| Starship V3 ship | Wikipedia "SpaceX Starship (spacecraft)", "SpaceX Starship", "SpaceX Raptor" | height 52.1 m, Ø 9 m, rings 1.83 m × 3.97 mm ≈ 1.6 t, ~100 t dry (2021 statement), 6 Raptors (3 SL + 3 RVac), cold-gas RCS on ullage gas, header tanks in the nose (LOX top), heat shield 18 000 silica tiles ≈ 10.5 t (Block 1), catch points in the nosecone | medium (vehicle masses are not published; budget in spec/01 is a reconstruction) |
| Raptor 3 | Wikipedia "SpaceX Raptor" (May 2026 V3 update) | SL 250 tf (2.45 MN), RVac 275 tf, SL engine 1525 kg | medium; Isp_SL 350 s assumed (A-ISP) |
| Flap mass | user-provided analysis (NSF forum estimates) | 4 flaps ≈ 12–15 t | low/medium |
| PICA / PICA-X | Wikipedia "Atmospheric entry" §PICA; Tran et al., NASA TM-110440 (1997) (from memory, not re-read) | virgin 0.27 g/cm³, char ≈ 0.22, k ≈ 0.2 W/m K (virgin), Stardust peak 1.2 kW/cm²; PICA-X/PICA-3 are proprietary → PICA properties used as proxy | medium |
| 30X / 304L steel | general metallurgy (from memory) | ρ 7900, cp 450–650 J/kg K, ε (oxidised, coated) 0.85, strength falls above ~1100–1150 K, melting ≈ 1670 K | medium |
| MEESST | CORDIS project 899298 page; user-provided summaries | compact cryostat + HTS magnet; ~0.66 T demonstrated at the probe surface, target 1–2 T; MHD heat-flux reduction and "magnetic window" for blackout | medium (only qualitative project text was readable) |
| MHD theory | textbook relations (Stuart number, generalized Ohm's law with Hall term, Faraday loading factor), Bush 1958, Poggie & Gaitonde 2002 (from memory) | standoff growth with Q, Hall-limited conductivity, momentum bound | medium; correlations are labelled as such in spec/02 |
| Equilibrium air | first principles (partition functions; spectroscopic constants from memory) | 9-species composition, verified against well-known values (spec/05) | high for the trend, ±10–20 % for ne |
| Heating | Sutton–Graves (k = 1.7415e-4), Tauber–Sutton radiative, Lees distribution, Eckert reference enthalpy | standard engineering correlations | medium (±30 %) |
| Aerodynamics | modified Newtonian; Jorgensen, NASA TR R-474 (1977) | standard engineering methods | medium (±20 % on coefficients, worse transonic) |
| Atmosphere | U.S. Standard Atmosphere 1976 | exact layers to 86 km, tables above | high |
| Geomagnetic field | IGRF degree-1 (2025, rounded) | tilted dipole | medium |
| Pad 2 location | public maps (approx.) | 25.9925 N, 97.1573 W | low (±200 m) — assumption A-PAD |

Web search was unavailable (HTTP 402) during this session; sources were fetched directly.
