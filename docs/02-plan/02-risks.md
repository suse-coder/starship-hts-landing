# Risks

| Risk | Impact | Mitigation in the engine |
|---|---|---|
| MHD model is a prescribed-flow approximation (no coupled CFD) | force could be off by ×2–3 | momentum bound caps it; κ/B/seed exposed as design parameters; authority ratio recorded |
| Hall closure κ unknown | the whole nose-first concept hinges on it | Q4 default 0.03 with the failing conservative value documented |
| Cold-gas endurance | loss of roll/heading → miss | free axes, deadbands, 2 t N2, gas use recorded per phase |
| Predictor/flight mismatch | range errors (seen: 700 km before ADR-8) | consistent tables, online MHD-drag estimate, range bias |
| Pitch-up transient | 4–6 g, α overshoot | pitch-up at the loft's q minimum |
| Bare steel heating | > 1150 K windward | barbecue roll, α schedule ≤ 14°, thermal mass |
| Equilibrium chemistry above 70 km | n_e error | MHD/heating small there; documented in map/20 |
| Pad coordinates ±200 m | catch geometry | catch is relative to the modelled pad, consistent |
