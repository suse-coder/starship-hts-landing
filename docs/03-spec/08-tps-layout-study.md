> Mass note: the vehicle is 143.65 t after the T08 caesium double-count fix (was 144.05 t);
the trade deltas below are unchanged within 0.3 %.

# spec/08 — TPS layout study (Q30): can the tile shield survive the 99 m/s entry?

Status: study complete (2026-11-21, agent Q30). **No code, mass input, trajectory or vehicle was
changed**; nothing in `packages/` was edited. Every number below is reproducible from
`.tmp/agentQ30/` (§10) and is tagged:

| tag | meaning |
|---|---|
| **(R)** | recorded by the 99 m/s mission (`.tmp/mission_final.log` → `summary`) |
| **(C)** | computed by this study (`packages/heat` + `packages/craft` called from a scratch script) |
| **(S)** | sourced input already in the repo, with its file/line |
| **(A)** | assumed / estimated here, with the reasoning |

## 1. The question this study answers

Q30 (plan/05) records that at the only entry that closes the 99 m/s deorbit, the tile surface peaks
**1760 K (R)** against a 1530 K LI-900 limit (S, `heat/src/limits.ts:22`, `craft/src/tiles.ts:19`)
and a 1673 K sourced SpaceX claim (S, `craft/src/tiles.ts:70`). The backing is 328 K (R) against a
600 K tile-bond limit (S/A, `limits.ts:23`). The hottest tile cell is station **x = 37.40 m (R)**,
which is 0.53 m aft of the PICA boundary x0 = 37.93 m (C, `designPica`: `x0 = Lc − 0.5 = 38.43 − 0.5`).

Four questions were put to this study:

1. does extending PICA aft over the hot strip fix the tile surface, and at what mass/CG cost;
2. what does that do to the PICA bondline (614 K (R), over the 560 K limit (S)) and the recession;
3. the alternatives — a hotter tile, a thicker tile, or a flux reduction;
4. the minimum physical change that makes the protection survive both limits.

The answer in one sentence: **no TPS-only change survives the LI-900 limit — the tile peak is not a
coverage problem but an attitude problem — and the extension buys −27 K (C) for +0.38 t (C) and
nothing under the conservative bound (C).**

## 2. Method, and why the replay can be trusted for comparisons

No mission was run. The saved 30 s sample rows of `.tmp/mission_final.log` carry `t, h, V, q̄, α` and
the logged stagnation flux `qs`; from each row the study rebuilds the exact `HeatInput` the simulator
used (`ρ = 2q̄/V²` (C), `atmosphere(h)`, `lookupStag(SHOCK_TABLE, h, V)`, `vhat` in the pitch plane,
and the MHD stagnation factor backed out of the logged `qs`), calls `surfaceHeating` on the real
`makeThermalGrid`, and advances `thermalStep` at 1 s from t = 1690 s to 2650 s. That is the same
physics as the mission, driven by the mission's own recorded trajectory.

Audit against the record (all in K):

| quantity | recorded (R) | replay (C) | Δ |
|---|---|---|---|
| tile surface peak | 1759.8 | 1738 | −1.2 % |
| tile peak location | x 37.40 m | x 37.40 m | same |
| tile backing peak | 327.5 | 376 | +15 % |
| PICA surface peak | 1964.0 | 1959 | −0.3 % |
| PICA bondline peak | 613.8 | 747 | +22 % |

The two *surface* peaks match to 1 %, which is what the tile verdict depends on: a surface at
1738 K is radiation-bound (`εσT⁴` dominates the energy balance by ~20:1), so it is insensitive to
the pulse shape. The two *integral* quantities (backing, bondline) read **15–22 % hot**, because
re-evaluating a trajectory sampled every 30 s broadens the pulse while keeping its peak. The replay
is therefore **conservative on every integrated quantity** and its bondline deltas are upper bounds;
absolute bondline values are quoted from the record (614 K), deltas from the replay.

Two model-resolution facts that shape the whole study (C):

- the thermal grid is `THERMAL_STATIONS = [52.1, 51.85, 51.3, 50.5, 49.2, 47.4, 45.2, 42.6, 39.8,
  37.4, 35.0, 31.0, 25.0, 18.0, 10.0, 3.0]` (S, `heating.ts:33`) with 8 rays per station. The
  tile/PICA boundary sits **between** 39.8 (PICA) and 37.4 (tile), so the grid resolves the seam
  only to 2.4 m. x0 = 37.40, 36.50 and 35.50 flip **exactly the same cell** and are thermally
  identical; only their PICA mass differs.
- "x = 37.40 m" is a station label, not a measured peak position. The physical peak lies somewhere
  in the 37.4–39.8 m cell, i.e. the true distance from the seam is 0.53–2.4 m, not exactly 0.53 m.

## 3. Where the heat actually goes — the finding that decides Q30

The tile has **two** hot pulses, and the second one sets the peak (C):

| t (s) | phase | α (R) | V (m/s, R) | tile surface (C) | hottest cell (C) |
|---|---|---|---|---|---|
| 2125 | entry, low α | 10.6° | 6621 | 1432 K (first peak) | x 10.0 m |
| 2306 | pitch-up → high α | 43.3° (commanded 80°) | 5387 | **1738 K (peak)** | x 37.40 m |
| 2375 | high α | 22.6° | 3947 | 1449 K | x 37.40 m |

The tile exceeds 1530 K for **35 s, from t = 2285 s to 2320 s (C)** — entirely inside the pitch-up /
high-α window, and *not* at the peak-stagnation-flux moment (t = 2095 s, qs = 96 W/cm²). This is the
central point: **the tile peak is set by the pitch-up attitude, not by the entry heat pulse.** Note
that the first, low-α pulse on its own (1432 K, and it peaks on the **aft** barrel at x = 10 m, not
near the seam) is only 98 K under the limit, so there is no reserve to spend on a hotter second pulse.

This is possible because the tile surface is optically thin: its hot-face node is 2.1 mm thick
(`TILE_FR[0] × 3 cm`, S `thermal.ts:34`), i.e. 389 J/m²K (C), so the surface reaches radiative
equilibrium with a sub-second time constant. The peak surface temperature therefore follows the
**instantaneous** flux (`T ≈ (q/εσ)^¼`) and not the integrated heat load — which is why a 35 s pulse
can set the record, and why the tile is insensitive to its thickness (§6).

Cold-wall flux per station (max over the 8 rays, C) at the two pulses, kW/m²:

| x (m) | cover | t = 2095 s, α 10.1° | t = 2306 s, α 43.3° | ratio to 37.4 m at 43.3° |
|---|---|---|---|---|
| 52.10 | PICA | 946 | 301 | 0.62 |
| 51.30 | PICA | 752 | 419 | 0.87 |
| 49.20 | PICA | 782 | 630 | 1.30 |
| 47.40 | PICA | 784 | 590 | 1.22 |
| 45.20 | PICA | 578 | 568 | 1.18 |
| 42.60 | PICA | 246 | 542 | 1.12 |
| 39.80 | PICA | 56 | 509 | 1.05 |
| **37.40** | tile | **43** | **484** | **1.00** |
| 35.00 | tile | 41 | 471 | 0.97 |
| 31.00 | tile | 38 | 453 | 0.94 |
| 25.00 | tile | 75 | 432 | 0.89 |
| 18.00 | tile | 143 | 414 | 0.86 |
| 10.00 | tile | 199 | 397 | 0.82 |
| 3.00 | tile | 206 | 386 | 0.80 |

Two consequences, both (C):

- at the pitch-up attitude the **entire windward barrel is at 0.80–1.00× the hot-station flux**
  (turbulent flat plate, `St ∝ s^-0.2`: 45 m of running length is only 20 % cooler than 15 m);
- the equilibrium surface temperature follows `T ∝ q^¼`, so a **2.7 % flux reduction (37.4 → 35.0 m)
  buys 0.7 % of temperature** — 11 K, not 200 K.

That is why "extend PICA over the hot strip" cannot work on this trajectory: there is no hot
*strip*, there is a hot *barrel*. The only x0 that removes the problem is 3.0 m — i.e. no tiles at
all (§4).

## 4. Task 1 — extend PICA aft: area, mass, CG, new tile peak

The parametric family is `picaThicknessAt` (S, `config.ts:231`): 6.5 cm on the cap, 2.5 cm at x0,
quadratic taper, `xc = xCapEnd − 1 = 49.47 m`. Two ways to move x0 are reported:

- **parametric** — call the model as designed (`designPica` with the new x0). The taper span grows,
  so the *whole* nose gets thicker as well; that is the number the mass budget would take.
- **pure flank** — keep the existing nose thickness and add a constant 2.5 cm flank from the new x0
  to 37.93 m. This is what an engineer would actually build (and, unlike the parametric case, it
  isolates the effect of *coverage* from the effect of *thickness*).

Barrel area per metre of length: 2πR = 28.27 m²/m (C). PICA flank areal mass 0.025 × 270 + 3.0
= 9.75 kg/m² (C, from S 270 kg/m³ in `config.ts:215` and the 3 kg/m² carrier, `config.ts:227`); tile
areal mass 5.82 kg/m² (S, `TILE` 3 cm × 144 + 1.5). Net cost of a coverage swap: **+111 kg per metre
of length** (C) — Q30's "≈0.2 t per 2 m" is right for the pure flank (+0.11 t/m) and low by 1.8× for
the taper (+0.21 t/m).

### 4.1 Mass, dry/wet mass and CG

Baseline (C, via `massProps` + `initialConsumables`): dry **107.80 t**, wet **144.05 t**,
x_cg **27.629 m** from the aft skirt (the study baseline; spec/01 now has dry 107.40 t, wet 143.65 t after the T08 Cs fix). +x is toward the
nose, so a positive Δx_cg moves the CG forward (more stable). The model books the PICA and tile
items at a shell midpoint; this study recomputes both at their area centroid (baseline x_cg 27.604 m
instead of 27.629 m, a 25 mm bookkeeping difference) and reports **deltas** between variants on the
area-centroid convention.

All rows of this table are (C): `picaAreaMass`/`tileAreaMass` replicate `designPica`/`tiledArea`
exactly (the baseline reproduces the model's 331.6 m² / 4619 kg and 1072 m² / 6242 kg), and the mass
and CG columns come from `massProps` with `initialConsumables`.

| x0 (m) | mode | PICA m² | PICA kg | tile m² | tile kg | Δdry kg | dry t | wet t | Δx_cg mm |
|---|---|---|---|---|---|---|---|---|---|
| 37.93 | – | 331.6 | 4619 | 1072 | 6242 | 0 | 107.80 | 144.05 | 0 |
| 37.40 | flank | 346.6 | 4765 | 1057 | 6154 | **+59** | 107.85 | 144.10 | **+1.4** |
| 37.40 | param. | 346.6 | 4814 | 1057 | 6154 | +108 | 107.90 | 144.15 | +6.9 |
| 36.50 | flank | 372.0 | 5013 | 1032 | 6006 | +159 | 107.95 | 144.20 | +3.2 |
| 36.50 | param. | 372.0 | 5145 | 1032 | 6006 | +291 | 108.09 | 144.34 | +17.8 |
| 35.50 | flank | 400.3 | 5289 | 1004 | 5842 | +270 | 108.07 | 144.32 | +4.6 |
| 35.50 | param. | 400.3 | 5515 | 1004 | 5842 | +496 | 108.29 | 144.54 | +28.6 |
| 34.50 | flank | 428.6 | 5564 | 975 | 5677 | **+381** | 108.18 | 144.43 | **+5.3** |
| 34.50 | param. | 428.6 | 5885 | 975 | 5677 | +702 | 108.50 | 144.75 | +38.1 |
| 30.00 | param. | 556 | 7558 | 848 | 4937 | +1635 | 109.43 | 145.68 | +63.6 |
| 25.00 | param. | 697 | 9427 | 707 | 4114 | +2681 | 110.48 | 146.73 | +58.8 |
| 20.00 | param. | 839 | 11302 | 565 | 3291 | +3733 | 111.53 | 147.78 | +18.9 |

The CG cost is negligible everywhere the option could be considered (≤ 6 mm to x0 = 34.5 m flank,
≤ 38 mm parametric). The mass cost is not: **+0.38 t (flank) to +0.70 t (parametric) for the 3.4 m
that removes the hottest tile cell**, and +3.7 t for the 18 m that would (just) clear the sourced
1673 K claim. For context the landing propellant has ≈ 6 t of margin over the measured 20 t use
(Q24), so +0.38 t costs 6 % of that margin and +3.7 t costs 62 % of it — both on an entry that
already fails, so neither is a mass the vehicle can be shown to carry today.

### 4.2 The new tile surface

Replay (C), hottest surviving tile cell and peak:

| x0 (m) | hottest tile x (m) | tile peak (K) | Δ vs 1738 K | verdict vs 1530 (S) | vs 1673 (S) |
|---|---|---|---|---|---|
| 37.93 | 37.40 | 1738 | – | +208 K | +65 K |
| 37.40 | 35.00 | 1727 | −11 | +197 | +54 |
| 36.50 | 35.00 | 1727 | −11 | +197 | +54 |
| 35.50 | 35.00 | 1727 | −11 | +197 | +54 |
| 34.50 | 31.00 | 1711 | −27 | +181 | +38 |
| 30.00 | 25.00 | 1694 | −44 | +164 | +21 |
| 25.00 | 18.00 | 1678 | −60 | +148 | +5 |
| 20.00 | 18.00 | 1678 | −60 | +148 | +5 |
| 10.00 | 3.00 | 1649 | −89 | +119 | −24 |

Rows 37.40 / 36.50 / 35.50 are identical because they flip the same grid cell (§2). The **minimum
x0 that clears the sourced 1673 K claim is ≈ 10–20 m** — a PICA barrel over 64–85 % of the wetted
area, costing +3.7…+5.8 t — and the margin at 20 m is 5 K, which is not a margin. **No x0 clears
1530 K**: the only non-tile configuration is x0 = 3.0 m, and then the tiles are gone (§5).

**Conservative bound (R5).** Load every tile cell with the x = 37.4 m station flux, so that removing
the hottest strip provably cannot lower the peak (C): the tile peak is **1785 K for every x0 from
37.93 to 34.5 m** — the extension buys **exactly 0 K**. Under this bound the extension is +381 kg
(or +702 kg) of mass for no thermal benefit, and it also makes the entry heavier (out of scope here,
but it is the wrong direction).

## 5. Task 2 — PICA bondline and recession at the new area

Per-station peak wall state from the replay (C). Thicknesses are the model's (S `picaThicknessAt`);
values are replay-absolute and the bondline carries the +22 % conservative bias of §2:

| x (m) | PICA thickness (cm) | surface peak (K) | bondline peak (K) | recession (cm) |
|---|---|---|---|---|
| 52.10 | 6.50 | 1959 | 545 | 1.09 |
| 51.85 | 6.50 | 1948 | 549 | 1.18 |
| 51.30 | 6.50 | 1850 | 507 | 0.93 |
| 50.50 | 6.50 | 1599 | 417 | 0.39 |
| 49.20 | 6.32 | 1892 | 480 | **1.42** |
| 47.40 | 5.19 | 1862 | 615 | 1.29 |
| 45.20 | 4.09 | 1726 | 719 | 0.97 |
| **42.60** | 3.16 | 1667 | **747** | 0.53 |
| 39.80 | 2.61 | 1600 | 690 | 0.29 |
| 37.40 *(new, 2.5 cm flank)* | 2.50 | 1532 | **667** | 0.21 |
| 35.00 *(new, 2.5 cm flank)* | 2.50 | 1519 | 660 | 0.19 |

Answers:

- **The extension does not raise the bondline.** The governing cell is at **42.6 m** (747 K replay,
  **614 K record**, over 560 K (S)) and stays there for every x0: **Δ bondline = 0 K** for the pure
  flank. The new strip runs 667 K — 80 K (11 %) *cooler* than the governing cell, i.e. ≈ 548 K on the
  record's scale. The reason is that at the low-α pulse the ogive at 42.6 m sees 5.7× the flux of the
  new strip (246 vs 43 kW/m², C) and it is only 3.2 cm thick, whereas the new strip's surface barely
  reaches 1532 K. (The *parametric* variant lowers the peak to 660 K (−87 K), but that is the taper
  thickening the whole nose — 3.16 → 3.67 cm at 42.6 m — not the extra coverage; see its +702 kg.)
- **The bondline violation is unchanged and remains open (Q21).** It is not created or worsened by
  the layout, and no layout change addresses it.
- **Recession is not a problem for the strip**: 2.1 mm of 25 mm (8 %) at 37.4 m. The recession peak
  is 1.42 cm at 49.2 m (on the 6.3 cm ogive, 22 % of the local thickness) and is unchanged by the
  extension, consistent with the char/oxidation model above 1100 K (S `thermal.ts:185`).
- If a PICA barrel were ever built, 2.5 cm is too thin for the 560 K limit: the strip needs
  **3.5 cm (511 K) or 4.0 cm (448 K)**; 3.0 cm gives 583 K (C). At 3.5 cm the areal mass is
  12.45 kg/m², i.e. ≈ 13.3 t for the 1072 m² barrel, **+7.1 t** over the tiles, plus it is ablative.

## 6. Task 3 — the alternatives

| option | effect on the tile surface | effect elsewhere | verdict |
|---|---|---|---|
| PICA to x0 = 34.5 (flank) | 1738 → 1711 K (C); 0 K under the R5 bound | +381 kg, +5 mm CG, bondline unchanged | does not close Q30 |
| PICA to x0 ≈ 10–20 m | 1738 → 1649–1678 K (C) | +3.7…+5.8 t, non-reusable barrel | buys at most 24 K of margin for +5.8 t — no |
| SpaceX 1673 K hex tile claim (S) | raises the limit to 1673 K; peak 1760 K (R) still 87 K over | none | **still fails**, by 1.22× in flux |
| thicker tile (5 cm vs 3 cm) | **1738 K → 1738 K** (C, 2 cm and 5 cm both) | backing 376 → 300 K (C) | no surface effect at all |
| higher emissivity | ε 0.8 → 1.0 gives 1664 K (C, `1760 × 0.8^0.25`) | none | clears the 1673 K claim only at ε ≥ 0.98 (C); still 134 K over LI-900 |
| flux reduction | the only lever that works | attitude/trajectory, out of scope | see below |

**Why thickness does nothing** (C): at the peak the surface energy balance is
`q_abs = εσT⁴ + k·∂T/∂x`. At the hottest cell and t = 2306 s the model returns 435.2 kW/m² into
the surface, 414.0 kW/m² re-radiated (ε 0.8) and 21.2 kW/m² conducted into the tile — **4.9 %**.
Doubling the thickness halves a 5 % term, so it moves the surface by ~1 %. The replay confirms it
exactly: 2 cm, 3 cm and 5 cm all peak at 1738 K, while the backing moves 459 → 376 → 300 K.
Thickness is a *backing* lever, and the backing already passes (328 K record) — this is why the
design can only be fixed at the surface.

**The required flux reduction** (S/R + C). From the record: `(1760/1530)⁴ = 1.75×` and
`(1760/1673)⁴ = 1.22×`. The replay, which lets ~5 % of the absorbed heat conduct inward, returns
**1510 K at 1.75× and 1654 K at 1.22×** — the same requirement within 5 %, so the quoted factors
stand and are if anything slightly conservative.

**What sets the flux, and the size of the lever** (C). At the tile-peak state (51.5 km, 5387 m/s,
q̄ 12.4 kPa) the flux at x = 37.4 m is:

| α | q at 37.4 m (kW/m²) | factor vs 43.3° | equilibrium T, ε 0.8 |
|---|---|---|---|
| 43.3° (flown) | 484 | 1.00 | 1807 K |
| 40° | 465 | 1.04 | 1790 K |
| 35° | 426 | 1.13 | 1751 K |
| 30° | 376 | 1.29 | 1696 K |
| 25° | 315 | 1.53 | 1624 K |
| 20° | 248 | **1.95** | 1529 K |
| 15° | 49 | 9.9 | 1018 K |
| 10° | 20 | 24.0 | 817 K |

So the 1.75× requirement is met by holding the high-α plateau at **α ≲ 22°** instead of 43–48°, or
by pitching up at a lower speed: the same attitude at 4.5 km/s instead of 5.4 km/s scales the flux
by ≈ `(4500/5387)³ = 0.58` (A, Sutton–Graves cubic at fixed density) — i.e. roughly the factor that
is needed. The nonlinearity between 15° and 20° is the laminar→turbulent transition on the barrel
(`Re_θ/M_e` smoothstep 200–300, S `heating.ts:181`); below ≈ 18° the windward barrel stays laminar
and the flux collapses by an order of magnitude. Both levers change the entry's lift and range, so
they belong to the trajectory/guidance work (Q28), not to this study.

One combination deserves to be stated because a reviewer will ask: **a 34.5 m extension plus a
higher-emissivity coating plus the 1673 K claim** gives `1711 × (0.8/0.9)^¼ = 1663 K (C)` at ε 0.9 —
i.e. it nominally clears the *claim* by 10 K. It still misses the LI-900 design limit by 133 K, so it
does not make the protection survive; and it stacks an uncertified limit (S, claim only), an
un-sourced coating (ε 0.9 vs the 0.8 assumed today, A) and +0.38 t. It is listed here so that it is
not mistaken for an untried answer.

## 7. The trade table

One page, all of Q30 (record K, replay K, mass kg, CG mm):

| option | tile peak (K) | over 1530 / 1673 | Δdry (kg) | Δx_cg (mm) | PICA bondline | reuse |
|---|---|---|---|---|---|---|
| **as flown, x0 = 37.93** | 1760 (R) / 1738 (C) | +230 / +87 | 0 | 0 | 614 (R) **over 560** | yes |
| x0 = 37.4, flank | 1727 (C) | +197 / +54 | +59 | +1.4 | 614 (R), unchanged | yes |
| x0 = 35.5, flank | 1727 (C) | +197 / +54 | +270 | +4.6 | 614 (R), unchanged | yes |
| x0 = 34.5, flank | 1711 (C) | +181 / +38 | +381 | +5.3 | 614 (R), unchanged | yes |
| x0 = 34.5, taper | 1711 (C) | +181 / +38 | +702 | +38.1 | 660 (C taper) | yes |
| x0 ≈ 20, taper | 1678 (C) | +148 / **+5** | +3733 | +18.9 | fails | ablative |
| x0 ≈ 10, taper | 1649 (C) | +119 / −24 | +5845 | – | fails | ablative |
| no tiles (x0 = 3) | – | – | +7327 (C) | – | fails | ablative |
| 5 cm tile, x0 = 37.93 | 1738 (C) | +208 / +65 | +3087 | ~0 | 614 (R) | yes |
| ε = 1.0 coating | 1664 (C) | +134 / −9 | ~0 | 0 | 614 (R) | yes |
| 1673 K hex tile (S) | 1760 (R) | +230 / +87 | 0 | 0 | 614 (R) | claim only |
| **flux × 1/1.75** | **1510 (C)** | **−20 / −163** | 0 | 0 | 600 (C) | yes |
| flux × 1/1.22 | 1654 (C) | +124 / −19 | 0 | 0 | 689 (C) | yes |

## 8. Recommendation

**Recommended x0: none — keep 37.93 m.** Extending PICA over the strip is a mass purchase with no
thermal payoff (0 K under the conservative bound, −27 K under the nominal replay) and it does not
touch either binding limit.

**The minimum physical change that makes the protection survive both limits is not a TPS change.**
It is a **1.75× reduction of the absorbed flux at the pitch-up**: hold the high-α plateau at
α ≲ 22° (or pitch up at ≲ 4.5 km/s instead of 5.4 km/s). That is the only option that brings the
tile under 1530 K with the tile material the vehicle actually has, and it costs no mass. It is a
guidance/attitude constraint, so **Q30 should be reclassified from a TPS-coverage question to a
T-constraint on the pitch-up**, to be closed with Q28.

If the project instead accepts the sourced 1673 K claim (it should not — it is a public claim, not a
certified reuse limit, `tiles.ts:59`), the entry **still fails by 87 K**, so accepting the claim does
not close the item either. The only TPS way to clear 1673 K is PICA back to ≈ 20 m (+3.7 t,
5 K margin), which is a barrel redesign and still fails the 560 K bondline.

The third option Q30 lists — **accept the tile loss and report it (R8)** — is what the harness does
today and is the honest interim position: the 99 m/s entry is "outside limits" with 35 s above 1530 K,
reported and not clipped. It is not a survival answer, and it should not be adopted as one before the
pitch-up is re-flown.

**Design consequence, stated plainly:** Q30 is not solvable in TPS mass. The TPS is already at its
material limit; the remaining 230 K must come out of the trajectory. Consequently:

- the tile mass (6.24 t) and the PICA mass (4.62 t) do not change;
- the PICA bondline (614 K, over 560 K) is **unchanged by every layout option here** and stays open
  under Q21 — this study does not hide it and does not fix it;
- the tile backing (328 K) and the tile bond (600 K limit) pass with margin, so the tile thickness
  (3 cm assumed) is confirmed adequate and no mass should be spent on it;
- the 99 m/s entry remains "outside limits" until the pitch-up is re-flown.

**Conservative alternative (R5).** If a reviewer refuses the 30 s-resampled replay and wants a bound
rather than a nominal: load every tile cell at the worst station flux (1785 K, C). Then **every**
PICA extension to 34.5 m yields exactly the same tile peak, so the conservative choice is
**x0 = 37.93 m and no added PICA mass**, with Q30 left open and the flux reduction as the only
remedy. The second conservative choice is to ignore the 1673 K claim entirely (the LI-900 1530 K
limit is the design limit), which this study does.

## 9. Open items

1. **Q30 stays open** and is now a trajectory/attitude constraint (α ≲ 22° at pitch-up, or a lower
   pitch-up speed). It cannot be closed by TPS mass; it needs a re-flown entry (Q28).
2. **Q21 (PICA bondline 614 K vs 560 K) is untouched.** The layout change is neither a cure nor a
   new violation. The new barrel strip at 2.5 cm would itself run ≈ 548 K (replay-scaled), so a
   barrel PICA would need 3.5 cm.
3. **Not modelled here:** the coupled effect of a heavier vehicle on the entry (the +0.38…+3.7 t
   variants enter slightly deeper). Any mass added to the TPS must be re-flown before the number is
   claimed; this study deliberately did not run a mission.
4. **The model's seam resolution is 2.4 m** and the "0.53 m aft of the seam" is a station label.
   A finer grid around x = 37–40 m would put a number on the true seam-adjacent peak; it would not
   change the conclusion (the flux is flat to 20 % over the whole barrel).
5. **The laminar/turbulent switch at ≈ 18–20° α** (C) is the largest single uncertainty in the
   attitude lever: if the real boundary layer stays turbulent at low α, the low-α barrel flux would
   be higher than the model shows. That pulse already puts the **aft** barrel at 1420–1432 K
   (t ≈ 2095–2155 s), only ~100 K under the limit, so the margin for a lower-α profile is smaller
   than the peak numbers alone suggest. Treat the α ≲ 22° lever as an upper bound until it is re-run.
6. **The tile material remains the assumed LI-900 class at 3 cm** (S for the density and limit,
   A for the thickness). No other sourced tile material exists in `TILE_MATERIAL_OPTIONS`.

## 10. Reproduction

Analysis only; no mission, no profile, no browser, nothing in `packages/` edited. Every script was
run under `nice -n 19`:

```
nice -n 19 bun .tmp/agentQ30/flux.ts      # flux vs station at the two pulses (§3)
nice -n 19 bun .tmp/agentQ30/replay.ts    # replay audit + parametric x0 sweep (§2, §4.2)
nice -n 19 bun .tmp/agentQ30/variants.ts  # flank vs taper, R5 bound, flux factor, thickness (§4-§6)
nice -n 19 bun .tmp/agentQ30/mass.ts      # dry/wet/x_cg table (§4.1)
nice -n 19 bun .tmp/agentQ30/strip.ts     # per-station surface/bondline/recession (§5)
nice -n 19 bun .tmp/agentQ30/levers.ts    # attitude lever + compliant strip thickness (§6)
nice -n 19 bun .tmp/agentQ30/window.ts    # 1 s tile history, >1530 K window (§3)
nice -n 19 bun .tmp/agentQ30/hotcell.ts   # which tile cell is hottest, by time (§3)
nice -n 19 bun .tmp/agentQ30/split.ts     # surface energy split: re-radiation vs conduction (§6)
```

Inputs read, not modified: `craft/src/config.ts` (`designPica`, `picaThicknessAt`,
`DEFAULT_DESIGN`), `craft/src/tiles.ts` (`TILE`, `TILE_MATERIAL_OPTIONS`, `tiledArea`),
`heat/src/{heating,thermal,limits}.ts`, `.tmp/mission_final.log`.
