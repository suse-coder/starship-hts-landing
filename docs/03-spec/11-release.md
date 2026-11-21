# spec/11 — Release gate and private-WIP policy

Status: **BLOCKED** (2026-11-21). This document defines what "released" means for this project and
what must be true before the repository and the GitHub Pages site become public. It is normative for
M8; it does not change the physical-fidelity rules in spec/00 G0.

## 1. Public WIP exception (user approval 2026-11-21)

The user explicitly approved publishing the current repository and Pages site as a **public WIP
snapshot**. That does **not** satisfy the success gate below: the site must carry the WIP/failure
banner, the catch is `lost`, and no "caught"/flight-ready claim may be made. A successful-mission
release (a site presented as the mission result) still requires every gate in §2.

The earlier private-WIP rule applies to a claimed successful release, not to this labelled WIP
snapshot:
- The repository **must not claim a successful release** and the Pages site **must show the WIP
  status** until every gate below is met and the user explicitly approves the success release.
- No public URL, no public screenshots presented as a successful mission, no "flight-ready" or
  "operational" claim. The current simulation is a work in progress.
- The documentation must keep the banner in `README.md` and the current-status section in
  `docs/02-plan/00-roadmap.md` consistent with the actual outcome. A stale "caught" claim is a
  release blocker, not a documentation detail.
- The false catch of 2026-11-21 is retracted: a `CAUGHT` outcome is only valid when the chopsticks
  are commanded closed (`armsClose`) and the pins carry load on the rails. An outcome with the arms
  open is `lost`, regardless of a single pin grazing a rail.
- An honest `lost` or "outside limits" result may be published only as an explicitly labelled
  failure; it is not a successful release. G0 still outranks delivery: do not tune or clip the
  physics to manufacture a catch.

## 2. Release gate

A public release requires **all** of the following. M3 and M8 are not ticked until they are true.

| # | Gate | Acceptance | Current state (2026-11-21) |
|---|---|---|---|
| R-1 | **True catch (M3)** | Outcome `caught`; `armsClose` true; both pins on the rails; ship axis within **±1 m** of the tower centreline; contact speed **≤ 1 m/s**; arrival from the south, moving north along the arms (`v_y > 0`, `abs(v_x) < 0.8 m/s`); **no translational cold gas after the flip; short roll-only pulses allowed, ≤25 kg N₂**; 1–3 sea-level Raptors with a 1-engine gimbal phase; engines off instantly on catch; coil discharged | **OPEN — `lost`.** No true catch in the tree. |
| R-2 | **TPS within limits** | Tile surface **≤ 1673 K** (SpaceX flight-tile claim, user decision 2026-11-21; NASA LI-900 1530 K reported as a margin), steel backing **≤ 600 K**, PICA-X bondline **≤ 560 K**, steel **≤ 1150 K** on the released trajectory; no clipping | **OPEN — bondline over.** Best high-α tile 1650 K passes the flight claim by **+23 K** (NASA 1530 margin: −120 K); the 14:08 run 1684 K fails by −11 K. PICA bondline 695–701 K vs 560 K fails. Backing 363 K passes. |
| R-3 | **T‑mc robustness** | Monte Carlo over wind, Hall closure κ, magnetic field B and mass ±5 %: **≥ 18/20 caught** within the catch envelope, with a positive wind margin | **NOT RUN.** A single nominal run, even if caught, does not satisfy this gate. |
| R-4 | **No false or disabled physics** | review/00 audit current; no state writes outside the integrators; no phase-disabled physics; no hidden aero/MHD oracle; R11a cold-gas use reported | **PARTIAL.** Review/01 found R6/R11a/R11c issues; R6 and R11c were fixed, R11a remains open in T14. |
| R-5 | **Verification green** | `bun run typecheck` clean; `bun run test` green; the two `test.todo` acceptance items resolved or explicitly accepted; T-mission and T-catch-envelope passing | **PARTIAL.** Typecheck clean; 238 pass, 2 todo, 0 fail. T-mission/T-catch are open. |
| R-6 | **Build and deployment** | `bun run build` succeeds; Pages workflow green on a test branch; the public URL renders the current recording with the WIP/failure banner | **WIP DEPLOYED.** <https://suse-coder.github.io/starship-hts-landing/>; run 44940950934 green; loaded end-to-end, outcome `LOST`, TPS badge outside limits. This is the public WIP snapshot, not a success release. |
| R-7 | **User approval** | The user explicitly approves the public repository, the Pages deployment and the wording of the release | **WIP APPROVED.** The user approved the public WIP repo/Pages on 2026-11-21; the success-release approval is still pending. |

## 3. Exact blockers

These are the three blockers that must be closed first; the remaining gates follow.

### 3.1 Catch (M3)

The catch is **not achieved**. The latest evidence:

- Integrated mission: `lost` at the rail plane, **11 m off**, climbing 3.0 m/s
  (`.tmp/mission_verify.log`, 2026-11-21).
- Best full-6-DOF optimiser candidate: **6.8 m lateral / 11.7 m/s**, `lost`
  ([bench/02](../05-benchmarks/02-optimizer.md), `fast-best`).
- Later direction-gate / 150 m hand-over sweeps: all `lost`; best lateral 153.3 m
  (`.tmp/review/smooth.log`); the best across-arms geometry was 7.7 m across but 230.6 m along
  (`.tmp/review/gate1500v2.log`).
- The active **no-flap belly-first** sequence (2026-11-21): **5150 m** off (across 1369, along 4965), **68.3 m/s**, roll 60°, arms open; `lost`. Tile 1671 K (under the 1673 K claim), bondline 678 K (over 560), steel 285–363 K.
- The new closed-loop lateral field (MPC worker, 2026-11-21): **37.5 m** off at the plane
  (36.0 m across, 10.6 m along), 29.2 m/s contact, body **73.3°** off E–W, arrival southward
  (`vN = −2.51 m/s`), arms open; `lost`. It improves the total lateral from 118 m to 37.5 m but the
  residual is **attitude-azimuth bandwidth**, not the translational law: the body cannot turn the
  command direction fast enough, and the roll channel is starved while pitch/yaw saturate.
- The root cause is the landing lateral controller and attitude/tilt tracking, not authority,
  propellant or wind. The two-phase plan commands a tilt reversal near the pad that the body cannot
  follow; a joint 3-D rate-limited PDG including the attitude state is the next step
  ([review/02](../04-review/02-subagent-review.md)).

Acceptance is the full R-1 definition. A pin grazing an open rail is **not** a catch.

Code-level note: the current catch condition checks `armsClose`, the pin-axis lateral offset, the
northward velocity and the **total** pin load (≥ 45 % of weight). A per-pin check — both pins
individually loaded on a rail — is still a T09 item. Do not treat a single-pin load as a catch.

### 3.2 TPS

The released trajectory must be inside the thermal limits, not merely reported:

| Limit | Value | Latest measured | Margin |
|---|---:|---:|---:|
| Tile surface, NASA LI-900 margin | 1530 K | 1650 K / 1684 K | **−120 K / −154 K** |
| Tile surface, SpaceX flight claim (gate) | 1673 K | 1650 K / 1684 K | **+23 K / −11 K** |
| PICA-X bondline | 560 K | 695 K (latest sweeps) / 701 K (14:08 integrated run) | **−135 K / −141 K** |
| Tile steel backing | 600 K | 363 K / 345 K | passes |
| Steel design limit | 1150 K | 363 K / 345 K | passes |

The tile peak is a pitch-up plateau problem, not a peak-q problem. Under the user-selected SpaceX
flight-tile claim (1673 K), the best high-α run at 1650 K passes by 23 K; the 14:08 run at 1684 K
fails by 11 K, and the unretargeted zero-α run at 2417 K fails. Against the NASA LI-900 1530 K
margin the 1650 K run is over by 120 K. Q30 found no TPS-only mass fix: extend-PICA buys 0 K and
ε = 1 still gives 1664 K; the absorbed flux must fall 1.22× for the claim (1.75× for LI-900), i.e.
hold the high-α plateau at α ≲ 22° or pitch up at ≲ 4.5 km/s. That is an entry-guidance constraint
coupled to Q28. The PICA bondline (695 K vs 560 K) remains the hard thermal blocker.

### 3.3 T‑mc

**Not run.** There is no Monte Carlo ensemble in the tree. The gate is **≥ 18/20 caught** over wind,
κ, B and mass ±5 %, with a positive wind margin. A nominal catch alone does not close T09 or the
release gate. The ensemble must use the full 6-DOF mission and the same catch condition as R-1.

## 4. Release checklist

- [ ] M3: true catch with `armsClose`, pin load, ±1 m, ≤ 1 m/s, northward arrival, with wind.
- [ ] TPS: tile ≤ 1673 K (SpaceX flight claim; NASA 1530 K margin reported), bondline ≤ 560 K, backing ≤ 600 K, steel ≤ 1150 K on the released run.
- [ ] T‑mc: ≥ 18/20 caught in the wind/κ/B/mass ensemble.
- [ ] R11a: cold-gas use during the coil phase reported and within the stated fallback policy.
- [ ] R11b: coil switch-off/discharge at the real time; coil current at touchdown reported.
- [ ] `bun run typecheck` clean and `bun run test` green; unresolved `test.todo` items accepted or closed.
- [ ] `bun run build` succeeds; Pages workflow green on a test branch.
- [ ] README, roadmap, spec/04, spec/05 and this document updated to the released outcome.
- [ ] User explicitly approves the public repo, Pages URL and release wording.
- [ ] Public URL verified with a screenshot and the WIP/failure status visible.

## 5. Evidence

- [Roadmap current status](../02-plan/00-roadmap.md#current-status--2026-11-21)
- [Review 02 — landing root cause](../04-review/02-subagent-review.md)
- [Bench 02 — first optimiser search](../05-benchmarks/02-optimizer.md)
- [Verification matrix](05-verification.md)
- `.tmp/mission_verify.log`, `.tmp/review/gate150*.log`, `.tmp/review/smooth.log`,
  `.tmp/review/taufast.log`, `.tmp/review/terminal80.log` (local evidence; not published assets)
