# review/02 — Review of the optimiser (O) and landing (L) results

Date: 2026-11-21. Method: code read + 6-DOF landing traces at 0.25–0.5 s (`.tmp/review/trace.ts`,
logs `.tmp/review/trace_{default,best,alloc,alloc2,slew2}.log`). Verdict: **not the best solution**. The
search tuned around structural defects in the landing law; no tuning of the current law can reliably
deliver ±1 m / ≤1 m/s.

## 1. Optimiser (O)

| # | Finding | Evidence |
|---|---|---|
| O1 | `fast.ts` ranks a different landing controller (PD on a linear-approach profile) than the flight (two-phase lateral MPC). A single fudge factor `LATERAL_EFFICIENCY = 0.45`, fitted to one baseline point, stands in for the plant. | CMA top-6 all worse in 6-DOF; `fast-best` predicted margin 0.7 m, flew 30.2 m |
| O2 | The fast belly flop tracks its plan perfectly. | fast miss 94 m vs 1456 m flown for the same tuning |
| O3 | Constant mass through a ~16 t burn (12 % of the mass, so the throttle-floor acceleration rises ~12 %). | fixed by O during the pause |
| O4 | `terminalSwitchAlt` is a dead knob: the terminal phase also needs `dHoriz < 8 m && vHoriz < 6 m/s`, which no run meets. | `fast-best` ≡ `terminal-late` to the digit |
| O5 | CMA's active knobs omitted the landing MPC weights/roll loop, i.e. the knobs that matter. | `opt_full.log` vector |
| O6 | `scripts/optimize.ts` duplicates the run loop; the app never receives a `MissionTuning`. | `runWorld` vs `runMission` |

`score.ts` is sound: pin-plane crossing, pin mid-point, roll vs east–west, arms/pin-load gate.

## 2. Landing (L) — root causes in the traces

| # | Finding | Evidence |
|---|---|---|
| L1 | **No thrust-slew model in the two-phase lateral MPC.** The plan is a free 2-D vector with a first-order lag, so it plans ~170° thrust-azimuth reversals (accelerate-then-brake) in one replan. A reversal is a ~100° body rotation through vertical: ~5 s at the measured rates. | best run 3068.3 s: cmd az −61°→180°, attitude error 87–106° for 5.5 s, overshoot past the pad |
| L2 | **Command oscillation exceeds attitude bandwidth.** The cmd tilt goes 44°→5°→47° in 2 s; the body's momentum carries it through vertical, and stopping/reversing costs 4 s. | `trace_alloc2.log` 3071.5–3078 s |
| L3 | **Non-prioritised gimbal allocation.** Min-norm (roll, pitch, yaw) followed by per-engine clipping: when the roll demand saturates a gimbal, the pitch/yaw torque is bent and the thrust axis runs off. **Fixed** (see §3). | default run diverged exactly at the `pz < 250 m` roll gate (59 m, v_h 7 m/s at 346 m → 150 m off) |
| L4 | **Two-phase split vs engine floor (architectural).** The lateral budget is `tan(tilt)·u_z`; as `u_z` falls (8.7→0.5 m/s²) the plan thinks it has 1.2 m/s² of lateral authority, while `assemble` puts the floor surplus (~14 m/s²) sideways in the plan's direction. Plan and plant disagree by an order of magnitude. | `LD` dump 3066–3070 s |
| L5 | **The flip is a large uncontrolled lateral impulse.** Realised lateral reaches −30 m/s² (three engines, nose toward the pad) and adds ~50 m/s of closing speed, while the gate model assumes `flipDv` 35 m/s vertical. `flipAzimuth` always points the kick at the pad, even when the ship is already closing faster than it can stop. | best run v_h 33→89 m/s; default 458 m to go at 91 m/s |
| L6 | **Horizon collapse.** `tDiv = (s − s_F)/a` with `s = max(−v_z, 0)`: once the descent stalls (v_z > 0), the divert window drops to 0.6 s and the plan is degenerate (no recovery). | `trace_slew2.log` |
| L7 | A larger hand-over miss gave a *better* catch (1456 m → 6.8 m; 653 m → 154 m): the residual is set by L1–L5, not by the delivery. | O round 1 |

## 3. Changes made in this review

- `gnc/src/control.ts` `allocateGimbals`: **priority allocation**. The exact split of the min-norm
  solution into pitch/yaw and roll (differential) parts; pitch/yaw are scaled uniformly into the gimbal box
  (torque direction preserved); roll gets the remaining margin. Feasible commands are unchanged.
  Tests green.
- `gnc/src/mpc.ts` `MpcProblem.slew` + `solveSlew`: a plan over increments δ (|δ| ≤ rate·h ball, hard
  magnitude clamp, Gershgorin diagonal majoriser, FISTA with adaptive restart). It converges
  monotonically. Wired as tuning `landingSlewRate`, **default 0 (off)**: bounding the whole vector also
  slew-limits throttle (magnitude) changes, which are fast, so it could not unwind the −30 m/s² flip
  transient (ship flew away in 6-DOF). It is kept as a building block, not a fix.

## 4. Recommended work package (instead of more tuning)

1. **Joint 3-D lossless-convex PDG** (Açıkmeşe/Blackmore) replacing the two-phase split: |u| ∈ [ρ_floor,
   ρ_max] via the Γ slack, tilt cone, free final time by line search, **direction-rate** constraint
   (|d_{k+1} − d_k| ≤ ω·h on the unit thrust axis; magnitude free within the throttle band). Fixes L1, L2,
   L4, L6.
2. **Flip targeting:** choose the flip azimuth/kick so the post-flip state admits a brake-only divert
   (d ≈ v_h·T/2), and model the flip's lateral impulse in the gate. Fixes L5.
3. Roll alignment as a rate-limited target from the start of the burn (now safe with priority
   allocation), not a step at 250 m.
4. Then the optimiser: fast model = the same PDG (no fudge factor), knobs = PDG weights, ω, flip
   targeting; full 6-DOF certification; then T-mc.

## 5. Follow-up after the review (2026-11-21, later)

### Fixed and tested
- **Tower collision.** `towerContact` had no solid structure: only the pins and the arms' inner
  faces. A misaligned ship could pass through the tower column, carriage, arm tops/tips and mount.
  Added sphere-box collision for all of them (moving with the arm gap) and enabled contact in every
  phase near the pad (`hBase < 260 m`, within 300 m). Tests: column push-out, arm-top support,
  closing-arm squeeze, pin equilibrium/friction unchanged.
- **Arm timing.** The arms no longer close on a fixed altitude alone. They start when the pins are
  `armCloseLead = 3.7 s` from the rails (the close takes 3 s), using the *predicted* position at
  closure (`x + vx·lead`, `y + vy·lead`). They finish just before contact; the hull is squeezed by
  the arm boxes and the pins land on the rails.
- **Vertical lag trim was never applied.** Guidance computed `uzTrim` and passed it to
  `planLanding`, but the two-phase branch ignored it. It is now applied to the cruise `u_z`
  (`review/02 L2`). This is a real bug, though the lateral divergence still dominates.
- **Right-mouse pan** in the 3D view: left-drag orbits, right-drag pans the target in the screen
  plane, persists across frames (it used to be re-centred on the ship every frame); context menu
  suppressed. Fit/Home/`setMode("chase")` reset it.
- **Small transparent altitude graph** (top right, left of the ViewCube): altitude vs mission
  time, phase/altitude scale, live marker and current altitude during playback.
- **Smoothness sweep** (`landingRd` 0.1/0.6/1.2 × `landingQvXY` 0/0.8, 6 full 6-DOF runs):
  best 153.3 m lateral, no catch. The two-phase MPC's near-vertical-then-tilt command is not a
  weighting artefact.

### Joint PDG experiment (`gnc/src/pdg.ts`, not wired into the flight)
- Joint 3-D convex problem (cone + engine floor + terminal catch), time-to-go search, hard floor
  projection. In 3-DOF from the 1.08 km flip state it reaches **5.5 m across / 4.1 m along at the
  plane** (was 30 m), but still crosses at 6.2 m/s down. From the nominal 527 m state it is 14 m.
- The hard floor makes the problem non-convex; the cost gradient does not model the direction-rate
  limit, and the time-to-go search is sensitive. It is kept as an experiment, **not** a claimed
  controller. It is off by default (`landingSlewRate = 0` and `planPdg` unused).

### Heat distribution (answer to the plasma question)
The flown angle of attack is **10.1° at 50 km** and **25.6–32.8° through the high-α glide**
(55→44 km). The ship is therefore *not* falling nose-first: the nose is the stagnation region, but
the windward flank is inside the shock layer and is heated too. The coil/MHD force steers the shock
layer (≈410–458 kN at 48–50 km in the log) but it is not a heat shield and does not remove the trim
angle. A nose-only hot spot would require α ≈ 0, which removes the lift used for range and changes
the whole entry. The 1650 K tile surface is over the conservative NASA LI-900 1530 K margin but
**under the user-selected SpaceX 1673 K flight-tile claim by 23 K** (the 1684 K run is over by 11 K);
that is the known Q30 design issue, not a wrong fall direction. The visual hot windward flank is physically correct.

### Still open
The catch is still not achieved. The remaining blocker is the landing lateral controller: the
two-phase plan commands a near-vertical tilt and then a 40–50° tilt again (the 3071–3073 s
reversal), and the body cannot follow. A full joint 3-D PDG with a direction-rate constraint in the
optimisation (not just a rollout) is the next step; the current `pdg.ts` is the scaffold.

### End-path / arm-centreline fix (this pass)
- **Anisotropic catch cost.** The lateral MPC weighted across-arm x and along-arm y equally, but the
  catch is ±1 m across and the rails are 38 m long. `MpcWeights.wX/wY` now weight x ≫ y (4 / 0.35),
  so the terminal plan centres across the arms first instead of trading x for y.
- **Terminal path export.** In the terminal phase the overlay now uses the analytic final line, with x
  collapsing faster than y, ending at the pin centreline. Previously it showed the last landing plan,
  whose final segment could approach diagonally.
- **Arm timing** is time-to-rails (`armCloseLead` 3.7 s), with the predicted closure position.
- **Tower collision** now covers the column, carriage, arm tops/tips and mount in every phase near
  the pad.
- **Vertical trim** (`uzTrim`) is applied in the two-phase branch (it was computed and dropped).
- **Right-mouse pan** and the transparent altitude sparkline are in the app.

Still open: the two-phase lateral plan still commands a tilt reversal near the pad (11° → 48° at
~220 m) that the body cannot follow; the run remains lost. The next test is a lower assumed pointing
lag (τ = 0.35/0.25 s instead of 1.4/0.7 s), which the trace suggests is 4× too large.

### Active no-flap belly-first sequence (2026-11-21)
- Flaps are **not used** in the flown path; the high-α belly-first regime at ~52.5 km / q 12 kPa is the active mode, coil until Q25 authority loss, belly drag to the 1.35 km flip gate, then staged engine landing.
- Latest run: **5150 m** off (across 1369, along 4965), **68.3 m/s**, roll 60°, arms open; `lost`. Tile 1671 K (under the 1673 K SpaceX claim by 2 K), bondline **678 K > 560 K**, steel 285–363 K (safe); 112 kg roll-only N₂.
- Missing: low-q one-engine flip run, post-flip 20–35° drag-attitude law, PD+FF tracker and exact allocation/a0 feedback. The current law is the two-phase vertical + lateral field; no catch claim.

### Closed-loop lateral field result (MPC worker, 2026-11-21)
- `landing.ts` `LateralField`: per-axis `v_ref = −(p+vτ_lead)/τ_eff`, x-before-y budget, stopping-
  distance caps, time-to-go schedule, floor surplus in the field direction; wired after the flip.
- Default result: **37.5 m** off at the rail plane (36.0 m across, 10.6 m along), **29.17 m/s**
  contact, body y **73.3°** off E–W, arrival southward (`vN = −2.51 m/s`), arms open, `lost`.
- It improves the total lateral from 118 m to 37.5 m. The residual is **attitude-azimuth bandwidth**:
  the body's azimuth response is ~1.5–2 s and the measured attitude error reaches 50–180°; roll is
  starved while pitch/yaw saturate. Tuning the translational law further cannot fix a command
  direction the body cannot reach. The next structural change is an attitude-state-aware planner
  (q, ω, gimbal/torque limits) or the joint 3-D rate-limited PDG.

### End-path attempt results (final state of this pass)
- Kept: x-weighted lateral cost (wX 4 / wY 0.35), terminal-phase centreline path export,
  `uzTrim` applied, arm close by time-to-rails, solid tower/arm collision, right-mouse pan,
  transparent altitude graph.
- Tried and removed: a hard across-arm ZEM/ZEV corridor below 350 m. It did centre x (−5 m at
  125 m), but the x-command sign changes arrived faster than the body could rotate; the run ended
  in a ground impact, worse than the baseline. The corridor is not in the tree.
- Tried and rejected by measurement: τ = 0.35/0.25 s (243 m), terminal switch at 80 m (196 m),
  `landingRd`/`landingQvXY` sweep (best 153 m).
- Baseline after the kept changes: pins reach the plane 118 m off (x) at ~38 m/s, `lost`.
  The remaining cause is the plan's tilt reversal at ~220–300 m: the body is at 1–11° when the
  plan asks for 48°, or vice versa. This cannot be fixed by weights; it needs the joint 3-D PDG
  with the thrust-direction rate in the optimisation (the `pdg.ts` scaffold) or a closer
  belly-flop delivery.

### Why the two overlay lines differ (2026-11-21, final)
- Cyan/blue = the recording's **actual future** (the 6-DOF trajectory). It is what the ship really does.
- Pale/white = the **MPC plan** (`rec.pred`). It is the intended path, not a control input.
- If the MPC's internal model matched the plant, the two lines would overlap over the prediction
  horizon. They do not: the lateral plan is a point-mass model with a first-order pointing lag, while
  the real vehicle has torque-limited attitude dynamics. At 3072.8 s the plan asks for a 48° tilt
  while the body is at 9–15°, a 60–85° attitude error. The white line is therefore optimistic.
- The terminal white line was briefly a synthetic target line (looked "right" while the ship
  followed blue). It is now a real rollout of the terminal law, so it shows the same miss.
- Attempted: adding the solver's direction-rate `dyn` model to the two-phase lateral plan. The
  commands oscillated (west/east sign flip) because the cost gradient does not include the
  direction-rate nonlinearity; the run was worse, so it was removed.
- The correct fix is an MPC that includes the attitude state (q, w) and the torque/gimbal limits in
  the prediction, or a command shaper built from the measured closed-loop attitude response. A
  point-mass lateral plan + lag cannot make the white line equal the blue.

### Approach-direction gate and the 48 km loft (this pass)
- **Direction, not just a point.** The lateral divert now ends at a gate **south of the catch**
  (`pRefH = [0, −12]`): the plan must reach x = 0, y = −12 m at the end of the divert, and the
  terminal law then flies north (+y) down the arm centreline. This forbids an approach from the
  tower side/behind. The terminal switch was widened (45 m / 20 m / 20 m/s) so the last 45 m is the
  analytic law; `armCloseAlong` is 16 m (inside the 38 m rails).
- **48 km climb is the high-α loft, not a guidance error.** After the pitch-up at ~48 km the ship
  flies at 28–33° α; the lift vector has an upward component, so altitude rises ~7 km while speed
  falls (6.5 → 5.2 km/s). This is the deliberate energy-management pull-up. The hottest region is
  still the windward stagnation region at the nose, but the finite α puts the windward flank inside
  the shock layer too. The coil/MHD force (≈0.3–0.46 MN around 48–51 km) steers/trims the shock; it
  cannot make the ship fall nose-first at α = 0 without removing the lift that produces the loft and
  the range. “Falling toward the hottest plasma” is not a separate objective; the plasma is the
  consequence of the trajectory.
- **Gate150 test:** move the divert/terminal handover from 30 m to 150 m (`landingZProfMin/Max =
  150`, terminal switch 120 m) so the lateral channel has time to null before the arms. Result in
  `.tmp/review/gate150.log` (pending at the time of writing).

### Direction gates (final)
- The divert ends at a **south gate** `pRefH = [0, −12]`; the terminal law flies north (+y) to the
  pins. The terminal switch additionally requires `y < 5` and `v_y > −0.5`.
- Arms close only with `v_y > −0.3`; a catch requires `v_y > 0` (northward along the arms) and
  `|v_x| < 0.8` (no sideways drift). A north-to-south or sideways arrival is now rejected.
- The 150 m hand-over (`landingZProfMin/Max = 150`) moves the lateral/terminal transition earlier;
  the candidate `gate1500-final` combines it with O's 1.88 km flip gate (best across-arm geometry)
  and the direction gates. Result in `.tmp/review/gate1500-final.log`.
