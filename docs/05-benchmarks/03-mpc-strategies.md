# bench/03 — Controller bake-off: six lateral/powered-descent strategies on one plant

Measured 2026-11-21 on the shared devcontainer with

```bash
nice -n 19 bun scripts/mpc-bench.ts
bun test packages/gnc/test/mpcBench.test.ts
npx tsc -p tsconfig.json --noEmit
```

**This is a controller/model comparison, not a catch claim.** The plant is a 3-DOF point mass with a
first-order or torque-limited thrust axis; the full 6-DOF mission remains the ground truth. The point
of this bench is to answer two narrow questions with numbers in seconds:

1. Which lateral/powered-descent strategy follows a given reference path best on the same plant?
2. Is the fast plant's thrust/mass/gimbal model consistent with the real `@ship/craft` engine data?

**Q1 answer:** PD + acceleration feedforward. It follows the shared reference to **1.1 m RMS
(nominal)** and **1.2 m RMS (hard)**, with terminal errors **2.2 m / 0.71 m·s⁻¹** and **3.9 m /
1.28 m·s⁻¹**. ZEM/ZEV has the best terminal gate error among the non-perfect strategies, but its
cross-track RMS is 9.6 m / 14.2 m.

**Q2 answer:** consistent. All 17 model checks pass (thrust at 40/60/100 % for 2 and 3 engines,
10 s propellant flow, gimbal moment at ±15°, sea-level/1 km pressure correction). The largest printed
relative error is 0.0000 % at four decimal places; the test asserts ≤1 % for acceleration, ≤0.1 % for
flow and ≤0.5 % for the gimbal.

## 1. What is compared

| id | strategy | cadence | what it is allowed to see |
|---|---|---|---|
| `open-loop` | reference feedforward only | — | the timed reference path |
| `pd-ff` | PD on path error + reference acceleration feedforward | every 10 ms | the timed reference path (the "perfect path" lower bound) |
| `two-phase` | flight `planLanding({ mode: "twoPhase" })` | 10 Hz | measured state, engine limits, held cruise/phase-break, measured vertical trim |
| `pdg` | joint `planPdg` (lossless-convex powered descent) | 5 Hz | measured state, engine limits, warm starts |
| `zem-zev` | ZEM/ZEV lateral + analytic vertical profile | every 10 ms | gate state and vertical profile |
| `bang-bang` | one-switch double-integrator lateral + analytic vertical profile | every 10 ms | gate state and vertical profile |

The two-phase strategy calls `planLanding` directly in `mode: "twoPhase"`, with the flight's
`vF = 0.7 m/s`, `tau = 1.4 s` above 120 m and `0.7 s` below, `glide = 0.04`, `terminalAlt = 3 m`,
`wMag = 0.05`, and the held `aCruise`/`zFinal` schedule. It uses the planner's lateral MPC (not the
outer `guidance.ts` lateral-field branch, whose 4 m/s along-arm cap is tuned for a near-gate approach,
not a 500–1000 m bench start) and a measured vertical trim equivalent to the flight's `uzTrim`
(desired vertical thrust ÷ realised vertical thrust, clamped 0.4–1.2). It is the planner mode, not the
complete flight controller. The PDG is called with its default 9 time-to-go
candidates, 28 steps and 45 iterations; its frame is shifted so its origin is the terminal gate.

## 2. The plant

### 2.1 State and integration

- Pin-station state `(p, v)` in pad ENU relative to the catch point.
- Thrust axis `d` (unit vector) and mass; throttle and gimbal have their own first-order/slew state.
- Gravity **9.79 m/s²** along −z; no aero.
- Semi-implicit Euler at **10 ms**; each strategy is stepped at the same cadence and holds its command
  between replans.
- The four 6-DOF heavy items (`q`, ω, inertia tensor, aero) are deliberately absent; the direction
  state is the minimal second-order stand-in for the attitude loop.

### 2.2 Real engine and mass data

From `makeVehicle()` + `massProps()` + the three `RSL` engine records in `@ship/craft`:

| quantity | value | source |
|---|---:|---|
| landing mass | **138,795.45 kg** | `massProps(vehicle, header-full / main-empty)` |
| CG station | 28.22 m | `massProps` |
| `Iyy` | **41,299,986.93 kg·m²** | `massProps` |
| gimbal arm (CG → gimbal x) | **24.62 m** | `massProps` − engine `gimbal.x` |
| `thrustSL` (100 %, sea level) | **2.450 MN** | engine record |
| `isp0` (sea level) | **350 s** | engine record |
| `exitArea` | **1.33 m²** | engine record |
| `minThrottle` | **0.40** | engine record |
| `gimbalMax` / rate | **±15° / 20°·s⁻¹** | engine record |
| `F_vac,100` | **2,584,762.25 N** | `thrustSL + 101325·exitArea` |
| `Isp_vac` | **369.3 s** | `isp0·F_vac,100/thrustSL` |

The 2-engine sea-level net-acceleration band at this mass is

- 40 % floor: **+3.17 m/s²** (no hover);
- 100 %: **+25.51 m/s²**.

### 2.3 Thrust, flow, floor and pressure

Per engine, with ambient pressure `p∞`:

```
F(throttle, p∞) = max(0, throttle·F_vac,100 − p∞·A_e)
ṁ(throttle)     = throttle·F_vac,100 / (Isp_vac·g0)      (independent of p∞)
```

The command boundary clamps `throttle ∈ [0.4, 1]`. Thrust responds through a first-order lag with
`τ_throttle = 0.3 s`; mass follows the same ṁ. Ambient pressure is the US76 value at the current
pin-station altitude (`@ship/earth.atmosphere`), so the sea-level correction is automatic.

### 2.4 Pointing modes

- **`rate`** (sensitivity): first-order direction tracking with `τ_dir = 0.35 s`, hard-capped at
  **0.45 rad/s** (25.8°/s, close to the flight's 26°/s landing slew limit).
- **`torque`** (the main comparison): a transverse rigid axis. A PD gimbal command
  `δ = clamp(kp·θ − kd·ω, ±15°)` slews at 20°/s; the angular acceleration is
  `α = n·T·arm·sin δ / Iyy`. This ties the pointing response to the real engine torque and inertia
  instead of a hand-set direction rate.

### 2.5 Disturbance / crosswind

No aero means a velocity wind has no dynamic effect. The crosswind case therefore applies the
equivalent constant lateral acceleration

```
a_wind = k·W,   k = 0.1 s⁻¹,   W = 10 m/s  →  1.0 m/s²
```

This is stated explicitly in the JSON metadata and in the crosswind table.

## 3. Scenarios, gate and shared reference

Both start states are post-flip, engines lit at the 40 % floor, thrust axis vertical:

| scenario | start `p` (m) | start `v` (m/s) | vertical profile time |
|---|---:|---:|---:|
| nominal | (500, −100, 800) | (−60, 0, −100) | **15.89 s** |
| hard | (1000, −450, 800) | (−85, 30, −85) | **18.67 s** |

Terminal gate: **p = (0, −12, 0) m**, **v = (0, 0.4, −0.7) m/s** (12 m south of the catch point, on
the rail plane).

The reference path is fixed for every strategy:

- **vertical**: the exact constant-deceleration profile `z(t) = z0 + vz0·t + ½·az·t²` with
  `az = (vzf − vz0)/T` and `T = −2z0/(vz0 + vzf)`; `az = +6.25 m/s²` (nominal) and `+4.52 m/s²`
  (hard). This is the hoverslam family, not a hover.
- **horizontal**: a C² quintic in the same `T` with zero initial/final lateral acceleration, matching
  the start and gate velocities. Peak lateral acceleration is ≈5.8 m/s² (nominal) and ≈8.8 m/s²
  (hard), inside the 48° corridor budget.

The reference is shared by `open-loop`, `pd-ff` and the cross-track metric. The plan-based strategies
(`two-phase`, `pdg`) do not receive the reference path; they plan their own trajectory, which is
exactly why their cross-track RMS against this reference is a fair comparison of *path following*, not
just terminal reaching.

## 4. Metrics

Measured at the rail-plane crossing `z = 0` (linearly interpolated across the last 10 ms step):

- `dp` — **|p − p_gate|** (m);
- `dv` — **|v − v_gate|** (m/s);
- `RMS` — RMS point-to-reference-polyline distance over the whole run, sampled every 50 ms (m);
- `dir err` — maximum angle between the realised thrust axis and the shaped command (deg);
- `fuel` — propellant consumed from the collector (kg); for a `*` run it is taken at the closest
  approach, not after the climb-away;
- `t` — crossing time (s); a **`*`** means the run never crossed the plane, and the closest approach
  is scored instead;
- `ctrl ms` — wall time inside `command(...)`, including solver and replans (ms);
- `wall ms` — plant+controller wall time (ms).

Ranking composite: **`dp + 2·dv + RMS`** (m-equivalent). The factor 2 on velocity is the same
catch-style weighting used by the mission scorer (`2×` contact speed excess), and RMS keeps
path-following visible even when a strategy reaches the gate. Component columns are shown so the
reader can re-rank differently.

## 5. Model checks (real `@ship/craft` data)

All checks are asserted in `packages/gnc/test/mpcBench.test.ts`. `z = 800 m` is the start altitude for
the acceleration checks; the pressure checks use sea level and 1 km.

| check | measured | analytic | rel. error | result |
|---|---:|---:|---:|---|
| 2 engines, 40 %: thrust/m − g | 3.3436 m/s² | 3.3436 m/s² | 0.0000 % | PASS |
| 2 engines, 60 % | 10.7927 m/s² | 10.7927 m/s² | 0.0000 % | PASS |
| 2 engines, 100 % | 25.6910 m/s² | 25.6910 m/s² | 0.0000 % | PASS |
| 3 engines, 40 % | 9.9104 m/s² | 9.9104 m/s² | 0.0000 % | PASS |
| 3 engines, 60 % | 21.0841 m/s² | 21.0841 m/s² | 0.0000 % | PASS |
| 3 engines, 100 % | 43.4315 m/s² | 43.4315 m/s² | 0.0000 % | PASS |
| 2 engines, 60 %, 10 s flow | 8,565.6162 kg | 8,565.6162 kg | 0.0000 % | PASS |
| 2 engines, 100 %, 10 s flow | 14,276.0270 kg | 14,276.0270 kg | 0.0000 % | PASS |
| fuel collector vs mass loss (both throttles) | equal | equal | 0.0000 % | PASS |
| gimbal +15°: `n·T·arm·sin δ / Iyy` | +0.7597 rad/s² | +0.7597 rad/s² | 0.0000 % | PASS |
| gimbal −15° | −0.7597 rad/s² | −0.7597 rad/s² | 0.0000 % | PASS |
| gimbal +15° moment | 31,376,784.5106 N·m | 31,376,784.5106 N·m | 0.0000 % | PASS |
| `F_vac,100` | 2,584,762.2500 N | 2,584,762.2500 N | 0.0000 % | PASS |
| 1 Raptor 100 % at sea level | 2,450,000.0000 N | 2,450,000.0000 N | 0.0000 % | PASS |
| 1 Raptor 100 % at 1 km | 2,465,226.7907 N | 2,465,226.7907 N | 0.0000 % | PASS |
| 1 Raptor 40 % at sea level | 899,142.6500 N | 899,142.6500 N | 0.0000 % | PASS |

The test also asserts the reference path endpoints, the rate-mode `rate·dt` cap, the torque-mode
analytic angular acceleration at full gimbal, and that the 2-engine 40 % floor cannot hover.

## 6. Results — torque mode (the honest comparison)

`*` = never crossed the rail plane; the closest approach is scored. Metrics are deterministic;
timing columns are from one representative run (`ctrl ms` varies with the shared CPU; repeated runs
ranged from a few tenths of a millisecond to ~1.2 ms per two-phase replan and ~7–12 ms per PDG replan).

| strategy | scenario | dp (m) | dv (m/s) | t (s) | RMS (m) | dir err (°) | fuel (kg) | replans | ctrl ms | wall ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| open-loop | nominal | 70.0 | 2.97 | 15.64 | 37.1 | 6.9 | 10,697 | 0 | <0.1 | 9 |
| **PD + FF** | **nominal** | **2.2** | **0.71** | **15.84** | **1.1** | 14.0 | 10,878 | 0 | <0.1 | 11 |
| two-phase | nominal | 28.8 | 26.73 | 18.99* | 71.6 | 95.8 | 12,700 | 400 | 316 | 324 |
| PDG | nominal | 14.8 | 20.28 | 15.96 | 16.0 | 18.3 | 10,322 | 80 | 683 | 687 |
| ZEM/ZEV | nominal | 0.9 | 0.70 | 15.96 | 9.6 | 24.3 | 10,867 | 0 | <0.1 | 3 |
| bang-bang | nominal | 513.8 | 52.05 | 19.95* | 337.6 | 95.8 | 15,915 | 0 | <0.1 | 10 |
| open-loop | hard | 97.3 | 3.20 | 18.63* | 331.6 | 14.9 | 11,939 | 0 | <0.1 | 4 |
| **PD + FF** | **hard** | **3.9** | **1.28** | **18.38** | **1.2** | 22.8 | 11,863 | 0 | <0.1 | 10 |
| two-phase | hard | 14.9 | 25.05 | 21.69 | 23.9 | 89.7 | 14,412 | 217 | 229 | 232 |
| PDG | hard | 9.2 | 8.45 | 20.43* | 317.3 | 92.5 | 12,423 | 200 | 1326 | 1335 |
| ZEM/ZEV | hard | 1.7 | 0.72 | 18.97 | 14.2 | 28.2 | 11,925 | 0 | <0.1 | 4 |
| bang-bang | hard | 770.6 | 50.05 | 20.34* | 509.2 | 95.9 | 15,640 | 0 | <0.1 | 15 |

### 6.1 Ranking (torque mode)

| rank | strategy | score | worst dp | worst dv | worst RMS | fuel (kg) | ctrl ms |
|---:|---|---:|---:|---:|---:|---:|---:|
| 1 | **PD + accel feedforward** | **12.3** | **3.9** | **1.28** | **1.2** | 22,741 | <0.1 |
| 2 | ZEM/ZEV lateral + profile | 29.2 | 1.7 | 0.72 | 14.2 | 22,792 | <0.1 |
| 3 | planLanding two-phase @10 Hz | 242.7 | 28.8 | 26.73 | 71.6 | 27,112 | 545 |
| 4 | joint planPdg @5 Hz | 414.8 | 14.8 | 20.28 | 317.3 | 22,745 | 2009 |
| 5 | open-loop feedforward | 548.3 | 97.3 | 3.20 | 331.6 | 22,636 | <0.1 |
| 6 | one-switch bang-bang | 2335.4 | 770.6 | 52.05 | 509.2 | 31,555 | <0.1 |

The PD loop is the best path follower by a wide margin: **9–12× lower RMS than ZEM/ZEV**, **20–65×
lower than the two-phase planner**, and **more than 250× lower than PDG on the hard start**. It
combines that RMS with a terminal gate error of 2.2–3.9 m; ZEM/ZEV is slightly better on terminal
position/velocity but is not trying to follow the reference, so its RMS is 9–14 m.
The plan-based strategies are solving their own terminal problem; when the reference is the thing being
judged, their free path is a liability.

### 6.2 Why the plan-based strategies lose here

- **two-phase**: the vertical branch is a fixed cruise/brake profile whose final phase uses the
  engine-floor deceleration (`Γ_min − g ≈ 3.17 m/s²`). From 800 m and −100 m/s with two engines, the
  average deceleration needed is 6.25 m/s²; the profile must fly above the floor. With the pointing lag
  and the 40 % floor the nominal run bottoms at 0.4 m above the plane and never crosses, and the hard
  run crosses with 25 m/s velocity error. The measured vertical trim helps (without it the nominal run stalls above the plane), but it cannot
turn the fixed final brake into a hoverslam. This is the
  known no-hover limitation of the two-phase split, now quantified on this start state.
- **PDG**: the joint planner optimises its own terminal cost, not the shared reference. Its 3-DOF
  rollout does not hard-limit the thrust-direction rate, so the torque-limited plant cannot fly the
  planned direction reversals; the result is an on-time crossing at 20.3 m/s velocity error in the
  nominal case and a closest approach at 0.7 m altitude in the hard case. The rate-mode sensitivity (below) is better, which
  confirms the direction-lag diagnosis.
- **bang-bang**: a per-axis switching curve has no path or terminal-velocity matching. It saturates
  the 48° corridor and does not reach the plane on either start.
- **open-loop**: the reference feedforward is exact for the ideal plant but has no error feedback; the
  throttle/pointing transients and mass loss leave 70 m (nominal) and 97 m (hard, closest approach)
  of error.
- **ZEM/ZEV**: excellent terminal gate error, but the lateral law is a terminal controller, not a path
  tracker; its cross-track RMS is 9.6 m / 14.2 m and the 10 m/s crosswind makes it swing wide.

## 7. Rate-mode sensitivity

The rate mode uses a first-order direction response with a 0.45 rad/s cap instead of the torque axis.
The ranking changes:

| rank | strategy | score | worst dp | worst dv | worst RMS | fuel (kg) |
|---:|---|---:|---:|---:|---:|---:|
| 1 | PD + accel feedforward | 2.9 | 0.8 | 0.29 | 0.3 | 22,886 |
| 2 | ZEM/ZEV | 39.7 | 1.3 | 1.33 | 18.3 | 22,620 |
| 3 | open-loop | 98.1 | 29.2 | 11.26 | 10.6 | 21,288 |
| 4 | PDG | 231.6 | 62.7 | 26.12 | 33.3 | 21,116 |
| 5 | two-phase | 411.0 | 12.2 | 7.88 | 209.9 | 24,472 |
| 6 | bang-bang | 2141.7 | 743.2 | 55.96 | 477.3 | 31,479 |

PD is still first and ZEM/ZEV still second. PDG improves from 414.8 to 231.6 when the direction
response is not tied to the real inertia — expected, and the reason the torque mode is the primary
table. The two-phase and bang-bang remain poor because their limits are not primarily pointing-rate
limited.

## 8. 10 m/s crosswind (best two)

Applied as the equivalent 1.0 m/s² lateral acceleration described in §2.5.

| strategy | scenario | dp (m) | dv (m/s) | t (s) | RMS (m) | dir err (°) | fuel (kg) |
|---|---|---:|---:|---:|---:|---:|---:|
| PD + FF | nominal | 2.8 | 0.81 | 15.92 | 1.2 | 20.7 | 10,805 |
| PD + FF | hard | 8.3 | 8.69 | 18.08 | 2.5 | 78.4 | 11,718 |
| ZEM/ZEV | nominal | 0.9 | 2.27 | 16.35* | 153.6 | 96.0 | 10,960 |
| ZEM/ZEV | hard | 2.5 | 2.81 | 19.33* | 127.8 | 96.0 | 11,954 |

The PD loop still tracks the path spatially (1.2–2.5 m RMS) and crosses the plane within 3–8 m, but
the hard case crosses with 8.7 m/s velocity error because the last-second direction reversal is
behind the torque-limited axis. The ZEM/ZEV runs under wind **do not cross the plane**; the star is
the closest approach (0.13 m nominal, 0.54 m hard), where the vertical velocity has already been
nulled. Their path swings 128–154 m RMS and saturates the corridor (96° direction error). If the requirement is "stay on the reference path in
wind", PD is the clear choice; if the requirement is "arrive at the gate position regardless of path",
ZEM/ZEV is competitive.

## 9. Timing — fastest and slowest

On the shared devcontainer, `nice -n 19`, the whole script (12 torque runs + 12 rate runs + 4 crosswind
runs + 17 model checks + JSON/CSV) is **4–8 s wall** in repeated runs (4.25 s on a quiet load, 7.6 s
with the shared CPU busy). Metrics are deterministic; only the timing columns vary with machine load.

| strategy | mean per run (torque) | replans | per replan |
|---|---:|---:|---:|
| open-loop / PD / ZEM-ZEV / bang-bang | **<0.01 ms** | 0 | — |
| planLanding two-phase @10 Hz | 201–357 ms | 617 | **0.65–1.16 ms** |
| joint planPdg @5 Hz | **992–1706 ms** | 280 | **7.1–12.2 ms** |

The four analytic laws are effectively free. **The slowest solver is the joint `planPdg` at 7–12 ms
per replan / 1.0–1.7 s per run**; `planLanding` two-phase is 0.65–1.2 ms per replan / 0.2–0.36 s
per run. The PDG's 5 Hz cadence and 9 candidate times make it the dominant CPU cost.

## 10. Model consistency conclusion

The fast plant's thrust, mass-flow, gimbal-moment and pressure models agree with the real
`@ship/craft` engine record to the printed precision:

- thrust `n·(throttle·F_vac,100 − p∞·A_e)` reproduces the sea-level 2.45 MN and the 1 km
  2.4652 MN per engine;
- the 40 % floor gives 3.17 m/s² net at the landing mass (no hover), as the flight code assumes;
- mass flow uses `Isp_vac = 369.3 s` and reproduces 14.276 t in 10 s for 2 engines at 100 %;
- the gimbal moment at ±15° is 31.38 MN·m and the resultant `α = M/Iyy = 0.7597 rad/s²`.

This is a consistency check of the *fast* plant, not a validation of the full 6-DOF vehicle. The
torque mode is a reduced transverse-axis model; it deliberately omits the attitude quaternion, aero,
slosh and the roll axis. It is the more honest of the two pointing modes for ranking because it uses
the real inertia and engine torque, but the full mission remains the only ground truth.

## 11. Reproduction and files

```bash
nice -n 19 bun scripts/mpc-bench.ts
bun test packages/gnc/test/mpcBench.test.ts
npx tsc -p tsconfig.json --noEmit
```

The script writes:

- `.tmp/mpc-bench.json` — full run records (torque, rate, crosswind), checks, ranking, engine/mass metadata;
- `.tmp/mpc-bench.csv` — one row per run, including terminal `p`/`v` and all metrics.

`.tmp/` is not committed; this document holds the final table.

New files:

| file | role |
|---|---|
| `packages/gnc/src/bench/types.ts` | state, command, controller and plant types |
| `packages/gnc/src/bench/plant.ts` | real-engine 3-DOF plant, mass/CG/inertia, rate and torque modes |
| `packages/gnc/src/bench/reference.ts` | shared constant-decel + quintic reference and cross-track distance |
| `packages/gnc/src/bench/checks.ts` | the 17 model checks used by the script and test |
| `packages/gnc/src/bench/strategies.ts` | the six controllers and the shared command shaper |
| `packages/gnc/src/bench/harness.ts` | scenarios, gate, integration loop, metrics, aggregation, crosswind |
| `packages/gnc/src/bench/index.ts` | barrel export |
| `scripts/mpc-bench.ts` | CLI: tables, JSON, CSV |
| `packages/gnc/test/mpcBench.test.ts` | tight model-check and reference assertions |
