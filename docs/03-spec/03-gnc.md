# spec/03 — Guidance, navigation and control

Guidance and control are flight software (spec/00 R6). They act **only** through actuator commands
within physical limits. Navigation is perfect (idealisation I1). Code: `packages/gnc/src`
(`guidance.ts` phase machine, `predictor.ts` 3-DOF NPC, `mpc.ts` landing MPC, `control.ts` attitude
and allocation, `mhdAlloc.ts`). Trade studies: plan/01 TS-01…TS-05, TS-12. Prior art: map/30.

Status: `[x]` implemented and verified in the 6-DOF run · `[~]` implemented, not meeting its
acceptance yet · `[ ]` open.

## 1. Phase machine

| Phase | Guidance law | Attitude target | Actuators | Exit | Status |
|---|---|---|---|---|---|
| deorbitAlign | burn timing from the Flight 15 plan | nose retrograde | cold gas | t ≥ t_burn − Δv/2a | [x] |
| deorbitBurn | Δv integration | same | RSL1 60 % | Δv = 105 m/s (2026-11-21, after the passive-trim and 2-axis feed-forward fixes; the earlier 99 m/s sweep was against the wrong 80° trim) | [x] |
| entryAlign | coil ramp from t_EI − 780 s | nose along v_air | cold gas | h < 120 km | [x] |
| entry | NPC bank magnitude (Lu 2014), deadband reversals > 200 km | α_nf(V) 10→14°, bank σ | **coil electrodes** (pitch/yaw); roll → MHD per R11a (T14), cold gas only as fallback | V < 6 km/s | [x] / R11a [ ] |
| loft | lift up | α_nf, σ = 0 | coil | q at its minimum | [x] |
| pitchUp | – | α_trim(M) | coil + aero | trim reached | [x] |
| highAlpha | **planned-reversal NPC**: \|σ\| for downrange, t_rev for crossrange, 5 s lead, bias to the correctable side | lift by σ (lift transport, 1.5°/s) | cold gas (α-axis free), barbecue 4°/s > 1.3 km/s | M < 2 | [x] |
| bellyFlop (shallow) | same law, belly trim, 1.8°/s | σ | cold gas | γ < −60° | [x] |
| bellyFlop (steep) | ground-azimuth ψ law, (ψ₁, τ, ψ₂) search; last 6 km: nose to the N/E/S/W nearest the pad bearing + catch roll | ψ, roll for the pins | cold gas | h < flip altitude | [x] (miss 0.67–1.8 km) |
| flip | sink-rate kill, near-vertical thrust (≤ 10° tilt) | thrust direction, roll held | **3 × Raptor SL** for the rotation, then shutdown to 1 | pointing < 12° & rates < 3°/s, or v_z > −10 m/s | [x] / sequence revised 2026-11-21 |
| landing (1-engine) | **MPC path** (TS-02, TS-15), real-time replan; guided fall on 1 Raptor | thrust direction, pins → E/W | **1 × Raptor SL**, gimbal pitch/yaw + **short roll-only cold-gas pulses** (user 2026-11-21); no translational RCS | add engines before the vertical/lateral budget is exhausted | [~] no catch |
| landing (staged) | same MPC; stage 1 → 2 (3 only if the decel needs it) | thrust direction, pins → E/W | 2–3 Raptors, gimbal + roll-only RCS; no translational cold gas | < 3 m, < 1.5 m/s, 3 m above the aim | [~] no catch |
| terminal | timed deceleration to the rails (2×40 % T/W 1.34 at 136.6 t: no hover), v → 0 at pin contact | same | 1–3 Raptors, roll-only RCS; engines off instantly on `caught` | both pins loaded, ≤1 m / ≤1 m/s | [~] implemented, no catch |

**Actuators by regime (R11).**

| Regime | Pitch / yaw | Roll | Translation | Cold gas |
|---|---|---|---|---|
| Orbit, deorbit | cold gas / RSL1 gimbal | cold gas | RSL1 | yes |
| Coil energised (entry, loft, pitch-up, early high-α): **≈ 95 → ≈ 30 km, V ≳ 2 km/s** | MHD electrodes | MHD (asymmetric patterns) | lift via bank | fallback only, reported |
| Coil off (late high-α, belly flop): **≈ 30 → ≈ 0.8 km**, no plasma | aero trim (free α-axis, passively stable) | cold gas (bank, heading) | lift via bank | yes |
| Flip → catch | Raptor gimbals | differential gimbal (≥ 2 engines) | Raptor thrust | **none** |
| caught / lost | – | – | – | run ends (no actuator) |

## 2. Entry and high-α guidance (NPC)

- Predictor: spherical rotating Earth, J2, US-76, wind forecast + measured offset, the same aero
  tables, MHD ΔC_D/ΔC_L table from the flight allocator, online MHD-drag scale, signed-bank lift
  dynamics with lift transport and rate limit, zero-lift footprint centre below Mach 2 (or flown σ
  through the belly flop), stop at the flip altitude. Miss measured in the pad frame along the inbound
  line (the track when < 15 km). — [x]
- Bank magnitude: grid scan → crossing nearest the current |σ| → secant. — [x]
- Reversal time: bisection (10 steps, 0.4 s) on the end crossrange with a range-scaled bias; update
  every 10/6/4 s, 1 s when a reversal is < 25 s away. — [x]
- Fallback when crossrange cannot be nulled: minimise the total miss over |σ|. — [x]

## 3. Belly-flop guidance

Shallow: the NPC above with belly trim. Steep: candidates (ψ₁ for τ, then ψ₂), ψ slewed at 1.8°/s,
point mass with wind; catch alignment in the final 6 km. — [x], accuracy not yet met (G-belly).

## 4. Landing MPC (packages/gnc/src/mpc.ts)

State p, v (pad ENU, relative to the aim point); control u (thrust acceleration); horizon 25 steps
over the remaining time; costs q_p, q_v, r_u, r_d (rate) + terminal Q_p, Q_v + glide slope; limits:
u_z ∈ [T_min, T_max]/m, tilt cone 25°, weak-axis slew penalty r_w. FISTA 60 iterations,
warm-started, 20 Hz. — [x] solver, [~] closed loop (overshoot).

Next (M3, last): SCvx 6-DoF free-final-time flip + landing (Szmuk & Açıkmeşe 2018; SCpp MIT
formulation) with an ADMM SOCP in TypeScript. — [ ]

## 5. Attitude control (TS-01)

ω_c = sat(k·e) (pointing vector-saturated, roll separately; optional spin), then a per-axis
time-optimal cap |ω_c,i| ≤ 0.8√(2 a_i |e_i|) with a_i the real authority (landing: computed from
the engine actually lit). τ = I K_d (ω_c − ω) + ω × Iω − M_ff; phase-plane deadband; torque about free
axes removed. — [x]

## 6. Allocation

1. MHD electrodes: box-constrained LS (weights 3:1:1), u ∈ [0, 1.6]⁸, momentum-bound fixed point. — [x]
2. Raptor gimbals: common gp/gy; roll-priority projection of the unconstrained solution onto
   |c·s| ≤ 0.7·RCS roll capacity (exact for the isotropic small-angle map). — [x]
3. Cold-gas couples: duty = demand / capacity; pitch/yaw dropped while the coil has authority or the
   engine residual exceeds 2× couple capacity. — [x]

## 7. Navigation

Perfect state (I1). Planned: MEKF with IMU/GNSS noise and biases (prior art: x7k2m9q 15-state
MEKF, Apache-2.0) for Monte Carlo realism. — [ ]

## 8. Coil switch-off and the engine flip (user decision Q25, 2026-11-21)

- The HTS coil stays on while the plasma still steers with it, so the ship does not tumble during the fall.
  Steering authority = the largest moment the coil can realise ÷ the moment the attitude law requests now. It is a
  flight-software estimate (R6), never the simulator's internal value.
- The coil is switched off (battery ramp-down at P_max, no dump) only when the authority stays below 1 for 3 s.
  There is no re-ramp.
- After the switch-off the ship flips to engine-first and the Raptors take the steering. The flip is no longer
  timed to 1.7 km; it follows the switch-off. The propellant consequence is checked in the mission run (Q24).
- This replaces the stagnation-flux criterion of the earlier session, which switched off at 2292 s while the
  bondline heating rate rose by about 40 %.
- Status: [x] implemented 2026-11-21. Authority = `allocateMhd` max pitch/yaw moment ÷ `torqueCommand` demand; hold ≥ 1.0; ramp-down after 3 s below 1.0 or when the plasma is gone. The flip is gated on the switch-off but still waits for the ignition altitude (the three-engine tail cannot fly from Mach 2).

## 9. Where the coil can steer: the plasma corridor (Q31, 2026-11-21)

- **Placement (unchanged, the only coil):** the HTS loop sits inside the nose, 1.20 m behind the tip (x = 50.90 m of
  52.1 m, loop radius 1.75 m, 2 T at the stagnation point), under the PICA-X cap, so its field threads the shock-layer
  plasma directly ahead of the nose. The nose-first glide is aerodynamically unstable; the attitude loop runs at
  2.5 × the unstable mode ω_u = √(|∂M/∂α|/I) and the electrode-controlled Lorentz force supplies the restoring and
  steering moment. The Q31 tail magnet was a separate, additional actuator; it was studied and rejected.

- The coil exerts force only on a conducting medium: |f| ≈ σ u B². The field strength or the number of coils
  does not matter where σ ≈ 0. Physics and numbers: research/03 §"Low-altitude MHD actuation".
- **Corridor flown:** from the q̄ > 30 Pa gate (≈ 95 km at entry speed) down to the authority switch-off at
  **29–37 km, V ≈ 1.9–2.5 km/s** (Q25). Below that the shock layer is no longer conducting, even seeded.
- **The coil is not a landing actuator in this design.** At 2 km (≈ 100–150 m/s) the air is neutral.
  Artificial ionisation would need ≈ 10¹¹–10¹³ W against recombination (on board: 450 kW). An MHD plume deflector
  was studied and rejected (spec/09: ≤ 2.5 kN per engine vs 634 kN from a 15° gimbal, 2.6–5.1 t with caesium).
- **The fall below 30 km stays stable without the coil:** the flapless hull trims passively at high α in the
  belly flop (statically stable in pitch/yaw), cold gas flies bank and heading, and after the flip one Raptor
  gimbal takes pitch/yaw while **short roll-only cold-gas pulses** take roll; 2/3 engines are staged in for the
  deceleration. Translational cold gas remains forbidden; R11c is revised accordingly (user 2026-11-21).
- **Switch-off is dynamic and complete (2026-11-21):** guidance keeps the coil while its own authority estimate
  (achievable MHD moment ÷ demanded moment) is ≥ 1; after 3 s below 1, or when the plasma is gone, it commands a
  **planned discharge** through the 1 Ω tap of the dump resistor bank (V = I_op·R = 3 kV, the insulation limit;
  τ = L/R ≈ 65 s). Measured (`.tmp/agentG/m17_final.log`): discharge from T+2799 s; at the flip (T+3092, 293 s later)
  I = 1.1 % (32 A, 0.01 % of the energy), at touchdown 0.5 %. An earlier flip raises this a little; the current then gives
  no steering moment (≤ 0.4 kN·m, agent P). Formerly the 450 kW battery ramp-down left ≈ 2 kA at touchdown. The timeline coil track ends at the real discharge (|I| < 50 A).
- Status: [x] documented; Q31 option 2 studied and rejected (docs/03-spec/09-plume-mhd-study.md). The Raptor gimbals
  remain the only landing actuator.

## 10. Catch envelope and wind margin (T09, user 2026-11-21)

The tower catches the ship only if **both pins land on the rails**. From `tower/src/tower.ts` and the
vehicle pins: closed gap 9.2 m (inner faces ±4.6 m), pin tips at ±5.3 m from the axis. The ship axis must
therefore be within **≈ ±1 m** of the tower centreline; the arms-close lateral trigger (`armCloseLat`,
default 1 m) is not the acceptance condition by itself. Vertical: pins at the rail plane, contact speed
**≤ ≈ 1 m/s** for the pin shock absorbers (2.4 MN/m, 0.9 MN·s/m). The chopsticks need 3 s, so they close on
**time-to-rails** (`armCloseLead` 3.7 s) at the predicted closure position; the contact still needs the
±1 m alignment and a northward arrival (`v_y > 0`, `|v_x| < 0.8`). The current catch condition uses
the **total** pin load; a per-pin contact check is still a T09 item (spec/11 §3.1), so a single-pin
load must not be reported as a catch.

- **Wind:** the arrival must be inside that envelope *with wind*, using the forecast plus the measured
  offset, with a positive margin. The acceptance test is the Monte Carlo **T-mc** (wind, κ, B, mass ±5 %):
  **≥ 18/20 caught**. A single nominal catch is not enough.
- **Current integrated run:** no catch. The integrated mission reaches the rail plane **11 m off**,
  climbing 3.0 m/s (`mission_verify.log`); the best full-6-DOF optimiser candidate is 6.8 m lateral /
  11.7 m/s (`bench/02`), and the later direction-gate sweeps are 74–231 m off. All are `lost`.
- **Design tension:** the user wants the flip as late as possible (L's latest feasible gate 1.4 km), but
  the entry at that gate delivers a 2.1 km miss while the catch needs ≤1 m after the landing's divert and
  the wind. The entry must deliver **≤0.5–0.8 km at the flip**; if the late gate cannot give a positive
  wind margin, the minimum earlier gate is the honest answer (R10).

### Gate trade measured by agent L (2026-11-21)

Lateral demand `a = (4L − 2 v_c T)/T²` vs the tilt-limited authority at the vertical the 40 % floor allows:

| Flip gate | Divert window | Needed | Available | Margin |
|---:|---:|---:|---:|---:|
| 1.4 km (latest vertical-feasible) | 19.8 s | 18.4 m/s² | 10.0 | **−8.5** |
| 1.7 km | 27.8 s | 10.6 | 11.3 | +0.7 (thin with 5.5 m/s wind + tracking) |
| 2.0 km | 35.7 s | 7.1 | 11.9 | **+4.8** |

At 2.0 km the L baseline still ended 186 m off because the achieved thrust-axis tilt lagged the command by
7–12° (≈2–2.5 m/s² lateral, ≈1.5 m/s² vertical). The vertical stall is fixed: the pins cross the rail
plane monotonically at ≤1 m/s. Later tuning reached 7 m and the optimiser's best candidate 6.8 m / 11.7 m/s,
but **no run caught**; the catch therefore needs the tilt tracking fixed and the latest **robust** gate
(2.0 km); 1.4 km stays open only if the belly-flop delivers ≤400 m.

### False catch found and retracted (2026-11-21)

The "CAUGHT" lines in `.tmp/agentG/m27_caught.log` and the first orchestrator run were **false catches**:
the old condition `pinLoad > 0.45 m g` fired with the arms still **open** (±13 m rails), a single pin grazing
a rail at 12.8 m off-centre. The catch now requires `mem.armsClose` (arms commanded closed, which only
happens inside 2 m). The first integrated run with the fix was `lost`: pins at the rail plane, **11 m off**,
climbing 3.0 m/s, arms open. Later tuning reached 7 m and the optimiser's best candidate 6.8 m / 11.7 m/s,
but **no run has caught**. No catch is claimed. The landing must finish the lateral null before the vertical
brake and fix the 7–12° tilt-tracking lag; the 2.0 km gate has +4.8 m/s² of authority, so this is
timing/tracking, not authority.

### Entry speed budget and flip floor (agent G, 2026-11-21)

Measured at Δv 108 (`m36`): entry 7399 → 117 m/s at the flip, **98.4 % of the speed shed** (entry 1402,
loft 367, high-α drag brake **4989**, belly flop 486). The flip speed is the belly-flop terminal velocity
(117 m/s; vertical −110), not a guidance choice: the passive trim is α ≈70°, C_D ≈3.0, v_t = √(2mg/ρSC_D);
the lift is 0.4 g so it cannot flare slower. More speed loss is not available inside the tile limit
(monotonic variants: tile 2029–2089 K, 104–199 kPa, 21–23 g, 46–112 km miss). Δv 108 improved the 2.0 km
delivery to **702 m** and cut the landing burn to **14.3 t** (header 11.75 t left); tile 1650 K (over 1530),
bondline 695 K (over 560), PICA 2089 K. The 4°/s barbecue roll is removed (roll target 0, short cold-gas
pulses; entry 2.3°/s, pitch-up transient max 5.8°/s, flip 0.00°/s; N₂ 2243 kg total).
