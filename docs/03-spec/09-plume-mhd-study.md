# spec/09 — Plume-MHD actuator study (Q31, option 2): can a magnet deflect the Raptor plume to steer the landing?

**Status:** study closed — **the actuator is rejected** on quantitative grounds; the physics and the
numbers are recorded here so the question is not reopened without new data. No code in `packages/`
is changed by this study.

**Question (Q31, user 2026-11-21, option 2).** Route C of research/03 §"Low-altitude MHD actuation":
the nose coil cannot act below ≈ 30 km (no plasma). The engines do burn there. An **aft magnet** around
the Raptor plume would act on gas that is already hot and partly conducting. Is that a useful supplement
to the gimbals — redundancy, faster roll, a gimbal-fail backup — and what does it cost the landing?

**Answer in one paragraph.** No. The Raptor exhaust leaves the nozzle at 1310 K and 0.83 bar; at that
temperature even 1 % caesium seeding gives σ ≈ 0.02 S/m, so the MHD force over the whole nozzle is
**2.5 kN of side force** (0.1 % of thrust, 0.07° of equivalent gimbal) at a 2 T field, and a plain ring
gives exactly zero. The gas is only a conductor (σ ≈ 5–70 S/m) at the throat and in the first metre of
the expansion, whose volume is 0.1–0.5 m³ — too small to matter; getting useful σ needs T > 2200 K, i.e.
the nozzle interior, not the plume. Seeding 1 % Cs costs **39 kg/s = 2.4 t per engine per minute**, the
magnet system costs **0.84 t per engine** and must hold 25 K next to a 900 K nozzle that radiates 206 kW onto it, and the
result is **0.4 % of one gimballed Raptor's moment** and less than a cold-gas pair. The mission-level
cost (Q19 precedent: 12 m/s of impact velocity per tonne at the T/W-limited margin) is 25–63 m/s for a
2.0–5.1 t system. The gimbal already gives 634 kN at 15°; a 10 % differential throttle between two
engines gives 0.32 MN·m for 0 kg. **Recommendation: reject the actuator; keep this study as the record.**
The nose coil remains a 95→30 km actuator only.

---

## 0. Method, labels and what was reused

Every number is labelled **S** (sourced: published data or the repository's own design values),
**C** (computed here from the numbers in the row) or **A** (assumed, with the conservative alternative).
Scripts: `.tmp/agentP/{common,plume,force,magnet,bandwidth,mass,trade}.ts`, run with
`nice -n 19 bun .tmp/agentP/<file>.ts`; outputs are quoted verbatim in the tables.

The physics chain is the same one R3 requires for the nose coil and the same code base
(`packages/mhd/src/mhd.ts`): equilibrium/Saha ionisation, electron–neutral + Coulomb collision
frequency, σ = n_e e²/(m_e ν), j = σ(u×B), f = j×B, reaction −∫j×B dV on the vehicle, and a momentum
bound. What is new is the **plume state** (a rocket nozzle, not a shock layer) and the **aft geometry**.

**Design values used (S):** chamber 300 bar / 3500 K class, F_SL 2.45 MN,
Isp_SL 350 s, A_e 1.33 m², 3 SL Raptors at r = 1.3 m, gimbal point x = 3.6 m, ±15°, 20°/s,
`designCoil` (2 T, 291 MJ, 4.15 t, 400 kg Cs), 450 kW converter, 25 K cold mass, 1.4 kg/W cryocoolers,
MLI ε_eff 0.003. Vehicle at the landing burn: 135.58 t, x_cg 28.06 m, I_yy 4.10·10⁷ kg·m²
(`.tmp/agentP/mass.ts` from `packages/craft`).

### 0.1 Assumptions, and the conservative alternative for each (R5)

| # | Assumption | Value | Basis | Conservative alternative and its effect |
|---|---|---|---|---|
| A1 | Combustion products: γ, molar mass | 1.2, 23 g/mol | A (methalox, 1.14–1.24, 22–24 g/mol) | γ 1.25 → T_exit 1037 K, σ_exit 0.0002 S/m, F 0.70 kN (computed) |
| A2 | Chamber stagnation | 300 bar, 3500 K | A (Raptor 3 is quoted at ~350 bar; T_c 3400–3600 K) | 3200 K → T_exit 1198 K, F 0.82 kN; 3200 K + γ 1.25 → F 0.24 kN (computed) |
| A3 | e–neutral cross-section of the products | 1·10⁻¹⁹ m² | A (H₂O is 3–10× larger at 0.3 eV) | 1·10⁻¹⁸ m² → σ 10× lower, F 10× lower (0.33 kN) |
| A4 | Seed | Cs, 1 % mole, injected in the chamber | A (Cs is the lowest stable alkali, 3.894 eV, S) | 0.3 % → F/1.8; potassium is worse (4.34 eV) |
| A5 | Current closure | κ = 1 (the plume current closes freely) | A, optimistic | κ = 0.15 → σ_eff/(1+0.15β²) ≈ /2 at the exit (β 2.6) and /1.2 in the throat |
| A6 | Field at the plume axis | 2.0 T | S (same design field as the nose coil, Q3) | 1.0 T → F/4 (0.64 kN) |
| A7 | Field fill of the interaction volume | on-axis value, no derating in the bore | A | a 0.8 fill factor → F −20 % |
| A8 | Shield temperature around the cold mass | 400 K | A | 600 K → 336 W cold load, 471 kg of cold heads |
| A9 | Plume emissivity, nozzle wall temperature | 0.2, 900 K | A | the shield load is already 615 kW; a hotter nozzle only worsens it |
| A10 | Plume electron density from real chemi-ionisation/alkali traces | not credited | — | even 10¹⁶ m⁻³ (optimistic measured class) gives σ 2·10⁻³ S/m → F ≈ 0.25 kN |

The assumptions all push the **same** way: the computed force is an **upper bound**. The conservative
column therefore only strengthens the verdict.

---

## 1. Q1 — the plume state and its conductivity

### 1.1 Nozzle state (isentropic, frozen γ)

The nozzle is choked, so the **exit plane is altitude-independent**; altitude changes the ambient
pressure and therefore the jet after the exit. With the design's own thrust, Isp and exit area:

| Quantity | Value | Basis |
|---|---|---|
| mass flow ṁ = F_SL/(Isp_SL g₀) | 713.8 kg/s | C |
| throat area A\* from ṁ, p₀, T₀ | 413 cm² (r\* = 115 mm) | C |
| area ratio A_e/A\* | 32.2 | C |
| exit Mach | 4.09 | C |
| **exit static T** | **1310 K** | C |
| **exit static p** | **0.825 bar** | C |
| **exit velocity** | **3.08 km/s** | C |
| exit density / number density | 0.174 kg/m³ / 4.56·10²⁴ m⁻³ | C |
| supersonic nozzle volume (15° cone) | 1.07 m³ | C |
| thrust check ṁu + (p_e−p_a)A_e | 2.18 MN vs 2.45 MN design; Isp_vac 330 s vs 369 s | C |

The 11 % thrust shortfall is the frozen-γ model against the design's real-gas Isp (recombination in the
nozzle raises the effective exhaust velocity). For conductivity this is the **safe** direction: the
model's gas is not hotter than the real one. If the real exit is 1450 K instead of 1310 K, σ rises by
≈ 6× at 1 % Cs — still far from the 10 S/m class, so the conclusion does not depend on it.

The nozzle is slightly **over-expanded at sea level** (p_e 0.83 bar < p_a 1.01 bar) and under-expanded
above ≈ 2 km:

| h (km) | p_a (kPa) | p_e/p_a | jet T after expansion (K) | jet u (km/s) | n (m⁻³) | regime |
|---|---|---|---|---|---|---|
| 0 | 101.3 | 0.81 | 1310 | 3.08 | 4.6·10²⁴ | over-expanded |
| 1 | 89.9 | 0.92 | 1310 | 3.08 | 4.6·10²⁴ | over-expanded |
| 2 | 79.5 | 1.04 | 1302 | 3.09 | 4.6·10²⁴ | under-expanded |
| 3 | 70.1 | 1.18 | 1275 | 3.11 | 4.5·10²⁴ | under-expanded |
| 4 | 61.6 | 1.34 | 1248 | 3.13 | 4.3·10²⁴ | under-expanded |
| 5 | 54.0 | 1.53 | 1221 | 3.14 | 4.2·10²⁴ | under-expanded |

The landing burn starts at ≈ 0.8 km, i.e. in the **over-expanded** column: the near field sits at exit
conditions, and expansion only makes it colder and less conducting downstream.

### 1.2 Conductivity (Saha + collisions)

σ = n_e e²/(m_e ν), with the seed ionised by Saha (Cs 3.894 eV, K 4.34 eV, Na 5.14 eV, all S: NIST),
ν = ν_e–neutral + ν_e–seed + ν_e–ion (Coulomb). At the exit plane (1310 K, 0.825 bar):

| seed | mole fraction | n_e (m⁻³) | ν (s⁻¹) | **σ (S/m)** | β at 1 T | β at 2 T |
|---|---|---|---|---|---|---|
| none, equilibrium (NO, 9.26 eV) | — | 1.6·10⁶ | 1.0·10¹¹ | 4·10⁻¹³ | — | — |
| none, trace K 1 ppm | 1·10⁻⁶ | 1.4·10¹⁴ | 1.0·10¹¹ | 4·10⁻⁵ | — | — |
| Cs | 1·10⁻⁴ | 1.0·10¹⁶ | 1.0·10¹¹ | 2.9·10⁻³ | 1.7 | 3.4 |
| Cs | 1·10⁻³ | 3.3·10¹⁶ | 1.1·10¹¹ | 8.8·10⁻³ | 1.7 | 3.3 |
| **Cs** | **1·10⁻²** | **1.0·10¹⁷** | **1.3·10¹¹** | **0.022** | **1.3** | **2.6** |
| Cs | 5·10⁻² | 2.3·10¹⁷ | 2.6·10¹¹ | 0.026 | 0.7 | 1.4 |
| Cs | 10⁻¹ (saturated) | 3.3·10¹⁷ | 4.1·10¹¹ | 0.023 | 0.4 | 0.9 |
| K | 1·10⁻² | 1.4·10¹⁶ | 1.3·10¹¹ | 0.0031 | 1.3 | 2.6 |
| Na | 1·10⁻² | 4.2·10¹⁴ | 1.3·10¹¹ | 8.9·10⁻⁵ | 1.3 | 2.6 |

Three things matter here.

1. **The exit is cold for MHD.** 1310 K is 700–1000 K below the open-cycle MHD-generator regime
   (σ ≈ 10 S/m at 1 bar needs 1 % Cs at 2000 K, 0.1 % at 2200 K, or ≈ 0.05 % above 2500 K).
   The plume behaves like a *cold* gas: 1 % Cs buys 0.02 S/m, not 10 S/m.
2. **Saturation.** Above a few percent seed the electron density saturates (the Saha root ∝ √(χ n) at
   small χ) and the extra Cs only adds collisions, so σ *falls*. There is no seed fraction that makes
   the exit a 10 S/m conductor.
3. **The Hall parameter is not small at the exit** (β = 1.3 at 1 T, 2.6 at 2 T), so the effective
   conductivity there is σ/(1+κβ²) — another 3–8× penalty on top of the low σ. It does not change the
   force integral: the exit contributes ≈ 2 % of ∫σu dV and the region that dominates (the throat to
   ε ≈ 6) has β < 0.25, where even κ = 1 costs < 10 %.

Inside the nozzle the picture is different, because the gas is hot:

| ε = A/A\* | A (m²) | T (K) | p (bar) | u (km/s) | n (m⁻³) | n_e (m⁻³) | ν (s⁻¹) | **σ (S/m)** | β (1 T) |
|---|---|---|---|---|---|---|---|---|---|
| 1.0 (throat) | 0.041 | 3182 | 169 | 1.17 | 3.9·10²⁶ | 4.7·10²² | 1.9·10¹³ | **72** | 0.01 |
| 1.5 | 0.062 | 2664 | 58 | 1.90 | 1.6·10²⁶ | 6.7·10²¹ | 6.8·10¹² | 28 | 0.03 |
| 2 | 0.083 | 2461 | 36 | 2.12 | 1.1·10²⁶ | 2.6·10²¹ | 4.4·10¹² | 17 | 0.04 |
| 3 | 0.124 | 2223 | 20 | 2.35 | 6.4·10²⁵ | 6.9·10²⁰ | 2.5·10¹² | 7.9 | 0.07 |
| 4 | 0.165 | 2076 | 13 | 2.49 | 4.6·10²⁵ | 2.7·10²⁰ | 1.7·10¹² | 4.5 | 0.10 |
| 6 | 0.248 | 1891 | 7.5 | 2.64 | 2.9·10²⁵ | 6.9·10¹⁹ | 1.0·10¹² | 1.9 | 0.17 |
| 8 | 0.330 | 1772 | 5.1 | 2.74 | 2.1·10²⁵ | 2.5·10¹⁹ | 7.1·10¹¹ | 1.0 | 0.25 |
| 16 | 0.660 | 1522 | 2.0 | 2.93 | 9.7·10²⁴ | 1.9·10¹⁸ | 3.1·10¹¹ | 0.17 | 0.58 |
| 32.2 (exit) | 1.330 | 1310 | 0.83 | 3.08 | 4.6·10²⁴ | 1.0·10¹⁷ | 1.3·10¹¹ | 0.022 | 1.32 |

**The conducting gas is the throat, and the throat is tiny.** The volume-weighted integral — the only
quantity the force integral cares about — is

| quantity | 1 % Cs | conservative (A3: 1e-18 m²) |
|---|---|---|
| ∫σ dV over the whole nozzle | **0.89 S·m²** | 0.11 S·m² |
| ∫σ u dV | **2.03·10³ S·m²·m/s** | 0.26·10³ |

That is the number that decides the study: a 10 S/m class **medium** exists only in a 0.1–0.5 m³ throat
region, and the exit — where the volume is — is 0.02 S/m. There is no arrangement of the magnet that
changes this; it is a property of the gas.

### 1.3 Seed mass (the consumable)

| Cs mole fraction | mass fraction | ṁ_seed per engine | 60 s | 40 s |
|---|---|---|---|---|
| 1·10⁻⁴ | 0.06 % | 0.41 kg/s | 25 kg | 16 kg |
| 1·10⁻³ | 0.57 % | 4.10 kg/s | 246 kg | 164 kg |
| **1·10⁻²** | **5.5 %** | **39.4 kg/s** | **2.36 t** | **1.58 t** |
| 5·10⁻² | 23 % | 165 kg/s | 9.9 t | 6.6 t |

The nose system already carries 400 kg of Cs: **10.2 s** of one engine at 1 %, or 98 s at 0.1 % (where
the force falls to 0.8 kN per engine). The seed is not a trim item; it is a propellant-sized consumable
(2.36 t per engine-minute, density 1930 kg/m³ → 1.22 m³ per engine-minute, in a heated tank).

---

## 2. Q2 — the force

### 2.1 Two mechanisms, and why a ring does nothing

For a flow u along the engine axis and a field **B**:

- j = σ(u×B), f = j×B. With u = u x̂ and B = B_x x̂ + B_y ŷ:
  **f_x = −σu B_y²** (axial braking = thrust loss) and **f_y = +σu B_x B_y** (side force).
- A **pure axial field** (B_y = 0, i.e. a plain ring or solenoid around the plume — the obvious
  "one ring per engine") therefore gives **f = 0**: u×B is azimuthal, j is azimuthal, j×B is radial
  and integrates to zero. Computed: 0.000 kN. A ring is not an actuator.
- A **pure transverse field** (B_x = 0) gives pure braking along the jet: **thrust loss, no side force**.
- A **canted field** (both components, e.g. a saddle/transverse winding superposed on the axial
  background) gives a genuine side force, because the Lorentz force on the gas lies in the (u, B) plane
  and pushes the plume toward the transverse component of B. The vehicle recoils the other way. This is
  the only geometry that works without electrodes, and it needs **no electrical power** (the current is
  induced; the energy comes from the jet's axial momentum), which is why it is the one costed here.

The momentum picture is consistent: a lateral force F on the gas gives the jet lateral momentum F, so
the jet is turned by δ = F/(ṁu) and the vehicle feels −F. There is no amplification: a 50 kN side force
needs a 50 kN Lorentz force, which is 2 % of the jet's 2.2 MN momentum flux.

### 2.2 Geometry and the integral

Coil: a 2 T field at a cant angle α — at 25° that is 1.81 T axial + 0.85 T transverse — from a solenoid
(radius 0.85 m, length 2.6 m, centred on the nozzle, NI 2.47 MA·t) plus a saddle winding. Field from the finite-solenoid formula, integrated over 400 volume
elements of the 15°-cone nozzle (1.07 m³), with σ, u from §1.2. **One engine:**

| quantity | 25° cant, 2 T, 1 % Cs | Basis |
|---|---|---|
| ∫σu dV | 2.03·10³ S·m²·m/s | C |
| **lateral force** | **2.54 kN** | C |
| axial force (thrust loss) | 1.18 kN (0.048 % of thrust) | C |
| lateral / thrust | 0.104 % | C |
| equivalent gimbal angle δ = F/(ṁu) | **0.066°** (the gimbal: 15°) | C |
| moment about the landing CG (arm 24.46 m) | 0.062 MN·m | C |
| fraction of one gimballed Raptor's moment (15.51 MN·m) | **0.40 %** | C |
| fraction of a cold-gas pair (0.16 MN·m) | 39 % | C |
| angular acceleration (I_yy 4.10·10⁷) | 0.0015 rad/s² (gimbal 0.378) | C |
| momentum bound ṁu = 2.20 MN | MHD uses 0.12 % | C |

Sensitivity (one engine, nozzle interior):

| seed χ | 0 | 1e-4 | 1e-3 | **1e-2** | 5e-2 |
|---|---|---|---|---|---|
| F_lat at 25°, 2 T (kN) | 0 | 0.22 | 0.67 | **2.54** | 1.93 (saturated) |
| F_lat at 45°, 2 T (kN) | 0 | 0.43 | 1.33 | 3.31 | 3.85 |
| F_lat at 25°, 1 T (kN) | 0 | 0.055 | 0.17 | 0.64 | 0.97 |
| conservative cross-section (1e-18), 1 %, 25°, 2 T | | | | **0.33** | |

Cant angle: 15°/25°/45° give 1.66/2.54/3.31 kN, but the axial loss rises as 0.44/1.18/3.31 kN. A
15–25° cant is the best ratio (lateral/axial ≈ 3.7–2.2); 45° is a pure brake.

The induced current is not free of dissipation: the ohmic heating in the gas is
B_t²·∫σu² dV = **3.44 MW**, i.e. 0.10 % of the jet's 3.39 GW kinetic power. It is paid from the jet, not
from the battery, so the actuator needs no power supply — but it does heat the plume and the magnet's
thermal shield (which is already at 615 kW, §4.2).

Conservative gas assumptions, same nozzle and field (computed, full integral):

| variant | T_exit (K) | ∫σu dV (S·m²·m/s) | F_lat (kN) |
|---|---|---|---|
| baseline T₀ 3500 K, γ 1.20 | 1310 | 2.03·10³ | **2.54** |
| T₀ 3200 K | 1198 | 680 | 0.82 |
| γ 1.25 | 1037 | 623 | 0.70 |
| T₀ 3200 K and γ 1.25 | 948 | 219 | **0.24** |

**Two engines:** 5.08 kN, 0.124 MN·m (0.80 % of the gimbal moment) — the case in the trade table.

**Beyond the exit:** the near-field plume (r 0.65→1.6 m over 3 m, 12.6 m³) at exit conditions gives
σ 0.022 S/m, ∫σdV 0.28 S·m² and **≤ 0.4 kN** — *less* than the nozzle, because the volume grows but the
temperature does not. A bigger magnet around the plume does not help; the gas is colder there.

**The power-limited electrode variant** (axial B + segmented electrodes driving a transverse current,
the classical cross-field accelerator, S: Resler & Sears 1958; Rosciszewski 1965 studied exactly this in
a rocket throat): F = B√(PσV). At the 450 kW converter: **1.1 kN** in the nozzle, 0.4–0.9 kN in the
plume. To reach 50 kN this way needs **≈ 900 MW** of ohmic power — 2000× the converter. Electrodes also
ablate in a 2500 K seeded flow (S: the ablation problem is the standard life limit of MPD/cross-field
thrusters).

### 2.3 What the force can and cannot do

- It cannot replace a gimbal: 2.5 kN/engine vs 634 kN (0.4 %), and it works only while the engines burn —
  exactly when the gimbals already work. It adds no capability in any phase where the vehicle has no
  other actuator.
- It cannot back up a gimbal failure: with two engines, a **10 % differential throttle** gives
  0.32 MN·m for 0 kg (5× the MHD moment); a 40 % differential gives 1.3 MN·m.
- It cannot give roll: a lateral force at the aft gives no torque about x, and an azimuthal (roll) force
  of the same 2.5 kN at r = 0.65 m would be 1.6 kN·m against the cold-gas couple's 36 kN·m.
- It can, in principle, apply 1.18 kN of extra axial braking (0.05 % of thrust) — worthless.

---

## 3. Q3 — bandwidth: a superconducting field cannot be steered, so neither can the actuator

The nose coil cannot be modulated (L = 64.6 H, I = 3 kA, 450 kW → V = 150 V, dI/dt = 2.3 A/s: a 10 %
field change takes 129 s). The aft magnet is much smaller but the same physics applies:

| actuator | L | dI/dt at 450 kW | 10 % change | full swing | first-order bandwidth |
|---|---|---|---|---|---|
| nose coil (S) | 64.6 H | 2.32 A/s | 129 s | 21.5 min | — |
| aft axial magnet (2.09 H, 3 kA) | 2.09 H | 71.8 A/s | 4.2 s | 42 s | 0.24 rad/s |
| fast transverse trim winding (0.52 T, 300 A) | 4.99 H | 301 A/s | 0.10 s | 1.00 s | **1.0 rad/s** |
| electrode loading (nozzle, σ 4.5 S/m) | ~20 µH loop | — | — | L/R 60 µs, flow transit 0.5 ms | **kHz** |

How it would be steered, therefore, is exactly the nose-system split: a **persistent background field**
(axial, set before the burn) plus **fast modulation** in the plasma. Two options:

1. **Electrode loading** (the R11a method): microsecond current control, but only where σ is high (the
   nozzle throat), and the power limit caps it at ≈ 1 kN. The force is fast; the force is also useless.
2. **A transverse trim winding**: 1.0 rad/s first-order, i.e. barely at the 0.8 rad/s landing loop, and
   it must exchange 225 kJ on every swing (450 kJ from +300 to −300 A). Hysteretic AC loss in the tape (0.1–1 % of the stored energy per
   cycle, A) is 28–286 W at 25 K when cycled at 0.8 rad/s — 1.3–13 kW of cryocooling power and 40–400 kg
   of cold heads, on top of the magnet's own load. Sustained control at the loop frequency is not
   affordable; the winding can be repositioned every few seconds, not used as a control surface.

So the honest bandwidth statement: **the field is a slow trim (0.1–0.24 rad/s), the only fast mechanism
is the electrode current (kHz) which produces ~1 kN, and the useful canted-field force has no fast
modulation at all.** An MHD-only attitude loop would close at ≈ 0.29 rad/s (√(0.062/0.46)·0.8), below
the 0.8 rad/s the landing uses. The frequency budget is not the blocker — the authority is.

Direction control is the other half of the same problem: the side force is ∝ B_x·B_t, so **reversing it
means reversing the transverse component** — a 450 kJ swing of the trim winding (1 s, 28–286 W of 25 K
AC loss), not a valve command. A segmented saddle (several persistent saddle sectors, one per direction)
does not help: switching a 300 A persistent current between 5 H sectors is a quench/arc event, not a
control action. Only electrode loading could reverse the force quickly, and it is the 1 kN mechanism.

---

## 4. Q4 — mass, power, placement, and the comparison with the gimbals

### 4.1 The magnet (sized with the `designCoil` rules)

Per engine: bore 0.85 m (nozzle exit radius 0.65 m + clearance), length 2.6 m, B₀ 2 T, I_op 3 kA,
J_e 700 A/mm² at 25 K (Q8), tape fraction 0.35, virial structure, 15 kg/m² cryostat, 1.4 kg/W at 25 K.

| item | per engine | basis |
|---|---|---|
| NI / turns / L / stored energy | 2.47 MA·t / 824 / 2.09 H / 9.39 MJ | C (nose: 9.93 MA·t, 64.6 H, 291 MJ) |
| REBCO tape | 174 kg | C, NI·2πa/J_e·ρ |
| CF structure | 25 kg | C, virial, safety 2 |
| cryostat + MLI | 340 kg | C, 15 kg/m² × 22.6 m² |
| actively cooled thermal shield | 70 kg | A |
| converter + busbars | 100 kg | A |
| cold heads (400 K shield) | 127 kg | C, 90.5 W at 25 K |
| **dry total** | **835 kg per engine; 1.67 t for two** | C |
| seed tank + pumps | 315 kg (for 3.2 t of Cs: two engines × 40 s at 1 %) | A, 10 % of the seed |
| **at the flip, two engines, 1 % Cs, 40 s** | **2.0 t dry + 3.2 t Cs = 5.1 t** | C |
| battery | **0 (reused)** — 9.39 MJ = 2.6 kWh against the existing 100 kWh coil battery, which is free after the nose-coil discharge | C |

### 4.2 Thermal environment — the part the nose coil never sees

The coil surrounds a 900 K metal nozzle and sits in the plume's radiation field:

| source | value | basis |
|---|---|---|
| nozzle wall 900 K, 6.9 m², ε 0.8 | **206 kW** onto the shield | C |
| plume 1310 K, 3 m × Ø1.3 m, ε 0.2 | **409 kW** onto the shield | C |
| total on the shield | **615 kW** | C |
| coolant: CH₄ at 155 kg/s, cp 3.5 kJ/kgK | +1.13 K — feasible, but it is a new cooled panel in the engine bay | C |
| MLI leak to 25 K, shield 300/400/500/600 K | 49 / 90 / 178 / 336 W | C |
| cold heads at 400 K shield | 127 kg, 4.07 kW electric | C |
| conservative (600 K shield) | 471 kg, 15.1 kW | C |

This is an engineered cryostat, not a bolt-on: a cooled shield inside the plume radiation field, HTS
current leads (≈ 5 W at 3 kA with an N₂ intercept), a 25 K cold mass ≈ 0.2 m from a 900 K nozzle, and a
4 kW cryocooler. It is *possible*; it is not cheap, and every kilogram sits at the worst place for the
landing (the engine bay, at the flip).

Open engineering items not costed: the field on the engine hardware (≈ 0.6 T at the powerhead, A) and
the torque on the steel; the nozzle is a **conductor**, so the MHD current can close through it (the
nozzle becomes an electrode — good for the force path, bad for erosion); Raptor acoustic/vibration
qualification of a cold mass; and the fact that a 2 T field's Maxwell stress B²/2μ₀ = 1.6 MPa acts on
whatever excludes it (the nozzle and the engine bay structure).

### 4.3 The comparison the design already has

| actuator | side force | moment about the landing CG | mass | consumable | phase |
|---|---|---|---|---|---|
| one Raptor, 15° gimbal | **634 kN** | **15.51 MN·m** | 0 (existing) | 0.29 kg/kN·s | after the flip |
| 10 % differential throttle, 2 engines | — | 0.32 MN·m | 0 | existing propellant | after the flip |
| cold-gas pair, 2 × 4 kN | 8 kN | 0.16 MN·m | 0 (existing, 5 t N₂) | 1.4 kg/kN·s (Isp 73 s) | belly flop |
| **plume MHD, 2 engines, 1 % Cs** | **5.1 kN** | **0.124 MN·m** | **2.0 t + 3.2 t Cs** | **15.5 kg Cs/kN·s** | only while burning |
| plume MHD, 1 engine | 2.5 kN | 0.062 MN·m | 1.0 t + 1.6 t Cs | 15.5 kg Cs/kN·s | only while burning |

The seed cost per unit impulse is 11× cold gas and 53× the gimbal: even if the force were free, the
consumable would make it the most expensive way to make a newton-second on the vehicle.

---

## 5. Q5 — verdict

### 5.1 What it would cost the landing

Options at a 2 T design field, 40 s of seeding during the ~80 s burn (half of it needs lateral authority):

| option | dry (t) | Cs (t) | total at the flip (t) | F_lat (kN) | moment (MN·m) | % of gimbal | % of a cold-gas pair | ideal Δv prop (t) | impact velocity at the Q19 rate (m/s) |
|---|---|---|---|---|---|---|---|---|---|
| 1 engine, 1 % Cs | 0.99 | 1.58 | **2.57** | 2.54 | 0.062 | 0.40 | 39 | 0.10 | **32** |
| 2 engines, 1 % Cs | 1.99 | 3.15 | **5.14** | 5.08 | 0.124 | 0.80 | 78 | 0.20 | **63** |
| 2 engines, 0.1 % Cs | 1.70 | 0.33 | **2.03** | 1.61 | 0.039 | 0.25 | 25 | 0.08 | **25** |

The force behind these rows is 0.24–2.54 kN per engine depending on the gas assumptions (A1/A2), so the
rows are the optimistic end of a range that only gets worse. Two very different cost estimates bracket the
bill, and both are bad:

- **Thermodynamic minimum:** at the measured landing Δv ≈ 130 m/s, 1 t of dry mass costs 39 kg of
  propellant (e^(Δv/v_e) − 1, Isp 350 s). The 5.1 t option costs 0.20 t — 0.8 % of the 26 t header.
  That alone would be affordable.
- **Mission-level (measured):** Q19 recorded that 1.37 t of dump resistor cost **16.8 m/s** of impact
  velocity at this vehicle's T/W-limited hoverslam — 12.3 m/s per tonne. At that rate 5.1 t costs
  **63 m/s** of impact velocity, i.e. it turns a marginal catch into a loss. The 2.57 t option costs
  32 m/s. The mechanism is not the Δv but the ignition altitude and the residual propellant in a burn
  that cannot hover (T/W 1.84 at full thrust, 0.74 at the 40 % minimum).
- The T/W band itself still closes with +5 t (hover throttle 0.54 → 0.57), so the vehicle is not
  *structurally* excluded — it is excluded by the margin the mission actually has.

**The thrust loss is the one cheap item.** The axial MHD force (1.18 kN, 0.048 % of thrust) over 40 s at
Isp 350 s costs 13.8 kg of propellant — negligible, and *smaller* than the mass penalty of carrying the
magnet. The trade is entirely mass and consumable versus authority, not efficiency.

### 5.2 What would have to be true for a useful actuator

For 50 kN of lateral force (8 % of the gimbal, 2 % of ṁu) at 2 T and a 25° cant, the integral must be
**16× larger** than the nozzle can give (3.26·10⁴ vs 2.03·10³ S·m²·m/s). Since σ ∝ √χ at fixed T:

- **seed × 258** → 258 % mole Cs (more caesium than exhaust) — impossible;
- **field × 4** → 8 T on a 1.2 m bore around a 900 K nozzle — a different magnet class entirely;
- **volume × 16** → a dedicated seeded MHD channel at T > 2200 K, 16 m³ — that is a new engine
  (an MHD-augmented nozzle), not an add-on, and it inherits the 900 MW-class power or the seed flow.

Cross-field rocket-nozzle accelerators were studied from 1958 (S: Resler & Sears 1958; Rosciszewski 1965, "Rocket motor with electric acceleration in the throat") and their documented limits are the ones met here: conductivity, electrode ablation and power.

### 5.3 Recommendation

**Reject the plume-MHD actuator. Keep this study (Q31 option 2 answered: do not build it).**

Reasons, in order of weight:

1. **Authority.** 2.5 kN/engine = 0.07° of gimbal, 0.4 % of the gimbal moment at a 2 T field and 1 % Cs;
   0.33 kN under the conservative cross-section. The 10–100 kN of research/03 §Route C is withdrawn: it
   assumed σ ≈ 10 S/m, which the exit gas reaches only at ≈ 2000 K, not 1310 K.
2. **It exists only where the gimbals already are.** After the flip the vehicle has 634 kN of side force
   per engine and 0.32 MN·m of free differential-throttle torque. Nothing in the landing phase needs a
   third, weaker, mass-carrying actuator.
3. **The consumable.** 39 kg/s of Cs = 15.5 kg per kN·s, 11× cold gas and 53× the gimbal; 400 kg on
   board buys 10 s at 1 %.
4. **The mass and the thermal problem.** 0.84 t per engine of 25 K hardware ≈ 0.2 m from a 900 K nozzle
   radiating 206 kW at it, 4 kW of cryocooling, and 2–5 t at the flip — 25–63 m/s of impact velocity at
   the measured marginal-mass sensitivity.
5. **Bandwidth.** The field is a 0.1–0.24 rad/s trim; the fast electrode force is ~1 kN; a trim winding
   reaches 1 rad/s only by burning 1.3–13 kW of cryogenic cooling.

**If a decision later reopens this,** the only physically credible route is not an aft magnet but a
seeded MHD *nozzle stage* (T > 2200 K, alkali seeding at the several-percent level or a closed cycle,
ceramic nozzle) — an engine
programme, judged against the gimbal it would replace, not an actuator added to the vehicle.

### 5.4 What the nose coil can and cannot do below 30 km (must be stated explicitly)

| | fact | number |
|---|---|---|
| Can | steer in the plasma corridor 95 → 29–37 km (R11a, Q25) | flown: authority → 0 at 29–37 km, 1.9–2.5 km/s |
| Cannot | act below 30 km: the air is neutral, σ ≈ 0 at 100–150 m/s | 0 kN (research/03 §Route A) |
| Cannot | reach the plume: its field at the engine bay (47 m away) is μ₀m/4πr³ | **9·10⁻⁵ T** vs the 2 T needed |
| Cannot | help as a "left-on" magnetic torque: Earth-field dipole torque m×B | **4.8 kN·m** at full current = 0.03 % of the gimbal moment |
| Must | be discharged after the steering switch-off (Q22: 3 kV, 1 Ω tap, τ = 65 s) | at the landing ignition (168 s later) **7.6 %**, 223 A, 0.36 kN·m; at touchdown (248 s) 2 % (a battery ramp-down instead would leave 87 %, 4.2 kN·m — still nothing) |
| Note | so the coil is *not* a landing actuator in any mode, and its residual current is physically irrelevant | see open item O1 |

---

## 6. Open items

| # | Item | Status |
|---|---|---|
| O1 | spec/03 §9 says the discharge is "below 1 % ≈ 300 s later, **before the landing burn**". `mission_final.log` switches off at T+2606 s and ignites at T+2774 s (168 s later), so with the 1 Ω tap (τ = 65 s) the current at ignition is 7.6 % (223 A, 0.36 kN·m) and 1 % arrives at ≈ T+2905 s, after touchdown. The log's event wording is the older "battery ramp-down" path; under that path the current at ignition would be 87 % (≈2.6 kA, 4.2 kN·m). Either way the torque is ≤ 0.03 % of the gimbal moment, so the physics conclusion is robust; only the sentence in spec/03 §9 needs fixing (agent C2 / spec author). | **open (docs wording)** |
| O2 | Real plume ionisation from chemi-ionisation and propellant alkali traces (measured rocket plumes reach 10¹³–10¹⁶ m⁻³). Even 10¹⁶ gives σ 2·10⁻³ S/m and F ≈ 0.25 kN, so the verdict does not depend on it — but a measured Raptor value would replace assumption A10. | open, not blocking |
| O3 | Nozzle/engine interaction of a 2 T field: current closure through the metal nozzle, Maxwell stress 1.6 MPa, turbopump torque. Only matters if the concept is ever revisited. | open, not blocking |
| O4 | A seeded MHD nozzle stage (T > 2200 K, 10 % Cs or closed cycle) is the only route to a useful force; it needs an engine study, not a vehicle study. | out of scope |
| O5 | Q31 wording in `docs/02-plan/05-open-questions.md` still says "≈ 10–100 kN per engine (vs ≈ 630 kN)". This study replaces it with ≤ 2.5 kN at 2 T / 1 % Cs (≤ 0.33 kN conservative). The research/03 §Route C text should be corrected by whoever owns that file. | **open (docs)** |

## 7. Reproduction

```
nice -n 19 bun .tmp/agentP/plume.ts       # Q1: nozzle/plume state, Saha, seed mass
nice -n 19 bun .tmp/agentP/force.ts       # Q2: field integral, canted/ring/electrode variants, bounds
nice -n 19 bun .tmp/agentP/magnet.ts      # Q4: magnet mass, plume radiation, cryocooler
nice -n 19 bun .tmp/agentP/bandwidth.ts   # Q3: coil/trim/electrode bandwidth vs 0.8 rad/s
nice -n 19 bun .tmp/agentP/mass.ts        # CG, inertia, gimbal/cold-gas authority, marginal propellant
nice -n 19 bun .tmp/agentP/trade.ts       # Q5: the trade table and the performance gap
```

No mission, profile or browser run was used. The only repository code imported is `@ship/craft`
(pure mass properties) in `mass.ts`; the physics is re-derived in `common.ts` so the study can be read
without the engine, with the same formulas as `packages/mhd/src/mhd.ts` and `packages/gas/src/plasma.ts`.
