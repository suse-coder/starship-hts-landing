# spec/02 — Physics models (normative)

These models are the **only** way the state may change (spec/00 G0, R1–R3, R7). A model may be
improved or replaced by a more faithful one; it may never be bypassed, scaled or switched off to
make a phase work. Idealisations are listed in review/00.

## 1. Dynamics
State (r, v) ECI, q body→ECI, ω body. ṙ = v; v̇ = g_J2(r) + q⊗F_b⊗q* /m + F_contact/m;
I ω̇ = M − ω × Iω (principal axes); q̇ = ½ q ⊗ (0, ω). RK4; body loads (aero, MHD, engines, RCS,
geomagnetic torque) held over the step; gravity and contact evaluated per stage. Steps: 0.5 s in
vacuum, 0.1/0.02 s in entry, 0.01 s flip/landing, 4 ms terminal/catch.

## 2. Equilibrium air
Species N2 O2 NO N O NO⁺ N⁺ O⁺ e⁻; elements N:O = 0.79:0.21 ×2; per-volume partition
functions q = q_tr q_rot q_vib q_el with q_tr = (2π m kT/h²)^{3/2}, q_rot = T/(σθ_r),
q_vib = (1 − e^{−θ_v/T})⁻¹, q_el = Σ g_i e^{−θ_i/T}; K_c = Π q_products / Π q_reactants ·
e^{−ΔE0/kT} for N2⇌2N (9.759 eV), O2⇌2O (5.116), NO⇌N+O (6.497), NO⇌NO⁺+e (9.264),
N⇌N⁺+e (14.534), O⇌O⁺+e (13.618). Unknowns ln[N], ln[O], ln[e]; Newton with analytic Jacobian.
Normal shock: solve ε = ρ1/ρ2 from p_eq(ρ1/ε, h2(ε)) = p1 + ρ1U²(1−ε), h2 = h1 + ½U²(1−ε²);
stagnation by low-Mach isentropic compression.

## 3. Aerodynamics
Newtonian: Cp = Cp,max (v̂_p·n)² on windward panels, Cp,max from the shock table (→ 2 in free
molecular flow), leeward −0.5/M² ≥ −0.3, base Cp 0 with engines firing; skin friction
(laminar 1.328/√Re, turbulent 0.074 Re^{-0.2}/(1+0.144M²)^{0.65}). Strips: dN/dx = ½ρV²(−dS/dx)
sin2α cos(α/2) [nose-first] + η Cdc(M_c) ½ρV_c² 2r, Cdc 0.62 (transcritical) … 1.6 (M_c 1.3) …
1.25; η 0.68 → 1; axial CA0(M) − base part when firing. Blend w = smoothstep(2.5, 5.5, M).
Checked: subsonic broadside trim 84° stable, terminal velocity 84.5 m/s (100 t, sea level).

## 4. Heating and walls
q_s = 1.7415e-4 √(ρ∞/R_n) U³ (hot wall, × MHD factor); cap: Lees q/q_s = 0.1 + 0.9 cos^{1.6}θ';
elsewhere C_h = St* ρ* u_e with Eckert h* = 0.28h_e + 0.5h_w + 0.22h_r, St lam 0.332 Pr^{-2/3}Re*^{-1/2},
turb 0.0296 Pr^{-2/3}Re*^{-0.2}, transition Re_θ/M_e 200–300; edge from Newtonian pressure and
isentropic expansion from the stagnation state (γ_e from the table). Wall models: research/02.

## 5. MHD
Field per ampere-turn tabulated on (θ, height) from 5 filament loops (elliptic integrals). Cells:
θ ∈ [0, 80°] × 10, 2 layers, 16 azimuths, layer Δ(θ) = Δ0 (1+Q)^{1/4}(1 + 0.6θ²),
Δ0 = 0.78 ε R_n. Local p/p0 = cos²θ' + (p∞/p0) sin²θ' about the α-shifted stagnation point,
T = T0 (p/p0)^{(γ−1)/γ}, n = n0 (p/p0)^{1/γ}, u_e = √(2 c_p T0 (1 − (p/p0)^{(γ−1)/γ})),
n_e = n_e0 √(n/n0) exp(−(I_NO/2k)(1/T − 1/T0)) + Saha Cs (inner layer when seeding),
ν = ν_en0 (n/n0)√(T/T0) + n_Cs Q_Cs v̄_e + ν_ei, σ = n_e e²/(m_e ν), β = eB/(m_e ν),
σ_eff = σ/(1 + κβ²), j = σ_eff (1 − K_s)(u × B), f = j × B; vehicle: F = −Σ f dV, M = −Σ r × f dV;
power Σ K_s(1 − K_s) σ_eff |u×B|² dV; momentum bound s = 1/(1 + |F_lin|/F_cap),
F_cap = 1.2 q∞ π (R_n + Δ)²; Q = σ_eff B² R_n/(ρ∞U∞); heat factor (Δ0/Δ)^{1/2} with
Δ/Δ0 = √(1 + |F|/(q∞πR_n²)).

## 6. Magnet circuit
The coil is the ship's stabiliser instead of flaps (G3). Its authority is not a parameter: it is
whatever the plasma, the field and the electrodes give at that instant (§5), and it is zero when
there is no conducting shock layer. The magnet obeys its circuit, REBCO critical current I_c(B, T)
and cryogenic limits below; a quench removes the field and the authority, with its consequences.
dI/dt = V/L, V = min(1200 V, 450 kW/I); persistent otherwise; quench if T_cold > T_cs(I) → dump
τ = L/R = 2 s, 30 % of the energy into the cold mass; C dT/dt = 25 W + 0.003σA(T_w⁴ − T⁴) + AC −
lift(T); cryocooler electrical = 45 × lift.

## 7. Contact
Pins: vertical spring–damper on the arm rails when laterally over a rail, Coulomb friction μ 0.35;
hull vs arm inner faces: 3 MN/m with damping.
