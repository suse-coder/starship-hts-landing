# Glossary

| Term | Meaning here |
|---|---|
| **Ship 42 / Flight 15** | the Block 3 ship caught at Pad 2 on 2026-11-14; our vehicle is its flapless derivative |
| **nose-first glide** | entry with the PICA-X nose leading at α 10–14°; aerodynamically unstable, trimmed by the coil |
| **coil / magnet** | REBCO HTS loop (a = 1.75 m, 9.93 MA·t, 2.0 T at the stagnation point) 1.2 m behind the nose tip |
| **electrode sector / loading factor K** | 8 azimuthal electrode groups; j = σ_eff(1−K)(u×B). K = 1 open circuit (no force), 0 < K < 1 generator (extracts power), K < 0 driven (accelerator). Code uses u = 1 − K ∈ [0, 1.6] |
| **Stuart number Q** | magnetic interaction parameter σ_eff B² R_n /(ρ∞ U∞) at the stagnation point |
| **Hall parameter β** | ω_ce/ν: electron gyrations per collision. Large β short-circuits the azimuthal current unless the Hall current is blocked |
| **Hall closure κ** | fraction of the Hall current that closes inside the shock layer: σ_eff = σ/(1+κβ²). κ = 0 ideal blocking (insulating, segmented wall), κ = 1 fully shorted |
| **seeding** | caesium injected at the nose to raise n_e and collision frequency (lower β) |
| **momentum bound** | the Lorentz force cannot exceed the momentum flux through the interaction region: F → F/(1+F/F_cap) |
| **trim lift loss** | the restoring nose force points to the windward side: it is negative lift (ΔCL −0.15…−0.23) and comes with ΔCD +0.22…+0.37 |
| **authority ratio** | achievable MHD moment along the command / demanded moment |
| **loft** | lift vector up (bank 0) after V < 6 km/s to lower q before pitch-up |
| **pitch-up** | release of the unstable nose-first attitude into the passive aerodynamic trim (α ≈ 51° hypersonic) at the loft's q minimum |
| **barbecue roll** | slow spin about the body axis (4–6°/s) so the windward heating sweeps all rays of the heat shield and steel backing |
| **free axis** | the α-pitch axis x_b × v̂ in aerodynamically stable flight: no torque is spent on it |
| **couple** | two cold-gas thrusters fired together: pure torque, no net force |
| **NPC** | numerical predictor–corrector entry guidance (bank magnitude from repeated 3-DOF predictions) |
| **aim bias** | target point past the pad (60 km entry, 6 km high-α) because later phases can only dissipate range |
| **chopsticks / arms** | Mechazilla's two catch arms; the ship's two pins land on their rails |
