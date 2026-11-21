# Release gate

The repository/Pages site is currently a **public WIP snapshot** (user-approved 2026-11-21). It
carries the WIP/failure banner and the catch is `lost`. A successful-mission release — a site
presented as the mission result — ships only when the physics has been verified, not merely when CI
is green.

## Public Pages deploy

Deploy publicly only when **all** of the following hold:

1. **True catch** — both pins on the rails, ship axis within ±1 m of the tower centreline, contact
   ≤1 m/s, arms closed (`armsClose`), with wind. A single pin grazing an open rail is a false catch.
2. **TPS within limits** — tile surface ≤1530 K and bondline ≤560 K through the flown trajectory.
3. **T-mc ≥18/20** — Monte Carlo caught within the envelope with wind, κ, B and mass ±5 %.
4. **Explicit user approval** — the user has approved the public release after the two checks above.

Then run the mechanical gate and let the workflow deploy:

```sh
bash scripts/release-check.sh   # typecheck + test + build + Pages artifact sanity
```

Pushing to `main` runs `.github/workflows/pages.yml` (the same typecheck/test/build, then uploads
`packages/app/dist` and deploys with `actions/deploy-pages`). The workflow is the deploy path; the
gate above is the authority.

## Public WIP deployment (approved 2026-11-21)

The user explicitly approved a public WIP repo and Pages site:

- Repo: <https://github.com/suse-coder/starship-hts-landing>
- Site: <https://suse-coder.github.io/starship-hts-landing/>
- Workflow run 44940950934: build + deploy green; the page loads, simulates in the worker and shows
  `LOST: landing missed the arms · OUTSIDE TPS LIMITS`.
- This is a labelled WIP snapshot, not a successful-mission release. Keep the banner; do not present
  it as a caught/operational result.

Private work-in-progress is also allowed at any time. WIP builds may show a lost catch or over-limit
heating, but must not be presented as the public mission result. Do not enable a success-release
deployment merely because CI is green.

## Status is not inferred from CI

A green workflow only proves that the code typechecks, tests pass and the site builds. Confirm the
current catch, TPS and T-mc status in `docs/03-spec/05-verification.md` and
`docs/02-plan/00-roadmap.md` before enabling or approving a public deploy. If any gate is open,
public release stays blocked.
