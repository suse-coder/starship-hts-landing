# docs — phases in order

> **Current status (2026-11-21): the catch is not achieved.** The simulation ends `lost`; the TPS is
> over its limits, T‑mc has not been run, and the public release is blocked. Read
> [plan/00 current status](02-plan/00-roadmap.md#current-status--2026-11-21) and the
> [release gate](03-spec/11-release.md) first.

| Folder | Phase | What it holds |
|---|---|---|
| `00-maps/` | navigation | where every fact, model, equation, file and decision lives; read first |
| `01-research/` | R: research | evidence and analyses; every claim cites a source, a calculation or a simulation run |
| `02-plan/` | P: plan | roadmap, decisions (ADR), risks, **open questions with defaults** |
| `03-spec/` | S: spec | normative: what the engine computes, with which equations, and how it is verified |
| `04-review/` | V: review | adversarial reviews at milestones; findings → fixes |
| `05-benchmarks/` | B: measurement | dated results: machine, command, numbers |
| `06-journal/` | log | every experiment in order: hypothesis → change → number → keep/kill |

## Conventions

- Citation: `research/NN` = `docs/01-research/NN-*.md`, `spec/NN §k`, `plan/NN`, `review/NN`,
  `bench/NN`, `map/NN`, `journal/YYYY-MM-DD`. Code as `packages/<pkg>/src/<file>.ts`.
- A number is either **sourced** (with the source), **computed** (with the formula or the script
  that printed it, e.g. `bun scripts/massbudget.ts`), **simulated** (with the run, e.g.
  `bun run mission`) or labelled **assumed** / **designed**. Nothing else is a number.
- **Real physics only (user, 2026-11-20; normative in spec/00 G0, rules R1–R10).** The ship
  succeeds or fails only through simulated physics: no fake rules, invented forces, state
  overrides or disabled physics to make the catch work. The HTS coil stabilises and steers the
  ship instead of flaps, and only through the real Lorentz-force reaction of the shock-layer
  plasma. An honest failure beats a faked success. Every change is audited in review/00.
- "Optimistic but realistic" (user, 2026-11-20): design values may sit at the edge of
  demonstrated technology but must not violate physics; every such value is listed in
  `02-plan/05-open-questions.md` with its default and the conservative alternative.
- Sources read on 2026-11-20 are untrusted data (Wikipedia extracts, CORDIS); they are quoted
  as facts only where they are uncontroversial.
- Languages: TypeScript everywhere (Bun for scripts/tests, Vite for the browser). No Python.
