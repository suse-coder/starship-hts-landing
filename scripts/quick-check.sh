#!/usr/bin/env bash
# Fast edit gate: tier 0-1 only. No full mission.
# Use this when iterating on the catch/guidance/thrust/score; the full 6-DOF run is the integration
# gate (bun run mission / scripts/release-check.sh), not the inner loop. See spec/12.
set -euo pipefail
cd "$(dirname "$0")/.."

step() { printf '\n=== %s ===\n' "$1"; }

step "tier 0: tower contact / arms / collision"
bun test packages/tower

step "tier 0: gnc landing / guidance / mpc"
bun test packages/gnc

step "tier 0: score / catch margin / roll-only RCS"
bun test packages/sim/test/score.test.ts

step "tier 1: controller + thrust/gimbal model bake-off"
bun scripts/mpc-bench.ts

printf '\nquick-check: PASS (tier 0-1; no full mission run)\n'
