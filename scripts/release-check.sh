#!/usr/bin/env bash
# Release gate for the public Pages build: typecheck, tests, production build, artifact sanity.
# Usage: bash scripts/release-check.sh
#
# A green run here is necessary but not sufficient for a public deploy — see .github/RELEASE.md.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

summary=()
failures=()

run_gate() {
  local name="$1"
  shift
  local start=$SECONDS
  echo
  echo "==> ${name}: $*"
  local code=0
  "$@" || code=$?
  local elapsed=$((SECONDS - start))
  if ((code == 0)); then
    summary+=("PASS  ${name}  (${elapsed}s)")
    echo "==> ${name}: PASS (${elapsed}s)"
  else
    summary+=("FAIL  ${name}  (${elapsed}s, exit ${code})")
    echo "==> ${name}: FAIL (exit ${code})"
    failures+=("${name}")
  fi
}

check_artifact() {
  local index="packages/app/dist/index.html"
  if [[ ! -f "${index}" ]]; then
    echo "missing ${index}"
    return 1
  fi
  if grep -q 'src="/assets/\|href="/assets/' "${index}"; then
    echo "${index} references absolute /assets/ paths (breaks a Pages subpath)"
    return 1
  fi
  if ! grep -q 'src="\./assets/' "${index}"; then
    echo "${index} has no relative ./assets/ script (Vite base is not './'?)"
    return 1
  fi
  echo "pages artifact OK: ${index} uses relative ./assets/ paths"
  du -sh packages/app/dist | sed 's/^/artifact size: /'
}

run_gate "typecheck" bun run typecheck
run_gate "test" bun run test
run_gate "build" bun run build
run_gate "artifact" check_artifact

echo
echo "================== RELEASE GATE =================="
for line in "${summary[@]}"; do
  echo "  ${line}"
done
echo "=================================================="

if ((${#failures[@]} > 0)); then
  echo "GATE: FAILED — ${failures[*]}"
  exit 1
fi

echo "GATE: PASSED — typecheck, tests, build and Pages artifact are green."
echo "Reminder: public Pages also requires a true catch, TPS/T-mc sign-off and user approval."
echo "See .github/RELEASE.md before deploying."
