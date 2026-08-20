#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bad=0

patterns=(
  'telemetryUpdated|PATH_DIAG|DECISION_DIAG'
  'attackRunLocked|lockedAttackPlan|missionActive_'
  'controlFromDz10Motion|simulateNextDynamicRuntime|livePath|telemetryTime'
  'DROP_TRACE|MAIN_DROP|DROP_DIAG|RELAXED'
  'test0[1-9]|test10|checker_[0-9]|checker\.log'
  'hitRadius[[:space:]]*\*[[:space:]]*[01]\.'
)

for pattern in "${patterns[@]}"; do
  if grep -R -n -E "$pattern" "$ROOT/include" "$ROOT/src"; then
    bad=1
  fi
done

if ((bad != 0)); then
  echo "NO_HACKS_FAIL"
  exit 1
fi

echo "NO_HACKS_PASS"
