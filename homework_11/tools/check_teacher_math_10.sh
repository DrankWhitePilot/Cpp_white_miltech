#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHECKER="${HW11_CHECKER:-$ROOT/tests/bin/checker_linux_x86_64}"
RESULT_ROOT="${HW11_RESULTS_DIR:-$ROOT/test-results}"

if [[ ! -f "$CHECKER" ]]; then
  echo "Checker not found: $CHECKER"
  exit 1
fi

chmod +x "$CHECKER"
if ! mkdir -p "$RESULT_ROOT"; then
  echo "Cannot create results directory: $RESULT_ROOT"
  exit 1
fi
if ! LOGDIR="$(mktemp -d "$RESULT_ROOT/selftest_XXXXXXXX")"; then
  echo "Cannot create selftest log directory"
  exit 1
fi

pass=0
fail=0

for test_number in {1..10}; do
  log="$LOGDIR/selftest_${test_number}.log"
  "$CHECKER" "$test_number" --selftest >"$log" 2>&1
  checker_rc=$?

  if ((checker_rc == 0)) && grep -Fq ' -> HIT ' "$log"; then
    echo "T${test_number} PASS"
    pass=$((pass + 1))
  else
    echo "T${test_number} FAIL rc=$checker_rc"
    tail -40 "$log"
    fail=$((fail + 1))
  fi
done

echo "SELFTEST_LOGDIR=$LOGDIR"
echo "SELFTEST_PASS=$pass SELFTEST_FAIL=$fail"

((fail == 0))
