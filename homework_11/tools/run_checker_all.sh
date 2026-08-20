#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ONE="$ROOT/tools/run_checker_one.sh"

trap 'exit 130' INT
trap 'exit 143' TERM

pass=0
fail=0

for test_number in {1..10}; do
  echo "===== TEST $test_number ====="
  if "$ONE" "$test_number" 1; then
    pass=$((pass + 1))
  else
    fail=$((fail + 1))
  fi
done

echo "===== FULL SUMMARY ====="
echo "PASS=$pass FAIL=$fail"

((fail == 0))
