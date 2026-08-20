#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${HW11_BUILD_DIR:-$ROOT/build}"
BIN="${HW11_BIN:-$BUILD_DIR/mission_uart_drop}"
CHECKER="${HW11_CHECKER:-$ROOT/tests/bin/checker_linux_x86_64}"
RESULT_ROOT="${HW11_RESULTS_DIR:-$ROOT/test-results/live}"
TIME_SCALE="${HW11_SCALE:-1}"
CHECKER_TIMEOUT="${HW11_TIMEOUT:-130}"

test_number="${1:-10}"
runs="${2:-1}"

if ! [[ "$test_number" =~ ^([1-9]|10)$ ]]; then
  echo "Test number must be 1..10"
  exit 1
fi

if ! [[ "$runs" =~ ^[1-9][0-9]*$ ]]; then
  echo "Run count must be a positive integer"
  exit 1
fi

if ! [[ "$TIME_SCALE" =~ ^[1-9][0-9]*$ ]]; then
  echo "HW11_SCALE must be a positive integer"
  exit 1
fi

if ! [[ "$CHECKER_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
  echo "HW11_TIMEOUT must be a positive integer"
  exit 1
fi

for command_name in grep setsid socat stdbuf sudo timeout; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Missing command: $command_name"
    exit 1
  fi
done

if [[ ! -x "$BIN" ]]; then
  echo "Student binary not found or not executable: $BIN"
  exit 1
fi

if [[ ! -f "$CHECKER" ]]; then
  echo "Checker not found: $CHECKER"
  exit 1
fi

chmod +x "$CHECKER"
if ! mkdir -p "$RESULT_ROOT"; then
  echo "Cannot create results directory: $RESULT_ROOT"
  exit 1
fi
cd "$ROOT" || exit 1

sudo -v || exit 1
if ! sudo modprobe gpio-sim; then
  echo "Cannot load gpio-sim"
  exit 1
fi

socat_pid=""
checker_pid=""
student_pid=""

stop_group() {
  local pid="${1:-}"
  if [[ -z "$pid" ]]; then
    return
  fi

  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 "$pid" 2>/dev/null; then
        break
      fi
      sleep 0.05
    done
  fi

  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup_run() {
  stop_group "$student_pid"
  stop_group "$checker_pid"
  stop_group "$socat_pid"
  student_pid=""
  checker_pid=""
  socat_pid=""
}
trap cleanup_run EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

pass=0
fail=0

for ((run_number = 1; run_number <= runs; ++run_number)); do
  cleanup_run

  logdir=""
  if ! logdir="$(mktemp -d \
    "$RESULT_ROOT/T${test_number}_R${run_number}_XXXXXXXX")"; then
    echo "Cannot create run log directory"
    exit 1
  fi
  ttydir="$logdir/ttys"
  if ! mkdir -p "$ttydir"; then
    echo "Cannot create PTY directory: $ttydir"
    exit 1
  fi

  echo "RUN test=$test_number run=$run_number scale=$TIME_SCALE"
  echo "LOGDIR=$logdir"

  setsid socat -d -d \
    pty,raw,echo=0,link="$ttydir/ttyA" \
    pty,raw,echo=0,link="$ttydir/ttyB" \
    >"$logdir/socat.log" 2>&1 &
  socat_pid=$!

  sleep 0.5

  setsid timeout --kill-after=3s "${CHECKER_TIMEOUT}s" \
    sudo stdbuf -oL -eL "$CHECKER" "$test_number" \
      --time-scale "$TIME_SCALE" \
      --uart "$ttydir/ttyB" \
      --start-line 24 \
      --drop-line 23 \
    >"$logdir/checker.log" 2>&1 &
  checker_pid=$!

  gpiochip=""
  for _ in {1..150}; do
    gpiochip="$(grep -o 'gpiochip[0-9]\+' "$logdir/checker.log" |
      tail -1 || true)"
    if [[ -n "$gpiochip" ]]; then
      break
    fi
    sleep 0.1
  done

  if [[ -z "$gpiochip" ]]; then
    echo "FAIL: gpiochip was not reported by checker"
    tail -40 "$logdir/checker.log"
    fail=$((fail + 1))
    cleanup_run
    continue
  fi

  setsid sudo "$BIN" \
    --uart "$ttydir/ttyA" \
    --gpiochip "$gpiochip" \
    --start-line 24 \
    --drop-line 23 \
    >"$logdir/student.log" 2>&1 &
  student_pid=$!

  wait "$checker_pid"
  checker_rc=$?
  checker_pid=""

  cleanup_run

  if ((checker_rc == 0)) &&
     grep -Fq 'REZULTAT: HIT' "$logdir/checker.log"; then
    echo "HIT test=$test_number run=$run_number"
    pass=$((pass + 1))
  else
    echo "FAIL test=$test_number run=$run_number rc=$checker_rc"
    tail -50 "$logdir/checker.log"
    fail=$((fail + 1))
  fi
done

trap - EXIT INT TERM
cleanup_run

echo "SUMMARY test=$test_number runs=$runs pass=$pass fail=$fail"

((fail == 0))
