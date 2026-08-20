#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REFERENCE="$ROOT/tests/reference/homework_10"

files=(
  include/types.hpp
  src/types.cpp
  include/model_math.hpp
  src/model_math.cpp
  include/ballistic_table.hpp
  src/ballistic_table.cpp
  include/table_solver.hpp
  src/table_solver.cpp
  include/drone_state.hpp
  src/drone_state.cpp
  data/ballistic_table.txt
)

for file in "${files[@]}"; do
  if ! cmp -s "$REFERENCE/$file" "$ROOT/$file"; then
    echo "MATH_FILE_COMPARE_FAIL: $file"
    diff -u "$REFERENCE/$file" "$ROOT/$file" | sed -n '1,120p'
    exit 1
  fi
done

temporary_dir="$(mktemp -d /tmp/hw11_math_check_XXXXXX)"
cleanup() {
  rm -rf "$temporary_dir"
}
trap cleanup EXIT INT TERM

g++ -std=c++20 -Wall -Wextra -Werror -pedantic \
  -I"$ROOT/include" \
  "$ROOT/src/types.cpp" \
  "$ROOT/src/model_math.cpp" \
  "$ROOT/src/ballistic_table.cpp" \
  "$ROOT/src/table_solver.cpp" \
  "$ROOT/src/drone_state.cpp" \
  "$ROOT/tools/math_probe.cpp" \
  -o "$temporary_dir/math_probe"

"$temporary_dir/math_probe" "$ROOT/data/ballistic_table.txt"

echo "MATH_FILE_COMPARE_PASS"
echo "MATH_BLOCK_PASS"
