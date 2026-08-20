# Validation status

Baseline archive:

- `HW11_AUDIT_CURRENT_20260820_203626.tar.gz`
- SHA-256: `d2f3bc365ea2f1d8588aa918d091697fd0e9cd2c38c6e778a019136337731b5f`

## Completed in the isolated copy

- Mission planning and low-level drone control are separate modules.
- Homework 10 math block is byte-identical to the bundled reference.
- Official `drone_link.h` is unchanged.
- UART fragmented-packet, CRC recovery, nonblocking read and CONTROL tests pass.
- Drone control and actual-speed DROP-window unit tests pass.
- Full source compiles with warnings-as-errors against both libgpiod v1 and v2
  API stubs.
- Supplied checker selftest reports HIT for tests 1–10.
- Anti-hack scan passes.

## Required on the target Ubuntu machine

The current refactored source still requires a clean build and live GPIO/UART
checker run with the real `libgpiod` and `gpio-sim`:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target mission_uart_drop -j"$(nproc)"
ctest --test-dir build --output-on-failure
./tools/run_checker_one.sh 10 3
./tools/run_checker_all.sh
```

Do not mark the project ready for submission until these live checks pass.
