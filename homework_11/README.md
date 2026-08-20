# HW11 — UART/GPIO mission control

The program receives mission data through UART, chooses an attack plan, sends
normalized drone controls and produces one GPIO DROP pulse.

## Architecture

- `MissionProcessor` owns target history, ballistics, target selection,
  guidance and the one-shot DROP decision.
- `DroneController` converts the mission destination and desired speed into
  UART `accel` and `turnRate` values in `[-1, 1]`.
- `UartLink` owns the raw 115200 8N1 nonblocking transport and packet parser.
- `GpioController` owns START and DROP GPIO lines.

The inherited Homework 10 math files are unchanged. Their exact comparison is
performed by `tools/check_math_block.sh`.

## Build

Ubuntu packages:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build libgpiod-dev gpiod socat kmod
```

Configure and build:

```bash
cd HW11_HW11
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target mission_uart_drop -j"$(nproc)"
```

## Checker

The live checker requires Linux with the `gpio-sim` kernel module. The official
time scale is the default value `1`.

```bash
./tools/run_checker_one.sh 10 3
./tools/run_checker_all.sh
```

An accelerated diagnostic run can be requested explicitly:

```bash
HW11_SCALE=50 ./tools/run_checker_one.sh 10 1
```

The scripts use only processes they started, keep every run in a unique
`test-results/` directory and return a nonzero exit code on any failure.

## Static checks

```bash
./tools/check_math_block.sh
./tools/check_no_hacks.sh
./tools/check_teacher_math_10.sh
g++ -std=c++20 -Wall -Wextra -Werror -pedantic \
  -Iinclude tests/drone_controller_test.cpp \
  src/drone_controller.cpp src/model_math.cpp src/types.cpp \
  -o /tmp/drone_controller_test && /tmp/drone_controller_test
g++ -std=c++20 -Wall -Wextra -Werror -pedantic \
  -Iinclude tests/uart_link_test.cpp src/uart_link.cpp \
  -lutil -o /tmp/uart_link_test && /tmp/uart_link_test
```

`check_teacher_math_10.sh` validates the supplied checker's reference
autopilot; it does not replace the live UART/GPIO run.

## Direct launch

Simulator example:

```bash
sudo ./build/mission_uart_drop \
  --uart /tmp/ttyA \
  --gpiochip gpiochipN \
  --start-line 24 \
  --drop-line 23
```

Replace `gpiochipN` with the chip name printed by the checker.

Raspberry Pi example:

```bash
sudo ./build/mission_uart_drop \
  --uart /dev/ttyAMA1 \
  --gpiochip gpiochip0 \
  --start-line 24 \
  --drop-line 23
```

START is raised immediately after UART/GPIO initialization and remains high
for the mission. DROP is a single nominal 100 ms pulse.
