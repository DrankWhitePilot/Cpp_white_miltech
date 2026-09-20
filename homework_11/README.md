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

The inherited Homework 10 math files are unchanged.

## Build

Ubuntu packages:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build libgpiod-dev gpiod socat kmod
```

Configure and build:

```bash
cd homework_11
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target mission_uart_drop -j"$(nproc)"
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

The live checker is supplied separately and requires Linux with the `gpio-sim`
kernel module. Run it at the official time scale `1` against the built
`mission_uart_drop` executable.

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
