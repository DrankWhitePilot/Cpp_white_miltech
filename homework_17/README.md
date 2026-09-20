# Домашнє завдання 17 — MAVLink 2 через UDP

Програма розширює симулятор із ДЗ11, не змінюючи його математичну модель.
Вона читає фізичну телеметрію через UART, надсилає керування назад у симулятор
і паралельно публікує MAVLink 2 у QGroundControl або курсовий чекер.

## Підготовка

Після клонування репозиторію потрібно отримати MAVLink-підмодуль:

```bash
git submodule update --init --depth 1 c_library_v2
```

## Збірка і тести

```bash
cmake -S . -B build_hw17 -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build_hw17 -j2
ctest --test-dir build_hw17 --output-on-failure
```

## Запуск

QGroundControl за замовчуванням слухає `127.0.0.1:14550`:

```bash
./build_hw17/homework_17/mission_mavlink_udp \
  --uart /tmp/ttyA \
  --gpiochip gpiochip0 \
  --start-line 24 \
  --drop-line 23
```

Інший одержувач задається параметрами:

```bash
--mavlink-address 127.0.0.1 --mavlink-port 14550
```

Якщо програма працює у Docker, а QGroundControl запущений у Windows,
використовуйте адресу хоста Docker:

```bash
--mavlink-address host.docker.internal --mavlink-port 14550
```

У Docker всередині WSL цей DNS-аліас може бути відсутній. Тоді адресу Windows
можна подивитися у WSL командою `ip route` (адреса після `default via`) і
передати її через `--mavlink-address`. У QGroundControl також має бути
увімкнено `Application Settings -> Comm Links -> AutoConnect -> UDP`.

Програма надсилає `HEARTBEAT` приблизно раз на секунду, а
`GLOBAL_POSITION_INT` і `ATTITUDE` — на кожному кроці отриманої фізичної
телеметрії. Під час скиду вона надсилає `MAV_CMD_USER_1`, очікує
`COMMAND_ACK`, повторює команду з таймаутом до п'яти разів і припиняє повтори
після `MAV_RESULT_ACCEPTED`.
