# Homework 18: UART + I2C device

ESP32-S3 firmware for Wokwi. The device reads acceleration from an MPU-6050
over I2C and reports one stable status line over UART at a timer-controlled
period.

## Interfaces

- UART0: 115200 8N1, Wokwi Serial Monitor.
- I2C0: MPU-6050, SDA GPIO 8, SCL GPIO 9, address `0x68`.

## UART protocol

Periodic report:

```text
t=12345 ms ax=0.02 ay=-0.01 az=1.00 mode=periodic
```

Change the measurement period while the simulator is running:

```text
p 100
```

The accepted range is 100 to 5000 ms. A valid command is acknowledged with:

```text
ok period=100 ms
```

## Build and run

```bash
. /home/user/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

Open this directory in VS Code with the Wokwi extension and run
`Wokwi: Start Simulator`.
