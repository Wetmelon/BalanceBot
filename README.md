# BalanceBot

A two-wheeled balancing robot: an Arduino Portenta C33 on a Portenta Hat Carrier, two ODrives driving hub motors in torque mode over CAN1, and a BNO080 IMU on Wire2. The control is [Damp](https://github.com/Wetmelon/damp): two LQIs designed at compile time from the bot's nameplate, run on ETL containers and Damp's own float math. The SIL runs the same controller object on the host against the nonlinear plant.

## Build

```bash
git clone --recurse-submodules=external/damp git@github.com:Wetmelon/BalanceBot.git
# or, in an existing clone: git submodule update --init   (not --recursive: Damp's own submodules are its test deps)

pio run -e sil -t exec               # host SIL: exits 0 when every check holds
pio run -e portenta_c33              # firmware
pio run -e portenta_c33 -t upload    # flash
pio device monitor                   # telemetry at 10 Hz: mode, pitch [rad], x [m], wheel torques [N·m], ODrive health
pio run -e portenta_c33 -t compiledb # compile_commands.json for clangd
```

The Renesas core's own GCC has no C++20; `platformio.ini` swaps in GCC 12.3. The Arduino IDE cannot build this tree.

## Before the first run

1. **ODrives:** torque control, CAN at 500 kbps, node 0 left and node 1 right. Each sends its encoder estimates every 10 ms and its heartbeat at least every 100 ms (the default). The firmware sets the controller mode, the limits and the axis state; it does not configure the cyclic messages.
2. **Nameplate:** weigh and measure the bot, and put the numbers in `bot::Nameplate` ([include/balance_controller.hpp](include/balance_controller.hpp)). The values marked "measure" are estimates. Then rerun the SIL: it checks the design on the new nameplate and holds it to ±20 % with every reading a period late.
3. **Pitch trim:** with `kPitchTrim = 0`, hold the bot at its balance point and read `pitch` off the telemetry. That reading is `kPitchTrim`. The position integrator would absorb the error anyway, but the trim keeps the engage window (5°) centred and the lean honest.

## Behaviour

- Idle until held within 5° of upright for 2 s with both ODrives heard from, then it balances and holds the spot and the heading it was let go at.
- A push moves it; it comes back. A shove of more than 30 cm moves the hold point.
- Past 30° it has fallen: motors to idle, and it waits to be stood up again. An ODrive axis error, or 250 ms without a heartbeat from either drive, does the same; D7 goes high on an axis error. If the IMU goes quiet for 1 s while it balances, it latches a fault until reset.
- Nothing is sent to an ODrive until it has been heard from, and a drive that appears gets a clear-errors first.
- Speed and yaw-rate commands go in `bot::Command`. Nothing feeds them yet: `main.cpp` passes zeros, so the bot stands still.

## CAN on the C33

`lib/Arduino_CAN` is the core's library (Renesas Portenta core 1.6.0) with three changes, each marked `BalanceBot patch` in the source. PlatformIO takes a library in `lib/` over the core's copy of the same name.

- **RX:** `available()` drains the channel's 16-frame receive FIFO. The stock driver read one frame per call from a single receive buffer, which the next frame overwrites, so two ODrive frames arriving between polls lost one.
- **TX:** `write()` takes whichever of the 16 transmit buffers is free. The stock driver always used buffer 0, so a frame written while the previous one was still on the bus was refused.
- **Filters:** CAN0's standard-ID rule no longer delivers into CAN1's buffers ([arduino/ArduinoCore-renesas#513](https://github.com/arduino/ArduinoCore-renesas/issues/513)).

Channel error interrupts stay off, as the core ships them: with them on, a bus error re-enters the interrupt forever ([#532](https://github.com/arduino/ArduinoCore-renesas/issues/532)). The heartbeat timeout is what notices a drive dropping off.

## Files

| File | |
| --- | --- |
| [include/balance_controller.hpp](include/balance_controller.hpp) | nameplate, plant, design, the tick (`bot::Controller`): no Arduino |
| [src/main.cpp](src/main.cpp) | FreeRTOS tasks: control (CAN in, tick, CAN out) at 100 Hz, IMU at 1 kHz, telemetry |
| [include/bot_can.hpp](include/bot_can.hpp), `can_*.hpp`, `ODriveEnums.h` | ODrive CAN simple protocol, heartbeat timeouts |
| [include/imu.hpp](include/imu.hpp) | BNO080: game rotation vector and raw gyro at 100 Hz |
| [sil/balance_sil.cpp](sil/balance_sil.cpp) | nonlinear plant, scripted scenarios, robustness check |
| [docs/derivation.md](docs/derivation.md) | equations of motion, measurement map, design |
| [include/damp_profile.hpp](include/damp_profile.hpp) | Damp backend: ETL, Damp math |
| `lib/Arduino_CAN/` | the core's CAN library, patched (above) |
| `lib/BNO080/` | SparkFun BNO080 library, vendored |
