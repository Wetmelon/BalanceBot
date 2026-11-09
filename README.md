# BalanceBot

A two-wheeled balancing robot: two ODrives driving hub motors in torque mode over CAN, and a BNO080 IMU on I²C. It builds for two boards side by side, from the same sources:

| | Portenta C33 (on a Portenta Hat Carrier) | Adafruit Feather M4 CAN |
| --- | --- | --- |
| env | `portenta_c33` | `feather_m4_can` |
| CAN | CAN1, the carrier's transceiver | CAN0, the on-board TCAN1051 (5 V boost on) |
| BNO080 | `Wire2` | `Wire` (SDA/SCL, STEMMA QT) |
| axis-error output | D7 | D5 |
| 1 Hz blink | blue LED | red LED (D13) |
| FreeRTOS | the core's `Arduino_FreeRTOS` | `FreeRTOS_SAMD51` 1.3.0 |

The differences live in [include/board.hpp](include/board.hpp) and the driver calls at the bottom of [include/bot_can.hpp](include/bot_can.hpp). The control is [Damp](https://github.com/Wetmelon/damp): two LQIs designed at compile time from the bot's nameplate, run on ETL containers and Damp's own float math. The SIL runs the same controller object on the host against the nonlinear plant.

## Build

```bash
git clone --recurse-submodules=external/damp git@github.com:Wetmelon/BalanceBot.git
# or, in an existing clone: git submodule update --init   (not --recursive: Damp's own submodules are its test deps)

pio run                              # both boards and the SIL
pio run -e sil -t exec               # host SIL: exits 0 when every check holds
pio run -e feather_m4_can -t upload  # flash one board (or portenta_c33)
pio device monitor                   # telemetry at 10 Hz: mode, pitch [rad], x [m], wheel torques [N·m], ODrive health
pio run -e portenta_c33 -t compiledb # compile_commands.json for clangd (one board at a time)
```

Neither core's GCC has C++20; `platformio.ini` swaps in GCC 12.3 for both. The Arduino IDE cannot build this tree.

## Before the first run

1. **ODrives:** torque control, CAN at 500 kbps, node 0 left and node 1 right. Each sends its encoder estimates every 10 ms and its heartbeat at least every 100 ms (the default). The firmware sets the controller mode, the limits and the axis state; it does not configure the cyclic messages.
2. **Nameplate:** weigh and measure the bot, and put the numbers in `bot::Nameplate` ([include/balance_controller.hpp](include/balance_controller.hpp)). The values marked "measure" are estimates. Then rerun the SIL: it checks the design on the new nameplate and holds it to ±20 % with every reading a period late.
3. **Pitch trim:** with `kPitchTrim = 0`, hold the bot at its balance point and read `pitch` off the telemetry. That reading is `kPitchTrim`. The position integrator would absorb the error anyway, but the trim keeps the engage window (5°) centred and the lean honest.

## Behaviour

- Idle until held within 5° of upright for 2 s with both ODrives heard from, then it balances and holds the spot and the heading it was let go at.
- A push moves it; it comes back. A shove of more than 30 cm moves the hold point.
- Past 30° it has fallen: motors to idle, and it waits to be stood up again. An ODrive axis error, or 250 ms without a heartbeat from either drive, does the same; the axis-error output (D7, or D5 on the Feather) goes high on an axis error. If the IMU goes quiet for 1 s while it balances, it latches a fault until reset.
- Nothing is sent to an ODrive until it has been heard from, and a drive that appears gets a clear-errors first.
- Speed and yaw-rate commands go in `bot::Command`. Nothing feeds them yet: `main.cpp` passes zeros, so the bot stands still.

## CAN on the C33

`lib/Arduino_CAN` is the core's library (Renesas Portenta core 1.6.0) with three changes, each marked `BalanceBot patch` in the source. PlatformIO takes a library in `lib/` over the core's copy of the same name.

- **RX:** `available()` drains the channel's 16-frame receive FIFO. The stock driver read one frame per call from a single receive buffer, which the next frame overwrites, so two ODrive frames arriving between polls lost one.
- **TX:** `write()` takes whichever of the 16 transmit buffers is free. The stock driver always used buffer 0, so a frame written while the previous one was still on the bus was refused.
- **Filters:** CAN0's standard-ID rule no longer delivers into CAN1's buffers ([arduino/ArduinoCore-renesas#513](https://github.com/arduino/ArduinoCore-renesas/issues/513)).

Channel error interrupts stay off, as the core ships them: with them on, a bus error re-enters the interrupt forever ([#532](https://github.com/arduino/ArduinoCore-renesas/issues/532)). The heartbeat timeout is what notices a drive dropping off.

## CAN on the Feather M4 CAN

Adafruit's CAN fork (1.2.1), unpatched. It polls an 8-frame receive FIFO, which holds a 10 ms tick's ODrive traffic with room to spare. Transmit is one buffer: each write waits, spinning, until its frame is on the bus (about 0.25 ms at 500 kbps, longer if it loses arbitration), and gives up after 8000 polls (8 ms by the source's comment, nearer 1 ms at 120 MHz); the next write then overwrites a frame that may still be pending. The control task pays that wait twice per tick, six times on a mode change. `endPacket()` reports success either way, so a refused frame is never printed here.

## Files

| File | |
| --- | --- |
| [include/balance_controller.hpp](include/balance_controller.hpp) | nameplate, plant, design, the tick (`bot::Controller`): no Arduino |
| [src/main.cpp](src/main.cpp) | FreeRTOS tasks: control (CAN in, tick, CAN out) at 100 Hz, IMU at 1 kHz, telemetry |
| [include/board.hpp](include/board.hpp) | per-board pins, I²C port, LEDs, FreeRTOS header |
| [include/bot_can.hpp](include/bot_can.hpp), `can_*.hpp`, `ODriveEnums.h` | ODrive CAN simple protocol, heartbeat timeouts, per-board CAN driver calls |
| [include/imu.hpp](include/imu.hpp) | BNO080: game rotation vector and raw gyro at 100 Hz |
| [sil/balance_sil.cpp](sil/balance_sil.cpp) | nonlinear plant, scripted scenarios, robustness check |
| [docs/derivation.md](docs/derivation.md) | equations of motion, measurement map, design |
| [include/damp_profile.hpp](include/damp_profile.hpp) | Damp backend: ETL, Damp math |
| `lib/Arduino_CAN/` | the C33 core's CAN library, patched (above) |
| `lib/BNO080/` | SparkFun BNO080 library, vendored |
