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
pio test -e feather_m4_can           # the CAN driver's FIFOs in loopback, on a board with no bus (or portenta_c33)
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

`lib/Arduino_CAN` is the core's library from the Renesas Portenta core 1.6.0, the latest release (`renesas-ra@1.9.0`, pinned in `platformio.ini`); the core's `main` has nothing newer for it. The core compiles in Arduino's fork of FSP 4.0, which has no transmit FIFO support and a broken error handler, so the patches work on the CANFD registers where FSP falls short. Each is marked `BalanceBot patch` in [R7FA6M5_CAN.cpp](lib/Arduino_CAN/src/R7FA6M5_CAN.cpp). PlatformIO takes a library in `lib/` over the core's copy of the same name.

- **RX:** `available()` drains the channel's receive FIFO, 64 frames deep (the core sets 16), into the library's 32-frame ring. The stock driver read one frame per call from a single receive buffer, which the next frame overwrites.
- **TX:** `write()` puts the frame in a 32-frame transmit FIFO (common FIFO 0 of the channel, set up in `begin()`), sent in the order written, and returns at once, or returns `-FSP_ERR_CAN_TRANSMIT_NOT_READY` when the FIFO is full. The stock driver wrote transmit buffer 0, and FSP 4.0 lets a write overwrite a buffer whose frame has not gone out yet: its check only refuses a buffer while it is on the bus.
- **Error interrupts:** on again (warning, passive, bus-off, recovery, overload), with a handler of our own on the channel's error vector. The FSP fork handled that interrupt as a global error and never cleared the channel's flags, so it fired again on return, forever ([arduino/ArduinoCore-renesas#532](https://github.com/arduino/ArduinoCore-renesas/issues/532)); the core's answer was to turn them off. `isError()` now reports the latched `canfd_error_t` flags until `clearError()`. Bus-off recovers on its own, per ISO 11898-1.
- **Filters:** CAN0's standard-ID rule no longer delivers into CAN1's buffers ([#513](https://github.com/arduino/ArduinoCore-renesas/issues/513)).
- **Loopback:** `enableInternalLoopback()` loops back internally and leaves the channel running; the stock call asked for the external loopback and left the channel halted.

Buffer RAM: two 64-frame RX FIFOs and two 32-frame TX FIFOs take 3880 of the RA6M5's 4864 bytes.

## CAN on the Feather M4 CAN

`lib/CAN_Adafruit_Fork` is Adafruit's CAN fork (1.2.1, the latest) with two changes, each marked `BalanceBot patch` in [CANSAME5x.cpp](lib/CAN_Adafruit_Fork/src/CANSAME5x.cpp). It takes over from the registry copy, which is no longer a dependency.

- **RX:** the receive FIFO is 64 frames, the M_CAN's maximum, up from 8: 16 ms of a saturated bus, against a 10 ms poll.
- **TX:** `endPacket()` puts the frame in a 32-frame transmit FIFO, sent in the order written, and returns at once, or returns 0 when the FIFO is full. The stock driver had one transmit buffer, which every write overwrote whether or not its last frame had gone out, and then spun up to 8000 polls waiting for it.

## Loopback test

`pio test -e portenta_c33` or `pio test -e feather_m4_can` runs [test/test_can_loopback](test/test_can_loopback/test_main.cpp) on the board in its controller's loopback mode, with no bus: a burst of writes stops at the full 32-frame TX FIFO without blocking, and 64 frames come back in order. The stock drivers fail it.

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
| `lib/CAN_Adafruit_Fork/` | Adafruit's CAN library for the Feather, patched (above) |
| [test/test_can_loopback/](test/test_can_loopback/test_main.cpp) | each board's CAN FIFOs, on the board |
| `lib/BNO080/` | SparkFun BNO080 library, vendored |
