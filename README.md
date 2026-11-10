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
pio test -e portenta_c33             # CAN tests on the board (or feather_m4_can); see below
pio device monitor                   # telemetry at 10 Hz: mode, pitch [rad], x [m], wheel torques [N·m], ODrive health
pio run -e portenta_c33 -t compiledb # compile_commands.json for clangd (one board at a time)
```

Neither core's GCC has C++20; `platformio.ini` swaps in GCC 12.3 for both. The Arduino IDE cannot build this tree.

## Before the first run

1. **ODrives:** torque control, CAN at 500 kbps, node 0 left and node 1 right. Each sends its encoder estimates every 10 ms and its heartbeat at least every 100 ms (the default). The firmware sets the controller mode, the limits and the axis state; it does not configure the cyclic messages. Set `can.config.baud_rate = 500000` explicitly: the default, 0, is autobaud, and the bot sends nothing until it hears a drive. With the watchdog on (`enable_watchdog`, `watchdog_timeout = 0.1`), a drive that stops getting torques idles its motor; the bot sends one every 10 ms, idle or not, so the drive's heartbeat reports `WATCHDOG_TIMER_EXPIRED` only until the bot is on the bus.
2. **Nameplate:** weigh and measure the bot, and put the numbers in `bot::Nameplate` ([include/balance_controller.hpp](include/balance_controller.hpp)). The values marked "measure" are estimates. Then rerun the SIL: it checks the design on the new nameplate and holds it to ±20 % with every reading a period late.
3. **Pitch trim:** with `kPitchTrim = 0`, hold the bot at its balance point and read `pitch` off the telemetry. That reading is `kPitchTrim`. The position integrator would absorb the error anyway, but the trim keeps the engage window (5°) centred and the lean honest.

## Behaviour

- Idle until held within 5° of upright for 2 s with both ODrives heard from, then it balances and holds the spot and the heading it was let go at.
- A push moves it; it comes back. A shove of more than 30 cm moves the hold point.
- Past 30° it has fallen: motors to idle, and it waits to be stood up again. An ODrive axis error, or 250 ms without a heartbeat from either drive, does the same; the axis-error output (D7, or D5 on the Feather) goes high on an axis error. If the IMU goes quiet for 1 s while it balances, it latches a fault until reset.
- Nothing is sent to an ODrive until it has been heard from, and a drive that appears gets a clear-errors first.
- Speed and yaw-rate commands go in `bot::Command`. Nothing feeds them yet: `main.cpp` passes zeros, so the bot stands still.

## CAN log

Each control tick goes out on the bus after the torques, for a logger such as the ODrive USB-CAN adapter: three standard frames at 100 Hz, little-endian, on IDs no ODrive answers to and behind the drives in priority (about 8 % of the 500 kbps bus). The wheels are in the drives' own encoder estimates. As with the drive commands, nothing goes out until a drive is heard: with no node to ACK them, the frames would fill the TX FIFO.

| ID | bytes 0–3 | bytes 4–7 |
| --- | --- | --- |
| `0x600` | pitch [rad], float32 | pitch rate [rad/s], float32 |
| `0x601` | torque left [N·m], float32 | torque right [N·m], float32: the controller's, before the left motor's mirror |
| `0x602` | x [m], float32 | mode, uint8 (0 idle, 1 active, 2 fault); flags, uint8 (bit 0 IMU fresh, bit 1 drives healthy); excitation [mN·m], int16 |

Set `kExcitation` in [src/main.cpp](src/main.cpp) above 0 (it ships at 0) and, while it balances, the bot adds a pseudo-random torque of that size (PRBS9, a new bit every 20 ms) to both wheels, already included in `0x601`'s torques: a known input, uncorrelated with the hand and the floor, to identify the plant in closed loop. The method and what it found are in [docs/derivation.md](docs/derivation.md#identification-on-the-bot).

## CAN on the C33

`lib/Arduino_CAN` is the core's library from the Renesas Portenta core 1.6.0, the latest release (`renesas-ra@1.9.0`, pinned in `platformio.ini`); the core's `main` has nothing newer for it. The core links Arduino's fork of FSP 4.0 as a prebuilt `libfsp.a`, which has no transmit FIFO support, so the TX patch works on the CANFD registers directly. Each library change is marked `BalanceBot patch` in [R7FA6M5_CAN.cpp](lib/Arduino_CAN/src/R7FA6M5_CAN.cpp). PlatformIO takes a library in `lib/` over the core's copy of the same name.

[r_canfd.c](lib/Arduino_CAN/src/r_canfd.c) is the FSP driver itself: `arduino/fsp`'s, as the core builds it, plus two fixes on their way upstream, [arduino/fsp#2](https://github.com/arduino/fsp/pull/2) (`R_CANFD_Write` overwrote a frame still pending in its buffer) and [arduino/fsp#3](https://github.com/arduino/fsp/pull/3) (the error handler never cleared the channel's error flags). The linker takes it in place of the `r_canfd.o` in `libfsp.a`; `lib_archive = no` makes that certain, since a link that pulled both would fail on duplicate symbols. Once the core ships a `libfsp.a` with both fixes, the file goes.

- **RX:** `available()` drains the channel's receive FIFO, 64 frames deep (the core sets 16), into the library's 32-frame ring. The stock driver read one frame per call from a single receive buffer, which the next frame overwrites.
- **TX:** `write()` puts the frame in a 32-frame transmit FIFO (common FIFO 0 of the channel, set up in `begin()`), sent in the order written, and returns at once, or returns `-FSP_ERR_CAN_TRANSMIT_NOT_READY` when the FIFO is full. The stock driver wrote transmit buffer 0, and FSP 4.0 lets a write overwrite a buffer whose frame has not gone out yet: its check only refuses a buffer while it is on the bus.
- **Error interrupts:** on again (warning, passive, bus-off, recovery, overload), handled by the fixed `canfd_error_isr` in `r_canfd.c`. The FSP fork handled that interrupt as a global error and never cleared the channel's flags, so it fired again on return, forever ([arduino/ArduinoCore-renesas#532](https://github.com/arduino/ArduinoCore-renesas/issues/532)); the core's answer was to turn them off. `isError()` now reports the latched `canfd_error_t` flags until `clearError()`. Bus-off recovers on its own, per ISO 11898-1.
- **Filters:** CAN0's standard-ID rule no longer delivers into CAN1's buffers ([#513](https://github.com/arduino/ArduinoCore-renesas/issues/513)).
- **Loopback:** `enableInternalLoopback()` loops back internally and leaves the channel running; the stock call asked for the external loopback and left the channel halted.

Buffer RAM: two 64-frame RX FIFOs and two 32-frame TX FIFOs take 3880 of the RA6M5's 4864 bytes.

## CAN on the Feather M4 CAN

`lib/Adafruit_CAN` is [Adafruit_CAN](https://github.com/adafruit/Adafruit_CAN) 0.2.3, the maintained successor of the archived CAN Adafruit Fork, with one change: the `same5x-tx-rx-fifos` branch of [Wetmelon/Adafruit_CAN](https://github.com/Wetmelon/Adafruit_CAN/tree/same5x-tx-rx-fifos), on its way upstream. The vendored copy is that branch, byte for byte; once Adafruit releases it, the registry version can take its place.

- **RX:** the receive FIFO is 64 frames, the M_CAN's maximum, up from 8: 16 ms of a saturated bus, against a 10 ms poll.
- **TX:** `endPacket()` puts the frame in a 32-frame transmit FIFO, sent in the order written, and returns at once, or returns 0 when the FIFO is full. The stock driver had one transmit buffer, which every write overwrote whether or not its last frame had gone out, and then spun up to 8000 polls waiting for it.

## CAN tests on the boards

On a board on USB; `pio test -e <board>` runs every test for it, `-f <test>` one of them.

- [test_can_loopback](test/test_can_loopback/test_main.cpp), both boards, any bus: the controller in loopback. A burst of writes stops at the full 32-frame TX FIFO without blocking, and 64 frames come back in order. The stock drivers fail it.
- [test_can_no_ack](test/test_can_no_ack/test_main.cpp), C33 only, **no other powered node on the bus**: frames nobody ACKs. A write to a TX buffer whose frame is still pending is refused (arduino/fsp#2), and the error interrupts raised as the transmit error count passes warning and passive return, reported by `isError()` (arduino/fsp#3). With the core's `libfsp.a` alone, the board hangs in the first of those interrupts.

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
| `lib/Arduino_CAN/` | the C33 core's CAN library, patched, and FSP's `r_canfd.c` with two fixes (above) |
| `lib/Adafruit_CAN/` | Adafruit_CAN for the Feather, patched (above) |
| `test/` | CAN tests on the boards (see "CAN tests on the boards") |
| `c33_upload.py` | C33 upload: a 1200 bps touch into the DFU bootloader first, since dfu-util's detach through the running sketch times out |
| `lib/BNO080/` | SparkFun BNO080 library, vendored |
