# BalanceBot

A two-wheeled balancing robot: Arduino MKR WiFi 1010 with a MKR CAN shield, two ODrives driving hub motors in torque mode, and a BNO080 IMU. The control is [Damp](https://github.com/Wetmelon/damp): two LQIs designed at compile time from the bot's nameplate, run on ETL containers and Damp's own float math. The SIL runs the same controller object on the host against the nonlinear plant.

## Build

```bash
git clone --recurse-submodules=external/damp git@github.com:Wetmelon/BalanceBot.git
# or, in an existing clone: git submodule update --init   (not --recursive: Damp's own submodules are its test deps)

pio run -e sil -t exec              # host SIL: exits 0 when every check holds
pio run -e mkrwifi1010              # firmware
pio run -e mkrwifi1010 -t upload    # flash
pio device monitor                  # telemetry at 10 Hz: mode, pitch [rad], x [m], wheel torques [N·m]
```

The SAMD core ships GCC 7.2, which has no C++20; `platformio.ini` swaps in GCC 12.3. The Arduino IDE cannot build this tree.

## Before the first run

1. **ODrives:** torque control, CAN at 500 kbps, node 0 left and node 1 right, the encoder estimates and the heartbeat sent cyclically at 100 Hz. The firmware sets the controller mode, the limits and the axis state; it does not configure the cyclic messages.
2. **Nameplate:** weigh and measure the bot, and put the numbers in `bot::Nameplate` ([src/balance_controller.hpp](src/balance_controller.hpp)). The values marked "measure" are estimates. Then rerun the SIL: it checks the design on the new nameplate and holds it to ±20 % with every reading a period late.
3. **Pitch trim:** with `kPitchTrim = 0`, hold the bot at its balance point and read `pitch` off the telemetry. That reading is `kPitchTrim`. The position integrator would absorb the error anyway, but the trim keeps the engage window (5°) centred and the lean honest.

## Behaviour

- Idle until held within 5° of upright for 2 s, then it balances and holds the spot and the heading it was let go at.
- A push moves it; it comes back. A shove of more than 30 cm moves the hold point.
- Past 30° it has fallen: motors to idle, and it waits to be stood up again. An ODrive axis error does the same. If the IMU goes quiet for 1 s while it balances, it latches a fault until reset.
- Speed and yaw-rate commands go in `bot::Command`. Nothing feeds them yet: `main.cpp` passes zeros, so the bot stands still.

## Files

| File | |
| --- | --- |
| [src/balance_controller.hpp](src/balance_controller.hpp) | nameplate, plant, design, the tick (`bot::Controller`): no Arduino |
| [src/main.cpp](src/main.cpp) | FreeRTOS tasks: control (CAN in, tick, CAN out) at 100 Hz, IMU at 1 kHz, telemetry |
| [src/bot_can.hpp](src/bot_can.hpp), `can_*.hpp`, `ODriveEnums.h` | ODrive CAN simple protocol |
| [src/imu.hpp](src/imu.hpp) | BNO080: game rotation vector and raw gyro at 100 Hz |
| [sil/balance_sil.cpp](sil/balance_sil.cpp) | nonlinear plant, scripted scenarios, robustness check |
| [docs/derivation.md](docs/derivation.md) | equations of motion, measurement map, design |
| [include/damp_profile.hpp](include/damp_profile.hpp) | Damp backend: ETL, Damp math |
| `lib/BNO080/` | SparkFun BNO080 library, vendored |
