# Plant and design

The code is [src/balance_controller.hpp](../src/balance_controller.hpp). The SIL [sil/balance_sil.cpp](../sil/balance_sil.cpp) integrates the nonlinear equations below and closes the loop around the same `bot::Controller` the firmware runs.

## Coordinates

Pitch plane: `p` is the axle's forward travel, `θ` the body's lean from vertical (forward positive), `τ = τ_R + τ_L` the sum of the motor torques on the wheels. The motor's reaction is `−τ` on the body. Yaw: `ψ` counter-clockwise from above, `Δτ = τ_R − τ_L`.

| Symbol | Nameplate field | Meaning |
| --- | --- | --- |
| r | `wheel_radius` | wheel radius |
| d | `track_width` | distance between the tire contact patches |
| l | `com_height − wheel_radius` | body CoM above the axle |
| M, J_b | `body_mass`, `body_inertia` | everything that pitches; its inertia about its own CoM |
| m_w, J_w | `wheel_mass`, `wheel_inertia` | per side: everything on the axle; the rotating part about the axle |
| J_z | `yaw_inertia` | whole bot about the vertical, wheels not spinning |

A hub motor's stator rides on the axle but does not spin, so it counts in `m_w` and not in `J_w`.

## Pitch plane

The wheels roll without slipping, so a wheel turns `p/r` against the ground and `p/r − θ` against the body. The motor torque does work through that relative angle, so the generalized forces on `(p, θ)` are `(τ/r, −τ)`. With

- `a = M + 2(m_w + J_w/r²)`, the translating mass with the wheel inertia reflected through r,
- `b = M l`,
- `c = J_b + M l²`, the body's pitch inertia about the axle,

the kinetic energy is `T = ½ a ṗ² + b cos θ ṗ θ̇ + ½ c θ̇²` and the potential `V = b g cos θ`. Lagrange's equations give

```text
a p̈ + b cos θ θ̈ − b sin θ θ̇² = τ / r
b cos θ p̈ + c θ̈ − b g sin θ  = −τ
```

The SIL solves this 2×2 system at every RK4 stage, with viscous friction between each wheel and the body subtracted from its motor torque.

Linearized about upright (`sin θ ≈ θ`, `cos θ ≈ 1`, `θ̇² ≈ 0`), with `D = ac − b²`:

```text
p̈ = (−b² g θ + (c/r + b) τ) / D
θ̈ = ( a b g θ − (a + b/r) τ) / D
```

That is `pitch_plant()`: `x = [p, v, θ, ω]`, `u = τ`, `y = p`. On the nameplate the fall is unstable at `√(abg/D) = 8.4 rad/s`. Torque to position has zeros at `±√(bg/(c + br)) = ±6.2 rad/s`: to roll forward the wheels first roll back to lean the body over. That right-half-plane zero caps how fast the position loop can be.

## Yaw

Each wheel pushes the ground with `τ_i / r` at `±d/2` from the centre line, so the yaw moment is `(d / 2r) Δτ`. Turning also spins the wheels at `±ψ̇ d / 2r`, which adds `J_w d² / 2r²` to the inertia (`yaw_inertia()`):

```text
J_z,eff ψ̈ = (d / 2r) Δτ
```

`yaw_plant()`: `x = [ψ, ψ̇]`, `u = Δτ`, `y = ψ`. Pitch and yaw are decoupled; the coupling from turning while driving is left out.

## Measurement

The ODrive encoder reads the rotor against the stator, which is the wheel against the body. With `φ_R`, `φ_L` the two encoder angles (forward positive, the left motor's sign flipped in `main.cpp`):

```text
p = r ( (φ_R + φ_L)/2 + θ )      v  = r ( (φ̇_R + φ̇_L)/2 + ω )
ψ = r (φ_R − φ_L) / d            ψ̇ = r (φ̇_R − φ̇_L) / d
```

θ cancels from the heading. The IMU is mounted on its side: its x axis is the pitch axis and the game rotation vector reads a roll of +90° upright, so `θ = roll − 90° − kPitchTrim` and `ω` is its gyro x. Heading comes from odometry, not the IMU, so it does not depend on the IMU's mounting.

## Design

Each axis is a `design::lqi` on the continuous plant sampled at `Ts = 10 ms`: ZOH for the dynamics, Van Loan for the cost, so the weights are in continuous time and keep their meaning if the rate changes. The integrator is on the measured output: position for pitch, heading for yaw. It removes the steady error that a CoM offset, an IMU mount error, a slope or friction would otherwise leave. The weights are Bryson's, `1 / (acceptable excursion)²`, and the comments next to them give the excursions.

| Pitch closed loop | Pole |
| --- | --- |
| lean | 21 rad/s pair, ζ 0.78 |
| position | 2.5 rad/s pair, ζ 0.76 |
| integral | 0.7 rad/s |

Faster lean poles balance more stiffly but lose margin: the robustness check below fails with the lean at 29 rad/s.

## The tick

```mermaid
flowchart LR
    CAN[CAN in: encoder estimates, heartbeats] --> M[Measurement]
    IMU[IMU task: last report] --> M
    M --> S{Supervisor}
    S -- Idle / Fault --> Z[zero torque]
    S -- Active --> R[references: p_ref, ψ_ref from the filtered commands, leashed]
    R --> L[update → control → mix → back_calculate → predict]
    L --> OUT[CAN out: torques]
    Z --> OUT
```

- **Supervisor:** Idle engages once the bot has been held within 5° for 2 s with fresh IMU data and both drives healthy. Active drops to Idle past 30° or on an axis error, and to Fault (latched) when the IMU goes quiet.
- **References:** on engage they start at the measured pose and the integrators clear, so the bot holds the spot it was let go at. The speed and yaw-rate commands pass a 0.5 Hz low-pass and integrate into `p_ref` and `ψ_ref`. A push drags each reference at most `kLeash` behind the bot, so a big shove moves the hold point instead of winding up a large catch-up.
- **Mixer:** pitch takes up to `2 · kWheelTorqueMax`, and steering gets what that leaves on each wheel. The torques actually applied go back to each law (`back_calculate`) so the integrators do not wind up against the clamp.

## Robustness check

The SIL runs its script (hold, engage, release at 2°, push, drive, turn, tip over) twice: on the nameplate, and on a plant 20 % heavier and taller with 20 % lighter wheels, a 2° IMU mount error and every reading one period late. Then it linearizes the pitch loop with the readings one period late, `z = [x; ξ; x_prev]`, on all 16 corners of ±20 % on body mass, body inertia, CoM height and wheel inertia, and checks every closed-loop pole is inside the unit circle. On the current design the largest is 0.993.

After measuring the real bot, put the numbers in `Nameplate` and rerun the SIL. If a check fails, slow the lean (raise `R`) before anything else.
