# Claude Code Project Instructions

This is an STM32G431 + GM3506 + AS5048A + MPU6050 single-axis FOC self-stabilizing gimbal project for the 2026 embedded competition ST track.

Before doing project work, read these files first:

- `AGENTS.md`
- `PROJECT_STATUS.md`
- `TODO_NEXT.md`
- `DECISIONS.md`
- `DEBUG_LOG.md`
- `.agent/skills/stm32-foc-gimbal-debug/SKILL.md` when working on motor control, telemetry, IMU, tuning, or gimbal behavior

Do not rely on chat history as project memory. Project state must be read from and written back to markdown files in the project root.

## Safety Rules

Never suggest enabling stabilize mode `G` directly after flashing.

Use this safe bring-up order:

```text
power on without e -> e -> d -> e -> S30 -> S-30 -> S60 -> S-60 -> S0 -> H -> P10 -> P0 -> P-10 -> IMU read-only -> GLIM5 -> H -> G
```

Before `G`, confirm:

- small-angle position loop is stable
- `IMU_RDY=1`
- IMU pitch is stable
- `H` has recorded encoder zero and IMU home offset
- `STAB_LIM` is small, preferably 5 degrees

## Update Rules

After meaningful work:

- update `DEBUG_LOG.md` after hardware tests or telemetry analysis
- update `PROJECT_STATUS.md` after project state changes
- update `TODO_NEXT.md` when the next task changes
- update `DECISIONS.md` for permanent technical decisions

## Hardware Facts

- AS5048A is mounted at the motor bottom and measures motor angle.
- MPU6050 is mounted on the fixed base/stator/bracket side and measures base tilt.
- MPU6050 must not be mounted on the rotor arm, shaft, payload, or any rotating part.
- Current MPU6050 driver uses software I2C bit-bang on `PB5=SCL`, `PB4=SDA`; it does not require CubeMX hardware I2C.
- Rotor arm attaches to the rotating output side.

## Control Goal

The self-stabilizing gimbal target is:

```text
theta_base + phi_motor = 0
phi_target = -IMU_SIGN * (imu_pitch - imu_home_offset)
```

Control chain:

```text
position PD -> speed PI -> current PI -> PWM
```
