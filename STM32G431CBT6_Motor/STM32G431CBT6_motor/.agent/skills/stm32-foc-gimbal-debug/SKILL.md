---
name: stm32-foc-gimbal-debug
description: Use this skill when working on the STM32G431 single-axis FOC self-stabilizing gimbal project, including GM3506 motor bring-up, current loop, speed loop, position loop, MPU6050 integration, AS5048A encoder feedback, stabilize mode, UART commands, telemetry logs, safe tuning, and competition demo preparation.
---

# STM32 FOC Single-Axis Gimbal Debug Skill

## Project Context

This project is a 2026 embedded competition STM32/ST track single-axis self-stabilizing gimbal.

Hardware:

- MCU: STM32G431
- Motor: GM3506 gimbal motor
- Encoder: AS5048A absolute magnetic encoder, mounted at motor bottom
- IMU: MPU6050, mounted on the fixed base/stator/bracket side
- Driver: custom/self-written FOC, not MCSDK

Control goal:

- MPU6050 measures base tilt `theta`.
- AS5048A measures motor/load angle `phi`.
- Payload should remain level in world frame.
- Desired relationship: `theta + phi = 0`.
- Stabilize target: `phi_target = -IMU_SIGN * (imu_pitch - imu_home_offset)`.

Control chain:

```text
MPU6050 pitch -> complementary filter -> stabilize target
AS5048A angle -> position feedback

position PD -> speed reference deg/s -> speed PI -> current PI -> PWM
```

Use position PD, not PI, because AS5048A is absolute and has no accumulated position drift. D term provides damping.

## Control Modes

```text
CTRL_SPEED      speed control, target from S<dps> or legacy ]/[ commands
CTRL_POSITION   position hold, target from P<deg>
CTRL_STABILIZE  self-stabilize, target from IMU pitch compensation
```

## UART Commands

```text
e          enable motor
d          disable motor
H          home encoder and IMU pitch offset; use only when payload is mechanically level
P<deg>     position target, examples P0, P10, P30, P-30
S<dps>     speed target, examples S30, S-30, S0
KP<val>    set position P gain
KD<val>    set position D gain
PLIM<val>  set position-loop speed limit in deg/s
DZ<val>    set position deadband in degrees
G          enter stabilize mode
GLIM<val>  set stabilize target limit in degrees, start with GLIM5
STEP<deg>  position step response test
HTEST      hold/disturbance recovery test
```

Command parsing rule: full-string commands such as `HTEST` must be checked before single-character `H`.

## Safe Bring-Up Order

Never recommend enabling stabilize mode directly after flashing.

Use this order:

```text
1. Power on, do not send e. Confirm no overcurrent, no self-rotation.
2. Send e. Confirm ST=RUN, IqRef near 0, motor not spinning.
3. Send d. Confirm motor disables correctly.
4. Speed loop: S30, S-30, S60, S-60, then S0.
5. Position loop small angle: H, P10, P0, P-10, P0.
6. IMU read-only: confirm IMU_RDY=1 and IMU pitch is stable.
7. Stabilize small angle only: GLIM5, H, G.
8. If direction is correct and stable, gradually test GLIM10, then GLIM20/30.
```

Do not start with `P30/P-30` repeated tests or large `GLIM` until small-angle tests are stable.

## Conservative Parameters

Safe starting values:

```text
KP_POS = 1.0
KD_POS = 0.02
PLIM = 30 to 45 deg/s
DZ = 0.5 deg
GLIM = 5 to 10 deg
```

Original planning values such as `KP_POS=3.0`, `KD_POS=0.05`, and `SPD_LIM=180 deg/s` are only planning references. For real bring-up, start conservative.

## Position Loop Logic

Position actual should be relative to home:

```c
pos_actual = MOTOR_SIGN * wrap180(enc_raw - pos_home_offset);
```

For position mode:

```c
target = clamp(pos_target_deg, -90, +90);
pos_err = target - pos_actual;
spd_cmd = kp_pos * pos_err - kd_pos * vel_pos;
spd_ref = clamp(spd_cmd, -pos_limit_spd, +pos_limit_spd);
```

For stabilize mode:

```c
imu_rel = imu_pitch - imu_home_offset;
target = clamp(-IMU_SIGN * imu_rel, -stab_lim_deg, +stab_lim_deg);
pos_err = target - pos_actual;
```

Use filtered velocity for D term, not raw instantaneous velocity.

## Safety Protections

Required protections:

- Target soft limit around +/-90 degrees.
- Actual angle warning above about +/-100 degrees.
- Hard protection above about +/-110 degrees.
- Wrap-crossing protection near +/-170 degrees must clear speed integrator, omega reference, and current reference in the same control tick.
- Stabilize target must have independent `GLIM` limit; default should be small.
- Deadband may set speed reference to zero, but watch for speed PI integrator kick when leaving deadband.

## Telemetry Fields

When reading logs, prioritize:

```text
ST
CTRL
Ref
IqRef
Iq
Id
POS_TGT
POS_ACT
POS_ERR
IMU
IMU_RDY
IMU_HOME
STAB_LIM
KVL_REC
```

Healthy signs:

- `ST=RUN` only after `e`
- `IqRef` not stuck at a large value
- `Id` average close to 0
- `KVL_REC` close to 0
- position error converges
- `IMU_RDY=1`
- IMU pitch stable when still
- `IMU_HOME` updates after `H`

## Mechanical Rules

MPU6050 must be fixed to the base/stator/bracket side. It measures base tilt.

MPU6050 must not be fixed to:

- rotor arm
- motor shaft
- payload
- any rotating part

AS5048A is already at the motor bottom and should be treated as the motor angle sensor.

The motor stator/back side should be fixed to the bracket. The rotor arm should attach to the rotating output side.

## Debugging Method

When analyzing a problem:

1. Identify current control mode.
2. Check command target and telemetry target.
3. Compare actual angle/speed and error.
4. Check `IqRef/Iq`, `Id`, and `KVL_REC`.
5. Decide whether the issue is sign, tuning, command parsing, IMU data, mechanical mounting, or current/speed loop stability.
6. Do not propose code changes until the failure mode is explained.

For stabilize mode, separate these causes:

- IMU not ready
- IMU zero offset not recorded
- `IMU_SIGN` wrong
- `MOTOR_SIGN` wrong
- position loop not stable
- mechanical structure not rigid
- MPU6050 mounted on the wrong part

## Development Stages

Stage 1: AS5048A position hold

- Position PD
- Deadband
- Soft/hard limits
- Runtime KP/KD/PLIM/DZ tuning
- `P0/P10/P-10`, then `P30/P-30` only after small-angle stability

Stage 2: Telemetry and monitoring

- `POS_TGT/POS_ACT/POS_ERR`
- `IMU/IMU_RDY/IMU_HOME/STAB_LIM`
- PC monitor curves for target, actual, error, IMU, speed reference, current

Stage 3: MPU6050 read-only

- I2C driver
- Complementary filter
- Static pitch stable within about 1 degree

Stage 4: Stabilize mode

- `GLIM5 -> H -> G`
- Confirm compensation direction with +/-5 degree base tilt
- Increase limit only after stable behavior

Stage 5: Demo polish

- STEP/HTEST response test
- Optional trapezoid trajectory
- Optional OLED display
- Optional gyro feedforward to reduce phase lag
