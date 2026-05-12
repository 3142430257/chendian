# Project Rules

This is an STM32G431 + GM3506 + AS5048A + MPU6050 single-axis FOC self-stabilizing gimbal project for the 2026 embedded competition ST track.

Before doing project work, read:

- `PROJECT_STATUS.md`
- `TODO_NEXT.md`
- `DECISIONS.md`
- `DEBUG_LOG.md`

Do not rely on chat history as project memory.

Always use conservative motor bring-up. Do not suggest opening stabilize mode directly after flashing.

Safe default test order:

```text
power on without e -> e -> d -> e -> S30 -> S-30 -> S60 -> S-60 -> S0 -> H -> P10 -> P0 -> P-10 -> IMU read-only -> GLIM5 -> H -> G
```

Before `G`, confirm:

- small-angle position loop is stable
- `IMU_RDY=1`
- IMU pitch is stable
- `H` has recorded encoder zero and IMU home offset
- `STAB_LIM` is small, preferably 5 degrees

Key mechanics:

- MPU6050 is fixed to base/stator/bracket side
- AS5048A is at motor bottom
- rotor arm attaches to the rotating output side
- payload should remain level by enforcing `theta + phi = 0`
- current MPU6050 firmware uses software I2C bit-bang on `PB5=SCL`, `PB4=SDA`
