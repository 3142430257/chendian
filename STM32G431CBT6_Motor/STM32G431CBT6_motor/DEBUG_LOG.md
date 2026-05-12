# Debug Log

Use this file to preserve hardware and telemetry results across AI sessions.

## 2026-05-04 Project Memory Setup

Action:

- Scanned current firmware and project rules.
- Added Claude Code project memory files.

Observed from source scan:

- `App/app_control.c` defaults: `KP=1.0`, `KD=0.02`, `PLIM=45`, `DZ=0.5`, `STAB_LIM=10`.
- Stabilize mode uses IMU home offset: `mpu6050_get_pitch_deg() - s_imu_home_offset_deg`.
- `H` records IMU home offset.
- `GLIM` command exists.
- Telemetry includes `IMU`, `IMU_RDY`, `STAB_LIM`, and `IMU_HOME`.
- MPU6050 implementation uses software I2C bit-bang on `PB5=SCL`, `PB4=SDA`.

Current conclusion:

- Do not rely on previous chat history for state.
- Use `CLAUDE.md`, `AGENTS.md`, `PROJECT_STATUS.md`, `TODO_NEXT.md`, `DEBUG_LOG.md`, and `DECISIONS.md` as project memory.

## Test Entry Template

Copy this block after each hardware test:

```text
Date:
Mechanical setup:
Firmware commit/build:
Commands:
Parameters:
Telemetry summary:
Observed behavior:
Pass/fail:
Conclusion:
Next action:
```
