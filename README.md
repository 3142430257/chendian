# STM32G431 FOC 云台自稳系统

基于 STM32G431CBT6 + GM3506 无刷电机的**单轴云台姿态自稳**系统。

全国大学生嵌入式芯片与系统设计竞赛 2026 · 意法半导体赛道 · 选题四：工业 4.0/电机控制/机器人

## 功能演示

| 功能 | 命令序列 | 效果 |
|---|---|---|
| 闭环速度控制 | `a → e → S40 → S-40 → S100 → S0 → d` | 电机按指定速度正反转 |
| 云台自稳 | `a → e → H → G` | 倾斜底座时负载臂保持水平 |

## 系统架构

```
IMU (MPU6050) ──→ 位置目标 ──→ ISR 级 PID ──→ IqRef ──→ 电流环 (Simulink PI) ──→ SVPWM ──→ 电机
                                    ↑                                                    ↓
                              编码器 (AS5048A SPI) ←──────────────────────────────────── GM3506
```

- **电流环**：Simulink 生成 PI 控制器，20kHz ISR 执行
- **位置环**：ISR 级 PID（1kHz 编码器更新率），非线性 KP + 积分消除重力静差
- **ALIGN**：旋转式校准（openloop 采样 + π/2 修正 + Iq 脉冲方向验证）
- **自稳**：IMU pitch → 位置目标，电机反向补偿保持水平

## 硬件

| 组件 | 型号 |
|---|---|
| MCU | STM32G431CBT6 (Cortex-M4, 170MHz) |
| 电机 | GM3506 无刷云台电机 (11 极对) |
| 驱动板 | ATK-PD6010B |
| 编码器 | AS5048A (14bit SPI 磁编码器) |
| IMU | MPU6050 (I2C) |
| 电源 | 12V DC |

## 目录结构

```
├── STM32G431CBT6_Motor/     # 固件工程（CMake 构建）
│   └── App/                 # 应用层代码
│       ├── app_control.c    # 状态机 + 位置跟踪 + 自稳逻辑
│       ├── foc_interface.c  # ISR：电流环 + ISR级PID + 安全保护
│       ├── foc_controller.c # Simulink 生成的电流环 PI
│       ├── encoder_spi.c    # AS5048A SPI 驱动 + 速度估计
│       ├── mpu6050.c        # IMU 驱动
│       └── telemetry.c      # 串口遥测 (100Hz)
├── cubemx_out/              # STM32CubeMX 工程
├── foc_controller_ert_rtw/  # Simulink Coder 生成代码
├── foc_controller.slx       # Simulink 电流环模型
├── foc_sim.slx              # 闭环仿真模型
├── foc_params.m             # 电机/PI 参数
├── tmp/foc_ctl.py           # 串口控制 + watchdog 工具
├── gimbal_mount/            # 3D 打印支架 STL
└── 原理图&封装/              # 硬件设计文档
```

## 快速开始

### 编译烧录

```powershell
cmake --build STM32G431CBT6_Motor\STM32G431CBT6_motor\build\Debug

& 'D:\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD mode=UR reset=HWrst `
  -w 'STM32G431CBT6_Motor\STM32G431CBT6_motor\build\Debug\STM32G431CBT6_motor.elf' `
  -v -rst
```

### 串口控制

```bash
python tmp/foc_ctl.py run "d:1;a:4.5;e:0.3;H:0.5;G:30;d:0.5"
```

### 命令列表

| 命令 | 功能 |
|---|---|
| `a` | 旋转式电角度校准 (ALIGN) |
| `e` | 使能 RUN 模式 |
| `d` | 停机 |
| `H` | 设当前位置为零点 (Home) |
| `G` | 启动 IMU 自稳模式 |
| `S<dps>` | 速度模式（如 S40 = 40°/s） |
| `I<A>` | 直接力矩注入（如 I0.15） |
| `O` | 开环旋转测试 |
| `X` | 停止开环 |
| `F<deg>` | 手动微调电角度偏移 |

## 控制参数

| 参数 | 值 | 说明 |
|---|---|---|
| 电流环带宽 | 200 Hz | Simulink PI |
| ISR_KP_NEAR | 0.015 A/deg | 位置误差 <10° |
| ISR_KP_FAR | 0.030 A/deg | 位置误差 >10° |
| ISR_KD | 0.010 A/(deg/s) | 阻尼 |
| ISR_KI | 0.0005 A/(deg·s) | 积分消除重力静差 |
| 速度保护 | 400°/s | ISR 级切断力矩 |

## 依赖

- STM32CubeCLT (含 arm-none-eabi-gcc + STM32CubeProgrammer)
- Python 3 + pyserial
- MATLAB R2024b + Simulink + Embedded Coder（仅修改电流环时需要）

## 许可证

MIT License
