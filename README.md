# STM32G431 FOC 单轴云台自稳系统

基于 **STM32G431CBT6 + GM3506 无刷云台电机** 的单轴 FOC 姿态自稳系统。项目覆盖下位机固件、Qt 图形上位机、Simulink 电流环模型、机械结构和竞赛文档。

全国大学生嵌入式芯片与系统设计竞赛 2026 · 意法半导体赛道 · 选题四：工业 4.0 / 电机控制 / 机器人

## 功能概览

| 功能 | 命令序列 | 说明 |
|---|---|---|
| 电角度校准 | `a` | 旋转式 ALIGN，建立可靠 dq 坐标 |
| 闭环速度控制 | `a -> e -> S40 -> S-40 -> S0 -> d` | 支持正反转和停机 |
| 单轴云台自稳 | `a -> e -> H -> G` | 底座倾斜时，负载臂保持相对水平 |
| 遥测与诊断 | Qt 上位机连接串口 | 实时显示位置、Iq、IMU pitch 和状态 |

## 系统架构

```text
MPU6050 IMU
    |
    v
位置目标 -> ISR 级位置 PID -> IqRef -> Simulink 电流 PI -> SVPWM -> GM3506
              ^                                             |
              |                                             v
        AS5048A SPI 编码器 <----------------------------- 转子位置
```

- **电流环**：Simulink 生成 PI 控制器，20 kHz PWM/ADC 同步中断执行。
- **位置环**：ISR 级 PID，使用编码器角度反馈，支持非线性 KP 和积分补偿。
- **校准流程**：开环旋转采样 + 电角度修正 + Iq 脉冲方向验证。
- **自稳模式**：MPU6050 pitch 映射为位置目标，电机反向补偿底座倾斜。
- **通信接口**：USART3，115200 8N1，支持单字符/行命令和 `T,...` CSV 遥测。

## 硬件组成

| 模块 | 型号/说明 |
|---|---|
| MCU | STM32G431CBT6，Cortex-M4，170 MHz |
| 电机 | GM3506 无刷云台电机，11 极对 |
| 驱动板 | ATK-PD6010B |
| 编码器 | AS5048A，14 bit SPI 磁编码器 |
| IMU | MPU6050，I2C |
| 电源 | 12 V DC |
| 串口 | USART3，PB10 TX，PB11 RX，115200 8N1 |

## 仓库结构

```text
firmware/
  STM32G431CBT6_motor/       当前主固件工程，CMake 构建
    App/                     FOC 接口、状态机、编码器、IMU、遥测
    Core/                    CubeMX 生成的 HAL 外设代码
  cubemx_reference/          早期 CubeMX 参考工程

host/
  qt_host/                   PySide6 图形上位机

models/
  foc_controller.slx         Simulink 电流环模型
  foc_sim.slx                闭环仿真模型
  foc_params.m               电机和 PI 参数
  foc_controller_ert_rtw/    Simulink Coder 生成代码与报告

hardware/
  gimbal_mount/              3D 打印支架 STL 与渲染图
  schematics_and_package/    原理图、封装、器件资料
  controller_board/          主控/驱动板参考资料

docs/
  report/                    竞赛报告与技术文档
  media/                     README/报告用图片
  references/                外部参考资料
```

说明：本地调试脚本目录 `tools/` 不上传 GitHub，已在 `.gitignore` 中忽略。

## 编译与烧录

依赖：

- STM32CubeCLT，包含 `arm-none-eabi-gcc`、CMake/Ninja 和 STM32CubeProgrammer
- STM32G431CBT6 目标板，SWD 下载接口

编译：

```powershell
cd firmware\STM32G431CBT6_motor
cmake --preset Debug
cmake --build --preset Debug
```

使用 STM32CubeProgrammer CLI 烧录：

```powershell
& 'D:\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD mode=UR reset=HWrst `
  -w 'build\Debug\STM32G431CBT6_motor.elf' `
  -v -rst
```

如果 CMake 在中文路径下异常，建议把仓库复制到纯英文路径后再编译。

## Qt 上位机

依赖：

- Python 3.10+
- PySide6
- pyqtgraph
- pyserial
- numpy

启动：

```powershell
cd host\qt_host
pip install -r requirements.txt
python main.py
```

连接方式：

- USB 转 TTL 接 MCU `USART3 @ 115200`
- MCU `PB10` 为 TX，接 USB 转 TTL RX
- MCU `PB11` 为 RX，接 USB 转 TTL TX
- GND 必须共地
- 上电后等待状态进入 `ST=READY`，再执行使能或自稳流程

## 串口命令

| 命令 | 功能 |
|---|---|
| `a` | 旋转式电角度校准，ALIGN |
| `e` | 进入 RUN 使能状态 |
| `d` | 停机 |
| `H` | 将当前位置设为 Home 零点 |
| `G` | 启动 IMU 自稳模式 |
| `S<dps>` | 速度模式，例如 `S40` 表示 40 deg/s |
| `I<A>` | 直接力矩注入，例如 `I0.15` |
| `O` | 开环旋转测试 |
| `X` | 停止开环测试 |
| `F<deg>` | 手动微调电角度偏移 |

典型流程：

```text
a -> e -> H -> G
```

## 关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| 电流环频率 | 20 kHz | PWM/ADC 同步 ISR |
| 电流环带宽 | 200 Hz | Simulink PI |
| 位置环更新 | 约 1 kHz | 编码器反馈 |
| 遥测频率 | 100 Hz | UART CSV 帧 |
| ISR_KP_NEAR | 0.015 A/deg | 小误差位置增益 |
| ISR_KP_FAR | 0.030 A/deg | 大误差位置增益 |
| ISR_KD | 0.010 A/(deg/s) | 阻尼项 |
| ISR_KI | 0.0005 A/(deg*s) | 重力静差积分补偿 |
| 速度保护 | 400 deg/s | ISR 级切断力矩 |

## 开发说明

- 主固件入口：`firmware/STM32G431CBT6_motor/`
- Qt 上位机入口：`host/qt_host/main.py`
- 电流环模型：`models/foc_controller.slx`
- 竞赛报告草稿：`docs/report/竞赛作品报告_填报稿.md`
- 本地视频、压缩包和调试脚本不纳入 GitHub

## 许可证

MIT License
