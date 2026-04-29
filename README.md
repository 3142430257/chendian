# FOC Motor Controller (STM32G431CBT6)

基于 Simulink + Embedded Coder 的 **FOC（磁场定向控制）** 无刷电机驱动项目，目标驱动 **GM3506** 云台无刷电机。

## 项目概述

- **控制策略**：闭环矢量控制（FOC），包含电流环 / 速度环 / 位置环
- **电流检测**：双电阻采样 + 重构算法
- **编码器**：SPI 磁编码器（AS5047P 兼容）
- **PWM**：SVPWM（空间矢量脉宽调制），20 kHz
- **支持模式**：速度模式、位置模式、自稳模式（IMU）
- **自动保护**：过流保护、低压保护、过温保护、编码器断线检测

## 目录结构

```
├── foc_controller.slx       # Simulink 控制器模型（电流环 PI + SVPWM）
├── foc_sim.slx              # Simulink 闭环仿真模型（电机 + 控制器）
├── foc_params.m             # 全局参数文件（电机参数 / PI 参数）
├── foc_monitor.py           # Python 串口实时诊断工具
├── foc_closedloop.png       # 闭环仿真波形截图
├── foc_stall_test.png       # 堵转测试波形
├── foc_verified_final.png   # 最终验证波形
├── cubemx_out/              # STM32CubeMX 工程（STM32G431CBT6）
├── foc_controller_ert_rtw/  # Simulink Coder 生成的 ERT 代码
├── STM32G431CBT6_Motor/     # STM32CubeIDE / 固件工程
├── 原理图&封装/              # 原理图 PDF + PCB 封装
├── gimbal_mount/            # 云台支架 3D 打印 STL 文件
├── schema_p1.png            # 原理图第1页截图
├── schema_p2.png            # 原理图第2页截图
└── ATK-PD6010B_V1.0.pdf     # 驱动器参考手册
```

## 硬件平台

| 组件       | 型号 / 参数                 |
| ---------- | --------------------------- |
| MCU        | STM32G431CBT6 (Cortex-M4)   |
| 电机       | GM3506 无刷云台电机 (11极对) |
| 驱动器     | ATK-PD6010B                 |
| 编码器     | 14位 SPI 磁编码器           |
| 母线电压   | 12V                         |
| PWM 频率   | 20 kHz                      |
| 控制周期   | 50 μs                       |

## 快速开始

### 1. Simulink 仿真

1. 打开 MATLAB（R2024b 或更新版本）
2. 在 MATLAB 中切换到项目根目录
3. 运行 `foc_params` 加载全局参数
4. 打开 `foc_sim.slx`，点击 **Run** 运行仿真
5. 查看波形结果

### 2. Python 诊断工具

```bash
# 安装依赖
pip install pyserial

# 运行（默认 COM11, 115200）
python foc_monitor.py

# 指定串口和波特率
python foc_monitor.py COM5 921600
```

**命令列表：**

| 命令 | 功能             |
| ---- | ---------------- |
| `a`  | 校准 (ALIGN)     |
| `e`  | 使能 (ENABLE)    |
| `d`  | 停止 (DISABLE)   |
| `+`  | Iq 参考 +0.05A   |
| `-`  | Iq 参考 -0.05A   |
| `]`  | 速度 +30°/s      |
| `[`  | 速度 -30°/s      |
| `0`  | 速度归零         |
| `s`  | 查询状态         |
| `G`  | 自稳模式         |
| `H`  | 回零（同步IMU零点）|
| `q`  | 退出             |

### 3. 固件编译与烧录

使用 STM32CubeIDE 或 CMake：

```bash
cd STM32G431CBT6_Motor
# 导入到 STM32CubeIDE，编译后烧录
```

或使用 CubeMX 重新生成：

```bash
# 打开 cubemx_out/cubemx_out.ioc
# 生成代码后编译烧录
```

## 电机参数

| 参数         | 值          |
| ------------ | ----------- |
| 极对数 (p)   | 11          |
| 相电阻 (Rs)  | 2.7 Ω       |
| d轴电感 (Ld) | 0.2 mH      |
| q轴电感 (Lq) | 0.2 mH      |
| 磁链 (Ψf)    | 0.01 Wb     |
| 转动惯量 (J) | 1×10⁻⁵ kg·m²|

## 控制参数

| 参数          | 值         |
| ------------- | ---------- |
| 电流环带宽    | 500 Hz     |
| Kp_d (d轴)    | ~2.51      |
| Ki_d (d轴)    | ~3770      |
| Kp_q (q轴)    | ~2.51      |
| Ki_q (q轴)    | ~3770      |
| SVPWM 电压上限 | Vbus/√3    |

## 3D 打印件

`gimbal_mount/` 目录包含云台支架的 STL 模型：

- `motor_back_mount.stl` — 电机后支架
- `motor_shaft_clamp.stl` — 轴夹
- `rotor_arm.stl` — 转子臂
- `修复-motor_back_mount.stl` — 修复版后支架

## 依赖

- **MATLAB R2024b** + Simulink + Simulink Coder + Embedded Coder
- **STM32CubeMX / STM32CubeIDE**
- **Python 3.8+** + `pyserial`

## 许可证

MIT License