# FOC 云台 Qt 上位机

深色工业风桌面控制台，对接固件 **USART3 @ 115200** CSV 遥测与单字符/行命令协议。

## 功能

- 串口连接、实时解析 `T,...` 遥测与 `[CMD]` 回显
- 实时曲线：位置 / Iq / IMU Pitch（约 60 s 滚动窗口）
- 一键流程：**校准 (a)** → **使能 (e)** → **Home (H)** → **自稳 (G)**
- 速度滑块 `S<dps>`、直接力矩 `I<amps>`、急停 **d**（连发 3 次）
- 软件看门狗（与 `tools/foc_ctl.py` 阈值一致，可关）

## 环境

- Python 3.10+
- Windows / Linux（需 Qt 平台插件）

```powershell
cd host\qt_host
pip install -r requirements.txt
python main.py
```

## 硬件

USB 转 TTL 接 MCU **USART3**（与 `foc_ctl.py` 相同）。上电后先等 `ST=READY` 再点「使能」。

## 目录

```
host/qt_host/
  main.py                 # 启动
  foc_host/
    main_window.py        # UI
    serial_worker.py      # 串口线程
    telemetry_parser.py   # CSV 解析
    theme.py              # 样式
```
