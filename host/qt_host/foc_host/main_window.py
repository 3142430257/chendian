"""Main window — FOC Gimbal Host."""
from __future__ import annotations

import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .serial_worker import SerialWorker, SerialThread
from .telemetry_parser import TelemetryFrame
from .theme import ACCENT_AMBER, ACCENT_BLUE, ACCENT_CYAN, ACCENT_RED, PLOT_BG, PLOT_FG, STYLESHEET


class MetricCard(QFrame):
    def __init__(self, title: str, unit: str = "", parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("card")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        t = QLabel(title)
        t.setStyleSheet(f"color: {PLOT_FG}; font-size: 11px;")
        self._val = QLabel("—")
        self._val.setObjectName("metricValue")
        self._val.setStyleSheet(f"color: {ACCENT_CYAN}; font-size: 22px;")
        u = QLabel(unit)
        u.setObjectName("metricUnit")
        lay.addWidget(t)
        lay.addWidget(self._val)
        lay.addWidget(u)

    def set_value(self, text: str, color: str | None = None) -> None:
        self._val.setText(text)
        if color:
            self._val.setStyleSheet(f"color: {color}; font-size: 22px; font-weight: 700;")


class MainWindow(QMainWindow):
    emergency_stop = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("FOC 云台控制台 · STM32G431")
        self.resize(1280, 800)
        self.setStyleSheet(STYLESHEET)

        pg.setConfigOptions(antialias=True, background=PLOT_BG, foreground=PLOT_FG)

        self._history_len = 600
        self._t0: float | None = None
        self._t_hist: deque[float] = deque(maxlen=self._history_len)
        self._pos_act: deque[float] = deque(maxlen=self._history_len)
        self._pos_tgt: deque[float] = deque(maxlen=self._history_len)
        self._imu: deque[float] = deque(maxlen=self._history_len)
        self._iq_ref: deque[float] = deque(maxlen=self._history_len)
        self._iq_meas: deque[float] = deque(maxlen=self._history_len)
        self._spd: deque[float] = deque(maxlen=self._history_len)

        self._last_frame: TelemetryFrame | None = None
        self._watchdog_enabled = True

        self._worker = SerialWorker()
        self._thread = SerialThread(self._worker)
        self._worker.moveToThread(self._thread)
        self._thread.start()

        self._build_ui()
        self._wire_signals()

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(50)
        self._plot_timer.timeout.connect(self._refresh_plots)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 12, 16, 16)
        root.setSpacing(12)

        # Header
        header = QHBoxLayout()
        title_col = QVBoxLayout()
        title = QLabel("FOC 云台自稳")
        title.setObjectName("titleLabel")
        sub = QLabel("GM3506 · AS5048A · MPU6050 · 115200 CSV 遥测")
        sub.setObjectName("subtitleLabel")
        title_col.addWidget(title)
        title_col.addWidget(sub)
        header.addLayout(title_col)
        header.addStretch()

        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(160)
        self._btn_refresh_ports = QPushButton("刷新端口")
        self._btn_connect = QPushButton("连接")
        self._btn_connect.setObjectName("primaryBtn")
        header.addWidget(self._port_combo)
        header.addWidget(self._btn_refresh_ports)
        header.addWidget(self._btn_connect)
        root.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(14)

        # Left — controls
        left = QVBoxLayout()
        left.setSpacing(10)

        flow = QGroupBox("标准流程")
        fl = QVBoxLayout(flow)
        btn_align = QPushButton("① 电角度校准 (a)")
        btn_align.setObjectName("accentBtn")
        btn_enable = QPushButton("② 使能运行 (e)")
        btn_enable.setObjectName("accentBtn")
        btn_home = QPushButton("③ 回零 Home (H)")
        btn_stab = QPushButton("④ 自稳模式 (G)")
        btn_stab.setObjectName("accentBtn")
        for b in (btn_align, btn_enable, btn_home, btn_stab):
            fl.addWidget(b)
        btn_align.clicked.connect(lambda: self._send("a"))
        btn_enable.clicked.connect(lambda: self._send("e"))
        btn_home.clicked.connect(lambda: self._send("H"))
        btn_stab.clicked.connect(lambda: self._send("G"))
        left.addWidget(flow)

        spd_g = QGroupBox("速度模式")
        sg = QVBoxLayout(spd_g)
        row = QHBoxLayout()
        self._spd_slider = QSlider(Qt.Horizontal)
        self._spd_slider.setRange(-120, 120)
        self._spd_slider.setValue(0)
        self._spd_spin = QSpinBox()
        self._spd_spin.setRange(-300, 300)
        self._spd_spin.setSuffix(" °/s")
        row.addWidget(self._spd_slider, 1)
        row.addWidget(self._spd_spin)
        sg.addLayout(row)
        btn_s0 = QPushButton("停止 S0")
        btn_s_apply = QPushButton("应用速度 S")
        btn_s_apply.setObjectName("accentBtn")
        sg.addWidget(btn_s0)
        sg.addWidget(btn_s_apply)
        self._spd_slider.valueChanged.connect(self._spd_spin.setValue)
        self._spd_spin.valueChanged.connect(self._spd_slider.setValue)
        btn_s0.clicked.connect(lambda: self._apply_speed(0))
        btn_s_apply.clicked.connect(lambda: self._apply_speed(self._spd_spin.value()))
        left.addWidget(spd_g)

        tor_g = QGroupBox("力矩 / 调试")
        tg = QGridLayout(tor_g)
        self._iq_spin = QDoubleSpinBox()
        self._iq_spin.setRange(-0.45, 0.45)
        self._iq_spin.setSingleStep(0.05)
        self._iq_spin.setDecimals(3)
        tg.addWidget(QLabel("Iq (A)"), 0, 0)
        tg.addWidget(self._iq_spin, 0, 1)
        btn_i = QPushButton("I 指令")
        btn_i.clicked.connect(lambda: self._send(f"I{self._iq_spin.value():.3f}"))
        tg.addWidget(btn_i, 0, 2)
        btn_status = QPushButton("状态 s")
        btn_status.clicked.connect(lambda: self._send("s"))
        tg.addWidget(btn_status, 1, 0, 1, 3)
        left.addWidget(tor_g)

        self._chk_watchdog = QCheckBox("软件看门狗（超限自动 d）")
        self._chk_watchdog.setChecked(True)
        left.addWidget(self._chk_watchdog)

        self._btn_estop = QPushButton("急停 STOP (d)")
        self._btn_estop.setObjectName("dangerBtn")
        self._btn_estop.clicked.connect(self._emergency_stop)
        left.addWidget(self._btn_estop)
        left.addStretch()
        body.addLayout(left, 0)

        # Center — plots
        plot_col = QVBoxLayout()
        self._plot_pos = self._make_plot("位置 (°)", ["实际", "目标"])
        self._plot_iq = self._make_plot("Iq (A)", ["参考", "测量"])
        self._plot_imu = self._make_plot("IMU Pitch (°)", ["Pitch"])
        plot_col.addWidget(self._plot_pos, 1)
        plot_col.addWidget(self._plot_iq, 1)
        plot_col.addWidget(self._plot_imu, 1)
        body.addLayout(plot_col, 1)

        # Right — metrics + log
        right = QVBoxLayout()
        self._card_state = MetricCard("系统状态")
        self._card_vbus = MetricCard("母线", "V")
        self._card_pos = MetricCard("位置误差", "°")
        self._card_spd = MetricCard("速度估计", "°/s")
        self._card_enc = MetricCard("编码器", "ms")
        for c in (
            self._card_state,
            self._card_vbus,
            self._card_pos,
            self._card_spd,
            self._card_enc,
        ):
            right.addWidget(c)

        log_g = QGroupBox("串口日志")
        ll = QVBoxLayout(log_g)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(400)
        ll.addWidget(self._log)
        right.addWidget(log_g, 1)
        body.addLayout(right, 0)

        root.addLayout(body, 1)
        self._refresh_ports()

    def _make_plot(self, title: str, names: list[str]) -> pg.PlotWidget:
        w = pg.PlotWidget(title=title)
        w.showGrid(x=True, y=True, alpha=0.25)
        w.setLabel("bottom", "时间", units="s")
        colors = [ACCENT_CYAN, ACCENT_AMBER, ACCENT_BLUE]
        w._curves = []
        for i, name in enumerate(names):
            pen = pg.mkPen(colors[i % len(colors)], width=2)
            w._curves.append(w.plot(pen=pen, name=name))
        w.addLegend(offset=(10, 10))
        return w

    def _wire_signals(self) -> None:
        self._btn_refresh_ports.clicked.connect(self._refresh_ports)
        self._btn_connect.clicked.connect(self._toggle_connect)
        self._worker.frame_received.connect(self._on_frame)
        self._worker.log_line.connect(self._append_log)
        self._worker.error.connect(lambda e: self._append_log(f"[error] {e}"))
        self._worker.connected_changed.connect(self._on_connected)

    def _refresh_ports(self) -> None:
        self._port_combo.clear()
        for dev, desc in SerialWorker.list_ports():
            self._port_combo.addItem(f"{dev}  {desc}", dev)
        if self._port_combo.count() == 0:
            self._port_combo.addItem("无可用端口", "")

    def _toggle_connect(self) -> None:
        if self._worker.is_connected():
            self._plot_timer.stop()
            self._worker.request_close.emit()
            return
        port = self._port_combo.currentData()
        if not port:
            self._append_log("[host] 请选择串口")
            return
        self._worker.request_open.emit(port, 115200)
        self._plot_timer.start()

    @Slot(bool)
    def _on_connected(self, ok: bool) -> None:
        self._btn_connect.setText("断开" if ok else "连接")
        if ok:
            self._t0 = None
            self._clear_history()

    def _clear_history(self) -> None:
        for d in (
            self._t_hist,
            self._pos_act,
            self._pos_tgt,
            self._imu,
            self._iq_ref,
            self._iq_meas,
            self._spd,
        ):
            d.clear()

    def _send(self, cmd: str) -> None:
        self._worker.request_send.emit(cmd)

    def _apply_speed(self, dps: int) -> None:
        self._send(f"S{dps}")

    def _emergency_stop(self) -> None:
        for _ in range(3):
            self._send("d")
        self._spd_slider.setValue(0)
        self._spd_spin.setValue(0)

    def _append_log(self, line: str) -> None:
        self._log.appendPlainText(line)

    def _watchdog_trip(self, reason: str) -> None:
        self._append_log(f"[WATCHDOG] {reason}")
        self._emergency_stop()

    @Slot(object)
    def _on_frame(self, frame: TelemetryFrame) -> None:
        self._last_frame = frame
        if self._chk_watchdog.isChecked():
            if frame.fault:
                self._watchdog_trip(f"FAULT=0x{frame.fault:02X}")
                return
            if abs(frame.pos_act) > 200:
                self._watchdog_trip(f"|POS|={frame.pos_act:.1f}°")
                return
            if abs(frame.spd_est) > 15:
                self._watchdog_trip(f"SPD={frame.spd_est:.2f} rad/s")
                return
            if abs(frame.iq_ref) > 0.55:
                self._watchdog_trip(f"IQ_REF={frame.iq_ref:.3f} A")
                return

        if self._t0 is None:
            self._t0 = frame.t_ms / 1000.0
        t = frame.t_ms / 1000.0 - self._t0

        self._t_hist.append(t)
        self._pos_act.append(frame.pos_act)
        self._pos_tgt.append(frame.pos_tgt)
        self._imu.append(frame.imu if frame.imu_rdy else float("nan"))
        self._iq_ref.append(frame.iq_ref)
        self._iq_meas.append(frame.iq_meas)
        self._spd.append(frame.spd_est_dps)

        st_color = ACCENT_CYAN
        if frame.st == 5:
            st_color = ACCENT_RED
        elif frame.st == 4:
            st_color = ACCENT_CYAN
        elif frame.st == 2:
            st_color = ACCENT_AMBER
        faults = frame.fault_list()
        st_text = frame.state_name
        if frame.ctrl_name != "SPEED" or frame.spd_mode:
            st_text += f" · {frame.ctrl_name}"
        if faults:
            st_text += " ⚠ " + ",".join(faults)
        self._card_state.set_value(st_text, st_color)

        vb_color = ACCENT_CYAN if 9 < frame.vbus < 26 else ACCENT_RED
        self._card_vbus.set_value(f"{frame.vbus:.1f}", vb_color)
        self._card_pos.set_value(f"{frame.pos_err:+.1f}", ACCENT_BLUE)
        self._card_spd.set_value(f"{frame.spd_est_dps:+.0f}", ACCENT_AMBER)
        enc_c = ACCENT_CYAN if frame.enc_val and frame.enc_age < 30 else ACCENT_RED
        self._card_enc.set_value(
            f"{frame.enc_age} {'OK' if frame.enc_val else 'LOST'}",
            enc_c,
        )

    def _refresh_plots(self) -> None:
        if len(self._t_hist) < 2:
            return
        t = np.array(self._t_hist, dtype=float)
        self._plot_pos._curves[0].setData(t, np.array(self._pos_act))
        self._plot_pos._curves[1].setData(t, np.array(self._pos_tgt))
        self._plot_iq._curves[0].setData(t, np.array(self._iq_ref))
        self._plot_iq._curves[1].setData(t, np.array(self._iq_meas))
        self._plot_imu._curves[0].setData(t, np.array(self._imu))

    def closeEvent(self, event) -> None:
        self._plot_timer.stop()
        self._worker.request_close.emit()
        self._thread.quit()
        self._thread.wait(2000)
        super().closeEvent(event)