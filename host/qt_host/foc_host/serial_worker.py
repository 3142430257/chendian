"""QThread serial I/O for FOC host."""
from __future__ import annotations

from PySide6.QtCore import QObject, QThread, QTimer, Signal, Slot

import serial
from serial.tools import list_ports

from .telemetry_parser import (
    StreamAssembler,
    parse_dq_line,
    parse_telemetry_line,
)


class SerialWorker(QObject):
    frame_received = Signal(object)
    dq_received = Signal(object)
    log_line = Signal(str)
    connected_changed = Signal(bool)
    error = Signal(str)
    request_open = Signal(str, int)
    request_close = Signal()
    request_send = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._ser: serial.Serial | None = None
        self._running = False
        self._assembler = StreamAssembler()
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(15)
        self._poll_timer.timeout.connect(self.poll)
        self.request_open.connect(self.open_port)
        self.request_close.connect(self.close_port)
        self.request_send.connect(self.send_command)

    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        return [(p.device, p.description) for p in list_ports.comports()]

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @Slot(str, int)
    def open_port(self, port: str, baud: int = 115200) -> None:
        self.close_port()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
            self._running = True
            self._assembler = StreamAssembler()
            self._poll_timer.start()
            self.connected_changed.emit(True)
            self.log_line.emit(f"[host] Opened {port} @ {baud}")
        except Exception as e:
            self.error.emit(str(e))
            self.connected_changed.emit(False)

    @Slot()
    def close_port(self) -> None:
        self._poll_timer.stop()
        self._running = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.connected_changed.emit(False)

    @Slot(str)
    def send_command(self, cmd: str) -> None:
        if not self._ser or not self._ser.is_open:
            self.error.emit("Not connected")
            return
        line = cmd.strip()
        if not line:
            return
        try:
            self._ser.write(line.encode("ascii"))
            self.log_line.emit(f">>> {line}")
        except Exception as e:
            self.error.emit(str(e))

    @Slot()
    def poll(self) -> None:
        if not self._running or not self._ser or not self._ser.is_open:
            return
        try:
            chunk = self._ser.read(4096)
            if not chunk:
                return
            lines, frames = self._assembler.feed(chunk)
            for line in lines:
                if line.startswith("[CMD]") or line.startswith("[ENC]"):
                    self.log_line.emit(line)
                dq = parse_dq_line(line)
                if dq:
                    self.dq_received.emit(dq)
                fr = parse_telemetry_line(line)
                if fr:
                    self.frame_received.emit(fr)
            for seg in frames:
                fr = parse_telemetry_line(seg)
                if fr:
                    self.frame_received.emit(fr)
        except Exception as e:
            self.error.emit(str(e))
            self.close_port()


class SerialThread(QThread):
    def __init__(self, worker: SerialWorker) -> None:
        super().__init__()
        self.worker = worker
        self.setObjectName("SerialThread")

    def run(self) -> None:
        self.exec()