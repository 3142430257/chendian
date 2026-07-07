"""Parse firmware CSV telemetry lines (T,... and DQ_STAT,...)."""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional

STATE_NAMES = {
    0: "INIT",
    1: "CALIB",
    2: "ALIGN",
    3: "READY",
    4: "RUN",
    5: "FAULT",
}
CTRL_NAMES = {0: "SPEED", 1: "POSITION", 2: "STABILIZE"}
FAULT_BITS = {
    0x01: "VBUS_LOW",
    0x02: "VBUS_HIGH",
    0x04: "TEMP",
    0x08: "ENCODER",
    0x10: "ADC_SAT",
}


def _f(m: re.Match, i: int) -> float:
    return float(m.group(i))


def _i(m: re.Match, i: int) -> int:
    return int(m.group(i))


# Core telemetry: the middle section is intentionally parsed lazily.
TELEMETRY_RE = re.compile(
    r"T,(\d+),IU,([-\d.]+),IV,([-\d.]+),IW,([-\d.]+),"
    r"VBUS,([-\d.]+),M_ANG,([-\d.]+),E_ANG,([-\d.]+),ST,(\d+),FAULT,(\d+),"
    r"IWR,([-\d.]+),KVL,([-\d.]+),"
    r"VEC,(\d+),"
    r"ENC_AGE,(\d+),ENC_VAL,(\d+),ENC_BAD,(\d+),ENC_DROP,(\d+),ENC_RAW,(\d+),ENC_RX,(\d+),"
    r"SPD_EST,([-\d.]+),SPD_MODE,(\d+),SPD_REF,([-\d.]+),IQ_REF,([-\d.]+),"
    r"ID_MEAS,([-\d.]+),IQ_MEAS,([-\d.]+),"
    r"CTRL,(\d+),POS_TGT,([-\d.]+),POS_ACT,([-\d.]+),POS_ERR,([-\d.]+),"
    r"IMU,([-\d.]+),IMU_RDY,(\d+),STAB_LIM,([-\d.]+),IMU_HOME,([-\d.]+),"
    r"OL_ACT,(\d+),OL_W,([-\d.]+),"
    r"THETA_E,([-\d.]+),OFFSET,([-\d.]+),FINE,([-\d.]+)"
)

DQ_RE = re.compile(
    r"DQ_STAT,ID_AVG,([-\d.]+),IQ_AVG,([-\d.]+),"
    r"ID_RMS,([-\d.]+),IQ_RMS,([-\d.]+),"
    r"IQREF_AVG,([-\d.]+),IQERR_AVG,([-\d.]+)"
)

FRAME_HEAD = re.compile(r"T,\d+,IU,")


@dataclass
class TelemetryFrame:
    t_ms: int = 0
    iu: float = 0.0
    iv: float = 0.0
    iw: float = 0.0
    vbus: float = 0.0
    m_ang: float = 0.0
    e_ang: float = 0.0
    st: int = 0
    fault: int = 0
    enc_age: int = 0
    enc_val: int = 0
    enc_bad: int = 0
    enc_drop: int = 0
    spd_est: float = 0.0
    spd_mode: int = 0
    spd_ref: float = 0.0
    iq_ref: float = 0.0
    id_meas: float = 0.0
    iq_meas: float = 0.0
    ctrl: int = 0
    pos_tgt: float = 0.0
    pos_act: float = 0.0
    pos_err: float = 0.0
    imu: float = 0.0
    imu_rdy: int = 0
    stab_lim: float = 10.0
    theta_e: float = 0.0
    offset: float = 0.0
    fine: float = 0.0

    @property
    def spd_est_dps(self) -> float:
        return self.spd_est * 57.2958

    @property
    def spd_ref_dps(self) -> float:
        return self.spd_ref * 57.2958

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.st, f"ST{self.st}")

    @property
    def ctrl_name(self) -> str:
        return CTRL_NAMES.get(self.ctrl, f"CTRL{self.ctrl}")

    def fault_list(self) -> list[str]:
        out = []
        for bit, name in FAULT_BITS.items():
            if self.fault & bit:
                out.append(name)
        return out


@dataclass
class DqStats:
    id_avg: float = 0.0
    iq_avg: float = 0.0
    id_rms: float = 0.0
    iq_rms: float = 0.0
    iqref_avg: float = 0.0
    iqerr_avg: float = 0.0


def parse_telemetry_line(line: str) -> Optional[TelemetryFrame]:
    m = TELEMETRY_RE.search(line)
    if not m:
        return None
    return TelemetryFrame(
        t_ms=_i(m, 1),
        iu=_f(m, 2),
        iv=_f(m, 3),
        iw=_f(m, 4),
        vbus=_f(m, 5),
        m_ang=_f(m, 6),
        e_ang=_f(m, 7),
        st=_i(m, 8),
        fault=_i(m, 9),
        enc_age=_i(m, 13),
        enc_val=_i(m, 14),
        enc_bad=_i(m, 15),
        enc_drop=_i(m, 16),
        spd_est=_f(m, 19),
        spd_mode=_i(m, 20),
        spd_ref=_f(m, 21),
        iq_ref=_f(m, 22),
        id_meas=_f(m, 23),
        iq_meas=_f(m, 24),
        ctrl=_i(m, 25),
        pos_tgt=_f(m, 26),
        pos_act=_f(m, 27),
        pos_err=_f(m, 28),
        imu=_f(m, 29),
        imu_rdy=_i(m, 30),
        stab_lim=_f(m, 31),
        theta_e=_f(m, 34),
        offset=_f(m, 35),
        fine=_f(m, 36),
    )


def parse_dq_line(line: str) -> Optional[DqStats]:
    m = DQ_RE.search(line)
    if not m:
        return None
    return DqStats(
        id_avg=float(m.group(1)),
        iq_avg=float(m.group(2)),
        id_rms=float(m.group(3)),
        iq_rms=float(m.group(4)),
        iqref_avg=float(m.group(5)),
        iqerr_avg=float(m.group(6)),
    )


class StreamAssembler:
    """Split UART byte stream into lines and T-frames (firmware may omit newlines between frames)."""

    def __init__(self) -> None:
        self._buf = ""

    def feed(self, data: bytes) -> tuple[list[str], list[str]]:
        text = data.decode("ascii", errors="ignore")
        self._buf += text.replace("\r", "\n")
        lines: list[str] = []
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line:
                lines.append(line)

        frames: list[str] = []
        flat = self._buf.replace("\n", "")
        while True:
            m1 = FRAME_HEAD.search(flat)
            if not m1:
                break
            m2 = FRAME_HEAD.search(flat, m1.end())
            if not m2:
                break
            seg = flat[m1.start() : m2.start()]
            flat = flat[m2.start() :]
            frames.append(seg)
        self._buf = flat
        return lines, frames
