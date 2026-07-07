"""Dark industrial theme for FOC Gimbal Host."""

STYLESHEET = """
QWidget {
    background-color: #0d1117;
    color: #e6edf3;
    font-family: "Segoe UI", "Microsoft YaHei UI", sans-serif;
    font-size: 13px;
}

QMainWindow {
    background-color: #010409;
}

QGroupBox {
    border: 1px solid #30363d;
    border-radius: 10px;
    margin-top: 14px;
    padding: 12px 10px 10px 10px;
    font-weight: 600;
    color: #58a6ff;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 8px;
    background-color: #0d1117;
}

QPushButton {
    background-color: #21262d;
    border: 1px solid #30363d;
    border-radius: 8px;
    padding: 8px 14px;
    min-height: 20px;
}

QPushButton:hover {
    background-color: #30363d;
    border-color: #58a6ff;
}

QPushButton:pressed {
    background-color: #161b22;
}

QPushButton#primaryBtn {
    background-color: #238636;
    border-color: #2ea043;
    color: #ffffff;
    font-weight: 600;
}

QPushButton#primaryBtn:hover {
    background-color: #2ea043;
}

QPushButton#dangerBtn {
    background-color: #da3633;
    border-color: #f85149;
    color: #ffffff;
    font-weight: 700;
    font-size: 15px;
    min-height: 44px;
}

QPushButton#dangerBtn:hover {
    background-color: #f85149;
}

QPushButton#accentBtn {
    background-color: #1f6feb;
    border-color: #388bfd;
    color: #ffffff;
}

QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #161b22;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 6px 10px;
    selection-background-color: #388bfd;
}

QComboBox:focus, QLineEdit:focus {
    border-color: #58a6ff;
}

QPlainTextEdit {
    background-color: #010409;
    border: 1px solid #21262d;
    border-radius: 8px;
    font-family: "Cascadia Mono", "Consolas", monospace;
    font-size: 11px;
    color: #8b949e;
}

QScrollBar:vertical {
    background: #0d1117;
    width: 10px;
    border-radius: 5px;
}

QScrollBar::handle:vertical {
    background: #30363d;
    border-radius: 5px;
    min-height: 24px;
}

QLabel#titleLabel {
    font-size: 22px;
    font-weight: 700;
    color: #f0f6fc;
    letter-spacing: 0.5px;
}

QLabel#subtitleLabel {
    font-size: 12px;
    color: #8b949e;
}

QLabel#metricValue {
    font-size: 26px;
    font-weight: 700;
    color: #3fb950;
    font-family: "Cascadia Mono", "Consolas", monospace;
}

QLabel#metricUnit {
    font-size: 11px;
    color: #6e7681;
}

QFrame#card {
    background-color: #161b22;
    border: 1px solid #30363d;
    border-radius: 12px;
}

QCheckBox {
    spacing: 8px;
}

QCheckBox::indicator {
    width: 18px;
    height: 18px;
    border-radius: 4px;
    border: 1px solid #30363d;
    background: #21262d;
}

QCheckBox::indicator:checked {
    background: #238636;
    border-color: #2ea043;
}
"""

ACCENT_CYAN = "#39d353"
ACCENT_BLUE = "#58a6ff"
ACCENT_AMBER = "#d29922"
ACCENT_RED = "#f85149"
PLOT_BG = "#0d1117"
PLOT_FG = "#8b949e"