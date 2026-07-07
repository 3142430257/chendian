#!/usr/bin/env python3
"""Launch FOC Gimbal Qt host."""
import sys

from PySide6.QtWidgets import QApplication

from foc_host.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("FOC Gimbal Host")
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())