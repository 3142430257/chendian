#!/usr/bin/env python3
"""FOC Gimbal Qt host — entry point."""
import sys

from PySide6.QtWidgets import QApplication

from foc_host.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("FOC Gimbal Host")
    app.setOrganizationName("chendian-github")
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())