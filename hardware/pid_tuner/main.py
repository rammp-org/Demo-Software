"""
Main entry point for the PID Tuner application.
"""

import atexit
import signal
import sys

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QTimer

from .ui.main_window import MainWindow


def main():
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("PID Tuner")
    app.setOrganizationName("MEBot")
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()

    def _cleanup():
        window._drive_wheel_display.shutdown()
        window._serial_handler.disconnect_port()

    atexit.register(_cleanup)

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    _sigint_timer = QTimer()
    _sigint_timer.timeout.connect(lambda: None)
    _sigint_timer.start(200)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
