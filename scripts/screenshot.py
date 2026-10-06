#!/usr/bin/env python3

"""
Screenshot tool. Lists every connected screen, lets you tick which ones to grab,
and saves a PNG per selected screen every time you press capture. The window
stays open so you can keep taking shots.

Press identify to flash a big number on each screen, the same way the Ubuntu
display settings do, so you can tell which entry is which.

    ./scripts/screenshot.sh ~/rover_screenshots
"""

import argparse
import datetime
import signal
import sys
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QGraphicsOpacityEffect,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QToolTip,
    QVBoxLayout,
    QWidget,
)

HIDE_SETTLE_MS = 200
IDENTIFY_MS = 2500
IDENTIFY_SIZE = 220
TOOLTIP_MS = 20000
SIGNAL_POLL_MS = 200

TOOLTIP_STYLE = """
QToolTip {
    font-size: 17px;
    padding: 10px;
    border: 1px solid palette(mid);
    background: palette(base);
    color: palette(text);
}
"""


class InfoIcon(QLabel):
    """Circled "i" that explains the control next to it when hovered."""

    def __init__(self, tooltip: str) -> None:
        super().__init__("i")

        self.tooltip = tooltip
        self.setAlignment(Qt.AlignCenter)
        self.setFixedSize(18, 18)
        self.setCursor(Qt.WhatsThisCursor)
        # Scoped to the type on purpose, an unscoped rule also lands on the
        # tooltip this widget pops up and shrinks its text to icon size
        self.setStyleSheet(
            """
            InfoIcon {
                border: 1px solid palette(window-text);
                border-radius: 9px;
                color: palette(window-text);
                font-size: 11px;
                font-style: italic;
                font-weight: bold;
            }
            """
        )

        # The icon should sit quietly next to its control, not compete with it
        faded = QGraphicsOpacityEffect(self)
        faded.setOpacity(0.8)
        self.setGraphicsEffect(faded)

    def enterEvent(self, event) -> None:
        # Qt's own tooltip handling is unreliable on a target this small, so
        # drive it ourselves the moment the cursor lands on the icon
        QToolTip.showText(self.mapToGlobal(self.rect().bottomLeft()), self.tooltip, self, self.rect(), TOOLTIP_MS)
        super().enterEvent(event)

    def leaveEvent(self, event) -> None:
        QToolTip.hideText()
        super().leaveEvent(event)


class IdentifyOverlay(QWidget):
    """Borderless number badge parked in the middle of one screen for a moment."""

    def __init__(self, screen, number: int) -> None:
        super().__init__(None, Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Tool)

        # Never steal focus or a spot in the taskbar from whatever is underneath
        self.setAttribute(Qt.WA_ShowWithoutActivating)
        self.setFocusPolicy(Qt.NoFocus)

        label = QLabel(str(number))
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont(self.font().family(), 96, QFont.Bold))

        name = QLabel(screen.name())
        name.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(label)
        layout.addWidget(name)

        self.setStyleSheet("background: #202020; color: white; border-radius: 16px;")

        center = screen.geometry().center()
        self.setGeometry(
            center.x() - IDENTIFY_SIZE // 2,
            center.y() - IDENTIFY_SIZE // 2,
            IDENTIFY_SIZE,
            IDENTIFY_SIZE,
        )


class ScreenEntry:
    """A single screen: its checkbox row plus the grabbing it knows how to do."""

    def __init__(self, screen, index: int) -> None:
        self.screen = screen
        self.index = index

        size = screen.geometry()
        self.checkbox = QCheckBox(f"{index + 1}.  {screen.name()}  ({size.width()}x{size.height()})")
        self.checkbox.setChecked(True)

    @property
    def selected(self) -> bool:
        return self.checkbox.isChecked()

    def grab(self) -> QPixmap:
        # 0 is the root window, which Qt clips to this screen's geometry
        return self.screen.grabWindow(0)

    def file_stem(self) -> str:
        name = "".join(c if c.isalnum() or c in "-_" else "_" for c in self.screen.name())
        return name or f"screen{self.index + 1}"


class ScreenshotWindow(QWidget):
    def __init__(self, output_dir: Path) -> None:
        super().__init__()

        self.output_dir = output_dir
        self.saved_geometry = None
        self.overlays: list[IdentifyOverlay] = []
        self.setWindowTitle("Screenshot")

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(f"Saving to: {output_dir}"))

        self.entries = [ScreenEntry(screen, i) for i, screen in enumerate(QApplication.screens())]
        for entry in self.entries:
            layout.addWidget(entry.checkbox)

        identify_button = QPushButton("Identify screens")
        identify_button.clicked.connect(self.identify)
        layout.addWidget(identify_button)

        self.delay = QDoubleSpinBox()
        self.delay.setRange(0.0, 30.0)
        self.delay.setSingleStep(0.5)
        self.delay.setValue(0.0)
        self.delay.setSuffix(" s delay")
        delay_info = InfoIcon(
            "How long to wait between pressing capture and taking the shot,\n"
            "so you can open a menu or hover something first."
        )

        self.hide_self = QCheckBox("Hide this window while capturing")
        self.hide_self.setChecked(True)
        hide_info = InfoIcon(
            f"Keeps this window out of the shot.\n"
            f"Forces a minimum delay of {HIDE_SETTLE_MS} ms, the time the window manager\n"
            f"needs to unmap the window. Uncheck this for a true zero delay."
        )

        capture_button = QPushButton("Capture")
        capture_button.setDefault(True)
        capture_button.clicked.connect(self.start_capture)

        controls = QHBoxLayout()
        controls.addWidget(delay_info)
        controls.addWidget(self.delay)
        controls.addSpacing(12)
        controls.addWidget(hide_info)
        controls.addWidget(self.hide_self)
        controls.addStretch()
        controls.addWidget(capture_button)
        layout.addLayout(controls)

        self.status = QLabel("Ready")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

    def identify(self) -> None:
        self.clear_overlays()
        for entry in self.entries:
            overlay = IdentifyOverlay(entry.screen, entry.index + 1)
            overlay.show()
            self.overlays.append(overlay)
        QTimer.singleShot(IDENTIFY_MS, self.clear_overlays)

    def clear_overlays(self) -> None:
        for overlay in self.overlays:
            overlay.close()
        self.overlays.clear()

    def start_capture(self) -> None:
        selected = [entry for entry in self.entries if entry.selected]
        if not selected:
            self.status.setText("Nothing selected")
            return

        # The badges are ours, they have no business showing up in a screenshot
        self.clear_overlays()

        delay_ms = int(self.delay.value() * 1000)

        if self.hide_self.isChecked():
            # Most window managers place a re-shown window wherever they like,
            # so remember where the user put it
            self.saved_geometry = self.saveGeometry()
            self.hide()
            # Give the window manager a chance to actually unmap us, otherwise
            # we end up in our own screenshot
            delay_ms = max(delay_ms, HIDE_SETTLE_MS)

        QTimer.singleShot(delay_ms, lambda: self.capture(selected))

    def capture(self, entries: list[ScreenEntry]) -> None:
        stamp = "{date:%Y-%m-%d_%H-%M-%S-%f}".format(date=datetime.datetime.now())

        saved = []
        failed = []
        for entry in entries:
            path = self.output_dir / f"{entry.file_stem()}_{stamp}.png"
            if entry.grab().save(str(path), "PNG"):
                saved.append(path.name)
            else:
                failed.append(path.name)

        if self.isHidden():
            self.show()
            if self.saved_geometry is not None:
                self.restoreGeometry(self.saved_geometry)
        self.raise_()
        self.activateWindow()

        message = f"Saved {len(saved)}: {', '.join(saved)}" if saved else "Saved nothing"
        if failed:
            message += f"  |  failed: {', '.join(failed)}"
        self.status.setText(message)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, help="folder to save the screenshots into")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyleSheet(TOOLTIP_STYLE)

    output_dir = args.output_dir.expanduser()
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        QMessageBox.critical(None, "Screenshot", f"Cannot use {output_dir}: {e}")
        return 1

    window = ScreenshotWindow(output_dir.resolve())
    window.show()

    # Qt's event loop never returns to the interpreter on its own, so a plain
    # ctrl-c in the terminal would go unnoticed until the next click
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    heartbeat = QTimer()
    heartbeat.timeout.connect(lambda: None)
    heartbeat.start(SIGNAL_POLL_MS)

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
