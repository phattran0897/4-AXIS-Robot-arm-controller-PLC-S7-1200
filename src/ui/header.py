"""
src/ui/header.py – Modern professional header with VAA branding (PySide6).

Provides a consistent header and footer component across all pages.
"""

from __future__ import annotations

import logging
import os
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QIcon, QPixmap
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from src.ui.theme import ACCENT, HEADER_BG, PANEL_BORDER, ROW_BG, SUCCESS, TEXT_DIM, TEXT_PRIMARY, TEXT_SECONDARY, WARNING, btn_style
from src.ui.visuals import line_icon
from src.ui.help_text import SECTION_HELP, STATUS_HELP, set_help

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)

# Brand colors
VAA_PRIMARY: str = "#0A1628"  # Deep navy
VAA_ACCENT: str = "#00D4FF"  # Cyan accent
VAA_SECONDARY: str = "#1E3A5F"  # Medium navy
VAA_TEXT: str = "#FFFFFF"  # White text
VAA_TEXT_DIM: str = "#94A3B8"  # Dimmed text


def get_logo_path() -> str:
    """Filesystem path of the brand logo asset (project-relative)."""
    import sys
    root = os.path.dirname(sys.executable) if getattr(sys, "frozen", False) else os.path.join(os.path.dirname(__file__), "..", "..")
    return os.path.join(root, "assets", "vaa_logo.png")


def apply_app_icon(window: Any, path: str | None = None) -> bool:
    """
    Register the brand logo as the OS window / taskbar icon.

    Parameters
    ----------
    window:
        The Qt window or widget.
    path:
        Explicit icon image path; defaults to :func:`get_logo_path`.

    Returns
    -------
    bool
        ``True`` when the icon was applied.
    """
    icon_path = path or get_logo_path()
    try:
        icon = QIcon(icon_path)
        if not icon.isNull():
            if hasattr(window, "setWindowIcon"):
                window.setWindowIcon(icon)
            log.info("App icon set from %s", icon_path)
            return True
        return False
    except Exception as exc:  # Missing/corrupt asset must never block start-up
        log.debug("App icon not applied (%s): %s", icon_path, exc)
        return False


class VAAHeader(QWidget):
    """Compact brand bar and segmented workspace navigation."""

    def __init__(self, parent=None, controller=None, current_page="PageAuto"):
        super().__init__(parent)
        self.controller = controller
        self.setObjectName("AppHeader")
        self.setFixedHeight(72)
        self.setStyleSheet(f"QWidget#AppHeader {{ background: {HEADER_BG}; border-bottom: 1px solid {PANEL_BORDER}; }}")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 12, 24, 12)
        layout.setSpacing(14)

        logo = QLabel(self)
        logo.setFixedSize(42, 42)
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo.setStyleSheet("background: #E8F0F5; border-radius: 10px; padding: 5px;")
        pixmap = QPixmap(get_logo_path())
        if not pixmap.isNull():
            logo.setPixmap(pixmap.scaled(32, 32, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        else:
            logo.setText("VAA")
        layout.addWidget(logo)
        brand = QVBoxLayout()
        brand.setSpacing(2)
        title = QLabel("VAA  /  ROBOTICS", self)
        title.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 15px; font-weight: 600; letter-spacing: 2px;")
        brand.addWidget(title)
        subtitle = QLabel("HỌC VIỆN HÀNG KHÔNG VIỆT NAM", self)
        subtitle.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 10px; letter-spacing: 1px;")
        brand.addWidget(subtitle)
        layout.addLayout(brand)
        layout.addStretch()

        tabs = QFrame(self)
        tabs.setStyleSheet(f"QFrame {{ background: {ROW_BG}; border: 1px solid {PANEL_BORDER}; border-radius: 9px; }}")
        tab_layout = QHBoxLayout(tabs)
        tab_layout.setContentsMargins(4, 4, 4, 4)
        tab_layout.setSpacing(4)
        self._buttons = {}
        for name, text, icon in (("PageAuto", "AUTO MODE", "play"), ("PageManual", "MANUAL MODE", "target"), ("PageSettings", "SETTINGS", "settings")):
            button = QPushButton(text, tabs)
            title = {"PageAuto": "Automation overview", "PageManual": "Manual workspace", "PageSettings": "System configuration"}[name]
            set_help(button, text, SECTION_HELP[title])
            button.setFixedSize(132, 34)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.clicked.connect(lambda checked=False, page=name: self.controller.show_frame(page) if self.controller else None)
            button.setProperty("iconName", icon)
            self._buttons[name] = button
            tab_layout.addWidget(button)
        self._btn_auto = self._buttons["PageAuto"]
        self._btn_manual = self._buttons["PageManual"]
        self._btn_settings = self._buttons["PageSettings"]
        layout.addWidget(tabs)
        self.update_active_tab(current_page)

    def update_active_tab(self, page_name):
        for name, button in self._buttons.items():
            active = name == page_name
            button.setStyleSheet(btn_style(
                bg="#243D42" if active else "transparent", hover="#233345",
                text=ACCENT if active else TEXT_SECONDARY, radius=6, font_size=11,
                padding="4px 8px", border="1px solid #31564F" if active else "1px solid transparent"))
            button.setIcon(line_icon(button.property("iconName"), ACCENT if active else TEXT_SECONDARY, 16))


class VAAFooter(QWidget):
    """Persistent, compact connection indicators."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("AppFooter")
        self.setFixedHeight(34)
        self.setStyleSheet(f"QWidget#AppFooter {{ background: {HEADER_BG}; border-top: 1px solid {PANEL_BORDER}; }}")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(24, 0, 24, 0)
        layout.setSpacing(24)
        self.lbl_status, self.lbl_plc, self.lbl_camera = QLabel(), QLabel(), QLabel()
        set_help(self.lbl_status, "Trạng thái tổng quát", STATUS_HELP)
        set_help(self.lbl_plc, "Kết nối PLC", "CONNECTED: ứng dụng đang kết nối PLC. OFFLINE: không có kết nối hoặc chưa đọc được trạng thái; kiểm tra nguồn PLC, cáp mạng và cấu hình địa chỉ.")
        set_help(self.lbl_camera, "Kết nối camera", "ACTIVE: camera đang hoạt động. OFFLINE: camera chưa mở hoặc bị ngắt; kiểm tra thiết bị và chọn nguồn ở Source.")
        for label in (self.lbl_status, self.lbl_plc, self.lbl_camera):
            layout.addWidget(label)
        layout.addStretch()
        version = QLabel("4 AXIS  /  S7-1200     ·     VAA ROBOT SYSTEM")
        version.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; letter-spacing: 1px;")
        layout.addWidget(version)
        self.update_status(False, False, False)

    def update_status(self, plc_connected, camera_active, system_ok):
        state = "SYSTEM READY" if system_ok else ("SYSTEM OFFLINE" if not plc_connected else "ATTENTION REQUIRED")
        for label, text, color in (
            (self.lbl_status, state, SUCCESS if system_ok else WARNING),
            (self.lbl_plc, "PLC CONNECTED" if plc_connected else "PLC OFFLINE", SUCCESS if plc_connected else TEXT_DIM),
            (self.lbl_camera, "CAMERA ACTIVE" if camera_active else "CAMERA OFFLINE", SUCCESS if camera_active else TEXT_DIM),
        ):
            label.setText("●  " + text)
            label.setStyleSheet(f"color: {color}; font-size: 10px; font-weight: 600; border: none; background: transparent;")
