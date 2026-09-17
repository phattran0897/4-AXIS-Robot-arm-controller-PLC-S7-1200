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

from src.ui.theme import DANGER, SUCCESS, TEXT_SECONDARY, btn_style

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
    return os.path.join(os.path.dirname(__file__), "..", "..", "assets", "vaa_logo.png")


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
    """
    Professional header with VAA branding and navigation tabs.

    Layout:
    ┌─────────────────────────────────────────────────────────────────┐
    │ [LOGO]  HỌC VIỆN HÀNG KHÔNG VIỆT NAM     [AUTO] [MANUAL]        │
    │         4-AXIS ROBOT CONTROL SYSTEM                             │
    └─────────────────────────────────────────────────────────────────┘
    """

    def __init__(
        self,
        parent: QWidget | None = None,
        controller: "RobotApp" | None = None,
        current_page: str = "PageAuto",
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.setFixedHeight(78)
        self.setStyleSheet(f"background-color: {VAA_PRIMARY};")

        self._build_layout(current_page)

    def _build_layout(self, current_page: str) -> None:
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(15, 8, 20, 8)
        main_layout.setSpacing(15)

        # ── Left section: Logo + Typography ──────────────────────────────────
        left_widget = QWidget(self)
        left_layout = QHBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(15)

        # Logo image (optional asset)
        logo_path = get_logo_path()
        logo_loaded = False
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            if not pixmap.isNull():
                scaled = pixmap.scaledToHeight(
                    50, Qt.TransformationMode.SmoothTransformation
                )
                logo_label = QLabel(left_widget)
                logo_label.setPixmap(scaled)
                left_layout.addWidget(logo_label)
                logo_loaded = True

        if not logo_loaded:
            # Accent bar fallback placeholder
            bar = QFrame(left_widget)
            bar.setFixedSize(6, 54)
            bar.setStyleSheet(
                f"background-color: {VAA_ACCENT}; border-radius: 3px; border: none;"
            )
            left_layout.addWidget(bar)

        # Text branding
        text_widget = QWidget(left_widget)
        text_layout = QVBoxLayout(text_widget)
        text_layout.setContentsMargins(0, 2, 0, 2)
        text_layout.setSpacing(2)

        lbl_title = QLabel("HỌC VIỆN HÀNG KHÔNG VIỆT NAM", text_widget)
        lbl_title.setStyleSheet(
            f"color: {VAA_ACCENT}; font-size: 16px; font-weight: bold; border: none;"
        )
        text_layout.addWidget(lbl_title)

        subtitle_widget = QWidget(text_widget)
        sub_layout = QHBoxLayout(subtitle_widget)
        sub_layout.setContentsMargins(0, 0, 0, 0)
        sub_layout.setSpacing(4)

        lbl_sub1 = QLabel("VIETNAM AVIATION ACADEMY", subtitle_widget)
        lbl_sub1.setStyleSheet(
            f"color: {VAA_TEXT_DIM}; font-size: 10px; font-weight: bold; border: none;"
        )
        sub_layout.addWidget(lbl_sub1)

        lbl_pipe = QLabel(" | ", subtitle_widget)
        lbl_pipe.setStyleSheet("color: #334155; font-size: 10px; border: none;")
        sub_layout.addWidget(lbl_pipe)

        lbl_sub2 = QLabel("4-AXIS INDUSTRIAL ROBOT CONTROL SYSTEM", subtitle_widget)
        lbl_sub2.setStyleSheet(
            f"color: {VAA_TEXT}; font-size: 11px; font-weight: normal; border: none;"
        )
        sub_layout.addWidget(lbl_sub2)
        sub_layout.addStretch()

        text_layout.addWidget(subtitle_widget)
        left_layout.addWidget(text_widget)
        left_layout.addStretch()
        main_layout.addWidget(left_widget)

        # ── Right section: Navigation tabs ───────────────────────────────────
        tabs_widget = QWidget(self)
        tabs_layout = QHBoxLayout(tabs_widget)
        tabs_layout.setContentsMargins(0, 0, 0, 0)
        tabs_layout.setSpacing(8)

        self._btn_auto = QPushButton("AUTO MODE", tabs_widget)
        self._btn_auto.setFixedSize(126, 38)
        self._btn_auto.clicked.connect(
            lambda: self.controller.show_frame("PageAuto") if self.controller else None
        )
        # Compatibility attribute for tests
        setattr(self._btn_auto, "cget", lambda k: self._btn_auto.text() if k == "text" else "")

        self._btn_manual = QPushButton("MANUAL MODE", tabs_widget)
        self._btn_manual.setFixedSize(126, 38)
        self._btn_manual.clicked.connect(
            lambda: self.controller.show_frame("PageManual") if self.controller else None
        )
        setattr(self._btn_manual, "cget", lambda k: self._btn_manual.text() if k == "text" else "")

        tabs_layout.addWidget(self._btn_auto)
        tabs_layout.addWidget(self._btn_manual)
        main_layout.addWidget(tabs_widget)

        # Apply initial highlight
        self.update_active_tab(current_page)

    def update_active_tab(self, page_name: str) -> None:
        """Update tab highlighting based on current page (active tab glows cyan)."""
        active_css = btn_style(
            bg=VAA_ACCENT, hover=VAA_ACCENT, text=VAA_PRIMARY, radius=9, font_size=12
        )
        inactive_css = btn_style(
            bg=VAA_SECONDARY, hover="#274B75", text=VAA_TEXT, radius=9, font_size=12
        )

        if page_name == "PageAuto":
            self._btn_auto.setStyleSheet(active_css)
            self._btn_manual.setStyleSheet(inactive_css)
        else:
            self._btn_manual.setStyleSheet(active_css)
            self._btn_auto.setStyleSheet(inactive_css)


class VAAFooter(QWidget):
    """
    Professional footer with status indicators and version info.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedHeight(40)
        self.setStyleSheet(f"background-color: {VAA_PRIMARY};")

        self._build_layout()

    @staticmethod
    def _chip(text: str, color: str = DANGER) -> QLabel:
        """Create one status chip label."""
        lbl = QLabel(text)
        lbl.setStyleSheet(
            f"color: {color}; font-size: 11px; font-weight: bold; border: none;"
        )
        return lbl

    def _build_layout(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 0, 20, 0)
        layout.setSpacing(20)

        # Left status indicators
        self.lbl_status = self._chip("● SYSTEM READY", SUCCESS)
        layout.addWidget(self.lbl_status)

        self.lbl_plc = self._chip("PLC: DISCONNECTED", DANGER)
        layout.addWidget(self.lbl_plc)

        self.lbl_camera = self._chip("CAM: OFFLINE", DANGER)
        layout.addWidget(self.lbl_camera)

        layout.addStretch()

        # Right version label
        lbl_version = QLabel("v2.0.0 | VAA Robot System", self)
        lbl_version.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; border: none;"
        )
        layout.addWidget(lbl_version)

    def update_status(
        self, plc_connected: bool, camera_active: bool, system_ok: bool
    ) -> None:
        """Update footer status indicators."""
        if system_ok:
            self.lbl_status.setText("● SYSTEM READY")
            self.lbl_status.setStyleSheet(
                f"color: {SUCCESS}; font-size: 11px; font-weight: bold; border: none;"
            )
        else:
            self.lbl_status.setText("● SYSTEM ERROR")
            self.lbl_status.setStyleSheet(
                f"color: {DANGER}; font-size: 11px; font-weight: bold; border: none;"
            )

        if plc_connected:
            self.lbl_plc.setText("PLC: CONNECTED")
            self.lbl_plc.setStyleSheet(
                f"color: {SUCCESS}; font-size: 11px; font-weight: bold; border: none;"
            )
        else:
            self.lbl_plc.setText("PLC: DISCONNECTED")
            self.lbl_plc.setStyleSheet(
                f"color: {DANGER}; font-size: 11px; font-weight: bold; border: none;"
            )

        if camera_active:
            self.lbl_camera.setText("CAM: ACTIVE")
            self.lbl_camera.setStyleSheet(
                f"color: {SUCCESS}; font-size: 11px; font-weight: bold; border: none;"
            )
        else:
            self.lbl_camera.setText("CAM: OFFLINE")
            self.lbl_camera.setStyleSheet(
                f"color: {DANGER}; font-size: 11px; font-weight: bold; border: none;"
            )
