"""
src/ui/base_page.py – Abstract base class for all application pages (PySide6).

Every page (Auto / Manual) inherits :class:`BasePage`, which wires up the
controller reference and exposes shared helpers: card + section-header
factories, status-bar updates, camera-selector construction, and the
``update_gui_data`` contract.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from src.ui.theme import (
    ACCENT,
    ACCENT_DIM,
    ACCENT_GLOW,
    CARD_BG,
    CONTENT_BG,
    DANGER,
    FONT_MONO,
    HEADER_BG,
    PANEL_BORDER,
    PANEL_BORDER_LIGHT,
    ROW_BG,
    SUCCESS,
    TEXT_DIM,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    btn_style,
    card_style,
    video_container_style,
)

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


class BasePage(QWidget):
    """
    Shared base widget for every page in the application.

    Sub-classes must implement :meth:`update_gui_data`.

    Parameters
    ----------
    parent:
        The container widget managed by :class:`RobotApp`.
    controller:
        The main application instance.
    page_color:
        Accent colour for page-specific title labels (hex string).
    """

    def __init__(
        self,
        parent: QWidget | None = None,
        controller: "RobotApp" | None = None,
        page_color: str = "#00D4FF",
    ) -> None:
        super().__init__(parent)
        self.controller: "RobotApp" = controller  # type: ignore[assignment]
        self.page_color: str = page_color

        # Shared widgets populated by sub-class build helpers
        self.video_label: QLabel | None = None
        self.lbl_err_status: QLabel | None = None

        # Proper reference to prevent pixmap garbage collection
        self._current_pixmap: QPixmap | None = None

        # Dynamic video display size (updated on container resize)
        self._video_display_width: int = 480
        self._video_display_height: int = 360

        self.setStyleSheet(f"background-color: {CONTENT_BG};")

    # ------------------------------------------------------------------
    # Contract
    # ------------------------------------------------------------------

    @abstractmethod
    def update_gui_data(self, data: dict[str, Any]) -> None:
        """
        Refresh dynamic widgets with *data* from the PLC cyclic read.

        Called on the main Qt event thread via signal/slot.
        """

    # ------------------------------------------------------------------
    # Video label (thread-safe update via main thread slot)
    # ------------------------------------------------------------------

    def update_video(self, img: QImage | QPixmap | Any) -> None:
        """
        Replace current video frame with *img*.
        """
        if isinstance(img, QImage):
            pixmap = QPixmap.fromImage(img)
        elif isinstance(img, QPixmap):
            pixmap = img
        else:
            pixmap = img

        self._current_pixmap = pixmap

        if self.video_label is not None:
            if hasattr(pixmap, "scaled"):
                # Scale keeping aspect ratio to fill the current label size
                target_w = max(self.video_label.width() - 4, 10)
                target_h = max(self.video_label.height() - 4, 10)
                scaled = pixmap.scaled(
                    target_w,
                    target_h,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            else:
                scaled = pixmap

            if hasattr(self.video_label, "setPixmap"):
                self.video_label.setPixmap(scaled)

    # ------------------------------------------------------------------
    # Shared UI factory helpers
    # ------------------------------------------------------------------

    def _create_card(self, parent: QWidget | None = None) -> QFrame:
        """Create a modern bordered card frame."""
        frame = QFrame(parent or self)
        frame.setStyleSheet(card_style(CARD_BG, PANEL_BORDER, 14))
        return frame

    def build_section_header(
        self,
        parent: QWidget,
        icon: str,
        title: str,
        color: str,
    ) -> QWidget:
        """
        Build the standard section header row (icon + title + underline)
        and return the container widget.
        """
        header_widget = QWidget(parent)
        header_widget.setStyleSheet("background: transparent; border: none;")
        layout = QVBoxLayout(header_widget)
        layout.setContentsMargins(15, 12, 15, 6)
        layout.setSpacing(6)

        row = QWidget(header_widget)
        row.setStyleSheet("background: transparent; border: none;")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(10)

        lbl_icon = QLabel(icon, row)
        lbl_icon.setStyleSheet(
            f"color: {color}; font-size: 18px; border: none; background: transparent;"
        )
        row_layout.addWidget(lbl_icon)

        lbl_title = QLabel(title, row)
        lbl_title.setStyleSheet(
            f"color: {color}; font-size: 13px; font-weight: bold; "
            f"letter-spacing: 1px; border: none; background: transparent;"
        )
        row_layout.addWidget(lbl_title)
        row_layout.addStretch()

        layout.addWidget(row)

        # Gradient underline
        underline = QFrame(header_widget)
        underline.setFixedHeight(2)
        underline.setStyleSheet(
            f"background-color: {color}; border-radius: 1px; border: none;"
        )
        layout.addWidget(underline)

        if parent.layout() is not None:
            parent.layout().addWidget(header_widget)

        return header_widget

    def create_data_row(
        self,
        parent: QWidget,
        name: str,
        value: str,
        value_color: str | None = None,
        mono_font: bool = True,
    ) -> QLabel:
        """
        Create a styled label/value row and return the value label.
        """
        row_frame = QFrame(parent)
        row_frame.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        row_layout = QHBoxLayout(row_frame)
        row_layout.setContentsMargins(16, 12, 16, 12)

        name_label = QLabel(name, row_frame)
        name_label.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 13px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        row_layout.addWidget(name_label)

        row_layout.addStretch()

        font_family = FONT_MONO if mono_font else "Segoe UI"
        color = value_color or TEXT_PRIMARY
        val_label = QLabel(value, row_frame)
        val_label.setStyleSheet(
            f"color: {color}; font-family: '{font_family}', monospace; "
            f"font-size: 16px; font-weight: bold; border: none; background: transparent;"
        )
        row_layout.addWidget(val_label)

        if parent.layout() is not None:
            parent.layout().addWidget(row_frame)

        return val_label

    def build_camera_column(
        self,
        parent: QWidget,
        column: int = 2,
        title_color: str = "#00E5FF",
    ) -> QFrame:
        """
        Build the camera column (card frame + selector + video label).

        The video container fills all available vertical space so the
        camera feed is always maximised within the column.
        """
        frame = self._create_card(parent)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 8, 8, 12)
        layout.setSpacing(6)

        # Section header
        self.build_section_header(frame, "📷", "AI VISION", title_color)

        # Camera selector row
        selector_widget = QWidget(frame)
        selector_widget.setStyleSheet("background: transparent; border: none;")
        sel_layout = QHBoxLayout(selector_widget)
        sel_layout.setContentsMargins(15, 0, 15, 0)
        sel_layout.setSpacing(8)

        # Live indicator dot
        lbl_live = QLabel("●", selector_widget)
        lbl_live.setStyleSheet(
            f"color: {SUCCESS}; font-size: 14px; border: none; background: transparent;"
        )
        sel_layout.addWidget(lbl_live)

        lbl_src = QLabel("LIVE", selector_widget)
        lbl_src.setStyleSheet(
            f"color: {SUCCESS}; font-size: 11px; font-weight: bold; "
            f"letter-spacing: 1px; border: none; background: transparent;"
        )
        sel_layout.addWidget(lbl_src)
        sel_layout.addStretch()

        lbl_source = QLabel("Source:", selector_widget)
        lbl_source.setStyleSheet(
            f"color: {TEXT_DIM}; font-size: 10px; border: none; background: transparent;"
        )
        sel_layout.addWidget(lbl_source)

        # Available cameras
        available = (
            self.controller.detector.available_cameras(max_index=2)
            if self.controller and hasattr(self.controller, "detector")
            else [0]
        )
        cam_values = [f"Camera {i}" for i in available] if available else ["No Camera"]

        cam_selector = QComboBox(selector_widget)
        cam_selector.addItems(cam_values)
        cam_selector.setFixedSize(130, 28)
        if self.controller and hasattr(self.controller, "change_camera_source"):
            cam_selector.currentTextChanged.connect(self.controller.change_camera_source)
        sel_layout.addWidget(cam_selector)
        layout.addWidget(selector_widget)

        # ── Video container (expands to fill) ────────────────────────────
        video_container = QFrame(frame)
        video_container.setStyleSheet(video_container_style(ACCENT_DIM))
        video_container.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        v_layout = QVBoxLayout(video_container)
        v_layout.setContentsMargins(3, 3, 3, 3)

        self.video_label = QLabel(video_container)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setText("🎥  Initializing AI Vision...")
        self.video_label.setMinimumSize(320, 200)
        self.video_label.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored
        )
        self.video_label.setStyleSheet(
            f"color: {TEXT_DIM}; font-size: 13px; "
            f"background-color: #000000; border: none; border-radius: 10px;"
        )
        v_layout.addWidget(self.video_label)

        # Video container takes all remaining vertical space
        layout.addWidget(video_container, 1)

        # ── Bottom info row ──────────────────────────────────────────────
        info_row = QWidget(frame)
        info_row.setStyleSheet("background: transparent; border: none;")
        info_layout = QHBoxLayout(info_row)
        info_layout.setContentsMargins(15, 2, 15, 0)
        info_layout.setSpacing(4)

        lbl_model = QLabel("● YOLO", info_row)
        lbl_model.setStyleSheet(
            f"color: {ACCENT_DIM}; font-size: 10px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        info_layout.addWidget(lbl_model)
        info_layout.addStretch()

        lbl_res = QLabel("ROI Detection", info_row)
        lbl_res.setStyleSheet(
            f"color: {TEXT_DIM}; font-size: 10px; border: none; background: transparent;"
        )
        info_layout.addWidget(lbl_res)
        layout.addWidget(info_row)

        if parent.layout() is not None:
            parent.layout().addWidget(frame)

        return frame

    def build_status_bar(self) -> QLabel:
        """
        Build the bottom status bar common to all pages.
        """
        frame_bottom = QFrame(self)
        frame_bottom.setFixedHeight(50)
        frame_bottom.setStyleSheet(
            f"background-color: {HEADER_BG}; "
            f"border: 1px solid {PANEL_BORDER}; border-radius: 12px;"
        )
        layout = QHBoxLayout(frame_bottom)
        layout.setContentsMargins(20, 8, 20, 8)
        layout.setSpacing(15)

        self.lbl_err_status = QLabel("● SYSTEM STABLE", frame_bottom)
        self.lbl_err_status.setStyleSheet(
            f"color: {SUCCESS}; font-size: 12px; font-weight: bold; "
            f"letter-spacing: 0.5px; border: none; background: transparent;"
        )
        layout.addWidget(self.lbl_err_status)

        # Clear Error button
        btn_clear = QPushButton("⚠  Clear Error", frame_bottom)
        btn_clear.setStyleSheet(
            btn_style(
                bg="transparent",
                hover=DANGER,
                text=DANGER,
                radius=8,
                border=f"1px solid {DANGER}",
                padding="4px 12px",
            )
        )
        if self.controller and hasattr(self.controller, "clear_all_errors"):
            btn_clear.clicked.connect(self.controller.clear_all_errors)
        layout.addWidget(btn_clear)
        layout.addStretch()

        if self.layout() is not None:
            self.layout().addWidget(frame_bottom)

        return self.lbl_err_status

    # ------------------------------------------------------------------
    # Shared status update (called by sub-classes)
    # ------------------------------------------------------------------

    def resizeEvent(self, event) -> None:
        """Update display sizes when window resizes."""
        super().resizeEvent(event)
        # Redraw the current frame immediately upon resize
        if self._current_pixmap is not None:
            self.update_video(self._current_pixmap)

    def _refresh_error_status(self, data: dict[str, Any]) -> None:
        """Update the error status label shared by every page."""
        if self.lbl_err_status is None:
            return
        if data.get("error_flag", False):
            self.lbl_err_status.setText("⚠  WARNING: SYSTEM ERROR!")
            self.lbl_err_status.setStyleSheet(
                f"color: {DANGER}; font-size: 12px; font-weight: bold; "
                f"border: none; background: transparent;"
            )
        else:
            self.lbl_err_status.setText("● SYSTEM STABLE")
            self.lbl_err_status.setStyleSheet(
                f"color: {SUCCESS}; font-size: 12px; font-weight: bold; "
                f"border: none; background: transparent;"
            )
