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
    QScrollArea,
    QVBoxLayout,
    QWidget,
)
from src.ui.visuals import CameraPreview, line_icon
from src.ui.help_text import CAMERA_MODE_HELP, SECTION_HELP, STATUS_HELP, set_help

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

        self.setObjectName("RobotPage")
        self.setStyleSheet(f"QWidget#RobotPage {{ background-color: {CONTENT_BG}; }}")

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

    def add_workspace(self, layout, columns, title: str, description: str, tag: str, action=None) -> None:
        """Page identity stays visible while dense controls can scroll on small screens."""
        heading = QWidget(self)
        row = QHBoxLayout(heading)
        row.setContentsMargins(2, 2, 2, 8)
        text = QVBoxLayout()
        text.setSpacing(3)
        label = QLabel(title)
        label.setStyleSheet(f"font-size: 24px; font-weight: 600; color: {TEXT_PRIMARY};")
        text.addWidget(label)
        subtitle = QLabel(description)
        subtitle.setStyleSheet(f"font-size: 12px; color: {TEXT_SECONDARY};")
        for widget in (label, subtitle):
            set_help(widget, title, SECTION_HELP[title])
        text.addWidget(subtitle)
        row.addLayout(text)
        row.addStretch()
        badge = QLabel(tag)
        badge.setStyleSheet(f"background: {CARD_BG}; color: {TEXT_SECONDARY}; border: 1px solid {PANEL_BORDER}; border-radius: 6px; padding: 7px 12px; font-size: 10px; letter-spacing: 1px;")
        if action is None:
            set_help(badge, title, SECTION_HELP[title])
            row.addWidget(badge)
        else:
            badge.deleteLater()
            action.setFixedSize(200, 40)
            row.addWidget(action)
        layout.addWidget(heading)
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(columns)
        scroll.setMinimumHeight(180)
        layout.addWidget(scroll, 1)

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

        lbl_icon = QLabel(row)
        glyph = "camera" if "CAMERA" in title or "VISION" in title else "settings" if "TIMING" in title or "LIMIT" in title else "target"
        lbl_icon.setPixmap(line_icon(glyph, TEXT_SECONDARY, 17).pixmap(17, 17))
        row_layout.addWidget(lbl_icon)

        lbl_title = QLabel(title, row)
        for widget in (header_widget, lbl_title, lbl_icon):
            set_help(widget, title, SECTION_HELP[title])
        lbl_title.setStyleSheet(
            f"color: {TEXT_PRIMARY}; font-size: 11px; font-weight: 600; "
            f"letter-spacing: 1px; border: none; background: transparent;"
        )
        row_layout.addWidget(lbl_title)
        row_layout.addStretch()

        layout.addWidget(row)

        # Gradient underline
        underline = QFrame(header_widget)
        underline.setFixedHeight(1)
        underline.setStyleSheet(
            f"background-color: {PANEL_BORDER}; border: none;"
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
        row_frame.setStyleSheet(f"QFrame {{ background: {ROW_BG}; border: none; border-radius: 6px; }}")
        row_layout = QHBoxLayout(row_frame)
        row_layout.setContentsMargins(12, 9, 12, 9)

        name_label = QLabel(name, row_frame)
        name_label.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 12px; font-weight: normal; "
            f"border: none; background: transparent;"
        )
        row_layout.addWidget(name_label)

        row_layout.addStretch()

        font_family = FONT_MONO if mono_font else "Segoe UI"
        color = TEXT_PRIMARY
        val_label = QLabel(value, row_frame)
        val_label.setStyleSheet(
            f"color: {color}; font-family: '{font_family}', monospace; "
            f"font-size: 18px; font-weight: normal; border: none; background: transparent;"
        )
        row_layout.addWidget(val_label)

        if name.startswith("J"):
            help_text = SECTION_HELP["JOINT STATUS"]
        elif name.startswith("θ"):
            help_text = "Góc khớp tính từ tọa độ X, Y, Z và lựa chọn Elbow, đơn vị độ (°). Nhấn Calculate để cập nhật. Send targets ghi J1–J3 và tọa độ đến PLC; J4 chỉ tham gia tính toán ở màn hình này."
        elif name == "Pick Z":
            help_text = "Độ cao gắp vật đang áp dụng, đơn vị mm. Có thể sửa Pick height ở Settings rồi nhấn Apply & save settings."
        else:
            help_text = f"Tọa độ {name} của đầu công tác tính từ các góc J1–J4, đơn vị mm. Nhấn Calculate để cập nhật. Dấu --- nghĩa là chưa có kết quả hợp lệ."
        for widget in (row_frame, name_label, val_label):
            set_help(widget, name, help_text)

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
        self.build_section_header(frame, "", "CAMERA PREVIEW", title_color)

        # Camera selector row
        selector_widget = QWidget(frame)
        selector_widget.setStyleSheet("background: transparent; border: none;")
        sel_layout = QHBoxLayout(selector_widget)
        sel_layout.setContentsMargins(15, 0, 15, 0)
        sel_layout.setSpacing(8)

        # Live indicator dot
        lbl_live = QLabel("●", selector_widget)
        self._camera_dot = lbl_live
        lbl_live.setStyleSheet(
            f"color: {SUCCESS}; font-size: 14px; border: none; background: transparent;"
        )
        sel_layout.addWidget(lbl_live)

        lbl_src = QLabel("STANDBY", selector_widget)
        self._camera_state = lbl_src
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
        if self.controller:
            default_camera = f"Camera {self.controller.cfg.camera.default_index}"
            if default_camera in cam_values:
                cam_selector.setCurrentText(default_camera)
        cam_selector.setFixedSize(130, 28)
        for widget in (lbl_source, cam_selector):
            set_help(widget, "Nguồn camera", "Chọn camera theo số thứ tự thiết bị. Đổi nguồn sẽ chuyển camera đang dùng; nếu không có hình, kiểm tra kết nối và thử nguồn khác.")
        for widget in (lbl_live, lbl_src):
            set_help(widget, "Trạng thái camera", SECTION_HELP["CAMERA PREVIEW"])
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

        self.video_label = CameraPreview(video_container)
        set_help(self.video_label, "Ảnh camera", SECTION_HELP["CAMERA PREVIEW"] + " " + CAMERA_MODE_HELP)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setText("Waiting for camera")
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

        mode = getattr(getattr(getattr(self.controller, "cfg", None), "app", None), "sorting_mode", "vision")
        lbl_model = QLabel("QR READER" if mode == "qr" else "YOLO DETECTION", info_row)
        self._camera_mode = lbl_model
        lbl_model.setStyleSheet(
            f"color: {ACCENT_DIM}; font-size: 10px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        info_layout.addWidget(lbl_model)
        info_layout.addStretch()

        lbl_res = QLabel("Full-frame decoding" if mode == "qr" else "Region of interest", info_row)
        self._camera_detail = lbl_res
        for widget in (lbl_model, lbl_res):
            set_help(widget, "Chế độ xử lý ảnh", CAMERA_MODE_HELP)
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
        frame_bottom.setFixedHeight(40)
        frame_bottom.setStyleSheet(
            f"background-color: {HEADER_BG}; "
            f"border: 1px solid {PANEL_BORDER}; border-radius: 12px;"
        )
        layout = QHBoxLayout(frame_bottom)
        layout.setContentsMargins(16, 4, 8, 4)
        layout.setSpacing(15)

        self.lbl_err_status = QLabel("● WAITING FOR PLC STATUS", frame_bottom)
        set_help(self.lbl_err_status, "Trạng thái hệ thống", STATUS_HELP)
        self.lbl_err_status.setStyleSheet(
            f"color: {SUCCESS}; font-size: 12px; font-weight: bold; "
            f"letter-spacing: 0.5px; border: none; background: transparent;"
        )
        layout.addWidget(self.lbl_err_status)
        layout.addStretch()

        # Clear Error button
        btn_clear = QPushButton("Clear error", frame_bottom)
        set_help(btn_clear, "Xác nhận và xóa lỗi", "Sau khi xử lý nguyên nhân, nhấn để yêu cầu xóa lỗi và khôi phục trạng thái điều khiển. Ứng dụng chỉ đặt lại khi trạng thái PLC cho phép; xem Activity log nếu chưa xóa được.")
        btn_clear.setStyleSheet(
            btn_style(
                bg="transparent",
                hover=DANGER,
                text=DANGER,
                radius=8,
                border=f"1px solid {PANEL_BORDER}",
                padding="4px 12px",
            )
        )
        if self.controller and hasattr(self.controller, "clear_all_errors"):
            btn_clear.clicked.connect(self.controller.clear_all_errors)
        layout.addWidget(btn_clear)

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
        if hasattr(self, "_camera_state"):
            active = data.get("camera_active", False)
            self._camera_state.setText("LIVE" if active else "STANDBY")
            for label in (self._camera_state, self._camera_dot):
                label.setStyleSheet(f"color: {SUCCESS if active else TEXT_DIM}; font-size: 10px; font-weight: 600; background: transparent; border: none;")
            mode = self.controller.cfg.app.sorting_mode if self.controller else "vision"
            self._camera_mode.setText("QR READER" if mode == "qr" else "YOLO DETECTION")
            self._camera_detail.setText("Full-frame decoding" if mode == "qr" else "Region of interest")
            if "camera_active" in data and not active and self._current_pixmap is not None:
                self._current_pixmap = None
                self.video_label.clear()
        if self.lbl_err_status is None:
            return
        sorter = getattr(self.controller, "sorter", None)
        local_error = getattr(getattr(sorter, "state", None), "name", "") == "ERROR"
        if data.get("connected") is False or not data:
            text, color = "● PLC OFFLINE — STATUS UNAVAILABLE", DANGER
        elif data.get("error_flag", False) or local_error:
            text, color = "⚠  WARNING: SYSTEM ERROR!", DANGER
        elif data.get("stop_robot", False):
            text, color = "● STOP ACTIVE", DANGER
        else:
            text, color = "● SYSTEM STABLE", SUCCESS
        self.lbl_err_status.setText(text)
        self.lbl_err_status.setStyleSheet(
            f"color: {color}; font-size: 12px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
