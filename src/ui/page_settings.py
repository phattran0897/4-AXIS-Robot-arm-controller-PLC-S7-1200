"""
src/ui/page_settings.py – Application settings configuration page.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QDoubleValidator, QIntValidator
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    CARD_BG,
    ENTRY_BG,
    INFO,
    PANEL_BORDER,
    ROW_BG,
    SUCCESS,
    SUCCESS_HOVER,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    DANGER,
    btn_style,
    card_style,
)
from src.config_loader import save_config

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)

class PageSettings(BasePage):
    def __init__(
        self, parent: QWidget | None = None, controller: "RobotApp" | None = None
    ) -> None:
        super().__init__(parent, controller, page_color=ACCENT)
        self._inputs = {}
        self._build_ui()

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 12, 16, 12)
        main_layout.setSpacing(12)

        columns_widget = QWidget(self)
        col_layout = QHBoxLayout(columns_widget)
        col_layout.setContentsMargins(0, 0, 0, 0)
        col_layout.setSpacing(12)

        # Col 0: Vision & ROI Settings
        self._build_vision_panel(columns_widget)

        # Col 1: Robot & App Settings
        self._build_robot_panel(columns_widget)

        # Col 2: Camera Preview
        self.build_camera_column(columns_widget, column=2, title_color=ACCENT)

        col_layout.setStretch(0, 2)
        col_layout.setStretch(1, 2)
        col_layout.setStretch(2, 3)

        main_layout.addWidget(columns_widget, 1)
        self.lbl_err_status = self.build_status_bar()

    def _create_input_row(self, parent: QWidget, name: str, default_val: Any, key: str, is_int: bool = False) -> None:
        row_frame = QFrame(parent)
        row_frame.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        row_layout = QHBoxLayout(row_frame)
        row_layout.setContentsMargins(12, 7, 12, 7)

        name_label = QLabel(name, row_frame)
        name_label.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 11px; font-weight: bold; border: none; background: transparent;"
        )
        row_layout.addWidget(name_label)
        row_layout.addStretch()

        inp = QLineEdit(str(default_val), row_frame)
        inp.setFixedWidth(80)
        inp.setStyleSheet(
            f"background-color: {ENTRY_BG}; color: {TEXT_PRIMARY}; border: 1px solid {PANEL_BORDER}; border-radius: 4px; padding: 2px 4px;"
        )
        if is_int:
            inp.setValidator(QIntValidator(0, 9999))
        else:
            inp.setValidator(QDoubleValidator(-9999.0, 9999.0, 4))
            
        if key in ("yolo.roi_x", "yolo.roi_y", "yolo.roi_width", "yolo.roi_height"):
            inp.textChanged.connect(self._preview_roi_changed)

        row_layout.addWidget(inp)
        parent.layout().addWidget(row_frame)
        self._inputs[key] = (inp, is_int)

    def _create_combo_row(self, parent: QWidget, name: str, options: list[str], default_val: str, key: str) -> None:
        row_frame = QFrame(parent)
        row_frame.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        row_layout = QHBoxLayout(row_frame)
        row_layout.setContentsMargins(12, 7, 12, 7)

        name_label = QLabel(name, row_frame)
        name_label.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 11px; font-weight: bold; border: none; background: transparent;"
        )
        row_layout.addWidget(name_label)
        row_layout.addStretch()

        combo = QComboBox(row_frame)
        combo.addItems(options)
        combo.setCurrentText(default_val)
        combo.setStyleSheet(
            f"background-color: {ENTRY_BG}; color: {TEXT_PRIMARY}; border: 1px solid {PANEL_BORDER}; border-radius: 4px; padding: 2px 4px;"
        )
        combo.setFixedWidth(80)
        row_layout.addWidget(combo)
        parent.layout().addWidget(row_frame)
        self._inputs[key] = (combo, "combo")

    def _build_vision_panel(self, parent: QWidget) -> None:
        frame = self._create_card(parent)
        layout = QVBoxLayout(frame)
        self.build_section_header(frame, "👁", "VISION & ROI", ACCENT)

        cfg = self.controller.cfg.yolo
        self._create_input_row(frame, "YOLO Threshold", cfg.thresh, "yolo.thresh")
        self._create_input_row(frame, "PX to MM Scale", cfg.px2mm, "yolo.px2mm")
        
        self.build_section_header(frame, "⛶", "ROI BOX", INFO)
        layout.addStretch(1)
        
        self._create_input_row(frame, "ROI X", cfg.roi_x, "yolo.roi_x", True)
        self._create_input_row(frame, "ROI Y", cfg.roi_y, "yolo.roi_y", True)
        self._create_input_row(frame, "ROI Width", cfg.roi_width, "yolo.roi_width", True)
        self._create_input_row(frame, "ROI Height", cfg.roi_height, "yolo.roi_height", True)
        
        layout.addStretch(1)
        parent.layout().addWidget(frame)

    def _build_robot_panel(self, parent: QWidget) -> None:
        frame = self._create_card(parent)
        layout = QVBoxLayout(frame)
        self.build_section_header(frame, "⚙", "ROBOT LIMITS", INFO)
        layout.addStretch(1)
        
        cfg = self.controller.cfg
        self._create_input_row(frame, "Pick Z Down (mm)", cfg.sort_positions.pick_z_down, "sort_positions.pick_z_down")
        self._create_input_row(frame, "Pick Z Up (mm)", cfg.sort_positions.pick_z_up, "sort_positions.pick_z_up")
        self._create_input_row(frame, "Place Good Z Down", cfg.sort_positions.place_good.z_down, "place_good.z_down")
        self._create_input_row(frame, "Place Bad Z Down", cfg.sort_positions.place_bad.z_down, "place_bad.z_down")
        
        self.build_section_header(frame, "⏱", "TIMING", INFO)
        self._create_input_row(frame, "Move Cooldown (s)", cfg.app.move_cooldown, "app.move_cooldown")
        self._create_input_row(frame, "PLC Poll Rate (s)", cfg.app.plc_poll_interval, "app.plc_poll_interval")
        
        self.build_section_header(frame, "🔄", "SORTING", INFO)
        self._create_combo_row(frame, "Sorting Mode", ["vision", "qr"], cfg.app.sorting_mode, "app.sorting_mode")
        
        # Removed addStretch
        btn_save = QPushButton("💾 APPLY & SAVE")
        btn_save.setFixedHeight(46)
        btn_save.setStyleSheet(btn_style(SUCCESS, SUCCESS_HOVER, TEXT_PRIMARY, radius=8, font_size=16))
        btn_save.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_save.clicked.connect(self._save_settings)
        layout.addWidget(btn_save)
        
        layout.addStretch(1)

        parent.layout().addWidget(frame)

    def _preview_roi_changed(self) -> None:
        """Dynamically update the App config ROI so the live camera loop draws it."""
        try:
            x = int(self._inputs["yolo.roi_x"][0].text() or 0)
            y = int(self._inputs["yolo.roi_y"][0].text() or 0)
            w = int(self._inputs["yolo.roi_width"][0].text() or 0)
            h = int(self._inputs["yolo.roi_height"][0].text() or 0)
            
            if w > 0 and h > 0:
                self.controller.cfg.yolo.roi_x = x
                self.controller.cfg.yolo.roi_y = y
                self.controller.cfg.yolo.roi_width = w
                self.controller.cfg.yolo.roi_height = h
        except ValueError:
            pass

    def _save_settings(self) -> None:
        try:
            cfg = self.controller.cfg
            
            # Extract
            def get_val(key):
                inp, is_int = self._inputs[key]
                if is_int == "combo":
                    return inp.currentText()
                text = inp.text().replace(",", ".")
                if not text:
                    text = "0"
                return int(text) if is_int else float(text)

            cfg.yolo.thresh = get_val("yolo.thresh")
            cfg.yolo.px2mm = get_val("yolo.px2mm")
            
            cfg.yolo.roi_x = get_val("yolo.roi_x")
            cfg.yolo.roi_y = get_val("yolo.roi_y")
            cfg.yolo.roi_width = get_val("yolo.roi_width")
            cfg.yolo.roi_height = get_val("yolo.roi_height")

            cfg.sort_positions.pick_z_down = get_val("sort_positions.pick_z_down")
            cfg.sort_positions.pick_z_up = get_val("sort_positions.pick_z_up")
            cfg.sort_positions.place_good.z_down = get_val("place_good.z_down")
            cfg.sort_positions.place_bad.z_down = get_val("place_bad.z_down")

            cfg.app.move_cooldown = get_val("app.move_cooldown")
            cfg.app.plc_poll_interval = get_val("app.plc_poll_interval")
            cfg.app.sorting_mode = get_val("app.sorting_mode")

            save_config(cfg)
            
            # Reload YOLO threshold
            self.controller.detector.conf_thresh = cfg.yolo.thresh
            
            QMessageBox.information(self, "Success", "Settings saved successfully to config.yaml!")
            
        except Exception as e:
            log.error(f"Failed to save settings: {e}")
            QMessageBox.critical(self, "Error", f"Failed to save settings:\n{e}")
