"""
src/ui/page_settings.py – Application settings configuration page.
"""

from __future__ import annotations

import logging
from contextlib import nullcontext
from copy import deepcopy
from dataclasses import fields
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QIntValidator
from src.ui.validators import DecimalValidator
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
from src.ui.help_text import APPLY_HELP, SETTINGS_HELP, set_help
from src.ui.visuals import line_icon
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
        main_layout.setContentsMargins(20, 14, 20, 10)
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

        self.add_workspace(main_layout, columns_widget, 'System configuration', 'Tune vision and motion settings for your current installation.', 'CONFIGURATION', action=self.btn_apply)
        self.lbl_err_status = self.build_status_bar()

    def _create_input_row(self, parent: QWidget, name: str, default_val: Any, key: str, is_int: bool = False) -> None:
        row_frame = QFrame(parent)
        row_frame.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        row_layout = QHBoxLayout(row_frame)
        row_layout.setContentsMargins(12, 4, 12, 4)

        name_label = QLabel(name, row_frame)
        name_label.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 12px; font-weight: normal; border: none; background: transparent;"
        )
        row_layout.addWidget(name_label)
        row_layout.addStretch()

        inp = QLineEdit(str(default_val), row_frame)
        inp.setFixedWidth(92)
        inp.setMinimumHeight(30)
        inp.setAlignment(Qt.AlignmentFlag.AlignRight)
        inp.setStyleSheet(
            f"background-color: {ENTRY_BG}; color: {TEXT_PRIMARY}; border: 1px solid {PANEL_BORDER}; border-radius: 6px; padding: 4px 8px; font-size: 12px;"
        )
        if is_int:
            inp.setValidator(QIntValidator(0, 9999))
        else:
            inp.setValidator(DecimalValidator(-9999.0, 9999.0, 4))

        row_layout.addWidget(inp)
        parent.layout().addWidget(row_frame)
        self._inputs[key] = (inp, is_int)
        for widget in (row_frame, name_label, inp):
            set_help(widget, name, SETTINGS_HELP[key])
        inp.setAccessibleName(name)

    def _create_combo_row(self, parent: QWidget, name: str, options: list[str], default_val: str, key: str) -> None:
        row_frame = QFrame(parent)
        row_frame.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        row_layout = QHBoxLayout(row_frame)
        row_layout.setContentsMargins(12, 4, 12, 4)

        name_label = QLabel(name, row_frame)
        name_label.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 12px; font-weight: normal; border: none; background: transparent;"
        )
        row_layout.addWidget(name_label)
        row_layout.addStretch()

        combo = QComboBox(row_frame)
        combo.addItems(options)
        combo.setCurrentText(default_val)
        combo.setStyleSheet(
            f"background-color: {ENTRY_BG}; color: {TEXT_PRIMARY}; border: 1px solid {PANEL_BORDER}; border-radius: 6px; padding: 4px 8px; font-size: 12px;"
        )
        combo.setFixedWidth(92)
        combo.setMinimumHeight(30)
        row_layout.addWidget(combo)
        parent.layout().addWidget(row_frame)
        self._inputs[key] = (combo, "combo")
        for widget in (row_frame, name_label, combo):
            set_help(widget, name, SETTINGS_HELP[key])
        combo.setAccessibleName(name)

    def _build_vision_panel(self, parent: QWidget) -> None:
        frame = self._create_card(parent)
        layout = QVBoxLayout(frame)
        self.build_section_header(frame, "👁", "VISION & ROI", ACCENT)

        cfg = self.controller.cfg.yolo
        self._create_input_row(frame, "YOLO Threshold", cfg.thresh, "yolo.thresh")
        self._create_input_row(frame, "PX to MM Scale", cfg.px2mm, "yolo.px2mm")
        self._create_combo_row(frame, "Coordinate origin", ["roi", "frame"], cfg.coordinate_origin, "yolo.coordinate_origin")

        self.build_section_header(frame, "⛶", "ROI BOX", INFO)

        self._create_input_row(frame, "ROI X", cfg.roi_x, "yolo.roi_x", True)
        self._create_input_row(frame, "ROI Y", cfg.roi_y, "yolo.roi_y", True)
        self._create_input_row(frame, "ROI Width", cfg.roi_width, "yolo.roi_width", True)
        self._create_input_row(frame, "ROI Height", cfg.roi_height, "yolo.roi_height", True)
        note = QLabel("Changes take effect after APPLY & SAVE.\nThe camera shows the currently applied ROI.", frame)
        note.setWordWrap(True)
        set_help(note, "Áp dụng thay đổi", APPLY_HELP)
        note.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 11px; border: none; background: transparent;")
        layout.addWidget(note)

        layout.addStretch(1)
        parent.layout().addWidget(frame)

    def _build_robot_panel(self, parent: QWidget) -> None:
        frame = self._create_card(parent)
        layout = QVBoxLayout(frame)
        self.build_section_header(frame, "⚙", "ROBOT LIMITS", INFO)

        cfg = self.controller.cfg
        self._create_input_row(frame, "Pick height (mm)", cfg.sort_positions.pick_z_down, "sort_positions.pick_z_down")
        self._create_input_row(frame, "Approach height (mm)", cfg.sort_positions.pick_z_up, "sort_positions.pick_z_up")
        self._create_input_row(frame, "GOOD place height", cfg.sort_positions.place_good.z_down, "place_good.z_down")
        self._create_input_row(frame, "BAD place height", cfg.sort_positions.place_bad.z_down, "place_bad.z_down")

        self.build_section_header(frame, "⏱", "TIMING", INFO)
        self._create_input_row(frame, "Move Cooldown (s)", cfg.app.move_cooldown, "app.move_cooldown")
        self._create_input_row(frame, "PLC Poll Rate (s)", cfg.app.plc_poll_interval, "app.plc_poll_interval")

        self.build_section_header(frame, "🔄", "SORTING", INFO)
        self._create_combo_row(frame, "Sorting Mode", ["vision", "qr"], cfg.app.sorting_mode, "app.sorting_mode")

        # Removed addStretch
        self.btn_apply = btn_save = QPushButton("Apply && save settings")
        set_help(btn_save, "Áp dụng và lưu cài đặt", APPLY_HELP)
        btn_save.setFixedHeight(46)
        btn_save.setStyleSheet(btn_style(ACCENT, "#6BE3CB", "#10231F", radius=8, font_size=13))
        btn_save.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_save.clicked.connect(self._save_settings)

        layout.addStretch(1)

        parent.layout().addWidget(frame)

    def update_gui_data(self, data: dict[str, Any]) -> None:
        self._refresh_error_status(data)

    def _save_settings(self) -> None:
        try:
            with getattr(self.controller, "_sort_start_lock", nullcontext()):
                if self.controller._sort_thread is not None and self.controller._sort_thread.is_alive():
                    raise ValueError("Wait for the current sorting cycle before applying settings.")
                classification = getattr(self.controller, "_classification_running", None)
                if classification is not None and classification.is_set() is True:
                    raise ValueError("Wait for the current camera classification before applying settings.")
                cfg = deepcopy(self.controller.cfg)

                # Extract
                def get_val(key):
                    inp, is_int = self._inputs[key]
                    if is_int == "combo":
                        return inp.currentText()
                    text = inp.text().strip().replace(",", ".")
                    if not text:
                        raise ValueError(f"{key}: enter a value before saving.")
                    try:
                        return int(text) if is_int else float(text)
                    except ValueError as error:
                        raise ValueError(f"{key}: enter a valid {'integer' if is_int else 'decimal number'}.") from error

                cfg.yolo.thresh = get_val("yolo.thresh")
                cfg.yolo.px2mm = get_val("yolo.px2mm")
                if "yolo.coordinate_origin" in self._inputs:
                    cfg.yolo.coordinate_origin = get_val("yolo.coordinate_origin")

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

                if cfg.app.sorting_mode == "vision":
                    self.controller.validate_vision_configuration(cfg)
                save_config(cfg)

                # Reload YOLO threshold
                live = self.controller.cfg
                # Preserve objects already referenced by the sorting worker.
                for name in ("yolo", "app"):
                    for item in fields(getattr(cfg, name)):
                        setattr(getattr(live, name), item.name, getattr(getattr(cfg, name), item.name))
                for name in ("pick_z_down", "pick_z_up"):
                    setattr(live.sort_positions, name, getattr(cfg.sort_positions, name))
                live.sort_positions.place_good.z_down = cfg.sort_positions.place_good.z_down
                live.sort_positions.place_bad.z_down = cfg.sort_positions.place_bad.z_down
                self.controller.detector.thresh = cfg.yolo.thresh
                self.controller.detector.px2mm = cfg.yolo.px2mm
                self.controller._sort_cooldown_duration = cfg.app.move_cooldown
                self.controller.page_auto.btn_capture.setVisible(cfg.app.sorting_mode == "vision")
                if hasattr(self.controller, "refresh_sorting_controls"):
                    self.controller.refresh_sorting_controls()

            QMessageBox.information(self, "Success", "Settings saved successfully to config.yaml!")


        except Exception as e:
            log.error(f"Failed to save settings: {e}")
            QMessageBox.critical(self, "Error", f"Failed to save settings:\n{e}")
