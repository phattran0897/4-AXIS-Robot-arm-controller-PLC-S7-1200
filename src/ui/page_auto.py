"""
src/ui/page_auto.py – Automatic (AI-driven) operation page (PySide6).

Displays live YOLO camera feed, real-time joint-angle readouts, and
system-control buttons for the automated pick-and-sort workflow.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from src.plc.plc_controller import ADDR
from src.robot.sorting_controller import RobotState, SortResult
from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    ACCENT_DIM,
    CARD_BG,
    DANGER,
    INFO,
    PANEL_BG,
    PANEL_BORDER,
    ROW_BG,
    SUCCESS,
    SUCCESS_HOVER,
    TEXT_DIM,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    WARNING,
    btn_style,
    card_style,
)

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


class PageAuto(BasePage):
    """
    Automatic-mode page with modern professional UI.

    Layout
    ------
    ┌─────────────────────────────────────────────────────────────────┐
    │  Column 0          │  Column 1           │  Column 2           │
    │  Control Panel     │  Status Panel       │  Camera Feed        │
    │  (START/PAUSE)     │  (Joint Angles)     │  (YOLO Detection)   │
    └─────────────────────────────────────────────────────────────────┘
    """

    def __init__(
        self, parent: QWidget | None = None, controller: "RobotApp" | None = None
    ) -> None:
        super().__init__(parent, controller, page_color=ACCENT)
        # UI Interpolation states for smooth running numbers ("chạy số")
        self._disp = [0.0, 0.0, 0.0, 0.0]
        self._target = [0.0, 0.0, 0.0, 0.0]
        self._last_rendered = ["", "", "", ""]

        self._build_ui()

        # Start the smooth interpolation loop via QTimer
        self._interp_timer = QTimer(self)
        self._interp_timer.timeout.connect(self._interpolate_gui_numbers)
        self._interp_timer.start(33)  # ~30 FPS

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 12, 16, 12)
        main_layout.setSpacing(12)

        # 3-column container
        columns_widget = QWidget(self)
        col_layout = QHBoxLayout(columns_widget)
        col_layout.setContentsMargins(0, 0, 0, 0)
        col_layout.setSpacing(12)

        # Column 0: Control Panel
        self._build_control_panel(columns_widget)

        # Column 1: Status Panel
        self._build_status_panel(columns_widget)

        # Column 2: Camera Panel
        self.build_camera_column(columns_widget, column=2, title_color=ACCENT)

        # Set stretch factors: Col 0: 2, Col 1: 2, Col 2: 3
        col_layout.setStretch(0, 2)
        col_layout.setStretch(1, 2)
        col_layout.setStretch(2, 3)

        main_layout.addWidget(columns_widget, 1)

        # Bottom status bar
        self.lbl_err_status = self.build_status_bar()

    def _build_control_panel(self, parent: QWidget) -> None:
        """Build the control panel with START/PAUSE buttons."""
        frame = self._create_card(parent)
        frame.setMinimumWidth(260)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 8, 8, 12)
        layout.setSpacing(8)

        self.build_section_header(frame, "⚡", "CONTROL PANEL", ACCENT)

        btn_frame = QWidget(frame)
        btn_frame.setStyleSheet("background: transparent; border: none;")
        btn_layout = QVBoxLayout(btn_frame)
        btn_layout.setContentsMargins(12, 6, 12, 12)
        btn_layout.setSpacing(8)

        # START button
        self.btn_start = QPushButton("▶   START AUTO", btn_frame)
        self.btn_start.setFixedHeight(48)
        self.btn_start.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_start.setStyleSheet(
            btn_style(SUCCESS, SUCCESS_HOVER, radius=12, font_size=14)
        )
        self.btn_start.clicked.connect(
            lambda: self.controller.plc.send_pulse(*ADDR.START_AUTO)
            if self.controller
            else None
        )
        btn_layout.addWidget(self.btn_start)

        # PAUSE button
        self.btn_pause = QPushButton("⏸   PAUSE", btn_frame)
        self.btn_pause.setFixedHeight(48)
        self.btn_pause.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_pause.setStyleSheet(
            btn_style(WARNING, "#D97706", radius=12, font_size=14)
        )
        self.btn_pause.clicked.connect(
            lambda: self.controller.plc.send_pulse(*ADDR.PAUSE)
            if self.controller
            else None
        )
        btn_layout.addWidget(self.btn_pause)

        # Status indicator card
        status_card = QFrame(btn_frame)
        status_card.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        sc_layout = QVBoxLayout(status_card)
        sc_layout.setContentsMargins(12, 10, 12, 10)

        self.lbl_operation = QLabel("● IDLE", status_card)
        self.lbl_operation.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 13px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        sc_layout.addWidget(self.lbl_operation)
        btn_layout.addWidget(status_card)

        # Capture & Classify button
        self.btn_capture = QPushButton("📸   CHỤP & PHÂN LOẠI", btn_frame)
        self.btn_capture.setFixedHeight(48)
        self.btn_capture.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_capture.setStyleSheet(
            btn_style(INFO, "#2563EB", radius=12, font_size=13)
        )
        self.btn_capture.clicked.connect(
            lambda: self.controller.trigger_manual_classification()
            if self.controller
            else None
        )
        btn_layout.addWidget(self.btn_capture)

        # Reset counters button
        btn_reset = QPushButton("🔄   RESET COUNTERS", btn_frame)
        btn_reset.setFixedHeight(36)
        btn_reset.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_reset.setStyleSheet(
            btn_style(
                bg="transparent",
                hover="#475569",
                text=TEXT_PRIMARY,
                radius=8,
                font_size=11,
                border="1px solid #475569",
            )
        )
        btn_reset.clicked.connect(self._on_reset_counters)
        btn_layout.addWidget(btn_reset)
        btn_layout.addStretch()

        layout.addWidget(btn_frame)

        if parent.layout() is not None:
            parent.layout().addWidget(frame)

    def _build_status_panel(self, parent: QWidget) -> None:
        """Build the joint status panel."""
        frame = self._create_card(parent)
        frame.setMinimumWidth(260)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 8, 8, 12)
        layout.setSpacing(6)

        self.build_section_header(frame, "📊", "JOINT STATUS", ACCENT)

        data_frame = QWidget(frame)
        data_frame.setStyleSheet("background: transparent; border: none;")
        data_layout = QVBoxLayout(data_frame)
        data_layout.setContentsMargins(12, 6, 12, 12)
        data_layout.setSpacing(4)

        # Joint values
        self.lbl_j1 = self.create_data_row(data_frame, "J1 (Xoay)", "0.00 °", ACCENT)
        self.lbl_j2 = self.create_data_row(data_frame, "J2 (Vai)", "0.00 °", ACCENT)
        self.lbl_j3 = self.create_data_row(data_frame, "J3 (Khuỷu)", "0.00 °", ACCENT)
        self.lbl_j4 = self.create_data_row(data_frame, "J4 (Cổ tay)", "0.00 °", ACCENT)

        # Pick height readout
        pick_z = (
            self.controller.cfg.sort_positions.pick_z_down
            if self.controller and hasattr(self.controller, "cfg")
            else 80.0
        )
        self.lbl_pick_z = self.create_data_row(
            data_frame, "Pick Z", f"{pick_z:.1f} mm", INFO
        )

        # Gripper status container
        grip_frame = QFrame(data_frame)
        grip_frame.setStyleSheet(card_style(PANEL_BG, PANEL_BORDER, 10))
        gf_layout = QVBoxLayout(grip_frame)
        gf_layout.setContentsMargins(12, 8, 12, 8)

        self.lbl_gripper = QLabel("GRIPPER: OPEN", grip_frame)
        self.lbl_gripper.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 12px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        gf_layout.addWidget(self.lbl_gripper)
        data_layout.addWidget(grip_frame)

        # Classification status indicator
        classify_frame = QFrame(data_frame)
        classify_frame.setStyleSheet(card_style(PANEL_BG, PANEL_BORDER, 10))
        cf_layout = QVBoxLayout(classify_frame)
        cf_layout.setContentsMargins(12, 10, 12, 10)

        self.lbl_classification = QLabel("⏳ CHỜ PHÂN LOẠI", classify_frame)
        self.lbl_classification.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 13px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        cf_layout.addWidget(self.lbl_classification)
        data_layout.addWidget(classify_frame)

        # Sorting statistics container (GOOD / BAD cards)
        stats_widget = QWidget(data_frame)
        stats_widget.setStyleSheet("background: transparent; border: none;")
        stats_layout = QHBoxLayout(stats_widget)
        stats_layout.setContentsMargins(0, 6, 0, 0)
        stats_layout.setSpacing(8)

        # GOOD card
        good_card = QFrame(stats_widget)
        good_card.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        good_layout = QVBoxLayout(good_card)
        good_layout.setContentsMargins(10, 8, 10, 8)
        good_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        lbl_g_title = QLabel("✅ TỐT", good_card)
        lbl_g_title.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        lbl_g_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        good_layout.addWidget(lbl_g_title)

        self.lbl_good = QLabel("0", good_card)
        self.lbl_good.setStyleSheet(
            f"color: {SUCCESS}; font-size: 22px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        self.lbl_good.setAlignment(Qt.AlignmentFlag.AlignCenter)
        good_layout.addWidget(self.lbl_good)
        stats_layout.addWidget(good_card)

        # BAD card
        bad_card = QFrame(stats_widget)
        bad_card.setStyleSheet(card_style(ROW_BG, PANEL_BORDER, 10))
        bad_layout = QVBoxLayout(bad_card)
        bad_layout.setContentsMargins(10, 8, 10, 8)
        bad_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        lbl_b_title = QLabel("❌ XẤU", bad_card)
        lbl_b_title.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        lbl_b_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bad_layout.addWidget(lbl_b_title)

        self.lbl_bad = QLabel("0", bad_card)
        self.lbl_bad.setStyleSheet(
            f"color: {DANGER}; font-size: 22px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        self.lbl_bad.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bad_layout.addWidget(self.lbl_bad)
        stats_layout.addWidget(bad_card)

        data_layout.addWidget(stats_widget)
        data_layout.addStretch()

        layout.addWidget(data_frame)

        if parent.layout() is not None:
            parent.layout().addWidget(frame)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_reset_counters(self) -> None:
        if self.controller and hasattr(self.controller, "sorter"):
            self.controller.sorter.reset_counters()
        self.lbl_good.setText("0")
        self.lbl_bad.setText("0")

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh joint-angle targets and error status from *data*."""
        self._target = [
            float(data.get("j1_target", 0.0)),
            float(data.get("j2_target", 0.0)),
            float(data.get("j3_target", 0.0)),
            float(data.get("j4_target", 0.0)),
        ]

        if not self.controller or not hasattr(self.controller, "sorter"):
            return

        sorter = self.controller.sorter
        # Update sorting counters
        self.lbl_good.setText(str(sorter.counter_good))
        self.lbl_bad.setText(str(sorter.counter_bad))

        # Update state displays
        state_str = sorter.state.name
        if sorter.state == RobotState.IDLE:
            self.lbl_operation.setText("● IDLE")
            self.lbl_operation.setStyleSheet(
                f"color: {TEXT_SECONDARY}; font-size: 13px; font-weight: bold; border: none; background: transparent;"
            )
            self.lbl_gripper.setText("GRIPPER: OPEN")
            self.lbl_gripper.setStyleSheet(
                f"color: {TEXT_SECONDARY}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
            )
        elif sorter.state == RobotState.ERROR:
            self.lbl_operation.setText("● SYSTEM ERROR")
            self.lbl_operation.setStyleSheet(
                f"color: {DANGER}; font-size: 13px; font-weight: bold; border: none; background: transparent;"
            )
            self.lbl_gripper.setText("GRIPPER: FAULT")
            self.lbl_gripper.setStyleSheet(
                f"color: {DANGER}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
            )
        else:
            self.lbl_operation.setText(f"● {state_str}")
            self.lbl_operation.setStyleSheet(
                f"color: {WARNING}; font-size: 13px; font-weight: bold; border: none; background: transparent;"
            )
            if sorter.state == RobotState.GRIPPING:
                self.lbl_gripper.setText("GRIPPER: CLOSED")
                self.lbl_gripper.setStyleSheet(
                    f"color: {SUCCESS}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
                )
            elif sorter.state == RobotState.RELEASING:
                self.lbl_gripper.setText("GRIPPER: OPENING")
                self.lbl_gripper.setStyleSheet(
                    f"color: {TEXT_SECONDARY}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
                )

        # Update classification status
        if not sorter.is_idle() and sorter.last_sort_result is not None:
            if sorter.last_sort_result == SortResult.GOOD:
                self.lbl_classification.setText("✅ HÀNG TỐT")
                self.lbl_classification.setStyleSheet(
                    f"color: {SUCCESS}; font-size: 13px; font-weight: bold; border: none; background: transparent;"
                )
            else:
                self.lbl_classification.setText("❌ HÀNG XẤU")
                self.lbl_classification.setStyleSheet(
                    f"color: {DANGER}; font-size: 13px; font-weight: bold; border: none; background: transparent;"
                )
        else:
            self.lbl_classification.setText("⏳ CHỜ PHÂN LOẠI")
            self.lbl_classification.setStyleSheet(
                f"color: {TEXT_SECONDARY}; font-size: 13px; font-weight: bold; border: none; background: transparent;"
            )

        self._refresh_error_status(data)

    def _interpolate_gui_numbers(self) -> None:
        """Smoothly step displayed joint values towards their targets."""
        step_max = 5.0  # Max degrees per frame

        def move_towards(current: float, target: float, max_step: float) -> float:
            diff = target - current
            if abs(diff) <= max_step:
                return target
            return current + math.copysign(max_step, diff)

        visible = (
            getattr(self.controller, "current_page", "PageAuto") == "PageAuto"
            if self.controller
            else True
        )

        labels = (self.lbl_j1, self.lbl_j2, self.lbl_j3, self.lbl_j4)
        for i, label in enumerate(labels):
            self._disp[i] = move_towards(self._disp[i], self._target[i], step_max)
            text = f"{self._disp[i]:.2f} °"
            if visible and text != self._last_rendered[i]:
                label.setText(text)
                self._last_rendered[i] = text
