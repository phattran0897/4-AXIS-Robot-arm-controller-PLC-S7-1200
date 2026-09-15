"""
src/ui/page_auto.py – Automatic (AI-driven) operation page.

Displays live YOLO camera feed, real-time joint-angle readouts, and
system-control buttons for the automated pick-and-sort workflow.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any

import customtkinter as ctk

from src.plc.plc_controller import ADDR
from src.robot.sorting_controller import RobotState, SortResult
from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    DANGER,
    INFO,
    PANEL_BG,
    PANEL_BORDER,
    ROW_BG,
    SUCCESS,
    SUCCESS_HOVER,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    WARNING,
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

    def __init__(self, parent: ctk.CTkFrame, controller: "RobotApp") -> None:
        super().__init__(parent, controller, page_color=ACCENT)
        # UI Interpolation states for smooth running numbers ("chạy số")
        self._disp = [0.0, 0.0, 0.0, 0.0]
        self._target = [0.0, 0.0, 0.0, 0.0]
        self._last_rendered = ["", "", "", ""]

        self._build_ui()

        # Start the smooth interpolation loop
        self._interpolate_gui_numbers()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # Status bar first (packed side="bottom", fill="x")
        self.lbl_err_status = self.build_status_bar()

        # Main content area
        main_layout = ctk.CTkFrame(self, fg_color="transparent")
        main_layout.pack(fill="both", expand=True, padx=20, pady=15)
        main_layout.grid_columnconfigure(0, weight=1, minsize=290)
        main_layout.grid_columnconfigure(1, weight=1, minsize=290)
        main_layout.grid_columnconfigure(2, weight=2, minsize=460)
        main_layout.grid_rowconfigure(0, weight=1)

        # Column 0: Control Panel
        self._build_control_panel(main_layout)

        # Column 1: Status Panel
        self._build_status_panel(main_layout)

        # Column 2: Camera Panel
        self.build_camera_column(main_layout, column=2, title_color=ACCENT)

    def _build_control_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the control panel with START/PAUSE buttons."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        self.build_section_header(frame, "⚡", "CONTROL PANEL", ACCENT)

        # Buttons container
        btn_frame = ctk.CTkFrame(frame, fg_color="transparent")
        btn_frame.pack(fill="both", expand=True, padx=15, pady=(6, 12))

        # START button
        self.btn_start = ctk.CTkButton(
            btn_frame,
            text="▶   START AUTO",
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color=SUCCESS,
            hover_color=SUCCESS_HOVER,
            text_color="white",
            height=52,
            corner_radius=12,
            command=lambda: self.controller.plc.send_pulse(*ADDR.START_AUTO),
        )
        self.btn_start.pack(fill="x", pady=(4, 10))

        # PAUSE button
        self.btn_pause = ctk.CTkButton(
            btn_frame,
            text="⏸   PAUSE",
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color=WARNING,
            hover_color="#D97706",
            text_color="white",
            height=52,
            corner_radius=12,
            command=lambda: self.controller.plc.send_pulse(*ADDR.PAUSE),
        )
        self.btn_pause.pack(fill="x", pady=5)

        # Status indicator
        status_card = ctk.CTkFrame(
            btn_frame,
            fg_color=ROW_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        status_card.pack(fill="x", pady=(14, 8), ipady=8)
        self.lbl_operation = ctk.CTkLabel(
            status_card,
            text="● IDLE",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=TEXT_SECONDARY,
        )
        self.lbl_operation.pack()

        # Capture & Classify button
        self.btn_capture = ctk.CTkButton(
            btn_frame,
            text="📸   CHỤP & PHÂN LOẠI",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color=INFO,
            hover_color="#2563EB",
            text_color="white",
            height=52,
            corner_radius=12,
            command=self.controller.trigger_manual_classification,
        )
        self.btn_capture.pack(fill="x", pady=(10, 15))

        # Reset button
        ctk.CTkButton(
            btn_frame,
            text="🔄   RESET COUNTERS",
            font=ctk.CTkFont(size=11, weight="bold"),
            fg_color="transparent",
            border_color="#475569",
            border_width=1,
            hover_color="#475569",
            text_color=TEXT_PRIMARY,
            height=38,
            corner_radius=8,
            command=self._on_reset_counters,
        ).pack(fill="x", pady=5)

    def _build_status_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the joint status panel."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=1, padx=8, pady=8, sticky="nsew")

        self.build_section_header(frame, "📊", "JOINT STATUS", ACCENT)

        # Joint data container
        data_frame = ctk.CTkFrame(frame, fg_color="transparent")
        data_frame.pack(fill="both", expand=True, padx=15, pady=(6, 12))

        # Joint values (styled as clean modern cells)
        self.lbl_j1 = self.create_data_row(data_frame, "J1 (Xoay)", "0.00 °", ACCENT)
        self.lbl_j2 = self.create_data_row(data_frame, "J2 (Vai)", "0.00 °", ACCENT)
        self.lbl_j3 = self.create_data_row(data_frame, "J3 (Khuỷu)", "0.00 °", ACCENT)
        self.lbl_j4 = self.create_data_row(data_frame, "J4 (Cổ tay)", "0.00 °", ACCENT)

        # Gripper status container
        grip_row = ctk.CTkFrame(
            data_frame,
            fg_color=PANEL_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        grip_row.pack(fill="x", pady=(10, 4), ipady=7)

        self.lbl_gripper = ctk.CTkLabel(
            grip_row,
            text="GRIPPER: OPEN",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=TEXT_SECONDARY,
        )
        self.lbl_gripper.pack(padx=12, anchor="w")

        # Classification status indicator
        classify_row = ctk.CTkFrame(
            data_frame,
            fg_color=PANEL_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        classify_row.pack(fill="x", pady=4, ipady=9)

        self.lbl_classification = ctk.CTkLabel(
            classify_row,
            text="⏳ CHỜ PHÂN LOẠI",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=TEXT_SECONDARY,
        )
        self.lbl_classification.pack(padx=12)

        # Sorting statistics container
        sort_stats_frame = ctk.CTkFrame(data_frame, fg_color="transparent")
        sort_stats_frame.pack(fill="x", pady=(12, 4))
        sort_stats_frame.grid_columnconfigure(0, weight=1, uniform="stats")
        sort_stats_frame.grid_columnconfigure(1, weight=1, uniform="stats")

        # GOOD counter card
        good_card = ctk.CTkFrame(
            sort_stats_frame,
            fg_color=ROW_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        good_card.grid(row=0, column=0, padx=(0, 4), sticky="nsew", ipady=6)
        ctk.CTkLabel(
            good_card,
            text="TỐT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=TEXT_SECONDARY,
        ).pack()
        self.lbl_good = ctk.CTkLabel(
            good_card,
            text="0",
            font=ctk.CTkFont(size=20, weight="bold"),
            text_color=SUCCESS,
        )
        self.lbl_good.pack()

        # BAD counter card
        bad_card = ctk.CTkFrame(
            sort_stats_frame,
            fg_color=ROW_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        bad_card.grid(row=0, column=1, padx=(4, 0), sticky="nsew", ipady=6)
        ctk.CTkLabel(
            bad_card,
            text="XẤU",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=TEXT_SECONDARY,
        ).pack()
        self.lbl_bad = ctk.CTkLabel(
            bad_card,
            text="0",
            font=ctk.CTkFont(size=20, weight="bold"),
            text_color=DANGER,
        )
        self.lbl_bad.pack()

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_reset_counters(self) -> None:
        self.controller.sorter.reset_counters()
        self.lbl_good.configure(text="0")
        self.lbl_bad.configure(text="0")

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh joint-angle targets and error status from *data*."""
        if not self.winfo_exists():
            return
        # Update targets from live PLC telemetry (GUI loop interpolates towards these)
        self._target = [
            float(data.get("j1_target", 0.0)),
            float(data.get("j2_target", 0.0)),
            float(data.get("j3_target", 0.0)),
            float(data.get("j4_target", 0.0)),
        ]

        sorter = self.controller.sorter
        # Update sorting counters
        self.lbl_good.configure(text=str(sorter.counter_good))
        self.lbl_bad.configure(text=str(sorter.counter_bad))

        # Update state displays
        state_str = sorter.state.name
        if sorter.state == RobotState.IDLE:
            self.lbl_operation.configure(text="● IDLE", text_color=TEXT_SECONDARY)
            self.lbl_gripper.configure(text="GRIPPER: OPEN", text_color=TEXT_SECONDARY)
        elif sorter.state == RobotState.ERROR:
            self.lbl_operation.configure(text="● SYSTEM ERROR", text_color=DANGER)
            self.lbl_gripper.configure(text="GRIPPER: FAULT", text_color=DANGER)
        else:
            self.lbl_operation.configure(text=f"● {state_str}", text_color=WARNING)
            if sorter.state == RobotState.GRIPPING:
                self.lbl_gripper.configure(text="GRIPPER: CLOSED", text_color=SUCCESS)
            elif sorter.state == RobotState.RELEASING:
                self.lbl_gripper.configure(
                    text="GRIPPER: OPENING", text_color=TEXT_SECONDARY
                )

        # Update classification status from sorting controller state
        if not sorter.is_idle() and sorter.last_sort_result is not None:
            # Sorting cycle in progress – show the active classification
            if sorter.last_sort_result == SortResult.GOOD:
                self.lbl_classification.configure(
                    text="✅ HÀNG TỐT", text_color=SUCCESS
                )
            else:
                self.lbl_classification.configure(text="❌ HÀNG XẤU", text_color=DANGER)
        else:
            self.lbl_classification.configure(
                text="⏳ CHỜ PHÂN LOẠI", text_color=TEXT_SECONDARY
            )

        self._refresh_error_status(data)

    def _interpolate_gui_numbers(self) -> None:
        """
        Smoothly step displayed joint values towards their targets.

        The loop keeps scheduling at ~30 FPS but only touches Tk widgets
        while this page is visible and when a value actually changed.
        """
        step_max = 5.0  # Max degrees to move per frame (creates a smooth roll)

        def move_towards(current: float, target: float, max_step: float) -> float:
            diff = target - current
            if abs(diff) <= max_step:
                return target
            return current + math.copysign(max_step, diff)

        visible = getattr(self.controller, "current_page", "PageAuto") == "PageAuto"

        labels = (self.lbl_j1, self.lbl_j2, self.lbl_j3, self.lbl_j4)
        for i, label in enumerate(labels):
            self._disp[i] = move_towards(self._disp[i], self._target[i], step_max)
            text = f"{self._disp[i]:.2f} °"
            if visible and text != self._last_rendered[i]:
                label.configure(text=text)
                self._last_rendered[i] = text

        # Run at ~30 FPS (33ms)
        self.after(33, self._interpolate_gui_numbers)
