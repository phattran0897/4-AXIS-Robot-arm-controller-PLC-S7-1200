"""
src/ui/page_auto.py – Automatic (AI-driven) operation page.

Displays live YOLO camera feed, real-time joint-angle readouts, and
system-control buttons for the automated pick-and-sort workflow.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import customtkinter as ctk
from tkinter import messagebox

from src.robot.sorting_controller import RobotState, SortResult
from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    ACCENT_DARK,
    PANEL_BG,
    PANEL_BORDER,
    SUCCESS,
    WARNING,
    DANGER,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    CARD_BG,
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
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # Status bar first (packed side="bottom", fill="x")
        self.lbl_err_status = self.build_status_bar(
            navigate_text=">> MANUAL MODE >>",
            navigate_target="PageManual",
            navigate_color="#4F46E5",
        )

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

        # Section header
        header = ctk.CTkFrame(frame, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(15, 10))

        icon_label = ctk.CTkLabel(
            header,
            text="⚡",
            font=ctk.CTkFont(size=20),
            text_color=ACCENT,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="CONTROL PANEL",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=ACCENT,
        ).pack(side="left")

        # Buttons container
        btn_frame = ctk.CTkFrame(frame, fg_color="transparent")
        btn_frame.pack(fill="both", expand=True, padx=15, pady=10)

        # START button
        self.btn_start = ctk.CTkButton(
            btn_frame,
            text="▶  START AUTO",
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color=SUCCESS,
            hover_color="#059669",
            text_color="white",
            height=50,
            corner_radius=12,
            command=lambda: self.controller.plc.send_pulse(0, 0),
        )
        self.btn_start.pack(fill="x", pady=(5, 10))

        # PAUSE button
        self.btn_pause = ctk.CTkButton(
            btn_frame,
            text="⏸  PAUSE",
            font=ctk.CTkFont(size=14, weight="bold"),
            fg_color=WARNING,
            hover_color="#D97706",
            text_color="white",
            height=50,
            corner_radius=12,
            command=lambda: self.controller.plc.send_pulse(0, 1),
        )
        self.btn_pause.pack(fill="x", pady=5)

        # Status indicator
        self.lbl_operation = ctk.CTkLabel(
            btn_frame,
            text="● IDLE",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=TEXT_SECONDARY,
        )
        self.lbl_operation.pack(pady=15)

        # Reset button
        ctk.CTkButton(
            btn_frame,
            text="🔄  Reset Counter",
            font=ctk.CTkFont(size=11, weight="bold"),
            fg_color="transparent",
            border_color="#475569",
            border_width=1,
            hover_color="#475569",
            text_color=TEXT_PRIMARY,
            height=38,
            corner_radius=8,
            command=self._on_reset_stock,
        ).pack(fill="x", pady=5)

    def _build_status_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the joint status panel."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=1, padx=8, pady=8, sticky="nsew")

        # Section header
        header = ctk.CTkFrame(frame, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(15, 10))

        icon_label = ctk.CTkLabel(
            header,
            text="📊",
            font=ctk.CTkFont(size=20),
            text_color=ACCENT,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="JOINT STATUS",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=ACCENT,
        ).pack(side="left")

        # Joint data container
        data_frame = ctk.CTkFrame(frame, fg_color="transparent")
        data_frame.pack(fill="both", expand=True, padx=15, pady=10)

        # Joint values (styled as clean modern cells)
        self.lbl_j1 = self._create_joint_row(data_frame, "J1 (Xoay)", "0.00 °")
        self.lbl_j2 = self._create_joint_row(data_frame, "J2 (Vai)", "0.00 °")
        self.lbl_j3 = self._create_joint_row(data_frame, "J3 (Khuỷu)", "0.00 °")
        self.lbl_j4 = self._create_joint_row(data_frame, "J4 (Cổ tay)", "0.00 °")

        # Separator
        separator = ctk.CTkFrame(data_frame, height=1, fg_color=PANEL_BORDER)
        separator.pack(fill="x", pady=12)

        # Gripper status container
        grip_row = ctk.CTkFrame(
            data_frame,
            fg_color=PANEL_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        grip_row.pack(fill="x", pady=4, ipady=6)

        self.lbl_gripper = ctk.CTkLabel(
            grip_row,
            text="GRIPPER: OPEN",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=TEXT_SECONDARY,
        )
        self.lbl_gripper.pack(padx=12, anchor="w")

        # Separator for classification status
        separator2 = ctk.CTkFrame(data_frame, height=1, fg_color=PANEL_BORDER)
        separator2.pack(fill="x", pady=12)

        # Classification status indicator
        classify_row = ctk.CTkFrame(
            data_frame,
            fg_color=PANEL_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        classify_row.pack(fill="x", pady=4, ipady=8)

        self.lbl_classification = ctk.CTkLabel(
            classify_row,
            text="⏳ CHỜ PHÂN LOẠI",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=TEXT_SECONDARY,
        )
        self.lbl_classification.pack(padx=12)

        # Separator for sorting statistics
        separator3 = ctk.CTkFrame(data_frame, height=1, fg_color=PANEL_BORDER)
        separator3.pack(fill="x", pady=12)

        # Sorting statistics container
        sort_stats_frame = ctk.CTkFrame(data_frame, fg_color="transparent")
        sort_stats_frame.pack(fill="x", pady=4)
        sort_stats_frame.grid_columnconfigure(0, weight=1, uniform="stats")
        sort_stats_frame.grid_columnconfigure(1, weight=1, uniform="stats")

        # GOOD counter label
        self.lbl_good = ctk.CTkLabel(
            sort_stats_frame,
            text="✅ Tốt: 0",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=SUCCESS,
            anchor="w",
        )
        self.lbl_good.grid(row=0, column=0, padx=(0, 4), sticky="w")

        # BAD counter label
        self.lbl_bad = ctk.CTkLabel(
            sort_stats_frame,
            text="❌ Xấu: 0",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=DANGER,
            anchor="e",
        )
        self.lbl_bad.grid(row=0, column=1, padx=(4, 0), sticky="e")

        # Reset statistics button
        ctk.CTkButton(
            data_frame,
            text="🔄  Reset Counters",
            font=ctk.CTkFont(size=11, weight="bold"),
            fg_color="transparent",
            border_color="#475569",
            border_width=1,
            hover_color="#475569",
            text_color=TEXT_PRIMARY,
            height=32,
            corner_radius=8,
            command=self._on_reset_counters,
        ).pack(fill="x", pady=(10, 0))

    def _create_joint_row(self, parent: ctk.CTkFrame, name: str, value: str) -> ctk.CTkLabel:
        """Create a single joint status row styled as a clean card cell."""
        row = ctk.CTkFrame(
            parent,
            fg_color="#090E1A",  # Darker interior for premium look
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        row.pack(fill="x", pady=5, ipady=6)

        name_label = ctk.CTkLabel(
            row,
            text=name,
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=TEXT_SECONDARY,
            anchor="w",
        )
        name_label.pack(side="left", padx=12)

        value_label = ctk.CTkLabel(
            row,
            text=value,
            font=ctk.CTkFont(family="Consolas", size=14, weight="bold"),
            text_color=ACCENT,  # Sleek cyan glow color for values
            anchor="e",
        )
        value_label.pack(side="right", padx=12)

        return value_label

    def _create_card(self, parent: ctk.CTkFrame) -> ctk.CTkFrame:
        """Create a modern card with border."""
        frame = ctk.CTkFrame(
            parent,
            fg_color=CARD_BG,
            border_color=PANEL_BORDER,
            border_width=1,
            corner_radius=12,
        )
        return frame

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_reset_stock(self) -> None:
        messagebox.showinfo("Info", "Stock counter reset signal sent!")

    def _on_reset_counters(self) -> None:
        self.controller._sorter.reset_counters()
        self.lbl_good.configure(text="✅ Tốt: 0")
        self.lbl_bad.configure(text="❌ Xấu: 0")

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh joint-angle labels and error status from *data*."""
        self.lbl_j1.configure(text=f"{data.get('j1_target', 0.0):.2f} °")
        self.lbl_j2.configure(text=f"{data.get('j2_target', 0.0):.2f} °")
        self.lbl_j3.configure(text=f"{data.get('j3_target', 0.0):.2f} °")
        self.lbl_j4.configure(text=f"{data.get('j4_target', 0.0):.2f} °")

        # Update sorting counters
        sorter = self.controller._sorter
        self.lbl_good.configure(text=f"✅ Tốt: {sorter.counter_good}")
        self.lbl_bad.configure(text=f"❌ Xấu: {sorter.counter_bad}")

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
                self.lbl_gripper.configure(text="GRIPPER: OPENING", text_color=TEXT_SECONDARY)

        # Update classification status from sorting controller state
        if not sorter.is_idle() and sorter.last_sort_result is not None:
            # Sorting cycle in progress – show the active classification
            if sorter.last_sort_result == SortResult.GOOD:
                self.lbl_classification.configure(
                    text="✅ HÀNG TỐT", text_color=SUCCESS
                )
            else:
                self.lbl_classification.configure(
                    text="❌ HÀNG XẤU", text_color=DANGER
                )
        else:
            self.lbl_classification.configure(
                text="⏳ CHỜ PHÂN LOẠI", text_color=TEXT_SECONDARY
            )

        self._refresh_error_status(data)
