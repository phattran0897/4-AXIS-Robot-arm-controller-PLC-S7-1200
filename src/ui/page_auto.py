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
        # Main content area
        main_layout = ctk.CTkFrame(self, fg_color="transparent")
        main_layout.pack(fill="both", expand=True, padx=20, pady=15)
        main_layout.grid_columnconfigure(0, weight=2, uniform="group")
        main_layout.grid_columnconfigure(1, weight=2, uniform="group")
        main_layout.grid_columnconfigure(2, weight=3, uniform="group")
        main_layout.grid_rowconfigure(0, weight=1)

        # Column 0: Control Panel
        self._build_control_panel(main_layout)

        # Column 1: Status Panel
        self._build_status_panel(main_layout)

        # Column 2: Camera Panel
        self.build_camera_column(main_layout, column=2, title_color=ACCENT)

        # Status bar
        self.lbl_err_status = self.build_status_bar(
            navigate_text=">> MANUAL MODE >>",
            navigate_target="PageManual",
            navigate_color="#4F46E5",
        )

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
            hover_color="#16A34A",
            text_color="white",
            height=50,
            corner_radius=10,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.move
            ),
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
            corner_radius=10,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.idle
            ),
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
            font=ctk.CTkFont(size=11),
            fg_color="#475569",
            hover_color="#64748B",
            text_color=TEXT_PRIMARY,
            height=35,
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

        # Joint angles
        label_font = ctk.CTkFont(size=13)
        label_font_bold = ctk.CTkFont(size=13, weight="bold")

        self.lbl_j1 = self._create_joint_row(data_frame, "J1 (Base)", "0.00 °")
        self.lbl_j2 = self._create_joint_row(data_frame, "J2 (Shoulder)", "0.00 °")
        self.lbl_j3 = self._create_joint_row(data_frame, "J3 (Elbow)", "0.00 °")
        self.lbl_j4 = self._create_joint_row(data_frame, "J4 (Wrist)", "0.00 °")

        # Separator
        separator = ctk.CTkFrame(data_frame, height=1, fg_color=PANEL_BORDER)
        separator.pack(fill="x", pady=12)

        # Gripper status
        self.lbl_gripper = ctk.CTkLabel(
            data_frame,
            text="GRIPPER: OPEN",
            font=label_font_bold,
            text_color=TEXT_SECONDARY,
        )
        self.lbl_gripper.pack(pady=8, anchor="w")

    def _create_joint_row(self, parent: ctk.CTkFrame, name: str, value: str) -> ctk.CTkLabel:
        """Create a single joint status row."""
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=4)

        name_label = ctk.CTkLabel(
            row,
            text=name,
            font=ctk.CTkFont(size=11),
            text_color=TEXT_SECONDARY,
            width=90,
            anchor="w",
        )
        name_label.pack(side="left")

        value_label = ctk.CTkLabel(
            row,
            text=value,
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=TEXT_PRIMARY,
            anchor="e",
        )
        value_label.pack(side="right")

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

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh joint-angle labels and error status from *data*."""
        self.lbl_j1.configure(text=f"{data.get('j1_target', 0.0):.2f} °")
        self.lbl_j2.configure(text=f"{data.get('j2_target', 0.0):.2f} °")
        self.lbl_j3.configure(text=f"{data.get('j3_target', 0.0):.2f} °")
        self.lbl_j4.configure(text=f"{data.get('j4_target', 0.0):.2f} °")

        if data.get("motion_done", False):
            self.lbl_gripper.configure(
                text="GRIPPER: PICK COMPLETE",
                text_color=SUCCESS,
            )
            self.lbl_operation.configure(text="● OPERATION COMPLETE", text_color=SUCCESS)
        else:
            self.lbl_gripper.configure(text="GRIPPER: OPEN", text_color=TEXT_SECONDARY)
            self.lbl_operation.configure(text="● RUNNING", text_color=WARNING)

        self._refresh_error_status(data)
