"""
src/ui/page_manual.py – Manual (operator-driven) operation page.

Provides direct joint-angle entry (forward / inverse kinematics toggle),
gripper control buttons, and a synchronised camera feed.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import customtkinter as ctk
from tkinter import messagebox

from src.kinematics import inverse_kinematics
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
    MANUALACCENT,
)

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


class PageManual(BasePage):
    """
    Manual-mode page with modern professional UI.

    Layout
    ------
    ┌─────────────────────────────────────────────────────────────────┐
    │  Column 0              │  Column 1           │  Column 2       │
    │  Kinematics Input       │  Actuator Control   │  Camera Feed   │
    │  (FK/IK Toggle)        │  (Home/Gripper)    │               │
    └─────────────────────────────────────────────────────────────────┘
    """

    def __init__(self, parent: ctk.CTkFrame, controller: "RobotApp") -> None:
        super().__init__(parent, controller, page_color=MANUALACCENT)
        self._kine_mode: str = "Forward"
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

        # Column 0: Kinematics Panel
        self._build_kinematics_panel(main_layout)

        # Column 1: Actuator Control Panel
        self._build_actuator_panel(main_layout)

        # Column 2: Camera Panel
        self.build_camera_column(main_layout, column=2, title_color=ACCENT)

        # Status bar
        self.lbl_err_status = self.build_status_bar(
            navigate_text="<< AUTO MODE <<",
            navigate_target="PageAuto",
            navigate_color="#4F46E5",
        )

    def _build_kinematics_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the kinematics input panel."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        # Section header
        header = ctk.CTkFrame(frame, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(15, 10))

        icon_label = ctk.CTkLabel(
            header,
            text="🎯",
            font=ctk.CTkFont(size=20),
            text_color=MANUALACCENT,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="KINEMATICS INPUT",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=MANUALACCENT,
        ).pack(side="left")

        # Mode toggle
        mode_frame = ctk.CTkFrame(frame, fg_color="transparent")
        mode_frame.pack(fill="x", padx=15, pady=5)

        self.lbl_kine_mode = ctk.CTkLabel(
            mode_frame,
            text="▶ FORWARD KINEMATICS",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=SUCCESS,
        )
        self.lbl_kine_mode.pack(side="left")

        ctk.CTkButton(
            mode_frame,
            text="Toggle",
            font=ctk.CTkFont(size=10),
            fg_color="#475569",
            hover_color="#64748B",
            text_color=TEXT_PRIMARY,
            width=70,
            height=28,
            corner_radius=6,
            command=self._toggle_kine_mode,
        ).pack(side="right")

        # Entry fields
        entry_frame = ctk.CTkFrame(frame, fg_color="transparent")
        entry_frame.pack(fill="both", expand=True, padx=15, pady=10)

        entry_font = ctk.CTkFont(size=12)
        entry_bg = "#1E293B"
        entry_fg = TEXT_PRIMARY

        self.ent_j1 = ctk.CTkEntry(
            entry_frame,
            placeholder_text="J1 - Base angle (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            border_color=PANEL_BORDER,
            height=38,
            corner_radius=8,
        )
        self.ent_j1.pack(pady=6, fill="x")

        self.ent_j2 = ctk.CTkEntry(
            entry_frame,
            placeholder_text="J2 - Shoulder angle (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            border_color=PANEL_BORDER,
            height=38,
            corner_radius=8,
        )
        self.ent_j2.pack(pady=6, fill="x")

        self.ent_j3 = ctk.CTkEntry(
            entry_frame,
            placeholder_text="J3 - Elbow angle (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            border_color=PANEL_BORDER,
            height=38,
            corner_radius=8,
        )
        self.ent_j3.pack(pady=6, fill="x")

        self.ent_j4 = ctk.CTkEntry(
            entry_frame,
            placeholder_text="J4 - Wrist angle (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            border_color=PANEL_BORDER,
            height=38,
            corner_radius=8,
        )
        self.ent_j4.pack(pady=6, fill="x")

        # Send button
        ctk.CTkButton(
            entry_frame,
            text="✈  SEND JOINT TARGETS",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=MANUALACCENT,
            hover_color="#9333EA",
            text_color="white",
            height=42,
            corner_radius=8,
            command=self._send_manual_positions,
        ).pack(pady=10, fill="x")

    def _build_actuator_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the actuator/gripper control panel."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=1, padx=8, pady=8, sticky="nsew")

        # Section header
        header = ctk.CTkFrame(frame, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(15, 10))

        icon_label = ctk.CTkLabel(
            header,
            text="🦾",
            font=ctk.CTkFont(size=20),
            text_color=MANUALACCENT,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="ACTUATOR CONTROL",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=MANUALACCENT,
        ).pack(side="left")

        # Buttons container
        btn_frame = ctk.CTkFrame(frame, fg_color="transparent")
        btn_frame.pack(fill="both", expand=True, padx=15, pady=10)

        # HOME button
        ctk.CTkButton(
            btn_frame,
            text="🏠  MOVE TO HOME",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color="#4F46E5",
            hover_color="#4338CA",
            text_color="white",
            height=45,
            corner_radius=8,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.home
            ),
        ).pack(fill="x", pady=(5, 10))

        # GRIP button
        ctk.CTkButton(
            btn_frame,
            text="✊  GRIP (CLOSE)",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=SUCCESS,
            hover_color="#16A34A",
            text_color="white",
            height=45,
            corner_radius=8,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.move
            ),
        ).pack(fill="x", pady=5)

        # RELEASE button
        ctk.CTkButton(
            btn_frame,
            text="🖐  RELEASE (OPEN)",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color="#64748B",
            hover_color="#475569",
            text_color="white",
            height=45,
            corner_radius=8,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.idle
            ),
        ).pack(fill="x", pady=5)

        # STOP button
        ctk.CTkButton(
            btn_frame,
            text="⏹  STOP",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=_DANGER,
            hover_color="#DC2626",
            text_color="white",
            height=45,
            corner_radius=8,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.stop
            ),
        ).pack(fill="x", pady=(10, 5))

    def _create_card(self, parent: ctk.CTkFrame) -> ctk.CTkFrame:
        """Create a modern card with border."""
        frame = ctk.CTkFrame(
            parent,
            fg_color=_CARD_BG,
            border_color=PANEL_BORDER,
            border_width=1,
            corner_radius=12,
        )
        return frame

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _toggle_kine_mode(self) -> None:
        if self._kine_mode == "Forward":
            self._kine_mode = "Inverse"
            self.lbl_kine_mode.configure(
                text="▶ INVERSE KINEMATICS",
                text_color="#3B82F6",
            )
        else:
            self._kine_mode = "Forward"
            self.lbl_kine_mode.configure(
                text="▶ FORWARD KINEMATICS",
                text_color=SUCCESS,
            )

    def _send_manual_positions(self) -> None:
        """Parse entry fields and dispatch joint targets to the PLC."""
        try:
            j1 = float(self.ent_j1.get().strip()) if self.ent_j1.get().strip() else 0.0
            j2 = float(self.ent_j2.get().strip()) if self.ent_j2.get().strip() else 0.0
            j3 = float(self.ent_j3.get().strip()) if self.ent_j3.get().strip() else 0.0
            j4 = float(self.ent_j4.get().strip()) if self.ent_j4.get().strip() else 0.0
        except ValueError:
            messagebox.showerror(
                "Input Error",
                "All joint fields must contain valid decimal numbers.",
            )
            return

        # Validate joint angle ranges to prevent damage or PLC faults
        if not (-180.0 <= j1 <= 180.0):
            messagebox.showerror("Input Error", f"J1 must be between -180° and 180° (got {j1}°)")
            return
        if not (-180.0 <= j2 <= 180.0):
            messagebox.showerror("Input Error", f"J2 must be between -180° and 180° (got {j2}°)")
            return
        if not (-180.0 <= j3 <= 180.0):
            messagebox.showerror("Input Error", f"J3 must be between -180° and 180° (got {j3}°)")
            return
        if not (-360.0 <= j4 <= 360.0):
            messagebox.showerror("Input Error", f"J4 must be between -360° and 360° (got {j4}°)")
            return

        self.controller.plc.send_joint_targets(j1, j2, j3, j4)
        log.info(
            "Manual joint targets dispatched: J1=%.2f J2=%.2f J3=%.2f J4=%.2f",
            j1, j2, j3, j4,
        )
        messagebox.showinfo(
            "Success",
            f"Joint targets sent:\nJ1={j1:.2f}°  J2={j2:.2f}°\nJ3={j3:.2f}°  J4={j4:.2f}°",
        )

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh the error status label from *data*."""
        self._refresh_error_status(data)
