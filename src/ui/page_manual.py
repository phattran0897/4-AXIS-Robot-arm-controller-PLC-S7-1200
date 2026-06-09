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

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)

_ACCENT: str = "#E11D48"
_ACCENT_LIGHT: str = "#F43F5E"


class PageManual(BasePage):
    """
    Manual-mode page.

    Layout
    ------
    Column 0 – Kinematics input panel (FK / IK toggle + joint-angle entries).
    Column 1 – Gripper and home-position control buttons.
    Column 2 – Live camera feed with source selector.
    """

    def __init__(self, parent: ctk.CTkFrame, controller: "RobotApp") -> None:
        super().__init__(parent, controller, page_color=_ACCENT)
        self._kine_mode: str = "Forward"
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        ctk.CTkLabel(
            self,
            text="MANUAL MODE",
            font=ctk.CTkFont(size=22, weight="bold"),
            text_color=_ACCENT,
        ).pack(pady=15)

        main_layout = ctk.CTkFrame(self, fg_color="transparent")
        main_layout.pack(fill="both", expand=True, padx=20, pady=10)
        main_layout.grid_columnconfigure(0, weight=3, uniform="group")
        main_layout.grid_columnconfigure(1, weight=3, uniform="group")
        main_layout.grid_columnconfigure(2, weight=4, uniform="group")
        main_layout.grid_rowconfigure(0, weight=1)

        self._build_kinematics_column(main_layout)
        self._build_control_column(main_layout)
        self.build_camera_column(main_layout, column=2, title_color=_ACCENT_LIGHT)

        self.lbl_err_status = self.build_status_bar(
            navigate_text="<< SWITCH TO AUTO",
            navigate_target="PageAuto",
            navigate_color=_ACCENT,
        )

    def _build_kinematics_column(self, parent: ctk.CTkFrame) -> None:
        frame = ctk.CTkFrame(parent)
        frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        ctk.CTkLabel(
            frame,
            text="Kinematics",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=_ACCENT_LIGHT,
        ).pack(pady=10)

        self.lbl_kine_mode = ctk.CTkLabel(
            frame,
            text="FORWARD KINEMATICS",
            font=ctk.CTkFont(weight="bold"),
            text_color="#10B981",
        )
        self.lbl_kine_mode.pack(pady=2)

        ctk.CTkButton(
            frame,
            text="Toggle FK / IK",
            command=self._toggle_kine_mode,
        ).pack(pady=4)

        self.ent_j1 = ctk.CTkEntry(frame, placeholder_text="Joint 1 angle (°)")
        self.ent_j1.pack(pady=3, padx=20, fill="x")
        self.ent_j2 = ctk.CTkEntry(frame, placeholder_text="Joint 2 angle (°)")
        self.ent_j2.pack(pady=3, padx=20, fill="x")
        self.ent_j3 = ctk.CTkEntry(frame, placeholder_text="Joint 3 angle (°)")
        self.ent_j3.pack(pady=3, padx=20, fill="x")
        self.ent_j4 = ctk.CTkEntry(frame, placeholder_text="Joint 4 angle (°)")
        self.ent_j4.pack(pady=3, padx=20, fill="x")

        ctk.CTkButton(
            frame,
            text="SEND JOINT TARGETS",
            fg_color=_ACCENT,
            hover_color="#991B1B",
            command=self._send_manual_positions,
        ).pack(pady=10, padx=20, fill="x")

    def _build_control_column(self, parent: ctk.CTkFrame) -> None:
        frame = ctk.CTkFrame(parent)
        frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        ctk.CTkLabel(
            frame,
            text="Actuator Control",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=_ACCENT_LIGHT,
        ).pack(pady=10)

        ctk.CTkButton(
            frame,
            text="MOVE TO HOME",
            fg_color="#4F46E5",
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.home
            ),
        ).pack(fill="x", padx=20, pady=12)

        ctk.CTkLabel(
            frame,
            text="Gripper Control:",
            font=ctk.CTkFont(weight="bold"),
        ).pack(pady=3)

        ctk.CTkButton(
            frame,
            text="GRIP (CLOSE)",
            fg_color="#10B981",
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.move
            ),
        ).pack(fill="x", padx=20, pady=4)

        ctk.CTkButton(
            frame,
            text="RELEASE (OPEN)",
            fg_color="#6B7280",
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.idle
            ),
        ).pack(fill="x", padx=20, pady=4)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _toggle_kine_mode(self) -> None:
        if self._kine_mode == "Forward":
            self._kine_mode = "Inverse"
            self.lbl_kine_mode.configure(
                text="INVERSE KINEMATICS", text_color="#3B82F6"
            )
        else:
            self._kine_mode = "Forward"
            self.lbl_kine_mode.configure(
                text="FORWARD KINEMATICS", text_color="#10B981"
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
