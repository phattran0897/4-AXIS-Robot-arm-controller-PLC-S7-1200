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

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)

_ACCENT: str = "#1F6AA5"
_ACCENT_LIGHT: str = "#38BDF8"


class PageAuto(BasePage):
    """
    Automatic-mode page.

    Layout
    ------
    Column 0 – System control buttons (START / PAUSE).
    Column 1 – Live joint-angle data readout from PLC.
    Column 2 – YOLO camera feed with source selector.
    """

    def __init__(self, parent: ctk.CTkFrame, controller: "RobotApp") -> None:
        super().__init__(parent, controller, page_color=_ACCENT)
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        ctk.CTkLabel(
            self,
            text="AUTOMATIC MODE",
            font=ctk.CTkFont(size=22, weight="bold"),
            text_color=_ACCENT,
        ).pack(pady=15)

        main_layout = ctk.CTkFrame(self, fg_color="transparent")
        main_layout.pack(fill="both", expand=True, padx=20, pady=10)
        main_layout.grid_columnconfigure(0, weight=3, uniform="group")
        main_layout.grid_columnconfigure(1, weight=3, uniform="group")
        main_layout.grid_columnconfigure(2, weight=4, uniform="group")
        main_layout.grid_rowconfigure(0, weight=1)

        self._build_control_column(main_layout)
        self._build_data_column(main_layout)
        self.build_camera_column(main_layout, column=2, title_color=_ACCENT_LIGHT)

        self.lbl_err_status = self.build_status_bar(
            navigate_text="SWITCH TO MANUAL >>",
            navigate_target="PageManual",
            navigate_color=_ACCENT,
        )

    def _build_control_column(self, parent: ctk.CTkFrame) -> None:
        frame = ctk.CTkFrame(parent)
        frame.grid(row=0, column=0, padx=10, pady=10, sticky="new")

        ctk.CTkLabel(
            frame,
            text="Control",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=_ACCENT_LIGHT,
        ).pack(pady=10)

        ctk.CTkButton(
            frame,
            text="START AUTO",
            fg_color="green",
            hover_color="#005500",
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.move
            ),
        ).pack(fill="x", padx=20, pady=15)

        ctk.CTkButton(
            frame,
            text="PAUSE",
            fg_color="#D97706",
            hover_color="#B45309",
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.idle
            ),
        ).pack(fill="x", padx=20, pady=15)

    def _build_data_column(self, parent: ctk.CTkFrame) -> None:
        frame = ctk.CTkFrame(parent)
        frame.grid(row=0, column=1, padx=10, pady=10, sticky="new")

        ctk.CTkLabel(
            frame,
            text="Joint Status",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=_ACCENT_LIGHT,
        ).pack(pady=10)

        label_font = ctk.CTkFont(size=13)
        self.lbl_j1 = ctk.CTkLabel(frame, text="J1: 0.00 °", font=label_font)
        self.lbl_j1.pack(pady=4, anchor="w", padx=20)
        self.lbl_j2 = ctk.CTkLabel(frame, text="J2: 0.00 °", font=label_font)
        self.lbl_j2.pack(pady=4, anchor="w", padx=20)
        self.lbl_j3 = ctk.CTkLabel(frame, text="J3: 0.00 °", font=label_font)
        self.lbl_j3.pack(pady=4, anchor="w", padx=20)
        self.lbl_j4 = ctk.CTkLabel(frame, text="J4: 0.00 °", font=label_font)
        self.lbl_j4.pack(pady=4, anchor="w", padx=20)

        self.lbl_gripper = ctk.CTkLabel(
            frame,
            text="Gripper: OPEN",
            font=ctk.CTkFont(size=13),
            text_color=_ACCENT_LIGHT,
        )
        self.lbl_gripper.pack(pady=8, anchor="w", padx=20)

        ctk.CTkButton(
            frame,
            text="Reset Stock Counter",
            fg_color="#4B5563",
            command=self._on_reset_stock,
        ).pack(fill="x", padx=20, pady=5)

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
        self.lbl_j1.configure(text=f"J1: {data.get('j1_target', 0.0):.2f} °")
        self.lbl_j2.configure(text=f"J2: {data.get('j2_target', 0.0):.2f} °")
        self.lbl_j3.configure(text=f"J3: {data.get('j3_target', 0.0):.2f} °")
        self.lbl_j4.configure(text=f"J4: {data.get('j4_target', 0.0):.2f} °")

        if data.get("motion_done", False):
            self.lbl_gripper.configure(
                text="Gripper: PICK COMPLETE", text_color="green"
            )
        else:
            self.lbl_gripper.configure(text="Gripper: OPEN", text_color=_ACCENT_LIGHT)

        self._refresh_error_status(data)
