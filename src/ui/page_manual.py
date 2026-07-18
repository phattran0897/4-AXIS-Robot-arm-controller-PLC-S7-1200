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

from src.kinematics import forward_kinematics, inverse_kinematics
from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    PANEL_BORDER,
    SUCCESS,
    DANGER,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    CARD_BG,
    MANUAL_ACCENT,
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
        super().__init__(parent, controller, page_color=MANUAL_ACCENT)
        self._kine_mode: str = "Forward"
        # Store last computed results for sending to PLC
        self._last_fk_result: tuple[float, float, float, float] | None = None
        self._last_ik_result: tuple[float, float, float, float] | None = None
        self._last_fk_coords: tuple[float, float, float] | None = None
        self._last_ik_coords: tuple[float, float, float] | None = None
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # Status bar first (packed side="bottom", fill="x")
        self.lbl_err_status = self.build_status_bar(
            navigate_text="<< AUTO MODE <<",
            navigate_target="PageAuto",
            navigate_color="#4F46E5",
        )

        # Main content area
        main_layout = ctk.CTkFrame(self, fg_color="transparent")
        main_layout.pack(fill="both", expand=True, padx=20, pady=15)
        main_layout.grid_columnconfigure(0, weight=1, minsize=290)
        main_layout.grid_columnconfigure(1, weight=1, minsize=290)
        main_layout.grid_columnconfigure(2, weight=2, minsize=460)
        main_layout.grid_rowconfigure(0, weight=1)

        # Column 0: Kinematics Panel
        self._build_kinematics_panel(main_layout)

        # Column 1: Actuator Control Panel
        self._build_actuator_panel(main_layout)

        # Column 2: Camera Panel
        self.build_camera_column(main_layout, column=2, title_color=ACCENT)

    def _build_kinematics_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the kinematics input panel with 2-column layout (Input | Result)."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=0, padx=8, pady=8, sticky="nsew")

        # Section header
        header = ctk.CTkFrame(frame, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(15, 10))

        icon_label = ctk.CTkLabel(
            header,
            text="🎯",
            font=ctk.CTkFont(size=20),
            text_color=MANUAL_ACCENT,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="KINEMATICS INPUT",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=MANUAL_ACCENT,
        ).pack(side="left")

        # Mode toggle – two buttons: THUẬN (16.5) and NGHỊCH (16.6)
        mode_frame = ctk.CTkFrame(frame, fg_color="transparent")
        mode_frame.pack(fill="x", padx=15, pady=5)

        self.lbl_kine_mode = ctk.CTkLabel(
            mode_frame,
            text="▶ FORWARD KINEMATICS",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=SUCCESS,
        )
        self.lbl_kine_mode.pack(side="left")

        # Button container (right-aligned)
        kine_btn_frame = ctk.CTkFrame(mode_frame, fg_color="transparent")
        kine_btn_frame.pack(side="right")

        self.btn_thuan = ctk.CTkButton(
            kine_btn_frame,
            text="THUẬN",
            font=ctk.CTkFont(size=10, weight="bold"),
            fg_color=SUCCESS,
            hover_color="#059669",
            text_color="white",
            width=70,
            height=28,
            corner_radius=6,
            command=self._select_forward_mode,
        )
        self.btn_thuan.pack(side="left", padx=(0, 4))

        self.btn_nghich = ctk.CTkButton(
            kine_btn_frame,
            text="NGHỊCH",
            font=ctk.CTkFont(size=10, weight="bold"),
            fg_color="transparent",
            border_color="#475569",
            border_width=1,
            hover_color="#475569",
            text_color=TEXT_PRIMARY,
            width=70,
            height=28,
            corner_radius=6,
            command=self._select_inverse_mode,
        )
        self.btn_nghich.pack(side="left")

        # ── Entry fields container ──────────────────────────────────────
        entry_frame = ctk.CTkFrame(frame, fg_color="transparent")
        entry_frame.pack(fill="both", expand=True, padx=15, pady=10)

        entry_font = ctk.CTkFont(size=12)
        entry_bg = "#1E293B"
        entry_fg = TEXT_PRIMARY

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # FORWARD KINEMATICS – 2-column layout
        # Input: J1(°), J2(°), J3(°), J4(°)  →  Result: X, Y, Z
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self._fk_frame = ctk.CTkFrame(entry_frame, fg_color="transparent")
        self._fk_frame.pack(fill="both", expand=True)

        # Two-column grid
        fk_columns = ctk.CTkFrame(self._fk_frame, fg_color="transparent")
        fk_columns.pack(fill="both", expand=True)
        fk_columns.grid_columnconfigure(0, weight=1, uniform="fk")
        fk_columns.grid_columnconfigure(1, weight=1, uniform="fk")

        # ── FK Left Column: INPUT ──
        fk_left = ctk.CTkFrame(fk_columns, fg_color="transparent")
        fk_left.grid(row=0, column=0, padx=(0, 4), sticky="nsew")

        ctk.CTkLabel(
            fk_left,
            text="📥 INPUT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=MANUAL_ACCENT,
        ).pack(anchor="w", pady=(0, 4))

        self.ent_j1 = ctk.CTkEntry(
            fk_left,
            placeholder_text="θ₁ - Base (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_j1.pack(pady=3, fill="x")

        self.ent_j2 = ctk.CTkEntry(
            fk_left,
            placeholder_text="θ₂ - Shoulder (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_j2.pack(pady=3, fill="x")

        self.ent_j3 = ctk.CTkEntry(
            fk_left,
            placeholder_text="θ₃ - Elbow (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_j3.pack(pady=3, fill="x")

        self.ent_j4 = ctk.CTkEntry(
            fk_left,
            placeholder_text="θ₄ - Wrist (°)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_j4.pack(pady=3, fill="x")

        # ── FK Right Column: RESULT ──
        fk_right = ctk.CTkFrame(fk_columns, fg_color="transparent")
        fk_right.grid(row=0, column=1, padx=(4, 0), sticky="nsew")

        ctk.CTkLabel(
            fk_right,
            text="📤 RESULT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=SUCCESS,
        ).pack(anchor="w", pady=(0, 4))

        result_bg = "#0F172A"
        result_font = ctk.CTkFont(size=13, weight="bold")

        self.lbl_fk_x = self._create_result_row(fk_right, "X", "--- mm", result_bg, result_font)
        self.lbl_fk_y = self._create_result_row(fk_right, "Y", "--- mm", result_bg, result_font)
        self.lbl_fk_z = self._create_result_row(fk_right, "Z", "--- mm", result_bg, result_font)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # INVERSE KINEMATICS – 2-column layout (Input X,Y,Z | Result J1-J4)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self._ik_frame = ctk.CTkFrame(entry_frame, fg_color="transparent")

        # Two-column grid
        ik_columns = ctk.CTkFrame(self._ik_frame, fg_color="transparent")
        ik_columns.pack(fill="both", expand=True)
        ik_columns.grid_columnconfigure(0, weight=1, uniform="ik")
        ik_columns.grid_columnconfigure(1, weight=1, uniform="ik")

        # ── IK Left Column: INPUT ──
        ik_left = ctk.CTkFrame(ik_columns, fg_color="transparent")
        ik_left.grid(row=0, column=0, padx=(0, 4), sticky="nsew")

        ctk.CTkLabel(
            ik_left,
            text="📥 INPUT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=MANUAL_ACCENT,
        ).pack(anchor="w", pady=(0, 4))

        self.ent_x = ctk.CTkEntry(
            ik_left,
            placeholder_text="X (mm)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_x.pack(pady=4, fill="x")

        self.ent_y = ctk.CTkEntry(
            ik_left,
            placeholder_text="Y (mm)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_y.pack(pady=4, fill="x")

        self.ent_z = ctk.CTkEntry(
            ik_left,
            placeholder_text="Z (0-150 mm)",
            font=entry_font,
            fg_color=entry_bg,
            text_color=entry_fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        self.ent_z.pack(pady=4, fill="x")

        # ── IK Right Column: RESULT ──
        ik_right = ctk.CTkFrame(ik_columns, fg_color="transparent")
        ik_right.grid(row=0, column=1, padx=(4, 0), sticky="nsew")

        ctk.CTkLabel(
            ik_right,
            text="📤 RESULT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=SUCCESS,
        ).pack(anchor="w", pady=(0, 4))

        self.lbl_ik_j1 = self._create_result_row(ik_right, "θ₁", "--- °", result_bg, result_font)
        self.lbl_ik_j2 = self._create_result_row(ik_right, "θ₂", "--- °", result_bg, result_font)
        self.lbl_ik_j3 = self._create_result_row(ik_right, "θ₃", "--- °", result_bg, result_font)

        # ── Action buttons row ──────────────────────────────────────────
        btn_row = ctk.CTkFrame(entry_frame, fg_color="transparent")
        btn_row.pack(fill="x", pady=(8, 5))
        btn_row.grid_columnconfigure(0, weight=1, uniform="btn")
        btn_row.grid_columnconfigure(1, weight=1, uniform="btn")

        # CALCULATE button
        ctk.CTkButton(
            btn_row,
            text="🔢  CALCULATE",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color="#3B82F6",
            hover_color="#2563EB",
            text_color="white",
            height=42,
            corner_radius=10,
            command=self._calculate,
        ).grid(row=0, column=0, padx=(0, 4), sticky="ew")

        # SEND TARGETS button
        ctk.CTkButton(
            btn_row,
            text="✈  SEND",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=MANUAL_ACCENT,
            hover_color="#9333EA",
            text_color="white",
            height=42,
            corner_radius=10,
            command=self._send_positions,
        ).grid(row=0, column=1, padx=(4, 0), sticky="ew")

    def _create_result_row(
        self,
        parent: ctk.CTkFrame,
        label: str,
        value: str,
        bg: str,
        font: ctk.CTkFont,
    ) -> ctk.CTkLabel:
        """Create a styled result display row and return the value label."""
        row = ctk.CTkFrame(
            parent,
            fg_color="#090E1A",  # Darker premium interior background
            corner_radius=8,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        row.pack(fill="x", pady=4, ipady=4)

        ctk.CTkLabel(
            row,
            text=label,
            font=ctk.CTkFont(family="Segoe UI", size=11, weight="bold"),
            text_color=MANUAL_ACCENT,  # Vibrant purple manual accent
            width=25,
        ).pack(side="left", padx=(8, 4))

        val_label = ctk.CTkLabel(
            row,
            text=value,
            font=ctk.CTkFont(family="Consolas", size=14, weight="bold"),
            text_color=TEXT_PRIMARY,
        )
        val_label.pack(side="right", padx=8)

        return val_label

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
            text_color=MANUAL_ACCENT,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="ACTUATOR CONTROL",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=MANUAL_ACCENT,
        ).pack(side="left")

        # Buttons container
        btn_frame = ctk.CTkFrame(frame, fg_color="transparent")
        btn_frame.pack(fill="both", expand=True, padx=15, pady=10)

        # HOME button
        ctk.CTkButton(
            btn_frame,
            text="🏠  MOVE TO HOME",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color="#4F46E5",
            hover_color="#4338CA",
            text_color="white",
            height=45,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(16, 1),
        ).pack(fill="x", pady=(5, 10))

        # Unified Grip/Release Row
        grip_frame = ctk.CTkFrame(btn_frame, fg_color="transparent")
        grip_frame.pack(fill="x", pady=5)
        grip_frame.grid_columnconfigure(0, weight=1, uniform="grip")
        grip_frame.grid_columnconfigure(1, weight=1, uniform="grip")

        # GRIP button
        ctk.CTkButton(
            grip_frame,
            text="✊  GRIP (CLOSE)",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=SUCCESS,
            hover_color="#059669",
            text_color="white",
            height=45,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(16, 2),
        ).grid(row=0, column=0, padx=(0, 4), sticky="nsew")

        # RELEASE button
        ctk.CTkButton(
            grip_frame,
            text="🖐  RELEASE (OPEN)",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color="#475569",
            hover_color="#64748B",
            text_color="white",
            height=45,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(16, 3),
        ).grid(row=0, column=1, padx=(4, 0), sticky="nsew")

        # STOP button
        ctk.CTkButton(
            btn_frame,
            text="⏹  STOP ROBOT",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color=DANGER,
            hover_color="#DC2626",
            text_color="white",
            height=45,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(16, 4),
        ).pack(fill="x", pady=(10, 5))

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

    def _select_forward_mode(self) -> None:
        """Switch to Forward Kinematics mode and pulse PLC address 16.5."""
        if self._kine_mode != "Forward":
            self._kine_mode = "Forward"
            self.lbl_kine_mode.configure(
                text="▶ FORWARD KINEMATICS",
                text_color=SUCCESS,
            )
            # Update button styles
            self.btn_thuan.configure(
                fg_color=SUCCESS, hover_color="#059669",
                text_color="white", border_width=0,
            )
            self.btn_nghich.configure(
                fg_color="transparent", hover_color="#475569",
                text_color=TEXT_PRIMARY, border_color="#475569", border_width=1,
            )
            # Hide IK entries, show FK entries
            self._ik_frame.pack_forget()
            self._fk_frame.pack(fill="both", expand=True)
        # Always pulse PLC bit 16.5 (DONG_HOC_THUAN)
        self.controller.plc.send_pulse(16, 5)

    def _select_inverse_mode(self) -> None:
        """Switch to Inverse Kinematics mode and pulse PLC address 16.6."""
        if self._kine_mode != "Inverse":
            self._kine_mode = "Inverse"
            self.lbl_kine_mode.configure(
                text="▶ INVERSE KINEMATICS",
                text_color="#3B82F6",
            )
            # Update button styles
            self.btn_nghich.configure(
                fg_color="#3B82F6", hover_color="#2563EB",
                text_color="white", border_width=0,
            )
            self.btn_thuan.configure(
                fg_color="transparent", hover_color="#475569",
                text_color=TEXT_PRIMARY, border_color="#475569", border_width=1,
            )
            # Hide FK entries, show IK entries
            self._fk_frame.pack_forget()
            self._ik_frame.pack(fill="both", expand=True)
        # Always pulse PLC bit 16.6 (DONG_HOC_NGHICH)
        self.controller.plc.send_pulse(16, 6)

    def _calculate(self) -> None:
        """Calculate kinematics and display results in the result column."""
        if self._kine_mode == "Forward":
            self._calculate_forward()
        else:
            self._calculate_inverse()

    def _calculate_forward(self) -> None:
        """Compute FK from θ₁(°), θ₂(°), θ₃(°), θ₄(°) and display X, Y, Z."""
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

        if not self._validate_joints(j1, j2, j3, j4):
            return

        # Compute FK: (θ₁°, θ₂°, θ₃°, θ₄°) → (X, Y, Z)
        x, y, z = forward_kinematics(j1, j2, j3, j4)

        # Store computed result for later sending
        self._last_fk_result = (j1, j2, j3, j4)
        self._last_fk_coords = (x, y, z)

        # Update result labels
        self.lbl_fk_x.configure(text=f"{x:.2f} mm", text_color=SUCCESS)
        self.lbl_fk_y.configure(text=f"{y:.2f} mm", text_color=SUCCESS)
        self.lbl_fk_z.configure(text=f"{z:.2f} mm", text_color=SUCCESS)

        log.info("FK Calculate: θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° θ₄=%.2f° → X=%.2f Y=%.2f Z=%.2f",
                 j1, j2, j3, j4, x, y, z)

    def _calculate_inverse(self) -> None:
        """Compute IK from X, Y, Z inputs and display θ₁, θ₂, θ₃, θ₄."""
        try:
            x = float(self.ent_x.get().strip())
            y = float(self.ent_y.get().strip())
            z = float(self.ent_z.get().strip())
        except ValueError:
            messagebox.showerror(
                "Input Error",
                "X, Y, Z fields must contain valid decimal numbers.",
            )
            return

        try:
            # IK: (X, Y, Z) → (θ₁°, θ₂°, θ₃°, θ₄°)
            j1, j2, j3, j4 = inverse_kinematics(x, y, z, phi=self.controller.cfg.kinematics.default_phi)

            # Store computed result for later sending
            self._last_ik_result = (j1, j2, j3, j4)
            self._last_ik_coords = (x, y, z)

            # Update result labels
            self.lbl_ik_j1.configure(text=f"{j1:.2f} °", text_color=SUCCESS)
            self.lbl_ik_j2.configure(text=f"{j2:.2f} °", text_color=SUCCESS)
            self.lbl_ik_j3.configure(text=f"{j3:.2f} °", text_color=SUCCESS)

            log.info("IK Calculate: X=%.2f Y=%.2f Z=%.2f → θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° θ₄=%.2f°",
                     x, y, z, j1, j2, j3, j4)

        except Exception as exc:
            self.lbl_ik_j1.configure(text="ERR", text_color=DANGER)
            self.lbl_ik_j2.configure(text="ERR", text_color=DANGER)
            self.lbl_ik_j3.configure(text="ERR", text_color=DANGER)
            self._last_ik_result = None
            self._last_ik_coords = None
            messagebox.showerror("IK Error", f"Cannot compute inverse kinematics:\n{exc}")

    def _send_positions(self) -> None:
        """Send the last computed results to PLC."""
        if self._kine_mode == "Forward":
            self._send_forward_kinematics()
        else:
            self._send_inverse_kinematics()

    def _send_forward_kinematics(self) -> None:
        """Send joint values and computed coordinates to PLC."""
        if self._last_fk_result is None or self._last_fk_coords is None:
            messagebox.showwarning(
                "No Result",
                "Please click CALCULATE first to compute kinematics.",
            )
            return

        j1, j2, j3, j4 = self._last_fk_result
        x, y, z = self._last_fk_coords

        if not self._validate_joints(j1, j2, j3, j4):
            return

        self.controller.plc.send_forward_kinematics_data(j1, j2, j3, x, y, z)
        log.info("FK Send: θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° (XY HT = [%.2f, %.2f, %.2f])", j1, j2, j3, x, y, z)
        messagebox.showinfo(
            "Success",
            f"Sent to PLC:\nθ₁={j1:.2f}° (Base)\nθ₂={j2:.2f}° (Shoulder)\nθ₃={j3:.2f}° (Elbow)\n\n"
            f"Computed Coordinates (HT):\nX={x:.2f} mm\nY={y:.2f} mm\nZ={z:.2f} mm",
        )

    def _send_inverse_kinematics(self) -> None:
        """Send computed IK joint values and input coordinates to PLC."""
        if self._last_ik_result is None or self._last_ik_coords is None:
            messagebox.showwarning(
                "No Result",
                "Please click CALCULATE first to compute kinematics.",
            )
            return

        j1, j2, j3, j4 = self._last_ik_result
        x, y, z = self._last_ik_coords

        if not self._validate_joints(j1, j2, j3, j4):
            return

        self.controller.plc.send_inverse_kinematics_data(x, y, z, j1, j2, j3)
        log.info("IK Send: XYZ=[%.2f, %.2f, %.2f] (Theta HT = [%.2f, %.2f, %.2f])", x, y, z, j1, j2, j3)
        messagebox.showinfo(
            "Success",
            f"Sent to PLC:\nCoordinates:\nX={x:.2f} mm\nY={y:.2f} mm\nZ={z:.2f} mm\n\n"
            f"Computed Joint Angles (HT):\nθ₁={j1:.2f}° (Base)\nθ₂={j2:.2f}° (Shoulder)\nθ₃={j3:.2f}° (Elbow)",
        )

    def _validate_joints(self, j1: float, j2: float, j3: float, j4: float) -> bool:
        """Validate joint angle ranges to prevent damage or PLC faults."""
        if not (-180.0 <= j1 <= 180.0):
            messagebox.showerror("Input Error", f"θ₁ must be between -180° and 180° (got {j1:.2f}°)")
            return False
        if not (0.0 <= j2 <= 250.0):
            messagebox.showerror("Input Error", f"θ₂ must be between 0° and 250° (got {j2:.2f}°)")
            return False
        if not (0.0 <= j3 <= 200.0):
            messagebox.showerror("Input Error", f"θ₃ must be between 0° and 200° (got {j3:.2f}°)")
            return False
        if not (-180.0 <= j4 <= 180.0):
            messagebox.showerror("Input Error", f"θ₄ must be between -180° and 180° (got {j4:.2f}°)")
            return False
        return True

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh the error status label from *data*."""
        self._refresh_error_status(data)
