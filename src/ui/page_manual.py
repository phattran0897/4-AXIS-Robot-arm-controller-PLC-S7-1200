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

from src.kinematics import JOINT_LIMITS, forward_kinematics, inverse_kinematics
from src.plc.plc_controller import ADDR
from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    DANGER,
    DANGER_HOVER,
    ENTRY_BG,
    INFO,
    MANUAL_ACCENT,
    PANEL_BORDER,
    SUCCESS,
    SUCCESS_HOVER,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
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
    │  (FK/IK Toggle)        │  (Home/Gripper)     │                │
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
        # JOG toggle states and button references (keys == PLC bit addresses)
        self._jog_states: dict[tuple[int, int], bool] = {
            ADDR.JOG_J1_LEFT: False,
            ADDR.JOG_J1_RIGHT: False,
            ADDR.JOG_J2_UP: False,
            ADDR.JOG_J2_DOWN: False,
            ADDR.JOG_J3_UP: False,
            ADDR.JOG_J3_DOWN: False,
        }
        self._jog_buttons: dict[tuple[int, int], ctk.CTkButton] = {}
        self._build_ui()

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

        self.build_section_header(frame, "🎯", "KINEMATICS INPUT", MANUAL_ACCENT)

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
            hover_color=SUCCESS_HOVER,
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
        entry_bg = ENTRY_BG
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

        self.ent_j1 = self._make_joint_entry(
            fk_left, "θ₁ - Base (°)", entry_font, entry_bg, entry_fg
        )
        self.ent_j2 = self._make_joint_entry(
            fk_left, "θ₂ - Shoulder (°)", entry_font, entry_bg, entry_fg
        )
        self.ent_j3 = self._make_joint_entry(
            fk_left, "θ₃ - Elbow (°)", entry_font, entry_bg, entry_fg
        )
        self.ent_j4 = self._make_joint_entry(
            fk_left, "θ₄ - Wrist (°)", entry_font, entry_bg, entry_fg
        )

        # ── FK Right Column: RESULT ──
        fk_right = ctk.CTkFrame(fk_columns, fg_color="transparent")
        fk_right.grid(row=0, column=1, padx=(4, 0), sticky="nsew")

        ctk.CTkLabel(
            fk_right,
            text="📤 RESULT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=SUCCESS,
        ).pack(anchor="w", pady=(0, 4))

        self.lbl_fk_x = self.create_data_row(fk_right, "X", "--- mm", SUCCESS)
        self.lbl_fk_y = self.create_data_row(fk_right, "Y", "--- mm", SUCCESS)
        self.lbl_fk_z = self.create_data_row(fk_right, "Z", "--- mm", SUCCESS)

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

        self.ent_x = self._make_coord_entry(
            ik_left, "X (mm)", entry_font, entry_bg, entry_fg
        )
        self.ent_y = self._make_coord_entry(
            ik_left, "Y (mm)", entry_font, entry_bg, entry_fg
        )
        self.ent_z = self._make_coord_entry(
            ik_left, "Z (mm)", entry_font, entry_bg, entry_fg
        )

        # ── IK Right Column: RESULT ──
        ik_right = ctk.CTkFrame(ik_columns, fg_color="transparent")
        ik_right.grid(row=0, column=1, padx=(4, 0), sticky="nsew")

        ctk.CTkLabel(
            ik_right,
            text="📤 RESULT",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=SUCCESS,
        ).pack(anchor="w", pady=(0, 4))

        self.lbl_ik_j1 = self.create_data_row(ik_right, "θ₁", "--- °", SUCCESS)
        self.lbl_ik_j2 = self.create_data_row(ik_right, "θ₂", "--- °", SUCCESS)
        self.lbl_ik_j3 = self.create_data_row(ik_right, "θ₃", "--- °", SUCCESS)
        self.lbl_ik_j4 = self.create_data_row(ik_right, "θ₄", "--- °", SUCCESS)

        # ── Action buttons row ──────────────────────────────────────────
        btn_row = ctk.CTkFrame(entry_frame, fg_color="transparent")
        btn_row.pack(fill="x", pady=(8, 5))
        btn_row.grid_columnconfigure(0, weight=1, uniform="btn")
        btn_row.grid_columnconfigure(1, weight=1, uniform="btn")

        # CALCULATE button
        ctk.CTkButton(
            btn_row,
            text="🔢   CALCULATE",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=INFO,
            hover_color="#2563EB",
            text_color="white",
            height=42,
            corner_radius=10,
            command=self._calculate,
        ).grid(row=0, column=0, padx=(0, 4), sticky="ew")

        # SEND TARGETS button
        ctk.CTkButton(
            btn_row,
            text="✈   SEND",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=MANUAL_ACCENT,
            hover_color="#9333EA",
            text_color="white",
            height=42,
            corner_radius=10,
            command=self._send_positions,
        ).grid(row=0, column=1, padx=(4, 0), sticky="ew")

    def _make_joint_entry(
        self,
        parent: ctk.CTkFrame,
        placeholder: str,
        font: ctk.CTkFont,
        bg: str,
        fg: str,
    ) -> ctk.CTkEntry:
        """Create one styled joint-angle entry field."""
        entry = ctk.CTkEntry(
            parent,
            placeholder_text=placeholder,
            font=font,
            fg_color=bg,
            text_color=fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        entry.pack(pady=3, fill="x")
        return entry

    def _make_coord_entry(
        self,
        parent: ctk.CTkFrame,
        placeholder: str,
        font: ctk.CTkFont,
        bg: str,
        fg: str,
    ) -> ctk.CTkEntry:
        """Create one styled Cartesian coordinate entry field."""
        entry = ctk.CTkEntry(
            parent,
            placeholder_text=placeholder,
            font=font,
            fg_color=bg,
            text_color=fg,
            placeholder_text_color=TEXT_SECONDARY,
            border_color=PANEL_BORDER,
            height=36,
            corner_radius=10,
        )
        entry.pack(pady=4, fill="x")
        return entry

    def _build_actuator_panel(self, parent: ctk.CTkFrame) -> None:
        """Build the actuator/gripper control panel."""
        frame = self._create_card(parent)
        frame.grid(row=0, column=1, padx=8, pady=8, sticky="nsew")

        self.build_section_header(frame, "🦾", "ACTUATOR CONTROL", MANUAL_ACCENT)

        # Buttons container
        btn_frame = ctk.CTkFrame(frame, fg_color="transparent")
        btn_frame.pack(fill="both", expand=True, padx=15, pady=(6, 12))

        # HOME button
        ctk.CTkButton(
            btn_frame,
            text="🏠   MOVE TO HOME",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color=INFO,
            hover_color="#2563EB",
            text_color="white",
            height=46,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(*ADDR.MOVE_TO_HOME),
        ).pack(fill="x", pady=(4, 10))

        # Pick / Release row
        grip_frame = ctk.CTkFrame(btn_frame, fg_color="transparent")
        grip_frame.pack(fill="x", pady=5)
        grip_frame.grid_columnconfigure(0, weight=1, uniform="grip")
        grip_frame.grid_columnconfigure(1, weight=1, uniform="grip")

        # PICK button (GAP_VAT – close gripper & pick)
        ctk.CTkButton(
            grip_frame,
            text="✊   GẮP VẬT",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=SUCCESS,
            hover_color=SUCCESS_HOVER,
            text_color="white",
            height=46,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(*ADDR.GAP_VAT),
        ).grid(row=0, column=0, padx=(0, 4), sticky="nsew")

        # RELEASE button (NHA_VAT – open gripper & release)
        ctk.CTkButton(
            grip_frame,
            text="🖐   NHẢ VẬT",
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color="#475569",
            hover_color="#64748B",
            text_color="white",
            height=46,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(*ADDR.NHA_VAT),
        ).grid(row=0, column=1, padx=(4, 0), sticky="nsew")

        # STOP button
        ctk.CTkButton(
            btn_frame,
            text="⏹   STOP ROBOT",
            font=ctk.CTkFont(size=13, weight="bold"),
            fg_color=DANGER,
            hover_color=DANGER_HOVER,
            text_color="white",
            height=46,
            corner_radius=10,
            command=lambda: self.controller.plc.send_pulse(*ADDR.STOP_ROBOT),
        ).pack(fill="x", pady=(10, 5))

        # ── JOG CONTROL section ─────────────────────────────────────────
        jog_header = ctk.CTkFrame(btn_frame, fg_color="transparent")
        jog_header.pack(fill="x", pady=(14, 5))

        ctk.CTkLabel(
            jog_header,
            text="🕹",
            font=ctk.CTkFont(size=16),
            text_color=MANUAL_ACCENT,
        ).pack(side="left", padx=(0, 6))

        ctk.CTkLabel(
            jog_header,
            text="JOG CONTROL",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=MANUAL_ACCENT,
        ).pack(side="left")

        # JOG button definitions: (label_left, addr_left, label_right, addr_right)
        jog_rows = [
            ("J1", "◀ QUAY TRÁI", ADDR.JOG_J1_LEFT, "QUAY PHẢI ▶", ADDR.JOG_J1_RIGHT),
            ("J2", "▲ LÊN", ADDR.JOG_J2_UP, "XUỐNG ▼", ADDR.JOG_J2_DOWN),
            ("J3", "▲ LÊN", ADDR.JOG_J3_UP, "XUỐNG ▼", ADDR.JOG_J3_DOWN),
        ]

        for axis, lbl_l, addr_l, lbl_r, addr_r in jog_rows:
            # Axis label
            ctk.CTkLabel(
                btn_frame,
                text=axis,
                font=ctk.CTkFont(size=10, weight="bold"),
                text_color=TEXT_SECONDARY,
            ).pack(anchor="w", pady=(6, 1))

            row_frame = ctk.CTkFrame(btn_frame, fg_color="transparent")
            row_frame.pack(fill="x", pady=1)
            row_frame.grid_columnconfigure(0, weight=1, uniform="jog")
            row_frame.grid_columnconfigure(1, weight=1, uniform="jog")

            # Left button
            btn_left = ctk.CTkButton(
                row_frame,
                text=f"{lbl_l} [OFF]",
                font=ctk.CTkFont(size=11, weight="bold"),
                fg_color="#475569",
                hover_color="#64748B",
                text_color="white",
                height=38,
                corner_radius=8,
                command=lambda a=addr_l, t=lbl_l: self._toggle_jog(a, t),
            )
            btn_left.grid(row=0, column=0, padx=(0, 3), sticky="nsew")
            self._jog_buttons[addr_l] = btn_left

            # Right button
            btn_right = ctk.CTkButton(
                row_frame,
                text=f"{lbl_r} [OFF]",
                font=ctk.CTkFont(size=11, weight="bold"),
                fg_color="#475569",
                hover_color="#64748B",
                text_color="white",
                height=38,
                corner_radius=8,
                command=lambda a=addr_r, t=lbl_r: self._toggle_jog(a, t),
            )
            btn_right.grid(row=0, column=1, padx=(3, 0), sticky="nsew")
            self._jog_buttons[addr_r] = btn_right

    def _toggle_jog(self, addr: tuple[int, int], label: str) -> None:
        """Toggle a JOG button state and send the new value to PLC."""
        new_state = not self._jog_states[addr]
        self._jog_states[addr] = new_state
        self.controller.plc.write_bit(addr[0], addr[1], new_state)

        btn = self._jog_buttons[addr]
        if new_state:
            btn.configure(
                text=f"{label} [ON]",
                fg_color=SUCCESS,
                hover_color=SUCCESS_HOVER,
            )
        else:
            btn.configure(
                text=f"{label} [OFF]",
                fg_color="#475569",
                hover_color="#64748B",
            )
        log.info("JOG toggle %d.%d → %s", addr[0], addr[1], new_state)
        try:
            self.controller.log_tx(
                f"JOG {label} → {'ON' if new_state else 'OFF'} (DB {addr[0]}.{addr[1]})"
            )
        except Exception:
            pass  # log_tx may not exist in test/mock contexts

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
                fg_color=SUCCESS,
                hover_color=SUCCESS_HOVER,
                text_color="white",
                border_width=0,
            )
            self.btn_nghich.configure(
                fg_color="transparent",
                hover_color="#475569",
                text_color=TEXT_PRIMARY,
                border_color="#475569",
                border_width=1,
            )
            # Hide IK entries, show FK entries
            self._ik_frame.pack_forget()
            self._fk_frame.pack(fill="both", expand=True)
        # Always pulse PLC bit 16.5 (DONG_HOC_THUAN)
        self.controller.plc.send_pulse(*ADDR.KINE_FORWARD)

    def _select_inverse_mode(self) -> None:
        """Switch to Inverse Kinematics mode and pulse PLC address 16.6."""
        if self._kine_mode != "Inverse":
            self._kine_mode = "Inverse"
            self.lbl_kine_mode.configure(
                text="▶ INVERSE KINEMATICS",
                text_color=INFO,
            )
            # Update button styles
            self.btn_nghich.configure(
                fg_color=INFO,
                hover_color="#2563EB",
                text_color="white",
                border_width=0,
            )
            self.btn_thuan.configure(
                fg_color="transparent",
                hover_color="#475569",
                text_color=TEXT_PRIMARY,
                border_color="#475569",
                border_width=1,
            )
            # Hide FK entries, show IK entries
            self._fk_frame.pack_forget()
            self._ik_frame.pack(fill="both", expand=True)
        # Always pulse PLC bit 16.6 (DONG_HOC_NGHICH)
        self.controller.plc.send_pulse(*ADDR.KINE_INVERSE)

    def _calculate(self) -> None:
        """Calculate kinematics and display results in the result column."""
        if self._kine_mode == "Forward":
            self._calculate_forward()
        else:
            self._calculate_inverse()

    @staticmethod
    def _entry_float(entry: Any, default: float = 0.0) -> float:
        """Parse an entry's contents as float; empty string → *default*."""
        text = entry.get().strip()
        return float(text) if text else default

    def _calculate_forward(self) -> None:
        """Compute FK from θ₁(°), θ₂(°), θ₃(°), θ₄(°) and display X, Y, Z."""
        try:
            j1 = self._entry_float(self.ent_j1)
            j2 = self._entry_float(self.ent_j2)
            j3 = self._entry_float(self.ent_j3)
            j4 = self._entry_float(self.ent_j4)
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

        # Tự động gán kết quả FK sang làm đầu vào chờ sẵn cho phần IK
        for entry, value in ((self.ent_x, x), (self.ent_y, y), (self.ent_z, z)):
            entry.delete(0, "end")
            entry.insert(0, f"{value:.2f}")

        log.info(
            "FK Calculate: θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° θ₄=%.2f° → X=%.2f Y=%.2f Z=%.2f",
            j1,
            j2,
            j3,
            j4,
            x,
            y,
            z,
        )
        try:
            self.controller.log_tx(
                f"FK Calc: J=[{j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}]° → "
                f"XYZ=[{x:.1f}, {y:.1f}, {z:.1f}]mm"
            )
        except Exception:
            pass

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
            j1, j2, j3, j4 = inverse_kinematics(
                x, y, z, phi=self.controller.cfg.kinematics.default_phi
            )

            # Store computed result for later sending
            self._last_ik_result = (j1, j2, j3, j4)
            self._last_ik_coords = (x, y, z)

            # Update result labels
            for label, value in (
                (self.lbl_ik_j1, j1),
                (self.lbl_ik_j2, j2),
                (self.lbl_ik_j3, j3),
                (self.lbl_ik_j4, j4),
            ):
                label.configure(text=f"{value:.2f} °", text_color=SUCCESS)

            log.info(
                "IK Calculate: X=%.2f Y=%.2f Z=%.2f → "
                "θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° θ₄=%.2f°",
                x,
                y,
                z,
                j1,
                j2,
                j3,
                j4,
            )

        except Exception as exc:
            for label in (
                self.lbl_ik_j1,
                self.lbl_ik_j2,
                self.lbl_ik_j3,
                self.lbl_ik_j4,
            ):
                label.configure(text="ERR", text_color=DANGER)
            self._last_ik_result = None
            self._last_ik_coords = None
            messagebox.showerror(
                "IK Error", f"Cannot compute inverse kinematics:\n{exc}"
            )

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
        log.info(
            "FK Send: θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° (XY HT = [%.2f, %.2f, %.2f])",
            j1,
            j2,
            j3,
            x,
            y,
            z,
        )
        try:
            self.controller.log_tx(
                f"FK Send → PLC: J=[{j1:.1f}, {j2:.1f}, {j3:.1f}]° "
                f"XYZ=[{x:.1f}, {y:.1f}, {z:.1f}]mm"
            )
        except Exception:
            pass
        messagebox.showinfo(
            "Success",
            f"Sent to PLC:\nθ₁={j1:.2f}° (Base)\nθ₂={j2:.2f}° (Shoulder)\n"
            f"θ₃={j3:.2f}° (Elbow)\n\n"
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
        log.info(
            "IK Send: XYZ=[%.2f, %.2f, %.2f] (Theta HT = [%.2f, %.2f, %.2f])",
            x,
            y,
            z,
            j1,
            j2,
            j3,
        )
        try:
            self.controller.log_tx(
                f"IK Send → PLC: XYZ=[{x:.1f}, {y:.1f}, {z:.1f}]mm "
                f"J=[{j1:.1f}, {j2:.1f}, {j3:.1f}]°"
            )
        except Exception:
            pass
        messagebox.showinfo(
            "Success",
            f"Sent to PLC:\nCoordinates:\nX={x:.2f} mm\nY={y:.2f} mm\nZ={z:.2f} mm\n\n"
            f"Computed Joint Angles (HT):\nθ₁={j1:.2f}° (Base)\n"
            f"θ₂={j2:.2f}° (Shoulder)\nθ₃={j3:.2f}° (Elbow)",
        )

    def _validate_joints(self, j1: float, j2: float, j3: float, j4: float) -> bool:
        """
        Validate joint angle ranges against the configured limits
        (config.yaml → kinematics module) to prevent damage or PLC faults.
        """
        names = ("θ₁", "θ₂", "θ₃", "θ₄")
        keys = ("j1", "j2", "j3", "j4")
        for name, key, value in zip(names, keys, (j1, j2, j3, j4)):
            low, high = JOINT_LIMITS[key]
            if not (low <= value <= high):
                messagebox.showerror(
                    "Input Error",
                    f"{name} must be between {low:g}° and {high:g}° (got {value:.2f}°)",
                )
                return False
        return True

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh the error status label from *data*."""
        self._refresh_error_status(data)
