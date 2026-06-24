"""
src/ui/header.py – Modern professional header with VAA branding.

Provides a consistent header component across all pages.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import customtkinter as ctk
from PIL import Image, ImageDraw, ImageTk

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


# Brand colors
VAA_PRIMARY: str = "#0A1628"       # Deep navy
VAA_ACCENT: str = "#00D4FF"        # Cyan accent
VAA_SECONDARY: str = "#1E3A5F"    # Medium navy
VAA_GOLD: str = "#FFD700"          # Gold for highlights
VAA_TEXT: str = "#FFFFFF"          # White text
VAA_TEXT_DIM: str = "#94A3B8"      # Dimmed text


class VAAHeader(ctk.CTkFrame):
    """
    Professional header with VAA branding and navigation tabs.

    Layout:
    ┌─────────────────────────────────────────────────────────────────┐
    │ [LOGO]  VIETNAM AVIATION ACADEMY          [Auto] [Manual]     │
    │         4-Axis Robot Control System                               │
    └─────────────────────────────────────────────────────────────────┘
    """

    def __init__(
        self,
        parent: ctk.CTkFrame,
        controller: "RobotApp",
        current_page: str = "PageAuto",
    ) -> None:
        super().__init__(
            parent,
            fg_color=VAA_PRIMARY,
            height=80,
            corner_radius=0,
        )
        self.controller = controller
        self.pack(fill="x", padx=0, pady=0)
        self.pack_propagate(False)

        self._build_layout(current_page)

    def _create_text_logo(self) -> ImageTk.PhotoImage:
        """Create a professional text-based VAA logo."""
        img = Image.new("RGBA", (200, 60), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        # Draw accent line
        draw.rectangle([0, 50, 200, 54], fill=self._hex_to_rgb(VAA_ACCENT))

        return ImageTk.PhotoImage(img)

    def _hex_to_rgb(self, hex_color: str) -> tuple:
        """Convert hex to RGB tuple."""
        hex_color = hex_color.lstrip("#")
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

    def _build_layout(self, current_page: str) -> None:
        import os

        # Left section - Logo + Branding
        left_frame = ctk.CTkFrame(self, fg_color="transparent")
        left_frame.pack(side="left", padx=15, pady=8, fill="both", expand=True)

        # Logo image
        logo_path = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "vaa_logo.png")
        try:
            logo_img = Image.open(logo_path)
            logo_img = logo_img.resize((140, 50), Image.LANCZOS)
            self._logo_tk = ctk.CTkImage(logo_img, size=(140, 50))
            logo_label = ctk.CTkLabel(left_frame, image=self._logo_tk, text="")
            logo_label.pack(side="left", padx=(0, 15))
        except (FileNotFoundError, OSError, Image.UnidentifiedImageError) as exc:
            log.debug("Logo not loaded: %s (%s)", logo_path, exc)

        # Text branding (beside logo)
        text_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
        text_frame.pack(side="left", fill="y", pady=2)

        # Academy name
        lbl_academy = ctk.CTkLabel(
            text_frame,
            text="HỌC VIỆN HÀNG KHÔNG VIỆT NAM",
            font=ctk.CTkFont(size=18, weight="bold"),
            text_color=VAA_ACCENT,
        )
        lbl_academy.pack(anchor="w")

        # English name
        lbl_english = ctk.CTkLabel(
            text_frame,
            text="VIETNAM AVIATION ACADEMY",
            font=ctk.CTkFont(size=11, weight="bold"),
            text_color=VAA_TEXT_DIM,
        )
        lbl_english.pack(anchor="w")

        # System name
        lbl_system = ctk.CTkLabel(
            text_frame,
            text="4-AXIS SCARA ROBOT CONTROL SYSTEM",
            font=ctk.CTkFont(size=13, weight="normal"),
            text_color=VAA_TEXT,
        )
        lbl_system.pack(anchor="w", pady=(5, 0))

        # Right section - Navigation tabs
        right_frame = ctk.CTkFrame(self, fg_color="transparent")
        right_frame.pack(side="right", padx=20, pady=10)

        # Navigation buttons
        self._btn_auto = ctk.CTkButton(
            right_frame,
            text="AUTO MODE",
            width=120,
            height=36,
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=VAA_SECONDARY if current_page != "PageAuto" else VAA_ACCENT,
            hover_color=VAA_SECONDARY,
            text_color=VAA_TEXT if current_page != "PageAuto" else VAA_PRIMARY,
            corner_radius=8,
            command=lambda: self.controller.show_frame("PageAuto"),
        )
        self._btn_auto.pack(side="left", padx=5)

        self._btn_manual = ctk.CTkButton(
            right_frame,
            text="MANUAL MODE",
            width=120,
            height=36,
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=VAA_SECONDARY if current_page != "PageManual" else VAA_ACCENT,
            hover_color=VAA_SECONDARY,
            text_color=VAA_TEXT if current_page != "PageManual" else VAA_PRIMARY,
            corner_radius=8,
            command=lambda: self.controller.show_frame("PageManual"),
        )
        self._btn_manual.pack(side="left", padx=5)

    def update_active_tab(self, page_name: str) -> None:
        """Update tab highlighting based on current page."""
        if page_name == "PageAuto":
            self._btn_auto.configure(
                fg_color=VAA_ACCENT,
                text_color=VAA_PRIMARY,
            )
            self._btn_manual.configure(
                fg_color=VAA_SECONDARY,
                text_color=VAA_TEXT,
            )
        else:
            self._btn_manual.configure(
                fg_color=VAA_ACCENT,
                text_color=VAA_PRIMARY,
            )
            self._btn_auto.configure(
                fg_color=VAA_SECONDARY,
                text_color=VAA_TEXT,
            )


class VAAFooter(ctk.CTkFrame):
    """
    Professional footer with status indicators and version info.
    """

    def __init__(self, parent: ctk.CTkFrame) -> None:
        super().__init__(
            parent,
            fg_color=VAA_PRIMARY,
            height=40,
            corner_radius=0,
        )
        self.pack(fill="x", padx=0, pady=0)
        self.pack_propagate(False)

        self._build_layout()

    def _build_layout(self) -> None:
        # Use pack manager since parent header uses pack (not grid)
        # Left - Status
        left_frame = ctk.CTkFrame(self, fg_color="transparent")
        left_frame.pack(side="left", padx=20, fill="both", expand=True)

        self.lbl_status = ctk.CTkLabel(
            left_frame,
            text="● SYSTEM READY",
            font=ctk.CTkFont(size=11, weight="normal"),
            text_color="#22C55E",
        )
        self.lbl_status.pack(side="left", pady=5)

        self.lbl_plc = ctk.CTkLabel(
            left_frame,
            text="PLC: DISCONNECTED",
            font=ctk.CTkFont(size=11, weight="normal"),
            text_color="#EF4444",
        )
        self.lbl_plc.pack(side="left", padx=20, pady=5)

        self.lbl_camera = ctk.CTkLabel(
            left_frame,
            text="CAM: OFFLINE",
            font=ctk.CTkFont(size=11, weight="normal"),
            text_color="#EF4444",
        )
        self.lbl_camera.pack(side="left", padx=20, pady=5)

        # Right - Version
        right_frame = ctk.CTkFrame(self, fg_color="transparent")
        right_frame.pack(side="right", padx=20)

        lbl_version = ctk.CTkLabel(
            right_frame,
            text="v1.0.0 | VAA Robot System",
            font=ctk.CTkFont(size=10, weight="normal"),
            text_color=VAA_TEXT_DIM,
        )
        lbl_version.pack(side="right", pady=5)

    def update_status(self, plc_connected: bool, camera_active: bool, system_ok: bool) -> None:
        """Update footer status indicators."""
        if system_ok:
            self.lbl_status.configure(text="● SYSTEM READY", text_color="#22C55E")
        else:
            self.lbl_status.configure(text="● SYSTEM ERROR", text_color="#EF4444")

        if plc_connected:
            self.lbl_plc.configure(text="PLC: CONNECTED", text_color="#22C55E")
        else:
            self.lbl_plc.configure(text="PLC: DISCONNECTED", text_color="#EF4444")

        if camera_active:
            self.lbl_camera.configure(text="CAM: ACTIVE", text_color="#22C55E")
        else:
            self.lbl_camera.configure(text="CAM: OFFLINE", text_color="#EF4444")
