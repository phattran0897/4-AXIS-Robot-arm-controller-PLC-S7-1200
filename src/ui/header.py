"""
src/ui/header.py – Modern professional header with VAA branding.

Provides a consistent header and footer component across all pages.
"""

from __future__ import annotations

import logging
import os
from typing import TYPE_CHECKING

import customtkinter as ctk
from PIL import Image, ImageTk

from src.ui.theme import DANGER, SUCCESS, TEXT_SECONDARY

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


# Brand colors
VAA_PRIMARY: str = "#0A1628"  # Deep navy
VAA_ACCENT: str = "#00D4FF"  # Cyan accent
VAA_SECONDARY: str = "#1E3A5F"  # Medium navy
VAA_TEXT: str = "#FFFFFF"  # White text
VAA_TEXT_DIM: str = "#94A3B8"  # Dimmed text


def get_logo_path() -> str:
    """Filesystem path of the brand logo asset (project-relative)."""
    return os.path.join(os.path.dirname(__file__), "..", "..", "assets", "vaa_logo.png")


def apply_app_icon(window: ctk.CTk | ctk.CTkFrame, path: str | None = None) -> bool:
    """
    Register the brand logo as the OS window / taskbar icon.

    Parameters
    ----------
    window:
        The Tk root window (``RobotApp``).
    path:
        Explicit icon image path; defaults to :func:`get_logo_path`.

    Returns
    -------
    bool
        ``True`` when the icon was applied.
    """
    icon_path = path or get_logo_path()
    try:
        img = Image.open(icon_path)
        photo = ImageTk.PhotoImage(img)
        window.iconphoto(True, photo)
        # Keep a reference – Tk does not protect the PhotoImage from GC.
        window._app_icon_ref = photo
        log.info("App icon set from %s", icon_path)
        return True
    except Exception as exc:  # Missing/corrupt asset must never block start-up
        log.debug("App icon not applied (%s): %s", icon_path, exc)
        return False


class VAAHeader(ctk.CTkFrame):
    """
    Professional header with VAA branding and navigation tabs.

    Layout:
    ┌─────────────────────────────────────────────────────────────────┐
    │ [LOGO]  HỌC VIỆN HÀNG KHÔNG VIỆT NAM     [AUTO] [MANUAL]        │
    │         4-AXIS ROBOT CONTROL SYSTEM                             │
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
            height=78,
            corner_radius=0,
        )
        self.controller = controller
        self.pack(fill="x", padx=0, pady=0)
        self.pack_propagate(False)

        self._build_layout(current_page)

    def _build_layout(self, current_page: str) -> None:
        # Left section - Logo + Branding
        left_frame = ctk.CTkFrame(self, fg_color="transparent")
        left_frame.pack(side="left", padx=15, pady=8, fill="both", expand=True)

        # Logo image (optional asset)
        logo_path = get_logo_path()
        try:
            logo_src = Image.open(logo_path)
            orig_w, orig_h = logo_src.size
            target_h = 50
            target_w = int(orig_w * (target_h / orig_h)) if orig_h > 0 else 140
            logo_img = logo_src.resize((target_w, target_h), Image.Resampling.LANCZOS)
            self._logo_tk = ctk.CTkImage(logo_img, size=(target_w, target_h))
            logo_label = ctk.CTkLabel(left_frame, image=self._logo_tk, text="")
            logo_label.pack(side="left", padx=(0, 15))
        except (OSError, Image.UnidentifiedImageError) as exc:
            log.debug("Logo not loaded: %s (%s)", logo_path, exc)
            # Accent bar placeholder keeps the brand mark present without an asset
            ctk.CTkFrame(
                left_frame, width=6, height=54, fg_color=VAA_ACCENT, corner_radius=3
            ).pack(side="left", padx=(0, 14), pady=2)

        # Text branding (beside logo)
        text_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
        text_frame.pack(side="left", fill="y", pady=2)

        # Academy name
        ctk.CTkLabel(
            text_frame,
            text="HỌC VIỆN HÀNG KHÔNG VIỆT NAM",
            font=ctk.CTkFont(size=17, weight="bold"),
            text_color=VAA_ACCENT,
        ).pack(anchor="w")

        # English name + system name on one line for a tighter hierarchy
        subtitle_row = ctk.CTkFrame(text_frame, fg_color="transparent")
        subtitle_row.pack(anchor="w", pady=(3, 0))

        ctk.CTkLabel(
            subtitle_row,
            text="VIETNAM AVIATION ACADEMY",
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=VAA_TEXT_DIM,
        ).pack(side="left")

        ctk.CTkLabel(
            subtitle_row,
            text="  |  ",
            font=ctk.CTkFont(size=10),
            text_color="#334155",
        ).pack(side="left")

        ctk.CTkLabel(
            subtitle_row,
            text="4-AXIS INDUSTRIAL ROBOT CONTROL SYSTEM",
            font=ctk.CTkFont(size=11, weight="normal"),
            text_color=VAA_TEXT,
        ).pack(side="left")

        # Right section - Navigation tabs
        right_frame = ctk.CTkFrame(self, fg_color="transparent")
        right_frame.pack(side="right", padx=20, pady=10)

        self._btn_auto = self._make_tab(right_frame, "AUTO MODE", "PageAuto")
        self._btn_auto.pack(side="left", padx=(0, 8))
        self._btn_manual = self._make_tab(right_frame, "MANUAL MODE", "PageManual")
        self._btn_manual.pack(side="left")

        # Apply initial highlight
        self.update_active_tab(current_page)

    def _make_tab(
        self, parent: ctk.CTkFrame, label: str, page_name: str
    ) -> ctk.CTkButton:
        """Create one navigation tab button."""
        return ctk.CTkButton(
            parent,
            text=label,
            width=126,
            height=38,
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=VAA_SECONDARY,
            hover_color=VAA_SECONDARY,
            text_color=VAA_TEXT,
            corner_radius=9,
            border_width=0,
            command=lambda: self.controller.show_frame(page_name),
        )

    def update_active_tab(self, page_name: str) -> None:
        """Update tab highlighting based on current page (active tab glows cyan)."""
        active = {
            "fg_color": VAA_ACCENT,
            "hover_color": VAA_ACCENT,
            "text_color": VAA_PRIMARY,
        }
        inactive = {
            "fg_color": VAA_SECONDARY,
            "hover_color": "#274B75",
            "text_color": VAA_TEXT,
        }
        if page_name == "PageAuto":
            self._btn_auto.configure(**active)
            self._btn_manual.configure(**inactive)
        else:
            self._btn_manual.configure(**active)
            self._btn_auto.configure(**inactive)


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

    @staticmethod
    def _chip(parent: ctk.CTkFrame, text: str) -> ctk.CTkLabel:
        """Create one status chip label."""
        return ctk.CTkLabel(
            parent,
            text=text,
            font=ctk.CTkFont(size=11, weight="bold"),
            text_color=DANGER,
        )

    def _build_layout(self) -> None:
        # Use pack manager since the parent container uses pack (not grid)
        # Left - Status
        left_frame = ctk.CTkFrame(self, fg_color="transparent")
        left_frame.pack(side="left", padx=20, fill="both", expand=True)

        self.lbl_status = self._chip(left_frame, "● SYSTEM READY")
        self.lbl_status.configure(text_color=SUCCESS)
        self.lbl_status.pack(side="left", pady=5)

        self.lbl_plc = self._chip(left_frame, "PLC: DISCONNECTED")
        self.lbl_plc.pack(side="left", padx=20, pady=5)

        self.lbl_camera = self._chip(left_frame, "CAM: OFFLINE")
        self.lbl_camera.pack(side="left", padx=20, pady=5)

        # Right - Version
        right_frame = ctk.CTkFrame(self, fg_color="transparent")
        right_frame.pack(side="right", padx=20)

        ctk.CTkLabel(
            right_frame,
            text="v2.0.0 | VAA Robot System",
            font=ctk.CTkFont(size=10, weight="normal"),
            text_color=TEXT_SECONDARY,
        ).pack(side="right", pady=5)

    def update_status(
        self, plc_connected: bool, camera_active: bool, system_ok: bool
    ) -> None:
        """Update footer status indicators."""
        if system_ok:
            self.lbl_status.configure(text="● SYSTEM READY", text_color=SUCCESS)
        else:
            self.lbl_status.configure(text="● SYSTEM ERROR", text_color=DANGER)

        self.lbl_plc.configure(
            text="PLC: CONNECTED" if plc_connected else "PLC: DISCONNECTED",
            text_color=SUCCESS if plc_connected else DANGER,
        )

        self.lbl_camera.configure(
            text="CAM: ACTIVE" if camera_active else "CAM: OFFLINE",
            text_color=SUCCESS if camera_active else DANGER,
        )
