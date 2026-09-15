"""
src/ui/base_page.py – Abstract base class for all application pages.

Every page (Auto / Manual) inherits :class:`BasePage`, which wires up the
controller reference and exposes shared helpers: card + section-header
factories, status-bar updates, camera-selector construction, and the
``update_gui_data`` contract.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from typing import TYPE_CHECKING, Any

import customtkinter as ctk

from src.ui.theme import (
    CARD_BG,
    CONTENT_BG,
    DANGER,
    HEADER_BG,
    PANEL_BG,
    PANEL_BORDER,
    ROW_BG,
    SUCCESS,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
)

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


class BasePage(ctk.CTkFrame):
    """
    Shared base frame for every page in the application.

    Sub-classes must implement :meth:`update_gui_data`.

    Parameters
    ----------
    parent:
        The container frame managed by :class:`RobotApp`.
    controller:
        The main application instance, used to access ``plc``, ``detector``,
        ``show_frame()``, and ``change_camera_source()``.
    page_color:
        Accent colour for page-specific title labels (hex string).
    """

    def __init__(
        self,
        parent: ctk.CTkFrame,
        controller: "RobotApp",
        page_color: str = "#00D4FF",
    ) -> None:
        super().__init__(parent, fg_color=CONTENT_BG)
        self.controller: "RobotApp" = controller
        self.page_color: str = page_color

        # Shared widgets populated by sub-class build helpers
        self.video_label: ctk.CTkLabel | None = None
        self.lbl_err_status: ctk.CTkLabel | None = None

        # Proper reference to prevent PhotoImage garbage collection
        self._current_tk_image: ctk.CTkImage | None = None

        # Dynamic video display size (updated on container resize)
        self._video_display_width: int = 440
        self._video_display_height: int = 310

    # ------------------------------------------------------------------
    # Contract
    # ------------------------------------------------------------------

    @abstractmethod
    def update_gui_data(self, data: dict[str, Any]) -> None:
        """
        Refresh all dynamic widgets with *data* from the PLC cyclic read.

        Called from the background ``cyclic_update`` thread via
        ``self.after()`` – implementations must be thread-safe
        (CustomTkinter is *not* thread-safe; use ``self.after()`` if
        needed).
        """

    # ------------------------------------------------------------------
    # Video label (thread-safe update)
    # ------------------------------------------------------------------

    def update_video(self, img: ctk.CTkImage) -> None:
        """
        Replace the current video frame with *img*.

        Stores the reference in ``_current_tk_image`` to prevent garbage
        collection. Thread-safe to call from any thread.
        """
        self._current_tk_image = img
        if self.video_label is not None:
            self.video_label.configure(text="", image=img)

    # ------------------------------------------------------------------
    # Shared UI factory helpers
    # ------------------------------------------------------------------

    def _create_card(self, parent: ctk.CTkFrame) -> ctk.CTkFrame:
        """Create a modern bordered card (single implementation app-wide)."""
        return ctk.CTkFrame(
            parent,
            fg_color=CARD_BG,
            border_color=PANEL_BORDER,
            border_width=1,
            corner_radius=14,
        )

    def build_section_header(
        self,
        parent: ctk.CTkFrame,
        icon: str,
        title: str,
        color: str,
    ) -> ctk.CTkFrame:
        """
        Build the standard section header row (icon + title + underline)
        and return the header frame.
        """
        header = ctk.CTkFrame(parent, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(14, 8))

        row = ctk.CTkFrame(header, fg_color="transparent")
        row.pack(fill="x")

        ctk.CTkLabel(
            row,
            text=icon,
            font=ctk.CTkFont(size=18),
            text_color=color,
        ).pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            row,
            text=title,
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=color,
        ).pack(side="left")

        # Accent underline for a crisp, professional finish
        ctk.CTkFrame(header, height=2, fg_color=color, corner_radius=1).pack(
            fill="x", pady=(6, 0)
        )
        return header

    def create_data_row(
        self,
        parent: ctk.CTkFrame,
        name: str,
        value: str,
        value_color: str | None = None,
        mono_font: bool = True,
    ) -> ctk.CTkLabel:
        """
        Create a styled label/value row and return the value label.

        Used for joint readouts, kinematics results, and similar key-value
        displays so they look identical across pages.
        """
        row = ctk.CTkFrame(
            parent,
            fg_color=ROW_BG,
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        row.pack(fill="x", pady=4, ipady=5)

        ctk.CTkLabel(
            row,
            text=name,
            font=ctk.CTkFont(size=11, weight="bold"),
            text_color=TEXT_SECONDARY,
            anchor="w",
        ).pack(side="left", padx=12)

        val_label = ctk.CTkLabel(
            row,
            text=value,
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=value_color or TEXT_PRIMARY,
            anchor="e",
        )
        val_label.pack(side="right", padx=12)
        return val_label

    def build_camera_column(
        self,
        parent: ctk.CTkFrame,
        column: int,
        title_color: str,
    ) -> None:
        """
        Build the camera column (frame + selector + video label) in one call.

        Eliminates duplication that existed when each sub-class implemented
        this layout manually.
        """
        frame = self._create_card(parent)
        frame.grid(row=0, column=column, padx=8, pady=8, sticky="nsew")

        # Section header
        self.build_section_header(frame, "📷", "AI VISION", title_color)

        # Camera selector
        selector_frame = ctk.CTkFrame(frame, fg_color="transparent")
        selector_frame.pack(fill="x", padx=15, pady=(0, 8))

        ctk.CTkLabel(
            selector_frame,
            text="Source:",
            font=ctk.CTkFont(size=11),
            text_color=TEXT_SECONDARY,
        ).pack(side="left")

        # Populate camera options dynamically
        available_cameras = self.controller.detector.available_cameras(max_index=2)
        if available_cameras:
            cam_values = [f"Camera {i}" for i in available_cameras]
        else:
            cam_values = ["No Camera"]

        cam_selector = ctk.CTkComboBox(
            selector_frame,
            values=cam_values,
            command=self.controller.change_camera_source,
            width=140,
            height=28,
            fg_color=PANEL_BG,
            text_color=TEXT_PRIMARY,
            button_color=title_color,
            button_hover_color=title_color,
            dropdown_fg_color=PANEL_BG,
            dropdown_text_color=TEXT_PRIMARY,
            border_color=PANEL_BORDER,
            corner_radius=8,
        )
        cam_selector.pack(side="right")
        if cam_values and cam_values[0] != "No Camera":
            cam_selector.set(cam_values[0])
        else:
            cam_selector.set("No Camera")

        # Video label with modern styling
        video_container = ctk.CTkFrame(
            frame,
            fg_color="black",
            corner_radius=10,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        video_container.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        self.video_label = ctk.CTkLabel(
            video_container,
            text="🎥  Initializing AI Vision...",
            fg_color="black",
            text_color=TEXT_SECONDARY,
            font=ctk.CTkFont(size=12),
        )
        self.video_label.pack(fill="both", expand=True, padx=5, pady=5)

        # Track container size for dynamic camera scaling on fullscreen
        video_container.bind("<Configure>", self._on_video_container_resize)

    def build_status_bar(self) -> ctk.CTkLabel:
        """
        Build the bottom status bar common to all pages.

        Returns the error-status :class:`ctk.CTkLabel`.
        """
        frame_bottom = ctk.CTkFrame(
            self,
            fg_color=HEADER_BG,
            height=56,
            corner_radius=12,
            border_color=PANEL_BORDER,
            border_width=1,
        )
        frame_bottom.pack(fill="x", side="bottom", padx=20, pady=(0, 15))
        frame_bottom.pack_propagate(False)

        # Left: Status
        status_frame = ctk.CTkFrame(frame_bottom, fg_color="transparent")
        status_frame.pack(side="left", padx=15, fill="both", expand=True)

        self.lbl_err_status = ctk.CTkLabel(
            status_frame,
            text="● SYSTEM STABLE",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=SUCCESS,
        )
        self.lbl_err_status.pack(side="left", pady=14)

        # Clear Error button
        ctk.CTkButton(
            status_frame,
            text="⚠  Clear Error",
            font=ctk.CTkFont(size=11, weight="bold"),
            fg_color="transparent",
            border_color=DANGER,
            border_width=1,
            hover_color=DANGER,
            text_color=DANGER,
            width=110,
            height=32,
            corner_radius=8,
            command=self.controller.clear_all_errors,
        ).pack(side="left", padx=20, pady=11)

        return self.lbl_err_status

    # ------------------------------------------------------------------
    # Shared status update (called by sub-classes)
    # ------------------------------------------------------------------

    def _on_video_container_resize(self, event) -> None:
        """Track video container size so the camera feed scales on fullscreen."""
        # Ignore spurious tiny sizes during initial layout
        if event.width > 50 and event.height > 50:
            self._video_display_width = event.width - 10  # padding margin
            self._video_display_height = event.height - 10

    def _refresh_error_status(self, data: dict[str, Any]) -> None:
        """Update the error status label shared by every page."""
        if self.lbl_err_status is None:
            return
        if data.get("error_flag", False):
            self.lbl_err_status.configure(
                text="⚠  WARNING: SYSTEM ERROR!",
                text_color=DANGER,
            )
        else:
            self.lbl_err_status.configure(
                text="● SYSTEM STABLE",
                text_color=SUCCESS,
            )
