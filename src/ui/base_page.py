"""
src/ui/base_page.py – Abstract base class for all application pages.

Every page (Auto / Manual) inherits :class:`BasePage`, which wires up the
controller reference and exposes shared helpers: status-bar updates,
camera selector construction, and the ``update_gui_data`` contract.
"""

from __future__ import annotations

import logging
from abc import abstractmethod
from typing import TYPE_CHECKING, Any

import customtkinter as ctk
from PIL import ImageTk

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
    HEADER_BG,
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
        super().__init__(parent, fg_color="#0F172A")
        self.controller: "RobotApp" = controller
        self.page_color: str = page_color

        # Shared widgets populated by sub-class build helpers
        self.video_label: ctk.CTkLabel | None = None
        self.lbl_err_status: ctk.CTkLabel | None = None

        # Proper reference to prevent PhotoImage garbage collection
        self._current_tk_image: ImageTk.PhotoImage | None = None

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

    def update_video(self, img_tk: ImageTk.PhotoImage) -> None:
        """
        Replace the current video frame with *img_tk*.

        Stores the reference in ``_current_tk_image`` to prevent garbage
        collection. Thread-safe to call from any thread.
        """
        self._current_tk_image = img_tk
        if self.video_label is not None:
            self.video_label.configure(image=img_tk)

    # ------------------------------------------------------------------
    # Shared UI factory helpers
    # ------------------------------------------------------------------

    def build_camera_column(
        self,
        parent: ctk.CTkFrame,
        column: int,
        title_color: str,
    ) -> None:
        """
        Build the camera column (frame + selector + video label) in one call.

        Eliminates the duplication that existed when each sub-class implemented
        this layout manually.

        Parameters
        ----------
        parent:
            The parent grid container.
        column:
            Grid column index to place the camera column.
        title_color:
            Hex colour string for the selector label.
        """
        frame = self._create_card(parent)
        frame.grid(row=0, column=column, padx=8, pady=8, sticky="nsew")

        # Section header
        header = ctk.CTkFrame(frame, fg_color="transparent")
        header.pack(fill="x", padx=15, pady=(12, 8))

        icon_label = ctk.CTkLabel(
            header,
            text="📷",
            font=ctk.CTkFont(size=18),
            text_color=title_color,
        )
        icon_label.pack(side="left", padx=(0, 8))

        ctk.CTkLabel(
            header,
            text="AI VISION",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color=title_color,
        ).pack(side="left")

        # Camera selector
        selector_frame = ctk.CTkFrame(frame, fg_color="transparent")
        selector_frame.pack(fill="x", padx=15, pady=(0, 8))

        ctk.CTkLabel(
            selector_frame,
            text="Source:",
            font=ctk.CTkFont(size=11),
            text_color=TEXT_SECONDARY,
        ).pack(side="left")

        cam_selector = ctk.CTkComboBox(
            selector_frame,
            values=["Camera 0", "Camera 1", "Camera 2"],
            command=self.controller.change_camera_source,
            width=140,
            fg_color=PANEL_BG,
            text_color=TEXT_PRIMARY,
            button_color=title_color,
            button_hover_color=title_color,
            dropdown_fg_color=PANEL_BG,
            dropdown_text_color=TEXT_PRIMARY,
        )
        cam_selector.pack(side="right")
        cam_selector.set("Camera 0")

        # Video label with modern styling
        video_container = ctk.CTkFrame(
            frame,
            fg_color="black",
            corner_radius=8,
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

    def build_status_bar(
        self,
        navigate_text: str,
        navigate_target: str,
        navigate_color: str,
    ) -> ctk.CTkLabel:
        """
        Build the bottom status bar common to all pages.

        Returns the error-status :class:`ctk.CTkLabel`.
        """
        frame_bottom = ctk.CTkFrame(
            self,
            fg_color=HEADER_BG,
            height=60,
            corner_radius=0,
        )
        frame_bottom.pack(fill="x", side="bottom", padx=20, pady=0)
        frame_bottom.pack_propagate(False)

        # Left: Status
        status_frame = ctk.CTkFrame(frame_bottom, fg_color="transparent")
        status_frame.pack(side="left", padx=10, fill="both", expand=True)

        self.lbl_err_status = ctk.CTkLabel(
            status_frame,
            text="● SYSTEM STABLE",
            font=ctk.CTkFont(size=12, weight="bold"),
            text_color=SUCCESS,
        )
        self.lbl_err_status.pack(side="left", pady=15)

        # Center: Clear Error button
        ctk.CTkButton(
            status_frame,
            text="⚠  Clear Error",
            font=ctk.CTkFont(size=11, weight="bold"),
            fg_color=DANGER,
            hover_color="#DC2626",
            text_color="white",
            width=110,
            height=32,
            corner_radius=6,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.idle
            ),
        ).pack(side="left", padx=20, pady=10)

        # Right: Navigate button
        ctk.CTkButton(
            frame_bottom,
            text=navigate_text,
            font=ctk.CTkFont(size=12, weight="bold"),
            fg_color=navigate_color,
            hover_color="#4338CA",
            text_color="white",
            width=160,
            height=36,
            corner_radius=8,
            command=lambda: self.controller.show_frame(navigate_target),
        ).pack(side="right", padx=15, pady=12)

        return self.lbl_err_status

    # ------------------------------------------------------------------
    # Shared status update (called by sub-classes)
    # ------------------------------------------------------------------

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
