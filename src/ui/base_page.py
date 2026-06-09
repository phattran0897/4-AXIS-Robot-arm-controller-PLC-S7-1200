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
        page_color: str = "#1F6AA5",
    ) -> None:
        super().__init__(parent)
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
        frame = ctk.CTkFrame(parent)
        frame.grid(row=0, column=column, padx=10, pady=10, sticky="nsew")
        self.video_label = self.build_camera_selector(frame, title_color)

    def build_camera_selector(
        self,
        parent_frame: ctk.CTkFrame,
        title_color: str,
    ) -> ctk.CTkLabel:
        """
        Create the camera-source combo-box row and attach the video label.

        Returns the :class:`ctk.CTkLabel` used to display live video.
        """
        top = ctk.CTkFrame(parent_frame, fg_color="transparent")
        top.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(
            top,
            text="Camera Source:",
            font=ctk.CTkFont(size=13, weight="bold"),
            text_color=title_color,
        ).pack(side="left", padx=5)

        cam_selector = ctk.CTkComboBox(
            top,
            values=["Camera 0 (Default)", "Camera 1 (External)", "Camera 2 (Aux)"],
            command=self.controller.change_camera_source,
            width=200,
        )
        cam_selector.pack(side="right", padx=5)
        cam_selector.set("Camera 0 (Default)")

        video_label = ctk.CTkLabel(
            parent_frame,
            text="Scanning via YOLOv11…",
            fg_color="black",
            width=440,
            height=310,
        )
        video_label.pack(padx=10, pady=5, expand=True, fill="both")

        return video_label

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
        frame_bottom = ctk.CTkFrame(self)
        frame_bottom.pack(fill="x", side="bottom", padx=30, pady=20)

        lbl_err = ctk.CTkLabel(
            frame_bottom,
            text="SYSTEM STABLE",
            font=ctk.CTkFont(weight="bold"),
            text_color="green",
        )
        lbl_err.pack(side="left", padx=20)

        ctk.CTkButton(
            frame_bottom,
            text="Clear Error",
            fg_color="red",
            hover_color="#990000",
            width=100,
            command=lambda: self.controller.plc.send_command(
                self.controller.cfg.plc.commands.idle
            ),
        ).pack(side="left", padx=10)

        ctk.CTkButton(
            frame_bottom,
            text=navigate_text,
            fg_color=navigate_color,
            width=220,
            command=lambda: self.controller.show_frame(navigate_target),
        ).pack(side="right", padx=20)

        return lbl_err

    # ------------------------------------------------------------------
    # Shared status update (called by sub-classes)
    # ------------------------------------------------------------------

    def _refresh_error_status(self, data: dict[str, Any]) -> None:
        """Update the error status label shared by every page."""
        if self.lbl_err_status is None:
            return
        if data.get("error_flag", False):
            self.lbl_err_status.configure(
                text="WARNING: SYSTEM ERROR!", text_color="red"
            )
        else:
            self.lbl_err_status.configure(text="SYSTEM STABLE", text_color="green")
