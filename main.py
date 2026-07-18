"""
aamain.py – Application entry point for the 4-Axis Industrial Robot Control System.

Responsibilities
----------------
* Bootstrap configuration, logging, PLC controller, and YOLO detector.
* Manage the CustomTkinter page-routing container.
* Own the two background threads (PLC cyclic poll + AI/camera loop).
* Provide a clean, deadlock-free shutdown via :class:`threading.Event`.
"""

from __future__ import annotations

import logging
import threading
import time
from logging.handlers import RotatingFileHandler
from typing import Any

import customtkinter as ctk

from src.ai.yolo_detector import YOLODetector
from src.config_loader import RobotConfig, load_config
from src.kinematics import inverse_kinematics, InverseKinematicsError
from src.plc.plc_controller import PLCController
from src.robot.sorting_controller import SortingController, SortResult
from src.ui.page_auto import PageAuto
from src.ui.page_manual import PageManual
from src.ui.header import VAAHeader, VAAFooter
from src.ui.theme import ACCENT, CARD_BG, PANEL_BORDER

from PIL import Image

# ---------------------------------------------------------------------------
# Logging (console + rotating file)
# ---------------------------------------------------------------------------
_robot_log = logging.getLogger("RobotApp")
_robot_log.setLevel(logging.INFO)

_console = logging.StreamHandler()
_console.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s – %(message)s"))
_robot_log.addHandler(_console)

_log_file = RotatingFileHandler(
    "robot_app.log",
    maxBytes=5_000_000,   # 5 MB per file
    backupCount=3,
    encoding="utf-8",
)
_log_file.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s – %(message)s"))
_robot_log.addHandler(_log_file)

log = _robot_log

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_THREAD_JOIN_TIMEOUT: float = 5.0
_CAMERA_SWITCH_DEBOUNCE: float = 0.5   # seconds


class RobotApp(ctk.CTk):
    """
    Main application window.

    Owns all hardware controllers, background threads, and GUI page routing.
    """

    def __init__(self, cfg: RobotConfig) -> None:
        super().__init__()
        self.cfg: RobotConfig = cfg

        # ── Window setup ────────────────────────────────────────────────────
        ctk.set_appearance_mode(cfg.app.appearance_mode)
        ctk.set_default_color_theme(cfg.app.color_theme)
        self.title(cfg.app.title)
        self.geometry(cfg.app.geometry)

        # ── Hardware controllers ─────────────────────────────────────────────
        self.plc: PLCController = PLCController(cfg.plc)
        self.detector: YOLODetector = YOLODetector(
            model_path=cfg.yolo.model_path,
            thresh=cfg.yolo.thresh,
            px2mm=cfg.yolo.px2mm,
            home_x=cfg.yolo.home_x,
            home_y=cfg.yolo.home_y,
            inference_width=cfg.camera.inference_width,
            inference_height=cfg.camera.inference_height,
            camera_read_timeout=cfg.camera.read_timeout,
        )
        self._sorter: SortingController = SortingController(
            plc=self.plc,
            positions=cfg.sort_positions,
            kinematics=cfg.kinematics,
            plc_commands=cfg.plc.commands,
        )
        self._sort_thread: threading.Thread | None = None

        # ── Thread synchronisation ───────────────────────────────────────────
        self._stop_event: threading.Event = threading.Event()

        # ── Camera switch debounce ───────────────────────────────────────────
        self._camera_switch_timer: threading.Timer | None = None

        # ── Object Lock tracking (AI vision loop) ────────────────────────────
        self._lock_start_time: float | None = None  # monotonic time lock started
        self._lock_duration: float = 2.5            # seconds to hold lock
        self._lock_tolerance: float = 15.0          # max position drift (mm)
        self._lock_target_x: float = 0.0            # locked X position (mm)
        self._lock_target_y: float = 0.0            # locked Y position (mm)
        self._lock_class_id: int = 0                # locked class ID
        self._is_locked: bool = False               # lock confirmed flag

        # ── GUI container + page routing ─────────────────────────────────────
        self._container: ctk.CTkFrame = ctk.CTkFrame(self)
        self._container.pack(side="top", fill="both", expand=True, padx=0, pady=0)
        self._container.grid_rowconfigure(0, weight=0)  # Header
        self._container.grid_rowconfigure(1, weight=1)  # Content
        self._container.grid_columnconfigure(0, weight=1)

        # Header
        self.header: VAAHeader = VAAHeader(
            parent=self._container,
            controller=self,
        )
        self.header.pack(fill="x")

        # Content area (pages)
        self._content_area: ctk.CTkFrame = ctk.CTkFrame(self._container, fg_color="#0F172A")
        self._content_area.pack(fill="both", expand=True)
        self._content_area.grid_rowconfigure(0, weight=1)
        self._content_area.grid_columnconfigure(0, weight=1)

        # ── PLC Tx Console ──────────────────────────────────────────────────
        self.tx_console_frame = ctk.CTkFrame(
            self._container,
            fg_color=CARD_BG,
            border_color=PANEL_BORDER,
            border_width=1,
            corner_radius=12,
        )
        self.tx_console_frame.pack(fill="x", padx=20, pady=(0, 10))

        # Title / header line
        console_header = ctk.CTkFrame(self.tx_console_frame, fg_color="transparent")
        console_header.pack(fill="x", padx=12, pady=(8, 4))
        
        ctk.CTkLabel(
            console_header,
            text="💻  PLC TRANSMISSION TELEMETRY",
            font=ctk.CTkFont(size=11, weight="bold"),
            text_color=ACCENT,
        ).pack(side="left")

        # Textbox for scrolling log
        self.tx_textbox = ctk.CTkTextbox(
            self.tx_console_frame,
            height=85,
            font=ctk.CTkFont(family="Consolas", size=11),
            fg_color="#020617",
            text_color="#38BDF8",
            border_width=0,
            corner_radius=6,
        )
        self.tx_textbox.pack(fill="x", padx=12, pady=(0, 10))
        self.tx_textbox.configure(state="disabled")

        # Wire up callback to PLCController
        self.plc.tx_callback = self.log_tx

        # Footer
        self.footer: VAAFooter = VAAFooter(parent=self._container)
        self.footer.pack(fill="x", side="bottom")

        self._current_page: str = "PageAuto"
        self.frames: dict[str, PageAuto | PageManual] = {}
        for PageClass in (PageAuto, PageManual):
            name = PageClass.__name__
            frame = PageClass(parent=self._content_area, controller=self)
            self.frames[name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("PageAuto")

        # ── Background threads ───────────────────────────────────────────────
        self._plc_thread: threading.Thread = threading.Thread(
            target=self._cyclic_update,
            name="PLCPollThread",
            daemon=True,
        )
        self._ai_thread: threading.Thread = threading.Thread(
            target=self._yolo_processing_loop,
            name="AIVisionThread",
            daemon=True,
        )
        self._plc_thread.start()
        self._ai_thread.start()

        log.info("RobotApp initialised – all threads started.")

    # ------------------------------------------------------------------
    # Page routing
    # ------------------------------------------------------------------

    def show_frame(self, page_name: str) -> None:
        """Raise the named page to the top of the stacking order."""
        if page_name not in self.frames:
            log.warning("Unknown page requested: %s", page_name)
            return
        self._current_page = page_name
        self.frames[page_name].tkraise()
        self.header.update_active_tab(page_name)
        log.debug("Switched to page: %s", page_name)
        if page_name == "PageAuto":
            self.plc.send_pulse(0, 2)   # AUTO_MODE (offset 0.2)
        elif page_name == "PageManual":
            self.plc.send_pulse(16, 0)  # MANUAL_MODE (offset 16.0)

    # ------------------------------------------------------------------
    # Camera switching (debounced)
    # ------------------------------------------------------------------

    def change_camera_source(self, choice_str: str) -> None:
        """
        Parse the combo-box selection string and debounce-switch the camera.

        Expected format: ``"Camera <index> (<label>)"``
        e.g. ``"Camera 1 (External)"``
        """
        if self._camera_switch_timer is not None:
            self._camera_switch_timer.cancel()

        self._camera_switch_timer = threading.Timer(
            _CAMERA_SWITCH_DEBOUNCE,
            self._do_change_camera,
            args=(choice_str,),
        )
        self._camera_switch_timer.start()

    def _do_change_camera(self, choice_str: str) -> None:
        """Actually perform the camera switch (called after debounce)."""
        try:
            import re
            match = re.search(r'\d+', choice_str)
            if not match:
                raise ValueError("No digits found in string")
            idx: int = int(match.group())
        except (ValueError, AttributeError) as exc:
            log.error("Cannot parse camera index from '%s': %s", choice_str, exc)
            return

        success: bool = self.detector.switch_camera(idx)

        if success:
            log.info("Camera switched to index %d.", idx)
        else:
            from tkinter import messagebox
            messagebox.showerror(
                "Hardware Error",
                f"Cannot open camera at index {idx}.\n"
                "Check the device connection and try again.",
            )

    # ------------------------------------------------------------------
    # Background thread: AI / camera loop
    # ------------------------------------------------------------------

    def _yolo_processing_loop(self) -> None:
        """
        Continuously capture frames, run YOLO inference, push detections
        to the PLC, and refresh the GUI video label.

        Object Locking: when a defect is detected, the system enters a
        locking phase (~2.5 seconds). During this phase the object's
        position must remain stable (within tolerance). Only after the
        lock is confirmed is the sorting command sent to the PLC.

        Runs until ``_stop_event`` is set.
        """
        import math as _math

        cam_cfg = self.cfg.camera
        yolo_cfg = self.cfg.yolo

        try:
            if not self.detector.start_camera(cam_cfg.default_index):
                log.error(
                    "Cannot open default camera (index %d).", cam_cfg.default_index
                )

            frame_interval: float = 1.0 / max(cam_cfg.fps, 1)
            last_detection: tuple[bool, float, float, int] = (False, 0.0, 0.0, 0)

            while not self._stop_event.is_set():
                # ── Step 1: read frame from thread-safe queue ────────────────────
                frame = self.detector.read_frame()

                # ── Step 2: inference unlocked ───────────────────────────────────
                if frame is not None:
                    try:
                        result = self.detector.annotate_frame(frame)
                        last_detection = (
                            result.has_defect,
                            result.robot_x,
                            result.robot_y,
                            result.class_id,
                        )

                        # ── Step 3: Object Locking logic ────────────────────────────
                        has_defect, rx, ry, class_id = last_detection
                        annotated = result.annotated_frame

                        if has_defect and self.plc.is_connected() and self._sorter.is_idle():
                            now = time.monotonic()

                            if self._lock_start_time is None:
                                # ── First detection: start locking ──────────────────
                                self._lock_start_time = now
                                self._lock_target_x = rx
                                self._lock_target_y = ry
                                self._lock_class_id = class_id
                                self._is_locked = False
                                log.info(
                                    "Object detected – starting lock (X=%.2f Y=%.2f class=%d)",
                                    rx, ry, class_id,
                                )
                            else:
                                # ── Ongoing lock: check position stability ──────────
                                drift = _math.hypot(
                                    rx - self._lock_target_x,
                                    ry - self._lock_target_y,
                                )

                                if drift > self._lock_tolerance:
                                    # Object moved too much – restart lock
                                    log.info(
                                        "Lock reset – object drifted %.1fmm (tolerance=%.1fmm)",
                                        drift, self._lock_tolerance,
                                    )
                                    self._lock_start_time = now
                                    self._lock_target_x = rx
                                    self._lock_target_y = ry
                                    self._lock_class_id = class_id
                                    self._is_locked = False
                                else:
                                    # Position stable – check if lock duration met
                                    elapsed = now - self._lock_start_time

                                    if elapsed >= self._lock_duration and not self._is_locked:
                                        # ── Lock confirmed → dispatch to PLC ────────
                                        self._is_locked = True
                                        # Model class mapping: 0 = Defect (BAD), 1 = Good (GOOD)
                                        sort_result = (
                                            SortResult.BAD
                                            if self._lock_class_id == 0
                                            else SortResult.GOOD
                                        )
                                        log.info(
                                            "Object LOCKED (%.1fs) → Spawning sort "
                                            "(Result=%s) X=%.2f Y=%.2f",
                                            elapsed,
                                            sort_result.name,
                                            self._lock_target_x,
                                            self._lock_target_y,
                                        )
                                        self._sort_thread = threading.Thread(
                                            target=self._run_sort_and_reset_lock,
                                            args=(
                                                self._lock_target_x,
                                                self._lock_target_y,
                                                sort_result,
                                            ),
                                            daemon=True,
                                        )
                                        self._sort_thread.start()

                            # ── Draw lock overlay on the frame ──────────────────────
                            if annotated is not None and self._lock_start_time is not None:
                                elapsed = now - self._lock_start_time
                                progress = min(elapsed / self._lock_duration, 1.0)
                                self.detector.draw_lock_overlay(
                                    annotated, progress, self._is_locked
                                )

                        else:
                            # No defect or sorter busy – reset lock state
                            if self._lock_start_time is not None and not self._is_locked:
                                log.info("Lock cancelled – object lost or sorter busy.")
                            if not self._is_locked:
                                self._lock_start_time = None

                        # ── Step 4: Update GUI video label ──────────────────────────
                        pil = result.to_pil(cam_cfg.display_width, cam_cfg.display_height)
                        if pil is not None:
                            self._update_video_label(pil)

                    except Exception as frame_exc:
                        log.error("Exception during frame processing: %s", frame_exc, exc_info=True)

                self._stop_event.wait(frame_interval)
        except Exception as thread_exc:
            log.critical("AIVisionThread crashed with exception: %s", thread_exc, exc_info=True)

        log.info("AI vision thread exited cleanly.")

    def _run_sort_and_reset_lock(
        self, rx: float, ry: float, sort_result: SortResult
    ) -> None:
        """Execute sorting cycle and reset lock state when finished."""
        try:
            self._sorter.execute_sort(rx, ry, sort_result)
        except Exception as exc:
            log.error("Sorting cycle failed: %s", exc)
        finally:
            # Reset lock state so next detection can start fresh
            self._lock_start_time = None
            self._is_locked = False

    def _dispatch_target_to_plc(self, rx: float, ry: float) -> None:
        """Compute IK and send joint targets + move command to the PLC."""
        try:
            j1, j2, j3, j4 = inverse_kinematics(rx, ry)
            self.plc.send_joint_targets(j1, j2, j3, j4)
            self.plc.send_command(self.cfg.plc.commands.move)
        except InverseKinematicsError:
            log.error("IK failed: unreachable target X=%.2f Y=%.2f", rx, ry)
        except Exception as exc:
            log.error("Unexpected error during defect response: %s", exc)

    def _update_video_label(self, img: Image.Image) -> None:
        """Push a new frame to whichever page is currently visible (thread-safe)."""
        self.after(0, self._do_update_video, img)

    def _do_update_video(self, img: Image.Image) -> None:
        """Internal method to update video label on main thread."""
        page = self.frames.get(self._current_page)
        if page is None:
            return
        try:
            cam_cfg = self.cfg.camera
            ctk_img = ctk.CTkImage(
                light_image=img,
                dark_image=img,
                size=(cam_cfg.display_width, cam_cfg.display_height),
            )
            page.update_video(ctk_img)
        except Exception as exc:
            log.debug("Video label update skipped: %s", exc)

    def log_tx(self, message: str) -> None:
        """Append a timestamped message to the transmission telemetry log (thread-safe)."""
        def _append():
            timestamp = time.strftime("%H:%M:%S")
            self.tx_textbox.configure(state="normal")
            self.tx_textbox.insert("end", f"[{timestamp}] {message}\n")
            self.tx_textbox.see("end")
            content = self.tx_textbox.get("1.0", "end")
            lines = content.splitlines()
            if len(lines) > 100:
                self.tx_textbox.delete("1.0", f"{len(lines) - 100}.0")
            self.tx_textbox.configure(state="disabled")
        self.after(0, _append)

    # ------------------------------------------------------------------
    # Background thread: PLC cyclic poll with exponential backoff
    # ------------------------------------------------------------------

    def _cyclic_update(self) -> None:
        """
        Poll the PLC at the configured interval and push fresh data to
        all page ``update_gui_data()`` callbacks via ``self.after()``.

        Falls back to reconnection with exponential backoff when the PLC is offline.
        Runs until ``_stop_event`` is set.
        """
        interval: float = self.cfg.app.plc_poll_interval
        retry_delay: float = interval
        max_retry_delay: float = 30.0
        consecutive_failures: int = 0

        while not self._stop_event.is_set():
            if self.plc.is_connected():
                data: dict[str, Any] = self.plc.read_status()
                if data:
                    self.after(0, self._dispatch_plc_data, data)
                retry_delay = interval
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures <= 3 or consecutive_failures % 10 == 0:
                    log.warning(
                        "PLC offline – attempting reconnection (attempt %d, backoff %.1fs)…",
                        consecutive_failures, retry_delay
                    )
                try:
                    self.plc.connect()
                except Exception as exc:
                    log.error("PLC connection failed: %s", exc)
                retry_delay = min(retry_delay * 2, max_retry_delay)

            self._stop_event.wait(retry_delay)

        log.info("PLC poll thread exited cleanly.")

    def _dispatch_plc_data(self, data: dict[str, Any]) -> None:
        """Forward PLC data to every registered page (runs on main thread)."""
        for frame in self.frames.values():
            try:
                frame.update_gui_data(data)
            except Exception as exc:
                log.debug("update_gui_data error on %s: %s", type(frame).__name__, exc)

        # Update footer status
        plc_connected = self.plc.is_connected()
        camera_active = self.detector.is_camera_active() if hasattr(self.detector, 'is_camera_active') else False
        system_ok = not data.get("error_flag", False)
        self.footer.update_status(plc_connected, camera_active, system_ok)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def on_closing(self) -> None:
        """
        Graceful shutdown sequence:

        1. Signal all background threads to stop via ``_stop_event``.
        2. Release hardware resources (camera, PLC socket).
        3. Join threads with a timeout to prevent hangs.
        4. Destroy the Tk window.
        """
        log.info("Shutdown initiated…")
        self._stop_event.set()

        if self._camera_switch_timer is not None:
            self._camera_switch_timer.cancel()

        self.detector.stop()
        self.plc.disconnect()

        # Wait for active sorting thread to complete gracefully
        if self._sort_thread is not None and self._sort_thread.is_alive():
            log.info("Waiting for active sorting thread to complete...")
            self._sort_thread.join(timeout=5.0)

        for thread in (self._plc_thread, self._ai_thread):
            if thread.is_alive():
                thread.join(timeout=_THREAD_JOIN_TIMEOUT)
                if thread.is_alive():
                    log.warning("Thread '%s' did not exit within timeout.", thread.name)

        log.info("All threads stopped. Destroying window.")
        self.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    config: RobotConfig = load_config()
    app = RobotApp(cfg=config)
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
