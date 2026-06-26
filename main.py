"""
main.py – Application entry point for the 4-Axis SCARA Robot Control System.

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
from PIL import ImageTk

from src.ai.yolo_detector import YOLODetector
from src.config_loader import RobotConfig, load_config
from src.kinematics import inverse_kinematics, InverseKinematicsError
from src.plc.plc_controller import PLCController
from src.ui.page_auto import PageAuto
from src.ui.page_manual import PageManual
from src.ui.header import VAAHeader, VAAFooter

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

        # ── Thread synchronisation ───────────────────────────────────────────
        self._stop_event: threading.Event = threading.Event()
        self._camera_lock: threading.Lock = threading.Lock()

        # ── Camera switch debounce ───────────────────────────────────────────
        self._camera_switch_timer: threading.Timer | None = None

        # ── Stability tracking (AI vision loop) ──────────────────────────────
        self._vision_stable: bool = False      # True once stable target is sent
        self._vision_lock_x: float = 0.0      # locked X (mm)
        self._vision_lock_y: float = 0.0      # locked Y (mm)
        self._vision_stable_count: int = 0    # consecutive stable frames
        self._vision_prev_x: float = 0.0
        self._vision_prev_y: float = 0.0

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
            idx: int = int(choice_str.split(" ")[1])
        except (IndexError, ValueError) as exc:
            log.error("Cannot parse camera index from '%s': %s", choice_str, exc)
            return

        with self._camera_lock:
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

        Lock is held only during the fast frame-read step.
        YOLO inference runs unlocked, so camera switches are not blocked.

        Stability tracking: once a stable target is sent to the PLC, further
        sends are suppressed until the object moves significantly or disappears,
        reducing PLC traffic and eliminating jitter-driven duplicate commands.

        Runs until ``_stop_event`` is set.
        """
        import math as _math

        cam_cfg = self.cfg.camera
        yolo_cfg = self.cfg.yolo

        if not self.detector.start_camera(cam_cfg.default_index):
            log.error(
                "Cannot open default camera (index %d).", cam_cfg.default_index
            )

        frame_interval: float = 1.0 / max(cam_cfg.fps, 1)
        last_detection: tuple[bool, float, float] = (False, 0.0, 0.0)

        while not self._stop_event.is_set():
            # ── Step 1: fast read under lock ───────────────────────────────
            frame = None
            with self._camera_lock:
                frame = self.detector.read_frame()

            # ── Step 2: inference unlocked ───────────────────────────────────
            pil_img: ImageTk.PhotoImage | None = None
            if frame is not None:
                result = self.detector.annotate_frame(frame)
                last_detection = (result.has_defect, result.robot_x, result.robot_y)

                pil = result.to_pil(cam_cfg.display_width, cam_cfg.display_height)
                if pil is not None:
                    pil_img = ImageTk.PhotoImage(image=pil)
                    self._update_video_label(pil_img)

            # ── Step 3: PLC dispatch with stability tracking ───────────────
            has_defect, rx, ry = last_detection

            if not has_defect:
                # Object disappeared – reset stability so next detection triggers send
                if self._vision_stable:
                    log.info("Target lost – resetting stable lock.")
                self._vision_stable = False
                self._vision_stable_count = 0

            elif has_defect and self.plc.is_connected():
                if self._vision_stable:
                    # Already locked: only re-trigger if object moved significantly
                    dist = _math.hypot(rx - self._vision_lock_x, ry - self._vision_lock_y)
                    if dist > yolo_cfg.stable_threshold_mm:
                        log.info(
                            "Target moved %.2f mm (was locked at X=%.2f Y=%.2f, "
                            "now X=%.2f Y=%.2f) – re-sending.",
                            dist, self._vision_lock_x, self._vision_lock_y, rx, ry
                        )
                        self._vision_stable = False
                        self._vision_stable_count = 0
                    # else: still stable – skip send, no log spam

                if not self._vision_stable:
                    # Compute distance from previous frame's position
                    dist = _math.hypot(rx - self._vision_prev_x, ry - self._vision_prev_y)

                    if dist <= yolo_cfg.stable_threshold_mm:
                        self._vision_stable_count += 1
                    else:
                        self._vision_stable_count = 0

                    self._vision_prev_x = rx
                    self._vision_prev_y = ry

                    if self._vision_stable_count >= yolo_cfg.stable_frames:
                        # Stable enough – send once and lock
                        self._dispatch_target_to_plc(rx, ry)
                        self._vision_stable = True
                        self._vision_lock_x = rx
                        self._vision_lock_y = ry
                        self._vision_stable_count = 0
                        log.info(
                            "Target stable for %d frames – sent to PLC (X=%.2f Y=%.2f).",
                            yolo_cfg.stable_frames, rx, ry
                        )
                    else:
                        log.debug(
                            "Stability: %d/%d frames (dist=%.2f mm)",
                            self._vision_stable_count, yolo_cfg.stable_frames, dist
                        )

            self._stop_event.wait(frame_interval)

        log.info("AI vision thread exited cleanly.")

    def _dispatch_target_to_plc(self, rx: float, ry: float) -> None:
        """Compute IK and send joint targets + move command to the PLC."""
        try:
            j1, j2, j3, j4 = inverse_kinematics(
                rx, ry,
                l1=self.cfg.kinematics.l1,
                l2=self.cfg.kinematics.l2,
            )
            self.plc.send_joint_targets(j1, j2, j3, j4)
            self.plc.send_command(self.cfg.plc.commands.move)
        except InverseKinematicsError:
            log.error("IK failed: unreachable target X=%.2f Y=%.2f", rx, ry)
        except Exception as exc:
            log.error("Unexpected error during defect response: %s", exc)

    def _update_video_label(self, img_tk: ImageTk.PhotoImage) -> None:
        """Push a new frame to whichever page is currently visible (thread-safe)."""
        self.after(0, self._do_update_video, img_tk)

    def _do_update_video(self, img_tk: ImageTk.PhotoImage) -> None:
        """Internal method to update video label on main thread."""
        page = self.frames.get(self._current_page)
        if page is None:
            return
        try:
            page.update_video(img_tk)
        except Exception as exc:
            log.debug("Video label update skipped: %s", exc)

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
