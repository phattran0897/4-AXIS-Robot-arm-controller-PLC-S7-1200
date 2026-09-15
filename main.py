"""
main.py – Application entry point for the 4-Axis Industrial Robot Control System.

Responsibilities
----------------
* Bootstrap configuration, logging, PLC controller, and YOLO detector.
* Manage the CustomTkinter page-routing container.
* Own the two background threads (PLC cyclic poll + AI/camera loop).
* Provide a clean, deadlock-free shutdown via :class:`threading.Event`.
"""

from __future__ import annotations

import logging
import os
import re
import threading
import time
from logging.handlers import RotatingFileHandler
from typing import Any

import customtkinter as ctk
import tkinter

from src.ai.yolo_detector import YOLODetector
from src.config_loader import RobotConfig, load_config
from src.kinematics import configure as configure_kinematics
from src.plc.plc_controller import ADDR, PLCController
from src.robot.sorting_controller import SortResult, SortingController
from src.ui.header import VAAFooter, VAAHeader, apply_app_icon
from src.ui.page_auto import PageAuto
from src.ui.page_manual import PageManual
from src.ui.theme import ACCENT, CARD_BG, PANEL_BORDER
from PIL import Image

# ---------------------------------------------------------------------------
# Logging (console + rotating file under ./logs)
# ---------------------------------------------------------------------------
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
_LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s – %(message)s"


def _setup_logging() -> logging.Logger:
    """Attach console + rotating-file handlers exactly once."""
    robot_log = logging.getLogger("RobotApp")
    if robot_log.handlers:  # Already configured (e.g. test_gui_no_plc import)
        return robot_log
    robot_log.setLevel(logging.INFO)

    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter(_LOG_FORMAT))
    robot_log.addHandler(console)

    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        log_file = RotatingFileHandler(
            os.path.join(LOG_DIR, "robot_app.log"),
            maxBytes=20_000_000,  # 20 MB per file
            backupCount=5,
            encoding="utf-8",
        )
        log_file.setFormatter(logging.Formatter(_LOG_FORMAT))
        robot_log.addHandler(log_file)
    except OSError as exc:
        robot_log.warning("File logging disabled (cannot write %s): %s", LOG_DIR, exc)

    return robot_log


log = _setup_logging()

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_THREAD_JOIN_TIMEOUT: float = 5.0
_SORT_JOIN_TIMEOUT: float = 10.0
_CAMERA_SWITCH_DEBOUNCE: float = 0.5  # seconds


class RobotApp(ctk.CTk):
    """
    Main application window.

    Owns all hardware controllers, background threads, and GUI page routing.
    """

    def __init__(self, cfg: RobotConfig) -> None:
        super().__init__()
        self.cfg: RobotConfig = cfg

        # ── Kinematics: make config.yaml the single source of truth ────────
        kin = cfg.kinematics
        configure_kinematics(
            a1=kin.a1,
            d1=kin.d1,
            a2=kin.a2,
            a3=kin.a3,
            a4=kin.a4,
            limits={
                "j1": (kin.j1_min, kin.j1_max),
                "j2": (kin.j2_min, kin.j2_max),
                "j3": (kin.j3_min, kin.j3_max),
                "j4": (kin.j4_min, kin.j4_max),
            },
        )

        # ── Window setup ────────────────────────────────────────────────────
        ctk.set_appearance_mode(cfg.app.appearance_mode)
        ctk.set_default_color_theme(cfg.app.color_theme)
        self.title(cfg.app.title)
        self.geometry(cfg.app.geometry)
        apply_app_icon(self)  # Brand logo → window / taskbar icon

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
        self.sorter: SortingController = SortingController(
            plc=self.plc,
            positions=cfg.sort_positions,
            kinematics=cfg.kinematics,
            plc_commands=cfg.plc.commands,
        )
        self._sort_thread: threading.Thread | None = None
        self._sort_start_lock = threading.Lock()

        # Backwards-compatible alias (older code accessed controller._sorter)
        self._sorter = self.sorter

        # ── Thread synchronisation ───────────────────────────────────────────
        self._stop_event: threading.Event = threading.Event()

        # ── Camera switch debounce ───────────────────────────────────────────
        self._camera_switch_timer: threading.Timer | None = None

        # ── Manual Capture Trigger (AI vision loop) ──────────────────────────
        self._manual_classify_trigger: threading.Event = threading.Event()

        # ── Post-sort cooldown (prevents rapid re-detection) ─────────────────
        self._sort_cooldown_duration: float = (
            3.0  # seconds to ignore detections after sort
        )
        self._sort_cooldown_end: float = 0.0  # monotonic time when cooldown expires

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
        self._content_area: ctk.CTkFrame = ctk.CTkFrame(
            self._container, fg_color="#0F172A"
        )
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
            text="PLC TRANSMISSION TELEMETRY",
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
    # Thread-safe Tk dispatch
    # ------------------------------------------------------------------

    def _safe_after(self, milliseconds: int, func, *args) -> None:
        """
        Schedule *func* on the Tk main loop, ignoring the benign races that
        occur when a background thread fires during window teardown.
        """
        try:
            self.after(milliseconds, func, *args)
        except (tkinter.TclError, RuntimeError) as exc:
            log.debug("GUI dispatch skipped during shutdown: %s", exc)

    # ------------------------------------------------------------------
    # Page routing
    # ------------------------------------------------------------------

    @property
    def current_page(self) -> str:
        """Name of the currently visible page (e.g. ``"PageAuto"``)."""
        return self._current_page

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
            self.plc.send_pulse(*ADDR.AUTO_MODE)  # AUTO_MODE (offset 0.2)
        elif page_name == "PageManual":
            self.plc.send_pulse(*ADDR.MANUAL_MODE)  # MANUAL_MODE (offset 16.0)

    # ------------------------------------------------------------------
    # Camera switching (debounced)
    # ------------------------------------------------------------------

    def change_camera_source(self, choice_str: str) -> None:
        """
        Parse the combo-box selection string and debounce-switch the camera.

        Expected format: ``"Camera <index>"``
        e.g. ``"Camera 1"``
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
        match = re.search(r"\d+", choice_str)
        if not match:
            log.error("Cannot parse camera index from '%s'.", choice_str)
            return
        idx: int = int(match.group())

        success: bool = self.detector.switch_camera(idx)

        if success:
            log.info("Camera switched to index %d.", idx)
        else:
            # Tkinter dialogs must run on the main thread – marshal safely.
            self._safe_after(
                0,
                self._show_hardware_error_dialog,
                f"Cannot open camera at index {idx}.\n"
                "Check the device connection and try again.",
            )

    def _show_hardware_error_dialog(self, message: str) -> None:
        """Show a hardware error box (main thread only)."""
        from tkinter import messagebox

        messagebox.showerror("Hardware Error", message, parent=self)

    def trigger_manual_classification(self) -> None:
        """Trigger a manual capture and classification cycle."""
        self._manual_classify_trigger.set()
        log.info("Manual classification triggered via GUI.")

    # ------------------------------------------------------------------
    # Background thread: AI / camera loop
    # ------------------------------------------------------------------

    def _yolo_processing_loop(self) -> None:
        """
        Continuously capture frames and display them on the GUI with an ROI
        overlay.  When ``_manual_classify_trigger`` is set, extracts the ROI,
        runs YOLO inference, and dispatches the sorting cycle.

        The GUI preview is throttled to ``camera.preview_fps`` to keep CPU
        usage low; classification requests are always processed immediately.
        """
        import cv2

        cam_cfg = self.cfg.camera
        roi_x = self.cfg.yolo.roi_x
        roi_y = self.cfg.yolo.roi_y
        roi_w = self.cfg.yolo.roi_width
        roi_h = self.cfg.yolo.roi_height

        preview_interval: float = 1.0 / max(cam_cfg.preview_fps, 1)
        last_preview_time: float = 0.0

        try:
            if not self.detector.start_camera(cam_cfg.default_index):
                log.error(
                    "Cannot open default camera (index %d).", cam_cfg.default_index
                )

            frame_interval: float = 1.0 / max(cam_cfg.fps, 1)

            while not self._stop_event.is_set():
                frame = self.detector.read_frame()

                if frame is not None:
                    try:
                        force_preview = False

                        # Manual Trigger Check (uses the raw frame)
                        if self._manual_classify_trigger.is_set():
                            self._manual_classify_trigger.clear()
                            self._handle_manual_classification(frame)
                            force_preview = True

                        # Annotate + convert ONLY when the preview will
                        # actually be redrawn – skipped ticks save the copy,
                        # the overlay drawing, and the colour conversion.
                        now = time.monotonic()
                        if (
                            force_preview
                            or (now - last_preview_time) >= preview_interval
                        ):
                            last_preview_time = now

                            # Draw ROI overlay in place (frame is not reused)
                            cv2.rectangle(
                                frame,
                                (roi_x, roi_y),
                                (roi_x + roi_w, roi_y + roi_h),
                                (0, 255, 0),
                                2,
                            )
                            cv2.putText(
                                frame,
                                "ROI - Dat Hop Vao Day",
                                (roi_x, roi_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 0),
                                2,
                            )

                            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            pil_image = Image.fromarray(rgb)
                            self._update_video_label(pil_image)

                    except Exception as frame_exc:
                        log.error(
                            "Exception during frame processing: %s",
                            frame_exc,
                            exc_info=True,
                        )

                self._stop_event.wait(frame_interval)
        except Exception as thread_exc:
            log.critical(
                "AIVisionThread crashed with exception: %s", thread_exc, exc_info=True
            )

        log.info("AI vision thread exited cleanly.")

    def _handle_manual_classification(self, frame: Any) -> None:
        """
        Run one classification cycle: crop the configured ROI, sharpen it,
        run YOLO inference, and dispatch the sorting cycle when an object
        is actually detected.
        """
        import cv2

        roi_x = self.cfg.yolo.roi_x
        roi_y = self.cfg.yolo.roi_y
        roi_w = self.cfg.yolo.roi_width
        roi_h = self.cfg.yolo.roi_height

        # Ensure ROI is within bounds
        fh, fw = frame.shape[:2]
        if roi_x >= fw or roi_y >= fh or roi_w <= 0 or roi_h <= 0:
            log.warning(
                "ROI (x=%d y=%d w=%d h=%d) lies outside %dx%d frame – "
                "capture skipped.",
                roi_x,
                roi_y,
                roi_w,
                roi_h,
                fw,
                fh,
            )
            self.log_tx("Capture skipped: ROI outside camera frame.")
            return
        cx = max(0, min(roi_x, fw))
        cy = max(0, min(roi_y, fh))
        cw = max(1, min(roi_w, fw - cx))
        ch = max(1, min(roi_h, fh - cy))

        roi_crop = frame[cy : cy + ch, cx : cx + cw]

        # Apply Unsharp Masking to sharpen ROI before inference
        blurred = cv2.GaussianBlur(roi_crop, (0, 0), 2.0)
        roi_crop = cv2.addWeighted(roi_crop, 1.5, blurred, -0.5, 0)

        # Run inference on crop
        result = self.detector.annotate_frame(roi_crop)

        if not result.has_defect:
            log.info("Manual Capture: no object detected above threshold – skipped.")
            self.log_tx("Capture: no object detected – sort skipped.")
            return

        sort_result = SortResult.BAD if result.class_id == 0 else SortResult.GOOD

        log.info(
            "Manual Capture: Object Classified as %s (class=%d, X=%.1fmm, Y=%.1fmm)",
            sort_result.name,
            result.class_id,
            result.robot_x,
            result.robot_y,
        )
        self.log_tx(f"Classified: {sort_result.name} ({result.class_name or 'obj'})")

        # Visual feedback handled by caller overlay; dispatch the cycle with
        # the detected coordinates so picking is vision-guided.
        self._start_sort_cycle(result.robot_x, result.robot_y, sort_result)

    def _start_sort_cycle(self, rx: float, ry: float, result: SortResult) -> bool:
        """
        Launch the sorting cycle on a worker thread unless one is already
        running or the post-sort cooldown is active.

        Returns ``True`` if a cycle was started.
        """
        # Post-sort cooldown prevents immediate re-triggering on the same part
        remaining = self._sort_cooldown_end - time.monotonic()
        if remaining > 0:
            log.info("Sort suppressed – cooldown active for another %.1fs.", remaining)
            self.log_tx(f"Sort suppressed (cooldown {remaining:.1f}s).")
            return False

        with self._sort_start_lock:
            if self._sort_thread is not None and self._sort_thread.is_alive():
                log.warning("Sort thread still running – skipping new sort.")
                return False
            self._sort_thread = threading.Thread(
                target=self._run_sort_and_reset_lock,
                args=(rx, ry, result),
                name="SortCycleThread",
                daemon=True,
            )
            self._sort_thread.start()
            return True

    def _run_sort_and_reset_lock(
        self, rx: float, ry: float, sort_result: SortResult
    ) -> None:
        """Execute sorting cycle, guarding against PLC faults."""
        # E-stop gate: never start motion while the PLC reports an error
        status = self.plc.read_status()
        if status and status.get("error_flag", False):
            log.error("Sort aborted – PLC error flag is active.")
            self.log_tx("Sort ABORTED: PLC error flag active.")
            return
        if status and not status.get("auto_mode", True):
            # Manual mode – pulsing START_AUTO here could combine with an
            # operator's jog inputs into unintended motion.
            log.warning("Sort aborted – AUTO mode bit is off.")
            self.log_tx("Sort ABORTED: AUTO mode is off (manual mode).")
            return

        try:
            self.sorter.execute_sort(rx, ry, sort_result)
        except Exception as exc:
            log.error("Sorting cycle failed: %s", exc)
        finally:
            # Activate post-sort cooldown to prevent immediate re-detection
            self._sort_cooldown_end = time.monotonic() + self._sort_cooldown_duration
            log.info(
                "Post-sort cooldown activated (%.1fs).",
                self._sort_cooldown_duration,
            )

    def clear_all_errors(self) -> None:
        """Clear errors on PLC and reset sorting controller state."""
        self.plc.send_command(self.cfg.plc.commands.idle)
        self.sorter.clear_error()

    def _update_video_label(self, img: Image.Image) -> None:
        """Push a new frame to whichever page is currently visible (thread-safe)."""
        self._safe_after(0, self._do_update_video, img)

    def _do_update_video(self, img: Image.Image) -> None:
        """Internal method to update video label on main thread."""
        page = self.frames.get(self._current_page)
        if page is None:
            return
        try:
            # Use dynamic size from video container (scales on fullscreen)
            disp_w = getattr(
                page, "_video_display_width", self.cfg.camera.display_width
            )
            disp_h = getattr(
                page, "_video_display_height", self.cfg.camera.display_height
            )

            # Preserve aspect ratio while fitting into the container
            orig_w, orig_h = img.size
            if orig_w > 0 and orig_h > 0:
                aspect = orig_w / orig_h
                container_aspect = disp_w / disp_h

                if aspect > container_aspect:
                    # Image is wider than container, constrain by width
                    target_w = disp_w
                    target_h = int(disp_w / aspect)
                else:
                    # Image is taller than container, constrain by height
                    target_h = disp_h
                    target_w = int(disp_h * aspect)
            else:
                target_w, target_h = disp_w, disp_h

            ctk_img = ctk.CTkImage(
                light_image=img,
                dark_image=img,
                size=(target_w, target_h),
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

        self._safe_after(0, _append)

    # ------------------------------------------------------------------
    # Background thread: PLC cyclic poll with exponential backoff
    # ------------------------------------------------------------------

    def _cyclic_update(self) -> None:
        """
        Poll the PLC at the configured interval and push fresh data to
        all page ``update_gui_data()`` callbacks via ``self.after()``.

        Falls back to reconnection with exponential backoff when the PLC
        is offline.  Runs until ``_stop_event`` is set.
        """
        interval: float = self.cfg.app.plc_poll_interval
        retry_delay: float = interval
        max_retry_delay: float = 30.0
        consecutive_failures: int = 0

        while not self._stop_event.is_set():
            if self.plc.is_connected():
                data: dict[str, Any] = self.plc.read_status()
                if data:
                    self._safe_after(0, self._dispatch_plc_data, data)
                retry_delay = interval
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures <= 3 or consecutive_failures % 10 == 0:
                    log.warning(
                        "PLC offline – attempting reconnection "
                        "(attempt %d, backoff %.1fs)…",
                        consecutive_failures,
                        retry_delay,
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
        camera_active = (
            self.detector.is_camera_active()
            if hasattr(self.detector, "is_camera_active")
            else False
        )
        system_ok = not data.get("error_flag", False)
        self.footer.update_status(plc_connected, camera_active, system_ok)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def on_closing(self) -> None:
        """
        Graceful shutdown sequence:

        1. Signal all background threads to stop via ``_stop_event``.
        2. Let an in-flight sort cycle finish (bounded wait) BEFORE cutting
           PLC comms — disconnecting first would abort the robot mid-motion
           with an undefined gripper state.
        3. Release hardware resources (camera, PLC socket).
        4. Join remaining threads with a timeout to prevent hangs.
        5. Destroy the Tk window.
        """
        log.info("Shutdown initiated…")
        self._stop_event.set()

        if self._camera_switch_timer is not None:
            self._camera_switch_timer.cancel()

        # Wait for active sorting thread to complete gracefully while the
        # PLC connection is still alive.
        if self._sort_thread is not None and self._sort_thread.is_alive():
            log.info("Waiting for active sorting thread to complete...")
            self._sort_thread.join(timeout=_SORT_JOIN_TIMEOUT)

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
