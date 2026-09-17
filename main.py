"""
main.py – Application entry point for the 4-Axis Industrial Robot Control System (PySide6).

Responsibilities
----------------
* Bootstrap configuration, logging, PLC controller, and YOLO detector.
* Manage the PySide6 QStackedWidget page-routing container.
* Own the two background threads (PLC cyclic poll + AI/camera loop).
* Provide thread-safe GUI updates via Qt Signals and Slots.
* Provide a clean, deadlock-free shutdown via :class:`threading.Event` and closeEvent().
"""

from __future__ import annotations

import logging
import os
import re
import sys
import threading
import time
from logging.handlers import RotatingFileHandler
from typing import Any

import cv2
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QCloseEvent, QFont, QImage, QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from src.ai.yolo_detector import YOLODetector
from src.config_loader import RobotConfig, load_config
from src.kinematics import configure as configure_kinematics
from src.plc.plc_controller import ADDR, PLCController
from src.robot.sorting_controller import SortResult, SortingController
from src.ui.header import VAAFooter, VAAHeader, apply_app_icon
from src.ui.page_auto import PageAuto
from src.ui.page_manual import PageManual
from src.ui.theme import (
    ACCENT,
    ACCENT_DIM,
    CARD_BG,
    GLOBAL_QSS,
    PANEL_BORDER,
    TEXT_DIM,
    card_style,
)

# ---------------------------------------------------------------------------
# Logging (console + rotating file under ./logs)
# ---------------------------------------------------------------------------
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
_LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s – %(message)s"


def _setup_logging() -> logging.Logger:
    """Attach console + rotating-file handlers exactly once."""
    robot_log = logging.getLogger("RobotApp")
    if robot_log.handlers:  # Already configured
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


class RobotApp(QMainWindow):
    """
    Main application window (PySide6 QMainWindow).

    Owns all hardware controllers, background threads, and GUI page routing.
    """

    # Qt Signals for thread-safe worker-to-UI dispatch
    frame_ready = Signal(QImage)
    plc_data_ready = Signal(dict)
    tx_logged = Signal(str)
    show_error_dialog = Signal(str)

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
        self.setWindowTitle(cfg.app.title)
        if "x" in cfg.app.geometry:
            try:
                gw, gh = map(int, cfg.app.geometry.split("x"))
                self.resize(gw, gh)
            except Exception:
                self.resize(1280, 820)
        else:
            self.resize(1280, 820)

        apply_app_icon(self)

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
            calibration_path=cfg.yolo.calibration_path,
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

        # ── Camera switch debounce (QTimer on Qt loop) ────────────────────────
        self._camera_timer: QTimer = QTimer(self)
        self._camera_timer.setSingleShot(True)
        self._pending_camera_choice: str = ""
        self._camera_timer.timeout.connect(self._execute_pending_camera_switch)

        # ── Manual Capture Trigger (AI vision loop) ──────────────────────────
        self._manual_classify_trigger: threading.Event = threading.Event()

        # ── Post-sort cooldown (prevents rapid re-detection) ─────────────────
        self._sort_cooldown_duration: float = 3.0  # seconds
        self._sort_cooldown_end: float = 0.0

        # ── Signal / Slot connections ────────────────────────────────────────
        self.frame_ready.connect(
            self._on_frame_ready, Qt.ConnectionType.QueuedConnection
        )
        self.plc_data_ready.connect(
            self._dispatch_plc_data, Qt.ConnectionType.QueuedConnection
        )
        self.tx_logged.connect(self._on_tx_logged, Qt.ConnectionType.QueuedConnection)
        self.show_error_dialog.connect(
            self._show_hardware_error_dialog, Qt.ConnectionType.QueuedConnection
        )

        # ── GUI layout ───────────────────────────────────────────────────────
        self._build_central_ui()

        # Wire up callback to PLCController
        self.plc.tx_callback = self.log_tx

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
    # GUI construction
    # ------------------------------------------------------------------

    def _build_central_ui(self) -> None:
        central_widget = QWidget(self)
        central_widget.setObjectName("CentralWidget")
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Header
        self.header: VAAHeader = VAAHeader(
            parent=central_widget,
            controller=self,
            current_page="PageAuto",
        )
        main_layout.addWidget(self.header)

        # Stacked pages
        self.stacked_widget = QStackedWidget(central_widget)
        self.frames: dict[str, PageAuto | PageManual] = {}

        self.frames["PageAuto"] = PageAuto(parent=self.stacked_widget, controller=self)
        self.frames["PageManual"] = PageManual(
            parent=self.stacked_widget, controller=self
        )

        self.stacked_widget.addWidget(self.frames["PageAuto"])
        self.stacked_widget.addWidget(self.frames["PageManual"])

        main_layout.addWidget(self.stacked_widget, 1)

        # ── PLC Tx Console ──────────────────────────────────────────────────
        self.tx_console_frame = QFrame(central_widget)
        self.tx_console_frame.setStyleSheet(card_style(CARD_BG, PANEL_BORDER, 12))
        tx_layout = QVBoxLayout(self.tx_console_frame)
        tx_layout.setContentsMargins(14, 8, 14, 10)
        tx_layout.setSpacing(6)

        # Title row with accent bar
        title_row = QWidget(self.tx_console_frame)
        title_row.setStyleSheet("background: transparent; border: none;")
        tr_layout = QHBoxLayout(title_row)
        tr_layout.setContentsMargins(0, 0, 0, 0)
        tr_layout.setSpacing(8)

        accent_bar = QFrame(title_row)
        accent_bar.setFixedSize(3, 14)
        accent_bar.setStyleSheet(
            f"background-color: {ACCENT}; border-radius: 1px; border: none;"
        )
        tr_layout.addWidget(accent_bar)

        lbl_console_title = QLabel(
            "PLC TRANSMISSION TELEMETRY", title_row
        )
        lbl_console_title.setStyleSheet(
            f"color: {ACCENT}; font-size: 11px; font-weight: bold; "
            f"letter-spacing: 1px; border: none; background: transparent;"
        )
        tr_layout.addWidget(lbl_console_title)
        tr_layout.addStretch()

        lbl_tx_badge = QLabel("● LIVE", title_row)
        lbl_tx_badge.setStyleSheet(
            f"color: {ACCENT_DIM}; font-size: 10px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        tr_layout.addWidget(lbl_tx_badge)
        tx_layout.addWidget(title_row)

        # Scrolling text log
        self.tx_textbox = QTextEdit(self.tx_console_frame)
        self.tx_textbox.setFixedHeight(80)
        self.tx_textbox.setReadOnly(True)
        self.tx_textbox.setFont(QFont("Consolas", 10))
        self.tx_textbox.setStyleSheet(
            f"background-color: #020617; color: #38BDF8; "
            f"border: 1px solid {PANEL_BORDER}; border-radius: 8px; padding: 6px;"
        )
        tx_layout.addWidget(self.tx_textbox)

        # Outer wrapper with padding
        tx_wrapper = QWidget(central_widget)
        tx_wrapper.setStyleSheet("background: transparent; border: none;")
        tx_w_layout = QVBoxLayout(tx_wrapper)
        tx_w_layout.setContentsMargins(16, 4, 16, 8)
        tx_w_layout.addWidget(self.tx_console_frame)
        main_layout.addWidget(tx_wrapper)

        # Footer
        self.footer: VAAFooter = VAAFooter(parent=central_widget)
        main_layout.addWidget(self.footer)

        self._current_page: str = "PageAuto"
        self.show_frame("PageAuto")

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
        self.stacked_widget.setCurrentWidget(self.frames[page_name])
        self.header.update_active_tab(page_name)
        log.debug("Switched to page: %s", page_name)
        if page_name == "PageAuto":
            self.plc.send_pulse(*ADDR.AUTO_MODE)
        elif page_name == "PageManual":
            self.plc.send_pulse(*ADDR.MANUAL_MODE)

    # ------------------------------------------------------------------
    # Camera switching (debounced via QTimer)
    # ------------------------------------------------------------------

    def change_camera_source(self, choice_str: str) -> None:
        """Debounce camera switch request on the Qt event loop."""
        self._pending_camera_choice = choice_str
        self._camera_timer.start(int(_CAMERA_SWITCH_DEBOUNCE * 1000))

    def _execute_pending_camera_switch(self) -> None:
        if self._pending_camera_choice:
            self._do_change_camera(self._pending_camera_choice)

    def _do_change_camera(self, choice_str: str) -> None:
        """Actually perform the camera switch."""
        match = re.search(r"\d+", choice_str)
        if not match:
            log.error("Cannot parse camera index from '%s'.", choice_str)
            return
        idx: int = int(match.group())
        success: bool = self.detector.switch_camera(idx)
        if success:
            log.info("Camera switched to index %d.", idx)
        else:
            self.show_error_dialog.emit(
                f"Cannot open camera at index {idx}.\n"
                "Check the device connection and try again."
            )

    def _show_hardware_error_dialog(self, message: str) -> None:
        """Show hardware error box on main thread."""
        QMessageBox.critical(self, "Hardware Error", message)

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
        overlay. Direct QImage conversion without PIL.
        """
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

                        now = time.monotonic()
                        if (
                            force_preview
                            or (now - last_preview_time) >= preview_interval
                        ):
                            last_preview_time = now

                            # Draw ROI overlay on a copy for display
                            display_frame = frame.copy()
                            cv2.rectangle(
                                display_frame,
                                (roi_x, roi_y),
                                (roi_x + roi_w, roi_y + roi_h),
                                (0, 255, 0),
                                2,
                            )
                            cv2.putText(
                                display_frame,
                                "ROI - Dat Hop Vao Day",
                                (roi_x, roi_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 0),
                                2,
                            )

                            h, w, ch = display_frame.shape
                            bytes_per_line = ch * w
                            # Create thread-safe detached QImage
                            qimg = QImage(
                                display_frame.data,
                                w,
                                h,
                                bytes_per_line,
                                QImage.Format.Format_BGR888,
                            ).copy()
                            self.frame_ready.emit(qimg)

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
        """Run one classification cycle on cropped ROI."""
        roi_x = self.cfg.yolo.roi_x
        roi_y = self.cfg.yolo.roi_y
        roi_w = self.cfg.yolo.roi_width
        roi_h = self.cfg.yolo.roi_height

        fh, fw = frame.shape[:2]
        if roi_x >= fw or roi_y >= fh or roi_w <= 0 or roi_h <= 0:
            log.warning("ROI lies outside frame – capture skipped.")
            self.log_tx("Capture skipped: ROI outside camera frame.")
            return

        cx = max(0, min(roi_x, fw))
        cy = max(0, min(roi_y, fh))
        cw = max(1, min(roi_w, fw - cx))
        ch = max(1, min(roi_h, fh - cy))

        roi_crop = frame[cy : cy + ch, cx : cx + cw]

        if getattr(self.cfg.camera, "enable_unsharp_mask", True):
            blurred = cv2.GaussianBlur(roi_crop, (0, 0), 2.0)
            roi_crop = cv2.addWeighted(roi_crop, 1.5, blurred, -0.5, 0)

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
        self._start_sort_cycle(result.robot_x, result.robot_y, sort_result)

    def _start_sort_cycle(self, rx: float, ry: float, result: SortResult) -> bool:
        """Launch the sorting cycle on a worker thread."""
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
        status = self.plc.read_status()
        if status and status.get("error_flag", False):
            log.error("Sort aborted – PLC error flag is active.")
            self.log_tx("Sort ABORTED: PLC error flag active.")
            return
        if status and not status.get("auto_mode", True):
            log.warning("Sort aborted – AUTO mode bit is off.")
            self.log_tx("Sort ABORTED: AUTO mode is off (manual mode).")
            return

        try:
            self.sorter.execute_sort(rx, ry, sort_result)
        except Exception as exc:
            log.error("Sorting cycle failed: %s", exc)
        finally:
            self._sort_cooldown_end = time.monotonic() + self._sort_cooldown_duration
            log.info("Post-sort cooldown activated (%.1fs).", self._sort_cooldown_duration)

    def clear_all_errors(self) -> None:
        """Clear errors on PLC and reset sorting controller state."""
        self.plc.send_command(self.cfg.plc.commands.idle)
        self.sorter.clear_error()

    # ------------------------------------------------------------------
    # Main thread slots
    # ------------------------------------------------------------------

    def _on_frame_ready(self, qimg: QImage) -> None:
        """Deliver new frame to the currently active page."""
        current_page = self.stacked_widget.currentWidget()
        if hasattr(current_page, "update_video"):
            current_page.update_video(qimg)

    def log_tx(self, message: str) -> None:
        """Emit telemetry message to be appended on the main thread."""
        self.tx_logged.emit(message)

    def _on_tx_logged(self, message: str) -> None:
        """Append a timestamped message to the transmission telemetry log."""
        timestamp = time.strftime("%H:%M:%S")
        self.tx_textbox.append(f"[{timestamp}] {message}")
        doc = self.tx_textbox.document()
        if doc.blockCount() > 100:
            cursor = self.tx_textbox.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            cursor.select(QTextCursor.SelectionType.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    # ------------------------------------------------------------------
    # Background thread: PLC cyclic poll with exponential backoff
    # ------------------------------------------------------------------

    def _cyclic_update(self) -> None:
        """Poll PLC and emit status data via Qt Signal."""
        interval: float = self.cfg.app.plc_poll_interval
        retry_delay: float = interval
        max_retry_delay: float = 30.0
        consecutive_failures: int = 0

        while not self._stop_event.is_set():
            if self.plc.is_connected():
                data: dict[str, Any] = self.plc.read_status()
                if data:
                    self.plc_data_ready.emit(data)
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

        plc_connected = self.plc.is_connected()
        camera_active = (
            self.detector.is_camera_active()
            if hasattr(self.detector, "is_camera_active")
            else False
        )
        system_ok = not data.get("error_flag", False)
        self.footer.update_status(plc_connected, camera_active, system_ok)

    # ------------------------------------------------------------------
    # Compatibility methods
    # ------------------------------------------------------------------

    def title(self, title_str: str) -> None:
        self.setWindowTitle(title_str)

    def protocol(self, name: str, func: Any) -> None:
        pass

    def mainloop(self) -> int:
        self.show()
        qapp = QApplication.instance()
        return qapp.exec() if qapp else 0

    def on_closing(self) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def closeEvent(self, event: QCloseEvent) -> None:
        """Graceful shutdown on window close."""
        log.info("Shutdown initiated via closeEvent...")
        self._stop_event.set()

        if hasattr(self, "_camera_timer") and self._camera_timer.isActive():
            self._camera_timer.stop()

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

        log.info("All threads stopped. Window closing.")
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(GLOBAL_QSS)

    config: RobotConfig = load_config()
    main_window = RobotApp(cfg=config)
    main_window.show()
    sys.exit(app.exec())
