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
from pathlib import Path
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
    QPushButton,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from src.ai.yolo_detector import YOLODetector
from src.config_loader import RobotConfig, load_config
from src.kinematics import configure as configure_kinematics
from src.plc.plc_controller import ADDR, PLCController
from src.ai.qr_reader import QRReader
from src.robot.sorting_controller import SortResult, SortingController
from src.ui.header import VAAFooter, VAAHeader, apply_app_icon
from src.ui.help_text import LOG_HELP, set_help, set_help_notice
from src.ui.page_auto import PageAuto
from src.ui.page_manual import PageManual
from src.ui.page_settings import PageSettings
from src.ui.theme import (
    ACCENT,
    ACCENT_DIM,
    CARD_BG,
    GLOBAL_QSS,
    PANEL_BORDER,
    TEXT_DIM,
    TEXT_SECONDARY,
    btn_style,
    card_style,
)

# ---------------------------------------------------------------------------
# Logging (console + rotating file under ./logs)
# ---------------------------------------------------------------------------
_RUNTIME_DIR = Path(sys.executable).parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
LOG_DIR = str(_RUNTIME_DIR / "logs")
_LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s – %(message)s"


def _setup_logging() -> logging.Logger:
    """Attach console + rotating-file handlers exactly once."""
    robot_log = logging.getLogger("RobotApp")
    if robot_log.handlers:  # Already configured
        return robot_log
    robot_log.setLevel(logging.INFO)
    robot_log.propagate = False

    console = logging.StreamHandler()
    console.setFormatter(logging.Formatter(_LOG_FORMAT))
    robot_log.addHandler(console)

    directories = [Path(LOG_DIR), Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "RobotController" / "logs"]
    for directory in directories:
        try:
            directory.mkdir(parents=True, exist_ok=True)
            log_file = RotatingFileHandler(
                directory / "robot_app.log",
                maxBytes=20_000_000,
                backupCount=5,
                encoding="utf-8",
            )
            log_file.setFormatter(logging.Formatter(_LOG_FORMAT))
            robot_log.addHandler(log_file)
            break
        except OSError as exc:
            robot_log.warning("Cannot write logs to %s: %s", directory, exc)

    # Capture PLC, camera, kinematics and sorter diagnostics in the same file.
    # Keep unrelated library/root logging under the embedding application's control.
    source_log = logging.getLogger("src")
    source_log.setLevel(logging.INFO)
    source_log.propagate = False
    for handler in robot_log.handlers:
        if handler not in source_log.handlers:
            source_log.addHandler(handler)

    return robot_log


log = _setup_logging()

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_THREAD_JOIN_TIMEOUT: float = 5.0
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
    qr_status_ready = Signal(str, str)

    def __init__(self, cfg: RobotConfig) -> None:
        super().__init__()
        self.cfg: RobotConfig = cfg
        self._closing = False
        self._vision_error = ""

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
            class_quality=cfg.yolo.class_quality,
            lazy_model=cfg.app.sorting_mode == "qr",
        )
        try:
            self.plc: PLCController = PLCController(cfg.plc)
            self.sorter: SortingController = SortingController(
                plc=self.plc,
                positions=cfg.sort_positions,
                kinematics=cfg.kinematics,
                plc_commands=cfg.plc.commands,
            )
            self.qr_reader = QRReader(debounce_frames=cfg.app.qr_debounce_frames,
                                      clear_frames=cfg.app.qr_clear_frames,
                                      miss_tolerance=cfg.app.qr_miss_tolerance)
            self._sort_thread: threading.Thread | None = None
            self._sort_start_lock = threading.Lock()
            self._camera_request_lock = threading.Lock()
            self._requested_camera_index: int | None = None
            self._latest_plc_data: dict[str, Any] = {"connected": False}

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
            self._classification_running = threading.Event()

            # ── Post-sort cooldown (prevents rapid re-detection) ─────────────────
            self._sort_cooldown_duration: float = cfg.app.move_cooldown
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

            self.refresh_sorting_controls()
            if self._vision_error:
                message = self._vision_error
                QTimer.singleShot(0, lambda: self.show_error_dialog.emit(message))

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
            self._frame_pending = False
            self._plc_thread.start()
            self._ai_thread.start()


            log.info("RobotApp initialised – all threads started.")
        except Exception:
            self._closing = True
            if hasattr(self, "_stop_event"):
                self._stop_event.set()
            self.detector.stop()
            if hasattr(self, "plc"):
                self.plc.disconnect()
            raise


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
        self.page_auto = PageAuto(parent=self.stacked_widget, controller=self)
        self.page_manual = PageManual(parent=self.stacked_widget, controller=self)
        self.page_settings = PageSettings(parent=self.stacked_widget, controller=self)

        self.stacked_widget.addWidget(self.page_auto)
        self.stacked_widget.addWidget(self.page_manual)
        self.stacked_widget.addWidget(self.page_settings)

        self.frames = {
            "PageAuto": self.page_auto,
            "PageManual": self.page_manual,
            "PageSettings": self.page_settings,
        }

        # Ensure page_auto has update_qr_status, connect signal
        if hasattr(self.page_auto, "update_qr_status"):
            self.qr_status_ready.connect(
                self.page_auto.update_qr_status, Qt.ConnectionType.QueuedConnection
            )

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
            "ACTIVITY LOG", title_row
        )
        lbl_console_title.setStyleSheet(
            f"color: {ACCENT}; font-size: 11px; font-weight: bold; "
            f"letter-spacing: 1px; border: none; background: transparent;"
        )
        tr_layout.addWidget(lbl_console_title)
        tr_layout.addStretch()

        lbl_tx_badge = QLabel("PLC COMMANDS & EVENTS", title_row)
        lbl_tx_badge.setStyleSheet(
            f"color: {ACCENT_DIM}; font-size: 10px; font-weight: bold; "
            f"border: none; background: transparent;"
        )
        tr_layout.addWidget(lbl_tx_badge)
        log_toggle = QPushButton("Show log", title_row)
        log_toggle.setCheckable(True)
        log_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        log_toggle.setStyleSheet(btn_style(bg="transparent", hover="#243349", text=TEXT_SECONDARY,
                                          font_size=11, padding="4px 12px", border=f"1px solid {PANEL_BORDER}"))
        tr_layout.addWidget(log_toggle)
        tx_layout.addWidget(title_row)

        # Scrolling text log
        self.tx_textbox = QTextEdit(self.tx_console_frame)
        self.tx_textbox.setFixedHeight(80)
        self.tx_textbox.setReadOnly(True)
        for widget in (lbl_console_title, lbl_tx_badge, log_toggle, self.tx_textbox):
            set_help(widget, "Nhật ký hoạt động", LOG_HELP)
        self.tx_textbox.setPlaceholderText("PLC commands and application events appear here.")
        self.tx_textbox.document().setMaximumBlockCount(100)
        self.tx_textbox.setFont(QFont("Consolas", 10))
        self.tx_textbox.setStyleSheet(
            f"background-color: #020617; color: #38BDF8; "
            f"border: 1px solid {PANEL_BORDER}; border-radius: 8px; padding: 6px;"
        )
        tx_layout.addWidget(self.tx_textbox)
        self.tx_textbox.hide()
        log_toggle.toggled.connect(self.tx_textbox.setVisible)
        log_toggle.toggled.connect(lambda expanded: log_toggle.setText("Hide log" if expanded else "Show log"))

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
        if self._closing:
            return
        if page_name == "PageManual" and (self._classification_running.is_set() or
                (self._sort_thread is not None and self._sort_thread.is_alive())):
            self.log_tx("Cannot switch to Manual while a sort cycle is running; use STOP first.")
            return
        if getattr(self, "_current_page", None) == "PageManual" and page_name != "PageManual":
            if self.page_manual.release_jog() is False:
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

    def validate_vision_configuration(self, config=None) -> None:
        config = config or self.cfg
        if config.app.sorting_mode == "vision":
            self.plc.validate_motion_layout()

    def refresh_sorting_controls(self) -> None:
        self._vision_error = ""
        try:
            self.validate_vision_configuration()
        except ValueError as error:
            self._vision_error = str(error)
        enabled = not self._vision_error and not self._closing
        if hasattr(self, "_latest_plc_data"):
            enabled = enabled and self._latest_plc_data.get("connected", False)
            enabled = enabled and not self._latest_plc_data.get("error_flag", False)
            enabled = enabled and not self._latest_plc_data.get("stop_robot", False)
        enabled = enabled and self.sorter.is_idle()
        self.page_auto.btn_capture.setVisible(self.cfg.app.sorting_mode == "vision")
        self.page_auto.btn_capture.setEnabled(enabled)
        self.page_auto.btn_start.setEnabled(enabled)
        notice = self._vision_error
        if not notice and not enabled:
            status = getattr(self, "_latest_plc_data", {})
            if self._closing:
                notice = "Ứng dụng đang đóng."
            elif status.get("connected") is False:
                notice = "PLC đang ngoại tuyến. Kiểm tra kết nối và chờ trạng thái được cập nhật."
            elif status.get("error_flag"):
                notice = "PLC đang báo lỗi. Xử lý nguyên nhân rồi dùng Clear error."
            elif status.get("stop_robot"):
                notice = "Lệnh Stop đang được giữ. Kiểm tra hệ thống trước khi dùng Clear error."
            else:
                notice = "Bộ điều khiển đang bận hoặc cần xóa lỗi. Xem trạng thái chu trình và Activity log."
        for button in (self.page_auto.btn_capture, self.page_auto.btn_start):
            set_help_notice(button, notice)
        if self._vision_error:
            self.page_auto.update_qr_status(self._vision_error, "#EF4444")

    def start_auto(self) -> None:
        if self._closing:
            return
        if self._classification_running.is_set() or not self.sorter.is_idle() or (self._sort_thread is not None and self._sort_thread.is_alive()):
            self.log_tx("START AUTO ignored: a cycle is running or the controller needs a reset.")
            return
        try:
            self.validate_vision_configuration()
        except ValueError as error:
            self.show_error_dialog.emit(str(error))
            return
        if self.plc.send_pulse(*ADDR.START_AUTO) is False:
            self.log_tx("START AUTO failed; PLC disconnected or STOP is latched.")

    def stop_robot(self) -> None:
        self._manual_classify_trigger.clear()
        if self.sorter.request_stop() is False:
            self.log_tx("STOP requested but PLC write failed; machine stop is unconfirmed.")

    def change_camera_source(self, choice_str: str) -> None:
        """Debounce camera switch request on the Qt event loop."""
        self._pending_camera_choice = choice_str
        self._camera_timer.start(int(_CAMERA_SWITCH_DEBOUNCE * 1000))

    def _execute_pending_camera_switch(self) -> None:
        if self._pending_camera_choice:
            self._do_change_camera(self._pending_camera_choice)

    def _do_change_camera(self, choice_str: str) -> None:
        """Queue camera I/O for the AI worker, keeping the Qt thread responsive."""
        match = re.search(r"\d+", choice_str)
        if not match:
            log.error("Cannot parse camera index from '%s'.", choice_str)
            return
        idx: int = int(match.group())
        with self._camera_request_lock:
            self._requested_camera_index = idx

    def _apply_camera_request(self) -> None:
        with self._camera_request_lock:
            idx = self._requested_camera_index
            self._requested_camera_index = None
        if idx is None or self._stop_event.is_set():
            return
        self._manual_classify_trigger.clear()
        success: bool = self.detector.switch_camera(idx)
        if success:
            self.qr_reader.reset()
            self.sorter.process_qr_target("CLEAR")
            log.info("Camera switched to index %d.", idx)
        else:
            self.show_error_dialog.emit(
                f"Cannot open camera at index {idx}.\n"
                "Check the device connection and try again."
            )

    def _show_hardware_error_dialog(self, message: str) -> None:
        """Show hardware error box on main thread."""
        if self._closing:
            return
        QMessageBox.critical(self, "Hardware Error", message)

    def trigger_manual_classification(self) -> None:
        """Trigger a manual capture and classification cycle."""
        if self._closing or self.cfg.app.sorting_mode != "vision":
            return
        if self._classification_running.is_set() or not self.sorter.is_idle() or (self._sort_thread is not None and self._sort_thread.is_alive()):
            self.log_tx("Capture ignored: wait for the current cycle or clear the fault.")
            return
        if time.monotonic() < self._sort_cooldown_end:
            self.log_tx("Capture ignored: post-sort cooldown is active.")
            return
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
        preview_interval: float = 1.0 / max(cam_cfg.preview_fps, 1)
        last_preview_time: float = 0.0
        last_mode = self.cfg.app.sorting_mode

        try:
            if not self.detector.start_camera(cam_cfg.default_index):
                log.error(
                    "Cannot open default camera (index %d).", cam_cfg.default_index
                )

            frame_interval: float = 1.0 / max(cam_cfg.fps, 1)

            while not self._stop_event.is_set():
                self._apply_camera_request()
                frame = self.detector.read_frame()

                if frame is not None:
                    try:
                        force_preview = False
                        if self.cfg.app.sorting_mode != last_mode:
                            self.qr_reader.reset()
                            self.sorter.process_qr_target("CLEAR")
                            self._manual_classify_trigger.clear()
                            last_mode = self.cfg.app.sorting_mode

                        if self.cfg.app.sorting_mode == "qr":
                            # PLC-driven autonomous QR mode
                            status, qr_val = self.qr_reader.decode_qr(frame)
                            if status == "clear":
                                self.sorter.process_qr_target("CLEAR")
                                self.qr_status_ready.emit("QR: Băng chuyền đang chạy...", "#94A3B8")
                                force_preview = True
                            elif status == "value" and qr_val:
                                if qr_val not in self.cfg.sort_positions.locations:
                                    # Mã không có trong bảng cấu hình
                                    self.sorter.process_qr_target(qr_val)
                                    self.qr_status_ready.emit(f"QR: Không tồn tại [{qr_val}]", "#EF4444")
                                elif self.sorter.process_qr_target(qr_val):
                                    # Ghi xuống PLC thành công
                                    self.log_tx(f"QR Scanned & Written: {qr_val}")
                                    self.qr_status_ready.emit(f"QR: Ghi thành công [{qr_val}]", "#10B981")
                                elif not self.sorter.is_qr_handled(qr_val):
                                    # Ghi thất bại (PLC offline hoặc rate-limited)
                                    self.qr_status_ready.emit(
                                        f"QR: Không ghi được xuống PLC [{qr_val}]", "#EF4444"
                                    )
                                force_preview = True
                        else:
                            # PC-Master YOLO Vision Mode
                            if self._manual_classify_trigger.is_set():
                                with self._sort_start_lock:
                                    self._manual_classify_trigger.clear()
                                    self._classification_running.set()
                                try:
                                    self._handle_manual_classification(frame)
                                finally:
                                    self._classification_running.clear()
                                force_preview = True

                        now = time.monotonic()
                        if (
                            force_preview
                            or (now - last_preview_time) >= preview_interval
                        ):
                            last_preview_time = now

                            # Draw ROI overlay if in Vision Mode
                            display_frame = frame.copy()
                            if self.cfg.app.sorting_mode != "qr":
                                roi_x = self.cfg.yolo.roi_x
                                roi_y = self.cfg.yolo.roi_y
                                roi_w = self.cfg.yolo.roi_width
                                roi_h = self.cfg.yolo.roi_height
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

                            if not self._frame_pending:
                                h, w, ch = display_frame.shape
                                bytes_per_line = ch * w
                                qimg = QImage(
                                    display_frame.data,
                                    w,
                                    h,
                                    bytes_per_line,
                                    QImage.Format.Format_BGR888,
                                ).copy()  # Deep copy is essential because display_frame is temporary
                                self._frame_pending = True
                                self.frame_ready.emit(qimg)

                    except Exception as frame_exc:
                        log.error(
                            "Exception during frame processing: %s",
                            frame_exc,
                            exc_info=True,
                        )
                        self.log_tx(f"Camera processing failed: {frame_exc}")

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

        geometry = {}
        if self.cfg.yolo.coordinate_origin == "frame":
            geometry = {"pixel_offset": (cx, cy), "pixel_origin": (fw / 2.0, fh / 2.0)}
        result = self.detector.annotate_frame(roi_crop, **geometry)

        if not result.has_defect:
            log.info("Manual Capture: no object detected above threshold – skipped.")
            self.log_tx("Capture: no object detected – sort skipped.")
            return

        quality = self.detector.classify_quality(result.class_id, result.class_name)
        if quality is None:
            self.log_tx(f"Capture skipped: unmapped model class {result.class_name!r}.")
            return
        sort_result = SortResult(quality)
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
        if self._closing or self._stop_event.is_set():
            return False
        try:
            self.validate_vision_configuration()
        except ValueError as error:
            self.show_error_dialog.emit(str(error))
            return False
        remaining = self._sort_cooldown_end - time.monotonic()
        if remaining > 0:
            log.info("Sort suppressed – cooldown active for another %.1fs.", remaining)
            self.log_tx(f"Sort suppressed (cooldown {remaining:.1f}s).")
            return False

        with self._sort_start_lock:
            # Recheck under the lock: Settings or closeEvent may have run
            # while inference was finishing.
            if self._closing or self._stop_event.is_set() or self.cfg.app.sorting_mode != "vision":
                return False
            if not self.sorter.is_idle():
                self.log_tx("Sort suppressed: controller is not IDLE; clear the fault first.")
                return False
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
        if not status:
            self.log_tx("Sort ABORTED: PLC status unavailable.")
            return
        if status.get("error_flag", False):
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
            self._sort_cooldown_duration = self.cfg.app.move_cooldown
            self._sort_cooldown_end = time.monotonic() + self._sort_cooldown_duration
            log.info("Post-sort cooldown activated (%.1fs).", self._sort_cooldown_duration)

    def clear_all_errors(self) -> None:
        """Acknowledge a PLC fault explicitly; reset local state only when healthy."""
        with self._sort_start_lock:
            if self._sort_thread is not None and self._sort_thread.is_alive():
                self.log_tx("Cannot clear errors while a sort cycle is running.")
                return
            status = self.plc.read_status()
            if not status:
                self.log_tx("Cannot reset: PLC status unavailable.")
                return
            if status.get("error_flag", False):
                if self.plc.acknowledge_error():
                    self.log_tx("PLC error acknowledgement sent; wait for the fault to clear, then reset again.")
                else:
                    self.log_tx("PLC fault still active. Acknowledge it at the PLC or configure its error_ack bit.")
                return
            if not status.get("motion_done", False):
                self.log_tx("Cannot reset: PLC has not confirmed motion stopped/completed.")
                return
            if self.plc.resume_commands() is False:
                self.log_tx("Cannot reset: failed to release software STOP.")
                return
            self.sorter.clear_error()
            self.log_tx("PLC status healthy; local error cleared and commands enabled.")

    # ------------------------------------------------------------------
    # Main thread slots
    # ------------------------------------------------------------------

    def _on_frame_ready(self, qimg: QImage) -> None:
        """Deliver new frame to the currently active page."""
        self._frame_pending = False
        current_page = self.stacked_widget.currentWidget()
        if hasattr(current_page, "update_video"):
            current_page.update_video(qimg)

    def log_tx(self, message: str) -> None:
        """Emit telemetry message to be appended on the main thread."""
        self.tx_logged.emit(message)

    def _on_tx_logged(self, message: str) -> None:
        """Append a timestamped message to the transmission telemetry log."""
        timestamp = time.strftime("%H:%M:%S")
        cursor = self.tx_textbox.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        if not self.tx_textbox.document().isEmpty():
            cursor.insertBlock()
        cursor.insertText(f"[{timestamp}] {message}")
        self.tx_textbox.setTextCursor(cursor)
        self.tx_textbox.ensureCursorVisible()

    # ------------------------------------------------------------------
    # Background thread: PLC cyclic poll with exponential backoff
    # ------------------------------------------------------------------

    def _cyclic_update(self) -> None:
        """Poll PLC and emit status data via Qt Signal."""
        retry_delay: float = self.cfg.app.plc_poll_interval
        max_retry_delay: float = 30.0
        consecutive_failures: int = 0

        while not self._stop_event.is_set():
            interval = self.cfg.app.plc_poll_interval
            data: dict[str, Any] = {}
            try:
                if not self.plc.is_connected():
                    self.plc.connect()
                if not self._stop_event.is_set() and self.plc.is_connected():
                    data = self.plc.read_status()
            except Exception:
                log.exception("PLC polling failed; reconnect will be retried.")
            if data:
                self.plc_data_ready.emit({**data, "connected": True})
                retry_delay = interval
                consecutive_failures = 0
            else:
                # Explicitly invalidate old feedback on every failed read. The
                # GUI must not show CONNECTED/READY based on a stale snapshot.
                self.plc_data_ready.emit({"connected": False})
                consecutive_failures += 1
                if consecutive_failures <= 3 or consecutive_failures % 10 == 0:
                    log.warning(
                        "PLC offline – attempting reconnection "
                        "(attempt %d, backoff %.1fs)…",
                        consecutive_failures,
                        retry_delay,
                    )
                retry_delay = min(retry_delay * 2, max_retry_delay)

            self._stop_event.wait(retry_delay)

        log.info("PLC poll thread exited cleanly.")

    def _dispatch_plc_data(self, data: dict[str, Any]) -> None:
        """Forward PLC data to every registered page (runs on main thread)."""
        if self._closing:
            return
        camera_active = bool(self.detector.is_camera_active()) if hasattr(self.detector, "is_camera_active") else False
        data = {**data, "camera_active": camera_active}
        self._latest_plc_data = data
        for frame in self.frames.values():
            try:
                frame.update_gui_data(data)
            except Exception as exc:
                log.debug("update_gui_data error on %s: %s", type(frame).__name__, exc)

        plc_connected = data.get("connected", False)
        system_ok = bool(plc_connected and not data.get("error_flag", False)
                         and not data.get("stop_robot", False)
                         and self.sorter.state.name != "ERROR" and not self._vision_error)
        self.footer.update_status(plc_connected, camera_active, system_ok)
        self.refresh_sorting_controls()

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
        with self._sort_start_lock:
            self._closing = True
            self._stop_event.set()
            self._manual_classify_trigger.clear()
            if hasattr(self, "_camera_timer"):
                self._camera_timer.stop()
            try:
                if self.sorter.request_stop() is False:
                    log.error("Shutdown STOP could not be written; physical stop is unconfirmed.")
            except Exception:
                log.exception("Shutdown STOP failed; physical stop is unconfirmed.")

        # Stop producers before disconnecting so they cannot reconnect or issue
        # another destination after STOP. Cancellation wakes every sort delay.
        for thread in (self._sort_thread, self._plc_thread, self._ai_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=_THREAD_JOIN_TIMEOUT)
                if thread.is_alive():
                    log.error("Thread %s did not stop before shutdown timeout.", thread.name)
        self.detector.stop()
        self.plc.disconnect()

        log.info("Shutdown cleanup finished. Window closing.")
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    """Launch with a visible configuration/startup error in windowed builds."""
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(GLOBAL_QSS)
    try:
        config: RobotConfig = load_config()
        main_window = RobotApp(cfg=config)
    except Exception as error:
        log.exception("Application startup failed.")
        QMessageBox.critical(None, "Startup Error", f"Cannot start Robot Controller.\n\n{error}\n\nCheck config.yaml and the application log.")
        return 1
    main_window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
