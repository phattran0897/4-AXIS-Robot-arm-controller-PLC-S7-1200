"""
src/ai/yolo_detector.py – YOLO-based defect detector with camera management.

Provides :class:`YOLODetector`, a production-hardened wrapper around
Ultralytics YOLO for real-time defect detection on live camera frames.
"""

from __future__ import annotations

import logging
import os
import queue
import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import cv2
import numpy as np
from PIL import Image
from ultralytics import YOLO

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Drawing constants (BGR colour tuples for OpenCV)
# ---------------------------------------------------------------------------
_CROSSHAIR_COLOR: tuple[int, int, int] = (255, 255, 255)
_CROSSHAIR_THICKNESS: int = 1
_CROSSHAIR_HALF_LEN: int = 12

_BBOX_COLOR: tuple[int, int, int] = (0, 0, 255)
_BBOX_THICKNESS: int = 2

_CENTROID_COLOR: tuple[int, int, int] = (0, 255, 0)
_CENTROID_RADIUS: int = 5

_TEXT_COLOR: tuple[int, int, int] = (0, 255, 0)
_TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX
_TEXT_SCALE: float = 0.6
_TEXT_THICKNESS: int = 2
_TEXT_Y_OFFSET: int = 20

# Default inference resolution (width, height) – smaller = faster YOLO inference
_DEFAULT_INFERENCE_WIDTH: int = 640
_DEFAULT_INFERENCE_HEIGHT: int = 480


@dataclass(slots=True)
class DetectionResult:
    """Immutable result of a single processed frame."""

    has_defect: bool = False
    robot_x: float = 0.0
    robot_y: float = 0.0
    annotated_frame: np.ndarray | None = None

    # Store raw frame for lazy PIL conversion
    _bgr_frame: np.ndarray | None = field(default=None, repr=False)

    def to_pil(self, display_w: int, display_h: int) -> Image.Image | None:
        """Convert the annotated frame to a resized PIL Image for GUI display."""
        if self._bgr_frame is None:
            return None
        frame = self._bgr_frame
        if frame.shape[1] != display_w or frame.shape[0] != display_h:
            frame = cv2.resize(frame, (display_w, display_h))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)


class YOLODetector:
    """
    Real-time defect detection and robot-coordinate conversion.

    The detector owns a single :class:`cv2.VideoCapture` and exposes
    ``read_frame()`` (locked, fast) and ``annotate_frame()`` (unlocked, slow)
    separately so callers can release the camera lock during heavy inference.

    Parameters
    ----------
    model_path:
        Filesystem path to a YOLO ``.pt`` weight file.
    thresh:
        Minimum confidence score (0–1) for a detection to be accepted.
    px2mm:
        Pixel-to-millimetre conversion factor (calibrated per installation).
    home_x:
        Robot home X coordinate in millimetres (image-centre reference).
    home_y:
        Robot home Y coordinate in millimetres (image-centre reference).
    inference_width:
        Frame width used for YOLO inference (default 640). Smaller values
        speed up inference at the cost of accuracy.
    inference_height:
        Frame height used for YOLO inference (default 480).
    camera_read_timeout:
        Maximum seconds to wait for a frame from the camera (default 2.0).
        If exceeded, the read is treated as failed.
    """

    def __init__(
        self,
        model_path: str,
        thresh: float = 0.50,
        px2mm: float = 0.50,
        home_x: float = 200.0,
        home_y: float = 0.0,
        inference_width: int = _DEFAULT_INFERENCE_WIDTH,
        inference_height: int = _DEFAULT_INFERENCE_HEIGHT,
        camera_read_timeout: float = 2.0,
    ) -> None:
        resolved = self._resolve_model_path(model_path)
        self.model: YOLO = YOLO(resolved, task="detect")
        self.thresh: float = float(thresh)
        self.px2mm: float = float(px2mm)
        self.home_x: float = float(home_x)
        self.home_y: float = float(home_y)
        self.inference_width: int = inference_width
        self.inference_height: int = inference_height
        self.camera_read_timeout: float = camera_read_timeout

        self._cap: cv2.VideoCapture | None = None
        self._current_idx: int = 0

        # Camera read timeout infrastructure
        self._read_queue: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=1)
        self._read_thread: threading.Thread | None = None
        self._read_stop = threading.Event()
        self._dropped_frames: int = 0
        self._last_drop_warning: float = 0.0

        log.info(
            "YOLODetector initialised – model=%s thresh=%.2f px2mm=%.3f "
            "inference=%dx%d timeout=%.1fs",
            resolved,
            thresh,
            px2mm,
            inference_width,
            inference_height,
            camera_read_timeout,
        )

    def _resolve_model_path(self, model_path: str) -> str:
        """Resolve model path relative to project root if not absolute."""
        if os.path.isabs(model_path) and os.path.isfile(model_path):
            return model_path
        resolved = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", model_path)
        )
        if not os.path.isfile(resolved):
            raise FileNotFoundError(
                f"YOLO model weight file not found: {model_path}\n"
                f"Resolved to: {resolved}"
            )
        return resolved

    # ------------------------------------------------------------------
    # Camera lifecycle
    # ------------------------------------------------------------------

    def start_camera(self, usb_idx: int = 0) -> bool:
        """
        Open a USB camera device and start the background read thread.

        Parameters
        ----------
        usb_idx:
            OpenCV device index (0 = first USB camera).

        Returns
        -------
        bool
            ``True`` if the camera opened successfully.
        """
        self._stop_read_thread()
        self._current_idx = usb_idx
        self._cap = cv2.VideoCapture(usb_idx)
        if not self._cap.isOpened():
            log.error("Failed to open camera at index %d.", usb_idx)
            return False

        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._read_stop.clear()
        self._read_thread = threading.Thread(
            target=self._camera_read_loop,
            name="CameraReadThread",
            daemon=True,
        )
        self._read_thread.start()
        log.info("Camera %d opened successfully.", usb_idx)
        return True

    def switch_camera(self, new_idx: int) -> bool:
        """
        Release the current camera and open a new one.

        Parameters
        ----------
        new_idx:
            OpenCV device index for the replacement camera.

        Returns
        -------
        bool
            ``True`` if the new camera opened successfully.
        """
        self._stop_read_thread()
        log.info("Switching camera → index %d", new_idx)
        return self.start_camera(new_idx)

    def available_cameras(self, max_index: int = 10) -> list[int]:
        """
        Probe indices 0..max_index and return those that open successfully.

        Parameters
        ----------
        max_index:
            Highest index to probe (default 10).

        Returns
        -------
        list[int]
            Indices of cameras that responded to :meth:`isOpened`.
        """
        available: list[int] = []
        for idx in range(max_index + 1):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                available.append(idx)
                cap.release()
        return available

    def stop(self) -> None:
        """Stop the read thread and release the camera device."""
        self._stop_read_thread()
        self._release_capture()
        log.info("YOLODetector stopped.")

    def is_camera_active(self) -> bool:
        """Check if camera is currently active and readable."""
        return self._cap is not None and self._cap.isOpened()

    # ------------------------------------------------------------------
    # Frame reading (called under lock)
    # ------------------------------------------------------------------

    def read_frame(self) -> np.ndarray | None:
        """
        Retrieve one frame from the camera using the background read thread.

        Blocks up to ``camera_read_timeout`` seconds. Returns ``None`` if
        no frame is available within that window.

        This method is designed to be called while holding ``_camera_lock``;
        it is fast (no heavy computation) and will not block the caller
        beyond the configured timeout.

        Returns
        -------
        np.ndarray | None
            BGR frame, or ``None`` on timeout / camera unavailable.
        """
        if self._cap is None or not self._cap.isOpened():
            return None
        try:
            return self._read_queue.get(timeout=self.camera_read_timeout)
        except queue.Empty:
            log.warning(
                "Camera read timeout after %.1fs (index %d).",
                self.camera_read_timeout,
                self._current_idx,
            )
            return None

    # ------------------------------------------------------------------
    # Frame processing (called without lock)
    # ------------------------------------------------------------------

    def annotate_frame(
        self,
        frame: np.ndarray,
        inference_w: int | None = None,
        inference_h: int | None = None,
    ) -> DetectionResult:
        """
        Run YOLO inference on *frame* and return a detection result.

        This method does NOT acquire any locks — it should be called outside
        the camera lock so inference does not block camera access.

        Parameters
        ----------
        frame:
            BGR frame captured from :meth:`read_frame`.
        inference_w, inference_h:
            Inference resolution override. When ``None`` the instance
            defaults (``inference_width`` / ``inference_height``) are used.

        Returns
        -------
        DetectionResult
            Immutable result containing ``has_defect``, ``robot_x``, ``robot_y``,
            and the annotated BGR frame.
        """
        inference_w = inference_w or self.inference_width
        inference_h = inference_h or self.inference_height

        # Resize for inference (performance optimisation – YOLO runs faster
        # on a smaller image than the full camera resolution)
        inference_frame = cv2.resize(
            frame, (inference_w, inference_h), interpolation=cv2.INTER_LINEAR
        )

        h, w = inference_frame.shape[:2]
        frame_cx, frame_cy = w / 2.0, h / 2.0

        # Draw centre crosshair on the inference-resolution frame
        inference_frame = self._draw_crosshair(inference_frame)

        # Run inference
        has_defect, robot_x, robot_y = self._run_inference(
            inference_frame, frame_cx, frame_cy
        )

        return DetectionResult(
            has_defect=has_defect,
            robot_x=robot_x,
            robot_y=robot_y,
            _bgr_frame=inference_frame,
        )

    # ------------------------------------------------------------------
    # Convenience: full pipeline (locked read → unlocked inference)
    # Kept for backwards compatibility. Prefer read_frame() + annotate_frame().
    # ------------------------------------------------------------------

    def process_frame(
        self,
    ) -> tuple[bool, float, float, Image.Image | None]:
        """
        Capture, infer, and return results + PIL image.

        .. deprecated::
            Prefer calling :meth:`read_frame` (under lock) followed by
            :meth:`annotate_frame` (outside lock) for better concurrency.

        Returns
        -------
        has_defect, robot_x, robot_y, pil_image
        """
        frame = self.read_frame()
        if frame is None:
            return False, self.home_x, self.home_y, None

        result = self.annotate_frame(frame)
        pil = result.to_pil(self.inference_width, self.inference_height)
        return result.has_defect, result.robot_x, result.robot_y, pil

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _camera_read_loop(self) -> None:
        """Background thread: continuously read frames into the queue."""
        import time
        while not self._read_stop.is_set():
            if self._cap is None:
                break
            ret, frame = self._cap.read()
            if not ret or frame is None:
                continue
            # Non-blocking put; drop the old frame if inference is slow
            try:
                self._read_queue.put_nowait(frame)
                self._dropped_frames = 0
            except queue.Full:
                self._dropped_frames += 1
                # Log warning every 5 seconds if frames are being dropped
                current_time = time.time()
                if self._dropped_frames > 10 and (current_time - self._last_drop_warning) > 5.0:
                    log.warning(
                        "Frame drops detected (%d consecutive). Consider reducing inference load.",
                        self._dropped_frames
                    )
                    self._last_drop_warning = current_time

    def _stop_read_thread(self) -> None:
        """Signal and join the camera read thread."""
        if self._read_thread is not None:
            self._read_stop.set()
            self._read_thread.join(timeout=3.0)
            self._read_thread = None

    def _release_capture(self) -> None:
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
        self._cap = None

    def _draw_crosshair(self, frame: np.ndarray) -> np.ndarray:
        """Draw a centre crosshair onto *frame* in-place."""
        h, w = frame.shape[:2]
        cx, cy = int(w / 2), int(h / 2)
        half = _CROSSHAIR_HALF_LEN
        cv2.line(frame, (cx - half, cy), (cx + half, cy), _CROSSHAIR_COLOR, _CROSSHAIR_THICKNESS)
        cv2.line(frame, (cx, cy - half), (cx, cy + half), _CROSSHAIR_COLOR, _CROSSHAIR_THICKNESS)
        return frame

    def _run_inference(
        self, frame: np.ndarray, frame_cx: float, frame_cy: float
    ) -> tuple[bool, float, float]:
        """
        Execute YOLO inference on *frame* and return detection results.

        Only the single highest-confidence detection above *thresh* is
        used for coordinate computation; the rest are still drawn.

        Parameters
        ----------
        frame:
            BGR frame at inference resolution.
        frame_cx, frame_cy:
            Centre coordinates of the inference-resolution frame.

        Returns
        -------
        has_defect, robot_x, robot_y
        """
        results = self.model(frame, verbose=False)
        boxes = results[0].boxes

        has_defect = False
        robot_x = self.home_x
        robot_y = self.home_y
        detection_count = 0

        for i in range(len(boxes)):
            xyxy: np.ndarray = boxes[i].xyxy.cpu().numpy().squeeze()
            if xyxy.ndim == 0 or xyxy.size != 4:
                continue

            xmin, ymin, xmax, ymax = xyxy.astype(int)
            conf: float = float(boxes[i].conf.item())

            if conf <= self.thresh:
                continue

            defect_cx = (xmin + xmax) / 2.0
            defect_cy = (ymin + ymax) / 2.0

            if not has_defect:
                robot_x = self.home_x + (defect_cx - frame_cx) * self.px2mm
                robot_y = self.home_y + (defect_cy - frame_cy) * self.px2mm
                has_defect = True

            detection_count += 1

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), _BBOX_COLOR, _BBOX_THICKNESS)
            cv2.circle(frame, (int(defect_cx), int(defect_cy)), _CENTROID_RADIUS, _CENTROID_COLOR, -1)
            # Draw per-detection coordinates
            cv2.putText(
                frame,
                f"#{detection_count} X:{robot_x:.1f} Y:{robot_y:.1f}",
                (xmin, max(ymin, ymax + _TEXT_Y_OFFSET)),
                _TEXT_FONT,
                _TEXT_SCALE,
                _TEXT_COLOR,
                _TEXT_THICKNESS,
            )

        return has_defect, robot_x, robot_y
