"""
src/ai/qr_reader.py - QR code decoding and debouncing module.
"""

from __future__ import annotations

import cv2
import numpy as np


class QRReader:
    """
    Decodes QR codes from camera frames with debounce logic to ensure stability.
    """

    def __init__(self, debounce_frames: int = 3, clear_frames: int = 15) -> None:
        self.debounce_frames = debounce_frames
        self.clear_frames = clear_frames
        self._detector = cv2.QRCodeDetector()
        self._last_raw_value: str | None = None
        self._consecutive_count: int = 0
        self._none_count: int = 0

    def decode_qr(self, frame: np.ndarray) -> str | None:
        """
        Detect and decode a QR code in the frame.
        Applies debounce logic: the same value must be read N consecutive times.
        
        Returns:
            The decoded string if stable, else None.
        """
        value, points, _ = self._detector.detectAndDecode(frame)
        
        # Optionally draw the bounding box for debug/UI
        if points is not None and len(points) > 0 and value:
            pts = np.int32(points).reshape(-1, 2)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.putText(frame, value, (pts[0][0], pts[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if value:
            self._none_count = 0
            if value == self._last_raw_value:
                self._consecutive_count += 1
            else:
                self._last_raw_value = value
                self._consecutive_count = 1
                
            if self._consecutive_count >= self.debounce_frames:
                return value
        else:
            self._none_count += 1
            # Reset debounce if no QR found
            self._last_raw_value = None
            self._consecutive_count = 0
            if self._none_count >= self.clear_frames:
                self._none_count = 0
                return "CLEAR"
            
        return None
