"""
src/ai/stability_tracker.py – Vision-target stability tracker.

Provides :class:`StabilityTracker`, a small state machine that determines
when a detected object position has been stable long enough to dispatch
a pick command.  Separated from the GUI so it can be unit-tested without
Tkinter.

Usage
-----
    tracker = StabilityTracker(threshold_mm=5.0, required_frames=10)
    for x, y in detections:
        if tracker.update(x, y):
            print("Locked!", tracker.locked_position)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class StabilityTracker:
    """
    Track whether a detected position has been stable across consecutive
    frames.

    Parameters
    ----------
    threshold_mm:
        Maximum jump in mm between consecutive frames before the lock
        counter resets.
    required_frames:
        Number of consecutive stable frames needed to achieve a lock.
    """

    threshold_mm: float = 5.0
    required_frames: int = 10

    # Internal state (not part of the public API)
    _last_x: float = field(default=0.0, repr=False)
    _last_y: float = field(default=0.0, repr=False)
    _stable_count: int = field(default=0, repr=False)
    _locked: bool = field(default=False, repr=False)
    _locked_x: float = field(default=0.0, repr=False)
    _locked_y: float = field(default=0.0, repr=False)
    _has_previous: bool = field(default=False, repr=False)

    def update(self, x: float, y: float) -> bool:
        """
        Feed a new detection position.

        Parameters
        ----------
        x, y:
            Detected object position in millimetres.

        Returns
        -------
        bool
            ``True`` if the position is now locked (stable for
            ``required_frames`` consecutive frames).
        """
        if self._locked:
            # Already locked — check if object moved significantly
            dist = math.hypot(x - self._locked_x, y - self._locked_y)
            if dist > self.threshold_mm:
                self.reset()
                self._last_x, self._last_y = x, y
                self._has_previous = True
                return False
            return True

        if not self._has_previous:
            self._last_x, self._last_y = x, y
            self._has_previous = True
            self._stable_count = 1
            return False

        dist = math.hypot(x - self._last_x, y - self._last_y)
        if dist <= self.threshold_mm:
            self._stable_count += 1
        else:
            self._stable_count = 1

        self._last_x, self._last_y = x, y

        if self._stable_count >= self.required_frames:
            self._locked = True
            self._locked_x = x
            self._locked_y = y
            return True
        return False

    @property
    def is_locked(self) -> bool:
        """Whether the tracker is currently in the locked state."""
        return self._locked

    @property
    def locked_position(self) -> tuple[float, float] | None:
        """Return ``(x, y)`` if locked, else ``None``."""
        if self._locked:
            return (self._locked_x, self._locked_y)
        return None

    @property
    def stable_count(self) -> int:
        """Number of consecutive stable frames observed."""
        return self._stable_count

    @property
    def progress(self) -> float:
        """Lock progress from 0.0 to 1.0."""
        return min(1.0, self._stable_count / max(1, self.required_frames))

    def reset(self) -> None:
        """Reset all tracking state."""
        self._last_x = 0.0
        self._last_y = 0.0
        self._stable_count = 0
        self._locked = False
        self._locked_x = 0.0
        self._locked_y = 0.0
        self._has_previous = False
