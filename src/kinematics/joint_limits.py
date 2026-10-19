"""
src/kinematics/joint_limits.py – Centralized joint limit definitions and validation.

Single source of truth for joint angle range checking across Kinematics,
Manual Mode, and Automatic Sorting pipelines.
"""

from __future__ import annotations

import math
import threading
from typing import Mapping

# Module-level joint limits (degrees): (min_angle, max_angle)
# Defaults serve as placeholders; overridden at start-up via configure_joint_limits().
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "j1": (-180.0, 180.0),
    "j2": (-180.0, 250.0),
    "j3": (-200.0, 200.0),  # Allows both elbow-up (>=0) and elbow-down (<=0)
    "j4": (-180.0, 180.0),
}

_limits_lock = threading.RLock()


def configure_joint_limits(limits: Mapping[str, tuple[float, float]]) -> None:
    """Update joint limits atomically from config."""
    with _limits_lock:
        updated = {}
        for joint, pair in limits.items():
            if joint not in JOINT_LIMITS:
                raise ValueError(f"Unknown joint limit: {joint}.")
            if not isinstance(pair, (tuple, list)) or len(pair) != 2:
                raise ValueError(f"Joint {joint} limits must be a (minimum, maximum) pair.")
            low, high = float(pair[0]), float(pair[1])
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError(f"Joint {joint} limits must be finite and strictly ordered.")
            updated[joint] = (low, high)
        JOINT_LIMITS.update(updated)


def validate_joint_angles(
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    tol: float = 1e-4,
) -> None:
    """
    Validate that joint angles satisfy all configured JOINT_LIMITS.

    Raises
    ------
    WorkspaceError
        If any angle is outside its valid range [min - tol, max + tol].
    """
    from src.kinematics.kinematics import WorkspaceError

    if not math.isfinite(tol) or tol < 0:
        raise ValueError("Joint-limit tolerance must be finite and non-negative.")
    with _limits_lock:
        limits = JOINT_LIMITS.copy()
    angles = {"j1": j1, "j2": j2, "j3": j3, "j4": j4}
    for name, val in angles.items():
        if not math.isfinite(val):
            raise WorkspaceError(f"Joint {name} must be finite.")
        if name in limits:
            lo, hi = limits[name]
            if val < (lo - tol) or val > (hi + tol):
                raise WorkspaceError(
                    f"Joint limit exceeded: {name}={val:.2f}°, limit=[{lo:.1f}, {hi:.1f}]"
                )
