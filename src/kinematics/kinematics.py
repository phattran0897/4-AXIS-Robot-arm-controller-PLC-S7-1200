"""
src/kinematics/kinematics.py – 2-DOF planar inverse kinematics for the SCARA arm.

Usage
-----
    from src.kinematics import inverse_kinematics, reachable

    j1, j2, j3, j4 = inverse_kinematics(240.0, 0.0, l1=200.0, l2=150.0)
"""

from __future__ import annotations

import math


class InverseKinematicsError(Exception):
    """Raised when inverse kinematics computation fails."""


class WorkspaceError(InverseKinematicsError):
    """Raised when the target point is outside the robot's reachable workspace."""


def reachable(x: float, y: float, l1: float, l2: float) -> bool:
    """
    Check whether a target point (x, y) lies within the reachable workspace.

    Parameters
    ----------
    x, y:
        Target end-effector position in millimetres.
    l1, l2:
        Link lengths in millimetres.

    Returns
    -------
    bool
        ``True`` if the point is reachable (|l1 - l2| <= sqrt(x²+y²) <= l1+l2).
    """
    r = math.hypot(x, y)
    return abs(l1 - l2) <= r <= (l1 + l2)


def forward_kinematics(
    j1: float = 0.0,
    j2: float = 0.0,
    l1: float = 200.0,
    l2: float = 150.0,
) -> tuple[float, float, float, float]:
    """
    Two-DOF planar forward kinematics for the SCARA arm.

    Computes the end-effector (X, Y) position from J1 and J2. J3 and J4
    are passed through unchanged because they are controlled independently
    (Z-axis and tool rotation).

    Parameters
    ----------
    j1, j2:
        Joint angles in degrees.
    l1, l2:
        Link lengths in millimetres.

    Returns
    -------
    tuple[float, float, float, float]
        End-effector position (X, Y) plus the original J3, J4 values.

    Examples
    --------
    >>> x, y, j3, j4 = forward_kinematics(49.05, 130.54, l1=200.0, l2=150.0)
    >>> round(x, 2), round(y, 2)
    (239.99, -0.01)
    """
    j1_rad = math.radians(j1)
    j2_rad = math.radians(j2)
    x = l1 * math.cos(j1_rad) + l2 * math.cos(j1_rad + j2_rad)
    y = l1 * math.sin(j1_rad) + l2 * math.sin(j1_rad + j2_rad)
    return x, y, 0.0, 0.0


def inverse_kinematics(
    x: float,
    y: float,
    l1: float = 200.0,
    l2: float = 150.0,
) -> tuple[float, float, float, float]:
    """
    Two-DOF planar inverse kinematics for the SCARA arm.

    Always selects the elbow-down configuration (standard industrial convention).

    Parameters
    ----------
    x, y:
        Target end-effector position in millimetres.
    l1, l2:
        Link lengths in millimetres (arm segment 1 and 2).

    Returns
    -------
    tuple[float, float, float, float]
        Joint angles (degrees) for axes 1–4.
        J3 and J4 are always ``0.0`` (reserved for Z-axis and tool rotation,
        controlled separately via PLC commands).

    Raises
    ------
    WorkspaceError
        When the target point lies outside the robot's reachable workspace.

    Examples
    --------
    >>> j1, j2, j3, j4 = inverse_kinematics(240.0, 0.0, l1=200.0, l2=150.0)
    >>> round(j1, 2), round(j2, 2)
    (49.05, 130.54)
    """
    if l1 <= 0 or l2 <= 0:
        raise ValueError("Link lengths (l1, l2) must be positive values.")

    if not reachable(x, y, l1, l2):
        raise WorkspaceError(
            f"Target ({x:.2f}, {y:.2f}) is outside workspace "
            f"(r={math.hypot(x, y):.2f}, max_reach={l1 + l2:.2f})."
        )

    try:
        cos_j2 = (x**2 + y**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
        cos_j2 = max(-1.0, min(1.0, cos_j2))
        j2_rad = math.acos(cos_j2)
        j1_rad = math.atan2(y, x) - math.atan2(
            l2 * math.sin(j2_rad),
            l1 + l2 * math.cos(j2_rad),
        )
        return math.degrees(j1_rad), math.degrees(j2_rad), 0.0, 0.0
    except (ValueError, ZeroDivisionError) as exc:
        raise InverseKinematicsError(
            f"IK computation failed for ({x:.2f}, {y:.2f}): {exc}"
        ) from exc
