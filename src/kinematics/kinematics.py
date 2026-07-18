"""
src/kinematics/kinematics.py – 4-DOF articulated robot arm kinematics.

DH Table (mm)
─────────────────────────────────────────────────────
  i   θ_i      d_i (mm)    a_i (mm)    α_i
  1   θ₁       100         0           +90°
  2   θ₂       0           106         0
  3   θ₃       0           103         0
  4   θ₄       0           85          0
─────────────────────────────────────────────────────

Forward Kinematics
    (θ₁°, θ₂°, θ₃°, θ₄°)  →  (X mm, Y mm, Z mm)
    x = (106·cos θ₂ + 103·cos(θ₂+θ₃) + 85·cos(θ₂+θ₃+θ₄)) · cos θ₁
    y = (106·cos θ₂ + 103·cos(θ₂+θ₃) + 85·cos(θ₂+θ₃+θ₄)) · sin θ₁
    z = 100 + 106·sin θ₂ + 103·sin(θ₂+θ₃) + 85·sin(θ₂+θ₃+θ₄)

Inverse Kinematics  (geometric, with end-effector pitch φ)
    (X mm, Y mm, Z mm, φ°)  →  (θ₁°, θ₂°, θ₃°, θ₄°)
    θ₁ = atan2(Y, X)
    Wrist position (removing link-4 contribution):
        r_w = r_total − 85·cos φ       (horizontal projection)
        z_w = Z − 100 − 85·sin φ       (vertical from base)
    Then solve 2-link planar IK (a₂=106, a₃=103) for θ₂, θ₃,
    and θ₄ = φ − θ₂ − θ₃.

Usage
-----
    from src.kinematics import forward_kinematics, inverse_kinematics

    x, y, z = forward_kinematics(j1=0.0, j2=0.0, j3=0.0, j4=0.0)
    j1, j2, j3, j4 = inverse_kinematics(200.0, 0.0, 150.0, phi=0.0)
"""

from __future__ import annotations

import math


# ── DH link parameters (mm) ─────────────────────────────────────────────────
D1 = 100.0   # base height (d₁)
A2 = 106.0   # link-2 length (a₂)
A3 = 103.0   # link-3 length (a₃)
A4 = 85.0    # link-4 / end-effector length (a₄)


class InverseKinematicsError(Exception):
    """Raised when inverse kinematics computation fails."""


class WorkspaceError(InverseKinematicsError):
    """Raised when the target point is outside the robot's reachable workspace."""


def reachable(
    x: float,
    y: float,
    z: float,
    phi: float = 0.0,
) -> bool:
    """
    Check whether a target point (x, y, z) lies within the reachable workspace
    for a given end-effector pitch angle *phi* (degrees).

    Parameters
    ----------
    x, y, z:
        Target end-effector position in millimetres.
    phi:
        End-effector pitch angle in degrees (φ = θ₂ + θ₃ + θ₄).

    Returns
    -------
    bool
        ``True`` if the point is reachable.
    """
    try:
        _inverse_kinematics_impl(x, y, z, phi)
        return True
    except InverseKinematicsError:
        return False


def forward_kinematics(
    j1: float = 0.0,
    j2: float = 0.0,
    j3: float = 0.0,
    j4: float = 0.0,
    **_kwargs,
) -> tuple[float, float, float]:
    """
    4-DOF articulated-arm forward kinematics.

    Computes the end-effector (X, Y, Z) position from four revolute joint
    angles using the DH parameters:
        d₁ = 100 mm,  a₂ = 106 mm,  a₃ = 103 mm,  a₄ = 85 mm

    Parameters
    ----------
    j1:
        Base rotation angle θ₁ in degrees.
    j2:
        Shoulder angle θ₂ in degrees.
    j3:
        Elbow angle θ₃ in degrees.
    j4:
        Wrist angle θ₄ in degrees.

    Returns
    -------
    tuple[float, float, float]
        End-effector position (X, Y, Z) in millimetres.

    Examples
    --------
    >>> x, y, z = forward_kinematics(0.0, 0.0, 0.0, 0.0)
    >>> round(x, 2), round(y, 2), round(z, 2)
    (294.0, 0.0, 100.0)
    """
    t1 = math.radians(j1)
    t2 = math.radians(j2)
    t23 = math.radians(j2 + j3)
    t234 = math.radians(j2 + j3 + j4)

    # Horizontal projection (radial distance from Z-axis)
    r = A2 * math.cos(t2) + A3 * math.cos(t23) + A4 * math.cos(t234)

    x = r * math.cos(t1)
    y = r * math.sin(t1)
    z = D1 + A2 * math.sin(t2) + A3 * math.sin(t23) + A4 * math.sin(t234)

    return x, y, z


def inverse_kinematics(
    x: float,
    y: float,
    z: float = 100.0,
    phi: float = 0.0,
    **_kwargs,
) -> tuple[float, float, float, float]:
    """
    4-DOF articulated-arm inverse kinematics (geometric method).

    Computes joint angles from a Cartesian target and desired end-effector
    pitch angle *phi* (φ = θ₂ + θ₃ + θ₄).

    Parameters
    ----------
    x, y:
        Target end-effector position in millimetres (horizontal plane).
    z:
        Target Z height in millimetres.
    phi:
        Desired end-effector pitch angle in degrees.
        φ = 0° means the end-effector points horizontally.

    Returns
    -------
    tuple[float, float, float, float]
        Joint angles (θ₁°, θ₂°, θ₃°, θ₄°).

    Raises
    ------
    WorkspaceError
        When the target is unreachable or at the rotation centre.

    Examples
    --------
    >>> j1, j2, j3, j4 = inverse_kinematics(294.0, 0.0, 100.0, phi=0.0)
    >>> round(j1, 1), round(j2, 1), round(j3, 1), round(j4, 1)
    (0.0, 0.0, 0.0, 0.0)
    """
    return _inverse_kinematics_impl(x, y, z, phi)


def _inverse_kinematics_impl(
    x: float,
    y: float,
    z: float,
    phi: float,
) -> tuple[float, float, float, float]:
    """Internal IK implementation (shared by ``inverse_kinematics`` and ``reachable``)."""

    # ── θ₁: base rotation ────────────────────────────────────────────────
    r_total = math.hypot(x, y)

    if r_total < 1e-6 and abs(z - D1) < 1e-6:
        raise WorkspaceError(
            f"Target ({x:.2f}, {y:.2f}, {z:.2f}) is at the rotation centre – "
            "θ₁ angle is ambiguous."
        )

    theta1 = math.atan2(y, x)

    # ── Wrist position (remove link-4 contribution) ──────────────────────
    phi_rad = math.radians(phi)
    r_w = r_total - A4 * math.cos(phi_rad)
    z_w = z - D1 - A4 * math.sin(phi_rad)

    # ── 2-link planar IK (a₂, a₃) for θ₂, θ₃ ────────────────────────────
    dist_sq = r_w * r_w + z_w * z_w
    dist = math.sqrt(dist_sq)

    # Check reachability of the wrist point
    if dist > (A2 + A3) or dist < abs(A2 - A3):
        raise WorkspaceError(
            f"Target ({x:.2f}, {y:.2f}, {z:.2f}) with φ={phi:.1f}° is "
            f"outside the reachable workspace (wrist distance={dist:.2f} mm, "
            f"arm range=[{abs(A2 - A3):.1f}, {A2 + A3:.1f}] mm)."
        )

    # cos θ₃ by the law of cosines
    cos_theta3 = (dist_sq - A2 * A2 - A3 * A3) / (2.0 * A2 * A3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # clamp for numerical safety

    # Elbow-up solution (negative θ₃)
    theta3 = -math.acos(cos_theta3)

    # θ₂ via atan2 formulation
    s3 = math.sin(theta3)
    c3 = math.cos(theta3)
    alpha = math.atan2(z_w, r_w)
    beta = math.atan2(A3 * s3, A2 + A3 * c3)
    theta2 = alpha - beta

    # θ₄ from pitch constraint
    theta4 = phi_rad - theta2 - theta3

    return (
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3),
        math.degrees(theta4),
    )
