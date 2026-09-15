"""
src/kinematics/kinematics.py – 4-DOF articulated robot arm kinematics.

DH Table (mm)
─────────────────────────────────────────────────────
  i   α_i        a_i (mm)    d_i (mm)    θ_i (biến)
  1   −π/2       40          300         θ₁
  2   0          190         0           θ₂
  3   0          110         0           θ₃
  4   0          65          0           θ₄
─────────────────────────────────────────────────────

Forward Kinematics
    (θ₁°, θ₂°, θ₃°, θ₄°)  →  (Px mm, Py mm, Pz mm)
    Px = cos(θ₁) · (a₄·cos(θ₂+θ₃+θ₄) + a₃·cos(θ₂+θ₃) + a₂·cos(θ₂) + a₁)
    Py = sin(θ₁) · (a₄·cos(θ₂+θ₃+θ₄) + a₃·cos(θ₂+θ₃) + a₂·cos(θ₂) + a₁)
    Pz = d₁ − a₃·sin(θ₂+θ₃) − a₂·sin(θ₂) − a₄·sin(θ₂+θ₃+θ₄)

Inverse Kinematics  (geometric, with end-effector pitch φ)
    (Px mm, Py mm, Pz mm, φ°)  →  (θ₁°, θ₂°, θ₃°, θ₄°)
    θ₁ = atan2(Py, Px)
    r  = √(Px² + Py²) − a₁
    z  = Pz − d₁
    r₄ = r − a₄·cos(φ)
    z₄ = z + a₄·sin(φ)
    C₃ = (r₄² + z₄² − a₂² − a₃²) / (2·a₂·a₃)   clamped to [-1, 1]
    S₃ = √(1 − C₃²)
    θ₃ = atan2(S₃, C₃)
    α  = atan2(z₄, r₄)
    β  = atan2(a₃·sin(θ₃), a₂ + a₃·cos(θ₃))
    θ₂ = α − β
    θ₄ = φ − θ₂ − θ₃

Configuration
-------------
The module ships with the default DH parameters above.  At application
start-up :meth:`configure` is called with the values parsed from
``config.yaml`` so the YAML file is the single source of truth:

    from src.kinematics import configure
    configure(a1=cfg.kinematics.a1, d1=cfg.kinematics.d1, ...)

Usage
-----
    from src.kinematics import forward_kinematics, inverse_kinematics

    x, y, z = forward_kinematics(j1=0.0, j2=0.0, j3=0.0, j4=0.0)
    j1, j2, j3, j4 = inverse_kinematics(200.0, 0.0, 150.0, phi=0.0)
"""

from __future__ import annotations

import math
import threading


# ── DH link parameters (mm) – defaults; overridden via configure() ──────────
A1: float = 40.0  # base horizontal offset (a₁)
D1: float = 300.0  # base height (d₁)
A2: float = 190.0  # link-2 length (a₂)
A3: float = 110.0  # link-3 length (a₃)
A4: float = 65.0  # link-4 / end-effector length (a₄)

# ── Joint limits (degrees) – used by UI validation; set via configure() ─────
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "j1": (-180.0, 180.0),
    "j2": (0.0, 250.0),
    "j3": (0.0, 200.0),
    "j4": (-180.0, 180.0),
}

_cfg_lock = threading.Lock()


def configure(
    a1: float | None = None,
    d1: float | None = None,
    a2: float | None = None,
    a3: float | None = None,
    a4: float | None = None,
    limits: dict[str, tuple[float, float]] | None = None,
) -> None:
    """
    Update the module-level DH parameters and joint limits.

    Called once at start-up with values from ``config.yaml`` so the YAML
    file drives the kinematics instead of these hardcoded defaults.
    Thread-safe; not intended to be called while the robot is moving.
    """
    global A1, D1, A2, A3, A4
    with _cfg_lock:
        # Validate ALL values first, then apply atomically — a failed call
        # must never leave partially-updated module state behind.
        new_values: dict[str, float] = {
            "a1": A1 if a1 is None else float(a1),
            "d1": D1 if d1 is None else float(d1),
            "a2": A2 if a2 is None else float(a2),
            "a3": A3 if a3 is None else float(a3),
            "a4": A4 if a4 is None else float(a4),
        }
        for key in ("d1", "a2", "a3", "a4"):
            if new_values[key] <= 0.0:
                raise ValueError(
                    f"Link parameter {key} must be positive "
                    f"(got {new_values[key]})."
                )
        A1 = new_values["a1"]
        D1 = new_values["d1"]
        A2 = new_values["a2"]
        A3 = new_values["a3"]
        A4 = new_values["a4"]
        if limits:
            for joint, pair in limits.items():
                if joint in JOINT_LIMITS:
                    JOINT_LIMITS[joint] = (float(pair[0]), float(pair[1]))


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

    Computes the end-effector (Px, Py, Pz) position from four revolute joint
    angles using the currently-configured DH parameters.

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
        End-effector position (Px, Py, Pz) in millimetres.

    Examples
    --------
    >>> x, y, z = forward_kinematics(0.0, 0.0, 0.0, 0.0)
    >>> round(x, 2), round(y, 2), round(z, 2)
    (405.0, 0.0, 300.0)
    """
    t1 = math.radians(j1)
    t2 = math.radians(j2)
    t23 = math.radians(j2 + j3)
    t234 = math.radians(j2 + j3 + j4)

    # Horizontal projection (radial distance from Z-axis)
    r = A4 * math.cos(t234) + A3 * math.cos(t23) + A2 * math.cos(t2) + A1

    px = r * math.cos(t1)
    py = r * math.sin(t1)
    pz = D1 - A3 * math.sin(t23) - A2 * math.sin(t2) - A4 * math.sin(t234)

    return px, py, pz


def inverse_kinematics(
    x: float,
    y: float,
    z: float = 300.0,
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
    >>> j1, j2, j3, j4 = inverse_kinematics(405.0, 0.0, 300.0, phi=0.0)
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

    # FK uses: Pz = d1 - height. So downward depth is d1 - Pz
    z_depth = D1 - z

    # r = sqrt(Px² + Py²) - a1
    r = r_total - A1
    # r4 = r - a4·cos(φ)
    r4 = r - A4 * math.cos(phi_rad)
    # The downward depth of the wrist:
    z4 = z_depth - A4 * math.sin(phi_rad)

    # ── 2-link planar IK (a₂, a₃) for θ₂, θ₃ ────────────────────────────
    dist_sq = r4 * r4 + z4 * z4
    dist = math.sqrt(dist_sq)

    # Check reachability of the wrist point
    if round(dist, 4) > (A2 + A3) or round(dist, 4) < abs(A2 - A3):
        raise WorkspaceError(
            f"Target ({x:.2f}, {y:.2f}, {z:.2f}) with φ={phi:.1f}° is "
            f"outside the reachable workspace (wrist distance={dist:.2f} mm, "
            f"arm range=[{abs(A2 - A3):.1f}, {A2 + A3:.1f}] mm)."
        )

    # C₃ = (r₄² + z₄² − a₂² − a₃²) / (2·a₂·a₃)
    cos_theta3 = (dist_sq - A2 * A2 - A3 * A3) / (2.0 * A2 * A3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # clamp for numerical safety

    # S₃ = √(1 − C₃²)  (elbow-up solution)
    sin_theta3 = math.sqrt(1.0 - cos_theta3 * cos_theta3)

    # θ₃ = atan2(S₃, C₃)
    theta3 = math.atan2(sin_theta3, cos_theta3)

    # α = atan2(z₄, r₄)
    alpha = math.atan2(z4, r4)

    # β = atan2(a₃·sin(θ₃), a₂ + a₃·cos(θ₃))
    beta = math.atan2(A3 * sin_theta3, A2 + A3 * cos_theta3)

    # θ₂ = α − β
    theta2 = alpha - beta

    # θ₄ = φ − θ₂ − θ₃
    theta4 = phi_rad - theta2 - theta3

    return (
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3),
        math.degrees(theta4),
    )
