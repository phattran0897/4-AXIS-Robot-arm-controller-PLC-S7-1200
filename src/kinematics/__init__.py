"""
src/kinematics/__init__.py – Kinematics module for SCARA robot.
"""

from src.kinematics.kinematics import (
    InverseKinematicsError,
    WorkspaceError,
    forward_kinematics,
    inverse_kinematics,
    reachable,
)

__all__ = [
    "forward_kinematics",
    "inverse_kinematics",
    "InverseKinematicsError",
    "WorkspaceError",
    "reachable",
]
