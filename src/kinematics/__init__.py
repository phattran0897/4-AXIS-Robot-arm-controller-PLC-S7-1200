"""
src/kinematics/__init__.py – Kinematics module for SCARA robot.
"""

from src.kinematics.kinematics import (
    InverseKinematicsError,
    WorkspaceError,
    inverse_kinematics,
    reachable,
)

__all__ = [
    "inverse_kinematics",
    "InverseKinematicsError",
    "WorkspaceError",
    "reachable",
]
