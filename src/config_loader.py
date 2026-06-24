"""
src/config_loader.py – Safe YAML configuration loader.

Usage
-----
    from src.config_loader import load_config, RobotConfig

    cfg = load_config()
    print(cfg.plc.ip)
    print(cfg.yolo.thresh)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any

import yaml


# ---------------------------------------------------------------------------
# Typed configuration dataclasses (all use __slots__ for memory efficiency)
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class PLCOffsets:
    cmd_word: int = 0
    status: int = 2
    j1_target: int = 4
    j2_target: int = 8
    j3_target: int = 12
    j4_target: int = 16
    motion_done_byte: int = 20
    motion_done_bit: int = 0
    error_flag_byte: int = 20
    error_flag_bit: int = 1


@dataclass(slots=True)
class PLCCommands:
    idle: int = 0
    home: int = 1
    move: int = 2
    stop: int = 3
    grip: int = 4


@dataclass(slots=True)
class PLCConfig:
    ip: str = "192.168.0.1"
    rack: int = 0
    slot: int = 1
    db_number: int = 10
    offsets: PLCOffsets = field(default_factory=PLCOffsets)
    commands: PLCCommands = field(default_factory=PLCCommands)


@dataclass(slots=True)
class YOLOConfig:
    model_path: str = "models/best.pt"
    thresh: float = 0.50
    px2mm: float = 0.50
    home_x: float = 200.0
    home_y: float = 0.0


@dataclass(slots=True)
class CameraConfig:
    default_index: int = 0
    display_width: int = 440
    display_height: int = 310
    fps: int = 30
    # Inference resolution (smaller = faster YOLO inference)
    inference_width: int = 640
    inference_height: int = 480
    # Camera read timeout in seconds
    read_timeout: float = 2.0


@dataclass(slots=True)
class KinematicsConfig:
    l1: float = 200.0
    l2: float = 150.0


@dataclass(slots=True)
class AppConfig:
    title: str = "4-AXIS SCARA ROBOT CONTROL SYSTEM"
    geometry: str = "1150x700"
    appearance_mode: str = "dark"
    color_theme: str = "blue"
    plc_poll_interval: float = 0.10
    move_cooldown: float = 1.50


@dataclass(slots=True)
class RobotConfig:
    plc: PLCConfig = field(default_factory=PLCConfig)
    yolo: YOLOConfig = field(default_factory=YOLOConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    kinematics: KinematicsConfig = field(default_factory=KinematicsConfig)
    app: AppConfig = field(default_factory=AppConfig)


# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config.yaml"
)


def _nested_get(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    """Safely navigate nested dict keys, returning *default* on any miss."""
    cursor: Any = data
    for key in keys:
        if not isinstance(cursor, dict):
            return default
        cursor = cursor.get(key, default)
    return cursor


def compute_db_read_size(offsets: PLCOffsets) -> int:
    """
    Compute the minimum DB read size covering all offsets.

    Returns the smallest multiple of 4 that is large enough to contain
    the last byte used by any field in the Data Block layout.
    """
    last_byte = max(
        offsets.j4_target + 4,   # j4_target REAL spans 4 bytes
        offsets.error_flag_byte + 1,
    )
    return ((last_byte + 3) // 4) * 4   # round up to nearest multiple of 4


def load_config(path: str | None = None) -> RobotConfig:
    """
    Load and validate ``config.yaml``, returning a fully-typed
    :class:`RobotConfig` instance.

    Parameters
    ----------
    path:
        Explicit filesystem path to the YAML file. When omitted the loader
        looks for ``config.yaml`` one directory above this module's package
        root (i.e. the project root).

    Raises
    ------
    FileNotFoundError
        If the YAML file cannot be found at the resolved path.
    yaml.YAMLError
        If the file contains invalid YAML syntax.
    """
    resolved = os.path.abspath(path or _DEFAULT_CONFIG_PATH)
    if not os.path.isfile(resolved):
        raise FileNotFoundError(
            f"Configuration file not found: {resolved}\n"
            "Copy config.yaml.example → config.yaml and edit as required."
        )

    with open(resolved, encoding="utf-8") as fh:
        raw: dict[str, Any] = yaml.safe_load(fh) or {}

    # ── PLC ─────────────────────────────────────────────────────────────────
    plc_raw: dict[str, Any] = raw.get("plc", {})
    off_raw: dict[str, Any] = plc_raw.get("offsets", {})
    cmd_raw: dict[str, Any] = plc_raw.get("commands", {})

    offsets = PLCOffsets(
        cmd_word=off_raw.get("cmd_word", 0),
        status=off_raw.get("status", 2),
        j1_target=off_raw.get("j1_target", 4),
        j2_target=off_raw.get("j2_target", 8),
        j3_target=off_raw.get("j3_target", 12),
        j4_target=off_raw.get("j4_target", 16),
        motion_done_byte=off_raw.get("motion_done_byte", 20),
        motion_done_bit=off_raw.get("motion_done_bit", 0),
        error_flag_byte=off_raw.get("error_flag_byte", 20),
        error_flag_bit=off_raw.get("error_flag_bit", 1),
    )
    commands = PLCCommands(
        idle=cmd_raw.get("idle", 0),
        home=cmd_raw.get("home", 1),
        move=cmd_raw.get("move", 2),
        stop=cmd_raw.get("stop", 3),
    )
    plc_cfg = PLCConfig(
        ip=plc_raw.get("ip", "192.168.0.1"),
        rack=plc_raw.get("rack", 0),
        slot=plc_raw.get("slot", 1),
        db_number=plc_raw.get("db_number", 10),
        offsets=offsets,
        commands=commands,
    )

    # ── YOLO ────────────────────────────────────────────────────────────────
    yolo_raw: dict[str, Any] = raw.get("yolo", {})
    yolo_cfg = YOLOConfig(
        model_path=yolo_raw.get("model_path", "models/best.pt"),
        thresh=float(yolo_raw.get("thresh", 0.50)),
        px2mm=float(yolo_raw.get("px2mm", 0.50)),
        home_x=float(yolo_raw.get("home_x", 200.0)),
        home_y=float(yolo_raw.get("home_y", 0.0)),
    )

    # ── Camera ──────────────────────────────────────────────────────────────
    cam_raw: dict[str, Any] = raw.get("camera", {})
    cam_cfg = CameraConfig(
        default_index=int(cam_raw.get("default_index", 0)),
        display_width=int(cam_raw.get("display_width", 440)),
        display_height=int(cam_raw.get("display_height", 310)),
        fps=int(cam_raw.get("fps", 30)),
        inference_width=int(cam_raw.get("inference_width", 640)),
        inference_height=int(cam_raw.get("inference_height", 480)),
        read_timeout=float(cam_raw.get("read_timeout", 2.0)),
    )

    # ── Kinematics ──────────────────────────────────────────────────────────
    kin_raw: dict[str, Any] = raw.get("kinematics", {})
    kin_cfg = KinematicsConfig(
        l1=float(kin_raw.get("l1", 200.0)),
        l2=float(kin_raw.get("l2", 150.0)),
    )

    # ── App ─────────────────────────────────────────────────────────────────
    app_raw: dict[str, Any] = raw.get("app", {})
    app_cfg = AppConfig(
        title=app_raw.get("title", "4-AXIS SCARA ROBOT CONTROL SYSTEM"),
        geometry=app_raw.get("geometry", "1150x700"),
        appearance_mode=app_raw.get("appearance_mode", "dark"),
        color_theme=app_raw.get("color_theme", "blue"),
        plc_poll_interval=float(app_raw.get("plc_poll_interval", 0.10)),
        move_cooldown=float(app_raw.get("move_cooldown", 1.50)),
    )

    return RobotConfig(
        plc=plc_cfg,
        yolo=yolo_cfg,
        camera=cam_cfg,
        kinematics=kin_cfg,
        app=app_cfg,
    )
