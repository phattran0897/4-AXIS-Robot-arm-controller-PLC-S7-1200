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
    phan_loai_hang_byte: int = 78
    phan_loai_hang_bit: int = 0
    hang_tot_byte: int = 78
    hang_tot_bit: int = 1
    hang_xau_byte: int = 78
    hang_xau_bit: int = 2


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
    # Stability tracking: stop re-dispatching once object is stable
    stable_threshold_mm: float = 2.0   # max distance (mm) between frames to count as stable
    stable_frames: int = 5             # number of consecutive stable frames before locking


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
    # DH link parameters (mm)
    d1: float = 100.0    # Base height
    a2: float = 106.0    # Link 2 length
    a3: float = 103.0    # Link 3 length
    a4: float = 85.0     # Link 4 / end-effector length
    # Joint angle limits (degrees)
    j1_min: float = -180.0
    j1_max: float = 180.0
    j2_min: float = 0.0
    j2_max: float = 250.0
    j3_min: float = 0.0
    j3_max: float = 200.0
    j4_min: float = -180.0
    j4_max: float = 180.0
    # Default end-effector pitch angle for IK (degrees)
    default_phi: float = 0.0


@dataclass(slots=True)
class AppConfig:
    title: str = "4-AXIS INDUSTRIAL ROBOT CONTROL SYSTEM"
    geometry: str = "1150x700"
    appearance_mode: str = "dark"
    color_theme: str = "blue"
    plc_poll_interval: float = 0.10
    move_cooldown: float = 1.50


@dataclass(slots=True)
class PlacePosition:
    x: float = 0.0
    y: float = 0.0
    z_down: float = -50.0
    z_up: float = 0.0


@dataclass(slots=True)
class SortPositionsConfig:
    place_good: PlacePosition = field(default_factory=PlacePosition)
    place_bad: PlacePosition = field(default_factory=PlacePosition)
    pick_z_down: float = -50.0
    pick_z_up: float = 0.0
    gripper_delay: float = 0.5


@dataclass(slots=True)
class RobotConfig:
    plc: PLCConfig = field(default_factory=PLCConfig)
    yolo: YOLOConfig = field(default_factory=YOLOConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    kinematics: KinematicsConfig = field(default_factory=KinematicsConfig)
    app: AppConfig = field(default_factory=AppConfig)
    sort_positions: SortPositionsConfig = field(default_factory=SortPositionsConfig)


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
        offsets.phan_loai_hang_byte + 1,
        offsets.hang_tot_byte + 1,
        offsets.hang_xau_byte + 1,
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
        phan_loai_hang_byte=off_raw.get("phan_loai_hang_byte", 78),
        phan_loai_hang_bit=off_raw.get("phan_loai_hang_bit", 0),
        hang_tot_byte=off_raw.get("hang_tot_byte", 78),
        hang_tot_bit=off_raw.get("hang_tot_bit", 1),
        hang_xau_byte=off_raw.get("hang_xau_byte", 78),
        hang_xau_bit=off_raw.get("hang_xau_bit", 2),
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
        stable_threshold_mm=float(yolo_raw.get("stable_threshold_mm", 2.0)),
        stable_frames=int(yolo_raw.get("stable_frames", 5)),
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
        d1=float(kin_raw.get("d1", 100.0)),
        a2=float(kin_raw.get("a2", 106.0)),
        a3=float(kin_raw.get("a3", 103.0)),
        a4=float(kin_raw.get("a4", 85.0)),
        j1_min=float(kin_raw.get("j1_min", -180.0)),
        j1_max=float(kin_raw.get("j1_max", 180.0)),
        j2_min=float(kin_raw.get("j2_min", -180.0)),
        j2_max=float(kin_raw.get("j2_max", 180.0)),
        j3_min=float(kin_raw.get("j3_min", -180.0)),
        j3_max=float(kin_raw.get("j3_max", 180.0)),
        j4_min=float(kin_raw.get("j4_min", -180.0)),
        j4_max=float(kin_raw.get("j4_max", 180.0)),
        default_phi=float(kin_raw.get("default_phi", 0.0)),
    )

    # ── App ─────────────────────────────────────────────────────────────────
    app_raw: dict[str, Any] = raw.get("app", {})
    app_cfg = AppConfig(
        title=app_raw.get("title", "4-AXIS INDUSTRIAL ROBOT CONTROL SYSTEM"),
        geometry=app_raw.get("geometry", "1150x700"),
        appearance_mode=app_raw.get("appearance_mode", "dark"),
        color_theme=app_raw.get("color_theme", "blue"),
        plc_poll_interval=float(app_raw.get("plc_poll_interval", 0.10)),
        move_cooldown=float(app_raw.get("move_cooldown", 1.50)),
    )

    # ── Sort Positions ──────────────────────────────────────────────────────
    sort_raw: dict[str, Any] = raw.get("sort_positions", {})
    good_raw: dict[str, Any] = sort_raw.get("place_good", {})
    bad_raw: dict[str, Any] = sort_raw.get("place_bad", {})

    place_good = PlacePosition(
        x=float(good_raw.get("x", 150.0)),
        y=float(good_raw.get("y", 100.0)),
        z_down=float(good_raw.get("z_down", -50.0)),
        z_up=float(good_raw.get("z_up", 0.0)),
    )
    place_bad = PlacePosition(
        x=float(bad_raw.get("x", -150.0)),
        y=float(bad_raw.get("y", 100.0)),
        z_down=float(bad_raw.get("z_down", -50.0)),
        z_up=float(bad_raw.get("z_up", 0.0)),
    )
    sort_cfg = SortPositionsConfig(
        place_good=place_good,
        place_bad=place_bad,
        pick_z_down=float(sort_raw.get("pick_z_down", -50.0)),
        pick_z_up=float(sort_raw.get("pick_z_up", 0.0)),
        gripper_delay=float(sort_raw.get("gripper_delay", 0.5)),
    )

    return RobotConfig(
        plc=plc_cfg,
        yolo=yolo_cfg,
        camera=cam_cfg,
        kinematics=kin_cfg,
        app=app_cfg,
        sort_positions=sort_cfg,
    )
