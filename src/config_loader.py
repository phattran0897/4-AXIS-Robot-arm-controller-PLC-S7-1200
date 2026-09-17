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
    # Manual Capture ROI settings
    roi_x: int = 100
    roi_y: int = 80
    roi_width: int = 240
    roi_height: int = 240
    # Optional camera calibration file path (OpenCV YAML/JSON)
    calibration_path: str = ""


@dataclass(slots=True)
class CameraConfig:
    default_index: int = 0
    display_width: int = 440
    display_height: int = 310
    fps: int = 30
    # GUI preview redraw rate – decoupled from camera fps to save CPU
    preview_fps: int = 20
    # Inference resolution (smaller = faster YOLO inference)
    inference_width: int = 640
    inference_height: int = 480
    # Camera read timeout in seconds
    read_timeout: float = 2.0
    # Enable unsharp mask image sharpening before inference
    enable_unsharp_mask: bool = True


@dataclass(slots=True)
class KinematicsConfig:
    # DH link parameters (mm)
    a1: float = 0.0  # Base horizontal offset
    d1: float = 189.6  # Base height
    a2: float = 190.0  # Link 2 length
    a3: float = 190.0  # Link 3 length
    a4: float = 74.0  # Link 4 / end-effector length
    # Joint angle limits (degrees)
    j1_min: float = -180.0
    j1_max: float = 180.0
    j2_min: float = -180.0
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
    # Defaults must lie inside the reachable workspace of the default arm
    # geometry (see kinematics.py); keep them consistent with config.yaml.
    z_down: float = 80.0
    z_up: float = 150.0


def _default_place_good() -> PlacePosition:
    return PlacePosition(x=150.0, y=100.0, z_down=80.0, z_up=150.0)


def _default_place_bad() -> PlacePosition:
    return PlacePosition(x=-150.0, y=100.0, z_down=80.0, z_up=150.0)


@dataclass(slots=True)
class SortPositionsConfig:
    place_good: PlacePosition = field(default_factory=_default_place_good)
    place_bad: PlacePosition = field(default_factory=_default_place_bad)
    pick_z_down: float = 80.0
    pick_z_up: float = 150.0
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

_DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config.yaml")


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
    Negative offsets (e.g. ``j4_target = -1``) indicate unmapped fields
    and are excluded from the size calculation.
    """
    # j4_target may be -1 to indicate "not mapped" – ignore negative offsets
    j4_end = (offsets.j4_target + 4) if offsets.j4_target >= 0 else 0
    last_byte = max(
        j4_end,
        offsets.error_flag_byte + 1,
        offsets.phan_loai_hang_byte + 1,
        offsets.hang_tot_byte + 1,
        offsets.hang_xau_byte + 1,
    )
    return ((last_byte + 3) // 4) * 4  # round up to nearest multiple of 4


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
    # NOTE: `or {}` (not `default={}`) because an *empty* YAML section parses
    # to None while still being a present key, which would crash `.get()`.
    plc_raw: dict[str, Any] = raw.get("plc") or {}
    off_raw: dict[str, Any] = plc_raw.get("offsets") or {}
    cmd_raw: dict[str, Any] = plc_raw.get("commands") or {}

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
        grip=cmd_raw.get("grip", 4),
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
    yolo_raw: dict[str, Any] = raw.get("yolo") or {}
    yolo_cfg = YOLOConfig(
        model_path=yolo_raw.get("model_path", "models/best.pt"),
        thresh=float(yolo_raw.get("thresh", 0.50)),
        px2mm=float(yolo_raw.get("px2mm", 0.50)),
        home_x=float(yolo_raw.get("home_x", 200.0)),
        home_y=float(yolo_raw.get("home_y", 0.0)),
        roi_x=int(yolo_raw.get("roi_x", 100)),
        roi_y=int(yolo_raw.get("roi_y", 80)),
        roi_width=int(yolo_raw.get("roi_width", 240)),
        roi_height=int(yolo_raw.get("roi_height", 240)),
        calibration_path=str(yolo_raw.get("calibration_path", "")),
    )

    # ── Camera ──────────────────────────────────────────────────────────────
    cam_raw: dict[str, Any] = raw.get("camera") or {}
    cam_cfg = CameraConfig(
        default_index=int(cam_raw.get("default_index", 0)),
        display_width=int(cam_raw.get("display_width", 440)),
        display_height=int(cam_raw.get("display_height", 310)),
        fps=int(cam_raw.get("fps", 30)),
        preview_fps=int(cam_raw.get("preview_fps", 20)),
        inference_width=int(cam_raw.get("inference_width", 640)),
        inference_height=int(cam_raw.get("inference_height", 480)),
        read_timeout=float(cam_raw.get("read_timeout", 2.0)),
        enable_unsharp_mask=bool(cam_raw.get("enable_unsharp_mask", True)),
    )

    # ── Kinematics ──────────────────────────────────────────────────────────
    kin_raw: dict[str, Any] = raw.get("kinematics") or {}
    kin_cfg = KinematicsConfig(
        a1=float(kin_raw.get("a1", 40.0)),
        d1=float(kin_raw.get("d1", 300.0)),
        a2=float(kin_raw.get("a2", 190.0)),
        a3=float(kin_raw.get("a3", 110.0)),
        a4=float(kin_raw.get("a4", 65.0)),
        j1_min=float(kin_raw.get("j1_min", -180.0)),
        j1_max=float(kin_raw.get("j1_max", 180.0)),
        j2_min=float(kin_raw.get("j2_min", 0.0)),
        j2_max=float(kin_raw.get("j2_max", 250.0)),
        j3_min=float(kin_raw.get("j3_min", 0.0)),
        j3_max=float(kin_raw.get("j3_max", 200.0)),
        j4_min=float(kin_raw.get("j4_min", -180.0)),
        j4_max=float(kin_raw.get("j4_max", 180.0)),
        default_phi=float(kin_raw.get("default_phi", 0.0)),
    )

    # ── App ─────────────────────────────────────────────────────────────────
    app_raw: dict[str, Any] = raw.get("app") or {}
    app_cfg = AppConfig(
        title=app_raw.get("title", "4-AXIS INDUSTRIAL ROBOT CONTROL SYSTEM"),
        geometry=app_raw.get("geometry", "1150x700"),
        appearance_mode=app_raw.get("appearance_mode", "dark"),
        color_theme=app_raw.get("color_theme", "blue"),
        plc_poll_interval=float(app_raw.get("plc_poll_interval", 0.10)),
        move_cooldown=float(app_raw.get("move_cooldown", 1.50)),
    )

    # ── Sort Positions ──────────────────────────────────────────────────────
    sort_raw: dict[str, Any] = raw.get("sort_positions") or {}
    good_raw: dict[str, Any] = sort_raw.get("place_good") or {}
    bad_raw: dict[str, Any] = sort_raw.get("place_bad") or {}

    # NOTE: fallback values here MUST stay in sync with the dataclass defaults
    # above and with config.yaml — they are the safe in-workspace poses.
    place_good = PlacePosition(
        x=float(good_raw.get("x", 150.0)),
        y=float(good_raw.get("y", 100.0)),
        z_down=float(good_raw.get("z_down", 80.0)),
        z_up=float(good_raw.get("z_up", 150.0)),
    )
    place_bad = PlacePosition(
        x=float(bad_raw.get("x", -150.0)),
        y=float(bad_raw.get("y", 100.0)),
        z_down=float(bad_raw.get("z_down", 80.0)),
        z_up=float(bad_raw.get("z_up", 150.0)),
    )
    sort_cfg = SortPositionsConfig(
        place_good=place_good,
        place_bad=place_bad,
        pick_z_down=float(sort_raw.get("pick_z_down", 80.0)),
        pick_z_up=float(sort_raw.get("pick_z_up", 150.0)),
        gripper_delay=float(sort_raw.get("gripper_delay", 0.5)),
    )

    # ── Validation ──────────────────────────────────────────────────────────
    if not 0.0 < yolo_cfg.thresh <= 1.0:
        raise ValueError(f"yolo.thresh must be in (0, 1] (got {yolo_cfg.thresh}).")
    if yolo_cfg.px2mm <= 0.0:
        raise ValueError(f"yolo.px2mm must be positive (got {yolo_cfg.px2mm}).")
    if app_cfg.plc_poll_interval <= 0.0:
        raise ValueError(
            f"app.plc_poll_interval must be positive (got {app_cfg.plc_poll_interval})."
        )
    for link_name in ("a1", "a2", "a3", "a4"):
        if getattr(kin_cfg, link_name) < 0.0:
            raise ValueError(
                f"kinematics.{link_name} must be non-negative "
                f"(got {getattr(kin_cfg, link_name)})."
            )
    if kin_cfg.d1 <= 0.0:
        raise ValueError(f"kinematics.d1 must be positive (got {kin_cfg.d1}).")

    # Joint limits must be strictly ordered so validation logic in the UI
    # has a non-empty valid range.
    for axis in ("j1", "j2", "j3", "j4"):
        low = getattr(kin_cfg, f"{axis}_min")
        high = getattr(kin_cfg, f"{axis}_max")
        if not low < high:
            raise ValueError(
                f"kinematics.{axis}_min must be < {axis}_max (got [{low}, {high}])."
            )

    # PLC offsets: bytes non-negative, bits within one byte (0–7).
    # j4_target = -1 is the documented "unmapped" sentinel.
    byte_field_names = (
        "cmd_word",
        "status",
        "j1_target",
        "j2_target",
        "j3_target",
        "motion_done_byte",
        "error_flag_byte",
        "phan_loai_hang_byte",
        "hang_tot_byte",
        "hang_xau_byte",
    )
    for name in byte_field_names:
        if getattr(offsets, name) < 0:
            raise ValueError(
                f"plc.offsets.{name} must be >= 0 (got {getattr(offsets, name)})."
            )
    if offsets.j4_target < -1:
        raise ValueError(
            f"plc.offsets.j4_target must be >= -1 (-1 = unmapped; got {offsets.j4_target})."
        )
    bit_field_names = (
        "motion_done_bit",
        "error_flag_bit",
        "phan_loai_hang_bit",
        "hang_tot_bit",
        "hang_xau_bit",
    )
    for name in bit_field_names:
        bit_value = getattr(offsets, name)
        if not 0 <= bit_value <= 7:
            raise ValueError(f"plc.offsets.{name} must be in 0..7 (got {bit_value}).")

    # Sort positions: delays non-negative, approach must descend below retreat.
    if sort_cfg.gripper_delay < 0.0:
        raise ValueError(
            f"sort_positions.gripper_delay must be >= 0 (got {sort_cfg.gripper_delay})."
        )
    for label, pos in (
        ("place_good", sort_cfg.place_good),
        ("place_bad", sort_cfg.place_bad),
    ):
        if not pos.z_down < pos.z_up:
            raise ValueError(
                f"sort_positions.{label}.z_down ({pos.z_down}) must be "
                f"below z_up ({pos.z_up})."
            )

    # Camera / vision sanity.
    if cam_cfg.fps <= 0 or cam_cfg.preview_fps <= 0:
        raise ValueError("camera.fps and camera.preview_fps must be positive.")
    if cam_cfg.display_width <= 0 or cam_cfg.display_height <= 0:
        raise ValueError("camera.display dimensions must be positive.")
    if cam_cfg.inference_width <= 0 or cam_cfg.inference_height <= 0:
        raise ValueError("camera.inference dimensions must be positive.")
    if cam_cfg.read_timeout <= 0.0:
        raise ValueError(
            f"camera.read_timeout must be positive (got {cam_cfg.read_timeout})."
        )
    if yolo_cfg.roi_width <= 0 or yolo_cfg.roi_height <= 0:
        raise ValueError("yolo.roi_width/roi_height must be positive.")
    if yolo_cfg.roi_x < 0 or yolo_cfg.roi_y < 0:
        raise ValueError(
            f"yolo.roi_x/roi_y must be non-negative "
            f"(got {yolo_cfg.roi_x}, {yolo_cfg.roi_y})."
        )

    return RobotConfig(
        plc=plc_cfg,
        yolo=yolo_cfg,
        camera=cam_cfg,
        kinematics=kin_cfg,
        app=app_cfg,
        sort_positions=sort_cfg,
    )
