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
# Custom exception
# ---------------------------------------------------------------------------


class ConfigValidationError(ValueError):
    """
    Raised by :func:`load_config` when a required key is absent from the
    YAML file (or from its expected sub-section).

    Unlike a plain :exc:`ValueError` this signals to the caller (e.g.
    ``main.py``) that the problem is a missing *configuration key* rather
    than an invalid value, so a more actionable error message can be shown.
    """


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
    theta_j1: int = 18
    theta_j2: int = 22
    theta_j3: int = 26
    py_j2_ht: int = 30
    pz_j3_ht: int = 34
    px_j1_ht: int = 38
    px_j1: int = 42
    py_j2: int = 46
    pz_j3: int = 50
    theta_j2_ht: int = 54
    theta_j3_ht: int = 58
    theta_j1_ht: int = 62
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
    class_quality: dict[str, str] = field(default_factory=lambda: {"0": "BAD", "1": "GOOD"})


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
    a1: float = 0.0   # Base horizontal offset
    d1: float = 189.6  # Base height
    a2: float = 190.0  # Link 2 length
    a3: float = 190.0  # Link 3 length
    a4: float = 74.0   # Link 4 / end-effector length
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
    sorting_mode: str = "vision"
    plc_poll_interval: float = 0.10
    move_cooldown: float = 1.50
    qr_debounce_frames: int = 5
    qr_clear_frames: int = 3
    qr_miss_tolerance: int = 2


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
class JointPosition:
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0


@dataclass(slots=True)
class SortPositionsConfig:
    # Vision mode settings
    place_good: PlacePosition = field(default_factory=_default_place_good)
    place_bad: PlacePosition = field(default_factory=_default_place_bad)
    pick_z_down: float = 80.0
    pick_z_up: float = 150.0
    gripper_delay: float = 0.5
    
    # QR mode settings
    pick: JointPosition | None = None
    locations: dict[str, JointPosition] = field(default_factory=dict)


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

import sys

if getattr(sys, 'frozen', False):
    # PyInstaller creates a frozen executable. Look for config.yaml next to the .exe
    _DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(sys.executable), "config.yaml")
else:
    # Running from source. Look for config.yaml in the project root (above src/)
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
    real_fields = ("j1_target", "j2_target", "j3_target", "j4_target",
                   "theta_j1", "theta_j2", "theta_j3", "py_j2_ht", "pz_j3_ht",
                   "px_j1_ht", "px_j1", "py_j2", "pz_j3", "theta_j2_ht",
                   "theta_j3_ht", "theta_j1_ht")
    ends = [getattr(offsets, name) + 4 for name in real_fields if getattr(offsets, name) >= 0]
    ends += [offsets.cmd_word + 2, offsets.status + 2,
             offsets.motion_done_byte + 1, offsets.error_flag_byte + 1,
             offsets.phan_loai_hang_byte + 1, offsets.hang_tot_byte + 1,
             offsets.hang_xau_byte + 1, 17]
    return ((max(ends) + 3) // 4) * 4



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

    # Helper: raise ConfigValidationError on missing required keys.
    def _require(section_data: dict[str, Any], section: str, *keys: str) -> None:
        for key in keys:
            if key not in section_data:
                raise ConfigValidationError(
                    f"Missing required config key: {section}.{key}"
                )

    # PLC
    # NOTE: `or {}` (not `default={}`) because an *empty* YAML section parses
    # to None while still being a present key, which would crash `.get()`.
    plc_raw: dict[str, Any] = raw.get("plc") or {}
    off_raw: dict[str, Any] = plc_raw.get("offsets") or {}
    cmd_raw: dict[str, Any] = plc_raw.get("commands") or {}

    # Validate required PLC keys (fail-fast: no silent fallback to wrong numbers)
    _require(plc_raw, "plc", "db_number")
    _require(off_raw, "plc.offsets", "j1_target", "j2_target", "j3_target", "j4_target")

    offsets = PLCOffsets(
        cmd_word=off_raw.get("cmd_word", 0),
        status=off_raw.get("status", 2),
        j1_target=off_raw["j1_target"],
        j2_target=off_raw["j2_target"],
        j3_target=off_raw["j3_target"],
        j4_target=off_raw["j4_target"],
        theta_j1=off_raw.get("theta_j1", 18),
        theta_j2=off_raw.get("theta_j2", 22),
        theta_j3=off_raw.get("theta_j3", 26),
        py_j2_ht=off_raw.get("py_j2_ht", 30),
        pz_j3_ht=off_raw.get("pz_j3_ht", 34),
        px_j1_ht=off_raw.get("px_j1_ht", 38),
        px_j1=off_raw.get("px_j1", 42),
        py_j2=off_raw.get("py_j2", 46),
        pz_j3=off_raw.get("pz_j3", 50),
        theta_j2_ht=off_raw.get("theta_j2_ht", 54),
        theta_j3_ht=off_raw.get("theta_j3_ht", 58),
        theta_j1_ht=off_raw.get("theta_j1_ht", 62),
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
        db_number=plc_raw["db_number"],
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
        class_quality={str(k): v for k, v in yolo_raw.get("class_quality", {"0": "BAD", "1": "GOOD"}).items()},
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
    # Kinematics
    kin_raw: dict[str, Any] = raw.get("kinematics") or {}

    # Validate required kinematics keys (fail-fast: no silent fallback to wrong numbers)
    _require(
        kin_raw,
        "kinematics",
        "a1",
        "d1",
        "a2",
        "a3",
        "a4",
        "j1_min",
        "j1_max",
        "j2_min",
        "j2_max",
        "j3_min",
        "j3_max",
        "j4_min",
        "j4_max",
    )

    kin_cfg = KinematicsConfig(
        a1=float(kin_raw["a1"]),
        d1=float(kin_raw["d1"]),
        a2=float(kin_raw["a2"]),
        a3=float(kin_raw["a3"]),
        a4=float(kin_raw["a4"]),
        j1_min=float(kin_raw["j1_min"]),
        j1_max=float(kin_raw["j1_max"]),
        j2_min=float(kin_raw["j2_min"]),
        j2_max=float(kin_raw["j2_max"]),
        j3_min=float(kin_raw["j3_min"]),
        j3_max=float(kin_raw["j3_max"]),
        j4_min=float(kin_raw["j4_min"]),
        j4_max=float(kin_raw["j4_max"]),
        default_phi=float(kin_raw.get("default_phi", 0.0)),
    )

    # ── App ─────────────────────────────────────────────────────────────────
    app_raw: dict[str, Any] = raw.get("app") or {}
    app_cfg = AppConfig(
        title=app_raw.get("title", "4-AXIS INDUSTRIAL ROBOT CONTROL SYSTEM"),
        geometry=app_raw.get("geometry", "1150x700"),
        appearance_mode=app_raw.get("appearance_mode", "dark"),
        color_theme=app_raw.get("color_theme", "blue"),
        sorting_mode=app_raw.get("sorting_mode", "vision"),
        plc_poll_interval=float(app_raw.get("plc_poll_interval", 0.10)),
        move_cooldown=float(app_raw.get("move_cooldown", 1.50)),
        qr_debounce_frames=app_raw.get("qr_debounce_frames", 5),
        qr_clear_frames=app_raw.get("qr_clear_frames", 3),
        qr_miss_tolerance=app_raw.get("qr_miss_tolerance", 2),
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
    def _parse_joint_pos(node: dict[str, Any] | None) -> JointPosition | None:
        if not node:
            return None
        return JointPosition(
            j1=float(node.get("j1", 0.0)),
            j2=float(node.get("j2", 0.0)),
            j3=float(node.get("j3", 0.0)),
            j4=float(node.get("j4", 0.0)),
        )

    parsed_locations: dict[str, JointPosition] = {}
    for loc_name, loc_data in sort_raw.get("locations", {}).items():
        parsed_loc = _parse_joint_pos(loc_data)
        if parsed_loc:
            parsed_locations[loc_name] = parsed_loc

    sort_cfg = SortPositionsConfig(
        place_good=place_good,
        place_bad=place_bad,
        pick_z_down=float(sort_raw.get("pick_z_down", 80.0)),
        pick_z_up=float(sort_raw.get("pick_z_up", 150.0)),
        gripper_delay=float(sort_raw.get("gripper_delay", 0.5)),
        pick=_parse_joint_pos(sort_raw.get("pick")),
        locations=parsed_locations,
    )

    # ── Validation ──────────────────────────────────────────────────────────
    config = RobotConfig(
        plc=plc_cfg,
        yolo=yolo_cfg,
        camera=cam_cfg,
        kinematics=kin_cfg,
        app=app_cfg,
        sort_positions=sort_cfg,
    )

    validate_config(config)
    return config


def validate_config(config: RobotConfig) -> None:
    """Validate loaded files and live settings before applying them."""
    import math
    from dataclasses import fields, is_dataclass

    def check_finite(value):
        if is_dataclass(value):
            for item in fields(value):
                check_finite(getattr(value, item.name))
        elif isinstance(value, dict):
            for item in value.values():
                check_finite(item)
        elif isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Configuration numbers must be finite.")
    check_finite(config)
    yolo_cfg, cam_cfg = config.yolo, config.camera
    app_cfg, kin_cfg = config.app, config.kinematics
    sort_cfg, offsets = config.sort_positions, config.plc.offsets
    if app_cfg.move_cooldown < 0:
        raise ValueError("app.move_cooldown must be non-negative.")
    if any(type(getattr(app_cfg, name)) is not int for name in ("qr_debounce_frames", "qr_clear_frames", "qr_miss_tolerance")):
        raise ValueError("QR frame thresholds must be integers.")
    if app_cfg.qr_debounce_frames < 1 or not 0 <= app_cfg.qr_miss_tolerance < app_cfg.qr_clear_frames:
        raise ValueError("Require qr_debounce_frames >= 1 and qr_clear_frames > qr_miss_tolerance >= 0.")
    commands = [getattr(config.plc.commands, name) for name in config.plc.commands.__dataclass_fields__]
    if any(type(value) is not int for value in commands) or len(set(commands)) != len(commands):
        raise ValueError("PLC command values must be distinct integers.")
    if app_cfg.sorting_mode not in ("qr", "vision"):
        raise ValueError("app.sorting_mode must be qr or vision.")
    if not yolo_cfg.class_quality or any(v not in ("GOOD", "BAD") for v in yolo_cfg.class_quality.values()):
        raise ValueError("yolo.class_quality must map class names or IDs to GOOD or BAD.")
    if not sort_cfg.pick_z_down < sort_cfg.pick_z_up:
        raise ValueError("pick_z_down must be below pick_z_up.")
    for name in offsets.__dataclass_fields__:
        value = getattr(offsets, name)
        if type(value) is not int:
            raise ValueError(f"plc.offsets.{name} must be an integer.")
        if name != "j4_target" and value < 0:
            raise ValueError(f"plc.offsets.{name} must be non-negative.")
    spans = sorted((getattr(offsets, f"j{i}_target"), i) for i in range(1, 5) if getattr(offsets, f"j{i}_target") >= 0)
    if any(right[0] < left[0] + 4 for left, right in zip(spans, spans[1:])):
        raise ValueError("PLC joint target REAL fields must not overlap.")
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
    if app_cfg.sorting_mode == "qr":
        if not sort_cfg.pick:
            raise ValueError("sorting_mode is 'qr' but sort_positions.pick is missing.")
        if not sort_cfg.locations:
            raise ValueError("sorting_mode is 'qr' but sort_positions.locations is empty.")



def save_config(config: RobotConfig, path: str = _DEFAULT_CONFIG_PATH) -> None:
    """
    Save the runtime configuration back to YAML.
    Uses regex replacements to preserve comments and file structure.
    """
    import re
    import tempfile
    validate_config(config)

    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updates = {
        ("yolo", "thresh"): config.yolo.thresh,
        ("yolo", "px2mm"): config.yolo.px2mm,
        ("yolo", "roi_x"): config.yolo.roi_x,
        ("yolo", "roi_y"): config.yolo.roi_y,
        ("yolo", "roi_width"): config.yolo.roi_width,
        ("yolo", "roi_height"): config.yolo.roi_height,
        ("app", "move_cooldown"): config.app.move_cooldown,
        ("app", "plc_poll_interval"): config.app.plc_poll_interval,
        ("app", "sorting_mode"): f'"{config.app.sorting_mode}"',
        ("sort_positions", "pick_z_down"): config.sort_positions.pick_z_down,
        ("sort_positions", "pick_z_up"): config.sort_positions.pick_z_up,
        ("place_good", "z_down"): config.sort_positions.place_good.z_down,
        ("place_good", "z_up"): config.sort_positions.place_good.z_up,
        ("place_bad", "z_down"): config.sort_positions.place_bad.z_down,
        ("place_bad", "z_up"): config.sort_positions.place_bad.z_up,
    }

    current_section = None
    current_subsection = None

    for i, line in enumerate(lines):
        sec_match = re.match(r"^([a-zA-Z0-9_]+):", line)
        if sec_match:
            current_section = sec_match.group(1)
            current_subsection = None
            continue

        subsec_match = re.match(r"^ {2}([a-zA-Z0-9_]+):\s*(?:#.*)?$", line)
        if subsec_match and current_section == "sort_positions":
            current_subsection = subsec_match.group(1)
            continue

        kv_match = re.match(r"^(\s+)([a-zA-Z0-9_]+):\s*([^#\n\r]+)(.*)$", line)
        if kv_match:
            indent = kv_match.group(1)
            key = kv_match.group(2)
            tail = kv_match.group(4)

            if current_section == "sort_positions" and len(indent) == 2:
                current_subsection = None
            lookup = None
            if current_section == "sort_positions" and current_subsection in ("place_good", "place_bad"):
                lookup = (current_subsection, key)
            else:
                lookup = (current_section, key)

            if lookup in updates:
                new_val = updates[lookup]
                new_val_str = "true" if new_val is True else "false" if new_val is False else str(new_val)
                lines[i] = f"{indent}{key}: {new_val_str} {tail}\n".replace("  \n", "\n")

    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=os.path.dirname(os.path.abspath(path)), suffix=".yaml", delete=False) as f:
            temp_path = f.name
            f.writelines(lines)
        load_config(temp_path)
        os.replace(temp_path, path)
    finally:
        if temp_path and os.path.exists(temp_path):
            os.unlink(temp_path)
