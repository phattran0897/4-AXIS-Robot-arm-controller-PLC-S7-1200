"""Regression tests for live settings, repeated QR items and PLC motion safety."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from src.config_loader import (
    JointPosition, KinematicsConfig, PLCCommands, PLCConfig, PLCOffsets,
    SortPositionsConfig, compute_db_read_size, load_config, save_config, validate_config,
)
from src.ai.qr_reader import QRReader
from src.plc.plc_controller import PLCController
from src.robot.sorting_controller import SortingController

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def sorter():
    plc = MagicMock()
    plc.write_joint_targets.return_value = True
    positions = SortPositionsConfig(locations={"A1": JointPosition(0, 45, 90, 0)})
    return SortingController(plc, positions, KinematicsConfig(), PLCCommands())


@pytest.fixture
def plc():
    with patch("src.plc.plc_controller.snap7.client.Client") as factory:
        client = factory.return_value
        client.get_connected.return_value = True
        client.db_read.side_effect = lambda db, offset, size: bytearray(size)
        controller = PLCController(PLCConfig())
        yield controller, client
        controller.disconnect()


def test_identical_qr_boxes_with_short_gap_are_both_written(sorter):
    reader = QRReader(debounce_frames=3)
    reader._detector = MagicMock()
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    # Stable item, 3 blank frames (rather than 15), second identical item.
    for value in ["A1"] * 6 + [""] * 3 + ["A1"] * 6:
        reader._detector.detectAndDecode.return_value = (value, None, None)
        status, decoded = reader.decode_qr(frame)
        if status == "clear":
            sorter.process_qr_target("CLEAR")
        elif status == "value":
            sorter.process_qr_target(decoded)
    assert sorter.plc.write_joint_targets.call_count == 2
    assert sorter.qr_written == 2
    assert sorter.counter_good == 0


def test_qr_flicker_does_not_duplicate_item(sorter):
    reader = QRReader(debounce_frames=1)
    reader._detector = MagicMock()
    for value in ["A1", "", "", "A1"]:
        reader._detector.detectAndDecode.return_value = (value, None, None)
        status, decoded = reader.decode_qr(np.zeros((20, 20, 3), dtype=np.uint8))
        if status == "value":
            sorter.process_qr_target(decoded)
    assert sorter.plc.write_joint_targets.call_count == 1


def test_unknown_qr_counted_once_per_item(sorter):
    for _ in range(5):
        assert not sorter.process_qr_target("UNKNOWN")
    assert sorter.qr_invalid == 1
    sorter.process_qr_target("CLEAR")
    sorter.process_qr_target("UNKNOWN")
    assert sorter.qr_invalid == 2
    sorter.plc.write_joint_targets.assert_not_called()


def test_qr_retry_recovers_and_clear_removes_old_backoff(sorter):
    sorter.plc.write_joint_targets.return_value = False
    with patch("src.robot.sorting_controller.time.monotonic", return_value=10):
        assert not sorter.process_qr_target("A1")
        assert not sorter.process_qr_target("A1")
        assert sorter.plc.write_joint_targets.call_count == 1
        sorter.process_qr_target("CLEAR")
        sorter.plc.write_joint_targets.return_value = True
        assert sorter.process_qr_target("A1")


def test_stale_done_then_busy_then_done_required(sorter):
    sorter.plc.read_status.side_effect = [
        {"motion_done": True}, {"motion_done": True},
        {"motion_done": False}, {"motion_done": True},
    ]
    sorter._wait_event = MagicMock()
    sorter._move_and_wait(1, 2, 3, 4)
    assert sorter.plc.read_status.call_count == 4


def test_explicit_timeout_is_not_shortened_by_motion_estimate(sorter):
    clock = [0.0]
    sorter._wait_event.wait = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
    sorter.plc.read_status.side_effect = lambda: {"motion_done": clock[0] >= 4}
    with patch("src.robot.sorting_controller.time.monotonic", side_effect=lambda: clock[0]):
        sorter._move_and_wait(1, 2, 3, 4, timeout=10)
    assert clock[0] >= 4


def test_home_write_failure_never_emits_command(sorter):
    sorter.plc.write_joint_targets.return_value = False
    with pytest.raises(RuntimeError, match="command suppressed"):
        sorter._move_and_wait(0, 0, 0, 0, cmd=sorter.plc_commands.home)
    sorter.plc.send_command.assert_not_called()
    sorter.plc.send_joint_targets.assert_not_called()


def test_home_emits_only_home_command(sorter):
    sorter.plc.read_status.side_effect = [{"motion_done": False}, {"motion_done": True}]
    sorter._wait_event = MagicMock()
    sorter._return_home()
    sorter.plc.send_command.assert_called_once_with(sorter.plc_commands.home)
    sorter.plc.send_joint_targets.assert_not_called()


def test_move_l_preflights_whole_path(sorter):
    from src.kinematics import InverseKinematicsError
    sorter._ik = MagicMock(side_effect=[(0, 0, 0, 0), InverseKinematicsError("bad path")])
    sorter._move_and_wait = MagicMock()
    with pytest.raises(InverseKinematicsError):
        sorter._move_l_and_wait(1, 2, 3, 4, 5, 6, steps=2)
    sorter._move_and_wait.assert_not_called()


def test_noncontiguous_target_write_preserves_gap(plc):
    controller, client = plc
    controller._cfg.offsets = PLCOffsets(j1_target=90, j2_target=98, j3_target=110, j4_target=-1)
    assert controller.write_joint_targets(1, 2, 3, 4)
    writes = client.db_write.call_args_list
    assert [call.args[1] for call in writes] == [90, 98, 110]
    assert all(len(call.args[2]) == 4 for call in writes)


def test_failed_target_write_prevents_home_pulse(plc):
    controller, client = plc
    client.db_write.side_effect = RuntimeError("offline")
    controller.send_command = MagicMock()
    controller.send_joint_targets_and_command(0, 0, 0, 0, controller._cfg.commands.home)
    controller.send_command.assert_not_called()


def test_read_size_covers_every_configured_field():
    for name in PLCOffsets.__dataclass_fields__:
        if name.endswith("_bit"):
            continue
        offsets = PLCOffsets()
        setattr(offsets, name, 200)
        assert compute_db_read_size(offsets) >= 204


def test_pc_motion_rejects_overlapping_status_bytes(plc):
    controller, _ = plc
    with pytest.raises(ValueError, match="overlaps REAL"):
        controller.validate_motion_layout()
    controller._cfg.offsets.motion_done_byte = 70
    controller._cfg.offsets.error_flag_byte = 70
    controller.validate_motion_layout()


def test_fk_read_write_uses_configured_offsets(plc):
    from snap7.util import get_real
    controller, client = plc
    controller._cfg.offsets.theta_j1 = 94
    controller._db_read_size = compute_db_read_size(controller._cfg.offsets)
    controller.send_forward_kinematics_data(10, 20, 30, 40, 50, 60)
    first_write = client.db_write.call_args_list[0].args
    assert first_write[1] == 94
    assert get_real(first_write[2], 0) == 10
    raw = bytearray(controller._db_read_size)
    from snap7.util import set_real
    set_real(raw, 94, 123)
    client.db_read.side_effect = None
    client.db_read.return_value = raw
    assert controller.read_status()["theta_j1"] == 123


@pytest.mark.parametrize("section,name,value", [
    ("yolo", "thresh", float("nan")), ("app", "move_cooldown", -1),
    ("app", "sorting_mode", "typo"), ("kinematics", "a2", float("inf")),
    ("app", "qr_clear_frames", 2),
])
def test_bad_live_config_is_rejected(section, name, value):
    config = load_config()
    setattr(getattr(config, section), name, value)
    with pytest.raises(ValueError):
        validate_config(config)


def test_invalid_save_leaves_original_file_untouched(tmp_path):
    target = tmp_path / "config.yaml"
    original = (ROOT / "config.yaml").read_bytes()
    target.write_bytes(original)
    config = load_config(str(target))
    config.yolo.thresh = -1
    with pytest.raises(ValueError):
        save_config(config, str(target))
    assert target.read_bytes() == original


def test_settings_round_trip_preserves_comments_and_values(tmp_path):
    target = tmp_path / "config.yaml"
    target.write_bytes((ROOT / "config.yaml").read_bytes())
    config = load_config(str(target))
    config.yolo.thresh = .75
    config.app.move_cooldown = 4
    config.sort_positions.place_good.z_down = 70
    config.sort_positions.pick_z_down = 65
    save_config(config, str(target))
    loaded = load_config(str(target))
    assert loaded.yolo.thresh == .75
    assert loaded.app.move_cooldown == 4
    assert loaded.sort_positions.place_good.z_down == 70
    assert loaded.sort_positions.pick_z_down == 65
    assert "# Explicit model class mapping" in target.read_text(encoding="utf-8")


def test_frozen_model_path_is_relative_to_executable(tmp_path, monkeypatch):
    from src.ai.yolo_detector import YOLODetector
    import sys
    executable = tmp_path / "RobotController.exe"
    model = tmp_path / "my_model" / "my_model.pt"
    model.parent.mkdir()
    model.touch()
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "executable", str(executable))
    detector = YOLODetector.__new__(YOLODetector)
    assert Path(detector._resolve_model_path("my_model/my_model.pt")) == model


def test_class_names_override_ids_and_unknown_class_is_rejected():
    from src.ai.yolo_detector import YOLODetector
    detector = YOLODetector.__new__(YOLODetector)
    detector.class_quality = {"good": "GOOD", "defect": "BAD"}
    assert detector.classify_quality(0, "good") == "GOOD"
    assert detector.classify_quality(1, "defect") == "BAD"
    assert detector.classify_quality(2, "new") is None


def test_header_highlights_settings_and_clears_it_on_navigation():
    from PySide6.QtWidgets import QApplication
    from src.ui.header import VAAHeader
    app = QApplication.instance() or QApplication([])
    header = VAAHeader(current_page="PageSettings")
    assert header._btn_auto.styleSheet() == header._btn_manual.styleSheet()
    assert header._btn_settings.styleSheet() != header._btn_manual.styleSheet()
    header.update_active_tab("PageManual")
    assert header._btn_settings.styleSheet() == header._btn_auto.styleSheet()
    assert header._btn_settings.styleSheet() != header._btn_manual.styleSheet()


@pytest.mark.parametrize("threshold,success", [("0.8", True), ("-1", False)])
def test_settings_apply_validates_before_mutating_live_config(threshold, success):
    from src.ui.page_settings import PageSettings
    config = load_config()
    original = deepcopy(config)
    controller = SimpleNamespace(cfg=config, detector=MagicMock(), page_auto=MagicMock(), _sort_thread=None)
    values = {
        "yolo.thresh": threshold, "yolo.px2mm": "0.7", "yolo.roi_x": "190",
        "yolo.roi_y": "100", "yolo.roi_width": "200", "yolo.roi_height": "190",
        "sort_positions.pick_z_down": "80", "sort_positions.pick_z_up": "150",
        "place_good.z_down": "80", "place_bad.z_down": "80",
        "app.move_cooldown": "5", "app.plc_poll_interval": "0.1", "app.sorting_mode": "qr",
    }
    inputs = {}
    for key, value in values.items():
        widget = MagicMock()
        widget.text.return_value = value
        widget.currentText.return_value = value
        inputs[key] = widget, "combo" if key == "app.sorting_mode" else key.startswith("yolo.roi_")
    page = SimpleNamespace(controller=controller, _inputs=inputs)
    with patch("src.ui.page_settings.save_config", side_effect=validate_config), \
         patch("src.ui.page_settings.QMessageBox") as dialogs:
        PageSettings._save_settings(page)
    if success:
        assert config.yolo.thresh == .8
        assert controller.detector.thresh == .8
        assert controller.detector.px2mm == .7
        assert controller._sort_cooldown_duration == 5
        dialogs.information.assert_called_once()
    else:
        assert config == original
        dialogs.critical.assert_called_once()

def test_live_preview_reads_updated_roi_each_frame():
    import main
    config = load_config()
    config.app.sorting_mode = "vision"
    detector = MagicMock()
    detector.start_camera.return_value = True
    frames = [np.zeros((480, 640, 3), dtype=np.uint8) for _ in range(2)]
    def next_frame():
        if len(frames) == 1:
            config.yolo.roi_x = 20
            config.yolo.roi_y = 30
        return frames.pop(0)
    detector.read_frame.side_effect = next_frame
    stop = MagicMock()
    stop.is_set.side_effect = [False, False, True]
    trigger = MagicMock()
    trigger.is_set.return_value = False
    app = SimpleNamespace(cfg=config, detector=detector, _stop_event=stop,
                          _manual_classify_trigger=trigger, _frame_pending=True)
    with patch("main.cv2.rectangle") as rectangle, patch("main.cv2.putText"), \
         patch("main.time.monotonic", side_effect=[1.0, 2.0]):
        main.RobotApp._yolo_processing_loop(app)
    assert rectangle.call_args_list[0].args[1] == (190, 100)
    assert rectangle.call_args_list[1].args[1] == (20, 30)


def test_low_threshold_is_forwarded_to_model_and_hysteresis_resets():
    from src.ai.yolo_detector import YOLODetector
    detector = YOLODetector.__new__(YOLODetector)
    detector.thresh = .1
    detector.home_x, detector.home_y = 200, 0
    detector.px2mm = .5
    detector.class_quality = {"0": "BAD", "1": "GOOD"}
    detector._last_robot_x, detector._last_robot_y = 201, 0
    detector.model = MagicMock()
    detector.model.names = {0: "bad", 1: "good"}
    boxes = []
    for confidence, cls in [(.15, 0), (.9, 1)]:
        box = MagicMock()
        box.xyxy.cpu().numpy.return_value = np.array([[0, 0, 4, 4]])
        box.conf.item.return_value = confidence
        box.cls.item.return_value = cls
        boxes.append(box)
    detector.model.return_value = [SimpleNamespace(boxes=boxes)]
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    result = detector._run_inference(frame, 10, 10)
    assert result[0] is True
    assert result[3] == 1  # Highest confidence, independent of box order.
    assert detector.model.call_args.kwargs["conf"] == .1
    detector.model.return_value = [SimpleNamespace(boxes=[])]
    assert detector._run_inference(frame, 10, 10)[0] is False
    assert detector._last_robot_x is detector._last_robot_y is None


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_motion_values_are_rejected(value, plc):
    from src.kinematics import inverse_kinematics, validate_joint_angles, InverseKinematicsError
    with pytest.raises(InverseKinematicsError):
        inverse_kinematics(value, 0, 80)
    with pytest.raises(InverseKinematicsError):
        validate_joint_angles(value, 0, 0, 0)
    controller, client = plc
    assert not controller.write_joint_targets(value, 0, 0, 0)
    client.db_write.assert_not_called()


def test_start_pulse_failure_is_reported_to_motion_caller(plc):
    controller, _ = plc
    controller.send_pulse = MagicMock(return_value=False)
    assert controller.send_joint_targets(10, 20, 30, 0) is False


def test_classification_failure_aborts_before_motion(sorter):
    from src.robot.sorting_controller import SortResult, RobotState
    sorter.plc.write_classification.return_value = False
    with pytest.raises(RuntimeError, match="classification"):
        sorter.execute_sort(150, 100, SortResult.GOOD)
    assert sorter.state == RobotState.ERROR
    sorter.plc.send_joint_targets.assert_not_called()
    sorter.plc.write_bit.assert_not_called()


def test_runtime_stager_copies_configured_model_and_calibration(tmp_path, monkeypatch):
    import yaml
    from tools import prepare_build
    source = tmp_path / "source"
    source.mkdir()
    (source / "weights").mkdir()
    (source / "weights" / "test.pt").write_bytes(b"model")
    (source / "calibration.yaml").write_text("camera calibration", encoding="utf-8")
    (source / "assets").mkdir()
    (source / "assets" / "test.png").write_bytes(b"asset")
    (source / "config.yaml").write_text(yaml.safe_dump({"yolo": {
        "model_path": "weights/test.pt", "calibration_path": "calibration.yaml",
    }}), encoding="utf-8")
    monkeypatch.setattr(prepare_build, "ROOT", source)
    destination = tmp_path / "output"
    prepare_build.stage_runtime(destination)
    assert (destination / "weights" / "test.pt").read_bytes() == b"model"
    assert (destination / "calibration.yaml").is_file()
    assert (destination / "assets" / "test.png").is_file()
    assert (destination / "config.yaml").is_file()


def test_application_boots_and_closes_with_mocked_hardware():
    from PySide6.QtWidgets import QApplication
    import main
    app = QApplication.instance() or QApplication([])
    config = load_config()
    with patch("main.PLCController") as plc_factory, patch("main.YOLODetector") as detector_factory, \
         patch("main.configure_kinematics"):
        plc_factory.return_value.read_status.return_value = {}
        detector = detector_factory.return_value
        detector.available_cameras.return_value = [0]
        detector.read_frame.return_value = None
        detector.start_camera.return_value = False
        window = main.RobotApp(config)
        window.show_frame("PageSettings")
        app.processEvents()
        window.close()
        assert not window._ai_thread.is_alive()
        assert not window._plc_thread.is_alive()
