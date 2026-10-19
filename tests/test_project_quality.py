"""Project-wide configuration, lifecycle and operator regressions without devices."""
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
import threading
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import yaml

from src.config_loader import ConfigValidationError, load_config, save_config


@pytest.fixture
def config_file(tmp_path):
    target = tmp_path / "config.yaml"
    target.write_bytes((Path(__file__).parents[1] / "config.yaml").read_bytes())
    return target


@pytest.mark.parametrize("section,key,value", [
    ("camera", "enable_unsharp_mask", "false"),
    ("camera", "fps", 12.5), ("camera", "default_index", True),
    ("yolo", "roi_width", 20.8), ("app", "sorting_mode", 3),
    ("sort_positions", "locations", ["A1"]),
])
def test_config_rejects_lossy_coercions(config_file, section, key, value):
    raw = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    raw[section][key] = value
    config_file.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ConfigValidationError):
        load_config(config_file)


@pytest.mark.parametrize("text", ["[]", "plc: [1, 2]", "plc:\n  db_number: 5\n  db_number: 6\n"])
def test_malformed_config_has_actionable_validation_error(config_file, text):
    config_file.write_text(text, encoding="utf-8")
    with pytest.raises(ConfigValidationError):
        load_config(config_file)


def test_save_inserts_missing_sections_and_optional_keys(config_file):
    raw = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    del raw["yolo"]["coordinate_origin"]
    del raw["camera"]
    raw["operator_notes"] = "preserve # this value"
    config_file.write_text("# Keep the operator's calibration notes\n" + yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    config = load_config(config_file)
    config.yolo.coordinate_origin = "frame"
    config.camera.default_index = 2
    config.camera.enable_unsharp_mask = False
    save_config(config, config_file)
    assert load_config(config_file) == config
    persisted = config_file.read_text(encoding="utf-8")
    assert persisted.startswith("# Keep the operator's calibration notes")
    assert yaml.safe_load(persisted)["operator_notes"] == raw["operator_notes"]


def test_save_removes_deleted_qr_locations_and_handles_flow_yaml(config_file):
    config = load_config(config_file)
    config_file.write_text(yaml.safe_dump(asdict(config), default_flow_style=True), encoding="utf-8")
    del config.sort_positions.locations["B2"]
    save_config(config, config_file)
    assert load_config(config_file) == config


def test_atomic_save_failure_retains_original(config_file):
    original = config_file.read_bytes()
    cfg = load_config(config_file)
    cfg.yolo.thresh = .7
    with patch("src.config_loader.os.replace", side_effect=OSError("read only")):
        with pytest.raises(OSError, match="read only"):
            save_config(cfg, config_file)
    assert config_file.read_bytes() == original
    assert list(config_file.parent.glob("*.yaml")) == [config_file]


@pytest.mark.parametrize("limits", [{"j1": (1, 1)}, {"j1": (0, float("nan"))}, {"wrong": (0, 1)}])
def test_invalid_limits_do_not_partially_reconfigure_geometry(limits):
    from src.kinematics import kinematics as kin
    previous = (kin.A1, kin.D1, kin.A2, kin.A3, kin.A4, dict(kin.JOINT_LIMITS))
    with pytest.raises(ValueError):
        kin.configure(a2=250, limits=limits)
    assert (kin.A1, kin.D1, kin.A2, kin.A3, kin.A4, dict(kin.JOINT_LIMITS)) == previous


@pytest.fixture
def detector():
    from src.ai.yolo_detector import YOLODetector
    with patch.object(YOLODetector, "_probe_cameras_background"):
        value = YOLODetector("does-not-exist.pt", lazy_model=True)
    value._camera_probe_done.set()
    yield value
    value.stop()


def test_qr_mode_defers_model_loading_until_inference(detector):
    with patch("src.ai.yolo_detector.YOLO") as loader:
        assert detector._model is None
        with pytest.raises(FileNotFoundError):
            detector.annotate_frame(np.zeros((20, 20, 3), dtype=np.uint8))
        loader.assert_not_called()


def test_failed_camera_open_releases_both_native_handles(detector):
    failed = [MagicMock(), MagicMock()]
    for capture in failed:
        capture.isOpened.return_value = False
    with patch("src.ai.yolo_detector.cv2.VideoCapture", side_effect=failed), patch("src.ai.yolo_detector.time.sleep"):
        assert not detector.start_camera(0)
    for capture in failed:
        capture.release.assert_called_once()


def test_blocked_camera_reader_is_not_replaced_or_released_concurrently(detector):
    entered, unblock = threading.Event(), threading.Event()
    capture = MagicMock()
    capture.isOpened.return_value = True
    def blocked_read():
        entered.set()
        assert unblock.wait(5)
        return True, np.zeros((20, 20, 3), dtype=np.uint8)
    capture.read.side_effect = blocked_read
    with patch("src.ai.yolo_detector.cv2.VideoCapture", return_value=capture) as factory:
        assert detector.start_camera(0)
        assert entered.wait(1)
        try:
            # Simulate the driver's join deadline without making the test sleep.
            with patch.object(detector._read_thread, "join"):
                assert not detector.switch_camera(1)
            factory.assert_called_once()
            capture.release.assert_not_called()
        finally:
            unblock.set()
            detector._read_thread.join(2)
        capture.release.assert_called_once()
        assert detector._read_queue.empty()  # No frame published after cancellation.


def test_malformed_calibration_releases_file_and_preserves_fallback(detector):
    storage = MagicMock()
    matrix, distortion = MagicMock(), MagicMock()
    matrix.empty.return_value = distortion.empty.return_value = False
    matrix.mat.return_value = np.eye(2)
    distortion.mat.return_value = np.zeros((1, 5))
    storage.getNode.side_effect = [matrix, distortion]
    with patch("src.ai.yolo_detector.os.path.isfile", return_value=True), patch("src.ai.yolo_detector.cv2.FileStorage", return_value=storage):
        detector._load_calibration("invalid.yaml")
    assert not detector._calibration_loaded
    storage.release.assert_called_once()


def test_poll_worker_emits_offline_then_recovers_after_exception():
    import main
    stop = threading.Event()
    plc = MagicMock()
    plc.is_connected.return_value = True
    plc.read_status.side_effect = [RuntimeError("connection lost"), {"motion_done": True}]
    emitted = []
    def emit(data):
        emitted.append(data)
        if len(emitted) == 2:
            stop.set()
    cfg = load_config()
    cfg.app.plc_poll_interval = .001
    app = SimpleNamespace(cfg=cfg, plc=plc, _stop_event=stop,
                          plc_data_ready=SimpleNamespace(emit=emit))
    main.RobotApp._cyclic_update(app)
    assert emitted == [{"connected": False}, {"motion_done": True, "connected": True}]


def test_camera_choice_queues_io_and_applies_only_latest_request():
    import main
    app = SimpleNamespace(_camera_request_lock=threading.Lock(), _requested_camera_index=None,
                          _stop_event=threading.Event(), detector=MagicMock(),
                          qr_reader=MagicMock(), sorter=MagicMock(), _manual_classify_trigger=threading.Event())
    main.RobotApp._do_change_camera(app, "Camera 1")
    main.RobotApp._do_change_camera(app, "Camera 2")
    app.detector.switch_camera.assert_not_called()
    main.RobotApp._apply_camera_request(app)
    app.detector.switch_camera.assert_called_once_with(2)
    assert app._requested_camera_index is None


def test_telemetry_renders_untrusted_qr_as_plain_text(qtbot):
    from PySide6.QtWidgets import QTextEdit
    import main
    widget = QTextEdit()
    qtbot.addWidget(widget)
    widget.document().setMaximumBlockCount(100)
    app = SimpleNamespace(tx_textbox=widget)
    for _ in range(120):
        main.RobotApp._on_tx_logged(app, "<b>QR value</b>")
    assert "<b>QR value</b>" in widget.toPlainText()
    assert widget.document().blockCount() == 100


def test_failed_startup_shows_error_in_windowed_build(qapp):
    import main
    with patch("main.load_config", side_effect=ValueError("invalid settings")), patch("main.QMessageBox.critical") as dialog:
        assert main.main() == 1
    assert "invalid settings" in dialog.call_args.args[2]


def test_failed_window_construction_cleans_up_controllers(qapp):
    import main
    with patch("main.PLCController") as plc, patch("main.YOLODetector") as detector, \
         patch("main.SortingController", side_effect=ValueError("bad waypoints")), \
         patch("main.configure_kinematics"):
        with pytest.raises(ValueError, match="bad waypoints"):
            main.RobotApp(load_config())
        plc.return_value.disconnect.assert_called_once()
        detector.return_value.stop.assert_called_once()
