"""Integration regressions for review findings, using no physical PLC/camera."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
import threading

import numpy as np
import pytest

from src.config_loader import load_config, compute_db_read_size, validate_config, validate_motion_layout
from src.plc.plc_controller import PLCController, ADDR
from src.robot.sorting_controller import SortingController, SortResult, RobotState
from test_gui_no_plc import MockPLCController


@pytest.fixture
def cfg():
    return load_config()


@pytest.fixture
def controller(cfg):
    plc_config = deepcopy(cfg.plc)
    plc_config.offsets.motion_done_byte = plc_config.offsets.error_flag_byte = 70
    for index in range(1, 5):
        setattr(plc_config.offsets, f"j{index}_target", 80 + index * 4)
    memory = bytearray(128)
    writes = []
    with patch("src.plc.plc_controller.snap7.client.Client") as factory:
        client = factory.return_value
        client.get_connected.return_value = True
        client.db_read.side_effect = lambda db, offset, size: memory[offset:offset+size]
        def record_write(db, offset, data):
            memory[offset:offset+len(data)] = data
            writes.append((offset, bytes(data)))
        client.db_write.side_effect = record_write
        plc = PLCController(plc_config)
        yield plc, client, memory, writes
        plc.disconnect()


@pytest.fixture
def simulator(cfg):
    return MockPLCController(cfg.plc)


@pytest.fixture
def sorter(cfg, simulator):
    return SortingController(simulator, cfg.sort_positions, cfg.kinematics, cfg.plc.commands)


def test_mock_qr_write_has_no_motion_until_sensor(simulator, sorter):
    assert sorter.process_qr_target("A1")
    status = simulator.read_status()
    assert status["j1_target"] == 90
    assert status["theta_j1"] == 0
    assert status["motion_done"] is True
    assert simulator.simulate_box_arrival()
    assert simulator.read_status()["motion_done"] is False


def test_mock_home_uses_write_only_then_single_command(simulator, sorter):
    assert simulator.write_joint_targets(90, 45, 90, 0)
    simulator.simulate_box_arrival()
    with patch.object(simulator, "send_pulse", wraps=simulator.send_pulse) as pulses:
        sorter._return_home()
    assert pulses.call_args_list[0].args == ADDR.MOVE_TO_HOME
    assert all(call.args != ADDR.START_AUTO for call in pulses.call_args_list)
    assert sorter.state is RobotState.IDLE


def test_poll_rate_changes_apply_without_restart(cfg):
    import main
    waits = []
    stop = MagicMock()
    stop.is_set.side_effect = lambda: len(waits) >= 2
    def wait(delay):
        waits.append(delay)
        cfg.app.plc_poll_interval = .7
    stop.wait.side_effect = wait
    plc = MagicMock()
    plc.is_connected.return_value = True
    plc.read_status.return_value = {"auto_mode": True}
    app = SimpleNamespace(cfg=cfg, plc=plc, _stop_event=stop, plc_data_ready=MagicMock())
    main.RobotApp._cyclic_update(app)
    assert waits == [.1, .7]


def test_repeat_pulse_queues_low_gap_without_blocking_wait(controller):
    plc, _, memory, writes = controller
    with plc._lock:
        assert plc.send_pulse(*ADDR.START_AUTO)
        first = plc._pulse_timers[ADDR.START_AUTO][1]
        # The worker can use its own event, but callers must not create a
        # blocking wait object for the repeat gap while holding the client lock.
        with patch("src.plc.plc_controller.threading.Event", side_effect=AssertionError("blocking gap")):
            assert plc.send_pulse(*ADDR.START_AUTO)
        assert ADDR.START_AUTO in plc._pulse_starts
        assert not memory[0] & 1
        before = len(writes)
        plc._start_pulse(*ADDR.START_AUTO, first)
        plc._finish_pulse(*ADDR.START_AUTO, first)
        assert len(writes) == before


def test_stop_cancels_pending_rise_and_latches_across_disconnect(controller):
    plc, _, memory, _ = controller
    assert plc.write_bit(*ADDR.GRIP, True)
    assert plc.write_bit(*ADDR.JOG_J1_LEFT, True)
    with plc._lock:
        plc.send_pulse(*ADDR.START_AUTO)
        plc.send_pulse(*ADDR.START_AUTO)
        assert plc.stop_motion()
        assert not plc._pulse_starts
    threading.Event().wait(.08)
    assert not memory[0] & 1
    assert memory[16] & (1 << 4)
    assert memory[14] & 1  # Do not drop a held load.
    assert memory[80] & 0x3f == 0
    assert not plc.write_joint_targets(1, 2, 3, 4)
    assert not plc.write_bit(*ADDR.JOG_J1_LEFT, True)
    assert not plc.send_pulse(*ADDR.START_AUTO)
    plc.disconnect()
    assert memory[16] & (1 << 4)
    assert not plc._pulse_thread.is_alive()


def test_read_glitch_recovery_requires_new_busy_sample(cfg):
    plc = MagicMock()
    plc.read_status.side_effect = [
        {"motion_done": False}, {}, {"motion_done": True},
        {"motion_done": False}, {"motion_done": True},
    ]
    sorter = SortingController(plc, cfg.sort_positions, cfg.kinematics, cfg.plc.commands)
    sorter._wait_event = MagicMock()
    sorter._move_and_wait(0, 45, 90, 0)
    assert plc.read_status.call_count == 5
    plc.send_joint_targets.assert_called_once()


def test_repeated_read_failure_aborts_and_requests_stop(cfg):
    plc = MagicMock()
    plc.read_status.return_value = {}
    sorter = SortingController(plc, cfg.sort_positions, cfg.kinematics, cfg.plc.commands)
    sorter._wait_event = MagicMock()
    with pytest.raises(RuntimeError, match="3 failed reads"):
        sorter.execute_sort(150, 100, SortResult.GOOD)
    assert sorter.state is RobotState.ERROR
    plc.stop_motion.assert_called_once()
    assert sorter.counter_good == 0
    plc.write_bit.assert_not_called()


def test_motion_fault_is_not_retried(cfg):
    plc = MagicMock()
    plc.read_status.return_value = {"motion_done": False, "error_flag": True}
    sorter = SortingController(plc, cfg.sort_positions, cfg.kinematics, cfg.plc.commands)
    with pytest.raises(RuntimeError, match="error flag"):
        sorter.execute_sort(150, 100, SortResult.GOOD)
    assert plc.read_status.call_count == 1
    plc.stop_motion.assert_called_once()


def test_cancellation_wakes_delay_and_prevents_new_commands(sorter, simulator):
    started = threading.Event()
    exited = threading.Event()
    errors = []
    def run():
        started.set()
        try:
            sorter._delay(30)
        except RuntimeError as error:
            errors.append(error)
        finally:
            exited.set()
    worker = threading.Thread(target=run)
    worker.start()
    assert started.wait(1)
    assert sorter.request_stop()
    assert exited.wait(1)
    worker.join()
    assert errors
    with pytest.raises(RuntimeError, match="cancelled"):
        sorter._move_and_wait(1, 2, 3, 4)
    assert not sorter.process_qr_target("A1")
    assert simulator.read_status()["stop_robot"] is True


def test_close_event_cancels_active_waypoint_before_disconnect(sorter, simulator):
    import main
    commanded = threading.Event()
    errors = []
    original = simulator.send_joint_targets
    def send(*targets):
        result = original(*targets)
        commanded.set()
        return result
    simulator.send_joint_targets = send
    def move():
        try:
            sorter._move_and_wait(180, 45, 90, 0)
        except RuntimeError as error:
            errors.append(error)
    worker = threading.Thread(target=move)
    worker.start()
    assert commanded.wait(1)
    inactive = MagicMock()
    inactive.is_alive.return_value = False
    app = SimpleNamespace(_sort_start_lock=threading.Lock(), _stop_event=threading.Event(),
        _manual_classify_trigger=threading.Event(), _camera_timer=MagicMock(),
        sorter=sorter, _sort_thread=worker, _plc_thread=inactive, _ai_thread=inactive,
        detector=MagicMock(), plc=simulator)
    event = MagicMock()
    main.RobotApp.closeEvent(app, event)
    assert not worker.is_alive()
    assert errors
    assert simulator._stopped
    assert not simulator.is_connected()
    event.accept.assert_called_once()


def test_local_reset_never_pulses_pause(cfg, simulator, sorter):
    import main
    sorter.request_stop()
    simulator.send_command = MagicMock()
    app = SimpleNamespace(_sort_start_lock=threading.Lock(), _sort_thread=None,
                          plc=simulator, sorter=sorter, log_tx=MagicMock())
    main.RobotApp.clear_all_errors(app)
    simulator.send_command.assert_not_called()
    assert not simulator._stopped
    assert sorter.state is RobotState.IDLE


def test_fault_ack_does_not_reset_local_state_until_plc_is_healthy(cfg, sorter):
    import main
    sorter.request_stop()
    plc = MagicMock()
    plc.read_status.return_value = {"error_flag": True, "motion_done": True}
    app = SimpleNamespace(_sort_start_lock=threading.Lock(), _sort_thread=None,
                          plc=plc, sorter=sorter, log_tx=MagicMock())
    main.RobotApp.clear_all_errors(app)
    plc.acknowledge_error.assert_called_once()
    plc.resume_commands.assert_not_called()
    assert sorter.state is RobotState.ERROR


def test_reset_does_not_release_stop_while_motion_unconfirmed(sorter):
    import main
    plc = MagicMock()
    plc.read_status.return_value = {"error_flag": False, "motion_done": False}
    app = SimpleNamespace(_sort_start_lock=threading.Lock(), _sort_thread=None,
                          plc=plc, sorter=sorter, log_tx=MagicMock())
    main.RobotApp.clear_all_errors(app)
    plc.resume_commands.assert_not_called()


def test_vision_layout_rejects_xyz_alias_even_after_status_is_remapped(cfg):
    offsets = deepcopy(cfg.plc.offsets)
    offsets.motion_done_byte = offsets.error_flag_byte = 70
    with pytest.raises(ValueError, match="overlaps px_j1"):
        validate_motion_layout(offsets)


@pytest.mark.parametrize("byte,bit", [(16, 4), (20, 0), (42, 0), (78, 1)])
def test_fault_ack_requires_dedicated_bit(cfg, byte, bit):
    cfg.plc.offsets.error_ack_byte, cfg.plc.offsets.error_ack_bit = byte, bit
    with pytest.raises(ValueError):
        validate_config(cfg)


def test_fault_ack_uses_configured_bit(controller):
    plc, _, memory, _ = controller
    plc._cfg.offsets.error_ack_byte = 74
    plc._cfg.offsets.error_ack_bit = 3
    assert plc.stop_motion()
    assert plc.acknowledge_error()
    assert memory[74] & (1 << 3)
    assert memory[16] & (1 << 4)
    assert not memory[0] & (1 << 1)  # PAUSE never used as fault ack.


def detector_with_box(x, y):
    from src.ai.yolo_detector import YOLODetector
    detector = YOLODetector.__new__(YOLODetector)
    detector.thresh, detector.px2mm = .3, .5
    detector.home_x, detector.home_y = 200, 0
    detector.inference_width, detector.inference_height = 640, 480
    detector.class_quality = {"0": "BAD"}
    detector._last_robot_x = detector._last_robot_y = None
    box = MagicMock()
    box.xyxy.cpu().numpy.return_value = np.array([[x-2, y-2, x+2, y+2]])
    box.conf.item.return_value, box.cls.item.return_value = .9, 0
    detector.model = MagicMock(return_value=[SimpleNamespace(boxes=[box])])
    detector.model.names = {0: "defect"}
    return detector


def test_frame_origin_applies_crop_offset_in_original_pixels():
    detector = detector_with_box(100, 95)
    frame = np.zeros((190, 200, 3), dtype=np.uint8)
    result = detector.annotate_frame(frame, pixel_offset=(190, 100), pixel_origin=(320, 240))
    assert (result.robot_x, result.robot_y) == (185, -22.5)


def test_roi_origin_preserves_existing_calibration():
    detector = detector_with_box(100, 95)
    result = detector.annotate_frame(np.zeros((190, 200, 3), dtype=np.uint8))
    assert (result.robot_x, result.robot_y) == (200, 0)


def test_resize_preserves_mm_scale_from_original_image():
    detector = detector_with_box(150, 100)
    result = detector.annotate_frame(np.zeros((300, 400, 3), dtype=np.uint8), inference_w=200, inference_h=150)
    assert (result.robot_x, result.robot_y) == (250, 25)


def test_undecodable_visible_qr_does_not_rearm_same_item():
    from src.ai.qr_reader import QRReader
    reader = QRReader(debounce_frames=1)
    reader._detector = MagicMock()
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    points = np.array([[[0, 0], [5, 0], [5, 5], [0, 5]]])
    reader._detector.detectAndDecode.return_value = ("A1", points, None)
    assert reader.decode_qr(frame) == ("value", "A1")
    reader._detector.detectAndDecode.return_value = ("", points, None)
    assert all(reader.decode_qr(frame) == ("none", None) for _ in range(10))
    reader._detector.detectAndDecode.return_value = ("", None, None)
    statuses = [reader.decode_qr(frame)[0] for _ in range(12)]
    assert statuses.count("clear") == 1


def test_tracker_counts_first_frame_after_unlock_and_supports_one_frame():
    from src.ai.stability_tracker import StabilityTracker
    tracker = StabilityTracker(required_frames=3)
    assert [tracker.update(0, 0) for _ in range(3)] == [False, False, True]
    assert [tracker.update(100, 0) for _ in range(3)] == [False, False, True]
    assert StabilityTracker(required_frames=1).update(0, 0)


def test_manual_decimal_comma_is_accepted():
    from src.ui.page_manual import PageManual
    assert PageManual._entry_float(SimpleNamespace(text=lambda: "12,5")) == 12.5


def test_counter_completion_is_serialized_with_reset(cfg):
    plc = MagicMock()
    sorter = SortingController(plc, cfg.sort_positions, cfg.kinematics, cfg.plc.commands)
    at_home = threading.Event()
    sorter._pick, sorter._place = MagicMock(), MagicMock()
    def home():
        sorter._state = RobotState.IDLE
        at_home.set()
    sorter._return_home = home
    with sorter._qr_lock:
        worker = threading.Thread(target=sorter.execute_sort, args=(150, 100, SortResult.GOOD))
        worker.start()
        assert at_home.wait(1)
        sorter.reset_counters()
    worker.join(1)
    assert not worker.is_alive()
    assert sorter.counter_good == 1

def test_startup_blocks_invalid_vision_map_before_operator_capture(cfg):
    import main
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    cfg.app.sorting_mode = "vision"
    with patch("main.PLCController") as factory, patch("main.YOLODetector") as detector_factory, \
         patch("main.configure_kinematics"), patch("main.QMessageBox.critical") as dialog:
        plc = factory.return_value
        plc.validate_motion_layout.side_effect = lambda: validate_motion_layout(cfg.plc.offsets)
        plc.read_status.return_value = {}
        detector = detector_factory.return_value
        detector.available_cameras.return_value = [0]
        detector.read_frame.return_value = None
        window = main.RobotApp(cfg)
        try:
            assert not window.page_auto.btn_capture.isEnabled()
            assert not window.page_auto.btn_start.isEnabled()
            assert "byte 20" in window.page_auto.btn_capture.toolTip()
            app.processEvents()
            app.processEvents()
            assert dialog.called
            plc.send_pulse.reset_mock()
            window.start_auto()
            plc.send_pulse.assert_not_called()
        finally:
            window.close()


def test_settings_invalid_vision_change_keeps_live_config_and_file_untouched(cfg):
    from src.ui.page_settings import PageSettings
    values = {
        "yolo.thresh": ".8", "yolo.px2mm": ".5", "yolo.roi_x": "190",
        "yolo.roi_y": "100", "yolo.roi_width": "200", "yolo.roi_height": "190",
        "sort_positions.pick_z_down": "80", "sort_positions.pick_z_up": "150",
        "place_good.z_down": "80", "place_bad.z_down": "80",
        "app.move_cooldown": "1.5", "app.plc_poll_interval": ".1",
        "app.sorting_mode": "vision", "yolo.coordinate_origin": "frame",
    }
    inputs = {}
    for key, value in values.items():
        widget = MagicMock()
        widget.text.return_value = widget.currentText.return_value = value
        kind = "combo" if key in ("app.sorting_mode", "yolo.coordinate_origin") else key.startswith("yolo.roi_")
        inputs[key] = widget, kind
    original = deepcopy(cfg)
    controller = SimpleNamespace(cfg=cfg, _sort_thread=None, _sort_start_lock=threading.Lock())
    controller.validate_vision_configuration = lambda config: validate_motion_layout(config.plc.offsets)
    page = SimpleNamespace(controller=controller, _inputs=inputs)
    with patch("src.ui.page_settings.QMessageBox") as dialog, \
         patch("src.ui.page_settings.save_config") as save:
        PageSettings._save_settings(page)
    save.assert_not_called()
    assert cfg == original
    dialog.critical.assert_called_once()


def test_joint_display_uses_pc_targets_instead_of_xyz_aliases(cfg):
    from src.ui.page_auto import PageAuto
    cfg.app.sorting_mode = "vision"
    sorter = MagicMock()
    sorter.current_joints = (10, 20, 30, 40)
    sorter.state = RobotState.IDLE
    page = MagicMock(spec=PageAuto)
    page.controller = SimpleNamespace(cfg=cfg, sorter=sorter)
    for name in ("lbl_joint_source", "lbl_g_title", "lbl_b_title", "lbl_good", "lbl_bad", "lbl_operation",
                 "lbl_gripper", "lbl_classification", "btn_capture", "lbl_pick_z"):
        setattr(page, name, MagicMock())
    # The remainder of update_gui_data can refresh any label without real Qt widgets.
    PageAuto.update_gui_data(page, {"j1_target": 200, "j2_target": 300, "j3_target": 400})
    assert page._target == [10, 20, 30, 40]
    page.lbl_joint_source.setText.assert_called_with("PC COMMAND TARGETS (deg)")


def test_initial_camera_population_does_not_request_switch(cfg):
    from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox
    from src.ui.base_page import BasePage
    app = QApplication.instance() or QApplication([])
    cfg.camera.default_index = 1
    controller = MagicMock()
    controller.cfg = cfg
    controller.detector.available_cameras.return_value = [0, 1]
    parent = QWidget()
    QVBoxLayout(parent)
    # Use the actual concrete page so the common camera builder is exercised.
    from src.ui.page_auto import PageAuto
    page = PageAuto(parent=parent, controller=controller)
    selector = page.findChild(QComboBox)
    assert selector.currentText() == "Camera 1"
    controller.change_camera_source.assert_not_called()
    selector.setCurrentText("Camera 0")
    controller.change_camera_source.assert_called_once_with("Camera 0")
    page.close()
    parent.close()


def test_decimal_validator_accepts_comma_and_dot():
    from src.ui.validators import DecimalValidator
    from PySide6.QtGui import QValidator
    validator = DecimalValidator(-100, 100, 2)
    assert validator.validate("12,5", 4)[0] == QValidator.State.Acceptable
    assert validator.validate("12.5", 4)[0] == QValidator.State.Acceptable
    assert validator.validate("12,5.2", 6)[0] == QValidator.State.Invalid


def test_real_mock_plc_gui_qr_path_boots_and_accepts_target(cfg):
    import main
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    with patch("main.PLCController", MockPLCController), patch("main.YOLODetector") as factory, \
         patch("main.configure_kinematics"):
        detector = factory.return_value
        detector.available_cameras.return_value = [0]
        detector.read_frame.return_value = None
        window = main.RobotApp(cfg)
        try:
            assert window.sorter.process_qr_target("A1")
            assert window.sorter.qr_written == 1
            assert window.plc.read_status()["motion_done"] is True
            assert window.plc.simulate_box_arrival()
            assert window.plc.read_status()["motion_done"] is False
        finally:
            window.close()
        assert not window._ai_thread.is_alive()
        assert not window._plc_thread.is_alive()
