"""Operator-facing regressions tested with Qt widgets and no hardware."""

import threading
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import MagicMock, call

import pytest
from PySide6.QtCore import QEvent, QPoint
from PySide6.QtGui import QHelpEvent
from PySide6.QtWidgets import QApplication, QLabel, QToolTip

from src.config_loader import load_config
from src.robot.sorting_controller import RobotState
from src.ui.page_auto import PageAuto
from src.ui.page_manual import PageManual
from src.ui.page_settings import PageSettings


@pytest.fixture
def controller():
    cfg = load_config()
    plc = MagicMock()
    detector = MagicMock()
    detector.available_cameras.return_value = [0]
    sorter = MagicMock()
    sorter.state = RobotState.IDLE
    return SimpleNamespace(
        cfg=cfg, plc=plc, detector=detector, sorter=sorter,
        _sort_thread=None, _sort_start_lock=threading.Lock(),
        _classification_running=threading.Event(),
        log_tx=MagicMock(), stop_robot=MagicMock(), clear_all_errors=MagicMock(),
        reset_sorting_counters=MagicMock(), start_auto=MagicMock(),
        manual_classify=MagicMock(), change_camera_source=MagicMock(),
    )


@pytest.fixture
def dialogs(monkeypatch):
    manual = MagicMock()
    settings = MagicMock()
    monkeypatch.setattr("src.ui.page_manual.messagebox", manual)
    monkeypatch.setattr("src.ui.page_settings.QMessageBox", settings)
    return manual, settings


@pytest.fixture
def manual(qtbot, controller, dialogs):
    page = PageManual(controller=controller)
    qtbot.addWidget(page)
    return page


def test_editing_fk_input_invalidates_values_before_send(manual, dialogs):
    manual.ent_j1.setText("30")
    manual.ent_j2.setText("45")
    manual.ent_j3.setText("30")
    manual.ent_j4.setText("0")
    manual._calculate_forward()
    assert manual._last_fk_result is not None
    manual.ent_j1.setText("40")
    assert manual._last_fk_result is None
    assert manual.lbl_fk_x.text() == "--- mm"
    manual._send_forward_kinematics()
    manual.controller.plc.send_forward_kinematics_data.assert_not_called()
    dialogs[0].showwarning.assert_called_once()


def test_ik_input_and_elbow_changes_invalidate_previous_solution(manual):
    manual._last_ik_result = (10, 20, 30, 0)
    manual._last_ik_coords = (150, 150, 100)
    manual.ent_x.setText("160")
    assert manual._last_ik_result is None
    manual._last_ik_result = (10, 20, 30, 0)
    manual.controller.cfg.kinematics.j3_min = -180
    manual._set_elbow("down")
    assert manual._last_ik_result is None


@pytest.mark.parametrize("direction", ["forward", "inverse"])
def test_failed_plc_send_never_displays_success(manual, dialogs, direction):
    prefix = "fk" if direction == "forward" else "ik"
    setattr(manual, f"_last_{prefix}_result", (30, 45, 30, 0))
    setattr(manual, f"_last_{prefix}_coords", (150, 150, 100))
    method = getattr(manual.controller.plc, f"send_{direction}_kinematics_data")
    method.return_value = False
    getattr(manual, f"_send_{direction}_kinematics")()
    dialogs[0].showerror.assert_called_once()
    dialogs[0].showinfo.assert_not_called()
    manual.controller.log_tx.assert_not_called()


def test_failed_jog_write_keeps_ui_off(manual, dialogs):
    manual.controller.plc.write_bit.return_value = False
    manual._toggle_jog((80, 0), "LEFT")
    assert not manual._jog_states[(80, 0)]
    dialogs[0].showerror.assert_called_once()


def test_jog_reversal_clears_opposite_before_setting_new_bit(manual):
    manual._toggle_jog((80, 0), "LEFT")
    manual.controller.plc.write_bit.reset_mock()
    manual._toggle_jog((80, 1), "RIGHT")
    assert manual.controller.plc.write_bit.call_args_list == [call(80, 0, False), call(80, 1, True)]
    assert not manual._jog_states[(80, 0)]
    assert manual._jog_states[(80, 1)]


def test_failed_opposite_release_never_asserts_new_direction(manual):
    manual._toggle_jog((80, 0), "LEFT")
    manual.controller.plc.write_bit.reset_mock()
    manual.controller.plc.write_bit.return_value = False
    manual._toggle_jog((80, 1), "RIGHT")
    manual.controller.plc.write_bit.assert_called_once_with(80, 0, False)
    assert not manual._jog_states[(80, 1)]


def test_window_deactivation_releases_active_jog(manual):
    manual._toggle_jog((80, 2), "UP")
    manual.event(QEvent(QEvent.Type.WindowDeactivate))
    manual.controller.plc.write_bits.assert_called_once_with(80, {i: False for i in range(6)})
    assert not any(manual._jog_states.values())


def test_failed_jog_release_requests_stop(manual):
    manual._toggle_jog((80, 4), "UP")
    manual.controller.plc.write_bits.return_value = False
    assert not manual.release_jog()
    manual.controller.stop_robot.assert_called_once()


@pytest.mark.parametrize("value", ["nan", "inf", "-inf"])
def test_manual_numbers_must_be_finite(value):
    with pytest.raises(ValueError):
        PageManual._entry_float(SimpleNamespace(text=lambda: value))


def test_roi_edits_and_failed_save_do_not_change_live_configuration(qtbot, controller, dialogs, monkeypatch):
    page = PageSettings(controller=controller)
    qtbot.addWidget(page)
    original = deepcopy(controller.cfg)
    controller._sort_thread = MagicMock()
    controller._sort_thread.is_alive.return_value = True
    page._inputs["yolo.roi_x"][0].setText("20")
    page._inputs["yolo.roi_width"][0].setText("400")
    assert controller.cfg == original
    save = MagicMock()
    monkeypatch.setattr("src.ui.page_settings.save_config", save)
    page._save_settings()
    save.assert_not_called()
    assert controller.cfg == original


def test_settings_cannot_apply_during_inference(qtbot, controller, dialogs, monkeypatch):
    page = PageSettings(controller=controller)
    qtbot.addWidget(page)
    controller._classification_running.set()
    save = MagicMock()
    monkeypatch.setattr("src.ui.page_settings.save_config", save)
    page._save_settings()
    save.assert_not_called()
    assert "classification" in dialogs[1].critical.call_args.args[2]


def test_settings_blank_field_is_not_silently_saved_as_zero(qtbot, controller, dialogs, monkeypatch):
    page = PageSettings(controller=controller)
    qtbot.addWidget(page)
    page._inputs["app.move_cooldown"][0].clear()
    save = MagicMock()
    monkeypatch.setattr("src.ui.page_settings.save_config", save)
    page._save_settings()
    save.assert_not_called()
    assert "app.move_cooldown" in dialogs[1].critical.call_args.args[2]


def test_settings_status_tracks_offline_and_local_error(qtbot, controller, dialogs):
    page = PageSettings(controller=controller)
    qtbot.addWidget(page)
    page.update_gui_data({"connected": False})
    assert "OFFLINE" in page.lbl_err_status.text()
    controller.sorter.state = RobotState.ERROR
    page.update_gui_data({"connected": True, "error_flag": False})
    assert "ERROR" in page.lbl_err_status.text()


def test_auto_disconnection_hides_stale_joint_values(qtbot, controller):
    page = PageAuto(controller=controller)
    qtbot.addWidget(page)
    page.update_gui_data({"connected": False})
    page._interpolate_gui_numbers()
    assert "OFFLINE" in page.lbl_operation.text()
    assert "UNKNOWN" in page.lbl_gripper.text()
    assert page.lbl_j1.text() == "--- °"


def test_hover_help_is_available_on_settings_names_and_inputs(qtbot, controller):
    page = PageSettings(controller=controller)
    qtbot.addWidget(page)
    page.show()
    for key, (control, _) in page._inputs.items():
        labels = control.parentWidget().findChildren(QLabel)
        assert labels, key
        assert control.toolTip(), key
        assert labels[0].toolTip() == control.toolTip(), key
    label = page._inputs["yolo.roi_width"][0].parentWidget().findChild(QLabel)
    point = QPoint(5, 5)
    try:
        QApplication.sendEvent(label, QHelpEvent(QEvent.Type.ToolTip, point, label.mapToGlobal(point)))
        assert QToolTip.isVisible()
        assert "pixel" in QToolTip.text()
        controller.plc.write_bit.assert_not_called()
        controller.plc.send_pulse.assert_not_called()
    finally:
        QToolTip.hideText()


def test_disabled_control_can_explain_why_it_is_unavailable(manual):
    manual.show()
    manual._fk_frame.hide()
    manual._ik_frame.show()
    button = manual.btn_elbow_down
    assert not button.isEnabled()
    point = QPoint(5, 5)
    try:
        QApplication.sendEvent(button, QHelpEvent(QEvent.Type.ToolTip, point, button.mapToGlobal(point)))
        assert QToolTip.isVisible()
        assert "J3" in QToolTip.text()
        assert "bị khóa" in QToolTip.text()
        manual.controller.plc.send_pulse.assert_not_called()
    finally:
        QToolTip.hideText()


def test_plc_refresh_preserves_help_and_clears_only_temporary_notice(qtbot, controller):
    from main import RobotApp

    page = PageAuto(controller=controller)
    qtbot.addWidget(page)
    original = page.btn_start.toolTip()
    state = SimpleNamespace(
        cfg=controller.cfg, page_auto=page, sorter=controller.sorter,
        _closing=False, _latest_plc_data={"connected": True},
        validate_vision_configuration=MagicMock(side_effect=ValueError("byte 20 <conflict>")),
    )
    RobotApp.refresh_sorting_controls(state)
    assert not page.btn_start.isEnabled()
    assert original in page.btn_start.toolTip()
    assert "byte 20 &lt;conflict&gt;" in page.btn_start.toolTip()
    state.validate_vision_configuration.side_effect = None
    RobotApp.refresh_sorting_controls(state)
    RobotApp.refresh_sorting_controls(state)
    assert page.btn_start.isEnabled()
    assert page.btn_start.toolTip() == original
    state._latest_plc_data = {"connected": False}
    RobotApp.refresh_sorting_controls(state)
    assert not page.btn_start.isEnabled()
    assert "ngoại tuyến" in page.btn_start.toolTip()
    assert original in page.btn_start.toolTip()


def test_pick_height_readout_tracks_applied_settings_even_when_offline(qtbot, controller):
    page = PageAuto(controller=controller)
    qtbot.addWidget(page)
    controller.cfg.sort_positions.pick_z_down = 75.5
    page.update_gui_data({"connected": False})
    assert page.lbl_pick_z.text() == "75.5 mm"
