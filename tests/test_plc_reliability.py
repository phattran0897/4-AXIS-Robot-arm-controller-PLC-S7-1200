"""PLC transaction and sorting lifecycle regressions; no real hardware I/O."""

import threading
from unittest.mock import MagicMock, patch

import pytest
from snap7.util import get_bool, get_real, set_real

from src.config_loader import (
    JointPosition, KinematicsConfig, PLCCommands, PLCConfig, SortPositionsConfig,
)
from src.plc.plc_controller import ADDR, PLCController
from src.robot.sorting_controller import RobotState, SortResult, SortingController


@pytest.fixture
def controller():
    memory = bytearray(160)
    snapshots = []
    with patch("src.plc.plc_controller.snap7.client.Client") as factory:
        client = factory.return_value
        client.get_connected.return_value = True
        client.db_read.side_effect = lambda db, offset, size: memory[offset:offset + size]

        def write(db, offset, data):
            memory[offset:offset + len(data)] = data
            snapshots.append(bytes(memory))

        client.db_write.side_effect = write
        plc = PLCController(PLCConfig())
        yield plc, client, memory, snapshots
        plc.disconnect()


@pytest.fixture
def sorter():
    plc = MagicMock()
    plc.write_joint_targets.return_value = True
    return SortingController(
        plc, SortPositionsConfig(locations={"A1": JointPosition(0, 45, 90, 0)}),
        KinematicsConfig(), PLCCommands(),
    )


def test_notification_failure_does_not_replay_successful_qr_write(controller):
    plc, client, _, _ = controller
    plc.tx_callback = MagicMock(side_effect=RuntimeError("GUI has closed"))
    sorter = SortingController(
        plc, SortPositionsConfig(locations={"A1": JointPosition(0, 45, 90, 0)}),
        KinematicsConfig(), PLCCommands(),
    )
    assert sorter.process_qr_target("A1")
    assert not sorter.process_qr_target("A1")
    assert sorter.qr_written == 1
    assert client.db_write.call_count == 1
    assert plc.is_connected()
    assert plc.write_classification(True)


def test_float32_overflow_never_partially_writes_targets(controller):
    plc, client, _, _ = controller
    offsets = plc._cfg.offsets
    offsets.j1_target, offsets.j2_target, offsets.j3_target = 88, 100, 116
    assert not plc.write_joint_targets(1, 2, 1e100, 0)
    client.db_write.assert_not_called()
    assert plc.is_connected()


@pytest.mark.parametrize("method", ["send_forward_kinematics_data", "send_inverse_kinematics_data"])
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 1e100])
def test_kinematics_payload_is_fully_validated_before_writing(controller, method, bad):
    plc, client, _, _ = controller
    assert getattr(plc, method)(1, 2, 3, 4, 5, bad) is False
    client.db_write.assert_not_called()


@pytest.mark.parametrize("method", ["send_forward_kinematics_data", "send_inverse_kinematics_data"])
def test_kinematics_write_obeys_stop_and_reports_io_failure(controller, method):
    plc, client, _, _ = controller
    assert plc.stop_motion()
    client.db_write.reset_mock()
    assert getattr(plc, method)(1, 2, 3, 4, 5, 6) is False
    client.db_write.assert_not_called()
    assert plc.resume_commands()
    client.db_write.side_effect = RuntimeError("network failure")
    assert getattr(plc, method)(1, 2, 3, 4, 5, 6) is False


def test_split_classification_publishes_flag_after_payload(controller):
    plc, _, memory, snapshots = controller
    offsets = plc._cfg.offsets
    offsets.hang_tot_byte = 90
    offsets.hang_xau_byte = 92
    memory[offsets.phan_loai_hang_byte] = 1 << offsets.phan_loai_hang_bit
    memory[92] = 1 << offsets.hang_xau_bit  # Previous item was BAD.
    assert plc.write_classification(True)
    assert len(snapshots) >= 3
    for snapshot in snapshots[:-1]:
        assert not get_bool(snapshot, offsets.phan_loai_hang_byte, offsets.phan_loai_hang_bit)
    assert get_bool(snapshots[-1], offsets.phan_loai_hang_byte, offsets.phan_loai_hang_bit)
    assert get_bool(snapshots[-1], 90, offsets.hang_tot_bit)
    assert not get_bool(snapshots[-1], 92, offsets.hang_xau_bit)


def test_failed_classification_payload_never_asserts_valid(controller):
    plc, client, memory, _ = controller
    plc._cfg.offsets.hang_tot_byte = 90
    original_write = client.db_write.side_effect

    def fail_payload(db, offset, data):
        if offset == 90:
            raise RuntimeError("payload failed")
        original_write(db, offset, data)

    client.db_write.side_effect = fail_payload
    assert plc.write_classification(True) is False
    assert not get_bool(memory, 78, plc._cfg.offsets.phan_loai_hang_bit)


def test_default_classification_still_uses_one_byte_transaction(controller):
    plc, client, _, _ = controller
    assert plc.write_classification(True)
    assert client.db_write.call_count == 1
    assert plc.clear_classification()
    assert client.db_write.call_count == 2


def test_nonfinite_status_is_not_exposed_as_healthy_telemetry(controller):
    plc, _, memory, _ = controller
    set_real(memory, plc._cfg.offsets.theta_j2, float("nan"))
    assert plc.read_status() == {}


def test_old_pulse_handle_cannot_cancel_new_generation(controller):
    plc, _, memory, _ = controller
    with plc._lock:
        assert plc.send_pulse(*ADDR.START_AUTO)
        old_handle = plc._pulse_timers[ADDR.START_AUTO][0]
        assert plc.send_pulse(*ADDR.START_AUTO)
        generation = plc._pulse_timers[ADDR.START_AUTO][1]
        old_handle.cancel()
        assert plc._pulse_timers[ADDR.START_AUTO][1] == generation
        assert plc._start_pulse(*ADDR.START_AUTO, generation)
        assert memory[0] & 1


def test_disconnect_reconnect_restarts_pulse_worker(controller):
    plc, _, memory, _ = controller
    old_worker = plc._pulse_thread
    plc.disconnect()
    assert not old_worker.is_alive()
    assert not plc.send_pulse(*ADDR.START_AUTO)
    assert plc.connect()
    assert plc._pulse_thread is not old_worker
    assert plc._pulse_thread.is_alive()
    cleared = threading.Event()
    original_write_bit = plc.write_bit

    def observe_write(byte, bit, value):
        result = original_write_bit(byte, bit, value)
        if (byte, bit) == ADDR.START_AUTO and not value and result:
            cleared.set()
        return result

    plc.write_bit = observe_write
    plc.PULSE_WIDTH_S = .01
    assert plc.send_pulse(*ADDR.START_AUTO)
    assert cleared.wait(1)
    assert not memory[0] & 1


def test_disconnect_error_does_not_leave_positive_connection_cache(controller):
    plc, client, _, _ = controller
    assert plc.is_connected()
    client.disconnect.side_effect = RuntimeError("disconnect failed")
    plc.disconnect()
    assert not plc.is_connected()
    assert not plc.send_pulse(*ADDR.START_AUTO)


def test_read_failure_requires_new_transport_even_if_snap7_reports_connected(controller):
    plc, client, _, _ = controller
    client.db_read.side_effect = RuntimeError("socket broken")
    assert plc.read_status() == {}
    assert not plc.is_connected()
    client.db_write.reset_mock()
    assert not plc.write_joint_targets(1, 2, 3, 0)
    client.db_write.assert_not_called()
    assert plc.connect()
    client.disconnect.assert_called_once()
    client.connect.assert_called_once()
    assert plc.is_connected()


def test_combined_command_reports_failure(controller):
    plc, client, _, _ = controller
    client.db_write.side_effect = RuntimeError("offline")
    assert plc.send_joint_targets_and_command(1, 2, 3, 0, plc._cfg.commands.home) is False


def test_qr_cannot_replace_targets_during_an_active_cycle(sorter):
    sorter._begin_cycle()
    assert not sorter.is_idle()
    assert not sorter.process_qr_target("A1")
    sorter.plc.write_joint_targets.assert_not_called()
    sorter._end_cycle()


def test_missing_motion_done_does_not_count_as_busy(sorter):
    sorter.plc.read_status.side_effect = [
        {"error_flag": False}, {"motion_done": True},
        {"motion_done": False}, {"motion_done": True},
    ]
    sorter._wait_event = MagicMock()
    sorter._wait_for_motion(1)
    assert sorter.plc.read_status.call_count == 4


def test_stop_at_home_completion_does_not_publish_success(sorter):
    sorter._pick = MagicMock()
    sorter._place = MagicMock()
    sorter._return_home = sorter.request_stop
    with pytest.raises(RuntimeError, match="cancelled"):
        sorter.execute_sort(150, 100, SortResult.GOOD)
    assert sorter.state is RobotState.ERROR
    assert sorter.counter_good == 0
    assert not sorter.is_idle()


def test_failed_final_cleanup_leaves_error_and_no_success_counter(sorter):
    sorter._pick = MagicMock()
    sorter._place = MagicMock()
    sorter._return_home = MagicMock()
    sorter.plc.clear_classification.return_value = False
    with pytest.raises(RuntimeError, match="clear_classification failed"):
        sorter.execute_sort(150, 100, SortResult.GOOD)
    assert sorter.state is RobotState.ERROR
    assert sorter.counter_good == 0
    sorter.plc.stop_motion.assert_called_once()


def test_failed_command_does_not_replace_displayed_joint_targets(sorter):
    previous = sorter.current_joints
    sorter.plc.send_joint_targets.return_value = False
    with pytest.raises(RuntimeError, match="failed"):
        sorter._move_and_wait(10, 20, 30, 40)
    assert sorter.current_joints == previous
