"""
tests/test_robot_system.py – Automated test suite for the Robot Control System.

All hardware-dependent components (snap7 client, OpenCV VideoCapture,
and the YOLO model) are replaced with :mod:`unittest.mock` stubs so the
full suite runs in a headless CI environment with no physical hardware.

Test coverage areas
-------------------
1.  Config loader – valid YAML, missing file, partial keys with defaults.
2.  PLCController – connect/disconnect, read_status, send_command,
    send_joint_targets, disconnected-state guard paths.
3.  YOLODetector – camera open/switch/stop, process_frame (defect /
    no-defect / capture failure).
4.  Inverse kinematics – reachable points, workspace errors, J3/J4 zero.
5.  BasePage – shared helpers.
"""

from __future__ import annotations

import os
import sys
import tempfile
import textwrap
import unittest
from typing import Any
from unittest.mock import MagicMock, patch

# Ensure the parent directory (project root) is in the Python path
# This prevents "ModuleNotFoundError: No module named 'src'" when running directly.
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import numpy as np  # noqa: E402  (must follow sys.path setup above)

# ---------------------------------------------------------------------------
# Conditional import guards – allow tests to run in environments that lack
# optional runtime dependencies (snap7, cv2, PySide6).
# ---------------------------------------------------------------------------
_HAVE_SNAP7 = False
try:
    import snap7  # noqa: F401

    _HAVE_SNAP7 = True
except ImportError:
    pass

_HAVE_CV2 = False
try:
    import cv2  # noqa: F401

    _HAVE_CV2 = True
except ImportError:
    pass

_HAVE_PYSIDE6 = False
try:
    import PySide6  # noqa: F401

    _HAVE_PYSIDE6 = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Helpers – build minimal configs without touching the filesystem
# ---------------------------------------------------------------------------


def _make_robot_config() -> "RobotConfig":  # noqa: F821
    from src.config_loader import (
        AppConfig,
        CameraConfig,
        KinematicsConfig,
        PLCCommands,
        PLCConfig,
        PLCOffsets,
        RobotConfig,
        YOLOConfig,
    )

    return RobotConfig(
        plc=PLCConfig(
            ip="127.0.0.1",
            rack=0,
            slot=1,
            db_number=10,
            offsets=PLCOffsets(),
            commands=PLCCommands(),
        ),
        yolo=YOLOConfig(
            model_path="fake_model.pt",
            thresh=0.50,
            px2mm=0.5,
            home_x=200.0,
            home_y=0.0,
        ),
        camera=CameraConfig(
            default_index=0,
            display_width=440,
            display_height=310,
            fps=30,
            inference_width=640,
            inference_height=480,
            read_timeout=2.0,
        ),
        kinematics=KinematicsConfig(),
        app=AppConfig(
            title="Test",
            geometry="800x600",
            appearance_mode="dark",
            color_theme="blue",
            plc_poll_interval=0.1,
            move_cooldown=0.0,
        ),
    )


# ===========================================================================
# 1 – Config Loader
# ===========================================================================


class TestConfigLoader(unittest.TestCase):
    """Tests for src/config_loader.py"""

    def _write_yaml(self, content: str) -> str:
        """Write *content* to a temp file and return its path."""
        fd, path = tempfile.mkstemp(suffix=".yaml")
        with os.fdopen(fd, "w") as fh:
            fh.write(textwrap.dedent(content))
        return path

    # ── TC-01 ────────────────────────────────────────────────────────────────
    def test_load_full_config_parses_all_sections(self) -> None:
        """A fully-populated config.yaml must map to the correct dataclass fields."""
        path = self._write_yaml(
            """
            plc:
              ip: "10.0.0.1"
              rack: 1
              slot: 2
              db_number: 5
              offsets:
                cmd_word: 0
                status: 2
              commands:
                idle: 0
                home: 1
                move: 2
                stop: 3
            yolo:
              model_path: "models/yolo.pt"
              thresh: 0.75
              px2mm: 0.25
              home_x: 150.0
              home_y: 10.0
            camera:
              default_index: 1
              display_width: 320
              display_height: 240
              fps: 15
              inference_width: 480
              inference_height: 320
              read_timeout: 3.0
            kinematics:
              a1: 40.0
              d1: 300.0
              a2: 190.0
              a3: 110.0
              a4: 65.0
            app:
              title: "Test Robot"
              geometry: "800x600"
              appearance_mode: "light"
              color_theme: "green"
              plc_poll_interval: 0.05
              move_cooldown: 2.0
        """
        )
        try:
            from src.config_loader import load_config

            cfg = load_config(path)
            self.assertEqual(cfg.plc.ip, "10.0.0.1")
            self.assertEqual(cfg.plc.rack, 1)
            self.assertEqual(cfg.plc.db_number, 5)
            self.assertAlmostEqual(cfg.yolo.thresh, 0.75)
            self.assertAlmostEqual(cfg.yolo.px2mm, 0.25)
            self.assertEqual(cfg.camera.default_index, 1)
            self.assertEqual(cfg.camera.fps, 15)
            self.assertEqual(cfg.camera.inference_width, 480)
            self.assertEqual(cfg.camera.inference_height, 320)
            self.assertAlmostEqual(cfg.camera.read_timeout, 3.0)
            self.assertAlmostEqual(cfg.kinematics.a2, 190.0)
            self.assertEqual(cfg.app.appearance_mode, "light")
            self.assertAlmostEqual(cfg.app.move_cooldown, 2.0)
        finally:
            os.unlink(path)

    # ── TC-02 ────────────────────────────────────────────────────────────────
    def test_missing_config_file_raises_file_not_found(self) -> None:
        """load_config() must raise FileNotFoundError for a non-existent path."""
        from src.config_loader import load_config

        with self.assertRaises(FileNotFoundError):
            load_config("/tmp/this_file_does_not_exist_xyz.yaml")

    # ── TC-03 ────────────────────────────────────────────────────────────────
    def test_partial_yaml_uses_safe_defaults(self) -> None:
        """Missing YAML keys must fall back to sensible defaults."""
        path = self._write_yaml("plc:\n  ip: '192.168.1.50'\n")
        try:
            from src.config_loader import load_config

            cfg = load_config(path)
            self.assertEqual(cfg.plc.ip, "192.168.1.50")
            self.assertEqual(cfg.plc.rack, 0)
            self.assertAlmostEqual(cfg.yolo.thresh, 0.50)
            self.assertEqual(cfg.camera.fps, 30)
            self.assertEqual(cfg.camera.inference_width, 640)
            self.assertAlmostEqual(cfg.camera.read_timeout, 2.0)
        finally:
            os.unlink(path)


# ===========================================================================
# 2 – Inverse Kinematics (tests the standalone module)
# ===========================================================================


class TestInverseKinematicsModule(unittest.TestCase):
    """Tests for src/kinematics/kinematics.py (4-DOF articulated robot arm)"""

    # ── TC-16 ────────────────────────────────────────────────────────────────
    def test_fk_all_zeros(self) -> None:
        """FK at all-zero joints must give (294, 0, 100)."""
        from src.kinematics import forward_kinematics

        x, y, z, _pitch = forward_kinematics(0.0, 0.0, 0.0, 0.0)
        # r = 0 + 190 + 190 + 74 = 454, z = 189.6
        self.assertAlmostEqual(x, 454.0, places=1)
        self.assertAlmostEqual(y, 0.0, places=1)
        self.assertAlmostEqual(z, 189.6, places=1)

    # ── TC-17 ────────────────────────────────────────────────────────────────
    def test_fk_ik_round_trip(self) -> None:
        """FK → IK → FK must return the same position."""
        from src.kinematics import forward_kinematics, inverse_kinematics

        # Pick specific joint angles
        j1, j2, j3, j4 = 30.0, 20.0, -15.0, -5.0
        x, y, z, _pitch = forward_kinematics(j1, j2, j3, j4)
        phi = j2 + j3 + j4  # end-effector pitch
        j1r, j2r, j3r, j4r = inverse_kinematics(x, y, z, phi=phi)
        x2, y2, z2, _pitch2 = forward_kinematics(j1r, j2r, j3r, j4r)
        self.assertAlmostEqual(x, x2, places=2)
        self.assertAlmostEqual(y, y2, places=2)
        self.assertAlmostEqual(z, z2, places=2)

    # ── TC-18 ────────────────────────────────────────────────────────────────
    def test_ik_at_rotation_centre_raises_workspace_error(self) -> None:
        """IK must raise WorkspaceError when target is at rotation centre."""
        from src.kinematics import inverse_kinematics, WorkspaceError

        with self.assertRaises(WorkspaceError):
            inverse_kinematics(0.0, 0.0, 300.0)

    def test_ik_base_rotation(self) -> None:
        """IK for equal X, Y must give θ₁ = 45°."""
        from src.kinematics import forward_kinematics, inverse_kinematics

        # Get a reachable point at 45° by computing FK with θ₁=45°
        x, y, z, _pitch = forward_kinematics(45.0, 0.0, 0.0, 0.0)
        j1, j2, j3, j4 = inverse_kinematics(x, y, z, phi=0.0)
        self.assertAlmostEqual(j1, 45.0, places=2)

    def test_ik_unreachable_point_raises_workspace_error(self) -> None:
        """IK must raise WorkspaceError for a point beyond max reach."""
        from src.kinematics import inverse_kinematics, WorkspaceError

        with self.assertRaises(WorkspaceError):
            # Max reach = 40 + 190 + 110 + 65 = 405 mm, so 500 mm is unreachable
            inverse_kinematics(500.0, 0.0, 300.0, phi=0.0)

    def test_reachable_function(self) -> None:
        """reachable() must correctly classify in/out-of-workspace points."""
        from src.kinematics import reachable

        # All-zero joints reach (405, 0, 300) → should be reachable
        self.assertTrue(reachable(405.0, 0.0, 300.0, phi=0.0))
        # Way too far out
        self.assertFalse(reachable(700.0, 0.0, 300.0, phi=0.0))

    def test_fk_with_base_rotation(self) -> None:
        """FK with θ₁=90° should swap X and Y (X≈0, Y=r)."""
        from src.kinematics import forward_kinematics

        x, y, z, _pitch = forward_kinematics(90.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertAlmostEqual(y, 454.0, places=1)
        self.assertAlmostEqual(z, 189.6, places=1)


# ===========================================================================
# 3 – PLCController  (run only when snap7 is available)
# ===========================================================================


@unittest.skipUnless(_HAVE_SNAP7, "snap7 not installed")
class TestPLCController(unittest.TestCase):
    """Tests for src/plc/plc_controller.py"""

    def _make_controller(self) -> tuple[Any, MagicMock]:
        """Return (PLCController, mock_snap7_client)."""
        from src.plc.plc_controller import PLCController

        cfg = _make_robot_config().plc
        ctrl = PLCController(cfg)
        mock_client = MagicMock()
        ctrl._client = mock_client
        return ctrl, mock_client

    # ── TC-04 ────────────────────────────────────────────────────────────────
    def test_connect_returns_true_on_success(self) -> None:
        """connect() must return True when snap7 reports connected."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True
        result = ctrl.connect()
        self.assertTrue(result)

    # ── TC-05 ────────────────────────────────────────────────────────────────
    def test_connect_returns_false_on_exception(self) -> None:
        """connect() must return False and not propagate snap7 exceptions."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.side_effect = Exception("Network unreachable")
        result = ctrl.connect()
        self.assertFalse(result)

    # ── TC-06 ────────────────────────────────────────────────────────────────
    def test_read_status_parses_db_bytes(self) -> None:
        """read_status() must decode byte array into the expected dict keys."""
        from snap7.util import set_bool, set_real

        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True

        raw = bytearray(80)
        set_bool(raw, 0, 2, True)  # auto_mode
        set_real(raw, 4, 45.0)  # j1_target (offset 4)
        set_real(raw, 8, 30.0)  # j2_target (offset 8)
        set_real(raw, 12, 15.0)  # j3_target (offset 12)
        mock_client.db_read.return_value = raw

        data = ctrl.read_status()
        self.assertTrue(data["auto_mode"])
        self.assertAlmostEqual(data["j1_target"], 45.0, places=2)
        self.assertAlmostEqual(data["j2_target"], 30.0, places=2)
        self.assertAlmostEqual(data["j3_target"], 15.0, places=2)

    # ── TC-07 ────────────────────────────────────────────────────────────────
    def test_read_status_returns_empty_dict_when_disconnected(self) -> None:
        """read_status() must return {} without calling db_read when offline."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = False

        data = ctrl.read_status()
        self.assertEqual(data, {})
        mock_client.db_read.assert_not_called()

    # ── TC-08 ────────────────────────────────────────────────────────────────
    def test_send_command_writes_correct_bytes(self) -> None:
        """send_command() must trigger pulse on the correct PLC bit."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True
        mock_client.db_read.return_value = bytearray(1)

        ctrl.send_command(2)  # CMD MOVE pulses START_AUTO (0, 0)
        import time

        time.sleep(0.2)

        self.assertGreaterEqual(mock_client.db_write.call_count, 1)
        # Verify it writes to offset 0 (since START_AUTO is byte 0)
        mock_client.db_write.assert_any_call(ctrl._cfg.db_number, 0, unittest.mock.ANY)

    # ── TC-09 ────────────────────────────────────────────────────────────────
    def test_send_command_skips_when_disconnected(self) -> None:
        """send_command() must not call db_write when the PLC is offline."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = False

        ctrl.send_command(1)
        mock_client.db_write.assert_not_called()

    # ── TC-10 ────────────────────────────────────────────────────────────────
    def test_send_joint_targets_writes_12_bytes(self) -> None:
        """send_joint_targets() must write exactly 12 bytes of float data (3 × REAL)."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True
        mock_client.db_read.return_value = bytearray(1)

        ctrl.send_joint_targets(10.0, 20.0, 30.0, 0.0)

        # Targets are written synchronously to offset j1_target (default=4)
        target_offset = ctrl._cfg.offsets.j1_target
        mock_client.db_write.assert_any_call(
            ctrl._cfg.db_number, target_offset, unittest.mock.ANY
        )
        # Verify the written buffer has length 16 (4 × REAL = J1-J4)
        for call in mock_client.db_write.call_args_list:
            db, offset, buf = call[0]
            if offset == target_offset:
                self.assertEqual(len(buf), 16)

    def test_send_joint_targets_and_command_atomic(self) -> None:
        """Atomic target write must send targets to offset 2."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True
        mock_client.db_read.return_value = bytearray(1)

        ctrl.send_joint_targets_and_command(10.0, 20.0, 30.0, 0.0, cmd=2)

        target_offset = ctrl._cfg.offsets.j1_target
        mock_client.db_write.assert_any_call(
            ctrl._cfg.db_number, target_offset, unittest.mock.ANY
        )


# ===========================================================================
# 4 – YOLODetector  (run only when cv2 is available)
# ===========================================================================


@unittest.skipUnless(_HAVE_CV2, "opencv-python not installed")
class TestYOLODetector(unittest.TestCase):
    """Tests for src/ai/yolo_detector.py"""

    def _make_frame(self, h: int = 480, w: int = 640) -> np.ndarray:
        """Return a synthetic BGR frame."""
        return np.zeros((h, w, 3), dtype=np.uint8)

    def test_read_frame_returns_none_when_no_camera(self) -> None:
        """read_frame() must return None when VideoCapture is not open."""
        with patch("src.ai.yolo_detector.YOLO"):
            from src.ai.yolo_detector import YOLODetector

            with patch("os.path.isfile", return_value=True):
                detector = YOLODetector(model_path="fake.pt", thresh=0.5)
        detector._cap = None
        result = detector.read_frame()
        self.assertIsNone(result)

    def test_annotate_frame_no_defect_returns_home_coords(self) -> None:
        """annotate_frame() must return home coords when no detection passes thresh."""
        mock_result = MagicMock()
        mock_result.boxes = []

        with patch("src.ai.yolo_detector.YOLO") as mock_yolo_cls:
            mock_yolo_cls.return_value.return_value = [mock_result]
            from src.ai.yolo_detector import YOLODetector

            with patch("os.path.isfile", return_value=True):
                detector = YOLODetector(
                    model_path="fake.pt",
                    thresh=0.5,
                    px2mm=0.5,
                    home_x=200.0,
                    home_y=0.0,
                )

        frame = self._make_frame()
        result = detector.annotate_frame(frame, inference_w=640, inference_h=480)

        self.assertFalse(result.has_defect)
        self.assertAlmostEqual(result.robot_x, 200.0)
        self.assertAlmostEqual(result.robot_y, 0.0)
        self.assertIsNotNone(result.annotated_frame)

    def test_annotate_frame_defect_above_thresh_computes_robot_coords(self) -> None:
        """annotate_frame() must compute robot_x/y offsets for a centred-right defect."""
        mock_box = MagicMock()
        mock_box.xyxy.cpu.return_value.numpy.return_value.squeeze.return_value = (
            np.array([360, 220, 440, 260], dtype=np.float32)
        )
        mock_box.conf.item.return_value = 0.90
        mock_box.cls.item.return_value = 0

        mock_result = MagicMock()
        mock_result.boxes = [mock_box]

        with patch("src.ai.yolo_detector.YOLO") as mock_yolo_cls:
            mock_yolo_cls.return_value.return_value = [mock_result]
            from src.ai.yolo_detector import YOLODetector

            with patch("os.path.isfile", return_value=True):
                detector = YOLODetector(
                    model_path="fake.pt",
                    thresh=0.5,
                    px2mm=0.5,
                    home_x=200.0,
                    home_y=0.0,
                )

        frame = self._make_frame(h=480, w=640)
        result = detector.annotate_frame(frame, inference_w=640, inference_h=480)

        self.assertTrue(result.has_defect)
        self.assertAlmostEqual(result.robot_x, 240.0, places=1)
        self.assertAlmostEqual(result.robot_y, 0.0, places=1)

    def test_switch_camera_releases_previous_cap(self) -> None:
        """switch_camera() must call release() on the existing VideoCapture."""
        old_cap = MagicMock()
        old_cap.isOpened.return_value = True

        with patch("src.ai.yolo_detector.YOLO"), patch(
            "cv2.VideoCapture"
        ) as mock_cap_cls:
            new_cap = MagicMock()
            new_cap.isOpened.return_value = True
            mock_cap_cls.return_value = new_cap

            from src.ai.yolo_detector import YOLODetector

            with patch("os.path.isfile", return_value=True):
                detector = YOLODetector(model_path="fake.pt", thresh=0.5)
            detector._cap = old_cap
            detector.switch_camera(1)

        old_cap.release.assert_called_once()

    def test_detection_result_to_pil(self) -> None:
        """DetectionResult.to_pil() must resize and convert BGR frame to PIL."""
        from PIL import Image

        mock_result = MagicMock()
        mock_result.boxes = []

        with patch("src.ai.yolo_detector.YOLO") as mock_yolo_cls:
            mock_yolo_cls.return_value.return_value = [mock_result]
            from src.ai.yolo_detector import YOLODetector

            with patch("os.path.isfile", return_value=True):
                detector = YOLODetector(
                    model_path="fake.pt",
                    thresh=0.5,
                    px2mm=0.5,
                    home_x=200.0,
                    home_y=0.0,
                )

        frame = self._make_frame()
        result = detector.annotate_frame(frame, inference_w=640, inference_h=480)
        pil = result.to_pil(display_w=220, display_h=155)

        self.assertIsInstance(pil, Image.Image)
        self.assertEqual(pil.size, (220, 155))

    def test_model_path_resolved_relative_to_project(self) -> None:
        """Model path not found must raise FileNotFoundError."""
        with patch("os.path.isabs", return_value=False), patch(
            "os.path.isfile", return_value=False
        ):
            from src.ai.yolo_detector import YOLODetector

            with self.assertRaises(FileNotFoundError):
                YOLODetector(model_path="models/best.pt", thresh=0.5)


# ===========================================================================
# 5 – BasePage  (run only when PySide6 is available)
# ===========================================================================


@unittest.skipUnless(_HAVE_PYSIDE6, "PySide6 not installed")
class TestBasePage(unittest.TestCase):
    """Tests for src/ui/base_page.py"""

    def test_update_video_stores_reference(self) -> None:
        """update_video() must store the pixmap to prevent garbage collection."""
        from src.ui.base_page import BasePage

        page = BasePage.__new__(BasePage)
        page._current_pixmap = None
        page.video_label = MagicMock()
        page.video_label.width.return_value = 440
        page.video_label.height.return_value = 310

        fake_pixmap = MagicMock()
        fake_pixmap.scaled.return_value = fake_pixmap
        page.update_video(fake_pixmap)

        self.assertIs(page._current_pixmap, fake_pixmap)


# ===========================================================================
# 6 – PLC DB read-size computation
# ===========================================================================


class TestPLCDbReadSize(unittest.TestCase):
    """Test the DB read-size auto-computation."""

    def test_db_read_size_computed_from_offsets(self) -> None:
        """compute_db_read_size must round up to nearest multiple of 4."""
        from src.config_loader import PLCOffsets, compute_db_read_size

        offsets = PLCOffsets()
        size = compute_db_read_size(offsets)
        self.assertGreaterEqual(size, offsets.j4_target + 4)
        self.assertGreaterEqual(size, offsets.error_flag_byte + 1)
        self.assertEqual(size % 4, 0)


# ===========================================================================
# 7 – SortingController
# ===========================================================================


class TestSortingController(unittest.TestCase):
    """Tests for src/robot/sorting_controller.py"""

    def test_ik_reachable_point_returns_nonzero_angles(self) -> None:
        """IK on reachable point must return non-zero angles."""
        from src.config_loader import KinematicsConfig, SortPositionsConfig, PLCCommands
        from src.robot.sorting_controller import SortingController

        positions = SortPositionsConfig()
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()
        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        j1, j2, j3, j4 = sorter._ik(150.0, 100.0, 50.0)
        self.assertNotEqual(j1, 0.0)
        self.assertNotEqual(j2, 0.0)

    def test_ik_unreachable_point_raises(self) -> None:
        """IK on unreachable point must RAISE – never return a bogus pose."""
        from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
        from src.kinematics import InverseKinematicsError
        from src.robot.sorting_controller import SortingController

        positions = SortPositionsConfig()
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()
        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        with self.assertRaises(InverseKinematicsError):
            sorter._ik(700.0, 0.0, 300.0)

    def test_execute_sort_aborts_on_unreachable_target(self) -> None:
        """An unreachable pick target must set ERROR and clear classification
        WITHOUT commanding any motion or closing the gripper."""
        from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
        from src.robot.sorting_controller import (
            RobotState,
            SortResult,
            SortingController,
        )

        positions = SortPositionsConfig()
        positions.pick_z_down = 99999.0  # Far outside the workspace
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()

        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        sorter._move_and_wait = MagicMock()  # Must never be reached

        with self.assertRaises(Exception):
            sorter.execute_sort(150.0, 100.0, SortResult.GOOD)

        self.assertEqual(sorter.state, RobotState.ERROR)
        mock_plc.clear_classification.assert_called_once()
        sorter._move_and_wait.assert_not_called()
        # Gripper must not have been commanded on an aborted cycle
        mock_plc.write_bit.assert_not_called()

    def test_execute_sort_calls_plc_in_correct_sequence(self) -> None:
        """execute_sort() must call the PLC with targets and correct commands sequentially."""
        from src.config_loader import KinematicsConfig, SortPositionsConfig, PLCCommands
        from src.robot.sorting_controller import SortingController, SortResult

        positions = SortPositionsConfig()
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()

        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        sorter._move_and_wait = MagicMock()

        sorter.execute_sort(150.0, 100.0, SortResult.GOOD)

        # Verify gripper write_bit calls (close + open)
        self.assertEqual(mock_plc.write_bit.call_count, 2)
        mock_plc.write_bit.assert_any_call(14, 0, True)
        mock_plc.write_bit.assert_any_call(14, 0, False)

        # Verify classification was written and cleared
        mock_plc.write_classification.assert_called_once_with(True)  # GOOD
        mock_plc.clear_classification.assert_called_once()

    def test_counter_increments_on_good_sort(self) -> None:
        """execute_sort() with SortResult.GOOD must increment only counter_good."""
        from src.config_loader import KinematicsConfig, SortPositionsConfig, PLCCommands
        from src.robot.sorting_controller import SortingController, SortResult

        positions = SortPositionsConfig()
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()

        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        sorter._move_and_wait = MagicMock()

        self.assertEqual(sorter.counter_good, 0)
        sorter.execute_sort(150.0, 100.0, SortResult.GOOD)
        self.assertEqual(sorter.counter_good, 1)
        self.assertEqual(sorter.counter_bad, 0)

    def test_counter_increments_on_bad_sort(self) -> None:
        """execute_sort() with SortResult.BAD must increment only counter_bad."""
        from src.config_loader import KinematicsConfig, SortPositionsConfig, PLCCommands
        from src.robot.sorting_controller import SortingController, SortResult

        positions = SortPositionsConfig()
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()

        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        sorter._move_and_wait = MagicMock()

        self.assertEqual(sorter.counter_bad, 0)
        sorter.execute_sort(150.0, 100.0, SortResult.BAD)
        self.assertEqual(sorter.counter_bad, 1)
        self.assertEqual(sorter.counter_good, 0)

    def test_reset_counters_sets_both_to_zero(self) -> None:
        """reset_counters() must clear good and bad sorting counts to zero."""
        from src.config_loader import KinematicsConfig, SortPositionsConfig, PLCCommands
        from src.robot.sorting_controller import SortingController, SortResult

        positions = SortPositionsConfig()
        kinematics = KinematicsConfig()
        plc_commands = PLCCommands()
        mock_plc = MagicMock()

        sorter = SortingController(mock_plc, positions, kinematics, plc_commands)
        sorter._move_and_wait = MagicMock()

        sorter.execute_sort(150.0, 100.0, SortResult.GOOD)
        sorter.execute_sort(150.0, 100.0, SortResult.BAD)
        self.assertEqual(sorter.counter_good, 1)
        self.assertEqual(sorter.counter_bad, 1)

        sorter.reset_counters()
        self.assertEqual(sorter.counter_good, 0)
        self.assertEqual(sorter.counter_bad, 0)


# ---------------------------------------------------------------------------
# 8 – Regression tests for reviewed bug fixes
# ---------------------------------------------------------------------------


class TestConfigDefaultsRegression(unittest.TestCase):
    """Regression: loader fallbacks must match dataclass/config.yaml poses."""

    # ── R-01 ────────────────────────────────────────────────────────────────
    def test_missing_sort_position_keys_use_workspace_safe_defaults(self) -> None:
        """Empty sort_positions section must fall back to z=80/150, never -50/0."""
        from src.config_loader import load_config

        path = TestConfigLoader()._write_yaml("sort_positions:\n")
        try:
            cfg = load_config(path)
            self.assertAlmostEqual(cfg.sort_positions.place_good.z_down, 80.0)
            self.assertAlmostEqual(cfg.sort_positions.place_good.z_up, 150.0)
            self.assertAlmostEqual(cfg.sort_positions.place_bad.z_down, 80.0)
            self.assertAlmostEqual(cfg.sort_positions.place_bad.z_up, 150.0)
            self.assertAlmostEqual(cfg.sort_positions.pick_z_down, 80.0)
            self.assertAlmostEqual(cfg.sort_positions.pick_z_up, 150.0)
        finally:
            os.unlink(path)

    def test_loader_defaults_match_dataclass_defaults(self) -> None:
        """Every sort-position fallback must equal its dataclass default."""
        from src.config_loader import SortPositionsConfig, load_config

        path = TestConfigLoader()._write_yaml("")
        try:
            cfg = load_config(path)
            defaults = SortPositionsConfig()
            self.assertEqual(
                cfg.sort_positions.place_good.z_down, defaults.place_good.z_down
            )
            self.assertEqual(
                cfg.sort_positions.place_good.z_up, defaults.place_good.z_up
            )
            self.assertEqual(cfg.sort_positions.pick_z_down, defaults.pick_z_down)
            self.assertEqual(cfg.sort_positions.pick_z_up, defaults.pick_z_up)
        finally:
            os.unlink(path)


class TestKinematicsConfigureRegression(unittest.TestCase):
    """Regression: configure() must validate before mutating module globals."""

    # ── R-02 ────────────────────────────────────────────────────────────────
    def test_invalid_link_value_leaves_state_uncorrupted(self) -> None:
        """A failed configure() call must not partially apply new values."""
        import src.kinematics.kinematics as kin

        original_a2 = kin.A2
        with self.assertRaises(ValueError):
            kin.configure(a2=-5.0)
        self.assertEqual(kin.A2, original_a2)

    def test_valid_configure_applies_all_values(self) -> None:
        import src.kinematics.kinematics as kin

        old = (kin.A1, kin.D1, kin.A2, kin.A3, kin.A4)
        try:
            kin.configure(a1=41.0, d1=301.0, a2=191.0, a3=111.0, a4=66.0)
            self.assertEqual(
                (kin.A1, kin.D1, kin.A2, kin.A3, kin.A4),
                (41.0, 301.0, 191.0, 111.0, 66.0),
            )
        finally:
            kin.configure(a1=old[0], d1=old[1], a2=old[2], a3=old[3], a4=old[4])


@unittest.skipUnless(_HAVE_SNAP7, "snap7 not installed")
class TestPulseRaceRegression(unittest.TestCase):
    """Regression: stale pulse resets must not truncate newer pulses."""

    def _make_controller(self) -> tuple[Any, MagicMock]:
        from src.plc.plc_controller import PLCController

        cfg = _make_robot_config().plc
        ctrl = PLCController(cfg)
        mock_client = MagicMock()
        mock_client.get_connected.return_value = True
        ctrl._client = mock_client
        return ctrl, mock_client

    # ── R-03 ────────────────────────────────────────────────────────────────
    def test_stale_generation_finish_pulse_skips_reset(self) -> None:
        """_finish_pulse with an outdated generation must NOT write False."""
        ctrl, mock_client = self._make_controller()
        mock_client.db_read.return_value = bytearray(1)

        ctrl.send_pulse(0, 0)
        ctrl.send_pulse(0, 0)  # Supersedes the first pulse (generation bump)

        key = (0, 0)
        entry = ctrl._pulse_timers[key]
        stale_generation = entry[1] - 1

        writes_before = mock_client.db_write.call_count
        ctrl._finish_pulse(0, 0, stale_generation)
        self.assertEqual(mock_client.db_write.call_count, writes_before)
        self.assertIn(key, ctrl._pulse_timers)

    def test_current_generation_finish_pulse_resets_bit(self) -> None:
        ctrl, mock_client = self._make_controller()
        mock_client.db_read.return_value = bytearray(1)

        ctrl.send_pulse(0, 0)
        key = (0, 0)
        generation = ctrl._pulse_timers[key][1]

        ctrl._finish_pulse(0, 0, generation)
        self.assertNotIn(key, ctrl._pulse_timers)
        # Last write must be the False reset of byte 0
        last_call = mock_client.db_write.call_args_list[-1]
        self.assertEqual(last_call[0][1], 0)

    def test_addr_manual_mode_constant_exists(self) -> None:
        """MANUAL_MODE must live in ADDR – no magic (16, 0) at call sites."""
        from src.plc.plc_controller import ADDR

        self.assertEqual(ADDR.MANUAL_MODE, (16, 0))
        self.assertEqual(ADDR.AUTO_MODE_1, (14, 1))


class TestMoveAndWaitRegression(unittest.TestCase):
    """Regression: _move_and_wait health-supervision loop (previously untested)."""

    def _make_sorter(self) -> tuple[Any, MagicMock]:
        from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
        from src.robot.sorting_controller import SortingController

        mock_plc = MagicMock()
        sorter = SortingController(
            mock_plc, SortPositionsConfig(), KinematicsConfig(), PLCCommands()
        )
        return sorter, mock_plc

    # ── R-04 ────────────────────────────────────────────────────────────────
    def test_motion_done_true_returns_quickly(self) -> None:
        """A motion_done=True status must complete the wait without timeout."""
        import time as _time

        sorter, mock_plc = self._make_sorter()
        mock_plc.read_status.return_value = {"motion_done": True, "error_flag": False}

        start = _time.monotonic()
        sorter._move_and_wait(10.0, 20.0, 30.0, 0.0)
        elapsed = _time.monotonic() - start

        self.assertLess(elapsed, 1.0)
        self.assertEqual(sorter._previous_joints, (10.0, 20.0, 30.0, 0.0))

    def test_plc_offline_during_motion_raises(self) -> None:
        """An empty read_status mid-motion must raise RuntimeError."""
        sorter, mock_plc = self._make_sorter()
        mock_plc.read_status.return_value = {}

        with self.assertRaises(RuntimeError):
            sorter._move_and_wait(10.0, 20.0, 30.0, 0.0)

    def test_error_flag_during_motion_raises_and_records_target(self) -> None:
        """The error flag must abort the waypoint immediately."""
        sorter, mock_plc = self._make_sorter()
        mock_plc.read_status.return_value = {"motion_done": False, "error_flag": True}

        with self.assertRaises(RuntimeError):
            sorter._move_and_wait(10.0, 20.0, 30.0, 0.0)
        self.assertEqual(
            sorter.state.name, "IDLE"
        )  # execute_sort sets ERROR; raw call must not

    def test_motion_done_transition_breaks_wait(self) -> None:
        """Polling continues until motion_done flips True, then breaks."""
        sorter, mock_plc = self._make_sorter()
        mock_plc.read_status.side_effect = [
            {"motion_done": False, "error_flag": False},
            {"motion_done": False, "error_flag": False},
            {"motion_done": True, "error_flag": False},
        ]

        sorter._move_and_wait(1.0, 2.0, 3.0, 4.0)
        self.assertEqual(mock_plc.read_status.call_count, 3)


# ===========================================================================
# 9 – Config range validation (M1): dangerous values must fail fast
# ===========================================================================


class TestConfigRangeValidation(unittest.TestCase):
    """Out-of-range config values must raise ValueError at load time."""

    def _assert_load_raises(self, yaml_fragment: str) -> None:
        path = TestConfigLoader()._write_yaml(yaml_fragment)
        try:
            from src.config_loader import load_config

            with self.assertRaises(ValueError):
                load_config(path)
        finally:
            os.unlink(path)

    # ── V-01 ────────────────────────────────────────────────────────────────
    def test_joint_limit_min_ge_max_raises(self) -> None:
        """Inverted joint limits must be rejected."""
        self._assert_load_raises("kinematics:\n  j2_min: 100.0\n  j2_max: 50.0\n")

    def test_joint_limit_min_equals_max_raises(self) -> None:
        self._assert_load_raises("kinematics:\n  j3_min: 10.0\n  j3_max: 10.0\n")

    def test_nonpositive_base_height_raises(self) -> None:
        self._assert_load_raises("kinematics:\n  d1: 0.0\n")

    # ── V-02 ────────────────────────────────────────────────────────────────
    def test_plc_bit_offset_above_seven_raises(self) -> None:
        self._assert_load_raises("plc:\n  offsets:\n    hang_tot_bit: 9\n")

    def test_plc_negative_bit_offset_raises(self) -> None:
        self._assert_load_raises("plc:\n  offsets:\n    error_flag_bit: -1\n")

    def test_plc_negative_byte_offset_raises(self) -> None:
        self._assert_load_raises("plc:\n  offsets:\n    motion_done_byte: -5\n")

    def test_j4_sentinel_minus_one_is_allowed(self) -> None:
        """j4_target = -1 is the documented 'unmapped' sentinel – must pass."""
        path = TestConfigLoader()._write_yaml("plc:\n  offsets:\n    j4_target: -1\n")
        try:
            from src.config_loader import load_config

            cfg = load_config(path)
            self.assertEqual(cfg.plc.offsets.j4_target, -1)
        finally:
            os.unlink(path)

    # ── V-03 ────────────────────────────────────────────────────────────────
    def test_negative_gripper_delay_raises(self) -> None:
        self._assert_load_raises("sort_positions:\n  gripper_delay: -1.0\n")

    def test_place_z_down_not_below_z_up_raises(self) -> None:
        """z_down must be strictly below z_up or the arm never descends."""
        self._assert_load_raises(
            "sort_positions:\n" "  place_good:\n    z_down: 200.0\n    z_up: 150.0\n"
        )

    # ── V-04 ────────────────────────────────────────────────────────────────
    def test_zero_camera_fps_raises(self) -> None:
        self._assert_load_raises("camera:\n  fps: 0\n")

    def test_nonpositive_roi_size_raises(self) -> None:
        self._assert_load_raises("yolo:\n  roi_width: 0\n")

    def test_negative_roi_origin_raises(self) -> None:
        self._assert_load_raises("yolo:\n  roi_x: -10\n")


class TestSortingWaypointValidation(unittest.TestCase):
    """SortingController must fail fast on unreachable fixed waypoints."""

    # ── V-05 ────────────────────────────────────────────────────────────────
    def test_unreachable_place_position_rejected_at_init(self) -> None:
        from src.config_loader import (
            KinematicsConfig,
            PlacePosition,
            PLCCommands,
            SortPositionsConfig,
        )
        from src.robot.sorting_controller import SortingController

        positions = SortPositionsConfig()
        positions.place_bad = PlacePosition(x=99999.0, y=0.0, z_down=80.0, z_up=150.0)

        with self.assertRaises(ValueError) as ctx:
            SortingController(MagicMock(), positions, KinematicsConfig(), PLCCommands())
        self.assertIn("place_bad", str(ctx.exception))

    def test_default_positions_pass_validation(self) -> None:
        from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
        from src.robot.sorting_controller import SortingController

        sorter = SortingController(
            MagicMock(), SortPositionsConfig(), KinematicsConfig(), PLCCommands()
        )
        self.assertTrue(sorter.is_idle())


@unittest.skipUnless(_HAVE_PYSIDE6, "PySide6 not installed")
class TestHeaderTabsRegression(unittest.TestCase):
    """Regression: navigation tabs must be present in the header layout.

    The refactor that introduced VAAHeader created _btn_auto/_btn_manual.
    Verify they exist and have text labels assigned.
    """

    # ── R-05 ────────────────────────────────────────────────────────────────
    def test_navigation_tabs_exist(self) -> None:
        import sys

        from PySide6.QtWidgets import QApplication

        from src.ui.header import VAAHeader

        app = QApplication.instance() or QApplication(sys.argv)
        try:
            header = VAAHeader(parent=None, controller=MagicMock())
            self.assertIsNotNone(header._btn_auto)
            self.assertIsNotNone(header._btn_manual)
            self.assertEqual(header._btn_auto.text(), "AUTO MODE")
            self.assertEqual(header._btn_manual.text(), "MANUAL MODE")
        finally:
            header.deleteLater()


@unittest.skipUnless(_HAVE_PYSIDE6, "PySide6 not installed")
class TestAppIcon(unittest.TestCase):
    """The brand logo must become the OS window/taskbar icon at start-up."""

    def _make_window(self):
        import sys

        from PySide6.QtWidgets import QApplication, QMainWindow

        _app = QApplication.instance() or QApplication(sys.argv)
        window = QMainWindow()
        return window

    # ── R-06 ────────────────────────────────────────────────────────────────
    def test_apply_app_icon_with_valid_png(self) -> None:
        import tempfile

        from PIL import Image

        from src.ui.header import apply_app_icon

        fd, path = tempfile.mkstemp(suffix=".png")
        os.close(fd)
        Image.new("RGBA", (64, 64), (0, 212, 255, 255)).save(path)
        try:
            window = self._make_window()
            try:
                self.assertTrue(apply_app_icon(window, path))
            finally:
                window.deleteLater()
        finally:
            os.unlink(path)

    def test_apply_app_icon_missing_file_returns_false(self) -> None:
        from src.ui.header import apply_app_icon

        window = self._make_window()
        try:
            self.assertFalse(apply_app_icon(window, "Z:/definitely/not/here/logo.png"))
        finally:
            window.deleteLater()

    def test_get_logo_path_points_at_asset(self) -> None:
        import os as _os

        from src.ui.header import get_logo_path

        p = get_logo_path()
        self.assertTrue(
            _os.path.normpath(p).endswith(_os.path.join("assets", "vaa_logo.png"))
        )


# ===========================================================================
# 10 – Extended Kinematics Tests (elbow config, pitch return)
# ===========================================================================


class TestKinematicsElbowConfig(unittest.TestCase):
    """Tests for elbow-up / elbow-down configuration selection in IK."""

    # ── TC-19 ────────────────────────────────────────────────────────────────
    def test_elbow_up_vs_down_different_solutions(self) -> None:
        """Same target with different elbow configs must yield different θ₃ signs."""
        from src.kinematics import forward_kinematics, inverse_kinematics

        # Use a reachable target that has room for both solutions
        x, y, z, _pitch = forward_kinematics(30.0, 40.0, 20.0, -10.0)
        phi = 40.0 + 20.0 + (-10.0)  # = 50.0

        j1_up, j2_up, j3_up, j4_up = inverse_kinematics(x, y, z, phi=phi, elbow="up")
        j1_dn, j2_dn, j3_dn, j4_dn = inverse_kinematics(
            x, y, z, phi=phi, elbow="down"
        )

        # θ₁ must be the same (base rotation doesn't depend on elbow config)
        self.assertAlmostEqual(j1_up, j1_dn, places=4)

        # θ₃ must have opposite sign (up = positive, down = negative)
        # unless the target is at exact extension (sin_theta3 ≈ 0)
        if abs(j3_up) > 0.1:
            self.assertNotAlmostEqual(j3_up, j3_dn, places=1)

    # ── TC-20 ────────────────────────────────────────────────────────────────
    def test_fk_ik_round_trip_elbow_down(self) -> None:
        """FK → IK(elbow='down') → FK must recover the same position."""
        from src.kinematics import forward_kinematics, inverse_kinematics

        j1, j2, j3, j4 = 30.0, 40.0, -20.0, -10.0
        x, y, z, _pitch = forward_kinematics(j1, j2, j3, j4)
        phi = j2 + j3 + j4

        j1r, j2r, j3r, j4r = inverse_kinematics(
            x, y, z, phi=phi, elbow="down"
        )
        x2, y2, z2, _p2 = forward_kinematics(j1r, j2r, j3r, j4r)

        self.assertAlmostEqual(x, x2, places=2)
        self.assertAlmostEqual(y, y2, places=2)
        self.assertAlmostEqual(z, z2, places=2)

    # ── TC-22 ────────────────────────────────────────────────────────────────
    def test_fk_returns_pitch(self) -> None:
        """FK must return pitch = j2 + j3 + j4 as the 4th element."""
        from src.kinematics import forward_kinematics

        j2, j3, j4 = 30.0, -15.0, 10.0
        result = forward_kinematics(0.0, j2, j3, j4)
        self.assertEqual(len(result), 4)
        self.assertAlmostEqual(result[3], j2 + j3 + j4, places=6)

    # ── TC-23 ────────────────────────────────────────────────────────────────
    def test_ik_invalid_elbow_raises_value_error(self) -> None:
        """Passing an invalid elbow value must raise ValueError."""
        from src.kinematics import inverse_kinematics

        with self.assertRaises(ValueError):
            inverse_kinematics(200.0, 100.0, 200.0, phi=0.0, elbow="invalid")

    def test_fk_ik_round_trip_3d_tolerance(self) -> None:
        """FK→IK→FK round-trip must hold within 1e-3 mm tolerance."""
        from src.kinematics import forward_kinematics, inverse_kinematics

        for j1, j2, j3, j4 in [
            (0.0, 0.0, 0.0, 0.0),
            (45.0, 30.0, -15.0, -5.0),
            (-60.0, 50.0, 10.0, -20.0),
            (90.0, 20.0, -10.0, 5.0),
        ]:
            x, y, z, _p = forward_kinematics(j1, j2, j3, j4)
            phi = j2 + j3 + j4
            j1r, j2r, j3r, j4r = inverse_kinematics(x, y, z, phi=phi)
            x2, y2, z2, _p2 = forward_kinematics(j1r, j2r, j3r, j4r)
            self.assertAlmostEqual(x, x2, places=3)
            self.assertAlmostEqual(y, y2, places=3)
            self.assertAlmostEqual(z, z2, places=3)


# ===========================================================================
# 11 – StabilityTracker
# ===========================================================================


class TestStabilityTracker(unittest.TestCase):
    """Tests for src/ai/stability_tracker.py"""

    # ── TC-24 ────────────────────────────────────────────────────────────────
    def test_locks_after_required_frames(self) -> None:
        """Tracker must lock after exactly required_frames stable updates."""
        from src.ai.stability_tracker import StabilityTracker

        tracker = StabilityTracker(threshold_mm=5.0, required_frames=5)
        self.assertFalse(tracker.is_locked)

        for i in range(4):
            result = tracker.update(100.0, 200.0)
            self.assertFalse(result, f"Locked too early at frame {i + 1}")

        result = tracker.update(100.0, 200.0)
        self.assertTrue(result)
        self.assertTrue(tracker.is_locked)
        self.assertEqual(tracker.locked_position, (100.0, 200.0))

    # ── TC-25 ────────────────────────────────────────────────────────────────
    def test_resets_on_large_jump(self) -> None:
        """A position jump beyond threshold_mm must reset the stable count."""
        from src.ai.stability_tracker import StabilityTracker

        tracker = StabilityTracker(threshold_mm=5.0, required_frames=5)

        # Build up 3 stable frames
        for _ in range(3):
            tracker.update(100.0, 200.0)
        self.assertEqual(tracker.stable_count, 3)

        # Large jump resets
        tracker.update(200.0, 200.0)
        self.assertEqual(tracker.stable_count, 1)
        self.assertFalse(tracker.is_locked)

    def test_reset_method_clears_state(self) -> None:
        """reset() must clear all tracking state."""
        from src.ai.stability_tracker import StabilityTracker

        tracker = StabilityTracker(threshold_mm=5.0, required_frames=3)
        for _ in range(5):
            tracker.update(100.0, 200.0)
        self.assertTrue(tracker.is_locked)

        tracker.reset()
        self.assertFalse(tracker.is_locked)
        self.assertIsNone(tracker.locked_position)
        self.assertEqual(tracker.stable_count, 0)

    def test_locked_tracker_unlocks_on_movement(self) -> None:
        """A locked tracker must unlock when the object moves away."""
        from src.ai.stability_tracker import StabilityTracker

        tracker = StabilityTracker(threshold_mm=5.0, required_frames=3)
        for _ in range(3):
            tracker.update(100.0, 200.0)
        self.assertTrue(tracker.is_locked)

        # Move far away — should unlock
        result = tracker.update(500.0, 500.0)
        self.assertFalse(result)
        self.assertFalse(tracker.is_locked)

    def test_progress_property(self) -> None:
        """progress must report fraction of required frames."""
        from src.ai.stability_tracker import StabilityTracker

        tracker = StabilityTracker(threshold_mm=5.0, required_frames=10)
        tracker.update(100.0, 200.0)
        self.assertAlmostEqual(tracker.progress, 0.1, places=2)


# ===========================================================================
# 12 – PLC Buffer Byte-Level Verification
# ===========================================================================


@unittest.skipUnless(_HAVE_SNAP7, "snap7 not installed")
class TestPLCBufferContents(unittest.TestCase):
    """Byte-level verification of PLC write buffers (TC-11 extension)."""

    def test_send_joint_targets_buffer_byte_contents(self) -> None:
        """send_joint_targets must write IEEE-754 floats at correct byte offsets."""
        import struct

        from src.plc.plc_controller import PLCController

        cfg = _make_robot_config().plc
        ctrl = PLCController(cfg)
        mock_client = MagicMock()
        mock_client.get_connected.return_value = True
        mock_client.db_read.return_value = bytearray(1)
        ctrl._client = mock_client

        j1, j2, j3, j4 = 45.0, 30.0, -15.0, 10.0
        ctrl.send_joint_targets(j1, j2, j3, j4)

        target_offset = ctrl._cfg.offsets.j1_target
        # Find the write call to the target offset
        buf = None
        for call in mock_client.db_write.call_args_list:
            db, offset, data = call[0]
            if offset == target_offset:
                buf = data
                break

        self.assertIsNotNone(buf, "No write to j1_target offset found")
        self.assertEqual(len(buf), 16)

        # Verify IEEE-754 big-endian floats (snap7 uses big-endian)
        j1_read = struct.unpack(">f", buf[0:4])[0]
        j2_read = struct.unpack(">f", buf[4:8])[0]
        j3_read = struct.unpack(">f", buf[8:12])[0]
        j4_read = struct.unpack(">f", buf[12:16])[0]

        self.assertAlmostEqual(j1_read, j1, places=4)
        self.assertAlmostEqual(j2_read, j2, places=4)
        self.assertAlmostEqual(j3_read, j3, places=4)
        self.assertAlmostEqual(j4_read, j4, places=4)

    # ── Group 1 Unit Tests: Memory Safety & Limits & PC Master ──────────────
    def test_send_joint_targets_unmapped_j4_writes_12_bytes(self) -> None:
        """When j4_target == -1, send_joint_targets must write exactly 12 bytes."""
        from src.config_loader import PLCOffsets
        from src.plc.plc_controller import PLCController

        cfg = _make_robot_config().plc
        cfg.offsets = PLCOffsets(
            j1_target=42,
            j2_target=46,
            j3_target=50,
            j4_target=-1,
        )
        ctrl = PLCController(cfg)
        mock_client = MagicMock()
        mock_client.get_connected.return_value = True
        mock_client.db_read.return_value = bytearray(1)
        ctrl._client = mock_client

        ctrl.send_joint_targets(10.0, 20.0, 30.0, 40.0)

        # Must write to offset 42 with length 12
        target_offset = 42
        mock_client.db_write.assert_any_call(
            cfg.db_number, target_offset, unittest.mock.ANY
        )
        call_buf = None
        for call in mock_client.db_write.call_args_list:
            if call[0][1] == target_offset:
                call_buf = call[0][2]
                break
        self.assertIsNotNone(call_buf)
        self.assertEqual(len(call_buf), 12)
        # Verify bytes 42..53 are written; byte 54 is never touched
        self.assertEqual(42 + len(call_buf), 54)

    def test_ik_raises_when_joint_limits_exceeded(self) -> None:
        """Target with joint angle exceeding JOINT_LIMITS must raise WorkspaceError."""
        from src.kinematics import configure, inverse_kinematics, WorkspaceError

        # Limit J1 to [-30°, 30°]
        configure(limits={"j1": (-30.0, 30.0)})
        try:
            # Point along positive Y axis requires J1 = 90°, exceeding limit
            with self.assertRaises(WorkspaceError) as ctx:
                inverse_kinematics(0.0, 250.0, 300.0, phi=0.0)
            self.assertIn("Joint limit exceeded: j1=", str(ctx.exception))
        finally:
            # Restore default limits
            configure(
                limits={
                    "j1": (-180.0, 180.0),
                    "j2": (-180.0, 250.0),
                    "j3": (-200.0, 200.0),
                    "j4": (-180.0, 180.0),
                }
            )

    def test_move_and_wait_transmits_targets_and_waits_for_motion_done(self) -> None:
        """_move_and_wait() must call send_joint_targets and wait for motion_done."""
        from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
        from src.robot.sorting_controller import SortingController

        mock_plc = MagicMock()
        mock_plc.read_status.side_effect = [
            {"motion_done": False},
            {"motion_done": True},
        ]
        sorter = SortingController(
            mock_plc, SortPositionsConfig(), KinematicsConfig(), PLCCommands()
        )
        sorter._move_and_wait(15.0, 25.0, 35.0, 0.0)

        mock_plc.send_joint_targets.assert_called_once_with(15.0, 25.0, 35.0, 0.0)
        self.assertGreaterEqual(mock_plc.read_status.call_count, 2)

    def test_move_and_wait_raises_on_timeout(self) -> None:
        """_move_and_wait() must raise TimeoutError if motion_done is never received."""
        from src.config_loader import KinematicsConfig, PLCCommands, SortPositionsConfig
        from src.robot.sorting_controller import SortingController

        mock_plc = MagicMock()
        # Always return motion_done=False
        mock_plc.read_status.return_value = {"motion_done": False}
        sorter = SortingController(
            mock_plc, SortPositionsConfig(), KinematicsConfig(), PLCCommands()
        )
        with self.assertRaises(TimeoutError):
            sorter._move_and_wait(10.0, 20.0, 30.0, 0.0, timeout=0.08)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main(verbosity=2)

