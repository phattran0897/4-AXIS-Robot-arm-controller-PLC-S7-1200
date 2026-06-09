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

import math
import os
import sys
import tempfile
import textwrap
import threading
import unittest
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np

# ---------------------------------------------------------------------------
# Conditional import guards – allow tests to run in environments that lack
# optional runtime dependencies (snap7, cv2, customtkinter).
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

_HAVE_CUSTOMTKINTER = False
try:
    import customtkinter  # noqa: F401
    _HAVE_CUSTOMTKINTER = True
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
        kinematics=KinematicsConfig(l1=200.0, l2=150.0),
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
        path = self._write_yaml("""
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
              l1: 180.0
              l2: 120.0
            app:
              title: "Test Robot"
              geometry: "800x600"
              appearance_mode: "light"
              color_theme: "green"
              plc_poll_interval: 0.05
              move_cooldown: 2.0
        """)
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
            self.assertAlmostEqual(cfg.kinematics.l1, 180.0)
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
    """Tests for src/kinematics/kinematics.py"""

    # ── TC-16 ────────────────────────────────────────────────────────────────
    def test_ik_home_position_produces_nonzero_angles(self) -> None:
        """IK for a reachable point must not return all zeros."""
        from src.kinematics import inverse_kinematics
        j1, j2, j3, j4 = inverse_kinematics(200.0, 100.0, l1=200.0, l2=150.0)
        self.assertFalse(j1 == 0.0 and j2 == 0.0)
        self.assertAlmostEqual(j3, 0.0)
        self.assertAlmostEqual(j4, 0.0)

    # ── TC-17 ────────────────────────────────────────────────────────────────
    def test_ik_out_of_reach_raises_workspace_error(self) -> None:
        """IK must raise WorkspaceError for out-of-reach targets (no silent clamp)."""
        from src.kinematics import inverse_kinematics, WorkspaceError
        with self.assertRaises(WorkspaceError):
            inverse_kinematics(1000.0, 1000.0, l1=200.0, l2=150.0)

    # ── TC-18 ────────────────────────────────────────────────────────────────
    def test_ik_j3_j4_always_zero(self) -> None:
        """IK must always return 0.0 for J3 and J4 (reserved axes)."""
        from src.kinematics import inverse_kinematics
        for x, y in [(100, 50), (200, 0), (0, 150), (50, 300)]:
            _, _, j3, j4 = inverse_kinematics(x, y, l1=200.0, l2=150.0)
            self.assertAlmostEqual(j3, 0.0, msg=f"J3 non-zero for ({x},{y})")
            self.assertAlmostEqual(j4, 0.0, msg=f"J4 non-zero for ({x},{y})")

    def test_ik_negative_link_length_raises(self) -> None:
        """IK must reject negative or zero link lengths."""
        from src.kinematics import inverse_kinematics
        with self.assertRaises(ValueError):
            inverse_kinematics(100.0, 50.0, l1=-200.0, l2=150.0)

    def test_ik_edge_of_workspace_reaches_elbow(self) -> None:
        """IK at maximum reach (l1+l2) must not raise."""
        from src.kinematics import inverse_kinematics
        j1, j2, j3, j4 = inverse_kinematics(350.0, 0.0, l1=200.0, l2=150.0)
        self.assertAlmostEqual(j1, 0.0, places=2)
        self.assertAlmostEqual(j2, 0.0, places=2)
        self.assertAlmostEqual(j3, 0.0)
        self.assertAlmostEqual(j4, 0.0)

    def test_ik_at_minimum_reach(self) -> None:
        """IK at minimum reach (|l1-l2|) must not raise."""
        from src.kinematics import inverse_kinematics
        j1, j2, j3, j4 = inverse_kinematics(50.0, 0.0, l1=200.0, l2=150.0)
        self.assertAlmostEqual(j1, 0.0, places=2)
        self.assertAlmostEqual(j2, 180.0, places=2)
        self.assertAlmostEqual(j3, 0.0)
        self.assertAlmostEqual(j4, 0.0)

    def test_reachable_function(self) -> None:
        """reachable() must correctly classify in/out-of-workspace points."""
        from src.kinematics import reachable
        self.assertTrue(reachable(200.0, 0.0, l1=200.0, l2=150.0))
        self.assertTrue(reachable(100.0, 50.0, l1=200.0, l2=150.0))
        self.assertTrue(reachable(350.0, 0.0, l1=200.0, l2=150.0))
        self.assertTrue(reachable(50.0, 0.0, l1=200.0, l2=150.0))
        self.assertFalse(reachable(400.0, 0.0, l1=200.0, l2=150.0))
        self.assertFalse(reachable(20.0, 0.0, l1=200.0, l2=150.0))


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
        from snap7.util import set_int, set_real
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True

        raw = bytearray(24)
        set_int(raw, 0, 2)
        set_int(raw, 2, 1)
        set_real(raw, 4, 45.0)
        set_real(raw, 8, 30.0)
        set_real(raw, 12, 0.0)
        set_real(raw, 16, 0.0)
        mock_client.db_read.return_value = raw

        data = ctrl.read_status()
        self.assertEqual(data["cmd_word"], 2)
        self.assertAlmostEqual(data["j1_target"], 45.0, places=2)
        self.assertAlmostEqual(data["j2_target"], 30.0, places=2)

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
        """send_command() must write a 2-byte buffer with the command integer."""
        from snap7.util import get_int
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True

        ctrl.send_command(2)

        mock_client.db_write.assert_called_once()
        _db, _offset, buf = mock_client.db_write.call_args[0]
        self.assertEqual(get_int(buf, 0), 2)

    # ── TC-09 ────────────────────────────────────────────────────────────────
    def test_send_command_skips_when_disconnected(self) -> None:
        """send_command() must not call db_write when the PLC is offline."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = False

        ctrl.send_command(1)
        mock_client.db_write.assert_not_called()

    # ── TC-10 ────────────────────────────────────────────────────────────────
    def test_send_joint_targets_writes_16_bytes(self) -> None:
        """send_joint_targets() must write exactly 16 bytes (4 × REAL)."""
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True

        ctrl.send_joint_targets(10.0, 20.0, 30.0, 40.0)

        mock_client.db_write.assert_called_once()
        _db, _offset, buf = mock_client.db_write.call_args[0]
        self.assertEqual(len(buf), 16)

    def test_send_joint_targets_and_command_atomic(self) -> None:
        """Atomic write must include both joints and command in one db_write."""
        from snap7.util import get_int, get_real
        ctrl, mock_client = self._make_controller()
        mock_client.get_connected.return_value = True

        ctrl.send_joint_targets_and_command(10.0, 20.0, 30.0, 40.0, cmd=2)

        mock_client.db_write.assert_called_once()
        _db, _offset, buf = mock_client.db_write.call_args[0]
        self.assertEqual(get_int(buf, 0), 2)
        self.assertAlmostEqual(get_real(buf, 4), 10.0)
        self.assertAlmostEqual(get_real(buf, 8), 20.0)
        self.assertAlmostEqual(get_real(buf, 12), 30.0)
        self.assertAlmostEqual(get_real(buf, 16), 40.0)


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
                    model_path="fake.pt", thresh=0.5, px2mm=0.5,
                    home_x=200.0, home_y=0.0,
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

        mock_result = MagicMock()
        mock_result.boxes = [mock_box]

        with patch("src.ai.yolo_detector.YOLO") as mock_yolo_cls:
            mock_yolo_cls.return_value.return_value = [mock_result]
            from src.ai.yolo_detector import YOLODetector
            with patch("os.path.isfile", return_value=True):
                detector = YOLODetector(
                    model_path="fake.pt", thresh=0.5, px2mm=0.5,
                    home_x=200.0, home_y=0.0,
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

        with patch("src.ai.yolo_detector.YOLO"), \
             patch("cv2.VideoCapture") as mock_cap_cls:
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
                    model_path="fake.pt", thresh=0.5, px2mm=0.5,
                    home_x=200.0, home_y=0.0,
                )

        frame = self._make_frame()
        result = detector.annotate_frame(frame, inference_w=640, inference_h=480)
        pil = result.to_pil(display_w=220, display_h=155)

        self.assertIsInstance(pil, Image.Image)
        self.assertEqual(pil.size, (220, 155))

    def test_model_path_resolved_relative_to_project(self) -> None:
        """Model path not found must raise FileNotFoundError."""
        with patch("os.path.isabs", return_value=False), \
             patch("os.path.isfile", return_value=False):
            from src.ai.yolo_detector import YOLODetector
            with self.assertRaises(FileNotFoundError):
                YOLODetector(model_path="models/best.pt", thresh=0.5)


# ===========================================================================
# 5 – BasePage  (run only when customtkinter is available)
# ===========================================================================

@unittest.skipUnless(_HAVE_CUSTOMTKINTER, "customtkinter not installed")
class TestBasePage(unittest.TestCase):
    """Tests for src/ui/base_page.py"""

    def test_update_video_stores_reference(self) -> None:
        """update_video() must store the PhotoImage to prevent garbage collection."""
        from src.ui.base_page import BasePage

        page = BasePage.__new__(BasePage)
        page._current_tk_image = None
        page.video_label = MagicMock()

        fake_tk = MagicMock()
        page.update_video(fake_tk)

        self.assertIs(page._current_tk_image, fake_tk)
        page.video_label.configure.assert_called_once_with(image=fake_tk)


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


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main(verbosity=2)
