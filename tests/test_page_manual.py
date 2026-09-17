"""
tests/test_page_manual.py – Unit tests for PageManual UI logic.

Tests JOG toggle state management, button visual feedback, joint validation,
and TX telemetry logging. All tests use mock objects — no Qt window required.
"""

from __future__ import annotations

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

# Ensure project root is on sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


# ---------------------------------------------------------------------------
# Helper: build a minimal PageManual instance without Tk
# ---------------------------------------------------------------------------


def _make_page_manual():
    """
    Construct a PageManual-like object with only the logic attributes
    initialised (no Qt widgets). We use MagicMock with spec to avoid
    instantiating the full QWidget hierarchy.
    """
    import PySide6  # noqa: F401

    from src.ui.page_manual import PageManual

    # Use a simple namespace object to avoid QWidget __init__
    page = MagicMock(spec=PageManual)

    # Restore the real methods we want to test
    page._toggle_jog = PageManual._toggle_jog.__get__(page)
    page._validate_joints = PageManual._validate_joints.__get__(page)

    # ── Mock controller with PLC ─────────────────────────────────────────
    page.controller = MagicMock()
    page.controller.plc = MagicMock()
    page.controller.log_tx = MagicMock()
    page.controller.cfg = MagicMock()
    page.controller.cfg.kinematics.default_phi = 0.0

    # ── JOG state & button dicts (same as __init__) ─────────────────────
    page._jog_states = {
        (80, 0): False,
        (80, 1): False,
        (80, 2): False,
        (80, 3): False,
        (80, 4): False,
        (80, 5): False,
    }
    page._jog_buttons = {}
    for addr in page._jog_states:
        page._jog_buttons[addr] = MagicMock()  # mock QPushButton

    # ── Kinematics mode ─────────────────────────────────────────────────
    page._kine_mode = "Forward"
    page._last_fk_result = None
    page._last_ik_result = None
    page._last_fk_coords = None
    page._last_ik_coords = None

    return page


# ===========================================================================
# Test Suite
# ===========================================================================


class TestJogToggle(unittest.TestCase):
    """Tests for the JOG toggle mechanism on PageManual."""

    def setUp(self):
        self.page = _make_page_manual()

    # ── TC-01 ─────────────────────────────────────────────────────────────
    def test_jog_toggle_on(self):
        """First press on a JOG button must set state True and call write_bit(True)."""
        addr = (80, 0)
        self.page._toggle_jog(addr, "◀ QUAY TRÁI")

        self.assertTrue(self.page._jog_states[addr])
        self.page.controller.plc.write_bit.assert_called_once_with(80, 0, True)

    # ── TC-02 ─────────────────────────────────────────────────────────────
    def test_jog_toggle_off(self):
        """Second press must flip state back to False and call write_bit(False)."""
        addr = (80, 0)
        # First press → ON
        self.page._toggle_jog(addr, "◀ QUAY TRÁI")
        # Second press → OFF
        self.page._toggle_jog(addr, "◀ QUAY TRÁI")

        self.assertFalse(self.page._jog_states[addr])
        self.page.controller.plc.write_bit.assert_called_with(80, 0, False)

    # ── TC-03 ─────────────────────────────────────────────────────────────
    def test_jog_toggle_all_addresses(self):
        """All 6 JOG addresses (80.0-80.5) must toggle independently."""
        addrs = [(80, i) for i in range(6)]
        for addr in addrs:
            self.page._toggle_jog(addr, f"btn_{addr[1]}")

        # All should be ON
        for addr in addrs:
            self.assertTrue(self.page._jog_states[addr])

        # Toggle only the first one off
        self.page._toggle_jog(addrs[0], "btn_0")
        self.assertFalse(self.page._jog_states[addrs[0]])

        # Others still ON
        for addr in addrs[1:]:
            self.assertTrue(self.page._jog_states[addr])

    # ── TC-04 ─────────────────────────────────────────────────────────────
    def test_jog_button_text_on(self):
        """Button must have setText called with '[ON]' suffix when toggled ON."""
        addr = (80, 2)
        self.page._toggle_jog(addr, "▲ LÊN")

        btn = self.page._jog_buttons[addr]
        btn.setText.assert_called()
        text_arg = btn.setText.call_args[0][0]
        self.assertIn("[ON]", text_arg)

    # ── TC-05 ─────────────────────────────────────────────────────────────
    def test_jog_button_text_off(self):
        """Button must have setText called with '[OFF]' suffix when toggled OFF."""
        addr = (80, 3)
        self.page._toggle_jog(addr, "XUỐNG ▼")  # ON
        self.page._jog_buttons[addr].setText.reset_mock()
        self.page._toggle_jog(addr, "XUỐNG ▼")  # OFF

        btn = self.page._jog_buttons[addr]
        text_arg = btn.setText.call_args[0][0]
        self.assertIn("[OFF]", text_arg)

    # ── TC-06 ─────────────────────────────────────────────────────────────
    def test_toggle_logs_to_tx(self):
        """JOG toggle must call controller.log_tx() for TX telemetry."""
        addr = (80, 4)
        self.page._toggle_jog(addr, "▲ LÊN")

        self.page.controller.log_tx.assert_called_once()
        msg = self.page.controller.log_tx.call_args[0][0]
        self.assertIn("ON", msg)
        self.assertIn("80.4", msg)


class TestJointValidation(unittest.TestCase):
    """Tests for _validate_joints on PageManual."""

    def setUp(self):
        self.page = _make_page_manual()

    # ── TC-07 ─────────────────────────────────────────────────────────────
    @patch("src.ui.page_manual.messagebox")
    def test_validate_joints_valid(self, mock_msgbox):
        """Valid joint angles must pass validation and return True."""
        result = self.page._validate_joints(90.0, 45.0, 30.0, 0.0)
        self.assertTrue(result)
        mock_msgbox.showerror.assert_not_called()

    # ── TC-08 ─────────────────────────────────────────────────────────────
    @patch("src.ui.page_manual.messagebox")
    def test_validate_joints_j1_out_of_range(self, mock_msgbox):
        """J1 > 180° must fail validation."""
        result = self.page._validate_joints(200.0, 45.0, 30.0, 0.0)
        self.assertFalse(result)
        mock_msgbox.showerror.assert_called_once()

    # ── TC-09 ─────────────────────────────────────────────────────────────
    @patch("src.ui.page_manual.messagebox")
    def test_validate_joints_j2_out_of_range(self, mock_msgbox):
        """J2 > 250° must fail validation."""
        result = self.page._validate_joints(0.0, 300.0, 30.0, 0.0)
        self.assertFalse(result)
        mock_msgbox.showerror.assert_called_once()


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main(verbosity=2)
