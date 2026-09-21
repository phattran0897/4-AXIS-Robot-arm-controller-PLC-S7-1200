"""
src/ui/page_manual.py – Manual (operator-driven) operation page (PySide6).

Provides direct joint-angle entry (forward / inverse kinematics toggle),
gripper control buttons, and a synchronised camera feed.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from src.kinematics import (
    JOINT_LIMITS,
    WorkspaceError,
    forward_kinematics,
    inverse_kinematics,
    validate_joint_angles,
)
from src.plc.plc_controller import ADDR
from src.ui.base_page import BasePage
from src.ui.theme import (
    ACCENT,
    CARD_BG,
    DANGER,
    DANGER_HOVER,
    ENTRY_BG,
    INFO,
    MANUAL_ACCENT,
    PANEL_BORDER,
    ROW_BG,
    SUCCESS,
    SUCCESS_HOVER,
    TEXT_PRIMARY,
    TEXT_SECONDARY,
    btn_style,
    card_style,
)

if TYPE_CHECKING:
    from main import RobotApp

log = logging.getLogger(__name__)


class messagebox:
    """Compatibility shim delegating to QMessageBox for runtime and test patching."""

    @staticmethod
    def showerror(title: str, message: str, parent: QWidget | None = None) -> None:
        QMessageBox.critical(parent, title, message)

    @staticmethod
    def showwarning(title: str, message: str, parent: QWidget | None = None) -> None:
        QMessageBox.warning(parent, title, message)

    @staticmethod
    def showinfo(title: str, message: str, parent: QWidget | None = None) -> None:
        QMessageBox.information(parent, title, message)


class PageManual(BasePage):
    """
    Manual-mode page with modern professional UI.

    Layout
    ------
    ┌─────────────────────────────────────────────────────────────────┐
    │  Column 0              │  Column 1           │  Column 2       │
    │  Kinematics Input       │  Actuator Control   │  Camera Feed   │
    │  (FK/IK Toggle)        │  (Home/Gripper)     │                │
    └─────────────────────────────────────────────────────────────────┘
    """

    def __init__(
        self, parent: QWidget | None = None, controller: "RobotApp" | None = None
    ) -> None:
        super().__init__(parent, controller, page_color=MANUAL_ACCENT)
        self._kine_mode: str = "Forward"
        # Store last computed results for sending to PLC
        self._last_fk_result: tuple[float, float, float, float] | None = None
        self._last_ik_result: tuple[float, float, float, float] | None = None
        self._last_fk_coords: tuple[float, float, float] | None = None
        self._last_ik_coords: tuple[float, float, float] | None = None
        # JOG toggle states and button references (keys == PLC bit addresses)
        self._jog_states: dict[tuple[int, int], bool] = {
            ADDR.JOG_J1_LEFT: False,
            ADDR.JOG_J1_RIGHT: False,
            ADDR.JOG_J2_UP: False,
            ADDR.JOG_J2_DOWN: False,
            ADDR.JOG_J3_UP: False,
            ADDR.JOG_J3_DOWN: False,
        }
        self._jog_buttons: dict[tuple[int, int], Any] = {}
        # Elbow configuration for IK ("up" or "down")
        self._elbow_config: str = "up"
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 12, 16, 12)
        main_layout.setSpacing(12)

        columns_widget = QWidget(self)
        col_layout = QHBoxLayout(columns_widget)
        col_layout.setContentsMargins(0, 0, 0, 0)
        col_layout.setSpacing(12)

        # Column 0: Kinematics Panel
        self._build_kinematics_panel(columns_widget)

        # Column 1: Actuator Control Panel
        self._build_actuator_panel(columns_widget)

        # Column 2: Camera Panel
        self.build_camera_column(columns_widget, column=2, title_color=ACCENT)

        col_layout.setStretch(0, 2)
        col_layout.setStretch(1, 2)
        col_layout.setStretch(2, 3)

        main_layout.addWidget(columns_widget, 1)

        # Bottom status bar
        self.lbl_err_status = self.build_status_bar()

    def _build_kinematics_panel(self, parent: QWidget) -> None:
        """Build the kinematics input panel with 2-column layout (Input | Result)."""
        frame = self._create_card(parent)
        frame.setMinimumWidth(260)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 8, 8, 12)
        layout.setSpacing(8)

        self.build_section_header(frame, "🎯", "KINEMATICS INPUT", MANUAL_ACCENT)
        layout.addStretch(1)

        # Mode toggle – two buttons: THUẬN (16.5) and NGHỊCH (16.6)
        mode_widget = QWidget(frame)
        mode_layout = QHBoxLayout(mode_widget)
        mode_layout.setContentsMargins(15, 4, 15, 4)

        self.lbl_kine_mode = QLabel("▶ FORWARD KINEMATICS", mode_widget)
        self.lbl_kine_mode.setStyleSheet(
            f"color: {SUCCESS}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
        )
        mode_layout.addWidget(self.lbl_kine_mode)
        mode_layout.addStretch()

        self.btn_thuan = QPushButton("THUẬN", mode_widget)
        self.btn_thuan.setFixedSize(70, 28)
        self.btn_thuan.setStyleSheet(
            btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
        )
        self.btn_thuan.clicked.connect(self._select_forward_mode)
        mode_layout.addWidget(self.btn_thuan)

        self.btn_nghich = QPushButton("NGHỊCH", mode_widget)
        self.btn_nghich.setFixedSize(70, 28)
        self.btn_nghich.setStyleSheet(
            btn_style(
                bg="transparent",
                hover="#475569",
                text=TEXT_PRIMARY,
                radius=6,
                font_size=10,
                border="1px solid #475569",
            )
        )
        self.btn_nghich.clicked.connect(self._select_inverse_mode)
        mode_layout.addWidget(self.btn_nghich)

        layout.addWidget(mode_widget)

        # ── Entry fields container ──────────────────────────────────────
        entry_container = QWidget(frame)
        self._entry_container_layout = QVBoxLayout(entry_container)
        self._entry_container_layout.setContentsMargins(15, 6, 15, 8)
        self._entry_container_layout.setSpacing(6)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # FORWARD KINEMATICS – 2-column layout (Input: J1-J4 | Result: X,Y,Z)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self._fk_frame = QWidget(entry_container)
        fk_layout = QHBoxLayout(self._fk_frame)
        fk_layout.setContentsMargins(0, 0, 0, 0)
        fk_layout.setSpacing(8)

        # FK Left: INPUT
        fk_left = QWidget(self._fk_frame)
        fk_l_layout = QVBoxLayout(fk_left)
        fk_l_layout.setContentsMargins(0, 0, 0, 0)
        fk_l_layout.setSpacing(4)

        lbl_fk_in = QLabel("📥 INPUT", fk_left)
        lbl_fk_in.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        fk_l_layout.addWidget(lbl_fk_in)

        self.ent_j1 = self._make_joint_entry(fk_left, "θ₁ - Base (°)")
        self.ent_j2 = self._make_joint_entry(fk_left, "θ₂ - Shoulder (°)")
        self.ent_j3 = self._make_joint_entry(fk_left, "θ₃ - Elbow (°)")
        self.ent_j4 = self._make_joint_entry(fk_left, "θ₄ - Wrist (°)")
        fk_l_layout.addStretch()
        fk_layout.addWidget(fk_left, 1)

        # FK Right: RESULT
        fk_right = QWidget(self._fk_frame)
        fk_r_layout = QVBoxLayout(fk_right)
        fk_r_layout.setContentsMargins(0, 0, 0, 0)
        fk_r_layout.setSpacing(4)

        lbl_fk_out = QLabel("📤 RESULT", fk_right)
        lbl_fk_out.setStyleSheet(
            f"color: {SUCCESS}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        fk_r_layout.addWidget(lbl_fk_out)

        self.lbl_fk_x = self.create_data_row(fk_right, "X", "--- mm", SUCCESS)
        self.lbl_fk_y = self.create_data_row(fk_right, "Y", "--- mm", SUCCESS)
        self.lbl_fk_z = self.create_data_row(fk_right, "Z", "--- mm", SUCCESS)
        fk_r_layout.addStretch()
        fk_layout.addWidget(fk_right, 1)

        self._entry_container_layout.addWidget(self._fk_frame)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # INVERSE KINEMATICS – 2-column layout (Input X,Y,Z | Result J1-J4)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self._ik_frame = QWidget(entry_container)
        ik_layout = QHBoxLayout(self._ik_frame)
        ik_layout.setContentsMargins(0, 0, 0, 0)
        ik_layout.setSpacing(8)

        # IK Left: INPUT
        ik_left = QWidget(self._ik_frame)
        ik_l_layout = QVBoxLayout(ik_left)
        ik_l_layout.setContentsMargins(0, 0, 0, 0)
        ik_l_layout.setSpacing(4)

        lbl_ik_in = QLabel("📥 INPUT", ik_left)
        lbl_ik_in.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        ik_l_layout.addWidget(lbl_ik_in)

        self.ent_x = self._make_coord_entry(ik_left, "X (mm)")
        self.ent_y = self._make_coord_entry(ik_left, "Y (mm)")
        self.ent_z = self._make_coord_entry(ik_left, "Z (mm)")

        # Elbow configuration toggle
        elbow_widget = QWidget(ik_left)
        el_layout = QHBoxLayout(elbow_widget)
        el_layout.setContentsMargins(0, 4, 0, 0)
        el_layout.setSpacing(4)

        lbl_elbow = QLabel("Elbow:", elbow_widget)
        lbl_elbow.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        el_layout.addWidget(lbl_elbow)

        self.btn_elbow_up = QPushButton("UP", elbow_widget)
        self.btn_elbow_up.setFixedSize(50, 26)
        self.btn_elbow_up.setStyleSheet(
            btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
        )
        self.btn_elbow_up.clicked.connect(lambda: self._set_elbow("up"))
        el_layout.addWidget(self.btn_elbow_up)

        self.btn_elbow_down = QPushButton("DOWN", elbow_widget)
        self.btn_elbow_down.setFixedSize(55, 26)
        self.btn_elbow_down.setStyleSheet(
            btn_style(
                bg="transparent",
                hover="#475569",
                text=TEXT_PRIMARY,
                radius=6,
                font_size=10,
                border="1px solid #475569",
            )
        )
        self.btn_elbow_down.clicked.connect(lambda: self._set_elbow("down"))
        el_layout.addWidget(self.btn_elbow_down)

        ik_l_layout.addWidget(elbow_widget)
        ik_layout.addWidget(ik_left, 1)

        # IK Right: RESULT
        ik_right = QWidget(self._ik_frame)
        ik_r_layout = QVBoxLayout(ik_right)
        ik_r_layout.setContentsMargins(0, 0, 0, 0)
        ik_r_layout.setSpacing(4)

        lbl_ik_out = QLabel("📤 RESULT", ik_right)
        lbl_ik_out.setStyleSheet(
            f"color: {SUCCESS}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        ik_r_layout.addWidget(lbl_ik_out)

        self.lbl_ik_j1 = self.create_data_row(ik_right, "θ₁", "--- °", SUCCESS)
        self.lbl_ik_j2 = self.create_data_row(ik_right, "θ₂", "--- °", SUCCESS)
        self.lbl_ik_j3 = self.create_data_row(ik_right, "θ₃", "--- °", SUCCESS)
        self.lbl_ik_j4 = self.create_data_row(ik_right, "θ₄", "--- °", SUCCESS)
        
        ik_layout.addWidget(ik_right, 1)

        self._entry_container_layout.addWidget(self._ik_frame)
        self._ik_frame.setVisible(False)  # Initial mode is Forward

        layout.addWidget(entry_container)

        # ── Action buttons row ──────────────────────────────────────────
        btn_row = QWidget(frame)
        btn_row_layout = QHBoxLayout(btn_row)
        btn_row_layout.setContentsMargins(15, 6, 15, 8)
        btn_row_layout.setSpacing(8)

        # CALCULATE button
        btn_calc = QPushButton("🔢   CALCULATE", btn_row)
        btn_calc.setFixedHeight(42)
        btn_calc.setStyleSheet(
            btn_style(INFO, "#2563EB", radius=10, font_size=12)
        )
        btn_calc.clicked.connect(self._calculate)
        btn_row_layout.addWidget(btn_calc, 1)

        # SEND TARGETS button
        btn_send = QPushButton("✈   SEND", btn_row)
        btn_send.setFixedHeight(42)
        btn_send.setStyleSheet(
            btn_style(MANUAL_ACCENT, "#9333EA", radius=10, font_size=12)
        )
        btn_send.clicked.connect(self._send_positions)
        btn_row_layout.addWidget(btn_send, 1)

        layout.addWidget(btn_row)
        layout.addStretch(1)

        if parent.layout() is not None:
            parent.layout().addWidget(frame)

    def _make_joint_entry(self, parent: QWidget, placeholder: str) -> QLineEdit:
        """Create one styled joint-angle entry field."""
        entry = QLineEdit(parent)
        entry.setPlaceholderText(placeholder)
        entry.setFixedHeight(36)
        entry.setValidator(QDoubleValidator(-360.0, 360.0, 2, entry))
        if parent.layout() is not None:
            parent.layout().addWidget(entry)
        return entry

    def _make_coord_entry(self, parent: QWidget, placeholder: str) -> QLineEdit:
        """Create one styled Cartesian coordinate entry field."""
        entry = QLineEdit(parent)
        entry.setPlaceholderText(placeholder)
        entry.setFixedHeight(36)
        entry.setValidator(QDoubleValidator(-1000.0, 1000.0, 2, entry))
        if parent.layout() is not None:
            parent.layout().addWidget(entry)
        return entry

    def _build_actuator_panel(self, parent: QWidget) -> None:
        """Build the actuator/gripper control panel."""
        frame = self._create_card(parent)
        frame.setMinimumWidth(260)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(8, 8, 8, 12)
        layout.setSpacing(8)

        self.build_section_header(frame, "🦾", "ACTUATOR CONTROL", MANUAL_ACCENT)

        btn_frame = QWidget(frame)
        btn_layout = QVBoxLayout(btn_frame)
        btn_layout.setContentsMargins(15, 12, 15, 12)
        btn_layout.setSpacing(12)

        # HOME button
        btn_home = QPushButton("🏠   MOVE TO HOME", btn_frame)
        btn_home.setFixedHeight(54)
        btn_home.setStyleSheet(
            btn_style(INFO, "#2563EB", radius=10, font_size=13)
        )
        btn_home.clicked.connect(
            lambda: self.controller.plc.send_pulse(*ADDR.MOVE_TO_HOME)
            if self.controller
            else None
        )
        btn_layout.addWidget(btn_home)

        # Pick / Release row
        grip_widget = QWidget(btn_frame)
        grip_layout = QHBoxLayout(grip_widget)
        grip_layout.setContentsMargins(0, 0, 0, 0)
        grip_layout.setSpacing(8)

        btn_pick = QPushButton("✊   GẮP VẬT", grip_widget)
        btn_pick.setFixedHeight(54)
        btn_pick.setStyleSheet(
            btn_style(SUCCESS, SUCCESS_HOVER, radius=10, font_size=12)
        )
        btn_pick.clicked.connect(
            lambda: self.controller.plc.send_pulse(*ADDR.GAP_VAT)
            if self.controller
            else None
        )
        grip_layout.addWidget(btn_pick, 1)

        btn_rel = QPushButton("🖐   NHẢ VẬT", grip_widget)
        btn_rel.setFixedHeight(54)
        btn_rel.setStyleSheet(
            btn_style("#475569", "#64748B", radius=10, font_size=12)
        )
        btn_rel.clicked.connect(
            lambda: self.controller.plc.send_pulse(*ADDR.NHA_VAT)
            if self.controller
            else None
        )
        grip_layout.addWidget(btn_rel, 1)
        btn_layout.addWidget(grip_widget)

        # STOP button
        btn_stop = QPushButton("⏹   STOP ROBOT", btn_frame)
        btn_stop.setFixedHeight(54)
        btn_stop.setStyleSheet(
            btn_style(DANGER, DANGER_HOVER, radius=10, font_size=13)
        )
        btn_stop.clicked.connect(
            lambda: self.controller.plc.send_pulse(*ADDR.STOP_ROBOT)
            if self.controller
            else None
        )
        btn_layout.addWidget(btn_stop)

        # ── JOG CONTROL section ─────────────────────────────────────────
        jog_header = QWidget(btn_frame)
        jh_layout = QHBoxLayout(jog_header)
        jh_layout.setContentsMargins(0, 8, 0, 2)
        jh_layout.setSpacing(6)

        lbl_j_icon = QLabel("🕹", jog_header)
        lbl_j_icon.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 16px; border: none; background: transparent;"
        )
        jh_layout.addWidget(lbl_j_icon)

        lbl_j_title = QLabel("JOG CONTROL", jog_header)
        lbl_j_title.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
        )
        jh_layout.addWidget(lbl_j_title)
        jh_layout.addStretch()
        btn_layout.addWidget(jog_header)

        # JOG button definitions: (label_left, addr_left, label_right, addr_right)
        jog_rows = [
            ("J1", "◀ QUAY TRÁI", ADDR.JOG_J1_LEFT, "QUAY PHẢI ▶", ADDR.JOG_J1_RIGHT),
            ("J2", "▲ LÊN", ADDR.JOG_J2_UP, "XUỐNG ▼", ADDR.JOG_J2_DOWN),
            ("J3", "▲ LÊN", ADDR.JOG_J3_UP, "XUỐNG ▼", ADDR.JOG_J3_DOWN),
        ]

        for axis, lbl_l, addr_l, lbl_r, addr_r in jog_rows:
            axis_lbl = QLabel(axis, btn_frame)
            axis_lbl.setStyleSheet(
                f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
            )
            btn_layout.addWidget(axis_lbl)

            row_widget = QWidget(btn_frame)
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(6)

            # Left button
            btn_l = QPushButton(f"{lbl_l} [OFF]", row_widget)
            btn_l.setFixedHeight(38)
            btn_l.setStyleSheet(
                btn_style("#475569", "#64748B", radius=8, font_size=11)
            )
            btn_l.clicked.connect(lambda _, a=addr_l, t=lbl_l: self._toggle_jog(a, t))
            row_layout.addWidget(btn_l, 1)
            self._jog_buttons[addr_l] = btn_l

            # Right button
            btn_r = QPushButton(f"{lbl_r} [OFF]", row_widget)
            btn_r.setFixedHeight(38)
            btn_r.setStyleSheet(
                btn_style("#475569", "#64748B", radius=8, font_size=11)
            )
            btn_r.clicked.connect(lambda _, a=addr_r, t=lbl_r: self._toggle_jog(a, t))
            row_layout.addWidget(btn_r, 1)
            self._jog_buttons[addr_r] = btn_r

            btn_layout.addWidget(row_widget)

        layout.addWidget(btn_frame)
        layout.addStretch(1)

        if parent.layout() is not None:
            parent.layout().addWidget(frame)

    def _toggle_jog(self, addr: tuple[int, int], label: str) -> None:
        """Toggle a JOG button state and send the new value to PLC."""
        new_state = not self._jog_states[addr]
        self._jog_states[addr] = new_state
        if self.controller and hasattr(self.controller, "plc"):
            self.controller.plc.write_bit(addr[0], addr[1], new_state)

        btn = self._jog_buttons.get(addr)
        if btn is not None:
            if new_state:
                new_text = f"{label} [ON]"
                new_bg = SUCCESS
            else:
                new_text = f"{label} [OFF]"
                new_bg = "#475569"

            if hasattr(btn, "setText"):
                btn.setText(new_text)
            if hasattr(btn, "setStyleSheet"):
                btn.setStyleSheet(
                    btn_style(new_bg, SUCCESS_HOVER if new_state else "#64748B", radius=8, font_size=11)
                )

        log.info("JOG toggle %d.%d → %s", addr[0], addr[1], new_state)
        try:
            if self.controller and hasattr(self.controller, "log_tx"):
                self.controller.log_tx(
                    f"JOG {label} → {'ON' if new_state else 'OFF'} (DB {addr[0]}.{addr[1]})"
                )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _set_elbow(self, config: str) -> None:
        """Switch elbow configuration for IK and update toggle visuals."""
        self._elbow_config = config
        if config == "up":
            self.btn_elbow_up.setStyleSheet(
                btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
            )
            self.btn_elbow_down.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#475569",
                    text=TEXT_PRIMARY,
                    radius=6,
                    font_size=10,
                    border="1px solid #475569",
                )
            )
        else:
            self.btn_elbow_down.setStyleSheet(
                btn_style(INFO, "#2563EB", radius=6, font_size=10)
            )
            self.btn_elbow_up.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#475569",
                    text=TEXT_PRIMARY,
                    radius=6,
                    font_size=10,
                    border="1px solid #475569",
                )
            )
        log.info("Elbow configuration set to: %s", config)

    def _select_forward_mode(self) -> None:
        """Switch to Forward Kinematics mode and pulse PLC address 16.5."""
        if self._kine_mode != "Forward":
            self._kine_mode = "Forward"
            self.lbl_kine_mode.setText("▶ FORWARD KINEMATICS")
            self.lbl_kine_mode.setStyleSheet(
                f"color: {SUCCESS}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
            )
            self.btn_thuan.setStyleSheet(
                btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
            )
            self.btn_nghich.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#475569",
                    text=TEXT_PRIMARY,
                    radius=6,
                    font_size=10,
                    border="1px solid #475569",
                )
            )
            self._ik_frame.setVisible(False)
            self._fk_frame.setVisible(True)

        if self.controller and hasattr(self.controller, "plc"):
            self.controller.plc.send_pulse(*ADDR.KINE_FORWARD)

    def _select_inverse_mode(self) -> None:
        """Switch to Inverse Kinematics mode and pulse PLC address 16.6."""
        if self._kine_mode != "Inverse":
            self._kine_mode = "Inverse"
            self.lbl_kine_mode.setText("▶ INVERSE KINEMATICS")
            self.lbl_kine_mode.setStyleSheet(
                f"color: {INFO}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
            )
            self.btn_nghich.setStyleSheet(
                btn_style(INFO, "#2563EB", radius=6, font_size=10)
            )
            self.btn_thuan.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#475569",
                    text=TEXT_PRIMARY,
                    radius=6,
                    font_size=10,
                    border="1px solid #475569",
                )
            )
            self._fk_frame.setVisible(False)
            self._ik_frame.setVisible(True)

        if self.controller and hasattr(self.controller, "plc"):
            self.controller.plc.send_pulse(*ADDR.KINE_INVERSE)

    def _calculate(self) -> None:
        """Calculate kinematics and display results in the result column."""
        if self._kine_mode == "Forward":
            self._calculate_forward()
        else:
            self._calculate_inverse()

    @staticmethod
    def _entry_float(entry: Any, default: float = 0.0) -> float:
        """Parse a QLineEdit's contents as float; empty string → *default*."""
        text = entry.text().strip()
        return float(text) if text else default

    def _calculate_forward(self) -> None:
        """Compute FK from θ₁(°), θ₂(°), θ₃(°), θ₄(°) and display X, Y, Z."""
        try:
            j1 = self._entry_float(self.ent_j1)
            j2 = self._entry_float(self.ent_j2)
            j3 = self._entry_float(self.ent_j3)
            j4 = self._entry_float(self.ent_j4)
        except ValueError:
            messagebox.showerror(
                "Input Error",
                "All joint fields must contain valid decimal numbers.",
                parent=self,
            )
            return

        if not self._validate_joints(j1, j2, j3, j4):
            return

        # Compute FK: (θ₁°, θ₂°, θ₃°, θ₄°) → (X, Y, Z)
        x, y, z, _pitch = forward_kinematics(j1, j2, j3, j4)

        # Store computed result for later sending
        self._last_fk_result = (j1, j2, j3, j4)
        self._last_fk_coords = (x, y, z)

        # Update result labels
        self.lbl_fk_x.setText(f"{x:.2f} mm")
        self.lbl_fk_y.setText(f"{y:.2f} mm")
        self.lbl_fk_z.setText(f"{z:.2f} mm")

        # Tự động gán kết quả FK sang làm đầu vào chờ sẵn cho phần IK
        for entry, value in ((self.ent_x, x), (self.ent_y, y), (self.ent_z, z)):
            entry.setText(f"{value:.2f}")

        log.info(
            "FK Calculate: θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° θ₄=%.2f° → X=%.2f Y=%.2f Z=%.2f",
            j1,
            j2,
            j3,
            j4,
            x,
            y,
            z,
        )
        try:
            if self.controller and hasattr(self.controller, "log_tx"):
                self.controller.log_tx(
                    f"FK Calc: J=[{j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}]° → "
                    f"XYZ=[{x:.1f}, {y:.1f}, {z:.1f}]mm"
                )
        except Exception:
            pass

    def _calculate_inverse(self) -> None:
        """Compute IK from X, Y, Z inputs and display θ₁, θ₂, θ₃, θ₄."""
        try:
            x = self._entry_float(self.ent_x)
            y = self._entry_float(self.ent_y)
            z = self._entry_float(self.ent_z)
        except ValueError:
            messagebox.showerror(
                "Input Error",
                "X, Y, Z fields must contain valid decimal numbers.",
                parent=self,
            )
            return

        try:
            phi = (
                self.controller.cfg.kinematics.default_phi
                if self.controller and hasattr(self.controller, "cfg")
                else 0.0
            )
            j1, j2, j3, j4 = inverse_kinematics(
                x,
                y,
                z,
                phi=phi,
                elbow=self._elbow_config,
            )

            self._last_ik_result = (j1, j2, j3, j4)
            self._last_ik_coords = (x, y, z)

            for label, value in (
                (self.lbl_ik_j1, j1),
                (self.lbl_ik_j2, j2),
                (self.lbl_ik_j3, j3),
                (self.lbl_ik_j4, j4),
            ):
                label.setText(f"{value:.2f} °")

            log.info(
                "IK Calculate: X=%.2f Y=%.2f Z=%.2f → "
                "θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° θ₄=%.2f°",
                x,
                y,
                z,
                j1,
                j2,
                j3,
                j4,
            )

        except Exception as exc:
            for label in (
                self.lbl_ik_j1,
                self.lbl_ik_j2,
                self.lbl_ik_j3,
                self.lbl_ik_j4,
            ):
                label.setText("ERR")
            self._last_ik_result = None
            self._last_ik_coords = None
            messagebox.showerror(
                "IK Error", f"Cannot compute inverse kinematics:\n{exc}", parent=self
            )

    def _send_positions(self) -> None:
        """Send the last computed results to PLC."""
        if self._kine_mode == "Forward":
            self._send_forward_kinematics()
        else:
            self._send_inverse_kinematics()

    def _send_forward_kinematics(self) -> None:
        """Send joint values and computed coordinates to PLC."""
        if self._last_fk_result is None or self._last_fk_coords is None:
            messagebox.showwarning(
                "No Result",
                "Please click CALCULATE first to compute kinematics.",
                parent=self,
            )
            return

        j1, j2, j3, j4 = self._last_fk_result
        x, y, z = self._last_fk_coords

        if not self._validate_joints(j1, j2, j3, j4):
            return

        if self.controller and hasattr(self.controller, "plc"):
            self.controller.plc.send_forward_kinematics_data(j1, j2, j3, x, y, z)
        log.info(
            "FK Send: θ₁=%.2f° θ₂=%.2f° θ₃=%.2f° (XY HT = [%.2f, %.2f, %.2f])",
            j1,
            j2,
            j3,
            x,
            y,
            z,
        )
        try:
            if self.controller and hasattr(self.controller, "log_tx"):
                self.controller.log_tx(
                    f"FK Send → PLC: J=[{j1:.1f}, {j2:.1f}, {j3:.1f}]° "
                    f"XYZ=[{x:.1f}, {y:.1f}, {z:.1f}]mm"
                )
        except Exception:
            pass
        messagebox.showinfo(
            "Success",
            f"Sent to PLC:\nθ₁={j1:.2f}° (Base)\nθ₂={j2:.2f}° (Shoulder)\n"
            f"θ₃={j3:.2f}° (Elbow)\n\n"
            f"Computed Coordinates (HT):\nX={x:.2f} mm\nY={y:.2f} mm\nZ={z:.2f} mm",
            parent=self,
        )

    def _send_inverse_kinematics(self) -> None:
        """Send computed IK joint values and input coordinates to PLC."""
        if self._last_ik_result is None or self._last_ik_coords is None:
            messagebox.showwarning(
                "No Result",
                "Please click CALCULATE first to compute kinematics.",
                parent=self,
            )
            return

        j1, j2, j3, j4 = self._last_ik_result
        x, y, z = self._last_ik_coords

        if not self._validate_joints(j1, j2, j3, j4):
            return

        if self.controller and hasattr(self.controller, "plc"):
            self.controller.plc.send_inverse_kinematics_data(x, y, z, j1, j2, j3)
        log.info(
            "IK Send: XYZ=[%.2f, %.2f, %.2f] (Theta HT = [%.2f, %.2f, %.2f])",
            x,
            y,
            z,
            j1,
            j2,
            j3,
        )
        try:
            if self.controller and hasattr(self.controller, "log_tx"):
                self.controller.log_tx(
                    f"IK Send → PLC: XYZ=[{x:.1f}, {y:.1f}, {z:.1f}]mm "
                    f"J=[{j1:.1f}, {j2:.1f}, {j3:.1f}]°"
                )
        except Exception:
            pass
        messagebox.showinfo(
            "Success",
            f"Sent to PLC:\nCoordinates:\nX={x:.2f} mm\nY={y:.2f} mm\nZ={z:.2f} mm\n\n"
            f"Computed Joint Angles (HT):\nθ₁={j1:.2f}° (Base)\n"
            f"θ₂={j2:.2f}° (Shoulder)\nθ₃={j3:.2f}° (Elbow)",
            parent=self,
        )

    def _validate_joints(self, j1: float, j2: float, j3: float, j4: float) -> bool:
        """
        Validate joint angle ranges against the configured limits
        (config.yaml → kinematics module) to prevent damage or PLC faults.
        """
        try:
            validate_joint_angles(j1, j2, j3, j4)
            return True
        except WorkspaceError as exc:
            messagebox.showerror("Input Error", str(exc), parent=self)
            return False

    # ------------------------------------------------------------------
    # BasePage contract
    # ------------------------------------------------------------------

    def update_gui_data(self, data: dict[str, Any]) -> None:
        """Refresh the error status label from *data*."""
        self._refresh_error_status(data)
