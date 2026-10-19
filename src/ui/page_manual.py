"""
src/ui/page_manual.py – Manual (operator-driven) operation page (PySide6).

Provides direct joint-angle entry (forward / inverse kinematics toggle),
gripper control buttons, and a synchronised camera feed.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any

from PySide6.QtCore import QEvent, Qt
from src.ui.validators import DecimalValidator
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
from src.ui.help_text import SECTION_HELP, set_help, set_help_notice
from src.ui.visuals import line_icon
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
        self._jog_labels: dict[tuple[int, int], str] = {}
        # Elbow configuration for IK ("up" or "down")
        self._elbow_config: str = "up"
        self._build_ui()
        for entry in (self.ent_j1, self.ent_j2, self.ent_j3, self.ent_j4):
            entry.textChanged.connect(self._invalidate_forward_result)
        for entry in (self.ent_x, self.ent_y, self.ent_z):
            entry.textChanged.connect(self._invalidate_inverse_result)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 14, 20, 10)
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

        self.add_workspace(main_layout, columns_widget, 'Manual workspace', 'Calculate a target, operate the gripper and position each joint.', 'MANUAL CONTROL')

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
        # Mode toggle – two buttons: THUẬN (16.5) and NGHỊCH (16.6)
        mode_widget = QWidget(frame)
        mode_layout = QVBoxLayout(mode_widget)
        mode_layout.setContentsMargins(15, 4, 15, 4)

        self.lbl_kine_mode = QLabel("FORWARD KINEMATICS", mode_widget)
        set_help(self.lbl_kine_mode, "Phương pháp tính", SECTION_HELP["KINEMATICS INPUT"])
        self.lbl_kine_mode.setStyleSheet(
            f"color: {SUCCESS}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
        )
        mode_layout.addWidget(self.lbl_kine_mode)
        mode_buttons = QHBoxLayout()
        mode_buttons.setSpacing(6)
        mode_layout.addLayout(mode_buttons)

        self.btn_thuan = QPushButton("Forward", mode_widget)
        set_help(self.btn_thuan, "Động học thuận", "Chọn nhập góc J1–J4 để tính tọa độ X, Y, Z. Nút này đồng thời gửi lệnh chọn chế độ Forward đến PLC.")
        self.btn_thuan.setFixedHeight(30)
        self.btn_thuan.setStyleSheet(
            btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
        )
        self.btn_thuan.clicked.connect(self._select_forward_mode)
        mode_buttons.addWidget(self.btn_thuan, 1)

        self.btn_nghich = QPushButton("Inverse", mode_widget)
        set_help(self.btn_nghich, "Động học nghịch", "Chọn nhập tọa độ X, Y, Z để tìm góc các khớp. Nút này đồng thời gửi lệnh chọn chế độ Inverse đến PLC.")
        self.btn_nghich.setFixedHeight(30)
        self.btn_nghich.setStyleSheet(
            btn_style(
                bg="transparent",
                hover="#243349",
                text=TEXT_PRIMARY,
                radius=6,
                font_size=10,
                border="1px solid #475569",
            )
        )
        self.btn_nghich.clicked.connect(self._select_inverse_mode)
        mode_buttons.addWidget(self.btn_nghich, 1)

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

        lbl_fk_in = QLabel("INPUT", fk_left)
        set_help(lbl_fk_in, "Góc đầu vào", "Nhập góc J1–J4 theo độ (°), sau đó nhấn Calculate. Ô trống được tính là 0; chấp nhận số thập phân dùng dấu phẩy hoặc dấu chấm.")
        lbl_fk_in.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        fk_l_layout.addWidget(lbl_fk_in)

        self.ent_j1 = self._make_joint_entry(fk_left, "J1  Base (°)")
        self.ent_j2 = self._make_joint_entry(fk_left, "J2  Shoulder (°)")
        self.ent_j3 = self._make_joint_entry(fk_left, "J3  Elbow (°)")
        self.ent_j4 = self._make_joint_entry(fk_left, "J4  Wrist (°)")
        fk_l_layout.addStretch()
        fk_layout.addWidget(fk_left, 1)

        # FK Right: RESULT
        fk_right = QWidget(self._fk_frame)
        fk_r_layout = QVBoxLayout(fk_right)
        fk_r_layout.setContentsMargins(0, 0, 0, 0)
        fk_r_layout.setSpacing(4)

        lbl_fk_out = QLabel("RESULT", fk_right)
        set_help(lbl_fk_out, "Kết quả động học thuận", "Tọa độ X, Y, Z tính từ các góc vừa nhập, đơn vị mm. Kết quả bị xóa khi sửa đầu vào để tránh gửi giá trị cũ.")
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

        lbl_ik_in = QLabel("INPUT", ik_left)
        set_help(lbl_ik_in, "Tọa độ đầu vào", "Nhập tọa độ X, Y, Z của đầu công tác theo hệ tọa độ robot, đơn vị mm; chọn tư thế Elbow rồi nhấn Calculate.")
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
        set_help(lbl_elbow, "Tư thế khuỷu tay", "Chọn nhánh nghiệm động học nghịch: UP hoặc DOWN. Chỉ chọn tư thế được giới hạn cơ khí cho phép; đổi lựa chọn cần tính lại kết quả.")
        lbl_elbow.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold; border: none; background: transparent;"
        )
        el_layout.addWidget(lbl_elbow)

        self.btn_elbow_up = QPushButton("UP", elbow_widget)
        set_help(self.btn_elbow_up, "Elbow UP", "Tìm nghiệm với khuỷu tay ở nhánh UP. Nhấn Calculate sau khi chọn để cập nhật góc các khớp.")
        self.btn_elbow_up.setFixedSize(50, 26)
        self.btn_elbow_up.setStyleSheet(
            btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
        )
        self.btn_elbow_up.clicked.connect(lambda: self._set_elbow("up"))
        el_layout.addWidget(self.btn_elbow_up)

        self.btn_elbow_down = QPushButton("DOWN", elbow_widget)
        set_help(self.btn_elbow_down, "Elbow DOWN", "Tìm nghiệm với khuỷu tay ở nhánh DOWN. Cần cấu hình cho phép J3 âm; nhấn Calculate sau khi chọn.")
        self.btn_elbow_down.setFixedSize(55, 26)
        self.btn_elbow_down.setStyleSheet(
            btn_style(
                bg="transparent",
                hover="#243349",
                text=TEXT_PRIMARY,
                radius=6,
                font_size=10,
                border="1px solid #475569",
            )
        )
        self.btn_elbow_down.clicked.connect(lambda: self._set_elbow("down"))
        if self.controller and self.controller.cfg.kinematics.j3_min >= 0:
            self.btn_elbow_down.setEnabled(False)
            set_help_notice(self.btn_elbow_down, "Giới hạn J3 hiện tại không cho phép góc âm nên nhánh DOWN bị khóa.")
        el_layout.addWidget(self.btn_elbow_down)

        ik_l_layout.addWidget(elbow_widget)
        ik_layout.addWidget(ik_left, 1)

        # IK Right: RESULT
        ik_right = QWidget(self._ik_frame)
        ik_r_layout = QVBoxLayout(ik_right)
        ik_r_layout.setContentsMargins(0, 0, 0, 0)
        ik_r_layout.setSpacing(4)

        lbl_ik_out = QLabel("RESULT", ik_right)
        set_help(lbl_ik_out, "Kết quả động học nghịch", "Các góc J1–J4 tính từ tọa độ và tư thế Elbow, đơn vị độ (°). Send targets hiện ghi J1–J3 và X, Y, Z; J4 chỉ tham gia tính toán ở màn hình này.")
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
        btn_calc = QPushButton("Calculate", btn_row)
        set_help(btn_calc, "Tính toán", "Tính kết quả theo Forward hoặc Inverse đang chọn và kiểm tra giới hạn khớp. Thao tác này chỉ tính trên PC, chưa gửi mục tiêu đến PLC. Nếu sửa đầu vào, cần tính lại trước khi gửi.")
        btn_calc.setFixedHeight(40)
        btn_calc.setStyleSheet(
            btn_style("#243349", "#34465D", radius=8, font_size=12)
        )
        btn_calc.clicked.connect(self._calculate)
        btn_row_layout.addWidget(btn_calc, 1)

        # SEND TARGETS button
        btn_send = QPushButton("Send targets", btn_row)
        btn_send.setFixedHeight(40)
        btn_send.setStyleSheet(
            btn_style(INFO, "#688FEF", radius=8, font_size=12)
        )
        btn_send.clicked.connect(self._send_positions)
        set_help(btn_send, "Gửi mục tiêu đến PLC", "Ghi J1–J3 và X, Y, Z của kết quả Calculate gần nhất vào PLC theo chế độ đã chọn. J4 không được ghi qua nút này. Phải tính lại nếu sửa đầu vào; việc thực hiện chuyển động phụ thuộc chương trình PLC.")
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
        entry.setValidator(DecimalValidator(-360.0, 360.0, 2, entry))
        joint = placeholder.split()[0]
        guidance = "Nhập góc theo độ (°), tối đa 2 chữ số thập phân; dùng dấu phẩy hoặc dấu chấm. Ô trống được tính là 0. Nhấn Calculate để kiểm tra và tính kết quả."
        if self.controller:
            limits = self.controller.cfg.kinematics
            guidance += f" Giới hạn {joint} hiện tại: {getattr(limits, joint.lower() + '_min'):g}° đến {getattr(limits, joint.lower() + '_max'):g}°."
        if joint == "J4":
            guidance += " J4 dùng trong tính toán; nút Send targets hiện chỉ ghi J1–J3 và tọa độ."
        set_help(entry, placeholder, guidance)
        entry.setAccessibleName(placeholder)
        if parent.layout() is not None:
            parent.layout().addWidget(entry)
        return entry

    def _make_coord_entry(self, parent: QWidget, placeholder: str) -> QLineEdit:
        """Create one styled Cartesian coordinate entry field."""
        entry = QLineEdit(parent)
        entry.setPlaceholderText(placeholder)
        entry.setFixedHeight(36)
        entry.setValidator(DecimalValidator(-1000.0, 1000.0, 2, entry))
        set_help(entry, placeholder, "Nhập tọa độ của đầu công tác theo hệ tọa độ robot, đơn vị mm. Chấp nhận dấu phẩy hoặc dấu chấm, tối đa 2 chữ số thập phân; ô trống được tính là 0. Nhấn Calculate để kiểm tra khả năng với tới và tính góc khớp.")
        entry.setAccessibleName(placeholder)
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
        btn_home = QPushButton("Move to home", btn_frame)
        set_help(btn_home, "Về vị trí Home", "Gửi lệnh MOVE TO HOME đến PLC; robot có thể bắt đầu di chuyển theo chương trình PLC. Kiểm tra vùng chuyển động trước khi nhấn.")
        btn_home.setIcon(line_icon("home"))
        btn_home.setFixedHeight(44)
        btn_home.setStyleSheet(
            btn_style("#243349", "#34465D", radius=8, font_size=13)
        )
        btn_home.clicked.connect(
            lambda: self._send_manual_pulse(ADDR.MOVE_TO_HOME, "MOVE TO HOME")
            if self.controller
            else None
        )
        btn_layout.addWidget(btn_home)

        # Pick / Release row
        grip_widget = QWidget(btn_frame)
        grip_layout = QHBoxLayout(grip_widget)
        grip_layout.setContentsMargins(0, 0, 0, 0)
        grip_layout.setSpacing(8)

        btn_pick = QPushButton("Close gripper", grip_widget)
        set_help(btn_pick, "Đóng kẹp", "Gửi lệnh đóng kẹp đến PLC để giữ vật. Quan sát vị trí vật và cơ cấu kẹp trước khi thao tác.")
        btn_pick.setFixedHeight(42)
        btn_pick.setStyleSheet(
            btn_style("#243349", "#34465D", radius=8, font_size=12)
        )
        btn_pick.clicked.connect(
            lambda: self._send_manual_pulse(ADDR.GAP_VAT, "CLOSE GRIPPER")
            if self.controller
            else None
        )
        grip_layout.addWidget(btn_pick, 1)

        btn_rel = QPushButton("Open gripper", grip_widget)
        set_help(btn_rel, "Mở kẹp", "Gửi lệnh mở kẹp đến PLC. Vật đang giữ có thể được thả ra; kiểm tra vị trí thả trước khi nhấn.")
        btn_rel.setFixedHeight(42)
        btn_rel.setStyleSheet(
            btn_style("#243349", "#34465D", radius=10, font_size=12)
        )
        btn_rel.clicked.connect(
            lambda: self._send_manual_pulse(ADDR.NHA_VAT, "OPEN GRIPPER")
            if self.controller
            else None
        )
        grip_layout.addWidget(btn_rel, 1)
        btn_layout.addWidget(grip_widget)

        # STOP button
        btn_stop = QPushButton("Stop robot", btn_frame)
        set_help(btn_stop, "Dừng robot", "Yêu cầu dừng chu trình trên PC và gửi lệnh STOP đến PLC. Kiểm tra trạng thái và Activity log để biết lệnh có được ghi thành công. Đây là lệnh dừng phần mềm; nút dừng khẩn cấp phần cứng vẫn cần dùng khi có nguy hiểm.")
        btn_stop.setIcon(line_icon("stop", "#351018"))
        btn_stop.setFixedHeight(44)
        btn_stop.setStyleSheet(
            btn_style(DANGER, DANGER_HOVER, text="#351018", radius=10, font_size=13)
        )
        btn_stop.clicked.connect(
            lambda: self.controller.stop_robot()
            if self.controller
            else None
        )
        btn_layout.addWidget(btn_stop)

        # ── JOG CONTROL section ─────────────────────────────────────────
        jog_header = QWidget(btn_frame)
        jh_layout = QHBoxLayout(jog_header)
        jh_layout.setContentsMargins(0, 8, 0, 2)
        jh_layout.setSpacing(6)

        lbl_j_icon = QLabel(jog_header)
        lbl_j_icon.setPixmap(line_icon("target", MANUAL_ACCENT, 16).pixmap(16, 16))
        lbl_j_icon.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 16px; border: none; background: transparent;"
        )
        jh_layout.addWidget(lbl_j_icon)

        lbl_j_title = QLabel("JOG CONTROL", jog_header)
        for widget in (jog_header, lbl_j_icon, lbl_j_title):
            set_help(widget, "Điều khiển JOG", SECTION_HELP["JOG CONTROL"])
        lbl_j_title.setStyleSheet(
            f"color: {MANUAL_ACCENT}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
        )
        jh_layout.addWidget(lbl_j_title)
        jh_layout.addStretch()
        btn_layout.addWidget(jog_header)

        # JOG button definitions: (label_left, addr_left, label_right, addr_right)
        jog_rows = [
            ("J1", "Left", ADDR.JOG_J1_LEFT, "Right", ADDR.JOG_J1_RIGHT),
            ("J2", "Up", ADDR.JOG_J2_UP, "Down", ADDR.JOG_J2_DOWN),
            ("J3", "Up", ADDR.JOG_J3_UP, "Down", ADDR.JOG_J3_DOWN),
        ]

        for axis, lbl_l, addr_l, lbl_r, addr_r in jog_rows:
            axis_lbl = QLabel(axis, btn_frame)
            set_help(axis_lbl, f"JOG {axis}", SECTION_HELP["JOG CONTROL"])
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
            btn_l.setIcon(line_icon("arrow-" + lbl_l.lower(), TEXT_PRIMARY, 14))
            set_help(btn_l, f"JOG {axis} — {'trái' if axis == 'J1' else 'lên'}", SECTION_HELP["JOG CONTROL"])
            btn_l.setFixedHeight(38)
            btn_l.setStyleSheet(
                btn_style("#243349", "#34465D", radius=8, font_size=11)
            )
            btn_l.clicked.connect(lambda _, a=addr_l, t=lbl_l: self._toggle_jog(a, t))
            row_layout.addWidget(btn_l, 1)
            self._jog_buttons[addr_l] = btn_l
            self._jog_labels[addr_l] = lbl_l

            # Right button
            btn_r = QPushButton(f"{lbl_r} [OFF]", row_widget)
            btn_r.setIcon(line_icon("arrow-" + lbl_r.lower(), TEXT_PRIMARY, 14))
            set_help(btn_r, f"JOG {axis} — {'phải' if axis == 'J1' else 'xuống'}", SECTION_HELP["JOG CONTROL"])
            btn_r.setFixedHeight(38)
            btn_r.setStyleSheet(
                btn_style("#243349", "#34465D", radius=8, font_size=11)
            )
            btn_r.clicked.connect(lambda _, a=addr_r, t=lbl_r: self._toggle_jog(a, t))
            row_layout.addWidget(btn_r, 1)
            self._jog_buttons[addr_r] = btn_r
            self._jog_labels[addr_r] = lbl_r

            btn_layout.addWidget(row_widget)

        layout.addWidget(btn_frame)
        layout.addStretch(1)

        if parent.layout() is not None:
            parent.layout().addWidget(frame)

    def _toggle_jog(self, addr: tuple[int, int], label: str) -> None:
        """Toggle JOG only after writing successfully; interlock opposite directions."""
        new_state = not self._jog_states[addr]
        if not self.controller or not hasattr(self.controller, "plc"):
            return
        opposite = (addr[0], addr[1] ^ 1)
        # Never assert both direction bits for one axis. Clear the existing
        # direction first, and leave it cleared if the next write fails.
        if new_state and self._jog_states.get(opposite, False):
            if self.controller.plc.write_bit(*opposite, False) is False:
                messagebox.showerror("JOG Error", "Could not release the opposite JOG direction.", parent=self)
                return
            PageManual._set_jog_display(self, opposite, False)
        if self.controller.plc.write_bit(*addr, new_state) is False:
            messagebox.showerror("JOG Error", "JOG command failed. Check PLC connection and STOP status.", parent=self)
            return
        PageManual._set_jog_display(self, addr, new_state, label)
        log.info("JOG toggle %d.%d → %s", addr[0], addr[1], new_state)
        if hasattr(self.controller, "log_tx"):
            self.controller.log_tx(
                f"JOG {label} → {'ON' if new_state else 'OFF'} (DB {addr[0]}.{addr[1]})"
            )

    def _set_jog_display(self, addr: tuple[int, int], state: bool, label: str | None = None) -> None:
        self._jog_states[addr] = state
        btn = self._jog_buttons.get(addr)
        if btn is not None:
            if label is None:
                label = getattr(self, "_jog_labels", {}).get(addr, f"JOG {addr[1]}")
            if state:
                new_text = f"{label} [ON]"
                new_bg = SUCCESS
            else:
                new_text = f"{label} [OFF]"
                new_bg = "#243349"

            if hasattr(btn, "setText"):
                btn.setText(new_text)
            if hasattr(btn, "setStyleSheet"):
                btn.setStyleSheet(
                    btn_style(new_bg, SUCCESS_HOVER if state else "#34465D", radius=8, font_size=11)
                )

    def release_jog(self) -> bool:
        """Clear all JOG inputs before leaving Manual or losing window focus."""
        if not any(self._jog_states.values()):
            return True
        if not self.controller or self.controller.plc.write_bits(80, {i: False for i in range(6)}) is False:
            log.error("Could not release JOG; requesting STOP.")
            if self.controller:
                self.controller.stop_robot()
            return False
        for addr in self._jog_states:
            self._set_jog_display(addr, False)
        return True

    def event(self, event) -> bool:
        if event.type() == QEvent.Type.WindowDeactivate and hasattr(self, "_jog_states"):
            self.release_jog()
        return super().event(event)

    def _send_manual_pulse(self, addr: tuple[int, int], name: str) -> bool:
        if not self.controller or self.controller.plc.send_pulse(*addr) is False:
            messagebox.showerror("PLC Error", f"{name} failed. Check PLC connection and STOP status.", parent=self)
            return False
        return True

    def _invalidate_forward_result(self) -> None:
        self._last_fk_result = self._last_fk_coords = None
        for label in (self.lbl_fk_x, self.lbl_fk_y, self.lbl_fk_z):
            label.setText("--- mm")

    def _invalidate_inverse_result(self) -> None:
        self._last_ik_result = self._last_ik_coords = None
        for label in (self.lbl_ik_j1, self.lbl_ik_j2, self.lbl_ik_j3, self.lbl_ik_j4):
            label.setText("--- °")

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _set_elbow(self, config: str) -> None:
        """Switch elbow configuration for IK and update toggle visuals."""
        if config == "down" and self.controller and self.controller.cfg.kinematics.j3_min >= 0:
            return
        if self._elbow_config != config:
            self._invalidate_inverse_result()
        self._elbow_config = config
        if config == "up":
            self.btn_elbow_up.setStyleSheet(
                btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
            )
            self.btn_elbow_down.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#243349",
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
                    hover="#243349",
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
            self.lbl_kine_mode.setText("FORWARD KINEMATICS")
            self.lbl_kine_mode.setStyleSheet(
                f"color: {SUCCESS}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
            )
            self.btn_thuan.setStyleSheet(
                btn_style(SUCCESS, SUCCESS_HOVER, radius=6, font_size=10)
            )
            self.btn_nghich.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#243349",
                    text=TEXT_PRIMARY,
                    radius=6,
                    font_size=10,
                    border="1px solid #475569",
                )
            )
            self._ik_frame.setVisible(False)
            self._fk_frame.setVisible(True)

        if self.controller and hasattr(self.controller, "plc"):
            self._send_manual_pulse(ADDR.KINE_FORWARD, "FORWARD MODE")

    def _select_inverse_mode(self) -> None:
        """Switch to Inverse Kinematics mode and pulse PLC address 16.6."""
        if self._kine_mode != "Inverse":
            self._kine_mode = "Inverse"
            self.lbl_kine_mode.setText("INVERSE KINEMATICS")
            self.lbl_kine_mode.setStyleSheet(
                f"color: {INFO}; font-size: 12px; font-weight: bold; border: none; background: transparent;"
            )
            self.btn_nghich.setStyleSheet(
                btn_style(INFO, "#2563EB", radius=6, font_size=10)
            )
            self.btn_thuan.setStyleSheet(
                btn_style(
                    bg="transparent",
                    hover="#243349",
                    text=TEXT_PRIMARY,
                    radius=6,
                    font_size=10,
                    border="1px solid #475569",
                )
            )
            self._fk_frame.setVisible(False)
            self._ik_frame.setVisible(True)

        if self.controller and hasattr(self.controller, "plc"):
            self._send_manual_pulse(ADDR.KINE_INVERSE, "INVERSE MODE")

    def _calculate(self) -> None:
        """Calculate kinematics and display results in the result column."""
        if self._kine_mode == "Forward":
            self._calculate_forward()
        else:
            self._calculate_inverse()

    @staticmethod
    def _entry_float(entry: Any, default: float = 0.0) -> float:
        """Parse a QLineEdit's contents as float; empty string → *default*."""
        text = entry.text().strip().replace(",", ".")
        value = float(text) if text else default
        if not math.isfinite(value):
            raise ValueError("A finite decimal number is required.")
        return value

    def _calculate_forward(self) -> None:
        """Compute FK from θ₁(°), θ₂(°), θ₃(°), θ₄(°) and display X, Y, Z."""
        self._invalidate_forward_result()
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
        self._invalidate_inverse_result()
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

        if not self.controller or self.controller.plc.send_forward_kinematics_data(j1, j2, j3, x, y, z) is False:
            messagebox.showerror("PLC Error", "Could not send FK values. Check PLC connection and STOP status.", parent=self)
            return
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

        if not self.controller or self.controller.plc.send_inverse_kinematics_data(x, y, z, j1, j2, j3) is False:
            messagebox.showerror("PLC Error", "Could not send IK values. Check PLC connection and STOP status.", parent=self)
            return
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
        if data.get("stop_robot"):
            for addr in self._jog_states:
                self._set_jog_display(addr, False)
        self._refresh_error_status(data)
