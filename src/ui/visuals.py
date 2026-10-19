"""Small vector UI assets; no emoji fonts or external icon package required."""
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import QLabel

from src.ui.theme import PANEL_BORDER, TEXT_DIM, TEXT_PRIMARY


def line_icon(name: str, color: str = TEXT_PRIMARY, size: int = 18) -> QIcon:
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.scale(size / 24, size / 24)
    painter.setPen(QPen(QColor(color), 1.6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
    if name == "play":
        painter.drawPolygon(QPolygonF([QPointF(8, 5), QPointF(19, 12), QPointF(8, 19)]))
    elif name == "pause":
        painter.drawLine(8, 5, 8, 19)
        painter.drawLine(16, 5, 16, 19)
    elif name == "stop":
        painter.drawRoundedRect(QRectF(5, 5, 14, 14), 2, 2)
    elif name == "camera":
        painter.drawRoundedRect(QRectF(3, 7, 18, 13), 3, 3)
        painter.drawEllipse(QRectF(8, 10, 8, 8))
        painter.drawPolyline(QPolygonF([QPointF(7, 7), QPointF(9, 4), QPointF(15, 4), QPointF(17, 7)]))
    elif name == "settings":
        for x, y in ((5, 9), (12, 15), (19, 7)):
            painter.drawLine(x, 3, x, 21)
            painter.setBrush(QColor("#141D2B"))
            painter.drawEllipse(QPointF(x, y), 2.5, 2.5)
    elif name == "home":
        painter.drawPolyline(QPolygonF([QPointF(3, 11), QPointF(12, 4), QPointF(21, 11)]))
        painter.drawPolyline(QPolygonF([QPointF(6, 9), QPointF(6, 20), QPointF(18, 20), QPointF(18, 9)]))
    elif name.startswith("arrow-"):
        painter.translate(12, 12)
        painter.rotate({"arrow-right": 0, "arrow-down": 90, "arrow-left": 180, "arrow-up": 270}[name])
        painter.drawLine(-7, 0, 7, 0)
        painter.drawPolyline(QPolygonF([QPointF(1, -6), QPointF(7, 0), QPointF(1, 6)]))
    else:
        painter.drawEllipse(QRectF(6, 6, 12, 12))
        painter.drawLine(12, 2, 12, 7)
        painter.drawLine(12, 17, 12, 22)
        painter.drawLine(2, 12, 7, 12)
        painter.drawLine(17, 12, 22, 12)
    painter.end()
    return QIcon(pixmap)


class CameraPreview(QLabel):
    """Quiet camera placeholder that gives way to the real preview frame."""

    def paintEvent(self, event):
        if self.pixmap() is not None and not self.pixmap().isNull():
            return super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0A1019"))
        painter.setPen(QPen(QColor("#121E2B"), 1))
        for x in range(0, self.width(), 32):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 32):
            painter.drawLine(0, y, self.width(), y)
        cx, cy = self.width() // 2, self.height() // 2
        painter.setPen(QPen(QColor(PANEL_BORDER), 1))
        painter.setBrush(QColor("#141F2E"))
        painter.drawRoundedRect(QRectF(cx - 30, cy - 68, 60, 60), 16, 16)
        painter.drawPixmap(cx - 14, cy - 52, line_icon("camera", TEXT_DIM, 28).pixmap(28, 28))
        painter.setPen(QColor(TEXT_PRIMARY))
        painter.setFont(QFont("Segoe UI", 11, QFont.Weight.DemiBold))
        painter.drawText(QRectF(0, cy + 8, self.width(), 24), Qt.AlignmentFlag.AlignCenter, "Waiting for camera")
        painter.setPen(QColor(TEXT_DIM))
        painter.setFont(QFont("Segoe UI", 9))
        painter.drawText(QRectF(0, cy + 36, self.width(), 44), Qt.AlignmentFlag.AlignHCenter,
                         "Select a source to view the live feed.\nThe preview appears here when frames arrive.")
        painter.end()
