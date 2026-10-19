"""Numeric editor validation shared by Settings and Manual mode."""
from PySide6.QtCore import QLocale
from PySide6.QtGui import QDoubleValidator


class DecimalValidator(QDoubleValidator):
    """Accept either decimal comma or decimal point, independent of OS locale."""

    def __init__(self, *args):
        super().__init__(*args)
        self.setLocale(QLocale.c())

    def validate(self, text, position):
        return super().validate(text.replace(",", "."), position)

    def fixup(self, text):
        return super().fixup(text.replace(",", "."))
