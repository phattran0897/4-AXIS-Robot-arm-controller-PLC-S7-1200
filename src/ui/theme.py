"""
src/ui/theme.py – Centralized theme constants for the robot control UI.

All colour values are defined here once and imported by every page.
This eliminates duplication and makes theme changes trivial.

The palette is a dark, industrial "mission control" look:
deep navy surfaces, cyan accents for AUTO mode, purple accents for
MANUAL mode, and semantic status colours (emerald / amber / rose).
"""

from __future__ import annotations

# ── Accents ──────────────────────────────────────────────────────────────────
ACCENT: str = "#00E5FF"  # Sleek cyan accent (AUTO mode)
ACCENT_DARK: str = "#00B0FF"  # Darker cyan for active elements
ACCENT_DIM: str = "#0E7490"  # Muted cyan for secondary highlights
ACCENT_GLOW: str = "#00E5FF40"  # Translucent glow (for borders/shadows)
MANUAL_ACCENT: str = "#A855F7"  # Vibrant purple manual-mode accent
MANUAL_GLOW: str = "#A855F740"  # Purple glow
INFO: str = "#3B82F6"  # Informational blue (primary actions)

# ── Surfaces ─────────────────────────────────────────────────────────────────
PANEL_BG: str = "#1E293B"  # Dark slate panel background
CARD_BG: str = "#0B1329"  # Deep blue-gray card background
CARD_BG_ALT: str = "#0D1530"  # Slightly lighter card variant
ROW_BG: str = "#090E1A"  # Darkest interior row background
HEADER_BG: str = "#050B1A"  # Deepest navy header/footer background
CONTENT_BG: str = "#0F172A"  # Page content background

# ── Lines & borders ──────────────────────────────────────────────────────────
PANEL_BORDER: str = "#1E3A5F"  # Refined slate-blue border
PANEL_BORDER_LIGHT: str = "#2D4A6F"  # Lighter border for hover states
ENTRY_BG: str = "#1E293B"  # Text-entry background
NEUTRAL_BTN: str = "#475569"  # Neutral button fill / OFF state
NEUTRAL_HOVER: str = "#64748B"  # Neutral button hover

# ── Semantic status colours ──────────────────────────────────────────────────
SUCCESS: str = "#10B981"  # Modern emerald green
SUCCESS_HOVER: str = "#059669"  # Emerald hover
WARNING: str = "#F59E0B"  # Professional amber warning
DANGER: str = "#EF4444"  # Clean rose-red danger
DANGER_HOVER: str = "#DC2626"  # Rose hover

# ── Typography ───────────────────────────────────────────────────────────────
TEXT_PRIMARY: str = "#F8FAFC"  # Soft white text
TEXT_SECONDARY: str = "#94A3B8"  # High-contrast dimmed text
TEXT_DIM: str = "#64748B"  # Very dim text for decorative elements
FONT_MONO: str = "Consolas"  # Numeric readouts
FONT_UI: str = "Segoe UI"  # Labels / body


# ── Qt Stylesheets (QSS) ─────────────────────────────────────────────────────

GLOBAL_QSS: str = f"""
QMainWindow, QWidget#CentralWidget {{
    background-color: {CONTENT_BG};
    color: {TEXT_PRIMARY};
    font-family: "{FONT_UI}", sans-serif;
}}

QWidget {{
    color: {TEXT_PRIMARY};
    font-family: "{FONT_UI}", sans-serif;
}}

QStackedWidget {{
    background-color: {CONTENT_BG};
}}

QLabel {{
    color: {TEXT_PRIMARY};
}}

QLineEdit {{
    background-color: {ENTRY_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {PANEL_BORDER};
    border-radius: 8px;
    padding: 6px 10px;
    font-size: 13px;
}}

QLineEdit:focus {{
    border: 1px solid {ACCENT};
}}

QComboBox {{
    background-color: {PANEL_BG};
    color: {TEXT_PRIMARY};
    border: 1px solid {PANEL_BORDER};
    border-radius: 8px;
    padding: 4px 10px;
    font-size: 12px;
}}

QComboBox:drop-down {{
    border: 0px;
    width: 24px;
}}

QComboBox QAbstractItemView {{
    background-color: {PANEL_BG};
    color: {TEXT_PRIMARY};
    selection-background-color: {ACCENT_DIM};
    selection-color: {TEXT_PRIMARY};
    border: 1px solid {PANEL_BORDER};
    border-radius: 6px;
    padding: 4px;
}}

QScrollBar:vertical {{
    background: {CONTENT_BG};
    width: 8px;
    margin: 0px;
}}

QScrollBar::handle:vertical {{
    background: {PANEL_BORDER};
    min-height: 20px;
    border-radius: 4px;
}}

QScrollBar::handle:vertical:hover {{
    background: {ACCENT_DIM};
}}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}

QTextEdit {{
    background-color: #020617;
    color: #38BDF8;
    border: 1px solid {PANEL_BORDER};
    border-radius: 8px;
    padding: 6px;
    font-family: "{FONT_MONO}", monospace;
    font-size: 10px;
    selection-background-color: {ACCENT_DIM};
}}
"""


def card_style(
    bg: str = CARD_BG, border: str = PANEL_BORDER, radius: int = 12
) -> str:
    """Return QSS for a bordered card panel."""
    return f"""
        QFrame {{
            background-color: {bg};
            border: 1px solid {border};
            border-radius: {radius}px;
        }}
    """


def card_style_glow(
    bg: str = CARD_BG,
    border: str = PANEL_BORDER,
    glow: str = ACCENT_GLOW,
    radius: int = 14,
) -> str:
    """Return QSS for a bordered card with subtle accent glow effect."""
    return f"""
        QFrame {{
            background-color: {bg};
            border: 1px solid {border};
            border-radius: {radius}px;
        }}
        QFrame:hover {{
            border: 1px solid {glow};
        }}
    """


def btn_style(
    bg: str = NEUTRAL_BTN,
    hover: str = NEUTRAL_HOVER,
    text: str = "white",
    radius: int = 8,
    font_size: int = 12,
    font_weight: str = "bold",
    padding: str = "8px 14px",
    border: str = "none",
) -> str:
    """Return QSS for a styled button with hover state."""
    return f"""
        QPushButton {{
            background-color: {bg};
            color: {text};
            border: {border};
            border-radius: {radius}px;
            font-size: {font_size}px;
            font-weight: {font_weight};
            padding: {padding};
        }}
        QPushButton:hover {{
            background-color: {hover};
        }}
        QPushButton:pressed {{
            background-color: {bg};
        }}
        QPushButton:disabled {{
            background-color: {PANEL_BG};
            color: {TEXT_SECONDARY};
            border: 1px solid {PANEL_BORDER};
        }}
    """


def video_container_style(border_color: str = ACCENT_DIM) -> str:
    """Return QSS for the camera video container with refined borders."""
    return f"""
        QFrame {{
            background-color: #000000;
            border: 2px solid {border_color};
            border-radius: 12px;
        }}
    """
