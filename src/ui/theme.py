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
MANUAL_ACCENT: str = "#A855F7"  # Vibrant purple manual-mode accent
INFO: str = "#3B82F6"  # Informational blue (primary actions)

# ── Surfaces ─────────────────────────────────────────────────────────────────
PANEL_BG: str = "#1E293B"  # Dark slate panel background
CARD_BG: str = "#0B1329"  # Deep blue-gray card background
ROW_BG: str = "#090E1A"  # Darkest interior row background
HEADER_BG: str = "#050B1A"  # Deepest navy header/footer background
CONTENT_BG: str = "#0F172A"  # Page content background

# ── Lines & borders ──────────────────────────────────────────────────────────
PANEL_BORDER: str = "#334155"  # Slate border for clean lines
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
FONT_MONO: str = "Consolas"  # Numeric readouts
FONT_UI: str = "Segoe UI"  # Labels / body
