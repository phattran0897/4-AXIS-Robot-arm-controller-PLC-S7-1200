"""
src/ui/theme.py – Centralized theme constants for the robot control UI.

All color values are defined here once and imported by every page.
This eliminates duplication and makes theme changes trivial.
"""

from __future__ import annotations

# Professional color palette
ACCENT: str = "#00D4FF"           # Cyan accent
ACCENT_DARK: str = "#0099CC"       # Darker cyan
PANEL_BG: str = "#1E293B"         # Dark slate panel
PANEL_BORDER: str = "#334155"      # Slate border
SUCCESS: str = "#22C55E"          # Green
WARNING: str = "#F59E0B"           # Amber
DANGER: str = "#EF4444"           # Red
TEXT_PRIMARY: str = "#F8FAFC"     # White text
TEXT_SECONDARY: str = "#94A3B8"   # Dimmed text
CARD_BG: str = "#0F172A"          # Dark card background
HEADER_BG: str = "#0A1628"        # Header background
MANUAL_ACCENT: str = "#A855F7"    # Purple for manual mode
