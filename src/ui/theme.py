"""
src/ui/theme.py – Centralized theme constants for the robot control UI.

All color values are defined here once and imported by every page.
This eliminates duplication and makes theme changes trivial.
"""

from __future__ import annotations

# Professional color palette
# Modern professional color palette (Tailored for VAA branding)
ACCENT: str = "#00E5FF"           # Sleek Cyan accent
ACCENT_DARK: str = "#00B0FF"      # Darker cyan for active elements
PANEL_BG: str = "#1E293B"         # Dark slate panel background
PANEL_BORDER: str = "#334155"      # Slate border for clean lines
SUCCESS: str = "#10B981"          # Modern emerald green
WARNING: str = "#F59E0B"          # Professional amber warning
DANGER: str = "#EF4444"           # Clean rose-red danger
TEXT_PRIMARY: str = "#F8FAFC"     # Soft white text
TEXT_SECONDARY: str = "#94A3B8"   # High contrast dimmed text
CARD_BG: str = "#0B1329"          # Deep blue-gray card background
HEADER_BG: str = "#050B1A"        # Deepest navy header background
MANUAL_ACCENT: str = "#A855F7"    # Vibrant purple manual mode accent

