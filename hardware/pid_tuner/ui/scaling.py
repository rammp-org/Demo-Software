"""
DPI-aware scaling utilities for the PID Tuner UI.

Provides functions to detect system DPI and calculate scale factors
for fonts, widgets, and spacing to ensure consistent appearance
across different screen sizes and resolutions.
"""

from __future__ import annotations
from typing import Union

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QGuiApplication
from functools import lru_cache


# Base DPI for scaling calculations
# Windows/Linux typically use 96 DPI as standard
# macOS uses 72 DPI as logical standard but has device pixel ratio > 1 for Retina
STANDARD_DPI_WINDOWS = 96.0
STANDARD_DPI_MACOS = 72.0

# Minimum and maximum scale factors to prevent extreme scaling
MIN_SCALE = 0.85
MAX_SCALE = 2.5


@lru_cache(maxsize=1)
def get_screen_dpi() -> float:
    """
    Get the primary screen's logical DPI.

    Returns:
        The logical DPI of the primary screen.
    """
    app = QApplication.instance()
    if app is None:
        return STANDARD_DPI_WINDOWS

    screen = QGuiApplication.primaryScreen()
    if screen is None:
        return STANDARD_DPI_WINDOWS

    return screen.logicalDotsPerInch()


@lru_cache(maxsize=1)
def get_device_pixel_ratio() -> float:
    """
    Get the device pixel ratio (for Retina/HiDPI displays).

    Returns:
        The device pixel ratio (1.0 for standard, 2.0 for Retina).
    """
    app = QApplication.instance()
    if app is None:
        return 1.0

    screen = QGuiApplication.primaryScreen()
    if screen is None:
        return 1.0

    return screen.devicePixelRatio()


@lru_cache(maxsize=1)
def get_scale_factor() -> float:
    """
    Calculate the UI scale factor based on screen DPI and device pixel ratio.

    This handles the differences between macOS (72 DPI base with high pixel ratio)
    and Windows/Linux (96 DPI base with scaling).

    Returns:
        A scale factor (1.0 = standard, >1.0 = high DPI, <1.0 = smaller screens).
    """
    dpi = get_screen_dpi()
    dpr = get_device_pixel_ratio()

    # Detect macOS by its 72 DPI standard
    if dpi <= 72:
        # macOS: Use device pixel ratio to determine scale
        # Retina (dpr=2) should be ~1.0 scale (standard UI size)
        # Non-Retina (dpr=1) should also be ~1.0
        # This approach keeps UI consistent on macOS
        scale = 1.0
    else:
        # Windows/Linux: Scale relative to 96 DPI baseline
        scale = dpi / STANDARD_DPI_WINDOWS

    # Apply a slight boost for high DPI displays
    if dpr > 1.5:
        scale *= 1.05

    # Clamp to reasonable bounds
    return max(MIN_SCALE, min(MAX_SCALE, scale))


def scaled(value: Union[int, float]) -> int:
    """
    Scale a dimension value by the current scale factor.

    Args:
        value: The base dimension value (in pixels at 96 DPI).

    Returns:
        The scaled dimension as an integer.
    """
    return int(value * get_scale_factor())


def scaled_font_size(base_size: int) -> int:
    """
    Scale a font size by the current scale factor.

    Args:
        base_size: The base font size in points.

    Returns:
        The scaled font size in points.
    """
    # Font scaling is slightly less aggressive than dimension scaling
    scale = get_scale_factor()
    # Apply a dampened scale (sqrt) to prevent fonts from getting too large
    font_scale = 0.7 + 0.3 * scale  # Range: ~0.94 at 0.8x to ~1.45 at 2.5x
    return max(8, int(base_size * font_scale))


def scaled_spacing(base_spacing: int) -> int:
    """
    Scale spacing/margin values.

    Args:
        base_spacing: The base spacing in pixels.

    Returns:
        The scaled spacing as an integer.
    """
    return max(2, scaled(base_spacing))


def get_scaled_sizes() -> dict:
    """
    Get a dictionary of commonly used scaled sizes.

    Returns:
        Dictionary with scaled dimension values.
    """
    return {
        # Margins and spacing
        "margin_small": scaled_spacing(4),
        "margin_medium": scaled_spacing(8),
        "margin_large": scaled_spacing(12),
        "spacing_small": scaled_spacing(4),
        "spacing_medium": scaled_spacing(6),
        "spacing_large": scaled_spacing(10),
        # Widget sizes
        "input_min_width": scaled(60),
        "input_preferred_width": scaled(80),
        "button_min_width": scaled(50),
        "button_padding_h": scaled(10),
        "button_padding_v": scaled(4),
        "combo_min_width": scaled(70),
        # Font sizes (in points)
        "font_small": scaled_font_size(9),
        "font_normal": scaled_font_size(10),
        "font_medium": scaled_font_size(11),
        "font_large": scaled_font_size(13),
        "font_header": scaled_font_size(14),
        # Control panel specific
        "control_panel_min_width": scaled(280),
        "control_panel_preferred_width": scaled(340),
        # Encoder bar
        "encoder_bar_height": scaled(28),
        "encoder_bar_max_height": scaled(36),
        # Minimum window size
        "window_min_width": scaled(900),
        "window_min_height": scaled(550),
    }


# Pre-compute sizes at module load for efficiency
SIZES = get_scaled_sizes()


def refresh_sizes():
    """
    Refresh the cached sizes after DPI changes.
    Call this if the screen configuration changes at runtime.
    """
    global SIZES
    get_screen_dpi.cache_clear()
    get_scale_factor.cache_clear()
    SIZES = get_scaled_sizes()
