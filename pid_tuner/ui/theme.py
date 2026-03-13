"""
Catppuccin Frappe theme definitions and stylesheet helpers.

Catppuccin is a community-driven pastel theme.
https://github.com/catppuccin/catppuccin
"""

from dataclasses import dataclass
from typing import Dict


@dataclass
class CatppuccinPalette:
    """Catppuccin color palette."""

    # Accent colors
    rosewater: str
    flamingo: str
    pink: str
    mauve: str
    red: str
    maroon: str
    peach: str
    yellow: str
    green: str
    teal: str
    sky: str
    sapphire: str
    blue: str
    lavender: str

    # Text colors
    text: str
    subtext1: str
    subtext0: str

    # Overlay colors
    overlay2: str
    overlay1: str
    overlay0: str

    # Surface colors
    surface2: str
    surface1: str
    surface0: str

    # Base colors
    base: str
    mantle: str
    crust: str


# Catppuccin Frappe palette
FRAPPE = CatppuccinPalette(
    rosewater="#f2d5cf",
    flamingo="#eebebe",
    pink="#f4b8e4",
    mauve="#ca9ee6",
    red="#e78284",
    maroon="#ea999c",
    peach="#ef9f76",
    yellow="#e5c890",
    green="#a6d189",
    teal="#81c8be",
    sky="#99d1db",
    sapphire="#85c1dc",
    blue="#8caaee",
    lavender="#babbf1",
    text="#c6d0f5",
    subtext1="#b5bfe2",
    subtext0="#a5adce",
    overlay2="#949cbb",
    overlay1="#838ba7",
    overlay0="#737994",
    surface2="#626880",
    surface1="#51576d",
    surface0="#414559",
    base="#303446",
    mantle="#292c3c",
    crust="#232634",
)

# Active theme (can be changed to support multiple themes)
THEME = FRAPPE

# Joint accent colors - one for each of the 6 joints
JOINT_COLORS = [
    THEME.red,  # Joint 1: RC
    THEME.peach,  # Joint 2: FC
    THEME.yellow,  # Joint 3: ML
    THEME.green,  # Joint 4: MR
    THEME.sapphire,  # Joint 5: ML_Carriage
    THEME.mauve,  # Joint 6: MR_Carriage
]

# State colors
STATE_COLORS = {
    0: THEME.blue,  # INIT
    1: THEME.green,  # IDLE
    2: THEME.yellow,  # TUNER_MODE
    3: THEME.red,  # ESTOP
    4: THEME.teal,  # SELF_LEVELING
    5: THEME.mauve,  # CONFIGURATION
    6: THEME.peach,  # AUTO_CURB_CLIMBING
}

STATE_NAMES = {
    0: "INIT",
    1: "IDLE",
    2: "TUNER_MODE",
    3: "ESTOP",
    4: "SELF_LEVELING",
    5: "CONFIGURATION",
    6: "AUTO_CURB",
}


def get_application_stylesheet() -> str:
    """Generate the main application stylesheet."""
    return f"""
        /* Main Window */
        QMainWindow, QWidget {{
            background-color: {THEME.base};
            color: {THEME.text};
        }}

        /* Labels */
        QLabel {{
            color: {THEME.text};
        }}

        /* Group Boxes */
        QGroupBox {{
            background-color: {THEME.surface0};
            border: 1px solid {THEME.surface1};
            border-radius: 6px;
            margin-top: 8px;
            padding-top: 8px;
            font-weight: bold;
            color: {THEME.text};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 10px;
            padding: 0 5px;
            color: {THEME.subtext1};
        }}

        /* Buttons */
        QPushButton {{
            background-color: {THEME.surface1};
            color: {THEME.text};
            border: 1px solid {THEME.surface2};
            border-radius: 4px;
            padding: 5px 12px;
            min-width: 60px;
        }}
        QPushButton:hover {{
            background-color: {THEME.surface2};
            border-color: {THEME.overlay0};
        }}
        QPushButton:pressed {{
            background-color: {THEME.overlay0};
        }}
        QPushButton:disabled {{
            background-color: {THEME.surface0};
            color: {THEME.overlay0};
            border-color: {THEME.surface1};
        }}
        QPushButton:checked {{
            background-color: {THEME.blue};
            color: {THEME.crust};
            border-color: {THEME.sapphire};
        }}

        /* Primary action buttons */
        QPushButton[primary="true"] {{
            background-color: {THEME.blue};
            color: {THEME.crust};
            border-color: {THEME.sapphire};
        }}
        QPushButton[primary="true"]:hover {{
            background-color: {THEME.sapphire};
        }}

        /* Danger buttons */
        QPushButton[danger="true"] {{
            background-color: {THEME.red};
            color: {THEME.crust};
            border-color: {THEME.maroon};
        }}
        QPushButton[danger="true"]:hover {{
            background-color: {THEME.maroon};
        }}

        /* Warning buttons */
        QPushButton[warning="true"] {{
            background-color: {THEME.peach};
            color: {THEME.crust};
            border-color: {THEME.yellow};
        }}
        QPushButton[warning="true"]:hover {{
            background-color: {THEME.yellow};
        }}

        /* Combo Boxes */
        QComboBox {{
            background-color: {THEME.surface0};
            color: {THEME.text};
            border: 1px solid {THEME.surface2};
            border-radius: 4px;
            padding: 4px 8px;
            min-width: 80px;
        }}
        QComboBox:hover {{
            border-color: {THEME.overlay0};
        }}
        QComboBox::drop-down {{
            border: none;
            width: 20px;
        }}
        QComboBox::down-arrow {{
            image: none;
            border-left: 4px solid transparent;
            border-right: 4px solid transparent;
            border-top: 6px solid {THEME.text};
            margin-right: 5px;
        }}
        QComboBox QAbstractItemView {{
            background-color: {THEME.surface0};
            color: {THEME.text};
            selection-background-color: {THEME.surface2};
            selection-color: {THEME.text};
            border: 1px solid {THEME.surface2};
        }}

        /* Spin Boxes */
        QSpinBox, QDoubleSpinBox {{
            background-color: {THEME.surface0};
            color: {THEME.text};
            border: 1px solid {THEME.surface2};
            border-radius: 4px;
            padding: 4px;
        }}
        QSpinBox:hover, QDoubleSpinBox:hover {{
            border-color: {THEME.overlay0};
        }}
        QSpinBox::up-button, QDoubleSpinBox::up-button,
        QSpinBox::down-button, QDoubleSpinBox::down-button {{
            background-color: {THEME.surface1};
            border: none;
            width: 16px;
        }}
        QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
        QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
            background-color: {THEME.surface2};
        }}

        /* Line Edits */
        QLineEdit {{
            background-color: {THEME.surface0};
            color: {THEME.text};
            border: 1px solid {THEME.surface2};
            border-radius: 4px;
            padding: 4px 8px;
        }}
        QLineEdit:hover {{
            border-color: {THEME.overlay0};
        }}
        QLineEdit:focus {{
            border-color: {THEME.blue};
        }}

        /* Text Edit */
        QTextEdit {{
            background-color: {THEME.mantle};
            color: {THEME.text};
            border: 1px solid {THEME.surface0};
            border-radius: 4px;
        }}

        /* Check Boxes */
        QCheckBox {{
            color: {THEME.text};
            spacing: 6px;
        }}
        QCheckBox::indicator {{
            width: 16px;
            height: 16px;
            border: 1px solid {THEME.surface2};
            border-radius: 3px;
            background-color: {THEME.surface0};
        }}
        QCheckBox::indicator:hover {{
            border-color: {THEME.overlay0};
        }}
        QCheckBox::indicator:checked {{
            background-color: {THEME.blue};
            border-color: {THEME.sapphire};
        }}

        /* Splitters */
        QSplitter::handle {{
            background-color: {THEME.surface1};
        }}
        QSplitter::handle:horizontal {{
            width: 2px;
        }}
        QSplitter::handle:vertical {{
            height: 2px;
        }}
        QSplitter::handle:hover {{
            background-color: {THEME.blue};
        }}

        /* Status Bar */
        QStatusBar {{
            background-color: {THEME.mantle};
            color: {THEME.subtext1};
            border-top: 1px solid {THEME.surface0};
        }}

        /* Scroll Bars */
        QScrollBar:vertical {{
            background-color: {THEME.mantle};
            width: 12px;
            margin: 0;
        }}
        QScrollBar::handle:vertical {{
            background-color: {THEME.surface2};
            border-radius: 4px;
            min-height: 20px;
            margin: 2px;
        }}
        QScrollBar::handle:vertical:hover {{
            background-color: {THEME.overlay0};
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
            height: 0;
        }}
        QScrollBar:horizontal {{
            background-color: {THEME.mantle};
            height: 12px;
            margin: 0;
        }}
        QScrollBar::handle:horizontal {{
            background-color: {THEME.surface2};
            border-radius: 4px;
            min-width: 20px;
            margin: 2px;
        }}
        QScrollBar::handle:horizontal:hover {{
            background-color: {THEME.overlay0};
        }}
        QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
            width: 0;
        }}

        /* Progress Bars */
        QProgressBar {{
            background-color: {THEME.surface0};
            border: 1px solid {THEME.surface1};
            border-radius: 4px;
            text-align: center;
            color: {THEME.text};
        }}
        QProgressBar::chunk {{
            background-color: {THEME.blue};
            border-radius: 3px;
        }}

        /* Tool Tips */
        QToolTip {{
            background-color: {THEME.surface0};
            color: {THEME.text};
            border: 1px solid {THEME.surface2};
            border-radius: 4px;
            padding: 4px;
        }}
    """


def get_plot_colors() -> Dict[str, str]:
    """Get colors for pyqtgraph plots."""
    return {
        "background": THEME.mantle,
        "foreground": THEME.text,
        "grid": THEME.surface1,
        "position": THEME.blue,
        "target": THEME.red,
        "velocity": THEME.green,
        "pwm": THEME.mauve,
    }


def get_console_stylesheet() -> str:
    """Get stylesheet for the serial console."""
    return f"""
        QTextEdit {{
            background-color: {THEME.crust};
            color: {THEME.text};
            border: 1px solid {THEME.surface0};
            font-family: 'Courier New', monospace;
            font-size: 9pt;
        }}
    """
