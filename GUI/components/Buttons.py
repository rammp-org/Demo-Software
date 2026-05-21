from PyQt5.QtWidgets import QPushButton
from PyQt5.QtGui import QIcon, QColor
from PyQt5.QtCore import QRect, QSize

BUTTON_COLORS = {
    "Default": QColor(240, 240, 240),  # Default color
    "LightBlue": QColor(173, 216, 230),  # Light Blue color
    "LightGreen": QColor(144, 238, 144),  # Light Green color
    "LightYellow": QColor(255, 255, 153),  # Light Yellow color
    "LightRed": QColor(255, 192, 203),  # Light Red color
    "BGColor": QColor(70, 65, 75),  # Background Color
}


# Internal base class (not intended for direct use)
class BaseButton(QPushButton):
    def __init__(self, parent, icon_path: str = None, geometry: tuple = None):
        super().__init__(parent)

        if geometry is None or len(geometry) != 4:
            raise ValueError(
                "Geometry must be a tuple of four integers (x, y, width, height)."
            )

        # Set geometry
        self.setGeometry(QRect(*geometry))

        # Set default color
        self.setBackgroundColor("Default")

        # Set icon and icon size
        if icon_path is not None:
            self.setIcon(QIcon(icon_path))

    def setBackgroundColor(self, color):
        if color not in BUTTON_COLORS:
            print("Invalid Color")
            return
        self.setStyleSheet(f"background-color: {BUTTON_COLORS[color].name()};")


# Public derived class for a regular PushButton
class PushButton(BaseButton):
    def __init__(self, parent, icon_path, geometry):
        super().__init__(parent, icon_path, geometry)
        self.setIconSize(QSize(75, 75))


# Public derived class for a NestedButton
class NestedButton(BaseButton):
    def __init__(self, parent, icon_path, geometry):
        super().__init__(parent, icon_path, geometry)
        # self.setIconSize(QSize(25, 25))
