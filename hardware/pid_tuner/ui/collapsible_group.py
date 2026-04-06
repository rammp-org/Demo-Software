"""
Collapsible Group Box widget for creating collapsible panels.

This provides a group box that can be collapsed by clicking on the header,
with smooth animation for expand/collapse transitions.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QLabel,
)
from PyQt6.QtCore import Qt, QPropertyAnimation, QEasingCurve, pyqtSignal

from .theme import THEME
from .scaling import SIZES, scaled


class CollapsibleGroupBox(QWidget):
    """
    A group box that can be collapsed by clicking the header.

    Features:
    - Click header to toggle collapsed/expanded state
    - Smooth animation for collapse/expand
    - Visual indicator showing collapsed state
    - Preserves content layout when collapsed
    """

    # Signal emitted when collapsed state changes
    collapsed_changed = pyqtSignal(bool)

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._title = title
        self._collapsed = False
        self._content_height = 0

        self._setup_ui()

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header (clickable)
        self._header = QFrame()
        self._header.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME.surface0};
                border-radius: {scaled(4)}px;
                padding: {scaled(4)}px;
            }}
            QFrame:hover {{
                background-color: {THEME.surface1};
            }}
        """)
        self._header.setCursor(Qt.CursorShape.PointingHandCursor)
        self._header.mousePressEvent = self._on_header_clicked

        header_layout = QHBoxLayout(self._header)
        header_layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )
        header_layout.setSpacing(SIZES["spacing_small"])

        # Collapse indicator (arrow)
        self._indicator = QLabel("\u25bc")  # Down arrow when expanded
        self._indicator.setFixedWidth(scaled(16))
        self._indicator.setStyleSheet(f"color: {THEME.subtext0};")
        header_layout.addWidget(self._indicator)

        # Title
        self._title_label = QLabel(self._title)
        self._title_label.setStyleSheet(f"font-weight: bold; color: {THEME.text};")
        header_layout.addWidget(self._title_label)
        header_layout.addStretch()

        layout.addWidget(self._header)

        # Content container
        self._content = QFrame()
        self._content.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME.mantle};
                border-bottom-left-radius: {scaled(4)}px;
                border-bottom-right-radius: {scaled(4)}px;
            }}
        """)
        self._content_layout = QVBoxLayout(self._content)
        self._content_layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )
        self._content_layout.setSpacing(SIZES["spacing_small"])
        layout.addWidget(self._content)

        # Animation for smooth collapse/expand
        self._animation = QPropertyAnimation(self._content, b"maximumHeight")
        self._animation.setDuration(200)
        self._animation.setEasingCurve(QEasingCurve.Type.InOutQuad)

    def addWidget(self, widget: QWidget):
        """Add a widget to the collapsible content area."""
        self._content_layout.addWidget(widget)

    def addLayout(self, layout):
        """Add a layout to the collapsible content area."""
        self._content_layout.addLayout(layout)

    def setContentLayout(self, layout):
        """Replace the content layout."""
        # Clear existing layout
        while self._content_layout.count():
            item = self._content_layout.takeAt(0)
            if item.widget():
                item.widget().setParent(None)

        # Add all items from new layout
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                self._content_layout.addWidget(item.widget())
            elif item.layout():
                self._content_layout.addLayout(item.layout())

    def _on_header_clicked(self, event):
        """Toggle collapsed state when header is clicked."""
        self.setCollapsed(not self._collapsed)

    def setCollapsed(self, collapsed: bool):
        """Set the collapsed state with animation."""
        if collapsed == self._collapsed:
            return

        self._collapsed = collapsed

        if collapsed:
            # Collapsing: animate from current height to 0
            self._content_height = self._content.height()
            self._animation.setStartValue(self._content_height)
            self._animation.setEndValue(0)
            self._indicator.setText("\u25b6")  # Right arrow when collapsed
        else:
            # Expanding: animate from 0 to content height
            # Need to temporarily show to get proper size hint
            self._content.setMaximumHeight(16777215)  # Remove max height constraint
            target_height = self._content.sizeHint().height()
            self._content.setMaximumHeight(0)
            self._animation.setStartValue(0)
            self._animation.setEndValue(target_height)
            self._indicator.setText("\u25bc")  # Down arrow when expanded

        self._animation.start()
        self.collapsed_changed.emit(collapsed)

    def isCollapsed(self) -> bool:
        """Return whether the group is collapsed."""
        return self._collapsed

    @property
    def title(self) -> str:
        """Get the title of the group."""
        return self._title

    def setTitle(self, title: str):
        """Set the title of the group."""
        self._title = title
        self._title_label.setText(title)
