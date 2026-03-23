# PID Tuner GUI Improvement Plan

## Overview

This document outlines the comprehensive plan for 10 QoL improvements and larger changes to the PyQt6 PID Tuner GUI.

### Improvement Categories

| Priority | Item | Description | Complexity | Est. Time |
|----------|------|-------------|------------|-----------|
| 1 | #9 | Memory/Config Debug Viewer | Complex | 4+ hours |
| 2 | #1 | Start GUI Maximized | Quick | 30 min |
| 3 | #10 | Remember Window Configuration | Quick | 1-2 hours |
| 4 | #3 | Quick Jogging (Hold-to-Jog PWM) | Medium | 2 hours |
| 5 | #4 | Motor Mode Visibility | Medium | 2-3 hours |
| 6 | #2 | Collapsible Panels with Visibility Toggle | Medium | 3-4 hours |
| 7 | #5 | CSV Export Button (Visual Indicator) | Quick | 1 hour |
| 8 | #7 | Motor Limits Visualization | Medium | 3 hours |
| 9 | #6 | IMU Visualization Contrast | Medium | 2 hours |
| 10 | #8 | Set Position Offset | Complex | 4+ hours |

---

## 1. Memory/Config Structure Debug Viewer (Priority #1)

**Complexity:** Complex  
**Files:** New `pid_tuner/ui/config_viewer.py`, modifications to `control_panel.py` or `main_window.py`  
**Estimated Time:** 4+ hours

### Current State
- Config is loaded via `G<joint>` command per joint
- Response parsed to `ConfigData` dataclass in `protocol.py`
- Individual fields shown in scattered PID input boxes in control panel
- Serial console shows raw CONFIG responses but requires filtering

### Requirements
- Display full `MotorConfig` struct for all 6 joints in a single tabular view
- Allow comparison of all joints at a glance
- "Load All Configs" button to request G1-G6 in sequence
- Highlight values that changed since last load

### Implementation Plan

#### 1.1 Create `ConfigViewerWidget` class

```python
# pid_tuner/ui/config_viewer.py

class ConfigViewerWidget(QWidget):
    """
    Tabular debug view showing all motor configurations.
    Displays the full MotorConfig struct for all 6 joints.
    """
```

#### 1.2 UI Layout

```
+---------------------------------------------------------------------------------+
| Configuration Viewer                                         [Load All] [Refresh]|
+---------------------------------------------------------------------------------+
| Joint | pos_p | pos_i | pos_d | pos_ff | vel_p | vel_i | vel_d | vel_ff | ...   |
+-------+-------+-------+-------+--------+-------+-------+-------+--------+-------+
| 1 RC  | 5.00  | 0.10  | 0.01  | 0.00   | 1.00  | 0.00  | 0.00  | 0.50   | ...   |
| 2 FC  | 3.00  | 0.05  | 0.00  | 0.00   | 0.80  | 0.00  | 0.00  | 0.40   | ...   |
| ...   |       |       |       |        |       |       |       |        |       |
+-------+-------+-------+-------+--------+-------+-------+-------+--------+-------+

Full columns: Joint, pos_p, pos_i, pos_d, pos_ff, vel_p, vel_i, vel_d, vel_ff, 
              pos_lpf, vel_lpf, input_lpf, pos_min, pos_max
```

#### 1.3 Implementation Details

1. **Create QTableWidget** with 6 rows (joints) and 14 columns (config fields)
2. **Store previous values** in a dict to detect changes
3. **Highlight changed cells** with yellow background
4. **Load All button** sends G1, G2, G3, G4, G5, G6 commands sequentially with small delay
5. **Connect to `data_store.config_updated` signal** to update table when config received
6. **Add to UI** as either:
   - A collapsible panel in control_panel.py (if collapsible panels are implemented first)
   - A separate dockable widget
   - A dialog accessible via menu/button

#### 1.4 Column Definitions

| Column | Field | Description |
|--------|-------|-------------|
| 0 | Joint | Joint ID and short name (e.g., "1 RC") |
| 1 | pos_p | Position P gain |
| 2 | pos_i | Position I gain |
| 3 | pos_d | Position D gain |
| 4 | pos_ff | Position Feed-Forward |
| 5 | vel_p | Velocity P gain |
| 6 | vel_i | Velocity I gain |
| 7 | vel_d | Velocity D gain |
| 8 | vel_ff | Velocity Feed-Forward |
| 9 | pos_lpf | Position LPF alpha |
| 10 | vel_lpf | Velocity LPF alpha |
| 11 | input_lpf | Input LPF alpha |
| 12 | pos_min | Position limit min |
| 13 | pos_max | Position limit max |

#### 1.5 Data Flow

```
User clicks "Load All"
    -> SerialHandler.get_config(1..6) 
    -> Teensy sends CONFIG,1,... through CONFIG,6,...
    -> ProtocolParser.parse_line() returns ConfigData
    -> DataStore.set_config() stores and emits config_updated signal
    -> ConfigViewerWidget._on_config_updated() updates table row
```

---

## 2. Start GUI Maximized (Priority #2)

**Complexity:** Quick win  
**Files:** `pid_tuner/ui/main_window.py`  
**Estimated Time:** 30 minutes

### Current State
- Window starts at minimum size defined by `setMinimumSize()` at line 69
- No initial window state set

### Implementation

Add to `MainWindow.__init__()` after `_setup_ui()` call (around line 60):

```python
def __init__(self):
    super().__init__()
    # ... existing code ...
    
    self._setup_ui()
    self._setup_status_bar()
    
    # Start maximized
    self.showMaximized()
    
    # Refresh ports on startup
    self._refresh_ports()
```

### Notes
- `showMaximized()` keeps the title bar and window decorations
- This will be overridden by settings restoration once #10 is implemented
- Consider adding fallback: if restored geometry is invalid, default to maximized

---

## 3. Remember Window Configuration (Priority #3)

**Complexity:** Quick win  
**Files:** `pid_tuner/ui/main_window.py`  
**Estimated Time:** 1-2 hours

### Current State
- No settings persistence
- No use of `QSettings`
- User must reconfigure window layout every launch

### Implementation

#### 3.1 Add QSettings initialization

```python
from PyQt6.QtCore import QSettings

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize settings storage
        self._settings = QSettings("MEBot", "PIDTuner")
        
        # ... existing initialization ...
        
        # Restore settings after UI setup
        self._restore_settings()
```

#### 3.2 Settings to persist

| Setting Key | Type | Description |
|-------------|------|-------------|
| `geometry` | QByteArray | Window position and size |
| `windowState` | QByteArray | Maximized/minimized state |
| `main_splitter` | list[int] | Main horizontal splitter sizes |
| `left_splitter` | list[int] | Left vertical splitter sizes |
| `last_port` | str | Last used serial port |
| `last_baud` | int | Last used baud rate |
| `last_joint` | int | Last selected joint |
| `plot_time_window` | int | Plot time window (seconds) |
| `plot_visibility` | dict | Which plots are visible |
| `console_filter` | str | Serial console filter |
| `collapsed_panels` | list[str] | Which panels are collapsed (once #6 implemented) |
| `hidden_panels` | list[str] | Which panels are hidden (once #6 implemented) |

#### 3.3 Restore settings method

```python
def _restore_settings(self):
    """Restore saved window configuration."""
    # Window geometry
    geometry = self._settings.value("geometry")
    if geometry:
        self.restoreGeometry(geometry)
    else:
        self.showMaximized()  # Default to maximized
    
    # Window state
    state = self._settings.value("windowState")
    if state:
        self.restoreState(state)
    
    # Splitter sizes (need to store references to splitters)
    main_sizes = self._settings.value("main_splitter")
    if main_sizes:
        self._main_splitter.setSizes([int(s) for s in main_sizes])
    
    left_sizes = self._settings.value("left_splitter")
    if left_sizes:
        self._left_splitter.setSizes([int(s) for s in left_sizes])
    
    # Last serial port
    last_port = self._settings.value("last_port", "")
    if last_port:
        index = self._port_combo.findText(last_port)
        if index >= 0:
            self._port_combo.setCurrentIndex(index)
    
    # Last baud rate
    last_baud = self._settings.value("last_baud", "115200")
    self._baud_combo.setCurrentText(str(last_baud))
    
    # Last joint
    last_joint = self._settings.value("last_joint", 0, type=int)
    self._joint_combo.setCurrentIndex(last_joint)
```

#### 3.4 Save settings method

```python
def _save_settings(self):
    """Save current window configuration."""
    self._settings.setValue("geometry", self.saveGeometry())
    self._settings.setValue("windowState", self.saveState())
    self._settings.setValue("main_splitter", self._main_splitter.sizes())
    self._settings.setValue("left_splitter", self._left_splitter.sizes())
    self._settings.setValue("last_port", self._port_combo.currentText())
    self._settings.setValue("last_baud", self._baud_combo.currentText())
    self._settings.setValue("last_joint", self._joint_combo.currentIndex())
```

#### 3.5 Update closeEvent

```python
def closeEvent(self, event):
    """Handle window close event."""
    self._save_settings()
    self._serial_handler.disconnect()
    event.accept()
```

#### 3.6 Store splitter references

Modify `_setup_ui()` to store splitter references:

```python
# Store reference for settings persistence
self._main_splitter = main_splitter
self._left_splitter = left_splitter
```

---

## 4. Quick Jogging (Hold-to-Jog PWM Buttons) (Priority #4)

**Complexity:** Medium  
**Files:** `pid_tuner/ui/control_panel.py`  
**Estimated Time:** 2 hours

### Current State
- Step/Jog group exists with amplitude-based stepping
- No direct PWM control buttons
- No hold-to-jog functionality

### Requirements
- 4 buttons: -0.2, -0.1, +0.1, +0.2 (as fractions of max PWM)
- Hold button to apply PWM, release to stop (PWM=0)
- Stop button to explicitly set PWM=0
- Works in Open Loop mode

### Implementation

#### 4.1 Add Quick Jog group after Step/Jog group

```python
def _create_quick_jog_group(self) -> QGroupBox:
    """Create quick jog buttons for open-loop PWM control."""
    group = QGroupBox("Quick Jog (Open Loop)")
    layout = QVBoxLayout(group)
    layout.setSpacing(SIZES["spacing_medium"])
    
    # Info label
    info_label = QLabel("Hold button to jog, release to stop")
    info_label.setStyleSheet(f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;")
    layout.addWidget(info_label)
    
    # Jog buttons row
    btn_layout = QHBoxLayout()
    btn_layout.setSpacing(SIZES["spacing_small"])
    
    # PWM values as fractions (will be multiplied by 32767)
    jog_values = [(-0.2, "-20%"), (-0.1, "-10%"), (0.1, "+10%"), (0.2, "+20%")]
    
    for pwm_fraction, label in jog_values:
        btn = QPushButton(label)
        btn.setAutoRepeat(False)  # We handle press/release manually
        # Connect press and release events
        btn.pressed.connect(lambda pf=pwm_fraction: self._on_jog_pressed(pf))
        btn.released.connect(self._on_jog_released)
        btn_layout.addWidget(btn)
    
    layout.addLayout(btn_layout)
    
    # Stop button
    self._jog_stop_btn = QPushButton("STOP (PWM=0)")
    self._jog_stop_btn.setStyleSheet(
        f"background-color: {THEME.red}; color: {THEME.crust};"
    )
    self._jog_stop_btn.clicked.connect(self._on_jog_stop)
    layout.addWidget(self._jog_stop_btn)
    
    return group
```

#### 4.2 Jog handler methods

```python
def _on_jog_pressed(self, pwm_fraction: float):
    """Handle jog button press - start jogging."""
    joint_id = self._data_store.selected_joint
    
    # Ensure we're in open loop mode
    if self._current_mode != MODE_OPEN_LOOP:
        # Optionally auto-switch to open loop, or show warning
        pass
    
    # Calculate PWM value (32767 is max PWM)
    pwm_value = int(pwm_fraction * 32767)
    
    # Send target command
    self._serial_handler.set_target(joint_id, pwm_value)
    self._data_store.set_target(joint_id, pwm_value)

def _on_jog_released(self):
    """Handle jog button release - stop jogging."""
    self._on_jog_stop()

def _on_jog_stop(self):
    """Stop all jogging - set PWM to 0."""
    joint_id = self._data_store.selected_joint
    self._serial_handler.set_target(joint_id, 0)
    self._data_store.set_target(joint_id, 0)
```

#### 4.3 Add to layout

In `_setup_ui()`, add after step group:

```python
# Quick Jog
layout.addWidget(self._create_quick_jog_group())
```

---

## 5. Motor Mode Visibility Enhancement (Priority #5)

**Complexity:** Medium  
**Files:** `pid_tuner/ui/control_panel.py`, `pid_tuner/ui/encoder_overview.py`  
**Estimated Time:** 2-3 hours

### Current State
- Mode shown as text label in "Current Values" group
- Mode dropdown in "Mode & PID Control" group
- No per-joint mode indication in encoder overview

### Requirements
- Large color banner showing selected motor's mode on control panel
- Per-joint mode indicators on encoder bars
- Color coding: Red = Open Loop, Yellow = Velocity, Blue = Position

### Implementation

#### 5.1 Mode color constants

Add to top of `control_panel.py`:

```python
MODE_COLORS = {
    MODE_OPEN_LOOP: THEME.red,
    MODE_VELOCITY: THEME.yellow,
    MODE_POSITION: THEME.blue,
}
```

#### 5.2 Create mode banner widget

Add above the scroll area in control panel:

```python
def _create_mode_banner(self) -> QWidget:
    """Create a prominent mode indicator banner."""
    self._mode_banner = QFrame()
    self._mode_banner.setFixedHeight(scaled(40))
    self._mode_banner.setStyleSheet(f"""
        QFrame {{
            background-color: {MODE_COLORS[MODE_OPEN_LOOP]};
            border-radius: {scaled(4)}px;
        }}
    """)
    
    layout = QHBoxLayout(self._mode_banner)
    layout.setContentsMargins(SIZES["margin_medium"], 0, SIZES["margin_medium"], 0)
    
    self._mode_banner_label = QLabel("MODE: OPEN LOOP")
    self._mode_banner_label.setStyleSheet(f"""
        font-size: {SIZES['font_large']}pt;
        font-weight: bold;
        color: {THEME.crust};
    """)
    self._mode_banner_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    layout.addWidget(self._mode_banner_label)
    
    return self._mode_banner
```

#### 5.3 Update banner when mode changes

```python
def _update_mode_banner(self):
    """Update the mode banner color and text."""
    mode_name = MODE_NAMES[self._current_mode]
    mode_color = MODE_COLORS[self._current_mode]
    
    self._mode_banner.setStyleSheet(f"""
        QFrame {{
            background-color: {mode_color};
            border-radius: {scaled(4)}px;
        }}
    """)
    self._mode_banner_label.setText(f"MODE: {mode_name.upper()}")
```

#### 5.4 Add mode indicators to EncoderBar

In `encoder_overview.py`, modify `EncoderBar`:

```python
class EncoderBar(QWidget):
    def __init__(self, ...):
        # ... existing code ...
        self._mode = MODE_OPEN_LOOP  # Track mode for this joint
    
    def set_mode(self, mode: int):
        """Set the control mode for this joint."""
        self._mode = mode
        self.update()
    
    def paintEvent(self, event):
        # ... existing painting code ...
        
        # Draw mode indicator dot (top-right corner of bar)
        mode_color = QColor(MODE_COLORS.get(self._mode, THEME.overlay0))
        indicator_size = scaled(10)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(mode_color))
        painter.drawEllipse(
            bar_left + bar_width - indicator_size - scaled(4),
            margin + scaled(2),
            indicator_size,
            indicator_size
        )
```

#### 5.5 Update mode indicators from telemetry

**Note:** This requires mode information to be included in telemetry from the Teensy, OR we track modes sent from the GUI.

Option A (Track in GUI):
- When mode is set via `M<joint>:<mode>`, update `DataStore` with mode for that joint
- EncoderOverview reads mode from DataStore

Option B (Firmware enhancement):
- Add mode to telemetry: `TELEMETRY,...,<mode1..6>,...`
- Parse and store in DataStore

---

## 6. Collapsible Panels with Visibility Toggle (Priority #6)

**Complexity:** Medium  
**Files:** New `pid_tuner/ui/collapsible_group.py`, modifications to `control_panel.py`  
**Estimated Time:** 3-4 hours

### Current State
- All groups are standard `QGroupBox` widgets
- Always visible, always expanded
- Scroll area handles overflow

### Requirements
- Click header to collapse/expand each group
- Checkbox menu to show/hide groups entirely
- Remember collapsed/hidden state in settings
- Animation for smooth collapse/expand

### Implementation

#### 6.1 Create CollapsibleGroupBox widget

```python
# pid_tuner/ui/collapsible_group.py

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QFrame, QLabel, QSizePolicy
from PyQt6.QtCore import Qt, QPropertyAnimation, QEasingCurve, pyqtProperty
from PyQt6.QtGui import QFont

class CollapsibleGroupBox(QWidget):
    """
    A group box that can be collapsed by clicking the header.
    """
    
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._title = title
        self._collapsed = False
        self._content_height = 0
        
        self._setup_ui()
    
    def _setup_ui(self):
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
        header_layout.setContentsMargins(SIZES["margin_small"], 0, SIZES["margin_small"], 0)
        
        # Collapse indicator
        self._indicator = QLabel("V")  # Down arrow when expanded
        self._indicator.setFixedWidth(scaled(16))
        header_layout.addWidget(self._indicator)
        
        # Title
        self._title_label = QLabel(self._title)
        self._title_label.setStyleSheet(f"font-weight: bold; color: {THEME.text};")
        header_layout.addWidget(self._title_label)
        header_layout.addStretch()
        
        layout.addWidget(self._header)
        
        # Content container
        self._content = QWidget()
        self._content_layout = QVBoxLayout(self._content)
        self._content_layout.setContentsMargins(
            SIZES["margin_small"], SIZES["margin_small"],
            SIZES["margin_small"], SIZES["margin_small"]
        )
        layout.addWidget(self._content)
        
        # Animation
        self._animation = QPropertyAnimation(self._content, b"maximumHeight")
        self._animation.setDuration(200)
        self._animation.setEasingCurve(QEasingCurve.Type.InOutQuad)
    
    def addWidget(self, widget: QWidget):
        """Add a widget to the collapsible content area."""
        self._content_layout.addWidget(widget)
    
    def setLayout(self, layout):
        """Set the layout for the content area."""
        # Clear existing
        while self._content_layout.count():
            self._content_layout.takeAt(0)
        # Add all widgets from new layout
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                self._content_layout.addWidget(item.widget())
    
    def _on_header_clicked(self, event):
        """Toggle collapsed state."""
        self.setCollapsed(not self._collapsed)
    
    def setCollapsed(self, collapsed: bool):
        """Set the collapsed state."""
        self._collapsed = collapsed
        
        if collapsed:
            self._content_height = self._content.height()
            self._animation.setStartValue(self._content_height)
            self._animation.setEndValue(0)
            self._indicator.setText(">")  # Right arrow when collapsed
        else:
            self._animation.setStartValue(0)
            self._animation.setEndValue(self._content_height or self._content.sizeHint().height())
            self._indicator.setText("V")  # Down arrow when expanded
        
        self._animation.start()
    
    def isCollapsed(self) -> bool:
        return self._collapsed
    
    @property
    def title(self) -> str:
        return self._title
```

#### 6.2 Create panel visibility menu

Add to control panel or main window:

```python
def _create_panel_visibility_menu(self) -> QMenu:
    """Create menu for toggling panel visibility."""
    menu = QMenu("Panels", self)
    
    for panel_name, panel_widget in self._panels.items():
        action = QAction(panel_name, menu)
        action.setCheckable(True)
        action.setChecked(panel_widget.isVisible())
        action.triggered.connect(lambda checked, w=panel_widget: w.setVisible(checked))
        menu.addAction(action)
    
    return menu
```

#### 6.3 Migrate existing groups

Replace `QGroupBox` creation with `CollapsibleGroupBox`:

```python
# Before
group = QGroupBox("Current Values")

# After  
group = CollapsibleGroupBox("Current Values")
```

#### 6.4 Store panel references for visibility control

```python
self._panels = {
    "Current Values": self._status_group,
    "Performance Analysis": self._analysis_group,
    "Mode & PID Control": self._pid_group,
    "Target Control": self._target_group,
    "Step/Jog Input": self._step_group,
    "Quick Jog": self._quick_jog_group,
    "Sine Wave Input": self._sine_group,
    "Self Leveling": self._leveling_group,
    "IMU Display": self._imu_display,
    "3D IMU": self._imu_3d_widget,
    "Config Viewer": self._config_viewer,  # Once implemented
}
```

---

## 7. CSV Export Button (Visual Indicator) (Priority #7)

**Complexity:** Quick win  
**Files:** `pid_tuner/ui/plot_widget.py`  
**Estimated Time:** 1 hour

### Current State
- Export available via right-click context menu (pyqtgraph built-in)
- No visual indication this feature exists
- Cumbersome multi-click process

### Requirements
- Add visible "Export CSV" button to plot toolbar
- Single-click export of all visible data
- Save dialog for file location

### Implementation

#### 7.1 Add export button to plot controls

In `plot_widget.py`, add to the controls bar:

```python
def _setup_controls(self):
    # ... existing controls ...
    
    # Export button
    self._export_btn = QPushButton("Export CSV")
    self._export_btn.setToolTip("Export all visible graph data to CSV file")
    self._export_btn.setStyleSheet(f"background-color: {THEME.green}; color: {THEME.crust};")
    self._export_btn.clicked.connect(self._on_export_csv)
    controls_layout.addWidget(self._export_btn)
```

#### 7.2 Export implementation

```python
def _on_export_csv(self):
    """Export visible graph data to CSV file."""
    from PyQt6.QtWidgets import QFileDialog
    import csv
    from datetime import datetime
    
    # Get save file path
    default_name = f"pid_tuner_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    file_path, _ = QFileDialog.getSaveFileName(
        self,
        "Export Graph Data",
        default_name,
        "CSV Files (*.csv);;All Files (*)"
    )
    
    if not file_path:
        return
    
    # Get data from data store
    joint_data = self._data_store.get_selected_joint_data()
    
    # Build headers and data based on visible plots
    headers = ["timestamp"]
    data_columns = [list(joint_data.timestamps)]
    
    if self._pos_visible:
        headers.extend(["position", "target"])
        data_columns.append(list(joint_data.positions))
        data_columns.append(list(joint_data.targets))
    
    if self._vel_visible:
        headers.append("velocity")
        data_columns.append(list(joint_data.velocities))
    
    if self._pwm_visible:
        headers.append("pwm")
        data_columns.append(list(joint_data.pwms))
    
    # Include linked joint if present
    linked_id = self._data_store.linked_joint
    if linked_id:
        linked_data = self._data_store.get_joint(linked_id)
        if linked_data:
            headers.extend(["linked_position", "linked_velocity", "linked_pwm"])
            data_columns.append(list(linked_data.positions))
            data_columns.append(list(linked_data.velocities))
            data_columns.append(list(linked_data.pwms))
    
    # Write CSV
    with open(file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        # Transpose columns to rows
        for i in range(len(data_columns[0])):
            row = [col[i] if i < len(col) else "" for col in data_columns]
            writer.writerow(row)
    
    # Show confirmation
    self._status_label.setText(f"Exported to {file_path}")
```

---

## 8. Motor Limits Visualization Enhancement (Priority #8)

**Complexity:** Medium  
**Files:** `pid_tuner/ui/encoder_overview.py`  
**Estimated Time:** 3 hours

### Current State
- Limits set via spinboxes in control panel
- EncoderBar shows position relative to limit range
- Limit switches shown as colored dots for joints 5,6

### Requirements
- Clear visual indication of where position is within limits
- Show numeric min/max values on the bar
- "Danger zone" coloring when approaching limits
- Better limit marker visibility

### Implementation

#### 8.1 Enhanced EncoderBar visualization

```
Current:
[J1 |=======XXXXXXXX............| +523.4]

Proposed:
[-1000|####XXXXXXXXXXXXXXXX....####|+2000] +523.4
       ^                         ^
    danger                    danger
    zone                      zone
```

#### 8.2 Update paintEvent in EncoderBar

```python
def paintEvent(self, event):
    painter = QPainter(self)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    
    # ... existing setup code ...
    
    # Draw limit labels
    font = QFont()
    font.setPointSize(SIZES["font_tiny"])
    painter.setFont(font)
    painter.setPen(QColor(THEME.subtext0))
    
    # Min limit label (left side)
    min_text = f"{self._min_val:.0f}"
    painter.drawText(
        bar_left, bar_rect_y + bar_rect_h + scaled(2),
        scaled(40), scaled(12),
        Qt.AlignmentFlag.AlignLeft, min_text
    )
    
    # Max limit label (right side)
    max_text = f"{self._max_val:.0f}"
    painter.drawText(
        bar_left + bar_width - scaled(40), bar_rect_y + bar_rect_h + scaled(2),
        scaled(40), scaled(12),
        Qt.AlignmentFlag.AlignRight, max_text
    )
    
    # Calculate danger zones (10% from each limit)
    danger_width = bar_width * 0.1
    
    # Draw danger zone backgrounds
    danger_color = QColor(THEME.red)
    danger_color.setAlpha(50)
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(QBrush(danger_color))
    
    # Left danger zone
    painter.drawRect(bar_rect_x, bar_rect_y, int(danger_width), bar_rect_h)
    
    # Right danger zone
    painter.drawRect(
        bar_rect_x + bar_width - int(danger_width),
        bar_rect_y, int(danger_width), bar_rect_h
    )
    
    # Draw limit markers (vertical lines at edges)
    painter.setPen(QPen(QColor(THEME.red), 2))
    painter.drawLine(bar_rect_x, bar_rect_y, bar_rect_x, bar_rect_y + bar_rect_h)
    painter.drawLine(
        bar_rect_x + bar_width, bar_rect_y,
        bar_rect_x + bar_width, bar_rect_y + bar_rect_h
    )
    
    # ... rest of existing painting ...
    
    # Color the position indicator based on proximity to limits
    range_span = self._max_val - self._min_val
    if range_span > 0:
        dist_to_min = abs(self._value - self._min_val) / range_span
        dist_to_max = abs(self._value - self._max_val) / range_span
        
        if dist_to_min < 0.1 or dist_to_max < 0.1:
            # In danger zone - red fill
            fill_color = QColor(THEME.red)
        elif dist_to_min < 0.2 or dist_to_max < 0.2:
            # Approaching limits - yellow fill
            fill_color = QColor(THEME.yellow)
        else:
            # Normal - joint color
            fill_color = self._color
```

---

## 9. IMU Visualization Contrast Improvement (Priority #9)

**Complexity:** Medium  
**Files:** `pid_tuner/ui/imu_3d_widget.py`  
**Estimated Time:** 2 hours

### Current State
- Both actual and target use `GLAxisItem`
- Actual: size 1.5
- Target: size 1.2
- Nearly identical appearance

### Requirements
- Actual IMU: Much thicker lines
- Target IMU: Thinner lines, kept as axis (not cube)
- Clear visual distinction between the two

### Implementation

#### 9.1 Create custom thick axis item

```python
import numpy as np
from pyqtgraph.opengl import GLLinePlotItem

class ThickAxisItem(GLLinePlotItem):
    """
    Custom axis item with configurable line width.
    """
    
    def __init__(self, size=1.0, width=3.0, **kwargs):
        # Create axis vertices
        # X axis: red
        # Y axis: green  
        # Z axis: blue
        
        verts = np.array([
            # X axis
            [0, 0, 0], [size, 0, 0],
            # Y axis
            [0, 0, 0], [0, size, 0],
            # Z axis
            [0, 0, 0], [0, 0, size],
        ])
        
        colors = np.array([
            # X axis - red
            [1, 0, 0, 1], [1, 0, 0, 1],
            # Y axis - green
            [0, 1, 0, 1], [0, 1, 0, 1],
            # Z axis - blue
            [0, 0, 1, 1], [0, 0, 1, 1],
        ])
        
        super().__init__(
            pos=verts,
            color=colors,
            width=width,
            mode='lines',
            **kwargs
        )
```

#### 9.2 Update IMU3DWidget

```python
def _setup_ui(self):
    # ... existing code ...
    
    # Actual Orientation Axes (THICK - very visible)
    self._actual_axis = ThickAxisItem(size=1.5, width=5.0)
    self._view.addItem(self._actual_axis)
    
    # Target Orientation Axes (thin, semi-transparent)
    self._target_axis = ThickAxisItem(size=1.2, width=1.5)
    # Make target more transparent by adjusting colors
    self._target_axis.setData(color=np.array([
        [1, 0.5, 0.5, 0.5], [1, 0.5, 0.5, 0.5],  # X - pale red
        [0.5, 1, 0.5, 0.5], [0.5, 1, 0.5, 0.5],  # Y - pale green
        [0.5, 0.5, 1, 0.5], [0.5, 0.5, 1, 0.5],  # Z - pale blue
    ]))
    self._view.addItem(self._target_axis)
```

#### 9.3 Add legend

```python
# Add legend explaining the visualization
legend_label = QLabel("Thick axes: Actual | Thin axes: Target")
legend_label.setStyleSheet(f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;")
layout.addWidget(legend_label)
```

---

## 10. Set Position Offset (Arbitrary Reference Point) (Priority #10)

**Complexity:** Complex  
**Files:** `pid_tuner/ui/control_panel.py`, `pid_tuner/serial_driver/protocol.py`, `pid_tuner/serial_driver/serial_handler.py`  
**Estimated Time:** 4+ hours (GUI only, firmware separate)

### Current State
- "Home Position" button sends `H<joint>` which zeros encoder at current position
- No way to set arbitrary offset
- Workaround: move to offset, home, move back

### Requirements
- "Set Position As" input: user enters desired position value
- When clicked: current physical position becomes that value
- Offset saved to EEPROM automatically

### GUI Implementation

#### 10.1 Add UI controls in Target Control group

```python
# In _create_target_group()

# Position offset row
offset_layout = QHBoxLayout()
offset_layout.setSpacing(SIZES["spacing_small"])

offset_layout.addWidget(QLabel("Set Position As:"))
self._position_offset_input = QLineEdit("0")
self._position_offset_input.setValidator(QDoubleValidator())
self._position_offset_input.setMinimumWidth(SIZES["input_min_width"])
self._position_offset_input.setToolTip(
    "Set the current physical position to this value.\n"
    "Example: If motor is at +25 ticks and you enter 150,\n"
    "the position will now read 150 ticks."
)
offset_layout.addWidget(self._position_offset_input)

self._set_offset_btn = QPushButton("Set Offset")
self._set_offset_btn.setToolTip("Set current position to the specified value")
self._set_offset_btn.clicked.connect(self._on_set_position_offset)
self._set_offset_btn.setStyleSheet(
    f"background-color: {THEME.mauve}; color: {THEME.crust};"
)
offset_layout.addWidget(self._set_offset_btn)

layout.addLayout(offset_layout)
```

#### 10.2 Handler method (GUI side)

```python
def _on_set_position_offset(self):
    """Set the current position to a specified value."""
    desired_position = self._get_float_from_lineedit(self._position_offset_input, 0.0)
    joint_id = self._data_store.selected_joint
    
    # Send offset command to Teensy
    # The Teensy will calculate: offset = desired_position - current_raw_position
    self._serial_handler.set_position_offset(joint_id, desired_position)
```

#### 10.3 Protocol additions

In `protocol.py`:

```python
@staticmethod
def set_position_offset(joint_id: int, desired_position: float) -> bytes:
    """
    Set encoder offset so current position reads as desired_position.
    Teensy calculates: offset = desired_position - current_raw_position
    """
    cmd = f"O{joint_id}:{desired_position:.2f}\n"
    return cmd.encode("ascii")
```

In `serial_handler.py`:

```python
def set_position_offset(self, joint_id: int, desired_position: float):
    """Set position offset for joint."""
    cmd = ProtocolEncoder.set_position_offset(joint_id, desired_position)
    self._write_queue.put(cmd)
```

### Firmware Requirements (NOT IMPLEMENTED IN THIS PLAN)

> **Note:** The following firmware changes are required but will be handled separately.

1. **Add new command handler** for `O<joint>:<desired_position>`
2. **Calculate offset**: `offset = desired_position - encoder.getRawPosition()`
3. **Apply offset** to encoder readings: `position = raw_position + offset`
4. **Save offset to EEPROM** automatically when set
5. **Load offset from EEPROM** on boot
6. **Add offset field to MotorConfig struct** if not already present

The firmware implementation should:
- Parse the `O` command in the command handler
- Store offset in `MotorConfig.saved_position` or a new `position_offset` field
- Apply offset in `EncoderContainer::getPosition()`
- Auto-save to EEPROM after setting

---

## File Change Summary

| File | Changes |
|------|---------|
| `main_window.py` | Added `showMaximized()` fallback, QSettings persistence, splitter references, `_restore_settings()`, `_save_settings()`, mode_changed signal connection |
| `control_panel.py` | Added `MODE_COLORS`, `mode_changed` signal, mode banner, `_create_quick_jog_group()`, position offset UI, integrated `ConfigViewerWidget` |
| `encoder_overview.py` | Added `MODE_COLORS`, `set_mode()` on `EncoderBar`, mode indicator dot in `paintEvent`, danger zones, dynamic fill color, limit edge markers, `set_mode_for_joint()` / `set_mode_for_all()` on `EncoderOverview` |
| `plot_widget.py` | Added "Export CSV" button and `_on_export_csv()` handler |
| `imu_3d_widget.py` | Created `ThickAxisItem` class, replaced `GLAxisItem` with thick (5.0) actual and thin (1.5) target axes, added legend |
| `protocol.py` | Added `set_position_offset()` static method to `ProtocolEncoder` |
| `serial_handler.py` | Added `set_position_offset()` method |
| **NEW** `collapsible_group.py` | `CollapsibleGroupBox` widget with animated collapse/expand (ready to integrate) |
| **NEW** `config_viewer.py` | `ConfigViewerWidget`: QTableWidget for all 6 joints × 14 config fields, Load All, change highlighting |

---

## Implementation Status

### DONE

| Priority | Item | Status | Notes |
|----------|------|--------|-------|
| 1 | Config Viewer | ✅ Complete | New `config_viewer.py`, integrated in `control_panel.py` |
| 2 | Start Maximized | ✅ Complete | `_restore_settings()` defaults to `showMaximized()` if no saved geometry |
| 3 | Settings Persistence | ✅ Complete | QSettings saves/restores geometry, splitters, last port/baud/joint |
| 4 | Quick Jogging | ✅ Complete | Hold-to-jog buttons (-20%, -10%, +10%, +20%) + STOP button |
| 5 | Mode Visibility | ✅ Complete | Color banner at top of control panel + mode dot on each encoder bar |
| 6 | Collapsible Panels | ✅ Complete | All groups migrated to `CollapsibleGroupBox`; Panels menu for visibility toggle |
| 7 | CSV Export | ✅ Complete | Export CSV button in plot toolbar; exports all visible series |
| 8 | Limits Visualization | ✅ Complete | Danger zones (10%), dynamic fill color, red edge markers |
| 9 | IMU Contrast | ✅ Complete | `ThickAxisItem` (width=5 actual, width=1.5 target), legend |
| 10 | Position Offset (GUI) | ✅ Complete | UI controls + protocol/handler; firmware 'O' command pending |

### DEFERRED / PARTIAL

- **Priority 10 firmware**: The Teensy firmware needs an `O<joint>:<position>` command handler. See firmware requirements in section 10 above.

---

## Testing Checklist

### Priority 1: Config Viewer
- [ ] Table displays all 6 joints
- [ ] "Load All" fetches configs for all joints sequentially (100 ms delay)
- [ ] Changed cells highlighted yellow after refresh
- [ ] Values match serial console `CONFIG,N,...` output

### Priority 2: Maximized Window
- [ ] Window starts maximized on first launch (no saved settings)
- [ ] Title bar and window controls remain visible

### Priority 3: Settings Persistence
- [ ] Window geometry restored on relaunch
- [ ] Splitter positions (main + left) restored
- [ ] Last port/baud pre-selected in dropdowns
- [ ] Last joint selection restored

### Priority 4: Quick Jogging
- [ ] Hold button sends correct PWM (e.g. +10% → 3277)
- [ ] Releasing button sends PWM=0
- [ ] STOP button sends PWM=0 independently
- [ ] Linked joint also jogged / stopped

### Priority 5: Mode Visibility
- [ ] Banner is Red for Open Loop, Yellow for Velocity, Blue for Position
- [ ] Encoder bar mode dot color matches
- [ ] Colors update immediately when mode combo changes

### Priority 6: Collapsible Panels
- [ ] All groups converted to `CollapsibleGroupBox`
- [ ] `CollapsibleGroupBox` collapses/expands with animation
- [ ] Arrow indicator flips between ▼ and ▶
- [ ] "Panels" button appears at top of control panel
- [ ] Menu shows checkboxes for each panel
- [ ] Unchecking a panel hides it completely
- [ ] "Show All" / "Hide All" utility actions work
- [ ] Performance Analysis and Sine Wave start collapsed by default

### Priority 7: CSV Export
- [ ] "Export CSV" button visible in plot control bar
- [ ] File dialog opens with timestamped default filename
- [ ] CSV includes only columns matching visible plot checkboxes
- [ ] Linked joint columns appended when linked joint is set
- [ ] Confirmation dialog shows sample count

### Priority 8: Limits Visualization
- [ ] Semi-transparent red zones appear at 10% from each limit
- [ ] Fill turns yellow when within 20% of a limit
- [ ] Fill turns red when within 10% of a limit
- [ ] Red edge markers drawn at exact limit positions

### Priority 9: IMU Contrast
- [ ] Actual axes are visibly thick (width 5)
- [ ] Target axes are thin and semi-transparent (width 1.5, alpha 0.5)
- [ ] "Thick: Actual | Thin: Target" legend visible

### Priority 10: Position Offset
- [ ] "Set Position As" input and "Set Offset" button present in Target Control group
- [ ] Button click sends `O<joint>:<value>` command over serial
- [ ] (Firmware pending) Teensy applies offset and saves to EEPROM

---

## 13. Strain Gauge Integration (Priority #13)

**Complexity:** Medium
**Files:**
- NEW `hardware/rammp_prototype_driver/firmware/Base/src/StrainGauge/StrainGauge.h`
- NEW `hardware/rammp_prototype_driver/firmware/Base/src/StrainGauge/StrainGauge.cpp`
- `hardware/rammp_prototype_driver/firmware/Base/Base.ino`
- `pid_tuner/serial_driver/protocol.py`
- `pid_tuner/data/data_store.py`
- `docs/shared/SERIAL_PROTOCOL.md`

### Background

`Constants.h` already defines the four load cell analog pins:
```cpp
#define FC_LOADCELL_PIN A17
#define ML_LOADCELL_PIN A15
#define MR_LOADCELL_PIN A14
#define RC_LOADCELL_PIN A16
```

All four strain gauges will be supported (FC, RC, ML, MR). FC and RC values will be streamed in telemetry first (fields 53–54); ML and MR will follow (55–56).

### Firmware — `StrainGauge` Class

```
src/StrainGauge/
├── StrainGauge.h    — Public interface
└── StrainGauge.cpp  — analogRead + IIR LPF
```

**Public API:**
```cpp
StrainGauge(int pin, float lpf_alpha = 0.5f);
void update(float dt);       // Call each loop(); dt reserved for future use
float getValue() const;      // Returns current filtered ADC reading
void setLpfAlpha(float a);
float getLpfAlpha() const;
```

**Filter:**
Same IIR pattern used across the project:
```cpp
_filtered_value += lpf_alpha * (raw - _filtered_value);
```
Default `lpf_alpha = 0.5` (moderate smoothing — expected noisy analog signal).

### Firmware — `Base.ino` Changes

1. **Include** `src/StrainGauge/StrainGauge.h`
2. **Global objects**: `StrainGauge sg_rc(RC_LOADCELL_PIN)`, `sg_fc(FC_LOADCELL_PIN)`, `sg_ml(ML_LOADCELL_PIN)`, `sg_mr(MR_LOADCELL_PIN)`
3. **`SystemTelemetry` struct**: add `sg_rc_value`, `sg_fc_value`, `sg_ml_value`, `sg_mr_value` float fields
4. **`updateTelemetry()`**: copy current gauge values into the struct
5. **`loop()` — Read Sensors**: call `sg_rc.update(dt)`, `sg_fc.update(dt)`, `sg_ml.update(dt)`, `sg_mr.update(dt)` alongside `EContr.retrieve_readings()`
6. **`sendTelemetry()`**: append four new values at the end (fields 53–56), maintaining full backward compatibility

### Python / GUI Changes

**`protocol.py`:**
- Add `sg_rc_value`, `sg_fc_value`, `sg_ml_value`, `sg_mr_value` float fields (default `0.0`) to `EncoderData`
- In `parse_line()`, add a `>= 53` length check after the existing leveling check to parse `values[49:53]` as the four strain gauge readings

**`data_store.py`:**
- Add four private `_sg_*_value: float` fields
- Add read-only `@property` for each
- In `process_encoder_data()`, update all four from the incoming `EncoderData`

### Telemetry Packet After Change

Fields 53–56 appended to the existing 52-field packet:
```
TELEMETRY,...,<sg_rc>,<sg_fc>,<sg_ml>,<sg_mr>\n
```

### Implementation Status: COMPLETE

**What was done:**

| File | Change |
|------|--------|
| NEW `src/StrainGauge/StrainGauge.h` | Class declaration: constructor, `update(dt)`, `getValue()`, `setLpfAlpha()`, `getLpfAlpha()`. Default `lpf_alpha = 0.5`. |
| NEW `src/StrainGauge/StrainGauge.cpp` | `analogRead()` + IIR LPF (`_filtered += alpha * (raw - _filtered)`). `pinMode(INPUT)` in constructor. `(void)dt` reserved for future use. |
| `Base.ino` — include | Added `#include "src/StrainGauge/StrainGauge.h"` |
| `Base.ino` — globals | Instantiated `sg_rc`, `sg_fc`, `sg_ml`, `sg_mr` using pins from `Constants.h` |
| `Base.ino` — `SystemTelemetry` | Added `sg_rc_value`, `sg_fc_value`, `sg_ml_value`, `sg_mr_value` float fields |
| `Base.ino` — `updateTelemetry()` | Copies `sg_*.getValue()` into telemetry struct |
| `Base.ino` — `loop()` sensor block | Calls `sg_*.update(dt)` alongside `EContr.retrieve_readings()` |
| `Base.ino` — `sendTelemetry()` | Changed last `z_target_mr` line from `println` → `print`, appended 4 strain gauge fields, final `println` on `sg_mr_value` |
| `pid_tuner/serial_driver/protocol.py` | Added `sg_rc_value`, `sg_fc_value`, `sg_ml_value`, `sg_mr_value` to `EncoderData`. Added `>= 53` parse block for `values[49:53]`. |
| `pid_tuner/data/data_store.py` | Added 4 private `_sg_*_value` fields, 4 read-only `@property` accessors, and update assignments in `process_encoder_data()`. |
| `docs/shared/SERIAL_PROTOCOL.md` | Updated format string and field list to reflect 53 total values (fields 53–56 = strain gauges). |

---

## 14. Strain Gauge Display Panel (Priority #14)

**Complexity:** Medium
**Files:**
- NEW `pid_tuner/ui/strain_gauge_display.py`
- `pid_tuner/data/data_store.py` (add signal)
- `pid_tuner/ui/control_panel.py` (integration)

### Requirements

1. **Bar graph visualization** with numeric values written inside each bar
2. **Dedicated signal** `strain_gauge_updated` in DataStore
3. **Color gradient** from green to orange based on magnitude; scale configurable
4. **Default state:** expanded (not collapsed)
5. **Collapsible/hidable** via existing panel menu system

### Interface Specification

#### DataStore Changes (`data_store.py`)

```python
# Add signal (around line 332, after leveling_updated):
strain_gauge_updated = pyqtSignal()  # Emitted when strain gauge data is updated

# In process_encoder_data() (after updating sg_* values):
self.strain_gauge_updated.emit()
```

#### New Widget: `StrainGaugeDisplay` (`strain_gauge_display.py`)

**Public Interface:**
```python
class StrainGaugeDisplay(QWidget):
    """
    Bar graph visualization of strain gauge (load cell) readings.
    
    Features:
    - 4 horizontal bars: RC, FC, ML, MR
    - Numeric value displayed inside each bar
    - Color gradient from green (low) to orange (high)
    - Configurable max_scale for gradient mapping
    """
    
    # Configurable scale for color gradient
    DEFAULT_MAX_SCALE = 4095.0  # Teensy 4.1 ADC max (12-bit)
    
    def __init__(self, data_store: DataStore, parent=None):
        ...
    
    def set_max_scale(self, scale: float):
        """Set the maximum scale for the color gradient."""
        ...
```

**Visual Layout:**
```
+------------------------------------------+
| RC  [████████████ 2345.6]                |
| FC  [████████████████ 3456.7]            |
| ML  [████████ 1234.5]                    |
| MR  [██████████████ 3890.2]              |
+------------------------------------------+
```

**Bar Width Calculation:**
- Bar width = `(value / max_scale) * available_width`
- Numeric value centered inside bar
- Background color from green→orange gradient based on `value / max_scale`

**Color Gradient:**
- 0% → Green (`#a6e3a1`)
- 100% → Orange (`#fab387`)
- Interpolated linearly in between

#### Control Panel Integration (`control_panel.py`)

```python
# Add import:
from .strain_gauge_display import StrainGaugeDisplay

# In _setup_ui(), after Config Viewer (around line 231):
# Strain Gauge Display (wrapped in collapsible group)
self._strain_gauge_display = StrainGaugeDisplay(self._data_store)
self._strain_gauge_group = CollapsibleGroupBox("Strain Gauges")
self._strain_gauge_group.addWidget(self._strain_gauge_display)
layout.addWidget(self._strain_gauge_group)
self._panels["Strain Gauges"] = self._strain_gauge_group
# Note: NOT calling setCollapsed(True) — starts expanded by default
```

### Implementation Tasks (Parallel)

| Task | File | Agent |
|------|------|-------|
| Add `strain_gauge_updated` signal and emit | `data_store.py` | Agent A |
| Create `StrainGaugeDisplay` widget | `strain_gauge_display.py` (NEW) | Agent B |
| Integrate into control panel | `control_panel.py` | Agent C |

### Dependencies

- Agent B depends on knowing the signal name (`strain_gauge_updated`)
- Agent C depends on Agent B's widget class name (`StrainGaugeDisplay`)
- All agents can work in parallel once interface is specified (above)

### Implementation Status: COMPLETE

**What was done:**

| File | Change |
|------|--------|
| `pid_tuner/data/data_store.py` | Added `strain_gauge_updated = pyqtSignal()` declaration after `leveling_updated`. Added `self.strain_gauge_updated.emit()` in `process_encoder_data()` after strain gauge values are updated. |
| NEW `pid_tuner/ui/strain_gauge_display.py` | Created `StrainGaugeBar` (custom painted widget for single bar with label, proportional fill, centered numeric value) and `StrainGaugeDisplay` (container with 4 bars for RC, FC, ML, MR). Green-to-orange color gradient based on value/max_scale ratio. Configurable `max_scale` (default 4095.0). Connects to `data_store.strain_gauge_updated` signal. |
| `pid_tuner/ui/control_panel.py` | Added import for `StrainGaugeDisplay`. Integrated as collapsible panel "Strain Gauges" after Config Viewer. Starts expanded by default (no `setCollapsed(True)`). Added to `self._panels` dict for visibility menu. |

---

## Dependencies

```
#2 (Maximized) -> #3 (Settings) [Settings defaults to maximized if no geometry saved]
#6 (Collapsible) -> #3 (Settings) [Settings should persist collapsed state once integrated]
#5 (Mode Visibility per-joint) -> May need firmware telemetry if each joint tracks its own mode
#10 (Position Offset) -> Requires Teensy firmware 'O' command handler
```

---

## Estimated vs Actual Time

| Item | Estimated | Notes |
|------|-----------|-------|
| #1 Config Viewer | 4 h | New file, table widget, signal wiring |
| #2 Maximized | 0.5 h | Folded into settings restore |
| #3 Settings | 2 h | QSettings save/restore |
| #4 Quick Jog | 2 h | Group + press/release handlers |
| #5 Mode Visibility | 3 h | Banner + encoder dot + signal |
| #6 Collapsible (widget only) | ~1 h | Full integration deferred |
| #7 CSV Export | 1 h | Button + DictWriter export |
| #8 Limits Viz | 3 h | Danger zones + dynamic color |
| #9 IMU Contrast | 2 h | ThickAxisItem + legend |
| #10 Position Offset (GUI) | 2 h | UI + protocol + handler |

*Note: Position offset firmware work is separate and not included.*

---

## 11. Self-Leveling Debug Visualization (Priority #11)

**Complexity:** Medium  
**Files:** `pid_tuner/ui/imu_3d_widget.py`, `pid_tuner/data/data_store.py`  
**Estimated Time:** 2-3 hours

### Current State
- Firmware now calculates and streams leveling target Z heights (`z_target_ml`, `z_target_rc`, `z_target_mr`) and attitude errors (`leveling_pitch_err`, `leveling_roll_err`).
- The `EncoderData` class parses this new data from the telemetry stream.
- No GUI visualization currently leverages these 5 new data points.

### Requirements
- Store the new leveling debug fields in `DataStore` or broadcast them directly.
- In `IMU3DWidget`, draw three vertical bars originating from the base plane representing the Z targets for ML, RC, and MR motors.
- Ensure the visual scales properly with the real-world geometry of the chassis so the debugging is intuitive.
- Add labels or a dedicated small data panel in the `IMU3DWidget` overlay showing the raw pitch/roll error and the current Z target values.

### Implementation Plan

#### 11.1 Update Data Store
Extract the new fields from `EncoderData` inside `DataStore.update_from_telemetry()` and emit a new signal or attach them to existing IMU data structures.

#### 11.2 Visualizing Z Heights
In `IMU3DWidget`, use `GLBarGraphItem` or custom 3D line items positioned at the relative locations of the ML, RC, and MR actuators to visualize the target heights.
- Base Chassis points relative to center (unscaled):
  - ML actuator: `x = -34`, `y = -31`
  - MR actuator: `x = -34`, `y = 31`
  - RC actuator: `x = 34`, `y = 0` (average of left and right casters)
- Map these physical dimensions into the visualization space (scaling down by a constant factor so they fit beside the orientation axes).
- Set the heights of these 3 bars/lines to `z_target_ml`, `z_target_rc`, and `z_target_mr`.

#### 11.3 Overlay Updates
Use a PyQt `QLabel` overlay on top of the `IMU3DWidget` viewport, or pyqtgraph TextItems, to print the exact `leveling_pitch_err` and `leveling_roll_err`.

---

## 12. Motor Control Trapezoidal Profile (Priority #12)

**Complexity:** Medium  
**Files:** `pid_tuner/ui/control_panel.py`, `pid_tuner/data/data_store.py`  
**Estimated Time:** 2-3 hours

### Current State
- The firmware has been updated to include a trapezoidal profile feature via maximum ramp rates on the output of both the Position and Velocity PID controllers.
- The variables `pos_max_ramp_rate` and `vel_max_ramp_rate` are defined in the `MotorConfig` struct.
- These variables are exposed via the `U<id>:<val>` (Pos Ramp) and `u<id>:<val>` (Vel Ramp) commands.
- The `ConfigData` dataclass has been updated to receive and parse these fields via the `CONFIG` command response.
- `ProtocolEncoder` and `SerialHandler` have been updated with `set_pos_ramp_rate` and `set_vel_ramp_rate` methods.

### Requirements
- Update `DataStore` to store the new `pos_max_ramp_rate` and `vel_max_ramp_rate` fields when processing `ConfigData`.
- In `control_panel.py`, add UI spinboxes for setting the max ramp rates (e.g., in a new "Trajectory / Ramp Control" group or under the respective PID groups).
- Ensure the values auto-update from the Teensy's config, just like the PID gains and Limits.
- Tooltips should explain that a value of `0.0` disables the ramp limit, while a positive value limits the rate of change of the PID output (units per second).
