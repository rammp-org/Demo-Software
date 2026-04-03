# UI Layer Reference

All UI components reside in `pid_tuner/ui/`. They are built with `PyQt6` and adhere to a unified Catppuccin Frappe color theme (`theme.py`).

## 1. `MainWindow` (`ui/main_window.py`)

**Lines: 1-592**

The primary application window.

- **Setup & Layout (Lines 90-218):** Instantiates all other widgets and arranges them in QSplitters and QTabWidgets to allow the user to resize panels.
- **Port Connection (Lines 446-483):** Handles scanning for COM/tty ports, opening/closing the connection, and spawning the `SerialHandler` thread.

## 2. `ControlPanel` (`ui/control_panel.py`)

**Lines: 1-1261+**

The largest and most complex widget, providing the interactive controls for the selected joint.

### Key Logical Sections

- **Initialization & Layout (Lines 125-252):** Sets up the scrolling layout, grouping controls into `CollapsibleGroupBox` containers.
- **Mode Banner (Lines 368-419):** Prominent indicator showing the current control mode (Open Loop, Velocity, Position).
- **PID Tuning (Lines 538-748):** Distinct sections for Position PID, Velocity PID, and LPF filters.
- **Target Controls (Lines 750-935):** Sliders, spinboxes, and buttons for absolute targets, relative steps, and homing.
- **Quick Jog (Lines 998-1062):** Hold-to-jog buttons for open-loop PWM control.
- **Sine Wave Generation (Lines 1064-1131):** Configurable amplitude, frequency, and duration for sine sweeps.
- **Self Leveling (Lines 1133-1218):** Controls for the robot's self-leveling mode.

## 3. `PlotWidget` (`ui/plot_widget.py`)

**Lines: 1-551**

Real-time graphing of the selected joint using `pyqtgraph`.

### Key Logical Sections

- **Graph Initialization (Lines 161-288):** Sets up multiple stacked plots (Position, Velocity, PWM, and optionally IMU).
- **Update Loop (Lines 300-388):** Connects to `DataStore.data_updated` and efficiently repaints using `pyqtgraph.setData()`.

## 4. `EncoderOverview` (`ui/encoder_overview.py`)

**Lines: 1-354**

Displays the current position of all 8 joints as horizontal progress bars.

- **`JointBox` Class (Lines 31-152):** Custom painted `QWidget` using `QPainter`. Displays physical limits for joints.
- **Update Loop (Lines 301-310):** Driven by a local 10Hz `QTimer` reading from `DataStore`.

## 5. `IMU3DWidget` (`ui/imu_3d_widget.py`)

**Lines: 1-407**

Visualizes the robot's orientation in 3D using `pyqtgraph.opengl`.

- Updates on `DataStore.imu_updated`.
- Converts IMU Quaternion data into a `QMatrix4x4` transformation for rendering.
- Visualizes self-leveling debug data, including Z targets for each actuator.

## 6. `SerialConsole` (`ui/serial_console.py`)

**Lines: 1-336**

A raw debugging terminal.

- **Efficient Appending (Lines 159-214):** Groups incoming lines and inserts them as a single block into `QTextEdit` to maintain high UI framerates under heavy telemetry load.
- **Regex Filtering (Lines 306-324):** Filters telemetry spam (e.g., `!^TELEMETRY`).

## 7. `SequenceEditor` (`ui/sequence_editor.py`)

**Lines: 1-1055**

A keyframe sequence builder for the `AUTO_CURB_CLIMBING` mode.

- Allows users to define a series of targets for all 8 motors.
- Supports relative targets and per-motor durations.
- Handles uploading sequences to the robot and stepping through them.

## 8. `SequencePlotter` (`ui/sequence_plotter.py`)

**Lines: 1-266**

Visualizes the execution of a sequence over time.

- Shows position and target curves for all joints in a multi-plot layout.
- Useful for verifying trajectory following during complex maneuvers.

## 9. `IMUDisplay` (`ui/imu_display.py`)

**Lines: 1-120**

Text readout of IMU data.

- Displays Pitch, Roll, and Yaw with color coding for level detection.
- Shows Accelerometer X, Y, and Z values.

## 10. `DriveWheelDisplay` (`ui/drive_wheel_display.py`)

**Lines: 1-297**

Visualizes drive wheel velocities using arc tachometers.

- Includes a D-pad for manual teleoperation via the `LuciClient` ROS bridge.
- Displays real-time feedback of wheel speeds.

## 11. `StateIndicator` (`ui/state_indicator.py`)

**Lines: 1-140**

A colored dot showing the current system state (INIT, IDLE, ESTOP, etc.).

- Blinks red when the system is in the `ESTOP` state.

## 12. `StrainGaugeDisplay` (`ui/strain_gauge_display.py`)

**Lines: 1-180**

Visualizes the current load cell / strain gauge values as progress bars to indicate the weight distribution across the chassis.

## 13. `ConfigViewer` (`ui/config_viewer.py`)

**Lines: 1-505**

Provides a read-only table view of the current EEPROM configuration loaded from the Teensy, allowing the user to verify saved parameters (PID gains, limits, ramp rates, directions).

## 14. `CollapsibleGroupBox` (`ui/collapsible_group.py`)

**Lines: 1-177**

A reusable UI component that provides a group box that can be collapsed by clicking on the header, with smooth animation.

## 15. `Scaling` (`ui/scaling.py`)

**Lines: 1-194**

DPI-aware scaling utilities to ensure a consistent appearance across different screen sizes and resolutions.

