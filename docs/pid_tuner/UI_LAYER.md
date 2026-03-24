# UI Layer Reference

All UI components reside in `pid_tuner/ui/`. They are built with `PyQt6` and adhere to a unified Catppuccin Frappe color theme (`theme.py`).

## 1. `MainWindow` (`ui/main_window.py`)

**Lines: 1-343**

The primary application window.

- **Setup & Layout (Lines 60-140):** Instantiates all other widgets and arranges them in QSplitters to allow the user to resize panels.
- **Port Connection (Lines 226-258):** Handles scanning for COM/tty ports, opening/closing the connection, and spawning the `SerialHandler` thread.

## 2. `ControlPanel` (`ui/control_panel.py`)

**Lines: 1-1409**

The largest and most complex widget, providing the interactive controls for the selected joint.

### Key Logical Sections

- **Initialization & Layout (Lines 46-242):** Sets up the scrolling layout, grouping controls into `QGroupBox` containers.
- **Joint Selection (Lines 244-278):** Updates the UI when a new joint is selected from the dropdown or the `EncoderOverview` bar.
- **Target Controls (Lines 291-382):** Sliders, spinboxes, and buttons for absolute targets, relative steps, and homing.
- **Sine Sweep Generation (Lines 384-486):** A local `QTimer` generates a sine wave and sends positional target updates at ~50Hz.
- **Control Mode & Limits (Lines 488-662):** Switches between Open Loop, Velocity, and Position control.
- **PID Tuning SpinBoxes (Lines 664-964):** Distinct sections for Position PID, Velocity PID, and LPF filters.
- **Configuration & Saving (Lines 966-1050):** Fetches config from EEPROM and writes modifications back.
- **Motor Control Actions (Lines 1052-1185):** Handles Direction flipping, Encoder flipping, and ESTOP.

## 3. `PlotWidget` (`ui/plot_widget.py`)

**Lines: 1-549**

Real-time graphing of the selected joint using `pyqtgraph`.

### Key Logical Sections

- **Graph Initialization (Lines 67-156):** Sets up multiple stacked plots (Position, Velocity, PWM).
- **Update Loop (Lines 158-202):** Connects to `DataStore.data_updated` and efficiently repaints using `pyqtgraph.setData()`.

## 4. `EncoderOverview` (`ui/encoder_overview.py`)

**Lines: 1-317**

Displays the current position of all 6 joints as horizontal progress bars.

- **`EncoderBar` Class (Lines 22-201):** Custom painted `QWidget` using `QPainter`. Displays physical limits (`ml_fwd`, `ml_bwd`) for carriage joints.
- **Update Loop (Lines 266-279):** Driven by a local 10Hz `QTimer` reading from `DataStore`.

## 5. `IMU3DWidget` (`ui/imu_3d_widget.py`)

**Lines: 1-101**

Visualizes the robot's orientation in 3D using `pyqtgraph.opengl`.

- Updates on `DataStore.imu_updated`.
- Converts IMU Quaternion data into a `QMatrix4x4` transformation for rendering.
- Re-calculates Target orientation using Pitch/Roll constraints (Lines 84-101).

## 6. `SerialConsole` (`ui/serial_console.py`)

**Lines: 1-336**

A raw debugging terminal.

- **Efficient Appending (Lines 159-214):** Groups incoming lines and inserts them as a single block into `QTextEdit` to maintain high UI framerates under heavy telemetry load.
- **Regex Filtering (Lines 226-262):** Filters telemetry spam (e.g., `!^TELEMETRY`).

## 7. `StrainGaugeDisplay` (`ui/strain_gauge_display.py`)

Visualizes the current load cell / strain gauge values as progress bars to indicate the weight distribution across the chassis.

## 8. `ConfigViewer` (`ui/config_viewer.py`)

Provides a read-only table view of the current EEPROM configuration loaded from the Teensy, allowing the user to verify saved parameters (PID gains, limits, ramp rates, directions).
