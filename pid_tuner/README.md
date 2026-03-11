# PID Tuner for Teensy 4.1

A PyQt6-based GUI application for tuning PID controllers running on Teensy 4.1 microcontrollers. Designed for MEBot/RAMMP robots with 12 encoder-driven joints.

## Features

- **Real-time plotting** of encoder position vs target position
- **Joint selection** dropdown for all 12 joints
- **Target controls**: Set absolute targets, step inputs (+/-), quick step buttons
- **Sine wave generator**: Configurable amplitude, frequency, and duration for testing
- **Simulation mode**: Preview target signals without hardware connected
- **Serial console**: View raw serial data stream
- **Rolling time window**: Configurable 5-60 second display window

## Installation

```bash
cd pid_tuner
pip install -r requirements.txt
```

### Dependencies

- Python 3.8+
- PyQt6
- pyqtgraph
- pyserial
- numpy

## Usage

### Running the Application

```bash
python run.py
```

### Connecting to Teensy

1. Select the serial port from the dropdown (click "Refresh" to update)
2. Select baud rate (default: 115200)
3. Click "Connect"

### Using Simulation Mode

To preview target signals before connecting to hardware:

1. Click the **"Simulate"** button in the plot toolbar (turns green when active)
2. Set targets or start a sine wave - they will be plotted in real-time
3. Simulation mode auto-disables when you connect to a real device

### Controls

| Control | Description |
|---------|-------------|
| **Set Target** | Send an absolute target position (in encoder ticks) |
| **Use Current** | Copy current encoder position to target input |
| **Set Zero** | Set target to 0 |
| **Disable Motors** | Send 'z' command to disable all motors |
| **Step +/-** | Add/subtract step size from current target |
| **Quick Steps** | One-click buttons for common step sizes (+10, +50, +100, +500, +1000) |
| **Start Sine** | Begin sine wave oscillation around current target |
| **Stop Sine** | Stop sine wave and return to center position |

### Plot Controls

| Button | Description |
|--------|-------------|
| **Simulate** | Enable simulation mode for offline target preview |
| **Pause/Resume** | Freeze/unfreeze the plot display |
| **Clear** | Clear all plotted data for selected joint |
| **Auto Scale** | Reset zoom to fit all data |
| **Time Window** | Select rolling window size (5s, 10s, 20s, 30s, 60s) |

## Joint Mapping

| Joint ID | Name | Description |
|----------|------|-------------|
| 1 | RC Top | Right Calf Top |
| 2 | FC Bottom | Front Calf Bottom |
| 3 | RC Bottom | Right Calf Bottom |
| 4 | FC Top | Front Calf Top |
| 5 | MR Back | Middle Right Back |
| 6 | ML Front | Middle Left Front |
| 7 | ML Back | Middle Left Back |
| 8 | MR Front | Middle Right Front |
| 9 | ML Drive | Middle Left Drive Wheel |
| 10 | MR Drive | Middle Right Drive Wheel |
| 11 | ML Carriage | Middle Left Carriage |
| 12 | MR Carriage | Middle Right Carriage |

## Teensy Firmware Requirements

The Teensy firmware (Base.ino) must be modified to support this tuner. See [TEENSY_PROTOCOL.md](TEENSY_PROTOCOL.md) for the required serial protocol specification.

### Quick Summary

**Teensy sends** (at ~200Hz):
```
ENC:<timestamp_ms>,<enc1>,<enc2>,...,<enc12>
```

**PC sends**:
```
T<joint>:<ticks>   # Set target (e.g., T1:5000)
z                  # Disable motors
```

## File Structure

```
pid_tuner/
├── run.py                  # Entry point
├── requirements.txt        # Python dependencies
├── README.md               # This file
├── TEENSY_PROTOCOL.md      # Firmware protocol specification
├── main.py                 # Application initialization
├── data/
│   ├── data_store.py       # Rolling buffer data storage
│   └── joint_config.py     # Joint names and descriptions
├── serial/
│   ├── protocol.py         # Message parsing/encoding
│   └── serial_handler.py   # Serial thread with Qt signals
└── ui/
    ├── main_window.py      # Main application window
    ├── plot_widget.py      # Real-time pyqtgraph plot
    ├── control_panel.py    # Target/step/sine controls
    └── serial_console.py   # Raw serial data display
```

## Troubleshooting

**No serial ports found**: Ensure Teensy is connected and drivers are installed. On macOS, the port typically appears as `/dev/cu.usbmodemXXXX`.

**No data appearing**: Verify the Teensy firmware is outputting data in the expected `ENC:` format. Check the serial console to see raw data.

**Plot not updating**: Ensure simulation mode is enabled (if not connected) or that valid encoder data is being received.
