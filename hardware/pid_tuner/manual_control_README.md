# Manual Control

ROS 2 Node that subscribes to `/joy` and forwards gamepad input to the Teensy over the PID tuner serial link. Joint motors receive PWM commands; ODrives receive velocity setpoints. Drive wheels are commanded through LUCI.

The `manual_control_node` runs inside the PID tuner process (it shares `SerialHandler` with the GUI). Do not run it as a standalone node.

## Setup

Run these in separate terminals.

1. **PID tuner** — connect to the Teensy in the GUI, then **Connect LUCI** on the Drive Wheels panel (top). One LUCI connection is shared by the gamepad, sequence editor carriage return, and the d-pad.

1. **Joy node**

   ```bash
   ros2 run joy joy_node
   ```

## Enabling manual control

Press **Start** once to enter manual-control mode; press **Start** again to exit.

## Gamepad layout

| Input                       | Function                                                                 |
| --------------------------- | ------------------------------------------------------------------------ |
| **Button 1** (`buttons[1]`) | **ESTOP** — stops all motors (works anytime, including during sequences) |
| **Right stick (Y)**         | ODrive velocity — both ODrives move together; forward/back only          |
| **Left stick**              | Drive wheels (forward, back, strafe left/right via LUCI)                 |
| **X**                       | RC                                                                       |
| **Y**                       | FC                                                                       |
| **B**                       | Calibration                                                              |
| **LB** / **RB**             | Left carriage / right carriage                                           |
| **LT** / **RT**             | Middle left / middle right                                               |
| **D-pad up / down**         | Direction for button-controlled motors (see below)                       |

### Button-controlled motors

RC, FC, carriages, and middle joints are **not** driven by the sticks. To move them:

1. Hold the motor button(s) you want (e.g. **LB** and **RB** for both carriages).
1. While holding, press **D-pad up** or **D-pad down** for direction.

Both the motor button(s) and the D-pad must be held at the same time. You can hold several motor buttons together to move multiple joints in the same direction.

Drive wheels and ODrives are controlled only by the joysticks (when not running a sequence).

### Drive wheels during sequences (sequence editor)

The wheelchair is **not** driven by the **Drive_FB** / **Drive_LR** target columns. Use the **LUCI** row on each keyframe (`0` = off, any other integer = `forward_back`).

With **Connect LUCI** enabled, carriage return is forwarded from Teensy telemetry while the robot is in `AUTO_CURB` (sequence playing). The Sequences tab shows **LUCI: on** when connected.

Press **Start** for manual mode is **not** required for sequence carriage return.

### LUCI priority (single publisher)

1. Drive Wheels **d-pad** (while held)
1. Sequence **LUCI** row / `carriage_return_direction` during `AUTO_CURB`
1. Gamepad **left stick** in manual-control mode
1. Idle (no drive command)

## Calibration

Press **B** to start calibration. While calibration is running, button-controlled motors are ignored (the firmware is in a calibrating state). Joysticks may still send commands but should be left alone.

Wait for calibration to finish (~6 seconds, or until the firmware reports `CAL_DONE`) before using button controls again.
