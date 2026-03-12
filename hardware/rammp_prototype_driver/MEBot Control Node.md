# MEBot Control Node

> **summary**
> This Node is used to communicate with Teensy (USB-Uart) to control the MeBot and also communicate with `LUCI Node`.\
> When receive `/drive_enable` == `true`, it should talk to `LUCI` to enable joystick control.
> When receive `/drive_enable` == `false`, it should talk to LUCI to disable joystick control.
> This Node is also response to `/self_level_enable`, `/manual_seat_control`, `/curb_ascend` and `/curb_descend`.
> When receive `/estop`, it should disable joystick control.

> **TODO:**
>
> - \[ \] Determine the correct interface for LUCI node
> - \[ \] Add publishers for encoders, velocities, accelerations, etc.

### Publishers:

| Topic      | Type                   |
| ---------- | ---------------------- |
| /tf        | tf2_msgs/msg/TFMessage |
| /mebot/imu | sensor_msgs/msg/Imu    |

### Subscriber:

| Topic                      | Type              |
| -------------------------- | ----------------- |
| /mebot/seat/manual_control |                   |
| /estop                     | std_msgs/msg/Bool |

### Service Servers:

| Topic                         | Type                 |
| ----------------------------- | -------------------- |
| /mebot/drive/enable           | std_srvs/srv/SetBool |
| /mebot/seat/self_level/enable | std_srvs/srv/SetBool |

### Service Clients:

| Topic                          | Type               | Note                         |
| ------------------------------ | ------------------ | ---------------------------- |
| /luci/set_auto_remote_input    | std_srvs/srv/Empty | disable physical joystick    |
| /luci/remove_auto_remote_input | std_srvs/srv/Empty | re-enables physical joystick |

### Action Servers:

| Topic                | Type |
| -------------------- | ---- |
| /mebot/curb/traverse |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
