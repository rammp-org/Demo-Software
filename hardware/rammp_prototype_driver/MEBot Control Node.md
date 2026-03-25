# MEBot Control Node

> **summary**
> This Node is used to communicate with Teensy (USB-Uart) to control the MeBot and also communicate with `LUCI Node`.
> When receive `/base/drive_enable` == `true`, it should talk to `LUCI` to enable joystick control.
> When receive `/base/drive_enable` == `false`, it should talk to LUCI to disable joystick control.
> This Node is also response to `/base/self_level_enable`, `/base/manual_seat_control`, `/base/curb_ascend` and `/base/curb_descend`.
> When receive `/base/estop`, it should disable joystick control.
>
> All topics are published under the `/base` namespace, set via the launch file.
> The tf tree is managed externally by a Robot State Publisher node with a URDF — this node does not publish transforms directly.

> **TODO:**
>
> - \[ \] Determine the correct interface for LUCI node
> - \[ \] Implement LUCI service client calls in `drive_enable` callback

### Parameters:

| Parameter     | Type   | Default        | Note                                                                                             |
| ------------- | ------ | -------------- | ------------------------------------------------------------------------------------------------ |
| `serial_port` | string | `/dev/ttyACM0` | Serial port for Teensy connection. Override at launch for Bluetooth: `serial_port:=/dev/rfcomm0` |

### Publishers:

| Topic                       | Type                                               | Note                                                                                                                         |
| --------------------------- | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| /base/imu                   | sensor_msgs/msg/Imu                                | Orientation as quaternion; linear acceleration from Teensy. Computed from pitch/roll until Teensy sends quaternion directly. |
| /base/joint_states          | sensor_msgs/msg/JointState                         | Encoder positions for all joints                                                                                             |
| /base/rammp_prototype_state | rammp_prototype_interfaces/msg/RAMMPPrototypeState | Full state from Teensy. IMU fields (orientation, linear_acceleration, angular_velocity) are pending Teensy protocol update.  |
| /diagnostics                | diagnostic_msgs/msg/DiagnosticArray                | Not namespaced per ROS convention. Not yet implemented.                                                                      |

### Subscribers:

| Topic                     | Type                            |
| ------------------------- | ------------------------------- |
| /base/manual_seat_control | std_msgs/msg/Bool (placeholder) |
| /base/estop               | std_msgs/msg/Bool               |

### Service Servers:

| Topic                   | Type                 |
| ----------------------- | -------------------- |
| /base/drive_enable      | std_srvs/srv/SetBool |
| /base/self_level_enable | std_srvs/srv/SetBool |

### Service Clients:

| Topic                          | Type               | Note                         |
| ------------------------------ | ------------------ | ---------------------------- |
| /luci/set_auto_remote_input    | std_srvs/srv/Empty | disable physical joystick    |
| /luci/remove_auto_remote_input | std_srvs/srv/Empty | re-enables physical joystick |

### Action Servers:

| Topic               | Type                                           |
| ------------------- | ---------------------------------------------- |
| /base/curb_traverse | rammp_prototype_interfaces/action/CurbTraverse |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
