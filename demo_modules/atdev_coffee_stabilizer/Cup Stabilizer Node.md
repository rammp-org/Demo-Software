# Cup Stabilizer Node

> **summary**
> This Node perform `cup_stabilize` task upon request.
> When receive `/stabilize_enable` == `true`, it should start running `cup stabilization` algorithm and start publishing `/arm/atdev/twist` to control the arm.
> When receive `/stabilize_enable` == `false`, the `cup stabilization` algorithm should stop.

> **TODO:**
>
> - \[ \] Define initialization and shutdown behavior

### Publishers:

| Topic            | Type                    |
| ---------------- | ----------------------- |
| /arm/atdev/twist | geometry_msgs/msg/Twist |

### Subscriber:

| Topic                      | Type                       |
| -------------------------- | -------------------------- |
| /camera/wrist/accel/sample | sensor_msgs/msg/Imu        |
| /camera/wrist/gyro/sample  | sensor_msgs/msg/Imu        |
| /arm/joint_states          | sensor_msgs/msg/JointState |

### Service Servers:

| Topic                       | Type                 |
| --------------------------- | -------------------- |
| /arm/drink/stabilize/enable | std_srvs/srv/SetBool |

### Service Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
|       |      |

### Action Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
