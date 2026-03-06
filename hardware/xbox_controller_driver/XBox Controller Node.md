# XBox Controller Node

> **summary**
> This Node is used to get xbox controller input and control the arm manually.  It is also act like an Emergency stop input.
>
> - At any time, when Emergency Stop button pressed, publish `/estop`
> - Publish `/arm/xbox/twist` when get input from xbox controller.

> **TODO:**
>
> - \[x\] Does the controller node handle converting raw control inputs into actions? I think, yes

### Publishers:

| Topic           | Type                    |
| --------------- | ----------------------- |
| /arm/xbox/twist | geometry_msgs/msg/Twist |
| /estop          | std_msgs/msg/Bool       |

### Subscriber:

| Topic | Type |
| ----- | ---- |
|       |      |

### Service Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Service Clients:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
