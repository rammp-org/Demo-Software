# Drinking Node

> **summary**
> This Node perform `order drink` and `drinking` action upon request.
>
> - When receive `/order_drink`, it should start running `order_drink` algorithm and start publishing `/arm/cornell/twist` to control the arm. During this action, it should ignore new action request.
> - When `order_drink` action finished, the `order_drink` algorithm should stop and release GPU resources.
> - When receive `/drink`, it should start running `drinking` algorithm and start publishing `arm/cornell/twist` to control the arm. During this action, it should ignore new action request.
> - when `/drink` action finished, the `drinking` algorithm should stop and release GPU resources.

> **TODO:**
>
> - \[ \] Define initialization and shutdown behavior
> - \[ \] Ensure that node input and node output match requirements from Cornell team

### Publishers:

| Topic              | Type                    |
| ------------------ | ----------------------- |
| /arm/cornell/twist | geometry_msgs/msg/Twist |

### Subscriber:

| Topic                          | Type                  |
| ------------------------------ | --------------------- |
| /camera/wrist/color/image_raw  | sensor_msgs/msg/Image |
| /camera/wrist/depth/image_rect | sensor_msgs/msg/Image |

### Service Servers:

| Topic | Type |
| --------------------------------- | ---- |
| /arm/drink/stream_cup_handle      |      |

### Service Clients:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Servers:

| Topic                             | Type |
| --------------------------------  | ---- |
| /arm/drink/pickup_and_order       |      |
| /arm/drink/grab_cup_from_table    |      |
| /arm/drink/bring_cup_to_mouth     |      |
| /arm/drink/home_cup               |      |
| /arm/drink/put_cup_back_to_holder |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
