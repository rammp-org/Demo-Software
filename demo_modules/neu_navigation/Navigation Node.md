# Navigation Node

> **summary**
> This Node is perform curb detection action based on request.
> When receive `/navigate_to_curb`, it will perform `curb detection` algorithm and publish `/curb_info`.
> When finished, `curb_detection` algorithm will stop to release GPU resource.

> **TODO:**
>
> - \[ \] Do we use a compressed camera stream for this node?
> - \[ \] What needs to be published to the GUI?
> - \[ \] What is the `curb_info` format? How does this info display to user.
> - \[ \] Clean up the code.

### Publishers:

| Topic                   | Type                                   |
| ----------------------- | -------------------------------------- |
| /nav/curb/info          | neu_navigation_interfaces/msg/CurbInfo |
| /perception/curb_visual | visualization_msgs/msg/Marker          |

### Subscriber:

`# The below topics are not actual, and will change based on what we are publishing on MeBot.`

| Topic                   | Type                       |
| ----------------------- | -------------------------- |
| /camera/nav/image_raw   | sensor_msgs/msg/Image      |
| /camera/nav/depth_raw   | sensor_msgs/msg/Image      |
| /camera/nav/camera_info | sensor_msgs/msg/CameraInfo |

### Service Servers:

| Topic            | Type                                     |
| ---------------- | ---------------------------------------- |
| /nav/curb/detect | neu_navigation_interfaces/srv/DetectCurb |

### Service Clients:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Servers:

| Topic              | Type |
| ------------------ | ---- |
| /nav/curb/navigate |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
