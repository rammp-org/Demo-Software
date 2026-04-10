# Navigation Node

> **summary**
> This Node is perform curb detection and descent detection action based on request.
> When receive `/navigate_to_curb` (or respective descent topics), it will perform `curb detection` algorithm and publish `/curb_info` or `/nav/curb_descent/info`.
> When finished, the algorithm will stop to release GPU resource.
> Weights are hosted at -- https://drive.google.com/drive/folders/1AGhQ3uLKEJPdeeedGgDtzoICqgoFGrTw?usp=sharing
> Demo -- https://youtu.be/ABBS57FzwaM

> **TODO:**
>
> - \[ \] Do we use a compressed camera stream for this node?
> - \[ \] What needs to be published to the GUI?
> - \[ \] What is the `curb_info` format? How does this info display to user.
> - \[ \] Clean up the code.
> - \[ \] Test gpu release.

### Publishers:

| Topic                               | Type                                   |
| ----------------------------------- | -------------------------------------- |
| /nav/curb/info                      | neu_navigation_interfaces/msg/CurbInfo |
| /nav/curb_descent/info              | neu_navigation_interfaces/msg/CurbInfo |
| /perception/curb_visual             | visualization_msgs/msg/Marker          |
| /perception/curb_descent_visual     | visualization_msgs/msg/Marker          |
| /perception/curb_mask               | sensor_msgs/msg/Image (mono8)          |
| /perception/curb_descent_mask       | sensor_msgs/msg/Image (mono8)          |
| /perception/curb_mask_image         | sensor_msgs/msg/Image (bgr8)           |
| /perception/curb_descent_mask_image | sensor_msgs/msg/Image (bgr8)           |

### Subscriber:

> \[!NOTE\]
> The below topics are not actual, and will change based on what we are publishing on MeBot.

| Topic                                  | Type                       |
| -------------------------------------- | -------------------------- |
| /camera/nav1/color/image_rotated       | sensor_msgs/msg/Image      |
| /camera/nav1/depth/image_rotated       | sensor_msgs/msg/Image      |
| /camera/nav1/color/camera_info_rotated | sensor_msgs/msg/CameraInfo |

### Service Servers:

| Topic                    | Type                 |
| ------------------------ | -------------------- |
| /nav/curb/detect         | std_srvs/srv/SetBool |
| /nav/curb_descent/detect | std_srvs/srv/SetBool |

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

### Installation

```
cd demo_modules/neu_navigation
pip3 install -r requirements.txt
cd $ROS_WS
colcon build
source install/setup.bash
ros2 run neu_navigation perception_curb_detection_node
# Or for descent
ros2 run neu_navigation perception_curb_descent_detection_node

# on a second terminal
# Enable streaming detections
ros2 service call /nav/curb/detect std_srvs/srv/SetBool "{data: true}"
ros2 service call /nav/curb_descent/detect std_srvs/srv/SetBool "{data: true}"

# Disable and free GPU
ros2 service call /nav/curb/detect std_srvs/srv/SetBool "{data: false}"
ros2 service call /nav/curb_descent/detect std_srvs/srv/SetBool "{data: false}"
```
