# Drinking Node

> **summary**
> `drink_action_server` runs the Cornell drink stack (vendored RAMMP `rammp` package) and exposes each drinking skill as a ROS 2 action. It drives the shared `arm_driver` over `/arm/*` (joint, Cartesian and gripper commands) and reads the wrist RealSense for cup detection and MediaPipe head perception. `system_control` calls the actions in sequence: pickup and order, grab cup from table, bring cup to mouth (waits for a mouth-open gesture), home cup, put cup back to holder.
>
> It runs in the `cornell_feeding` Docker container on the Jetson (see `docker-compose.yml`; pass `run_on_robot:=true` on the chair). Head perception needs a one-time calibration in `rammp/perception/head_perception/mediapipe_config/drink/` (`python -m rammp.perception.head_perception.calibrate_head --tool drink`).

### Publishers:

| Topic                       | Type                                                                                              |
| --------------------------- | ------------------------------------------------------------------------------------------------- |
| /arm/drink/cup_info         | cornell_feeding_interfaces/msg/CupInfo (pose, bounding box, success; 10 Hz while detection is on) |
| /arm/cornell/joint_position | sensor_msgs/msg/JointState (via arm_client)                                                       |
| /arm/cornell/cartesian_pose | geometry_msgs/msg/PoseStamped (via arm_client)                                                    |

### Subscriber:

| Topic                                          | Type                                                            |
| ---------------------------------------------- | --------------------------------------------------------------- |
| /camera/wrist/color/image_raw                  | sensor_msgs/msg/Image                                           |
| /camera/wrist/color/camera_info                | sensor_msgs/msg/CameraInfo                                      |
| /camera/wrist/aligned_depth_to_color/image_raw | sensor_msgs/msg/Image                                           |
| /arm/joint_states                              | sensor_msgs/msg/JointState                                      |
| /arm/ee/pose                                   | geometry_msgs/msg/PoseStamped                                   |
| /tf, /tf_static                                | tf2_msgs/msg/TFMessage (base_link -> wrist_color_optical_frame) |

These subscriptions exist only while an action or cup-handle streaming is running (plus the head-perception warm start at launch); the idle node receives nothing but requests.

### Service Servers:

| Topic                       | Type                                                             |
| --------------------------- | ---------------------------------------------------------------- |
| /arm/drink/detection/enable | std_srvs/srv/SetBool (start/stop cup-handle detection streaming) |

### Service Clients:

| Topic              | Type                 |
| ------------------ | -------------------- |
| /arm/open_gripper  | std_srvs/srv/Trigger |
| /arm/close_gripper | std_srvs/srv/Trigger |

### Action Servers:

| Topic                             | Type                                          |
| --------------------------------- | --------------------------------------------- |
| /arm/drink/pickup_and_order       | cornell_feeding_interfaces/action/DrinkAction |
| /arm/drink/grab_cup_from_table    | cornell_feeding_interfaces/action/DrinkAction |
| /arm/drink/bring_cup_to_mouth     | cornell_feeding_interfaces/action/DrinkAction |
| /arm/drink/home_cup               | cornell_feeding_interfaces/action/DrinkAction |
| /arm/drink/put_cup_back_to_holder | cornell_feeding_interfaces/action/DrinkAction |
| /arm/drink/locate_cup             | cornell_feeding_interfaces/action/DrinkAction |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
