# Door Open Node
>**summary**
>This Node perform `open door` action upon request. 
>When receive `/open_door`, it should start running `open_door` algorithm and start publishing `/arm/cmu/twist` to control the arm. 
>When finished, the `open_door` algorithm should stop and release GPU resources. 

>**TODO:** 
>  - [ ] Confirm that the input/output for this node is correct and sufficient.
### Publishers:
| Topic             | Type                    |
| ----------------- | ----------------------- |
| /arm/cmu/twist    | geometry_msgs/msg/Twist |
| /arm/door/visible | std_msgs/msg/Bool       |
### Subscriber:
| Topic                          | Type                       |
| ------------------------------ | -------------------------- |
| /arm/joint_states              | sensor_msgs/msg/JointState |
| /camera/base/image_raw         | sensor_msgs/msg/Image      |
| /camera/wrist/color/image_raw  | sensor_msgs/msg/Image      |
| /camera/wrist/depth/image_rect | sensor_msgs/msg/Image      |
### Service Servers:
| Topic | Type |
| ----- | ---- |
|       |      |
### Service Clients:
| Topic | Type |
| ----- | ---- |
|       |      |
### Action Servers:
| Topic          | Type |
| -------------- | ---- |
| /arm/door/open |      |
### Action Clients:
| Topic | Type |
| ----- | ---- |
|       |      |
