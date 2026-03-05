# Arm Control Node
>**Summary**
Main Node that control the Arm. This is the only Node that communicate with the Arm directly. It publish the arm's `/joint_positions` and `arm/imu` to other nodes. It also receive `arm/xxx/twist` from other nodes. Based on the internal state, it will accept one of the `arm/xxx/twist` cmd to send to the arm.
This Node is also response for monitoring the status of the Arm. if it detected an error (collision or high torque), it will enter error state.

> **TODO:** 
>  - [ ] Determine how to recover from ERROR state

> **Possible Arm State implemetation**
>1. Retracted:
>	- Default standby state. Arm retracted to a safe location.
>	- In this state, the MeBot is able to drive.
>	- In this state, the Arm is ready to perform all the actions.
>2. Open Door:
>	- Enter this state when `/set_mode` is set to `open_door`.
>	- In this state, only response to `/arm/cum/twist` and `/estop`
>3. Order Drink:
>	- Enter this state when `/set_mode` is set to `order_drink`
>	- In this state, only response to `/arm/cornell/twist` and `/estop`
>4. Drinking:
>	- Enter this state when `/set_mode` is set to `drink`
>	- In this state, only response to `/arm/cornell/twist` and `/estop`
>5. Cup Stabilize:
>	- Enter this state when `/set_mode` is set to `cup_stabilizer`
>	- In this state, only response to `/arm/atdev/twist` and `/estop`
>6. Manual:
>	- Enter this state when `/set_mode` is set to `manual`
>	- In this state, only response to `/arm/xbox/twist` and `/estop`
>7. Retracting: 
>	- Enter this state when receive `/arm/retract`
>	- In this state, only response to `/estop`
>	- when the arm retracted, enter `Retracted` state.
>8. Error:
>	- Enter this state when detected an error, e.g. collision or high torque.
>	- Enter this state after receive `/estop`

### Publishers:
| Topic              | Type                                 |
| ------------------ | ------------------------------------ |
| /arm/joint_states  | sensor_msgs/msg/JointState           |
| /arm/imu           | sensor_msgs/msg/Imu                  |
| /arm/status        | diagnostic_msgs/msg/DiagnosticStatus |
| /robot_description | std_msgs/msg/String                  |
| /tf                | tf2_msgs/msg/TFMessage               |
### Subscriber:
| Topic              | Type                    |
| ------------------ | ----------------------- |
| /arm/atdev/twist   | geometry_msgs/msg/Twist |
| /arm/xbox/twist    | geometry_msgs/msg/Twist |
| /arm/cornell/twist | geometry_msgs/msg/Twist |
| /arm/cmu/twist     | geometry_msgs/msg/Twist |
| /estop             | std_msgs/msg/Bool       |
### Service Servers:
| Topic     | Type    |
| --------- | ------- |
| /set_mode | ArmMode |
### Service Clients:
| Topic | Type |
| ----- | ---- |
|       |      |
### Action Servers:
| Topic        | Type |
| ------------ | ---- |
| /arm/retract |      |
### Action Clients:
| Topic | Type |
| ----- | ---- |
|       |       |