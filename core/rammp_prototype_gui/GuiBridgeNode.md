# GUI Monitoring Node

> **Summary**
> Interface node between GUI and ROS. Possible use WebSocket to exchange data.
> Get `user_input` from GUI and publish it to other nodes.
> Get `curb_info` and `system_state` and send it to GUI.

> **TODO:**
>
> - [ ] Define `system_state` and `curb_info`.
> - [ ] Determine all inputs to the GUI node, including all necessary camera streams

### UserInputs:

An enum for user inputs from the UI. This Node subscribe the UI exposed porperty `User_Inputs`. When user interactive with UI, this porperty will change accordingly. This node is notified as a porperty value change event. After getting the new value, this Node will publish the user input to other subscriber Node.

```
# chair control / entry points
string CHAIR_CONTROL_MAIN="chair/main"

# chair features
string CHAIR_SELFLEVELING_ON="chair/selfLeveling/on"
string CHAIR_SELFLEVELING_OFF="chair/selfLeveling/off"
string CHAIR_SEAT_ELEVATE_UP="chair/seat/elevate/up"
string CHAIR_SEAT_ELEVATE_DOWN="chair/seat/elevate/down"
string CHAIR_SEAT_ELEVATE_HOME="chair/seat/elevate/home"
string CHAIR_SEAT_RECLINE_FORWARD="chair/seat/recline/forward"
string CHAIR_SEAT_ELEVATE_RECLINE_BACK="chair/seat/recline/back"
string CHAIR_SEAT_ELEVATE_RECLINE_HOME="chair/seat/recline/home"
string CHAIR_SEAT_ELEVATE_LTILT_LEFT="chair/seat/lateralTilt/left"
string CHAIR_SEAT_ELEVATE_LTILT_RIGHT="chair/seat/lateralTilt/right"
string CHAIR_SEAT_ELEVATE_LTILT_HOME="chair/seat/lateralTilt/home"
string CHAIR_SEAT_HOME="chair/seat/home"


# curb navigation
string CHAIR_CURB_NAVIGATION="chair/curb/navigation"
string CHAIR_CURB_ASCEND="chair/curb/ascend"
string CHAIR_CURB_DESCEND="chair/curb/descend"
string CHAIR_CURB_CANCEL="chair/curb/cancel"

# arm control / entry points
string ARM_CONTROL_MAIN="arm/main"

# arm modes / tasks
string ARM_RETRACT="arm/retract"
string ARM_HOME="arm/home"
string ARM_MANUAL_ON="arm/manual/on"
string ARM_MANUAL_OFF="arm/manual/off"
# arm/open door
string ARM_OPEN_DOOR="arm/openDoor/start"
string ARM_OPEN_DOOR_CONFIRM="arm/openDoor/confirm"
# arm/order drink
string ARM_ORDER_DRINK="arm/orderDrink/start"
string ARM_ORDER_DRINK_RELEASE_CUP="arm/orderDrink/releaseCup"
string ARM_ORDER_DRINK_RECEIVE="arm/orderDrink/receive/start"
string ARM_ORDER_DRINK_RECEIVE_CONFIRM="arm/orderDrink/receive/comfirm"
# arm/cup stablilize
string ARM_CUP_STABLE_ON="arm/cupStable/on"
string ARM_CUP_STABLE_OFF="arm/cupStable/off"
# arm/sipping drink
string ARM_DRINKING_START="arm/drinking/start"
string ARM_DRINKING_FINISH="arm/drinking/finish"
# arm/plack cup back to holder
string ARM_CUP_BACK="arm/cup/back"
# general arm cancel
string ARM_CANCEL="arm/cancel"

# System
string RESET="system/reset"
string ESTOP="system/stop"

```

### Publishers:

| Topic                    | Type              |
| ------------------------ | ----------------- |
| GuiBridge/gui_connection | std_msgs/msg/Bool |

### Subscriber:

| Topic                                   | Type                                      |
| --------------------------------------- | ----------------------------------------- |
| /system/state                           | std_msgs/msg/String                       |
| /arm/joint_states                       | sensor_msgs/msg/JointState                |
| /base/joint_states                      | sensor_msgs/msg/JointState                |
| /camera/wrist/color/image_raw           |                                           |
| /camera/wrist/depth/image_rect_raw      |                                           |
| /camera/wrist/extrinsics/depth_to_color |                                           |
| /camer/nav/color/image_raw              |                                           |
| /camera/nav/depth/image_rect_raw        |                                           |
| /camera/nav/extrinsics/depth_to_color   |                                           |
| /arm/door/button_info                   | cmu_door_opener_interfaces/msg/ButtonInfo |
| /nav/curb/info                          | neu_navigation_interfaces/msg/CurbInfo    |
| /arm/drink/cup_info                     | Not Defined yet                           |

### Service Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Service Clients:

| Topic       | Type                         |
| ----------- | ---------------------------- |
| /user_input | gui_interface/srv/UserInputs |

### Action Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
