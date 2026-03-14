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
    enum UserInputs
    {
        "ChairControl/main", // enter chair control interface
        "Chair/SelfLeveling/On", // self leveling enable
        "Chair/SelfLeveling/Off", // self leveling enable
        "Chair/Curb/Navigation"// enter navigation interface
        "Chair/Curb/Ascend"
        "Chair/Curb/Descend"
        "Chair/Curb/Cancel"
        "ArmControl/main" // enter arm control interface
        "Arm/Retract"
        "Arm/Manual"
        "Arm/OpenDoor"
        "Arm/OrderDrink"
        "Arm/Drinking"
        "Arm/CupStable"
        "Arm/Cancel"
        "Reset" // reset from Error mode
    }
```

### Publishers:

| Topic                      | Type              |
| -------------------------- | ----------------- |
| /user_input                |                   |
| /mebot/seat/manual_control |                   |
| /user_connection           | std_msgs/msg/Bool |

### Subscriber:

| Topic          | Type            |
| -------------- | --------------- |
| /state         | SystemState<br> |
| /nav/curb/info |                 |

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
