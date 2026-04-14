# Xbox Controller Node

> **Summary**
> This node reads Xbox controller input for manual arm control. It also acts as an emergency-stop input.
>
> - Publish `/arm/xbox/twist` when receiving joystick input.
> - Home button: pressing it moves the arm to the home position.
> - Manual control request button: pressing this button requests manual control from the system. The request may be rejected if the current system state cannot transition to manual control mode. Pressing this button again requests exit from manual control.
> - Open/close gripper button.

> **Note**
> The `home button`, `manual request button`, and `open/close gripper` controls do not send requests directly to the `Arm Driver`. Instead, this node sends a service request to `/GuiBridge/user_input` to simulate user input from the UI.
> Add more `UserInputs` definitions in `interfaces/gui_interfaces/` if needed.

### Publishers:

| Topic           | Type                    |
| --------------- | ----------------------- |
| /arm/xbox/twist | geometry_msgs/msg/Twist |

### Subscriber:

| Topic | Type |
| ----- | ---- |
|       |      |

### Service Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Service Clients:

| Topic                 | Type                          |
| --------------------- | ----------------------------- |
| /GuiBridge/user_input | gui_interfaces/srv/UserInputs |

### Action Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Clients:

| Topic | Type |
| ----- | ---- |
|       |      |
