# Behavior Tree Node (State Node)

> **Summary**
> This Node is the main control Node for this system.
> Based on the `user input` from Gui and the `system state`, it will send actions to other node.

> **Possible `System states` implementation**
>
> 1. MeBot Control
>    GUI should show MeBot control Options. User could use joystick to control the chair and also have the option to enable/disable `self_level`. User could also control the seat manually. From this state, user could enter `arm control` or `navigation`.
>    Possible action/service from this state:
>
> - `/drive_enable` = `true`
> - `/self_level_enable` = `true/false`
> - `/manual_seat_control`
> - `/navigate_to_curb`
>
> 2. Arm Control
>    Gui should show arm control options.
>    Should disable drive so the chair is not moving when control the arm.
>    User have all the arm control options. Base on the user input, send correspond action and also `/set_mode` to notice the `arm control node`.
>    When one action is running, should ignore new action request.
>    When one action is finished, if the arm is not retracted, send action `/arm/retract` to retract the arm to a safe position.
> 1. Error
>    Gui should show error to user.
>    Enter this state when receive `/estop` or `/status/arm` == `error`
>    Send `/drive_enable` = `false` to disable drive.
> 1. Navigation
>    Gui should show Mebot control interface with `curb_info`. User could use joystick to drive closer to the detected curb. User have the option to `/curb_ascend` and `/curb_descend`.

### Publishers:

| Topic                       | Type                 |
| --------------------------- | -------------------- |
| /state                      |                      |
| /nav/curb_traverse_progress | std_msgs/msg/Float32 |

### Subscriber:

| Topic             | Type                                 | Note                                                          |
| ----------------- | ------------------------------------ | ------------------------------------------------------------- |
| /user_input       | gui_interface/msg/UserInputs         |                                                               |
| /arm/status       | diagnostic_msgs/msg/DiagnosticStatus |                                                               |
| /estop            | std_msgs/msg/Bool                    |                                                               |
| /arm/door/visible | std_msgs/msg/Bool                    |                                                               |
| /user_connection  | std_msgs/msg/Bool                    | Go to Error state when User connection lost (UI closed/crash) |
|                   |                                      |                                                               |

### Service Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Service Clients:

| Topic                         | Type                 |
| ----------------------------- | -------------------- |
| /set_mode                     | ArmMode              |
| /mebot/drive/enable           | std_srvs/srv/SetBool |
| /mebot/seat/self_level/enable | std_srvs/srv/SetBool |
| /arm/drink/stabilize/enable   | std_srvs/srv/SetBool |

### Action Servers:

| Topic | Type |
| ----- | ---- |
|       |      |

### Action Clients:

| Topic                | Type         |
| -------------------- | ------------ |
| /nav/curb/navigate   |              |
| /arm/door/open       |              |
| /arm/drink/drink     |              |
| /arm/drink/order     |              |
| /arm/retract         |              |
| /mebot/curb/traverse | CurbTraverse |
