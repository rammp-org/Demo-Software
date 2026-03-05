
# GUI Monitoring Node

>**Summary** 
>Interface node between GUI and ROS. Possible use WebSocket to exchange data. 
Get `user_input` from GUI and publish it to other nodes. 
Get `curb_info` and `system_state` and send it to GUI. 

> **TODO:** 
> - [ ] Define `system_state` and `curb_info`. 
> - [ ] Determine all inputs to the GUI node, including all necessary camera streams
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
 