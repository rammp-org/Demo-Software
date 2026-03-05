# Navigation Node
>**summary**
This Node is perform curb detection action based on request. 
When receive `/navigate_to_curb`, it will perform `curb detection` algorithm and publish `/curb_info`.
When finished, `curb_detection` algorithm will stop to release GPU resource. 

>**TODO:**
>  - [ ] Do we use a compressed camera stream for this node?
>  - [ ] What needs to be published to the GUI?
>  - [ ] What is the `curb_info` format? How does this info display to user.

### Publishers:
| Topic          | Type |
| -------------- | ---- |
| /nav/curb/info |      |

### Subscriber:
| Topic                 | Type                  |
| --------------------- | --------------------- |
| /camera/nav/image_raw | sensor_msgs/msg/Image |
### Service Servers:
| Topic | Type |
| ----- | ---- |
|       |      |
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
