# robotiq_description

Minimal ROS 2 description package for the Robotiq 2F-85 gripper (links, joints, and meshes only).

## Why this exists

The upstream [`ros2_robotiq_gripper`](https://github.com/PickNikRobotics/ros2_robotiq_gripper) package bundles the gripper description together with a hardware driver (`robotiq_driver`) that depends on the `serial` C++ library. That library has no rosdep entry for ROS 2 Humble, making it a pain to build.

Since we only need the URDF/xacro description for visualisation and planning — not the hardware driver — we maintain this trimmed-down package instead. It contains only the files that `kortex_description` needs when loading the gripper via `gripper:=robotiq_2f_85`.

## What's included

- `urdf/robotiq_2f_85_macro.urdf.xacro` — gripper links, joints, and ros2_control macro
- `urdf/2f_85.ros2_control.xacro` — ros2_control hardware interface definition
- `meshes/visual/2f_85/` — visual meshes (.dae)
- `meshes/collision/2f_85/` — collision meshes (.stl)

## Patches applied

The upstream macro parameter names did not match what the Humble-era `kortex_description` passes:

| Parameter added        | Reason                                              |
| ---------------------- | --------------------------------------------------- |
| `fake_sensor_commands` | Renamed to `mock_sensor_commands` in newer upstream |
| `sim_ignition`         | Renamed to `sim_isaac` in newer upstream            |

These are accepted but unused — we are not running the hardware driver.
