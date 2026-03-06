# Demo-Software

Software framework for the RAMMP April 2026 Demo.

## Technical Specifications

* **Middleware**: ROS 2 Humble Hawksbill.
* **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish).
* **Build System**: `colcon` with `ament_cmake` and `ament_python`.

## Repository Structure
The demo codebase is organized into multiple ROS 2 packages, categorized by functional responsibility to ensure modularity and ease of integration.

### 1. Core Services (`core/`)
Foundational packages for the RAMMP prototype.

* **`rammp_prototype_behavior`**: Orchestration of mission-level tasks using Behavior Tree (BT) architectures.
* **`rammp_prototype_bringup`**: Centralized entry point containing top-level launch files and global system parameters.
* **`rammp_prototype_description`**: URDF/Xacro models, kinematic configurations, and meshes defining the `tf2` coordinate frames.
* **`rammp_prototype_gui`**: Interface layer for communication with the Unreal Engine-based GUI.

### 2. Demo Modules (`demo_modules/`)
Task-specific research modules provided by unveristy partners. These are self-contained "skills" (e.g., stabilization, navigation, manipulation) designed to interface with the core platform via standardized ROS 2 topics and actions.

### 3. Hardware Drivers (`hardware/`)
Interfaces between the ROS 2 computational graph and physical components.

* **`arm_driver`**: Low-level interface for the Kinova manipulator.
* **`rammp_prototype_driver`**: Driver for the mobile wheel base and seating system.
* **`xbox_controller_driver`**: Input handling for manual teleoperation and override logic.

### 4. Interfaces (`interfaces/`)

Centralized definitions for all custom ROS 2 messages (`.msg`), services (`.srv`), and actions (`.action`). All packages that requires custom interface types should define them in a separate package in this directory. These interface packages should never depend on the packages with executable logic.

* These packages contain no executable logic.
* All custom interface requirements must be defined here to ensure consistent data types across the distributed system.

---

## Getting Started

### Prerequisites
Ensure you have a functional ROS 2 Humble installation on Ubuntu 22.04.

### Installation
Clone this repository into the `src` directory of your ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/rammp-org/Demo-Software.git](https://github.com/rammp-org/Demo-Software.git)

```

### Building the Workspace

Install dependencies using `rosdep` and build the packages. Using `--symlink-install` is recommended for development:

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash

```

## Contributing

The tasks for the upcoming demo are organized in a couple of key locations

- [ros_specification](ros_specification.md) : This document outlines the communication specifications for all of the ROS nodes in the demo. All nodes should be developed to match these specifications. Each package also contains a README with the detailed specifications for the nodes required for those packages.
- [Github Issues](https://github.com/rammp-org/Demo-Software/issues): The individual tasks that need to be completed are all outlined here. Before starting on any work, please ensure that there is an issue open for it and that you are assigned to it.

Please refer to **[CONTRIBUTING.MD](CONTRIBUTING.MD)** for detailed guidelines regarding:

* Git branching strategy and naming conventions.
* Code style requirements (PEP 8 for Python, ROS 2 C++ Style Guide).
* Pull Request (PR) and code review workflows for the April 2026 milestone.

## Useful Tools

* **[GitHub CLI](https://cli.github.com/) (`gh`)**: Recommended for managing branches and PRs directly from the terminal.
