# Demo-Software

Software framework for the RAMMP April 2026 Demo.

## Technical Specifications

- **Middleware**: ROS 2 Humble Hawksbill.
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish).
- **Build System**: `colcon` with `ament_cmake` and `ament_python`.

## Repository Structure

The demo codebase is organized into multiple ROS 2 packages, categorized by functional responsibility to ensure modularity and ease of integration.

### 1. Core Services (`core/`)

Foundational packages for the RAMMP prototype.

- **`rammp_prototype_behavior`**: Orchestration of mission-level tasks using Behavior Tree (BT) architectures.
- **`rammp_prototype_bringup`**: Centralized entry point containing top-level launch files and global system parameters.
- **`rammp_prototype_description`**: URDF/Xacro models, kinematic configurations, and meshes defining the `tf2` coordinate frames.
- **`rammp_prototype_gui`**: Interface layer for communication with the Unreal Engine-based GUI.

### 2. Demo Modules (`demo_modules/`)

Task-specific research modules provided by university partners. These are self-contained "skills" (e.g., stabilization, navigation, manipulation) designed to interface with the core platform via standardized ROS 2 topics and actions.

### 3. Hardware Drivers (`hardware/`)

Interfaces between the ROS 2 computational graph and physical components.

- **`arm_driver`**: Low-level interface for the Kinova manipulator.
- **`rammp_prototype_driver`**: Driver for the mobile wheel base and seating system.
- **`xbox_controller_driver`**: Input handling for manual teleoperation and override logic.

### 4. Interfaces (`interfaces/`)

Centralized definitions for all custom ROS 2 messages (`.msg`), services (`.srv`), and actions (`.action`). All packages that requires custom interface types should define them in a separate package in this directory. These interface packages should never depend on the packages with executable logic.

- These packages contain no executable logic.
- All custom interface requirements must be defined here to ensure consistent data types across the distributed system.

______________________________________________________________________

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

### System Packages (Jetson)

On Jetson, the system-installed scipy and matplotlib conflict with the pip versions required by `arm_driver` due to binary incompatibility with newer numpy. Remove them before installing dependencies:

```bash
sudo apt remove python3-scipy python3-matplotlib
```

### Installing Dependencies

If this is your first time using `rosdep`, initialize it before running the script:

```bash
sudo rosdep init
```

After `rosdep` is setup, use the provided setup script, which handles both rosdep and pip dependencies:

```bash
cd ~/ros2_ws/src/Demo-Software
./setup.sh
```

### Building the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running the system

Bring-up is managed by [sheppy](https://rammp-org.github.io/sheppy): `sheppy-manifest.yaml`
at the repo root lists every node and its real and mock alternatives, and
`profiles/` says which to run. Install sheppy on the Jetson once:

```bash
curl -LsSf https://rammp-org.github.io/sheppy/install.sh | sh
```

Then, from a plain shell (sheppy's daemon inherits the environment of the shell
that starts it, including `ROS_DOMAIN_ID`, which must stay `0` to match the
containers):

```bash
cd ~/ros2_ws/src/Demo-Software
docker compose build cornell_feeding   # once, and after cornell_feeding changes
sheppy up full          # everything real, with GUI and calibration
sheppy status           # per-node state, CPU, memory, last output
sheppy logs <node> -n 100
sheppy woof <node>      # restart one node (crashes are never auto-restarted)
sheppy down             # stop every node and the daemon
```

`sheppy up mock` runs every mock with no hardware, on any machine with the
workspace built. To skip a node (the old `--no-arm`, `--no-cameras`,
`--no-luci`) copy `profiles/full.yaml`, delete that node's line, and
`sheppy up <your-profile>`. To change the serial port, chair IP or UE host, add
an `overrides:` block to the copy; `profiles/full.yaml` shows the shape.

Calibration is the `calibration` node: it waits for the base and arm calibrate
action servers, sends both goals, then idles. `sheppy woof calibration`
re-runs it; a failed goal shows the node as crashed. Per-node logs are under
`~/.sheppy/logs/<node>/`.

`scripts/validate_manifest.py` loads the manifest and every profile through
sheppy's own loader; run it after editing either.

## Contributing

The tasks for the upcoming demo are organized in a couple of key locations

- [ros_specification](ROS_Specification.md) : This document outlines the communication specifications for all of the ROS nodes in the demo. All nodes should be developed to match these specifications. Each package also contains a README with the detailed specifications for the nodes required for those packages. **Please ensure that any changes you make to the specification are reflected in these docs.**
- [Github Issues](https://github.com/rammp-org/Demo-Software/issues): The individual tasks that need to be completed are all outlined here. Before starting on any work, please ensure that there is an issue open for it and that you are assigned to it.

Please refer to **[CONTRIBUTING.MD](CONTRIBUTING.MD)** for detailed guidelines regarding:

- Git branching strategy and naming conventions.
- Code style requirements (PEP 8 for Python, ROS 2 C++ Style Guide).
- Pull Request (PR) and code review workflows for the April 2026 milestone.

## Useful Tools

- **[GitHub CLI](https://cli.github.com/) (`gh`)**: Recommended for managing branches and PRs directly from the terminal.
- **[pre-commit](https://pre-commit.com/)**: Enforces code style automatically before each commit. See [CONTRIBUTING.MD](CONTRIBUTING.MD) for setup instructions.
