# Sheppy Launch Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `core/rammp_prototype_bringup/launch/launch.sh` with a sheppy manifest, two profiles, and the small scripts they need, so the operator runs `sheppy up full` on the Jetson.

**Architecture:** A repo-root `sheppy-manifest.yaml` declares one sheppy node per subsystem with a real and (where one exists) a mock alternative, all sourcing the Jetson workspace through one `machines:` entry. `profiles/full.yaml` and `profiles/mock.yaml` pick alternatives. The calibration loop and the two-node NEU curb launch become a script and a launch file that the manifest points at. A validator loads all of it through sheppy's own loader.

**Tech Stack:** sheppy 1.0.0 (`~/atdev/sheppy`, Python, pyyaml), ROS 2 Humble launch files, docker compose, bash.

**Spec:** `docs/superpowers/specs/2026-09-23-sheppy-launch-design.md`

## Global Constraints

- Branch `feature/256-sheppy-launch`, based on `234-cornell-feeding`.
- Sheppy runs commands with `bash -c`; nothing may depend on zsh.
- Every `executable`, `launch_file`, and `process` alternative that needs ROS sets `machine: jetson`; the machine's `ros_setup` is `~/ros2_ws/install/setup.bash`.
- Manifest defaults match launch.sh defaults: serial `/dev/ttyACM0`, chair `10.2.10.3`, UE host `127.0.0.1`.
- No changes to `full.launch.py` or any existing launch file.
- No ROS on this workstation: validation runs through sheppy's loader only; running nodes is a Jetson step.
- Simplicity first: no configurability beyond what the spec names.

## Review Focus

1. A profile override for a param the manifest does not declare is silently dropped by sheppy. The validator must fail on that warning (Task 6 test).
1. A compose service name or file that does not exist only fails at launch in sheppy. The validator resolves compose references itself (Task 1, checked in Task 5).
1. A failed calibrate goal must be visible, not swallowed. `calibrate.sh` uses `set -e`, so a failed goal makes the node `crashed` instead of `running` (Task 4).
1. `sheppy up` from a shell with a different `ROS_DOMAIN_ID` than the compose file's `0` splits the graph. README says to run from a plain shell (Task 7).
1. A profile that selects a node name that no longer exists in the manifest is dropped with a warning. The validator fails on any reconcile warning (Task 6).

______________________________________________________________________

### Task 1: Manifest and profile validator

**Files:**

- Create: `scripts/validate_manifest.py`

**Interfaces:**

- Produces: `python scripts/validate_manifest.py` exits 0 when `sheppy-manifest.yaml` loads and every `profiles/*.yaml` reconciles without warnings; exits 1 and prints one line per problem otherwise.

- [ ] **Step 1: Confirm the sheppy checkout imports under uv**

Run from the repo root:

```bash
uv run --with /home/swapnil/atdev/sheppy python -c "import sheppy; print(sheppy.__version__)"
```

Expected: `1.0.0`. If `--with` refuses a directory, use `uv run --project /home/swapnil/atdev/sheppy python -c ...` instead and use that form in every later step.

- [ ] **Step 2: Write the validator**

```python
#!/usr/bin/env python3
"""Validate sheppy-manifest.yaml and every profile in profiles/ through
sheppy's own loader, so what loads green here loads on the Jetson.

Run from the repo root:
    uv run --with /path/to/sheppy python scripts/validate_manifest.py

Sheppy has no `validate` verb (rammp-org/sheppy#14), so this reaches into
its loader the way rammp-deployments does. Compose references are resolved
here as well, because sheppy only reads them at launch. The `build` and
`container_name` warnings are tolerated: docker-compose.yml is also used by
plain `docker compose`, and sheppy merely ignores those keys.
"""

import os
import sys

from sheppy.launch.docker.compose import load_service, service_to_docker_args
from sheppy.manifest.loader import load_manifest
from sheppy.profiles.reconcile import reconcile
from sheppy.profiles.store import ProfileStore

TOLERATED = ("compose 'build' is ignored", "compose 'container_name' is ignored")


def check_manifest(path):
    if not os.path.exists(path):
        return None, [f"{path}: not found"]
    result = load_manifest(path)
    problems = [f"{err.location}: {err.message}" for err in result.errors]
    if not result.ok:
        return None, problems
    manifest_dir = os.path.dirname(os.path.abspath(path))
    for node in result.manifest.nodes:
        for alt in node.alternatives:
            if alt.kind != "docker":
                continue
            where = f"{node.name}/{alt.id}"
            if "container" in alt.raw:
                service, base_dir = alt.raw["container"], manifest_dir
            else:
                ref = alt.raw["compose"]
                file = os.path.join(manifest_dir, ref["file"])
                try:
                    service, _ = load_service(file, ref["service"], os.environ)
                except (OSError, KeyError) as e:
                    problems.append(f"{where}: compose service unreadable: {e}")
                    continue
                base_dir = os.path.dirname(file)
            *_, errs, warns = service_to_docker_args(service, base_dir)
            problems += [f"{where}: {e}" for e in errs]
            problems += [f"{where}: warning: {w}" for w in warns
                         if not w.startswith(TOLERATED)]
    return result.manifest, problems


def check_profiles(manifest, profiles_dir):
    store = ProfileStore(profiles_dir)
    names = store.list_profiles()
    problems = [] if names else [f"{profiles_dir}: no profiles found"]
    for name in names:
        loaded = store.load(name)
        if loaded.profile is None:
            problems += [f"profiles/{name}: {e}" for e in loaded.errors]
            continue
        problems += [f"profiles/{name}: {w}"
                     for w in reconcile(loaded.profile, manifest).warnings]
    return names, problems


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    manifest, problems = check_manifest(os.path.join(root, "sheppy-manifest.yaml"))
    names = []
    if manifest is not None:
        names, more = check_profiles(manifest, os.path.join(root, "profiles"))
        problems += more
    for p in problems:
        print(p)
    if problems:
        return 1
    alts = sum(len(n.alternatives) for n in manifest.nodes)
    print(f"ok: {len(manifest.nodes)} nodes, {alts} alternatives, "
          f"{len(names)} profiles ({', '.join(names)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Run it against the repo with no manifest yet**

Run: `uv run --with /home/swapnil/atdev/sheppy python scripts/validate_manifest.py; echo exit=$?`
Expected: one line ending in `sheppy-manifest.yaml: not found`, then `exit=1`.

- [ ] **Step 4: Run it against a deliberately bad manifest**

```bash
printf 'nodes:\n  - name: x\n    alternatives:\n      - id: a\n        kind: docker\n        compose: {file: nope.yml, service: nope}\n' > sheppy-manifest.yaml
mkdir -p profiles && printf 'selections: {x: a}\n' > profiles/t.yaml
uv run --with /home/swapnil/atdev/sheppy python scripts/validate_manifest.py; echo exit=$?
rm sheppy-manifest.yaml profiles/t.yaml && rmdir profiles
```

Expected: a line `x/a: compose service unreadable: ...`, then `exit=1`.

- [ ] **Step 5: Commit**

```bash
git add scripts/validate_manifest.py
git commit -m "scripts: validate the sheppy manifest and profiles through sheppy's loader (#256)"
```

______________________________________________________________________

### Task 2: Robot-mode cornell_feeding compose service

**Files:**

- Modify: `docker-compose.yml` (the `services:` block, after `cornell_feeding`)

**Interfaces:**

- Produces: compose service `cornell_feeding_robot`, referenced by the manifest's `drink` node in Task 5.

- [ ] **Step 1: Add the service**

Insert after the `cornell_feeding` service's last volume line and before the `# ----` comment block:

```yaml
  # Same container in robot mode, for sheppy's `drink` node (see
  # sheppy-manifest.yaml). Needs the host to publish /camera/wrist/* and
  # /arm/*. Build the image once with `docker compose build cornell_feeding`;
  # sheppy runs images, it does not build them.
  cornell_feeding_robot:
    <<: *ros-common
    image: cornell-feeding:latest
    command: ros2 launch cornell_feeding cornell_feeding.launch.py run_on_robot:=true
    volumes:
      - ./core/rammp_prototype_bringup/config/cyclonedds.xml:/config/cyclonedds.xml:ro
      - ./demo_modules/cornell_feeding/rammp/perception/head_perception/mediapipe_config:/ros2_ws/src/cornell_feeding/rammp/perception/head_perception/mediapipe_config
```

- [ ] **Step 2: Check compose still parses and both services are present**

Run: `docker compose config --services`
Expected: two lines, `cornell_feeding` and `cornell_feeding_robot`.

Run: `docker compose config | grep -A3 'cornell_feeding_robot:' | head -5`
Expected: shows `command:` with `run_on_robot:=true` and no `build:` key under the robot service.

- [ ] **Step 3: Commit**

```bash
git add docker-compose.yml
git commit -m "compose: cornell_feeding_robot service for the sheppy drink node (#256)"
```

______________________________________________________________________

### Task 3: NEU curb detection launch wrapper

**Files:**

- Create: `core/rammp_prototype_bringup/launch/neu_navigation.launch.py`

**Interfaces:**

- Produces: `ros2 launch rammp_prototype_bringup neu_navigation.launch.py` starts both NEU nodes; referenced by the manifest's `curb_detection` node in Task 5. The bringup `CMakeLists.txt` already installs `launch/`, so nothing else changes.

- [ ] **Step 1: Write the launch file**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # NEU curb detection is two script nodes with no launch file of their
    # own. Lifted verbatim from full.launch.py so sheppy can start the pair
    # as one node.
    return LaunchDescription(
        [
            Node(
                package="neu_navigation",
                executable="perception_curb_descent_detection_node.py",
                name="perception_curb_descent_detection_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="neu_navigation",
                executable="perception_curb_detection_node.py",
                name="perception_curb_detection_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
```

- [ ] **Step 2: Syntax-check it (no ROS here, so compile only)**

Run: `python3 -m py_compile core/rammp_prototype_bringup/launch/neu_navigation.launch.py && echo ok`
Expected: `ok`.

- [ ] **Step 3: Confirm it matches full.launch.py**

Run: `grep -n -A5 'package="neu_navigation"' core/rammp_prototype_bringup/launch/full.launch.py | grep -E 'executable|name='`
Expected: the same two executables and node names as in the new file.

- [ ] **Step 4: Commit**

```bash
git add core/rammp_prototype_bringup/launch/neu_navigation.launch.py
git commit -m "bringup: neu_navigation.launch.py wraps the two NEU curb nodes (#256)"
```

______________________________________________________________________

### Task 4: Calibration script

**Files:**

- Create: `scripts/calibrate.sh` (executable)

**Interfaces:**

- Produces: `scripts/calibrate.sh` exits 0 after both calibrate goals succeed, nonzero if either goal fails. Referenced by the manifest's `calibration` node in Task 5.

- [ ] **Step 1: Write the script**

```bash
#!/usr/bin/env bash
# Calibrate the base, then the arm, as soon as each action server is up.
# Lifted from the calibration window of the old launch.sh. Needs the
# workspace sourced; under sheppy the manifest's ros_setup does that.
#
# `set -e` is deliberate: a failed goal exits nonzero, so the sheppy
# `calibration` node shows as crashed instead of sitting in `sleep infinity`.
set -euo pipefail

wait_for_action() {
    echo "Waiting for $1 action server..."
    until ros2 action list 2>/dev/null | grep -q "$1"; do sleep 3; done
}

wait_for_action /base/calibrate
echo "Base ready. Running base calibration..."
sleep 1
ros2 action send_goal /base/calibrate rammp_prototype_interfaces/action/Calibration '{enable: true}'

wait_for_action /arm/calibrate
echo "Arm ready. Running arm calibration..."
ros2 action send_goal /arm/calibrate arm_interfaces/action/Calibrate '{}'
echo "[calibration] done."
```

- [ ] **Step 2: Make it executable and syntax-check**

Run: `chmod +x scripts/calibrate.sh && bash -n scripts/calibrate.sh && echo ok`
Expected: `ok`.

- [ ] **Step 3: Confirm the goals match launch.sh**

Run: `grep -o 'ros2 action send_goal [^\\]*' core/rammp_prototype_bringup/launch/launch.sh`
Expected: the same two action names, types, and goal payloads as the script.

- [ ] **Step 4: Commit**

```bash
git add scripts/calibrate.sh
git commit -m "scripts: calibrate.sh, the base+arm calibration loop from launch.sh (#256)"
```

______________________________________________________________________

### Task 5: The manifest

**Files:**

- Create: `sheppy-manifest.yaml`

**Interfaces:**

- Consumes: compose service `cornell_feeding_robot` (Task 2), `neu_navigation.launch.py` (Task 3), `scripts/calibrate.sh` (Task 4).

- Produces: node names and alternative ids used by the profiles in Task 6: `description/description`, `base/{mebot_driver,mock}`, `luci/luci`, `arm/{arm_driver,mock}`, `gui_bridge/gui_bridge`, `system_control/system_control`, `door_opener/{cmu_door_opener,mock}`, `cameras/{cameras,mock}`, `curb_detection/{neu_navigation,mock}`, `cup_stabilizer/mock`, `drink/{cornell_feeding,mock}`, `gui/unreal`, `calibration/calibrate`.

- [ ] **Step 1: Write the manifest**

```yaml
# Sheppy manifest for the RAMMP demo. On the Jetson:
#   cd ~/ros2_ws/src/Demo-Software && sheppy up full
#
# One node per subsystem, transcribed from full.launch.py. Hardware nodes
# carry a mock alternative (the executables in rammp_prototype_behavior) so
# `profiles/mock.yaml` runs on a workstation with nothing attached. The mocks
# publish plausible messages at the right topics; they are not a simulation.
#
# Every non-docker alternative names the `jetson` machine, which is how
# sheppy sources the workspace before the command. That setup runs the
# bringup package's env hook, so RMW_IMPLEMENTATION and CYCLONEDDS_URI come
# along; no alternative sets its own env. Sheppy runs everything with
# `bash -c`, never zsh.
#
# Sheppy starts all selected nodes at once and never restarts one on its
# own. A crash shows in `sheppy status`; `sheppy woof <node>` restarts it.
machines:
  - name: jetson
    host: rammp-jetson
    user: herl
    ros_setup: ~/ros2_ws/install/setup.bash

nodes:
  - name: description
    description: robot_state_publisher and static TFs
    select: single
    alternatives:
      - id: description
        kind: launch_file
        machine: jetson
        package: rammp_prototype_description
        launch_file: description.launch.py

  - name: base
    description: MEBot wheel base over the Teensy serial link
    select: single
    alternatives:
      - id: mebot_driver
        kind: launch_file
        machine: jetson
        package: rammp_prototype_driver
        launch_file: control_node.launch.py
        params:
          serial_port: /dev/ttyACM0
      - id: mock
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: mock_chair_control

  - name: luci
    description: LUCI chair gRPC interface
    select: single
    alternatives:
      - id: luci
        kind: launch_file
        machine: jetson
        package: rammp_prototype_bringup
        launch_file: luci.launch.py
        params:
          chair_ip: 10.2.10.3
      # No mock: nothing else in the graph needs it, so a mock profile
      # leaves this node unselected.

  - name: arm
    description: Kinova arm driver
    select: single
    alternatives:
      - id: arm_driver
        kind: executable
        machine: jetson
        package: arm_driver
        executable: arm_driver
      - id: mock
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: mock_arm_driver
        publishes:
          - /arm/status
          - /arm/imu
          - /arm/ee_force
          - /estop

  - name: gui_bridge
    description: Bridge to the Unreal Engine GUI (a TCP client; reconnects on its own)
    select: single
    alternatives:
      - id: gui_bridge
        kind: launch_file
        machine: jetson
        package: rammp_prototype_gui
        launch_file: Gui_bridge.launch.py
        params:
          ue_host: 127.0.0.1

  - name: system_control
    description: Behavior state machine
    select: single
    alternatives:
      - id: system_control
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: system_control

  - name: door_opener
    description: CMU door opener (button detector + push controller)
    select: single
    alternatives:
      - id: cmu_door_opener
        kind: launch_file
        machine: jetson
        package: cmu_door_opener
        launch_file: cmu_door_opener.launch.py
      - id: mock
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: mock_opening_door

  - name: cameras
    description: RealSense wrist camera + Orbbec nav cameras (camera_demo_main.yaml)
    select: single
    alternatives:
      - id: cameras
        kind: launch_file
        machine: jetson
        package: rammp_prototype_bringup
        launch_file: camera.launch.py
        publishes:
          - /camera/wrist/color/image_raw
      - id: mock
        kind: process
        machine: jetson
        command: "ros2 topic pub -r 10 /camera/wrist/color/image_raw sensor_msgs/msg/Image '{}'"
        publishes:
          - /camera/wrist/color/image_raw

  - name: curb_detection
    description: NEU curb ascent/descent detection
    select: single
    alternatives:
      - id: neu_navigation
        kind: launch_file
        machine: jetson
        package: rammp_prototype_bringup
        launch_file: neu_navigation.launch.py
      - id: mock
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: mock_curb_detection

  - name: cup_stabilizer
    description: ATDev cup stabilizer; only the mock exists today
    select: single
    alternatives:
      - id: mock
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: mock_cup_stabilizer

  - name: drink
    description: Cornell feeding / drink action servers, in the cornell_feeding container
    select: single
    alternatives:
      - id: cornell_feeding
        kind: docker
        compose:
          file: docker-compose.yml
          service: cornell_feeding_robot
      - id: mock
        kind: executable
        machine: jetson
        package: rammp_prototype_behavior
        executable: mock_drinking_node

  - name: gui
    description: Unreal Engine GUI (launch_ui.sh in the operator's home, not in this repo)
    select: single
    alternatives:
      - id: unreal
        kind: process
        command: "cd $HOME && DISPLAY=:$(ls /tmp/.X11-unix | head -1 | tr -d X) exec ./launch_ui.sh"

  - name: calibration
    description: Calibrate base then arm once their action servers are up, then idle
    select: single
    alternatives:
      # Sheppy has no one-shot tasks and treats any exit as a crash, so the
      # script chains into sleep. A failed goal exits nonzero and the node
      # shows as crashed. Restarting this node reruns calibration.
      - id: calibrate
        kind: process
        machine: jetson
        command: "$HOME/ros2_ws/src/Demo-Software/scripts/calibrate.sh && sleep infinity"
```

- [ ] **Step 2: Validate (profiles do not exist yet, so expect exactly that complaint)**

Run: `uv run --with /home/swapnil/atdev/sheppy python scripts/validate_manifest.py; echo exit=$?`
Expected: exactly one line, ending in `profiles: no profiles found`, then `exit=1`. Any manifest error or docker warning other than that is a bug in the manifest; fix it before moving on.

- [ ] **Step 3: Cross-check against full.launch.py**

Run: `grep -cE '^  - name:' sheppy-manifest.yaml; grep -oE 'get_package_share_directory\("[a-z_]+"\)|package="[a-z_]+"' core/rammp_prototype_bringup/launch/full.launch.py | sort -u`
Expected: 13 nodes; every package listed from full.launch.py appears in the manifest (description, driver, bringup for luci/cameras, arm_driver, gui, behavior, cmu_door_opener, neu_navigation).

- [ ] **Step 4: Commit**

```bash
git add sheppy-manifest.yaml
git commit -m "sheppy manifest: one node per subsystem, transcribed from full.launch.py (#256)"
```

______________________________________________________________________

### Task 6: Profiles

**Files:**

- Create: `profiles/full.yaml`
- Create: `profiles/mock.yaml`

**Interfaces:**

- Consumes: node and alternative ids from Task 5.

- [ ] **Step 1: Write the full profile**

```yaml
# Everything real: what launch.sh started by default. Copy this file and
# delete a node's line to skip it (the old --no-arm / --no-cameras /
# --no-luci), or add an overrides block for --serial-port / --chair-ip /
# --ue-host, e.g.
#   overrides:
#     base: {serial_port: /dev/ttyACM1}
description: Everything real on the Jetson, with GUI and calibration
selections:
  description: description
  base: mebot_driver
  luci: luci
  arm: arm_driver
  gui_bridge: gui_bridge
  system_control: system_control
  door_opener: cmu_door_opener
  cameras: cameras
  curb_detection: mock
  cup_stabilizer: mock
  drink: cornell_feeding
  gui: unreal
  calibration: calibrate
```

Note `curb_detection: mock`: full.launch.py defaults `launch_neu_navigation` to false and launch.sh ran mock.launch.py, which starts `mock_curb_detection`. This keeps that behaviour. Switch it to `neu_navigation` when NEU is ready.

- [ ] **Step 2: Write the mock profile**

```yaml
# No hardware: runs on any machine with the workspace built. luci, gui and
# calibration have no mock and stay unselected (sheppy stops an unlisted node).
description: Every mock, no hardware, no GUI
selections:
  description: description
  base: mock
  arm: mock
  gui_bridge: gui_bridge
  system_control: system_control
  door_opener: mock
  cameras: mock
  curb_detection: mock
  cup_stabilizer: mock
  drink: mock
```

- [ ] **Step 3: Validate**

Run: `uv run --with /home/swapnil/atdev/sheppy python scripts/validate_manifest.py; echo exit=$?`
Expected: `ok: 13 nodes, 19 alternatives, 2 profiles (full, mock)` then `exit=0`.

- [ ] **Step 4: Prove the validator catches a dropped override and an unknown node**

```bash
printf 'selections: {base: mebot_driver, ghost: x}\noverrides:\n  base: {baud: 9600}\n' > profiles/bad.yaml
uv run --with /home/swapnil/atdev/sheppy python scripts/validate_manifest.py; echo exit=$?
rm profiles/bad.yaml
```

Expected: two lines, `profiles/bad: dropped selection: unknown node 'ghost'` and `profiles/bad: dropped override 'base.baud': not a declared param`, then `exit=1`.

- [ ] **Step 5: Commit**

```bash
git add profiles/full.yaml profiles/mock.yaml
git commit -m "sheppy profiles: full (launch.sh defaults) and mock (no hardware) (#256)"
```

______________________________________________________________________

### Task 7: Remove launch.sh and document the new flow

**Files:**

- Delete: `core/rammp_prototype_bringup/launch/launch.sh`

- Modify: `README.md` (add a "Running the system" section after the build instructions)

- [ ] **Step 1: Delete launch.sh**

Run: `git rm -q core/rammp_prototype_bringup/launch/launch.sh && git grep -n 'launch\.sh' -- ':!docs/superpowers' ':!third_party'`
Expected: no remaining references outside `docs/superpowers/` and `third_party/`. If any turn up, update them to point at the README section.

- [ ] **Step 2: Find where the build instructions end in README.md**

Run: `grep -n '^##' README.md`
Expected: a list of headings; the new section goes after the last "Getting Started" subsection (the colcon build step) and before any later top-level section.

- [ ] **Step 3: Add the README section**

````markdown
### Running the system

Bring-up is managed by [sheppy](https://rammp-org.github.io/sheppy): `sheppy-manifest.yaml`
at the repo root lists every node and its real and mock alternatives, and
`profiles/` says which to run. Install sheppy on the Jetson once:

```bash
curl -LsSf https://rammp-org.github.io/sheppy/install.sh | sh
````

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

````

- [ ] **Step 4: Check the README renders sanely and the validator still passes**

Run: `grep -n 'Running the system' README.md && uv run --with /home/swapnil/atdev/sheppy python scripts/validate_manifest.py`
Expected: the heading line, then the `ok:` line from the validator.

- [ ] **Step 5: Commit**

```bash
git add README.md core/rammp_prototype_bringup/launch/launch.sh
git commit -m "Replace launch.sh with sheppy; document the new bring-up (#256)"
````

______________________________________________________________________

### Task 8: Jetson verification (attended)

No files. This task is run on the Jetson by a person on the physical e-stop; it cannot be done from this workstation. Record the outcome in the PR.

- [ ] **Step 1: Prerequisites on the Jetson**

```bash
which sheppy docker && ls ~/ros2_ws/install/setup.bash ~/launch_ui.sh
cd ~/ros2_ws/src/Demo-Software && git checkout feature/256-sheppy-launch
cd ~/ros2_ws && colcon build --packages-select rammp_prototype_bringup && source install/setup.bash
cd ~/ros2_ws/src/Demo-Software && docker compose build cornell_feeding
```

Expected: all four paths exist; build succeeds. If the repo is not at `~/ros2_ws/src/Demo-Software`, edit the `calibration` command in `sheppy-manifest.yaml` to the real path and commit that.

- [ ] **Step 2: Mock profile first (no hardware moves)**

```bash
sheppy up mock; echo exit=$?
sheppy status
ros2 node list
sheppy down
```

Expected: `exit=0`; every selected node `running`; the node list shows the mocks, system_control and gui_bridge.

- [ ] **Step 3: Full profile**

```bash
sheppy up full; echo exit=$?
sheppy status
sheppy logs calibration -n 20
```

Expected: `exit=0`; all 13 nodes `running`; the calibration log ends with `[calibration] done.` and the GUI is on screen. Exercise a GUI action, then `sheppy down`.

- [ ] **Step 4: Note any deviation in the PR description and open the PR against `234-cornell-feeding`**

```bash
gh pr create --base 234-cornell-feeding --title "Migrate launch to sheppy (#256)" --body-file docs/superpowers/plans/2026-09-23-sheppy-launch.md
```
