# Migrate the RAMMP launch to sheppy

Issue: #256. Branch: `feature/256-sheppy-launch`, based on `234-cornell-feeding`.

## Goal

Replace `core/rammp_prototype_bringup/launch/launch.sh` (a tmux script that
starts the GUI, the mock launch, `full.launch.py`, and a calibration loop) with
a sheppy manifest and profiles. The operator brings the system up with one
command, sees per-node status and logs, and can restart one node without
restarting everything. Every hardware node gets a mock alternative so a profile
with no hardware runs on a workstation.

Sheppy is `rammp-org/sheppy`, version 1.0.0. What it does and does not do
shapes this design:

- Nodes with swappable alternatives of kind `executable`, `launch_file`,
  `process`, or `docker`; profiles select one alternative per node and can
  override params the manifest declares.
- Everything runs on the machine where `sheppyd` runs. The `machine` field
  only picks which `ros_setup` to source. No remote hosts.
- All nodes start at once. No ordering, readiness waits, hooks, or delays.
- A node that exits, even with status 0, is `crashed`. No auto-restart; the
  operator runs `sheppy restart <node>` (`woof` is an alias).
- Commands run through `bash -c`. No zsh.

## Layout

```
Demo-Software/
  sheppy-manifest.yaml                 # what can run
  profiles/full.yaml                   # everything real (launch.sh defaults)
  profiles/mock.yaml                   # no hardware, runs on a workstation
  scripts/calibrate.sh                 # wait-and-send loop lifted from launch.sh
  scripts/validate_manifest.py         # loads the manifest through sheppy's loader
  core/rammp_prototype_bringup/launch/neu_navigation.launch.py   # new wrapper
```

`launch.sh` is deleted. Its header comment becomes a "Running the system"
section in `README.md`.

Sheppy reads `sheppy-manifest.yaml` from the current directory, so the operator
flow on the Jetson is:

```bash
cd ~/ros2_ws/src/Demo-Software
sheppy up full        # or: sheppy up mock
sheppy status
sheppy logs <node> -n 100
sheppy woof <node>    # restart one node
sheppy down           # stop everything and the daemon
```

Per-node logs live under `~/.sheppy/logs/<node>/`, one file per run, which
replaces the `rammp_logs/` redirect in `launch.sh`.

## Machine and environment

One entry under `machines:`:

```yaml
machines:
  - name: jetson
    host: rammp-jetson
    user: rammp
    ros_setup: ~/ros2_ws/install/setup.bash
```

Every `executable`, `launch_file`, and `process` alternative sets
`machine: jetson`, which makes sheppy prefix the command with
`source ~/ros2_ws/install/setup.bash &&`. That overlay setup chains the Humble
underlay and runs the bringup package's env hook
(`core/rammp_prototype_bringup/env_hooks/rmw_config.dsv`), which sets
`RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`. No alternative sets its own env.

`ROS_DOMAIN_ID` is inherited from the shell that first runs `sheppy up`. The
docker alternative sets it explicitly under `environment:`, as the compose
file already does.

## Nodes

One sheppy node per subsystem. Real alternatives transcribe what
`full.launch.py` includes today; mocks are the existing executables in
`rammp_prototype_behavior`. `publishes`/`subscribes` lists are documentation
and are filled in from the ROS spec where known.

| node             | real alternative                                                                            | mock alternative                                                                              | declared params             |
| ---------------- | ------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- | --------------------------- |
| `description`    | `launch_file` rammp_prototype_description `description.launch.py`                           | none; runs anywhere                                                                           |                             |
| `base`           | `launch_file` rammp_prototype_driver `control_node.launch.py`                               | `executable` rammp_prototype_behavior `mock_chair_control`                                    | `serial_port: /dev/ttyACM0` |
| `luci`           | `launch_file` rammp_prototype_bringup `luci.launch.py`                                      | none; left unselected                                                                         | `chair_ip: 10.2.10.3`       |
| `arm`            | `executable` arm_driver `arm_driver`                                                        | `executable` rammp_prototype_behavior `mock_arm_driver`                                       |                             |
| `gui_bridge`     | `launch_file` rammp_prototype_gui `Gui_bridge.launch.py`                                    | none; it is a TCP client                                                                      | `ue_host: 127.0.0.1`        |
| `system_control` | `executable` rammp_prototype_behavior `system_control`                                      | none                                                                                          |                             |
| `door_opener`    | `launch_file` cmu_door_opener `cmu_door_opener.launch.py`                                   | `executable` rammp_prototype_behavior `mock_opening_door`                                     |                             |
| `cameras`        | `launch_file` rammp_prototype_bringup `camera.launch.py`                                    | `process` publishing an empty `sensor_msgs/Image` on `/camera/wrist/color/image_raw` at 10 Hz |                             |
| `curb_detection` | `launch_file` rammp_prototype_bringup `neu_navigation.launch.py` (new)                      | `executable` rammp_prototype_behavior `mock_curb_detection`                                   |                             |
| `cup_stabilizer` | `executable` rammp_prototype_behavior `mock_cup_stabilizer` (only alternative today)        |                                                                                               |                             |
| `drink`          | `docker`, `compose: {file: docker-compose.yml, service: cornell_feeding_robot}`             | `executable` rammp_prototype_behavior `mock_drinking_node`                                    |                             |
| `gui`            | `process`: `cd $HOME && DISPLAY=:$(ls /tmp/.X11-unix \| head -1 \| tr -d X) ./launch_ui.sh` | none; left unselected                                                                         |                             |
| `calibration`    | `process`: `<repo>/scripts/calibrate.sh && sleep infinity`                                  | none; left unselected                                                                         |                             |

Notes per node:

- **`base` params.** `serial_port` is declared in the manifest so a profile can
  override it. Sheppy drops overrides for params the manifest does not
  declare, so `chair_ip` and `ue_host` are declared for the same reason.
- **`cameras`.** `camera.launch.py` already defaults `params_file` to the
  installed `config/camera_demo_main.yaml`, the same file `full.launch.py`
  passes, so no param is declared. Its other args (`disable_*`, serials) are
  left at their defaults, as today.
- **`curb_detection` wrapper.** `full.launch.py` starts two script nodes from
  `neu_navigation` (`perception_curb_descent_detection_node.py` and
  `perception_curb_detection_node.py`). One sheppy alternative is one command,
  so `neu_navigation.launch.py` in the bringup package launches the pair.
  `full.launch.py` is not changed.
- **`drink` container.** The compose service `cornell_feeding` runs its launch
  in sim mode. A second service, `cornell_feeding_robot`, is added to
  `docker-compose.yml` with the same `<<: *ros-common` anchor and volumes, and
  `command: ros2 launch cornell_feeding cornell_feeding.launch.py run_on_robot:=true`.
  The sheppy alternative names that service. Sheppy ignores `build:` and
  `container_name:` with a warning and runs the named image, so the image must
  be built first with `docker compose build cornell_feeding`.
- **`gui`.** `launch_ui.sh` lives in the operator's home on the Jetson, not in
  this repo, exactly as today.
- **`calibration`.** `scripts/calibrate.sh` is the loop from `launch.sh`:
  poll `ros2 action list` every 3 s until `/base/calibrate` exists, send the
  base goal and wait for the result, then the same for `/arm/calibrate`. The
  alternative chains `&& sleep infinity` so the node stays `running`; without
  that, sheppy would flag the clean exit as `crashed` and `sheppy up` would
  exit 1. Calibration reruns whenever this node restarts, including when a
  profile change touches it.

## Profiles

`profiles/full.yaml` selects every real alternative, including `gui` and
`calibration`. Its overrides section is empty; the manifest defaults match
`launch.sh` defaults (serial `/dev/ttyACM0`, chair `10.2.10.3`, UE host
`127.0.0.1`).

`profiles/mock.yaml` selects every mock alternative and leaves `luci`, `gui`,
and `calibration` unselected. `description`, `gui_bridge`,
`system_control`, and `cup_stabilizer` are selected as-is, since they need no
hardware.

Today's `--no-arm`, `--no-cameras`, `--no-luci` flags become "copy `full.yaml`
and delete that node's line". An unlisted node is stopped by `sheppy up`. The
README says this. `--serial-port`, `--chair-ip`, `--ue-host` become an
`overrides:` block in a copied profile.

## Behaviour changes from launch.sh

- **No auto-respawn.** `full.launch.py` gave `arm_driver` and
  `system_control` `respawn=True` with a 2 s delay. Under sheppy a crash shows
  in `sheppy status` with its exit code and the operator runs `sheppy woof`.
  Accepted for this migration.
- **No start ordering.** `launch.sh` started the GUI 2 s before the ROS nodes.
  `gui_bridge` reconnects on its own, so the head start is dropped.
- **No serial-port preflight.** If `/dev/ttyACM0` is absent the `base` node
  crashes and its log says why.
- **Logs** go to sheppy's per-node files instead of one combined file.

## Validation

`scripts/validate_manifest.py` follows the rammp-deployments script: it loads
the manifest through `sheppy.manifest.loader.load_manifest`, fails on any
error, and prints docker-translation warnings. Unlike the rammp-deployments
version it tolerates the `build` and `container_name` warnings, since the
compose file is also used by plain `docker compose`. Any other warning fails.

It runs on a workstation with sheppy from the developer's checkout
(`uv run --project ~/atdev/sheppy python scripts/validate_manifest.py`) or a
pip install of sheppy. Adding it to CI is out of scope for this issue.

## Testing

1. **Manifest loads.** `validate_manifest.py` exits 0.
1. **Workstation smoke test.** On a machine with the workspace built:
   `sheppy up mock`, then `sheppy status` shows every selected node
   `running`, `ros2 node list` shows the mocks, `sheppy down` exits 0.
1. **Hardware test on the Jetson.** Attended, with someone on the e-stop:
   confirm sheppy and docker are installed and the cornell image is built;
   `sheppy up full`; `sheppy status` all `running`; `sheppy logs calibration`
   shows both goals succeeded; exercise the GUI; `sheppy down`.

## Out of scope

- Changes to `full.launch.py` or any existing launch file. `full.launch.py`
  stays usable outside sheppy.
- A CI workflow for the validator.
- Real alternatives for `cup_stabilizer` or a mock for `luci`.
- Running any node on a machine other than the Jetson.
