# keyboard_driver

ROS 2 node that maps a small **W / E / R / T** keypad to MEBot high-level
state-machine commands. Built for the SayoDevice 1x4P macropad but works with
any keyboard that emits `KEY_W/E/R/T`.

Each press calls the `/GuiBridge/user_input` service (like the GUI and gamepad),
so the state machine validates it and the UI stays in sync. The node subscribes
to `/system/state` so a key resolves to the right command for the current state.
See [SPEC.md](SPEC.md).

## Key behavior

| Key   | In `Nav_SLOff`/`Nav_SLOn` | In matching `*Detecting` | Anytime         |
| ----- | ------------------------- | ------------------------ | --------------- |
| **W** | `chair/curb/ascend`       | `system/confirm`         |                 |
| **E** | `chair/curb/descend`      | `system/confirm`         |                 |
| **R** | `chair/selfLeveling/on`   |                          |                 |
| **T** |                           |                          | `system/cancel` |

Curb commands are two-step (mirrors the GUI "curb climb -> confirm"): press
**W** once to arm detection, again to start the climb; same for **E**. **T**
stops/aborts at any point. Entering curb mode (`chair/curb/navigation`) and
recovering from a cancel (`system/reset`) are done from the GUI.

Commands invalid in the current state are rejected by the state machine
(logged; echoed on `/keyboard/event` as `rejected:<command>`).

## Dependencies

- `python3-evdev` (rosdep key `python3-evdev`, or `pip install evdev`)
- Read access to `/dev/input/event*` — be in the `input` group
  (`sudo usermod -aG input $USER`, then re-login) or install the udev rule below.

## udev rule (recommended)

The 1x4P exposes several `/dev/input/event*` nodes; only the one on USB
interface 0 actually emits the keypresses. `udev/99-mebot-keypad.rules` creates a
stable `/dev/mebot_keypad` symlink to that node so the launch file can pin it
directly (no multiplexing, deterministic across reboots).

This rule is installed by the repo-root **`setup.sh`** alongside the project's
other udev rules — run that once on the machine:

```bash
./setup.sh
```

To (re)install just this rule manually:

```bash
sudo cp hardware/keyboard_driver/udev/99-mebot-keypad.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/mebot_keypad   # -> ../input/event5 (or wherever interface 0 lands)
```

If the symlink is absent, the node logs a warning and falls back to
auto-selecting every node that advertises the target keys.

## Build & run

```bash
colcon build --packages-select keyboard_driver --symlink-install
source install/setup.bash
ros2 launch keyboard_driver keyboard.launch.py
# observe:
ros2 topic echo /keyboard/event
```

### Parameters

| Param         | Default             | Meaning                                             |
| ------------- | ------------------- | --------------------------------------------------- |
| `device_path` | `/dev/mebot_keypad` | Exact device/symlink to open. Empty => auto-select. |
| `device_name` | `""`                | Fallback name-substring filter for auto-select.     |
| `grab_device` | `false`             | `EVIOCGRAB` the device for exclusive access.        |

Override e.g. `ros2 launch keyboard_driver keyboard.launch.py` after editing the
launch file, or run the node directly with `--ros-args -p device_path:=...`.
