# Plan: Keyboard Input Node for MEBot State Machine

## Context

We want a physical 4-key keyboard (W / E / R / T) to drive the MEBot's high-level
state-machine actions: curb climb, curb descend, self-leveling, and cancel. This gives
operators a cheap, tactile control surface alongside the existing GUI and gamepad.

The repo already has the exact pattern to follow: **`hardware/gamepad_driver`**. Its
`gamepad_node` is a ROS 2 node that acts as a **client** of the `/GuiBridge/user_input`
service and injects high-level commands via `send_user_input(UserInputs.Request.X)`. The
service **server** lives in the behavior package (`core/rammp_prototype_behavior/.../system_control.py:861`),
which owns the state machine and validates/dispatches each input. The GUI bridge
(`Gui_bridge.py`) is itself just another client of the same service.

So "sending commands to the GUI bridge node" is realized by calling the same
`/GuiBridge/user_input` service the GUI bridge uses — the keyboard node becomes a third
input source (GUI, gamepad, keyboard) feeding the one state machine. The state machine
enforces validity per state, so no command logic needs to live in the keyboard node.

The command strings already exist in `interfaces/gui_interfaces/srv/UserInputs.srv`:

| Key | Action        | UserInputs request                                                   |
| --- | ------------- | -------------------------------------------------------------------- |
| W   | Curb climb    | `UserInputs.Request.CHAIR_CURB_ASCEND` (`chair/curb/ascend`)         |
| E   | Curb descend  | `UserInputs.Request.CHAIR_CURB_DESCEND` (`chair/curb/descend`)       |
| R   | Self-leveling | `UserInputs.Request.CHAIR_SELFLEVELING_ON` (`chair/selfLeveling/on`) |
| T   | Cancel        | `UserInputs.Request.CANCEL` (`system/cancel`)                        |

### Decisions (confirmed with user)

- **Input mechanism:** read the USB keyboard device directly via **`python3-evdev`** (works
  headless, no terminal focus; treats the small keyboard like a macropad).
- **Self-level key:** always sends `chair/selfLeveling/on` (stateless; no local toggle state).
- **Cancel key:** sends `system/cancel` (general state-machine cancel).
- Fire on **key-down only** (evdev `value == 1`); ignore autorepeat (`2`) and key-up (`0`).

## Why `/GuiBridge/user_input` keeps the UI in sync (verified)

The UI is **state-driven, not input-driven**, so routing through this service is the only
path that stays in sync:

- `system_control` publishes its authoritative state to `/system/state` on a 10 Hz timer
  (`system_control.py:738`, `publish_system_state` at `:762` — `msg.state = self.state`),
  **regardless of which client triggered the transition**.
- The GUI bridge renders from `/system/state`, not from button presses
  (`Gui_bridge.py:1210-1219` `system_state_callback` → `send_system_state_to_ue()`).

So: keypress → service call → state-machine transition → state broadcast → UI updates.
The keyboard becomes a third input peer alongside the GUI and gamepad, and the UI reflects
it automatically. **Do NOT** bypass the state machine (e.g. publishing to
`/base/manual_seat_control` or writing serial directly) — that moves hardware without
updating `self.state`, leaving `/system/state` and the UI out of sync. The behavior
service also runs its own validation in `_srv_user_inputs_callback`, so commands invalid in
the current state are safely rejected (`success=False`).

## Phased development

Build incrementally, verifying the ROS plumbing before wiring any real behavior.

- **Phase 1 (this step): standalone plumbing node — NO state-machine hookup.**
  A minimal `keyboard_node` that opens the evdev keyboard, reads W/E/R/T key-downs, and
  for each press just (a) logs via `self.get_logger().info(...)` and (b) publishes the
  action name as a `std_msgs/String` on a dummy topic `/keyboard/event`. Goal: prove
  the device read + ROS node + build/launch all work end-to-end, observable with
  `ros2 topic echo /keyboard/event`. No `UserInputs` service client yet.
- **Phase 2 (done): state-aware hook to the state machine.** The node calls
  `/GuiBridge/user_input` (gamepad's `send_user_input()` pattern) and subscribes to
  `/system/state` so each key resolves to the right command for the current state:
  - W: `chair/curb/ascend` from `Nav_SLOff`/`Nav_SLOn`, else `system/confirm` when in
    `Nav_ascendDetecting` (two-step "arm then confirm", mirroring the GUI confirm button).
  - E: `chair/curb/descend` / `system/confirm` (gated on `Nav_descendDetecting`).
  - R: `chair/selfLeveling/on`. T: `system/cancel` (valid in SL-on, detecting, traversing).
  - Routing logic lives in `key_map.resolve_action(key, state)` (ROS-free, host-tested);
    `keyboard_node.ACTION_TO_USERINPUT` maps symbolic actions to `UserInputs.Request`.
  - Entering curb mode (`chair/curb/navigation`) and reset-from-`Nav_paused`
    (`system/reset`) stay on the GUI. The mock auto-confirm in `system_control.py` is not
    run in the demo, so the two-step gating is real.

The package scaffolding (package.xml/setup.py/launch/tests) is created in Phase 1 and
carries forward; only the per-key action changes in Phase 2.

This plan file should also be saved into the repo for the team — recommend
`hardware/keyboard_driver/SPEC.md` (per the CLAUDE.md "generate a SPEC.md" convention).

## New package: `hardware/keyboard_driver`

Mirror `hardware/gamepad_driver` structure exactly (ament_python):

```
hardware/keyboard_driver/
├── package.xml          # clone gamepad's; deps: rclpy, gui_interfaces, python3-evdev
├── setup.py             # entry_point: keyboard_node = keyboard_driver.keyboard_node:main
├── setup.cfg            # clone gamepad's
├── requirements.txt     # evdev  (pip fallback if rosdep key unavailable)
├── resource/keyboard_driver        # empty ament marker
├── keyboard_driver/
│   ├── __init__.py      # empty
│   └── keyboard_node.py # the node
├── launch/keyboard.launch.py
└── test/                # copy test_flake8.py / test_pep257.py / test_copyright.py from gamepad
```

### `keyboard_node.py`

Model on `hardware/gamepad_driver/gamepad_driver/gamepad_node.py`. Key points:

- `class KeyboardNode(Node)` named `"keyboard_node"`.
- **Reuse the gamepad's `send_user_input()` verbatim** (`gamepad_node.py:94-114`): create
  `self.user_input_service_client = self.create_client(UserInputs, "/GuiBridge/user_input", callback_group=self._cb_group)`,
  then `call_async` + `threading.Event` wait. Use a `ReentrantCallbackGroup` and a
  `MultiThreadedExecutor` in `main()` (same as gamepad) so the client future resolves while
  the read thread blocks.
- **Parameters** (via `declare_parameter`, gamepad style):
  - `device_path` (string, default `""`) — explicit `/dev/input/eventX`.
  - `device_name` (string, default e.g. `"keyboard"`) — substring matched against
    `evdev.list_devices()` device names when `device_path` is empty.
  - `grab_device` (bool, default `False`) — call `dev.grab()` for exclusive capture so the
    keystrokes don't leak into other apps/terminals on the Jetson.
- **Key→command map** as a module-level dict keyed by evdev keycode, e.g.
  `{ecodes.KEY_W: UserInputs.Request.CHAIR_CURB_ASCEND, ecodes.KEY_E: ...CHAIR_CURB_DESCEND, ecodes.KEY_R: ...CHAIR_SELFLEVELING_ON, ecodes.KEY_T: ...CANCEL}`.
  Keep this a pure dict so the mapping is unit-testable without hardware.
- **Device acquisition:** helper `_open_device()` that resolves `device_path` or scans
  `evdev.list_devices()` by `device_name`; logs a clear error and the available device list
  if none found.
- **Read loop:** run `dev.read_loop()` in a **daemon thread** started after init. For each
  `EV_KEY` event with `value == 1`, look up the keycode in the map and call
  `send_user_input(...)`. Wrap in try/except and log unknown keys at debug. (Service call
  blocks up to ~5 s in the thread — fine, mirrors gamepad behavior.)
- Optional but recommended: simple reconnect — if `read_loop` raises `OSError` (device
  unplugged), log and retry `_open_device()` on a backoff. Keep minimal.

### `keyboard.launch.py`

Clone `gamepad.launch.py` but a single node (no `joy_node` equivalent — evdev reads the
device directly):

```python
Node(package="keyboard_driver", executable="keyboard_node",
     name="keyboard_node", output="screen",
     parameters=[{"device_name": "keyboard", "grab_device": False}])
```

## Files to create

- `hardware/keyboard_driver/keyboard_node.py` (new — the node)
- `hardware/keyboard_driver/{package.xml, setup.py, setup.cfg, requirements.txt}` (clone gamepad)
- `hardware/keyboard_driver/resource/keyboard_driver`, `keyboard_driver/__init__.py`
- `hardware/keyboard_driver/launch/keyboard.launch.py`
- `hardware/keyboard_driver/test/{test_flake8,test_pep257,test_copyright}.py` (copy from gamepad)

## Reuse (do not reinvent)

- `send_user_input()` and the MultiThreadedExecutor `main()` — copy from
  `hardware/gamepad_driver/gamepad_driver/gamepad_node.py`.
- Command constants from `gui_interfaces.srv.UserInputs.Request` — never hardcode the strings.
- Package scaffolding (setup.py/setup.cfg/package.xml/test/) — copy from `gamepad_driver`.

## Dependencies & permissions (document, do not execute here)

- `python3-evdev` — add `<depend>python3-evdev</depend>` to package.xml and `evdev` to
  `requirements.txt` (rosdep key is `python3-evdev`; pip fallback `pip install evdev`).
- Reading `/dev/input/event*` requires the running user to be in the `input` group
  (`sudo usermod -aG input $USER`, then re-login) **or** a udev rule. Document in the
  package README / launch comments.

## Verification (to be run by the user on the Jetson — I will not execute these)

Per project testing convention, I will write the node + tests and document the procedure
but will not run hardware-connected steps myself.

1. **Build:** from the colcon workspace root,
   `colcon build --packages-select keyboard_driver --symlink-install && source install/setup.bash`.
1. **Identify the device:** `python3 -c "import evdev; [print(p, evdev.InputDevice(p).name) for p in evdev.list_devices()]"`
   to confirm the keyboard's event path / name, set `device_name`/`device_path` accordingly.
1. **Dry-run without the state machine:** `ros2 launch keyboard_driver keyboard.launch.py`,
   press W/E/R/T, and confirm the node logs `Sending user input to ROS: chair/curb/ascend` etc.
   (It will log "service not available" if the behavior node isn't running — expected.)
1. **End-to-end:** with the behavior/GUI-bridge stack up, watch
   `ros2 service echo`/node logs and confirm each key drives the expected state transition
   (`system_control._srv_user_inputs_callback` logs `Received user input: ...`). Verify the
   state machine accepts the input only in valid states (e.g. curb ascend may require being
   in the chair/curb context first) and returns `success=True`.
1. **Unit test (host, no hardware):** a pytest asserting the key→`UserInputs.Request`
   mapping dict is correct and covers exactly W/E/R/T. This is runnable without a device.

## Notes / open items

- Current branch `feat/233-keyboard-input` matches this task; confirm it was branched from
  `dev` per CONTRIBUTING before opening the PR (PR target: `dev`).
- The legacy `GUI/components/TeensyController.py` mode+action protocol is **not** used by the
  current firmware — ignore it; all routing goes through `/GuiBridge/user_input`.
