# State-aware keypress routing, with no ROS or evdev imports so it can be
# unit-tested on a plain host.
#
# resolve_action(key_name, state) takes an evdev key name and the current
# SystemState.state string and returns a *symbolic* action. keyboard_node maps
# the symbolic action to the matching gui_interfaces UserInputs.Request string
# (kept out of here to keep this module ROS-free).
#
# Two-step curb commands mirror the GUI's "curb climb -> confirm" flow:
#   - 1st press (in Nav_SLOff/Nav_SLOn): ASCEND/DESCEND arms detection.
#   - 2nd press (in the matching *Detecting state): CONFIRM starts the traverse.
# CONFIRM is gated on the matching direction so a stray W can't confirm a descend.

# evdev key names this node acts on (used for device capability auto-select).
TARGET_KEYS = ("KEY_W", "KEY_E", "KEY_R", "KEY_T")

ASCEND = "ASCEND"
DESCEND = "DESCEND"
CONFIRM = "CONFIRM"
SELFLEVEL_ON = "SELFLEVEL_ON"
CANCEL = "CANCEL"


def resolve_action(key_name, state):
    state = state or ""
    if key_name == "KEY_W":
        return CONFIRM if "ascendDetecting" in state else ASCEND
    if key_name == "KEY_E":
        return CONFIRM if "descendDetecting" in state else DESCEND
    if key_name == "KEY_R":
        return SELFLEVEL_ON
    if key_name == "KEY_T":
        return CANCEL
    return None
