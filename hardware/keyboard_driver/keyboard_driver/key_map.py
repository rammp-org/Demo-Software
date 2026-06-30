# Pure key->action mapping with no ROS or evdev imports, so it can be unit-tested
# on a plain host (no rclpy, no keyboard device, no evdev C-extension).
#
# Keys are evdev key *names* (ecodes.KEY[event.code]).
#
# Phase 1 publishes these labels on /keyboard/event to verify plumbing.
# Phase 2 will map these same keys to gui_interfaces UserInputs.Request constants
# and call the /GuiBridge/user_input service instead of publishing a dummy topic.
KEY_TO_ACTION = {
    "KEY_W": "curb_ascend",
    "KEY_E": "curb_descend",
    "KEY_R": "self_leveling_on",
    "KEY_T": "cancel",
}
