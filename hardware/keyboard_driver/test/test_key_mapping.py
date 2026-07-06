# Host-runnable routing test: key_map has no ROS or evdev imports, so this runs
# on a plain host with just pytest (no rclpy, no keyboard device).

from keyboard_driver.key_map import (
    ASCEND,
    CANCEL,
    CONFIRM,
    DESCEND,
    SELFLEVEL_ON,
    resolve_action,
)


def test_first_press_arms_curb_commands():
    for state in ("Nav_SLOff", "Nav_SLOn"):
        assert resolve_action("KEY_W", state) == ASCEND
        assert resolve_action("KEY_E", state) == DESCEND


def test_second_press_confirms_matching_direction():
    assert resolve_action("KEY_W", "Nav_ascendDetecting") == CONFIRM
    assert resolve_action("KEY_E", "Nav_descendDetecting") == CONFIRM


def test_confirm_is_direction_gated():
    # W must not confirm a descend, and E must not confirm an ascend.
    assert resolve_action("KEY_W", "Nav_descendDetecting") == ASCEND
    assert resolve_action("KEY_E", "Nav_ascendDetecting") == DESCEND


def test_selflevel_and_cancel_are_stateless():
    for state in ("", "Nav_SLOff", "Nav_SLOn", "Nav_ascending", "Nav_paused"):
        assert resolve_action("KEY_R", state) == SELFLEVEL_ON
        assert resolve_action("KEY_T", state) == CANCEL


def test_unmapped_key_returns_none():
    assert resolve_action("KEY_X", "Nav_SLOff") is None


def test_none_state_is_safe():
    assert resolve_action("KEY_W", None) == ASCEND
