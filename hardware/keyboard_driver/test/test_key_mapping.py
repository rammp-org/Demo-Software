# Host-runnable mapping test: key_map has no ROS or evdev imports, so this runs
# on a plain host with just pytest (no rclpy, no keyboard device).

from keyboard_driver.key_map import KEY_TO_ACTION


def test_mapping_covers_exactly_wert():
    assert set(KEY_TO_ACTION.keys()) == {"KEY_W", "KEY_E", "KEY_R", "KEY_T"}


def test_mapping_targets():
    assert KEY_TO_ACTION["KEY_W"] == "curb_ascend"
    assert KEY_TO_ACTION["KEY_E"] == "curb_descend"
    assert KEY_TO_ACTION["KEY_R"] == "self_leveling_on"
    assert KEY_TO_ACTION["KEY_T"] == "cancel"
