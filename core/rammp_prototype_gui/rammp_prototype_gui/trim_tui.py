"""Live camera extrinsics trim adjuster for the GuiBridge node.

Keyboard-driven TUI that sets ROS 2 parameters on the running
Gui_bridge_node in real time so you can visually align cameras in UE.

Usage:
    python3 -m rammp_prototype_gui.trim_tui
    # or via colcon entry point:
    ros2 run rammp_prototype_gui TrimTUI
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Header, Static


CAMERAS = ["wrist", "nav1", "nav2"]
AXES = ["x", "y", "z", "roll", "pitch", "yaw"]
UNITS = ["cm", "cm", "cm", "deg", "deg", "deg"]
STEPS = [10.0, 1.0, 0.1, 0.01]
NODE_NAME = "/Gui_bridge_node"


@dataclass
class TrimState:
    values: dict[str, dict[str, float]] = field(default_factory=dict)

    def __post_init__(self):
        for cam in CAMERAS:
            self.values[cam] = {axis: 0.0 for axis in AXES}


def _set_ros_param(camera: str, axis: str, value: float) -> None:
    try:
        subprocess.Popen(
            [
                "ros2",
                "param",
                "set",
                NODE_NAME,
                f"{camera}_trim_{axis}",
                str(value),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


def _render_table(state: TrimState, cam_idx: int, axis_idx: int, step: float) -> str:
    lines = []
    lines.append(f"  Step: {step:>8.2f}   (s to cycle)")
    lines.append("")
    header = "  {:>8s}".format("")
    for i, cam in enumerate(CAMERAS):
        marker = " >" if i == cam_idx else "  "
        header += f"{marker}{cam:>10s}"
    lines.append(header)
    lines.append("  " + "─" * 40)

    for j, (axis, unit) in enumerate(zip(AXES, UNITS)):
        row_marker = "→" if j == axis_idx else " "
        label = f"  {row_marker} {axis:>5s} ({unit})"
        for i, cam in enumerate(CAMERAS):
            val = state.values[cam][axis]
            if i == cam_idx and j == axis_idx:
                cell = f"  [{val:>8.2f}]"
            else:
                cell = f"   {val:>8.2f} "
            label += cell
        lines.append(label)

    lines.append("")
    lines.append("  ←/→  adjust   ↑/↓  axis   Tab  camera   r  reset   q  quit")
    return "\n".join(lines)


class TrimDisplay(Static):
    pass


class TrimTUI(App):
    CSS = """
    TrimDisplay {
        width: 1fr;
        height: 1fr;
        padding: 1 2;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("s", "cycle_step", "Step"),
        Binding("tab", "next_camera", "Next Cam"),
        Binding("shift+tab", "prev_camera", "Prev Cam"),
        Binding("up", "prev_axis", "Axis ↑"),
        Binding("down", "next_axis", "Axis ↓"),
        Binding("right", "increment", "+"),
        Binding("left", "decrement", "−"),
        Binding("r", "reset", "Reset"),
    ]

    def __init__(self):
        super().__init__()
        self.state = TrimState()
        self.cam_idx = 0
        self.axis_idx = 0
        self.step_idx = 1

    def compose(self) -> ComposeResult:
        yield Header()
        yield TrimDisplay(self._render())
        yield Footer()

    def _render(self) -> str:
        return _render_table(
            self.state, self.cam_idx, self.axis_idx, STEPS[self.step_idx]
        )

    def _refresh(self) -> None:
        self.query_one(TrimDisplay).update(self._render())

    def _push(self) -> None:
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        _set_ros_param(cam, axis, self.state.values[cam][axis])

    def action_cycle_step(self) -> None:
        self.step_idx = (self.step_idx + 1) % len(STEPS)
        self._refresh()

    def action_next_camera(self) -> None:
        self.cam_idx = (self.cam_idx + 1) % len(CAMERAS)
        self._refresh()

    def action_prev_camera(self) -> None:
        self.cam_idx = (self.cam_idx - 1) % len(CAMERAS)
        self._refresh()

    def action_prev_axis(self) -> None:
        self.axis_idx = (self.axis_idx - 1) % len(AXES)
        self._refresh()

    def action_next_axis(self) -> None:
        self.axis_idx = (self.axis_idx + 1) % len(AXES)
        self._refresh()

    def action_increment(self) -> None:
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        self.state.values[cam][axis] += STEPS[self.step_idx]
        self._push()
        self._refresh()

    def action_decrement(self) -> None:
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        self.state.values[cam][axis] -= STEPS[self.step_idx]
        self._push()
        self._refresh()

    def action_reset(self) -> None:
        for cam in CAMERAS:
            for axis in AXES:
                self.state.values[cam][axis] = 0.0
                _set_ros_param(cam, axis, 0.0)
        self._refresh()


def main():
    app = TrimTUI()
    app.run()


if __name__ == "__main__":
    main()
