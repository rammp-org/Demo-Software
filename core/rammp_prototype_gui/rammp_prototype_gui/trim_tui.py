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
from textual.widgets import Footer, Header, Input, Static


CAMERAS = ["wrist", "nav1", "nav2"]
AXES = ["x", "y", "z", "roll", "pitch", "yaw"]
UNITS = ["cm", "cm", "cm", "deg", "deg", "deg"]
STEPS = [10.0, 1.0, 0.1, 0.01]
NODE_NAME = "/Gui_bridge_node"

# ── Visual style constants ─────────────────────────────────────────────────────
#   Each camera gets a distinct accent colour used for its header and column.
CAM_COLORS = ["cyan", "bright_yellow", "green"]

#   Step size → colour (coarse → hot, fine → cool).
STEP_COLORS = {10.0: "red", 1.0: "yellow", 0.1: "bright_green", 0.01: "bright_cyan"}
STEP_LABELS = {10.0: "coarse", 1.0: "medium", 0.1: "fine", 0.01: "precise"}

#   Table geometry (all widths are in visible terminal columns).
#   Each value cell:  "  {value:>8.2f}  "  →  2 + 8 + 2 = 12 columns.
#   Row-label prefix: marker(1) + space(1) + axis(5) + " (unit)"(6) = 13 columns.
CELL_W = 12
LABEL_W = 13


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
    cam = CAMERAS[cam_idx]
    axis = AXES[axis_idx]
    unit = UNITS[axis_idx]
    cam_color = CAM_COLORS[cam_idx]
    step_color = STEP_COLORS.get(step, "white")
    step_label = STEP_LABELS.get(step, "")

    # Separator spans the label prefix + all three value columns.
    sep_len = LABEL_W + CELL_W * len(CAMERAS)
    lines: list[str] = []

    # ── Status / context line ──────────────────────────────────────────────────
    lines.append(
        f"  [dim]selected[/dim]  "
        f"[bold {cam_color}]{cam}[/]"
        f"[dim] ╱ [/dim]"
        f"[bold bright_white]{axis}[/]"
        f"  [dim]({unit})[/dim]"
        f"          "
        f"[dim]step[/dim]  "
        f"[bold {step_color}]{step:.2f}[/]"
        f"  [dim italic]{step_label}  ·  s to cycle[/dim italic]"
    )
    lines.append("")

    # ── Column headers ─────────────────────────────────────────────────────────
    header = "  " + " " * LABEL_W
    for i, c in enumerate(CAMERAS):
        cc = CAM_COLORS[i]
        if i == cam_idx:
            header += f"[bold {cc} underline]{c:^{CELL_W}}[/]"
        else:
            header += f"[{cc} dim]{c:^{CELL_W}}[/]"
    lines.append(header)

    # ── Top separator ──────────────────────────────────────────────────────────
    lines.append(f"  [dim]{'─' * sep_len}[/dim]")

    # ── Data rows ──────────────────────────────────────────────────────────────
    for j, (ax, un) in enumerate(zip(AXES, UNITS)):
        is_active_row = j == axis_idx

        # Row marker + axis label — total 13 visible chars (= LABEL_W).
        if is_active_row:
            marker = "[bold bright_white]▶[/]"
            ax_str = f"[bold bright_white]{ax:>5}[/]"
        else:
            marker = " "
            ax_str = f"[dim]{ax:>5}[/dim]"
        # " ( cm)" / " (deg)" — always 6 visible chars via :>3 pad on unit.
        un_str = f"[dim] ({un:>3})[/dim]"

        row = f"  {marker} {ax_str}{un_str}"

        for i, c in enumerate(CAMERAS):
            val = state.values[c][ax]
            cc = CAM_COLORS[i]

            if i == cam_idx and j == axis_idx:
                # Active cell: bold + blue background for maximum contrast.
                row += f"[bold white on blue]  {val:>8.2f}  [/]"
            elif i == cam_idx:
                # Active column, non-selected row: tinted with camera colour.
                row += f"[{cc}]  {val:>8.2f}  [/]"
            elif j == axis_idx:
                # Active row, non-selected column: brighter than the rest.
                row += f"[white]  {val:>8.2f}  [/]"
            else:
                row += f"[dim]  {val:>8.2f}  [/dim]"

        lines.append(row)

    # ── Bottom separator ───────────────────────────────────────────────────────
    lines.append(f"  [dim]{'─' * sep_len}[/dim]")
    lines.append("")

    # ── Help bar ───────────────────────────────────────────────────────────────
    help_entries = [
        ("← →", "adjust"),
        ("↑ ↓", "axis"),
        ("c / C", "camera"),
        ("e", "enter val"),
        ("r", "reset"),
        ("q", "quit"),
    ]
    parts = [f"[bold white]{key}[/]  [dim]{act}[/dim]" for key, act in help_entries]
    lines.append("  " + "   [dim]·[/dim]   ".join(parts))

    return "\n".join(lines)


class TrimDisplay(Static):
    pass


class TrimTUI(App):
    DARK = True

    CSS = """
    TrimDisplay {
        width: 1fr;
        height: 1fr;
        padding: 2 3;
    }

    Input {
        dock: bottom;
        margin: 0 3 1 3;
        border: tall $primary-darken-2;
    }

    Input:focus {
        border: tall cyan;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("s", "cycle_step", "Step"),
        Binding("c", "next_camera", "Next Cam"),
        Binding("C", "prev_camera", "Prev Cam"),
        Binding("up", "prev_axis", "Axis ↑"),
        Binding("down", "next_axis", "Axis ↓"),
        Binding("right", "increment", "+"),
        Binding("left", "decrement", "−"),
        Binding("r", "reset", "Reset"),
        Binding("e", "edit", "Enter Value"),
        Binding("escape", "cancel_edit", "Cancel", show=False),
    ]

    def __init__(self):
        super().__init__()
        self.state = TrimState()
        self.cam_idx = 0
        self.axis_idx = 0
        self.step_idx = 1
        self.editing = False
        self._push_timer = None

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
        if self._push_timer is not None:
            self._push_timer.stop()
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        val = self.state.values[cam][axis]
        self._push_timer = self.set_timer(0.15, lambda: _set_ros_param(cam, axis, val))

    def on_input_submitted(self, event: Input.Submitted) -> None:
        raw = event.value.strip()
        try:
            new_val = float(raw)
        except ValueError:
            pass
        else:
            cam = CAMERAS[self.cam_idx]
            axis = AXES[self.axis_idx]
            self.state.values[cam][axis] = new_val
            self._push()
        self._close_edit()

    def _close_edit(self) -> None:
        self.editing = False
        try:
            inp = self.query_one(Input)
            inp.remove()
        except Exception:
            pass
        self._refresh()

    def action_edit(self) -> None:
        if self.editing:
            return
        self.editing = True
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        current = self.state.values[cam][axis]
        inp = Input(value=str(current), placeholder="Enter numeric value")
        self.mount(inp)
        inp.focus()

    def action_cancel_edit(self) -> None:
        if self.editing:
            self._close_edit()

    def action_cycle_step(self) -> None:
        if self.editing:
            return
        self.step_idx = (self.step_idx + 1) % len(STEPS)
        self._refresh()

    def action_next_camera(self) -> None:
        if self.editing:
            return
        self.cam_idx = (self.cam_idx + 1) % len(CAMERAS)
        self._refresh()

    def action_prev_camera(self) -> None:
        if self.editing:
            return
        self.cam_idx = (self.cam_idx - 1) % len(CAMERAS)
        self._refresh()

    def action_prev_axis(self) -> None:
        if self.editing:
            return
        self.axis_idx = (self.axis_idx - 1) % len(AXES)
        self._refresh()

    def action_next_axis(self) -> None:
        if self.editing:
            return
        self.axis_idx = (self.axis_idx + 1) % len(AXES)
        self._refresh()

    def action_increment(self) -> None:
        if self.editing:
            return
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        self.state.values[cam][axis] += STEPS[self.step_idx]
        self._push()
        self._refresh()

    def action_decrement(self) -> None:
        if self.editing:
            return
        cam = CAMERAS[self.cam_idx]
        axis = AXES[self.axis_idx]
        self.state.values[cam][axis] -= STEPS[self.step_idx]
        self._push()
        self._refresh()

    def action_reset(self) -> None:
        if self.editing:
            return
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
