"""Bare-metal Kinova sine sweep — no ROS, no arm_driver.

Connects directly to the arm via the Kortex API and runs a sine wave on
angular.x (tip/tilt) to measure true hardware response without any ROS
buffering or arm_driver state machine in the loop.

Usage (from any Python env with kortex_api installed):
    python3 kortex_sine_sweep.py

Output: per-sample CSV to stdout and a latency estimate from cross-correlation.
"""

import collections
import collections.abc
import math
import signal
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import correlate
from scipy.spatial.transform import Rotation as R

# Patch collections aliases removed in Python 3.10 (required by kortex_api 2.6.0)
collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence
collections.MutableSet = collections.abc.MutableSet
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence
collections.Callable = collections.abc.Callable

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient  # noqa: E402
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient  # noqa: E402
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient  # noqa: E402
from kortex_api.autogen.messages import Base_pb2, ControlConfig_pb2, Session_pb2  # noqa: E402
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions  # noqa: E402
from kortex_api.SessionManager import SessionManager  # noqa: E402
from kortex_api.TCPTransport import TCPTransport  # noqa: E402
from kortex_api.UDPTransport import UDPTransport  # noqa: E402

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
ARM_IP = "192.168.1.10"
CREDENTIALS = ("admin", "admin")

SINE_FREQ_HZ = 2  # Hz
SINE_AMP_DEGS = 100.3  # deg/s amplitude — matches ROS probe's 1.0 rad/s
COMMAND_HZ = 100  # Hz — command  loop rate
TEST_DURATION_S = 10.0  # seconds

# Joint speed limit applied to CARTESIAN_JOYSTICK mode before the test (deg/s)
JOINT_SPEED_LIMIT = 50.0

INTERP_HZ = 500  # interpolation grid for cross-correlation


# ---------------------------------------------------------------------------
# Connection helpers
# ---------------------------------------------------------------------------
def connect(ip, port, transport_cls, credentials):
    transport = transport_cls()
    router = RouterClient(transport, RouterClient.basicErrorCallback)
    transport.connect(ip, port)
    session_manager = None
    if credentials[0]:
        info = Session_pb2.CreateSessionInfo()
        info.username = credentials[0]
        info.password = credentials[1]
        info.session_inactivity_timeout = 10000
        info.connection_inactivity_timeout = 2000
        session_manager = SessionManager(router)
        session_manager.CreateSession(info)
    return transport, router, session_manager


def disconnect(transport, router, session_manager):
    if session_manager is not None:
        opts = RouterClientSendOptions()
        opts.timeout_ms = 1000
        session_manager.CloseSession(opts)
    transport.disconnect()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print(f"Connecting to arm at {ARM_IP}...")
    tcp_transport, tcp_router, tcp_session = connect(
        ARM_IP, 10000, TCPTransport, CREDENTIALS
    )
    udp_transport, udp_router, udp_session = connect(
        ARM_IP, 10001, UDPTransport, CREDENTIALS
    )

    base = BaseClient(tcp_router)
    base_cyclic = BaseCyclicClient(udp_router)
    control_config = ControlConfigClient(tcp_router)

    # Graceful shutdown on Ctrl-C
    def _stop(sig, frame):
        print("\nInterrupted — sending zero command and disconnecting...")
        _send_twist(base, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        _cleanup(
            base,
            tcp_transport,
            tcp_session,
            udp_transport,
            udp_session,
            tcp_router,
            udp_router,
        )
        sys.exit(0)

    signal.signal(signal.SIGINT, _stop)

    try:
        # Assert SINGLE_LEVEL_SERVOING
        servo_mode = Base_pb2.ServoingModeInformation()
        servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        base.SetServoingMode(servo_mode)
        print("Servoing mode: SINGLE_LEVEL_SERVOING")

        # Apply joint speed limit to CARTESIAN_JOYSTICK (governs SendTwistCommand)
        limits = ControlConfig_pb2.JointSpeedSoftLimits()
        limits.control_mode = ControlConfig_pb2.CARTESIAN_JOYSTICK
        limits.joint_speed_soft_limits.extend([JOINT_SPEED_LIMIT] * 7)
        control_config.SetJointSpeedSoftLimits(limits)
        print(f"CARTESIAN_JOYSTICK joint speed limit: {JOINT_SPEED_LIMIT} deg/s")

        # Brief settle
        _send_twist(base, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.5)

        print(f"\nRunning {SINE_FREQ_HZ} Hz sine sweep for {TEST_DURATION_S}s...")
        print(f"  amplitude: {SINE_AMP_DEGS} deg/s on angular.x (tip/tilt)")
        print("t_cmd,cmd_angular_x_degs,t_meas,meas_angular_x_degs")

        cmd_times, cmd_vals = [], []
        meas_times, meas_vals = [], []

        dt = 1.0 / COMMAND_HZ
        t0 = time.monotonic()
        n = 0

        while time.monotonic() - t0 < TEST_DURATION_S:
            t = time.monotonic() - t0
            cmd = SINE_AMP_DEGS * math.sin(2 * math.pi * SINE_FREQ_HZ * t)

            _send_twist(base, 0.0, 0.0, 0.0, cmd, 0.0, cmd)
            cmd_times.append(t)
            cmd_vals.append(cmd)

            # Read feedback immediately after command.
            # Kortex reports angular velocity in base frame (deg/s); rotate into
            # tool frame to match CARTESIAN_REFERENCE_FRAME_MIXED commands.
            fb = base_cyclic.RefreshFeedback()
            t_meas = time.monotonic() - t0
            tool_rot = np.deg2rad(
                [
                    fb.base.tool_pose_theta_x,
                    fb.base.tool_pose_theta_y,
                    fb.base.tool_pose_theta_z,
                ]
            )
            angular_base = np.array(
                [
                    fb.base.tool_twist_angular_x,
                    fb.base.tool_twist_angular_y,
                    fb.base.tool_twist_angular_z,
                ]
            )
            angular_tool = R.from_euler("xyz", tool_rot).inv().apply(angular_base)
            meas_angular_x = angular_tool[0]  # deg/s, tool frame
            meas_times.append(t_meas)
            meas_vals.append(meas_angular_x)

            print(f"{t:.4f},{cmd:.4f},{t_meas:.4f},{meas_angular_x:.4f}")

            # Pace loop
            n += 1
            next_t = t0 + n * dt
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

        _send_twist(base, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        print("\n# Sweep complete.")

        _analyze(cmd_times, cmd_vals, meas_times, meas_vals)

    finally:
        _cleanup(
            base,
            tcp_transport,
            tcp_session,
            udp_transport,
            udp_session,
            tcp_router,
            udp_router,
        )


def _send_twist(base, lx, ly, lz, ax, ay, az):
    cmd = Base_pb2.TwistCommand()
    cmd.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_MIXED
    cmd.duration = 0
    cmd.twist.linear_x = lx
    cmd.twist.linear_y = ly
    cmd.twist.linear_z = lz
    cmd.twist.angular_x = ax  # already in deg/s — no conversion needed here
    cmd.twist.angular_y = ay
    cmd.twist.angular_z = az
    base.SendTwistCommand(cmd)


def _analyze(cmd_times, cmd_vals, meas_times, meas_vals):
    t_end = min(cmd_times[-1], meas_times[-1])
    t_grid = np.arange(0, t_end, 1.0 / INTERP_HZ)

    cmd_interp = np.interp(t_grid, cmd_times, cmd_vals)
    meas_interp = np.interp(t_grid, meas_times, meas_vals)

    corr = correlate(
        meas_interp - meas_interp.mean(),
        cmd_interp - cmd_interp.mean(),
        mode="full",
    )
    lags = np.arange(-(len(cmd_interp) - 1), len(cmd_interp))
    positive = lags >= 0
    peak_lag = lags[positive][np.argmax(corr[positive])]
    delay_ms = peak_lag / INTERP_HZ * 1000.0

    norm = np.sqrt(
        np.sum((cmd_interp - cmd_interp.mean()) ** 2)
        * np.sum((meas_interp - meas_interp.mean()) ** 2)
    )
    peak_corr = float(np.max(corr)) / norm if norm > 0 else 0.0

    print("\n# --- Latency results ---")
    print(f"# Phase delay      : {delay_ms:.1f} ms")
    print(f"# Peak correlation : {peak_corr:.3f}  (1.0 = perfect tracking)")
    print(f"# Samples          : {len(cmd_times)} cmd / {len(meas_times)} meas")
    print(f"# Freq / Amplitude : {SINE_FREQ_HZ} Hz / {SINE_AMP_DEGS} deg/s")

    fig, axes = plt.subplots(2, 1, figsize=(12, 7))

    ax = axes[0]
    ax.plot(cmd_times, cmd_vals, label="Commanded (deg/s)", linewidth=1.5)
    ax.plot(
        meas_times,
        meas_vals,
        label="Measured tool_twist_angular_x (deg/s)",
        linewidth=1.5,
        alpha=0.8,
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular velocity (deg/s)")
    ax.set_title(
        f"Bare-metal Kortex — {SINE_FREQ_HZ} Hz sine, phase delay: {delay_ms:.1f} ms, corr: {peak_corr:.3f}"
    )
    ax.legend()
    ax.grid(True)

    ax = axes[1]
    lag_ms_axis = lags / INTERP_HZ * 1000.0
    ax.plot(lag_ms_axis, corr)
    ax.axvline(delay_ms, color="red", linestyle="--", label=f"Peak: {delay_ms:.1f} ms")
    ax.set_xlabel("Lag (ms)")
    ax.set_ylabel("Correlation")
    ax.set_title("Cross-correlation (positive lags only search)")
    ax.set_xlim(-500, 1000)
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig("bare_metal_kortex_sine_sweep.png")


def _cleanup(
    base, tcp_transport, tcp_session, udp_transport, udp_session, tcp_router, udp_router
):
    try:
        base.Stop()
    except Exception:
        pass
    disconnect(tcp_transport, tcp_router, tcp_session)
    disconnect(udp_transport, udp_router, udp_session)


if __name__ == "__main__":
    main()
