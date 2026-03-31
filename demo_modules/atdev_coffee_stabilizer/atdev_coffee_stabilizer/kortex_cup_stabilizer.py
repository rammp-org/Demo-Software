"""Cup stabilizer — aligns tool_y with gravity using raw accelerometer.

Usage:
    python3 kortex_cup_stabilizer.py
"""

import collections
import collections.abc
import csv
import math
import signal
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

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

ARM_IP = "192.168.1.10"
CREDENTIALS = ("admin", "admin")
CONTROL_HZ = 40
JOINT_SPEED_LIMIT = 80.0
CALIBRATION_S = 3.0
WARMUP_S = 3.0
KP = 8.0
KD = 1.0


HOME_TIMEOUT_S = 60.0


def go_home(base):
    event = threading.Event()

    def on_action_notification(notification):
        if notification.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
            event.set()

    handle = base.OnNotificationActionTopic(
        on_action_notification, Base_pb2.NotificationOptions()
    )

    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = next(
        (a.handle for a in action_list.action_list if a.name == "Home"), None
    )
    if action_handle is None:
        base.Unsubscribe(handle)
        sys.exit("ERROR: 'Home' action not found on arm")

    event.clear()
    base.ExecuteActionFromReference(action_handle)
    event.wait(HOME_TIMEOUT_S)
    base.Unsubscribe(handle)


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

    log_file = None
    log_path = None

    def _stop(sig, frame):
        print("\nStopping...")
        _send_twist(base, 0, 0, 0, 0, 0, 0)
        base.Stop()
        if log_file is not None:
            log_file.close()
            print(f"Log saved to {log_path}")
        disconnect(tcp_transport, tcp_router, tcp_session)
        disconnect(udp_transport, udp_router, udp_session)
        sys.exit(0)

    signal.signal(signal.SIGINT, _stop)

    servo_mode = Base_pb2.ServoingModeInformation()
    servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(servo_mode)

    limits = ControlConfig_pb2.JointSpeedSoftLimits()
    limits.control_mode = ControlConfig_pb2.CARTESIAN_JOYSTICK
    limits.joint_speed_soft_limits.extend([JOINT_SPEED_LIMIT] * 7)
    control_config.SetJointSpeedSoftLimits(limits)

    # print("Moving to Home position...")
    # go_home(base)

    # Calibrate gyro offset
    print(f"Calibrating gyro for {CALIBRATION_S}s — keep arm still...")
    gyro_samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < CALIBRATION_S:
        fb = base_cyclic.RefreshFeedback()
        gyro_samples.append(
            [
                fb.base.imu_angular_velocity_x,
                fb.base.imu_angular_velocity_y,
                fb.base.imu_angular_velocity_z,
            ]
        )
        time.sleep(0.01)
    gyro_offset = np.mean(gyro_samples, axis=0)
    print(f"Gyro offset: {gyro_offset}")

    print(f"Warming up for {WARMUP_S}s...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < WARMUP_S:
        base_cyclic.RefreshFeedback()
        time.sleep(1.0 / CONTROL_HZ)

    log_path = f"cup_stabilizer_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(
        [
            "t_s",
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x_rads",
            "gyro_y_rads",
            "gyro_z_rads",
            "tool_y_x",
            "tool_y_y",
            "tool_y_z",
            "error_x",
            "error_y",
            "error_z",
            "omega_x_cmd",
            "omega_y_cmd",
        ]
    )
    print(f"Logging to {log_path}")

    print("Running. Ctrl-C to stop.")
    dt = 1.0 / CONTROL_HZ
    t0 = time.monotonic()
    n = 0

    while True:
        fb = base_cyclic.RefreshFeedback()

        # Gravity direction from raw accel (normalized)
        accel = np.array(
            [
                fb.base.imu_acceleration_x,
                fb.base.imu_acceleration_y,
                fb.base.imu_acceleration_z,
            ]
        )
        g_hat = accel / np.linalg.norm(accel)
        up = -g_hat

        gyro_raw = np.array(
            [
                fb.base.imu_angular_velocity_x,
                fb.base.imu_angular_velocity_y,
                fb.base.imu_angular_velocity_z,
            ]
        )
        gyro_rads = np.deg2rad(gyro_raw - gyro_offset)

        ee_euler_deg = [
            fb.base.tool_pose_theta_x,
            fb.base.tool_pose_theta_y,
            fb.base.tool_pose_theta_z,
        ]
        rot = R.from_euler("xyz", np.deg2rad(ee_euler_deg))
        tool_y = rot.as_matrix()[:, 1]

        error = tool_y - up

        print(f"tool_orientation: {v(tool_y)}\tg_up: {v(up)}\terror {v(error)}")

        omega_x = KP * error[1] + KD * gyro_rads[0]
        omega_y = -(KP * error[0] + KD * gyro_rads[1])

        _send_twist(base, 0, 0, 0, omega_x, omega_y, 0)

        t = time.monotonic() - t0
        log_writer.writerow(
            [
                f"{t:.4f}",
                *[f"{x:.5f}" for x in accel],
                *[f"{x:.5f}" for x in gyro_rads],
                *[f"{x:.5f}" for x in tool_y],
                *[f"{x:.5f}" for x in error],
                f"{omega_x:.5f}",
                f"{omega_y:.5f}",
            ]
        )

        n += 1
        sleep_s = t0 + n * dt - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)


def v(vec):
    return f"[{vec[0]:+.2f}, {vec[1]:+.2f}, {vec[2]:+.2f}]"


def _send_twist(base, lx, ly, lz, ax, ay, az):
    cmd = Base_pb2.TwistCommand()
    cmd.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    cmd.duration = 0
    cmd.twist.linear_x = lx
    cmd.twist.linear_y = ly
    cmd.twist.linear_z = lz
    cmd.twist.angular_x = math.degrees(ax)
    cmd.twist.angular_y = math.degrees(ay)
    cmd.twist.angular_z = math.degrees(az)
    base.SendTwistCommand(cmd)


if __name__ == "__main__":
    main()
