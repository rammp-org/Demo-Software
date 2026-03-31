"""Bare-metal Kortex cup stabilizer — no ROS, no arm_driver.

Reads IMU data from the Kortex base, filters it (TODO), and commands an EE
twist to counteract platform disturbances.

Usage (from any Python env with kortex_api installed):
    python3 kortex_cup_stabilizer.py
"""

import collections
import collections.abc
import time

import numpy as np

# Patch collections aliases removed in Python 3.10 (required by kortex_api 2.6.0)
collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence
collections.MutableSet = collections.abc.MutableSet
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence
collections.Callable = collections.abc.Callable

from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient  # noqa: E402
from kortex_api.autogen.messages import Session_pb2  # noqa: E402
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions  # noqa: E402
from kortex_api.SessionManager import SessionManager  # noqa: E402
from kortex_api.TCPTransport import TCPTransport  # noqa: E402
from kortex_api.UDPTransport import UDPTransport  # noqa: E402

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
ARM_IP = "192.168.1.10"
CREDENTIALS = ("admin", "admin")

CONTROL_HZ = 40  # Hz — main loop rate

# Joint speed limit applied to CARTESIAN_JOYSTICK mode (deg/s)
JOINT_SPEED_LIMIT = 80.0

# Madgwick filter parameters
MADGWICK_GAIN = 0.4  # filter gain (higher = faster convergence, more accel noise)
GYRO_RANGE = 2000  # deg/s — used for gyroscope saturation detection

# Controller parameters
KP = 5.0  # proportional gain (rad/s per unit error) — tune from here
WARMUP_S = 3.0  # seconds to run filter before commanding twists


# ---------------------------------------------------------------------------
# Connection helpers (same pattern as kortex_sine_sweep.py)
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
# IMU reading
# ---------------------------------------------------------------------------
def read_imu(base_cyclic):
    """Return raw IMU data from the Kortex base.

    Returns:
        dict with keys:
            accel_xyz  — linear acceleration [x, y, z] in m/s²
            gyro_xyz   — angular velocity [x, y, z] in deg/s
    """
    fb = base_cyclic.RefreshFeedback()
    return {
        "accel_xyz": [
            fb.base.imu_acceleration_x,
            fb.base.imu_acceleration_y,
            fb.base.imu_acceleration_z,
        ],
        "gyro_xyz": [
            fb.base.imu_angular_velocity_x,
            fb.base.imu_angular_velocity_y,
            fb.base.imu_angular_velocity_z,
        ],
    }


def update_madgwick(ahrs, offset, imu, dt):
    """Run one Madgwick filter step and return the gravity vector in base frame.

    Args:
        ahrs: imufusion.Ahrs object (stateful).
        imu:  dict from read_imu().
        dt:   time step in seconds.

    Returns:
        g_base — np.ndarray [gx, gy, gz] in m/s², gravity expressed in the
        sensor (robot base) frame. At rest on a flat surface this should be
        approximately [0, 0, -9.81] if the IMU Z-axis points up, or
        [0, 0, 9.81] if it points down — depends on physical mounting.
    """
    gyro = np.array(imu["gyro_xyz"])
    # Convert to deg/s
    gyro = offset.update(np.array(imu["gyro_xyz"]))  # deg/s, bias corrected

    accel = np.array(imu["accel_xyz"]) / 9.81  # m/s² → g
    ahrs.update_no_magnetometer(gyro, accel, dt)
    return np.array(ahrs.gravity) * 9.81  # g → m/s², in base frame


# ---------------------------------------------------------------------------
def main():
    print(f"Connecting to arm at {ARM_IP}...")
    tcp_transport, tcp_router, tcp_session = connect(
        ARM_IP, 10000, TCPTransport, CREDENTIALS
    )
    udp_transport, udp_router, udp_session = connect(
        ARM_IP, 10001, UDPTransport, CREDENTIALS
    )

    base_cyclic = BaseCyclicClient(udp_router)

    # ahrs = imufusion.Ahrs()
    # ahrs.settings = imufusion.Settings(
    #     imufusion.CONVENTION_NWU,
    #     MADGWICK_GAIN,
    #     GYRO_RANGE,
    #     0.0,  # acceleration rejection (degrees)
    #     0.0,  # magnetic rejection (degrees, unused — no magnetometer)
    #     5 * CONTROL_HZ,  # recovery trigger period (unsigned int, samples)
    # )
    # offset = imufusion.Offset(CONTROL_HZ)

    # Measure gyro offset with a few seconds of stationary data
    print(f"Calibrating gyro offset with {WARMUP_S} seconds of stationary data...")
    t_start = time.monotonic()

    gyro_samples = []
    while time.monotonic() - t_start < WARMUP_S:
        imu_data = read_imu(base_cyclic)
        gyro_samples.append(imu_data["gyro_xyz"])
        time.sleep(0.01)
    gyro_samples = np.array(gyro_samples)
    gyro_offset = np.mean(gyro_samples, axis=0)
    print(f"Gyro offset (deg/s): {gyro_offset}")

    while True:
        imu_data = read_imu(
            base_cyclic
        )  # prime the pump (sometimes first reading is garbage)

        # Print IMU data
        accel = [f"{x:.3f}" for x in imu_data["accel_xyz"]]

        # Correct gyro data
        gyro_corrected = np.array(imu_data["gyro_xyz"]) - gyro_offset
        gyro_corrected_str = [f"{x:.3f}" for x in gyro_corrected]

        print(
            f"IMU accel (m/s²): [{', '.join(accel)}]  |  "
            f"gyro (deg/s): [{', '.join(gyro_corrected_str)}]  |  "
        )

        time.sleep(0.005)


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
