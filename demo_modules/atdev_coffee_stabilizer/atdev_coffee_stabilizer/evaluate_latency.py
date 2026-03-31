"""Arm angular twist latency probe — sine wave cross-correlation method.

Publishes a sine wave on /arm/atdev/twist (angular.x) via a ROS timer and
cross-correlates it against /arm/ee/velocity to measure end-to-end phase delay.

Publishing runs on a timer so the executor can service velocity callbacks
immediately as they arrive, independent of publish rate.

Usage:
    ros2 run atdev_coffee_stabilizer evaluate_latency
"""

import math
import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from arm_interfaces.srv import SetMode, SetSpeedPreset
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

# --- Tunable parameters ---
SINE_FREQ_HZ = 0.5  # Frequency of the test sine wave (Hz)
SINE_AMP = 0.2  # Amplitude (rad/s)
PUBLISH_HZ = 300  # Rate at which the publish timer fires
WARM_UP_S = 3.0  # Seconds to run sine before recording (avoids startup transients)
TEST_DURATION_S = 10.0  # Seconds of data to collect
INTERP_HZ = 500  # Common grid for cross-correlation interpolation


class LatencyProbe(Node):
    def __init__(self):
        super().__init__("latency_probe")
        self._twist_pub = self.create_publisher(Twist, "/arm/atdev/twist", 10)
        self.create_subscription(
            TwistStamped, "/arm/ee/velocity", self._on_velocity, 10
        )

        self._cmd_times: list[float] = []
        self._cmd_vals: list[float] = []
        self._meas_times: list[float] = []
        self._meas_vals: list[float] = []

        self._t_sine_start: float = 0.0  # when the sine started (for phase continuity)
        self._t0: float = 0.0  # when data collection started
        self._recording = False
        self._publish_timer = None
        self._warm_up_timer = None
        self._finish_timer = None

    # -------------------------------------------------------------------------

    def _on_velocity(self, msg: TwistStamped):
        if not self._recording:
            return
        self._meas_times.append(time.monotonic() - self._t0)
        self._meas_vals.append(msg.twist.angular.x)

    def _publish_tick(self):
        """Timer callback — publish sine command, record if collecting."""

        if self._recording:
            t_sine = time.monotonic() - self._t0
            cmd_val = SINE_AMP * math.sin(2 * math.pi * SINE_FREQ_HZ * t_sine)
            msg = Twist()
            msg.angular.x = cmd_val
            self._twist_pub.publish(msg)

            self._cmd_times.append(time.monotonic() - self._t0)
            self._cmd_vals.append(cmd_val)
        else:
            # Just publish zero during warm-up
            self._twist_pub.publish(Twist())

    def _begin_collection(self):
        """One-shot timer callback — fired after warm-up."""
        self._t0 = time.monotonic()
        self._recording = True
        self.get_logger().info("Warm-up done, collecting data...")
        self._finish_timer = self.create_timer(TEST_DURATION_S, self._finish)

        self._warm_up_timer.cancel()  # stop warm-up timer (no longer needed)

    def _finish(self):
        """One-shot timer callback — fired after TEST_DURATION_S."""
        self._recording = False
        self._publish_timer.cancel()
        self._twist_pub.publish(Twist())
        self._analyze()
        self._finish_timer.cancel()
        rclpy.shutdown()

    # -------------------------------------------------------------------------

    def _call_service(self, client, request, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{name} service not available")
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            msg = getattr(result, "message", "no response") if result else "timeout"
            self.get_logger().error(f"{name} failed: {msg}")
            return False
        return True

    def setup(self) -> bool:
        mode_client = self.create_client(SetMode, "/arm/set_mode")
        mode_req = SetMode.Request()
        mode_req.mode = SetMode.Request.MODE_CUP_STABILIZE
        if not self._call_service(mode_client, mode_req, "/arm/set_mode"):
            return False
        self.get_logger().info("Mode: CUP_STABILIZE")

        speed_client = self.create_client(SetSpeedPreset, "/arm/set_speed_preset")
        speed_req = SetSpeedPreset.Request()
        speed_req.preset = SetSpeedPreset.Request.PRESET_HIGH
        if not self._call_service(speed_client, speed_req, "/arm/set_speed_preset"):
            return False
        self.get_logger().info(
            "Speed preset: HIGH (50 deg/s per joint, CARTESIAN_JOYSTICK)"
        )
        return True

    def start(self):
        """Start the publish timer and schedule warm-up end."""
        self.get_logger().info(
            f"Warming up for {WARM_UP_S}s, then collecting {TEST_DURATION_S}s "
            f"of {SINE_FREQ_HZ} Hz sine at {SINE_AMP} rad/s..."
        )
        self._t_sine_start = time.monotonic()
        self._publish_timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_tick)
        self._warm_up_timer = self.create_timer(WARM_UP_S, self._begin_collection)

    # -------------------------------------------------------------------------

    def _analyze(self):
        if len(self._cmd_times) < 10 or len(self._meas_times) < 10:
            self.get_logger().error("Not enough data to analyze")
            return

        self.get_logger().info(
            f"Collected {len(self._cmd_times)} cmd samples, "
            f"{len(self._meas_times)} velocity samples"
        )

        t_end = min(self._cmd_times[-1], self._meas_times[-1])
        t_grid = np.arange(0, t_end, 1.0 / INTERP_HZ)

        cmd_interp = np.interp(t_grid, self._cmd_times, self._cmd_vals)
        meas_interp = np.interp(t_grid, self._meas_times, self._meas_vals)

        corr = np.correlate(
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
        peak_corr = float(np.max(corr[positive])) / norm if norm > 0 else 0.0

        self.get_logger().info("--- Results ---")
        self.get_logger().info(f"  Phase delay      : {delay_ms:.1f} ms")
        self.get_logger().info(
            f"  Peak correlation : {peak_corr:.3f}  (1.0 = perfect sine tracking)"
        )
        self.get_logger().info(
            f"  Test frequency   : {SINE_FREQ_HZ} Hz  |  Amplitude: {SINE_AMP} rad/s"
        )

        fig, axes = plt.subplots(2, 1, figsize=(12, 7))

        ax = axes[0]
        ax.plot(
            self._cmd_times, self._cmd_vals, label="Commanded (rad/s)", linewidth=1.5
        )
        ax.plot(
            self._meas_times,
            self._meas_vals,
            label="Measured /arm/ee/velocity angular.x (deg/s)",
            linewidth=1.5,
            alpha=0.8,
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angular velocity")
        ax.set_title(
            f"ROS probe — {SINE_FREQ_HZ} Hz sine  |  delay: {delay_ms:.1f} ms  |  corr: {peak_corr:.3f}"
        )
        ax.legend()
        ax.grid(True)

        ax = axes[1]
        lag_ms_axis = lags / INTERP_HZ * 1000.0
        ax.plot(lag_ms_axis, corr)
        ax.axvline(
            delay_ms, color="red", linestyle="--", label=f"Peak: {delay_ms:.1f} ms"
        )
        ax.set_xlabel("Lag (ms)")
        ax.set_ylabel("Correlation")
        ax.set_title("Cross-correlation (positive lags only)")
        ax.set_xlim(-200, 1000)
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.savefig("ros_probe_latency.png")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = LatencyProbe()

    if not node.setup():
        node.destroy_node()
        rclpy.shutdown()
        return

    node.start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
