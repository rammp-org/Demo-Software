#!/usr/bin/env python3
"""
Button detector node for CMU door opener.

Purely a perception node — no arm motion.

Lifecycle:
  - On launch: initialize everything (subscribers, publishers, services, TF,
    OpenCV window, timer). YOLO model is NOT loaded yet (preserve GPU memory).
  - On /arm/door/detection/enable = True: load YOLO model, start detection loop.
    Every cycle publishes a ButtonInfo — real filtered data when pipeline
    succeeds, or a failure message (all -1) when any step fails.
  - On /arm/door/detection/enable = False: stop detection, unload YOLO, free GPU.
"""

import os
import time
import traceback
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
import tf2_ros
import tf2_geometry_msgs
import torch
from ultralytics import YOLO
from realsense2_camera_msgs.msg import Extrinsics
import open3d as o3d
from scipy.spatial.transform import Rotation

from cmu_door_opener_interfaces.msg import ButtonInfo
from cmu_door_opener.reachability_checker import ReachabilityChecker

# Default invalid pose — signals "no valid detection this cycle"
INVALID_POSE = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]


def depth_to_meters(depth_cv: np.ndarray) -> np.ndarray:
    if depth_cv.dtype == np.uint16:
        return depth_cv.astype(np.float32) * 0.001
    return depth_cv.astype(np.float32)


class LogThrottle:
    """Rate-limited logger wrapper."""

    def __init__(self, logger):
        self._last = {}
        self._logger = logger

    def debug(self, key: str, msg: str, period_s: float = 1.0):
        now = time.time()
        if now - self._last.get(key, 0.0) >= period_s:
            self._logger.debug(msg)
            self._last[key] = now

    def info(self, key: str, msg: str, period_s: float = 1.0):
        now = time.time()
        if now - self._last.get(key, 0.0) >= period_s:
            self._logger.info(msg)
            self._last[key] = now

    def warn(self, key: str, msg: str, period_s: float = 1.0):
        now = time.time()
        if now - self._last.get(key, 0.0) >= period_s:
            self._logger.warn(msg)
            self._last[key] = now


class PoseFilter:
    """Exponential moving average filter for 3D pose (xyz + rpy).

    Only reports stable when at least `min_samples` have been collected
    AND the spread (max pairwise distance) of the last `min_samples`
    raw xyz readings is within `max_spread_m`.
    """

    def __init__(self, alpha: float = 0.3, min_samples: int = 3,
                 max_spread_m: float = 0.02):
        self.alpha = alpha
        self.min_samples = min_samples
        self.max_spread_m = max_spread_m
        self._count = 0
        self._xyz = np.zeros(3, dtype=np.float64)
        self._rpy = np.zeros(3, dtype=np.float64)
        self._recent_xyz: list[np.ndarray] = []

    def update(self, xyz: np.ndarray, rpy: np.ndarray):
        if self._count == 0:
            self._xyz = xyz.astype(np.float64)
            self._rpy = rpy.astype(np.float64)
        else:
            self._xyz = self.alpha * xyz + (1.0 - self.alpha) * self._xyz
            self._rpy = self.alpha * rpy + (1.0 - self.alpha) * self._rpy
        self._count += 1
        self._recent_xyz.append(xyz.astype(np.float64).copy())
        if len(self._recent_xyz) > self.min_samples:
            self._recent_xyz.pop(0)

    @property
    def spread(self) -> float:
        """Max pairwise distance among the last min_samples xyz readings."""
        if len(self._recent_xyz) < self.min_samples:
            return float('inf')
        pts = np.array(self._recent_xyz)
        max_dist = 0.0
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                d = float(np.linalg.norm(pts[i] - pts[j]))
                if d > max_dist:
                    max_dist = d
        return max_dist

    @property
    def is_stable(self) -> bool:
        return (self._count >= self.min_samples
                and self.spread <= self.max_spread_m)

    @property
    def xyz(self) -> np.ndarray:
        return self._xyz.astype(np.float64)

    @property
    def rpy(self) -> np.ndarray:
        return self._rpy.astype(np.float64)

    def reset(self):
        self._count = 0
        self._xyz = np.zeros(3, dtype=np.float64)
        self._rpy = np.zeros(3, dtype=np.float64)
        self._recent_xyz.clear()


class ButtonPressVisionNode(Node):
    def __init__(self):
        super().__init__("button_press_vision_node")
        self.lt = LogThrottle(self.get_logger())

        # ---- Parameters ----
        self.declare_parameter("rgb_topic", "/camera/wrist/color/image_raw")
        self.declare_parameter("color_info_topic", "/camera/wrist/color/camera_info")
        self.declare_parameter("depth_topic", "/camera/wrist/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic", "/camera/wrist/depth/camera_info")
        self.declare_parameter(
            "extrinsics_topic", "/camera/wrist/extrinsics/depth_to_color"
        )

        self.declare_parameter("color_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter(
            "yolo_model",
            os.path.join(os.path.dirname(__file__), "button_yolo_weights.pt"),
        )
        self.declare_parameter("detection_confidence", 0.5)
        self.declare_parameter("target_class", "")  # '' => accept any class
        self.declare_parameter("yolo_use_fp16", True)
        self.declare_parameter("yolo_imgsz", 640)

        self.declare_parameter("depth_stride", 4)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 3.00)
        self.declare_parameter("min_projected_points", 10)
        self.declare_parameter("mask_core_min_dist_px", 6.0)
        self.declare_parameter("mask_core_min_points", 40)
        self.declare_parameter("bbox_center_radius_px", 18)
        self.declare_parameter("normal_radius", 0.05)
        self.declare_parameter("normal_max_nn", 30)
        self.declare_parameter("press_offset", 0.0)

        self.declare_parameter(
            "fixed_pose_quat", [0.5, 0.5, 0.5, 0.5]
        )  # [x, y, z, w], fallback only

        # filter
        self.declare_parameter("filter_alpha", 0.3)
        self.declare_parameter("filter_min_samples", 3)
        self.declare_parameter("filter_max_spread_m", 0.02)

        # visualization
        self.declare_parameter("show_opencv_windows", False)
        self.declare_parameter("window_scale", 1.0)

        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("tf_timeout_s", 0.5)

        # ---- State ----
        self.bridge = CvBridge()
        self.color_info = None
        self.depth_info = None
        self.depth_to_color_extr = None
        self.color_frame_id = None
        self._warned_color_frame_override = False

        self.latest_rgb = None
        self.latest_depth_m = None

        self.last_rgb_t = None
        self.last_depth_t = None

        # Detection enabled flag — off by default
        self._detection_enabled = False

        # Pose filter
        self._pose_filter = PoseFilter(
            alpha=float(self.get_parameter("filter_alpha").value),
            min_samples=int(self.get_parameter("filter_min_samples").value),
            max_spread_m=float(self.get_parameter("filter_max_spread_m").value),
        )

        # Button ID counter
        self._button_id = 0

        # fps
        self._fps_t0 = time.time()
        self._fps_n = 0
        self._fps = 0.0

        # last YOLO outputs for display
        self.last_det_count = None
        self.last_best_bbox = None
        self.last_best_mask = None
        self.last_best_conf = None
        self.last_best_label = None

        # last computed points
        self.last_centroid_cam = None
        self.last_centroid_base = None
        self.last_centroid_uv = None
        self._dbg_projected_uvs = []
        self._dbg_within_radius_uvs = []
        self._dbg_bbox_center = None
        self._dbg_radius = 0
        quat_param = np.array(
            self.get_parameter("fixed_pose_quat").value, dtype=np.float32
        )
        if quat_param.shape[0] != 4:
            raise ValueError("fixed_pose_quat must have 4 values [x, y, z, w]")
        qn = np.linalg.norm(quat_param)
        if qn < 1e-9:
            raise ValueError("fixed_pose_quat must be non-zero")
        self.fixed_pose_quat = (quat_param / qn).astype(np.float32)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Publishers ----
        self.button_info_pub = self.create_publisher(
            ButtonInfo, "/arm/door/button_info", 10
        )
        self.debug_cam_pt_pub = self.create_publisher(
            PointStamped, "/button/debug_point_camera", 10
        )
        self.debug_base_pt_pub = self.create_publisher(
            PointStamped, "/button/debug_point_base", 10
        )
        self.normal_marker_pub = self.create_publisher(
            Marker, "/button/normal_marker", 10
        )

        # ---- Service: detection enable/disable ----
        self.create_service(
            SetBool, "/arm/door/detection/enable", self._srv_detection_enable
        )

        # ---- Subscribers (always active — data is buffered so pipeline starts instantly) ----
        self.create_subscription(
            CameraInfo,
            self.get_parameter("color_info_topic").value,
            self.cb_color_info,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_topic").value,
            self.cb_depth_info,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").value,
            self.cb_rgb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self.cb_depth,
            qos_profile_sensor_data,
        )

        qos_extr = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Extrinsics,
            self.get_parameter("extrinsics_topic").value,
            self.cb_extrinsics,
            qos_extr,
        )

        # ---- YOLO: NOT loaded at init — loaded on detection enable to preserve GPU memory ----
        self.yolo = None
        self.yolo_device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.yolo_half = bool(
            self.get_parameter("yolo_use_fp16").value
        ) and self.yolo_device.startswith("cuda")
        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)

        # ---- OpenCV window ----
        self.show_windows = bool(self.get_parameter("show_opencv_windows").value)
        self.window_scale = float(self.get_parameter("window_scale").value)
        if self.show_windows:
            cv2.namedWindow("button_viz", cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                "button_viz",
                int(1400 * self.window_scale),
                int(700 * self.window_scale),
            )
            self.get_logger().info("OpenCV window enabled (press 'q' to quit).")

        # ---- IK reachability checker (Kortex-backed, async) ----
        self._reachability_checker = ReachabilityChecker(self)

        rate = float(self.get_parameter("process_rate_hz").value)
        period = 1.0 / max(rate, 0.1)
        self.create_timer(period, self.process_once)

        self.get_logger().info(
            "YOLO model is NOT loaded yet (will load on detection enable)."
        )
        self.get_logger().info(
            "Detection is OFF. Call /arm/door/detection/enable to start."
        )
        self.get_logger().info("ButtonPressVisionNode started.")

    # ---- YOLO load / unload ----
    def _load_yolo(self):
        """Load YOLO model to GPU/CPU. Called when detection is enabled."""
        if self.yolo is not None:
            return  # already loaded

        model_path = self.get_parameter("yolo_model").value
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        try:
            self.yolo = YOLO(model_path)
            self.yolo.to(self.yolo_device)
            self.get_logger().info(
                f"YOLO loaded on {self.yolo_device} fp16={self.yolo_half} imgsz={self.yolo_imgsz}"
            )
            if hasattr(self.yolo, "names"):
                self.get_logger().info(f"YOLO classes: {self.yolo.names}")
        except Exception as e:
            if self.yolo_device.startswith("cuda"):
                self.get_logger().warn(
                    f"YOLO load failed on {self.yolo_device}: {e}. Retrying on CPU."
                )
                try:
                    self.yolo_device = "cpu"
                    self.yolo_half = False
                    self.yolo = YOLO(model_path)
                    self.yolo.to(self.yolo_device)
                    self.get_logger().info("YOLO loaded on CPU (fallback)")
                except Exception as e_cpu:
                    self.get_logger().fatal(
                        f"Failed to load YOLO model '{model_path}' on CPU fallback: {e_cpu}"
                    )
                    self.yolo = None
                    raise
            else:
                self.get_logger().fatal(
                    f"Failed to load YOLO model '{model_path}': {e}"
                )
                self.yolo = None
                raise

    def _unload_yolo(self):
        """Unload YOLO model and free GPU memory. Called when detection is disabled."""
        if self.yolo is None:
            return
        self.get_logger().info("Unloading YOLO model to free GPU memory")
        del self.yolo
        self.yolo = None
        try:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:
            pass

    # ---- Service callback ----
    def _srv_detection_enable(self, request, response):
        if request.data:
            # Enable detection: load model, reset filter, start pipeline
            try:
                self._load_yolo()
            except Exception as e:
                response.success = False
                response.message = f"Failed to load YOLO model: {e}"
                return response
            self._pose_filter.reset()
            self._detection_enabled = True
            self.get_logger().info("Detection ENABLED — YOLO loaded, pipeline running")
            response.message = "Detection pipeline started, YOLO model loaded"
        else:
            # Disable detection: stop pipeline, unload model, free GPU
            self._detection_enabled = False
            self._pose_filter.reset()
            self._unload_yolo()
            self.get_logger().info("Detection DISABLED — YOLO unloaded, GPU freed")
            response.message = "Detection pipeline stopped, YOLO model unloaded"
        response.success = True
        return response

    # ---- Callbacks ----

    def cb_color_info(self, msg: CameraInfo):
        self.color_info = msg
        if msg.header.frame_id:
            self.color_frame_id = msg.header.frame_id

    def cb_depth_info(self, msg: CameraInfo):
        self.depth_info = msg

    def cb_extrinsics(self, msg: Extrinsics):
        self.depth_to_color_extr = msg
        self.lt.debug(
            "extr",
            f"[EXTR] received depth->color extrinsics t="
            f"[{msg.translation[0]:+.4f},{msg.translation[1]:+.4f},{msg.translation[2]:+.4f}]",
            period_s=9999,
        )

    def cb_rgb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_rgb_t = time.time()
        except Exception as e:
            self.lt.debug("rgb_err", f"[RGB] convert failed: {e}", 2.0)

    def cb_depth(self, msg: Image):
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_m = depth_to_meters(depth_cv)
            self.last_depth_t = time.time()
        except Exception as e:
            self.lt.debug("depth_err", f"[DEPTH] convert failed: {e}", 2.0)

    # ---- YOLO ----
    def run_yolo(self, rgb_bgr: np.ndarray):
        self.last_best_bbox = None
        self.last_best_mask = None
        self.last_best_conf = None
        self.last_best_label = None

        if self.yolo is None:
            self.last_det_count = 0
            self.lt.debug("yolo_none", "[YOLO] model not loaded", 2.0)
            return None

        conf_th = float(self.get_parameter("detection_confidence").value)
        target = self.get_parameter("target_class").value.strip().lower()

        self.lt.debug("yolo_run", "[YOLO] running inference...", 0.8)
        try:
            results = self.yolo(
                rgb_bgr,
                conf=conf_th,
                verbose=False,
                device=self.yolo_device,
                half=self.yolo_half,
                imgsz=self.yolo_imgsz,
            )
        except Exception as e:
            err_text = str(e).lower()
            cuda_runtime_failure = self.yolo_device.startswith("cuda") and (
                "cuda" in err_text
                or "nvml" in err_text
                or "cudacachingallocator" in err_text
            )
            if not cuda_runtime_failure:
                self.lt.debug("yolo_infer_err", f"[YOLO] inference failed: {e}", 1.0)
                self.last_det_count = 0
                return None

            self.lt.debug(
                "yolo_cpu_fallback",
                f"[YOLO] CUDA inference failed ({e}); switching to CPU",
                0.5,
            )
            try:
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            except Exception:
                pass

            try:
                self.yolo_device = "cpu"
                self.yolo_half = False
                self.yolo.to(self.yolo_device)
                results = self.yolo(
                    rgb_bgr,
                    conf=conf_th,
                    verbose=False,
                    device=self.yolo_device,
                    half=self.yolo_half,
                    imgsz=self.yolo_imgsz,
                )
                self.lt.debug(
                    "yolo_cpu_ok", "[YOLO] CPU fallback inference succeeded", 1.0
                )
            except Exception as e_cpu:
                self.lt.debug(
                    "yolo_cpu_fail",
                    f"[YOLO] CPU fallback inference failed: {e_cpu}",
                    1.0,
                )
                self.last_det_count = 0
                return None

        if not results or results[0].boxes is None:
            self.last_det_count = 0
            self.lt.debug("yolo_nores", "[YOLO] no results object", 1.0)
            return None

        r0 = results[0]
        boxes = r0.boxes
        n_det = len(boxes) if boxes is not None else 0
        self.last_det_count = n_det
        self.lt.debug("yolo_det", f"[YOLO] detections={n_det}", 0.8)

        if n_det == 0 or r0.masks is None or r0.masks.data is None:
            return None

        masks = r0.masks.data  # (N,Hm,Wm)

        best_i = None
        best_conf = -1.0
        best_label = None

        for i in range(n_det):
            b = boxes[i]
            c = float(b.conf[0])
            cls_id = int(b.cls[0])
            label = (
                str(self.yolo.names.get(cls_id, cls_id)).lower()
                if hasattr(self.yolo, "names")
                else str(cls_id)
            )

            if target:
                if not ((target in label) or (label in target)):
                    continue

            if c > best_conf:
                best_conf = c
                best_i = i
                best_label = label

        if best_i is None:
            for i in range(n_det):
                c = float(boxes[i].conf[0])
                if c > best_conf:
                    best_conf = c
                    best_i = i
                    cls_id = int(boxes[i].cls[0])
                    best_label = (
                        str(self.yolo.names.get(cls_id, cls_id)).lower()
                        if hasattr(self.yolo, "names")
                        else str(cls_id)
                    )

        mask_i = masks[best_i].detach().cpu().numpy()
        h, w = rgb_bgr.shape[:2]
        mask_resized = (
            cv2.resize(
                mask_i.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR
            )
            > 0.5
        )
        bbox = boxes[best_i].xyxy[0].detach().cpu().numpy().astype(np.float32)

        self.last_best_bbox = bbox
        self.last_best_mask = mask_resized
        self.last_best_conf = float(best_conf)
        self.last_best_label = best_label

        mask_px = int(np.count_nonzero(mask_resized))
        self.lt.debug(
            "yolo_sel",
            f"[YOLO] selected idx={best_i} conf={best_conf:.2f} mask_px={mask_px}",
            0.8,
        )

        return mask_resized

    # ---- 3D from depth->color projection ----
    def compute_3d_from_bbox_center_depth_extrinsics(
        self, bbox_xyxy: np.ndarray, depth_m: np.ndarray
    ):
        if (
            self.color_info is None
            or self.depth_info is None
            or self.depth_to_color_extr is None
        ):
            self.lt.debug(
                "wait_meta", "[WAIT] missing color_info/depth_info/extrinsics", 1.0
            )
            return None

        Hc, Wc = int(self.color_info.height), int(self.color_info.width)
        Hd, Wd = depth_m.shape[:2]

        Kc = self.color_info.k
        fx_c, fy_c, cx_c, cy_c = float(Kc[0]), float(Kc[4]), float(Kc[2]), float(Kc[5])
        Kd = self.depth_info.k
        fx_d, fy_d, cx_d, cy_d = float(Kd[0]), float(Kd[4]), float(Kd[2]), float(Kd[5])

        R = np.array(self.depth_to_color_extr.rotation, dtype=np.float32).reshape(3, 3)
        t = np.array(self.depth_to_color_extr.translation, dtype=np.float32).reshape(3)

        x1, y1, x2, y2 = bbox_xyxy.astype(np.float32)
        uc = float(np.clip(0.5 * (x1 + x2), 0.0, max(0.0, Wc - 1.0)))
        vc = float(np.clip(0.5 * (y1 + y2), 0.0, max(0.0, Hc - 1.0)))
        rad = float(self.get_parameter("bbox_center_radius_px").value)

        stride = max(1, int(self.get_parameter("depth_stride").value))
        min_z = float(self.get_parameter("min_depth_m").value)
        max_z = float(self.get_parameter("max_depth_m").value)
        min_pts = int(self.get_parameter("min_projected_points").value)

        points_color = []
        nearest_d2 = 1e30
        dbg_total_pixels = 0
        dbg_valid_depth = 0
        dbg_in_color_fov = 0
        dbg_nearest_d = None
        dbg_projected_uvs = []  # all points that project into color FOV
        dbg_within_radius_uvs = []  # points within bbox_center_radius_px

        # Approximate depth image ROI from bbox + margin to avoid scanning entire image.
        # The depth and color cameras have different intrinsics, so we apply a generous
        # margin (scale bbox by ratio of focal lengths + extra padding).
        fx_ratio = fx_d / fx_c if fx_c > 0 else 1.0
        fy_ratio = fy_d / fy_c if fy_c > 0 else 1.0
        margin = 40  # extra pixels of padding
        d_x1 = max(0, int((x1 - cx_c) * fx_ratio + cx_d) - margin)
        d_x2 = min(Wd, int((x2 - cx_c) * fx_ratio + cx_d) + margin)
        d_y1 = max(0, int((y1 - cy_c) * fy_ratio + cy_d) - margin)
        d_y2 = min(Hd, int((y2 - cy_c) * fy_ratio + cy_d) + margin)

        self.get_logger().debug(
            f"[3D] bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}] center=({uc:.1f},{vc:.1f}) radius={rad}px"
        )
        self.get_logger().debug(
            f"[3D] depth ROI=[{d_x1},{d_y1},{d_x2},{d_y2}] (from bbox + margin)"
        )
        self.get_logger().debug(
            f"[3D] depth image: {Wd}x{Hd}, color image: {Wc}x{Hc}, stride={stride}"
        )
        self.get_logger().debug(
            f"[3D] depth range: [{min_z}, {max_z}]m, min_pts={min_pts}"
        )
        self.get_logger().debug(
            f"[3D] color K: fx={fx_c:.1f} fy={fy_c:.1f} cx={cx_c:.1f} cy={cy_c:.1f}"
        )
        self.get_logger().debug(
            f"[3D] depth K: fx={fx_d:.1f} fy={fy_d:.1f} cx={cx_d:.1f} cy={cy_d:.1f}"
        )

        for v in range(d_y1, d_y2, stride):
            z_row = depth_m[v, :]
            for u in range(d_x1, d_x2, stride):
                dbg_total_pixels += 1
                z = float(z_row[u])
                if not (min_z < z < max_z):
                    continue
                dbg_valid_depth += 1

                Xd = (u - cx_d) * z / fx_d
                Yd = (v - cy_d) * z / fy_d
                Pc = R @ np.array([Xd, Yd, z], dtype=np.float32) + t

                Zc = float(Pc[2])
                if Zc <= 0.0:
                    continue
                up = fx_c * (float(Pc[0]) / Zc) + cx_c
                vp = fy_c * (float(Pc[1]) / Zc) + cy_c
                if up < 0.0 or up >= Wc or vp < 0.0 or vp >= Hc:
                    continue
                dbg_in_color_fov += 1
                dbg_projected_uvs.append((int(round(up)), int(round(vp))))

                d2 = (up - uc) * (up - uc) + (vp - vc) * (vp - vc)
                if d2 < nearest_d2:
                    nearest_d2 = d2
                    dbg_nearest_d = d2**0.5
                if d2 <= rad * rad:
                    points_color.append(Pc)
                    dbg_within_radius_uvs.append((int(round(up)), int(round(vp))))

        self.get_logger().debug(
            f"[3D] scanned {dbg_total_pixels} pixels, {dbg_valid_depth} valid depth, "
            f"{dbg_in_color_fov} in color FOV, {len(points_color)} within radius"
        )
        if dbg_nearest_d is not None:
            self.get_logger().debug(
                f"[3D] nearest projected point distance to bbox center: {dbg_nearest_d:.1f}px (radius={rad}px)"
            )
        else:
            self.get_logger().debug("[3D] NO projected points found at all!")

        # Store for visualization
        self._dbg_projected_uvs = dbg_projected_uvs
        self._dbg_within_radius_uvs = dbg_within_radius_uvs
        self._dbg_bbox_center = (int(round(uc)), int(round(vc)))
        self._dbg_radius = int(round(rad))

        if len(points_color) >= min_pts:
            pts = np.stack(points_color, axis=0)
            centroid = np.median(pts, axis=0).astype(np.float32)
            self.get_logger().debug(
                f"[3D] SUCCESS: {len(points_color)} points, centroid=[{centroid[0]:+.4f},{centroid[1]:+.4f},{centroid[2]:+.4f}]"
            )
            return centroid, pts, (int(round(uc)), int(round(vc)))

        self.get_logger().debug(
            f"[3D] NOT ENOUGH POINTS: {len(points_color)} < {min_pts} — skipping this frame"
        )

        self.lt.debug("bbox_no_depth", "[3D] no valid depth near bbox center", 0.5)
        return None

    def estimate_surface_normal(self, points_3d: np.ndarray):
        self.get_logger().debug(
            f"[NORMAL] input points shape={points_3d.shape}, dtype={points_3d.dtype}"
        )
        if len(points_3d) < 10:
            self.get_logger().debug(
                f"[NORMAL] FAIL: too few points ({len(points_3d)} < 10)"
            )
            return None

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_3d)
            self.get_logger().debug(
                f"[NORMAL] point cloud created, {len(pcd.points)} points"
            )

            pts_np = np.asarray(pcd.points)
            self.get_logger().debug(
                f"[NORMAL] point cloud bounds: "
                f"x=[{pts_np[:, 0].min():.4f}, {pts_np[:, 0].max():.4f}] "
                f"y=[{pts_np[:, 1].min():.4f}, {pts_np[:, 1].max():.4f}] "
                f"z=[{pts_np[:, 2].min():.4f}, {pts_np[:, 2].max():.4f}]"
            )

            radius = float(self.get_parameter("normal_radius").value)
            max_nn = int(self.get_parameter("normal_max_nn").value)
            self.get_logger().debug(
                f"[NORMAL] params: radius={radius}, max_nn={max_nn}"
            )

            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=radius, max_nn=max_nn
                )
            )
            self.get_logger().debug(
                f"[NORMAL] normals estimated, count={len(pcd.normals)}"
            )

            pcd.orient_normals_towards_camera_location(
                camera_location=np.array([0, 0, 0])
            )

            normals = np.asarray(pcd.normals)
            self.get_logger().debug(
                f"[NORMAL] normals stats: "
                f"mean=[{normals[:, 0].mean():+.4f},{normals[:, 1].mean():+.4f},{normals[:, 2].mean():+.4f}] "
                f"std=[{normals[:, 0].std():.4f},{normals[:, 1].std():.4f},{normals[:, 2].std():.4f}]"
            )

            avg_normal = np.mean(normals, axis=0)
            norm_mag = np.linalg.norm(avg_normal)
            self.get_logger().debug(
                f"[NORMAL] avg normal before normalize: [{avg_normal[0]:+.4f},{avg_normal[1]:+.4f},{avg_normal[2]:+.4f}], mag={norm_mag:.6f}"
            )
            avg_normal /= norm_mag

            if avg_normal[2] > 0:
                self.get_logger().debug(
                    f"[NORMAL] flipping normal (z was positive: {avg_normal[2]:+.4f})"
                )
                avg_normal *= -1

            self.get_logger().debug(
                f"[NORMAL] RESULT: [{avg_normal[0]:+.4f},{avg_normal[1]:+.4f},{avg_normal[2]:+.4f}]"
            )
            return avg_normal.astype(np.float32)

        except Exception as e:
            self.get_logger().error(
                f"[NORMAL] EXCEPTION: {type(e).__name__}: {e}\n{traceback.format_exc()}"
            )
            return None

    def transform_point_to_base(self, point_cam_xyz: np.ndarray):
        param_cam_frame = str(self.get_parameter("color_optical_frame").value)
        cam_frame = self.color_frame_id if self.color_frame_id else param_cam_frame
        base_frame = self.get_parameter("base_frame").value
        tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        if (cam_frame != param_cam_frame) and (not self._warned_color_frame_override):
            self.lt.debug(
                "cam_frame_override",
                f"[TF] using CameraInfo frame '{cam_frame}' instead of param '{param_cam_frame}'",
                9999.0,
            )
            self._warned_color_frame_override = True

        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = cam_frame
        ps.point.x = float(point_cam_xyz[0])
        ps.point.y = float(point_cam_xyz[1])
        ps.point.z = float(point_cam_xyz[2])

        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame,
                cam_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout),
            )
            pb = tf2_geometry_msgs.do_transform_point(ps, tf)
            return ps, pb
        except Exception as e2:
            self.lt.debug(
                "tf_fail_fb",
                f"[TF] point transform {cam_frame}->{base_frame} failed: {e2}",
                1.0,
            )
            return None, None

    def transform_vector_to_base(self, vector_cam: np.ndarray):
        param_cam_frame = str(self.get_parameter("color_optical_frame").value)
        cam_frame = self.color_frame_id if self.color_frame_id else param_cam_frame
        base_frame = self.get_parameter("base_frame").value
        tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        self.get_logger().debug(
            f"[TF VEC] transforming vector [{vector_cam[0]:+.4f},{vector_cam[1]:+.4f},{vector_cam[2]:+.4f}] "
            f"from '{cam_frame}' to '{base_frame}'"
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame,
                cam_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout),
            )
            q = tf.transform.rotation
            self.get_logger().debug(
                f"[TF VEC] TF rotation quat=[{q.x:+.4f},{q.y:+.4f},{q.z:+.4f},{q.w:+.4f}]"
            )
            rot_base_cam = Rotation.from_quat([q.x, q.y, q.z, q.w])
            vector_base = rot_base_cam.apply(vector_cam)
            self.get_logger().debug(
                f"[TF VEC] result=[{vector_base[0]:+.4f},{vector_base[1]:+.4f},{vector_base[2]:+.4f}]"
            )
            return vector_base
        except Exception as e2:
            self.get_logger().debug(f"[TF VEC] FAILED: {type(e2).__name__}: {e2}")
            return None

    def publish_normal_marker(self, centroid_base: np.ndarray, normal_base: np.ndarray):
        base_frame = self.get_parameter("base_frame").value

        marker = Marker()
        marker.header.frame_id = base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "button_normal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.0

        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        start = Point()
        start.x = float(centroid_base[0])
        start.y = float(centroid_base[1])
        start.z = float(centroid_base[2])

        arrow_length = 0.1
        end = Point()
        end.x = float(centroid_base[0] + normal_base[0] * arrow_length)
        end.y = float(centroid_base[1] + normal_base[1] * arrow_length)
        end.z = float(centroid_base[2] + normal_base[2] * arrow_length)

        marker.points.append(start)
        marker.points.append(end)

        self.normal_marker_pub.publish(marker)

    def project_cam_point_to_color_pixel(self, point_cam: np.ndarray):
        if self.color_info is None:
            return None
        z = float(point_cam[2])
        if z <= 1e-6:
            return None
        fx = float(self.color_info.k[0])
        fy = float(self.color_info.k[4])
        cx = float(self.color_info.k[2])
        cy = float(self.color_info.k[5])
        u = int(round(fx * (float(point_cam[0]) / z) + cx))
        v = int(round(fy * (float(point_cam[1]) / z) + cy))
        return u, v

    # ---- Visualization ----
    def draw_viz(self):
        if not self.show_windows:
            return

        if self.latest_rgb is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                "Waiting for RGB...",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3,
            )
            cv2.imshow("button_viz", blank)
            cv2.waitKey(1)
            return

        rgb = self.latest_rgb
        left = rgb.copy()
        right = rgb.copy()

        now = time.time()
        age = (now - self.last_rgb_t) if self.last_rgb_t else 999.0

        self._fps_n += 1
        dt = now - self._fps_t0
        if dt >= 1.0:
            self._fps = self._fps_n / dt
            self._fps_n = 0
            self._fps_t0 = now

        status_text = "ON" if self._detection_enabled else "OFF"
        yolo_text = "loaded" if self.yolo is not None else "not loaded"
        cv2.putText(
            left,
            f"Det={status_text} YOLO={yolo_text} fps={self._fps:.1f} age={age:.2f}s",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        dets = self.last_det_count if self.last_det_count is not None else -1
        cv2.putText(
            right,
            f"YOLO det={dets}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        if self.last_best_bbox is not None:
            x1, y1, x2, y2 = self.last_best_bbox.astype(int)
            cv2.rectangle(right, (x1, y1), (x2, y2), (0, 255, 0), 2)
            lab = self.last_best_label if self.last_best_label else "obj"
            cf = self.last_best_conf if self.last_best_conf is not None else 0.0
            cv2.putText(
                right,
                f"{lab} {cf:.2f}",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )

        if self.last_best_mask is not None:
            mask = self.last_best_mask
            overlay = right.copy()
            overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array([0, 255, 0])).astype(
                np.uint8
            )
            right = overlay
            cnts, _ = cv2.findContours(
                mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(right, cnts, -1, (0, 255, 255), 2)
        else:
            cv2.putText(
                right,
                "NO DETECTION",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3,
            )

        if self.last_centroid_cam is not None:
            cx, cy, cz = self.last_centroid_cam
            cv2.putText(
                right,
                f"cam xyz=({cx:+.3f},{cy:+.3f},{cz:+.3f})",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
            )
        if self.last_centroid_base is not None:
            bx, by, bz = self.last_centroid_base
            cv2.putText(
                right,
                f"base xyz=({bx:+.3f},{by:+.3f},{bz:+.3f})",
                (10, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
            )
        if self.last_centroid_uv is not None:
            u, v = self.last_centroid_uv
            h, w = right.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(right, (u, v), 8, (0, 0, 255), -1)
                cv2.circle(right, (u, v), 16, (255, 255, 255), 2)
                cv2.putText(
                    right,
                    f"centroid px=({u},{v})",
                    (10, 190),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 255, 255),
                    2,
                )

        # Draw projected depth points
        if hasattr(self, "_dbg_projected_uvs") and self._dbg_projected_uvs:
            # All projected points in blue (small)
            for pu, pv in self._dbg_projected_uvs:
                if 0 <= pu < right.shape[1] and 0 <= pv < right.shape[0]:
                    cv2.circle(right, (pu, pv), 1, (255, 100, 0), -1)
            # Points within radius in green (larger)
            for pu, pv in self._dbg_within_radius_uvs:
                if 0 <= pu < right.shape[1] and 0 <= pv < right.shape[0]:
                    cv2.circle(right, (pu, pv), 3, (0, 255, 0), -1)
            # Draw the search radius circle
            if hasattr(self, "_dbg_bbox_center") and self._dbg_bbox_center is not None:
                cv2.circle(
                    right, self._dbg_bbox_center, self._dbg_radius, (0, 255, 255), 1
                )
            n_proj = len(self._dbg_projected_uvs)
            n_in = len(self._dbg_within_radius_uvs)
            cv2.putText(
                right,
                f"depth pts: {n_in}/{n_proj} in radius",
                (10, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

        combo = np.hstack([left, right])
        cv2.imshow("button_viz", combo)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Quit key pressed.")
            rclpy.shutdown()

    # ---- Build and publish ButtonInfo ----
    #
    # Three possible states:
    #   1. NO BUTTON FOUND — mask/bbox/confidence/pose/is_pressable all default/invalid
    #   2. BUTTON DETECTED, NOT PRESSABLE — mask/bbox/confidence are real,
    #      but pose = all -1, is_pressable = false (depth failed, TF failed,
    #      too far, IK unsolvable, filter warming up, etc.)
    #   3. BUTTON DETECTED AND PRESSABLE — everything is real/valid
    #

    def _publish_no_button(self):
        """State 1: No button found. All fields default/invalid."""
        msg = ButtonInfo()
        msg.id = self._button_id
        msg.segmentation_mask = self.bridge.cv2_to_imgmsg(
            np.zeros((1, 1), dtype=np.uint8), encoding="mono8"
        )
        msg.bounding_box = [0, 0, 0, 0]
        msg.confidence = 0.0
        msg.pose_xyzrpy = INVALID_POSE[:]
        msg.is_pressable = False
        self.button_info_pub.publish(msg)

    def _publish_detected_not_pressable(self, mask, bbox, confidence):
        """State 2: Button detected but not pressable. Has mask/bbox/confidence
        but pose is invalid and is_pressable is false."""
        msg = ButtonInfo()
        msg.id = self._button_id
        mask_uint8 = (mask.astype(np.uint8)) * 255
        msg.segmentation_mask = self.bridge.cv2_to_imgmsg(mask_uint8, encoding="mono8")
        msg.bounding_box = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
        msg.confidence = float(confidence)
        msg.pose_xyzrpy = INVALID_POSE[:]
        msg.is_pressable = False
        self.button_info_pub.publish(msg)

    def _publish_detected_pressable(
        self, filtered_xyz, filtered_rpy, mask, bbox, confidence, is_pressable
    ):
        """State 3: Button detected, pose valid. is_pressable set by reachability check."""
        msg = ButtonInfo()
        msg.id = self._button_id
        mask_uint8 = (mask.astype(np.uint8)) * 255
        msg.segmentation_mask = self.bridge.cv2_to_imgmsg(mask_uint8, encoding="mono8")
        msg.bounding_box = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
        msg.confidence = float(confidence)
        msg.pose_xyzrpy = [
            float(filtered_xyz[0]),
            float(filtered_xyz[1]),
            float(filtered_xyz[2]),
            float(filtered_rpy[0]),
            float(filtered_rpy[1]),
            float(filtered_rpy[2]),
        ]
        msg.is_pressable = is_pressable
        self.button_info_pub.publish(msg)

    # ---- Main loop ----
    def process_once(self):
        self.draw_viz()

        if not self._detection_enabled:
            return

        # ---------- Every cycle below here publishes a ButtonInfo ----------
        #
        # State 1 (no button):       camera not ready or YOLO finds nothing
        # State 2 (detected, not pressable): YOLO found button but depth/TF/filter failed
        # State 3 (detected, pressable):     full pipeline succeeded, filtered pose ready

        # Check camera prerequisites — no data means no button
        if self.latest_rgb is None:
            self.lt.debug("wait_rgb", "[WAIT] rgb not received yet", 1.0)
            self._publish_no_button()
            return
        if self.latest_depth_m is None:
            self.lt.debug("wait_depth", "[WAIT] depth not received yet", 1.0)
            self._publish_no_button()
            return
        if (
            self.color_info is None
            or self.depth_info is None
            or self.depth_to_color_extr is None
        ):
            self.lt.debug("wait_meta", "[WAIT] missing camera_info or extrinsics", 1.0)
            self._publish_no_button()
            return

        now = time.time()
        if self.last_rgb_t is not None and (now - self.last_rgb_t) > 1.0:
            self.lt.warn(
                "stale_rgb", f"RGB seems stale: age={now - self.last_rgb_t:.2f}s", 1.0
            )

        # 1) YOLO — no detection means no button (state 1)
        mask = self.run_yolo(self.latest_rgb)
        if mask is None or self.last_best_bbox is None:
            self.lt.debug("no_seg", "[YOLO] no valid segmentation", 1.0)
            self._publish_no_button()
            return

        # From here we have a detection (mask, bbox, confidence are valid).
        # If any downstream step fails, we publish state 2 (detected, not pressable).
        cur_mask = mask
        cur_bbox = self.last_best_bbox
        cur_conf = self.last_best_conf if self.last_best_conf is not None else 0.0

        # 2) 3D centroid (cam) from bbox center
        centroid_res = self.compute_3d_from_bbox_center_depth_extrinsics(
            cur_bbox, self.latest_depth_m
        )
        if centroid_res is None:
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return
        centroid_cam, points_cam, centroid_uv = centroid_res
        self.last_centroid_cam = centroid_cam
        self.last_centroid_uv = centroid_uv
        cam_frame = (
            self.color_frame_id
            if self.color_frame_id
            else str(self.get_parameter("color_optical_frame").value)
        )
        ps_cam = PointStamped()
        ps_cam.header.stamp = self.get_clock().now().to_msg()
        ps_cam.header.frame_id = cam_frame
        ps_cam.point.x = float(centroid_cam[0])
        ps_cam.point.y = float(centroid_cam[1])
        ps_cam.point.z = float(centroid_cam[2])
        self.debug_cam_pt_pub.publish(ps_cam)

        # 3) TF to base
        ps_cam, ps_base = self.transform_point_to_base(centroid_cam)
        if ps_base is None:
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return

        self.debug_base_pt_pub.publish(ps_base)
        centroid_base = np.array(
            [ps_base.point.x, ps_base.point.y, ps_base.point.z], dtype=np.float32
        )
        self.last_centroid_base = centroid_base

        # 4) Estimate surface normal in camera frame, transform to base
        # No fallback — if any step fails, skip this frame.
        self.get_logger().debug(
            f"[PIPELINE] step 4: estimating normal from {len(points_cam)} cam points"
        )
        normal_cam = self.estimate_surface_normal(points_cam)
        if normal_cam is None:
            self.get_logger().debug("[PIPELINE] normal_cam is None — skipping frame")
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return

        self.get_logger().debug(
            f"[PIPELINE] normal_cam=[{normal_cam[0]:+.4f},{normal_cam[1]:+.4f},{normal_cam[2]:+.4f}]"
        )
        normal_base = self.transform_vector_to_base(normal_cam)
        if normal_base is None:
            self.get_logger().debug(
                "[PIPELINE] transform_vector_to_base returned None — skipping frame"
            )
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return

        nrm = np.linalg.norm(normal_base)
        self.get_logger().debug(
            f"[PIPELINE] normal_base=[{normal_base[0]:+.4f},{normal_base[1]:+.4f},{normal_base[2]:+.4f}], norm={nrm:.6f}"
        )
        if nrm <= 1e-9:
            self.get_logger().debug(
                f"[PIPELINE] normal_base norm too small ({nrm}) — skipping frame"
            )
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return

        normal_base = (normal_base / nrm).astype(np.float32)
        # Tool Z = -normal (point into button)
        z_axis = -normal_base
        x_axis = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(x_axis, z_axis)) > 0.9:
            x_axis = np.array([0.0, 1.0, 0.0])
            self.get_logger().debug(
                "[PIPELINE] z_axis near x — switching initial x_axis to [0,1,0]"
            )
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
        self.get_logger().debug(
            f"[PIPELINE] rot_matrix:\n"
            f"  x_axis=[{x_axis[0]:+.4f},{x_axis[1]:+.4f},{x_axis[2]:+.4f}]\n"
            f"  y_axis=[{y_axis[0]:+.4f},{y_axis[1]:+.4f},{y_axis[2]:+.4f}]\n"
            f"  z_axis=[{z_axis[0]:+.4f},{z_axis[1]:+.4f},{z_axis[2]:+.4f}]"
        )
        rot = Rotation.from_matrix(rot_matrix)
        quat = rot.as_quat()  # [x,y,z,w]
        rpy = np.array(rot.as_euler("xyz"), dtype=np.float64)
        self.get_logger().debug(
            f"[PIPELINE] quat=[{quat[0]:+.4f},{quat[1]:+.4f},{quat[2]:+.4f},{quat[3]:+.4f}]"
        )
        self.get_logger().debug(
            f"[PIPELINE] rpy=[{rpy[0]:+.4f},{rpy[1]:+.4f},{rpy[2]:+.4f}] (radians)"
        )
        self.get_logger().debug(
            f"[PIPELINE] rpy=[{np.degrees(rpy[0]):+.1f},{np.degrees(rpy[1]):+.1f},{np.degrees(rpy[2]):+.1f}] (degrees)"
        )

        # Publish marker
        self.publish_normal_marker(centroid_base, normal_base)

        self.get_logger().debug(
            f"[PIPELINE] final RPY=[{rpy[0]:+.4f},{rpy[1]:+.4f},{rpy[2]:+.4f}] centroid_base=[{centroid_base[0]:+.4f},{centroid_base[1]:+.4f},{centroid_base[2]:+.4f}]"
        )

        # 5) Filter
        self._pose_filter.update(centroid_base.astype(np.float64), rpy)
        self.get_logger().debug(
            f"[FILTER] count={self._pose_filter._count}, spread={self._pose_filter.spread:.4f}m, stable={self._pose_filter.is_stable}"
        )
        self.get_logger().debug(
            f"[FILTER] filtered_xyz=[{self._pose_filter.xyz[0]:+.4f},{self._pose_filter.xyz[1]:+.4f},{self._pose_filter.xyz[2]:+.4f}]"
        )
        self.get_logger().debug(
            f"[FILTER] filtered_rpy=[{self._pose_filter.rpy[0]:+.4f},{self._pose_filter.rpy[1]:+.4f},{self._pose_filter.rpy[2]:+.4f}]"
        )

        if not self._pose_filter.is_stable:
            self.get_logger().debug(
                "[PIPELINE] filter not stable yet — publishing not_pressable"
            )
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return

        # 6) Reachability check — can the arm reach both the button surface
        #    (approach) and the full push overshoot?  The checker tests both
        #    poses to match what ButtonPushController actually commands.
        filtered_xyz = self._pose_filter.xyz
        filtered_rpy = self._pose_filter.rpy
        rot = Rotation.from_euler("xyz", filtered_rpy)
        check_quat = rot.as_quat()  # [x,y,z,w]
        self._reachability_checker.check_async(filtered_xyz, check_quat)
        is_pressable = self._reachability_checker.is_reachable

        self.get_logger().debug(
            f"[PIPELINE] reachability={is_pressable} — publishing ButtonInfo"
        )
        self._publish_detected_pressable(
            filtered_xyz=filtered_xyz,
            filtered_rpy=filtered_rpy,
            mask=cur_mask,
            bbox=cur_bbox,
            confidence=cur_conf,
            is_pressable=is_pressable,
        )

        xyz = self._pose_filter.xyz
        final_rpy = self._pose_filter.rpy
        self.get_logger().debug(
            f"[PUB] xyz=[{xyz[0]:+.4f},{xyz[1]:+.4f},{xyz[2]:+.4f}] "
            f"rpy=[{final_rpy[0]:+.4f},{final_rpy[1]:+.4f},{final_rpy[2]:+.4f}] "
            f"rpy_deg=[{np.degrees(final_rpy[0]):+.1f},{np.degrees(final_rpy[1]):+.1f},{np.degrees(final_rpy[2]):+.1f}]"
        )


def main():
    rclpy.init()
    node = ButtonPressVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
