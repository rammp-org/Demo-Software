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
import numpy as np
import cv2
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, Point
import tf2_ros
import tf2_geometry_msgs
import torch
from ultralytics import YOLO
from realsense2_camera_msgs.msg import Extrinsics
import open3d as o3d
from scipy.spatial.transform import Rotation

from cmu_door_opener_interfaces.msg import ButtonInfo

# Default invalid pose — signals "no valid detection this cycle"
INVALID_POSE = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]


def depth_to_meters(depth_cv: np.ndarray) -> np.ndarray:
    if depth_cv.dtype == np.uint16:
        return depth_cv.astype(np.float32) * 0.001
    return depth_cv.astype(np.float32)


class PrintThrottle:
    def __init__(self):
        self._last = {}

    def p(self, key: str, msg: str, period_s: float = 1.0):
        now = time.time()
        last = self._last.get(key, 0.0)
        if now - last >= period_s:
            print(msg, flush=True)
            self._last[key] = now


class PoseFilter:
    """Simple exponential moving average filter for 3D pose (xyz + rpy)."""

    def __init__(self, alpha: float = 0.3, min_samples: int = 3):
        self.alpha = alpha
        self.min_samples = min_samples
        self._count = 0
        self._xyz = np.zeros(3, dtype=np.float64)
        self._rpy = np.zeros(3, dtype=np.float64)

    def update(self, xyz: np.ndarray, rpy: np.ndarray):
        if self._count == 0:
            self._xyz = xyz.astype(np.float64)
            self._rpy = rpy.astype(np.float64)
        else:
            self._xyz = self.alpha * xyz + (1.0 - self.alpha) * self._xyz
            self._rpy = self.alpha * rpy + (1.0 - self.alpha) * self._rpy
        self._count += 1

    @property
    def is_stable(self) -> bool:
        return self._count >= self.min_samples

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


class ButtonPressVisionNode(Node):
    def __init__(self):
        super().__init__('button_press_vision_node')
        self.pt = PrintThrottle()

        # ---- Parameters ----
        self.declare_parameter('rgb_topic', '/camera/wrist/color/image_raw')
        self.declare_parameter('color_info_topic', '/camera/wrist/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/wrist/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/camera/wrist/depth/camera_info')
        self.declare_parameter('extrinsics_topic', '/camera/wrist/extrinsics/depth_to_color')

        self.declare_parameter('color_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('yolo_model', os.path.join(os.path.dirname(__file__), 'button_yolo_weights.pt'))
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('target_class', '')  # '' => accept any class
        self.declare_parameter('yolo_use_fp16', True)
        self.declare_parameter('yolo_imgsz', 640)

        self.declare_parameter('depth_stride', 4)
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 3.00)
        self.declare_parameter('min_projected_points', 30)
        self.declare_parameter('mask_core_min_dist_px', 6.0)
        self.declare_parameter('mask_core_min_points', 40)
        self.declare_parameter('bbox_center_radius_px', 18)
        self.declare_parameter('normal_radius', 0.05)
        self.declare_parameter('normal_max_nn', 30)
        self.declare_parameter('press_offset', 0.0)

        self.declare_parameter('fixed_pose_quat', [0.5, 0.5, 0.5, 0.5])  # [x, y, z, w], fallback only

        # filter
        self.declare_parameter('filter_alpha', 0.3)
        self.declare_parameter('filter_min_samples', 3)

        # visualization
        self.declare_parameter('show_opencv_windows', True)
        self.declare_parameter('window_scale', 1.0)

        self.declare_parameter('process_rate_hz', 5.0)
        self.declare_parameter('tf_timeout_s', 0.5)

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
            alpha=float(self.get_parameter('filter_alpha').value),
            min_samples=int(self.get_parameter('filter_min_samples').value),
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
        quat_param = np.array(self.get_parameter('fixed_pose_quat').value, dtype=np.float32)
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
        self.button_info_pub = self.create_publisher(ButtonInfo, '/arm/door/button_info', 10)
        self.debug_cam_pt_pub = self.create_publisher(PointStamped, '/button/debug_point_camera', 10)
        self.debug_base_pt_pub = self.create_publisher(PointStamped, '/button/debug_point_base', 10)
        self.normal_marker_pub = self.create_publisher(Marker, '/button/normal_marker', 10)

        # ---- Service: detection enable/disable ----
        self.create_service(SetBool, '/arm/door/detection/enable', self._srv_detection_enable)

        # ---- Subscribers (always active — data is buffered so pipeline starts instantly) ----
        self.create_subscription(CameraInfo, self.get_parameter('color_info_topic').value, self.cb_color_info, 10)
        self.create_subscription(CameraInfo, self.get_parameter('depth_info_topic').value, self.cb_depth_info, 10)
        self.create_subscription(Image, self.get_parameter('rgb_topic').value, self.cb_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.cb_depth, qos_profile_sensor_data)

        qos_extr = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        self.create_subscription(
            Extrinsics,
            self.get_parameter('extrinsics_topic').value,
            self.cb_extrinsics,
            qos_extr
        )

        # ---- YOLO: NOT loaded at init — loaded on detection enable to preserve GPU memory ----
        self.yolo = None
        self.yolo_device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.yolo_half = bool(self.get_parameter('yolo_use_fp16').value) and self.yolo_device.startswith('cuda')
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)

        # ---- OpenCV window ----
        self.show_windows = bool(self.get_parameter('show_opencv_windows').value)
        self.window_scale = float(self.get_parameter('window_scale').value)
        if self.show_windows:
            cv2.namedWindow("button_viz", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("button_viz", int(1400 * self.window_scale), int(700 * self.window_scale))
            print("[INIT] OpenCV window enabled (press 'q' to quit).", flush=True)

        rate = float(self.get_parameter('process_rate_hz').value)
        period = 1.0 / max(rate, 0.1)
        self.create_timer(period, self.process_once)

        print("[INIT] YOLO model is NOT loaded yet (will load on detection enable).", flush=True)
        print("[INIT] Detection is OFF. Call /arm/door/detection/enable to start.", flush=True)
        print("[INIT] ButtonPressVisionNode started.", flush=True)

    # ---- YOLO load / unload ----
    def _load_yolo(self):
        """Load YOLO model to GPU/CPU. Called when detection is enabled."""
        if self.yolo is not None:
            return  # already loaded

        model_path = self.get_parameter('yolo_model').value
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        try:
            self.yolo = YOLO(model_path)
            self.yolo.to(self.yolo_device)
            self.get_logger().info(
                f'YOLO loaded on {self.yolo_device} fp16={self.yolo_half} imgsz={self.yolo_imgsz}'
            )
            if hasattr(self.yolo, 'names'):
                self.get_logger().info(f'YOLO classes: {self.yolo.names}')
        except Exception as e:
            if self.yolo_device.startswith('cuda'):
                self.get_logger().warn(
                    f'YOLO load failed on {self.yolo_device}: {e}. Retrying on CPU.'
                )
                try:
                    self.yolo_device = 'cpu'
                    self.yolo_half = False
                    self.yolo = YOLO(model_path)
                    self.yolo.to(self.yolo_device)
                    self.get_logger().info(f'YOLO loaded on CPU (fallback)')
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
        self.get_logger().info('Unloading YOLO model to free GPU memory')
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
                response.message = f'Failed to load YOLO model: {e}'
                return response
            self._pose_filter.reset()
            self._detection_enabled = True
            self.get_logger().info('Detection ENABLED — YOLO loaded, pipeline running')
            response.message = 'Detection pipeline started, YOLO model loaded'
        else:
            # Disable detection: stop pipeline, unload model, free GPU
            self._detection_enabled = False
            self._pose_filter.reset()
            self._unload_yolo()
            self.get_logger().info('Detection DISABLED — YOLO unloaded, GPU freed')
            response.message = 'Detection pipeline stopped, YOLO model unloaded'
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
        self.pt.p("extr", f"[EXTR] received depth->color extrinsics t="
                         f"[{msg.translation[0]:+.4f},{msg.translation[1]:+.4f},{msg.translation[2]:+.4f}]",
                  period_s=9999)

    def cb_rgb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_rgb_t = time.time()
        except Exception as e:
            self.pt.p("rgb_err", f"[RGB] convert failed: {e}", 2.0)

    def cb_depth(self, msg: Image):
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_m = depth_to_meters(depth_cv)
            self.last_depth_t = time.time()
        except Exception as e:
            self.pt.p("depth_err", f"[DEPTH] convert failed: {e}", 2.0)

    # ---- YOLO ----
    def run_yolo(self, rgb_bgr: np.ndarray):
        self.last_best_bbox = None
        self.last_best_mask = None
        self.last_best_conf = None
        self.last_best_label = None

        if self.yolo is None:
            self.last_det_count = 0
            self.pt.p("yolo_none", "[YOLO] model not loaded", 2.0)
            return None

        conf_th = float(self.get_parameter('detection_confidence').value)
        target = self.get_parameter('target_class').value.strip().lower()

        self.pt.p("yolo_run", "[YOLO] running inference...", 0.8)
        try:
            results = self.yolo(
                rgb_bgr,
                conf=conf_th,
                verbose=False,
                device=self.yolo_device,
                half=self.yolo_half,
                imgsz=self.yolo_imgsz
            )
        except Exception as e:
            err_text = str(e).lower()
            cuda_runtime_failure = (
                self.yolo_device.startswith('cuda')
                and ('cuda' in err_text or 'nvml' in err_text or 'cudacachingallocator' in err_text)
            )
            if not cuda_runtime_failure:
                self.pt.p("yolo_infer_err", f"[YOLO] inference failed: {e}", 1.0)
                self.last_det_count = 0
                return None

            self.pt.p("yolo_cpu_fallback", f"[YOLO] CUDA inference failed ({e}); switching to CPU", 0.5)
            try:
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
            except Exception:
                pass

            try:
                self.yolo_device = 'cpu'
                self.yolo_half = False
                self.yolo.to(self.yolo_device)
                results = self.yolo(
                    rgb_bgr,
                    conf=conf_th,
                    verbose=False,
                    device=self.yolo_device,
                    half=self.yolo_half,
                    imgsz=self.yolo_imgsz
                )
                self.pt.p("yolo_cpu_ok", "[YOLO] CPU fallback inference succeeded", 1.0)
            except Exception as e_cpu:
                self.pt.p("yolo_cpu_fail", f"[YOLO] CPU fallback inference failed: {e_cpu}", 1.0)
                self.last_det_count = 0
                return None

        if not results or results[0].boxes is None:
            self.last_det_count = 0
            self.pt.p("yolo_nores", "[YOLO] no results object", 1.0)
            return None

        r0 = results[0]
        boxes = r0.boxes
        n_det = len(boxes) if boxes is not None else 0
        self.last_det_count = n_det
        self.pt.p("yolo_det", f"[YOLO] detections={n_det}", 0.8)

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
            label = str(self.yolo.names.get(cls_id, cls_id)).lower() if hasattr(self.yolo, 'names') else str(cls_id)

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
                    best_label = str(self.yolo.names.get(cls_id, cls_id)).lower() if hasattr(self.yolo, 'names') else str(cls_id)

        mask_i = masks[best_i].detach().cpu().numpy()
        h, w = rgb_bgr.shape[:2]
        mask_resized = cv2.resize(mask_i.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR) > 0.5
        bbox = boxes[best_i].xyxy[0].detach().cpu().numpy().astype(np.float32)

        self.last_best_bbox = bbox
        self.last_best_mask = mask_resized
        self.last_best_conf = float(best_conf)
        self.last_best_label = best_label

        mask_px = int(np.count_nonzero(mask_resized))
        self.pt.p("yolo_sel", f"[YOLO] selected idx={best_i} conf={best_conf:.2f} mask_px={mask_px}", 0.8)

        return mask_resized

    # ---- 3D from depth->color projection ----
    def compute_3d_from_bbox_center_depth_extrinsics(self, bbox_xyxy: np.ndarray, depth_m: np.ndarray):
        if self.color_info is None or self.depth_info is None or self.depth_to_color_extr is None:
            self.pt.p("wait_meta", "[WAIT] missing color_info/depth_info/extrinsics", 1.0)
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
        rad = float(self.get_parameter('bbox_center_radius_px').value)

        stride = max(1, int(self.get_parameter('depth_stride').value))
        min_z = float(self.get_parameter('min_depth_m').value)
        max_z = float(self.get_parameter('max_depth_m').value)
        min_pts = int(self.get_parameter('min_projected_points').value)

        points_color = []
        nearest_point = None
        nearest_d2 = 1e30

        for v in range(0, Hd, stride):
            z_row = depth_m[v, :]
            for u in range(0, Wd, stride):
                z = float(z_row[u])
                if not (min_z < z < max_z):
                    continue

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

                d2 = (up - uc) * (up - uc) + (vp - vc) * (vp - vc)
                if d2 < nearest_d2:
                    nearest_d2 = d2
                    nearest_point = Pc
                if d2 <= rad * rad:
                    points_color.append(Pc)

        if len(points_color) >= min_pts:
            pts = np.stack(points_color, axis=0)
            centroid = np.median(pts, axis=0).astype(np.float32)
            return centroid, pts, (int(round(uc)), int(round(vc)))

        if nearest_point is not None:
            pts = np.expand_dims(nearest_point.astype(np.float32), axis=0)
            centroid = pts[0]
            self.pt.p("bbox_near_only", "[3D] using nearest depth point to bbox center", 0.5)
            return centroid, pts, (int(round(uc)), int(round(vc)))

        self.pt.p("bbox_no_depth", "[3D] no valid depth near bbox center", 0.5)
        return None

    def estimate_surface_normal(self, points_3d: np.ndarray):
        if len(points_3d) < 10:
            self.pt.p("normal_few", f"[NORMAL] too few points for normal estimation: {len(points_3d)}", 1.0)
            return None

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_3d)

            radius = float(self.get_parameter('normal_radius').value)
            max_nn = int(self.get_parameter('normal_max_nn').value)

            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
            )

            pcd.orient_normals_towards_camera_location(camera_location=np.array([0, 0, 0]))

            normals = np.asarray(pcd.normals)
            avg_normal = np.mean(normals, axis=0)
            avg_normal /= np.linalg.norm(avg_normal)

            if avg_normal[2] > 0:
                avg_normal *= -1

            self.pt.p("normal", f"[NORMAL] estimated [{avg_normal[0]:+.3f},{avg_normal[1]:+.3f},{avg_normal[2]:+.3f}]", 1.0)

            return avg_normal.astype(np.float32)

        except Exception as e:
            self.pt.p("normal_err", f"[NORMAL] estimation failed: {e}", 2.0)
            return None

    def transform_point_to_base(self, point_cam_xyz: np.ndarray):
        param_cam_frame = str(self.get_parameter('color_optical_frame').value)
        cam_frame = self.color_frame_id if self.color_frame_id else param_cam_frame
        base_frame = self.get_parameter('base_frame').value
        tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        if (cam_frame != param_cam_frame) and (not self._warned_color_frame_override):
            self.pt.p(
                "cam_frame_override",
                f"[TF] using CameraInfo frame '{cam_frame}' instead of param '{param_cam_frame}'",
                9999.0
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
                base_frame, cam_frame, rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout)
            )
            pb = tf2_geometry_msgs.do_transform_point(ps, tf)
            return ps, pb
        except Exception as e2:
            self.pt.p("tf_fail_fb", f"[TF] point transform {cam_frame}->{base_frame} failed: {e2}", 1.0)
            return None, None

    def transform_vector_to_base(self, vector_cam: np.ndarray):
        param_cam_frame = str(self.get_parameter('color_optical_frame').value)
        cam_frame = self.color_frame_id if self.color_frame_id else param_cam_frame
        base_frame = self.get_parameter('base_frame').value
        tf_timeout = float(self.get_parameter('tf_timeout_s').value)

        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame, cam_frame, rclpy.time.Time(),
                timeout=Duration(seconds=tf_timeout)
            )
            q = tf.transform.rotation
            rot_base_cam = Rotation.from_quat([q.x, q.y, q.z, q.w])
            vector_base = rot_base_cam.apply(vector_cam)
            return vector_base
        except Exception as e2:
            self.pt.p("tf_vec_fail_fb", f"[TF] vector transform {cam_frame}->{base_frame} failed: {e2}", 1.0)
            return None

    def publish_normal_marker(self, centroid_base: np.ndarray, normal_base: np.ndarray):
        base_frame = self.get_parameter('base_frame').value

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
            cv2.putText(blank, "Waiting for RGB...", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
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
        cv2.putText(left, f"Det={status_text} YOLO={yolo_text} fps={self._fps:.1f} age={age:.2f}s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        dets = self.last_det_count if self.last_det_count is not None else -1
        cv2.putText(right, f"YOLO det={dets}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        if self.last_best_bbox is not None:
            x1, y1, x2, y2 = self.last_best_bbox.astype(int)
            cv2.rectangle(right, (x1, y1), (x2, y2), (0, 255, 0), 2)
            lab = self.last_best_label if self.last_best_label else "obj"
            cf = self.last_best_conf if self.last_best_conf is not None else 0.0
            cv2.putText(right, f"{lab} {cf:.2f}", (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        if self.last_best_mask is not None:
            mask = self.last_best_mask
            overlay = right.copy()
            overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)
            right = overlay
            cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(right, cnts, -1, (0, 255, 255), 2)
        else:
            cv2.putText(right, "NO DETECTION", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        if self.last_centroid_cam is not None:
            cx, cy, cz = self.last_centroid_cam
            cv2.putText(right, f"cam xyz=({cx:+.3f},{cy:+.3f},{cz:+.3f})",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        if self.last_centroid_base is not None:
            bx, by, bz = self.last_centroid_base
            cv2.putText(right, f"base xyz=({bx:+.3f},{by:+.3f},{bz:+.3f})",
                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        if self.last_centroid_uv is not None:
            u, v = self.last_centroid_uv
            h, w = right.shape[:2]
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(right, (u, v), 8, (0, 0, 255), -1)
                cv2.circle(right, (u, v), 16, (255, 255, 255), 2)
                cv2.putText(right, f"centroid px=({u},{v})", (10, 190),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        combo = np.hstack([left, right])
        cv2.imshow("button_viz", combo)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("[QUIT] pressed q", flush=True)
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
            np.zeros((1, 1), dtype=np.uint8), encoding='mono8'
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
        msg.segmentation_mask = self.bridge.cv2_to_imgmsg(mask_uint8, encoding='mono8')
        msg.bounding_box = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
        msg.confidence = float(confidence)
        msg.pose_xyzrpy = INVALID_POSE[:]
        msg.is_pressable = False
        self.button_info_pub.publish(msg)

    def _publish_detected_pressable(self, filtered_xyz, filtered_rpy, mask, bbox, confidence):
        """State 3: Button detected and pressable. All fields valid."""
        msg = ButtonInfo()
        msg.id = self._button_id
        mask_uint8 = (mask.astype(np.uint8)) * 255
        msg.segmentation_mask = self.bridge.cv2_to_imgmsg(mask_uint8, encoding='mono8')
        msg.bounding_box = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
        msg.confidence = float(confidence)
        msg.pose_xyzrpy = [
            float(filtered_xyz[0]), float(filtered_xyz[1]), float(filtered_xyz[2]),
            float(filtered_rpy[0]), float(filtered_rpy[1]), float(filtered_rpy[2]),
        ]
        # TODO: Yucheng to provide Swap the exact IK method/function
        msg.is_pressable = True
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
            self.pt.p("wait_rgb", "[WAIT] rgb not received yet", 1.0)
            self._publish_no_button()
            return
        if self.latest_depth_m is None:
            self.pt.p("wait_depth", "[WAIT] depth not received yet", 1.0)
            self._publish_no_button()
            return
        if self.color_info is None or self.depth_info is None or self.depth_to_color_extr is None:
            self.pt.p("wait_meta", "[WAIT] missing camera_info or extrinsics", 1.0)
            self._publish_no_button()
            return

        now = time.time()
        if self.last_rgb_t is not None and (now - self.last_rgb_t) > 1.0:
            self.pt.p("stale_rgb", f"[WARN] rgb seems stale: age={now-self.last_rgb_t:.2f}s", 1.0)

        # 1) YOLO — no detection means no button (state 1)
        mask = self.run_yolo(self.latest_rgb)
        if mask is None or self.last_best_bbox is None:
            self.pt.p("no_seg", "[YOLO] no valid segmentation", 1.0)
            self._publish_no_button()
            return

        # From here we have a detection (mask, bbox, confidence are valid).
        # If any downstream step fails, we publish state 2 (detected, not pressable).
        cur_mask = mask
        cur_bbox = self.last_best_bbox
        cur_conf = self.last_best_conf if self.last_best_conf is not None else 0.0

        # 2) 3D centroid (cam) from bbox center
        centroid_res = self.compute_3d_from_bbox_center_depth_extrinsics(cur_bbox, self.latest_depth_m)
        if centroid_res is None:
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return
        centroid_cam, points_cam, centroid_uv = centroid_res
        self.last_centroid_cam = centroid_cam
        self.last_centroid_uv = centroid_uv
        cam_frame = self.color_frame_id if self.color_frame_id else str(self.get_parameter('color_optical_frame').value)
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
        centroid_base = np.array([ps_base.point.x, ps_base.point.y, ps_base.point.z], dtype=np.float32)
        self.last_centroid_base = centroid_base

        # 4) Estimate surface normal in camera frame, transform to base
        normal_cam = self.estimate_surface_normal(points_cam)

        # Compute RPY from normal
        rpy = np.zeros(3, dtype=np.float64)
        if normal_cam is not None:
            normal_base = self.transform_vector_to_base(normal_cam)
            if normal_base is not None:
                nrm = np.linalg.norm(normal_base)
                if nrm > 1e-9:
                    normal_base = (normal_base / nrm).astype(np.float32)
                    # Tool Z = -normal (point into button)
                    z_axis = -normal_base
                    x_axis = np.array([1.0, 0.0, 0.0])
                    if abs(np.dot(x_axis, z_axis)) > 0.9:
                        x_axis = np.array([0.0, 1.0, 0.0])
                    y_axis = np.cross(z_axis, x_axis)
                    y_axis /= np.linalg.norm(y_axis)
                    x_axis = np.cross(y_axis, z_axis)
                    x_axis /= np.linalg.norm(x_axis)
                    rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
                    rot = Rotation.from_matrix(rot_matrix)
                    rpy = np.array(rot.as_euler('xyz'), dtype=np.float64)

                    # Publish marker
                    self.publish_normal_marker(centroid_base, normal_base)

        # 5) Filter
        self._pose_filter.update(centroid_base.astype(np.float64), rpy)

        if not self._pose_filter.is_stable:
            # Filter warming up — button is detected but pose is not stable yet
            self.pt.p("filter_warmup", f"[FILTER] warming up ({self._pose_filter._count}/{self._pose_filter.min_samples})", 0.5)
            self._publish_detected_not_pressable(cur_mask, cur_bbox, cur_conf)
            return

        # 6) Full success — publish state 3 (detected and pressable)
        self._publish_detected_pressable(
            filtered_xyz=self._pose_filter.xyz,
            filtered_rpy=self._pose_filter.rpy,
            mask=cur_mask,
            bbox=cur_bbox,
            confidence=cur_conf,
        )

        xyz = self._pose_filter.xyz
        self.pt.p("pub_info",
                  f"[PUB] ButtonInfo at [{xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f}]", 0.5)


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


if __name__ == '__main__':
    main()
