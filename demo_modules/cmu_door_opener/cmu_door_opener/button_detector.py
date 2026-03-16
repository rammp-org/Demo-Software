#!/usr/bin/env python3
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
from std_msgs.msg import ColorRGBA, Bool
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
        # Deprecated and ignored: camera mount should come from URDF/TF.
        self.declare_parameter('camera_mount_parent_frame', 'end_effector_link')
        self.declare_parameter('camera_mount_child_frame', 'camera_link')
        self.declare_parameter('camera_mount_xyz', [0.01, 0.0, 0.052])
        self.declare_parameter('camera_mount_rpy', [-math.pi / 2, -math.pi / 2, 0.0])
        self.declare_parameter('camera_mount_measurement_frame', 'parent')
        # Deprecated and ignored: kept only for backward compatibility in launch files.
        self.declare_parameter('assume_camera_at_mount_parent', False)

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

        # publish target pose here
        self.declare_parameter('pose_topic', '/button/target_pose')  # set to /button/target_pose if you want
        self.declare_parameter('fixed_pose_quat', [0.5, 0.5, 0.5, 0.5])  # [x, y, z, w], fallback only

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
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.debug_cam_pt_pub = self.create_publisher(PointStamped, '/button/debug_point_camera', 10)
        self.debug_base_pt_pub = self.create_publisher(PointStamped, '/button/debug_point_base', 10)
        self.normal_marker_pub = self.create_publisher(Marker, '/button/normal_marker', 10)
        self.visible_pub = self.create_publisher(Bool, '/arm/door/visible', 10)
        self._door_visible = False          # last published state
        self._no_detect_count = 0           # consecutive cycles with no detection
        self._no_detect_threshold = 10      # cycles before publishing False

        # ---- Subscribers ----
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

        # ---- YOLO ----
        self.yolo_device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.yolo_half = bool(self.get_parameter('yolo_use_fp16').value) and self.yolo_device.startswith('cuda')
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        model_path = self.get_parameter('yolo_model').value
        try:
            self.yolo = YOLO(model_path)
            self.yolo.to(self.yolo_device)
            print(f"[INIT] YOLO loaded: {model_path}", flush=True)
            print(
                f"[INIT] YOLO runtime device={self.yolo_device} fp16={self.yolo_half} imgsz={self.yolo_imgsz}",
                flush=True
            )
            if hasattr(self.yolo, 'names'):
                print(f"[INIT] YOLO classes: {self.yolo.names}", flush=True)
        except Exception as e:
            if self.yolo_device.startswith('cuda'):
                print(
                    f"[WARN] YOLO load failed on {self.yolo_device}: {e}. Retrying on CPU.",
                    flush=True
                )
                try:
                    self.yolo_device = 'cpu'
                    self.yolo_half = False
                    self.yolo = YOLO(model_path)
                    self.yolo.to(self.yolo_device)
                    print(f"[INIT] YOLO loaded: {model_path}", flush=True)
                    print(
                        f"[INIT] YOLO runtime device={self.yolo_device} fp16={self.yolo_half} imgsz={self.yolo_imgsz}",
                        flush=True
                    )
                    if hasattr(self.yolo, 'names'):
                        print(f"[INIT] YOLO classes: {self.yolo.names}", flush=True)
                except Exception as e_cpu:
                    print(
                        f"[FATAL] Failed to load YOLO model '{model_path}' on CPU fallback: {e_cpu}",
                        flush=True
                    )
                    raise
            else:
                print(f"[FATAL] Failed to load YOLO model '{model_path}' on device '{self.yolo_device}': {e}", flush=True)
                raise

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

        print(f"[INIT] Publishing pose on: {self.get_parameter('pose_topic').value}", flush=True)
        print("[INIT] Pose orientation mode: surface-normal based (dynamic quaternion).", flush=True)
        print(
            f"[INIT] Fallback quat: "
            f"[{self.fixed_pose_quat[0]:+.3f},{self.fixed_pose_quat[1]:+.3f},"
            f"{self.fixed_pose_quat[2]:+.3f},{self.fixed_pose_quat[3]:+.3f}]",
            flush=True
        )
        print("[INIT] Camera transform source: URDF/TF (base_frame <- camera frame).", flush=True)
        print("[INIT] camera_mount_* and assume_camera_at_mount_parent params are deprecated and ignored.", flush=True)
        print("[INIT] ButtonPressVisionNode started.", flush=True)

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
        # resets last outputs each call
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

    def compute_3d_from_mask_depth_extrinsics(self, mask_color: np.ndarray, depth_m: np.ndarray):
        if self.color_info is None or self.depth_info is None or self.depth_to_color_extr is None:
            self.pt.p("wait_meta", "[WAIT] missing color_info/depth_info/extrinsics", 1.0)
            return None

        Hc, Wc = mask_color.shape[:2]
        Hd, Wd = depth_m.shape[:2]

        Kc = self.color_info.k
        fx_c, fy_c, cx_c, cy_c = float(Kc[0]), float(Kc[4]), float(Kc[2]), float(Kc[5])

        Kd = self.depth_info.k
        fx_d, fy_d, cx_d, cy_d = float(Kd[0]), float(Kd[4]), float(Kd[2]), float(Kd[5])

        R = np.array(self.depth_to_color_extr.rotation, dtype=np.float32).reshape(3, 3)
        t = np.array(self.depth_to_color_extr.translation, dtype=np.float32).reshape(3)

        ys, xs = np.where(mask_color)
        if xs.size < 50:
            self.pt.p("mask_small", "[3D] mask too small (<50 px)", 0.5)
            return None

        # Keep only the interior of the mask so edge/background pixels do not pull centroid.
        dist = cv2.distanceTransform(mask_color.astype(np.uint8), cv2.DIST_L2, 5)
        core_min_dist = float(self.get_parameter('mask_core_min_dist_px').value)
        core_mask = dist >= core_min_dist
        core_min_points = int(self.get_parameter('mask_core_min_points').value)
        if int(np.count_nonzero(core_mask)) < core_min_points:
            # Fallback if mask is tiny: use full segmentation instead of dropping frame.
            core_mask = mask_color

        ys_core, xs_core = np.where(core_mask)
        x1, x2 = int(xs_core.min()), int(xs_core.max())
        y1, y2 = int(ys_core.min()), int(ys_core.max())

        stride = max(1, int(self.get_parameter('depth_stride').value))
        min_z = float(self.get_parameter('min_depth_m').value)
        max_z = float(self.get_parameter('max_depth_m').value)
        min_pts = int(self.get_parameter('min_projected_points').value)

        points_color = []
        point_weights = []

        for v in range(0, Hd, stride):
            z_row = depth_m[v, :]
            for u in range(0, Wd, stride):
                z = float(z_row[u])
                if not (min_z < z < max_z):
                    continue

                Xd = (u - cx_d) * z / fx_d
                Yd = (v - cy_d) * z / fy_d
                Pd = np.array([Xd, Yd, z], dtype=np.float32)

                Pc = R @ Pd + t
                Zc = float(Pc[2])
                if Zc <= 0.0:
                    continue

                uc = fx_c * (float(Pc[0]) / Zc) + cx_c
                vc = fy_c * (float(Pc[1]) / Zc) + cy_c
                ui = int(round(uc))
                vi = int(round(vc))

                if ui < x1 or ui > x2 or vi < y1 or vi > y2:
                    continue
                if ui < 0 or ui >= Wc or vi < 0 or vi >= Hc:
                    continue

                if core_mask[vi, ui]:
                    points_color.append(Pc)
                    point_weights.append(float(dist[vi, ui]))

        if len(points_color) < min_pts:
            self.pt.p("proj_few", f"[3D] too few projected points: {len(points_color)} < {min_pts}", 1.0)
            return None

        pts = np.stack(points_color, axis=0)
        w = np.array(point_weights, dtype=np.float32)
        if w.size > 0:
            wq = float(np.percentile(w, 40.0))
            core_keep = w >= wq
            if int(np.count_nonzero(core_keep)) >= max(min_pts // 2, 20):
                pts = pts[core_keep]
        centroid = np.median(pts, axis=0).astype(np.float32)
        self.pt.p("cent", f"[3D] centroid(cam) [{centroid[0]:+.3f},{centroid[1]:+.3f},{centroid[2]:+.3f}]", 1.0)
        return centroid, pts

    def estimate_surface_normal(self, points_3d: np.ndarray):
        """
        Estimate surface normal using Open3D
        Returns: normalized normal vector (3,) or None if failed
        """
        if len(points_3d) < 10:
            self.pt.p("normal_few", f"[NORMAL] too few points for normal estimation: {len(points_3d)}", 1.0)
            return None

        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_3d)
            
            # Estimate normals
            radius = float(self.get_parameter('normal_radius').value)
            max_nn = int(self.get_parameter('normal_max_nn').value)
            
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
            )
            
            # Orient normals towards camera (camera is at origin in camera frame)
            pcd.orient_normals_towards_camera_location(camera_location=np.array([0, 0, 0]))
            
            # Average normals for stability
            normals = np.asarray(pcd.normals)
            avg_normal = np.mean(normals, axis=0)
            avg_normal /= np.linalg.norm(avg_normal)
            
            # Normal should point towards camera (negative Z in camera frame)
            if avg_normal[2] > 0:
                avg_normal *= -1
            
            self.pt.p("normal", f"[NORMAL] estimated [{avg_normal[0]:+.3f},{avg_normal[1]:+.3f},{avg_normal[2]:+.3f}]", 1.0)
            
            return avg_normal.astype(np.float32)
            
        except Exception as e:
            self.pt.p("normal_err", f"[NORMAL] estimation failed: {e}", 2.0)
            return None

    # ---- Pose from Point and Normal ----
    def create_pose_from_point_normal(self, point: np.ndarray, normal: np.ndarray, frame_id: str):
        """
        Create PoseStamped with Z-axis aligned to normal
        Returns: PoseStamped or None if failed
        """
        try:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = frame_id
            
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            
            # Align Z-axis with normal
            z_axis = normal / np.linalg.norm(normal)
            
            # Choose X-axis perpendicular to Z
            x_axis = np.array([1.0, 0.0, 0.0])
            
            # If Z is too close to X, use Y as reference
            if abs(np.dot(x_axis, z_axis)) > 0.9:
                x_axis = np.array([0.0, 1.0, 0.0])
            
            # Build orthonormal basis
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis)
            
            # Create rotation matrix [X Y Z]
            rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
            rot = Rotation.from_matrix(rot_matrix)
            quat = rot.as_quat()  # [x, y, z, w]
            
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            
            return pose
            
        except Exception as e:
            self.pt.p("pose_err", f"[POSE] creation failed: {e}", 2.0)
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
        """Transform a vector (like normal) from camera frame to base frame"""
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
        """Publish visualization marker for surface normal"""
        base_frame = self.get_parameter('base_frame').value
        
        marker = Marker()
        marker.header.frame_id = base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "button_normal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow dimensions
        marker.scale.x = 0.01  # shaft diameter
        marker.scale.y = 0.02  # head diameter
        marker.scale.z = 0.0   # head length (auto)
        
        # Red color
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Start point (centroid)
        start = Point()
        start.x = float(centroid_base[0])
        start.y = float(centroid_base[1])
        start.z = float(centroid_base[2])

        # End point (centroid + normal * 0.1m)
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

        cv2.putText(left, f"RGB fps={self._fps:.1f} age={age:.2f}s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

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

        # show last computed points
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

    # ---- Main loop ----
    def process_once(self):
        self.draw_viz()

        # Waits
        if self.latest_rgb is None:
            self.pt.p("wait_rgb", "[WAIT] rgb not received yet", 1.0)
            return
        if self.latest_depth_m is None:
            self.pt.p("wait_depth", "[WAIT] depth not received yet", 1.0)
            return
        if self.color_info is None or self.depth_info is None or self.depth_to_color_extr is None:
            self.pt.p("wait_meta", "[WAIT] missing camera_info or extrinsics", 1.0)
            return

        now = time.time()
        if self.last_rgb_t is not None and (now - self.last_rgb_t) > 1.0:
            self.pt.p("stale_rgb", f"[WARN] rgb seems stale: age={now-self.last_rgb_t:.2f}s", 1.0)

        # 1) YOLO
        mask = self.run_yolo(self.latest_rgb)
        if mask is None:
            self.pt.p("no_seg", "[YOLO] no valid segmentation", 1.0)
            self._no_detect_count += 1
            if self._door_visible and self._no_detect_count >= self._no_detect_threshold:
                self.visible_pub.publish(Bool(data=False))
                self._door_visible = False
            return
        self._no_detect_count = 0
        if not self._door_visible:
            self.visible_pub.publish(Bool(data=True))
            self._door_visible = True

        # 2) 3D centroid (cam) from bbox center
        if self.last_best_bbox is None:
            self.pt.p("no_bbox", "[YOLO] no bbox for center-based centroid", 1.0)
            return
        centroid_res = self.compute_3d_from_bbox_center_depth_extrinsics(self.last_best_bbox, self.latest_depth_m)
        if centroid_res is None:
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

        # 3) TF to base (always needed for pose publish/logging)
        ps_cam, ps_base = self.transform_point_to_base(centroid_cam)
        if ps_base is None:
            return

        self.debug_base_pt_pub.publish(ps_base)
        centroid_base = np.array([ps_base.point.x, ps_base.point.y, ps_base.point.z], dtype=np.float32)
        self.last_centroid_base = centroid_base

        # 4) Estimate surface normal in camera frame
        normal_cam = self.estimate_surface_normal(points_cam)
        if normal_cam is None:
            pose = PoseStamped()
            pose.header = ps_base.header
            pose.pose.position.x = float(centroid_base[0])
            pose.pose.position.y = float(centroid_base[1])
            pose.pose.position.z = float(centroid_base[2])
            pose.pose.orientation.x = float(self.fixed_pose_quat[0])
            pose.pose.orientation.y = float(self.fixed_pose_quat[1])
            pose.pose.orientation.z = float(self.fixed_pose_quat[2])
            pose.pose.orientation.w = float(self.fixed_pose_quat[3])
            self.pose_pub.publish(pose)
            print(
                f"[POSE] calculated xyz=[{pose.pose.position.x:+.4f},{pose.pose.position.y:+.4f},{pose.pose.position.z:+.4f}] "
                f"quat=[{pose.pose.orientation.x:+.6f},{pose.pose.orientation.y:+.6f},{pose.pose.orientation.z:+.6f},{pose.pose.orientation.w:+.6f}] "
                f"(fallback: fixed quat, no normal)",
                flush=True
            )
            return

        normal_base = self.transform_vector_to_base(normal_cam)
        if normal_base is None:
            pose = PoseStamped()
            pose.header = ps_base.header
            pose.pose.position.x = float(centroid_base[0])
            pose.pose.position.y = float(centroid_base[1])
            pose.pose.position.z = float(centroid_base[2])
            pose.pose.orientation.x = float(self.fixed_pose_quat[0])
            pose.pose.orientation.y = float(self.fixed_pose_quat[1])
            pose.pose.orientation.z = float(self.fixed_pose_quat[2])
            pose.pose.orientation.w = float(self.fixed_pose_quat[3])
            self.pose_pub.publish(pose)
            print(
                f"[POSE] calculated xyz=[{pose.pose.position.x:+.4f},{pose.pose.position.y:+.4f},{pose.pose.position.z:+.4f}] "
                f"quat=[{pose.pose.orientation.x:+.6f},{pose.pose.orientation.y:+.6f},{pose.pose.orientation.z:+.6f},{pose.pose.orientation.w:+.6f}] "
                f"(fallback: fixed quat, normal TF failed)",
                flush=True
            )
            return
        nrm = np.linalg.norm(normal_base)
        if nrm < 1e-9:
            self.pt.p("normal_zero", "[NORMAL] transformed normal is near zero", 1.0)
            pose = PoseStamped()
            pose.header = ps_base.header
            pose.pose.position.x = float(centroid_base[0])
            pose.pose.position.y = float(centroid_base[1])
            pose.pose.position.z = float(centroid_base[2])
            pose.pose.orientation.x = float(self.fixed_pose_quat[0])
            pose.pose.orientation.y = float(self.fixed_pose_quat[1])
            pose.pose.orientation.z = float(self.fixed_pose_quat[2])
            pose.pose.orientation.w = float(self.fixed_pose_quat[3])
            self.pose_pub.publish(pose)
            print(
                f"[POSE] calculated xyz=[{pose.pose.position.x:+.4f},{pose.pose.position.y:+.4f},{pose.pose.position.z:+.4f}] "
                f"quat=[{pose.pose.orientation.x:+.6f},{pose.pose.orientation.y:+.6f},{pose.pose.orientation.z:+.6f},{pose.pose.orientation.w:+.6f}] "
                f"(fallback: fixed quat, zero normal)",
                flush=True
            )
            return
        normal_base = (normal_base / nrm).astype(np.float32)

        # Pre-press point: offset away from surface along +normal
        press_offset = float(self.get_parameter('press_offset').value)
        target_point = centroid_base + normal_base * press_offset

        # Tool should point into the button => tool Z = -normal_base
        pose = self.create_pose_from_point_normal(target_point, -normal_base, ps_base.header.frame_id)
        if pose is None:
            pose = PoseStamped()
            pose.header = ps_base.header
            pose.pose.position.x = float(target_point[0])
            pose.pose.position.y = float(target_point[1])
            pose.pose.position.z = float(target_point[2])
            pose.pose.orientation.x = float(self.fixed_pose_quat[0])
            pose.pose.orientation.y = float(self.fixed_pose_quat[1])
            pose.pose.orientation.z = float(self.fixed_pose_quat[2])
            pose.pose.orientation.w = float(self.fixed_pose_quat[3])

        self.pose_pub.publish(pose)
        self.publish_normal_marker(centroid_base, normal_base)
        calc_xyz = (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.position.z),
        )
        calc_quat = (
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        )
        print(
            f"[POSE] calculated xyz=[{calc_xyz[0]:+.4f},{calc_xyz[1]:+.4f},{calc_xyz[2]:+.4f}] "
            f"quat=[{calc_quat[0]:+.6f},{calc_quat[1]:+.6f},{calc_quat[2]:+.6f},{calc_quat[3]:+.6f}]",
            flush=True
        )

        self.pt.p("pub_pose", f"[PUB] pose -> {self.get_parameter('pose_topic').value} "
                              f"at [{pose.pose.position.x:+.3f},{pose.pose.position.y:+.3f},{pose.pose.position.z:+.3f}]",
                  0.5)


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
