#!/usr/bin/env python3

import os
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from PIL import Image as PILImage
import tf2_ros
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool
from neu_navigation_interfaces.msg import CurbInfo
import torch

# RF-DETR imports
from rfdetr import RFDETRSegSmall
import supervision as sv


class PerceptionCurbDescentDetectionNode(Node):
    def __init__(self):
        super().__init__("perception_curb_descent_detection_node")
        from ament_index_python.packages import get_package_share_directory

        # Parameters
        self.declare_parameter("model_name", "segmentation_descent.pth")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter(
            "input_image_topic", "/camera/nav1/color/image_rotated"
        )
        self.declare_parameter(
            "input_depth_topic", "/camera/nav1/depth/image_rotated"
        )
        self.declare_parameter(
            "input_info_topic", "/camera/nav1/color/camera_info_rotated"
        )
        self.declare_parameter("output_marker_topic", "/perception/curb_descent_visual")
        self.declare_parameter("curb_info_topic", "/nav/curb_descent/info")
        self.declare_parameter("segmentation_mask_topic", "/perception/curb_descent_mask")
        self.declare_parameter("mask_image_topic", "/perception/curb_descent_mask_image")
        
        # specific class mapping based on curb_road_point_extractor
        self.declare_parameter("curb_class_id", 0)  # curb class from model
        self.declare_parameter("road_class_id", 1)  # road class from model
        
        self.declare_parameter("rotation_degrees", 90)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("ransac_threshold", 0.1)
        self.declare_parameter("ransac_iterations", 100)
        self.declare_parameter("detection_rate_hz", 30.0)
        self.declare_parameter("edge_band_width", 0.5)
        self.declare_parameter("min_points", 10)

        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.conf_threshold = (
            self.get_parameter("confidence_threshold")
            .get_parameter_value()
            .double_value
        )
        self.image_topic = (
            self.get_parameter("input_image_topic").get_parameter_value().string_value
        )
        self.depth_topic = (
            self.get_parameter("input_depth_topic").get_parameter_value().string_value
        )
        info_topic = (
            self.get_parameter("input_info_topic").get_parameter_value().string_value
        )
        marker_topic = (
            self.get_parameter("output_marker_topic").get_parameter_value().string_value
        )
        info_publisher_topic = (
            self.get_parameter("curb_info_topic").get_parameter_value().string_value
        )
        mask_topic = (
            self.get_parameter("segmentation_mask_topic")
            .get_parameter_value()
            .string_value
        )
        mask_image_topic = (
            self.get_parameter("mask_image_topic").get_parameter_value().string_value
        )
        self.curb_class_id = (
            self.get_parameter("curb_class_id").get_parameter_value().integer_value
        )
        self.road_class_id = (
            self.get_parameter("road_class_id").get_parameter_value().integer_value
        )
        self.rotation = (
            self.get_parameter("rotation_degrees").get_parameter_value().integer_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.ransac_thresh = (
            self.get_parameter("ransac_threshold").get_parameter_value().double_value
        )
        self.ransac_iters = (
            self.get_parameter("ransac_iterations").get_parameter_value().integer_value
        )
        self.edge_band_width = (
            self.get_parameter("edge_band_width").get_parameter_value().double_value
        )
        self.min_points = (
            self.get_parameter("min_points").get_parameter_value().integer_value
        )
        detection_rate_hz = (
            self.get_parameter("detection_rate_hz").get_parameter_value().double_value
        )
        self._target_period = 1.0 / max(detection_rate_hz, 0.1)

        # Resolve model path
        package_share_dir = get_package_share_directory("neu_navigation")
        model_path = os.path.join(package_share_dir, "models", model_name)
        if not os.path.exists(model_path):
            abs_fallback = os.path.expanduser(f"~/rf-detr/output/{model_name}")
            if os.path.exists(abs_fallback):
                model_path = abs_fallback

        # Model state
        self.model_path = model_path
        self.model = None
        self.get_logger().info("Model will be loaded when detection is enabled.")

        # Thread-safe latest frame + inference thread control
        self._data_lock = threading.Lock()
        self.latest_data = None
        self._inference_thread = None
        self._stop_event = threading.Event()

        self.class_names = {self.curb_class_id: "curb", self.road_class_id: "road"}
        self.mask_annotator = sv.MaskAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.curb_info_pub = self.create_publisher(CurbInfo, info_publisher_topic, 10)
        self.mask_pub = self.create_publisher(Image, mask_topic, 10)
        self.mask_image_pub = self.create_publisher(Image, mask_image_topic, 10)

        self.cb_group = ReentrantCallbackGroup()

        # Service Server (SetBool: True = enable streaming, False = disable and free GPU)
        self.srv = self.create_service(
            SetBool,
            "nav/curb_descent/detect",
            self.set_detection_callback,
            callback_group=self.cb_group,
        )

        # Synchronized Subscribers
        self.image_sub = message_filters.Subscriber(
            self, Image, self.image_topic, callback_group=self.cb_group
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, callback_group=self.cb_group
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, info_topic, callback_group=self.cb_group
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.3
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Perception Curb Descent Node initialized.")

    def publish_marker(self, centroid, length, height, angle, stamp, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "curb_descent_edge"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(centroid[0])
        marker.pose.position.y = float(centroid[1])
        marker.pose.position.z = float(centroid[2])

        marker.pose.orientation.z = float(np.sin(angle / 2.0))
        marker.pose.orientation.w = float(np.cos(angle / 2.0))

        marker.scale.x = float(max(length, 0.1))
        marker.scale.y = 0.05
        marker.scale.z = float(max(abs(height), 0.02))

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 0.6

        self.marker_pub.publish(marker)

    def sync_callback(self, img_msg, depth_msg, info_msg):
        # Store latest frame; inference thread picks it up at its own rate.
        with self._data_lock:
            self.latest_data = (img_msg, depth_msg, info_msg)

    def set_detection_callback(self, request, response):
        if request.data:
            if self._inference_thread is not None and self._inference_thread.is_alive():
                response.success = True
                response.message = "Curb descent detection already enabled."
                return response

            model_path = self.model_path
            if not os.path.exists(model_path):
                parent_dir = os.path.dirname(os.path.dirname(model_path))
                alt_path = os.path.join(parent_dir, os.path.basename(model_path))
                if os.path.exists(alt_path):
                    model_path = alt_path
                else:
                    response.success = False
                    response.message = "Model file not found."
                    return response

            self.get_logger().info(f"Loading model: {os.path.basename(model_path)}")
            self.model = RFDETRSegSmall(pretrain_weights=model_path)
            self._stop_event.clear()
            self._inference_thread = threading.Thread(
                target=self._inference_loop, daemon=True
            )
            self._inference_thread.start()
            response.success = True
            response.message = "Curb descent detection enabled."
        else:
            self._stop_event.set()
            if self._inference_thread is not None:
                self._inference_thread.join(timeout=5.0)
                self._inference_thread = None
            if self.model is not None:
                del self.model
                self.model = None
            torch.cuda.empty_cache()
            self.get_logger().info("Curb descent detection disabled, GPU memory freed.")
            response.success = True
            response.message = "Curb descent detection disabled."

        return response

    def _inference_loop(self):
        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            with self._data_lock:
                data = self.latest_data
                self.latest_data = None

            if data is not None:
                img_msg, depth_msg, info_msg = data
                try:
                    res = self.process_integrated(
                        self.model, img_msg, depth_msg, info_msg
                    )
                    curb_info = CurbInfo()
                    if res:
                        curb_info.distance, curb_info.height, curb_info.orientation = res
                        curb_info.success = True
                        curb_info.message = "Detection successful."
                    else:
                        curb_info.success = False
                        curb_info.message = "No drop detected."
                    self.curb_info_pub.publish(curb_info)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    self.get_logger().error(f"Detection error: {e}")

            sleep_time = self._target_period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _fit_line_xy(self, points: np.ndarray):
        """
        Fit ax + by + d = 0 to the XY projection of points.
        Returns (a, b, d, best_inliers, centroid) or Non
        """
        x = points[:, 0]
        y = points[:, 1]
        n = len(x)

        idx1 = np.random.randint(0, n, self.ransac_iters * 2)
        idx2 = np.random.randint(0, n, self.ransac_iters * 2)
        same = idx1 == idx2
        idx2[same] = (idx2[same] + 1) % n

        vx = x[idx2] - x[idx1]
        vy = y[idx2] - y[idx1]
        mag = np.hypot(vx, vy)
        valid = mag > 0.05  # Enforce points must be at least 5cm apart to create a stable slope
        if not np.any(valid):
            return None

        vx, vy = vx[valid], vy[valid]
        px, py = x[idx1[valid]], y[idx1[valid]]
        mag = mag[valid]

        a_all = -vy / mag
        b_all = vx / mag
        d_all = -(a_all * px + b_all * py)

        best_idx = -1
        best_inliers = np.empty(0, dtype=int)
        
        # All distances
        all_dists = np.abs(a_all[:, None] * x + b_all[:, None] * y + d_all[:, None])
        inlier_counts = np.sum(all_dists < self.ransac_thresh, axis=1)
        best_idx = int(np.argmax(inlier_counts))
        best_inliers = np.where(all_dists[best_idx] < self.ransac_thresh)[0]

        if len(best_inliers) < 5:
            return None

        inp = points[best_inliers]
        centroid = np.mean(inp, axis=0)
        A = np.column_stack((inp[:, 0] - centroid[0], inp[:, 1] - centroid[1]))
        _, _, vh = np.linalg.svd(A, full_matrices=False)
        normal = vh[1, :]
        a_fit = float(normal[0])
        b_fit = float(normal[1])
        d_fit = -(a_fit * centroid[0] + b_fit * centroid[1])

        return a_fit, b_fit, d_fit, best_inliers, centroid

    def process_integrated(self, model, img_msg, depth_msg, info_msg):
        try:
            # 1. Segmentation
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            detections = model.predict(pil_image, threshold=self.conf_threshold)

            if len(detections) == 0 or detections.mask is None:
                return None

            # Get masks
            curb_idx = np.where(detections.class_id == self.curb_class_id)[0]
            road_idx = np.where(detections.class_id == self.road_class_id)[0]

            if len(curb_idx) == 0 or len(road_idx) == 0:
                return None

            curb_mask = np.any(detections.mask[curb_idx], axis=0)
            road_mask = np.any(detections.mask[road_idx], axis=0)

            if self.mask_pub.get_subscription_count() > 0:
                # Just build a combination to publish for viewing
                combined_mask = np.zeros_like(curb_mask, dtype=np.uint8)
                combined_mask[curb_mask] = 100
                combined_mask[road_mask] = 200

                self.mask_pub.publish(
                    self.bridge.cv2_to_imgmsg(
                        combined_mask, encoding="mono8", header=img_msg.header
                    )
                )

            if self.mask_image_pub.get_subscription_count() > 0:
                labels = [
                    f"{self.class_names.get(class_id, 'unknown')} {confidence:.2f}"
                    for class_id, confidence in zip(
                        detections.class_id, detections.confidence
                    )
                ]
                annotated = self.mask_annotator.annotate(
                    scene=cv_image.copy(), detections=detections
                )
                annotated = self.label_annotator.annotate(
                    scene=annotated, detections=detections, labels=labels
                )
                self.mask_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(
                        annotated, encoding="bgr8", header=img_msg.header
                    )
                )

            # 2. 3D Projection for curb and road
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")

            # Check if depth rotation is needed manually by comparing matrix bounds
            if curb_mask.shape != depth_img.shape:
                if self.rotation == 90:
                    depth_proc = cv2.rotate(depth_img, cv2.ROTATE_90_CLOCKWISE)
                elif self.rotation == -90 or self.rotation == 270:
                    depth_proc = cv2.rotate(depth_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif self.rotation == 180:
                    depth_proc = cv2.rotate(depth_img, cv2.ROTATE_180)
                else:
                    depth_proc = depth_img
            else:
                depth_proc = depth_img

            if curb_mask.shape != depth_proc.shape:
                curb_mask_u8 = cv2.resize(curb_mask.astype(np.uint8), (depth_proc.shape[1], depth_proc.shape[0]), interpolation=cv2.INTER_NEAREST)
                road_mask_u8 = cv2.resize(road_mask.astype(np.uint8), (depth_proc.shape[1], depth_proc.shape[0]), interpolation=cv2.INTER_NEAREST)
            else:
                curb_mask_u8 = curb_mask.astype(np.uint8)
                road_mask_u8 = road_mask.astype(np.uint8)

            # Use 2D morph dilation to intersect the masks. This reveals the true edge pixels where Curb and Road touch cleanly regardless of viewing angle.
            kernel = np.ones((11, 11), np.uint8)
            road_dilated = cv2.dilate(road_mask_u8, kernel, iterations=1)
            
            curb_edge_mask = (curb_mask_u8 > 0) & (road_dilated > 0)

            # Transform matrix fetch before projection
            try:
                source_frame = info_msg.header.frame_id
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    img_msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.0),
                )
                q = trans.transform.rotation
                t = trans.transform.translation
                x_q, y_q, z_q, w_q = q.x, q.y, q.z, q.w
                rot_mat = np.array([
                    [1 - 2*y_q**2 - 2*z_q**2, 2*x_q*y_q - 2*z_q*w_q,   2*x_q*z_q + 2*y_q*w_q],
                    [2*x_q*y_q + 2*z_q*w_q,   1 - 2*x_q**2 - 2*z_q**2, 2*y_q*z_q - 2*x_q*w_q],
                    [2*x_q*z_q - 2*y_q*w_q,   2*y_q*z_q + 2*x_q*w_q,   1 - 2*x_q**2 - 2*y_q**2],
                ])
                mat = np.eye(4)
                mat[:3, :3] = rot_mat
                mat[0, 3] = t.x
                mat[1, 3] = t.y
                mat[2, 3] = t.z
                out_header_frame = self.target_frame
            except Exception as e:
                self.get_logger().warn(f"TF Failed: {e}", throttle_duration_sec=2.0)
                return None

            fx, fy = info_msg.k[0], info_msg.k[4]
            cx, cy = info_msg.k[2], info_msg.k[5]

            def extract_and_project(mask_array):
                v_rot, u_rot = np.where(mask_array)
                if len(v_rot) < self.min_points:
                    return None
                
                # Downsample
                step = max(1, len(v_rot) // 500)
                v_rot, u_rot = v_rot[::step], u_rot[::step]
                
                depths = depth_proc[v_rot, u_rot].astype(np.float32) / 1000.0
                valid = (depths > 0.1) & (depths < 10.0)
                v_rot, u_rot, depths = v_rot[valid], u_rot[valid], depths[valid]
                
                if len(depths) < self.min_points:
                    return None

                # Do not un-rotate pixels, because info_topic provides rotated camera intrinsics
                u_orig, v_orig = u_rot, v_rot

                x_cam_rot = (u_orig - cx) * depths / fx
                y_cam_rot = (v_orig - cy) * depths / fy
                z_cam = depths

                # The computed points are in the rotated optical frame.
                # TF transforms from the unrotated optical frame (info_msg.header.frame_id).
                # We unrotate the 3D coordinates so TF aligns them correctly to base_link.
                if self.rotation == 90:
                    x_cam = y_cam_rot
                    y_cam = -x_cam_rot
                elif self.rotation == -90 or self.rotation == 270:
                    x_cam = -y_cam_rot
                    y_cam = x_cam_rot
                elif self.rotation == 180:
                    x_cam = -x_cam_rot
                    y_cam = -y_cam_rot
                else:
                    x_cam = x_cam_rot
                    y_cam = y_cam_rot

                points_cam = np.stack([x_cam, y_cam, z_cam, np.ones_like(x_cam)], axis=1)
                points_3d = (points_cam @ mat.T)[:, :3].astype(np.float32)
                return points_3d

            curb_edge = extract_and_project(curb_edge_mask)

            if curb_edge is None:
                return None

            # User Request: Align to the beginning of the curb (curb cut start)
            # Fit line on the curb outer edge instead of the road inner edge
            result = self._fit_line_xy(curb_edge)
            if result is None:
                return None

            a, b, d, inlier_idx, centroid = result
            curb_inliers = curb_edge[inlier_idx]

            # Compute height based on z-drop
            curb_z = float(np.mean(curb_inliers[:, 2]))
            
            # Fetch total road point cloud to extract flat ground samples
            road_pts = extract_and_project(road_mask_u8)
            if road_pts is not None:
                # Calculate perpendicular distance of all road points to our fitted curb plane
                road_dists = np.abs(a * road_pts[:, 0] + b * road_pts[:, 1] + d)
                
                # Filter road points to a flat band safely past the cliff drop artifact (15cm to 75cm away)
                road_band = road_pts[(road_dists > 0.15) & (road_dists < 0.75)]
                if len(road_band) >= 5:
                    road_z = float(np.mean(road_band[:, 2]))
                else:
                    road_z = float(np.mean(road_pts[:, 2]))
            else:
                road_z = curb_z - 0.15 # hardware fallback

            height = curb_z - road_z  # positive -> road is below curb

            # Metrics
            distance = float(abs(d))
            angle = float(np.arctan2(-a, b))

            dx, dy = np.cos(angle), np.sin(angle)
            rel = curb_inliers[:, :2] - centroid[:2]
            proj = rel[:, 0] * dx + rel[:, 1] * dy
            length = float(np.max(proj) - np.min(proj))

            # Move centroid down to midway between curb and road for visualization
            centroid_vis = centroid.copy()
            centroid_vis[2] = curb_z - (height / 2.0)

            self.publish_marker(
                centroid_vis, length, height, angle, img_msg.header.stamp, out_header_frame
            )
            
            return float(distance), float(height), float(angle)

        except Exception as e:
            import traceback
            traceback.print_exc()
            self.get_logger().error(f"Error in process_integrated: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionCurbDescentDetectionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_event.set()
        if node._inference_thread is not None:
            node._inference_thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
