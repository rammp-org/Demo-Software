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


class PerceptionCurbDetectionNode(Node):
    def __init__(self):
        super().__init__("perception_curb_detection_node")
        from ament_index_python.packages import get_package_share_directory

        # Parameters
        self.declare_parameter("model_name", "segmentation_ascent.pth")
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
        self.declare_parameter("output_marker_topic", "/perception/curb_visual")
        self.declare_parameter("curb_info_topic", "/nav/curb/info")
        self.declare_parameter("segmentation_mask_topic", "/perception/curb_mask")
        self.declare_parameter("mask_image_topic", "/perception/curb_mask_image")
        self.declare_parameter("curb_class_id", 0)
        self.declare_parameter("rotation_degrees", 90)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("ransac_threshold", 0.1)
        self.declare_parameter("ransac_iterations", 100)
        self.declare_parameter("detection_rate_hz", 30.0)

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
        self.curb_id = (
            self.get_parameter("curb_class_id").get_parameter_value().integer_value
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

        self.class_names = {0: "curb", 1: "road"}
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
            "nav/curb/detect",
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

        self.get_logger().info("Perception Curb Detection Node initialized.")

    def publish_marker(self, centroid, a, b, height, stamp, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "curb_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(centroid[0])
        marker.pose.position.y = float(centroid[1])
        marker.pose.position.z = float(centroid[2])

        angle = float(np.arctan2(-a, b))
        marker.pose.orientation.z = float(np.sin(angle / 2.0))
        marker.pose.orientation.w = float(np.cos(angle / 2.0))

        marker.scale.x = 2.0
        marker.scale.y = 0.05
        marker.scale.z = float(height)

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
                response.message = "Curb detection already enabled."
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
            response.message = "Curb detection enabled."
        else:
            self._stop_event.set()
            if self._inference_thread is not None:
                self._inference_thread.join(timeout=5.0)
                self._inference_thread = None
            if self.model is not None:
                del self.model
                self.model = None
            torch.cuda.empty_cache()
            self.get_logger().info("Curb detection disabled, GPU memory freed.")
            response.success = True
            response.message = "Curb detection disabled."

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
                        curb_info.distance, curb_info.height, curb_info.orientation = (
                            res
                        )
                        curb_info.success = True
                        curb_info.message = "Detection successful."
                    else:
                        curb_info.success = False
                        curb_info.message = "No curb detected."
                    self.curb_info_pub.publish(curb_info)
                except Exception as e:
                    self.get_logger().error(f"Detection error: {e}")

            sleep_time = self._target_period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def process_integrated(self, model, img_msg, depth_msg, info_msg):
        try:
            # 1. Segmentation
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            detections = model.predict(pil_image, threshold=self.conf_threshold)

            if len(detections) == 0 or detections.mask is None:
                return

            # Combine all masks for the target class
            mask_indices = np.where(detections.class_id == self.curb_id)[0]
            if len(mask_indices) == 0:
                return

            combined_mask = np.any(detections.mask[mask_indices], axis=0)

            # Publish segmentation mask (mono8) and annotated mask image
            self.mask_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    combined_mask.astype(np.uint8) * 255,
                    encoding="mono8",
                    header=img_msg.header,
                )
            )
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

            # 2. 3D Projection
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")

            # Rotate depth to match rotated color image
            if self.rotation == 90:
                depth_proc = cv2.rotate(depth_img, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotation == -90:
                depth_proc = cv2.rotate(depth_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif self.rotation == 180:
                depth_proc = cv2.rotate(depth_img, cv2.ROTATE_180)
            else:
                depth_proc = depth_img

            if combined_mask.shape != depth_proc.shape:
                combined_mask = (
                    cv2.resize(
                        combined_mask.astype(np.uint8),
                        (depth_proc.shape[1], depth_proc.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    > 0
                )

            v_rot, u_rot = np.where(combined_mask)
            if len(v_rot) < 10:
                return

            # Downsample for speed
            step = max(1, len(v_rot) // 500)
            v_rot, u_rot = v_rot[::step], u_rot[::step]

            depths = depth_proc[v_rot, u_rot].astype(np.float32) / 1000.0
            valid = depths > 0.1
            v_rot, u_rot, depths = v_rot[valid], u_rot[valid], depths[valid]

            if len(depths) < 10:
                return

            # Un-rotate to sensor frame
            if self.rotation == 90:
                u_orig, v_orig = v_rot, info_msg.height - 1 - u_rot
            elif self.rotation == -90:
                u_orig, v_orig = info_msg.width - 1 - v_rot, u_rot
            else:
                u_orig, v_orig = u_rot, v_rot

            # Project to camera 3D
            fx, fy = info_msg.k[0], info_msg.k[4]
            cx, cy = info_msg.k[2], info_msg.k[5]
            x_cam = (u_orig - cx) * depths / fx
            y_cam = (v_orig - cy) * depths / fy
            z_cam = depths

            # Transform to base_link (non-blocking — skip frame if TF not ready)
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
                rot_mat = np.array(
                    [
                        [
                            1 - 2 * y_q**2 - 2 * z_q**2,
                            2 * x_q * y_q - 2 * z_q * w_q,
                            2 * x_q * z_q + 2 * y_q * w_q,
                        ],
                        [
                            2 * x_q * y_q + 2 * z_q * w_q,
                            1 - 2 * x_q**2 - 2 * z_q**2,
                            2 * y_q * z_q - 2 * x_q * w_q,
                        ],
                        [
                            2 * x_q * z_q - 2 * y_q * w_q,
                            2 * y_q * z_q + 2 * x_q * w_q,
                            1 - 2 * x_q**2 - 2 * y_q**2,
                        ],
                    ]
                )

                mat = np.eye(4)
                mat[:3, :3] = rot_mat
                mat[0, 3] = t.x
                mat[1, 3] = t.y
                mat[2, 3] = t.z

                points_cam = np.stack(
                    [x_cam, y_cam, z_cam, np.ones_like(x_cam)], axis=1
                )
                points = (points_cam @ mat.T)[:, :3].astype(np.float32)
                out_header_frame = self.target_frame
            except Exception as e:
                self.get_logger().warn(
                    f"Could not transform points to {self.target_frame}: {e}"
                )
                points = np.stack([x_cam, y_cam, z_cam], axis=1).astype(np.float32)
                out_header_frame = info_msg.header.frame_id

            # 3. Curb Fitting (vectorized RANSAC)
            if len(points) < 10:
                return

            x_pts = points[:, 0]
            y_pts = points[:, 1]
            z_pts = points[:, 2]
            n_points = len(x_pts)

            idx1 = np.random.randint(0, n_points, self.ransac_iters)
            idx2 = np.random.randint(0, n_points, self.ransac_iters)
            collide = idx1 == idx2
            idx2[collide] = (idx2[collide] + 1) % n_points

            vx = x_pts[idx2] - x_pts[idx1]
            vy = y_pts[idx2] - y_pts[idx1]
            mag = np.sqrt(vx**2 + vy**2)
            valid_lines = mag > 1e-6
            if not np.any(valid_lines):
                return

            vx, vy, mag = vx[valid_lines], vy[valid_lines], mag[valid_lines]
            p1x, p1y = x_pts[idx1[valid_lines]], y_pts[idx1[valid_lines]]
            a = -vy / mag
            b = vx / mag
            d = -(a * p1x + b * p1y)

            # All distances in one broadcast: shape (n_lines, n_points)
            all_dists = np.abs(a[:, None] * x_pts + b[:, None] * y_pts + d[:, None])
            inlier_counts = np.sum(all_dists < self.ransac_thresh, axis=1)
            best_idx = int(np.argmax(inlier_counts))
            best_inliers = np.where(all_dists[best_idx] < self.ransac_thresh)[0]

            if len(best_inliers) < 5:
                return

            inlier_points = points[best_inliers]
            centroid = np.mean(inlier_points, axis=0)
            A = np.column_stack(
                (inlier_points[:, 0] - centroid[0], inlier_points[:, 1] - centroid[1])
            )
            _, _, vh = np.linalg.svd(A, full_matrices=False)
            a_fit, b_fit = vh[1, :]
            d_fit = -(a_fit * centroid[0] + b_fit * centroid[1])

            plane_angle = np.arctan2(-a_fit, b_fit)
            height = np.max(z_pts[best_inliers]) - np.min(z_pts[best_inliers])
            distance = np.abs(d_fit)

            self.publish_marker(
                centroid, a_fit, b_fit, height, img_msg.header.stamp, out_header_frame
            )
            return float(distance), float(height), float(plane_angle)

        except Exception as e:
            self.get_logger().error(f"Error in process_integrated: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionCurbDetectionNode()

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
