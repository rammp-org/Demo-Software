#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

class ImageRotateNode : public rclcpp::Node {
public:
  ImageRotateNode() : Node("image_rotate_node") {
    this->declare_parameter<int>("rotation_degrees", 90);
    int deg = this->get_parameter("rotation_degrees").as_int();
    rotation_degrees_ = deg;

    if (deg == 90 || deg == -270) {
      rotate_code_ = cv::ROTATE_90_CLOCKWISE;
    } else if (deg == 180 || deg == -180) {
      rotate_code_ = cv::ROTATE_180;
    } else if (deg == 270 || deg == -90) {
      rotate_code_ = cv::ROTATE_90_COUNTERCLOCKWISE;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Unsupported rotation: %d. Use 90, 180, or 270.", deg);
      rotate_code_ = -1;
    }

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        std::bind(&ImageRotateNode::image_callback, this,
                  std::placeholders::_1));

    pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("image_rotated", qos);

    // Camera info sub/pub
    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", qos,
        std::bind(&ImageRotateNode::info_callback, this,
                  std::placeholders::_1));

    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info_rotated", qos);

    RCLCPP_INFO(this->get_logger(),
                "Image rotate node started (rotation: %d deg)", deg);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (rotate_code_ < 0) {
      pub_->publish(*msg);
      return;
    }

    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      cv::rotate(cv_ptr->image, rotated_buf_, rotate_code_);
      cv_bridge::CvImage out_msg(msg->header, msg->encoding, rotated_buf_);
      pub_->publish(*out_msg.toImageMsg());
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    sensor_msgs::msg::CameraInfo out = *msg;

    if (rotate_code_ < 0) {
      info_pub_->publish(out);
      return;
    }

    uint32_t w = msg->width;
    uint32_t h = msg->height;

    // Original intrinsics
    double fx = msg->k[0];
    double fy = msg->k[4];
    double cx = msg->k[2];
    double cy = msg->k[5];

    int deg =
        ((rotation_degrees_ % 360) + 360) % 360; // normalize to 0,90,180,270

    if (deg == 90) {
      // 90° clockwise: (x,y) -> (y, w-1-x)
      out.width = h;
      out.height = w;
      double new_fx = fy;
      double new_fy = fx;
      double new_cx = cy;
      double new_cy = static_cast<double>(w) - 1.0 - cx;
      out.k = {new_fx, 0, new_cx, 0, new_fy, new_cy, 0, 0, 1};
      // P matrix (assuming no stereo baseline changes needed for rotation)
      out.p = {new_fx, 0, new_cx, 0, 0, new_fy, new_cy, 0, 0, 0, 1, 0};
    } else if (deg == 270) {
      // 270° clockwise (= 90° CCW): (x,y) -> (h-1-y, x)
      out.width = h;
      out.height = w;
      double new_fx = fy;
      double new_fy = fx;
      double new_cx = static_cast<double>(h) - 1.0 - cy;
      double new_cy = cx;
      out.k = {new_fx, 0, new_cx, 0, new_fy, new_cy, 0, 0, 1};
      out.p = {new_fx, 0, new_cx, 0, 0, new_fy, new_cy, 0, 0, 0, 1, 0};
    } else if (deg == 180) {
      // 180°: (x,y) -> (w-1-x, h-1-y), dimensions stay same
      double new_cx = static_cast<double>(w) - 1.0 - cx;
      double new_cy = static_cast<double>(h) - 1.0 - cy;
      out.k = {fx, 0, new_cx, 0, fy, new_cy, 0, 0, 1};
      out.p = {fx, 0, new_cx, 0, 0, fy, new_cy, 0, 0, 0, 1, 0};
    }

    // Zero out distortion for rotated image (distortion model no longer valid
    // after rotation)
    out.distortion_model = "plumb_bob";
    out.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Update ROI
    out.roi.x_offset = 0;
    out.roi.y_offset = 0;
    out.roi.width = out.width;
    out.roi.height = out.height;
    out.roi.do_rectify = false;

    info_pub_->publish(out);
  }

  int rotate_code_;
  int rotation_degrees_;
  cv::Mat rotated_buf_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageRotateNode>());
  rclcpp::shutdown();
  return 0;
}
