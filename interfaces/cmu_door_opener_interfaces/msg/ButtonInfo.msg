# Detected button information from the vision pipeline.

# Unique identifier for the detected button
uint8 id

# Segmentation mask (single-channel, same resolution as input image)
sensor_msgs/Image segmentation_mask

# Bounding box in pixel coordinates [x_min, y_min, x_max, y_max]
int32[4] bounding_box

# YOLO detection confidence [0.0 - 1.0]
float32 confidence

# Button pose w.r.t. base_link: [x, y, z, roll, pitch, yaw]
# Default / invalid: all -1.0
float64[6] pose_xyzrpy

# Whether the arm can reach and press this button (IK solvable)
# TODO: Yucheng to provide Swap the exact IK method/function
bool is_pressable
