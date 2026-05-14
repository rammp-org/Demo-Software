#header
std_msgs/Header header

# State Info
# State describes the current state / behavior of the robot:

# Create enum for the state field
uint8 STATE_INIT = 0
uint8 STATE_IDLE = 1
uint8 STATE_TUNER_MODE = 2
uint8 STATE_ESTOP = 3
uint8 STATE_SELF_LEVELING = 4
uint8 STATE_CONFIGURATION = 5
uint8 STATE_AUTO_CURB_CLIMBING = 6
uint8 STATE_CALIBRATING = 7
uint8 STATE_UNCALIBRATED = 8

uint8 state

# Position data from encoders
# FC - Front Caster
# RC - Rear Caster
# MR - Main Right
# ML - Main Left


# Vertical translation of the casters and main wheels
# Unit: Meters
float32 fc_pos
float32 rc_pos
float32 mr_pos
float32 ml_pos

# Forward/Backward translation of the casters and main wheels
# Unit: meters
float32 ml_carriage_pos
float32 mr_carriage_pos

# Wheel positions, in distance traveled by the wheel.
# Unit: Meters
float32 ml_wheel_pos
float32 mr_wheel_pos

# Velocities
float32 fc_vel
float32 rc_vel
float32 mr_vel
float32 ml_vel
float32 ml_carriage_vel
float32 mr_carriage_vel
float32 ml_wheel_vel
float32 mr_wheel_vel


#Loadcells
# Unit: TODO - confirm units
float32 rc_loadcell
float32 fc_loadcell
float32 mr_loadcell
float32 ml_loadcell



#IMU
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity
float64 tilt

#other
float64 app_time

# ODrive motor torque estimates (axis0.motor.torque_estimate), logical frame
# Unit: Newton-meters (Nm)
float32 odriveL_torque
float32 odriveR_torque
