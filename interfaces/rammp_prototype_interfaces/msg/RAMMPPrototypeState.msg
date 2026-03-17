#header
std_msgs/Header header

#encoders
float64 fc_enc
float64 fr_enc
float64 mr_enc
float64 ml_enc
float64 ml_carr_enc
float64 mr_carr_enc
float64 ml_wheel_enc
float64 mr_wheel_enc

#loacells
float64 fc_lc
float64 mr_lc
float64 ml_lc

#states
int64 ca_flag
string action

#VelAcc
float64 ml_vel
float64 mr_vel
float64 ml_acc
float64 mr_acc

#IMU
float64 pitch
float64 roll
float64 ax
float64 ay
float64 az
float64 tilt

#other
float64 app_time
float64 measure_height
