#include "IMU_Class.h"

IMU_Class::IMU_Class(Adafruit_BNO055 &bno_sense) : bno_sensor(bno_sense){};

void IMU_Class::initialize_BNO055_sensor() {
  /* Initialise BNO */
  if (!bno_sensor.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print(
        "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
}

void IMU_Class::retrieve_readings() {
  // Get quaternion data to avoid Euler angle discontinuities (gimbal lock / wraparound)
  imu::Quaternion quat = bno_sensor.getQuat();

  // Convert quaternion to Euler angles manually
  // This uses the standard aerospace sequence
  
  // X-axis rotation
  double sinr_cosp = 2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
  double cosr_cosp = 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
  double raw_x = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

  // Y-axis rotation
  double sinp = 2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
  double raw_y;
  if (abs(sinp) >= 1)
    raw_y = copysign(M_PI / 2, sinp) * (180.0 / PI); // use 90 degrees if out of range
  else
    raw_y = asin(sinp) * (180.0 / PI);

  // Z-axis rotation
  double siny_cosp = 2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
  double cosy_cosp = 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());
  double raw_z = atan2(siny_cosp, cosy_cosp) * (180.0 / PI);

  // Get linear acceleration
  imu::Vector<3> accel =
      bno_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ax = accel.x();
  ay = accel.y();
  az = accel.z();

  // Map axes to match legacy getVector(VECTOR_EULER) behavior
  // where euler.z() was pitch, euler.y() was roll, euler.x() was yaw
  double raw_pitch = raw_z;
  double raw_roll = raw_y;
  double raw_yaw = raw_x;

  // Apply offsets (matching legacy behavior)
  // IMU.pitch_offset was 5.0 (2.0 + 3.0)
  pitch = raw_pitch - 5.0;
  roll = raw_roll + 1.5;
  yaw = raw_yaw;

  // Apply low pass filter
  pitchf = pitchf + K * (pitch - pitchf);
  rollf = rollf + K * (roll - rollf);

  pitchrd = pitchf * (PI / 180.0);
  rollrd = -1.0 * rollf * (PI / 180.0);
}
