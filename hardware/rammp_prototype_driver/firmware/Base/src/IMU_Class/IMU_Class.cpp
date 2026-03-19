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
  
  delay(100);
  
  // Removed hardware axis remap because it scrambles the quaternion axes.
  // We will handle the upside-down mounting in software below.
}

void IMU_Class::retrieve_readings() {
  // Get quaternion data to avoid Euler angle discontinuities (gimbal lock / wraparound)
  imu::Quaternion quat = bno_sensor.getQuat();
  current_quat = quat; // Expose raw unfiltered quaternion for self-leveling kinematics

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

  // set pitch roll and yaw to the encoder axes expected
  double raw_pitch = raw_x;
  double raw_roll = raw_y;
  double raw_yaw = raw_z;

  // Software fix for upside-down mounting (moves the +/-180 discontinuity away from the flat resting position)
  raw_roll += 180.0;
  if (raw_roll > 180.0) raw_roll -= 360.0;

  pitch = raw_pitch;
  roll = raw_roll;
  yaw = raw_yaw;

  // Apply low pass filter using shortest path for continuous angles
  double diff_pitch = pitch - pitchf;
  while (diff_pitch > 180.0) diff_pitch -= 360.0;
  while (diff_pitch < -180.0) diff_pitch += 360.0;
  pitchf = pitchf + K * diff_pitch;
  while (pitchf > 180.0) pitchf -= 360.0;
  while (pitchf < -180.0) pitchf += 360.0;

  double diff_roll = roll - rollf;
  while (diff_roll > 180.0) diff_roll -= 360.0;
  while (diff_roll < -180.0) diff_roll += 360.0;
  rollf = rollf + K * diff_roll;
  while (rollf > 180.0) rollf -= 360.0;
  while (rollf < -180.0) rollf += 360.0;

  pitchrd = pitchf * (PI / 180.0);
  rollrd = -1.0 * rollf * (PI / 180.0);
}
