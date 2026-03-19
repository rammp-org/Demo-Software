#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "../Constants/Constants.h"

class IMU_Class {
public:
  float pitch = 0.0, pitch_offset = 0.0, pitchrd = 0.0, roll = 0.0,
        rollrd = 0.0, yaw = 0.0;
  float pitchf = 0.0, rollf = 0.0;
  float am = 0.001;
  float K = 0.08;
  float ax = 0.0, ay = 0.0, az = 0.0;
  Adafruit_BNO055 &bno_sensor;
  
  imu::Quaternion current_quat;

  IMU_Class(Adafruit_BNO055 &);
  void initialize_BNO055_sensor();
  void retrieve_readings();
};

#endif
