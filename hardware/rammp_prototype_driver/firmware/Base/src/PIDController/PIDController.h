#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float kff, float min_out,
                float max_out, float scaling);

  // Compute PID output given setpoint, measured value, and dt
  // Feed-forward is applied directly to the setpoint
  float compute(float setpoint, float measured, float dt);

  void setGains(float kp, float ki, float kd);
  void setFeedForward(float kff);
  void setOutputLimits(float min_out, float max_out);
  void reset();

  float kp, ki, kd, kff;
  float min_out, max_out;
  float integral;
  float prev_error;
  float scaling;
};

#endif
