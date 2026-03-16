# Closed-Loop Motor Control Bringup Plan

## Overview

This document outlines a comprehensive plan for bringing up the motor control system from basic PWM output to full closed-loop position control with IMU and limit switch integration. The plan is structured in phases, each building on the previous, with clear testing criteria before proceeding.

**Hardware**: Teensy 4.1 + 3x RoboClaw motor drivers + 6 motors + 12 quadrature encoders + BNO055 IMU + limit switches

**Software**: Arduino framework firmware (`base.ino`) + Python PID tuner GUI (`pid_tuner/`)

---

## Phase 0: Critical Bug Fixes (Pre-requisite)

These issues must be fixed before any testing can begin.

### 0.1 Joint ID Indexing Mismatch

**Problem**: Firmware uses 0-indexed actuator IDs (0-5), but GUI sends 1-indexed joint IDs (1-6).

**Location**: `base.ino:167-174`

**Current Code**:
```cpp
switch(cmd.actuator_id) {
    case 0: m = &rc; break;
    case 1: m = &fc; break;
    // ...
}
```

**Fix Options**:
1. **(Recommended)** Modify firmware to accept 1-indexed IDs by subtracting 1:
   ```cpp
   int motor_idx = cmd.actuator_id - 1;  // Convert 1-6 to 0-5
   switch(motor_idx) {
       case 0: m = &rc; break;
       // ...
   }
   ```
2. Modify GUI to send 0-indexed IDs (requires changes to `protocol.py` and `joint_config.py`)

**Testing**: Send `M1:0` from GUI, verify DEBUG output shows "Set Mode to 0" for motor index 0 (rc).

---

### 0.2 PWM Output Scaling

**Problem**: PID controllers output normalized values [-1.0, 1.0], but RoboClaw `DutyM1/M2` expects int16 values [-32767, 32767]. Current code passes the raw float, resulting in near-zero duty cycles.

**Location**: `base.ino:221-230`

**Current Code**:
```cpp
roboclaw_main.DutyM1(0x80, (int16_t)constrain(ml_pwm, -32767, 32767));
```

**Fix**: Scale the PWM output by 32767 before sending:
```cpp
#define PWM_SCALE 32767.0f

int16_t scaled_ml_pwm = (int16_t)constrain(ml_pwm * PWM_SCALE, -32767, 32767);
roboclaw_main.DutyM1(0x80, scaled_ml_pwm);
```

**Alternative**: Change PID output limits to [-32767, 32767] directly in `Motor::initPIDs()`.

**Testing**: 
1. Set motor to open loop: `M1:0`
2. Set target PWM: `T1:0.5` (should be 50% duty)
3. Verify motor spins at moderate speed

---

### 0.3 Watchdog Timeout Adjustment

**Problem**: 60-second timeout is dangerously long for a safety-critical system.

**Location**: `base.ino:50`

**Current Code**:
```cpp
CommandParser parser(60000);  // 60 second timeout
```

**Fix**: Reduce to 500ms as originally specified:
```cpp
CommandParser parser(500);  // 500ms timeout
```

**GUI Consideration**: Ensure GUI sends heartbeat commands at least every 250ms. The GUI should send periodic `c` (clear ESTOP) or implement a dedicated heartbeat command.

**Testing**: 
1. Connect GUI and send commands
2. Disconnect USB cable
3. Verify ESTOP triggers within ~500ms (motors stop)

---

## Phase 1: Open-Loop PWM Verification

**Goal**: Verify basic motor control works from GUI to RoboClaw.

### 1.1 Hardware Checklist

- [ ] Verify Serial3/4/5 wiring to RoboClaws (TX/RX, correct baud rate)
- [ ] Verify RoboClaw addresses are all 0x80 (or update firmware if different)
- [ ] Verify RoboClaw baud rate is 460800 (or update firmware)
- [ ] Verify power connections to RoboClaws
- [ ] Verify motor wiring to RoboClaw M1/M2 terminals

### 1.2 Motor-to-RoboClaw Mapping Verification

Document and verify the physical wiring matches firmware expectations:

| Motor Object | RoboClaw | Channel | Serial Port |
|--------------|----------|---------|-------------|
| `ml`         | main     | M1      | Serial5     |
| `mr`         | main     | M2      | Serial5     |
| `rc`         | casters  | M1      | Serial4     |
| `fc`         | casters  | M2      | Serial4     |
| `ml_carriage`| carriages| M1      | Serial3     |
| `mr_carriage`| carriages| M2      | Serial3     |

### 1.3 Open-Loop Test Procedure

1. Power on system with motors free to spin (wheels off ground or disconnected)
2. Connect GUI to Teensy
3. Select Joint 1 (RC)
4. Set Mode to "Open Loop (0)"
5. Set Target to small value (e.g., 0.1 = 10% duty)
6. Verify RC motor spins
7. Set Target to -0.1
8. Verify RC motor spins in opposite direction
9. Repeat for all 6 motors

### 1.4 Success Criteria

- [ ] All 6 motors respond to open-loop PWM commands
- [ ] Direction control works (positive/negative PWM)
- [ ] ESTOP (`z`) immediately stops all motors
- [ ] Clear ESTOP (`c`) re-enables control

---

## Phase 2: Encoder Verification & Velocity Estimation

**Goal**: Verify encoder feedback is working and implement velocity estimation.

### 2.1 Encoder Mapping Verification

**Problem**: The encoder-to-motor mapping is uncertain (see TODO at `base.ino:124`).

**Current Mapping (needs verification)**:
```cpp
rc.updateSensorData(EContr.encoderf[3], ...);   // RC bottom
fc.updateSensorData(EContr.encoderf[2], ...);   // FC bottom  
ml.updateSensorData(EContr.encoderf[7], ...);   // ML back
mr.updateSensorData(EContr.encoderf[5], ...);   // MR back
ml_carriage.updateSensorData(EContr.encoderf[11], ...);
mr_carriage.updateSensorData(EContr.encoderf[12], ...);
```

**Verification Procedure**:
1. Connect to Serial Monitor
2. For each motor:
   a. Manually rotate the motor shaft
   b. Observe which `encoderf[x]` value changes in telemetry
   c. Document the correct mapping
3. Update `base.ino` with verified mappings

**EncoderContainer Reference** (`EncoderContainer.h`):
| Index | Encoder | Physical Location |
|-------|---------|-------------------|
| 1     | Enc2    | RC top            |
| 2     | Enc4    | FC bottom         |
| 3     | Enc1    | RC bottom         |
| 4     | Enc3    | FC top            |
| 5     | Enc12   | MR back           |
| 6     | Enc6    | ML front          |
| 7     | Enc11   | ML back           |
| 8     | Enc10   | MR front          |
| 9     | Enc5    | ML drive wheel    |
| 10    | Enc8    | MR drive wheel    |
| 11    | Enc7    | ML carriage       |
| 12    | Enc9    | MR carriage       |

### 2.2 Velocity Estimation Implementation

**Problem**: All motors receive `0.0f` for velocity feedback, breaking velocity and position control.

**Location**: `base.ino:125-130`

**Fix**: Calculate velocity from position change over time.

**Option A - Simple Differentiation** (in Motor class):
```cpp
// In Motor.h, add:
float prev_pos;
float estimated_vel;

// In Motor::updateSensorData():
void Motor::updateSensorData(float current_pos, float dt) {
    if (dt > 0.0f) {
        this->current_vel = (current_pos - this->prev_pos) / dt;
    }
    this->prev_pos = current_pos;
    this->current_pos = current_pos;
}
```

**Option B - Filtered Velocity** (with low-pass filter):
```cpp
void Motor::updateSensorData(float current_pos, float dt) {
    if (dt > 0.0f) {
        float raw_vel = (current_pos - this->prev_pos) / dt;
        // Low-pass filter: alpha = 0.2 for smoothing
        this->current_vel = this->current_vel + 0.2f * (raw_vel - this->current_vel);
    }
    this->prev_pos = current_pos;
    this->current_pos = current_pos;
}
```

**Update in base.ino**:
```cpp
// In loop(), after timer.updateTime():
float dt = timer.elapsed_time;

// Update motor sensor data with dt for velocity calculation
rc.updateSensorData(EContr.encoderf[3], dt);
fc.updateSensorData(EContr.encoderf[2], dt);
// ... etc
```

### 2.3 Telemetry Velocity Verification

After implementing velocity estimation:
1. Spin a motor using open-loop control
2. Observe velocity values in GUI telemetry
3. Verify velocity is non-zero and approximately correct direction

### 2.4 Success Criteria

- [ ] Each motor's encoder correctly tracks its position
- [ ] Encoder direction matches motor direction (positive PWM = positive encoder change)
- [ ] Velocity estimation produces sensible values during motion
- [ ] Telemetry shows position and velocity updating in GUI

---

## Phase 3: Velocity Control Loop

**Goal**: Achieve stable velocity control using the velocity PID.

### 3.1 Prerequisites

- Phase 2 complete (velocity feedback working)
- PWM scaling fixed (Phase 0.2)

### 3.2 Velocity PID Tuning Procedure

Start with one motor (recommend ML or MR main drive for simplicity):

1. Set initial conservative gains:
   ```
   Vel P: 0.001
   Vel I: 0.0
   Vel D: 0.0
   ```

2. Set mode to Velocity Control: `M3:1` (for ML, joint 3)

3. Set target velocity: `T3:100` (encoder ticks/second - start small)

4. Observe response:
   - If no movement: Increase P gain
   - If oscillating: Decrease P gain, add D
   - If steady-state error: Add small I gain

5. Tune until stable velocity tracking is achieved

### 3.3 Velocity Control Verification

Test each motor:
1. Command constant velocity
2. Verify motor reaches target velocity
3. Apply light load (hand resistance)
4. Verify controller compensates (I term working)

### 3.4 Success Criteria

- [ ] All 6 motors achieve stable velocity control
- [ ] Velocity error settles to <5% within 1 second
- [ ] No oscillation at steady state
- [ ] Controller handles reasonable load disturbances

---

## Phase 4: Position Control Loop

**Goal**: Achieve stable cascaded position control (Position PID -> Velocity PID -> PWM).

### 4.1 Prerequisites

- Phase 3 complete (velocity control working)

### 4.2 Cascaded Control Architecture

The Motor class implements cascaded control:
```
Position Setpoint -> [Position PID] -> Velocity Setpoint -> [Velocity PID] -> PWM
```

This is already implemented in `Motor::update()`:
```cpp
case POSITION_CONTROL:
    target_vel = pos_pid.compute(target_pos, current_pos, dt);
    // Fallthrough to velocity control
case VELOCITY_CONTROL:
    target_pwm = vel_pid.compute(target_vel, current_vel, dt);
```

### 4.3 Position PID Tuning Procedure

With velocity PID already tuned:

1. Set initial conservative position gains:
   ```
   Pos P: 0.1
   Pos I: 0.0
   Pos D: 0.0
   ```

2. Set mode to Position Control: `M3:2`

3. Set target position: `T3:1000` (encoder ticks)

4. Observe step response:
   - Overshoot? Add D or reduce P
   - Slow response? Increase P
   - Steady-state error? Add small I

5. Use GUI sine wave function to test tracking

### 4.4 Position Control Verification

For each motor:
1. Command step input (+500 ticks)
2. Measure settling time and overshoot
3. Command sine wave (0.5 Hz, reasonable amplitude)
4. Verify smooth tracking without phase lag

### 4.5 Success Criteria

- [ ] All 6 motors achieve stable position control
- [ ] Step response: <10% overshoot, <500ms settling time
- [ ] Sine wave tracking: <5% amplitude error, <50ms phase lag
- [ ] Position holds under external disturbance

---

## Phase 5: IMU Integration

**Goal**: Initialize and read IMU data for self-leveling applications.

### 5.1 Enable IMU Initialization

**Location**: `base.ino:46-47`

**Current Code**:
```cpp
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// IMU_Class IMU = IMU_Class(bno);  // Commented out
```

**Fix**:
1. Uncomment IMU class
2. Add IMU initialization in `setup()`:
   ```cpp
   if (!bno.begin()) {
       Serial.println("ERROR: BNO055 not detected");
       // Handle error - maybe set a flag, don't block
   }
   bno.setExtCrystalUse(true);
   ```

### 5.2 IMU Data in Telemetry

Add pitch, roll, yaw to telemetry struct and output:
```cpp
// In updateTelemetry():
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
telemetry.imu_yaw = euler.x();
telemetry.imu_pitch = euler.y();
telemetry.imu_roll = euler.z();
```

Update `sendTelemetry()` to include IMU values.

### 5.3 GUI Protocol Update

Update `protocol.py` to parse extended telemetry with IMU data.

### 5.4 Success Criteria

- [ ] IMU initializes without blocking system
- [ ] Pitch/roll/yaw values appear in telemetry
- [ ] Values are reasonable (0 when level, change when tilted)
- [ ] IMU data rate doesn't slow down main loop significantly

---

## Phase 6: Limit Switch Integration

**Goal**: Add limit switch reading for safe carriage operation and homing.

### 6.1 Hardware Setup

- [ ] Identify limit switch GPIO pins on Teensy
- [ ] Wire normally-closed limit switches (safer failure mode)
- [ ] Add pull-up resistors if needed

### 6.2 Limit Switch Reading

Add to firmware:
```cpp
// Pin definitions
#define ML_CARRIAGE_LIMIT_PIN 33
#define MR_CARRIAGE_LIMIT_PIN 34

// In setup():
pinMode(ML_CARRIAGE_LIMIT_PIN, INPUT_PULLUP);
pinMode(MR_CARRIAGE_LIMIT_PIN, INPUT_PULLUP);

// In loop():
bool ml_limit_hit = !digitalRead(ML_CARRIAGE_LIMIT_PIN);  // Active low
bool mr_limit_hit = !digitalRead(MR_CARRIAGE_LIMIT_PIN);
```

### 6.3 Limit Switch Safety Logic

Implement software limits:
```cpp
// Before sending carriage PWM:
if (ml_limit_hit && mlc_pwm < 0) {
    mlc_pwm = 0;  // Prevent further motion in limit direction
}
```

### 6.4 Homing Procedure

Implement a homing state/command:
1. Move carriage slowly toward limit switch
2. When limit triggers, stop and zero encoder
3. Move off limit switch slightly
4. Set position reference

### 6.5 Success Criteria

- [ ] Limit switches correctly read (test with manual actuation)
- [ ] Motor stops when limit is reached
- [ ] Homing procedure works reliably
- [ ] Limit status included in telemetry

---

## Phase 7: Self-Leveling Mode

**Goal**: Implement closed-loop self-leveling using IMU feedback.

### 7.1 Prerequisites

- Phase 4 complete (position control working on carriages)
- Phase 5 complete (IMU data available)

### 7.2 Self-Leveling Algorithm

Basic proportional leveling:
```cpp
// In SELF_LEVELING state:
float pitch_error = 0.0f - telemetry.imu_pitch;  // Target = level

// Map pitch error to carriage position adjustment
float carriage_adjustment = pitch_error * LEVELING_GAIN;

// Apply to both carriages (opposite directions for pitch)
ml_carriage.setTargetPosition(ml_carriage.current_pos + carriage_adjustment);
mr_carriage.setTargetPosition(mr_carriage.current_pos - carriage_adjustment);
```

### 7.3 State Machine Integration

Add transition to SELF_LEVELING state:
```cpp
case CMD_L:  // New command for leveling mode
    if (current_state == IDLE) {
        current_state = SELF_LEVELING;
    }
    break;
```

### 7.4 Success Criteria

- [ ] Robot maintains level when placed on slope
- [ ] Responds to dynamic disturbances (manual tilting)
- [ ] Carriage motion is smooth, not jerky
- [ ] Can exit self-leveling mode cleanly

---

## Appendix A: Command Protocol Summary

### PC -> Teensy Commands

| Command | Format | Description |
|---------|--------|-------------|
| Set Target | `T<id>:<value>\n` | Set target (mode-dependent) |
| Set Mode | `M<id>:<mode>\n` | 0=Open Loop, 1=Velocity, 2=Position |
| Position P | `P<id>:<value>\n` | Set position Kp |
| Position I | `I<id>:<value>\n` | Set position Ki |
| Position D | `D<id>:<value>\n` | Set position Kd |
| Velocity P | `p<id>:<value>\n` | Set velocity Kp |
| Velocity I | `i<id>:<value>\n` | Set velocity Ki |
| Velocity D | `d<id>:<value>\n` | Set velocity Kd |
| ESTOP | `z\n` | Emergency stop |
| Clear ESTOP | `c\n` | Clear ESTOP, enter IDLE |

### Teensy -> PC Telemetry

```
TELEMETRY,<timestamp>,<state>,<rc_pos>,<fc_pos>,<ml_pos>,<mr_pos>,<mlc_pos>,<mrc_pos>,<rc_vel>,<fc_vel>,<ml_vel>,<mr_vel>,<mlc_vel>,<mrc_vel>,<rc_pwm>,<fc_pwm>,<ml_pwm>,<mr_pwm>,<mlc_pwm>,<mrc_pwm>\n
```

---

## Appendix B: Testing Checklist

### Pre-Flight Checklist
- [ ] All motor connections secure
- [ ] Power supply adequate (check voltage under load)
- [ ] Emergency stop accessible
- [ ] Wheels/joints free to move safely
- [ ] Serial connections verified

### Phase Completion Sign-off

| Phase | Status | Date | Notes |
|-------|--------|------|-------|
| 0.1 Joint ID Fix | | | |
| 0.2 PWM Scaling | | | |
| 0.3 Watchdog | | | |
| 1 Open-Loop PWM | | | |
| 2 Encoders & Velocity | | | |
| 3 Velocity Control | | | |
| 4 Position Control | | | |
| 5 IMU Integration | | | |
| 6 Limit Switches | | | |
| 7 Self-Leveling | | | |

---

## Appendix C: Troubleshooting

### Motor Doesn't Respond
1. Check RoboClaw power LED
2. Verify serial TX/RX not swapped
3. Check RoboClaw address (default 0x80)
4. Verify baud rate match (460800)
5. Check motor wiring polarity

### Encoder Reads Zero
1. Check encoder wiring (A, B, +5V, GND)
2. Verify correct Teensy pins
3. Try manually rotating motor shaft
4. Check for broken encoder cable

### PID Oscillation
1. Reduce P gain by 50%
2. Add D gain (start small, 10% of P)
3. Reduce I gain or add anti-windup
4. Check for mechanical backlash

### Communication Timeout
1. Verify USB connection
2. Check baud rate (115200)
3. Ensure GUI is sending heartbeat
4. Check for serial buffer overflow

---

*Document Version: 1.0*
*Created: 2025*
*Last Updated: Initial creation*
