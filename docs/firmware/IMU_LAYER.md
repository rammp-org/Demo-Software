# IMU Layer

The `IMU_Class` (`src/IMU_Class/IMU_Class.cpp`) wraps the Adafruit BNO055 9-DOF IMU. Its primary responsibilities are:

1. Reading raw quaternion and accelerometer data from the sensor each loop cycle.
1. Converting quaternions to Euler angles while handling the BNO055's **upside-down physical mounting**.
1. Producing a **yaw-free (swing) quaternion** for use by the self-leveling kinematics to avoid fighting the robot's heading.
1. Applying an IIR low-pass filter to Euler angles with proper wraparound arithmetic.

## Class Overview

```
src/IMU_Class/
├── IMU_Class.h   — Public members and declarations
└── IMU_Class.cpp — initialize(), retrieve_readings(), extractSwing()
```

### Key Public Members

| Member                 | Type              | Description                                              |
| ---------------------- | ----------------- | -------------------------------------------------------- |
| `pitch`, `roll`, `yaw` | `float`           | Raw (unfiltered) Euler angles in degrees                 |
| `pitchf`, `rollf`      | `float`           | Low-pass filtered pitch and roll in degrees              |
| `pitchrd`, `rollrd`    | `float`           | Filtered pitch/roll converted to radians                 |
| `ax`, `ay`, `az`       | `float`           | Linear acceleration vector (m/s²)                        |
| `current_quat`         | `imu::Quaternion` | Swing quaternion (yaw removed) for kinematics            |
| `K`                    | `float`           | IIR filter coefficient for Euler angles (default `0.08`) |

______________________________________________________________________

## Initialization — `initialize_BNO055_sensor()` (Lines 5–19)

Called once from `Base.ino::setup()`. Initializes the BNO055 over I²C and blocks indefinitely if the sensor is not detected.

```cpp
if (!bno_sensor.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
}
delay(100);
```

> **Important:** Hardware axis remapping via the BNO055's built-in `setAxisRemap()` API was deliberately **removed**. It was found to scramble the quaternion axes in an unpredictable way. The upside-down mounting compensation is instead handled in software within `retrieve_readings()`.

______________________________________________________________________

## `retrieve_readings()` — Full Pipeline (Lines 21–84)

Called every `loop()` cycle. The full processing chain is:

```mermaid
flowchart TD
    BNO[BNO055 Sensor] -->|getQuat()| RawQ[Raw Quaternion]
    RawQ --> extractSwing["extractSwing(q)\n(remove yaw component)"]
    extractSwing --> CQ[current_quat\nused by self-leveling]

    RawQ --> EulerConv[Quaternion → Euler\naerospace sequence]
    EulerConv --> RawX[raw_x = Roll]
    EulerConv --> RawY[raw_y = Pitch]
    EulerConv --> RawZ[raw_z = Yaw]

    RawX -->|BNO X = Robot Pitch| PitchAssign[pitch = raw_x]
    RawY -->|BNO Y = Robot Roll| RollAssign

    RollAssign["roll = raw_y\n+ 180° offset\n(upside-down fix)"] --> LPF_Pitch
    PitchAssign --> LPF_Pitch["pitchf IIR filter\nwraparound-safe"]
    RollAssign --> LPF_Roll["rollf IIR filter\nwraparound-safe"]

    LPF_Pitch --> pitchf[pitchf / pitchrd]
    LPF_Roll --> rollf[rollf / rollrd]

    BNO -->|getVector ACCELEROMETER| Accel[ax, ay, az]
```

### Step 1 — Quaternion to Euler Conversion (Lines 29–57)

The BNO055 natively provides Euler angles, but the firmware computes them manually from the quaternion instead. This avoids the gimbal lock and ±180° discontinuity problems in the BNO055's on-chip Euler output.

The conversion uses the **standard aerospace (intrinsic ZYX) sequence**:

```cpp
// X-axis rotation (Roll)
double sinr_cosp = 2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
double cosr_cosp = 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
double raw_x = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI);

// Y-axis rotation (Pitch), with singularity guard
double sinp = 2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
double raw_y = (abs(sinp) >= 1) ? copysign(90, sinp) : asin(sinp) * (180.0 / PI);

// Z-axis rotation (Yaw)
double raw_z = atan2(...) * (180.0 / PI);
```

### Step 2 — Axis Reassignment

The BNO055's physical X, Y, Z axes do not directly correspond to the robot's Pitch, Roll, Yaw axes due to the sensor's orientation on the board:

```cpp
double raw_pitch = raw_x;  // BNO X-axis = Robot Pitch
double raw_roll  = raw_y;  // BNO Y-axis = Robot Roll
double raw_yaw   = raw_z;
```

### Step 3 — Upside-Down Mount Compensation (Lines 59–65)

The IMU is mounted upside down on the chassis. In this orientation, when the robot is resting flat, the BNO055 reports a Roll of approximately ±180° rather than 0°. Without correction, the self-leveling controller would try to command the robot to invert itself to reach a "flat" target of 0°.

The fix shifts the zero point by adding 180° and then re-wrapping to the \[-180, +180\] range:

```cpp
raw_roll += 180.0;
if (raw_roll > 180.0) raw_roll -= 360.0;
```

This moves the ±180° discontinuity to the position where the robot would be physically inverted — an unreachable state in normal operation — ensuring clean, continuous angle output around the flat resting position.

### Step 4 — IIR Low-Pass Filter with Wraparound (Lines 67–83)

A simple exponential moving average filter smooths the Euler angles. However, Euler angles wrap around at ±180°, so naive arithmetic (`pitchf + K * (pitch - pitchf)`) would snap through zero and cause a large transient whenever the angle crosses the ±180° boundary.

The solution computes the **shortest angular difference**:

```cpp
double diff_pitch = pitch - pitchf;
while (diff_pitch > 180.0)  diff_pitch -= 360.0;
while (diff_pitch < -180.0) diff_pitch += 360.0;
pitchf = pitchf + K * diff_pitch;
// Re-wrap pitchf to [-180, 180]
while (pitchf > 180.0)  pitchf -= 360.0;
while (pitchf < -180.0) pitchf += 360.0;
```

**`K = 0.08`** is a relatively small coefficient, producing heavy smoothing. At 200Hz loop rate this gives roughly a 1–2Hz effective bandwidth. This is appropriate for a gravity-level kinematics controller that should not react to high-frequency vibrations, but it means filtered angles will lag the true angle by tens of milliseconds during fast motion.

### Final outputs

```cpp
pitchrd = pitchf * (PI / 180.0);
rollrd  = -1.0 * rollf * (PI / 180.0);  // Note: rollrd sign is negated
```

`rollrd` is negated relative to `rollf`. This sign convention matches the self-leveling kinematics controller's coordinate frame expectations.

______________________________________________________________________

## `extractSwing()` — Yaw Decomposition (Lines 86–109)

This is the most mathematically sophisticated function in the IMU layer. It decomposes the measured quaternion into a **twist** (yaw rotation around the vertical axis) and a **swing** (the remaining tilt), then returns only the swing component.

### Why remove yaw?

The self-leveling controller tries to move the legs to keep the chassis level (flat pitch/roll). If it used the full quaternion including yaw, rotating the robot's heading would generate spurious tilt errors — the controller would misinterpret heading rotation as the robot tilting and incorrectly command the legs. By zeroing yaw, the same target quaternion works regardless of which direction the robot is facing.

### The Math

The **twist** around the vertical (Z) axis is isolated by projecting the full quaternion onto the Z-rotation subspace:

```cpp
float twist_norm = sqrt(q.w() * q.w() + q.z() * q.z());
```

This is the magnitude of the `(w, z)` component pair — the part of the quaternion responsible for rotation around Z.

The **swing** is what remains after the twist is factored out. For a decomposition along the Z axis, this simplifies algebraically to:

```cpp
float swing_w = twist_norm;
float swing_x = (q.w() * q.x() + q.z() * q.y()) / twist_norm;
float swing_y = (q.w() * q.y() - q.z() * q.x()) / twist_norm;
float swing_z = 0.0f;  // Yaw is completely zeroed
```

This is equivalent to computing `Inverse(Twist) * Measured` but without constructing the inverse explicitly — the algebra cancels to the above. The result is a unit quaternion representing the same pitch/roll tilt as the original measurement but with heading forced to zero.

### Singularity Guard

```cpp
if (twist_norm < 0.0001f) {
    return imu::Quaternion(1.0f, 0.0f, 0.0f, 0.0f); // Identity
}
```

`twist_norm` approaches zero only if the robot is pitched exactly 90° or more — i.e., nearly inverted. This guard prevents a divide-by-zero. In this edge case the function returns an identity quaternion (no rotation), which is a safe fallback to prevent the self-leveling controller from receiving NaN values and producing undefined PWM outputs.

______________________________________________________________________

## IMU Data in Telemetry

The following IMU fields are included in the 10Hz telemetry packet (see `docs/shared/SERIAL_PROTOCOL.md`):

| Telemetry Field        | Source Member      | Notes                                      |
| ---------------------- | ------------------ | ------------------------------------------ |
| `pitch`                | `IMU.pitchf`       | Filtered, 2 decimal places                 |
| `roll`                 | `IMU.rollf`        | Filtered, 2 decimal places                 |
| `yaw`                  | `IMU.yaw`          | Unfiltered raw yaw, 2 decimal places       |
| `ax`, `ay`, `az`       | `IMU.ax/ay/az`     | Linear acceleration, 3 decimal places      |
| `qw`, `qx`, `qy`, `qz` | `IMU.current_quat` | Swing quaternion (yaw=0), 4 decimal places |

> **Note:** The telemetry quaternion is the **swing quaternion** (yaw removed), not the raw BNO055 quaternion. The Python GUI's 3D IMU visualization is therefore yaw-stabilized — it shows chassis tilt but will not rotate with the robot's heading.
