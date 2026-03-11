# Teensy Serial Protocol for PID Tuner

This document describes the serial communication protocol that must be implemented in `Base.ino` to work with the PID Tuner GUI application.

## Serial Configuration

- **Baud Rate:** 115200 (already configured in `setup()`)
- **Line Ending:** `\n` (newline)
- **Encoding:** ASCII

---

## Teensy -> PC: Encoder Data Output

### Format
```
ENC:<timestamp_ms>,<enc1>,<enc2>,<enc3>,<enc4>,<enc5>,<enc6>,<enc7>,<enc8>,<enc9>,<enc10>,<enc11>,<enc12>\n
```

### Fields
| Field | Description |
|-------|-------------|
| `timestamp_ms` | Milliseconds since Teensy boot (`millis()`) |
| `enc1` | Encoder 1: RC Top (signed long) |
| `enc2` | Encoder 2: FC Bottom (signed long) |
| `enc3` | Encoder 3: RC Bottom (signed long) |
| `enc4` | Encoder 4: FC Top (signed long) |
| `enc5` | Encoder 5: MR Back (signed long) |
| `enc6` | Encoder 6: ML Front (signed long) |
| `enc7` | Encoder 7: ML Back (signed long) |
| `enc8` | Encoder 8: MR Front (signed long) |
| `enc9` | Encoder 9: ML Drive Wheel (signed long) |
| `enc10` | Encoder 10: MR Drive Wheel (signed long) |
| `enc11` | Encoder 11: ML Carriage (signed long) |
| `enc12` | Encoder 12: MR Carriage (signed long) |

### Example Output
```
ENC:12345,100,-50,320,0,1500,-1200,800,900,15000,-15200,12000,-12500
```

### Implementation in Base.ino

Replace or modify the `displaydata()` function:

```cpp
void displaydata() {
  // Output encoder data for PID Tuner
  Serial.print("ENC:");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(EContr.encoder[1]);   // RC top
  Serial.print(",");
  Serial.print(EContr.encoder[2]);   // FC bottom
  Serial.print(",");
  Serial.print(EContr.encoder[3]);   // RC bottom
  Serial.print(",");
  Serial.print(EContr.encoder[4]);   // FC top
  Serial.print(",");
  Serial.print(EContr.encoder[5]);   // MR back
  Serial.print(",");
  Serial.print(EContr.encoder[6]);   // ML front
  Serial.print(",");
  Serial.print(EContr.encoder[7]);   // ML back
  Serial.print(",");
  Serial.print(EContr.encoder[8]);   // MR front
  Serial.print(",");
  Serial.print(EContr.encoder[9]);   // ML drive wheel
  Serial.print(",");
  Serial.print(EContr.encoder[10]);  // MR drive wheel
  Serial.print(",");
  Serial.print(EContr.encoder[11]);  // ML carriage
  Serial.print(",");
  Serial.println(EContr.encoder[12]); // MR carriage (println adds \n)
}
```

---

## PC -> Teensy: Commands

### 1. Set Target Position

**Format:**
```
T<joint_id>:<target_ticks>\n
```

**Fields:**
| Field | Description |
|-------|-------------|
| `joint_id` | Joint number 1-12 |
| `target_ticks` | Target position in encoder ticks (signed integer) |

**Example:**
```
T7:1500
```
Sets joint 7 (ML Back) target to 1500 ticks.

### 2. Step Input (Relative Change)

**Format:**
```
S<joint_id>:<step_ticks>\n
```

**Fields:**
| Field | Description |
|-------|-------------|
| `joint_id` | Joint number 1-12 |
| `step_ticks` | Step size in ticks (signed, can be negative) |

**Example:**
```
S7:100
```
Add 100 ticks to joint 7's current target.

```
S7:-100
```
Subtract 100 ticks from joint 7's current target.

### 3. Start Sine Wave

**Format:**
```
W<joint_id>:<amplitude>,<frequency_hz>,<duration_s>\n
```

**Fields:**
| Field | Description |
|-------|-------------|
| `joint_id` | Joint number 1-12 |
| `amplitude` | Amplitude in encoder ticks |
| `frequency_hz` | Frequency in Hz (e.g., 0.500) |
| `duration_s` | Duration in seconds (e.g., 10.0) |

**Example:**
```
W7:500,0.500,10.0
```
Start a sine wave on joint 7 with 500 tick amplitude, 0.5 Hz frequency, for 10 seconds.

**Note:** The Python GUI generates sine wave targets locally and sends `T` commands at ~50Hz. This command is provided for firmware-side sine generation if preferred.

### 4. Stop Sine Wave

**Format:**
```
X<joint_id>\n
```

**Example:**
```
X7
```
Stop any sine wave on joint 7.

### 5. Disable Motors (Emergency Stop)

**Format:**
```
z\n
```

**Description:**
Sends the 'z' command which triggers `NO_MOVEMENT()` in the existing firmware, stopping all motor outputs. This is the same as the existing 'z' command in the firmware.

**Example:**
```
z
```
Stop all motors immediately.

---

## Implementation Example for Base.ino

Add this to the `get_GUI_input_from_serial()` function or create a new parser:

```cpp
// Global variables for target positions (add near top of file)
signed long joint_targets[13] = {0}; // Index 1-12 for joints

void parse_pid_tuner_command() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() < 2) return;
    
    char cmd = input.charAt(0);
    
    switch (cmd) {
      case 'T': {
        // Set Target: T<joint>:<ticks>
        int colonIdx = input.indexOf(':');
        if (colonIdx > 1) {
          int joint = input.substring(1, colonIdx).toInt();
          long target = input.substring(colonIdx + 1).toInt();
          if (joint >= 1 && joint <= 12) {
            joint_targets[joint] = target;
            // Apply target to appropriate component
            apply_target_to_joint(joint, target);
          }
        }
        break;
      }
      
      case 'S': {
        // Step Input: S<joint>:<step>
        int colonIdx = input.indexOf(':');
        if (colonIdx > 1) {
          int joint = input.substring(1, colonIdx).toInt();
          long step = input.substring(colonIdx + 1).toInt();
          if (joint >= 1 && joint <= 12) {
            joint_targets[joint] += step;
            apply_target_to_joint(joint, joint_targets[joint]);
          }
        }
        break;
      }
      
      // Handle other commands (W, X) if implementing firmware-side sine
      
      default:
        // Fall back to existing single-character command handling
        action = cmd;
        break;
    }
  }
}

void apply_target_to_joint(int joint, long target_ticks) {
  // Map joint numbers to component targets
  // You'll need to implement the conversion from ticks to your units
  switch (joint) {
    case 1: // RC Top
      // Convert ticks to RC position units
      break;
    case 2: // FC Bottom
      break;
    case 3: // RC Bottom
      break;
    case 4: // FC Top
      break;
    case 5: // MR Back
      break;
    case 6: // ML Front
      break;
    case 7: // ML Back
      break;
    case 8: // MR Front
      break;
    case 9: // ML Drive Wheel
      break;
    case 10: // MR Drive Wheel
      break;
    case 11: // ML Carriage
      ML.carriage.des = convert_ticks_to_carriage_pos(target_ticks);
      break;
    case 12: // MR Carriage
      MR.carriage.des = convert_ticks_to_carriage_pos(target_ticks);
      break;
  }
}
```

---

## Joint Mapping Reference

| Joint ID | Encoder Index | Variable | Description |
|----------|---------------|----------|-------------|
| 1 | encoder[1] | Enc2 | RC (Rear Caster) Top |
| 2 | encoder[2] | Enc4 | FC (Front Caster) Bottom |
| 3 | encoder[3] | Enc1 | RC Bottom (0-850) |
| 4 | encoder[4] | Enc3 | FC Top |
| 5 | encoder[5] | Enc12 | MR (Main Right) Back |
| 6 | encoder[6] | Enc6 | ML (Main Left) Front |
| 7 | encoder[7] | Enc11 | ML Back |
| 8 | encoder[8] | Enc10 | MR Front |
| 9 | encoder[9] | Enc5 | ML Drive Wheel |
| 10 | encoder[10] | Enc8 | MR Drive Wheel |
| 11 | encoder[11] | Enc7 | ML Carriage |
| 12 | encoder[12] | Enc9 | MR Carriage |

---

## Important Notes

1. **Output Rate:** The `displaydata()` function is called every loop iteration (~200Hz with 5ms delay). This provides good data for PID tuning visualization.

2. **Raw Ticks:** All values should be in raw encoder ticks (signed long) - no conversion to cm or other units.

3. **Existing Commands:** The protocol is designed to be backward-compatible. Single-character commands (like '1', '2', 'q', 'a', etc.) will still work for existing functionality.

4. **Thread Safety:** The GUI sends commands at reasonable rates and waits for line completion, so no special buffering is required beyond Arduino's default.

5. **Testing:** You can test the output format using the Arduino Serial Monitor before connecting the GUI.
