# Guarded Movement: Load-Cell-Gated Trajectory Truncation

## TL;DR

> **Quick Summary**: Add closed-loop load cell guards to the firmware sequence player so that when a motor's localized strain gauge crosses a threshold (contact or clearance), its trajectory is asynchronously truncated and latched at the current position while remaining motors complete their interpolations. Includes full ROS driver protocol expansion.
>
> **Deliverables**:
> - Motor class with optional StrainGauge pointer and `current_load` member
> - GuardCondition enum and extended Keyframe struct in SequencePlayer
> - Guard-aware interpolation (Phase 1) and settling (Phase 2) in sequenceUpdate
> - 48-value payload parser (backward compatible with 32-value and 17-value legacy)
> - Python Keyframe model + ProtocolEncoder expansion (ROS driver + PID tuner)
> - SEQ_GUARD_TRIG telemetry parsing on the ROS side
> - Example guarded sequence JSON for testing
>
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 3 waves
> **Critical Path**: T1 → T4 → T5 → T7 | T2 → T5 → T7 | T3 → T6 → T7

---

## Context

### Original Request
Implement a "guarded movement" feature on the ML and MR motors (generalized to all 4 load-cell-equipped motors: RC, FC, ML, MR) that allows them to transition between "grounded" and "clearance" states by monitoring load cells. The sequence player must asynchronously truncate a motor's trajectory when its load cell crosses a directional threshold, latching its current position as the new final target.

### Interview Summary
**Key Discussions**:
- **Motor-StrainGauge coupling**: Pointer-based. Motor holds `StrainGauge*`, call `attachStrainGauge()` in setup. Motors without gauges keep nullptr and current_load=0.0.
- **Guard scope**: All 4 load-cell-equipped motors (rc, fc, ml, mr) get StrainGauge pointers. The feature is generic across all 8 motor slots but only 4 have physical sensors.
- **Guard direction**: GuardCondition enum (NONE, GREATER_THAN, LESS_THAN) decouples the comparison operator from the threshold value, enabling both contact detection (load > N) and clearance detection (load < N).
- **Payload format**: 48-value format = 6 blocks (targets, active, relative, durations, guard_thresholds, guard_conditions). Backward compatible with 32-value and 17-value legacy.
- **ROS driver**: In scope — Python Keyframe class, ProtocolEncoder, MEBot_control_node, and ProtocolParser all need updates.

### Research Findings
- **Motor class** (Motor.h/Motor.cpp): Has `current_pos`, `current_vel`, `target_pos` but NO `current_load`. Cascaded PID: position → velocity → PWM.
- **StrainGauge class** (StrainGauge.h/StrainGauge.cpp): `update(dt)`, `getValue()`, IIR LPF with alpha=0.8. Returns filtered ADC counts.
- **Sequence Player**: SEQ_NUM_MOTORS=8, MAX_SEQ_KEYFRAMES=20. Keyframe has 4 fields (targets, active, relative, duration_ms). Parser handles 32 or 17 values. Phase 1 = linear interpolation with per-motor timing. Phase 2 = settling with per-motor deadzones (50/500/2000) and 10s timeout.
- **Motor map**: rc=0, fc=1, ml=2, mr=3, ml_carriage=4, mr_carriage=5, drive_fb=6, drive_lr=7.
- **Strain gauges**: sg_rc(A16,0.8), sg_fc(A17,0.8), sg_ml(A15,0.8), sg_mr(A14,0.8). Updated every 5ms cycle. Currently no coupling to motor control.
- **ROS driver**: `ProtocolEncoder.send_keyframe()` builds 32-value CSV. `MEBot_control_node.send_sequence()` iterates keyframes. Python `Keyframe` class in keyframe.py (two copies: ROS driver + PID tuner).
- **JSON sequences**: `config/curb_ascending.json`, `curb_descending.json`, etc. Use Keyframe.from_dict format.

### Gap Analysis (Self-Performed — Metis Timed Out)
**Identified Gaps (addressed in plan)**:
- **Load units**: StrainGauge returns filtered ADC counts, not calibrated force. Thresholds will be in ADC units. Calibration is out of scope.
- **SEQ_GUARD_TRIG parsing**: Firmware emits these as inline serial messages. ROS ProtocolParser doesn't handle them. Added as part of T6.
- **PID tuner keyframe.py**: Duplicate copy at `hardware/pid_tuner/serial_driver/keyframe.py` needs same guard field additions. Included in T3.
- **Backward compatibility**: Existing JSON sequences have no guard fields. `from_dict` defaults them to 0.0/GUARD_NONE. Firmware parser falls back on 32/17 value formats.
- **Null strain gauge safety**: Motors without StrainGauge pointer keep current_load=0.0. Guard check with threshold>0 and current_load=0 will never trigger — safe by design.

---

## Work Objectives

### Core Objective
Enable the firmware sequence player to monitor per-motor load cell values during trajectory interpolation and asynchronously truncate individual motor movements when directional force thresholds are crossed, supporting both ground-contact and ground-clearance detection.

### Concrete Deliverables
- `Motor.h` / `Motor.cpp`: StrainGauge* pointer, `current_load`, `attachStrainGauge()`, `updateLoad()`
- `SequencePlayer.h`: `GuardCondition` enum, extended `Keyframe` struct, module-scope guard state arrays
- `SequencePlayer.cpp`: Modified `beginInterp`, expanded `parseKeyframePayload`, guard-aware `sequenceUpdate` Phase 1 + Phase 2
- `Base.ino`: Strain gauge wiring in `setup()`, `current_load` updates in main loop
- `keyframe.py` (2 copies): `guard_threshold`, `guard_condition` fields + serialization
- `protocol.py`: `send_keyframe` 48-value format + `SEQ_GUARD_TRIG` parser
- `MEBot_control_node.py`: `send_sequence` passes guard fields
- Example guarded sequence JSON

### Definition of Done
- [ ] Firmware compiles without errors/warnings
- [ ] Existing 32-value and 17-value keyframe payloads still parse correctly
- [ ] New 48-value payloads parse correctly with guard fields
- [ ] Guard threshold of 0.0 / GUARD_NONE produces identical behavior to pre-change code
- [ ] When guard triggers, SEQ_GUARD_TRIG message emitted on serial
- [ ] Triggered motor holds latch position through settling phase
- [ ] Non-triggered motors complete interpolation normally
- [ ] Python Keyframe class round-trips guard fields through to_dict/from_dict
- [ ] ROS driver sends 48-value keyframes when guard fields are present

### Must Have
- Per-motor guard threshold and directional condition in every keyframe
- Asynchronous trajectory truncation (triggered motor latches, others continue)
- Backward compatibility with existing sequences (no guard = no change in behavior)
- SEQ_GUARD_TRIG telemetry message on trigger event

### Must NOT Have (Guardrails)
- NO calibration layer for load cells (ADC-to-Newton conversion is out of scope)
- NO changes to the PID control loop itself (guard operates at sequence player level, not PID level)
- NO new SystemState enum values (guard is per-motor per-keyframe, not a global system state)
- NO changes to PID tuner GUI (only the shared keyframe.py data model is updated)
- NO changes to existing JSON sequence files (they must work as-is with defaults)
- NO additional filtering on strain gauge data (existing IIR LPF alpha=0.8 is sufficient)
- NO modification to motor position limits or deadzone values

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** for compilation and protocol tests.
> Hardware-dependent behavioral verification (actual load cell triggering) noted as requiring physical robot.

### Test Decision
- **Infrastructure exists**: NO (Arduino firmware — no unit test framework)
- **Automated tests**: None (firmware). Python-side verification via manual serial testing.
- **Framework**: N/A

### QA Policy
Every task includes agent-executable QA scenarios for what CAN be verified without hardware:
- Firmware compilation (Arduino CLI or PlatformIO)
- Serial protocol format verification (Python script sends test payloads)
- Python class unit tests (import, serialize, deserialize)
- Backward compatibility (existing JSON loads without error)

Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Foundation — type definitions, all independent):
├── Task 1: Motor class StrainGauge integration [quick]
│   Files: Motor.h, Motor.cpp
├── Task 2: SequencePlayer guard type definitions [quick]
│   Files: SequencePlayer.h, SequencePlayer.cpp (top-of-file declarations only)
└── Task 3: Python Keyframe model + ProtocolEncoder expansion [quick]
    Files: keyframe.py (×2), protocol.py (ProtocolEncoder.send_keyframe)

Wave 2 (Core Implementation — parallel across firmware/ROS):
├── Task 4: Base.ino strain gauge wiring [quick] (depends: T1)
│   Files: Base.ino
├── Task 5: SequencePlayer guard logic [unspecified-high] (depends: T2)
│   Files: SequencePlayer.cpp
└── Task 6: ROS driver integration + telemetry parsing [quick] (depends: T3)
    Files: MEBot_control_node.py, protocol.py (ProtocolParser)

Wave 3 (Integration & Verification):
└── Task 7: Integration test sequence + backward compat verification [quick] (depends: T5, T6)
    Files: config/guarded_test_sequence.json, verification scripts

Wave FINAL (After ALL tasks — 4 parallel reviews, then user okay):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA (unspecified-high)
└── Task F4: Scope fidelity check (deep)
-> Present results -> Get explicit user okay

Critical Path: T1 → T4 | T2 → T5 → T7 → F1-F4 → user okay
Parallel Speedup: ~50% faster than sequential
Max Concurrent: 3 (Wave 1 and Wave 2)
```

### Dependency Matrix

| Task | Depends On | Blocks | Wave |
|------|-----------|--------|------|
| T1   | —         | T4     | 1    |
| T2   | —         | T5     | 1    |
| T3   | —         | T6     | 1    |
| T4   | T1        | T7     | 2    |
| T5   | T2        | T7     | 2    |
| T6   | T3        | T7     | 2    |
| T7   | T4,T5,T6  | F1-F4  | 3    |

### Agent Dispatch Summary

- **Wave 1**: **3** — T1 → `quick`, T2 → `quick`, T3 → `quick`
- **Wave 2**: **3** — T4 → `quick`, T5 → `unspecified-high`, T6 → `quick`
- **Wave 3**: **1** — T7 → `quick`
- **FINAL**: **4** — F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

- [x] 1. Motor Class: StrainGauge Pointer + current_load Integration

  **What to do**:
  - Add `#include "StrainGauge/StrainGauge.h"` to Motor.h
  - Add private member `StrainGauge* _strain_gauge = nullptr;` to Motor class
  - Add public member `float current_load = 0.0f;` to Motor class
  - Add public method `void attachStrainGauge(StrainGauge* sg);` — stores pointer
  - Add public method `void updateLoad();` — if `_strain_gauge != nullptr`, sets `current_load = _strain_gauge->getValue();`, else leaves at 0.0f
  - In Motor constructor, ensure `_strain_gauge = nullptr` and `current_load = 0.0f`

  **Must NOT do**:
  - Do NOT modify the PID control loop (update() method)
  - Do NOT call strain_gauge->update(dt) inside Motor — that stays in Base.ino main loop
  - Do NOT add any calibration or unit conversion logic
  - Do NOT change any existing Motor member variables or method signatures

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Small, well-scoped change to 2 files with clear spec
  - **Skills**: []
  - **Skills Evaluated but Omitted**:
    - None needed — straightforward C++ class extension

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3)
  - **Blocks**: Task 4
  - **Blocked By**: None (can start immediately)

  **References**:

  **Pattern References**:
  - `src/Motor/Motor.h:1-67` — Full Motor class definition. Note existing public members (current_pos, current_vel at ~line 38-39), constructor pattern, and include style
  - `src/Motor/Motor.cpp:1-10` — Constructor implementation pattern. Follow same initialization style for new members

  **API/Type References**:
  - `src/StrainGauge/StrainGauge.h:1-18` — StrainGauge class interface. Only `getValue()` is needed by Motor. Note it returns `float` (filtered ADC value)

  **WHY Each Reference Matters**:
  - Motor.h shows WHERE to add the new members (after existing public floats around line 39) and the include convention (line 5-7)
  - Motor.cpp shows constructor init pattern to follow for _strain_gauge and current_load
  - StrainGauge.h confirms the API — Motor only needs to call `getValue()`, not `update(dt)`

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Motor compiles with StrainGauge integration
    Tool: Bash
    Preconditions: Firmware source in hardware/rammp_prototype_driver/firmware/base/
    Steps:
      1. Verify Motor.h includes StrainGauge.h: grep '#include.*StrainGauge' src/Motor/Motor.h
      2. Verify current_load member exists: grep 'float current_load' src/Motor/Motor.h
      3. Verify attachStrainGauge declaration: grep 'attachStrainGauge' src/Motor/Motor.h
      4. Verify updateLoad declaration: grep 'updateLoad' src/Motor/Motor.h
      5. Verify attachStrainGauge implementation: grep 'attachStrainGauge' src/Motor/Motor.cpp
      6. Verify updateLoad implementation with null check: grep -A3 'updateLoad' src/Motor/Motor.cpp
    Expected Result: All grep commands find matches. updateLoad contains nullptr check.
    Failure Indicators: Missing include, missing member declaration, no null guard in updateLoad
    Evidence: .sisyphus/evidence/task-1-motor-straingauge-integration.txt

  Scenario: Motor default state is safe without StrainGauge
    Tool: Bash
    Preconditions: Motor.cpp has constructor changes
    Steps:
      1. Verify constructor initializes _strain_gauge to nullptr: grep -n 'nullptr' src/Motor/Motor.cpp
      2. Verify current_load default is 0.0f: grep 'current_load.*0' src/Motor/Motor.h
    Expected Result: Default Motor has no strain gauge and current_load=0.0f
    Failure Indicators: Missing nullptr init, missing 0.0f default
    Evidence: .sisyphus/evidence/task-1-motor-safe-defaults.txt
  ```

  **Commit**: YES (groups with T2)
  - Message: `feat(firmware): add guard types to Motor and SequencePlayer`
  - Files: `src/Motor/Motor.h`, `src/Motor/Motor.cpp`

- [x] 2. SequencePlayer Guard Type Definitions

  **What to do**:
  - In SequencePlayer.h, add the `GuardCondition` enum BEFORE the Keyframe struct:
    ```cpp
    enum GuardCondition {
      GUARD_NONE = 0,
      GUARD_GREATER_THAN = 1,
      GUARD_LESS_THAN = 2
    };
    ```
  - Extend the `Keyframe` struct with two new arrays:
    ```cpp
    float guard_threshold[SEQ_NUM_MOTORS];    // 0.0 = no guard
    uint8_t guard_condition[SEQ_NUM_MOTORS];  // GuardCondition enum value
    ```
  - In SequencePlayer.cpp, add two module-scope static arrays after existing state variables (around line 23):
    ```cpp
    static bool seq_guard_triggered[SEQ_NUM_MOTORS];
    static float seq_latch_pos[SEQ_NUM_MOTORS];
    ```

  **Must NOT do**:
  - Do NOT modify any function implementations yet (beginInterp, parseKeyframePayload, sequenceUpdate)
  - Do NOT change existing Keyframe fields
  - Do NOT rename or reorder existing module-scope variables

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Adding an enum, 2 struct fields, and 2 static arrays — minimal, mechanical changes
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3)
  - **Blocks**: Task 5
  - **Blocked By**: None (can start immediately)

  **References**:

  **Pattern References**:
  - `src/SequencePlayer/SequencePlayer.h:10-34` — Current SEQ_NUM_MOTORS define (line 10) and Keyframe struct (lines 29-34). Add enum before struct, new fields inside struct after duration_ms
  - `src/SequencePlayer/SequencePlayer.cpp:9-23` — Module-scope state variables. Add seq_guard_triggered and seq_latch_pos after seq_auto_run (line 23)

  **WHY Each Reference Matters**:
  - SequencePlayer.h lines 29-34 show exactly WHERE to insert the new struct fields and WHERE the enum should go (before the struct definition)
  - SequencePlayer.cpp lines 9-23 show the static variable declaration pattern and WHERE to add the new guard state arrays

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: GuardCondition enum and Keyframe extension exist
    Tool: Bash
    Preconditions: SequencePlayer.h modified
    Steps:
      1. Verify GuardCondition enum: grep -A4 'enum GuardCondition' src/SequencePlayer/SequencePlayer.h
      2. Verify guard_threshold field: grep 'guard_threshold' src/SequencePlayer/SequencePlayer.h
      3. Verify guard_condition field: grep 'guard_condition' src/SequencePlayer/SequencePlayer.h
      4. Verify GUARD_NONE=0, GUARD_GREATER_THAN=1, GUARD_LESS_THAN=2 values
    Expected Result: Enum has 3 values. Keyframe struct has both new arrays of size SEQ_NUM_MOTORS.
    Failure Indicators: Missing enum values, wrong array sizes, uint8_t type missing for guard_condition
    Evidence: .sisyphus/evidence/task-2-guard-types.txt

  Scenario: Module-scope guard state arrays declared
    Tool: Bash
    Preconditions: SequencePlayer.cpp modified
    Steps:
      1. Verify seq_guard_triggered: grep 'static bool seq_guard_triggered' src/SequencePlayer/SequencePlayer.cpp
      2. Verify seq_latch_pos: grep 'static float seq_latch_pos' src/SequencePlayer/SequencePlayer.cpp
      3. Verify both use SEQ_NUM_MOTORS size
    Expected Result: Both static arrays declared with [SEQ_NUM_MOTORS] size
    Evidence: .sisyphus/evidence/task-2-guard-state-arrays.txt
  ```

  **Commit**: YES (groups with T1)
  - Message: `feat(firmware): add guard types to Motor and SequencePlayer`
  - Files: `src/SequencePlayer/SequencePlayer.h`, `src/SequencePlayer/SequencePlayer.cpp`

- [x] 3. Python Keyframe Model + ProtocolEncoder Expansion

  **What to do**:
  - **keyframe.py** (BOTH copies — update `hardware/rammp_prototype_driver/rammp_prototype_driver/keyframe.py` AND `hardware/pid_tuner/serial_driver/keyframe.py` identically):
    - Add `self.guard_threshold: List[float] = [0.0] * NUM_MOTORS` to `__init__`
    - Add `self.guard_condition: List[int] = [0] * NUM_MOTORS` to `__init__` (0=GUARD_NONE, 1=GUARD_GREATER_THAN, 2=GUARD_LESS_THAN)
    - Add both fields to `to_dict()` return value
    - Add parsing in `from_dict()` with defaults: `guard_threshold` defaults to `[0.0]*8`, `guard_condition` defaults to `[0]*8`. Pad to NUM_MOTORS if short.
  - **protocol.py** (`ProtocolEncoder.send_keyframe`):
    - Add optional parameters: `guard_threshold: Optional[List[float]] = None`, `guard_condition: Optional[List[int]] = None`
    - When guard fields are provided (any non-zero value), build 48-value format: `J{idx}:targets(8),active(8),relative(8),durations(8),guard_thresholds(8),guard_conditions(8)`
    - When guard fields are all zero or None, send existing 32-value format for backward compatibility
    - guard_threshold defaults to `[0.0]*8`, guard_condition defaults to `[0]*8`

  **Must NOT do**:
  - Do NOT modify ProtocolParser (that's Task 6)
  - Do NOT modify MEBot_control_node.py (that's Task 6)
  - Do NOT change existing function signatures in a breaking way
  - Do NOT add GUI elements to PID tuner

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Mechanical additions to Python data model and string formatting
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2)
  - **Blocks**: Task 6
  - **Blocked By**: None (can start immediately)

  **References**:

  **Pattern References**:
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/keyframe.py:17-61` — Full Keyframe class. Follow the exact pattern of `motor_durations` for adding new list fields: init with `[default]*NUM_MOTORS`, handle None/short lists in from_dict, include in to_dict
  - `hardware/pid_tuner/serial_driver/keyframe.py:18-62` — Identical copy. MUST receive same changes.
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py:562-587` — `ProtocolEncoder.send_keyframe()`. Current 32-value format: `J{idx}:t_str,a_str,r_str,d_str`. Extend with `gt_str,gc_str` for 48-value format.

  **WHY Each Reference Matters**:
  - keyframe.py:40-61 shows from_dict pattern with padding (while len < NUM_MOTORS: append default) — MUST follow this for guard fields
  - protocol.py:562-587 shows the exact string format construction — extend with 2 more comma-separated blocks
  - Two copies of keyframe.py MUST be kept in sync

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Keyframe round-trips guard fields via to_dict/from_dict
    Tool: Bash
    Preconditions: keyframe.py updated in ROS driver package
    Steps:
      1. Run: python3 -c "
         import sys; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.keyframe import Keyframe
         kf = Keyframe()
         kf.guard_threshold = [0,0,100.0,100.0,0,0,0,0]
         kf.guard_condition = [0,0,1,1,0,0,0,0]
         d = kf.to_dict()
         kf2 = Keyframe.from_dict(d)
         assert kf2.guard_threshold == [0,0,100.0,100.0,0,0,0,0], f'threshold mismatch: {kf2.guard_threshold}'
         assert kf2.guard_condition == [0,0,1,1,0,0,0,0], f'condition mismatch: {kf2.guard_condition}'
         print('PASS: round-trip OK')
         "
    Expected Result: Prints "PASS: round-trip OK" with exit code 0
    Failure Indicators: AssertionError, ImportError, missing attribute
    Evidence: .sisyphus/evidence/task-3-keyframe-roundtrip.txt

  Scenario: Existing JSON loads with guard defaults
    Tool: Bash
    Preconditions: keyframe.py updated
    Steps:
      1. Run: python3 -c "
         import json, sys; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.keyframe import Keyframe
         with open('hardware/rammp_prototype_driver/config/curb_ascending.json') as f:
             data = json.load(f)
         kfs = [Keyframe.from_dict(d) for d in data['keyframes']]
         for kf in kfs:
             assert kf.guard_threshold == [0.0]*8, f'non-zero default: {kf.guard_threshold}'
             assert kf.guard_condition == [0]*8, f'non-zero condition: {kf.guard_condition}'
         print(f'PASS: {len(kfs)} keyframes loaded with guard defaults')
         "
    Expected Result: All keyframes load with guard_threshold=[0.0]*8 and guard_condition=[0]*8
    Evidence: .sisyphus/evidence/task-3-backward-compat.txt

  Scenario: send_keyframe produces 48-value format with guards
    Tool: Bash
    Preconditions: protocol.py updated
    Steps:
      1. Run: python3 -c "
         import sys; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.protocol import ProtocolEncoder
         result = ProtocolEncoder.send_keyframe(
             0,
             [0]*8, [1,1,1,1,0,0,0,0], [1000]*8,
             relative=[False]*8,
             guard_threshold=[0,0,100.0,100.0,0,0,0,0],
             guard_condition=[0,0,1,1,0,0,0,0]
         )
         payload = result.decode('ascii').strip()
         parts = payload.split(':',1)[1]
         values = parts.split(',')
         assert len(values) == 48, f'Expected 48 values, got {len(values)}'
         print(f'PASS: 48-value format. Payload: {payload[:80]}...')
         "
    Expected Result: 48 comma-separated values in J0: format
    Evidence: .sisyphus/evidence/task-3-48value-format.txt
  ```

  **Commit**: YES
  - Message: `feat(driver): extend Python Keyframe model with guard fields`
  - Files: `hardware/rammp_prototype_driver/rammp_prototype_driver/keyframe.py`, `hardware/pid_tuner/serial_driver/keyframe.py`, `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py`

- [x] 4. Base.ino: Wire Strain Gauges to Motors + Update current_load

  **What to do**:
  - In `setup()` (after motor and strain gauge instantiation), call `attachStrainGauge` for all 4 load-cell-equipped motors:
    ```cpp
    rc.attachStrainGauge(&sg_rc);
    fc.attachStrainGauge(&sg_fc);
    ml.attachStrainGauge(&sg_ml);
    mr.attachStrainGauge(&sg_mr);
    ```
  - In the main loop sensor section (after the 4 `sg_*.update(dt)` calls at ~lines 571-574), add load updates for all 4 motors:
    ```cpp
    rc.updateLoad();
    fc.updateLoad();
    ml.updateLoad();
    mr.updateLoad();
    ```
  - The remaining 4 motors (ml_carriage, mr_carriage, drive_fb, drive_lr) get NO strain gauge — their current_load stays 0.0f by default.

  **Must NOT do**:
  - Do NOT move or modify the existing `sg_*.update(dt)` calls
  - Do NOT change motor instantiation or PID initialization
  - Do NOT add strain gauge objects for motors that don't have physical load cells
  - Do NOT modify the control loop or RoboClaw write sections

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: 8 lines of code in 2 locations within one file
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 5, 6)
  - **Blocks**: Task 7
  - **Blocked By**: Task 1 (needs Motor::attachStrainGauge and Motor::updateLoad)

  **References**:

  **Pattern References**:
  - `Base.ino:73-80` — Motor object declarations (rc, fc, ml, mr, ml_carriage, mr_carriage, drive_fb, drive_lr)
  - `Base.ino:101-104` — Strain gauge instantiation (sg_rc, sg_fc, sg_ml, sg_mr)
  - `Base.ino:571-574` — Strain gauge update calls in main loop sensor section: `sg_rc.update(dt); sg_fc.update(dt); sg_ml.update(dt); sg_mr.update(dt);`

  **WHY Each Reference Matters**:
  - Lines 73-80 confirm the 4 motors that have corresponding strain gauges (rc↔sg_rc, fc↔sg_fc, ml↔sg_ml, mr↔sg_mr)
  - Lines 101-104 confirm strain gauge pin wiring and LPF alpha (0.8)
  - Lines 571-574 show WHERE to add updateLoad() calls — immediately after the existing sg_*.update(dt) block

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Strain gauges wired to motors in setup
    Tool: Bash
    Preconditions: Base.ino modified
    Steps:
      1. grep 'attachStrainGauge' Base.ino — expect 4 matches (rc, fc, ml, mr)
      2. Verify rc.attachStrainGauge(&sg_rc) pattern for all 4
      3. Verify no attachStrainGauge calls for ml_carriage, mr_carriage, drive_fb, drive_lr
    Expected Result: Exactly 4 attachStrainGauge calls, one per load-cell-equipped motor
    Evidence: .sisyphus/evidence/task-4-setup-wiring.txt

  Scenario: updateLoad called in main loop after strain gauge updates
    Tool: Bash
    Preconditions: Base.ino modified
    Steps:
      1. grep -n 'updateLoad' Base.ino — expect 4 matches
      2. Verify updateLoad calls appear AFTER sg_*.update(dt) calls (higher line numbers)
      3. Verify only rc, fc, ml, mr call updateLoad
    Expected Result: 4 updateLoad calls positioned after strain gauge updates in sensor section
    Evidence: .sisyphus/evidence/task-4-loop-update.txt
  ```

  **Commit**: YES (groups with T5)
  - Message: `feat(firmware): implement guarded movement in sequence player`
  - Files: `Base.ino`

- [x] 5. SequencePlayer: Guard Logic Implementation (beginInterp + Parser + Phase 1 + Phase 2)

  **What to do**:
  This is the core implementation task. Modify SequencePlayer.cpp with 4 changes:

  **A. beginInterp modification** (~line 37-49):
  - After the existing `seq_start_pos[i] = motors[i]->current_pos;` line inside the for loop, add:
    ```cpp
    seq_guard_triggered[i] = false;
    ```

  **B. parseKeyframePayload expansion** (~line 54-93):
  - Change `MAX_VALS` from `SEQ_NUM_MOTORS * 4` to `SEQ_NUM_MOTORS * 6` (48 values)
  - Add a NEW format check for `count == SEQ_NUM_MOTORS * 6` (48 values) as the FIRST format branch:
    ```cpp
    if (count == SEQ_NUM_MOTORS * 6) {
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        kf.targets[i] = vals[i];
        kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
        kf.relative[i] = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
        kf.duration_ms[i] = (uint32_t)vals[SEQ_NUM_MOTORS * 3 + i];
        kf.guard_threshold[i] = vals[SEQ_NUM_MOTORS * 4 + i];
        kf.guard_condition[i] = (uint8_t)vals[SEQ_NUM_MOTORS * 5 + i];
      }
      return true;
    }
    ```
  - In the existing 32-value and 17-value branches, add defaults:
    ```cpp
    kf.guard_threshold[i] = 0.0f;
    kf.guard_condition[i] = GUARD_NONE;
    ```

  **C. sequenceUpdate Phase 1 — Guard check in interpolation loop** (~lines 192-254):
  - BEFORE the existing `float t_i = ...` calculation, insert the guard condition check:
    ```cpp
    // Guard Condition Check
    if (kf.guard_condition[i] != GUARD_NONE && !seq_guard_triggered[i]) {
        bool condition_met = false;
        float current_load = motors[i]->current_load;
        if (kf.guard_condition[i] == GUARD_GREATER_THAN) {
            condition_met = (current_load >= kf.guard_threshold[i]);
        } else if (kf.guard_condition[i] == GUARD_LESS_THAN) {
            condition_met = (current_load <= kf.guard_threshold[i]);
        }
        if (condition_met) {
            seq_guard_triggered[i] = true;
            seq_latch_pos[i] = motors[i]->current_pos;
            Serial.print("SEQ_GUARD_TRIG,m");
            Serial.print(i);
            Serial.print(",load=");
            Serial.println(current_load);
        }
    }
    ```
  - AFTER the guard check, wrap the existing lerp in a conditional:
    ```cpp
    if (seq_guard_triggered[i]) {
        motors[i]->setTargetPosition(seq_latch_pos[i]);
        // Triggered motor counts as "done" — do NOT set all_lerps_done = false
    } else {
        // Existing lerp code (t_i calculation, interpolation, setTargetPosition)
        float t_i = ...;
        if (t_i < 1.0f) { all_lerps_done = false; ... }
        float dest = finalTarget(kf, i);
        float pos = seq_start_pos[i] + t_i * (dest - seq_start_pos[i]);
        motors[i]->setTargetPosition(pos);
    }
    ```
  - In the `if (all_lerps_done)` block where final targets are set, respect latch:
    ```cpp
    float final_dest = seq_guard_triggered[i] ? seq_latch_pos[i] : finalTarget(kf, i);
    motors[i]->setTargetPosition(final_dest);
    ```

  **D. sequenceUpdate Phase 2 — Settling with latch respect** (~lines 256-307):
  - Replace `float dest = finalTarget(kf, i);` with:
    ```cpp
    float dest = seq_guard_triggered[i] ? seq_latch_pos[i] : finalTarget(kf, i);
    ```
  - The rest of settling logic (deadzone check, timeout) remains unchanged.

  **Must NOT do**:
  - Do NOT change the settling timeout value (SEQ_COMPLETION_TIMEOUT_MS)
  - Do NOT change deadzone values (SEQ_COMPLETION_DEADZONE)
  - Do NOT modify the auto-run / step advancement logic
  - Do NOT change the telemetry logging format for existing SEQ_LERP / SEQ_SETTLE_WAIT messages
  - Do NOT add any filtering on current_load (use it raw from Motor)

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Core behavioral logic change in safety-critical firmware. Requires careful placement of guard checks within the existing interpolation state machine. Multiple interleaved code sections.
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 6)
  - **Blocks**: Task 7
  - **Blocked By**: Task 2 (needs GuardCondition enum, extended Keyframe, module-scope arrays)

  **References**:

  **Pattern References**:
  - `src/SequencePlayer/SequencePlayer.cpp:37-49` — `beginInterp()` full implementation. Add guard reset inside existing for loop after seq_start_pos assignment (line 39)
  - `src/SequencePlayer/SequencePlayer.cpp:54-93` — `parseKeyframePayload()` full implementation. MAX_VALS on line 56, format branches at lines 71 and 82. New 48-value branch goes BEFORE existing 32-value branch.
  - `src/SequencePlayer/SequencePlayer.cpp:192-254` — Phase 1 interpolation loop. Guard check inserts before t_i calculation (~line 201). The `if (!kf.active[i]) continue;` at ~line 198 stays as-is. Lerp code at ~lines 201-215 gets wrapped in else branch of guard conditional.
  - `src/SequencePlayer/SequencePlayer.cpp:256-307` — Phase 2 settling. `float dest = finalTarget(kf, i);` at ~line 265 becomes conditional.
  - `src/SequencePlayer/SequencePlayer.cpp:30-34` — `finalTarget()` helper. NOT modified — still used for non-guarded motors.

  **API/Type References**:
  - `src/Motor/Motor.h` — `motors[i]->current_load` (float, public) — the load cell reading
  - `src/Motor/Motor.h` — `motors[i]->current_pos` (float, public) — used for latch position capture
  - `src/SequencePlayer/SequencePlayer.h` — `GuardCondition` enum, `Keyframe.guard_threshold[]`, `Keyframe.guard_condition[]` (from Task 2)

  **WHY Each Reference Matters**:
  - Lines 37-49 (beginInterp): Shows exact loop structure where guard reset goes — critical to clear state at start of every keyframe
  - Lines 54-93 (parser): Shows the format-detection pattern (count == N*4, count == N*2+1). New 48-value format MUST be checked first (before 32-value) because 48 > 32
  - Lines 192-254 (Phase 1): This is the most complex modification. The guard check must go AFTER `if (!kf.active[i]) continue;` but BEFORE `float t_i = ...`. The else branch wraps the ENTIRE existing lerp block including the all_lerps_done tracking.
  - Lines 256-307 (Phase 2): Simple conditional — just replace one line. But critical to prevent re-lifting chassis.

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: beginInterp clears guard state
    Tool: Bash
    Preconditions: SequencePlayer.cpp modified
    Steps:
      1. grep -A8 'void beginInterp' src/SequencePlayer/SequencePlayer.cpp
      2. Verify seq_guard_triggered[i] = false appears inside the for loop
    Expected Result: Guard state reset present inside beginInterp's for loop
    Evidence: .sisyphus/evidence/task-5-begininterp-reset.txt

  Scenario: Parser handles 48-value format with guard fields
    Tool: Bash
    Preconditions: SequencePlayer.cpp modified
    Steps:
      1. grep 'SEQ_NUM_MOTORS \* 6' src/SequencePlayer/SequencePlayer.cpp — verify MAX_VALS = 48
      2. grep -c 'guard_threshold\[i\]' src/SequencePlayer/SequencePlayer.cpp — expect ≥2 (48-value branch + default in legacy branches)
      3. grep -c 'guard_condition\[i\]' src/SequencePlayer/SequencePlayer.cpp — expect ≥2
      4. Verify 48-value branch appears BEFORE 32-value branch in the if/else chain
    Expected Result: Parser has 48-value branch with guard fields, legacy branches set defaults
    Evidence: .sisyphus/evidence/task-5-parser-48val.txt

  Scenario: Phase 1 guard check and latch logic present
    Tool: Bash
    Preconditions: SequencePlayer.cpp modified
    Steps:
      1. grep -c 'SEQ_GUARD_TRIG' src/SequencePlayer/SequencePlayer.cpp — expect ≥1
      2. grep 'seq_guard_triggered\[i\] = true' src/SequencePlayer/SequencePlayer.cpp — expect match
      3. grep 'seq_latch_pos\[i\] = motors\[i\]->current_pos' src/SequencePlayer/SequencePlayer.cpp — expect match
      4. grep 'GUARD_GREATER_THAN' src/SequencePlayer/SequencePlayer.cpp — expect match in condition check
      5. grep 'GUARD_LESS_THAN' src/SequencePlayer/SequencePlayer.cpp — expect match in condition check
      6. Verify guarded motor calls setTargetPosition(seq_latch_pos[i])
    Expected Result: Full guard logic present: condition evaluation, latch capture, serial message, position hold
    Evidence: .sisyphus/evidence/task-5-phase1-guard.txt

  Scenario: Phase 2 settling respects latch position
    Tool: Bash
    Preconditions: SequencePlayer.cpp modified
    Steps:
      1. In settling section, verify conditional dest: grep 'seq_guard_triggered\[i\] ? seq_latch_pos\[i\] : finalTarget' src/SequencePlayer/SequencePlayer.cpp
    Expected Result: Settling phase uses latch position for triggered motors, finalTarget for others
    Evidence: .sisyphus/evidence/task-5-phase2-settle.txt

  Scenario: Guard with GUARD_NONE produces no behavioral change (safety)
    Tool: Bash (code review)
    Preconditions: SequencePlayer.cpp modified
    Steps:
      1. Verify guard check starts with: if (kf.guard_condition[i] != GUARD_NONE && ...)
      2. Verify GUARD_NONE = 0 means the entire guard block is skipped
      3. Verify legacy parser sets guard_condition[i] = GUARD_NONE
    Expected Result: When guard_condition is GUARD_NONE (0), execution path is identical to pre-change code
    Evidence: .sisyphus/evidence/task-5-guard-none-safe.txt
  ```

  **Commit**: YES (groups with T4)
  - Message: `feat(firmware): implement guarded movement in sequence player`
  - Files: `src/SequencePlayer/SequencePlayer.cpp`

- [x] 6. ROS Driver Integration: send_sequence + SEQ_GUARD_TRIG Parsing

  **What to do**:
  - **MEBot_control_node.py** (`send_sequence` method, ~line 375-393):
    - Modify the keyframe iteration to pass guard fields through to `ProtocolEncoder.send_keyframe`:
      ```python
      self.write_serial_data(
          ProtocolEncoder.send_keyframe(
              idx, targets, active, durations, kf.relative,
              guard_threshold=kf.guard_threshold,
              guard_condition=kf.guard_condition,
          )
      )
      ```
  - **protocol.py** (`ProtocolParser` class, ~line 158-357):
    - Add a regex pattern for SEQ_GUARD_TRIG messages:
      ```python
      SEQ_GUARD_TRIG_PATTERN = re.compile(r"^SEQ_GUARD_TRIG,m(\d+),load=(.+)$")
      ```
    - Add a new dataclass:
      ```python
      @dataclass
      class SeqGuardTrigData:
          motor_index: int
          load_value: float
      ```
    - In `parse_line`, add a branch to match and return `SeqGuardTrigData`
  - **MEBot_control_node.py** (serial parsing section):
    - Handle `SeqGuardTrigData` in the serial parsing logic — log the guard trigger event. Find where `SeqStatusData` and `SeqAckData` are handled and add similar handling for `SeqGuardTrigData`.

  **Must NOT do**:
  - Do NOT modify telemetry (TELEMETRY,...) parsing
  - Do NOT add ROS topic publishers for guard events (out of scope — can be added later)
  - Do NOT modify the existing send_keyframe 32-value fallback behavior

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Straightforward Python integration — passing new fields through existing call chain + adding one regex pattern
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 5)
  - **Blocks**: Task 7
  - **Blocked By**: Task 3 (needs Keyframe guard fields and send_keyframe expansion)

  **References**:

  **Pattern References**:
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/MEBot_control_node.py:375-393` — `send_sequence()` method. Line 387-389 shows current `ProtocolEncoder.send_keyframe(idx, targets, active, durations, kf.relative)` call — add guard_threshold and guard_condition kwargs
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py:164-165` — SEQ_STATUS_PATTERN and SEQ_ACK_PATTERN regex definitions. Follow same pattern for SEQ_GUARD_TRIG
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py:222-242` — SEQ_STATUS and SEQ_ACK parse_line branches. Follow same try/except pattern for SEQ_GUARD_TRIG
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py:145-155` — SeqAckData and SeqStatusData dataclass pattern. Follow for SeqGuardTrigData

  **WHY Each Reference Matters**:
  - MEBot_control_node.py:387-389 shows the exact call site to modify — just adding 2 keyword arguments
  - protocol.py:164-165 shows regex pattern convention for sequence messages
  - protocol.py:222-242 shows the parse_line branching pattern — add SEQ_GUARD_TRIG before the TELEMETRY match
  - protocol.py:145-155 shows the dataclass pattern for parsed data types

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: send_sequence passes guard fields to send_keyframe
    Tool: Bash
    Preconditions: MEBot_control_node.py modified
    Steps:
      1. grep -A10 'send_keyframe' hardware/rammp_prototype_driver/rammp_prototype_driver/MEBot_control_node.py
      2. Verify guard_threshold and guard_condition are passed as kwargs
    Expected Result: send_keyframe call includes guard_threshold=kf.guard_threshold and guard_condition=kf.guard_condition
    Evidence: .sisyphus/evidence/task-6-send-sequence-guards.txt

  Scenario: SEQ_GUARD_TRIG parsed correctly
    Tool: Bash
    Preconditions: protocol.py modified
    Steps:
      1. Run: python3 -c "
         import sys; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.protocol import ProtocolParser
         result = ProtocolParser.parse_line('SEQ_GUARD_TRIG,m2,load=523.7')
         assert result is not None, 'Failed to parse SEQ_GUARD_TRIG'
         assert result.motor_index == 2, f'Wrong motor: {result.motor_index}'
         assert abs(result.load_value - 523.7) < 0.01, f'Wrong load: {result.load_value}'
         print(f'PASS: Parsed motor={result.motor_index}, load={result.load_value}')
         "
    Expected Result: Prints "PASS: Parsed motor=2, load=523.7"
    Failure Indicators: None return, wrong values, AttributeError
    Evidence: .sisyphus/evidence/task-6-guard-trig-parse.txt

  Scenario: Existing SEQ_STATUS and SEQ_ACK parsing unaffected
    Tool: Bash
    Preconditions: protocol.py modified
    Steps:
      1. Run: python3 -c "
         import sys; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.protocol import ProtocolParser
         s = ProtocolParser.parse_line('SEQ_STATUS,3,10,1')
         assert s.current_step == 3 and s.total_steps == 10 and s.state == 1
         a = ProtocolParser.parse_line('SEQ_ACK,5')
         assert a.step_idx == 5
         print('PASS: Existing parsers unaffected')
         "
    Expected Result: Both existing parsers return correct data
    Evidence: .sisyphus/evidence/task-6-existing-parsers-ok.txt
  ```

  **Commit**: YES
  - Message: `feat(driver): integrate guard fields in ROS driver protocol`
  - Files: `hardware/rammp_prototype_driver/rammp_prototype_driver/MEBot_control_node.py`, `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py`

- [x] 7. Integration Test: Guarded Sequence JSON + Backward Compatibility Verification

  **What to do**:
  - Create a test guarded sequence JSON file at `hardware/rammp_prototype_driver/config/guarded_test_sequence.json` with 2-3 keyframes:
    - Keyframe 1: Standard move (no guards) — all guard_threshold=0, guard_condition=0
    - Keyframe 2: Guarded contact move on ML and MR — guard_threshold=[0,0,100.0,100.0,0,0,0,0], guard_condition=[0,0,1,1,0,0,0,0] (GUARD_GREATER_THAN for motors 2,3)
    - Keyframe 3: Guarded clearance move on ML — guard_threshold=[0,0,50.0,0,0,0,0,0], guard_condition=[0,0,2,0,0,0,0,0] (GUARD_LESS_THAN for motor 2)
  - Verify the JSON round-trips through Python Keyframe class
  - Verify ProtocolEncoder produces correct 48-value and 32-value payloads from the sequence
  - Verify ALL existing JSON sequences (curb_ascending.json, curb_descending.json, dry_run_seq.json, dry_run_seq_2.json) still load and serialize correctly with guard defaults

  **Must NOT do**:
  - Do NOT modify existing JSON sequence files
  - Do NOT require hardware for this verification
  - Do NOT add ROS launch file changes

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Creating one JSON file and running verification scripts
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 3 (sequential — depends on all Wave 2 tasks)
  - **Blocks**: F1-F4
  - **Blocked By**: Tasks 4, 5, 6

  **References**:

  **Pattern References**:
  - `hardware/rammp_prototype_driver/config/curb_ascending.json:1-36` — Existing keyframe JSON format. Follow same structure, add guard_threshold and guard_condition arrays.
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/keyframe.py` — Keyframe.from_dict and to_dict for serialization
  - `hardware/rammp_prototype_driver/rammp_prototype_driver/protocol.py:562-587` — send_keyframe for payload generation

  **WHY Each Reference Matters**:
  - curb_ascending.json shows the existing JSON schema that new files must be compatible with
  - keyframe.py defines how JSON fields map to Keyframe attributes — test that guard fields round-trip
  - protocol.py send_keyframe shows what the firmware will receive — verify 48-value format is correct

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Guarded test sequence loads and serializes
    Tool: Bash
    Preconditions: guarded_test_sequence.json created, all Python changes in place
    Steps:
      1. Run: python3 -c "
         import json, sys; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.keyframe import Keyframe
         from rammp_prototype_driver.protocol import ProtocolEncoder
         with open('hardware/rammp_prototype_driver/config/guarded_test_sequence.json') as f:
             data = json.load(f)
         kfs = [Keyframe.from_dict(d) for d in data['keyframes']]
         print(f'Loaded {len(kfs)} keyframes')
         for i, kf in enumerate(kfs):
             has_guard = any(c != 0 for c in kf.guard_condition)
             targets = [t if t is not None else 0.0 for t in kf.targets]
             active = [t is not None for t in kf.targets]
             durations = [kf.motor_durations[j] if kf.motor_durations[j] is not None else kf.duration_ms for j in range(8)]
             payload = ProtocolEncoder.send_keyframe(i, targets, active, durations, kf.relative, kf.guard_threshold, kf.guard_condition)
             vals = payload.decode().strip().split(':',1)[1].split(',')
             expected = 48 if has_guard else 32
             assert len(vals) == expected, f'KF{i}: expected {expected} vals, got {len(vals)}'
             print(f'  KF{i}: {len(vals)} values, guard={has_guard}, label={kf.label}')
         print('PASS: All keyframes serialize correctly')
         "
    Expected Result: 3 keyframes load. KF0=32 values (no guard), KF1+KF2=48 values (has guard).
    Evidence: .sisyphus/evidence/task-7-guarded-sequence.txt

  Scenario: All existing sequences backward compatible
    Tool: Bash
    Preconditions: All Python changes in place
    Steps:
      1. Run: python3 -c "
         import json, sys, glob; sys.path.insert(0, 'hardware/rammp_prototype_driver')
         from rammp_prototype_driver.keyframe import Keyframe
         from rammp_prototype_driver.protocol import ProtocolEncoder
         for path in sorted(glob.glob('hardware/rammp_prototype_driver/config/*.json')):
             if 'guarded_test' in path: continue
             with open(path) as f: data = json.load(f)
             kfs = [Keyframe.from_dict(d) for d in data.get('keyframes', data if isinstance(data, list) else [])]
             for kf in kfs:
                 assert kf.guard_threshold == [0.0]*8
                 assert kf.guard_condition == [0]*8
                 targets = [t if t is not None else 0.0 for t in kf.targets]
                 active = [t is not None for t in kf.targets]
                 durations = [kf.motor_durations[j] if kf.motor_durations[j] is not None else kf.duration_ms for j in range(8)]
                 payload = ProtocolEncoder.send_keyframe(0, targets, active, durations, kf.relative, kf.guard_threshold, kf.guard_condition)
                 vals = payload.decode().strip().split(':',1)[1].split(',')
                 assert len(vals) == 32, f'{path}: expected 32 vals, got {len(vals)}'
             print(f'PASS: {path} ({len(kfs)} keyframes, all 32-value format)')
         "
    Expected Result: All existing JSON files load with guard defaults and produce 32-value payloads
    Evidence: .sisyphus/evidence/task-7-backward-compat.txt
  ```

  **Commit**: YES
  - Message: `test: add guarded sequence JSON and backward compat verification`
  - Files: `hardware/rammp_prototype_driver/config/guarded_test_sequence.json`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.
>
> **Do NOT auto-proceed after verification. Wait for user's explicit approval before marking work complete.**

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (read file, check code). For each "Must NOT Have": search codebase for forbidden patterns — reject with file:line if found. Check evidence files exist in .sisyphus/evidence/. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `unspecified-high`
  Review all changed files for: compilation errors, type mismatches, missing includes, unused variables, logic errors in guard condition evaluation. Check AI slop: excessive comments, over-abstraction, generic names. Verify firmware compiles. Verify Python imports work.
  Output: `Build [PASS/FAIL] | Files [N clean/N issues] | VERDICT`

- [x] F3. **Real Manual QA** — `unspecified-high`
  Execute EVERY QA scenario from EVERY task — follow exact steps, capture evidence. Test cross-task integration (Python keyframe → serial encode → firmware parse). Test backward compatibility with existing JSON sequences. Save to `.sisyphus/evidence/final-qa/`.
  Output: `Scenarios [N/N pass] | Integration [N/N] | Edge Cases [N tested] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff. Verify 1:1 — everything in spec was built (no missing), nothing beyond spec was built (no creep). Check "Must NOT do" compliance. Detect cross-task contamination. Flag unaccounted changes.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

| Commit | Tasks | Message | Key Files |
|--------|-------|---------|-----------|
| 1 | T1, T2 | `feat(firmware): add guard types to Motor and SequencePlayer` | Motor.h, Motor.cpp, SequencePlayer.h, SequencePlayer.cpp |
| 2 | T3 | `feat(driver): extend Python Keyframe model with guard fields` | keyframe.py (×2), protocol.py |
| 3 | T4, T5 | `feat(firmware): implement guarded movement in sequence player` | Base.ino, SequencePlayer.cpp |
| 4 | T6 | `feat(driver): integrate guard fields in ROS driver protocol` | MEBot_control_node.py, protocol.py |
| 5 | T7 | `test: add guarded sequence JSON and backward compat verification` | config/guarded_test_sequence.json |

---

## Success Criteria

### Verification Commands
```bash
# Firmware compiles (Arduino CLI — adjust board FQBN as needed)
arduino-cli compile --fqbn teensy:avr:teensy41 hardware/rammp_prototype_driver/firmware/base/

# Python imports work
python3 -c "from rammp_prototype_driver.keyframe import Keyframe; kf = Keyframe(); print('guard_threshold:', kf.guard_threshold)"

# Existing JSON still loads
python3 -c "
import json
from rammp_prototype_driver.keyframe import Keyframe
with open('hardware/rammp_prototype_driver/config/curb_ascending.json') as f:
    data = json.load(f)
kfs = [Keyframe.from_dict(d) for d in data['keyframes']]
print(f'Loaded {len(kfs)} keyframes, guard defaults: {kfs[0].guard_threshold}')
"
```

### Final Checklist
- [ ] All "Must Have" present
- [ ] All "Must NOT Have" absent
- [ ] Firmware compiles clean
- [ ] Existing sequences load and play identically
- [ ] New guarded sequences parse and transmit correctly
- [ ] SEQ_GUARD_TRIG telemetry message format verified
