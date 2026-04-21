## [2026-04-16] Session start — confirmed source state

### Motor.h (73 lines)
- Public members: current_pos, current_vel, prev_pos, prev_vel, target_pos, target_vel, target_pwm, scaled_target_pwm, lpf_input_alpha, direction, encoder_dir, pos_limit_min, pos_limit_max, limits_enabled
- NO current_load, NO StrainGauge*, NO attachStrainGauge(), NO updateLoad()
- Constructor in Motor.cpp line 4-7: initializes current_pos, current_vel, target_pos, target_vel, target_pwm, pos_pid, vel_pid, mode
- Include style: #include "../PIDController/PIDController.h" and <Arduino.h>

### SequencePlayer.h (67 lines)
- Keyframe struct lines 29-34: targets[8], active[8], relative[8], duration_ms[8]
- NO GuardCondition enum, NO guard_threshold, NO guard_condition
- SEQ_NUM_MOTORS=8, MAX_SEQ_KEYFRAMES=20

### SequencePlayer.cpp (325 lines)
- Module-scope state ends at line 23 (seq_auto_run)
- beginInterp: lines 37-49 — for loop at line 38-39, no guard reset
- parseKeyframePayload: lines 54-93 — MAX_VALS=SEQ_NUM_MOTORS*4 (line 56), 32-val branch line 70, 17-val branch line 81
- Phase 1 loop: lines 198-213 — t_i calc at line 202, setTargetPosition at line 213
- all_lerps_done block: lines 238-252 — finalTarget call at line 242
- Phase 2: lines 260-283 — dest=finalTarget at line 264, setTargetPosition at line 265

### Motor.cpp constructor (line 4-7)
- Initializer list: current_pos(0.0f), current_vel(0.0f), target_pos(0.0f), target_vel(0.0f), target_pwm(0.0f), pos_pid(...), vel_pid(...), mode(DISABLED)
- current_load NOT in initializer list — must be added as in-class default (= 0.0f) in header OR added to constructor

## [2026-04-15] Guard field support wired through Python keyframes

- Added guard_threshold and guard_condition defaults to both Python Keyframe copies so JSON round-trips preserve 8-motor shape.
- ProtocolEncoder.send_keyframe now stays on the 32-value payload unless any guard_condition is non-zero, then emits the 48-value guarded payload.
- Existing curb_ascending.json keyframes still deserialize with zeroed guard arrays.
