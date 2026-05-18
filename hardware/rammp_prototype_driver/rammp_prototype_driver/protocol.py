"""
Serial protocol definitions for Teensy communication.

Teensy -> PC Protocol:
    TELEMETRY,<timestamp_ms>,<state>,<6 positions>,<6 velocities>,<6 pwms>\n

    Joint order: RC, FC, ML, MR, ML_Carriage, MR_Carriage (1-6)

    Example: TELEMETRY,12345,1,10.5,-5.2,20.0,15.3,0.5,1.2,0.1,0.2,0.3,0.4,0.5,0.6,100,200,300,400,500,600

PC -> Teensy Protocol:
    Set Target: T<joint_id>:<target>\n
        Example: T1:15.5
        Note: Target units depend on control mode (PWM, velocity, or position)

    Set Mode: M<joint_id>:<mode>\n
        Example: M1:2 (0: OPEN_LOOP, 1: VELOCITY, 2: POSITION)

    Set PID:
        P<joint_id>:<val>, I<joint_id>:<val>, D<joint_id>:<val> (Position PID)
        F<joint_id>:<val> (Position Feed-Forward)
        p<joint_id>:<val>, i<joint_id>:<val>, d<joint_id>:<val> (Velocity PID)
        f<joint_id>:<val> (Velocity Feed-Forward)
        Example: P1:0.5

    Reset PID: R<joint_id>\n
        Example: R1 (clears integrator windup and previous error)

    Stop Sine: X<joint_id>\n
        Example: X1 (stop sine wave on joint 1)

    ESTOP: z\n
    Clear ESTOP: c\n

    ODrive position (TUNER_MODE): o<axis>:<pos> then newline.
        axis 0 = both L+R same setpoint; 1 = left only; 2 = right only.
        Example: o0:1.25
"""

from dataclasses import dataclass, field
from typing import Optional, List
import re


@dataclass
class EncoderData:
    """Parsed telemetry data from Teensy."""

    timestamp_ms: int
    state: int
    position_values: List[float]  # 6 positions
    velocity_values: List[float] = field(default_factory=list)  # 6 velocities
    pwm_values: List[float] = field(default_factory=list)  # 6 pwms

    # New fields for motor config
    direction_values: List[int] = field(default_factory=list)  # 6 motor directions
    encoder_direction_values: List[int] = field(
        default_factory=list
    )  # 6 encoder directions
    limit_switches: List[bool] = field(
        default_factory=list
    )  # 4 switches [ML_fwd, ML_bwd, MR_fwd, MR_bwd]

    # New fields for IMU
    imu_pitch: float = 0.0
    imu_roll: float = 0.0
    imu_yaw: float = 0.0
    imu_ax: float = 0.0
    imu_ay: float = 0.0
    imu_az: float = 0.0

    # New fields for IMU quaternion
    imu_qw: float = 1.0
    imu_qx: float = 0.0
    imu_qy: float = 0.0
    imu_qz: float = 0.0

    # New fields for leveling debug
    leveling_pitch_err: float = 0.0
    leveling_roll_err: float = 0.0
    z_target_ml: float = 0.0
    z_target_rc: float = 0.0
    z_target_mr: float = 0.0
    has_leveling_data: bool = False  # True only when firmware sent the 49-field packet

    # Strain gauge (load cell) readings — filtered ADC counts
    sg_rc_value: float = 0.0
    sg_fc_value: float = 0.0
    sg_ml_value: float = 0.0
    sg_mr_value: float = 0.0

    # Control modes per motor (0=Open Loop, 1=Velocity, 2=Position)
    control_mode_values: List[int] = field(default_factory=list)

    # Drive wheel telemetry
    drive_fb_pos: float = 0.0
    drive_lr_pos: float = 0.0
    drive_fb_vel: float = 0.0
    drive_lr_vel: float = 0.0
    drive_fb_pwm: float = 0.0
    drive_lr_pwm: float = 0.0
    drive_fb_mode: int = 0
    drive_lr_mode: int = 0
    raw_ml_enc_pos: float = 0.0
    raw_mr_enc_pos: float = 0.0
    raw_ml_enc_vel: float = 0.0
    raw_mr_enc_vel: float = 0.0
    drive_fb_dir: int = 1
    drive_lr_dir: int = 1
    drive_fb_enc_dir: int = 1
    drive_lr_enc_dir: int = 1
    odrive_l_pos: float = 0.0
    odrive_r_pos: float = 0.0
    odrive_l_torque_nm: float = 0.0
    odrive_r_torque_nm: float = 0.0

    @property
    def num_joints(self) -> int:
        return len(self.position_values)

    def get_joint_value(self, joint_id: int) -> Optional[float]:
        """Get position value for joint (1-indexed)."""
        if 1 <= joint_id <= len(self.position_values):
            return self.position_values[joint_id - 1]
        return None


@dataclass
class ConfigData:
    """Parsed configuration data from Teensy."""

    joint_id: int
    pos_p: float
    pos_i: float
    pos_d: float
    pos_ff: float
    vel_p: float
    vel_i: float
    vel_d: float
    vel_ff: float
    pos_lpf_alpha: float = 1.0
    vel_lpf_alpha: float = 1.0
    input_lpf_alpha: float = 0.5
    pos_limit_min: int = 0
    pos_limit_max: int = 0
    pos_max_ramp_rate: float = 0.0
    vel_max_ramp_rate: float = 0.0
    motor_dir: int = 1
    encoder_dir: int = 1


@dataclass
class SeqAckData:
    """ACK response after a keyframe is successfully uploaded."""

    step_idx: int


@dataclass
class SeqStatusData:
    current_step: int
    total_steps: int
    state: int


@dataclass
class SeqGuardTrigData:
    motor_index: int
    load_value: float


class ProtocolParser:
    """Parse incoming serial data from Teensy."""

    # Matches: TELEMETRY,timestamp,state,<values>
    ENCODER_PATTERN = re.compile(r"^TELEMETRY,(\d+),(\d+),(.+)$")
    CONFIG_PATTERN = re.compile(r"^CONFIG,(\d+),(.+)$")
    SEQ_STATUS_PATTERN = re.compile(r"^SEQ_STATUS,(\d+),(\d+),(\d+)$")
    SEQ_ACK_PATTERN = re.compile(r"^SEQ_ACK,(\d+)$")
    SEQ_GUARD_TRIG_PATTERN = re.compile(r"^SEQ_GUARD_TRIG,m(\d+),load=(.+)$")

    NUM_JOINTS = 8

    @classmethod
    def parse_line(cls, line: str):
        """
        Parse a line of serial data from Teensy.

        Args:
            line: Raw line from serial (newline stripped)

        Returns:
            EncoderData, ConfigData, or None
        """
        line = line.strip()

        # Try matching config first
        config_match = cls.CONFIG_PATTERN.match(line)
        if config_match:
            try:
                joint_id = int(config_match.group(1))
                values_str = config_match.group(2)
                values = [float(v.strip()) for v in values_str.split(",")]
                if len(values) >= 8:
                    config_data = ConfigData(
                        joint_id=joint_id,
                        pos_p=values[0],
                        pos_i=values[1],
                        pos_d=values[2],
                        pos_ff=values[3],
                        vel_p=values[4],
                        vel_i=values[5],
                        vel_d=values[6],
                        vel_ff=values[7],
                    )
                    # Handle backwards compatibility for LPF alphas
                    if len(values) >= 11:
                        config_data.pos_lpf_alpha = values[8]
                        config_data.vel_lpf_alpha = values[9]
                        config_data.input_lpf_alpha = values[10]
                    # Handle new limit fields
                    if len(values) >= 13:
                        config_data.pos_limit_min = int(values[11])
                        config_data.pos_limit_max = int(values[12])
                    # Handle new ramp rate fields
                    if len(values) >= 15:
                        config_data.pos_max_ramp_rate = values[13]
                        config_data.vel_max_ramp_rate = values[14]
                    if len(values) >= 17:
                        config_data.motor_dir = int(values[15])
                        config_data.encoder_dir = int(values[16])
                    return config_data
            except (ValueError, IndexError):
                pass
            return None

        # Try SEQ_STATUS
        seq_status_match = cls.SEQ_STATUS_PATTERN.match(line)
        if seq_status_match:
            try:
                return SeqStatusData(
                    current_step=int(seq_status_match.group(1)),
                    total_steps=int(seq_status_match.group(2)),
                    state=int(seq_status_match.group(3)),
                )
            except (ValueError, IndexError):
                pass
            return None

        # Try SEQ_ACK
        seq_ack_match = cls.SEQ_ACK_PATTERN.match(line)
        if seq_ack_match:
            try:
                return SeqAckData(step_idx=int(seq_ack_match.group(1)))
            except (ValueError, IndexError):
                pass
            return None

        seq_guard_trig_match = cls.SEQ_GUARD_TRIG_PATTERN.match(line)
        if seq_guard_trig_match:
            try:
                return SeqGuardTrigData(
                    motor_index=int(seq_guard_trig_match.group(1)),
                    load_value=float(seq_guard_trig_match.group(2)),
                )
            except (ValueError, IndexError):
                pass
            return None

        match = cls.ENCODER_PATTERN.match(line)
        if match:
            try:
                timestamp = int(match.group(1))
                state = int(match.group(2))
                values_str = match.group(3)
                values = [float(v.strip()) for v in values_str.split(",")]

                # Newest format: 49 values (44 previous + 5 leveling debug)
                if len(values) >= 40:
                    data = EncoderData(
                        timestamp_ms=timestamp,
                        state=state,
                        position_values=values[0:6],
                        velocity_values=values[6:12],
                        pwm_values=values[12:18],
                        direction_values=[int(v) for v in values[18:24]],
                        encoder_direction_values=[int(v) for v in values[24:30]],
                        limit_switches=[bool(v) for v in values[30:34]],
                        imu_pitch=values[34],
                        imu_roll=values[35],
                        imu_yaw=values[36],
                        imu_ax=values[37],
                        imu_ay=values[38],
                        imu_az=values[39],
                    )
                    # Support new 44-value format with quaternion
                    if len(values) >= 44:
                        data.imu_qw = values[40]
                        data.imu_qx = values[41]
                        data.imu_qy = values[42]
                        data.imu_qz = values[43]
                    # Support newest 49-value format with leveling debug
                    if len(values) >= 49:
                        data.leveling_pitch_err = values[44]
                        data.leveling_roll_err = values[45]
                        data.z_target_ml = values[46]
                        data.z_target_rc = values[47]
                        data.z_target_mr = values[48]
                        data.has_leveling_data = True
                    # Support 53-value format with strain gauge readings
                    if len(values) >= 53:
                        data.sg_rc_value = values[49]
                        data.sg_fc_value = values[50]
                        data.sg_ml_value = values[51]
                        data.sg_mr_value = values[52]
                    # Support 59-value format with per-motor control modes
                    if len(values) >= 59:
                        data.control_mode_values = [int(v) for v in values[53:59]]
                    # Support 63-value format with drive wheel telemetry
                    if len(values) >= 63:
                        data.drive_fb_pos = values[59]
                        data.drive_lr_pos = values[60]
                        data.drive_fb_vel = values[61]
                        data.drive_lr_vel = values[62]
                    if len(values) >= 65:
                        data.drive_fb_pwm = values[63]
                        data.drive_lr_pwm = values[64]
                    if len(values) >= 67:
                        data.drive_fb_mode = int(values[65])
                        data.drive_lr_mode = int(values[66])
                    if len(values) >= 71:
                        data.raw_ml_enc_pos = values[67]
                        data.raw_mr_enc_pos = values[68]
                        data.raw_ml_enc_vel = values[69]
                        data.raw_mr_enc_vel = values[70]
                    if len(values) >= 75:
                        data.drive_fb_dir = int(values[71])
                        data.drive_lr_dir = int(values[72])
                        data.drive_fb_enc_dir = int(values[73])
                        data.drive_lr_enc_dir = int(values[74])
                    if len(values) >= 77:
                        data.odrive_l_pos = values[75]
                        data.odrive_r_pos = values[76]
                    if len(values) >= 79:
                        data.odrive_l_torque_nm = values[77]
                        data.odrive_r_torque_nm = values[78]
                    return data
                # Older format: 34 values (18 original + 6 dirs + 4 limits + 6 imu)
                elif len(values) == 34:
                    return EncoderData(
                        timestamp_ms=timestamp,
                        state=state,
                        position_values=values[0:6],
                        velocity_values=values[6:12],
                        pwm_values=values[12:18],
                        direction_values=[int(v) for v in values[18:24]],
                        encoder_direction_values=[1] * 6,  # default
                        limit_switches=[bool(v) for v in values[24:28]],
                        imu_pitch=values[28],
                        imu_roll=values[29],
                        imu_yaw=values[30],
                        imu_ax=values[31],
                        imu_ay=values[32],
                        imu_az=values[33],
                    )
                # Backwards compatibility: 18 values (original format)
                elif len(values) == 18:
                    return EncoderData(
                        timestamp_ms=timestamp,
                        state=state,
                        position_values=values[0:6],
                        velocity_values=values[6:12],
                        pwm_values=values[12:18],
                    )
                # Backwards compatibility: 6 values (oldest format)
                elif len(values) == 6:
                    return EncoderData(
                        timestamp_ms=timestamp,
                        state=state,
                        position_values=values,
                        velocity_values=[0.0] * cls.NUM_JOINTS,
                        pwm_values=[0.0] * cls.NUM_JOINTS,
                    )
                else:
                    return None
            except (ValueError, IndexError):
                return None

        return None


class ProtocolEncoder:
    """Encode commands to send to Teensy."""

    @staticmethod
    def set_target(joint_id: int, target_cm: float) -> bytes:
        """
        Create command to set target position for a joint.
        """
        cmd = f"T{joint_id}:{target_cm:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_mode(joint_id: int, mode: int) -> bytes:
        """
        Create command to set the control mode of a joint (0: Open Loop, 1: Vel, 2: Pos).
        """
        cmd = f"M{joint_id}:{mode}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_pid(joint_id: int, param: str, value: float) -> bytes:
        """
        Create command to set a PID parameter.
        param should be 'P', 'I', 'D', 'p', 'i', or 'd'.
        """
        cmd = f"{param}{joint_id}:{value:.4f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def step_input(joint_id: int, step_cm: float) -> bytes:
        """
        Create command for step input (relative change).
        """
        cmd = f"S{joint_id}:{step_cm:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def start_sine_wave(
        joint_id: int, amplitude_cm: float, frequency_hz: float, duration_s: float
    ) -> bytes:
        """
        Create command to start sine wave input.
        """
        cmd = f"W{joint_id}:{amplitude_cm:.2f},{frequency_hz:.3f},{duration_s:.1f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def stop_sine_wave(joint_id: int) -> bytes:
        """
        Create command to stop sine wave on a joint.
        """
        cmd = f"X{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def disable_motors() -> bytes:
        """
        Create command to disable all motors (emergency stop / safe mode).
        """
        cmd = "z\n"
        return cmd.encode("ascii")

    @staticmethod
    def clear_estop() -> bytes:
        """
        Create command to clear ESTOP state.
        """
        cmd = "c\n"
        return cmd.encode("ascii")

    @staticmethod
    def reset_pid(joint_id: int) -> bytes:
        """
        Create command to reset PID state (clear integrator windup).
        """
        cmd = f"R{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_feed_forward(joint_id: int, param: str, value: float) -> bytes:
        """
        Create command to set a feed-forward gain.
        param should be 'F' (position FF) or 'f' (velocity FF).
        """
        cmd = f"{param}{joint_id}:{value:.4f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def home_position(joint_id: int) -> bytes:
        """
        Create command to home/zero encoder position.
        """
        cmd = f"H{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def toggle_direction(joint_id: int) -> bytes:
        """
        Create command to toggle motor direction.
        """
        cmd = f"V{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def toggle_encoder_direction(joint_id: int) -> bytes:
        """
        Create command to toggle encoder direction.
        """
        cmd = f"E{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def save_config(joint_id: int) -> bytes:
        """
        Save configuration to EEPROM for joint.
        """
        cmd = f"K{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def get_config(joint_id: int) -> bytes:
        """
        Request configuration from EEPROM for joint.
        """
        cmd = f"G{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_pos_limit_min(joint_id: int, limit: int) -> bytes:
        """
        Set the minimum position limit for a joint.
        """
        cmd = f"n{joint_id}:{limit}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_pos_limit_max(joint_id: int, limit: int) -> bytes:
        """
        Set the maximum position limit for a joint.
        """
        cmd = f"x{joint_id}:{limit}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_pos_lpf(joint_id: int, alpha: float) -> bytes:
        return f"Q{joint_id}:{alpha:.4f}\n".encode("ascii")

    @staticmethod
    def set_vel_lpf(joint_id: int, alpha: float) -> bytes:
        return f"q{joint_id}:{alpha:.4f}\n".encode("ascii")

    @staticmethod
    def set_pos_ramp_rate(joint_id: int, max_rate: float) -> bytes:
        return f"U{joint_id}:{max_rate:.4f}\n".encode("ascii")

    @staticmethod
    def set_vel_ramp_rate(joint_id: int, max_rate: float) -> bytes:
        return f"u{joint_id}:{max_rate:.4f}\n".encode("ascii")

    @staticmethod
    def set_input_lpf(joint_id: int, alpha: float) -> bytes:
        return f"l{joint_id}:{alpha:.4f}\n".encode("ascii")

    @staticmethod
    def set_self_leveling(enable: bool) -> bytes:
        """
        Enable or disable self-leveling mode.
        """
        val = 1 if enable else 0
        cmd = f"L1:{val}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_imu_target(pitch: float, roll: float) -> bytes:
        """
        Set target pitch and roll for self-leveling.
        """
        # We send two commands back to back
        cmd = f"A1:{pitch:.2f}\nA2:{roll:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_position_offset(joint_id: int, desired_position: float) -> bytes:
        """
        Set encoder offset so current position reads as desired_position.
        Teensy calculates: offset = desired_position - current_raw_position
        Note: Requires firmware support for 'O' command.
        """
        cmd = f"O{joint_id}:{desired_position:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_odrive_position(axis_id: int, position: float) -> bytes:
        """
        Set ODrive position in TUNER_MODE (Teensy `o<id>:<pos>`).
        axis_id 0 = both L and R same setpoint; 1 = left only; 2 = right only.
        """
        return f"o{int(axis_id)}:{position:.4f}\n".encode("ascii")

    @staticmethod
    def set_odrive_velocity(axis_id: int, velocity: float) -> bytes:
        """
        Set ODrive velocity in TUNER_MODE (Teensy `y<id>:<vel>`).
        axis_id 0 = both L and R same setpoint; 1 = left only; 2 = right only.
        """
        return f"y{int(axis_id)}:{velocity:.4f}\n".encode("ascii")

    @staticmethod
    def enter_sequence_mode(enable: bool) -> bytes:
        val = 1 if enable else 0
        return f"B1:{val}\n".encode("ascii")

    @staticmethod
    def seq_auto_run(enable: bool) -> bytes:
        val = 1 if enable else 0
        return f"B2:{val}\n".encode("ascii")

    @staticmethod
    def send_keyframe(
        index: int,
        targets: list,
        active: list,
        duration_ms,
        relative: Optional[List[bool]] = None,
        guard_threshold: Optional[List[float]] = None,
        guard_condition: Optional[List[int]] = None,
        odrive_active: Optional[List[bool]] = None,
        odrive_relative: Optional[List[bool]] = None,
        odrive_targets: Optional[List[float]] = None,
    ) -> bytes:
        """
        Upload one keyframe.  Sends the new 32-value format:
        targets(8), active(8), relative(8), durations(8).
        duration_ms: single int (broadcast to all motors) or list of 8 ints.
        relative: list of 8 bools (default all False).

        If odrive_* arrays are provided, they are appended as additional blocks:
        odrive_active(8), odrive_relative(8), odrive_targets(8).
        This produces:
          - 56 values (7 blocks) with no guards
          - 72 values (9 blocks) with guards
        """
        t_str = ",".join(f"{t:.2f}" for t in targets)
        a_str = ",".join(str(int(bool(a))) for a in active)

        if relative is None:
            relative = [False] * 8
        r_str = ",".join(str(int(bool(r))) for r in relative)

        if isinstance(duration_ms, (int, float)):
            d_str = ",".join(str(int(duration_ms)) for _ in range(8))
        else:
            d_str = ",".join(str(int(d)) for d in duration_ms)

        has_odrive = (
            odrive_active is not None
            or odrive_relative is not None
            or odrive_targets is not None
        )
        if has_odrive:
            _oa = odrive_active if odrive_active is not None else [False] * 8
            _or = odrive_relative if odrive_relative is not None else [False] * 8
            _ot = odrive_targets if odrive_targets is not None else [0.0] * 8
            oa_str = ",".join(str(int(bool(v))) for v in _oa)
            or_str = ",".join(str(int(bool(v))) for v in _or)
            ot_str = ",".join(f"{float(v):.2f}" for v in _ot)

        _gt = guard_threshold if guard_threshold is not None else [0.0] * 8
        _gc = guard_condition if guard_condition is not None else [0] * 8
        has_guard = any(c != 0 for c in _gc)

        if has_guard:
            gt_str = ",".join(f"{v:.4f}" for v in _gt)
            gc_str = ",".join(str(int(v)) for v in _gc)
            return (
                f"J{index}:{t_str},{a_str},{r_str},{d_str},{gt_str},{gc_str}\n".encode(
                    "ascii"
                )
            )

        if has_odrive:
            return f"J{index}:{t_str},{a_str},{r_str},{d_str},{oa_str},{or_str},{ot_str}\n".encode(
                "ascii"
            )
        return f"J{index}:{t_str},{a_str},{r_str},{d_str}\n".encode("ascii")

    @staticmethod
    def seq_step_forward() -> bytes:
        """Step forward to the next keyframe."""
        return b">\n"

    @staticmethod
    def seq_step_backward() -> bytes:
        """Step backward to the previous keyframe."""
        return b"<\n"

    @staticmethod
    def seq_goto(step_idx: int) -> bytes:
        return f"@{step_idx}\n".encode("ascii")
