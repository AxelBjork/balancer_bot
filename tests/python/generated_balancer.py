"""Auto-generated IPC bindings using C++26 static reflection."""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

class MsgId(IntEnum):
    """Balancer UDP message identifiers."""
    PhysicsTick = 1
    MotorTargets = 3002
    SystemTelemetry = 3003
    SimStartRun = 3005
    SimStartAck = 3006
    SimStopRun = 3007
    SimRunDone = 3008
    ImuRawData = 3009
    SimulatorTelemetry = 3010
    ExternalJoystickCommand = 3011
    PidConfigOverride = 3012
    PidConfigStatus = 3013

BalancerMsgId = MsgId

@dataclass
class PhysicsTickPayload:
    WIRE_SIZE = 16
    dt_s: float
    timestamp_us: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<dQ", self.dt_s, self.timestamp_us))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "PhysicsTickPayload":
        offset = 0
        dt_s, timestamp_us = struct.unpack_from("<dQ", data, offset)
        offset += struct.calcsize("<dQ")
        return cls(dt_s=dt_s, timestamp_us=timestamp_us)

    @classmethod
    def unpack(cls, data: bytes) -> "PhysicsTickPayload":
        return cls.unpack_wire(data)

@dataclass
class MotorTargetsPayload:
    WIRE_SIZE = 16
    left_sps: float
    right_sps: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<dd", self.left_sps, self.right_sps))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "MotorTargetsPayload":
        offset = 0
        left_sps, right_sps = struct.unpack_from("<dd", data, offset)
        offset += struct.calcsize("<dd")
        return cls(left_sps=left_sps, right_sps=right_sps)

    @classmethod
    def unpack(cls, data: bytes) -> "MotorTargetsPayload":
        return cls.unpack_wire(data)

@dataclass
class SystemTelemetryPayload:
    WIRE_SIZE = 392
    run_id: int
    controller_fault_flags: int
    controller_saturation_flags: int
    imu_timestamp_us: int
    t_sec: float
    age_ms: float
    pitch_deg: float
    pitch_rate_dps: float
    raw_acc_pitch_deg: float
    fused_pitch_deg: float
    gyro_pitch_rate_dps: float
    filtered_pitch_rate_dps: float
    u_sps: float
    turn_sps: float
    nominal_acceleration_mps2: float
    raw_completed_velocity_sps: float
    corrected_axle_velocity_sps: float
    velocity_damping_acceleration_mps2: float
    com_trim_deg: float
    pitch_error_deg: float
    pitch_sp_deg: float
    left_target_sps: float
    right_target_sps: float
    left_slewed_sps: float
    right_slewed_sps: float
    motor_update_dt_ms: float
    motor_feedback_age_ms: float
    left_actual_steps: int
    right_actual_steps: int
    actuator_saturation_flags: int
    command_saturated: bool
    actuator_fault: bool
    trim_learning_enabled: int
    trim_learning_block_reason: int
    trim_learning_reserved: int
    pitch_feedback_sps: float
    pitch_rate_feedback_sps: float
    pitch_accel_feedback_sps: float
    velocity_pitch_target_deg: float
    balance_unclamped_sps: float
    active_pitch_gain_sps_per_rad: float
    active_pitch_rate_gain_sps_per_rad_s: float
    active_pitch_accel_gain_sps_per_rad_s2: float
    active_velocity_pitch_gain_rad_per_sps: float
    active_velocity_control_cutoff_hz: float
    active_velocity_observer_cutoff_hz: float
    active_com_trim_gain_deg_per_sps_s: float
    active_com_trim_limit_deg: float
    active_accel_lpf_hz: float
    active_gyro_derivative_lpf_hz: float
    active_config_generation: int
    velocity_pitch_request_unclamped_deg: float
    velocity_pitch_request_limited_deg: float
    pitch_target_unclamped_deg: float
    active_velocity_pitch_limit_deg: float
    trim_quiet_rate_rms_dps: float
    velocity_authority_limited: bool
    trim_trusted: bool
    trim_learning_allowed: bool
    pitch_target_limit_reason: int
    velocity_control_sps: float
    completed_step_acceleration_sps2: float
    packet_seq: int
    loop_seq: int
    sender_monotonic_ns: int
    user_velocity_mps: float
    reference_velocity_mps: float
    reference_acceleration_mps2: float
    velocity_feedback_estimate_mps: float
    velocity_error_mps: float
    velocity_feedback_acceleration_mps2: float
    acceleration_raw_mps2: float
    acceleration_cmd_mps2: float
    drive_pitch_target_deg: float
    fixed_com_trim_deg: float
    velocity_feedback_valid: bool
    velocity_feedback_active: bool
    outer_acceleration_limited: bool
    outer_pitch_target_limited: bool
    active_drive_max_velocity_mps: float
    active_drive_max_acceleration_mps2: float
    active_drive_max_deceleration_mps2: float
    active_velocity_gain_per_s: float
    active_velocity_feedback_cutoff_hz: float
    active_outer_pitch_limit_deg: float
    active_fixed_com_trim_deg: float
    adaptive_com_trim_enabled: bool
    legacy_outer_fields_valid: bool
    reference_jerk_mps3: float
    velocity_p_acceleration_mps2: float
    velocity_i_acceleration_mps2: float
    velocity_integral_state_mps_s: float
    final_pitch_target_deg: float
    active_planner_max_acceleration_mps2: float
    active_planner_max_deceleration_mps2: float
    active_planner_max_jerk_mps3: float
    active_velocity_i_gain_per_s2: float
    active_velocity_i_leak_time_s: float
    active_velocity_i_acceleration_limit_mps2: float
    planner_acceleration_limited: bool
    planner_jerk_limited: bool
    velocity_integral_limited: bool
    velocity_anti_windup_active: bool

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<III4xQfffffffffffffffffffffffiiI??BBH2xfffffffffffffff4xQfffff???BffQQQffffffffff????fffffff??2xfffffffffff????4x", self.run_id, self.controller_fault_flags, self.controller_saturation_flags, self.imu_timestamp_us, self.t_sec, self.age_ms, self.pitch_deg, self.pitch_rate_dps, self.raw_acc_pitch_deg, self.fused_pitch_deg, self.gyro_pitch_rate_dps, self.filtered_pitch_rate_dps, self.u_sps, self.turn_sps, self.nominal_acceleration_mps2, self.raw_completed_velocity_sps, self.corrected_axle_velocity_sps, self.velocity_damping_acceleration_mps2, self.com_trim_deg, self.pitch_error_deg, self.pitch_sp_deg, self.left_target_sps, self.right_target_sps, self.left_slewed_sps, self.right_slewed_sps, self.motor_update_dt_ms, self.motor_feedback_age_ms, self.left_actual_steps, self.right_actual_steps, self.actuator_saturation_flags, self.command_saturated, self.actuator_fault, self.trim_learning_enabled, self.trim_learning_block_reason, self.trim_learning_reserved, self.pitch_feedback_sps, self.pitch_rate_feedback_sps, self.pitch_accel_feedback_sps, self.velocity_pitch_target_deg, self.balance_unclamped_sps, self.active_pitch_gain_sps_per_rad, self.active_pitch_rate_gain_sps_per_rad_s, self.active_pitch_accel_gain_sps_per_rad_s2, self.active_velocity_pitch_gain_rad_per_sps, self.active_velocity_control_cutoff_hz, self.active_velocity_observer_cutoff_hz, self.active_com_trim_gain_deg_per_sps_s, self.active_com_trim_limit_deg, self.active_accel_lpf_hz, self.active_gyro_derivative_lpf_hz, self.active_config_generation, self.velocity_pitch_request_unclamped_deg, self.velocity_pitch_request_limited_deg, self.pitch_target_unclamped_deg, self.active_velocity_pitch_limit_deg, self.trim_quiet_rate_rms_dps, self.velocity_authority_limited, self.trim_trusted, self.trim_learning_allowed, self.pitch_target_limit_reason, self.velocity_control_sps, self.completed_step_acceleration_sps2, self.packet_seq, self.loop_seq, self.sender_monotonic_ns, self.user_velocity_mps, self.reference_velocity_mps, self.reference_acceleration_mps2, self.velocity_feedback_estimate_mps, self.velocity_error_mps, self.velocity_feedback_acceleration_mps2, self.acceleration_raw_mps2, self.acceleration_cmd_mps2, self.drive_pitch_target_deg, self.fixed_com_trim_deg, self.velocity_feedback_valid, self.velocity_feedback_active, self.outer_acceleration_limited, self.outer_pitch_target_limited, self.active_drive_max_velocity_mps, self.active_drive_max_acceleration_mps2, self.active_drive_max_deceleration_mps2, self.active_velocity_gain_per_s, self.active_velocity_feedback_cutoff_hz, self.active_outer_pitch_limit_deg, self.active_fixed_com_trim_deg, self.adaptive_com_trim_enabled, self.legacy_outer_fields_valid, self.reference_jerk_mps3, self.velocity_p_acceleration_mps2, self.velocity_i_acceleration_mps2, self.velocity_integral_state_mps_s, self.final_pitch_target_deg, self.active_planner_max_acceleration_mps2, self.active_planner_max_deceleration_mps2, self.active_planner_max_jerk_mps3, self.active_velocity_i_gain_per_s2, self.active_velocity_i_leak_time_s, self.active_velocity_i_acceleration_limit_mps2, self.planner_acceleration_limited, self.planner_jerk_limited, self.velocity_integral_limited, self.velocity_anti_windup_active))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SystemTelemetryPayload":
        offset = 0
        run_id, controller_fault_flags, controller_saturation_flags, imu_timestamp_us, t_sec, age_ms, pitch_deg, pitch_rate_dps, raw_acc_pitch_deg, fused_pitch_deg, gyro_pitch_rate_dps, filtered_pitch_rate_dps, u_sps, turn_sps, nominal_acceleration_mps2, raw_completed_velocity_sps, corrected_axle_velocity_sps, velocity_damping_acceleration_mps2, com_trim_deg, pitch_error_deg, pitch_sp_deg, left_target_sps, right_target_sps, left_slewed_sps, right_slewed_sps, motor_update_dt_ms, motor_feedback_age_ms, left_actual_steps, right_actual_steps, actuator_saturation_flags, command_saturated, actuator_fault, trim_learning_enabled, trim_learning_block_reason, trim_learning_reserved, pitch_feedback_sps, pitch_rate_feedback_sps, pitch_accel_feedback_sps, velocity_pitch_target_deg, balance_unclamped_sps, active_pitch_gain_sps_per_rad, active_pitch_rate_gain_sps_per_rad_s, active_pitch_accel_gain_sps_per_rad_s2, active_velocity_pitch_gain_rad_per_sps, active_velocity_control_cutoff_hz, active_velocity_observer_cutoff_hz, active_com_trim_gain_deg_per_sps_s, active_com_trim_limit_deg, active_accel_lpf_hz, active_gyro_derivative_lpf_hz, active_config_generation, velocity_pitch_request_unclamped_deg, velocity_pitch_request_limited_deg, pitch_target_unclamped_deg, active_velocity_pitch_limit_deg, trim_quiet_rate_rms_dps, velocity_authority_limited, trim_trusted, trim_learning_allowed, pitch_target_limit_reason, velocity_control_sps, completed_step_acceleration_sps2, packet_seq, loop_seq, sender_monotonic_ns, user_velocity_mps, reference_velocity_mps, reference_acceleration_mps2, velocity_feedback_estimate_mps, velocity_error_mps, velocity_feedback_acceleration_mps2, acceleration_raw_mps2, acceleration_cmd_mps2, drive_pitch_target_deg, fixed_com_trim_deg, velocity_feedback_valid, velocity_feedback_active, outer_acceleration_limited, outer_pitch_target_limited, active_drive_max_velocity_mps, active_drive_max_acceleration_mps2, active_drive_max_deceleration_mps2, active_velocity_gain_per_s, active_velocity_feedback_cutoff_hz, active_outer_pitch_limit_deg, active_fixed_com_trim_deg, adaptive_com_trim_enabled, legacy_outer_fields_valid, reference_jerk_mps3, velocity_p_acceleration_mps2, velocity_i_acceleration_mps2, velocity_integral_state_mps_s, final_pitch_target_deg, active_planner_max_acceleration_mps2, active_planner_max_deceleration_mps2, active_planner_max_jerk_mps3, active_velocity_i_gain_per_s2, active_velocity_i_leak_time_s, active_velocity_i_acceleration_limit_mps2, planner_acceleration_limited, planner_jerk_limited, velocity_integral_limited, velocity_anti_windup_active = struct.unpack_from("<III4xQfffffffffffffffffffffffiiI??BBH2xfffffffffffffff4xQfffff???BffQQQffffffffff????fffffff??2xfffffffffff????4x", data, offset)
        offset += struct.calcsize("<III4xQfffffffffffffffffffffffiiI??BBH2xfffffffffffffff4xQfffff???BffQQQffffffffff????fffffff??2xfffffffffff????4x")
        return cls(run_id=run_id, controller_fault_flags=controller_fault_flags, controller_saturation_flags=controller_saturation_flags, imu_timestamp_us=imu_timestamp_us, t_sec=t_sec, age_ms=age_ms, pitch_deg=pitch_deg, pitch_rate_dps=pitch_rate_dps, raw_acc_pitch_deg=raw_acc_pitch_deg, fused_pitch_deg=fused_pitch_deg, gyro_pitch_rate_dps=gyro_pitch_rate_dps, filtered_pitch_rate_dps=filtered_pitch_rate_dps, u_sps=u_sps, turn_sps=turn_sps, nominal_acceleration_mps2=nominal_acceleration_mps2, raw_completed_velocity_sps=raw_completed_velocity_sps, corrected_axle_velocity_sps=corrected_axle_velocity_sps, velocity_damping_acceleration_mps2=velocity_damping_acceleration_mps2, com_trim_deg=com_trim_deg, pitch_error_deg=pitch_error_deg, pitch_sp_deg=pitch_sp_deg, left_target_sps=left_target_sps, right_target_sps=right_target_sps, left_slewed_sps=left_slewed_sps, right_slewed_sps=right_slewed_sps, motor_update_dt_ms=motor_update_dt_ms, motor_feedback_age_ms=motor_feedback_age_ms, left_actual_steps=left_actual_steps, right_actual_steps=right_actual_steps, actuator_saturation_flags=actuator_saturation_flags, command_saturated=command_saturated, actuator_fault=actuator_fault, trim_learning_enabled=trim_learning_enabled, trim_learning_block_reason=trim_learning_block_reason, trim_learning_reserved=trim_learning_reserved, pitch_feedback_sps=pitch_feedback_sps, pitch_rate_feedback_sps=pitch_rate_feedback_sps, pitch_accel_feedback_sps=pitch_accel_feedback_sps, velocity_pitch_target_deg=velocity_pitch_target_deg, balance_unclamped_sps=balance_unclamped_sps, active_pitch_gain_sps_per_rad=active_pitch_gain_sps_per_rad, active_pitch_rate_gain_sps_per_rad_s=active_pitch_rate_gain_sps_per_rad_s, active_pitch_accel_gain_sps_per_rad_s2=active_pitch_accel_gain_sps_per_rad_s2, active_velocity_pitch_gain_rad_per_sps=active_velocity_pitch_gain_rad_per_sps, active_velocity_control_cutoff_hz=active_velocity_control_cutoff_hz, active_velocity_observer_cutoff_hz=active_velocity_observer_cutoff_hz, active_com_trim_gain_deg_per_sps_s=active_com_trim_gain_deg_per_sps_s, active_com_trim_limit_deg=active_com_trim_limit_deg, active_accel_lpf_hz=active_accel_lpf_hz, active_gyro_derivative_lpf_hz=active_gyro_derivative_lpf_hz, active_config_generation=active_config_generation, velocity_pitch_request_unclamped_deg=velocity_pitch_request_unclamped_deg, velocity_pitch_request_limited_deg=velocity_pitch_request_limited_deg, pitch_target_unclamped_deg=pitch_target_unclamped_deg, active_velocity_pitch_limit_deg=active_velocity_pitch_limit_deg, trim_quiet_rate_rms_dps=trim_quiet_rate_rms_dps, velocity_authority_limited=velocity_authority_limited, trim_trusted=trim_trusted, trim_learning_allowed=trim_learning_allowed, pitch_target_limit_reason=pitch_target_limit_reason, velocity_control_sps=velocity_control_sps, completed_step_acceleration_sps2=completed_step_acceleration_sps2, packet_seq=packet_seq, loop_seq=loop_seq, sender_monotonic_ns=sender_monotonic_ns, user_velocity_mps=user_velocity_mps, reference_velocity_mps=reference_velocity_mps, reference_acceleration_mps2=reference_acceleration_mps2, velocity_feedback_estimate_mps=velocity_feedback_estimate_mps, velocity_error_mps=velocity_error_mps, velocity_feedback_acceleration_mps2=velocity_feedback_acceleration_mps2, acceleration_raw_mps2=acceleration_raw_mps2, acceleration_cmd_mps2=acceleration_cmd_mps2, drive_pitch_target_deg=drive_pitch_target_deg, fixed_com_trim_deg=fixed_com_trim_deg, velocity_feedback_valid=velocity_feedback_valid, velocity_feedback_active=velocity_feedback_active, outer_acceleration_limited=outer_acceleration_limited, outer_pitch_target_limited=outer_pitch_target_limited, active_drive_max_velocity_mps=active_drive_max_velocity_mps, active_drive_max_acceleration_mps2=active_drive_max_acceleration_mps2, active_drive_max_deceleration_mps2=active_drive_max_deceleration_mps2, active_velocity_gain_per_s=active_velocity_gain_per_s, active_velocity_feedback_cutoff_hz=active_velocity_feedback_cutoff_hz, active_outer_pitch_limit_deg=active_outer_pitch_limit_deg, active_fixed_com_trim_deg=active_fixed_com_trim_deg, adaptive_com_trim_enabled=adaptive_com_trim_enabled, legacy_outer_fields_valid=legacy_outer_fields_valid, reference_jerk_mps3=reference_jerk_mps3, velocity_p_acceleration_mps2=velocity_p_acceleration_mps2, velocity_i_acceleration_mps2=velocity_i_acceleration_mps2, velocity_integral_state_mps_s=velocity_integral_state_mps_s, final_pitch_target_deg=final_pitch_target_deg, active_planner_max_acceleration_mps2=active_planner_max_acceleration_mps2, active_planner_max_deceleration_mps2=active_planner_max_deceleration_mps2, active_planner_max_jerk_mps3=active_planner_max_jerk_mps3, active_velocity_i_gain_per_s2=active_velocity_i_gain_per_s2, active_velocity_i_leak_time_s=active_velocity_i_leak_time_s, active_velocity_i_acceleration_limit_mps2=active_velocity_i_acceleration_limit_mps2, planner_acceleration_limited=planner_acceleration_limited, planner_jerk_limited=planner_jerk_limited, velocity_integral_limited=velocity_integral_limited, velocity_anti_windup_active=velocity_anti_windup_active)

    @classmethod
    def unpack(cls, data: bytes) -> "SystemTelemetryPayload":
        return cls.unpack_wire(data)

@dataclass
class SimDisturbancePayload:
    WIRE_SIZE = 56
    kind: int
    reserved0: int
    reserved1: int
    start_s: float
    duration_s: float
    force_n: float
    com_bias_rad: float
    force_n_end: float
    com_bias_rad_end: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<BBH4xdddddd", self.kind, self.reserved0, self.reserved1, self.start_s, self.duration_s, self.force_n, self.com_bias_rad, self.force_n_end, self.com_bias_rad_end))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimDisturbancePayload":
        offset = 0
        kind, reserved0, reserved1, start_s, duration_s, force_n, com_bias_rad, force_n_end, com_bias_rad_end = struct.unpack_from("<BBH4xdddddd", data, offset)
        offset += struct.calcsize("<BBH4xdddddd")
        return cls(kind=kind, reserved0=reserved0, reserved1=reserved1, start_s=start_s, duration_s=duration_s, force_n=force_n, com_bias_rad=com_bias_rad, force_n_end=force_n_end, com_bias_rad_end=com_bias_rad_end)

    @classmethod
    def unpack(cls, data: bytes) -> "SimDisturbancePayload":
        return cls.unpack_wire(data)

@dataclass
class SimJoySegmentPayload:
    WIRE_SIZE = 48
    start_s: float
    duration_s: float
    forward: float
    turn: float
    forward_end: float
    turn_end: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<dddddd", self.start_s, self.duration_s, self.forward, self.turn, self.forward_end, self.turn_end))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimJoySegmentPayload":
        offset = 0
        start_s, duration_s, forward, turn, forward_end, turn_end = struct.unpack_from("<dddddd", data, offset)
        offset += struct.calcsize("<dddddd")
        return cls(start_s=start_s, duration_s=duration_s, forward=forward, turn=turn, forward_end=forward_end, turn_end=turn_end)

    @classmethod
    def unpack(cls, data: bytes) -> "SimJoySegmentPayload":
        return cls.unpack_wire(data)

@dataclass
class SimBraceRestEventPayload:
    WIRE_SIZE = 16
    start_s: float
    pitch_deg: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<dd", self.start_s, self.pitch_deg))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimBraceRestEventPayload":
        offset = 0
        start_s, pitch_deg = struct.unpack_from("<dd", data, offset)
        offset += struct.calcsize("<dd")
        return cls(start_s=start_s, pitch_deg=pitch_deg)

    @classmethod
    def unpack(cls, data: bytes) -> "SimBraceRestEventPayload":
        return cls.unpack_wire(data)

@dataclass
class SimStartRunPayload:
    WIRE_SIZE = 1264
    run_id: int
    physics_profile: int
    has_physics_override: int
    telemetry_stride: int
    transfer_scenario_index: int
    reserved1: int
    duration_s: float
    initial_pitch_deg: float
    com_angle_offset_rad: float
    total_mass_scale: float
    pitch_inertia_scale: float
    motor_max_force_n: float
    motor_no_load_speed_mps: float
    motor_velocity_damping: float
    motor_tau_s: float
    traction_coefficient: float
    pitch_damping: float
    cart_damping: float
    phase_error_limit_steps: float
    tire_stiffness_n_per_m: float
    tire_damping_n_s_per_m: float
    wheel_equivalent_mass_kg: float
    imu_pitch_lag_s: float
    imu_noise_seed: int
    accel_noise_std_mps2: float
    gyro_noise_std_rad_s: float
    imu_timestamp_jitter_us: float
    imu_sample_loss_rate: float
    accel_bias_mps2: list[float]
    gyro_bias_rad_s: list[float]
    disturbances: list[SimDisturbancePayload]
    joy_segments: list[SimJoySegmentPayload]
    pid_config_path: bytes
    initial_velocity_mps: float
    velocity_estimator_bias_mps: float
    velocity_estimator_bias_drift_mps_per_s: float
    velocity_estimator_scale: float
    velocity_estimator_latency_s: float
    initial_pitch_rate_dps: float
    brace_enabled: int
    brace_reserved0: int
    brace_reserved1: int
    brace_pitch_deg: float
    brace_stiffness_nm_per_rad: float
    brace_damping_nm_s_per_rad: float
    brace_rest_events: list[SimBraceRestEventPayload]

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBHHH4xdddddddddddddddddI4xdddd", self.run_id, self.physics_profile, self.has_physics_override, self.telemetry_stride, self.transfer_scenario_index, self.reserved1, self.duration_s, self.initial_pitch_deg, self.com_angle_offset_rad, self.total_mass_scale, self.pitch_inertia_scale, self.motor_max_force_n, self.motor_no_load_speed_mps, self.motor_velocity_damping, self.motor_tau_s, self.traction_coefficient, self.pitch_damping, self.cart_damping, self.phase_error_limit_steps, self.tire_stiffness_n_per_m, self.tire_damping_n_s_per_m, self.wheel_equivalent_mass_kg, self.imu_pitch_lag_s, self.imu_noise_seed, self.accel_noise_std_mps2, self.gyro_noise_std_rad_s, self.imu_timestamp_jitter_us, self.imu_sample_loss_rate))
        data.extend(struct.pack("<3d", *self.accel_bias_mps2))
        data.extend(struct.pack("<3d", *self.gyro_bias_rad_s))
        for item in self.disturbances:
            if not hasattr(item, 'pack_wire'):
                if isinstance(item, tuple):
                    item = SimDisturbancePayload(*item)
                elif isinstance(item, dict):
                    item = SimDisturbancePayload(**item)
                else:
                    item = SimDisturbancePayload(item)
            data.extend(item.pack_wire())
        for item in self.joy_segments:
            if not hasattr(item, 'pack_wire'):
                if isinstance(item, tuple):
                    item = SimJoySegmentPayload(*item)
                elif isinstance(item, dict):
                    item = SimJoySegmentPayload(**item)
                else:
                    item = SimJoySegmentPayload(item)
            data.extend(item.pack_wire())
        data.extend(struct.pack("<128sddddddBBH4xddd", self.pid_config_path, self.initial_velocity_mps, self.velocity_estimator_bias_mps, self.velocity_estimator_bias_drift_mps_per_s, self.velocity_estimator_scale, self.velocity_estimator_latency_s, self.initial_pitch_rate_dps, self.brace_enabled, self.brace_reserved0, self.brace_reserved1, self.brace_pitch_deg, self.brace_stiffness_nm_per_rad, self.brace_damping_nm_s_per_rad))
        for item in self.brace_rest_events:
            if not hasattr(item, 'pack_wire'):
                if isinstance(item, tuple):
                    item = SimBraceRestEventPayload(*item)
                elif isinstance(item, dict):
                    item = SimBraceRestEventPayload(**item)
                else:
                    item = SimBraceRestEventPayload(item)
            data.extend(item.pack_wire())
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimStartRunPayload":
        offset = 0
        run_id, physics_profile, has_physics_override, telemetry_stride, transfer_scenario_index, reserved1, duration_s, initial_pitch_deg, com_angle_offset_rad, total_mass_scale, pitch_inertia_scale, motor_max_force_n, motor_no_load_speed_mps, motor_velocity_damping, motor_tau_s, traction_coefficient, pitch_damping, cart_damping, phase_error_limit_steps, tire_stiffness_n_per_m, tire_damping_n_s_per_m, wheel_equivalent_mass_kg, imu_pitch_lag_s, imu_noise_seed, accel_noise_std_mps2, gyro_noise_std_rad_s, imu_timestamp_jitter_us, imu_sample_loss_rate = struct.unpack_from("<IBBHHH4xdddddddddddddddddI4xdddd", data, offset)
        offset += struct.calcsize("<IBBHHH4xdddddddddddddddddI4xdddd")
        accel_bias_mps2 = list(struct.unpack_from("<3d", data, offset))
        offset += struct.calcsize("<3d")
        gyro_bias_rad_s = list(struct.unpack_from("<3d", data, offset))
        offset += struct.calcsize("<3d")
        disturbances = []
        for _ in range(10):
            sub_size = SimDisturbancePayload.WIRE_SIZE
            item = SimDisturbancePayload.unpack_wire(data[offset:offset+sub_size])
            disturbances.append(item)
            offset += sub_size
        joy_segments = []
        for _ in range(4):
            sub_size = SimJoySegmentPayload.WIRE_SIZE
            item = SimJoySegmentPayload.unpack_wire(data[offset:offset+sub_size])
            joy_segments.append(item)
            offset += sub_size
        pid_config_path, initial_velocity_mps, velocity_estimator_bias_mps, velocity_estimator_bias_drift_mps_per_s, velocity_estimator_scale, velocity_estimator_latency_s, initial_pitch_rate_dps, brace_enabled, brace_reserved0, brace_reserved1, brace_pitch_deg, brace_stiffness_nm_per_rad, brace_damping_nm_s_per_rad = struct.unpack_from("<128sddddddBBH4xddd", data, offset)
        offset += struct.calcsize("<128sddddddBBH4xddd")
        brace_rest_events = []
        for _ in range(4):
            sub_size = SimBraceRestEventPayload.WIRE_SIZE
            item = SimBraceRestEventPayload.unpack_wire(data[offset:offset+sub_size])
            brace_rest_events.append(item)
            offset += sub_size
        return cls(run_id=run_id, physics_profile=physics_profile, has_physics_override=has_physics_override, telemetry_stride=telemetry_stride, transfer_scenario_index=transfer_scenario_index, reserved1=reserved1, duration_s=duration_s, initial_pitch_deg=initial_pitch_deg, com_angle_offset_rad=com_angle_offset_rad, total_mass_scale=total_mass_scale, pitch_inertia_scale=pitch_inertia_scale, motor_max_force_n=motor_max_force_n, motor_no_load_speed_mps=motor_no_load_speed_mps, motor_velocity_damping=motor_velocity_damping, motor_tau_s=motor_tau_s, traction_coefficient=traction_coefficient, pitch_damping=pitch_damping, cart_damping=cart_damping, phase_error_limit_steps=phase_error_limit_steps, tire_stiffness_n_per_m=tire_stiffness_n_per_m, tire_damping_n_s_per_m=tire_damping_n_s_per_m, wheel_equivalent_mass_kg=wheel_equivalent_mass_kg, imu_pitch_lag_s=imu_pitch_lag_s, imu_noise_seed=imu_noise_seed, accel_noise_std_mps2=accel_noise_std_mps2, gyro_noise_std_rad_s=gyro_noise_std_rad_s, imu_timestamp_jitter_us=imu_timestamp_jitter_us, imu_sample_loss_rate=imu_sample_loss_rate, accel_bias_mps2=accel_bias_mps2, gyro_bias_rad_s=gyro_bias_rad_s, disturbances=disturbances, joy_segments=joy_segments, pid_config_path=pid_config_path, initial_velocity_mps=initial_velocity_mps, velocity_estimator_bias_mps=velocity_estimator_bias_mps, velocity_estimator_bias_drift_mps_per_s=velocity_estimator_bias_drift_mps_per_s, velocity_estimator_scale=velocity_estimator_scale, velocity_estimator_latency_s=velocity_estimator_latency_s, initial_pitch_rate_dps=initial_pitch_rate_dps, brace_enabled=brace_enabled, brace_reserved0=brace_reserved0, brace_reserved1=brace_reserved1, brace_pitch_deg=brace_pitch_deg, brace_stiffness_nm_per_rad=brace_stiffness_nm_per_rad, brace_damping_nm_s_per_rad=brace_damping_nm_s_per_rad, brace_rest_events=brace_rest_events)

    @classmethod
    def unpack(cls, data: bytes) -> "SimStartRunPayload":
        return cls.unpack_wire(data)

@dataclass
class SimStartAckPayload:
    WIRE_SIZE = 8
    run_id: int
    accepted: int
    status_code: int
    reserved: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBH", self.run_id, self.accepted, self.status_code, self.reserved))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimStartAckPayload":
        offset = 0
        run_id, accepted, status_code, reserved = struct.unpack_from("<IBBH", data, offset)
        offset += struct.calcsize("<IBBH")
        return cls(run_id=run_id, accepted=accepted, status_code=status_code, reserved=reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "SimStartAckPayload":
        return cls.unpack_wire(data)

@dataclass
class SimStopRunPayload:
    WIRE_SIZE = 4
    run_id: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<I", self.run_id))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimStopRunPayload":
        offset = 0
        run_id = struct.unpack_from("<I", data, offset)[0]
        offset += struct.calcsize("<I")
        return cls(run_id=run_id)

    @classmethod
    def unpack(cls, data: bytes) -> "SimStopRunPayload":
        return cls.unpack_wire(data)

@dataclass
class SimRunDonePayload:
    WIRE_SIZE = 96
    run_id: int
    reason_code: int
    reserved0: int
    reserved1: int
    sample_count: int
    elapsed_s: float
    final_pitch_deg: float
    max_abs_pitch_deg: float
    tail_rms_pitch_deg: float
    tail_rail_fraction: float
    tail_mean_abs_pitch_deg: float
    max_abs_position_m: float
    tail_mean_abs_velocity_mps: float
    max_continuous_saturation_s: float
    actuator_fault_count: int
    controller_fault_flags: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBHI4xdddddddddII", self.run_id, self.reason_code, self.reserved0, self.reserved1, self.sample_count, self.elapsed_s, self.final_pitch_deg, self.max_abs_pitch_deg, self.tail_rms_pitch_deg, self.tail_rail_fraction, self.tail_mean_abs_pitch_deg, self.max_abs_position_m, self.tail_mean_abs_velocity_mps, self.max_continuous_saturation_s, self.actuator_fault_count, self.controller_fault_flags))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimRunDonePayload":
        offset = 0
        run_id, reason_code, reserved0, reserved1, sample_count, elapsed_s, final_pitch_deg, max_abs_pitch_deg, tail_rms_pitch_deg, tail_rail_fraction, tail_mean_abs_pitch_deg, max_abs_position_m, tail_mean_abs_velocity_mps, max_continuous_saturation_s, actuator_fault_count, controller_fault_flags = struct.unpack_from("<IBBHI4xdddddddddII", data, offset)
        offset += struct.calcsize("<IBBHI4xdddddddddII")
        return cls(run_id=run_id, reason_code=reason_code, reserved0=reserved0, reserved1=reserved1, sample_count=sample_count, elapsed_s=elapsed_s, final_pitch_deg=final_pitch_deg, max_abs_pitch_deg=max_abs_pitch_deg, tail_rms_pitch_deg=tail_rms_pitch_deg, tail_rail_fraction=tail_rail_fraction, tail_mean_abs_pitch_deg=tail_mean_abs_pitch_deg, max_abs_position_m=max_abs_position_m, tail_mean_abs_velocity_mps=tail_mean_abs_velocity_mps, max_continuous_saturation_s=max_continuous_saturation_s, actuator_fault_count=actuator_fault_count, controller_fault_flags=controller_fault_flags)

    @classmethod
    def unpack(cls, data: bytes) -> "SimRunDonePayload":
        return cls.unpack_wire(data)

@dataclass
class ImuRawPayload:
    WIRE_SIZE = 56
    acc: list[float]
    gyr: list[float]
    timestamp_us: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<3d", *self.acc))
        data.extend(struct.pack("<3d", *self.gyr))
        data.extend(struct.pack("<Q", self.timestamp_us))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "ImuRawPayload":
        offset = 0
        acc = list(struct.unpack_from("<3d", data, offset))
        offset += struct.calcsize("<3d")
        gyr = list(struct.unpack_from("<3d", data, offset))
        offset += struct.calcsize("<3d")
        timestamp_us = struct.unpack_from("<Q", data, offset)[0]
        offset += struct.calcsize("<Q")
        return cls(acc=acc, gyr=gyr, timestamp_us=timestamp_us)

    @classmethod
    def unpack(cls, data: bytes) -> "ImuRawPayload":
        return cls.unpack_wire(data)

@dataclass
class SimulatorTelemetryPayload:
    WIRE_SIZE = 552
    system: SystemTelemetryPayload
    seed: int
    plant_pitch_deg: float
    plant_pitch_rate_dps: float
    plant_position_m: float
    plant_velocity_mps: float
    target_wheel_velocity: float
    actual_wheel_velocity: float
    plant_velocity_error: float
    f_cmd: float
    f_app: float
    external_force_n: float
    external_com_bias_rad: float
    x_ddot: float
    theta_ddot: float
    phase_error_steps: float
    missed_steps: float
    traction_limit_n: float
    motor_force_limit_n: float
    total_mass_scale: float
    pitch_inertia_scale: float
    motor_max_force_n: float
    motor_no_load_speed_mps: float
    motor_velocity_damping: float
    motor_tau_s: float
    traction_coefficient: float
    pitch_damping: float
    cart_damping: float
    phase_error_limit_steps: float
    tire_stiffness_n_per_m: float
    tire_damping_n_s_per_m: float
    wheel_equivalent_mass_kg: float
    force_saturated: bool
    emitted_step_velocity_sps: float
    synthetic_estimator_velocity_sps: float
    controller_feedback_velocity_sps: float
    brace_enabled: bool
    brace_contact_active: bool
    brace_reserved: int
    brace_pitch_deg: float
    brace_penetration_deg: float
    brace_torque_nm: float
    recovery_command_active: bool
    fallover_inhibited: bool
    recovery_reserved: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        item = self.system
        if not hasattr(item, 'pack_wire'):
            if isinstance(item, tuple):
                item = SystemTelemetryPayload(*item)
            elif isinstance(item, dict):
                item = SystemTelemetryPayload(**item)
            else:
                item = SystemTelemetryPayload(item)
        data.extend(item.pack_wire())
        data.extend(struct.pack("<Iffffffffffffffffffffffffffffff?3xfff??Hfff??H", self.seed, self.plant_pitch_deg, self.plant_pitch_rate_dps, self.plant_position_m, self.plant_velocity_mps, self.target_wheel_velocity, self.actual_wheel_velocity, self.plant_velocity_error, self.f_cmd, self.f_app, self.external_force_n, self.external_com_bias_rad, self.x_ddot, self.theta_ddot, self.phase_error_steps, self.missed_steps, self.traction_limit_n, self.motor_force_limit_n, self.total_mass_scale, self.pitch_inertia_scale, self.motor_max_force_n, self.motor_no_load_speed_mps, self.motor_velocity_damping, self.motor_tau_s, self.traction_coefficient, self.pitch_damping, self.cart_damping, self.phase_error_limit_steps, self.tire_stiffness_n_per_m, self.tire_damping_n_s_per_m, self.wheel_equivalent_mass_kg, self.force_saturated, self.emitted_step_velocity_sps, self.synthetic_estimator_velocity_sps, self.controller_feedback_velocity_sps, self.brace_enabled, self.brace_contact_active, self.brace_reserved, self.brace_pitch_deg, self.brace_penetration_deg, self.brace_torque_nm, self.recovery_command_active, self.fallover_inhibited, self.recovery_reserved))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimulatorTelemetryPayload":
        offset = 0
        sub_size = SystemTelemetryPayload.WIRE_SIZE
        system = SystemTelemetryPayload.unpack_wire(data[offset:offset+sub_size])
        offset += sub_size
        seed, plant_pitch_deg, plant_pitch_rate_dps, plant_position_m, plant_velocity_mps, target_wheel_velocity, actual_wheel_velocity, plant_velocity_error, f_cmd, f_app, external_force_n, external_com_bias_rad, x_ddot, theta_ddot, phase_error_steps, missed_steps, traction_limit_n, motor_force_limit_n, total_mass_scale, pitch_inertia_scale, motor_max_force_n, motor_no_load_speed_mps, motor_velocity_damping, motor_tau_s, traction_coefficient, pitch_damping, cart_damping, phase_error_limit_steps, tire_stiffness_n_per_m, tire_damping_n_s_per_m, wheel_equivalent_mass_kg, force_saturated, emitted_step_velocity_sps, synthetic_estimator_velocity_sps, controller_feedback_velocity_sps, brace_enabled, brace_contact_active, brace_reserved, brace_pitch_deg, brace_penetration_deg, brace_torque_nm, recovery_command_active, fallover_inhibited, recovery_reserved = struct.unpack_from("<Iffffffffffffffffffffffffffffff?3xfff??Hfff??H", data, offset)
        offset += struct.calcsize("<Iffffffffffffffffffffffffffffff?3xfff??Hfff??H")
        return cls(system=system, seed=seed, plant_pitch_deg=plant_pitch_deg, plant_pitch_rate_dps=plant_pitch_rate_dps, plant_position_m=plant_position_m, plant_velocity_mps=plant_velocity_mps, target_wheel_velocity=target_wheel_velocity, actual_wheel_velocity=actual_wheel_velocity, plant_velocity_error=plant_velocity_error, f_cmd=f_cmd, f_app=f_app, external_force_n=external_force_n, external_com_bias_rad=external_com_bias_rad, x_ddot=x_ddot, theta_ddot=theta_ddot, phase_error_steps=phase_error_steps, missed_steps=missed_steps, traction_limit_n=traction_limit_n, motor_force_limit_n=motor_force_limit_n, total_mass_scale=total_mass_scale, pitch_inertia_scale=pitch_inertia_scale, motor_max_force_n=motor_max_force_n, motor_no_load_speed_mps=motor_no_load_speed_mps, motor_velocity_damping=motor_velocity_damping, motor_tau_s=motor_tau_s, traction_coefficient=traction_coefficient, pitch_damping=pitch_damping, cart_damping=cart_damping, phase_error_limit_steps=phase_error_limit_steps, tire_stiffness_n_per_m=tire_stiffness_n_per_m, tire_damping_n_s_per_m=tire_damping_n_s_per_m, wheel_equivalent_mass_kg=wheel_equivalent_mass_kg, force_saturated=force_saturated, emitted_step_velocity_sps=emitted_step_velocity_sps, synthetic_estimator_velocity_sps=synthetic_estimator_velocity_sps, controller_feedback_velocity_sps=controller_feedback_velocity_sps, brace_enabled=brace_enabled, brace_contact_active=brace_contact_active, brace_reserved=brace_reserved, brace_pitch_deg=brace_pitch_deg, brace_penetration_deg=brace_penetration_deg, brace_torque_nm=brace_torque_nm, recovery_command_active=recovery_command_active, fallover_inhibited=fallover_inhibited, recovery_reserved=recovery_reserved)

    @classmethod
    def unpack(cls, data: bytes) -> "SimulatorTelemetryPayload":
        return cls.unpack_wire(data)

@dataclass
class JoystickCommandPayload:
    WIRE_SIZE = 16
    forward: float
    turn: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<dd", self.forward, self.turn))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "JoystickCommandPayload":
        offset = 0
        forward, turn = struct.unpack_from("<dd", data, offset)
        offset += struct.calcsize("<dd")
        return cls(forward=forward, turn=turn)

    @classmethod
    def unpack(cls, data: bytes) -> "JoystickCommandPayload":
        return cls.unpack_wire(data)

@dataclass
class ConfigPidValuesPayload:
    WIRE_SIZE = 152
    drive_max_velocity_mps: float
    velocity_gain_per_s: float
    velocity_feedback_cutoff_hz: float
    outer_pitch_limit_deg: float
    fixed_com_trim_deg: float
    adaptive_com_trim_enabled: float
    adaptive_com_trim_gain_deg_per_mps_s: float
    adaptive_com_trim_limit_deg: float
    turn_max_sps: float
    balance_max_sps: float
    pitch_gain: float
    pitch_rate_gain: float
    pitch_accel_gain: float
    planner_max_acceleration_mps2: float
    planner_max_deceleration_mps2: float
    planner_max_jerk_mps3: float
    velocity_i_gain_per_s2: float
    velocity_i_leak_time_s: float
    velocity_i_acceleration_limit_mps2: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<ddddddddddddddddddd", self.drive_max_velocity_mps, self.velocity_gain_per_s, self.velocity_feedback_cutoff_hz, self.outer_pitch_limit_deg, self.fixed_com_trim_deg, self.adaptive_com_trim_enabled, self.adaptive_com_trim_gain_deg_per_mps_s, self.adaptive_com_trim_limit_deg, self.turn_max_sps, self.balance_max_sps, self.pitch_gain, self.pitch_rate_gain, self.pitch_accel_gain, self.planner_max_acceleration_mps2, self.planner_max_deceleration_mps2, self.planner_max_jerk_mps3, self.velocity_i_gain_per_s2, self.velocity_i_leak_time_s, self.velocity_i_acceleration_limit_mps2))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "ConfigPidValuesPayload":
        offset = 0
        drive_max_velocity_mps, velocity_gain_per_s, velocity_feedback_cutoff_hz, outer_pitch_limit_deg, fixed_com_trim_deg, adaptive_com_trim_enabled, adaptive_com_trim_gain_deg_per_mps_s, adaptive_com_trim_limit_deg, turn_max_sps, balance_max_sps, pitch_gain, pitch_rate_gain, pitch_accel_gain, planner_max_acceleration_mps2, planner_max_deceleration_mps2, planner_max_jerk_mps3, velocity_i_gain_per_s2, velocity_i_leak_time_s, velocity_i_acceleration_limit_mps2 = struct.unpack_from("<ddddddddddddddddddd", data, offset)
        offset += struct.calcsize("<ddddddddddddddddddd")
        return cls(drive_max_velocity_mps=drive_max_velocity_mps, velocity_gain_per_s=velocity_gain_per_s, velocity_feedback_cutoff_hz=velocity_feedback_cutoff_hz, outer_pitch_limit_deg=outer_pitch_limit_deg, fixed_com_trim_deg=fixed_com_trim_deg, adaptive_com_trim_enabled=adaptive_com_trim_enabled, adaptive_com_trim_gain_deg_per_mps_s=adaptive_com_trim_gain_deg_per_mps_s, adaptive_com_trim_limit_deg=adaptive_com_trim_limit_deg, turn_max_sps=turn_max_sps, balance_max_sps=balance_max_sps, pitch_gain=pitch_gain, pitch_rate_gain=pitch_rate_gain, pitch_accel_gain=pitch_accel_gain, planner_max_acceleration_mps2=planner_max_acceleration_mps2, planner_max_deceleration_mps2=planner_max_deceleration_mps2, planner_max_jerk_mps3=planner_max_jerk_mps3, velocity_i_gain_per_s2=velocity_i_gain_per_s2, velocity_i_leak_time_s=velocity_i_leak_time_s, velocity_i_acceleration_limit_mps2=velocity_i_acceleration_limit_mps2)

    @classmethod
    def unpack(cls, data: bytes) -> "ConfigPidValuesPayload":
        return cls.unpack_wire(data)

@dataclass
class PidConfigOverridePayload:
    WIRE_SIZE = 160
    request_id: int
    reserved: int
    values: ConfigPidValuesPayload

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<II", self.request_id, self.reserved))
        item = self.values
        if not hasattr(item, 'pack_wire'):
            if isinstance(item, tuple):
                item = ConfigPidValuesPayload(*item)
            elif isinstance(item, dict):
                item = ConfigPidValuesPayload(**item)
            else:
                item = ConfigPidValuesPayload(item)
        data.extend(item.pack_wire())
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "PidConfigOverridePayload":
        offset = 0
        request_id, reserved = struct.unpack_from("<II", data, offset)
        offset += struct.calcsize("<II")
        sub_size = ConfigPidValuesPayload.WIRE_SIZE
        values = ConfigPidValuesPayload.unpack_wire(data[offset:offset+sub_size])
        offset += sub_size
        return cls(request_id=request_id, reserved=reserved, values=values)

    @classmethod
    def unpack(cls, data: bytes) -> "PidConfigOverridePayload":
        return cls.unpack_wire(data)

@dataclass
class PidConfigStatusPayload:
    WIRE_SIZE = 160
    request_id: int
    accepted: int
    result_code: int
    reserved: int
    values: ConfigPidValuesPayload

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBH", self.request_id, self.accepted, self.result_code, self.reserved))
        item = self.values
        if not hasattr(item, 'pack_wire'):
            if isinstance(item, tuple):
                item = ConfigPidValuesPayload(*item)
            elif isinstance(item, dict):
                item = ConfigPidValuesPayload(**item)
            else:
                item = ConfigPidValuesPayload(item)
        data.extend(item.pack_wire())
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "PidConfigStatusPayload":
        offset = 0
        request_id, accepted, result_code, reserved = struct.unpack_from("<IBBH", data, offset)
        offset += struct.calcsize("<IBBH")
        sub_size = ConfigPidValuesPayload.WIRE_SIZE
        values = ConfigPidValuesPayload.unpack_wire(data[offset:offset+sub_size])
        offset += sub_size
        return cls(request_id=request_id, accepted=accepted, result_code=result_code, reserved=reserved, values=values)

    @classmethod
    def unpack(cls, data: bytes) -> "PidConfigStatusPayload":
        return cls.unpack_wire(data)

MESSAGE_BY_ID = {
    MsgId.PhysicsTick: PhysicsTickPayload,
    MsgId.MotorTargets: MotorTargetsPayload,
    MsgId.SystemTelemetry: SystemTelemetryPayload,
    MsgId.SimStartRun: SimStartRunPayload,
    MsgId.SimStartAck: SimStartAckPayload,
    MsgId.SimStopRun: SimStopRunPayload,
    MsgId.SimRunDone: SimRunDonePayload,
    MsgId.ImuRawData: ImuRawPayload,
    MsgId.SimulatorTelemetry: SimulatorTelemetryPayload,
    MsgId.ExternalJoystickCommand: JoystickCommandPayload,
    MsgId.PidConfigOverride: PidConfigOverridePayload,
    MsgId.PidConfigStatus: PidConfigStatusPayload,
}

PAYLOAD_SIZE_BY_ID = {
    MsgId.PhysicsTick: 16,
    MsgId.MotorTargets: 16,
    MsgId.SystemTelemetry: 392,
    MsgId.SimStartRun: 1264,
    MsgId.SimStartAck: 8,
    MsgId.SimStopRun: 4,
    MsgId.SimRunDone: 96,
    MsgId.ImuRawData: 56,
    MsgId.SimulatorTelemetry: 552,
    MsgId.ExternalJoystickCommand: 16,
    MsgId.PidConfigOverride: 160,
    MsgId.PidConfigStatus: 160,
}

PROTOCOL_HASH = "85a3e7ce1a73a4f4"
