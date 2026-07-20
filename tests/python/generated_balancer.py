"""Auto-generated IPC bindings using C++26 static reflection."""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

class MsgId(IntEnum):
    """Balancer UDP message identifiers."""
    PhysicsTick = 1
    JoystickCommand = 3001
    MotorTargets = 3002
    SystemTelemetry = 3003
    SimStartRun = 3005
    SimStartAck = 3006
    SimStopRun = 3007
    SimRunDone = 3008
    ImuRawData = 3009

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
    WIRE_SIZE = 504
    run_id: int
    seed: int
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
    target_velocity_sps: float
    vel_error: float
    measured_vel_sps: float
    velocity_p_term_deg: float
    velocity_i_term_deg: float
    pitch_error_deg: float
    pitch_sp_deg: float
    rate_setpoint_dps: float
    rate_error_dps: float
    command_saturated: float
    actuator_fault: float
    left_target_sps: float
    right_target_sps: float
    left_applied_sps: float
    right_applied_sps: float
    motor_update_dt_ms: float
    motor_feedback_age_ms: float
    left_actual_steps: int
    right_actual_steps: int
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
    force_saturated: float
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

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IIIIQdddddddddddddddddddddddddddqqddddddddddddddddddddddddddddddd", self.run_id, self.seed, self.controller_fault_flags, self.controller_saturation_flags, self.imu_timestamp_us, self.t_sec, self.age_ms, self.pitch_deg, self.pitch_rate_dps, self.raw_acc_pitch_deg, self.fused_pitch_deg, self.gyro_pitch_rate_dps, self.filtered_pitch_rate_dps, self.u_sps, self.turn_sps, self.target_velocity_sps, self.vel_error, self.measured_vel_sps, self.velocity_p_term_deg, self.velocity_i_term_deg, self.pitch_error_deg, self.pitch_sp_deg, self.rate_setpoint_dps, self.rate_error_dps, self.command_saturated, self.actuator_fault, self.left_target_sps, self.right_target_sps, self.left_applied_sps, self.right_applied_sps, self.motor_update_dt_ms, self.motor_feedback_age_ms, self.left_actual_steps, self.right_actual_steps, self.plant_pitch_deg, self.plant_pitch_rate_dps, self.plant_position_m, self.plant_velocity_mps, self.target_wheel_velocity, self.actual_wheel_velocity, self.plant_velocity_error, self.f_cmd, self.f_app, self.external_force_n, self.external_com_bias_rad, self.x_ddot, self.theta_ddot, self.force_saturated, self.phase_error_steps, self.missed_steps, self.traction_limit_n, self.motor_force_limit_n, self.total_mass_scale, self.pitch_inertia_scale, self.motor_max_force_n, self.motor_no_load_speed_mps, self.motor_velocity_damping, self.motor_tau_s, self.traction_coefficient, self.pitch_damping, self.cart_damping, self.phase_error_limit_steps, self.tire_stiffness_n_per_m, self.tire_damping_n_s_per_m, self.wheel_equivalent_mass_kg))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SystemTelemetryPayload":
        offset = 0
        run_id, seed, controller_fault_flags, controller_saturation_flags, imu_timestamp_us, t_sec, age_ms, pitch_deg, pitch_rate_dps, raw_acc_pitch_deg, fused_pitch_deg, gyro_pitch_rate_dps, filtered_pitch_rate_dps, u_sps, turn_sps, target_velocity_sps, vel_error, measured_vel_sps, velocity_p_term_deg, velocity_i_term_deg, pitch_error_deg, pitch_sp_deg, rate_setpoint_dps, rate_error_dps, command_saturated, actuator_fault, left_target_sps, right_target_sps, left_applied_sps, right_applied_sps, motor_update_dt_ms, motor_feedback_age_ms, left_actual_steps, right_actual_steps, plant_pitch_deg, plant_pitch_rate_dps, plant_position_m, plant_velocity_mps, target_wheel_velocity, actual_wheel_velocity, plant_velocity_error, f_cmd, f_app, external_force_n, external_com_bias_rad, x_ddot, theta_ddot, force_saturated, phase_error_steps, missed_steps, traction_limit_n, motor_force_limit_n, total_mass_scale, pitch_inertia_scale, motor_max_force_n, motor_no_load_speed_mps, motor_velocity_damping, motor_tau_s, traction_coefficient, pitch_damping, cart_damping, phase_error_limit_steps, tire_stiffness_n_per_m, tire_damping_n_s_per_m, wheel_equivalent_mass_kg = struct.unpack_from("<IIIIQdddddddddddddddddddddddddddqqddddddddddddddddddddddddddddddd", data, offset)
        offset += struct.calcsize("<IIIIQdddddddddddddddddddddddddddqqddddddddddddddddddddddddddddddd")
        return cls(run_id=run_id, seed=seed, controller_fault_flags=controller_fault_flags, controller_saturation_flags=controller_saturation_flags, imu_timestamp_us=imu_timestamp_us, t_sec=t_sec, age_ms=age_ms, pitch_deg=pitch_deg, pitch_rate_dps=pitch_rate_dps, raw_acc_pitch_deg=raw_acc_pitch_deg, fused_pitch_deg=fused_pitch_deg, gyro_pitch_rate_dps=gyro_pitch_rate_dps, filtered_pitch_rate_dps=filtered_pitch_rate_dps, u_sps=u_sps, turn_sps=turn_sps, target_velocity_sps=target_velocity_sps, vel_error=vel_error, measured_vel_sps=measured_vel_sps, velocity_p_term_deg=velocity_p_term_deg, velocity_i_term_deg=velocity_i_term_deg, pitch_error_deg=pitch_error_deg, pitch_sp_deg=pitch_sp_deg, rate_setpoint_dps=rate_setpoint_dps, rate_error_dps=rate_error_dps, command_saturated=command_saturated, actuator_fault=actuator_fault, left_target_sps=left_target_sps, right_target_sps=right_target_sps, left_applied_sps=left_applied_sps, right_applied_sps=right_applied_sps, motor_update_dt_ms=motor_update_dt_ms, motor_feedback_age_ms=motor_feedback_age_ms, left_actual_steps=left_actual_steps, right_actual_steps=right_actual_steps, plant_pitch_deg=plant_pitch_deg, plant_pitch_rate_dps=plant_pitch_rate_dps, plant_position_m=plant_position_m, plant_velocity_mps=plant_velocity_mps, target_wheel_velocity=target_wheel_velocity, actual_wheel_velocity=actual_wheel_velocity, plant_velocity_error=plant_velocity_error, f_cmd=f_cmd, f_app=f_app, external_force_n=external_force_n, external_com_bias_rad=external_com_bias_rad, x_ddot=x_ddot, theta_ddot=theta_ddot, force_saturated=force_saturated, phase_error_steps=phase_error_steps, missed_steps=missed_steps, traction_limit_n=traction_limit_n, motor_force_limit_n=motor_force_limit_n, total_mass_scale=total_mass_scale, pitch_inertia_scale=pitch_inertia_scale, motor_max_force_n=motor_max_force_n, motor_no_load_speed_mps=motor_no_load_speed_mps, motor_velocity_damping=motor_velocity_damping, motor_tau_s=motor_tau_s, traction_coefficient=traction_coefficient, pitch_damping=pitch_damping, cart_damping=cart_damping, phase_error_limit_steps=phase_error_limit_steps, tire_stiffness_n_per_m=tire_stiffness_n_per_m, tire_damping_n_s_per_m=tire_damping_n_s_per_m, wheel_equivalent_mass_kg=wheel_equivalent_mass_kg)

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
class SimStartRunPayload:
    WIRE_SIZE = 1120
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
        data.extend(struct.pack("<128s", self.pid_config_path))
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
        pid_config_path = struct.unpack_from("<128s", data, offset)[0]
        offset += struct.calcsize("<128s")
        return cls(run_id=run_id, physics_profile=physics_profile, has_physics_override=has_physics_override, telemetry_stride=telemetry_stride, transfer_scenario_index=transfer_scenario_index, reserved1=reserved1, duration_s=duration_s, initial_pitch_deg=initial_pitch_deg, com_angle_offset_rad=com_angle_offset_rad, total_mass_scale=total_mass_scale, pitch_inertia_scale=pitch_inertia_scale, motor_max_force_n=motor_max_force_n, motor_no_load_speed_mps=motor_no_load_speed_mps, motor_velocity_damping=motor_velocity_damping, motor_tau_s=motor_tau_s, traction_coefficient=traction_coefficient, pitch_damping=pitch_damping, cart_damping=cart_damping, phase_error_limit_steps=phase_error_limit_steps, tire_stiffness_n_per_m=tire_stiffness_n_per_m, tire_damping_n_s_per_m=tire_damping_n_s_per_m, wheel_equivalent_mass_kg=wheel_equivalent_mass_kg, imu_pitch_lag_s=imu_pitch_lag_s, imu_noise_seed=imu_noise_seed, accel_noise_std_mps2=accel_noise_std_mps2, gyro_noise_std_rad_s=gyro_noise_std_rad_s, imu_timestamp_jitter_us=imu_timestamp_jitter_us, imu_sample_loss_rate=imu_sample_loss_rate, accel_bias_mps2=accel_bias_mps2, gyro_bias_rad_s=gyro_bias_rad_s, disturbances=disturbances, joy_segments=joy_segments, pid_config_path=pid_config_path)

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
    WIRE_SIZE = 104
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
    timeline_hash: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBHI4xdddddddddIIQ", self.run_id, self.reason_code, self.reserved0, self.reserved1, self.sample_count, self.elapsed_s, self.final_pitch_deg, self.max_abs_pitch_deg, self.tail_rms_pitch_deg, self.tail_rail_fraction, self.tail_mean_abs_pitch_deg, self.max_abs_position_m, self.tail_mean_abs_velocity_mps, self.max_continuous_saturation_s, self.actuator_fault_count, self.controller_fault_flags, self.timeline_hash))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimRunDonePayload":
        offset = 0
        run_id, reason_code, reserved0, reserved1, sample_count, elapsed_s, final_pitch_deg, max_abs_pitch_deg, tail_rms_pitch_deg, tail_rail_fraction, tail_mean_abs_pitch_deg, max_abs_position_m, tail_mean_abs_velocity_mps, max_continuous_saturation_s, actuator_fault_count, controller_fault_flags, timeline_hash = struct.unpack_from("<IBBHI4xdddddddddIIQ", data, offset)
        offset += struct.calcsize("<IBBHI4xdddddddddIIQ")
        return cls(run_id=run_id, reason_code=reason_code, reserved0=reserved0, reserved1=reserved1, sample_count=sample_count, elapsed_s=elapsed_s, final_pitch_deg=final_pitch_deg, max_abs_pitch_deg=max_abs_pitch_deg, tail_rms_pitch_deg=tail_rms_pitch_deg, tail_rail_fraction=tail_rail_fraction, tail_mean_abs_pitch_deg=tail_mean_abs_pitch_deg, max_abs_position_m=max_abs_position_m, tail_mean_abs_velocity_mps=tail_mean_abs_velocity_mps, max_continuous_saturation_s=max_continuous_saturation_s, actuator_fault_count=actuator_fault_count, controller_fault_flags=controller_fault_flags, timeline_hash=timeline_hash)

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

MESSAGE_BY_ID = {
    MsgId.PhysicsTick: PhysicsTickPayload,
    MsgId.JoystickCommand: JoystickCommandPayload,
    MsgId.MotorTargets: MotorTargetsPayload,
    MsgId.SystemTelemetry: SystemTelemetryPayload,
    MsgId.SimStartRun: SimStartRunPayload,
    MsgId.SimStartAck: SimStartAckPayload,
    MsgId.SimStopRun: SimStopRunPayload,
    MsgId.SimRunDone: SimRunDonePayload,
    MsgId.ImuRawData: ImuRawPayload,
}

PAYLOAD_SIZE_BY_ID = {
    MsgId.PhysicsTick: 16,
    MsgId.JoystickCommand: 16,
    MsgId.MotorTargets: 16,
    MsgId.SystemTelemetry: 504,
    MsgId.SimStartRun: 1120,
    MsgId.SimStartAck: 8,
    MsgId.SimStopRun: 4,
    MsgId.SimRunDone: 104,
    MsgId.ImuRawData: 56,
}

PROTOCOL_HASH = "09915846710265fe"
