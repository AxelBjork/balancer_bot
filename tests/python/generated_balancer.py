"""Auto-generated IPC bindings using C++26 static reflection."""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

class MsgId(IntEnum):
    """Balancer UDP message identifiers."""
    PhysicsTick = 1
    ImuData = 3000
    JoystickCommand = 3001
    MotorTargets = 3002
    SystemTelemetry = 3003
    SimStartRun = 3005
    SimStartAck = 3006
    SimStopRun = 3007
    SimRunDone = 3008

BalancerMsgId = MsgId

@dataclass
class PhysicsTickPayload:
    WIRE_SIZE = 16
    dt_s: float
    sim_time_us: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<dQ", self.dt_s, self.sim_time_us))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "PhysicsTickPayload":
        offset = 0
        dt_s, sim_time_us = struct.unpack_from("<dQ", data, offset)
        offset += struct.calcsize("<dQ")
        return cls(dt_s=dt_s, sim_time_us=sim_time_us)

    @classmethod
    def unpack(cls, data: bytes) -> "PhysicsTickPayload":
        return cls.unpack_wire(data)

@dataclass
class ImuSamplePayload:
    WIRE_SIZE = 64
    pitch_rad: float
    acc: list[float]
    gyr: list[float]
    timestamp_us: int

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<d", self.pitch_rad))
        data.extend(struct.pack("<3d", *self.acc))
        data.extend(struct.pack("<3d", *self.gyr))
        data.extend(struct.pack("<Q", self.timestamp_us))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "ImuSamplePayload":
        offset = 0
        pitch_rad = struct.unpack_from("<d", data, offset)[0]
        offset += struct.calcsize("<d")
        acc = list(struct.unpack_from("<3d", data, offset))
        offset += struct.calcsize("<3d")
        gyr = list(struct.unpack_from("<3d", data, offset))
        offset += struct.calcsize("<3d")
        timestamp_us = struct.unpack_from("<Q", data, offset)[0]
        offset += struct.calcsize("<Q")
        return cls(pitch_rad=pitch_rad, acc=acc, gyr=gyr, timestamp_us=timestamp_us)

    @classmethod
    def unpack(cls, data: bytes) -> "ImuSamplePayload":
        return cls.unpack_wire(data)

@dataclass
class JoystickCommandPayload:
    WIRE_SIZE = 8
    forward: float
    turn: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<ff", self.forward, self.turn))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "JoystickCommandPayload":
        offset = 0
        forward, turn = struct.unpack_from("<ff", data, offset)
        offset += struct.calcsize("<ff")
        return cls(forward=forward, turn=turn)

    @classmethod
    def unpack(cls, data: bytes) -> "JoystickCommandPayload":
        return cls.unpack_wire(data)

@dataclass
class MotorTargetsPayload:
    WIRE_SIZE = 8
    left_sps: float
    right_sps: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<ff", self.left_sps, self.right_sps))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "MotorTargetsPayload":
        offset = 0
        left_sps, right_sps = struct.unpack_from("<ff", data, offset)
        offset += struct.calcsize("<ff")
        return cls(left_sps=left_sps, right_sps=right_sps)

    @classmethod
    def unpack(cls, data: bytes) -> "MotorTargetsPayload":
        return cls.unpack_wire(data)

@dataclass
class SystemTelemetryPayload:
    WIRE_SIZE = 120
    run_id: int
    t_sec: float
    sim_time_s: float
    age_ms: float
    pitch_deg: float
    pitch_rate_dps: float
    rate_sp_dps: float
    out_norm: float
    u_sps: float
    integ_pitch: float
    vel_error: float
    vel_p_term: float
    vel_i_term: float
    pitch_sp_deg: float
    effective_pitch_sp_deg: float
    pitch_trim_deg: float
    trim_active: float
    plant_pitch_deg: float
    plant_pitch_rate_dps: float
    plant_position_m: float
    plant_velocity_mps: float
    target_wheel_velocity: float
    actual_wheel_velocity: float
    plant_velocity_error: float
    f_cmd: float
    f_app: float
    x_ddot: float
    theta_ddot: float
    command_saturated: float
    force_saturated: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<Ifffffffffffffffffffffffffffff", self.run_id, self.t_sec, self.sim_time_s, self.age_ms, self.pitch_deg, self.pitch_rate_dps, self.rate_sp_dps, self.out_norm, self.u_sps, self.integ_pitch, self.vel_error, self.vel_p_term, self.vel_i_term, self.pitch_sp_deg, self.effective_pitch_sp_deg, self.pitch_trim_deg, self.trim_active, self.plant_pitch_deg, self.plant_pitch_rate_dps, self.plant_position_m, self.plant_velocity_mps, self.target_wheel_velocity, self.actual_wheel_velocity, self.plant_velocity_error, self.f_cmd, self.f_app, self.x_ddot, self.theta_ddot, self.command_saturated, self.force_saturated))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SystemTelemetryPayload":
        offset = 0
        run_id, t_sec, sim_time_s, age_ms, pitch_deg, pitch_rate_dps, rate_sp_dps, out_norm, u_sps, integ_pitch, vel_error, vel_p_term, vel_i_term, pitch_sp_deg, effective_pitch_sp_deg, pitch_trim_deg, trim_active, plant_pitch_deg, plant_pitch_rate_dps, plant_position_m, plant_velocity_mps, target_wheel_velocity, actual_wheel_velocity, plant_velocity_error, f_cmd, f_app, x_ddot, theta_ddot, command_saturated, force_saturated = struct.unpack_from("<Ifffffffffffffffffffffffffffff", data, offset)
        offset += struct.calcsize("<Ifffffffffffffffffffffffffffff")
        return cls(run_id=run_id, t_sec=t_sec, sim_time_s=sim_time_s, age_ms=age_ms, pitch_deg=pitch_deg, pitch_rate_dps=pitch_rate_dps, rate_sp_dps=rate_sp_dps, out_norm=out_norm, u_sps=u_sps, integ_pitch=integ_pitch, vel_error=vel_error, vel_p_term=vel_p_term, vel_i_term=vel_i_term, pitch_sp_deg=pitch_sp_deg, effective_pitch_sp_deg=effective_pitch_sp_deg, pitch_trim_deg=pitch_trim_deg, trim_active=trim_active, plant_pitch_deg=plant_pitch_deg, plant_pitch_rate_dps=plant_pitch_rate_dps, plant_position_m=plant_position_m, plant_velocity_mps=plant_velocity_mps, target_wheel_velocity=target_wheel_velocity, actual_wheel_velocity=actual_wheel_velocity, plant_velocity_error=plant_velocity_error, f_cmd=f_cmd, f_app=f_app, x_ddot=x_ddot, theta_ddot=theta_ddot, command_saturated=command_saturated, force_saturated=force_saturated)

    @classmethod
    def unpack(cls, data: bytes) -> "SystemTelemetryPayload":
        return cls.unpack_wire(data)

@dataclass
class SimDisturbancePayload:
    WIRE_SIZE = 24
    start_s: float
    duration_s: float
    forward: float
    turn: float

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<ddff", self.start_s, self.duration_s, self.forward, self.turn))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimDisturbancePayload":
        offset = 0
        start_s, duration_s, forward, turn = struct.unpack_from("<ddff", data, offset)
        offset += struct.calcsize("<ddff")
        return cls(start_s=start_s, duration_s=duration_s, forward=forward, turn=turn)

    @classmethod
    def unpack(cls, data: bytes) -> "SimDisturbancePayload":
        return cls.unpack_wire(data)

@dataclass
class SimStartRunPayload:
    WIRE_SIZE = 400
    run_id: int
    physics_profile: int
    reserved0: int
    reserved1: int
    duration_s: float
    initial_pitch_deg: float
    com_angle_offset_rad: float
    disturbances: list[SimDisturbancePayload]
    pid_config_path: bytes

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBHddd", self.run_id, self.physics_profile, self.reserved0, self.reserved1, self.duration_s, self.initial_pitch_deg, self.com_angle_offset_rad))
        for item in self.disturbances:
            if not hasattr(item, 'pack_wire'):
                if isinstance(item, tuple):
                    item = SimDisturbancePayload(*item)
                elif isinstance(item, dict):
                    item = SimDisturbancePayload(**item)
                else:
                    item = SimDisturbancePayload(item)
            data.extend(item.pack_wire())
        data.extend(struct.pack("<128s", self.pid_config_path))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimStartRunPayload":
        offset = 0
        run_id, physics_profile, reserved0, reserved1, duration_s, initial_pitch_deg, com_angle_offset_rad = struct.unpack_from("<IBBHddd", data, offset)
        offset += struct.calcsize("<IBBHddd")
        disturbances = []
        for _ in range(10):
            sub_size = SimDisturbancePayload.WIRE_SIZE
            item = SimDisturbancePayload.unpack_wire(data[offset:offset+sub_size])
            disturbances.append(item)
            offset += sub_size
        pid_config_path = struct.unpack_from("<128s", data, offset)[0]
        offset += struct.calcsize("<128s")
        return cls(run_id=run_id, physics_profile=physics_profile, reserved0=reserved0, reserved1=reserved1, duration_s=duration_s, initial_pitch_deg=initial_pitch_deg, com_angle_offset_rad=com_angle_offset_rad, disturbances=disturbances, pid_config_path=pid_config_path)

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
    WIRE_SIZE = 44
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

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<IBBHIffffffff", self.run_id, self.reason_code, self.reserved0, self.reserved1, self.sample_count, self.elapsed_s, self.final_pitch_deg, self.max_abs_pitch_deg, self.tail_rms_pitch_deg, self.tail_rail_fraction, self.tail_mean_abs_pitch_deg, self.max_abs_position_m, self.tail_mean_abs_velocity_mps))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SimRunDonePayload":
        offset = 0
        run_id, reason_code, reserved0, reserved1, sample_count, elapsed_s, final_pitch_deg, max_abs_pitch_deg, tail_rms_pitch_deg, tail_rail_fraction, tail_mean_abs_pitch_deg, max_abs_position_m, tail_mean_abs_velocity_mps = struct.unpack_from("<IBBHIffffffff", data, offset)
        offset += struct.calcsize("<IBBHIffffffff")
        return cls(run_id=run_id, reason_code=reason_code, reserved0=reserved0, reserved1=reserved1, sample_count=sample_count, elapsed_s=elapsed_s, final_pitch_deg=final_pitch_deg, max_abs_pitch_deg=max_abs_pitch_deg, tail_rms_pitch_deg=tail_rms_pitch_deg, tail_rail_fraction=tail_rail_fraction, tail_mean_abs_pitch_deg=tail_mean_abs_pitch_deg, max_abs_position_m=max_abs_position_m, tail_mean_abs_velocity_mps=tail_mean_abs_velocity_mps)

    @classmethod
    def unpack(cls, data: bytes) -> "SimRunDonePayload":
        return cls.unpack_wire(data)

MESSAGE_BY_ID = {
    MsgId.PhysicsTick: PhysicsTickPayload,
    MsgId.ImuData: ImuSamplePayload,
    MsgId.JoystickCommand: JoystickCommandPayload,
    MsgId.MotorTargets: MotorTargetsPayload,
    MsgId.SystemTelemetry: SystemTelemetryPayload,
    MsgId.SimStartRun: SimStartRunPayload,
    MsgId.SimStartAck: SimStartAckPayload,
    MsgId.SimStopRun: SimStopRunPayload,
    MsgId.SimRunDone: SimRunDonePayload,
}

PAYLOAD_SIZE_BY_ID = {
    MsgId.PhysicsTick: 16,
    MsgId.ImuData: 64,
    MsgId.JoystickCommand: 8,
    MsgId.MotorTargets: 8,
    MsgId.SystemTelemetry: 120,
    MsgId.SimStartRun: 400,
    MsgId.SimStartAck: 8,
    MsgId.SimStopRun: 4,
    MsgId.SimRunDone: 44,
}

PROTOCOL_HASH = "3a9cc0bac4a2e089"
