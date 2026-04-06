"""Auto-generated balancer_bot IPC bindings (C++26 reflection)."""

import struct
from dataclasses import dataclass
from enum import IntEnum

class BalancerMsgId(IntEnum):
    PhysicsTick = 1
    ImuData = 3000
    JoystickCommand = 3001
    MotorTargets = 3002
    SystemTelemetry = 3003

@dataclass
class PhysicsTickPayload:
    WIRE_SIZE = 16
    dt_s: float
    sim_time_us: int

    def pack(self) -> bytes:
        return struct.pack("<dQ", self.dt_s, self.sim_time_us)

    @classmethod
    def unpack(cls, data: bytes) -> 'PhysicsTickPayload':
        offset = 0
        dt_s, = struct.unpack_from("<d", data, offset)
        offset += 8
        sim_time_us, = struct.unpack_from("<Q", data, offset)
        offset += 8
        return cls(dt_s=dt_s, sim_time_us=sim_time_us)

@dataclass
class ImuSamplePayload:
    WIRE_SIZE = 64
    pitch_rad: float
    acc: list[float]
    gyr: list[float]
    timestamp_us: int

    def pack(self) -> bytes:
        return struct.pack("<d3d3dQ", self.pitch_rad, self.acc[0], self.acc[1], self.acc[2], self.gyr[0], self.gyr[1], self.gyr[2], self.timestamp_us)

    @classmethod
    def unpack(cls, data: bytes) -> 'ImuSamplePayload':
        offset = 0
        pitch_rad, = struct.unpack_from("<d", data, offset)
        offset += 8
        acc = list(struct.unpack_from("<3d", data, offset))
        offset += 24
        gyr = list(struct.unpack_from("<3d", data, offset))
        offset += 24
        timestamp_us, = struct.unpack_from("<Q", data, offset)
        offset += 8
        return cls(pitch_rad=pitch_rad, acc=acc, gyr=gyr, timestamp_us=timestamp_us)

@dataclass
class JoystickCommandPayload:
    WIRE_SIZE = 8
    forward: float
    turn: float

    def pack(self) -> bytes:
        return struct.pack("<ff", self.forward, self.turn)

    @classmethod
    def unpack(cls, data: bytes) -> 'JoystickCommandPayload':
        offset = 0
        forward, = struct.unpack_from("<f", data, offset)
        offset += 4
        turn, = struct.unpack_from("<f", data, offset)
        offset += 4
        return cls(forward=forward, turn=turn)

@dataclass
class MotorTargetsPayload:
    WIRE_SIZE = 8
    left_sps: float
    right_sps: float

    def pack(self) -> bytes:
        return struct.pack("<ff", self.left_sps, self.right_sps)

    @classmethod
    def unpack(cls, data: bytes) -> 'MotorTargetsPayload':
        offset = 0
        left_sps, = struct.unpack_from("<f", data, offset)
        offset += 4
        right_sps, = struct.unpack_from("<f", data, offset)
        offset += 4
        return cls(left_sps=left_sps, right_sps=right_sps)

@dataclass
class SystemTelemetryPayload:
    WIRE_SIZE = 60
    t_sec: float
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

    def pack(self) -> bytes:
        return struct.pack("<fffffffffffffff", self.t_sec, self.age_ms, self.pitch_deg, self.pitch_rate_dps, self.rate_sp_dps, self.out_norm, self.u_sps, self.integ_pitch, self.vel_error, self.vel_p_term, self.vel_i_term, self.pitch_sp_deg, self.effective_pitch_sp_deg, self.pitch_trim_deg, self.trim_active)

    @classmethod
    def unpack(cls, data: bytes) -> 'SystemTelemetryPayload':
        offset = 0
        t_sec, = struct.unpack_from("<f", data, offset)
        offset += 4
        age_ms, = struct.unpack_from("<f", data, offset)
        offset += 4
        pitch_deg, = struct.unpack_from("<f", data, offset)
        offset += 4
        pitch_rate_dps, = struct.unpack_from("<f", data, offset)
        offset += 4
        rate_sp_dps, = struct.unpack_from("<f", data, offset)
        offset += 4
        out_norm, = struct.unpack_from("<f", data, offset)
        offset += 4
        u_sps, = struct.unpack_from("<f", data, offset)
        offset += 4
        integ_pitch, = struct.unpack_from("<f", data, offset)
        offset += 4
        vel_error, = struct.unpack_from("<f", data, offset)
        offset += 4
        vel_p_term, = struct.unpack_from("<f", data, offset)
        offset += 4
        vel_i_term, = struct.unpack_from("<f", data, offset)
        offset += 4
        pitch_sp_deg, = struct.unpack_from("<f", data, offset)
        offset += 4
        effective_pitch_sp_deg, = struct.unpack_from("<f", data, offset)
        offset += 4
        pitch_trim_deg, = struct.unpack_from("<f", data, offset)
        offset += 4
        trim_active, = struct.unpack_from("<f", data, offset)
        offset += 4
        return cls(t_sec=t_sec, age_ms=age_ms, pitch_deg=pitch_deg, pitch_rate_dps=pitch_rate_dps, rate_sp_dps=rate_sp_dps, out_norm=out_norm, u_sps=u_sps, integ_pitch=integ_pitch, vel_error=vel_error, vel_p_term=vel_p_term, vel_i_term=vel_i_term, pitch_sp_deg=pitch_sp_deg, effective_pitch_sp_deg=effective_pitch_sp_deg, pitch_trim_deg=pitch_trim_deg, trim_active=trim_active)

MESSAGE_BY_ID = {
    BalancerMsgId.PhysicsTick: PhysicsTickPayload,
    BalancerMsgId.ImuData: ImuSamplePayload,
    BalancerMsgId.JoystickCommand: JoystickCommandPayload,
    BalancerMsgId.MotorTargets: MotorTargetsPayload,
    BalancerMsgId.SystemTelemetry: SystemTelemetryPayload,
}
