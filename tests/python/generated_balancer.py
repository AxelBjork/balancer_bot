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

    def pack_wire(self) -> bytes:
        data = bytearray()
        data.extend(struct.pack("<fffffffffffffff", self.t_sec, self.age_ms, self.pitch_deg, self.pitch_rate_dps, self.rate_sp_dps, self.out_norm, self.u_sps, self.integ_pitch, self.vel_error, self.vel_p_term, self.vel_i_term, self.pitch_sp_deg, self.effective_pitch_sp_deg, self.pitch_trim_deg, self.trim_active))
        return bytes(data)

    def pack(self) -> bytes:
        return self.pack_wire()

    @classmethod
    def unpack_wire(cls, data: bytes) -> "SystemTelemetryPayload":
        offset = 0
        t_sec, age_ms, pitch_deg, pitch_rate_dps, rate_sp_dps, out_norm, u_sps, integ_pitch, vel_error, vel_p_term, vel_i_term, pitch_sp_deg, effective_pitch_sp_deg, pitch_trim_deg, trim_active = struct.unpack_from("<fffffffffffffff", data, offset)
        offset += struct.calcsize("<fffffffffffffff")
        return cls(t_sec=t_sec, age_ms=age_ms, pitch_deg=pitch_deg, pitch_rate_dps=pitch_rate_dps, rate_sp_dps=rate_sp_dps, out_norm=out_norm, u_sps=u_sps, integ_pitch=integ_pitch, vel_error=vel_error, vel_p_term=vel_p_term, vel_i_term=vel_i_term, pitch_sp_deg=pitch_sp_deg, effective_pitch_sp_deg=effective_pitch_sp_deg, pitch_trim_deg=pitch_trim_deg, trim_active=trim_active)

    @classmethod
    def unpack(cls, data: bytes) -> "SystemTelemetryPayload":
        return cls.unpack_wire(data)

MESSAGE_BY_ID = {
    MsgId.PhysicsTick: PhysicsTickPayload,
    MsgId.ImuData: ImuSamplePayload,
    MsgId.JoystickCommand: JoystickCommandPayload,
    MsgId.MotorTargets: MotorTargetsPayload,
    MsgId.SystemTelemetry: SystemTelemetryPayload,
}

PAYLOAD_SIZE_BY_ID = {
    MsgId.PhysicsTick: 16,
    MsgId.ImuData: 64,
    MsgId.JoystickCommand: 8,
    MsgId.MotorTargets: 8,
    MsgId.SystemTelemetry: 60,
}

PROTOCOL_HASH = "70d7276f466217f9"
