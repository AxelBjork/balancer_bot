"""Auto-generated balancer_bot IPC bindings (C++26 reflection)."""

import struct
from dataclasses import dataclass
from enum import IntEnum

class BalancerMsgId(IntEnum):
    ImuData = 3000
    JoystickCommand = 3001
    MotorTargets = 3002
    SystemTelemetry = 3003

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
    WIRE_SIZE = 8
    core_cpu_usage: float
    loop_time_us: float

    def pack(self) -> bytes:
        return struct.pack("<ff", self.core_cpu_usage, self.loop_time_us)

    @classmethod
    def unpack(cls, data: bytes) -> 'SystemTelemetryPayload':
        offset = 0
        core_cpu_usage, = struct.unpack_from("<f", data, offset)
        offset += 4
        loop_time_us, = struct.unpack_from("<f", data, offset)
        offset += 4
        return cls(core_cpu_usage=core_cpu_usage, loop_time_us=loop_time_us)

MESSAGE_BY_ID = {
    BalancerMsgId.ImuData: ImuSamplePayload,
    BalancerMsgId.JoystickCommand: JoystickCommandPayload,
    BalancerMsgId.MotorTargets: MotorTargetsPayload,
    BalancerMsgId.SystemTelemetry: SystemTelemetryPayload,
}
