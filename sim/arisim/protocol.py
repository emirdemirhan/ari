"""HIL/PIL wire protocol v2. Mirrors ari/include/simulation/hil_protocol.h exactly.

Frame:  0xAA 0xBB | type | len | payload | crc16 (little endian)
crc16 = CRC-16/CCITT-FALSE over type, len and payload.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

SYNC_1 = 0xAA
SYNC_2 = 0xBB
TYPE_SIM_DATA = 0x01
TYPE_RESET = 0x02
TYPE_FC_DATA = 0x81

SIM_FLAG_RANGE_VALID = 1 << 0
SIM_FLAG_RANGE_NEW = 1 << 1
SIM_FLAG_BARO_VALID = 1 << 2
SIM_FLAG_BARO_NEW = 1 << 3
FC_FLAG_USE_ESKF = 1 << 0
FC_FLAG_CALIBRATING = 1 << 1
FC_FLAG_LANDED = 1 << 2
FC_FLAG_RANGE_LOST = 1 << 3
FC_FLAG_HEIGHT_FAILSAFE = 1 << 4

# HIL_Sim_Data_t: seq, time_us, accel[3], gyro[3], altitude_z, baro_alt, rc_roll, rc_pitch, rc_yaw_rate,
#                 rc_thrust, true_roll, true_pitch, true_yaw, flags
SIM_FORMAT = struct.Struct("<II3f3fff4f3fI")
# HIL_Fc_Data_t: seq, motor[4], kf_rpy[3], eskf_rpy[3], kf_alt, kf_vz, eskf_alt, eskf_vz,
#                eskf_vel[2], target_vz, total_thrust, roll_rate_ref, pitch_rate_ref, exec_cycles, flags
FC_FORMAT = struct.Struct("<I4f3f3f4f2f4fII")


def _make_table():
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


_CRC_TABLE = _make_table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


def encode_frame(frame_type: int, payload: bytes) -> bytes:
    head = bytes([frame_type, len(payload)])
    crc = crc16(head + payload)
    return bytes([SYNC_1, SYNC_2]) + head + payload + bytes([crc & 0xFF, crc >> 8])


@dataclass
class SimData:
    seq: int = 0
    time_us: int = 0
    accel: tuple = (0.0, 0.0, 9.81)
    gyro: tuple = (0.0, 0.0, 0.0)
    altitude_z: float = 0.0
    baro_alt: float = 0.0
    rc_roll: float = 0.0
    rc_pitch: float = 0.0
    rc_yaw_rate: float = 0.0
    rc_thrust: float = 0.0
    true_rpy: tuple = (0.0, 0.0, 0.0)
    flags: int = 0

    def pack(self) -> bytes:
        return SIM_FORMAT.pack(self.seq & 0xFFFFFFFF, self.time_us & 0xFFFFFFFF, *self.accel, *self.gyro,
                               self.altitude_z, self.baro_alt, self.rc_roll, self.rc_pitch, self.rc_yaw_rate, self.rc_thrust,
                               *self.true_rpy, self.flags)

    def frame(self) -> bytes:
        return encode_frame(TYPE_SIM_DATA, self.pack())


@dataclass
class FcData:
    seq: int = 0
    motor: tuple = (0.0, 0.0, 0.0, 0.0)
    kf_rpy: tuple = (0.0, 0.0, 0.0)
    eskf_rpy: tuple = (0.0, 0.0, 0.0)
    kf_alt: float = 0.0
    kf_vz: float = 0.0
    eskf_alt: float = 0.0
    eskf_vz: float = 0.0
    eskf_vel: tuple = (0.0, 0.0)
    target_vz: float = 0.0
    total_thrust: float = 0.0
    roll_rate_ref: float = 0.0
    pitch_rate_ref: float = 0.0
    exec_cycles: int = 0
    flags: int = 0

    @classmethod
    def unpack(cls, payload: bytes) -> "FcData":
        v = FC_FORMAT.unpack(payload)
        return cls(seq=v[0], motor=v[1:5], kf_rpy=v[5:8], eskf_rpy=v[8:11], kf_alt=v[11], kf_vz=v[12],
                   eskf_alt=v[13], eskf_vz=v[14], eskf_vel=v[15:17], target_vz=v[17], total_thrust=v[18],
                   roll_rate_ref=v[19], pitch_rate_ref=v[20], exec_cycles=v[21], flags=v[22])


class Decoder:
    """Streaming frame decoder; feed() returns the complete, CRC-valid frames as (type, payload)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.junk = bytearray()          # last bytes that were not part of a frame (boot text, fault reports)

    def _discard(self, n):
        self.junk += self.buf[:n]
        del self.junk[:-512]
        del self.buf[:n]

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(bytes([SYNC_1, SYNC_2]))
            if start < 0:
                # Keep a trailing sync byte, it may start the next frame
                self._discard(max(0, len(self.buf) - 1))
                break
            if start:
                self._discard(start)
            if len(self.buf) < 4:
                break
            length = self.buf[3]
            total = 4 + length + 2
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            crc = frame[-2] | (frame[-1] << 8)
            if crc16(frame[2:-2]) == crc:
                frames.append((frame[2], frame[4:-2]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                self._discard(1)        # resynchronise on the next sync pair
        return frames


def check_boot_message(text: str):
    """The firmware's boot line ends with its packet sizes ("... proto sim=72 fc=92"). Returns an
    error message when they differ from this simulator's (firmware not reflashed), else None."""
    i = text.find("Hello from STM32")
    if i < 0:
        return None
    line = text[i:].splitlines()[0]
    want = f"proto sim={SIM_FORMAT.size} fc={FC_FORMAT.size}"
    if want in line:
        return None
    have = line[line.find("proto"):] if "proto" in line else "no packet sizes (older firmware)"
    return (f"the board runs firmware with a different protocol ({have}; this simulator needs {want}). "
            f"Reflash it: BOOT0 high, `make flash` in ari/, BOOT0 low, reset")
