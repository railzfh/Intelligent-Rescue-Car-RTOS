"""Binary UART protocol v1 shared between MaixCAM (Python) and DM-MC02 (C++).

Frame layout (little-endian):
  [0]     0xAA (SOF0)
  [1]     0x55 (SOF1)
  [2]     version (0x01)
  [3]     msg_type
  [4]     seq (uint8)
  [5]     payload_len (uint8, <= 64)
  [6..]   payload (payload_len bytes)
  [last2] crc16_ccitt over [2 .. 6+payload_len-1] (uint16 LE)
"""
from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import List, Tuple

PROTO_SOF0 = 0xAA
PROTO_SOF1 = 0x55
PROTO_VERSION = 0x01
PROTO_HEADER_LEN = 6
PROTO_CRC_LEN = 2
PROTO_MAX_PAYLOAD = 64
PROTO_MAX_FRAME = PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + PROTO_CRC_LEN

MSG_HEARTBEAT = 0x01
MSG_SET_CONFIG = 0x02
MSG_VISION_STATE = 0x03
MSG_BENCH_ACTUATOR = 0x04
MSG_TELEMETRY = 0x81

TEAM_UNSET = 0
TEAM_RED = 1
TEAM_BLUE = 2

MODE_COMPETITION = 0
MODE_BENCH_STRAIGHT = 1

MISSION_PRE_MATCH = 0
MISSION_BENCH_STRAIGHT = 1
MISSION_LAUNCH_CROSS_BUMPS = 2
MISSION_SEARCH_TARGET = 3
MISSION_APPROACH_TARGET = 4
MISSION_INSPECT_CORRAL = 5
MISSION_BACK_OUT_SEPARATE = 6
MISSION_CLOSE_GATE_VERIFY = 7
MISSION_FIND_HOME = 8
MISSION_APPROACH_HOME = 9
MISSION_DROP_OPEN_AND_PUSH = 10
MISSION_DROP_BACKOUT_VERIFY = 11
MISSION_ESCAPE_RECOVERY = 12
MISSION_MATCH_FINISHED = 13
MISSION_FAULT_STOP = 14

CAMERA_VIEW_TRANSITION = 0
CAMERA_VIEW_FAR = 1
CAMERA_VIEW_NEAR = 2

TARGET_NONE = 0
TARGET_GREEN_MATERIAL = 1
TARGET_BLACK_CORE = 2
TARGET_ORANGE_CASUALTY = 3
TARGET_CYAN_HAZARD = 4

BENCH_SERVO_KEEP = 0
BENCH_SERVO_POS_A = 1  # Gate OPEN / Pitch FAR
BENCH_SERVO_POS_B = 2  # Gate CLOSED / Pitch NEAR

VIS_FLAG_TARGET_VALID = 1 << 0
VIS_FLAG_HOME_VALID = 1 << 1
VIS_FLAG_HOME_HALF_MATCH = 1 << 2
VIS_FLAG_RAMP_VISIBLE = 1 << 3
VIS_FLAG_CORRAL_SAFE = 1 << 4
VIS_FLAG_SWEEP_CLEAR = 1 << 5
VIS_FLAG_DROP_CONFIRMED = 1 << 6
VIS_FLAG_OPPONENT_NEAR = 1 << 7
VIS_FLAG_OPPONENT_ZONE_NEAR = 1 << 8
VIS_FLAG_REAR_CLEAR = 1 << 9
VIS_FLAG_MOTION_STALLED = 1 << 10
VIS_FLAG_DROP_PATH_CLEAR = 1 << 11
VIS_FLAG_GATE_OPEN_SETTLED_ACK = 1 << 12

TEL_FLAG_CONFIG_LOCKED = 1 << 0
TEL_FLAG_BATTERY_OK = 1 << 1
TEL_FLAG_IMU_OK = 1 << 2
TEL_FLAG_MOTORS_OK = 1 << 3
TEL_FLAG_VISION_FRESH = 1 << 4
TEL_FLAG_FIRST_GREEN_UNLOCKED = 1 << 5
TEL_FLAG_GATE_CLOSED = 1 << 6
TEL_FLAG_GATE_SETTLED = 1 << 7
TEL_FLAG_CAMERA_NEAR = 1 << 8
TEL_FLAG_CAMERA_SETTLED = 1 << 9
TEL_FLAG_STALLED = 1 << 10
TEL_FLAG_LINK_ALIVE = 1 << 11
TEL_FLAG_CARRYING_CASUALTY = 1 << 12

FAULT_FLAG_BATTERY = 1 << 0
FAULT_FLAG_MOTOR_TIMEOUT = 1 << 1
FAULT_FLAG_IMU_TIMEOUT = 1 << 2
FAULT_FLAG_CAN_TX = 1 << 3
FAULT_FLAG_VISION_TIMEOUT = 1 << 4
FAULT_FLAG_SERVO = 1 << 5
FAULT_FLAG_UART_RX = 1 << 6

_HEARTBEAT_STRUCT = struct.Struct("<IBBH")
_SET_CONFIG_STRUCT = struct.Struct("<BBBB")
_VISION_STATE_STRUCT = struct.Struct("<HBBHhHhHBBBBBB")
_BENCH_ACTUATOR_STRUCT = struct.Struct("<BBH")
_TELEMETRY_STRUCT = struct.Struct("<IIBBBBHHHhhhBBBB")


@dataclass
class Frame:
    msg_type: int
    seq: int
    payload: bytes


@dataclass
class Heartbeat:
    vision_uptime_ms: int = 0
    camera_ok: int = 1
    ui_state: int = 0
    reserved: int = 0


@dataclass
class MatchConfigCmd:
    team: int = TEAM_UNSET
    start_zone: int = 0
    run_mode: int = MODE_COMPETITION
    max_materials_per_trip: int = 1


@dataclass
class VisionState:
    frame_seq: int = 0
    camera_view: int = CAMERA_VIEW_TRANSITION
    target_type: int = TARGET_NONE
    flags: int = 0
    target_err_x_mrad: int = 0
    target_dist_mm: int = 0
    home_err_x_mrad: int = 0
    home_dist_mm: int = 0
    corral_green_cnt: int = 0
    corral_black_cnt: int = 0
    corral_orange_cnt: int = 0
    corral_cyan_cnt: int = 0
    corral_unknown_cnt: int = 0
    confidence: int = 0


@dataclass
class BenchActuatorCmd:
    gate_cmd: int = BENCH_SERVO_KEEP
    pitch_cmd: int = BENCH_SERVO_KEEP
    reserved: int = 0


@dataclass
class Telemetry:
    uptime_ms: int = 0
    match_elapsed_ms: int = 0
    mission_state: int = 0
    team: int = TEAM_UNSET
    start_zone: int = 0
    run_mode: int = MODE_COMPETITION
    status_flags: int = 0
    fault_flags: int = 0
    battery_mv: int = 0
    yaw_cdeg: int = 0
    vx_mm_s: int = 0
    omega_mrad_s: int = 0
    delivered_green: int = 0
    delivered_black: int = 0
    delivered_orange: int = 0
    max_materials_per_trip: int = 1


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > PROTO_MAX_PAYLOAD:
        raise ValueError(f"payload length {len(payload)} exceeds {PROTO_MAX_PAYLOAD}")
    body = bytes((PROTO_VERSION, msg_type & 0xFF, seq & 0xFF, len(payload))) + payload
    crc = crc16_ccitt(body)
    return bytes((PROTO_SOF0, PROTO_SOF1)) + body + struct.pack("<H", crc)


def encode_heartbeat(seq: int, msg: Heartbeat) -> bytes:
    payload = _HEARTBEAT_STRUCT.pack(
        msg.vision_uptime_ms & 0xFFFFFFFF,
        msg.camera_ok & 0xFF,
        msg.ui_state & 0xFF,
        msg.reserved & 0xFFFF,
    )
    return build_frame(MSG_HEARTBEAT, seq, payload)


def encode_set_config(seq: int, msg: MatchConfigCmd) -> bytes:
    payload = _SET_CONFIG_STRUCT.pack(
        msg.team & 0xFF,
        msg.start_zone & 0xFF,
        msg.run_mode & 0xFF,
        msg.max_materials_per_trip & 0xFF,
    )
    return build_frame(MSG_SET_CONFIG, seq, payload)


def encode_vision_state(seq: int, msg: VisionState) -> bytes:
    payload = _VISION_STATE_STRUCT.pack(
        msg.frame_seq & 0xFFFF,
        msg.camera_view & 0xFF,
        msg.target_type & 0xFF,
        msg.flags & 0xFFFF,
        max(-32768, min(32767, int(msg.target_err_x_mrad))),
        max(0, min(65535, int(msg.target_dist_mm))),
        max(-32768, min(32767, int(msg.home_err_x_mrad))),
        max(0, min(65535, int(msg.home_dist_mm))),
        msg.corral_green_cnt & 0xFF,
        msg.corral_black_cnt & 0xFF,
        msg.corral_orange_cnt & 0xFF,
        msg.corral_cyan_cnt & 0xFF,
        msg.corral_unknown_cnt & 0xFF,
        max(0, min(100, int(msg.confidence))),
    )
    return build_frame(MSG_VISION_STATE, seq, payload)


def encode_bench_actuator(seq: int, msg: BenchActuatorCmd) -> bytes:
    payload = _BENCH_ACTUATOR_STRUCT.pack(
        msg.gate_cmd & 0xFF,
        msg.pitch_cmd & 0xFF,
        msg.reserved & 0xFFFF,
    )
    return build_frame(MSG_BENCH_ACTUATOR, seq, payload)


def encode_telemetry(seq: int, msg: Telemetry) -> bytes:
    payload = _TELEMETRY_STRUCT.pack(
        msg.uptime_ms & 0xFFFFFFFF,
        msg.match_elapsed_ms & 0xFFFFFFFF,
        msg.mission_state & 0xFF,
        msg.team & 0xFF,
        msg.start_zone & 0xFF,
        msg.run_mode & 0xFF,
        msg.status_flags & 0xFFFF,
        msg.fault_flags & 0xFFFF,
        msg.battery_mv & 0xFFFF,
        max(-32768, min(32767, int(msg.yaw_cdeg))),
        max(-32768, min(32767, int(msg.vx_mm_s))),
        max(-32768, min(32767, int(msg.omega_mrad_s))),
        msg.delivered_green & 0xFF,
        msg.delivered_black & 0xFF,
        msg.delivered_orange & 0xFF,
        msg.max_materials_per_trip & 0xFF,
    )
    return build_frame(MSG_TELEMETRY, seq, payload)


def decode_telemetry(frame: Frame) -> Telemetry | None:
    if frame.msg_type != MSG_TELEMETRY or len(frame.payload) != _TELEMETRY_STRUCT.size:
        return None
    return Telemetry(*_TELEMETRY_STRUCT.unpack(frame.payload))


def decode_vision_state(frame: Frame) -> VisionState | None:
    if frame.msg_type != MSG_VISION_STATE or len(frame.payload) != _VISION_STATE_STRUCT.size:
        return None
    return VisionState(*_VISION_STATE_STRUCT.unpack(frame.payload))


class RescueProtocolParser:
    """Streaming frame parser matching Class_Rescue_Protocol_Parser in C++."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.valid_frames = 0
        self.bad_crc_frames = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[Frame]:
        frames: List[Frame] = []
        self._buf.extend(data)

        while True:
            sof_idx = self._buf.find(bytes((PROTO_SOF0, PROTO_SOF1)))
            if sof_idx < 0:
                # Retain trailing 0xAA in case 0x55 arrives in the next chunk.
                if self._buf and self._buf[-1] == PROTO_SOF0:
                    del self._buf[:-1]
                else:
                    self._buf.clear()
                break

            if sof_idx > 0:
                del self._buf[:sof_idx]

            if len(self._buf) < PROTO_HEADER_LEN:
                break

            ver = self._buf[2]
            msg_type = self._buf[3]
            seq = self._buf[4]
            payload_len = self._buf[5]
            if ver != PROTO_VERSION or payload_len > PROTO_MAX_PAYLOAD:
                del self._buf[0]
                continue

            total = PROTO_HEADER_LEN + payload_len + PROTO_CRC_LEN
            if len(self._buf) < total:
                break

            body = bytes(self._buf[2 : PROTO_HEADER_LEN + payload_len])
            rx_crc = struct.unpack_from("<H", self._buf, PROTO_HEADER_LEN + payload_len)[0]
            if crc16_ccitt(body) != rx_crc:
                self.bad_crc_frames += 1
                del self._buf[0]
                continue

            payload = bytes(self._buf[PROTO_HEADER_LEN : PROTO_HEADER_LEN + payload_len])
            del self._buf[:total]
            self.valid_frames += 1
            frames.append(Frame(msg_type=msg_type, seq=seq, payload=payload))

        return frames
