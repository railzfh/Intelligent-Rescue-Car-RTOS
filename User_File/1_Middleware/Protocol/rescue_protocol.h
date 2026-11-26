#ifndef RESCUE_PROTOCOL_H
#define RESCUE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

static constexpr uint8_t RESCUE_PROTO_SOF0 = 0xAAU;
static constexpr uint8_t RESCUE_PROTO_SOF1 = 0x55U;
static constexpr uint8_t RESCUE_PROTO_VERSION = 0x01U;
static constexpr uint8_t RESCUE_PROTO_HEADER_LEN = 6U;
static constexpr uint8_t RESCUE_PROTO_CRC_LEN = 2U;
static constexpr uint8_t RESCUE_PROTO_MAX_PAYLOAD = 64U;
static constexpr uint8_t RESCUE_PROTO_MAX_FRAME =
    RESCUE_PROTO_HEADER_LEN + RESCUE_PROTO_MAX_PAYLOAD + RESCUE_PROTO_CRC_LEN;

enum Enum_Rescue_Msg_Type : uint8_t
{
    RESCUE_MSG_HEARTBEAT = 0x01U,
    RESCUE_MSG_SET_CONFIG = 0x02U,
    RESCUE_MSG_VISION_STATE = 0x03U,
    RESCUE_MSG_BENCH_ACTUATOR = 0x04U,
    RESCUE_MSG_TELEMETRY = 0x81U,
};

enum Enum_Rescue_Team : uint8_t
{
    RESCUE_TEAM_UNSET = 0U,
    RESCUE_TEAM_RED = 1U,
    RESCUE_TEAM_BLUE = 2U,
};

enum Enum_Rescue_Run_Mode : uint8_t
{
    RESCUE_MODE_COMPETITION = 0U,
    RESCUE_MODE_BENCH_STRAIGHT = 1U,
};

enum Enum_Camera_View : uint8_t
{
    CAMERA_VIEW_TRANSITION = 0U,
    CAMERA_VIEW_FAR = 1U,
    CAMERA_VIEW_NEAR = 2U,
};

enum Enum_Target_Type : uint8_t
{
    TARGET_NONE = 0U,
    TARGET_GREEN_MATERIAL = 1U,
    TARGET_BLACK_CORE = 2U,
    TARGET_ORANGE_CASUALTY = 3U,
    TARGET_CYAN_HAZARD = 4U,
};

enum Enum_Bench_Servo_Cmd : uint8_t
{
    BENCH_SERVO_KEEP = 0U,
    BENCH_SERVO_POS_A = 1U, // Gate: OPEN, Pitch: FAR
    BENCH_SERVO_POS_B = 2U, // Gate: CLOSED, Pitch: NEAR
};

// Bitmask for Struct_Vision_State::flags
static constexpr uint16_t VIS_FLAG_TARGET_VALID = (1U << 0);
static constexpr uint16_t VIS_FLAG_HOME_VALID = (1U << 1);
static constexpr uint16_t VIS_FLAG_HOME_HALF_MATCH = (1U << 2);
static constexpr uint16_t VIS_FLAG_RAMP_VISIBLE = (1U << 3);
static constexpr uint16_t VIS_FLAG_CORRAL_SAFE = (1U << 4);
static constexpr uint16_t VIS_FLAG_SWEEP_CLEAR = (1U << 5);
static constexpr uint16_t VIS_FLAG_DROP_CONFIRMED = (1U << 6);
static constexpr uint16_t VIS_FLAG_OPPONENT_NEAR = (1U << 7);
static constexpr uint16_t VIS_FLAG_OPPONENT_ZONE_NEAR = (1U << 8);
static constexpr uint16_t VIS_FLAG_REAR_CLEAR = (1U << 9);
static constexpr uint16_t VIS_FLAG_MOTION_STALLED = (1U << 10);
static constexpr uint16_t VIS_FLAG_DROP_PATH_CLEAR = (1U << 11);
static constexpr uint16_t VIS_FLAG_GATE_OPEN_SETTLED_ACK = (1U << 12);

// Bitmask for Struct_Rescue_Telemetry::status_flags
static constexpr uint16_t TEL_FLAG_CONFIG_LOCKED = (1U << 0);
static constexpr uint16_t TEL_FLAG_BATTERY_OK = (1U << 1);
static constexpr uint16_t TEL_FLAG_IMU_OK = (1U << 2);
static constexpr uint16_t TEL_FLAG_MOTORS_OK = (1U << 3);
static constexpr uint16_t TEL_FLAG_VISION_FRESH = (1U << 4);
static constexpr uint16_t TEL_FLAG_FIRST_GREEN_UNLOCKED = (1U << 5);
static constexpr uint16_t TEL_FLAG_GATE_CLOSED = (1U << 6);
static constexpr uint16_t TEL_FLAG_GATE_SETTLED = (1U << 7);
static constexpr uint16_t TEL_FLAG_CAMERA_NEAR = (1U << 8);
static constexpr uint16_t TEL_FLAG_CAMERA_SETTLED = (1U << 9);
static constexpr uint16_t TEL_FLAG_STALLED = (1U << 10);
static constexpr uint16_t TEL_FLAG_LINK_ALIVE = (1U << 11);
static constexpr uint16_t TEL_FLAG_CARRYING_CASUALTY = (1U << 12);
static constexpr uint16_t TEL_FLAG_DROP_PUSHING = (1U << 13);

// Bitmask for Struct_Rescue_Telemetry::fault_flags
static constexpr uint16_t FAULT_FLAG_BATTERY = (1U << 0);
static constexpr uint16_t FAULT_FLAG_MOTOR_TIMEOUT = (1U << 1);
static constexpr uint16_t FAULT_FLAG_IMU_TIMEOUT = (1U << 2);
static constexpr uint16_t FAULT_FLAG_CAN_TX = (1U << 3);
static constexpr uint16_t FAULT_FLAG_VISION_TIMEOUT = (1U << 4);
static constexpr uint16_t FAULT_FLAG_SERVO = (1U << 5);
static constexpr uint16_t FAULT_FLAG_UART_RX = (1U << 6);

static constexpr uint8_t RESCUE_PAYLOAD_LEN_HEARTBEAT = 8U;
static constexpr uint8_t RESCUE_PAYLOAD_LEN_SET_CONFIG = 4U;
static constexpr uint8_t RESCUE_PAYLOAD_LEN_VISION_STATE = 20U;
static constexpr uint8_t RESCUE_PAYLOAD_LEN_BENCH_ACTUATOR = 4U;
static constexpr uint8_t RESCUE_PAYLOAD_LEN_TELEMETRY = 28U;

struct Struct_Rescue_Frame
{
    uint8_t msg_type = 0U;
    uint8_t seq = 0U;
    uint8_t payload_len = 0U;
    uint8_t payload[RESCUE_PROTO_MAX_PAYLOAD] = {};
};

struct Struct_Vision_Heartbeat
{
    uint32_t vision_uptime_ms = 0U;
    uint8_t camera_ok = 0U;
    uint8_t ui_state = 0U;
    uint16_t reserved = 0U;
};

struct Struct_Match_Config_Cmd
{
    uint8_t team = RESCUE_TEAM_UNSET;
    uint8_t start_zone = 0U;
    uint8_t run_mode = RESCUE_MODE_COMPETITION;
    uint8_t max_materials_per_trip = 1U;
};

struct Struct_Vision_State
{
    uint16_t frame_seq = 0U;
    uint8_t camera_view = CAMERA_VIEW_TRANSITION;
    uint8_t target_type = TARGET_NONE;
    uint16_t flags = 0U;
    int16_t target_err_x_mrad = 0;
    uint16_t target_dist_mm = 0U;
    int16_t home_err_x_mrad = 0;
    uint16_t home_dist_mm = 0U;
    uint8_t corral_green_cnt = 0U;
    uint8_t corral_black_cnt = 0U;
    uint8_t corral_orange_cnt = 0U;
    uint8_t corral_cyan_cnt = 0U;
    uint8_t corral_unknown_cnt = 0U;
    uint8_t confidence = 0U;
};

struct Struct_Bench_Actuator_Cmd
{
    uint8_t gate_cmd = BENCH_SERVO_KEEP;
    uint8_t pitch_cmd = BENCH_SERVO_KEEP;
    uint16_t reserved = 0U;
};

struct Struct_Rescue_Telemetry
{
    uint32_t uptime_ms = 0U;
    uint32_t match_elapsed_ms = 0U;
    uint8_t mission_state = 0U;
    uint8_t team = RESCUE_TEAM_UNSET;
    uint8_t start_zone = 0U;
    uint8_t run_mode = RESCUE_MODE_COMPETITION;
    uint16_t status_flags = 0U;
    uint16_t fault_flags = 0U;
    uint16_t battery_mv = 0U;
    int16_t yaw_cdeg = 0;
    int16_t vx_mm_s = 0;
    int16_t omega_mrad_s = 0;
    uint8_t delivered_green = 0U;
    uint8_t delivered_black = 0U;
    uint8_t delivered_orange = 0U;
    uint8_t max_materials_per_trip = 1U;
};

uint16_t Rescue_CRC16_CCITT(const uint8_t *data, size_t length);

size_t Rescue_Build_Frame(uint8_t msg_type, uint8_t seq, const uint8_t *payload,
                          uint8_t payload_len, uint8_t *out_buf, size_t out_capacity);

bool Rescue_Decode_Heartbeat(const Struct_Rescue_Frame &frame, Struct_Vision_Heartbeat &out);
bool Rescue_Decode_Set_Config(const Struct_Rescue_Frame &frame, Struct_Match_Config_Cmd &out);
bool Rescue_Decode_Vision_State(const Struct_Rescue_Frame &frame, Struct_Vision_State &out);
bool Rescue_Decode_Bench_Actuator(const Struct_Rescue_Frame &frame, Struct_Bench_Actuator_Cmd &out);

size_t Rescue_Encode_Heartbeat(uint8_t seq, const Struct_Vision_Heartbeat &msg,
                               uint8_t *out_buf, size_t out_capacity);
size_t Rescue_Encode_Set_Config(uint8_t seq, const Struct_Match_Config_Cmd &msg,
                                uint8_t *out_buf, size_t out_capacity);
size_t Rescue_Encode_Vision_State(uint8_t seq, const Struct_Vision_State &msg,
                                  uint8_t *out_buf, size_t out_capacity);
size_t Rescue_Encode_Bench_Actuator(uint8_t seq, const Struct_Bench_Actuator_Cmd &msg,
                                    uint8_t *out_buf, size_t out_capacity);
size_t Rescue_Encode_Telemetry(uint8_t seq, const Struct_Rescue_Telemetry &msg,
                               uint8_t *out_buf, size_t out_capacity);
bool Rescue_Decode_Telemetry(const Struct_Rescue_Frame &frame, Struct_Rescue_Telemetry &out);

// Byte-stream frame parser with resynchronization on corrupted headers or bad CRC.
class Class_Rescue_Protocol_Parser
{
public:
    void Reset();
    bool Feed_Byte(uint8_t byte, Struct_Rescue_Frame &out_frame);

    uint32_t Get_Valid_Frame_Count() const { return Valid_Frames; }
    uint32_t Get_Bad_Crc_Count() const { return Bad_Crc_Frames; }

private:
    bool Try_Shift_And_Resync();

    uint8_t Buffer[RESCUE_PROTO_MAX_FRAME] = {};
    uint8_t Count = 0U;
    uint8_t Expected_Total = 0U;
    uint32_t Valid_Frames = 0U;
    uint32_t Bad_Crc_Frames = 0U;
};

#endif
