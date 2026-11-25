#include "rescue_protocol.h"

#include <string.h>

static uint16_t Read_U16_LE(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8U);
}

static int16_t Read_I16_LE(const uint8_t *p)
{
    return static_cast<int16_t>(Read_U16_LE(p));
}

static uint32_t Read_U32_LE(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8U) |
           (static_cast<uint32_t>(p[2]) << 16U) |
           (static_cast<uint32_t>(p[3]) << 24U);
}

static void Write_U16_LE(uint8_t *p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFFU);
    p[1] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
}

static void Write_I16_LE(uint8_t *p, int16_t v)
{
    Write_U16_LE(p, static_cast<uint16_t>(v));
}

static void Write_U32_LE(uint8_t *p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFFU);
    p[1] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
    p[2] = static_cast<uint8_t>((v >> 16U) & 0xFFU);
    p[3] = static_cast<uint8_t>((v >> 24U) & 0xFFU);
}

uint16_t Rescue_CRC16_CCITT(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

size_t Rescue_Build_Frame(uint8_t msg_type, uint8_t seq, const uint8_t *payload,
                          uint8_t payload_len, uint8_t *out_buf, size_t out_capacity)
{
    const size_t total = static_cast<size_t>(RESCUE_PROTO_HEADER_LEN) + payload_len + RESCUE_PROTO_CRC_LEN;
    if (out_buf == nullptr || payload_len > RESCUE_PROTO_MAX_PAYLOAD || out_capacity < total)
    {
        return 0U;
    }
    if (payload_len > 0U && payload == nullptr)
    {
        return 0U;
    }

    out_buf[0] = RESCUE_PROTO_SOF0;
    out_buf[1] = RESCUE_PROTO_SOF1;
    out_buf[2] = RESCUE_PROTO_VERSION;
    out_buf[3] = msg_type;
    out_buf[4] = seq;
    out_buf[5] = payload_len;
    if (payload_len > 0U)
    {
        memcpy(out_buf + RESCUE_PROTO_HEADER_LEN, payload, payload_len);
    }

    const uint16_t crc = Rescue_CRC16_CCITT(out_buf + 2U, 4U + payload_len);
    Write_U16_LE(out_buf + RESCUE_PROTO_HEADER_LEN + payload_len, crc);
    return total;
}

bool Rescue_Decode_Heartbeat(const Struct_Rescue_Frame &frame, Struct_Vision_Heartbeat &out)
{
    if (frame.msg_type != RESCUE_MSG_HEARTBEAT || frame.payload_len != RESCUE_PAYLOAD_LEN_HEARTBEAT)
    {
        return false;
    }
    const uint8_t *p = frame.payload;
    out.vision_uptime_ms = Read_U32_LE(p + 0);
    out.camera_ok = p[4];
    out.ui_state = p[5];
    out.reserved = Read_U16_LE(p + 6);
    return true;
}

bool Rescue_Decode_Set_Config(const Struct_Rescue_Frame &frame, Struct_Match_Config_Cmd &out)
{
    if (frame.msg_type != RESCUE_MSG_SET_CONFIG || frame.payload_len != RESCUE_PAYLOAD_LEN_SET_CONFIG)
    {
        return false;
    }
    const uint8_t *p = frame.payload;
    out.team = p[0];
    out.start_zone = p[1];
    out.run_mode = p[2];
    out.max_materials_per_trip = p[3];
    return true;
}

bool Rescue_Decode_Vision_State(const Struct_Rescue_Frame &frame, Struct_Vision_State &out)
{
    if (frame.msg_type != RESCUE_MSG_VISION_STATE || frame.payload_len != RESCUE_PAYLOAD_LEN_VISION_STATE)
    {
        return false;
    }
    const uint8_t *p = frame.payload;
    out.frame_seq = Read_U16_LE(p + 0);
    out.camera_view = p[2];
    out.target_type = p[3];
    out.flags = Read_U16_LE(p + 4);
    out.target_err_x_mrad = Read_I16_LE(p + 6);
    out.target_dist_mm = Read_U16_LE(p + 8);
    out.home_err_x_mrad = Read_I16_LE(p + 10);
    out.home_dist_mm = Read_U16_LE(p + 12);
    out.corral_green_cnt = p[14];
    out.corral_black_cnt = p[15];
    out.corral_orange_cnt = p[16];
    out.corral_cyan_cnt = p[17];
    out.corral_unknown_cnt = p[18];
    out.confidence = p[19];
    return true;
}

bool Rescue_Decode_Bench_Actuator(const Struct_Rescue_Frame &frame, Struct_Bench_Actuator_Cmd &out)
{
    if (frame.msg_type != RESCUE_MSG_BENCH_ACTUATOR || frame.payload_len != RESCUE_PAYLOAD_LEN_BENCH_ACTUATOR)
    {
        return false;
    }
    const uint8_t *p = frame.payload;
    out.gate_cmd = p[0];
    out.pitch_cmd = p[1];
    out.reserved = Read_U16_LE(p + 2);
    return true;
}

size_t Rescue_Encode_Heartbeat(uint8_t seq, const Struct_Vision_Heartbeat &msg,
                               uint8_t *out_buf, size_t out_capacity)
{
    uint8_t payload[RESCUE_PAYLOAD_LEN_HEARTBEAT];
    Write_U32_LE(payload + 0, msg.vision_uptime_ms);
    payload[4] = msg.camera_ok;
    payload[5] = msg.ui_state;
    Write_U16_LE(payload + 6, msg.reserved);
    return Rescue_Build_Frame(RESCUE_MSG_HEARTBEAT, seq, payload, sizeof(payload), out_buf, out_capacity);
}

size_t Rescue_Encode_Set_Config(uint8_t seq, const Struct_Match_Config_Cmd &msg,
                                uint8_t *out_buf, size_t out_capacity)
{
    uint8_t payload[RESCUE_PAYLOAD_LEN_SET_CONFIG];
    payload[0] = msg.team;
    payload[1] = msg.start_zone;
    payload[2] = msg.run_mode;
    payload[3] = msg.max_materials_per_trip;
    return Rescue_Build_Frame(RESCUE_MSG_SET_CONFIG, seq, payload, sizeof(payload), out_buf, out_capacity);
}

size_t Rescue_Encode_Vision_State(uint8_t seq, const Struct_Vision_State &msg,
                                  uint8_t *out_buf, size_t out_capacity)
{
    uint8_t payload[RESCUE_PAYLOAD_LEN_VISION_STATE];
    Write_U16_LE(payload + 0, msg.frame_seq);
    payload[2] = msg.camera_view;
    payload[3] = msg.target_type;
    Write_U16_LE(payload + 4, msg.flags);
    Write_I16_LE(payload + 6, msg.target_err_x_mrad);
    Write_U16_LE(payload + 8, msg.target_dist_mm);
    Write_I16_LE(payload + 10, msg.home_err_x_mrad);
    Write_U16_LE(payload + 12, msg.home_dist_mm);
    payload[14] = msg.corral_green_cnt;
    payload[15] = msg.corral_black_cnt;
    payload[16] = msg.corral_orange_cnt;
    payload[17] = msg.corral_cyan_cnt;
    payload[18] = msg.corral_unknown_cnt;
    payload[19] = msg.confidence;
    return Rescue_Build_Frame(RESCUE_MSG_VISION_STATE, seq, payload, sizeof(payload), out_buf, out_capacity);
}

size_t Rescue_Encode_Bench_Actuator(uint8_t seq, const Struct_Bench_Actuator_Cmd &msg,
                                    uint8_t *out_buf, size_t out_capacity)
{
    uint8_t payload[RESCUE_PAYLOAD_LEN_BENCH_ACTUATOR];
    payload[0] = msg.gate_cmd;
    payload[1] = msg.pitch_cmd;
    Write_U16_LE(payload + 2, msg.reserved);
    return Rescue_Build_Frame(RESCUE_MSG_BENCH_ACTUATOR, seq, payload, sizeof(payload), out_buf, out_capacity);
}

size_t Rescue_Encode_Telemetry(uint8_t seq, const Struct_Rescue_Telemetry &msg,
                               uint8_t *out_buf, size_t out_capacity)
{
    uint8_t payload[RESCUE_PAYLOAD_LEN_TELEMETRY];
    Write_U32_LE(payload + 0, msg.uptime_ms);
    Write_U32_LE(payload + 4, msg.match_elapsed_ms);
    payload[8] = msg.mission_state;
    payload[9] = msg.team;
    payload[10] = msg.start_zone;
    payload[11] = msg.run_mode;
    Write_U16_LE(payload + 12, msg.status_flags);
    Write_U16_LE(payload + 14, msg.fault_flags);
    Write_U16_LE(payload + 16, msg.battery_mv);
    Write_I16_LE(payload + 18, msg.yaw_cdeg);
    Write_I16_LE(payload + 20, msg.vx_mm_s);
    Write_I16_LE(payload + 22, msg.omega_mrad_s);
    payload[24] = msg.delivered_green;
    payload[25] = msg.delivered_black;
    payload[26] = msg.delivered_orange;
    payload[27] = msg.max_materials_per_trip;
    return Rescue_Build_Frame(RESCUE_MSG_TELEMETRY, seq, payload, sizeof(payload), out_buf, out_capacity);
}

bool Rescue_Decode_Telemetry(const Struct_Rescue_Frame &frame, Struct_Rescue_Telemetry &out)
{
    if (frame.msg_type != RESCUE_MSG_TELEMETRY || frame.payload_len != RESCUE_PAYLOAD_LEN_TELEMETRY)
    {
        return false;
    }
    const uint8_t *p = frame.payload;
    out.uptime_ms = Read_U32_LE(p + 0);
    out.match_elapsed_ms = Read_U32_LE(p + 4);
    out.mission_state = p[8];
    out.team = p[9];
    out.start_zone = p[10];
    out.run_mode = p[11];
    out.status_flags = Read_U16_LE(p + 12);
    out.fault_flags = Read_U16_LE(p + 14);
    out.battery_mv = Read_U16_LE(p + 16);
    out.yaw_cdeg = Read_I16_LE(p + 18);
    out.vx_mm_s = Read_I16_LE(p + 20);
    out.omega_mrad_s = Read_I16_LE(p + 22);
    out.delivered_green = p[24];
    out.delivered_black = p[25];
    out.delivered_orange = p[26];
    out.max_materials_per_trip = p[27];
    return true;
}

void Class_Rescue_Protocol_Parser::Reset()
{
    Count = 0U;
    Expected_Total = 0U;
}

bool Class_Rescue_Protocol_Parser::Try_Shift_And_Resync()
{
    // Scan past byte[0] to preserve any real SOF embedded inside a corrupted packet.
    for (uint8_t i = 1U; i < Count; ++i)
    {
        if (Buffer[i] != RESCUE_PROTO_SOF0)
        {
            continue;
        }
        const uint8_t remaining = static_cast<uint8_t>(Count - i);
        if (remaining >= 2U && Buffer[i + 1U] != RESCUE_PROTO_SOF1)
        {
            continue;
        }
        memmove(Buffer, Buffer + i, remaining);
        Count = remaining;
        Expected_Total = 0U;
        if (Count >= RESCUE_PROTO_HEADER_LEN)
        {
            if (Buffer[2] == RESCUE_PROTO_VERSION && Buffer[5] <= RESCUE_PROTO_MAX_PAYLOAD)
            {
                Expected_Total = static_cast<uint8_t>(RESCUE_PROTO_HEADER_LEN + Buffer[5] + RESCUE_PROTO_CRC_LEN);
                if (Count >= Expected_Total)
                {
                    return true;
                }
            }
            else
            {
                // Keep scanning forward from this candidate.
                i = 0U;
                continue;
            }
        }
        return false;
    }
    Count = 0U;
    Expected_Total = 0U;
    return false;
}

bool Class_Rescue_Protocol_Parser::Feed_Byte(uint8_t byte, Struct_Rescue_Frame &out_frame)
{
    if (Count == 0U)
    {
        if (byte == RESCUE_PROTO_SOF0)
        {
            Buffer[0] = byte;
            Count = 1U;
        }
        return false;
    }

    if (Count == 1U)
    {
        if (byte == RESCUE_PROTO_SOF1)
        {
            Buffer[1] = byte;
            Count = 2U;
        }
        else if (byte == RESCUE_PROTO_SOF0)
        {
            Buffer[0] = RESCUE_PROTO_SOF0;
            Count = 1U;
        }
        else
        {
            Count = 0U;
        }
        return false;
    }

    if (Count >= RESCUE_PROTO_MAX_FRAME)
    {
        Reset();
        return false;
    }

    Buffer[Count++] = byte;

    while (true)
    {
        if (Count < RESCUE_PROTO_HEADER_LEN)
        {
            return false;
        }

        if (Expected_Total == 0U)
        {
            if (Buffer[2] != RESCUE_PROTO_VERSION || Buffer[5] > RESCUE_PROTO_MAX_PAYLOAD)
            {
                if (!Try_Shift_And_Resync())
                {
                    return false;
                }
                continue;
            }
            Expected_Total = static_cast<uint8_t>(RESCUE_PROTO_HEADER_LEN + Buffer[5] + RESCUE_PROTO_CRC_LEN);
        }

        if (Count < Expected_Total)
        {
            return false;
        }

        const uint8_t payload_len = Buffer[5];
        const uint16_t computed_crc = Rescue_CRC16_CCITT(Buffer + 2U, 4U + payload_len);
        const uint16_t rx_crc = Read_U16_LE(Buffer + RESCUE_PROTO_HEADER_LEN + payload_len);
        if (computed_crc != rx_crc)
        {
            ++Bad_Crc_Frames;
            if (!Try_Shift_And_Resync())
            {
                return false;
            }
            continue;
        }

        out_frame.msg_type = Buffer[3];
        out_frame.seq = Buffer[4];
        out_frame.payload_len = payload_len;
        if (payload_len > 0U)
        {
            memcpy(out_frame.payload, Buffer + RESCUE_PROTO_HEADER_LEN, payload_len);
        }
        ++Valid_Frames;

        if (Count > Expected_Total)
        {
            const uint8_t extra = static_cast<uint8_t>(Count - Expected_Total);
            memmove(Buffer, Buffer + Expected_Total, extra);
            Count = extra;
            Expected_Total = 0U;
            // Re-align any leftover bytes starting at Buffer[0].
            if (Buffer[0] != RESCUE_PROTO_SOF0 || (Count >= 2U && Buffer[1] != RESCUE_PROTO_SOF1))
            {
                Try_Shift_And_Resync();
            }
        }
        else
        {
            Reset();
        }
        return true;
    }
}
