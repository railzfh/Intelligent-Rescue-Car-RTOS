#include "1_Middleware/Protocol/rescue_protocol.h"

#include <cassert>
#include <cstdio>
#include <vector>

static void Test_Roundtrip_All_Messages()
{
    Class_Rescue_Protocol_Parser parser;
    uint8_t buf[RESCUE_PROTO_MAX_FRAME];
    Struct_Rescue_Frame frame;

    // 1. Heartbeat
    {
        const Struct_Vision_Heartbeat tx{123456U, 1U, 2U, 0U};
        const size_t n = Rescue_Encode_Heartbeat(7U, tx, buf, sizeof(buf));
        assert(n == RESCUE_PROTO_HEADER_LEN + RESCUE_PAYLOAD_LEN_HEARTBEAT + RESCUE_PROTO_CRC_LEN);
        bool got = false;
        for (size_t i = 0; i < n; ++i)
        {
            got = parser.Feed_Byte(buf[i], frame);
            assert(got == (i + 1U == n));
        }
        Struct_Vision_Heartbeat rx{};
        assert(frame.seq == 7U);
        assert(Rescue_Decode_Heartbeat(frame, rx));
        assert(rx.vision_uptime_ms == tx.vision_uptime_ms);
        assert(rx.camera_ok == 1U && rx.ui_state == 2U);
    }

    // 2. Set Config
    {
        const Struct_Match_Config_Cmd tx{RESCUE_TEAM_BLUE, 3U, RESCUE_MODE_COMPETITION, 2U};
        const size_t n = Rescue_Encode_Set_Config(8U, tx, buf, sizeof(buf));
        assert(n == RESCUE_PROTO_HEADER_LEN + RESCUE_PAYLOAD_LEN_SET_CONFIG + RESCUE_PROTO_CRC_LEN);
        bool got = false;
        for (size_t i = 0; i < n; ++i)
        {
            got = parser.Feed_Byte(buf[i], frame);
        }
        assert(got);
        Struct_Match_Config_Cmd rx{};
        assert(Rescue_Decode_Set_Config(frame, rx));
        assert(rx.team == RESCUE_TEAM_BLUE && rx.start_zone == 3U &&
               rx.run_mode == RESCUE_MODE_COMPETITION && rx.max_materials_per_trip == 2U);
    }

    // 3. Vision State (with negative signed bearing errors)
    {
        const Struct_Vision_State tx{
            420U, CAMERA_VIEW_NEAR, TARGET_GREEN_MATERIAL,
            static_cast<uint16_t>(VIS_FLAG_TARGET_VALID | VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR),
            -315, 240U, 185, 950U, 1U, 0U, 0U, 0U, 0U, 92U};
        const size_t n = Rescue_Encode_Vision_State(9U, tx, buf, sizeof(buf));
        assert(n == RESCUE_PROTO_HEADER_LEN + RESCUE_PAYLOAD_LEN_VISION_STATE + RESCUE_PROTO_CRC_LEN);
        bool got = false;
        for (size_t i = 0; i < n; ++i)
        {
            got = parser.Feed_Byte(buf[i], frame);
        }
        assert(got);
        Struct_Vision_State rx{};
        assert(Rescue_Decode_Vision_State(frame, rx));
        assert(rx.frame_seq == 420U && rx.camera_view == CAMERA_VIEW_NEAR);
        assert(rx.target_err_x_mrad == -315 && rx.home_err_x_mrad == 185);
        assert(rx.corral_green_cnt == 1U && rx.confidence == 92U);
    }

    // 4. Telemetry
    {
        const Struct_Rescue_Telemetry tx{
            50000U, 12345U, 5U, RESCUE_TEAM_RED, 2U, RESCUE_MODE_COMPETITION,
            static_cast<uint16_t>(TEL_FLAG_CONFIG_LOCKED | TEL_FLAG_BATTERY_OK | TEL_FLAG_FIRST_GREEN_UNLOCKED),
            0U, 24120U, -12345, -120, 450, 2U, 1U, 1U, 3U};
        const size_t n = Rescue_Encode_Telemetry(42U, tx, buf, sizeof(buf));
        assert(n == RESCUE_PROTO_HEADER_LEN + RESCUE_PAYLOAD_LEN_TELEMETRY + RESCUE_PROTO_CRC_LEN);
        bool got = false;
        for (size_t i = 0; i < n; ++i)
        {
            got = parser.Feed_Byte(buf[i], frame);
        }
        assert(got);
        Struct_Rescue_Telemetry rx{};
        assert(Rescue_Decode_Telemetry(frame, rx));
        assert(rx.uptime_ms == 50000U && rx.match_elapsed_ms == 12345U);
        assert(rx.battery_mv == 24120U && rx.yaw_cdeg == -12345 && rx.vx_mm_s == -120);
        assert(rx.delivered_green == 2U && rx.delivered_black == 1U && rx.delivered_orange == 1U);
    }
}

static void Test_Corruption_Resync_And_Split_Packets()
{
    Class_Rescue_Protocol_Parser parser;
    uint8_t valid[RESCUE_PROTO_MAX_FRAME];
    const Struct_Match_Config_Cmd cfg{RESCUE_TEAM_RED, 4U, RESCUE_MODE_COMPETITION, 1U};
    const size_t valid_len = Rescue_Encode_Set_Config(99U, cfg, valid, sizeof(valid));

    // Build noisy stream: garbage + duplicate 0xAA + fake header whose payload embeds a real frame!
    std::vector<uint8_t> stream = {0x00, 0xFF, 0xAA, 0xAA, 0x55, 0x01, 0x02, 0x00, 0x04, 0x11, 0x22};
    // Flip a corrupted CRC on a short frame then immediately follow with valid frame.
    std::vector<uint8_t> bad_crc(valid, valid + valid_len);
    bad_crc.back() ^= 0xFFU;
    stream.insert(stream.end(), bad_crc.begin(), bad_crc.end());
    // Embed valid frame right inside an invalid-length fake header prefix.
    stream.push_back(0xAAU);
    stream.push_back(0x55U);
    stream.push_back(0x01U);
    stream.push_back(0x03U);
    stream.push_back(0x01U);
    stream.push_back(0x02U); // Fake length=2 -> 2 payload bytes + 2 CRC bytes fail CRC, resync finds valid!
    stream.insert(stream.end(), valid, valid + valid_len);

    int decoded_valid = 0;
    Struct_Rescue_Frame frame;
    for (uint8_t b : stream)
    {
        if (parser.Feed_Byte(b, frame))
        {
            Struct_Match_Config_Cmd rx{};
            assert(Rescue_Decode_Set_Config(frame, rx));
            assert(frame.seq == 99U && rx.team == RESCUE_TEAM_RED && rx.start_zone == 4U);
            ++decoded_valid;
        }
    }
    assert(decoded_valid == 1);
    assert(parser.Get_Bad_Crc_Count() >= 2U);
    assert(parser.Get_Valid_Frame_Count() == 1U);
}

static void Test_Resync_Drains_All_Buffered_Frames_Without_Extra_Byte()
{
    uint8_t inner[64] = {};
    const size_t first = Rescue_Encode_Set_Config(
        1U, {RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION, 1U}, inner, sizeof(inner));
    const size_t second = Rescue_Encode_Vision_State(2U, {}, inner + first, sizeof(inner) - first);
    assert(first == 12U && second == 28U);

    uint8_t wire[RESCUE_PROTO_MAX_FRAME] = {};
    const size_t total = Rescue_Build_Frame(0x99U, 0U, inner, sizeof(inner), wire, sizeof(wire));
    assert(total == RESCUE_PROTO_MAX_FRAME);
    wire[total - 1U] ^= 0x01U;

    Class_Rescue_Protocol_Parser parser;
    Struct_Rescue_Frame frame;
    std::vector<uint8_t> received_types;
    for (size_t i = 0; i < total; ++i)
    {
        if (parser.Feed_Byte(wire[i], frame))
        {
            received_types.push_back(frame.msg_type);
            while (parser.Poll_Buffered_Frame(frame))
            {
                received_types.push_back(frame.msg_type);
            }
        }
    }
    assert(received_types.size() == 2U);
    assert(received_types[0] == RESCUE_MSG_SET_CONFIG);
    assert(received_types[1] == RESCUE_MSG_VISION_STATE);
    assert(!parser.Poll_Buffered_Frame(frame));
    assert(parser.Get_Valid_Frame_Count() == 2U);
}

int main()
{
    Test_Roundtrip_All_Messages();
    Test_Corruption_Resync_And_Split_Packets();
    Test_Resync_Drains_All_Buffered_Frames_Without_Extra_Byte();
    std::puts("PASS: rescue binary protocol CRC16, roundtrip, corruption resync, multi-frame buffered drain");
}
