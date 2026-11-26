"""Exercise the actual task's drive, mission, servo cold-boot, and protocol gates with mocks."""
import os
from pathlib import Path
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/native-rtos"
out.mkdir(parents=True, exist_ok=True)
task = (repo / "User_File/4_Task/tsk_config_and_callback.cpp").read_text(encoding="utf-8")
globals_text = task[task.index("bool init_finished"):task.index("/* Private function declarations")]
functions = task[task.index("static void Vision_Uart_RxCallback"):task.index("static void Wait_For_Next_Period")]
stub = r'''
#include "1_Middleware/Protocol/rescue_protocol.h"
#include "2_Device/Battery/dvc_battery.h"
#include "2_Device/Servo/dvc_servo.h"
#include "3_Robot/Drive/chassis_drive.h"
#include "3_Robot/Drive/straight_drive.h"
#include "3_Robot/Mission/rescue_mission.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

constexpr float BASIC_MATH_RPM_TO_RADPS = 3.14159265358979323846f / 30.0f;
enum Enum_Motor_DJI_ID { Motor_DJI_ID_0x201 = 1, Motor_DJI_ID_0x202 = 2 };
struct Struct_WS2812_Color { uint8_t Red, Green, Blue; };
struct Class_Motor_DJI_C610 {
    float angle = 0;
    float omega = 0;
    float torque = 0;
    float Get_Now_Angle() const { return angle; }
    float Get_Now_Omega() const { return omega; }
    void Set_Target_Torque(float t) { torque = t; }
    void TIM_Calculate_PeriodElapsedCallback() {}
};
struct Class_PWM_Servo {
    Class_Servo_Controller ctrl;
    bool initialized = false;
    bool fail_enable = false;
    int enable_calls = 0;
    int disable_calls = 0;
    bool Init_Mock(const Struct_Servo_Config &cfg, bool start_immediately = false) {
        ctrl.Init(cfg);
        initialized = true;
        if (!start_immediately) {
            ctrl.Disable();
        }
        return true;
    }
    void Set_Target_Us(uint16_t us) { ctrl.Set_Target_Us(us); }
    void Update_1ms() { ctrl.Update_1ms(); }
    void Disable() { ++disable_calls; ctrl.Disable(); }
    bool Enable() {
        ++enable_calls;
        if (!initialized || fail_enable) {
            ctrl.Disable();
            return false;
        }
        if (!ctrl.Is_Enabled()) {
            ctrl.Enable_At_Current();
        }
        return true;
    }
    bool Is_Enabled() const { return ctrl.Is_Enabled(); }
    bool Is_Settled() const { return ctrl.Is_Settled(); }
    uint16_t Get_Current_Us() const { return ctrl.Get_Current_Us(); }
};
struct UART_HandleTypeDef { bool rx_fault_latched = false; };
UART_HandleTypeDef huart1;
bool UART_Service_Rx_Recovery(UART_HandleTypeDef *h) { return !h->rx_fault_latched; }
bool UART_Is_Rx_Fault_Latched(UART_HandleTypeDef *h) { return h->rx_fault_latched; }
struct FDCAN_HandleTypeDef { bool pending = false, fail = false; int tx_calls = 0; };
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
uint8_t CAN1_0x200_Tx_Data[8], CAN2_0x200_Tx_Data[8];
constexpr int FDCAN_DLC_BYTES_8 = 8, HAL_OK = 0;
uint32_t now_ms = 0, irq_mask = 0;
bool key = false;
constexpr int KEY__INPUT_GPIO_Port = 0, KEY__INPUT_Pin = 0, GPIO_PIN_RESET = 0;
uint32_t HAL_GetTick() { return now_ms; }
uint32_t __get_PRIMASK() { return irq_mask; }
void __disable_irq() { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void __DMB() {}
int HAL_GPIO_ReadPin(int, int) { return key ? 0 : 1; }
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *) { return 0; }
bool HAL_FDCAN_IsTxBufferMessagePending(FDCAN_HandleTypeDef *c, uint32_t) { return c->pending; }
void HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *c, uint32_t) { c->pending = false; }
int CAN_Transmit_Data(FDCAN_HandleTypeDef *c, int, uint8_t *, int) { ++c->tx_calls; return c->fail ? 1 : 0; }
struct { float volume = 0; void Set_Loudness(float v) { volume = v; } } BSP_Buzzer;
'''
checks = r'''
void Feed_Config(uint8_t team, uint8_t zone, uint8_t mode) {
    uint8_t buf[RESCUE_PROTO_MAX_FRAME];
    static uint8_t seq = 1U;
    const size_t n = Rescue_Encode_Set_Config(seq++, {team, zone, mode, 1U}, buf, sizeof(buf));
    Vision_Uart_RxCallback(buf, static_cast<uint16_t>(n));
}
void Feed_Vision(uint16_t frame_seq, uint16_t flags = VIS_FLAG_TARGET_VALID) {
    uint8_t buf[RESCUE_PROTO_MAX_FRAME];
    static uint8_t seq = 10U;
    Struct_Vision_State vis{};
    vis.frame_seq = frame_seq;
    vis.camera_view = CAMERA_VIEW_FAR;
    vis.target_type = TARGET_GREEN_MATERIAL;
    vis.flags = flags;
    vis.target_dist_mm = 500U;
    vis.confidence = 90U;
    const size_t n = Rescue_Encode_Vision_State(seq++, vis, buf, sizeof(buf));
    Vision_Uart_RxCallback(buf, static_cast<uint16_t>(n));
}
void Step(uint32_t t, bool pressed, bool imu = true, bool update_battery = true) {
    now_ms = t; key = pressed;
    if (update_battery) {
        Battery.Update(24.0f, t);
    }
    M2006_Feedback_Received[0] = M2006_Feedback_Received[1] = true;
    M2006_Last_Feedback_Ms[0] = M2006_Last_Feedback_Ms[1] = t;
    Attitude_Valid = imu; Attitude_Sample_Ms = t; Imu_Quiet = true;
    Drive_Control();
    assert(irq_mask == 0);
}
void Stopped() {
    assert(!Straight_Drive.Is_Running());
    assert(!Chassis_Drive.Is_Active());
    assert(M2006[0].torque == 0 && M2006[1].torque == 0);
}
int main(int argc, char **argv) {
    // Match Task_Init(): configure timers with PWM stopped until first valid battery ADC sample.
    Gate_Servo.Init_Mock(Gate_Servo_Config, false);
    Pitch_Servo.Init_Mock(Pitch_Servo_Config, false);
    const bool held = argc > 1 && std::strcmp(argv[1], "held") == 0;
    const bool comp = argc > 1 && std::strcmp(argv[1], "comp") == 0;
    const bool can_fault = argc > 1 && std::strcmp(argv[1], "can_fault") == 0;
    const bool frozen = argc > 1 && std::strcmp(argv[1], "frozen") == 0;
    const bool cold_boot = argc > 1 && std::strcmp(argv[1], "cold_boot") == 0;
    const bool servo_fail = argc > 1 && std::strcmp(argv[1], "servo_fail") == 0;
    const bool reverse_abort = argc > 1 && std::strcmp(argv[1], "reverse_abort") == 0;

    if (reverse_abort) {
        // Coupled Drive_Control + Rescue_Mission + Chassis_Drive probe:
        // Active reverse losing rear clearance MUST cancel immediately to CHASSIS_MODE_IDLE
        // with ZERO torque (never entering CHASSIS_MODE_STOPPING with nonzero reverse torque).
        auto feed_near = [&](uint16_t fseq, uint8_t g, uint8_t c, uint16_t flags) {
            uint8_t buf[RESCUE_PROTO_MAX_FRAME];
            static uint8_t seq = 100U;
            Struct_Vision_State vis{};
            vis.frame_seq = fseq;
            vis.camera_view = CAMERA_VIEW_NEAR;
            vis.flags = flags;
            vis.corral_green_cnt = g;
            vis.corral_cyan_cnt = c;
            vis.confidence = 95U;
            const size_t n = Rescue_Encode_Vision_State(seq++, vis, buf, sizeof(buf));
            Vision_Uart_RxCallback(buf, static_cast<uint16_t>(n));
        };
        auto feed_far = [&](uint16_t fseq, uint8_t target, uint16_t flags, uint16_t dist_mm) {
            uint8_t buf[RESCUE_PROTO_MAX_FRAME];
            static uint8_t seq = 200U;
            Struct_Vision_State vis{};
            vis.frame_seq = fseq;
            vis.camera_view = CAMERA_VIEW_FAR;
            vis.target_type = target;
            vis.flags = flags;
            vis.target_dist_mm = dist_mm;
            vis.home_dist_mm = dist_mm;
            vis.confidence = 95U;
            const size_t n = Rescue_Encode_Vision_State(seq++, vis, buf, sizeof(buf));
            Vision_Uart_RxCallback(buf, static_cast<uint16_t>(n));
        };

        Feed_Config(RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION);
        uint16_t fseq = 1U;
        uint32_t t = 0U;
        // Cold boot -> settle both servos.
        for (; t < 200U; ++t) {
            if ((t % 40U) == 0U) { feed_far(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 500U); }
            Step(t, false);
        }
        // Launch LAUNCH_CROSS_BUMPS and physically integrate straight forward motion to build proven corridor.
        Step(t += 10U, true);
        Step(t += 30U, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
        for (int step = 0; step < 8000 && Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS; ++step) {
            ++t;
            if ((t % 40U) == 0U) { feed_far(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Step(t, false);
            for (int i = 0; i < 2; ++i) {
                M2006[i].angle += M2006[i].omega * 0.001f;
                M2006[i].omega += 40.0f * M2006[i].torque * 0.001f;
            }
        }
        assert(Rescue_Mission.Get_State() == MISSION_SEARCH_TARGET);
        // Settle wheels to standstill and transition SEARCH -> APPROACH -> INSPECT -> cyan -> BACK_OUT_SEPARATE.
        M2006[0].omega = M2006[1].omega = 0.0f;
        for (int step = 0; step < 300; ++step) {
            ++t;
            if ((t % 30U) == 0U) { feed_far(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Step(t, false);
            if (Rescue_Mission.Get_State() == MISSION_INSPECT_CORRAL) { break; }
        }
        assert(Rescue_Mission.Get_State() == MISSION_INSPECT_CORRAL);
        // Wait for pitch servo to settle near, then inject cyan to trigger margin-bounded reverse!
        for (int step = 0; step < 600 && !Pitch_Servo.Is_Settled(); ++step) {
            ++t;
            if ((t % 30U) == 0U) { feed_near(fseq++, 1U, 0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR); }
            Step(t, false);
        }
        assert(Rescue_Mission.Get_Proven_Rear_Clear_M() >= 0.15f);
        feed_near(fseq++, 0U, 1U, 0U);
        Step(++t, false);
        assert(Rescue_Mission.Get_State() == MISSION_BACK_OUT_SEPARATE);

        // Advance active reverse until wheels are visibly reversing (nonzero reverse speed & torque).
        bool saw_active_reverse = false;
        for (int step = 0; step < 300; ++step) {
            ++t;
            if ((t % 30U) == 0U) { feed_far(fseq++, TARGET_NONE, 0U, 500U); }
            Step(t, false);
            for (int i = 0; i < 2; ++i) {
                M2006[i].angle += M2006[i].omega * 0.001f;
                M2006[i].omega += 40.0f * M2006[i].torque * 0.001f;
            }
            if (Chassis_Drive.Get_Mode() == CHASSIS_MODE_DISTANCE &&
                M2006_Direction[0] * M2006[0].omega < -0.5f) {
                saw_active_reverse = true;
                break;
            }
        }
        assert(saw_active_reverse);

        // Abruptly lose rear clearance via yaw jump >= 0.10 rad -> MUST cancel IMMEDIATELY to IDLE with ZERO torque!
        Heading_Rad += 0.15f;
        Step(++t, false);
        assert(Rescue_Mission.Get_Proven_Rear_Clear_M() == 0.0f);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        assert(M2006[0].torque == 0.0f && M2006[1].torque == 0.0f);
        // While wheels still spin, MUST stay IDLE with zero torque (NOT jump directly into CHASSIS_MODE_TURN!).
        Step(++t, false);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        assert(M2006[0].torque == 0.0f && M2006[1].torque == 0.0f);
        // Once wheels reach standstill, separation cleanly enters CHASSIS_MODE_TURN!
        M2006[0].omega = M2006[1].omega = 0.0f;
        Step(++t, false);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_TURN);
        std::puts("PASS: coupled active-reverse abrupt clearance loss cancels to IDLE zero-torque and confirms standstill before turn");
        return 0;
    }

    if (cold_boot) {
        // Normal cold-boot sequence: first control ticks arrive BEFORE the first ADC DMA callback.
        Feed_Config(RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION);
        Feed_Vision(1U);
        for (uint32_t t = 0U; t < 5U; ++t) {
            Step(t, false, true, /*update_battery=*/false);
            assert(!Gate_Servo.Is_Enabled() && !Pitch_Servo.Is_Enabled());
            assert(!Servos_Started && !Servos_Fault_Latched);
            Stopped();
        }
        // First valid 24 V ADC sample arrives -> both servos MUST enable cleanly and settle!
        for (uint32_t t = 5U; t <= 200U; ++t) {
            if ((t % 40U) == 0U) {
                Feed_Vision(static_cast<uint16_t>(t));
            }
            Step(t, false, true, /*update_battery=*/true);
        }
        assert(Servos_Started && !Servos_Fault_Latched);
        assert(Gate_Servo.Is_Enabled() && Pitch_Servo.Is_Enabled());
        assert(Gate_Servo.Is_Settled() && Pitch_Servo.Is_Settled());

        // Competition launch now succeeds cleanly.
        Step(210U, true);
        Step(240U, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
        assert(M2006[0].torque < 0 && M2006[1].torque > 0);

        // Actual battery undervoltage fault must disable servos and remain latched forever.
        Battery.Update(19.0f, 250U);
        now_ms = 250U;
        Drive_Control();
        Stopped();
        assert(!Gate_Servo.Is_Enabled() && !Pitch_Servo.Is_Enabled());
        assert(Servos_Fault_Latched);
        // Even if voltage bounces back, battery & servos MUST remain latched off!
        Step(260U, false, true, /*update_battery=*/true);
        assert(!Gate_Servo.Is_Enabled() && !Pitch_Servo.Is_Enabled());
        assert(Servos_Fault_Latched);
        Stopped();
        std::puts("PASS: cold-boot deferred servo PWM startup, settling, launch, and latched battery fault");
        return 0;
    }

    if (servo_fail) {
        // If either servo fails to start PWM, both servos stay disabled, FAULT_FLAG_SERVO is exposed,
        // and competition launch is blocked.
        Pitch_Servo.fail_enable = true;
        Feed_Config(RESCUE_TEAM_BLUE, 3U, RESCUE_MODE_COMPETITION);
        Feed_Vision(1U);
        Step(0U, false);
        assert(Servos_Fault_Latched && !Servos_Started);
        assert(!Gate_Servo.Is_Enabled() && !Pitch_Servo.Is_Enabled());
        assert((Telemetry_Snapshot.fault_flags & FAULT_FLAG_SERVO) != 0U);
        Step(30U, false);
        Step(40U, true);
        Step(70U, true);
        assert(Rescue_Mission.Get_State() == MISSION_PRE_MATCH);
        Stopped();
        std::puts("PASS: servo PWM start failure latches fault, exposes FAULT_FLAG_SERVO, and blocks competition launch");
        return 0;
    }

    if (can_fault) {
        Feed_Config(RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION);
        Feed_Vision(1U);
        Step(0, false); Step(30, false);
        Step(40, true); Step(70, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
        assert(M2006[0].torque < 0 && M2006[1].torque > 0);

        hfdcan1.fail = true;
        Feed_Vision(2U);
        Step(80, false);
        assert(Rescue_Mission.Get_State() == MISSION_FAULT_STOP);
        Stopped();

        hfdcan1.fail = false;
        Feed_Vision(3U);
        Step(90, false);
        assert(Rescue_Mission.Get_State() == MISSION_FAULT_STOP);
        Stopped();
        std::puts("PASS: competition CAN TX failure latches MISSION_FAULT_STOP with zero torque");
        return 0;
    }

    if (frozen) {
        Feed_Config(RESCUE_TEAM_BLUE, 2U, RESCUE_MODE_COMPETITION);
        uint16_t fseq = 1U;
        Feed_Vision(fseq++);
        Step(0, false); Step(30, false);
        Step(40, true); Step(70, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);

        uint32_t fault_at_ms = 0U;
        for (uint32_t t = 71U; t <= 12000U; ++t) {
            if ((t % 50U) == 0U) {
                Feed_Vision(fseq++);
            }
            Step(t, false);
            if (Rescue_Mission.Get_State() == MISSION_FAULT_STOP) {
                fault_at_ms = t;
                break;
            }
        }
        assert(fault_at_ms > 0U && fault_at_ms < 11000U);
        Stopped();
        std::puts("PASS: frozen wheel/IMU exhausts recovery budget and faults well under 15 s");
        return 0;
    }

    if (comp) {
        Step(0, false, true);
        Step(30, false, true);
        Step(40, true, true);
        Step(70, true, true);
        Stopped();
        assert(Rescue_Mission.Get_State() == MISSION_PRE_MATCH);

        Feed_Config(RESCUE_TEAM_BLUE, 2U, RESCUE_MODE_COMPETITION);
        Feed_Vision(1U);
        Step(80, false, true);
        Step(110, false, true);
        assert(Status_LED_Color.Blue > 200U && Status_LED_Color.Red == 0U);
        assert(Telemetry_Snapshot.team == RESCUE_TEAM_BLUE && Telemetry_Snapshot.start_zone == 2U);

        Step(120, true, true);
        Step(150, true, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
        assert(Rescue_Mission.Is_Config_Locked());
        assert(M2006[0].torque < 0 && M2006[1].torque > 0);
        Feed_Config(RESCUE_TEAM_RED, 4U, RESCUE_MODE_COMPETITION);
        Step(160, false, true);
        assert(Rescue_Mission.Get_Team() == RESCUE_TEAM_BLUE);
        Battery.Update(19.0f, 170U);
        now_ms = 170U;
        Drive_Control();
        Stopped();
        assert(!Gate_Servo.Is_Enabled() && !Pitch_Servo.Is_Enabled());
        assert(Status_LED_Color.Red == 255U && Status_LED_Color.Green == 0U && Status_LED_Color.Blue == 0U);
        std::puts("PASS: competition UART config, LED echo, launch lock, servo fault cutoff");
        return 0;
    }

    // Bench straight-drive regression mode (preserves all original interlock checks).
    Feed_Config(RESCUE_TEAM_UNSET, 0U, RESCUE_MODE_BENCH_STRAIGHT);
    Step(0, held, false);
    Step(30, held, false);
    if (held) {
        Step(100, true);
        Stopped();
        Step(110, false);
        Step(140, false);
    } else {
        Step(40, true, false);
        Step(70, true, false);
        Step(100, true);
        Stopped();
        Step(110, false);
        Step(140, false);
    }
    Step(150, true);
    Step(180, true);
    assert(Straight_Drive.Is_Running());
    assert(M2006[0].torque < 0 && M2006[1].torque > 0);
    now_ms = 211;
    M2006_Last_Feedback_Ms[0] = M2006_Last_Feedback_Ms[1] = now_ms;
    Drive_Control();
    Stopped();
    Step(220, true);
    Stopped();
    Step(230, false); Step(260, false); Step(270, true); Step(300, true);
    assert(Straight_Drive.Is_Running());
    hfdcan2.pending = true;
    Step(301, true);
    assert(!hfdcan2.pending);
    Stopped();
    Step(310, false); Step(340, false); Step(350, true); Step(380, true);
    assert(Straight_Drive.Is_Running());
    hfdcan1.fail = true;
    Step(381, true);
    assert(!Straight_Drive.Is_Running());
    hfdcan1.fail = false;
    Step(382, true);
    Stopped();
    Step(390, false); Step(420, false); Step(430, true); Step(460, true);
    assert(Straight_Drive.Is_Running());
    M2006_Feedback_Received[1] = false;
    now_ms = 461;
    Drive_Control();
    Stopped();
    Step(470, false); Step(500, false); Step(510, true); Step(540, true);
    assert(Straight_Drive.Is_Running());
    Battery.Update(19.0f, 541);
    now_ms = 541; Drive_Control();
    Stopped();
    Step(550, false); Step(580, false); Step(590, true); Step(620, true);
    Stopped();
    std::puts("PASS: early/held key, forward signs, IMU/CAN/feedback faults, low-voltage latch");
}
'''
source = out / "test_rtos_drive.cpp"
source.write_text(stub + globals_text + functions + checks, encoding="utf-8")
exe = out / ("test_rtos_drive.exe" if os.name == "nt" else "test_rtos_drive")
cmd = ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-variable",
       "-I", str(repo / "User_File"), str(source),
       str(repo / "User_File/1_Middleware/Protocol/rescue_protocol.cpp"),
       str(repo / "User_File/2_Device/Servo/dvc_servo.cpp"),
       str(repo / "User_File/3_Robot/Drive/straight_drive.cpp"),
       str(repo / "User_File/3_Robot/Drive/chassis_drive.cpp"),
       str(repo / "User_File/3_Robot/Mission/rescue_mission.cpp"),
       "-o", str(exe)]
if os.name == "nt":
    cmd.append("-static")
subprocess.run(cmd, check=True)
for mode in ([], ["held"], ["comp"], ["can_fault"], ["frozen"], ["cold_boot"], ["servo_fail"], ["reverse_abort"]):
    subprocess.run([str(exe), *mode], check=True)
