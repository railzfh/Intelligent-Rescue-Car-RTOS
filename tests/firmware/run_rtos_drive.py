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
struct UART_HandleTypeDef {
    bool rx_healthy = true;
    bool rx_fault_latched = false;
    uint32_t rx_epoch = 0U;
    int service_rx_calls = 0;
    bool inject_fault_after_epoch_read = false;
};
UART_HandleTypeDef huart1;
bool UART_Service_Rx_Recovery(UART_HandleTypeDef *h) {
    ++h->service_rx_calls;
    return !h->rx_fault_latched;
}
bool UART_Is_Rx_Healthy(UART_HandleTypeDef *h) { return h->rx_healthy && !h->rx_fault_latched; }
bool UART_Is_Rx_Fault_Latched(UART_HandleTypeDef *h) { return h->rx_fault_latched; }
uint32_t UART_Get_Rx_Epoch(UART_HandleTypeDef *h) {
    const uint32_t sampled = h->rx_epoch;
    if (h->inject_fault_after_epoch_read) {
        h->inject_fault_after_epoch_read = false;
        h->rx_healthy = false;
        ++h->rx_epoch;
    }
    return sampled;
}
struct FDCAN_HandleTypeDef { bool pending = false, fail = false; int tx_calls = 0; };
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
uint8_t CAN1_0x200_Tx_Data[8], CAN2_0x200_Tx_Data[8];
constexpr int FDCAN_DLC_BYTES_8 = 8, HAL_OK = 0;
uint32_t now_ms = 0, irq_mask = 0;
bool key = false;
constexpr int KEY__INPUT_GPIO_Port = 0, KEY__INPUT_Pin = 0, GPIO_PIN_RESET = 0;
uint32_t HAL_GetTick() { return now_ms; }
void (*irq_after_unmask)() = nullptr;
uint32_t __get_PRIMASK() { return irq_mask; }
void __disable_irq() { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) {
    irq_mask = mask;
    if (mask == 0U && irq_after_unmask != nullptr) {
        auto fn = irq_after_unmask;
        irq_after_unmask = nullptr;
        fn();
    }
}
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
    M2006_Feedback_Seq[0] = M2006_Feedback_Seq[0] + 1U;
    M2006_Feedback_Seq[1] = M2006_Feedback_Seq[1] + 1U;
    Attitude_Valid = imu; Attitude_Sample_Ms = t; Imu_Quiet = true;
    Drive_Control();
    assert(irq_mask == 0);
    // 1 ms Drive_Control() MUST NEVER invoke blocking UART_Service_Rx_Recovery().
    assert(huart1.service_rx_calls == 0);
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
    const bool uart_unhealthy = argc > 1 && std::strcmp(argv[1], "uart_unhealthy") == 0;

    if (uart_unhealthy) {
        // Finding 6 (R5): 1 ms Drive_Control() uses non-blocking UART_Is_Rx_Healthy(),
        // NEVER calls blocking UART_Service_Rx_Recovery(), blocks pre-match launch while RX is unhealthy,
        // immediately inhibits motion during active competition when RX goes unhealthy, and latches
        // MISSION_FAULT_STOP if RX recovery permanently faults.
        Feed_Config(RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION);
        uint16_t fseq = 1U;
        uint32_t t = 0U;
        for (; t < 200U; ++t) {
            if ((t % 40U) == 0U) { Feed_Vision(fseq++); }
            Step(t, false);
        }
        assert(Servos_Started && Gate_Servo.Is_Settled() && Pitch_Servo.Is_Settled());

        // Transient RX recovery pending (not yet fault-latched) MUST block competition launch immediately.
        huart1.rx_healthy = false;
        Step(t += 10U, true);
        Step(t += 30U, true);
        assert(Rescue_Mission.Get_State() == MISSION_PRE_MATCH);
        assert((Telemetry_Snapshot.fault_flags & FAULT_FLAG_UART_RX) != 0U);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) == 0U);
        Stopped();

        // Release button, restore RX health WITHOUT a new vision frame ->
        // cached pre-fault vision MUST remain invalidated (TEL_FLAG_VISION_FRESH == 0, launch blocked)!
        Step(t += 10U, false);
        Step(t += 30U, false);
        // Queue a pre-recovery vision frame while RX is STILL unhealthy -> must be flushed on recovery!
        Feed_Vision(fseq++);
        Step(t += 5U, false);
        huart1.rx_healthy = true;
        Step(t += 5U, true);
        Step(t += 30U, true);
        assert(Rescue_Mission.Get_State() == MISSION_PRE_MATCH);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) == 0U);
        Stopped();

        // Release button and deliver a brand-new post-recovery vision frame -> launch now succeeds cleanly!
        Step(t += 10U, false);
        Step(t += 30U, false);
        Feed_Vision(fseq++);
        Step(t += 10U, true);
        Step(t += 30U, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
        assert(M2006[0].torque < 0.0f && M2006[1].torque > 0.0f);
        assert((Telemetry_Snapshot.fault_flags & FAULT_FLAG_UART_RX) == 0U);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) != 0U);

        // Sub-millisecond recovery epoch bump (rx_healthy stayed true between Drive_Control ticks!)
        // with pre-recovery bytes queued in ring buffer -> MUST flush ring, invalidate vision, and inhibit motion!
        Feed_Vision(fseq++);
        ++huart1.rx_epoch;
        Step(t += 5U, false);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) == 0U);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_STOPPING || Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        for (int k = 0; k < 100 && Chassis_Drive.Is_Active(); ++k) {
            Step(++t, false);
        }
        Stopped();

        // New post-epoch frame restores VISION_FRESH cleanly.
        Feed_Vision(fseq++);
        Step(t += 5U, false);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) != 0U);

        // RX becomes transiently unhealthy mid-run -> motion MUST inhibit immediately (even while latest frame is < 50 ms old)!
        huart1.rx_healthy = false;
        Step(t += 5U, false);
        assert((Telemetry_Snapshot.fault_flags & FAULT_FLAG_UART_RX) != 0U);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) == 0U);
        // Even after restoring rx_healthy=true with NO new frame, VISION_FRESH MUST stay 0 and motion stopped!
        huart1.rx_healthy = true;
        Step(t += 5U, false);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) == 0U);
        Stopped();

        // Permanently latched RX fault -> immediately latches MISSION_FAULT_STOP.
        huart1.rx_fault_latched = true;
        Step(t += 5U, false);
        assert(Rescue_Mission.Get_State() == MISSION_FAULT_STOP);
        assert((Telemetry_Snapshot.fault_flags & FAULT_FLAG_UART_RX) != 0U);
        Stopped();
        assert(huart1.service_rx_calls == 0);
        std::puts("PASS: non-blocking UART RX health/epoch gate flushes pre-recovery bytes, invalidates vision, blocks motion until new frame");
        return 0;
    }

    const bool standstill_can = argc > 1 && std::strcmp(argv[1], "standstill_can") == 0;
    if (standstill_can) {
        // Finding 1 (R6): Reverse stop interlock MUST NOT release on pre-stop cached CAN samples!
        // Last zero-speed CAN sample at t=6 ms -> reverse accelerates t=7..95 ms with no CAN update ->
        // clearance lost at t=96 ms -> t=97 ms (no new CAN) MUST stay IDLE (NOT TURN!) ->
        // t=98 ms (only motor 0 updated) MUST STILL stay IDLE ->
        // t=99 ms (BOTH motors deliver new standstill samples) releases cleanly to CHASSIS_MODE_TURN!
        Feed_Config(RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION);
        Struct_Vision_State vis{};
        vis.frame_seq = 1U;
        vis.camera_view = CAMERA_VIEW_FAR;
        vis.target_type = TARGET_GREEN_MATERIAL;
        vis.target_dist_mm = 190U;
        vis.flags = VIS_FLAG_TARGET_VALID;
        assert(Rescue_Mission.On_Vision_State(vis, 1U));
        Step(0U, false);
        Struct_Mission_Inputs min{};
        min.can_launch = min.hardware_ready = min.button_pressed = min.wheels_still = min.camera_settled = true;
        min.now_ms = 1U;
        Rescue_Mission.Update(min);
        min.button_pressed = false;
        min.chassis_motion_done = true;
        min.now_ms = 2U;
        Rescue_Mission.Update(min);
        min.chassis_motion_done = false;
        min.now_ms = 3U;
        Rescue_Mission.Update(min);
        min.now_ms = 4U;
        Rescue_Mission.Update(min);
        assert(Rescue_Mission.Get_State() == MISSION_INSPECT_CORRAL);
        vis.frame_seq++;
        vis.camera_view = CAMERA_VIEW_NEAR;
        vis.corral_cyan_cnt = 1U;
        assert(Rescue_Mission.On_Vision_State(vis, 5U));
        min.now_ms = 5U;
        Rescue_Mission.Update(min);
        assert(Rescue_Mission.Get_State() == MISSION_BACK_OUT_SEPARATE);
        vis.frame_seq++;
        vis.flags = VIS_FLAG_REAR_CLEAR;
        assert(Rescue_Mission.On_Vision_State(vis, 6U));
        Step(6U, false);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_DISTANCE);

        auto drive_no_can = [&](uint32_t tm) {
            now_ms = tm;
            Battery.Update(24.0f, tm);
            Attitude_Sample_Ms = tm;
            Drive_Control();
        };
        float modeled_omega = 0.0f;
        for (uint32_t tm = 7U; tm < 96U; ++tm) {
            drive_no_can(tm);
            modeled_omega += 40.0f * M2006_Direction[0] * M2006[0].torque * 0.001f;
        }
        assert(modeled_omega < -0.1f);
        // Cancel reverse at t=96 ms while CAN feedback is still cached from t=6 ms (omega == 0)!
        vis.frame_seq++;
        vis.flags = 0U;
        assert(Rescue_Mission.On_Vision_State(vis, 96U));
        drive_no_can(96U);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        assert(M2006[0].torque == 0.0f && M2006[1].torque == 0.0f);

        // Tick 97: NO new CAN sample -> MUST stay IDLE (never release on pre-stop t=6 ms sample!).
        drive_no_can(97U);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        assert(M2006[0].torque == 0.0f && M2006[1].torque == 0.0f);

        // Tick 98: ONLY motor 0 receives a fresh standstill CAN sample -> MUST STILL stay IDLE!
        M2006_Last_Feedback_Ms[0] = 98U;
        M2006_Feedback_Seq[0] = M2006_Feedback_Seq[0] + 1U;
        drive_no_can(98U);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);

        // Tick 99: Motor 1 also delivers a fresh standstill CAN sample -> now cleanly releases to CHASSIS_MODE_TURN!
        M2006_Last_Feedback_Ms[1] = 99U;
        M2006_Feedback_Seq[1] = M2006_Feedback_Seq[1] + 1U;
        drive_no_can(99U);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_TURN);
        std::puts("PASS: reverse standstill interlock rejects pre-stop CAN samples and requires fresh post-stop standstill from EACH motor");
        return 0;
    }

    const bool irq_races = argc > 1 && std::strcmp(argv[1], "irq_races") == 0;
    if (irq_races) {
        // Finding 2 (R7): UART RX fault injected immediately after pre-drain epoch read MUST suppress
        // frame commit, VISION_FRESH publication, competition launch, and motor torque in the same tick!
        Feed_Config(RESCUE_TEAM_RED, 1U, RESCUE_MODE_COMPETITION);
        Feed_Vision(1U);
        for (uint32_t tm = 0U; tm < 40U; ++tm) { Step(tm, false); }
        Step(40U, true);
        Feed_Vision(2U);
        huart1.inject_fault_after_epoch_read = true;
        Step(70U, true);
        assert(!huart1.rx_healthy);
        assert((Telemetry_Snapshot.status_flags & TEL_FLAG_VISION_FRESH) == 0U);
        assert((Telemetry_Snapshot.fault_flags & FAULT_FLAG_UART_RX) != 0U);
        assert(Rescue_Mission.Get_State() == MISSION_PRE_MATCH);
        assert(!Rescue_Mission.Is_Config_Locked());
        Stopped();

        // Finding 1 (R7): CAN zero-speed IRQs arriving AFTER Drive_Control's initial snapshot but BEFORE
        // Emergency_Stop is applied (while still reversing!) MUST NOT release the standstill interlock on the next tick!
        // Test BOTH abrupt reverse abort AND normal reverse completion!
        huart1.rx_healthy = true;
        Step(80U, false);
        Step(110U, false);
        uint16_t fseq = 10U;
        auto feed_far_r7 = [&](uint16_t s, uint8_t target, uint16_t flags, uint16_t dist_mm) {
            uint8_t buf[RESCUE_PROTO_MAX_FRAME];
            static uint8_t pseq = 20U;
            Struct_Vision_State vis{};
            vis.frame_seq = s;
            vis.camera_view = CAMERA_VIEW_FAR;
            vis.target_type = target;
            vis.flags = flags;
            vis.target_dist_mm = dist_mm;
            vis.confidence = 95U;
            const size_t n = Rescue_Encode_Vision_State(pseq++, vis, buf, sizeof(buf));
            Vision_Uart_RxCallback(buf, static_cast<uint16_t>(n));
        };
        auto feed_near_r7 = [&](uint16_t s, uint8_t g, uint8_t c, uint16_t flags) {
            uint8_t buf[RESCUE_PROTO_MAX_FRAME];
            static uint8_t pseq = 80U;
            Struct_Vision_State vis{};
            vis.frame_seq = s;
            vis.camera_view = CAMERA_VIEW_NEAR;
            vis.flags = flags;
            vis.corral_green_cnt = g;
            vis.corral_cyan_cnt = c;
            vis.confidence = 95U;
            const size_t n = Rescue_Encode_Vision_State(pseq++, vis, buf, sizeof(buf));
            Vision_Uart_RxCallback(buf, static_cast<uint16_t>(n));
        };
        uint32_t tm = 120U;
        for (; tm < 220U; ++tm) {
            if ((tm % 30U) == 0U) { feed_far_r7(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Step(tm, false);
        }
        Step(tm += 10U, true);
        Step(tm += 30U, true);
        assert(Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS);
        // Advance straight to reach SEARCH -> APPROACH -> INSPECT -> cyan -> BACK_OUT_SEPARATE.
        for (int k = 0; k < 8000 && Rescue_Mission.Get_State() == MISSION_LAUNCH_CROSS_BUMPS; ++k) {
            ++tm;
            if ((tm % 40U) == 0U) { feed_far_r7(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Step(tm, false);
            for (int i = 0; i < 2; ++i) {
                M2006[i].angle += M2006[i].omega * 0.001f;
                M2006[i].omega += 40.0f * M2006[i].torque * 0.001f;
            }
        }
        M2006[0].omega = M2006[1].omega = 0.0f;
        for (int k = 0; k < 300 && Rescue_Mission.Get_State() != MISSION_INSPECT_CORRAL; ++k) {
            ++tm;
            if ((tm % 30U) == 0U) { feed_far_r7(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Step(tm, false);
        }
        for (int k = 0; k < 600 && !Pitch_Servo.Is_Settled(); ++k) {
            ++tm;
            if ((tm % 30U) == 0U) { feed_near_r7(fseq++, 1U, 0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR); }
            Step(tm, false);
        }
        feed_near_r7(fseq++, 0U, 1U, 0U);
        Step(++tm, false);
        assert(Rescue_Mission.Get_State() == MISSION_BACK_OUT_SEPARATE);
        feed_far_r7(fseq++, TARGET_NONE, VIS_FLAG_REAR_CLEAR, 500U);
        Step(++tm, false);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_DISTANCE);

        auto drive_no_can = [&](uint32_t t_now) {
            now_ms = t_now;
            Battery.Update(24.0f, t_now);
            Attitude_Sample_Ms = t_now;
            Drive_Control();
        };
        // Case A: Abrupt reverse abort with pre-stop CAN IRQs firing right after initial snapshot while still in DISTANCE!
        for (int k = 0; k < 20; ++k) { drive_no_can(++tm); }
        feed_far_r7(fseq++, TARGET_NONE, 0U, 500U);
        Heading_Rad += 0.2f; // abrupt clearance loss!
        irq_after_unmask = [] {
            assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_DISTANCE);
            for (unsigned i = 0; i < 2U; ++i) {
                ++M2006_Feedback_Seq[i];
                M2006_Last_Feedback_Ms[i] = now_ms;
                M2006[i].omega = 0.0f;
            }
        };
        drive_no_can(++tm);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        // Next tick with NO post-stop CAN samples -> MUST stay IDLE (NOT release to CHASSIS_MODE_TURN!).
        drive_no_can(++tm);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        // Fresh post-stop standstill samples from BOTH motors -> NOW releases to CHASSIS_MODE_TURN!
        for (unsigned i = 0; i < 2U; ++i) {
            ++M2006_Feedback_Seq[i];
            M2006_Last_Feedback_Ms[i] = tm + 1U;
            M2006[i].omega = 0.0f;
        }
        drive_no_can(++tm);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_TURN);

        // Case B: Normal reverse completion (chassis_motion_done) with pre-stop CAN IRQs firing before stop applied!
        // Finish turn -> SEARCH -> APPROACH -> INSPECT -> cyan -> BACK_OUT_SEPARATE -> complete reverse distance normally!
        for (int k = 0; k < 2000 && Rescue_Mission.Get_State() == MISSION_BACK_OUT_SEPARATE; ++k) {
            ++tm;
            if ((tm % 40U) == 0U) { feed_far_r7(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Heading_Rad += 0.002f;
            Step(tm, false);
        }
        M2006[0].omega = M2006[1].omega = 0.0f;
        for (int k = 0; k < 300 && Rescue_Mission.Get_State() != MISSION_INSPECT_CORRAL; ++k) {
            ++tm;
            if ((tm % 30U) == 0U) { feed_far_r7(fseq++, TARGET_GREEN_MATERIAL, VIS_FLAG_TARGET_VALID, 190U); }
            Step(tm, false);
        }
        for (int k = 0; k < 600 && !Pitch_Servo.Is_Settled(); ++k) {
            ++tm;
            if ((tm % 30U) == 0U) { feed_near_r7(fseq++, 1U, 0U, VIS_FLAG_CORRAL_SAFE | VIS_FLAG_SWEEP_CLEAR); }
            Step(tm, false);
        }
        feed_near_r7(fseq++, 0U, 1U, 0U);
        Step(++tm, false);
        assert(Rescue_Mission.Get_State() == MISSION_BACK_OUT_SEPARATE);
        feed_far_r7(fseq++, TARGET_NONE, VIS_FLAG_REAR_CLEAR, 500U);
        Step(++tm, false);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_DISTANCE);
        // Move wheels the full reverse distance so Chassis_Drive completes normally on next tick!
        const float rev_rad = -0.25f / Drive_Wheel_Radius_M;
        M2006[0].angle += M2006_Direction[0] * rev_rad;
        M2006[1].angle += M2006_Direction[1] * rev_rad;
        M2006[0].omega = M2006[1].omega = 0.0f;
        Step(++tm, false); // Chassis_Drive hits Is_Motion_Done() on this tick; Mission handles completion on next tick!
        // On the mission completion tick, inject pre-stop CAN IRQs right after initial snapshot!
        irq_after_unmask = [] {
            for (unsigned i = 0; i < 2U; ++i) {
                ++M2006_Feedback_Seq[i];
                M2006_Last_Feedback_Ms[i] = now_ms;
                M2006[i].omega = 0.0f;
            }
        };
        drive_no_can(++tm);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        // Next tick with NO post-stop CAN sample -> MUST stay IDLE (NOT start turn)!
        drive_no_can(++tm);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_IDLE);
        // Post-stop CAN samples from BOTH motors -> cleanly starts turn!
        for (unsigned i = 0; i < 2U; ++i) {
            ++M2006_Feedback_Seq[i];
            M2006_Last_Feedback_Ms[i] = tm + 1U;
            M2006[i].omega = 0.0f;
        }
        drive_no_can(++tm);
        assert(Chassis_Drive.Get_Mode() == CHASSIS_MODE_TURN);
        std::puts("PASS: same-tick UART RX race suppressed and post-stop CAN boundary enforced for both reverse abort and normal completion");
        return 0;
    }

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
for mode in ([], ["held"], ["comp"], ["can_fault"], ["frozen"], ["cold_boot"], ["servo_fail"], ["reverse_abort"], ["uart_unhealthy"], ["standstill_can"], ["irq_races"]):
    subprocess.run([str(exe), *mode], check=True)
