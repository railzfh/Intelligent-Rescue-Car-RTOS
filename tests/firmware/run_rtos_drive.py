"""Exercise the actual task's drive gates with mocked GPIO, CAN and motors."""
import os
from pathlib import Path
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/native-rtos"
out.mkdir(parents=True, exist_ok=True)
task = (repo / "User_File/4_Task/tsk_config_and_callback.cpp").read_text(encoding="utf-8")
globals_text = task[task.index("bool init_finished"):task.index("/* Private function declarations")]
functions = task[task.index("static bool Drive_Button_Pressed"):task.index("static void Wait_For_Next_Period")]
stub = r'''
#include "3_Robot/Drive/straight_drive.h"
#include "2_Device/Battery/dvc_battery.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
constexpr float BASIC_MATH_RPM_TO_RADPS = 3.14159265358979323846f / 30.0f;
enum Enum_Motor_DJI_ID { Motor_DJI_ID_0x201 = 1, Motor_DJI_ID_0x202 = 2 };
struct Class_Motor_DJI_C610 {
    float torque = 0;
    float Get_Now_Angle() const { return 0; }
    float Get_Now_Omega() const { return 0; }
    void Set_Target_Torque(float t) { torque = t; }
    void TIM_Calculate_PeriodElapsedCallback() {}
};
struct FDCAN_HandleTypeDef { bool pending = false, fail = false; };
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
int HAL_GPIO_ReadPin(int, int) { return key ? 0 : 1; }
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *) { return 0; }
bool HAL_FDCAN_IsTxBufferMessagePending(FDCAN_HandleTypeDef *c, uint32_t) { return c->pending; }
void HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *c, uint32_t) { c->pending = false; }
int CAN_Transmit_Data(FDCAN_HandleTypeDef *c, int, uint8_t *, int) { return c->fail ? 1 : 0; }
struct { float volume = 0; void Set_Loudness(float v) { volume = v; } } BSP_Buzzer;
'''
checks = r'''
void Step(uint32_t t, bool pressed, bool imu = true) {
    now_ms = t; key = pressed;
    Battery.Update(24.0f, t);
    M2006_Feedback_Received[0] = M2006_Feedback_Received[1] = true;
    M2006_Last_Feedback_Ms[0] = M2006_Last_Feedback_Ms[1] = t;
    Attitude_Valid = imu; Attitude_Sample_Ms = t; Imu_Quiet = true;
    Drive_Control();
    assert(irq_mask == 0);
}
void Stopped() {
    assert(!Straight_Drive.Is_Running());
    assert(M2006[0].torque == 0 && M2006[1].torque == 0);
}
int main(int argc, char **argv) {
    const bool held = argc > 1 && std::strcmp(argv[1], "held") == 0;
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
    // An old IMU sample stops immediately and does not restart on recovery.
    now_ms = 211;
    M2006_Last_Feedback_Ms[0] = M2006_Last_Feedback_Ms[1] = now_ms;
    Drive_Control();
    Stopped();
    Step(220, true);
    Stopped();
    // Only a new press may start after a fault.
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
       str(repo / "User_File/3_Robot/Drive/straight_drive.cpp"), "-o", str(exe)]
if os.name == "nt":
    cmd.append("-static")
subprocess.run(cmd, check=True)
subprocess.run([str(exe)], check=True)
subprocess.run([str(exe), "held"], check=True)
