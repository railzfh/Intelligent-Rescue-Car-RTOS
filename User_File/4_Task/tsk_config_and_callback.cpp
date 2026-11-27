/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 双M2006差速底盘控制、50Hz双舵机、MaixCAM串口协议与比赛任务调度
 * @version 0.2
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "cmsis_os2.h"

#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/Protocol/rescue_protocol.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "2_Device/Battery/dvc_battery.h"
#include "2_Device/IMU/BMI088/dvc_bmi088.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Servo/dvc_servo.h"
#include "3_Robot/Drive/chassis_drive.h"
#include "3_Robot/Drive/straight_drive.h"
#include "3_Robot/Mission/rescue_mission.h"

#include <algorithm>
#include <array>
#include <cmath>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

// 右轮接CAN1、ID为1，左轮接CAN2、ID为2。
static const Enum_Motor_DJI_ID M2006_ID[2] = {Motor_DJI_ID_0x201, Motor_DJI_ID_0x202};
// 车体前进时，右轮用负转速，左轮用正转速。
static const float M2006_Direction[2] = {-1.0f, 1.0f};
static const float M2006_Forward_Rpm = 50.0f; // 原装36:1减速箱输出轴转速
static const float M2006_Acceleration_Rpm_Per_S = 50.0f;
// C610驱动的扭矩输入为输出轴 Nm。0.9 Nm 约对应 5 A 电流指令。
static const float M2006_Torque_Limit = 0.9f;
static const uint32_t M2006_Feedback_Timeout_Ms = 100U;
static Class_Motor_DJI_C610 M2006[2];
static volatile uint32_t M2006_Last_Feedback_Ms[2] = {};
static volatile bool M2006_Feedback_Received[2] = {};
static volatile uint32_t M2006_Feedback_Seq[2] = {};

// 驱动轮直径（mm）。0 表示未配置，禁止起步。
static constexpr float Drive_Wheel_Diameter_Mm = 58.0f;
// 实测约 0.90 m，编码器估算约 1.00 m。
static constexpr float Drive_Odometry_Scale = 0.90f;
static constexpr float Drive_Target_Distance_M = 1.0f;
// 履带等效轮距（m），含滑移裕量，实车需在定角转向台架标定。
static constexpr float Drive_Effective_Track_Width_M = 0.22f;
// 姿态模块以传感器 Z 轴为航向轴，主控板须水平安装。俯视左转为正。
static constexpr float Drive_Yaw_Sign = 1.0f;
static constexpr float Drive_Wheel_Radius_M = Drive_Wheel_Diameter_Mm * 0.0005f * Drive_Odometry_Scale;

static const Struct_Straight_Drive_Config Straight_Drive_Config = {
    Drive_Wheel_Radius_M,
    Drive_Target_Distance_M,
    M2006_Forward_Rpm * BASIC_MATH_RPM_TO_RADPS,
    M2006_Acceleration_Rpm_Per_S * BASIC_MATH_RPM_TO_RADPS,
    M2006_Torque_Limit,
};
// 左右轮 b0 可根据各自实测的扭矩和加速度分别调整。
static Class_Straight_Drive Straight_Drive(Straight_Drive_Config, 40.0f, 40.0f);

static const Struct_Chassis_Config Chassis_Drive_Config = {
    Drive_Wheel_Radius_M,
    Drive_Effective_Track_Width_M,
    M2006_Forward_Rpm * BASIC_MATH_RPM_TO_RADPS,
    60.0f * BASIC_MATH_RPM_TO_RADPS,
    M2006_Torque_Limit,
    0.78f,
    0.35f,
    600U,
};
static Class_Chassis_Drive Chassis_Drive(Chassis_Drive_Config, 40.0f, 40.0f);

// 两个 50 Hz 三线 PWM 舵机参数（上车前须台架标定 endpoints，禁止顶机械限位）：
// - Gate_Servo  : PE9 / TIM1_CH1 (右侧可动倒L杆：抬起开框 / 放下闭框)
// - Pitch_Servo : PA0 / TIM2_CH1 (MaixCAM 单轴俯仰：抬头远视 / 低头近视)
static constexpr uint16_t Gate_Open_Us = 1000U;
static constexpr uint16_t Gate_Closed_Us = 1900U;
static constexpr uint16_t Pitch_Far_Us = 1500U;
static constexpr uint16_t Pitch_Near_Us = 1950U;
static const Struct_Servo_Config Gate_Servo_Config = {900U, 2100U, Gate_Open_Us, 2000.0f, 120U};
static const Struct_Servo_Config Pitch_Servo_Config = {900U, 2100U, Pitch_Far_Us, 1800.0f, 150U};
static Class_PWM_Servo Gate_Servo;
static Class_PWM_Servo Pitch_Servo;
static bool Servos_Started = false;
static bool Servos_Fault_Latched = false;

// 舵机与 MaixCAM 推荐由外接独立 5V BEC 供电；板载 PC15 (2A 整轨) 默认不开启。
static constexpr bool Board_Enable_DC5_Rail = false;

static Class_Rescue_Mission Rescue_Mission;
static Class_Rescue_Protocol_Parser Vision_Parser;

// 单生产者（USART1 DMA-Idle 中断）/ 单消费者（controlTask）无锁字节环缓冲。
static constexpr uint16_t Vision_Rx_Ring_Size = 256U;
static uint8_t Vision_Rx_Ring[Vision_Rx_Ring_Size] = {};
static volatile uint16_t Vision_Rx_Head = 0U;
static volatile uint16_t Vision_Rx_Tail = 0U;
static volatile uint32_t Vision_Rx_Overflow_Count = 0U;
static uint32_t Last_Vision_Rx_Overflow_Count = 0U;
static uint32_t Last_Uart_Rx_Epoch = 0U;
static bool Vision_Rx_Was_Healthy = true;
static volatile uint32_t Uart_Rx_Fault_Event_Seq = 0U;
static volatile uint32_t Uart_Rx_Fault_Reported_Seq = 0U;
static volatile uint32_t Telemetry_Snapshot_Rx_Seq = 0U;
static volatile uint32_t Can_Tx_Fault_Event_Seq = 0U;
static volatile uint32_t Can_Tx_Fault_Reported_Seq = 0U;
static volatile uint32_t Telemetry_Snapshot_Can_Seq = 0U;
static volatile bool Telemetry_Tx_In_Flight = false;
static volatile bool Telemetry_Tx_Ack_Rx = false;
static volatile bool Telemetry_Tx_Ack_Can = false;
static volatile uint32_t Telemetry_Tx_In_Flight_Rx_Seq = 0U;
static volatile uint32_t Telemetry_Tx_In_Flight_Can_Seq = 0U;
static bool Standstill_Zero_Tx_Armed[2] = {};
static uint32_t Standstill_Zero_Tx_Buffer[2] = {};
static bool Standstill_Zero_Tx_Occurred[2] = {};

static bool Drive_Prev_Odom_Valid = false;
static std::array<float, 2> Drive_Prev_Angle = {};
static float Drive_Prev_Heading_Rad = 0.0f;

static volatile bool Attitude_Valid = false;
static volatile bool Imu_Quiet = false;
static volatile float Heading_Rad = 0.0f;
static volatile float Yaw_Rate_Rad_S = 0.0f;
static volatile uint32_t Attitude_Sample_Ms = 0U;

static Class_Battery Battery;
static volatile uint8_t Battery_LED_Green = 0U;

static Struct_WS2812_Color Status_LED_Color = {255U, 0U, 0U};
static Struct_Rescue_Telemetry Telemetry_Snapshot = {};
static uint8_t Telemetry_Tx_Seq = 0U;
static uint8_t Telemetry_Tx_Buffer[RESCUE_PROTO_MAX_FRAME] = {};

/* Private function declarations ---------------------------------------------*/

static void Battery_Process_Samples(uint16_t offset)
{
    // DMA写另一半缓冲区时，平均已完成的64次采样，抑制电压与灯色抖动。
    const volatile uint16_t *samples = ADC1_Manage_Object.ADC_Data;
    uint32_t sum = 0;
    for (uint16_t i = offset; i < offset + ADC_BUFFER_SIZE / 2; i++)
    {
        const uint16_t sample = samples[i];
        if (sample == 65535U)
        {
            // ADC已饱和，无法可信地判断电池电压。
            Battery.Stop();
        }
        sum += sample;
    }
    const float average = (float)sum / (ADC_BUFFER_SIZE / 2);
    Battery.Update(BSP_Power.Convert_ADC_To_Voltage(average), HAL_GetTick());
}

static void M2006_RxCallback(uint8_t motor_index, const FDCAN_RxHeaderTypeDef &header)
{
    if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
        header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8 ||
        header.Identifier != 0x200U + M2006_ID[motor_index])
    {
        return;
    }

    // 驱动直接读取各自CAN管理对象中的Rx_Buffer。
    M2006[motor_index].CAN_RxCpltCallback();
    M2006_Last_Feedback_Ms[motor_index] = HAL_GetTick();
    M2006_Feedback_Received[motor_index] = true;
    M2006_Feedback_Seq[motor_index] = M2006_Feedback_Seq[motor_index] + 1U;
}

static void M2006_CAN1_RxCallback(FDCAN_RxHeaderTypeDef &header, uint8_t *)
{
    M2006_RxCallback(0, header);
}

static void M2006_CAN2_RxCallback(FDCAN_RxHeaderTypeDef &header, uint8_t *)
{
    M2006_RxCallback(1, header);
}

static void Vision_Uart_RxCallback(uint8_t *buffer, uint16_t length)
{
    if (buffer == nullptr || length == 0U)
    {
        return;
    }
    uint16_t head = Vision_Rx_Head;
    const uint16_t tail = Vision_Rx_Tail;
    for (uint16_t i = 0U; i < length; ++i)
    {
        const uint16_t next = static_cast<uint16_t>((head + 1U) % Vision_Rx_Ring_Size);
        if (next == tail)
        {
            // 缓冲区满时丢弃剩余字节并累计溢出计数，由协议层重同步和超时保护兜底。
            Vision_Rx_Overflow_Count = Vision_Rx_Overflow_Count + static_cast<uint32_t>(length - i);
            break;
        }
        Vision_Rx_Ring[head] = buffer[i];
        head = next;
    }
    __DMB();
    Vision_Rx_Head = head;
}

static bool Drive_Button_Pressed(uint32_t now_ms)
{
    // 上电时按住按键，必须先松开再按才能起步。
    static bool stable_pressed = true;
    static bool previous_raw = true;
    static uint32_t changed_ms = 0;
    const bool raw_pressed = HAL_GPIO_ReadPin(KEY__INPUT_GPIO_Port, KEY__INPUT_Pin) == GPIO_PIN_RESET;
    if (raw_pressed != previous_raw)
    {
        previous_raw = raw_pressed;
        changed_ms = now_ms;
    }
    if (raw_pressed == stable_pressed || now_ms - changed_ms < 25U)
    {
        return false;
    }
    stable_pressed = raw_pressed;
    return stable_pressed;
}

static bool Drive_Tx_Ready(FDCAN_HandleTypeDef &can)
{
    // 只保留一帧待发电流指令，避免断线时自动重发积压旧扭矩。
    const uint32_t previous = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&can);
    if (!HAL_FDCAN_IsTxBufferMessagePending(&can, previous))
    {
        return true;
    }
    HAL_FDCAN_AbortTxRequest(&can, previous);
    return false;
}

static void Drive_Notify_Buzzer(bool can_start, bool reject_cue, uint32_t now_ms)
{
    static bool notified_ready = false;
    static bool sounding = false;
    static uint32_t started_ms = 0U;
    static uint32_t duration_ms = 0U;

    if (!notified_ready && can_start)
    {
        notified_ready = true;
        sounding = true;
        started_ms = now_ms;
        duration_ms = 100U;
        BSP_Buzzer.Set_Loudness(0.25f);
    }
    else if (reject_cue && !sounding)
    {
        sounding = true;
        started_ms = now_ms;
        duration_ms = 220U;
        BSP_Buzzer.Set_Loudness(0.25f);
    }
    else if (sounding && (uint32_t)(now_ms - started_ms) >= duration_ms)
    {
        sounding = false;
        BSP_Buzzer.Set_Loudness(0.0f);
    }
}

static bool Vision_Sample_Rx_State(bool &healthy, bool &fault_latched, uint32_t &epoch, uint32_t &overflow_count)
{
    // Read epoch and ring-overflow counter before health/fault while caller masks IRQs so
    // validation and commit are atomic against HAL_UART_ErrorCallback and Vision_Uart_RxCallback.
    epoch = UART_Get_Rx_Epoch(&huart1);
    overflow_count = Vision_Rx_Overflow_Count;
    healthy = UART_Is_Rx_Healthy(&huart1);
    fault_latched = UART_Is_Rx_Fault_Latched(&huart1);
    return healthy && !fault_latched;
}

static bool Vision_Commit_Frame(const Struct_Rescue_Frame &frame, uint32_t now_ms,
                                uint32_t expected_epoch, uint32_t expected_overflow)
{
    Struct_Vision_Heartbeat hb{};
    Struct_Match_Config_Cmd cfg{};
    Struct_Vision_State vis{};
    Struct_Bench_Actuator_Cmd cmd{};
    bool decoded = false;
    if (frame.msg_type == RESCUE_MSG_HEARTBEAT)
    {
        decoded = Rescue_Decode_Heartbeat(frame, hb);
    }
    else if (frame.msg_type == RESCUE_MSG_SET_CONFIG)
    {
        decoded = Rescue_Decode_Set_Config(frame, cfg);
    }
    else if (frame.msg_type == RESCUE_MSG_VISION_STATE)
    {
        decoded = Rescue_Decode_Vision_State(frame, vis);
    }
    else if (frame.msg_type == RESCUE_MSG_BENCH_ACTUATOR)
    {
        decoded = Rescue_Decode_Bench_Actuator(frame, cmd);
    }
    if (!decoded)
    {
        return true;
    }

    // Atomic validation + frame commit with UART RX/error IRQs masked.
    bool commit_ok = false;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool rx_h = false;
    bool rx_f = false;
    uint32_t rx_ep = 0U;
    uint32_t rx_ov = 0U;
    if (Vision_Sample_Rx_State(rx_h, rx_f, rx_ep, rx_ov) &&
        rx_ep == expected_epoch && rx_ov == expected_overflow)
    {
        commit_ok = true;
        if (frame.msg_type == RESCUE_MSG_HEARTBEAT)
        {
            Rescue_Mission.On_Heartbeat(hb, now_ms);
        }
        else if (frame.msg_type == RESCUE_MSG_SET_CONFIG)
        {
            Rescue_Mission.On_Set_Config(cfg);
        }
        else if (frame.msg_type == RESCUE_MSG_VISION_STATE)
        {
            Rescue_Mission.On_Vision_State(vis, now_ms);
        }
        else if (frame.msg_type == RESCUE_MSG_BENCH_ACTUATOR)
        {
            Rescue_Mission.On_Bench_Actuator(cmd);
        }
    }
    else
    {
        Last_Uart_Rx_Epoch = UART_Get_Rx_Epoch(&huart1);
        Last_Vision_Rx_Overflow_Count = Vision_Rx_Overflow_Count;
        Vision_Rx_Tail = Vision_Rx_Head;
        Rescue_Mission.On_Uart_Rx_Fault();
        Uart_Rx_Fault_Event_Seq = Uart_Rx_Fault_Event_Seq + 1U;
        Vision_Rx_Was_Healthy = false;
    }
    __set_PRIMASK(mask);
    if (!commit_ok)
    {
        Vision_Parser.Reset();
        return false;
    }
    return true;
}

static bool Vision_Drain_Rx(uint32_t now_ms, uint32_t expected_epoch, uint32_t expected_overflow)
{
    uint16_t tail = Vision_Rx_Tail;
    const uint16_t head = Vision_Rx_Head;
    __DMB();
    Struct_Rescue_Frame frame;
    // Drain any complete frame already buffered inside the parser before consuming new UART bytes.
    while (Vision_Parser.Poll_Buffered_Frame(frame))
    {
        if (!Vision_Commit_Frame(frame, now_ms, expected_epoch, expected_overflow))
        {
            return false;
        }
    }
    while (tail != head)
    {
        const uint8_t byte = Vision_Rx_Ring[tail];
        tail = static_cast<uint16_t>((tail + 1U) % Vision_Rx_Ring_Size);
        if (!Vision_Parser.Feed_Byte(byte, frame))
        {
            continue;
        }
        do
        {
            if (!Vision_Commit_Frame(frame, now_ms, expected_epoch, expected_overflow))
            {
                return false;
            }
        } while (Vision_Parser.Poll_Buffered_Frame(frame));
    }
    bool tail_ok = false;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool rx_h = false;
    bool rx_f = false;
    uint32_t rx_ep = 0U;
    uint32_t rx_ov = 0U;
    if (Vision_Sample_Rx_State(rx_h, rx_f, rx_ep, rx_ov) &&
        rx_ep == expected_epoch && rx_ov == expected_overflow)
    {
        Vision_Rx_Tail = tail;
        tail_ok = true;
    }
    else
    {
        Last_Uart_Rx_Epoch = UART_Get_Rx_Epoch(&huart1);
        Last_Vision_Rx_Overflow_Count = Vision_Rx_Overflow_Count;
        Vision_Rx_Tail = Vision_Rx_Head;
        Rescue_Mission.On_Uart_Rx_Fault();
        Uart_Rx_Fault_Event_Seq = Uart_Rx_Fault_Event_Seq + 1U;
        Vision_Rx_Was_Healthy = false;
    }
    __set_PRIMASK(mask);
    if (!tail_ok)
    {
        Vision_Parser.Reset();
        return false;
    }
    return true;
}

static Struct_WS2812_Color Compute_Status_LED(bool battery_ready, bool hardware_ready, uint32_t now_ms)
{
    // 电池低压/采样故障优先级最高，恒亮纯红。
    if (!battery_ready || Rescue_Mission.Get_State() == MISSION_FAULT_STOP)
    {
        return {255U, 0U, 0U};
    }
    if (Rescue_Mission.Get_State() == MISSION_ESCAPE_RECOVERY)
    {
        return {220U, 180U, 0U};
    }

    const uint8_t team = Rescue_Mission.Get_Team();
    const bool pulse_dim = !hardware_ready && (((now_ms / 250U) & 1U) != 0U);
    if (team == RESCUE_TEAM_RED)
    {
        return pulse_dim ? Struct_WS2812_Color{60U, 0U, 10U} : Struct_WS2812_Color{255U, 0U, 30U};
    }
    if (team == RESCUE_TEAM_BLUE)
    {
        return pulse_dim ? Struct_WS2812_Color{0U, 10U, 60U} : Struct_WS2812_Color{0U, 40U, 255U};
    }

    const uint8_t green = Battery_LED_Green;
    return {static_cast<uint8_t>(255U - green), green, 0U};
}

static int16_t Wrap_Heading_Cdeg(float heading_rad)
{
    if (!std::isfinite(heading_rad))
    {
        return 0;
    }
    float deg = heading_rad * (180.0f / 3.14159265358979323846f);
    deg = std::fmod(deg + 180.0f, 360.0f);
    if (deg < 0.0f)
    {
        deg += 360.0f;
    }
    deg -= 180.0f;
    return static_cast<int16_t>(std::clamp(std::lround(deg * 100.0f), -18000L, 18000L));
}

static void Drive_Control()
{
    std::array<float, 2> angle;
    std::array<float, 2> speed;
    std::array<uint32_t, 2> fb_seq = {};
    std::array<uint32_t, 2> fb_ms = {};
    bool feedback_fresh = true;
    bool latch_standstill_now = false;
    std::array<uint32_t, 2> latch_seq = {};
    std::array<uint32_t, 2> latch_ms = {};
    // CAN 中断可以抢占该任务，确保同一轮转角、速度、时间戳与待确认零扭矩发完边界来自同一快照。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    for (uint8_t i = 0; i < 2; ++i)
    {
        angle[i] = M2006_Direction[i] * M2006[i].Get_Now_Angle();
        speed[i] = M2006_Direction[i] * M2006[i].Get_Now_Omega();
        fb_seq[i] = M2006_Feedback_Seq[i];
        fb_ms[i] = M2006_Last_Feedback_Ms[i];
        feedback_fresh &= M2006_Feedback_Received[i] &&
                          (uint32_t)(now_ms - fb_ms[i]) < M2006_Feedback_Timeout_Ms;
        if (Standstill_Zero_Tx_Armed[i] && !Standstill_Zero_Tx_Occurred[i])
        {
            FDCAN_HandleTypeDef &can = (i == 0U) ? hfdcan1 : hfdcan2;
            if (CAN_Is_Tx_Buffer_Occurred(&can, Standstill_Zero_Tx_Buffer[i]))
            {
                Standstill_Zero_Tx_Occurred[i] = true;
            }
            else if (!HAL_FDCAN_IsTxBufferMessagePending(&can, Standstill_Zero_Tx_Buffer[i]))
            {
                // Request cleared without TXBTO (cancelled/reset) -> never transmitted!
                Standstill_Zero_Tx_Armed[i] = false;
            }
        }
    }
    if (CAN_Is_Bus_Off_Latched(&hfdcan1) || CAN_Is_Bus_Off_Latched(&hfdcan2))
    {
        // 总线离线会经 HAL_FDCAN_Stop 清空发送请求，立刻失效待确认停稳证明。
        Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
        Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
    }
    else if (Standstill_Zero_Tx_Armed[0] && Standstill_Zero_Tx_Occurred[0] &&
             Standstill_Zero_Tx_Armed[1] && Standstill_Zero_Tx_Occurred[1] &&
             CAN_Is_Rx_Fifo0_Empty(&hfdcan1) && CAN_Is_Rx_Fifo0_Empty(&hfdcan2))
    {
        latch_standstill_now = true;
        latch_seq = fb_seq;
        latch_ms = fb_ms;
        Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
        Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
    }
    const bool attitude_fresh = Attitude_Valid && (uint32_t)(now_ms - Attitude_Sample_Ms) < 30U;
    const bool imu_quiet = Imu_Quiet;
    const float heading = Heading_Rad;
    const float yaw_rate = Yaw_Rate_Rad_S;
    bool battery_ready = Battery.Can_Run();
    bool battery_fault = Battery.Is_Fault_Latched();
    float pack_voltage = Battery.Get_Pack_Voltage();
    __set_PRIMASK(primask);

    if (latch_standstill_now)
    {
        Rescue_Mission.Latch_Standstill_Boundary(latch_seq.data(), latch_ms.data());
    }
    else if (!Rescue_Mission.Is_Standstill_Gate_Pending_Boundary())
    {
        Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
        Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
    }

    // 1 ms 控制任务仅做非阻塞 RX 健康与纪元检查，绝不阻塞调用 HAL DMA 中止/重初始化；
    // 瞬态 RX 恢复或环缓冲溢出仅失效视觉并抑制运动，留待新帧安全重评估，同时保留诊断位直至 50 ms 遥测发出；
    // 仅永久 DMA 故障锁存（uart_rx_perm_fault）才触发硬故障锁停。
    bool uart_rx_healthy = false;
    bool uart_rx_perm_fault = false;
    uint32_t uart_rx_epoch = 0U;
    uint32_t uart_rx_overflow = 0U;
    bool need_drain = false;
    bool need_parser_reset = false;
    {
        const uint32_t rx_mask = __get_PRIMASK();
        __disable_irq();
        Vision_Sample_Rx_State(uart_rx_healthy, uart_rx_perm_fault, uart_rx_epoch, uart_rx_overflow);
        const bool overflow_occurred = (uart_rx_overflow != Last_Vision_Rx_Overflow_Count);
        if (!uart_rx_healthy || uart_rx_perm_fault || uart_rx_epoch != Last_Uart_Rx_Epoch || overflow_occurred)
        {
            Last_Uart_Rx_Epoch = UART_Get_Rx_Epoch(&huart1);
            Last_Vision_Rx_Overflow_Count = Vision_Rx_Overflow_Count;
            Vision_Rx_Tail = Vision_Rx_Head;
            Rescue_Mission.On_Uart_Rx_Fault();
            Uart_Rx_Fault_Event_Seq = Uart_Rx_Fault_Event_Seq + 1U;
            Vision_Rx_Was_Healthy = uart_rx_healthy && !uart_rx_perm_fault && !overflow_occurred;
            uart_rx_healthy = false;
            need_parser_reset = true;
        }
        else
        {
            if (!Vision_Rx_Was_Healthy)
            {
                Vision_Rx_Tail = Vision_Rx_Head;
                Vision_Rx_Was_Healthy = true;
                need_parser_reset = true;
            }
            need_drain = true;
        }
        __set_PRIMASK(rx_mask);
    }
    if (need_parser_reset)
    {
        Vision_Parser.Reset();
    }
    if (need_drain && !Vision_Drain_Rx(now_ms, uart_rx_epoch, uart_rx_overflow))
    {
        uart_rx_healthy = false;
        uart_rx_perm_fault = UART_Is_Rx_Fault_Latched(&huart1);
    }

    // 舵机控制：等待首帧有效电池采样后再启动 PWM；真实电池故障或 PWM 启动失败则永久锁停。
    if (battery_fault || Servos_Fault_Latched)
    {
        Servos_Fault_Latched = true;
        Gate_Servo.Disable();
        Pitch_Servo.Disable();
    }
    else if (battery_ready)
    {
        if (!Servos_Started)
        {
            const bool gate_ok = Gate_Servo.Enable();
            const bool pitch_ok = Pitch_Servo.Enable();
            if (gate_ok && pitch_ok)
            {
                Servos_Started = true;
            }
            else
            {
                Servos_Fault_Latched = true;
                Gate_Servo.Disable();
                Pitch_Servo.Disable();
            }
        }
    }
    else
    {
        // 冷启动尚未收到首帧有效 ADC 采样：保持 PWM 关闭，但不锁死故障。
        Gate_Servo.Disable();
        Pitch_Servo.Disable();
    }

    const bool wheels_still = feedback_fresh && std::fabs(speed[0]) < 0.1f && std::fabs(speed[1]) < 0.1f;
    const bool can1_ready = Drive_Tx_Ready(hfdcan1);
    const bool can2_ready = Drive_Tx_Ready(hfdcan2);
    const bool button_pressed = Drive_Button_Pressed(now_ms);

    float forward_delta_m = 0.0f;
    float yaw_delta_rad = 0.0f;
    if (feedback_fresh && attitude_fresh)
    {
        if (Drive_Prev_Odom_Valid)
        {
            forward_delta_m = 0.5f * Drive_Wheel_Radius_M *
                              ((angle[0] - Drive_Prev_Angle[0]) + (angle[1] - Drive_Prev_Angle[1]));
            yaw_delta_rad = heading - Drive_Prev_Heading_Rad;
        }
        Drive_Prev_Angle = angle;
        Drive_Prev_Heading_Rad = heading;
        Drive_Prev_Odom_Valid = true;
    }
    else
    {
        Drive_Prev_Odom_Valid = false;
    }

    // Atomic critical section (~3 us on 550 MHz H7) pairing final RX generation validation with
    // mission update, servo/chassis/M2006 torque calculation, BOTH FDCAN submissions, standstill
    // TXBTO+FIFO boundary arming, and telemetry/LED publication so no UART error ISR can ever run
    // between validation, match lock, CAN enqueue, and telemetry publication.
    bool is_bench_mode = false;
    bool ready = false;
    bool can_launch_commit = false;
    bool reset_parser_after_commit = false;
    Struct_Mission_Output cmd{};
    std::array<float, 2> torque = {};
    {
        const uint32_t commit_mask = __get_PRIMASK();
        __disable_irq();
        // 在最终原子提交临界区内使用当前最新滴答重新校验电池、CAN 硬件/中断 Bus-Off 与 UART RX 状态，
        // 防止采样后推进的 SysTick 或 ADC/CAN/UART 中断造成误超时或与发车/扭矩/供电/遥测提交发生竞态。
        const uint32_t commit_now_ms = HAL_GetTick();
        Battery.Check_Timeout(commit_now_ms);
        battery_ready = Battery.Can_Run();
        battery_fault = Battery.Is_Fault_Latched();
        pack_voltage = Battery.Get_Pack_Voltage();
        Battery_LED_Green = Battery.Get_LED_Green();
        BSP_Power.Set_DC24_0(battery_ready);
        BSP_Power.Set_DC24_1(battery_ready);
        BSP_Power.Set_DC5(battery_ready && Board_Enable_DC5_Rail);
        if (battery_fault || Servos_Fault_Latched)
        {
            Servos_Fault_Latched = true;
            Gate_Servo.Disable();
            Pitch_Servo.Disable();
        }
        else if (!battery_ready)
        {
            Gate_Servo.Disable();
            Pitch_Servo.Disable();
        }
        const bool servos_ready_commit =
            Servos_Started && !Servos_Fault_Latched && Gate_Servo.Is_Enabled() && Pitch_Servo.Is_Enabled();

        bool can1_bus_off = CAN_Consume_Bus_Off(&hfdcan1);
        bool can2_bus_off = CAN_Consume_Bus_Off(&hfdcan2);
        bool can_bus_off = can1_bus_off || can2_bus_off;
        if (can_bus_off)
        {
            Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
            Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
            Can_Tx_Fault_Event_Seq = Can_Tx_Fault_Event_Seq + 1U;
        }

        bool rx_ok_now = false;
        bool rx_perm_fault_now = false;
        uint32_t rx_epoch_now = 0U;
        uint32_t rx_overflow_now = 0U;
        Vision_Sample_Rx_State(rx_ok_now, rx_perm_fault_now, rx_epoch_now, rx_overflow_now);
        const bool rx_overflow_changed = (rx_overflow_now != Last_Vision_Rx_Overflow_Count);
        if (!rx_ok_now || rx_perm_fault_now || rx_epoch_now != Last_Uart_Rx_Epoch || rx_overflow_changed)
        {
            Last_Uart_Rx_Epoch = UART_Get_Rx_Epoch(&huart1);
            Last_Vision_Rx_Overflow_Count = Vision_Rx_Overflow_Count;
            Vision_Rx_Tail = Vision_Rx_Head;
            Rescue_Mission.On_Uart_Rx_Fault();
            Uart_Rx_Fault_Event_Seq = Uart_Rx_Fault_Event_Seq + 1U;
            uart_rx_healthy = false;
            uart_rx_perm_fault = uart_rx_perm_fault || rx_perm_fault_now;
            Vision_Rx_Was_Healthy = rx_ok_now && !rx_perm_fault_now && !rx_overflow_changed;
            reset_parser_after_commit = true;
        }

        is_bench_mode = (Rescue_Mission.Get_Run_Mode() == RESCUE_MODE_BENCH_STRAIGHT);
        ready = battery_ready && feedback_fresh && attitude_fresh &&
                can1_ready && can2_ready && !can_bus_off &&
                (is_bench_mode || (servos_ready_commit && !uart_rx_perm_fault));
        const bool can_start = ready && wheels_still && imu_quiet;
        can_launch_commit = can_start && (is_bench_mode || uart_rx_healthy);

        const uint16_t desired_gate_us = Rescue_Mission.Is_Gate_Closed_Cmd() ? Gate_Closed_Us : Gate_Open_Us;
        const uint16_t desired_pitch_us = Rescue_Mission.Is_Camera_Near_Cmd() ? Pitch_Near_Us : Pitch_Far_Us;

        const Enum_Mission_State prev_state = Rescue_Mission.Get_State();
        Struct_Mission_Inputs mission_in{};
        mission_in.now_ms = now_ms;
        mission_in.hardware_ready = ready;
        mission_in.can_launch = can_launch_commit;
        mission_in.uart_rx_healthy = is_bench_mode || uart_rx_healthy;
        mission_in.button_pressed = button_pressed;
        mission_in.chassis_active = (prev_state == MISSION_BENCH_STRAIGHT)
                                        ? Straight_Drive.Is_Running()
                                        : Chassis_Drive.Is_Active();
        mission_in.wheels_still = wheels_still;
        mission_in.chassis_motion_done = Chassis_Drive.Is_Motion_Done();
        mission_in.chassis_motion_aborted = Chassis_Drive.Is_Motion_Aborted();
        mission_in.chassis_stalled = Chassis_Drive.Is_Stalled();
        mission_in.gate_settled = servos_ready_commit && Gate_Servo.Is_Settled() &&
                                  (Gate_Servo.Get_Target_Us() == desired_gate_us);
        mission_in.camera_settled = servos_ready_commit && Pitch_Servo.Is_Settled() &&
                                    (Pitch_Servo.Get_Target_Us() == desired_pitch_us);
        mission_in.forward_delta_m = forward_delta_m;
        mission_in.yaw_delta_rad = yaw_delta_rad;
        mission_in.motor_feedback_seq[0] = fb_seq[0];
        mission_in.motor_feedback_seq[1] = fb_seq[1];
        mission_in.motor_feedback_ms[0] = fb_ms[0];
        mission_in.motor_feedback_ms[1] = fb_ms[1];
        mission_in.use_motor_feedback_seq = true;

        cmd = Rescue_Mission.Update(mission_in);
        if (Rescue_Mission.Consume_Pre_Match_Rx_Fence())
        {
            // Fence any partial parser state and queued RX bytes from before open/FAR settled.
            Vision_Rx_Tail = Vision_Rx_Head;
            reset_parser_after_commit = true;
        }

        if (servos_ready_commit)
        {
            Gate_Servo.Set_Target_Us(cmd.gate_closed ? Gate_Closed_Us : Gate_Open_Us);
            Pitch_Servo.Set_Target_Us(cmd.camera_near ? Pitch_Near_Us : Pitch_Far_Us);
            Gate_Servo.Update_1ms();
            Pitch_Servo.Update_1ms();
        }

        if (!ready)
        {
            Straight_Drive.Stop();
            Chassis_Drive.Emergency_Stop();
        }
        else if (cmd.drive_action == DRIVE_ACTION_BENCH_STRAIGHT)
        {
            Chassis_Drive.Emergency_Stop();
            if (prev_state == MISSION_PRE_MATCH && !Straight_Drive.Is_Running())
            {
                Straight_Drive.Start(angle, speed, heading, now_ms);
            }
            else if (button_pressed && Straight_Drive.Is_Running())
            {
                Straight_Drive.Request_Stop(now_ms);
            }
            torque = Straight_Drive.Update(angle, speed, heading, yaw_rate, now_ms);
        }
        else
        {
            Straight_Drive.Stop();
            switch (cmd.drive_action)
            {
            case DRIVE_ACTION_VELOCITY:
                Chassis_Drive.Set_Velocity(cmd.linear_mps, cmd.angular_rad_s, speed, now_ms);
                break;
            case DRIVE_ACTION_START_DISTANCE:
                Chassis_Drive.Start_Distance(cmd.distance_m, cmd.speed_mps, angle, speed, heading,
                                             now_ms, cmd.motion_timeout_ms);
                break;
            case DRIVE_ACTION_START_TURN:
                Chassis_Drive.Start_Turn(cmd.delta_yaw_rad, cmd.angular_rad_s, speed, heading,
                                         now_ms, cmd.motion_timeout_ms);
                break;
            case DRIVE_ACTION_SOFT_STOP:
                Chassis_Drive.Request_Stop(now_ms);
                break;
            case DRIVE_ACTION_EMERGENCY_STOP:
            case DRIVE_ACTION_IDLE:
                Chassis_Drive.Emergency_Stop();
                break;
            case DRIVE_ACTION_KEEP:
            default:
                break;
            }
            torque = Chassis_Drive.Update(angle, speed, heading, yaw_rate, now_ms);
        }

        for (uint8_t i = 0; i < 2; ++i)
        {
            M2006[i].Set_Target_Torque(M2006_Direction[i] * torque[i]);
            M2006[i].TIM_Calculate_PeriodElapsedCallback();
        }

        bool can1_bus_off_now = can1_bus_off || CAN_Consume_Bus_Off(&hfdcan1);
        bool can2_bus_off_now = can2_bus_off || CAN_Consume_Bus_Off(&hfdcan2);
        bool can1_submitted = false;
        bool can2_submitted = false;
        uint8_t can1_status = HAL_OK;
        uint8_t can2_status = HAL_OK;

        if (!can1_bus_off_now && !can2_bus_off_now)
        {
            if (can1_ready)
            {
                can1_status = CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
                can1_submitted = (can1_status == HAL_OK);
            }
            // Recheck BOTH buses between CAN1 and CAN2 submissions (masking CPU IRQs does not freeze FDCAN hardware).
            can1_bus_off_now = can1_bus_off_now || CAN_Consume_Bus_Off(&hfdcan1);
            can2_bus_off_now = can2_bus_off_now || CAN_Consume_Bus_Off(&hfdcan2);
            if (!can1_bus_off_now && !can2_bus_off_now && can1_status == HAL_OK && can2_ready)
            {
                can2_status = CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
                can2_submitted = (can2_status == HAL_OK);
            }
        }
        else if (can_bus_off)
        {
            // Already bus-off BEFORE torque calculation (torque is guaranteed 0.0f); send zero on any healthy ready bus.
            // If Bus-Off was FIRST detected at the pre-TX recheck above, TX buffers still hold nonzero torque:
            // skip transmitting here so the late_bus_off fault block regenerates zero-torque payloads first!
            if (can1_ready && !can1_bus_off_now)
            {
                can1_status = CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
                can1_submitted = (can1_status == HAL_OK);
            }
            if (can2_ready && !can2_bus_off_now)
            {
                can2_status = CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
                can2_submitted = (can2_status == HAL_OK);
            }
        }

        // Recheck BOTH buses immediately after submissions to catch a Bus-Off raised during the second TX request.
        can1_bus_off_now = can1_bus_off_now || CAN_Consume_Bus_Off(&hfdcan1);
        can2_bus_off_now = can2_bus_off_now || CAN_Consume_Bus_Off(&hfdcan2);
        const bool late_bus_off = !can_bus_off && (can1_bus_off_now || can2_bus_off_now);
        can1_bus_off = can1_bus_off_now;
        can2_bus_off = can2_bus_off_now;
        can_bus_off = can1_bus_off || can2_bus_off;
        const bool can_tx_ok = (can1_status == HAL_OK && can2_status == HAL_OK);
        if (!can_tx_ok || late_bus_off)
        {
            // CAN 发送失败或提交窗口内发生 Bus-Off：立即取消在途请求、撤销未提交发车或锁停任务，并在健康总线上补发零扭矩。
            Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
            Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
            Can_Tx_Fault_Event_Seq = Can_Tx_Fault_Event_Seq + 1U;
            ready = false;
            can_launch_commit = false;
            Straight_Drive.Stop();
            Chassis_Drive.Emergency_Stop();

            const uint32_t pending_buf1 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            if (pending_buf1 != 0U && HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, pending_buf1))
            {
                HAL_FDCAN_AbortTxRequest(&hfdcan1, pending_buf1);
            }
            const uint32_t pending_buf2 = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan2);
            if (pending_buf2 != 0U && HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, pending_buf2))
            {
                HAL_FDCAN_AbortTxRequest(&hfdcan2, pending_buf2);
            }

            const bool nonzero_submitted_to_healthy =
                (torque[0] != 0.0f && can1_submitted && !can1_bus_off) ||
                (torque[1] != 0.0f && can2_submitted && !can2_bus_off);
            if (prev_state == MISSION_PRE_MATCH && Rescue_Mission.Get_State() != MISSION_PRE_MATCH)
            {
                Rescue_Mission.Abort_Failed_Launch(now_ms, nonzero_submitted_to_healthy);
            }
            else
            {
                Rescue_Mission.Fault_Stop(now_ms);
            }
            torque = {0.0f, 0.0f};
            for (uint8_t i = 0; i < 2; ++i)
            {
                M2006[i].Set_Target_Torque(0.0f);
                M2006[i].TIM_Calculate_PeriodElapsedCallback();
            }
            const bool can1_zero_ok = !can1_bus_off && Drive_Tx_Ready(hfdcan1);
            const bool can2_zero_ok = !can2_bus_off && Drive_Tx_Ready(hfdcan2);
            if (can1_status == HAL_OK && can1_zero_ok)
            {
                CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
            }
            if (can2_status == HAL_OK && can2_zero_ok)
            {
                CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
            }
        }

        // Keep the reverse standstill gate closed until BOTH specific zero-torque CAN commands
        // positively complete transmission (TXBTO == 1, TXBRP == 0, TXBCF == 0) AND BOTH hardware
        // RX FIFO0 queues are completely empty, so cancelled TXs, bus-off restarts, or buffered
        // pre-completion RX frames (even with a deferred RX IRQ) can NEVER release the standstill interlock!
        if (!can_bus_off && can_tx_ok && can1_ready && can2_ready &&
            torque[0] == 0.0f && torque[1] == 0.0f &&
            Rescue_Mission.Is_Standstill_Gate_Pending_Boundary())
        {
            for (uint8_t i = 0; i < 2; ++i)
            {
                FDCAN_HandleTypeDef &can = (i == 0U) ? hfdcan1 : hfdcan2;
                if (!Standstill_Zero_Tx_Armed[i])
                {
                    const uint32_t buf = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&can);
                    if (buf != 0U)
                    {
                        Standstill_Zero_Tx_Buffer[i] = buf;
                        Standstill_Zero_Tx_Armed[i] = true;
                        Standstill_Zero_Tx_Occurred[i] = false;
                    }
                }
                if (Standstill_Zero_Tx_Armed[i] && !Standstill_Zero_Tx_Occurred[i])
                {
                    if (CAN_Is_Tx_Buffer_Occurred(&can, Standstill_Zero_Tx_Buffer[i]))
                    {
                        Standstill_Zero_Tx_Occurred[i] = true;
                    }
                    else if (!HAL_FDCAN_IsTxBufferMessagePending(&can, Standstill_Zero_Tx_Buffer[i]))
                    {
                        Standstill_Zero_Tx_Armed[i] = false;
                    }
                }
            }
            if (Standstill_Zero_Tx_Armed[0] && Standstill_Zero_Tx_Occurred[0] &&
                Standstill_Zero_Tx_Armed[1] && Standstill_Zero_Tx_Occurred[1] &&
                CAN_Is_Rx_Fifo0_Empty(&hfdcan1) && CAN_Is_Rx_Fifo0_Empty(&hfdcan2))
            {
                const uint32_t post_stop_seq[2] = {M2006_Feedback_Seq[0], M2006_Feedback_Seq[1]};
                const uint32_t post_stop_ms[2] = {M2006_Last_Feedback_Ms[0], M2006_Last_Feedback_Ms[1]};
                Rescue_Mission.Latch_Standstill_Boundary(post_stop_seq, post_stop_ms);
                Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
                Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
            }
        }
        else if (!Rescue_Mission.Is_Standstill_Gate_Pending_Boundary())
        {
            Standstill_Zero_Tx_Armed[0] = Standstill_Zero_Tx_Armed[1] = false;
            Standstill_Zero_Tx_Occurred[0] = Standstill_Zero_Tx_Occurred[1] = false;
        }

        // 发布灯光颜色与遥测快照，与本周期决策及 CAN 入队保持同一原子快照。
        Struct_Rescue_Telemetry tel{};
        tel.uptime_ms = now_ms;
        tel.match_elapsed_ms = Rescue_Mission.Get_Match_Elapsed_Ms(now_ms);
        tel.mission_state = static_cast<uint8_t>(Rescue_Mission.Get_State());
        tel.team = Rescue_Mission.Get_Team();
        tel.start_zone = Rescue_Mission.Get_Start_Zone();
        tel.run_mode = Rescue_Mission.Get_Run_Mode();

        const bool pub_rx_ok = uart_rx_healthy && UART_Is_Rx_Healthy(&huart1) &&
                               !UART_Is_Rx_Fault_Latched(&huart1) &&
                               (Vision_Rx_Overflow_Count == Last_Vision_Rx_Overflow_Count);
        const uint16_t pub_gate_us = Rescue_Mission.Is_Gate_Closed_Cmd() ? Gate_Closed_Us : Gate_Open_Us;
        const uint16_t pub_pitch_us = Rescue_Mission.Is_Camera_Near_Cmd() ? Pitch_Near_Us : Pitch_Far_Us;
        uint16_t status_flags = 0U;
        if (Rescue_Mission.Is_Config_Locked()) { status_flags |= TEL_FLAG_CONFIG_LOCKED; }
        if (battery_ready) { status_flags |= TEL_FLAG_BATTERY_OK; }
        if (attitude_fresh) { status_flags |= TEL_FLAG_IMU_OK; }
        if (feedback_fresh) { status_flags |= TEL_FLAG_MOTORS_OK; }
        if (Rescue_Mission.Is_Vision_Fresh(now_ms) && pub_rx_ok) { status_flags |= TEL_FLAG_VISION_FRESH; }
        if (Rescue_Mission.Is_First_Green_Unlocked()) { status_flags |= TEL_FLAG_FIRST_GREEN_UNLOCKED; }
        if (Rescue_Mission.Is_Gate_Closed_Cmd()) { status_flags |= TEL_FLAG_GATE_CLOSED; }
        if (servos_ready_commit && Gate_Servo.Is_Settled() && Gate_Servo.Get_Target_Us() == pub_gate_us)
        {
            status_flags |= TEL_FLAG_GATE_SETTLED;
        }
        if (Rescue_Mission.Is_Camera_Near_Cmd()) { status_flags |= TEL_FLAG_CAMERA_NEAR; }
        if (servos_ready_commit && Pitch_Servo.Is_Settled() && Pitch_Servo.Get_Target_Us() == pub_pitch_us)
        {
            status_flags |= TEL_FLAG_CAMERA_SETTLED;
        }
        if (Chassis_Drive.Is_Stalled()) { status_flags |= TEL_FLAG_STALLED; }
        if (Rescue_Mission.Is_Link_Alive(now_ms)) { status_flags |= TEL_FLAG_LINK_ALIVE; }
        if (Rescue_Mission.Is_Carrying_Casualty()) { status_flags |= TEL_FLAG_CARRYING_CASUALTY; }
        if (Rescue_Mission.Is_Drop_Pushing()) { status_flags |= TEL_FLAG_DROP_PUSHING; }
        uint8_t active_settle_gen = 0U;
        if (Rescue_Mission.Get_State() == MISSION_PRE_MATCH &&
            !Rescue_Mission.Is_Gate_Closed_Cmd() && !Rescue_Mission.Is_Camera_Near_Cmd() &&
            (status_flags & TEL_FLAG_GATE_SETTLED) != 0U &&
            (status_flags & TEL_FLAG_CAMERA_SETTLED) != 0U)
        {
            active_settle_gen = Rescue_Mission.Get_Pre_Match_Settle_Gen();
        }
        else if (Rescue_Mission.Get_State() == MISSION_CLOSE_GATE_VERIFY &&
                 Rescue_Mission.Is_Gate_Closed_Cmd() && Rescue_Mission.Is_Camera_Near_Cmd() &&
                 (status_flags & TEL_FLAG_GATE_SETTLED) != 0U &&
                 (status_flags & TEL_FLAG_CAMERA_SETTLED) != 0U)
        {
            active_settle_gen = Rescue_Mission.Get_Gate_Close_Settle_Gen();
        }
        else if (Rescue_Mission.Get_State() == MISSION_DROP_OPEN_AND_PUSH &&
                 !Rescue_Mission.Is_Drop_Pushing() &&
                 !Rescue_Mission.Is_Gate_Closed_Cmd() && Rescue_Mission.Is_Camera_Near_Cmd() &&
                 (status_flags & TEL_FLAG_GATE_SETTLED) != 0U &&
                 (status_flags & TEL_FLAG_CAMERA_SETTLED) != 0U)
        {
            active_settle_gen = Rescue_Mission.Get_Gate_Open_Settle_Gen();
        }
        status_flags |= static_cast<uint16_t>((active_settle_gen & 0x3U) << TEL_FLAG_SETTLE_GEN_SHIFT);
        tel.status_flags = status_flags;

        const bool rx_diag_pending = (Uart_Rx_Fault_Event_Seq != Uart_Rx_Fault_Reported_Seq);
        const bool can_diag_pending = (Can_Tx_Fault_Event_Seq != Can_Tx_Fault_Reported_Seq);
        uint16_t fault_flags = 0U;
        if (!battery_ready) { fault_flags |= FAULT_FLAG_BATTERY; }
        if (!feedback_fresh) { fault_flags |= FAULT_FLAG_MOTOR_TIMEOUT; }
        if (!attitude_fresh) { fault_flags |= FAULT_FLAG_IMU_TIMEOUT; }
        if (!can1_ready || !can2_ready || !can_tx_ok || can_bus_off || can_diag_pending)
        {
            fault_flags |= FAULT_FLAG_CAN_TX;
        }
        if (!Rescue_Mission.Is_Vision_Fresh(now_ms)) { fault_flags |= FAULT_FLAG_VISION_TIMEOUT; }
        if (Servos_Fault_Latched) { fault_flags |= FAULT_FLAG_SERVO; }
        if (uart_rx_perm_fault || !pub_rx_ok || rx_diag_pending) { fault_flags |= FAULT_FLAG_UART_RX; }
        tel.fault_flags = fault_flags;

        tel.battery_mv = (pack_voltage > 0.0f && std::isfinite(pack_voltage))
                             ? static_cast<uint16_t>(std::min(65535.0f, pack_voltage * 1000.0f))
                             : 0U;
        tel.yaw_cdeg = Wrap_Heading_Cdeg(heading);
        const float lin_mm_s = Chassis_Drive.Get_Linear_Speed_Mps(speed) * 1000.0f;
        const float ang_mrad_s = Chassis_Drive.Get_Angular_Speed_Rad_S(speed) * 1000.0f;
        tel.vx_mm_s = static_cast<int16_t>(std::clamp(std::lround(lin_mm_s), -32000L, 32000L));
        tel.omega_mrad_s = static_cast<int16_t>(std::clamp(std::lround(ang_mrad_s), -32000L, 32000L));
        tel.delivered_green = Rescue_Mission.Get_Delivered_Green();
        tel.delivered_black = Rescue_Mission.Get_Delivered_Black();
        tel.delivered_orange = Rescue_Mission.Get_Delivered_Orange();
        tel.max_materials_per_trip = Rescue_Mission.Get_Max_Materials_Per_Trip();

        const Struct_WS2812_Color led =
            Compute_Status_LED(battery_ready, ready && (is_bench_mode || pub_rx_ok), now_ms);
        Status_LED_Color = led;
        Telemetry_Snapshot = tel;
        Telemetry_Snapshot_Rx_Seq = Uart_Rx_Fault_Event_Seq;
        Telemetry_Snapshot_Can_Seq = Can_Tx_Fault_Event_Seq;
        __set_PRIMASK(commit_mask);
    }
    if (reset_parser_after_commit)
    {
        Vision_Parser.Reset();
    }
    Drive_Notify_Buzzer(can_launch_commit, cmd.launch_rejected_cue, now_ms);
}

static void Telemetry_Uart_Tx_Callback(bool completed)
{
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (Telemetry_Tx_In_Flight)
    {
        if (completed)
        {
            if (Telemetry_Tx_Ack_Rx)
            {
                Uart_Rx_Fault_Reported_Seq = Telemetry_Tx_In_Flight_Rx_Seq;
            }
            if (Telemetry_Tx_Ack_Can)
            {
                Can_Tx_Fault_Reported_Seq = Telemetry_Tx_In_Flight_Can_Seq;
            }
        }
        Telemetry_Tx_In_Flight = false;
        Telemetry_Tx_Ack_Rx = false;
        Telemetry_Tx_Ack_Can = false;
    }
    __set_PRIMASK(irq_state);
}

static bool Telemetry_Service_Tx()
{
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return false;
    }
    const uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    // 若先前在途 DMA 未走完成回调便回到 READY（例如中途错误或被中止），丢弃未完成记录且不确认诊断。
    Telemetry_Tx_In_Flight = false;
    Telemetry_Tx_Ack_Rx = false;
    Telemetry_Tx_Ack_Can = false;
    const Struct_Rescue_Telemetry tel = Telemetry_Snapshot;
    const uint32_t rx_seq_seen = Telemetry_Snapshot_Rx_Seq;
    const uint32_t can_seq_seen = Telemetry_Snapshot_Can_Seq;
    __set_PRIMASK(irq_state);

    const size_t len = Rescue_Encode_Telemetry(
        Telemetry_Tx_Seq++, tel, Telemetry_Tx_Buffer, sizeof(Telemetry_Tx_Buffer));
    if (len == 0U)
    {
        return false;
    }
    const uint32_t arm_mask = __get_PRIMASK();
    __disable_irq();
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        __set_PRIMASK(arm_mask);
        return false;
    }
    Telemetry_Tx_In_Flight = true;
    Telemetry_Tx_Ack_Rx = ((tel.fault_flags & FAULT_FLAG_UART_RX) != 0U);
    Telemetry_Tx_Ack_Can = ((tel.fault_flags & FAULT_FLAG_CAN_TX) != 0U);
    Telemetry_Tx_In_Flight_Rx_Seq = rx_seq_seen;
    Telemetry_Tx_In_Flight_Can_Seq = can_seq_seen;
    const uint8_t tx_status = UART_Transmit_Data(&huart1, Telemetry_Tx_Buffer, static_cast<uint16_t>(len));
    if (tx_status != HAL_OK)
    {
        Telemetry_Tx_In_Flight = false;
        Telemetry_Tx_Ack_Rx = false;
        Telemetry_Tx_Ack_Can = false;
    }
    __set_PRIMASK(arm_mask);
    return (tx_status == HAL_OK);
}

static void Wait_For_Next_Period(uint32_t &wake_tick, uint32_t period)
{
    wake_tick += period;
    if (osDelayUntil(wake_tick) != osOK)
    {
        // Skip missed deadlines instead of sending a burst of CAN commands.
        osDelay(1);
        wake_tick = osKernelGetTickCount();
    }
}

void Task_Init()
{
    SYS_Timestamp.Init(&htim5);
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK) { Error_Handler(); }

    M2006[0].Init(&hfdcan1, M2006_ID[0], Motor_DJI_Control_Method_TORQUE);
    M2006[1].Init(&hfdcan2, M2006_ID[1], Motor_DJI_Control_Method_TORQUE);
    CAN_Init(&hfdcan1, M2006_CAN1_RxCallback);
    CAN_Init(&hfdcan2, M2006_CAN2_RxCallback);

    // MaixCAM UART1 双向通信（115200 8N1，仅信号与共地，不跨接供电）。
    UART_Init(&huart1, Vision_Uart_RxCallback);
    UART_Set_Tx_Callback(&huart1, Telemetry_Uart_Tx_Callback);

    // 50 Hz 三线 PWM 舵机初始化（预装 50 Hz 周期与安全脉宽，但保持停波直到电池电压采样校验通过）。
    Servos_Started = false;
    Servos_Fault_Latched =
        !(Gate_Servo.Init(&htim1, TIM_CHANNEL_1, Gate_Servo_Config, false) &&
          Pitch_Servo.Init(&htim2, TIM_CHANNEL_1, Pitch_Servo_Config, false));

    // 电池采样有效前关闭电调与外设供电；主控继续运行以显示低压/故障红灯。
    BSP_Power.Init(false, false, false);
    SPI_Init(&hspi6, nullptr);
    BSP_WS2812.Init(255, 0, 0);
    BSP_Buzzer.Init();
    if (ADC_Init(&hadc1, ADC_BUFFER_SIZE) != HAL_OK)
    {
        Battery.Stop();
    }

    // 标记初始化完成
    init_finished = true;
}

void Task_Control_Run()
{
    // Control and driver delays use one millisecond ticks.
    if (osKernelGetTickFreq() != 1000U) { Error_Handler(); }
    uint32_t wake_tick = osKernelGetTickCount();
    for (;;)
    {
        const uint32_t irq_state = __get_PRIMASK();
        __disable_irq();
        Battery.Check_Timeout(HAL_GetTick());
        const bool battery_ready = Battery.Can_Run();
        Battery_LED_Green = Battery.Get_LED_Green();
        BSP_Power.Set_DC24_0(battery_ready);
        BSP_Power.Set_DC24_1(battery_ready);
        BSP_Power.Set_DC5(battery_ready && Board_Enable_DC5_Rail);
        __set_PRIMASK(irq_state);
        Drive_Control();
        IWDG_Independent_Feed();
        Wait_For_Next_Period(wake_tick, 1U);
    }
}

void Task_Imu_Run()
{
    uint32_t wake_tick = osKernelGetTickCount();
    for (;;)
    {
        if (!BMI088_Device.status.initialized)
        {
            if (!BMI088_Init())
            {
                osDelay(3000U);
                wake_tick = osKernelGetTickCount();
                continue;
            }
            wake_tick = osKernelGetTickCount();
        }
        const bool valid = BMI088_Update();
        const auto &attitude = BMI088_Device.attitude;
        const auto &rate = BMI088_Device.sample.gyro_radps;
        const bool quiet = valid && std::fabs(rate[0]) < 0.1f &&
                           std::fabs(rate[1]) < 0.1f && std::fabs(rate[2]) < 0.1f;
        const uint32_t irq_state = __get_PRIMASK();
        __disable_irq();
        if (valid)
        {
            Heading_Rad = Drive_Yaw_Sign * attitude.yawTotalAngle * BASIC_MATH_DEG_TO_RAD;
            Yaw_Rate_Rad_S = Drive_Yaw_Sign * attitude.yawSpeed;
            Attitude_Sample_Ms = HAL_GetTick();
            Imu_Quiet = quiet;
        }
        Attitude_Valid = valid;
        __set_PRIMASK(irq_state);
        Wait_For_Next_Period(wake_tick, 1U);
    }
}

void Task_Status_Run()
{
    uint32_t wake_tick = osKernelGetTickCount();
    uint8_t telemetry_div = 0U;
    for (;;)
    {
        const uint32_t irq_state = __get_PRIMASK();
        __disable_irq();
        const Struct_WS2812_Color led = Status_LED_Color;
        __set_PRIMASK(irq_state);

        BSP_WS2812.Set_RGB(led.Red, led.Green, led.Blue);
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();

        // 在非实时 10 ms 状态任务中执行 UART1 RX DMA 阻塞恢复，避免拖慢 1 ms 电机控制周期。
        (void)UART_Service_Rx_Recovery(&huart1);

        // 每 50 ms (20 Hz) 发送遥测帧至 MaixCAM，仅在 DMA 空闲时发送静态缓冲区并确认瞬态诊断已发出。
        if (++telemetry_div >= 5U)
        {
            telemetry_div = 0U;
            (void)Telemetry_Service_Tx();
        }
        Wait_For_Next_Period(wake_tick, 10U);
    }
}

void Task_Timestamp_Overflow()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Battery_Process_Samples(0);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Battery_Process_Samples(ADC_BUFFER_SIZE / 2);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Battery.Stop();
    }
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
    if (!init_finished)
    {
        return;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
