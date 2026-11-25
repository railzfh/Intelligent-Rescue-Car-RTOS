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

static void Vision_Drain_Rx(uint32_t now_ms)
{
    uint16_t tail = Vision_Rx_Tail;
    const uint16_t head = Vision_Rx_Head;
    __DMB();
    Struct_Rescue_Frame frame;
    while (tail != head)
    {
        const uint8_t byte = Vision_Rx_Ring[tail];
        tail = static_cast<uint16_t>((tail + 1U) % Vision_Rx_Ring_Size);
        if (!Vision_Parser.Feed_Byte(byte, frame))
        {
            continue;
        }
        if (frame.msg_type == RESCUE_MSG_HEARTBEAT)
        {
            Struct_Vision_Heartbeat hb;
            if (Rescue_Decode_Heartbeat(frame, hb))
            {
                Rescue_Mission.On_Heartbeat(hb, now_ms);
            }
        }
        else if (frame.msg_type == RESCUE_MSG_SET_CONFIG)
        {
            Struct_Match_Config_Cmd cfg;
            if (Rescue_Decode_Set_Config(frame, cfg))
            {
                Rescue_Mission.On_Set_Config(cfg);
            }
        }
        else if (frame.msg_type == RESCUE_MSG_VISION_STATE)
        {
            Struct_Vision_State vis;
            if (Rescue_Decode_Vision_State(frame, vis))
            {
                Rescue_Mission.On_Vision_State(vis, now_ms);
            }
        }
        else if (frame.msg_type == RESCUE_MSG_BENCH_ACTUATOR)
        {
            Struct_Bench_Actuator_Cmd cmd;
            if (Rescue_Decode_Bench_Actuator(frame, cmd))
            {
                Rescue_Mission.On_Bench_Actuator(cmd);
            }
        }
    }
    __DMB();
    Vision_Rx_Tail = tail;
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
    bool feedback_fresh = true;
    // CAN 中断可以抢占该任务，确保同一轮转角、速度和时间戳来自同一帧。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    for (uint8_t i = 0; i < 2; ++i)
    {
        angle[i] = M2006_Direction[i] * M2006[i].Get_Now_Angle();
        speed[i] = M2006_Direction[i] * M2006[i].Get_Now_Omega();
        feedback_fresh &= M2006_Feedback_Received[i] &&
                          (uint32_t)(now_ms - M2006_Last_Feedback_Ms[i]) < M2006_Feedback_Timeout_Ms;
    }
    const bool attitude_fresh = Attitude_Valid && (uint32_t)(now_ms - Attitude_Sample_Ms) < 30U;
    const bool imu_quiet = Imu_Quiet;
    const float heading = Heading_Rad;
    const float yaw_rate = Yaw_Rate_Rad_S;
    const bool battery_ready = Battery.Can_Run();
    const float pack_voltage = Battery.Get_Pack_Voltage();
    __set_PRIMASK(primask);

    Vision_Drain_Rx(now_ms);

    const bool wheels_still = feedback_fresh && std::fabs(speed[0]) < 0.1f && std::fabs(speed[1]) < 0.1f;
    const bool can1_ready = Drive_Tx_Ready(hfdcan1);
    const bool can2_ready = Drive_Tx_Ready(hfdcan2);
    const bool ready = battery_ready && feedback_fresh && attitude_fresh && can1_ready && can2_ready;
    const bool can_start = ready && wheels_still && imu_quiet;
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

    const Enum_Mission_State prev_state = Rescue_Mission.Get_State();
    Struct_Mission_Inputs mission_in{};
    mission_in.now_ms = now_ms;
    mission_in.hardware_ready = ready;
    mission_in.can_launch = can_start;
    mission_in.button_pressed = button_pressed;
    mission_in.chassis_active = (prev_state == MISSION_BENCH_STRAIGHT)
                                    ? Straight_Drive.Is_Running()
                                    : Chassis_Drive.Is_Active();
    mission_in.chassis_motion_done = Chassis_Drive.Is_Motion_Done();
    mission_in.chassis_motion_aborted = Chassis_Drive.Is_Motion_Aborted();
    mission_in.chassis_stalled = Chassis_Drive.Is_Stalled();
    mission_in.gate_settled = Gate_Servo.Is_Settled();
    mission_in.camera_settled = Pitch_Servo.Is_Settled();
    mission_in.forward_delta_m = forward_delta_m;
    mission_in.yaw_delta_rad = yaw_delta_rad;

    const Struct_Mission_Output cmd = Rescue_Mission.Update(mission_in);
    Drive_Notify_Buzzer(can_start, cmd.launch_rejected_cue, now_ms);

    // 舵机控制：欠压保护立即关闭 PWM，正常状态执行 1 ms 缓动插补。
    if (!battery_ready)
    {
        Gate_Servo.Disable();
        Pitch_Servo.Disable();
    }
    else
    {
        Gate_Servo.Set_Target_Us(cmd.gate_closed ? Gate_Closed_Us : Gate_Open_Us);
        Pitch_Servo.Set_Target_Us(cmd.camera_near ? Pitch_Near_Us : Pitch_Far_Us);
        Gate_Servo.Update_1ms();
        Pitch_Servo.Update_1ms();
    }

    std::array<float, 2> torque = {};
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

    const auto can1_status = can1_ready ? CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8) : HAL_OK;
    const auto can2_status = can2_ready ? CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8) : HAL_OK;
    const bool can_tx_ok = (can1_status == HAL_OK && can2_status == HAL_OK);
    if (!can_tx_ok)
    {
        // CAN 发送失败必须立即锁停任务，防止下一周期自动恢复扭矩；并在可用总线上立刻补发零扭矩。
        Straight_Drive.Stop();
        Chassis_Drive.Emergency_Stop();
        Rescue_Mission.Fault_Stop(now_ms);
        for (uint8_t i = 0; i < 2; ++i)
        {
            M2006[i].Set_Target_Torque(0.0f);
            M2006[i].TIM_Calculate_PeriodElapsedCallback();
        }
        const bool can1_zero_ok = Drive_Tx_Ready(hfdcan1);
        const bool can2_zero_ok = Drive_Tx_Ready(hfdcan2);
        if (can1_status == HAL_OK && can1_zero_ok)
        {
            CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
        }
        if (can2_status == HAL_OK && can2_zero_ok)
        {
            CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
        }
    }

    // 发布灯光颜色与遥测快照，供 statusTask 消费。
    Struct_Rescue_Telemetry tel{};
    tel.uptime_ms = now_ms;
    tel.match_elapsed_ms = Rescue_Mission.Get_Match_Elapsed_Ms(now_ms);
    tel.mission_state = static_cast<uint8_t>(Rescue_Mission.Get_State());
    tel.team = Rescue_Mission.Get_Team();
    tel.start_zone = Rescue_Mission.Get_Start_Zone();
    tel.run_mode = Rescue_Mission.Get_Run_Mode();

    uint16_t status_flags = 0U;
    if (Rescue_Mission.Is_Config_Locked()) { status_flags |= TEL_FLAG_CONFIG_LOCKED; }
    if (battery_ready) { status_flags |= TEL_FLAG_BATTERY_OK; }
    if (attitude_fresh) { status_flags |= TEL_FLAG_IMU_OK; }
    if (feedback_fresh) { status_flags |= TEL_FLAG_MOTORS_OK; }
    if (Rescue_Mission.Is_Vision_Fresh(now_ms)) { status_flags |= TEL_FLAG_VISION_FRESH; }
    if (Rescue_Mission.Is_First_Green_Unlocked()) { status_flags |= TEL_FLAG_FIRST_GREEN_UNLOCKED; }
    if (Rescue_Mission.Is_Gate_Closed_Cmd()) { status_flags |= TEL_FLAG_GATE_CLOSED; }
    if (Gate_Servo.Is_Settled()) { status_flags |= TEL_FLAG_GATE_SETTLED; }
    if (Rescue_Mission.Is_Camera_Near_Cmd()) { status_flags |= TEL_FLAG_CAMERA_NEAR; }
    if (Pitch_Servo.Is_Settled()) { status_flags |= TEL_FLAG_CAMERA_SETTLED; }
    if (Chassis_Drive.Is_Stalled()) { status_flags |= TEL_FLAG_STALLED; }
    if (Rescue_Mission.Is_Link_Alive(now_ms)) { status_flags |= TEL_FLAG_LINK_ALIVE; }
    if (Rescue_Mission.Is_Carrying_Casualty()) { status_flags |= TEL_FLAG_CARRYING_CASUALTY; }
    tel.status_flags = status_flags;

    uint16_t fault_flags = 0U;
    if (!battery_ready) { fault_flags |= FAULT_FLAG_BATTERY; }
    if (!feedback_fresh) { fault_flags |= FAULT_FLAG_MOTOR_TIMEOUT; }
    if (!attitude_fresh) { fault_flags |= FAULT_FLAG_IMU_TIMEOUT; }
    if (!can1_ready || !can2_ready || !can_tx_ok) { fault_flags |= FAULT_FLAG_CAN_TX; }
    if (!Rescue_Mission.Is_Vision_Fresh(now_ms)) { fault_flags |= FAULT_FLAG_VISION_TIMEOUT; }
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

    const Struct_WS2812_Color led = Compute_Status_LED(battery_ready, ready, now_ms);
    const uint32_t pub_mask = __get_PRIMASK();
    __disable_irq();
    Status_LED_Color = led;
    Telemetry_Snapshot = tel;
    __set_PRIMASK(pub_mask);
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

    // 50 Hz 三线 PWM 舵机初始化（预装安全初始脉宽并开启 OCxPE）。
    Gate_Servo.Init(&htim1, TIM_CHANNEL_1, Gate_Servo_Config);
    Pitch_Servo.Init(&htim2, TIM_CHANNEL_1, Pitch_Servo_Config);

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
        __set_PRIMASK(irq_state);
        BSP_Power.Set_DC24_0(battery_ready);
        BSP_Power.Set_DC24_1(battery_ready);
        BSP_Power.Set_DC5(battery_ready && Board_Enable_DC5_Rail);
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
        const Struct_Rescue_Telemetry tel = Telemetry_Snapshot;
        __set_PRIMASK(irq_state);

        BSP_WS2812.Set_RGB(led.Red, led.Green, led.Blue);
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();

        // 每 50 ms (20 Hz) 发送遥测帧至 MaixCAM，仅在 DMA 空闲时发送静态缓冲区。
        if (++telemetry_div >= 5U)
        {
            telemetry_div = 0U;
            if (huart1.gState == HAL_UART_STATE_READY)
            {
                const size_t len = Rescue_Encode_Telemetry(
                    Telemetry_Tx_Seq++, tel, Telemetry_Tx_Buffer, sizeof(Telemetry_Tx_Buffer));
                if (len > 0U)
                {
                    UART_Transmit_Data(&huart1, Telemetry_Tx_Buffer, static_cast<uint16_t>(len));
                }
            }
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
