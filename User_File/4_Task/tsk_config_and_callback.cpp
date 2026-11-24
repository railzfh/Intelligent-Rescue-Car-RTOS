/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 双M2006定距直行、航向纠偏与电池监测
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "cmsis_os2.h"

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Battery/dvc_battery.h"
#include "2_Device/IMU/BMI088/dvc_bmi088.h"
#include "2_Device/BSP/Buzzer/bsp_buzzer.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "3_Robot/Drive/straight_drive.h"

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
static const uint32_t M2006_Feedback_Timeout_Ms = 100;
static Class_Motor_DJI_C610 M2006[2];
static volatile uint32_t M2006_Last_Feedback_Ms[2] = {};
static volatile bool M2006_Feedback_Received[2] = {};

// 驱动轮直径（mm）。0 表示未配置，禁止起步。
static constexpr float Drive_Wheel_Diameter_Mm = 58.0f;
// 实测约 0.90 m，编码器估算约 1.00 m。
static constexpr float Drive_Odometry_Scale = 0.90f;
static constexpr float Drive_Target_Distance_M = 1.0f;
// 姿态模块以传感器 Z 轴为航向轴，主控板须水平安装。俯视左转为正。
static constexpr float Drive_Yaw_Sign = 1.0f;
static const Struct_Straight_Drive_Config Straight_Drive_Config = {
    Drive_Wheel_Diameter_Mm * 0.0005f * Drive_Odometry_Scale,
    Drive_Target_Distance_M,
    M2006_Forward_Rpm * BASIC_MATH_RPM_TO_RADPS,
    M2006_Acceleration_Rpm_Per_S * BASIC_MATH_RPM_TO_RADPS,
    M2006_Torque_Limit,
};
// 左右轮 b0 可根据各自实测的扭矩和加速度分别调整。
static Class_Straight_Drive Straight_Drive(Straight_Drive_Config, 40.0f, 40.0f);

static volatile bool Attitude_Valid = false;
static volatile bool Imu_Quiet = false;
static volatile float Heading_Rad = 0.0f;
static volatile float Yaw_Rate_Rad_S = 0.0f;
static volatile uint32_t Attitude_Sample_Ms = 0;

static Class_Battery Battery;
static volatile uint8_t Battery_LED_Green = 0;

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

static void Drive_Notify_Ready(bool can_start, uint32_t now_ms)
{
    static bool notified = false;
    static bool sounding = false;
    static uint32_t started_ms = 0;
    if (!notified && can_start)
    {
        notified = true;
        sounding = true;
        started_ms = now_ms;
        BSP_Buzzer.Set_Loudness(0.25f);
    }
    else if (sounding && now_ms - started_ms >= 100U)
    {
        sounding = false;
        BSP_Buzzer.Set_Loudness(0.0f);
    }
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
                          now_ms - M2006_Last_Feedback_Ms[i] < M2006_Feedback_Timeout_Ms;
    }
    const bool attitude_fresh = Attitude_Valid && now_ms - Attitude_Sample_Ms < 30U;
    const bool imu_quiet = Imu_Quiet;
    const float heading = Heading_Rad;
    const float yaw_rate = Yaw_Rate_Rad_S;
    const bool battery_ready = Battery.Can_Run();
    __set_PRIMASK(primask);
    const bool wheels_still = feedback_fresh && std::fabs(speed[0]) < 0.1f && std::fabs(speed[1]) < 0.1f;

    const bool can1_ready = Drive_Tx_Ready(hfdcan1);
    const bool can2_ready = Drive_Tx_Ready(hfdcan2);
    const bool ready = battery_ready && feedback_fresh && attitude_fresh && can1_ready && can2_ready;
    const bool can_start = ready && wheels_still && imu_quiet;
    const bool button_pressed = Drive_Button_Pressed(now_ms);
    Drive_Notify_Ready(can_start, now_ms);

    if (!ready)
    {
        Straight_Drive.Stop();
    }
    else if (button_pressed && Straight_Drive.Is_Running())
    {
        Straight_Drive.Request_Stop(now_ms);
    }
    else if (button_pressed && can_start)
    {
        Straight_Drive.Start(angle, speed, heading, now_ms);
    }

    const auto torque = Straight_Drive.Update(angle, speed, heading, yaw_rate, now_ms);
    for (uint8_t i = 0; i < 2; ++i)
    {
        M2006[i].Set_Target_Torque(M2006_Direction[i] * torque[i]);
        M2006[i].TIM_Calculate_PeriodElapsedCallback();
    }

    const auto can1_status = can1_ready ? CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8) : HAL_OK;
    const auto can2_status = can2_ready ? CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8) : HAL_OK;
    if (can1_status != HAL_OK || can2_status != HAL_OK)
    {
        Straight_Drive.Stop();
        Drive_Tx_Ready(hfdcan1);
        Drive_Tx_Ready(hfdcan2);
    }
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
    // 电池采样有效前关闭电调供电；主控继续运行以显示低压/故障红灯。
    BSP_Power.Init(false, false);
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
        if (!battery_ready) { BSP_Power.Set_DC5(false); }
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
    for (;;)
    {
        const uint8_t green = Battery_LED_Green;
        BSP_WS2812.Set_RGB(255U - green, green, 0);
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
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
    if (!init_finished)
    {
        return;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
