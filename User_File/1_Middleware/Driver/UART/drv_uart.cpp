/**
 * @file drv_uart.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 2.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-18 1.1 修改成cpp
 * @date 2024-05-05 1.2 新增错误中断
 * @date 2025-08-13 2.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART2_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART3_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART4_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART5_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART6_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART7_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART8_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART9_Manage_Object = {nullptr};
Struct_UART_Manage_Object UART10_Manage_Object = {nullptr};

/* Private function declarations ---------------------------------------------*/

static Struct_UART_Manage_Object *UART_Get_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart == nullptr)
    {
        return nullptr;
    }
    if (huart->Instance == USART1) { return &UART1_Manage_Object; }
    if (huart->Instance == USART2) { return &UART2_Manage_Object; }
    if (huart->Instance == USART3) { return &UART3_Manage_Object; }
    if (huart->Instance == UART4) { return &UART4_Manage_Object; }
    if (huart->Instance == UART5) { return &UART5_Manage_Object; }
    if (huart->Instance == USART6) { return &UART6_Manage_Object; }
    if (huart->Instance == UART7) { return &UART7_Manage_Object; }
    if (huart->Instance == UART8) { return &UART8_Manage_Object; }
    if (huart->Instance == UART9) { return &UART9_Manage_Object; }
    if (huart->Instance == USART10) { return &UART10_Manage_Object; }
    return nullptr;
}

static bool UART_Start_Rx_DMA(Struct_UART_Manage_Object *obj)
{
    if (obj == nullptr || obj->UART_Handler == nullptr || obj->Rx_Buffer_Active == nullptr)
    {
        return false;
    }
    UART_HandleTypeDef *huart = obj->UART_Handler;
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, obj->Rx_Buffer_Active, UART_BUFFER_SIZE) != HAL_OK)
    {
        // Never discard a failed rearm: defer full RX DMA abort + rearm to task context.
        obj->Rx_Recovery_Pending = true;
        return false;
    }
    if (huart->hdmarx != nullptr)
    {
        // Normal-mode double-buffered IDLE/TC reception must not swap buffers on HT.
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    return true;
}

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Callback Callback_Function)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr)
    {
        return;
    }
    obj->UART_Handler = huart;
    obj->Callback_Function = Callback_Function;
    obj->Rx_Buffer_Active = obj->Rx_Buffer_0;
    obj->Rx_Buffer_Ready = nullptr;
    obj->Rx_Recovery_Pending = false;
    (void)UART_Start_Rx_DMA(obj);
}

/**
 * @brief 重新初始化UART（任务上下文安全：先中止旧RX DMA再重装）
 *
 * @param huart UART编号
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr)
    {
        return;
    }
    if (obj->UART_Handler == nullptr)
    {
        obj->UART_Handler = huart;
    }
    obj->Rx_Recovery_Pending = true;
    (void)UART_Service_Rx_Recovery(huart);
}

/**
 * @brief 在任务上下文中恢复因TX/RX DMA或UART错误而停止的RX接收流
 *
 * @param huart UART编号
 * @return true RX DMA 处于有效运行态；false 恢复失败并保持待重试标志
 */
bool UART_Service_Rx_Recovery(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr || obj->UART_Handler != huart)
    {
        return false;
    }
    if (!obj->Rx_Recovery_Pending && huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        return true;
    }

    // Clear flag before recovery so any new error ISR arriving afterward re-asserts it.
    obj->Rx_Recovery_Pending = false;
    __DMB();

    // Bundled STM32H7 HAL UART_DMAError can reset huart->RxState to READY on a TX DMA error
    // while leaving hdmarx->State == HAL_DMA_STATE_BUSY. Stop the old RX stream completely first.
    if (HAL_UART_AbortReceive(huart) != HAL_OK)
    {
        obj->Rx_Recovery_Pending = true;
        return false;
    }
    if (huart->hdmarx != nullptr && huart->hdmarx->State != HAL_DMA_STATE_READY)
    {
        if (HAL_DMA_Abort(huart->hdmarx) != HAL_OK)
        {
            obj->Rx_Recovery_Pending = true;
            return false;
        }
    }

    obj->Rx_Buffer_Active = obj->Rx_Buffer_0;
    return UART_Start_Rx_DMA(obj);
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Ignore non-terminal half-transfer events while Normal-mode DMA is still active.
    if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr || obj->UART_Handler != huart)
    {
        return;
    }

    // 判断程序初始化完成
    if (!init_finished)
    {
        (void)UART_Start_Rx_DMA(obj);
        return;
    }

    obj->Rx_Buffer_Ready = obj->Rx_Buffer_Active;
    obj->Rx_Buffer_Active =
        (obj->Rx_Buffer_Active == obj->Rx_Buffer_0) ? obj->Rx_Buffer_1 : obj->Rx_Buffer_0;
    obj->Rx_Time_Stamp = SYS_Timestamp.Get_Current_Timestamp();

    (void)UART_Start_Rx_DMA(obj);

    if (obj->Callback_Function != nullptr)
    {
        obj->Callback_Function(obj->Rx_Buffer_Ready, Size);
    }
}

/**
 * @brief HAL库UART错误中断
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr || obj->UART_Handler != huart)
    {
        return;
    }
    // Only schedule task-context recovery when HAL has stopped RX (including TX DMA errors that
    // cleared RxState to READY while leaving hdmarx BUSY). Preserve active RX on non-stopping errors.
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        obj->Rx_Recovery_Pending = true;
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/