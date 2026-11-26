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

static constexpr uint8_t UART_Max_Rx_Recovery_Failures = 3U;

static void UART_Record_Recovery_Failure(Struct_UART_Manage_Object *obj)
{
    if (obj->Rx_Recovery_Fail_Count < UART_Max_Rx_Recovery_Failures)
    {
        ++obj->Rx_Recovery_Fail_Count;
    }
    if (obj->Rx_Recovery_Fail_Count >= UART_Max_Rx_Recovery_Failures)
    {
        obj->Rx_Recovery_Fault_Latched = true;
        obj->Rx_Recovery_Pending = false;
    }
    else
    {
        obj->Rx_Recovery_Pending = true;
    }
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
    obj->Tx_Callback_Function = nullptr;
    obj->Rx_Buffer_Active = obj->Rx_Buffer_0;
    obj->Rx_Buffer_Ready = nullptr;
    obj->Rx_Recovery_Pending = false;
    obj->Rx_Recovery_Fault_Latched = false;
    obj->Rx_Recovery_Epoch = 0U;
    obj->Rx_Recovery_Fail_Count = 0U;
    obj->Tx_Active = false;
    (void)UART_Start_Rx_DMA(obj);
}

void UART_Set_Tx_Callback(UART_HandleTypeDef *huart, UART_Tx_Callback Callback_Function)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr)
    {
        return;
    }
    obj->Tx_Callback_Function = Callback_Function;
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
    obj->Rx_Recovery_Epoch = obj->Rx_Recovery_Epoch + 1U;
    obj->Rx_Recovery_Pending = true;
    (void)UART_Service_Rx_Recovery(huart);
}

/**
 * @brief 非阻塞查询 UART RX DMA 当前是否处于健康运行态（适用于 1 ms 实时控制任务）
 */
bool UART_Is_Rx_Healthy(UART_HandleTypeDef *huart)
{
    const Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr || obj->UART_Handler != huart)
    {
        return false;
    }
    if (obj->Rx_Recovery_Fault_Latched || obj->Rx_Recovery_Pending)
    {
        return false;
    }
    const bool dma_running_ok = (huart->hdmarx == nullptr) || (huart->hdmarx->State == HAL_DMA_STATE_BUSY);
    return (huart->RxState == HAL_UART_STATE_BUSY_RX) && dma_running_ok;
}

/**
 * @brief 查询该 UART 的 RX DMA 恢复是否已锁存永久故障
 */
bool UART_Is_Rx_Fault_Latched(UART_HandleTypeDef *huart)
{
    const Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    return (obj != nullptr && obj->UART_Handler == huart && obj->Rx_Recovery_Fault_Latched);
}

/**
 * @brief 读取 RX 错误/恢复单调纪元计数，便于 1 ms 控制任务捕捉跨周期恢复并失效旧视觉帧
 */
uint32_t UART_Get_Rx_Epoch(UART_HandleTypeDef *huart)
{
    const Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr || obj->UART_Handler != huart)
    {
        return 0U;
    }
    return obj->Rx_Recovery_Epoch;
}

/**
 * @brief 在任务上下文中恢复因TX/RX DMA或UART错误而停止的RX接收流
 *
 * @param huart UART编号
 * @return true RX DMA 处于有效运行态；false 恢复失败并按预算重试或锁存故障
 */
bool UART_Service_Rx_Recovery(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj == nullptr || obj->UART_Handler != huart)
    {
        return false;
    }
    if (obj->Rx_Recovery_Fault_Latched)
    {
        return false;
    }
    const bool dma_running_ok = (huart->hdmarx == nullptr) || (huart->hdmarx->State == HAL_DMA_STATE_BUSY);
    if (!obj->Rx_Recovery_Pending && huart->RxState == HAL_UART_STATE_BUSY_RX && dma_running_ok)
    {
        return true;
    }

    // Clear flag before recovery so any new error ISR arriving afterward re-asserts it.
    obj->Rx_Recovery_Pending = false;
    __DMB();

    // Bundled STM32H7 HAL UART_DMAError can reset huart->RxState to READY on a TX DMA error
    // while leaving hdmarx->State == HAL_DMA_STATE_BUSY, and a HAL_DMA_Abort timeout puts
    // hdmarx->State into HAL_DMA_STATE_ERROR where subsequent HAL_DMA_Abort calls always fail.
    HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(huart);
    if (huart->hdmarx != nullptr)
    {
        if (huart->hdmarx->State == HAL_DMA_STATE_BUSY)
        {
            (void)HAL_DMA_Abort(huart->hdmarx);
        }
        if (huart->hdmarx->State != HAL_DMA_STATE_READY)
        {
            // Explicitly recover HAL_DMA_STATE_ERROR / non-ready stream via checked DeInit + Init.
            const HAL_StatusTypeDef deinit_st = HAL_DMA_DeInit(huart->hdmarx);
            const HAL_StatusTypeDef init_st = (deinit_st == HAL_OK) ? HAL_DMA_Init(huart->hdmarx) : deinit_st;
            if (init_st != HAL_OK || huart->hdmarx->State != HAL_DMA_STATE_READY)
            {
                UART_Record_Recovery_Failure(obj);
                return false;
            }
            // HAL_UART_AbortReceive returns early on DMA timeout before resetting huart->RxState.
            abort_status = HAL_UART_AbortReceive(huart);
        }
    }
    if (abort_status != HAL_OK || huart->RxState != HAL_UART_STATE_READY)
    {
        UART_Record_Recovery_Failure(obj);
        return false;
    }

    obj->Rx_Buffer_Active = obj->Rx_Buffer_0;
    obj->Rx_Recovery_Epoch = obj->Rx_Recovery_Epoch + 1U;
    __DMB();
    if (!UART_Start_Rx_DMA(obj))
    {
        UART_Record_Recovery_Failure(obj);
        return false;
    }
    obj->Rx_Recovery_Fail_Count = 0U;
    return true;
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
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool track = (obj != nullptr && obj->UART_Handler == huart);
    const bool was_active = track && obj->Tx_Active;
    if (track && !was_active)
    {
        obj->Tx_Active = true;
    }
    const uint8_t status = static_cast<uint8_t>(HAL_UART_Transmit_DMA(huart, Data, Length));
    if (status != HAL_OK && track)
    {
        // Preserve prior active TX state on HAL_BUSY/failed retry so original TX completion is never suppressed.
        obj->Tx_Active = was_active;
    }
    __set_PRIMASK(primask);
    return status;
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj != nullptr && obj->UART_Handler == huart && obj->Tx_Active)
    {
        obj->Tx_Active = false;
        if (obj->Tx_Callback_Function != nullptr)
        {
            obj->Tx_Callback_Function(true);
        }
    }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *obj = UART_Get_Manage_Object(huart);
    if (obj != nullptr && obj->UART_Handler == huart && obj->Tx_Active)
    {
        obj->Tx_Active = false;
        if (obj->Tx_Callback_Function != nullptr)
        {
            obj->Tx_Callback_Function(false);
        }
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
        obj->Rx_Recovery_Epoch = obj->Rx_Recovery_Epoch + 1U;
        obj->Rx_Recovery_Pending = true;
    }
    // Notify TX failure ONLY when an active TX transfer was actually terminated by HAL (gState returned to READY).
    // RX-only errors when no TX is active (or while TX stays BUSY_TX) MUST NOT erase TX acknowledgment state.
    if (obj->Tx_Active && huart->gState == HAL_UART_STATE_READY)
    {
        obj->Tx_Active = false;
        if (obj->Tx_Callback_Function != nullptr)
        {
            obj->Tx_Callback_Function(false);
        }
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/