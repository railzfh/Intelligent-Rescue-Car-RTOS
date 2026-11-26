"""Exercise the actual drv_uart.cpp DMA ReceiveToIdle, HT guard, and STM32H7 HAL error recovery."""
import os
from pathlib import Path
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/native-uart"
out.mkdir(parents=True, exist_ok=True)

(out / "stm32h7xx_hal.h").write_text(
    r'''#ifndef MOCK_STM32H7XX_HAL_H
#define MOCK_STM32H7XX_HAL_H
#include <stdint.h>
static inline void __DMB() {}
#endif
''',
    encoding="ascii",
)
(out / "1_Middleware/System/Timestamp").mkdir(parents=True, exist_ok=True)
(out / "1_Middleware/System/Timestamp/sys_timestamp.h").write_text(
    r'''#ifndef MOCK_SYS_TIMESTAMP_H
#define MOCK_SYS_TIMESTAMP_H
#include <stdint.h>
struct Mock_Timestamp { uint64_t Get_Current_Timestamp() const { return 1234ULL; } };
extern Mock_Timestamp SYS_Timestamp;
#endif
''',
    encoding="ascii",
)
(out / "usart.h").write_text(
    r'''#ifndef MOCK_USART_H
#define MOCK_USART_H
#include <stdint.h>
#define USART1 ((void *)1)
#define USART2 ((void *)2)
#define USART3 ((void *)3)
#define UART4  ((void *)4)
#define UART5  ((void *)5)
#define USART6 ((void *)6)
#define UART7  ((void *)7)
#define UART8  ((void *)8)
#define UART9  ((void *)9)
#define USART10 ((void *)10)

#define DMA_IT_HT (1U << 2)
#define USART_CR3_DMAR (1U << 6)
#define USART_CR1_IDLEIE (1U << 4)

enum HAL_StatusTypeDef : uint8_t {
    HAL_OK = 0U,
    HAL_ERROR = 1U,
    HAL_BUSY = 2U,
    HAL_TIMEOUT = 3U,
};

#define HAL_UART_STATE_READY   0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
#define HAL_UART_STATE_BUSY_RX 0x22U
#define HAL_UART_RXEVENT_TC    0U
#define HAL_UART_RXEVENT_HT    1U
#define HAL_UART_RXEVENT_IDLE  2U
#define HAL_UART_ERROR_NONE    0x00U
#define HAL_UART_ERROR_DMA     0x10U

#define HAL_DMA_STATE_READY    0x01U
#define HAL_DMA_STATE_BUSY     0x02U

struct DMA_HandleTypeDef {
    uint32_t State = HAL_DMA_STATE_READY;
    uint32_t disabled_it_mask = 0U;
    uint32_t ht_disable_calls = 0U;
    uint32_t abort_calls = 0U;
    bool fail_abort_once = false;
};
struct UART_HandleTypeDef {
    void *Instance = nullptr;
    DMA_HandleTypeDef *hdmarx = nullptr;
    DMA_HandleTypeDef *hdmatx = nullptr;
    uint32_t cr1 = 0U;
    uint32_t cr3 = 0U;
    uint32_t ErrorCode = HAL_UART_ERROR_NONE;
    uint32_t RxEventType = HAL_UART_RXEVENT_TC;
    uint32_t gState = HAL_UART_STATE_READY;
    uint32_t RxState = HAL_UART_STATE_READY;
    uint8_t *armed_buf = nullptr;
    uint16_t armed_len = 0U;
    uint32_t arm_calls = 0U;
    uint32_t arm_failures = 0U;
    uint32_t abort_rx_calls = 0U;
    bool fail_rearm_once = false;
};

#define __HAL_DMA_DISABLE_IT(hdma, it) do { \
    (hdma)->disabled_it_mask |= (it); \
    if ((it) == DMA_IT_HT) { ++(hdma)->ht_disable_calls; } \
} while (0)

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
#endif
''',
    encoding="ascii",
)

test_cpp = out / "test_drv_uart.cpp"
test_cpp.write_text(
    r'''
#include "1_Middleware/Driver/UART/drv_uart.h"
#include <cassert>
#include <cstdio>
#include <cstring>

bool init_finished = true;
Mock_Timestamp SYS_Timestamp;

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma) {
    ++hdma->abort_calls;
    if (hdma->fail_abort_once) {
        hdma->fail_abort_once = false;
        return HAL_TIMEOUT;
    }
    hdma->State = HAL_DMA_STATE_READY;
    return HAL_OK;
}

// Mirrors bundled Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c HAL_UART_AbortReceive().
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart) {
    ++huart->abort_rx_calls;
    huart->cr1 &= ~USART_CR1_IDLEIE;
    if ((huart->cr3 & USART_CR3_DMAR) != 0U) {
        huart->cr3 &= ~USART_CR3_DMAR;
        if (huart->hdmarx != nullptr) {
            if (HAL_DMA_Abort(huart->hdmarx) != HAL_OK) {
                huart->ErrorCode = HAL_UART_ERROR_DMA;
                return HAL_TIMEOUT;
            }
        }
    }
    huart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}

// Mirrors bundled UART_Start_Receive_DMA(): fails with HAL_ERROR if hdmarx->State != READY!
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    if (huart->RxState != HAL_UART_STATE_READY) {
        ++huart->arm_failures;
        return HAL_BUSY;
    }
    if (huart->fail_rearm_once) {
        huart->fail_rearm_once = false;
        ++huart->arm_failures;
        return HAL_BUSY;
    }
    if (huart->hdmarx != nullptr && huart->hdmarx->State != HAL_DMA_STATE_READY) {
        huart->ErrorCode = HAL_UART_ERROR_DMA;
        huart->RxState = HAL_UART_STATE_READY;
        ++huart->arm_failures;
        return HAL_ERROR;
    }
    if (huart->hdmarx != nullptr) {
        huart->hdmarx->State = HAL_DMA_STATE_BUSY;
    }
    huart->cr3 |= USART_CR3_DMAR;
    huart->cr1 |= USART_CR1_IDLEIE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    huart->armed_buf = pData;
    huart->armed_len = Size;
    ++huart->arm_calls;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *, uint16_t) {
    huart->gState = HAL_UART_STATE_BUSY_TX;
    if (huart->hdmatx != nullptr) {
        huart->hdmatx->State = HAL_DMA_STATE_BUSY;
    }
    return HAL_OK;
}

// Mirrors bundled stm32h7xx_hal_uart.c:3826 UART_DMAError() triggered by a TX DMA error
// while RX DMA is active: resets huart->RxState = READY and clears IDLEIE,
// but leaves USART_CR3_DMAR set and hdmarx->State == HAL_DMA_STATE_BUSY!
static void Simulate_Bundled_HAL_Tx_DMA_Error(UART_HandleTypeDef *huart) {
    if (huart->hdmatx != nullptr) {
        huart->hdmatx->State = HAL_DMA_STATE_READY;
    }
    huart->gState = HAL_UART_STATE_READY;
    if ((huart->cr3 & USART_CR3_DMAR) != 0U && huart->RxState == HAL_UART_STATE_BUSY_RX) {
        huart->cr1 &= ~USART_CR1_IDLEIE;
        huart->RxState = HAL_UART_STATE_READY;
        // Critically, bundled UART_EndRxTransfer does NOT clear CR3_DMAR or stop hdmarx!
    }
    huart->ErrorCode |= HAL_UART_ERROR_DMA;
    HAL_UART_ErrorCallback(huart);
}

static int cb_count = 0;
static uint8_t *cb_buf = nullptr;
static uint16_t cb_len = 0U;

static void On_Rx(uint8_t *buf, uint16_t len) {
    ++cb_count;
    cb_buf = buf;
    cb_len = len;
}

int main() {
    DMA_HandleTypeDef hdmarx{};
    DMA_HandleTypeDef hdmatx{};
    UART_HandleTypeDef huart1{};
    huart1.Instance = USART1;
    huart1.hdmarx = &hdmarx;
    huart1.hdmatx = &hdmatx;

    UART_Init(&huart1, On_Rx);
    assert(huart1.arm_calls == 1U);
    assert(huart1.armed_buf == UART1_Manage_Object.Rx_Buffer_0);
    assert((hdmarx.disabled_it_mask & DMA_IT_HT) != 0U);
    assert(hdmarx.ht_disable_calls == 1U);
    assert(!UART1_Manage_Object.Rx_Recovery_Pending);

    // 1. Mid-transfer HT event must be ignored while normal-mode DMA is still busy.
    std::memset(UART1_Manage_Object.Rx_Buffer_0, 0x5A, 300U);
    huart1.RxEventType = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1, UART_BUFFER_SIZE / 2U);
    assert(cb_count == 0);
    assert(huart1.arm_failures == 0U);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_0);

    // 2. Terminal IDLE event finishes normal-mode DMA and delivers exact Rx_Buffer_0 payload.
    hdmarx.State = HAL_DMA_STATE_READY;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, 300U);
    assert(cb_count == 1);
    assert(cb_buf == UART1_Manage_Object.Rx_Buffer_0);
    assert(cb_len == 300U);
    assert(huart1.armed_buf == UART1_Manage_Object.Rx_Buffer_1);
    assert(hdmarx.ht_disable_calls == 2U);

    // 3. Non-stopping error while RX is still BUSY_RX must preserve active Rx_Buffer_1.
    HAL_UART_ErrorCallback(&huart1);
    assert(!UART1_Manage_Object.Rx_Recovery_Pending);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_1);
    assert(UART_Service_Rx_Recovery(&huart1));
    assert(huart1.abort_rx_calls == 0U);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_1);

    // 4. Actual bundled STM32H7 HAL TX DMA error sequence:
    //    UART_DMAError sets huart->RxState = READY while leaving hdmarx->State == BUSY!
    uint8_t tx_dummy[4] = {1, 2, 3, 4};
    UART_Transmit_Data(&huart1, tx_dummy, 4U);
    Simulate_Bundled_HAL_Tx_DMA_Error(&huart1);
    assert(huart1.RxState == HAL_UART_STATE_READY);
    assert(hdmarx.State == HAL_DMA_STATE_BUSY);
    // ISR must NOT attempt a doomed rearm while hdmarx is BUSY; it defers to task context.
    assert(huart1.arm_failures == 0U);
    assert(UART1_Manage_Object.Rx_Recovery_Pending);

    // Task-context recovery stops lingering hdmarx stream, rearms RX DMA, and disables HT.
    assert(UART_Service_Rx_Recovery(&huart1));
    assert(huart1.abort_rx_calls == 1U);
    assert(hdmarx.abort_calls == 1U);
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);
    assert((huart1.cr1 & USART_CR1_IDLEIE) != 0U);
    assert(hdmarx.State == HAL_DMA_STATE_BUSY);
    assert(!UART1_Manage_Object.Rx_Recovery_Pending);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_0);
    assert(hdmarx.ht_disable_calls == 3U);

    // 5. Transient abort failure AND transient rearm failure must keep Rx_Recovery_Pending set
    //    and succeed on retry without stranding RX.
    Simulate_Bundled_HAL_Tx_DMA_Error(&huart1);
    assert(UART1_Manage_Object.Rx_Recovery_Pending);
    hdmarx.fail_abort_once = true;
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(UART1_Manage_Object.Rx_Recovery_Pending);
    assert(huart1.RxState == HAL_UART_STATE_READY);

    // Second tick: CR3_DMAR was already cleared by the failed AbortReceive, yet hdmarx is STILL BUSY!
    // Explicit HAL_DMA_Abort fallback must clear hdmarx, then fail_rearm_once simulates transient HAL_BUSY.
    huart1.fail_rearm_once = true;
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(UART1_Manage_Object.Rx_Recovery_Pending);
    assert(hdmarx.State == HAL_DMA_STATE_READY);

    // Third tick: clean rearm succeeds!
    assert(UART_Service_Rx_Recovery(&huart1));
    assert(!UART1_Manage_Object.Rx_Recovery_Pending);
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX);
    assert(hdmarx.ht_disable_calls == 4U);

    std::puts("PASS: drv_uart HT guard, bundled HAL TX-DMA-error task recovery, transient failure retry");
    return 0;
}
''',
    encoding="utf-8",
)

exe = out / ("test_drv_uart.exe" if os.name == "nt" else "test_drv_uart")
cmd = [
    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-missing-field-initializers",
    "-I", str(out), "-I", str(repo / "User_File"),
    str(test_cpp), str(repo / "User_File/1_Middleware/Driver/UART/drv_uart.cpp"),
    "-o", str(exe),
]
if os.name == "nt":
    cmd.append("-static")
subprocess.run(cmd, check=True)
subprocess.run([str(exe)], check=True)
