"""Exercise drv_uart.cpp with a HAL-faithful STM32H7 UART/DMA state-machine mock."""
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

#define HAL_DMA_STATE_RESET    0x00U
#define HAL_DMA_STATE_READY    0x01U
#define HAL_DMA_STATE_BUSY     0x02U
#define HAL_DMA_STATE_TIMEOUT  0x03U
#define HAL_DMA_STATE_ERROR    0x04U
#define HAL_DMA_ERROR_NONE     0x00U
#define HAL_DMA_ERROR_TIMEOUT  0x20U
#define HAL_DMA_ERROR_NO_XFER  0x100U

struct DMA_HandleTypeDef {
    uint32_t State = HAL_DMA_STATE_READY;
    uint32_t ErrorCode = HAL_DMA_ERROR_NONE;
    uint32_t disabled_it_mask = 0U;
    uint32_t ht_disable_calls = 0U;
    uint32_t abort_calls = 0U;
    uint32_t deinit_calls = 0U;
    uint32_t init_calls = 0U;
    bool timeout_abort_once = false;
    bool wedge_init_always = false;
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
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
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

// Mirrors bundled Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c:781 HAL_DMA_Abort():
// - Rejects any state != BUSY with HAL_DMA_ERROR_NO_XFER / HAL_ERROR.
// - On timeout, transitions State to HAL_DMA_STATE_ERROR and returns HAL_ERROR.
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma) {
    ++hdma->abort_calls;
    if (hdma->State != HAL_DMA_STATE_BUSY) {
        hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
        return HAL_ERROR;
    }
    if (hdma->timeout_abort_once) {
        hdma->timeout_abort_once = false;
        hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;
        hdma->State = HAL_DMA_STATE_ERROR;
        return HAL_ERROR;
    }
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    hdma->State = HAL_DMA_STATE_READY;
    return HAL_OK;
}

// Mirrors bundled stm32h7xx_hal_dma.c:468 HAL_DMA_DeInit().
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma) {
    ++hdma->deinit_calls;
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    hdma->State = HAL_DMA_STATE_RESET;
    return HAL_OK;
}

// Mirrors bundled stm32h7xx_hal_dma.c:216 HAL_DMA_Init().
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma) {
    ++hdma->init_calls;
    if (hdma->wedge_init_always) {
        hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;
        hdma->State = HAL_DMA_STATE_ERROR;
        return HAL_ERROR;
    }
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    hdma->State = HAL_DMA_STATE_READY;
    return HAL_OK;
}

// Mirrors bundled stm32h7xx_hal_uart.c:1807 HAL_UART_AbortReceive():
// On HAL_DMA_Abort timeout, returns HAL_TIMEOUT early BEFORE resetting huart->RxState!
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart) {
    ++huart->abort_rx_calls;
    huart->cr1 &= ~USART_CR1_IDLEIE;
    if ((huart->cr3 & USART_CR3_DMAR) != 0U) {
        huart->cr3 &= ~USART_CR3_DMAR;
        if (huart->hdmarx != nullptr) {
            if (HAL_DMA_Abort(huart->hdmarx) != HAL_OK) {
                if (huart->hdmarx->ErrorCode == HAL_DMA_ERROR_TIMEOUT) {
                    huart->ErrorCode = HAL_UART_ERROR_DMA;
                    return HAL_TIMEOUT;
                }
            }
        }
    }
    huart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}

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

static void Simulate_Bundled_HAL_Tx_DMA_Error(UART_HandleTypeDef *huart) {
    if (huart->hdmatx != nullptr) {
        huart->hdmatx->State = HAL_DMA_STATE_READY;
    }
    huart->gState = HAL_UART_STATE_READY;
    if ((huart->cr3 & USART_CR3_DMAR) != 0U && huart->RxState == HAL_UART_STATE_BUSY_RX) {
        huart->cr1 &= ~USART_CR1_IDLEIE;
        huart->RxState = HAL_UART_STATE_READY;
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
    assert((hdmarx.disabled_it_mask & DMA_IT_HT) != 0U);
    assert(UART_Is_Rx_Healthy(&huart1));
    assert(!UART_Is_Rx_Fault_Latched(&huart1));
    assert(UART_Get_Rx_Epoch(&huart1) == 0U);

    // 1. Mid-transfer HT event ignored; IDLE delivers Rx_Buffer_0 cleanly.
    std::memset(UART1_Manage_Object.Rx_Buffer_0, 0x5A, 300U);
    huart1.RxEventType = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1, UART_BUFFER_SIZE / 2U);
    assert(cb_count == 0);
    hdmarx.State = HAL_DMA_STATE_READY;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, 300U);
    assert(cb_count == 1 && cb_buf == UART1_Manage_Object.Rx_Buffer_0 && cb_len == 300U);

    // 2. Bundled HAL TX DMA error leaves hdmarx BUSY while setting RxState=READY -> task context recovers.
    uint8_t tx_dummy[4] = {1, 2, 3, 4};
    UART_Transmit_Data(&huart1, tx_dummy, 4U);
    const uint32_t epoch0 = UART_Get_Rx_Epoch(&huart1);
    Simulate_Bundled_HAL_Tx_DMA_Error(&huart1);
    assert(!UART_Is_Rx_Healthy(&huart1));
    assert(UART_Get_Rx_Epoch(&huart1) > epoch0);
    const uint32_t epoch1 = UART_Get_Rx_Epoch(&huart1);
    assert(UART1_Manage_Object.Rx_Recovery_Pending && huart1.arm_failures == 0U);
    assert(UART_Service_Rx_Recovery(&huart1));
    assert(UART_Get_Rx_Epoch(&huart1) > epoch1);
    assert(UART_Is_Rx_Healthy(&huart1));
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX && hdmarx.State == HAL_DMA_STATE_BUSY);
    assert(!UART_Is_Rx_Fault_Latched(&huart1));

    // 3. REAL HAL DMA abort timeout -> hdmarx transitions to HAL_DMA_STATE_ERROR!
    //    Driver MUST explicitly handle ERROR via HAL_DMA_DeInit + HAL_DMA_Init (NOT loop HAL_DMA_Abort!).
    Simulate_Bundled_HAL_Tx_DMA_Error(&huart1);
    assert(!UART_Is_Rx_Healthy(&huart1));
    hdmarx.timeout_abort_once = true;
    assert(UART_Service_Rx_Recovery(&huart1));
    assert(UART_Is_Rx_Healthy(&huart1));
    assert(hdmarx.deinit_calls == 1U && hdmarx.init_calls == 1U);
    assert(huart1.RxState == HAL_UART_STATE_BUSY_RX && hdmarx.State == HAL_DMA_STATE_BUSY);
    assert(!UART1_Manage_Object.Rx_Recovery_Pending && !UART_Is_Rx_Fault_Latched(&huart1));

    // 4. Transient rearm busy recovers on the next tick.
    Simulate_Bundled_HAL_Tx_DMA_Error(&huart1);
    huart1.fail_rearm_once = true;
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(!UART_Is_Rx_Healthy(&huart1));
    assert(UART1_Manage_Object.Rx_Recovery_Pending && !UART_Is_Rx_Fault_Latched(&huart1));
    assert(UART_Service_Rx_Recovery(&huart1));
    assert(UART_Is_Rx_Healthy(&huart1));
    assert(!UART1_Manage_Object.Rx_Recovery_Pending && !UART_Is_Rx_Fault_Latched(&huart1));

    // 5. Permanently wedged DMA stream (HAL_DMA_Init stays in HAL_DMA_STATE_ERROR) MUST latch fault!
    Simulate_Bundled_HAL_Tx_DMA_Error(&huart1);
    hdmarx.timeout_abort_once = true;
    hdmarx.wedge_init_always = true;
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(!UART_Is_Rx_Healthy(&huart1) && !UART_Is_Rx_Fault_Latched(&huart1));
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(!UART_Is_Rx_Healthy(&huart1) && !UART_Is_Rx_Fault_Latched(&huart1));
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(!UART_Is_Rx_Healthy(&huart1) && UART_Is_Rx_Fault_Latched(&huart1));
    const uint32_t init_before = hdmarx.init_calls;
    assert(!UART_Service_Rx_Recovery(&huart1));
    assert(hdmarx.init_calls == init_before); // Latched fault stops retrying wedged stream!

    std::puts("PASS: drv_uart HT guard, HAL-faithful DMA ERROR DeInit/Init recovery, wedged-stream fault latch");
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
