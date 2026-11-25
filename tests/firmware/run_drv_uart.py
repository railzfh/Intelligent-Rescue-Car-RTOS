"""Exercise the actual drv_uart.cpp DMA ReceiveToIdle double-buffer & HT guard."""
import os
from pathlib import Path
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/native-uart"
out.mkdir(parents=True, exist_ok=True)

(out / "stm32h7xx_hal.h").write_text("", encoding="ascii")
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
#define HAL_OK 0U
#define HAL_UART_STATE_READY   0x20U
#define HAL_UART_STATE_BUSY_RX 0x22U
#define HAL_UART_RXEVENT_TC   0U
#define HAL_UART_RXEVENT_HT   1U
#define HAL_UART_RXEVENT_IDLE 2U

struct DMA_HandleTypeDef {
    uint32_t disabled_it_mask = 0U;
    uint32_t ht_disable_calls = 0U;
};
struct UART_HandleTypeDef {
    void *Instance = nullptr;
    DMA_HandleTypeDef *hdmarx = nullptr;
    uint32_t RxEventType = HAL_UART_RXEVENT_TC;
    uint32_t RxState = HAL_UART_STATE_READY;
    bool dma_busy = false;
    uint8_t *armed_buf = nullptr;
    uint16_t armed_len = 0U;
    uint32_t arm_calls = 0U;
    uint32_t busy_rejects = 0U;
};

#define __HAL_DMA_DISABLE_IT(hdma, it) do { \
    (hdma)->disabled_it_mask |= (it); \
    if ((it) == DMA_IT_HT) { ++(hdma)->ht_disable_calls; } \
} while (0)

uint8_t HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
uint8_t HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
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

uint8_t HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    if (huart->dma_busy) {
        ++huart->busy_rejects;
        return 2U; // HAL_BUSY
    }
    huart->dma_busy = true;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    huart->armed_buf = pData;
    huart->armed_len = Size;
    ++huart->arm_calls;
    return 0U;
}

uint8_t HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t) { return 0U; }

static int cb_count = 0;
static uint8_t *cb_buf = nullptr;
static uint16_t cb_len = 0U;

static void On_Rx(uint8_t *buf, uint16_t len) {
    ++cb_count;
    cb_buf = buf;
    cb_len = len;
}

int main() {
    DMA_HandleTypeDef hdma{};
    UART_HandleTypeDef huart1{};
    huart1.Instance = USART1;
    huart1.hdmarx = &hdma;

    UART_Init(&huart1, On_Rx);
    assert(huart1.arm_calls == 1U);
    assert(huart1.armed_buf == UART1_Manage_Object.Rx_Buffer_0);
    assert((hdma.disabled_it_mask & DMA_IT_HT) != 0U);
    assert(hdma.ht_disable_calls == 1U);

    // Simulate a 300-byte burst where an HT event arrives mid-transfer while DMA is still busy!
    std::memset(UART1_Manage_Object.Rx_Buffer_0, 0x5A, 300U);
    huart1.RxEventType = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1, UART_BUFFER_SIZE / 2U);
    assert(cb_count == 0);
    assert(huart1.busy_rejects == 0U);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_0);

    // Terminal IDLE event finishes normal-mode DMA and delivers the exact Rx_Buffer_0 payload.
    huart1.dma_busy = false;
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, 300U);
    assert(cb_count == 1);
    assert(cb_buf == UART1_Manage_Object.Rx_Buffer_0);
    assert(cb_len == 300U);
    assert(cb_buf[0] == 0x5A && cb_buf[299] == 0x5A);
    assert(huart1.busy_rejects == 0U);
    assert(huart1.armed_buf == UART1_Manage_Object.Rx_Buffer_1);
    assert(hdma.ht_disable_calls == 2U);

    // Next burst completes via TC on Rx_Buffer_1.
    std::memset(UART1_Manage_Object.Rx_Buffer_1, 0xA5, UART_BUFFER_SIZE);
    huart1.dma_busy = false;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.RxEventType = HAL_UART_RXEVENT_TC;
    HAL_UARTEx_RxEventCallback(&huart1, UART_BUFFER_SIZE);
    assert(cb_count == 2);
    assert(cb_buf == UART1_Manage_Object.Rx_Buffer_1);
    assert(cb_len == UART_BUFFER_SIZE);
    assert(cb_buf[0] == 0xA5 && cb_buf[511] == 0xA5);
    assert(huart1.armed_buf == UART1_Manage_Object.Rx_Buffer_0);
    assert(hdma.ht_disable_calls == 3U);

    // TX-only error while RX DMA is still BUSY_RX must NOT clobber active RX buffer!
    // Swap to Rx_Buffer_1 active first.
    huart1.dma_busy = false;
    huart1.RxState = HAL_UART_STATE_READY;
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, 10U);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_1);
    HAL_UART_ErrorCallback(&huart1); // RxState is BUSY_RX -> must ignore!
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_1);
    assert(huart1.busy_rejects == 0U);

    // Real RX DMA error aborts RX (RxState == READY) -> HAL_UART_ErrorCallback re-arms cleanly and disables HT.
    huart1.dma_busy = false;
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UART_ErrorCallback(&huart1);
    assert(UART1_Manage_Object.Rx_Buffer_Active == UART1_Manage_Object.Rx_Buffer_0);
    assert(huart1.dma_busy && huart1.RxState == HAL_UART_STATE_BUSY_RX);
    assert(hdma.ht_disable_calls == 5U);

    std::puts("PASS: drv_uart HT guard, double-buffer ownership, TX-error preservation, RX-error recovery");
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
