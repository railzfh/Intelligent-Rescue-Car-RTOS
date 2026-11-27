"""Exercise drv_can.cpp with a HAL-faithful STM32H7 FDCAN PSR/IR Bus-Off mock."""
import os
from pathlib import Path
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/native-can"
out.mkdir(parents=True, exist_ok=True)

(out / "stm32h7xx_hal.h").write_text(
    r'''#ifndef MOCK_STM32H7XX_HAL_H
#define MOCK_STM32H7XX_HAL_H
#include <stdint.h>
static inline uint32_t __get_PRIMASK() { return 0U; }
static inline void __disable_irq() {}
static inline void __set_PRIMASK(uint32_t) {}
#endif
''',
    encoding="ascii",
)
(out / "1_Middleware/System/Timestamp").mkdir(parents=True, exist_ok=True)
(out / "1_Middleware/System/Timestamp/sys_timestamp.h").write_text(
    r'''#ifndef MOCK_SYS_TIMESTAMP_H
#define MOCK_SYS_TIMESTAMP_H
#include <stdint.h>
struct Mock_Timestamp { uint64_t Get_Current_Timestamp() const { return 4321ULL; } };
extern Mock_Timestamp SYS_Timestamp;
#endif
''',
    encoding="ascii",
)
(out / "fdcan.h").write_text(
    r'''#ifndef MOCK_FDCAN_H
#define MOCK_FDCAN_H
#include <stdint.h>

#define RESET 0U
#define HAL_OK 0U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_REJECT 0U
#define FDCAN_FILTER_REMOTE 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_RX_FIFO0 0U

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1U << 0)
#define FDCAN_IT_BUS_OFF              (1U << 25)
#define FDCAN_IT_ERROR_PASSIVE        (1U << 23)
#define FDCAN_IT_ARB_PROTOCOL_ERROR   (1U << 27)
#define FDCAN_IT_DATA_PROTOCOL_ERROR  (1U << 28)

#define FDCAN_PSR_BO                  (1U << 7)
#define FDCAN_IR_BO                   (1U << 25)

struct FDCAN_GlobalTypeDef {
    uint32_t TXBTO = 0U;
    uint32_t TXBRP = 0U;
    uint32_t TXBCF = 0U;
    uint32_t PSR = 0U;
    uint32_t IR = 0U;
};

extern FDCAN_GlobalTypeDef Mock_FDCAN1;
extern FDCAN_GlobalTypeDef Mock_FDCAN2;
extern FDCAN_GlobalTypeDef Mock_FDCAN3;
#define FDCAN1 (&Mock_FDCAN1)
#define FDCAN2 (&Mock_FDCAN2)
#define FDCAN3 (&Mock_FDCAN3)

struct FDCAN_HandleTypeDef {
    FDCAN_GlobalTypeDef *Instance = nullptr;
    uint32_t rx_fifo0_fill = 0U;
    uint32_t stop_calls = 0U;
    uint32_t start_calls = 0U;
};
extern FDCAN_HandleTypeDef hfdcan1;

struct FDCAN_FilterTypeDef {
    uint32_t IdType = 0U;
    uint32_t FilterIndex = 0U;
    uint32_t FilterType = 0U;
    uint32_t FilterConfig = 0U;
    uint32_t FilterID1 = 0U;
    uint32_t FilterID2 = 0U;
};
struct FDCAN_TxHeaderTypeDef {
    uint32_t Identifier = 0U;
    uint32_t IdType = 0U;
    uint32_t TxFrameType = 0U;
    uint32_t DataLength = 0U;
    uint32_t ErrorStateIndicator = 0U;
    uint32_t BitRateSwitch = 0U;
    uint32_t FDFormat = 0U;
    uint32_t TxEventFifoControl = 0U;
    uint32_t MessageMarker = 0U;
};
struct FDCAN_RxHeaderTypeDef {
    uint32_t Identifier = 0U;
};

uint8_t HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *);
uint8_t HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
uint8_t HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
uint8_t HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
uint8_t HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
uint8_t HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *, uint32_t);
uint8_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);
#endif
''',
    encoding="ascii",
)

test_cpp = out / "test_drv_can.cpp"
test_cpp.write_text(
    r'''
#include "1_Middleware/Driver/CAN/drv_can.h"
#include <cassert>
#include <cstdio>

bool init_finished = true;
Mock_Timestamp SYS_Timestamp;
FDCAN_GlobalTypeDef Mock_FDCAN1{};
FDCAN_GlobalTypeDef Mock_FDCAN2{};
FDCAN_GlobalTypeDef Mock_FDCAN3{};
FDCAN_HandleTypeDef hfdcan1{};

uint8_t HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *) { return HAL_OK; }
uint8_t HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t) { return HAL_OK; }
uint8_t HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t) { return HAL_OK; }
uint8_t HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) {
    ++h->start_calls;
    return HAL_OK;
}
uint8_t HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h) {
    ++h->stop_calls;
    if (h->Instance != nullptr) {
        h->Instance->TXBRP = 0U;
        h->Instance->TXBTO = 0U;
        h->Instance->TXBCF = 0U;
    }
    return HAL_OK;
}
uint8_t HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *) { return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h, uint32_t) { return h->rx_fifo0_fill; }
uint8_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *) { return 1U; }

int main() {
    hfdcan1.Instance = FDCAN1;
    CAN_Init(&hfdcan1, nullptr);
    assert(hfdcan1.start_calls == 1U && hfdcan1.stop_calls == 0U);
    assert(!CAN_Is_Bus_Off_Active(&hfdcan1));
    assert(!CAN_Is_Bus_Off_Latched(&hfdcan1));
    assert(!CAN_Consume_Bus_Off(&hfdcan1));
    assert(CAN_Get_Bus_Off_Count(&hfdcan1) == 0U);

    // 1. Hardware enters Bus-Off while IRQs are masked (ISR has NOT run yet):
    //    live PSR.BO and pending IR.BO MUST immediately report active/latched/consumed!
    Mock_FDCAN1.PSR = FDCAN_PSR_BO;
    Mock_FDCAN1.IR = FDCAN_IR_BO;
    assert(CAN_Is_Bus_Off_Active(&hfdcan1));
    assert(CAN_Is_Bus_Off_Latched(&hfdcan1));
    assert(CAN_Consume_Bus_Off(&hfdcan1));
    // Still active until hardware clears PSR.BO & IR.BO!
    assert(CAN_Is_Bus_Off_Active(&hfdcan1));

    // 2. HAL FDCAN IRQ clears IR.BO and invokes HAL_FDCAN_ErrorStatusCallback with PSR.BO == 1 (Bus-Off entry):
    //    MUST latch software Bus_Off_Latched, count once, and stop/start controller.
    Mock_FDCAN1.IR = 0U;
    HAL_FDCAN_ErrorStatusCallback(&hfdcan1, FDCAN_IT_BUS_OFF);
    assert(CAN_Get_Bus_Off_Count(&hfdcan1) == 1U);
    assert(hfdcan1.stop_calls == 1U && hfdcan1.start_calls == 2U);
    // Hardware exits Bus-Off after restart (PSR.BO -> 0): software latch STILL remains until consumed!
    Mock_FDCAN1.PSR = 0U;
    assert(!CAN_Is_Bus_Off_Active(&hfdcan1));
    assert(CAN_Is_Bus_Off_Latched(&hfdcan1));
    assert(CAN_Consume_Bus_Off(&hfdcan1));
    assert(!CAN_Is_Bus_Off_Latched(&hfdcan1));

    // 3. Recovery transition interrupt (FDCAN_IT_BUS_OFF with PSR.BO == 0):
    //    MUST NOT double-count Bus_Off_Count, MUST NOT set Bus_Off_Latched, MUST NOT stop/start again!
    HAL_FDCAN_ErrorStatusCallback(&hfdcan1, FDCAN_IT_BUS_OFF);
    assert(CAN_Get_Bus_Off_Count(&hfdcan1) == 1U);
    assert(hfdcan1.stop_calls == 1U && hfdcan1.start_calls == 2U);
    assert(!CAN_Is_Bus_Off_Latched(&hfdcan1));

    // 4. Pending IR.BO alone (even before PSR read settles) is also detected fail-closed.
    Mock_FDCAN1.IR = FDCAN_IR_BO;
    assert(CAN_Is_Bus_Off_Active(&hfdcan1));
    assert(CAN_Is_Bus_Off_Latched(&hfdcan1));
    Mock_FDCAN1.IR = 0U;
    assert(!CAN_Is_Bus_Off_Active(&hfdcan1));

    std::puts("PASS: drv_can live PSR.BO/IR.BO detection, Bus-Off entry vs recovery distinction, and ISR latch consume");
    return 0;
}
''',
    encoding="utf-8",
)

exe = out / ("test_drv_can.exe" if os.name == "nt" else "test_drv_can")
cmd = [
    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-missing-field-initializers", "-Wno-unused-parameter",
    "-I", str(out), "-I", str(repo / "User_File"),
    str(test_cpp), str(repo / "User_File/1_Middleware/Driver/CAN/drv_can.cpp"),
    "-o", str(exe),
]
if os.name == "nt":
    cmd.append("-static")
subprocess.run(cmd, check=True)
subprocess.run([str(exe)], check=True)
