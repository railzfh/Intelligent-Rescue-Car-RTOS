# RTOS 迁移

`dm02_test.ioc` 使用 STM32CubeMX 6.15.0、STM32Cube FW_H7 1.12.1，启用
CMSIS-RTOS2 / FreeRTOS 10.3.1，保留原 CMake 固件目标。

| 执行上下文 | 职责 |
| --- | --- |
| `controlTask`，High，1 ms | ADC 超时检查、供电联锁、按键、LADRC 定距直行、两路 CAN 发送，完成后喂独立看门狗 |
| `imuTask`，AboveNormal，1 ms | BMI088 静止校准、SPI2 采样、EKF 姿态与航向发布；初始化失败后每 3 s 重试 |
| `statusTask`，Normal，10 ms | 启动时初始化 USB CDC，随后刷新 WS2812 |
| 硬件中断 | CAN 接收解码、ADC DMA 半缓冲处理、TIM5 时间戳溢出及原有外设回调 |

控制与状态任务各有 2048 B 栈，IMU 任务 4096 B，沿用 12 KiB RTOS heap。
周期使用 RTOS 绝对 tick，错过周期不连续补发 CAN 命令。
SysTick 只驱动 FreeRTOS，TIM6 驱动 HAL 毫秒时基。TIM4、TIM7 和 TIM8 保留配置，但不再启动空回调
或用其调度任务。DMA、串口、FDCAN、EXTI 等相关 IRQ 的抢占优先级从 0 调整为 5，
与 CubeMX 的 FreeRTOS 中断配置一致；引脚、外设参数、时钟树、DMA 映射和 CAN 波特率不变。
启动保持零扭矩。有效电池、两轮反馈、最新姿态、静止条件及 CAN 发送条件均满足后短鸣一次，
重新按 USER KEY 才起步；再次按键减速停车。初始化期间提前按键不排队，复位时按住须先松开。

迁入实车版本的两轮 LADRC、编码器定距和航向纠偏：右 CAN1/ID1 方向 -1，左 CAN2/ID2 方向 +1；
最高 50 rpm，斜坡 50 rpm/s，扭矩上限 0.9 Nm，目标 1 m。轮直径 58 mm，里程比例系数 0.90。
电池/电机/姿态超时或 CAN 发送故障使直行任务停止，恢复反馈后不自行起步。
控制任务一次性快照电机和 IMU 数据，状态任务读取控制任务发布的灯色。

IMU 任务在 freertos.c 的 USER CODE 区创建，不增加 CubeMX 生成任务或修改 .ioc。
校准等待使用 osDelay 让出 CPU，并按 1 kHz tick 留出至少规定的寄存器等待时间。
姿态数据发布后才允许按键起步；SPI6 灯光仍由状态任务独占。TIM7 不启动，避免重复电机控制。

## 生成和构建

- 在隔离副本中从 `.ioc` 生成。保持 `statusTask` 位于任务列表第一项：当前 CubeMX 会将 USB CDC
  初始化放在第一项的入口，任务的运行优先级仍由各自配置决定。
- 本机 CubeMX CLI 在基线和候选均有 CMake 路径拼接错误，影响 `Core/Src/syscalls.c`、
  `Core/Src/sysmem.c` 以及生成元数据中的源文件引用。保留原版两个 C 文件，修正生成元数据中的
  绝对路径和重复项，再用仓库的 CMake 配置构建。无需修改全局 CubeMX 配置。
- 使用 `cmake --preset Debug` 和 `cmake --build --preset Debug`。CubeMX 生成后检查
  `main.c` 和 `freertos.c` 中的 `USER CODE` 是否保留。

## 验证范围

- 生成时无 NVIC/串口未就绪警告。对照基线检查了 6 个串口的波特率、11 个串口 DMA 请求，
  以及 ADC1、SPI2 DMA 的 stream、HAL 句柄绑定和 IRQ 调用。
- IOC 结构检查通过；数据库检查仍报基线已有的 5 条 USART 模式字段提示。
  生成日志检查仍报告上述路径错误及本机包扫描问题，不是无警告、无错误的生成验收。
- 干净 Debug 配置、编译及链接通过，Flash 77,236 B，RAM_D1 247,552 B。
  ELF 中已核对 SVC、PendSV、SysTick、TIM5、TIM6 和两个任务入口。
  25 项仓库测试及电池桌面逻辑测试通过。
- 上述验证为 RTOS 基线记录。此次直行移植额外执行干净 Debug 构建、直行/电池桌面测试、
  BMI088 实际驱动和 EKF 模拟测试，以及直接提取任务代码的按键/故障联锁测试。
  桌面命令：

  ```powershell
  python tests/firmware/run_rtos_drive.py
  python tests/firmware/run_bmi088.py --dsp-source <CMSIS-DSP/Source/MatrixFunctions>
  ```

- 本次移植未烧录。上板时先不按 USER KEY，验证 USB 枚举、两路 CAN 命令周期与零电流值、任务栈余量、
  看门狗、ADC 电压和 LED、欠压锁定及 PC13/PC14 供电使能实际状态。
