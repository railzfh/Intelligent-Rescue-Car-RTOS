# 智能救援赛项整车系统架构与推进规划

本文档对应 **智能+工程创新赛道 — 智能救援赛项**，基于仓库当前 FreeRTOS 基线（`DM-MC02 / STM32H723VGTx` + 双 `M2006/C610` + `BMI088` + `6S` 电池监测），定义已确认硬件方案下的整车软硬件架构、通信协议、比赛状态机与分阶段验收标准。

## 1. 已确认硬件方案与赛题硬约束

### 1.1 机械与视觉配置

- **履带底盘**：双 `M2006 + C610` 独立驱动，右轮 `FDCAN1 / ID 0x201`（前进方向 `-1`），左轮 `FDCAN2 / ID 0x202`（前进方向 `+1`），轮径 `58 mm`，里程系数 `0.90`。
- **物块转运机构（合规无夹持围框）**：
  - 前部固定竖直推板（无前伸托底唇边，严禁将目标托离地面放置于车体上）。
  - 左侧固定挡杆。
  - 右侧单舵机驱动可抬放**倒 L 型杆**：放下后形成松散长方形围框，抬起后保留前推板 + 左杆组成的开放 L 型区，用于单件剥离与过坡卸货。
- **单轴俯仰视觉云台**：**MaixCAM 触屏版** 下方仅配置 **1 个俯仰（Pitch）三线 PWM 舵机**（不做偏航舵机，偏航由履带车原地转向实现），支持**抬头远视**（找物块/找安全区/避对手）与**低头近视**（盯紧车前围框计数、倒 L 杆扫掠区避险与紫线卸货确认）双视角接力。

### 1.2 竞赛红线（写入下位机状态机强约束）

| 红线条款 | 固件/视觉强制执行策略 |
| --- | --- |
| 首单必须单独运 **1 个绿色普通物资** 完整进入本方物资半区 | `first_green_unlocked == false` 时仅允许追踪绿块；合框前近视计数必须严格为 `green=1, black=0, orange=0, cyan=0, unknown=0`；卸货倒车后须连续确认目标轮廓完整越过紫线内沿才解锁 |
| 后续单趟最多 **3 个** 物资（绿/黑可混运），伤员（橘色）必须**单独 1 个** | `Validate_Corral_For_Close()` 硬拦截任何 `orange > 0` 且总数 `> 1`、任何物资总数 `> max_materials_per_trip`（硬上限 `3`） |
| 危险目标（浅蓝）进入任一安全区（含紫坡围栏）或出界即结束本轮 | 远视永不选浅蓝块；近视只要 `corral_cyan_cnt > 0`、`!corral_safe` 或 `!sweep_clear`，严禁合框与推坡，立即抬杆直线后退剥离 |
| 物资与伤员进错本方半区（含部分进入）扣 10 分/个 | 推坡卸货前硬校验 `home_valid && home_half_matches_cargo` |
| 严禁进入对方安全区（含压围栏）、禁止主动进攻 | `opponent_zone_near` 或 `opponent_near` 立即中止逼近并后退转向绕避 |
| 严禁抓取或托载目标 | 仅使用松散围框贴地推行；倒 L 杆不夹紧、前推板无底托 |
| 停滞 **15 s** 判负、每轮运行 **3 分钟** | 堵转（高扭矩低轮速持续 `>= 600 ms`）或视觉打滑停滞（`>= 800 ms`）立即进入 `ESCAPE_RECOVERY` 有界退让换向；运行计时满 `180 s` 自动停机 |
| 裁判口令后仅按一次键启动，之后禁止触碰车体或电脑 | 赛前在 MaixCAM 触屏选红/蓝方和 `1~4` 号出发区，MC02 用 WS2812 与遥测回显确认；按一次 `PA15 USER KEY` 发车后锁定配置，忽略后续配置修改 |

## 2. DM-MC02 硬件引脚与供电映射

| 功能 | MC02 接口/引脚 | 定时器/外设参数 | 备注 |
| --- | --- | --- | --- |
| 右侧倒 L 杆舵机 | `PE9`（`SERVO_1__PWM`） | `TIM1_CH1`，`PSC=240-1`，运行时重设 `ARR=20000-1`（`50 Hz`，`1 tick = 1 us`）并开启 `OC1PE` | 软限位 `[900, 2100] us`，需台架实测标定 `OPEN/CLOSED` 脉宽 |
| MaixCAM 俯仰舵机 | `PA0`（`SERVO_3__PWM`） | `TIM2_CH1`，`PSC=240-1`，运行时重设 `ARR=20000-1`（`50 Hz`，`1 tick = 1 us`）并开启 `OC1PE` | 软限位 `[900, 2100] us`，需台架实测标定 `FAR/NEAR` 脉宽 |
| 备用舵机通道 | `PE13`（`TIM1_CH3`）、`PA2`（`TIM2_CH3`） | 保留未启用 | 本方案不使用 |
| MaixCAM 串口通信 | `PA9 (TX)` / `PA10 (RX)` | `USART1`，`115200 8N1`，DMA1 Stream5 RX（Idle/TC 双缓冲，禁用 HT 避免半传输错换缓冲）/ Stream6 TX | 对接 MaixCAM `UART1`（`A19 TX -> PA10 RX`，`A18 RX <- PA9 TX`，仅接 TX/RX/GND） |
| 实体一键启动键 | `PA15`（`KEY__INPUT`） | 低有效，25 ms 消抖，上电按住须先松开 | `PRE_MATCH` 启动比赛；运行中按下执行安全减速停车 |
| 状态回显灯 | `PA7`（`WS2812` / `SPI6`） | 10 ms 刷新（`statusTask` 独占） | 欠压/故障恒红优先；未选边显示电量红绿渐变；选定红/蓝方后回显队色 |
| 蜂鸣器 | `PB15`（`TIM12_CH2`） | 就绪短鸣提示 | 电量/IMU/电机/CAN 全就绪时短鸣一次 |
| 电池供电联锁 | `PC4`（`ADC1`）、`PC14/PC13`（`DC24_0/1`）、`PC15`（`DC5`） | 6S 低于 `19.8 V` 锁死停机 | **供电红线**：板载 `PC15` 5V 轨总上限仅 `2 A`；两个三线舵机与 MaixCAM 须由唯一车载 6S 电池分出独立 5V BEC 供电（共地不共正），固件默认保持 `DC5` 关闭且欠压必断 |

> **关于 `TIM1`/`TIM2` 50 Hz 配置**：`dm02_test.ioc` 默认生成 `ARR = 10000-1`（100 Hz）且关闭 `OCxPE`。固件在 `Class_PWM_Servo::Init()` 中安全停止通道后改写 `ARR = 20000-1`、开启 `OCxPE`、预装初始安全脉宽并触发 `TIM_EVENTSOURCE_UPDATE` 后再启动 PWM，避免无预装导致的中途毛刺脉冲，同时不修改 `.ioc` 避免 CubeMX 路径拼接问题。

## 3. RTOS 任务分配（严格控制在 12 KiB Heap 内）

沿用现有三任务结构，不额外创建高栈消耗通信线程：

| 任务 | 优先级 / 周期 | 职责 |
| --- | --- | --- |
| `controlTask` | `High` / `1 ms` | ADC 超时与供电联锁、非阻塞查询 `USART1` RX 健康与恢复纪元（异常或跨周期恢复时清空旧环缓冲/复位协议解析器/失效故障前缓存视觉并立即抑制运动）、消费 `USART1` 无锁字节环缓冲并解析帧、按键消抖、任务状态机与底盘差速/定距/定角/堵转控制（倒车停机联锁要求左右双电机在停机请求后各自回报新 CAN 静止样本才释放）、电池校验通过后启动双舵机并执行 1 ms 缓动插补、双路 CAN 发送、遥测快照发布、喂 IWDG |
| `imuTask` | `AboveNormal` / `1 ms` | `BMI088` 静止校准、SPI2 采样、EKF 姿态与相对航向发布 |
| `statusTask` | `Normal` / `10 ms` | 初始化 USB CDC、10 ms 刷新 WS2812 灯光、执行 `USART1` RX DMA 阻塞恢复（含 `HAL_DMA_STATE_ERROR` 显式 DeInit/Init 与上限锁存）、每 50 ms（`20 Hz`）在 `huart1` DMA 空闲时发送遥测帧（静态缓冲区） |
| 硬件中断 | 抢占优先级 `5` | CAN 接收解码、ADC DMA 半缓冲处理、`USART1` DMA-to-Idle 接收并将字节快速压入无锁环缓冲、TIM5 时间戳溢出 |

## 4. MaixCAM ↔ STM32 二进制串口协议 v1

### 4.1 帧格式（小端序，无浮点字段）

```text
+------+------+-----+------+-----+-----+---------------+--------+
| 0xAA | 0x55 | ver | type | seq | len | payload[len]  | crc16  |
| 1 B  | 1 B  | 1 B | 1 B  | 1 B | 1 B | 0..64 B       | 2 B LE |
+------+------+-----+------+-----+-----+---------------+--------+
```

- `ver = 0x01`，`len <= 64`。
- `crc16`：CRC16-CCITT（多项式 `0x1021`，初值 `0xFFFF`，无反射，无终异或），覆盖 `ver`、`type`、`seq`、`len` 与 `payload`。

### 4.2 消息定义与新鲜度规则

1. `0x01 MSG_HEARTBEAT`（MaixCAM → STM32，8 B）：`uptime_ms(u32)`, `camera_ok(u8)`, `ui_state(u8)`, `reserved(u16)`。
   - **新鲜度硬约束**：心跳仅更新链路存活标志，**绝不刷新视觉观测时间戳，绝不续期运动控制**。
2. `0x02 MSG_SET_CONFIG`（MaixCAM → STM32，4 B）：`team(u8: 0未定/1红/2蓝)`, `start_zone(u8: 1..4)`, `run_mode(u8: 0比赛/1台架1m直行)`, `max_materials_per_trip(u8: 1..3)`。
   - 仅在 `PRE_MATCH` 未发车时生效；一键发车后配置立即锁定。
3. `0x03 MSG_VISION_STATE`（MaixCAM → STM32，20 B，推荐 `20 Hz`）：
   - `frame_seq(u16)`：相机单调递增帧号（16 位回绕安全）；**重复或倒退的旧 `frame_seq` 绝不刷新视觉新鲜度**（仅在视觉真实断联 `>= 1000 ms` 后允许相机重启从 `0..5` 重新同步）。
   - `camera_view(u8)`：`0=TRANSITION`, `1=FAR`, `2=NEAR`；舵机切换期间 STM32 强制丢弃不匹配视角的观测。
   - `target_type(u8)`：`0=NONE`, `1=GREEN`, `2=BLACK`, `3=ORANGE`, `4=CYAN`。
   - `flags(u16)`：位掩码涵盖目标有效（bit 0）、本方区有效（bit 1）、半区匹配（bit 2）、紫坡可见（bit 3）、框内无危险块（bit 4）、右杆扫掠区清空（bit 5）、卸货完整入区确认（bit 6）、对手临近（bit 7）、对方安全区临近（bit 8）、后方清空（bit 9）、视觉停滞打滑（bit 10）、推进路径无危险/未知物（`VIS_FLAG_DROP_PATH_CLEAR`，bit 11）、开杆到位遥测回执（`VIS_FLAG_GATE_OPEN_SETTLED_ACK`，bit 12）、卸货前基线就绪（`VIS_FLAG_DROP_BASELINE_READY`，bit 13）。
     - `VIS_FLAG_DROP_PATH_CLEAR`：连续路径安全标志，确认本方区、关联紫坡、卸货区及前向推进走廊内无浅蓝危险块或未知前景；推进中自有合法货块移出围框边界不会误清该位，延迟的 `TEL_FLAG_DROP_PUSHING` 遥测不会误中止正常推进，但任一浅蓝/未知物出现即刻清零停机。
     - `VIS_FLAG_DROP_BASELINE_READY`：推进前基线授权标志，要求未进入显式推进态、本趟基线已清洁锁定且紫坡/中线无存量压线物块、当前框内货型与基线一致且路径无危险；MCU 在 `MISSION_DROP_OPEN_AND_PUSH` 的关杆核验与开杆等待阶段同时校验该位与 `VIS_FLAG_DROP_PATH_CLEAR`。
     - `VIS_FLAG_GATE_OPEN_SETTLED_ACK`：**证明内容**：MaixCAM 已收到 MCU 遥测 `TEL_FLAG_GATE_CLOSED=0 && TEL_FLAG_GATE_SETTLED=1`，且 `VIS_FLAG_DROP_BASELINE_READY` 为真；MCU 另在 `gate_settled` 上升沿锁存 `Gate_Settled_Frame_Seq`，仅接受严格晚于该帧号且携带该 ACK 的近视帧才启动推进。**失效条件**：任一危险/未知物出现、货型变化、进入显式推进态（`TEL_FLAG_DROP_PUSHING`）、关杆/未到位、切回远视、离开卸货状态或超过 `vision_timeout_ms (300 ms)` 即刻失效。
   - `target_err_x_mrad(i16)`、`target_dist_mm(u16)`、`home_err_x_mrad(i16)`、`home_dist_mm(u16)`（左偏为正，与车体 `+omega` 左转一致）。
   - `corral_{green,black,orange,cyan,unknown}_cnt(u8 × 5)`、`confidence(u8: 0..100)`。
   - **超时保护**：视觉有效超时默认 `300 ms`，超时即降速/停车，禁止盲跑。
4. `0x04 MSG_BENCH_ACTUATOR`（MaixCAM → STM32，4 B）：`gate_cmd(u8)`, `pitch_cmd(u8)`, `reserved(u16)`。仅在 `PRE_MATCH` 且 `run_mode == BENCH_STRAIGHT` 下生效，用于赛前单舵机端点台架标定。
5. `0x81 MSG_TELEMETRY`（STM32 → MaixCAM，28 B，`20 Hz`）：回传运行时间、比赛计时、状态机状态、已确认队色/出发区/模式、状态位（含 `TEL_FLAG_DROP_PUSHING` bit 13 显式标识卸货推进阶段，供视觉冻结本趟卸货基线）、故障位（bit 0 电池、`bit 1` 电机超时、`bit 2` IMU 超时、`bit 3` CAN 发送、`bit 4` 视觉超时、`bit 5 FAULT_FLAG_SERVO` 舵机 PWM 启动/锁存故障、`bit 6 FAULT_FLAG_UART_RX` UART1 RX DMA 异常或恢复锁存故障）、电池毫伏、相对航向 `0.01 deg`、当前速度与角速度、已交付各类计数及每趟上限。

## 5. 比赛状态机与脱困流程

```text
PRE_MATCH (触屏选边/出发区, WS2812回显)
   │  [一键按 PA15 + 全硬件就绪 + 队色已选]
   ▼
LAUNCH_CROSS_BUMPS (锁定配置, 抬右杆/抬头远视, 定距航向保持越过3根10mm减速带)
   ▼
SEARCH_TARGET (远视慢速原地扫描, 首单仅找绿块; 解锁后优先橘>黑>绿)
   ▼
APPROACH_TARGET (远视低速视觉伺服逼近, 遇对手或对方安全区立即退避)
   ▼
INSPECT_CORRAL (停车 → 切低头近视等稳定 → 连续多帧核验框内计数与右杆扫掠区)
   ├─ 含浅蓝/未知块/超量/首单非单绿/伤员混装 ──► BACK_OUT_SEPARATE (保持抬杆后退+偏角重进)
   ▼ 校验合法
CLOSE_GATE_VERIFY (落右倒L杆等稳定 → 近视复核无误扫 → 锁定本趟货型)
   ▼
FIND_HOME → APPROACH_HOME (切抬头远视找本方安全区 → 对准物资/伤员对应半区)
   ▼
DROP_OPEN_AND_PUSH (停车近视复核半区与无危险块 → 抬右杆 → 慢速直线顶过紫色三角斜坡)
   ▼
DROP_BACKOUT_VERIFY (直线倒车退出 → 近视连续确认物块完整越过紫线内沿)
   ├─ 首单确认完整入区 → first_green_unlocked = true
   └─ 返回 SEARCH_TARGET (满 180 s 进入 MATCH_FINISHED)

任意推进/搜索状态若触发 [底盘堵转 >=600ms] 或 [去均值空间位移视觉打滑 >=800ms]：
   ──► ESCAPE_RECOVERY (仅在有实测已验证前向清空走廊或显式后向传感器许可时有界倒退，全程监测对手与视觉新鲜度 → 交替方向转45°；连续脱困超过 3 次或累计 10 s 无进展立即锁停 FAULT_STOP)
```

## 6. 分阶段开发与上车验收清单

1. **阶段 A（本分支已完成固件与宿主验证）**：
   - `Class_Chassis_Drive`：复用 `Class_Wheel_LADRC`，补全 `(v, ω)` 差速运动学、双向定距直行、BMI088 闭环定角原地转向与堵转检测。
   - `Class_Servo_Controller` / `Class_PWM_Servo`：50 Hz 预装 PWM、软限位、缓动限速与无反馈到位延时估计。
   - `rescue_protocol` + `rescue_mission` + FreeRTOS 集成 + `maixcam/` 触屏视觉程序与跨语言协议单测。
2. **阶段 B（台架无载上电验证 — 未上板前不可跳过）**：
   - 示波器核实 `PE9`、`PA0` 输出周期 `20 ms (50 Hz)`、脉宽范围与上电无毛刺。
   - 实测外接 5V BEC 在两舵机同时堵转起步时的压降，确认不引发 MC02 或 MaixCAM 复位。
   - 用 `BENCH_STRAIGHT` + `MSG_BENCH_ACTUATOR` 逐一标定倒 L 杆 `OPEN/CLOSED` 脉宽、俯仰舵机 `FAR/NEAR` 脉宽，确保不顶机械死点。
   - 核实 `USART1` 与 MaixCAM `UART1` 双向通信、触屏选红蓝方时 MC02 WS2812 同步变色、拔线 `300 ms` 内自动停轮。
3. **阶段 C（实物场地标定与首单闭环实测）**：
   - 在现场光照下更新 `maixcam/config.json` 中 4 类物块、红蓝区、紫色围栏 LAB 阈值及 `FAR/NEAR` 像素标定 ROI。
   - 用实际 3D 打印物块与 `300×60×10 mm` 减速带、`20 mm` 紫色斜坡围栏复测推板离地间隙、越坡推距和倒车复核距离。
   - 先把首单单绿成功率调稳，再开启 15 分橘色伤员单运与 `max_materials_per_trip = 2..3` 物资混运。
