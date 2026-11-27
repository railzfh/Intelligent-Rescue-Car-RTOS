# MaixCAM 触屏版视觉与选边程序

本目录为运行在 **MaixCAM 触屏版（MaixPy v4）** 上的视觉识别、触屏选边/出发区设置与 `UART1` 二进制通信程序，与 DM-MC02 STM32 固件共用协议 v1。

## 1. 硬件接线（必须核对）

| MaixCAM 引脚 | DM-MC02 引脚 | 说明 |
| --- | --- | --- |
| `A19`（`UART1_TX`） | `PA10`（`USART1_RX`） | 3.3 V TTL |
| `A18`（`UART1_RX`） | `PA9`（`USART1_TX`） | 3.3 V TTL |
| `GND` | `GND` | 共地 |
| 独立 5V BEC | **不与 MC02 `USART1` 跨接供电** | 防止舵机堵转拉塌 MaixCAM 供电 |

## 2. 部署与启动

将 `maixcam/` 目录拷贝至 MaixCAM（例如 `/root/rescue/maixcam/`），通过以下入口启动：

```bash
python3 -m maixcam.main
```

启动后右侧触屏提供：
- `RED` / `BLUE`：选择本轮红/蓝方（DM-MC02 板载 WS2812 同步回显队色）。
- `Z1`~`Z4`：选择抽签出发区。
- `MATCH` / `BENCH 1M`：切换正式比赛模式与原版 1 m 直行台架测试模式。
- `MAX MAT: 1..3`：首单解锁后每趟物资混运上限（默认 `1`，稳定后再开 `2` 或 `3`）。
- `GATE` / `PITCH`：仅在 `BENCH 1M` 未发车时生效，用于台架核验右倒 L 杆开合和相机远/近俯仰端点。

裁判发令后按下 DM-MC02 实体按键 `PA15`，MC02 锁定配置并启动比赛，触屏按钮随之自动锁定。

## 3. 赛场 3 分钟调试标定

在 `maixcam/config.json`（若不存在可调用 `RescueVisionConfig().save(Path("maixcam/config.json"))` 生成）中调整：
1. `lab_thresholds`：按现场灯光标定绿/黑/橘/浅蓝块、红蓝安全区和紫色围栏 LAB 阈值。
2. `near_corral_roi`、`near_sweep_roi`、`near_drop_roi`、`near_ramp_band_y`：在低头近视档位下对齐实际围框、右倒 L 杆扫掠区及紫坡内沿像素区域。
3. `material_half_on_left_red` / `material_half_on_left_blue`：核对本场红蓝安全区中“物资区/伤员区”的左右分布。
