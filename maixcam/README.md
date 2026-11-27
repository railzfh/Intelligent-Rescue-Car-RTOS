# MaixCAM 触屏版视觉与选边程序（支持 YOLO11n + 轻量 LAB 混合模式）

本目录为运行在 **MaixCAM 触屏版（SG2002 / MaixPy v4）** 上的视觉识别、触屏选边/出发区设置与 `UART1` 二进制通信程序，与 DM-MC02 STM32 固件共用协议 v1。

## 1. 硬件接线（必须核对）

| MaixCAM 引脚 | DM-MC02 引脚 | 说明 |
| --- | --- | --- |
| `A19`（`UART1_TX`） | `PA10`（`USART1_RX`） | 3.3 V TTL |
| `A18`（`UART1_RX`） | `PA9`（`USART1_TX`） | 3.3 V TTL |
| `GND` | `GND` | 共地 |
| 独立 5V BEC | **不与 MC02 `USART1` 跨接供电** | 防止舵机堵转拉塌 MaixCAM 供电 |

## 2. 视觉后端模式（混合 YOLO 优先，一键回退 LAB）

在 `maixcam/config.json` 或赛前触屏右下角 `VIS:*` 按钮中可切换 3 种模式（发车锁定后禁止修改）：

| 模式 | 职责分工 | 适用场景 |
| --- | --- | --- |
| `hybrid`（默认推荐） | **YOLO NPU** 检绿/黑/橘/浅蓝块与对手车；**轻量 LAB** 检红/蓝安全区与紫坡边界，并在低头近视 ROI 内保留 **LAB 浅蓝危险块 ∨ 未知障碍兜底** | 陌生赛场光照、黑块阴影干扰、同色物块贴合计数 |
| `yolo` | YOLO 主检 + LAB 红/蓝区与紫坡（关闭近视 LAB 浅蓝/未知兜底） | 赛场地板反光严重、LAB 浅蓝/地板误报偏高时 |
| `lab` | 原始纯 LAB 流程（`extract_blobs_from_maix_image`） | 未部署模型或比赛现场物块外观与训练集偏差较大时的 3 分钟紧急回退 |

> 若未找到 `yolo_model_path`（默认 `/root/models/rescue_yolo11n.mud`）且 `yolo_fallback_to_lab=true`，程序会自动安全回退至 `LAB(FB)` 并在触屏 HUD 琥珀色提示。

## 3. 部署与启动

将 `maixcam/` 目录拷贝至 MaixCAM（例如 `/root/rescue/maixcam/`），模型文件（`.mud` 与 `.cvimodel` 同目录）拷贝至 `/root/models/`，然后启动：

```bash
python3 -m maixcam.main
```

右侧触屏按钮：
- `RED` / `BLUE`：选择本轮红/蓝方（DM-MC02 板载 WS2812 同步回显队色）。
- `Z1`~`Z4`：选择抽签出发区。
- `MATCH` / `BENCH 1M`：切换正式比赛模式与原版 1 m 直行台架测试模式。
- `MAX MAT: 1..3`：首单解锁后每趟物资混运上限（默认 `1`，稳定后再开 `2` 或 `3`）。
- `GATE` / `PITCH`：仅在 `BENCH 1M` 未发车时生效，用于台架核验右倒 L 杆开合和相机远/近俯仰端点。
- `VIS:HYB` / `VIS:YOLO` / `VIS:LAB`：赛前切换视觉后端（若模型缺失会显示 `VIS:LAB(FB)`）。

裁判发令后按下 DM-MC02 实体按键 `PA15`，MC02 锁定配置并启动比赛，触屏按钮随之自动锁定。

## 4. YOLO11n 数据采集、训练与 INT8 量化部署流程

### 4.1 在 MaixCAM 上采集实景数据

推荐将 `config.json` 中分辨率设为 `320×224`（与 MaixCAM YOLO11n 默认输入一致，零缩放开销，且所有默认近视 ROI `y2 <= 224` 均已兼容）：

```bash
python3 -m maixcam.capture_dataset
```

采样要点：
- 同时覆盖 **抬头远视（FAR）** 与 **低头近视（NEAR）**，以及不同色温/顶灯/侧窗光/阴影条件。
- 重点拍摄 **2~3 个同色或混色物块紧贴/半遮挡**、**物块压在红蓝区或紫坡上**、**浅蓝危险块只露一角** 的画面。
- 加入 15%~20% **纯负样本**（空地板、减速带、紫坡、车体履带阴影、围挡外人群脚步）。

### 4.2 PC 端训练与导出 ONNX

类别映射模板见 [yolo_dataset.yaml](yolo_dataset.yaml)（`0:green, 1:black, 2:orange, 3:cyan, 4:opponent`）。训练时务必**限制色相抖动 `hsv_h=0.01`**，避免绿/浅蓝、橘/红串类：

```bash
yolo detect train \
  model=yolo11n.pt \
  data=maixcam/yolo_dataset.yaml \
  imgsz=320 \
  epochs=120 \
  batch=32 \
  hsv_h=0.01 hsv_s=0.5 hsv_v=0.45 \
  degrees=8.0 translate=0.1 scale=0.4 fliplr=0.5

yolo export \
  model=runs/detect/train/weights/best.pt \
  format=onnx \
  imgsz=224,320 \
  opset=17 \
  simplify=True \
  dynamic=False
```

### 4.3 INT8 量化为 `.cvimodel` + `.mud`

使用 Sipeed **MaixHub 在线模型转换** 或本地 **TPU-MLIR** 将 ONNX 转为 SG2002（MaixCAM）INT8 模型：
- 从真实训练集抽取 **100~200 张**（涵盖远近视角与明暗光）作为 INT8 校准集。
- 将生成的 `rescue_yolo11n.mud` 与对应 `.cvimodel` 一并上传至 MaixCAM `/root/models/`。
- 在 `maixcam/config.json` 中确认：
  - `"frame_width": 320, "frame_height": 224`
  - `"vision_backend": "hybrid"`, `"yolo_arch": "yolo11"`, `"yolo_model_path": "/root/models/rescue_yolo11n.mud"`
  - `"yolo_dual_buff": false`（默认关闭以保证严格单帧同步；若追求更高帧率可设为 `true`，`main.py` 会自动把俯仰/开合杆到位排空深度 `+1` 并用上一帧图像对齐 LAB 与光流）。

## 5. 赛场 3 分钟调试标定

在 `maixcam/config.json`（若不存在可调用 `RescueVisionConfig().save(Path("maixcam/config.json"))` 生成）中快速核对：
1. `lab_thresholds`：混合模式下重点只需核对 **`red_zone`、`blue_zone`、`purple_ramp`**（以及近视兜底的 `cyan` 与 `arena_floor_lab`）；纯 `lab` 回退时才需全量标定。
2. `near_corral_roi`、`near_sweep_roi`、`near_drop_roi`、`near_ramp_band_y`：在低头近视档位下对齐实际围框、右倒 L 杆扫掠区及紫坡内沿像素区域（触屏预览会直接叠画 ROI 框与检测框）。
3. `material_half_on_left_red` / `material_half_on_left_blue`：核对本场红蓝安全区中“物资区/伤员区”的左右分布。
