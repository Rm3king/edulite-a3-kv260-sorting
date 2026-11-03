# 模型来源、版本、精度与推理后端

## 1. 零件检测（头部相机）

| 项 | 内容 |
|---|---|
| 模型 | YOLO11n-seg，四类：`screw`、`nut`、`screw_m6`、`nut_m6` |
| 来源 | Ultralytics YOLO11n-seg 预训练权重，在本项目工位数据上微调（`src/host/app/src/train_parts_v2.py`），AI PC 上 ROCm 训练，80 轮，batch 8，输入 1280 px |
| 数据 | 训练集 287 张、验证集 147 张（按采集会话划分），数据说明见 `data.yaml`；原图不随包提交 |
| 权重 | `parts_v2_best.pt`，SHA-256 `bbe3f4000909d5ea22fd8c0017db7e004d95f5ba8b687a56cb0da7f787e25c88` |
| 精度（FP32，验证集） | 掩码 mAP@0.5 0.976，掩码 mAP@0.5:0.95 0.729，框 mAP@0.5:0.95 0.852；明细见 `val_metrics.json` |
| GPU 后端 | ROCm 7.14.1，PyTorch 2.12.0+rocm7.14.1，FP32，Radeon 890M，置信度 0.25，NMS IoU 0.7 |
| NPU 后端 | ONNX（`parts_v2_fp32_736x1280.onnx`，SHA-256 `df1f677fbf0ae32aa41b81d87917cba4abd640a767e6dab4ab9d2ed5f6e4979c`）→ AMD Quark 0.12 A8W8 PTQ（`parts_v2_a8w8_736x1280.onnx`，SHA-256 `8248c1f72a20215e374ef451c7dfc0d29f94cd1d01a223267dd530836fda686c`）→ onnxruntime 1.27 + VitisAI EP（Ryzen AI Software 1.8.0）；输入 1×3×736×1280；检测头解码节点保持浮点；校准 64 张训练集图像 |
| 部署选择 | `src/host/app/config/live_workflow.json` 的 `detector_backend`：`npu`（默认）/ `gpu`；NPU 故障自动回退 GPU |

## 2. 本地语言/视觉语言模型

| 项 | 内容 |
|---|---|
| 模型 | `Qwen/Qwen2.5-VL-3B-Instruct`，revision `66285546d2b821cf421d4f5eb2576359d3770cd3`（Hugging Face），bf16，ROCm GPU，常驻进程 |
| 指令理解 | 基础模型 + 任务 LoRA（`task_lora_v2_adapter/`，160 步 ROCm 训练），规则优先、校验层兜底 |
| 结果复核 | 基础模型（不加载 LoRA），输入执行前/后托盘图与运行记录，输出中文说明 |
| 离线 | 权重本地加载（`local_files_only`），运行不访问网络 |

## 3. 固件

`kv260-can.bit.bin` SHA-256 `76034028fb0fb7f812cb5777ce066c9772f5815967d72c7a85b01658337df567`（`build/deploy/`，由 `build/` 脚本从源码重建）。
