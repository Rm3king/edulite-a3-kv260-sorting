# 头部相机零件检测：ROCm GPU 训练，Ryzen AI NPU 常驻推理

主归属评分项：AI PC 端侧 AI 能力（20）。

## 1. 实现了什么

- **模型**：YOLO11n-seg 四类实例分割（大螺丝 `screw`、大螺母 `nut`、小螺丝 `screw_m6`、小螺母 `nut_m6`），输入 1280 px。
- **训练**：在 AI PC 上用 ROCm PyTorch 训练（Radeon 890M，80 轮，batch 8），训练集 287 张、验证集 147 张（按采集会话划分），数据由本项目在工位上采集并标注。
- **推理**：同一份权重导出 ONNX，经 AMD Quark 做 A8W8 量化（校准用 64 张训练图），在 Ryzen AI NPU 上通过 VitisAI EP 运行；GPU 版本同时加载为自动回退。
- **接入**：头部检测程序 `--backend npu` 为默认；零件三维定位（深度 → 基座坐标）对 GPU 与 NPU 使用同一套代码。

## 2. 为什么把常驻检测放在 NPU

头部检测需要持续运行。若检测和本地 Qwen2.5-VL 都使用 Radeon 890M，大模型生成会增加检测延迟。以下是同条件对照：

| 检测完整单帧延迟（P50 / P95） | 空闲 | VLM 连续生成时 |
|---|---|---|
| GPU（ROCm FP32） | 42 / 67 ms | 133 / 306 ms |
| **NPU（Quark A8W8）** | 102 / 120 ms | **107 / 141 ms** |

采用 **NPU 持续检测、GPU 运行大模型** 的分工后，同载测试中的检测尾延迟较低。头部相机提供初始定位，腕部相机在接近目标后进一步修正局部偏差。下表单独比较两种推理后端的目标一致性。

## 3. 精度（抓取目标层面）

| 验收 | 结果 | 证据 |
|---|---|---|
| 抓取目标三维点：NPU vs 生产 GPU 预测 | 80 帧：GPU 274 个、NPU 273 个目标点，**匹配 270**，可抓判定 270/270 一致；基座坐标偏差 P50 **0.33** / P95 **1.1** / 最大 2.71 mm | `metrics/logs/npu_acceptance_acceptance.json` |
| 原始检测：NPU INT8 vs 同一浮点图（CPU） | 52 帧，216 个检测，匹配 209；框中心偏差 P50 0.51 / P95 2.03 px | `metrics/logs/npu_eval_a8w8_v2_evaluation.json` |
| 检测头解码部分保持浮点 | 框坐标（0–1280）与类别分数（0–1）在同一输出张量里，单一 INT8 尺度会压零分数；解码节点保持浮点后，卷积主体仍在 NPU | `data/model/quantize_a8w8.json` |

分割精度（FP32，验证集 147 张）：掩码 mAP@0.5 0.976，掩码 mAP@0.5:0.95 0.729，框 mAP@0.5:0.95 0.852；
分类掩码 mAP@0.5:0.95：大螺丝 0.853 · 大螺母 0.83 · 小螺丝 0.637 · 小螺母 0.595（`data/model/val_metrics.json`）。

## 4. 单样本延迟分项（batch=1，1920×1080 输入，含全分辨率实例掩码）

| 后端 | 预处理 P50 | 模型推理 P50 | 后处理 P50 | 总延迟 P50 / P95 / P99 |
|---|---|---|---|---|
| GPU（ROCm PyTorch FP32） | 8.7 ms | 29.8 ms | 4.6 ms | 45.4 / 49.1 / 55.3 ms |
| NPU（Quark A8W8） | 16.9 ms | 51.9 ms | 2.5 ms | 72.1 / 109.4 / 131.9 ms |

证据：`metrics/logs/detector_latency_gpu.json`、`detector_latency_npu.json`；复测命令见 `metrics/scripts/detector_latency_breakdown.py` 头部注释。

## 5. 复现入口

模型文件和校验摘要见 `data/model/MODEL.md`。导出、量化、评估和服务化分别在 GPU、Quark 与 Ryzen AI 隔离环境中执行，命令见 [NPU 工作流](../../skill/ryzen_ai_yolo_npu/README.md)。

运行后端由 `src/host/app/config/live_workflow.json` 的 `detector_backend` 选择。`npu` 为当前配置，`gpu` 用于 GPU 推理；切换后仍使用相同的三维定位处理。
