# 头部相机零件检测上 NPU（2026-10-31）

**结论**：生产检测（parts_v2 interim1，YOLO11n-seg 四类，sha `bbe3f400…`）的 AMD Quark INT8 版本在 Ryzen AI NPU 上运行，
精度在抓取目标层面与 GPU 等价；本地视觉大模型生成时 GPU 检测被拖慢 3–4.5 倍，NPU 不受影响。
生产默认已切到 NPU（`integration/config/live_workflow.json` → `"detector_backend": "npu"`，改成 `"gpu"` 即回退），
GPU 模型同时加载为自动回退。

## 为什么放在这里

- 头部检测一直在跑；Qwen2.5-VL（订单理解 + 结果复核）占同一块 Radeon 890M 和约 8 GB 显存。
- 头部相机只负责粗定位，腕部相机闭环修最后的毫米（连续 ±15 mm，停顿 ±35 mm），INT8 的毫米级差异不影响抓取。

## 做法

| 步骤 | 内容 |
|---|---|
| 导出 | `best.pt` → ONNX，输入 1×3×736×1280（与生产一致：1920×1080 缩到 1280×720，补边 114） |
| 量化 | AMD Quark 0.12 A8W8，校准 64 张 parts_v2 训练集图片；检测头的 55 个解码节点保持浮点（框坐标与类别分数同一张量，单一 INT8 尺度会把分数全部量化为 0：全量化版本 216/216 漏检，已保留为 `parts_v2_a8w8_allquant_BROKEN.onnx`） |
| 运行 | onnxruntime 1.27 VitisAI EP（`aipc.sh npu`），性能剖析确认 NPU 分区实际执行（VitisAI 12 / CPU 20 个事件） |
| 接入 | `src/detect_parts_live.py: NpuDetector`（独立进程，tmpfs 传帧，接口与 GPU `Detector.predict` 相同，任何故障自动回退 GPU）；`detect_parts_rgbd_live.py --backend npu`；`integration/workcell/npu_parts.py` |

## 结果

| 验收 | 结果 | 文件 |
|---|---|---|
| 原始检测 vs 浮点（52 帧实拍，未参与训练） | 置信度 ≥0.45 时 1 漏 1 多；中心偏差中位 0.5 px，P95 2.0 px | `eval_a8w8_v2/evaluation.json` |
| **抓取目标三维点 vs 生产 GPU 实际输出**（80 帧、6 个会话、同一模型） | 274 / 273 个三维点，**匹配 270，可抓判定 270/270 一致**；偏差中位 **0.33 mm**，P95 **1.1 mm**，最大 2.7 mm（一颗小螺丝） | `acceptance/acceptance.json` |
| 实时对照（同一场景各跑 40 s） | 四个零件三维位置相差 0.3–1 mm；NPU 单帧 61 ms，无回退 | `../npu_live_check_20261031/` |
| **与 VLM 争用**（检测完整单帧延迟） | GPU：空闲中位 42 / P95 67 ms → VLM 生成时 **133 / 306 ms**；NPU：空闲 89 / 101 ms → VLM 生成时 **107 / 141 ms** | `contention/*.json` |

## 如实说明

- 空闲时 GPU 检测更快（42 ms vs 89 ms）；NPU 的意义在于和大模型同时运行时稳定，而非单独比快。
- 检测循环本身约 4–5 帧/秒（受深度几何与保存限制），NPU 与 GPU 循环帧率相近（4.1 vs 4.7）。
- 量化只用了 64 张训练图校准；未做逐类 mAP 重新评估，验收以抓取目标三维点为准。
