# Skill：YOLO11n-seg → Ryzen AI NPU（Quark A8W8）

## 适用范围

以本项目四类零件的YOLO11n-seg分割模型为部署案例，将持续检测放到NPU，GPU留给本地大模型。当前脚本固定四类名称、736×1280输入和检测头节点 `/model.23/`；迁移到其他类别或模型时，先适配这些配置。

环境为Ryzen AI Software 1.8.0、onnxruntime 1.27（VitisAI EP）、AMD Quark 0.12。训练与导出使用ROCm/PyTorch环境，量化与NPU评估使用对应隔离环境。下列 `python` 均指已激活环境中的解释器，不要求安装本项目私有的 `aipc.sh` 启动器。

## 使用方法

从提交代码包根目录运行；示例中 `/path/to/` 替换为实际模型、训练图像或测试帧路径。

1. 导出ONNX，在训练/导出环境执行：

   ```sh
   yolo export model=/path/to/best.pt format=onnx imgsz=[736,1280] opset=17 simplify=False
   ```

2. 在Quark环境量化，校准数据使用训练集图像：

   ```sh
   python skill/ryzen_ai_yolo_npu/scripts/npu_parts.py quantize --model /path/to/best.onnx --output work/npu_quant --precision A8W8 --calibration /path/to/train/images --calibration-count 64
   ```

   脚本对检测头解码部分保持浮点，对卷积主体量化。输出为 `work/npu_quant/parts_v2_a8w8_736x1280.onnx`。

3. 在已配置XRT与VitisAI EP的NPU环境评估：

   ```sh
   python skill/ryzen_ai_yolo_npu/scripts/npu_parts.py evaluate --float /path/to/best.onnx --quant work/npu_quant/parts_v2_a8w8_736x1280.onnx --frames /path/to/frame_01.png /path/to/frame_02.png --output work/npu_eval
   ```

   检查 `evaluation.json` 中匹配数量和 `profile_node_events_by_provider`：必须有VitisAI实际执行事件。此步骤比较NPU量化图与CPU浮点ONNX图；GPU抓取目标级对照是下一步的独立验收。

4. 服务化入口：

   ```sh
   python skill/ryzen_ai_yolo_npu/scripts/npu_parts.py serve --model work/npu_quant/parts_v2_a8w8_736x1280.onnx --output work/npu_service
   ```

   服务通过stdin/stdout JSON行传递请求与结果，图像按路径读取。主程序接入点为 `src/host/app/src/detect_parts_live.py` 中的 `NpuDetector`，GPU回退由主程序管理。

5. 抓取目标级验收使用 `src/host/app/src/npu_detector_acceptance.py`，需同目录的检测与几何模块、RGB-D回放数据和标定配置。`scripts/` 中的同名文件是配套源码副本，不单独构成完整验收环境。

6. 延迟分项使用 `metrics/scripts/detector_latency_breakdown.py`；同载测试从应用根目录按模块调用 `python -m workcell.npu_contention`，需同包的 `order_language.py`、模型路径及环境。命令参数先用 `--help` 查看。

## 失效条件

- 检测头整体量化会使类别分数与框坐标共用尺度，可能导致零检测；检查解码节点排除配置和匹配数。
- 运行时不支持较新的ONNX IR版本时，本脚本将IR版本降至10并检查模型。
- XRT或VitisAI EP环境未正确配置时，不能只凭程序运行成功认定NPU执行；以provider和profile事件为准。
- 本项目验收覆盖“粗定位＋腕部闭环纠偏”的系统，迁移时按目标任务重新确定精度要求。

## 本项目验证

四类模型与GPU输出匹配 270/274 个抓取目标，两后端坐标差值 P50 0.33 / P95 1.1 mm；大模型同载时NPU检测 P95 141 ms，GPU为 306 ms。证据位于 `metrics/logs/npu_acceptance_acceptance.json` 及 `npu_contention_*.json`。
