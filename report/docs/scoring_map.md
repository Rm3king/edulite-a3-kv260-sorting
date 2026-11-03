# 评分项与证据对照

下表按评分项列出对应实现与证据入口。

| 评分项（权重） | 本项目交付 | 主归属证据 |
|---|---|---|
| 具身智能系统完整度（20） | 中文指令 → 感知 → 理解 → 规划 → 控制 → 执行 → 头部相机核对 → 复核的完整闭环；四类零件一次会话连续分拣 | `metrics/logs/task_trials.csv`、`report/docs/architecture.md`、`report/design_report.md`、`report/video/` |
| AI PC 端侧 AI 能力（20） | ROCm 训练的四类分割；NPU INT8 常驻检测（抓取目标 270/274 匹配，偏差 P95 1.1 mm）；本地 Qwen2.5-VL 指令理解与结果复核；GPU/NPU 异构分工实测 | `report/docs/npu_detector.md`、`report/docs/vlm_review.md`、`metrics/logs/npu_*`、`metrics/logs/detector_latency_*` |
| FPGA/Zynq 设计质量（25） | PL：5 ms 七次多项式求值与 CAN 调度、截止期队列、守护、七轴看门狗、腕部图像掩码核、在线修正通道；16 个测试平台；资源与时序报告 | `src/fpga/`、`build/report/`、`metrics/logs/board_bench_20261028.json`、`metrics/logs/tracking_latency_loopback_20261028.json` |
| AI PC 与 FPGA/Zynq 协同（20） | 内容寻址作业包、nonce/哈希绑定、PS↔PL 寄存器协议、板端视觉环写 PL 修正通道（对准环不经网络）、故障保持与确认门；RTT、吞吐、丢包实测 | `report/docs/interface.md`、`report/docs/continuous_alignment.md`、`metrics/logs/comm_bench.json` |
| 工程完成度与创新（15） | 板端连续对准（悬停到收敛中位数 1.06–2.43 s）；构建入口与依赖锁定；故障恢复流程；可复用工作流 | `skill/`、`src/host/deploy/`、`report/docs/hardware_setup.md` |

指标汇总：`metrics/metrics.csv`（官方表头，每行一项，证据文件均在包内）。
