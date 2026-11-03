# 可复用 Skill 与工作流

| 目录 | 内容 | 适用范围 |
|---|---|---|
| `ryzen_ai_yolo_npu/` | 把 Ultralytics YOLO 分割/检测模型部署到 Ryzen AI NPU：导出 → Quark A8W8 量化 → NPU 评估 → 服务化 → 与 GPU 的抓取目标级验收与自动回退 | 以本项目 YOLO11n-seg 四类模型为案例，迁移时适配类别、输入和检测头 |
| `kv260_pl_workcell_ops/` | KV260 固件的构建、部署、自检、板端运行时打包，以及机械臂工位的启停、清理与安全停止规程 | 以 KV260/K26 + CAN 执行器为基础的工位 |
| `can_wire_timing/` | 校验脚本：逻辑分析仪导出 → CAN 帧解码 → 周期间隔与偏差统计 → 分布图与线上时序图 | 本项目 CAN 扩展帧周期控制；迁移时修改帧分类与周期 |
| `embodied_pitfalls/` | 踩坑清单：机械臂、KV260、相机与构建环境的真实故障，按触发条件、现象、定位、处理整理 | KV260 + CAN 电机 + AI PC 视觉的机械臂系统 |
| `llm_hardware_collab/` | 提示词工作流：智能体授权边界模板、证据优先写作规则、上机检查清单 | 让编码智能体参与真实硬件开发的项目 |

每个 Skill 的 `README.md` 包含：适用范围、使用方法、失效条件、已验证的复用结果。

## 文件组织与已验证结果

这些材料分别以案例模板、工程工作流、离线分析脚本和规则模板提供。使用入口是各目录的 `README.md`；不需要将五个目录全部作为智能体插件安装。

| Skill | 类型 | 本项目验证与证据 |
|---|---|---|
| `ryzen_ai_yolo_npu` | 模型部署案例 | 四类模型量化、NPU运行、抓取目标级对照；`metrics/logs/npu_acceptance_acceptance.json` |
| `kv260_pl_workcell_ops` | 工程工作流 | 固件部署、板级自检与板端启停；`build/report/`、`metrics/logs/rtl_regression_validation.json` |
| `can_wire_timing` | 离线校验脚本 | 15364个5 ms周期的总线统计；`metrics/logs/bus_timing.json`与`bus_frames.npz` |
| `embodied_pitfalls` | 故障排查清单 | 17条项目故障的触发、定位和处理方法 |
| `llm_hardware_collab` | 规则与提示词模板 | 硬件协作、日志分析及文档检查流程；`report/llm_collaboration.md` |

各技能的README提供使用入口、环境配置和操作步骤。KV260构建入口从提交包自动展开工程源码与依赖，并校验文件摘要。
