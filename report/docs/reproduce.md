# 复现指南

以下命令从仓库根目录执行。先选择要复现的部分：查看报告和数据、离线重建 FPGA，或连接实物工位。各部分使用的环境与输入不同。

## 1. 查看方案与原始结果

- [技术报告](../tech_doc.pdf)：系统设计、实现与实测结果。
- [模型说明](../../data/model/MODEL.md)：权重、量化配置与摘要。
- [指标表](../../metrics/metrics.csv)：测量条件及证据路径。
- [构建验证](../../build/report/portable_rebuild_validation.json)：独立目录工程创建与 RTL 回归记录。

这部分不需要接入硬件。重新绘图时使用 Python、NumPy 和 Matplotlib，命令见 `metrics/scripts/` 对应脚本。

## 2. FPGA 离线构建

源码归档已包含 OpenCores CAN、K26 板卡定义和经过摘要校验的 HLS IP。启用 Vivado 2025.2 后执行：

```bash
python3 build/scripts/rebuild.py check
export XILINX_SETTINGS64=/path/to/Vivado/2025.2/settings64.sh
python3 sim/run_tests.py
bash build/scripts/build_can.sh create
```

`create` 创建工程并生成 Block Design；`bitstream` 继续完成综合、布局布线与位流生成。默认工作目录为 `build/work/`。部署打包还需要 `bootgen` 和 `dtc`。

```bash
bash build/scripts/build_can.sh bitstream
python3 build/scripts/package_can.py
```

已有位流与报告位于 `build/`、`build/report/`。重新生成的产物位于工作目录中。完整步骤见 [KV260 工作流](../../skill/kv260_pl_workcell_ops/README.md)。

## 3. 上位机环境与运行目录

`src/host/app/` 保存应用源码；现场启动程序使用 `aipc_app` 运行目录，并通过 `$HOME/.local/share/aipc-dev/aipc.sh` 选择 SDK、GPU、NPU 等隔离环境。迁到另一台机器时，需要先配置这些环境和路径。

| 运行目录中的位置 | 提交包来源或配置要求 |
|---|---|
| `aipc_app/integration/` | `src/host/app/` 中的模块、`config/` 与启动入口 |
| `aipc_app/src/` | `src/host/app/src/` 中的相机与检测程序 |
| `aipc_app/configs/` | `src/host/app/configs/` 中的配置与模型版本 |
| `aipc_app/reports/` | 运行时输出；校准文件由下表指定，历史进程标记不用迁移 |
| `live_workflow.json.runtime_root` | 指向完整的 `board/arm_runtime/` |
| `live_workflow.json.can_root` | 指向 `board/software/` |
| `live_workflow.json.ssh_helper` | 本机配置的板卡 SSH 入口；账号、密钥和地址由使用者配置 |
| 模型位置 | 检测权重来自 `data/model/`；GPU/NPU 启动配置指向对应文件 |

依赖锁定文件在 `src/host/deploy/`。本地千问基础权重需按 `data/model/MODEL.md` 的模型名和 revision 准备；提交包包含任务 LoRA 适配器，不包含该基础模型的完整权重。运行时采用本地加载。

## 4. 工位标定与几何输入

`data/workcell_reference/` 提供本工位的参考外参、盒底平面、腕部局部标定及几何模型。它们用于理解和复现已有工位；相机或机械臂重新安装后，应使用当前工位的标定结果。

| 配置项 | 参考文件 | 用途 |
|---|---|---|
| `calibration` | `camera_to_base.json` | 相机坐标到机械臂基座坐标变换 |
| `workplane` | `workplane.json` | 托盘支撑平面 |
| `geometry_xml` | `route_geometry.xml` 与 `meshes/` | MuJoCo 路径几何检查 |
| `heights` | `src/host/app/configs/parts_measured_heights_20261023_v1.json` | 零件实测高度 |
| 腕部局部标定 | `servo_config.json` | 运行目录中的 `reports/wrist_online_20261025_160952/servo_config.json` |

模型配置、托盘 ROI、目标槽位和关节方向/偏置也应与当前工位一致。数字孪生读取 `el_a3_legacy.urdf`，其查找路径见 `workcell/twin.py`，模型文件随 `board/arm_runtime/model/` 提供。

准备好运行目录后，先执行只读检查：

```bash
# 只检查提交包文件
python3 src/host/deploy/check_environment.py

# 再检查已经配置的现场运行目录
python3 src/host/deploy/check_environment.py --app-root /path/to/aipc_app
```

该命令检查文件是否存在，不连接相机、SSH 或机械臂。它通过后，再按 [硬件说明](hardware_setup.md)进行设备连接与现场检查。

## 5. 启动上位机

在隔离环境、模型、SSH、标定和设备服务准备完成后：

```bash
AIPC_APP_ROOT=/path/to/aipc_app bash src/host/app/run_console.sh
```

启动脚本打开本机控制台。先查看检测结果与设备状态，再输入“把大螺丝放到左盘”。勾选机械臂处于支撑位后，任务才进入实物执行；故障中途保持时，仍需操作员承托并确认。

## 6. 发布与视频展示

GitHub 首页使用仓库内的实拍图、架构图和控制流程图；PNG 用于直接浏览，SVG 便于缩放。三分钟旁白与剪辑安排见 `report/video/narration_3min.txt`。最终演示视频导出后，可补充视频链接或实际运行片段，图表素材本身不替代实物录像。
