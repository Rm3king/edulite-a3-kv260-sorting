# Skill：KV260固件、板端运行时与工位启停规程

## 适用范围

用于KV260/K26的PL CAN控制固件构建、仿真回归、部署打包及CAN执行器工位管理。AI PC通过SSH与专用网口管理板卡。

## 构建准备

需要Linux、Python 3和Vivado 2025.2。RTL回归默认使用Vivado自带XSim，也可选择Icarus Verilog；部署打包还需要 `bootgen` 和 `dtc`（device-tree-compiler）。

提交包的 `build/source_bundle.tar.gz` 包含构建源码、第三方CAN控制器、K26板卡定义、HLS IP及其校验记录。入口自动验证SHA-256并展开到工作目录，无需另行准备开发工程。`src/fpga/` 用于浏览源码；实际编译采用构建归档中的精确副本，保留文件大小写和IP内部引用。

以下命令从提交包根目录运行。先启用工具链，安装路径按实际情况填写：

```bash
export XILINX_SETTINGS64=/path/to/Vivado/2025.2/settings64.sh
python3 build/scripts/rebuild.py check
```

## 构建与回归

```bash
# 运行16项RTL回归（默认XSim）
python3 sim/run_tests.py

# 创建Vivado工程、生成并验证Block Design
bash build/scripts/build_can.sh create

# 从源码完成综合、布局布线和位流生成
bash build/scripts/build_can.sh bitstream

# 核对回归、构建记录与源码摘要，生成部署包
python3 build/scripts/package_can.py
```

默认工作目录为 `build/work/`，每个入口均可追加 `--work-dir /path/to/build-work`。同一工作目录的命令按顺序执行，脚本会阻止并发改写。若要修改源码，可先运行 `python3 build/scripts/rebuild.py prepare`，再编辑工作目录中的文件，重新回归和构建。

如使用Icarus Verilog，运行 `python3 sim/run_tests.py --simulator iverilog`。构建默认2个并行任务、4个线程，可通过 `FPGA_BUILD_JOBS`、`FPGA_BUILD_THREADS` 配置。

产物位置（相对工作目录）：

| 内容 | 路径 |
|---|---|
| Vivado工程 | `contest/kv260_can/build/project/kv260_can.xpr` |
| 新位流与硬件平台 | `contest/kv260_can/build/kv260_can.bit`、`kv260_can.xsa` |
| 本次构建/回归日志 | `contest/kv260_can/result/` |
| 完整部署包 | `.cache/kv260_can_deploy/` |

新构建结果单独保存；提交包原有的 `build/report/` 和已归档位流保留作为对应版本的证据。`skill/kv260_pl_workcell_ops/scripts/` 中的构建与打包命令调用同一入口。

## 部署与工位运行

部署使用上一步生成的完整部署包。指定实际板卡账号与主机名，不依赖开发机的SSH配置：

```bash
bash build/scripts/deploy_can.sh ubuntu@BOARD_HOST build/work/.cache/kv260_can_deploy
```

如使用独立SSH配置文件，可作为第三个参数传入。脚本上传已校验文件，通过板端 `install_board.sh` 加载固件并进行内部回环自检；板端需要配置对应Linux镜像、`xmutil`、编译工具和sudo权限。部署前应结束运动任务并保持机械臂承托。

`correction_channel.json` 绑定位流摘要，驱动据此决定是否访问修正通道。`board_validate.py`、`board_correction_validate.py` 和 `tracking_monitor.py validate` 在板端完整软件目录内运行，使用匹配的固件和MMIO库。

运行前核对承托位置、温度、相机及检测状态、服务端口和当前owner。正常停止由当前owner完成返回与结束序列；异常保持时由操作员承托并确认，再结束使能。停止后按拥有者清理进程并归档日志。

## 版本与运行检查

- 寄存器访问范围以当前固件ABI为准。
- Type18受限参数使用匹配的软拒绝提交接口。
- 视觉与控制进程按已验证配置分配CPU核心，保持控制心跳。
- 更换机器后配置工具链、SSH和板端权限，再执行对应验证。

## 本项目记录

本项目已完成固件部署、板级自检及板端运行时启停。视觉进程隔离后的连续对准记录中，各段心跳最大间隔的P50为 44.5 ms，记录中的最大间隔为 51.7 ms，PL租约为100 ms。构建报告见 `build/report/`，运行数据见 `metrics/logs/`。
