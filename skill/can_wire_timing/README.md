# Skill：用逻辑分析仪验证 CAN 周期控制的线上时序

## 适用范围

本项目的CAN扩展帧周期控制分析案例。脚本按当前电机协议分类Type1、Type18设定及Type17读取事务，默认周期5 ms；迁移到其他控制器时先调整帧分类：

- 只需一台普通逻辑分析仪（本项目用 Kingst LA2016）和一根探针接 CAN-H，不需要 CAN 分析仪；
- 输出周期间隔与偏差的分位数、偏差是否累积、晚点与哪类总线事务同时出现、总线占用率、位宽；
- 附带两张图：周期偏差分布与一次晚点前后的线上帧时序。

## 使用方法

1. **接线与采集**：探针接 CAN-H、地接系统地；采样率 50 MHz；CAN-H 通道阈值设 2.8 V（CAN-H 显性约 3.5 V、隐性约 2.5 V）；让控制器进入稳态周期发送后采集 1–3 分钟。
2. **导出**：KingstVIS → Export data → CSV（每个边沿一行：`Time[s], Channel N`）。
3. **分析**（AI PC，Python 3、numpy与matplotlib）：

   ```
   cd skill/can_wire_timing
   python3 bus_timing_analysis.py /path/to/capture.csv --out bus_timing.json --figure-dir work/      # 同时写出 work/bus_frames.npz
   python3 bus_timing_figures.py --json bus_timing.json --frames work/bus_frames.npz --out figures/
   ```

   脚本按“隐性超过 9.5 位”分帧 → 游程转位 → 去填充 → 解扩展 ID 与 DLC → 取指定 ID 的帧起点 → 计算相邻周期间隔与对名义周期的偏差；自动把稳态流之前的使能阶段与稳态段分开。
4. **读结果**：`steady_stream.deviation_us` 的 P50/P95/P99/最大；`large_deviation_with_read_between` 给出晚点与位置读取事务的重合数。
5. **改 ID 或周期**：分析脚本需调整 `NOMINAL`、`main()` 中的帧分类与稳态分段规则；绘图脚本需同步调整 `0x1200FD01`、`5e-3` 和事务分类。修改后用已知帧验证解码，再统计周期。

## 用已有记录离线复查

从代码包根目录运行以下命令，可直接重画本项目已归档的两张时序图，无需连接硬件：

```sh
python3 skill/can_wire_timing/bus_timing_figures.py --json metrics/logs/bus_timing.json --frames metrics/logs/bus_frames.npz --out work/can_figures
```

## 失效条件

- 阈值过高（本项目 3.0 V 时）会让 CAN-H 幅度偏低的节点帧解码失败；此时只解码本控制器发出的帧，起点统计不受影响。
- 采样率低于 20 倍位率时位宽误差变大；1 Mbps 总线至少 20 MHz。
- 只测线上帧起点，不代表执行器内部的响应时序。

## 来源与已验证的效果

- **从哪里总结出来**：项目需要证明 PL 的 5 ms 周期在线上确实稳定，内部计数器的统计无法说服外部评审；第一次用分析仪自带解码时，部分电机的帧解不出来，改为只用 CAN-H 单端、自行解码本控制器的帧后得到完整统计。
- **本项目结果**：一次三件运行的稳态段 77 s、15364 个周期，偏差 P50 0.4 µs、P95 67.0 µs，晚点超过 250 µs 的 548 个周期全部与位置读取事务同时出现；与 PL 内部发送计时监视器的统计互相印证（`metrics/logs/bus_timing.json`）。
