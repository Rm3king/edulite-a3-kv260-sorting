# KV260 ARM机械臂规划与执行运行时

2026-10-19运动架构对照新增`control_benchmark.py`、`native_reference.cpp`和`blended_motion.py`，使用同一批220字定点记录比较ARM C++与PL求值/目标生产。两组仍共享PL CAN控制器、截止期FIFO和运动守卫，不能称作无FPGA基线。三类参考为连续关节途经点、80mm XZ圆和完整空抓回收；连续途经点的内部速度非零，仅整条任务端点静止。普通控制参数不因实验代码存在而自动修改。

完整命令、固定条件、统计口径与采纳状态见[实验协议](../../reports/control_convergence_20261019/PROTOCOL.md)及同目录报告。数值验证入口`audit_native_bank.py`不会打开硬件；`validate_timing_observer.py`仅在物理TX受硬件抑制的内部模式核验计时统计。`control_benchmark.py execute`与`scripts/collect_motion_ablation.py`必须显式加`--execute-attended`；仅在现场条件及对应轨迹已确认后使用。正常试验返回本次承托姿态再失能，分析压缩在失能后；软件异常不会直接落入无条件reset。可选`--host-stall-ms 20`仅用于单独标注的双方同条件故障注入，不得混入正常性能或伪装成普通ARM负载。

逆解留在KV260 ARM，AI PC提供感知与任务目标。数值规划模块不访问硬件；`arm_service.py` 默认离线，显式 `--execute-job` 才调用已审核任务执行后端。导入模块不打开UIO，不启动后台保持。

## 最新状态：2026-10-17 C3竖直圆已验收

同步MoveJ、途中计划减速/保持/恢复、MoveL上下20mm、C2圆及七次C3参考的8cm XZ竖直圆已做有承托实测。C3实测用户确认更连贯、更丝滑，结束回零并7/7 reset。运行时41项测试在主机和KV260通过。完整证据见[本轮报告](../../reports/c3_vertical_20261017/REPORT.md)。下文保留早期开发阶段记录，历史“尚未接入”描述不代表目前状态。

`septic_trajectory.py`、`septic_job.py`生成/检查共享q/v/a/j的七次段；`septic_transport.py`要求固件0x50001并检查新能力寄存器。高阶求根、编码、端点去重在使能前完成，实际提交只写预验算的73字和commit；旧三次任务/45字段仍支持。C3暂停/恢复分支尚未验收，离线拒绝，不混用旧三次制动。

真实执行需要已经核对的机械臂姿态、范围与现场条件；准备和执行均再检查起点。FK/IK误差与编码器重建不等于外部TCP精度，碰撞、长期负载与完整任务仍待验证。当前CSP没有我们自己的显式动力学前馈；`dynamics_audit.py`只做离线审查，不输出力矩。下一步见[重力补偿核对](../../reports/c3_vertical_20261017/GRAVITY_REVIEW.md)。

## 已实现

- 复用同级EDULITE_A3原厂SDK的三个数值模块，保留MIT许可证及逐文件SHA256；使用GUI实际采用的legacy URDF与惯量标定文件。
- 六轴机器人坐标的FK/IK，有限值与关节限位检查，IK最多200次迭代；结果必须再次通过FK位置/姿态残差检查，失败不输出轨迹。夹爪是独立分支，不作为第七个TCP逆解自由度。
- MoveL原型：直线位置与SO(3)姿态插值、相邻初值IK、分支跳变阈值、三次关节段、超差细分/拒绝。逐段解析检查关节位置极值，按解析速度/加速度上界安排时长。
- 段内FK以33点检测TCP误差；验证程序另用独立NumPy URDF和每段257点复核。采样通过不证明连续误差上界。
- 独立URDF正解1000组、有限差分Jacobian120轴、邻近可达IK50组、不可达/输入非法/内部越限/细分预算耗尽拒绝、重力数值100组与耗时统计。

依赖固定Pinocchio 2.7.0、NumPy 1.26.4、PyYAML 6.0.2；完整版本见requirements.lock。主机使用项目`.venv`，KV260使用`~/kv260_motion_runtime/.venv`；不修改系统Python、Conda、ROS或原厂SDK环境。板端缺ensurepip时，部署脚本使用项目内pip wheel引导，不需要sudo。

## 重现

在项目根目录：

```bash
.venv/bin/python -m pip install --index-url https://pypi.org/simple -r contest/arm_runtime/requirements.lock
OPENBLAS_NUM_THREADS=1 .venv/bin/python contest/arm_runtime/validate_runtime.py --output /tmp/arm_validation.json
./scripts/deploy_arm_runtime.sh
```

部署脚本下载aarch64离线wheel、传输源码并在板端执行完整数值验证，结果取回`reports/arm_runtime_YYYYMMDD/board_validation.json`。不加载bitstream、不启动电机控制后台服务。

板端FK例子（这里是数值测试关节角，不是运动命令）：

```bash
cd ~/kv260_motion_runtime
OPENBLAS_NUM_THREADS=1 .venv/bin/python src/motion_runtime.py fk --q 0 1 -1 0 0.4 0
```

`ik`/`movel`增加`--target x y z roll pitch yaw`，位置单位m，姿态为Pinocchio RPY约定的rad。`--q`是机器人关节坐标；CAN原始电机角必须先经过现有joint_safety映射，禁止重复乘方向或写入新零点。

## 结果边界与下一步

实测数值、依赖失败/修复日志、主机与板端对照位于[本轮报告](../../reports/arm_runtime_20261016/)。初次板端测量中FK P50约0.21ms、完整IK P50约54ms；这是Python参考实现与有限样本，不能称5ms控制周期或最坏执行时间保证。ARM提前规划，PL独立执行；后续优化可用C++及已验证的PL计算核。

当前MoveL每个小段零端速度，会在段间停顿；下一步需连续段拼接与jerk约束。碰撞、自碰撞、奇异性裕量、连续Cartesian误差界、HLS量化后复核尚未完成。重力结果只验有限值和耗时，尚未验证惯量映射、负载与真实补偿效果；不能据此开启零重力模式。

先补PP夹爪保护/真实Type2/受限单轴控制，再集成HLS段队列，完成MoveJ及MoveL实物验收。现有七轴HLS仍是独立核，本轮没有新增综合或硬件加速PASS。

## 2026年10月17日 MoveJ与暂停规划接口

`motion_profile.json`固定第6次完整回零的每轴电流、速度、加速度基线，以及SDK六轴方向/限位和实测夹爪行程。它是设计配置，读取配置不会把参数写进电机；CSP模式下的实际跟踪能力仍待验证。7个通道中前6个为机器人关节rad，第7个为夹爪原始电机rad，不能把它作为TCP运动学的第七轴。

`joint_trajectory.py`新增七通道同步MoveJ三次时间律，根据所有轴的速度/加速度约束选共同完成时间，输出机器人坐标、原始电机坐标及HLS Q8.24系数。输入越限、非有限值、超速倍率和定点系数溢出直接拒绝；没有自动扩大昨晚验收的参数。MoveJ不等于TCP走直线。

暂停规划从当前期望位置与速度构造恒减速段，位置/速度连续、加速度有界，但边界加速度可跳变，尚无jerk约束。当前模拟流程为`DISABLED → HOLDING → MOVING → BRAKING → HOLDING`；停止后丢弃旧目标，不自动恢复。Hold仅表示已经停稳的保持，不能在MOVING时跳变为Hold。故障锁存后不自动重使能。这个状态机没有硬件驱动，不会产生维持力矩；物理异常停机仍是旧驱动的有承托reset流程。

`arm_service.py`提供JSON行接口Status/PlanMoveJ/PlanStop与显式Sim命令；它是板端服务接口的离线实现，**物理Enable/MoveJ/MoveL/Pause/Hold/Home/Disable尚未连接，均拒绝**。无后台守护、无网络监听、无CAN/MMIO导入。示例：

```bash
/usr/bin/python3 -m unittest discover -s contest/arm_runtime -p test_joint_trajectory.py -v
/usr/bin/python3 contest/arm_runtime/arm_service.py --request '{"sequence":0,"command":"Status"}'
```

本轮已部署同一源码到KV260并运行模拟流程。数值运行时根据七轴只读快照中的前六轴计算FK，结果位于`reports/arm_service_20261017/board_fk_actual_snapshot.json`；这是URDF预测，不是视觉/TCP物理测量。

`generate_hls_vectors.py`生成68组MoveJ/减速/边界测试，其中两组使用今日实物只读坐标。`contest/trajectory/tb/arm_vectors.hpp`为可重现输入，C/RTL状态和完整日志见本轮报告；只有采样通过，不能替代任意输入的定点极值证明或碰撞检查。再生命令：

```bash
/usr/bin/python3 contest/arm_runtime/generate_hls_vectors.py \
  --positions reports/arm_service_20261017/read_only_positions.json \
  --header contest/trajectory/tb/arm_vectors.hpp \
  --report reports/arm_service_20261017/planner_hls_cases.json
./contest/trajectory/run.sh csim
./contest/trajectory/run.sh cosim
```

连续执行的下一道关卡：原厂SDK的POSITION_CSP为run_mode=5、速度上限0x7017、位置0x7016；现有v4仅允许PP run_mode=1和0x7024/0x7025，不应绕过它。需要新增独立CSP准备/使能/限界校验，接入HLS短段和定时CAN，再先做有承托的小幅CSP跟踪。单纯将HLS采样点重复写入电机PP，不等价于验收PL主导的连续轨迹。

### PL执行集成（v5开发）

`pl_transport.py` 把MoveJ端点量化成保持零端速的Q8.24多项式，检查全段位置极值、速度/加速度和段间连续性，再原子提交给PL。PL定时插补与CAN发送无需ARM逐点更新。`csp_commission.py` 是显式运行、有监护的实物验收入口；默认只生成计划，执行使用固定review文件并重新核对起点。上层 `arm_service.py` 真实动作接口尚未开放，不能把验收脚本视为完整在线服务。

执行状态/实物结果见 `reports/csp_integration_20261017/`。本版PauseAtEnd只完成当前段后保持；原离线Pause减速候选尚未接入实际硬件。后续需要实现段中受控暂停、碰撞检查以及MoveL实物验收。

### 2026-10-17 CSP 跟踪参数修正

`velocity_rad_s` 约束 ARM 规划和 PL 多项式导数；新增的
`csp_velocity_limit_rad_s` 单独设置电机 `LIMIT_SPD(0x7017)` 及对应 PL 参数白名单上限。
调整后者不会提高轨迹速度。旧 profile 缺少该字段时沿用原行为，显式配置须覆盖规划速度；
commission 入口另拒绝超过 0.15 rad/s 的 CSP 上限。`--profile` 可指定诊断配置，review 的
profile 哈希须匹配，重新绑定实际起点时保留已审核的最短执行时间。

当前 CSP 上限为 `[0.06,0.06,0.06,0.06,0.06,0.06,0.04]` rad/s，
规划速度仍为 `[0.06,0.01,0.01,0.06,0.06,0.06,0.04]` rad/s。此前通过实测的电流为 `[1.5,3,4,2.5,1,1,0.5]` A。3 号在 3 A 下发生持续饱和、未到位，4 A 下完成同一绝对目标
及返回，采样电流峰值 3.4215 A。一次同步往返通过，末端小行程、长期负载和整个工作空间未验收。
速度上限的独立因果对照未完成，不能将全部改善单独归因于它。详见
`reports/csp_axis3_diagnosis_20261017/REPORT.md`。这些参数仅写 RAM，Flash 零点未修改；
测试最终七轴 reset，服务真实动作接口仍关闭。

用户随后要求 2、3 号均设为 4.5 A，当前默认 profile 电流为
`[1.5,4.5,4.5,2.5,1,1,0.5]` A。已在电机失能时写入并读回确认，无使能；
随后4.5 A配置下较大去程及独立回零通过；原整条回程RX溢出中止，不能记作连续整轨迹通过。

### 2026-10-17 显式回零及采集修正

`--finish zero` 要求review明确七通道最终零位，位置包络包含最终目标。终点必须在
±0.002 rad内连续稳定2秒（采样间隙超过80ms重新计时），再独立读回位置，最后reset七轴。
2/3号持续跟随误差超过0.03 rad达0.5秒即中止。默认 `--finish start` 仍返回本次起点。

独立双倍速度profile位于 `reports/csp_large_zero_20261017/profile_double_speed.json`，默认速度不变。
较大去程到位后回程因16项RX FIFO溢出中止；暂停采集期间循环GC后，独立回零通过。
此措施不消除Linux调度停顿，RX缓冲/持续采集仍待加强和长时间验收。
详见 `reports/csp_large_zero_20261017/REPORT.md`，两次原始数据均保留。

### 2026-10-17 连续 MoveJ / MoveL / MoveC 执行

`motion_executor.py` 的 `MoveJExecutor` 复用七轴CSP准备、读回、使能与停止握手。
`arm_service.py --profile PROFILE --execute-job JOB --output REPORT` 显式执行审核后的任务；
默认JSON行模式仍只规划/模拟。库接口可注入执行后端，再调用 `MoveJ`、`MoveL`、`MoveC`
或 `ExecuteReviewedJob`，单轴目标不能绕过任务校验。序号拒绝重放。停止失败时响应FAULT。

`prepare_motion_job.py` 从七轴只读快照创建有界任务。`continuous_trajectory.py` 在ARM上
完成连续分支IK、时间参数化、Hermite关节段、解析关节极值/导数和量化后FK采样检查；
`continuous_transport.py` 将系数提交既有v5 HLS，PL按5ms调度七轴CAN目标。
直线、圆形任务保持末端朝向，圆位于机器人基坐标系所选平面；夹爪为独立电机通道。

新增独立候选 `profile_grasp_candidate.json`：关节速度上限
[0.35,0.18,0.18,0.35,0.35,0.35,0.15] rad/s，CSP伺服上限
[0.5,0.3,0.3,0.5,0.5,0.5,0.25] rad/s；电流仍
[1.5,4.5,4.5,2.5,1,1,0.5] A。TCP请求25mm/s会因关节约束伸时，不能宣称实际达到请求值。
默认profile未提升。编码器速度窗口依据CSP伺服上限（另留5%和2个反馈LSB裕量），
并保留速度持续/突变、跟随误差、反馈新鲜度和CAN故障检查。

暂停在最多两个已提交的<=250ms短段之后接入提前验证的制动段，不替换正在执行的多项式；
零速后维持PL位置刷新与电机使能，按已审核的分支从制动终点重新规划恢复。
已验收预定触发的运动中暂停/保持/恢复；它有前瞻延迟，不是立即急停。
外部网络事件驱动、通信丢失的受控制动和无人值守常驻服务仍未验收。

v5每段输出n+1采样点。下一段编码从其5ms时刻开始，消除重复交界端点；省略区间按
速度/加速度检查，并有逐点时间网格对照。每个任务及所有制动/恢复分支在打开UIO前
通过同一提交校验。此修正无需重编FPGA。每轴使能保持目标采用该轴刚读取的位置，
起点漂移仍须<=2mrad；参数读回容差未放宽。最终正常停止必须取得7/7新鲜reset。

实物记录见 `reports/motion_paths_20261017/REPORT.md`：首轮暂停/直线/小圆及后续大圆
分别记录；用户报告仍有轻微抖动，尚未归因或解决。反馈+URDF重建精度与外部TCP测量
分开；未做相机/TCP外部标定、长期负载或全工作空间验收。

C2平顺性候选：`prepare_motion_job.py --circle-radius .04 --circle-only
--gripper-target 1.6 --keep-gripper --smooth-c2` 在原范围内生成直径80mm水平圆。
ARM以均匀时间IK节点、五次进度律和夹持边界C2三次样条降低加速度跳变；仍由现有PL
求值执行。C2功能通过独立位置/速度/加速度衔接与量化FK检查；平顺性需与现场观察分开记录。
`--keep-gripper` 结束时六关节回零、夹爪保持指定开度并全部失能；不是七通道全零。

### C3 与自研重力前馈候选

`septic_trajectory.py` 生成七次 Hermite C3 参考，`septic_transport.py` 提交到 v5.1
或 v6 的七次 HLS 通道。参考 q/v/a/jerk 连续不代表采样电机物理 C3。
已验收的 8 cm XZ 圆记录于 `reports/c3_vertical_20261017/`；C3 暂停/恢复尚未验收，
相关任务在打开硬件前拒绝。

`gravity_job.py`、`gravity_trajectory.py` 与 `gravity_executor.py` 是独立 v6 实验入口，
采用六轴 Type1 q/v/PD/重力、夹爪 CSP。ARM 预计算重力多项式；PL HLS 在 200 Hz
计算位置、速度、前馈力矩并打包发送。运行中不做 IK/重力拟合/求根。
`gravity_profile.json` 使用同级原厂工程的 Kp=80、Kd=4。原厂界面为4号设置的
2倍重力系数在本机静态对照中过补偿，已复验改为模型的1倍；六轴现均为1倍。
固定姿态先做补偿开/关渐变，随后相同 C3 圆 A/B 的 q/v/gain/时序完全一致。

所有段在 UIO 打开前经过连续极值与实际定点相位逐点检查。新固件版本检查先于
新增 MMIO 访问，防止旧映像未映射地址导致 SIGBUS。只有经过预计算且顺序相同的段
可提交。MIT 模式验证 0x700B 的 Nm 限额与读回，不把 0x7018 当作 MIT 的总力矩保护。
初始化先在失能状态下准备全部七轴，再成组提交使能，避免上游轴移动尚未使能的
下游轴。Type17参数回复不刷新Type2反馈表，成组使能前另行取得新鲜的失能反馈。
显式执行要求已确认的现场承托/监护；正常结束先返回实测承托起点，再七轴新鲜
reset握手。异常会停止并失能，所以承托仍须能承接；不在中间姿态切模式失能。
退出后只在已确认失能时恢复测试前的CSP模式和力矩限额，不改Flash或零点。

该候选没有替换 CSP 默认路径，也未进入 Kp=0 拖动。实物是否优于原 CSP 必须单独比较；
同模式 A/B 改善也不能证明已优于 CSP。质量/质心、摩擦、工具负载及外部 TCP 精度仍须验证。
当前验收进度与失败记录见 `reports/gravity_ff_20261017/REPORT.md`。

### 按当前反馈直接进入零重力拖动

`freedrive.py` 是独立的受监护拖动入口，六轴从首次使能起Kp=0，按实时实际角度
计算重力，保留原厂范围内的阻尼；夹爪保持当前CSP开度。它不要求预先离开零位、
匹配历史起点或先移动到某个姿态。初始化在全部轴准备好后重新采集当前角度，再
成组使能。`GravityExecutor.prepare(..., live_start=True)`仅允许Kp全0的模式使用；
自动轨迹执行仍按其参考轨迹校验，不能用拖动模式的实时接入去跳接历史轨迹。

此辅助模式由ARM按名义100Hz更新实测重力，PL负责CAN、编码限界、反馈和心跳；
没有使用HLS轨迹通道，不能称为PL求解完整动力学。重力超过临时力矩限额时采用
部分补偿并报告饱和，不因这项饱和自动失能。实际角度用于重力；Kp=0时的协议位置
字段投影到已有合法区间，不发出超出区间的位置指令。

用户决定拖动结束：没有定时失能、自动回位或15秒恢复刚度。放回承托并确认后，
向控制文件原子写入唯一`id`及`command: finish_supported`；程序取得新的静止窗口
后执行七轴reset并恢复测试前CSP模式及力矩限额。通信故障/失联仍会停止，操作人
需能承接臂段。运行中控制和状态文件放在`/dev/shm`，日志采用有界窗口；退出报告
明确记录截掉的历史数量。不得把陈旧的live文件当作进程仍运行的证据。

### 2026-10-18 按实时起点执行空抓回收

`pick_return.py` 生成右前方定点空抓的 C3 任务：张爪进场、竖直下降50mm、
合爪、上抬50mm、返回本次起点、张爪释放并失能。初次目标在基坐标
(-0.17,+0.12,+0.13)m，预抓高度0.18m；这是空载动作验证，尚未标定桌面/TCP。
局部-Z是URDF工具伸出方向，使用RPY(0,0,π/4)令工具朝下。

新任务显式携带`start_policy: CURRENT_FEEDBACK_REPLAN`。`motion_executor.py`
先完成全部七轴CSP配置再成组使能，以当前角度保持；`live_motion.py` 从新鲜
反馈重建首段MoveJ和返回段。不会把实时角度直接跳接到历史多项式，也没有
旧快照匹配、静止起点范围或接近零位禁止启动的判断。小于0.02rad的标定端点
测量偏出被映射到合法参考，命令区间不扩大；4个Q24码的内缩仅用于避免
七次多项式在精确端点的定点舍入外逸（约0.000014°）。明显超出物理行程的
异常读数、真实故障和失联仍报告错误。

2026-10-18当前入口补充：失能时LOC_REF读回可能跟随实测位置，不能当作命令回显；初始化发送经PL限界验证的合法位置报文，再检查全部准备位及新鲜成组使能回执。模式、速度、电流和超时仍读回验证；不会要求接近零位的实测角度先匹配旧参考。软件失败按`finish_failed_job`尝试完成已验证返回，不能因普通测量错误直接撤去力矩；真实故障和失联的物理行为不由这项策略保证。

数值进程在打开UIO/使能前完成启动，在保持期间接收七轴当前角度。
主进程唯一持有CAN设备并持续处理反馈/心跳；每次只接收和解码一个小段，
避免大型进程启动或一次解码整个任务堵塞控制循环。回传的任务与准备记录
保持段对象身份；运行时不重新求根、解IK或量化。PL沿用v6七次HLS通道，
自主每5ms求值并发送七轴CSP目标。旧的历史任务文件保留原始审查语义；
新的空抓入口使用实时重建。正常完成必须取得七轴reset确认。

生成、离线校验及实物证据位于`reports/pick_return_20261018/`；最新验收
以其中`validation.json`为准，不把空抓等同于真实螺丝抓取成功。

本轮用户确认流程正常，但提出抓取点偏高、启动等待、速度和可见抖动问题。
新增独立`profile_grasp_fast_candidate.json`，仍保留原电流，关节规划速度
[0.5,0.3,0.3,0.5,0.5,0.5]rad/s、夹爪0.45rad/s。使用`pick_return.py
--grasp-z .10 --approach-height .08 --tcp-speed .04`准备低3cm的空抓候选。
该速度配置的物理验收以本轮报告为准，不等于全工作空间/带载认证。

C3校验现先用整数Bernstein凸包证明整个区间的位置/导数边界；无法证明时仍用
原求根验证。标量Horner保持原NumPy运算顺序，已与冻结的704组实际发送数据
逐项比较一致。移除了构造与传输间重复的同值检查，每个原始和编码后多项式
仍完整校验；运行中仍只使用准备好的记录。板端新版224段实时绑定无运动实测
约5.90秒；具体实物启动时间另记。此优化不改变HLS定点算法、参考值或硬件保护。

原厂CSP增益已只读备份，未修改PID/Flash/零点。可见抖动仍是独立待修复项，
不因完成C3、提高速度或较小编码器误差就宣称已解决。

2026-10-19 实验入口：`GravityModel`可选`pl_joint3_damping_nm_s_rad`（0..2，默认0），对应v6.2第216字Q8元数据，旧固件在使能前拒绝非零功能。`autotune.py --full-pick-only --compare-pl-damping`构造除该字段外完全相同的完整抓取ABBA；所有增益须一致，最后回承托起点、reset后才分析。没有更改ArmService全局默认。另有独立`joint3_frequency_probe`诊断配置，仅允许单次显式replay，结果不得计作完整抓取。参见[实验协议](../../reports/frequency_retune_20261018/PROTOCOL.md)。
2026-10-19：新增离线正权重FIR参考平滑/部分动力学/阻尼分配实验银行，配置`full_pick_experiment`必须显式搭配`autotune.py --full-pick-only --compare-method`。银行在七轴失能时构建，随后重新采集持位反馈并使能；不增加旧起点偏移拒绝条件。FIR对所有关节使用统一时移并重查MoveL几何，不能默认关节滤波仍保持直线；部分动力学与阻尼实现均保持单因素ABBA。所有结果在支持回程、reset之后分析，无新增日志/相机失败失能分支。实验不是默认控制策略，实际采纳与限制见`reports/trajectory_retune_20261019/REPORT.md`。


2026-10-19收敛配置见[profiles](profiles/README.md)及[真实20轮报告](../../reports/control_convergence_20261019/REPORT.md)。正常ABBA与匹配20ms主服务暂停分开计数；ARM是Python调度＋原生C++求值/打包/MMIO，不是独立C++实时线程。PL采用C++批量上传系数，50Hz阶段观察，硬件仍200Hz执行；反馈逐帧检查保留。三类运动基线已实测，机械抖动不宣称修复。所有初版负结果和未使能预检失败都保留。
