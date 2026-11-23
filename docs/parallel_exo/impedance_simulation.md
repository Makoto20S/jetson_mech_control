# PH-EXO 连续阻抗软件闭环

运行链为 MuJoCo 模拟电机反馈 → JointState → 现有 ph_exo_state → RobotState → impedance_ros → 成对 JointTrajectory 力矩目标 → MuJoCo 执行端。各节点通过 ROS 2 DDS 交换消息，使用隔离域 179、随机命名空间和仅本机通信。

## 状态与控制

RobotState 新增 q_velocity=[ψ̇,β̇]、q_velocity_valid、q_velocity_reason、q_velocity_condition。MuJoCo 原生闭链约束给出 J_dep·v_dep=−J_active·α̇，求解后提取输出角速度。位置有效、输入速度有效且雅可比条件数不超过 1e8 时发布有效结果，否则填 NaN 并给出原因。电机轴序、映射与有效性沿用现有状态链。

控制律和参数沿用 [模型内验证入口](../../mujoco/ref01/README.md)。控制器按源反馈时间判定 60 ms 有效期，重复源时间不延长期限。反馈无效、过期、回退或计算失败时停发并锁存。MuJoCo 力矩执行端也独立检查 60 ms 目标有效期，过期归零并保持停止。

仿真启用 publish_on_joint_message，使新关节反馈触发状态发布。有效旧帧由定时器检查期限，控制订阅队列只保留最新一帧。默认状态节点仍采用原有周期发布方式。该选项用于 joint_states 输入。

## 复现

Windows 先用项目 Python 运行以下两项校验，再按 local_orin.md 同步和构建 servo_imu。

```powershell
.local/venv/Scripts/python.exe -B tools/project/verify_state_windows.py
.local/venv/Scripts/python.exe -B tools/project/test_impedance_state.py
```

Orin 开发副本加载环境后，指定新的输出目录。源码同步与测试按项目同步锁串行执行。

```bash
source /opt/ros/humble/setup.bash
source .local/ros/servo_imu/install/local_setup.bash
source tools/project/mujoco_ros_env.sh
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$PWD/.local/validation/<运行标识>/ros_logs"
python3 -B tools/project/test_impedance_ros.py --output .local/validation/<运行标识>/graph
```

测试运行 ψ 扰动、β 扰动、停止反馈、重复旧反馈四个工况。每次独立复位并重新启动三个节点。失效工况还恢复新反馈，核对锁存保持。输出含模型逐周期 CSV、接收目标、ROS 状态、控制器日志和判定摘要。测试结束按 [归档流程](local_orin.md#测试结果归档) 核验回传 Windows。

恢复判据沿用两轴角误差 ±0.05°、角速度 ±0.25°/s，并保持至结束。模型按每周期 5 ms 推进，实际主机间隔另行记录。目标周期、恢复曲线和主机调度统计各有独立时间含义。

连续闭环执行端为 MuJoCo 直接力矩接口，采用现有成对 MIT 目标消息格式。现有 MitPairEffortController、AK 驱动和 MIT 量化通过独立虚拟串口测试核验。本入口属于软件测试，实机力矩、速度标定及硬实时验收留待后续。

## 周期诊断

连续测试默认使用持久 SingleThreadedExecutor，并在三个 Python 进程进入循环前 collect/freeze，收尾时 unfreeze；运行中新对象仍自动回收。状态节点的 freeze_gc_before_spin 与阻抗节点的 --freeze-gc 由测试入口显式设置。

测试可加 --duration 30 延长两轴工况，或用 --cases psi beta 选取工况。--gc-policy automatic 用于原生自动回收对照；--gc-policy pause 为模型进程诊断选项。gc.json、state_gc.json 和 controller.json 的 gc_events 使用主机单调时间，timing 汇总同时记录模型周期与力矩发布间隔，分别核对各进程的长停顿。

## C++ 控制路径与 5 ms 软件验收

`ref01_native` 是显式选择的无设备软件构建组合。它在已有 `mech_controllers` 包内增加
`Ref01ImpedanceController`，把 MuJoCo 正运动学、角速度投影、零速重力项和耦合阻抗计算
放入 C++ `update()`。模型在配置时加载，激活时用当前输入预热；运行中使用固定大小数组和
有界迭代，不调用 Python、参数服务或文件记录。原有 Python 三进程入口保留为数值和消息链参考。

早期直接调用夹具 `test_ref01_native_ros.py` 实例化这个真实 `ControllerInterface` 实现，借出内存中的 state/command
接口，并在独立控制线程按 5 ms 调用 `read → update → write`，MuJoCo 作为执行端。
ROS 计划和状态观察由另一执行器线程以 50 Hz 运行；状态发布只读取复制快照，不阻塞控制线程。
这个入口直接调用插件，**未经过 controller_manager 的调度，也未连接物理 hardware plugin、
AK 驱动、CAN 或 MIT codec**。其结果不能替代实际 MIT 量化后的连续闭环或实机时延结果。

插件要求显式设置 `simulation_only=true`，默认拒绝配置。配置参数为 `model_path` 与两元素
`joints`，默认轴序为 `motor104_joint, motor105_joint`。每轴读取
`position, velocity, sample_time_s, sequence`，输出 `effort, command_generation`。
`sample_time_s` 必须是本机单调时钟秒数，`sequence` 是原始反馈的严格递增整数；重复读取或广播
不得改写这两个源字段。当前仿真映射固定为：

```text
alpha1 = position105 - pi/4;  alpha2 = position104 - pi/4
alpha1_dot = velocity105;    alpha2_dot = velocity104
q = [psi, beta];             alpha_dot = A * q_dot
tau_alpha = solve(A.T, gravity + K * (q_ref - q) - D * q_dot)
tau_motor104 = tau_alpha2;   tau_motor105 = tau_alpha1
```

两输入轴共同缩放以满足各自力矩上限，保留所求总力矩的方向。该映射只对当前软件夹具确认，
不是新的实物零位、极性或传动比标定。IMU 不进入此固定姿态控制律。

### 上层计划合同

上层通过私有话题 `~/plan` 发布 `std_msgs/msg/Float64MultiArray`，layout 必须为空、offset 为 0，
data 长度必须恰好为 12。参考、增益、上限和许可经过非实时检查后作为同一帧进入 RealtimeBuffer。

| 索引 | 字段 | 单位与限制 |
|---|---|---|
| 0 | schema | 固定为 1 |
| 1 | sequence | 严格递增正整数，最大 `2^53-1`；重复序号不续期 |
| 2 | issued_at | 与控制器同一主机的 monotonic 秒数；不得早于本次激活、晚于接收时刻或已过期 |
| 3 | enabled | 精确 0 或 1 |
| 4、5 | q_ref | `[psi,beta]`，rad；分别在 `[-pi/4,pi/4]`、`[0,pi/4]` 内 |
| 6、7 | K | N·m/rad；正值，天花板 `[2.4,2.0]` |
| 8、9 | D | N·m·s/rad；非负，天花板 `[0.18,0.15]` |
| 10、11 | torque_cap | `[alpha1,alpha2]` 输入轴 N·m；正值，天花板 `[0.2,0.2]` |

所有字段必须有限。上层只能在天花板内选择参数；这不是运行中自由调参接口。第一次执行时，
各参考与当前姿态相差不得超过 0.01 rad，各姿态角速度不得超过 0.05 rad/s。
首次有效执行后，参考、K、D 和 cap 在本次激活内固定；修改会锁存停止。
重新设定计划必须先停用、重新激活控制器，并取得激活后的新反馈。恢复数据或继续发送计划不会自动清除故障。

### 周期、源时效和停发

| 项目 | 本入口的行为 |
|---|---|
| 控制周期 | 名义 5 ms；开始闭环后的实际 update 间隔或传入 period 达到 10 ms 时锁存停止 |
| 计算截止期 | 单次 update 的单调时钟计算耗时达到 5 ms，拒绝发送这次结果并锁存；写出前再次检查源时效 |
| 模型输入跳变 | 激活预热后，任一 alpha 与前次成功值相差超过 0.15 rad，拒绝模型更新并锁存 |
| 上层计划 | 原始 issued_at 年龄达到 100 ms 时失效；接收时刻不延长期限 |
| 反馈 | 任一原始反馈年龄达到 60 ms，或两轴时间差达到 30 ms 时停止；非有限值、时间/序号回退和源字段矛盾均锁存 |
| 命令续期 | 只有完整的新反馈对成功计算后才增加 generation；原样重复帧和仅一轴更新不能续期 |
| 仿真执行端 | 独立检查 generation、有限值及 ±0.2 N·m 上限；最后一次有效命令超过 60 ms 后力矩归零并锁存 |

控制器故障时将其命令接口置零并停止 generation。接收端是否已执行零力矩，仍以其独立租约和
实际 `ctrl` 写入记录判断；不能仅凭控制器停止刷新就宣称执行端已经归零。上述 10 ms/5 ms 为
本次 PH-EXO 软件候选规格，不沿用其他电机台架的 4/6 ms 门限。

### 时序判定

测量使用同一 Orin 主机的 `monotonic_ns`，分别记录周期开始、源反馈、力矩实际写入 MuJoCo 和
周期结束。反馈到执行延迟是“本次命令所用源反馈 → 对应力矩实际写入”，不是目标发布间隔。
正常工况要求每周期处于 Following，且有新的实际命令 generation，不能用故障后的空循环充当有效控制样本。

`timing_contract.py` 将 `functional_passed`、`timing_passed` 与周期间隔超过名义 5 ms 的计数分别报告。
ψ、β 两个常规工况都须通过以下候选软件验收；断流和重复帧只检查功能，时序结果标记为不适用。

| 指标 | 候选验收值 |
|---|---|
| 每轴有效窗口 | 实测墙钟不少于 30 s，至少 6000 个周期 |
| 周期均值 | 4.95–5.05 ms |
| 周期 P99 | 4.5–5.5 ms |
| 绝对周期抖动 P99 | 相对 5 ms 不超过 0.5 ms |
| 最大周期 | 严格小于 10 ms，所有周期必须为正 |
| 反馈到执行延迟 P99 | 不超过 5 ms |
| 最大反馈到执行延迟 | 严格小于 10 ms |

门槛在运行前固定，不根据本次样本放宽。缺字段、非有限值、样本不足或任一轴失败，都不能得到
完整时序通过。周期间隔超过名义 5 ms 的次数仍单独保留；这不等于控制计算的截止期违约率。满足上表表示候选软件时序通过，
不表示每周期零超时或硬实时保证。模型恢复曲线使用固定 5 ms 仿真步长，不能用其模拟时间代替上述墙钟窗口。

### 构建与运行

在 Windows 正式仓库根目录先验证可移植内核与 MuJoCo 数值对照。验证入口需要已有项目 Python
环境以及兼容 g++/clang++ 命令行的 C++17 编译器；默认查找
`.local/toolchains/zig-0.13.0/ziglang/zig.exe`，也可通过 `--compiler` 显式指定。
工具链放在 `.local`，不随源码同步。首次可用项目 Python 执行
`-m pip install --no-deps --target .local/toolchains/zig-0.13.0 --cache-dir .local/cache/pip ziglang==0.13.0`。

```powershell
.local/venv/Scripts/python.exe -B tools/project/verify_ref01_native.py --output .local/validation/native_windows_<运行标识>
tools/project/Sync-Orin.ps1 sync
tools/project/Sync-Orin.ps1 build -Profile ref01_native
```

Windows 检查覆盖数值、轴序、限幅、源时效和失效逻辑；ROS 构建及 ARM64 时序结论来自 Orin。
`ref01_native` 按已有 Python 依赖中的 MuJoCo SDK 构建可选插件与软件测试程序，不改变默认硬件启动入口。

在 Orin 开发副本执行以下入口，每次使用新的运行标识。测试与源码同步共用外层 flock；
`EXO_SYNC_LOCK_HELD=1` 仅在已经取得此锁的子进程中设置。

```bash
cd /home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control
flock -n .local/sync.lock bash -c '
  set -e
  test ! -e .local/sync/incomplete.json
  export EXO_SYNC_LOCK_HELD=1
  export PYTHONDONTWRITEBYTECODE=1
  source /opt/ros/humble/setup.bash
  source .local/ros/ref01_native/install/local_setup.bash
  source tools/project/mujoco_ros_env.sh
  export ROS_LOG_DIR="$PWD/.local/validation/native_<运行标识>/ros_logs"
  mkdir -p "$ROS_LOG_DIR"
  .local/venv/bin/python -B tools/project/test_ref01_native_ros.py \
    --executable .local/ros/ref01_native/install/mech_controllers/lib/mech_controllers/ref01_impedance_software_loop \
    --output .local/validation/native_<运行标识>/graph --duration 30
'
```

运行后按 [结果归档流程](local_orin.md#测试结果归档) 将报告和原始输出逐文件校验至 Windows，
再清理对应 Orin 临时结果。这个入口不修改真实 MIT/电机参数、现有 release 链接或 Orin 系统调度配置，
不设置实时调度策略、CPU 绑定、内核或电源模式。

## 实际 controller_manager 与仿真 SystemInterface

`test_ref01_manager.py` 使用系统安装的 `controller_manager/ros2_control_node`，
由 manager 执行 `read → Ref01ImpedanceController.update → write`。
新增的 `mech_simulation/Ref01MujocoSystem` 为同一进程内的 MuJoCo 仿真接口；
`ref01_native` 构建 `mech_controllers`、`mech_simulation` 与 `mech_bringup`；本节默认夹具不加载设备驱动。
URDF 和参数由测试入口生成，显式要求 `simulation_only=true`、200 Hz、`use_sim_time=false`、
`lock_memory=false`；每场景独立进程、独立命名空间。该夹具只允许 5 ms 整数倍时长。

上层 Python 只以 50 Hz 发布固定姿态计划、查询状态和编排生命周期。模型计算及力矩写入
不经过 Python/DDS 消息链。hardware 读取两台模拟编码器的姿态、速度、原始时间与序号，
控制器写入两台电机的 effort 与成对 generation。硬件接收端只接受新 generation，
60 ms 未续租就归零并锁存；它与控制律独立，但仍在 manager 的 write 调度中执行，
不是独立线程或物理驱动 watchdog。新实例才能重新运行该一次性仿真测试。

`~/software_status`（`std_srvs/srv/Trigger`）在非控制路径返回 JSON；
success 只表示格式可读，mode 0/1/2/3 分别为 inactive/waiting/following/faulted，
reason_name 给出具体原因。更新路径以 try_lock 复制固定快照，查询回调在锁外格式化。
manager active 只表示生命周期，验收还必须结合 mode、原始反馈和实际 generation/ctrl 写入。
trace 在预分配内存记录，完成后冻结，再由后台线程写文件；`complete.json` 是最后完成标记。

按前节完成 Windows 验证与同步构建后，在 Orin 开发副本执行：

~~~bash
flock -n .local/sync.lock bash -c '
  set -e
  test ! -e .local/sync/incomplete.json
  export EXO_SYNC_LOCK_HELD=1 PYTHONDONTWRITEBYTECODE=1
  export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
  export XDG_CACHE_HOME="$PWD/.local/cache" TMPDIR="$PWD/.local/tmp"
  source /opt/ros/humble/setup.bash
  source .local/ros/ref01_native/install/local_setup.bash
  source tools/project/mujoco_ros_env.sh
  python3 -B tools/project/test_ref01_manager.py \
    --output .local/validation/manager_<唯一标识> --duration 30
'
~~~

该入口保存官方进程、线程实际策略、插件来源、接口声明/占用、生命周期、状态快照和硬件 trace。
官方 Humble manager 默认尝试进程内 FIFO 优先级 50；按主机已有权限返回结果记录，
本入口不修改系统权限、内核、电源、CPU 绑定或 release 链接。
因此不能将前节直接夹具的普通调度策略假设沿用到 manager 结果。

正常轴各需 30 s/6000 周期以上；断流、重复旧帧为 2.5 s 功能场景，在模型时间
1.2–1.65 s 冻结反馈，之后核实真实反馈恢复且目标不重启。
时延量测是同机反馈生成到新 generation 的 MuJoCo ctrl 写入，
清零时间用实际执行力矩变为零的积分结束时间保守统计。
当前不含 MIT 编解码、CAN、真实电机/电流环，也不承诺硬实时最坏上界。
原始结果与报告按项目归档流程保存 Windows，核验后清理 Orin 本次临时输出。

## 含 MIT 编解码的闭环

现有 MIT 双路接口已接入同一原生阻抗控制器，使用私有 PTY 与 MuJoCo。入口、命令有效期和量化语义见 [MIT 软件闭环](ref01_mit_software_loop.md)。该入口与上节直接 MuJoCo 夹具分别验收；本轮七场景的通信和五类故障停发通过，两轴量化后的静稳恢复尚未通过。

2026-11-11：新增[20 ms反馈与MIT量化的软件验证](sampled_impedance.md)，验证中点预测与直接力矩策略；实机ROS入口尚未选择该路径。
