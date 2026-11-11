# PH-EXO 实时状态接入（EXO-STATE-001）

2026-11-06。本模块是ROS观测支路：订阅已有反馈，发布统一状态；通过现有MuJoCo模型完成电机角度到ψ、β的闭链正运动学，不包含控制律或设备启动。沿用本项目的本地编辑、SSH同步、Orin构建流程。

## 输入与输出

新增一个包 `mech_exo_state`，默认订阅 `/joint_states`（sensor_msgs/JointState）、`/imu/imu1/data`、`/imu/imu2/data`（sensor_msgs/Imu），以100 Hz发布 `/ph_exo/state`（mech_exo_state/RobotState），均用SensorDataQoS。发布频率不是采样频率，也不是200 Hz整机实时性能验收。主题可通过启动参数调整。节点使用锁定的MuJoCo 3.17.0、NumPy 2.2.6和仓库内REF01 MJCF/STL。

关节按名字匹配，输出固定为motor104_joint、motor105_joint；多余关节允许存在，缺失/重复名字、非有限值及错误数组长度使关节源立即无效。position必需，velocity/effort允许空数组，不能以零代替缺失值；默认关节源沿用上游 SI 单位。MIT 专用源的转速换算与有效条件见下节。

模型输入顺序为alpha1、alpha2，历史配置为 `alpha1=theta105-pi/4`、`alpha2=theta104-pi/4`。当前伺服配置已将设备度数转换成ROS弧度，状态层不再换算度数或除减速比。映射缩放与偏置按alpha顺序配置，速度仅使用相同缩放，不加偏置。

`alpha_position`保留历史映射候选值；`mapping_confirmed=false`默认令`alpha_valid=false`。核对现场零位/方向/传动关系和机器人身份后，使用本机配置文件显式确认。该开关不是控制授权。速度缺失时alpha_velocity为NaN且alpha_velocity_valid=false；不得直接将其送入动力学计算。所有参数只在启动时设置，运行中修改被拒绝。

两路IMU保留各自header/frame、四元数、角速度、加速度和协方差；三个字段分别判有效。协方差首项-1表示不可用，全零表示精度未知，允许使用但不表示精度已验收。四元数非有限、零范数或平方范数偏离1超过0.05时orientation无效，不自动归一化掩盖错误。不将IMU的Euler角直接当REF01的psi/beta；安装变换、相对P0及融合另行接入。

## MIT 力矩模式的双轴反馈

本节说明如何消费状态。下发目标的轴序、CAN 载荷示例、刷新频率和逐轴失能应答见 [104、105 的 MIT 纯力矩接口](mit_communication.md)。

现有 MIT 驱动按逻辑总线和驱动 ID 接收 0x29 帧。Ak30System 导出同一接收帧的九个只读接口，joint_state_broadcaster 经 /mit_comm/dynamic_joint_states 发布。状态节点在 /ph_exo/state 的 mit_motors[0]、[1] 中分别保留 104、105，逐轴给出 position_deg、erpm、iq_a、temperature_c、fault_code、has_sample、rx_sequence、host_rx_us、age_ms、valid 和 reason。fault_code 保存完整状态字节，包括失能应答 0x77。首次收帧前检查 has_sample，连续广播同一帧不会刷新其年龄。时间戳是同一台 Orin 的单调主机接收时刻，无法推算电机内部采样时间。

选择 config/ph_exo_mit_state.yaml 后，关节位置取自两轴的新鲜反馈并由度换算为弧度。每轴默认 60 ms 超时，故障、驱动无效或超时即撤销该轴有效性，最后一帧原值可供诊断。两轴均有效且接收时间差不超过 30 ms 时，sources[0] 才有效。每轴另保留 output_velocity_rad_s 与 output_velocity_valid。候选输出轴速度为方向符号 × ERPM × 2π / (60 × 极对数 × 电机内部减速比)，默认按 [104,105] 配置 14 对极、8:1 和正号。该换算对应电机减速器输出轴。两轴逐项核实速度比例和符号并设置 mit_speed_confirmed 后，有效关节对才填充 joints.velocity。逐轴速度失效后最后换算值仅供诊断。模型 α 速度另受 mapping_confirmed 约束。Iq 按安培保留，MIT joints.effort 留空。ROS 伺服关节源仍为默认配置，MIT 模式独立确认机构零位、方向及轴侧映射后方可启用 mapping_confirmed。

在 Orin 开发副本中只启动 MIT 状态接收器可执行

    bash tools/project/ros_workspace.sh run-state servo_imu config:="$PWD/ros2_ws/src/mech_exo_state/config/ph_exo_mit_state.yaml"

Windows 历史回放使用 verify_state_windows.py --mit-replay <包含两份总线JSONL的归档目录>。Orin 软件验证使用 test_mit_feedback_pty.py 在私有 PTY 回放两条不同的历史报文，只加载状态广播器。修改 RobotState 消息后需用同版消息重新构建消费节点，并重新 source 开发安装层。

## MuJoCo 正运动学

状态链为 `JointState → 按名字匹配θ104/θ105 → alpha1/alpha2 → MuJoCo闭链投影 → q_position=[ψ,β]`，所有角度为rad。实现使用MJCF的几何与MuJoCo约束残差、雅可比，不在ROS状态链调用解析正/逆解。

`ref01_mujoco_kinematics.py`加载现有`ref01_dynamics_cad_v1.xml`一次，固定两个主动关节，数值补齐其余六个关节。`mj_fwdPosition`提供位置约束信息；每个主动角延续步最多0.15 rad，每步最多20次牛顿更新、每次最多8次回溯。收敛标准为最大约束位置残差≤1e-11 m，并检查原模型分支ψ∈[-45°,45°]、β∈[0°,45°]（边界数值容差1e-8 rad）。不自动绕回角度、不扩大到负β分支；不推进仿真时间，不运行`mj_step`，无需力矩输入。

新增`q_position`、`q_valid`、`q_reason`、`q_closure_residual_m`。必须具备有效电机源、已确认映射、收敛且在分支内，才发布有效q；否则q与残差为NaN，原因明确记录。源时间/接收年龄沿用`sources[0]`，header仍是聚合发布时间。失败后保留内部求解种子便于恢复，但不会把旧结果发布为当前有效值。ROS时钟回退重置种子。

IMU是否存在、有效或对齐都不影响q求解；IMU保留独立字段，阻抗输入与步态识别留在上层。此处没有新增IMU解算，也不输出尚未实现的q速度。

单独对主动角调用`mj_forward`不会自动求出未知的闭链关节位置；此接口补齐所需的位置约束，不另建几何公式。参考[MuJoCo官方计算接口](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)。数值收敛、ROS链路软件测试与实机零位/精度验收分别记录。

## 时间与有效性

`sources[0..2]`依次对应关节、IMU1、IMU2，记录ROS源消息stamp、接收ROS stamp、源年龄、单调接收年龄、接纳计数、valid和reason。`sample_time_verified`始终false。TTL按主机单调时钟计算，默认100 ms；ROS源年龄额外用于拒绝排队旧包和时钟异常，不能延长单调TTL。

拒绝零时间、未来超过5 ms、旧于100 ms、重复或倒序源stamp。收到坏帧立即使该源无效，旧的合法值不再当作当前有效状态。重复消息不刷新单调接收时间。ROS时钟回退清空缓存及时间高水位，等待新的输入。暂停仿真时钟时，steady定时器仍发布超时状态。时钟重置后无法仅靠header辨识旧epoch的所有排队帧，回放应清空旧队列并重启源。

`messages_aligned`仅表示三源消息有效且源stamp最大差不超过30 ms；不插值、不外推、不宣称物理同步。IMU某字段不可用时还需消费对应字段valid，不能只检查messages_aligned。

现有JointState通常带广播时间，CtrBoard在解码包时用ROS now()打戳；196字节旧协议没有每IMU独立采样新鲜度字段。因此 `hardware_freshness_verified`始终false，不能用此消息替代底层CAN新鲜度/故障锁存或授权控制。MIT 来源已提供每轴主机接收序号、年龄和故障状态。传感器内部采样时刻与跨设备同步仍待验证。

## 状态接口命名

PH-EXO 使用节点 ph_exo_state、话题 /ph_exo/state 和启动文件 ph_exo_state.launch.py。常规与 MIT 参数文件分别为 ph_exo_state.yaml、ph_exo_mit_state.yaml。自定义参数文件的节点键使用 ph_exo_state，新启动入口也可读取以 ref01_state 为键的既有现场文件。仍需旧节点、话题和参数文件的调用方可显式运行 ros2 launch mech_exo_state ref01_state.launch.py；该入口发布 /ref01/state。两个入口按需选用一个。

## 构建与使用

在本地仓库根目录：

```powershell
.\.local\venv\Scripts\python.exe -I -B tools/project/verify_state_windows.py
.\tools\project\Sync-Orin.ps1 sync
.\tools\project\Sync-Orin.ps1 build -Profile state
.\tools\project\Sync-Orin.ps1 test -Profile state
```

按用户要求，每次修改后先在Windows执行上述本地验证，通过后才同步Orin。sync入口也会自动执行该校验，失败时不建立传输。先按恢复说明创建本仓库`.local/venv`；同步入口优先选用它，也可用`-Python`指定含锁定依赖的解释器。Windows检查实际MuJoCo正解（16736组冻结基准、冷启动/跳变、失败恢复）并使用消息替身执行状态逻辑测试，校验Python/XML语法；不等同ROS消息生成、序列化或DDS验证。Orin继续用真实生成消息重跑同一组测试，并执行ROS收发/断流测试。

`state`组合仅构建状态消息与节点，不编译或启动设备驱动。需要状态和IMU驱动源码同时构建时使用servo_imu/all等对应组合，设备启动仍独立执行。

在Orin开发副本中，已有反馈源运行时：

```sh
bash tools/project/ros_workspace.sh run-state state
```

可在命令末尾传 `config:=/绝对路径/ph_exo_state.local.yaml` 指定现场配置。入口加载本仓库Python依赖和所选ROS安装层；手动source ROS/overlay时，还需`source tools/project/mujoco_ros_env.sh`。该launch只启动状态节点，无源时持续报告waiting_for_data。不自动打开串口、CAN、不启动controller_manager、不发送命令。无设备测试使用独立ROS domain 176、localhost和唯一主题命名空间。

## 历史依据与后续边界

- ADR-005：源时间与接收时间分开、单调TTL、不得用now()刷新旧样本。
- ADR-016及ADR-020：反馈质量诚实、未知不能填零；本伺服只有position已确认。
- config/parallel_exo/servo_pair.ref01.json：motor104/105命名、度→弧度映射；不代表机构零位认证。
- Windows `REF01_CAD运动学验证_20261103_v01.txt`：历史theta到alpha映射、坐标/分支、16736姿态及归档实验重放。
- Windows `REF01正逆动力学接口.txt`：q=[psi,beta]，模型力矩定义在机构输入轴；本节点的alpha不是该动力学接口的q。

来源版本、读取时间及SHA256见 `../../manifests/ref01_state_sources.json`；只引用，不修改历史文件。完整ROS→模型→控制输出和200 Hz整机时序不属于本切片。本切片已用MuJoCo获得psi/beta；后续仍需贯通硬件反馈质量、确认现场映射，并用现场运动记录核对输出轴速度。MIT测试工具中的占位状态不能当作实测角度输入。
