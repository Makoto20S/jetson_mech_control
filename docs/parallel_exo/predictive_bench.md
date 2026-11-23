# PH-EXO 预测阻抗实机候选启动说明

核对日期：2026-11-12。当前为实验候选，尚未通过原定的完整扰动恢复验收；本轮没有启动电机。

## 当前状态

- Windows 接收时间误差用例 64/64 通过；同一预测核心已接入 ROS 控制器和主机接受的 MIT 编码目标历史。
- Orin 开发副本构建通过，回归结果汇总 788 项无错误；两项旧伺服测试因长临时路径失败后，改用本项目 output/tmp 重跑通过。
- Orin ROS–MIT–MuJoCo 两轴连续运行及五类故障停发通过。
- **恢复性能未通过**：ψ/β 尾段速度分别为 1.686/1.360 °/s，原门槛为 1 °/s；尾段角度误差为 0.124/0.113°。不放宽门槛，不把此候选称为已完成稳定性验收。
- 完整报告仅保存在 Windows：docs/parallel_exo/reports/predictive_impedance_readiness_20261112.md。下列命令是待操作说明，本轮未执行。

## 固定参数与接口

| 项目 | 本次设置 |
|---|---|
| 控制器 | 现有 Ref01ImpedanceController，显式 --predictive |
| 输出空间刚度 | ψ、β 均 10 N·m/rad |
| 输出空间阻尼 | ψ、β 均 1 N·m·s/rad |
| MIT 驱动增益 | Kp=0、Kd=0，Orin 计算前馈目标力矩 |
| 参考姿态 | 启动前捕获，并在终端 START 后重新被动核对 |
| 电流 / 目标力矩 / 位移 | 6 A / 每电机 10 N·m / 相对启动位置 ±60°（2026-11-12用户指定） |
| 时长 | 最长 30 s |
| 力矩通道 | Direct；无原 0.5 N·m/s 慢斜率 |
| 速度 | 记录 ERPM，无有效量程内 RPM 停机阈值 |
| 预测时间依据 | 主机接收时刻 + 0～7 ms 候选估计；不是实测电机采样时刻 |
| 发送历史 | 主机 transport 接受的编码目标、主机提交时刻、序号；不是电机执行 ACK |

## 终端入口

软件准备完成不等于本轮已批准执行现场试验。再次开始时应先告知现场，确认机构独立支撑、无人穿戴、操作人可用急停；保留用户亲自输入 START 的流程。

Windows 可见终端先连接：

    ssh orin-nx

然后在 Orin 终端准备环境（这些命令不启动电机）：

    cd /home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
    source /opt/ros/humble/setup.bash
    source tools/project/mujoco_ros_env.sh
    source .local/ros/ref01_native/install/local_setup.bash
    export PYTHONDONTWRITEBYTECODE=1

确认接受当前候选尚未达到恢复速度门槛这一实验边界后，才手动运行：

    python3 -B tools/project/ref01_bench_hold.py --execute --predictive --duration 30

此入口会打开既定电机端口进行被动读取，然后停在 START 提示；提示前不会开始阻抗保持。AI 不代输 START。等待期间保持静止；输入后再次核对姿态，显示 HOLD ACTIVE 后才推动机构。

## 一轮试验的观察顺序

1. HOLD ACTIVE 后先保持静止约 5 s，记录是否自发漂移或抖动。
2. 沿 ψ 轻推、松手并记录松手时间；待响应结束再沿 β 操作。首轮不同时强推两轴、不顶机械限位。
3. 比较是否产生正确恢复方向、是否松手后继续加速、是否出现持续振荡；有异常立即停止该轮。
4. Ctrl+C 停止续发并执行原独立失能流程；正常超时也执行同一退出流程。以 DISABLED CONFIRMED: 104, 105 为软件确认；若显示 DISABLE UNKNOWN，现场断电，不自动重试。
5. 程序打印唯一结果目录。归档原始反馈、目标力矩、状态、首个退出原因和失能确认到 Windows，逐文件核验后清理 Orin 本轮临时结果。不要只保留汇总图。

实机采样相位、编码器/速度取整、转子惯量、摩擦和轴端力矩未因此完成标定。机械限位与急停也不构成控制稳定性证据。历史实机问题与仿真注入错相的区分见 Windows 的 sampling_skew_attribution_20261112.md。


## 普通阻抗对照：完整旁路预测（2026-11-12）

用户要求先剥离预测校正。现有控制器的普通分支已能完整旁路预测，入口新增明确的 --ordinary；它与 --predictive 互斥，未给模式参数时仍保持原普通默认。

在上述Orin环境已准备、现场条件已确认的情况下，下一次人工选择普通对照的命令为：

    python3 -B tools/project/ref01_bench_hold.py --execute --ordinary --duration 30

命令执行后仍等待用户亲自START，本次开发没有执行此命令。终端显示 ordinary measured-state impedance (prediction disabled)，结果control_mode为ordinary_measured_state。切换前必须让上一轮正常退出，不能热切换正在运行的控制器。

普通路径：电机位置/速度 → MuJoCo运动学q/dq、重力及雅可比 → g+K(q_ref−q)−B*dq → 双电机分配 → 原MIT编码。K10/B1、MIT Kp/Kd=0、5A/5Nm/30°/30s、Direct输出及原失效检查不变；无自适应惯量、外力预测、隐藏延迟时刻搜索、历史预测重放、中点预览或联合预测量化选择。

普通模式使用两轴各自最新的真实反馈；等两轴序列均更新后才重算目标，其余周期由既有命令租约保持。原反馈时间不刷新，不声称两轴同时采样。故障/过期仍按原流程停发。因此这是整条预测路径的旁路对照，并非只删除一项校正的严格单变量消融；更新节奏和量化选择也与预测模式不同。

Windows对这次实机反馈的6000步离线重算未触发状态/模型故障，但105最大相邻目标跳变约1.11Nm，高于原预测记录0.64Nm。去预测不等于去掉震荡，也不代表普通模式闭环已通过稳定性验收。先保留该对照入口，不自动启机或调大增益。完整离线结果仅在Windows的reports/ordinary_ablation_20261112.md。

## B=0普通阻抗实验（2026-11-12用户明确选择）

这次单变量对照保持K_psi=K_beta=10，显式使用两轴B=0，关闭预测。启动目标仍取本轮启动姿态，MIT Kp/Kd=0，30 s、每轴5 Nm、5 A和相对30°限制保持。零阻尼表示上位机不施加−B*dq，重力补偿与弹性恢复仍在；不是零力矩测试。反馈新鲜度与其余退出检查继续工作。

    python3 -B tools/project/ref01_bench_hold.py --execute --ordinary --damping 0 --duration 30

进入后核对终端显示 stiffness (10.0,10.0)、damping (0.0,0.0)，亲自输入START；显示HOLD ACTIVE前保持静止。先观察静置，再小幅推动与松手，保留松手时刻。B0在本轮模型对照未通过恢复，仅作为用户指定的诊断实验，不标记稳定性通过。Ctrl+C或30秒结束走原失能流程。

省略--damping继续使用B1，不改原预测候选；可变阻尼仅限普通模式。参数在终端、实际发布计划和result.json中使用同一个值。不得把主观手感变化直接当完整闭环验收。

2026-11-12：终端提示改为中文，START确认保留。上述位移为电机编码器相对启动角度，不修改ψ、β模型有效域。上限调整不代表10 N·m下的实机稳定性已验证。

## 普通阻抗中文调参入口

使用[中文调参界面](bench_panel.md)设置两轴K/B和限幅、查看双电机波形并用界面开始/停止（无预设时长）。
此入口替代本轮终端确认，原命令行START流程及预测候选脚本仍保留。
