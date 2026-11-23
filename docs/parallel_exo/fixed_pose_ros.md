# PH-EXO 普通固定姿态ROS入口
2026-11-12。精简封装；没有新增会话管理节点，中文START、计时和实机退出核验仍由原ref01_bench_hold.py负责。

## 接口
Ref01ImpedanceController的command_source：
- legacy_array（默认）：原~/plan，旧脚本行为保留。
- fixed_pose_v1：仅订阅~/fixed_pose_plan（ph_exo_interfaces/msg/FixedPosePlan），只支持普通阻抗。
两入口互斥。有效首帧绑定非零session_id，sequence递增，issued_at_monotonic_s必须来自同一Orin单调时钟，原100ms计划时效继续生效。一轮内参考、增益和上限不变；停止并重新激活后才能修改。
任务轴序[ψ,β]，motor_torque_cap_nm按[104,105]，入口转换为alpha顺序。~/stop沿用原接口。

~/status为ph_exo_interfaces/msg/ImpedanceStatus，由非实时50ms定时器发布。包含会话、已接受序号、状态/原因、姿态速度、参考及最后计算的电机目标。state_valid仅Following时为true；无效状态中的数值不可当作当前控制状态。
paired_source_time_monotonic_s是成对计算的最旧源时间，不是两个设备采样时刻。目标是计算结果，不是实测轴端力矩或执行ACK。disable_confirmed保持false，控制器不知道旧脚本的独立失能核验结果。
原JSON状态服务保留。PrepareFixedPose只有前阶段定义，本轮不实现、不调用，不扩展会话管理。

## 配置和软件启动
mech_bringup安装模型config资源、ph_exo_fixed_pose.json台架模板和ph_exo_configuration.py配置构造器。
台架模板K10/B0、30秒、10Nm/6A/60°经独立校验；不会自动替换旧脚本参数或覆盖仿真低力矩边界。
新launch仅使用MuJoCo软件后端及原0.2Nm验证范围，控制器加载后保持inactive，没有命令发布器。实机继续旧脚本。

加载开发overlay和MuJoCo环境后：
    ros2 launch mech_bringup ph_exo_fixed_pose.launch.py output:=/本项目中新建目录的绝对路径
output必须不存在。模型从安装后的share查找。启动成功不等于Following，激活与命令由外部调用方负责。

## 验证
test_ref01_manager.py新增--typed，默认仍原接口；按项目锁和ROS隔离域约定运行：
    python3 -B tools/project/test_ref01_manager.py --typed --output .local/validation/唯一目录 --duration 30
必须实际持有sync.lock并设置EXO_SYNC_LOCK_HELD=1，ROS_DOMAIN_ID=179、ROS_LOCALHOST_ONLY=1。
Windows配置4项、manager配置6项及状态33项通过。Orin9包构建通过；typed ψ/β各6001周期，原功能/时序判据通过，loss/duplicate停发通过。最大周期5.070073/5.111099ms，软件反馈至MuJoCo力矩写入最大0.198115/0.200355ms。
typed状态收到session_id=1、Following、state_valid=true。安装后的launch实测inactive且正常退出，无真实设备操作。
这是MuJoCo低力矩夹具验证，不是K10/B0实机稳定性验收。typed入口的完整并发/重激活矩阵尚未复测；MIT量化夹具结果见下节。
原始62文件在Mujoco-Linux/output/slim_package_20261112_archive，逐文件SHA256核对；本轮无控制参数调优。


## 新接口的 MIT 编解码软件闭环
test_ref01_mit_ros.py新增 --typed，复用双私有PTY与既有MIT/CDC实现，仅允许普通阻抗：
    python3 -B tools/project/test_ref01_mit_ros.py --typed --duration 6 --output .local/validation/唯一目录
同样需要实际持有sync.lock与上述ROS隔离环境；只用于Orin开发副本的无设备软件验证。

2026-11-12 Windows入口4项/MIT13项/manager配置6项/同步33项通过，Orin入口4项/MIT13项通过。
七个typed工况的通信与生命周期通过；两轴6秒扰动恢复未通过原门槛，旧接口同参数对照也未通过。
五个故障工况均120ms内停发且反馈恢复不续发；stale_state两次先触发update_period，其独立过期机制未验收。
本轮不调增益、不改判据、不启动电机。完整数据、时序口径与证据边界见本轮报告（项目本地记录 mit_typed_mujoco_20261112.md）。


## 状态冻结原因验收补齐（2026-11-13）
已定位旧stale_state首先报告update_period：硬件on_error同步写盘累计约14.9ms，拉长实际控制器更新间隔。
错误回调现只撤销并保留内存记录，JSONL延后到清理/关闭阶段；两轮软件冻结均按原60ms反馈期限锁存feedback_expired。
stale_state判据现在同时要求原始反馈推进、ROS状态冻结、底层取消和正确首因；单纯停发不再判为原因验收通过。
首故障时序只在旧software_status调试JSON提供，不扩展公共命令/状态消息或会话管理。
其他四类故障回归通过；此前两轴恢复失败未因此改变。详见补验报告（项目本地记录 stale_state_fault_20261113.md）。


## 现有上层管理节点接入（2026-11-14）
普通固定姿态新增既有 exoskeleton_control_node 的可选分支，接入 start/stop/status、软件急停和 reset。使用简短 hold.k/b/tau 配置和 state/q_ref/stop_ok/disable_ok 状态；不新增会话管理进程，原脚本保留。
接口、生命周期和验证边界见[上层阻抗接口](upper_impedance.md)。


动态普通阻抗可选入口见[动态参考接口](dynamic_reference.md)。本文固定姿态约定保持不变。
