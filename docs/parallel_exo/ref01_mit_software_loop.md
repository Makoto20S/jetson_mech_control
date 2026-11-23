# PH-EXO：MIT 编解码参与的 MuJoCo 软件闭环

本页的软件验证入口使用官方 Humble controller_manager 和已有的 MIT 双路传输实现。只允许脚本创建的数字编号 /dev/pts/* 端点；没有启动真实电机。实际运行结果和曲线保存在 Windows 的 .local/validation/ 及本地报告目录，不随源码同步至 Orin。

## 数据链

Ref01ImpedanceController（200 Hz）→ Ak30KinematicSystem → 原有 Ak30System / CompositeSystem 授权与 session → USB CDC / MIT 编码 → 两路私有 PTY → MuJoCo → 量化反馈帧 → 原有解码 → 角度、速度、来源时间、序号 → 控制器。

控制器继续使用任务空间 ψ、β 的固定姿态阻抗，MIT Kp、Kd 为零。仿真反馈按 0.1° 和 10 ERPM 量化，每约 20 ms 发送一次。MIT 力矩保持原来的 ±15 N·m / 12 bit 编解码；每档约 0.007326 N·m，名义零码对应约 −0.003663 N·m。本实现没有引入力矩抖动、平滑执行器或人为增加模型阻尼。

本软件夹具显式采用 ERPM→输出角速度系数 2π/(60×21×8)，以及 104→alpha2、105→alpha1、输出角减 45° 的模型映射。这是本次仿真的约定，不能替代实机极对数、减速比、方向、零位及扭矩标定。

## 反馈与命令的有效期

| 层次 | 有效期/周期 | 含义 |
|---|---:|---|
| 控制更新 | 5 ms | 并不意味着每周期都有新反馈 |
| 模拟反馈 | 20 ms | 保留原接收时间和序号 |
| 上游命令有限保持 | 40 ms | 只有新上游 generation 能续期；同代命令修改或到期会锁定 |
| 反馈有效期 | 60 ms | 没有以接收轮询时间替换来源时间 |
| 计划有效期 | 100 ms | 旧计划重放不能续期 |
| MIT 底层硬超时 | 6 ms | 保留既有上限，不为适配 20 ms 反馈而延长 |
| 模拟驱动 watchdog | 60 ms | 仅是仿真停止力矩的假设，不是实机失能验证 |

有限保持层在合法的 40 ms 窗口内每 5 ms 更新底层命令；它不修改控制器的上游 generation，也不伪造新反馈。命令到期先判定失效，迟到的新命令不能复活已失效的保持状态。控制器停用仍通过原有 CompositeSystem 撤销发送授权。故障后的恢复需要重新走生命周期，本测试不自动重启。

## 验证入口

- Windows：tools/project/verify_ref01_mit.py。实际编译并调用原有 C++ MIT codec、原生模型和阻抗计算核心；MuJoCo 数值运行 3 s。它不是 Windows ROS 或实时性能验收。
- 便携测试：tools/project/test_ref01_mit.py、tools/project/test_kinematic_lease.cpp。
- Orin：tools/project/test_ref01_mit_ros.py，复用 test_ref01_manager.py 的官方管理器、接口占用和生命周期检查。
- 新插件：mech_bringup/Ak30KinematicSystem，位于现有 mech_bringup 包内；不新增通信框架或控制器包。

Orin 只在开发副本、持有 .local/sync.lock、源码同步完整、加载 ref01_native overlay 后运行。设置 ROS_DOMAIN_ID=179、ROS_LOCALHOST_ONLY=1、EXO_SYNC_LOCK_HELD=1；最后一个变量必须在实际取得锁后设置。同时将 .local/venv/lib/python3.10/site-packages 加入 PYTHONPATH；使用系统 Python 保留 ROS 的系统依赖。运行参数示例：

    /usr/bin/python3 -B tools/project/test_ref01_mit_ros.py --output .local/validation/<新的运行目录>

七个场景为 psi、beta、loss、stale_state、plan_loss、plan_replay、controller_stop。stale_state 是冻结送给控制器的状态来源字段，底层反馈仍在更新；它不等价于从相同 CAN 数据字节识别出过期传感器样本。

故障验收要求：注入前两路正常发送、两路在注入后 120 ms 内停发、输入恢复后不续发。另行核对模拟驱动 watchdog 后实际施加的模型力矩为零。正常工况要求链路持续到记录结束、没有控制故障，再独立检查两轴恢复：目标误差≤0.5°、角速度≤1°/s，最迟在模型时刻 2.5 s 起持续满足。此门槛在测试前固定，未按结果放宽。

记录包含原始 CDC/MIT 数据、驱动 trace、控制状态、MuJoCo 姿态/速度/实际力矩、接口和生命周期证据。wire_period_ms 是仿真端读取发包的间隔，包含 Python 仿真端调度，不冒充控制线程唤醒间隔或物理端到端时延。主机时钟与模型时间分别记录。

采样期间关闭该有界 Python 夹具的自动循环 GC，退出时恢复原策略；记录容量与运行时长有上限。错误接口不可用时服务停用可能被 Humble 拒绝，夹具保留该错误并校验官方 manager 正常 shutdown，无强杀或自动重启。

当前结果：七场景通信/生命周期检查通过、五类故障停发通过；两轴量化后的静稳恢复尚未通过，不据此启动真实电机。逐帧核验时应使用驱动 JSONL 中的 syscall_write 对齐仿真端 wire_tx；后者时间包含仿真端调度。

## 独立实机台架入口（2026-11-11）

本节与上面的私有 PTY 仿真入口分开选择。tools/project/ref01_bench_hold.py
通过 simulation_only=false、bench_profile=ref01_supported_v1 显式使用受限实机路径，
复用同一原生模型/阻抗控制器、MIT 编解码、来源时间和发送有效期；
不更改原仿真入口的 0.2 N·m 限制，也不修改其他伺服/力矩部署。

当前参数由本次用户指定：单电机目标上限 5 N·m、Iq 上限 5 A、
相对启动编码器位置最大变化 ±30°、最长 30 s；2026-11-11按用户要求将
实机台架目标输出改为Direct，取消0.5 N·m/s对称斜率；减力、过零与反向
在下次正常写入时直接跟随控制器目标，仍经过原MIT量化与异常检查。
输出空间ψ、β两轴刚度均为10 N·m/rad，阻尼均为1.0 N·m·s/rad；
启动确认界面显示参数，result.json记录同一组参数。
MIT Kp=Kd=0。速度保留原始 ERPM 记录，运行中不按 100 ERPM 停机。
底层通用 runtime 仍要求正数阈值，台架配置取 327680 ERPM，覆盖协议
有符号 16 位速度字×10 的全部范围；这不是新的机械额定速度。
启动前的静止检查、反馈有效期、温度、模型有效域、计划有效期和独立失能保留。

实机使用 104→第二路 if02、105→第一路 if00 的稳定设备标识。
程序先核对占用和被动反馈，显示本段参数、姿态及
Type START then Enter 提示。必须由现场操作者在同一个可见终端输入 START；
空输入、其他文字或非交互 stdin 均不能启动。确认后再次被动采集，
位置变化超过 0.3°或不再停稳时退出；只有 HOLD ACTIVE 后再施加扰动。
SSH 输入排队成功不等于保持建立。每段使用新的输出目录，故障后不自动重启。

在已确认授权、机械条件、源码版本和 overlay 的前提下，加载官方 ROS、
tools/project/mujoco_ros_env.sh 与 .local/ros/ref01_native/install/local_setup.bash，
从开发仓库执行：

    python3 -B tools/project/ref01_bench_hold.py --execute --duration 30

省略--output时每次生成新的结果目录并显示路径。显式--output仍必须指向不存在的目录，
不会覆盖已有结果。监督器分别显示travel_limit、feedback_stale、current_limit等实际原因。

退出先向原生控制器请求停止，再结束 manager，逐轴执行原有独立 disable 并检查
0x77 应答。名义零写入与失能应答分别记录；底层已故障时不能保证末帧零写入。
用户急停为现场退出手段，不能把没有采到零帧写成已零输出。
本入口不修改 current/servo-current；结果逐文件核验后归档 Windows。

模型仍只接受 ψ∈[-45°,45°]、β∈[0°,45°]。±30°是相对编码器位移条件，
不会放宽耦合模型工作区。用户说明β的0～45°来自此前运动学测试范围，
实际机构可到−15°；按用户当前要求，本次仅调刚度和阻尼，不扩展软件范围。ERPM→模型角速度沿用候选比例，
运行中速度监测不代表已完成真实输出轴速度或动力学标定。
