# PH-EXO 动态参考入口

在原 Ref01ImpedanceController 中增加普通阻抗的动态参考；没有新增控制节点、会话管理或步态生成器。K/B 在一次激活期间固定。原固定姿态接口、上层 start/stop/reset 与调参脚本继续使用原模式，默认行为不变。

## 选择与接口

控制器配置 command_source: dynamic_ref_v1 时仅订阅 ~/ref，类型 ph_exo_interfaces/msg/ImpedanceRef。
fixed_pose_v1 仍订阅 ~/fixed_pose_plan；legacy_array 仍订阅 ~/plan。三个入口互斥；动态模式暂不接预测阻抗。

| 字段 | 含义 |
|---|---|
| id | 非零运行标识，首次有效帧绑定；必须先停用/重新激活才能更换 |
| seq | 严格递增整数，1..2^53−1 |
| t | 生成这一参考的 Orin 同机单调时钟秒数，不是 Windows 时间、ROS 仿真时间或接收时间 |
| on | true 启用；有效 false 帧沿用停发逻辑 |
| q | [ψ,β] 目标角度，rad |
| dq | [ψ,β] 目标角速度，rad/s；应为 q 轨迹的导数 |
| k, b | 两轴刚度和阻尼，沿用现有单位与范围；每帧携带，但单轮不允许变化 |
| tau | [104,105] 每台电机力矩上限，N·m；单轮不允许变化 |

同一消息原子更新 q/dq，不分别订阅角度和速度。控制律为
Aᵀ τ = g + K(q_ref−q) + B(dq_ref−dq)，保留原力矩分配和成对限幅。
不新增惯性前馈，也不自动插值或外推参考；两个参考之间保持最后一组 q/dq。
新参考在下一个新鲜的双电机反馈对上参与计算，控制周期仍为 5 ms。

## 启停与有效性

1. 加载/配置上述模式，先读取当前有效姿态，再激活控制器。
2. 第一帧从当前静止姿态开始，dq 为零；保留原初始位姿误差 0.01 rad、速度 0.05 rad/s 条件。之后平滑改变 q/dq。
3. 建议在 Orin 上每 20 ms 发布一帧。本次验证采用这个频率；轨迹更快时由调用方选择频率。
4. 沿用 100 ms 源时间有效期；重复序号、旧时间、未来时间、错误 id、非有限数及超原范围的 q 不接受，也不续期。无有效更新最终报 plan_expired，停发新力矩。
5. 单轮改变 K/B、上限报 plan_changed；故障锁存，恢复参考本身不恢复输出。
6. ~/stop 沿用现有停止服务。恢复仍需明确停用/重新激活；没有新增恢复权限或绕过现有检查。

~/status 继续返回实际状态、已接受序号和 reference_rad，新增 dq_ref 和 dynamic 便于区分参考模式。参考字段表示最近接受的输入，不是电机已执行证明；电机目标仍由既有状态字段提供。接口生成代码变动后，新启动的 ROS 进程需加载本次 overlay。

## 调用片段（在 Orin 的上层发布者中）

~~~python
import time
from ph_exo_interfaces.msg import ImpedanceRef

pub = node.create_publisher(ImpedanceRef, "/<namespace>/ref01_impedance/ref", 1)
# 激活后，以实际当前姿态作为第一帧；下面变量由上层提供。
seq += 1
pub.publish(ImpedanceRef(
    id=run_id, seq=seq, t=time.monotonic(), on=True,
    q=q_ref, dq=dq_ref, k=k_fixed, b=b_fixed, tau=tau_fixed))
~~~

旧固定姿态上层管理节点仍只管理固定模式；动态参考调用方独占新入口，并负责轨迹及激活/停止。无需让旧上层同时发布固定计划。调参界面本次不增加轨迹编辑器。

## 软件入口与验证

原软件 launch 增加可选项，默认仍为固定姿态：

~~~bash
ros2 launch mech_bringup ph_exo_fixed_pose.launch.py output:=/绝对路径/新的软件目录 command_source:=dynamic_ref_v1
~~~

该入口始终是 MuJoCo 软件硬件，控制器加载为 inactive，不发布参考、不连接真实电机。
回归脚本：tools/project/test_ref01_dynamic.py；Windows 通过本地 native probe 调用同一 C++ 核心，Orin 使用独立 ROS 域 179、正式 controller_manager 与 MuJoCo 插件。
验证数据保存在 Windows .local/validation/dynamic_reference_20261114/。


## MIT 编解码软件验证入口（2026-11-15）

同一动态接口已接入专用测试 runner：
tools/project/test_ref01_dynamic_mit.py --output .local/validation/新的目录
在 Windows 使用工程正式 C++ codec、Session 和 MuJoCo 进行数值循环。

Orin 加载 ref01_native overlay，并持有 .local/sync.lock、
EXO_SYNC_LOCK_HELD=1、ROS_DOMAIN_ID=179、ROS_LOCALHOST_ONLY=1 后，加 --ros。
仅生成两个私有 /dev/pts 端点，不接受实体端口；复用原MIT/CDC协议栈和MuJoCo。
本次固定K/B的接口与参考中断停发通过，但轨迹跟踪未达标；
完整数据、判据与限制见MIT动态软件报告（项目本地记录 dynamic_mit_20261115.md）。
测试保持原截断量化，不修改实际台架配置。
