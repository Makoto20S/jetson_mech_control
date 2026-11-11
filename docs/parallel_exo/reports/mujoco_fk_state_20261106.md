# MuJoCo 正运动学接入 ROS 状态链验证

日期：2026-11-06，Asia/Shanghai。正式源码位于统一控制仓库；本报告仅保留在Windows和Git，不同步到Orin。

## 结果与实现

已按用户最新要求采用MuJoCo模型正运动学。运行链为电机JointState → alpha1/alpha2 → MuJoCo闭链位置约束 → RobotState.q_position=[ψ,β]。解析FK试稿已撤回，原有逆解/动力学API与MJCF/STL未修改。

- 复用现有模型的约束残差与雅可比，固定主动角，求解六个从动坐标；不调用解析REF01正/逆解，不推进仿真时间。
- 新增q_position、q_valid、q_reason、q_closure_residual_m；坏数据、断流、时钟回退或求解失败均不会保留旧的有效q。
- q仅依赖电机角度；IMU保持独立，未实现上层阻抗输入或步态识别。
- 原模型分支ψ∈[-45°,45°]、β∈[0°,45°]，位置约束残差阈值1e-11 m；角度边界允许1e-8 rad数值容差。
- mapping_confirmed默认仍为false。现场需确认零位、方向与角度来源后启用；本轮仅在模拟反馈测试中设为true。未新增q速度估计。

## 验证

| 环境 | 验证内容 | 结果 |
| --- | --- | --- |
| Windows，独立.local/venv，MuJoCo 3.17.0 / NumPy 2.2.6 | 16,736组冻结输入对照、冷启动/跳变、无效输入与失败恢复、模型几何变化响应 | 通过；最大q偏差约1.02e-10 rad，最大闭链残差约9.97e-12 m |
| Windows | 26项状态逻辑 + 4项MuJoCo FK测试（ROS消息替身） | 30项全部通过 |
| Windows | 原逆解16,736组、原动力学100组及75点冻结轨迹回归 | 通过 |
| Windows / Orin | PowerShell解析、Git差异格式 / Bash语法与启动参数入口 | 通过 |
| Orin，ROS 2 Humble | state组合构建；真实生成消息、MuJoCo FK、DDS收发/断流、时序记录器 | 54项pytest断言用例全部通过：FK 4、状态26、DDS 2、时序22；colcon含4个CTest包装计数为58 |
| Orin | servo_imu综合组合重新构建 | 10个包通过；本轮没有重跑整个组合的驱动测试 |
| Orin安装后节点（独立进程） | 无源码模块路径注入，localhost/domain 176，模拟电机反馈，不发送IMU | ψ=0.17453292519943264 rad，β=0.34906585039886506 rad，即10°/20°；残差2.78e-17 m |
| Orin安装后节点 | 停止电机模拟消息，继续接收状态 | q_valid=false，q_position为NaN，q_reason=joint_source_invalid；收到103条状态消息 |

软件测试没有打开串口/CAN、启动电机或采集IMU。上述数值是模型一致性和ROS软件链证据，不作为本轮实机精度或整机实时性能结果。

## 同步与复现

Orin开发目录：`/home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control`。
受管源码725个文件，清单摘要：`b7c9e6b44113c278f71f17855f7b2221e61627b7f9220683296a2ef015a84440`。
Windows与Orin各自建立本仓库`.local/venv`，按同一锁定依赖恢复；没有复制跨平台虚拟环境。

日常恢复、同步和调用见[状态接口说明](../realtime_state.md)、[新设备恢复](../restore.md)。

```text
Windows: .local/venv/Scripts/python.exe -I -B tools/project/restore.py verify
Windows: tools/project/Sync-Orin.ps1 sync
Windows: tools/project/Sync-Orin.ps1 build -Profile state
Windows: tools/project/Sync-Orin.ps1 test -Profile state
Orin只读状态节点: bash tools/project/ros_workspace.sh run-state state
```

源代码改动集中于：
- `mujoco/ref01/ref01_mujoco_kinematics.py`：MuJoCo FK适配。
- `ros2_ws/src/mech_exo_state/`：状态字段、节点、模型安装与软件测试。
- `tools/project/`：Windows验证、仓库环境选择、恢复与ROS运行入口。
- `docs/parallel_exo/`、`manifests/ref01_state_sources.json`：接口、恢复和来源记录。

## 证据入口

- 共享观察日志：Windows恢复/验证 30ec38a9；同步 5d0c7711；Orin依赖 fca0edfc；state构建 420fb57c；state测试 e85ff796；安装后验证 2cd73c6e；综合构建 77454330；最终逐项统计 1e9dbcdf。
- Orin构建日志：`.local/sync/build-state-7d43c5081add48a5827e9828ae9effd7.log`。
- Orin测试日志：`.local/sync/test-state-45c91435b757437e8527a3c0b532eb4f.log`，JUnit位于`.local/ros/state/build/mech_exo_state/test_results/`。
- Windows安装后测试脚本及修改前副本：`并联外骨骼-ROS框架/Mujoco-Linux/output/fk_state_20261106/`。

两个发布入口仍指向原版本：`parallel_exo_ros_framework/current` → `releases/ref01-cad-dynamics-20261103-v01`；`jetson_mech_control/servo-current` → `releases/servo-two-bus-20261008-v1`。此次仅更新开发副本与开发构建，没有提交、推送或创建PR；共享SSH实现及维护方资料未修改。
