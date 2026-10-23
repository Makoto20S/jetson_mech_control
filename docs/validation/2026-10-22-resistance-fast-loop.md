# 2026-10-22 抗阻快环软件验证

任务：EXO-ARCH-001。针对旧路径Python50 Hz计算、下层500 Hz重复发送的边界，
新增通用C++抗阻控制器和50 Hz上层许可路径。详见[架构与参数](../development/resistance_fast_loop.md)。
本轮无真实CAN、串口查询、设备使能、寄存器写入或运动测试；未提交或推送。

## 验证结果

| 检查 | 结果与范围 |
|---|---|
| ARM64构建 | mech_controllers、mech_bringup、mech_assist_control通过；最终安装增量构建2.64秒 |
| 控制器gtest | 53/53通过，含10项快环测试；完整包CTest 3/3，汇总56 tests无错误/失败/跳过 |
| 上层应用回归 | 80项通过，涵盖策略、许可、失联撤销、位置保护、旧路径 |
| DM4310启动检查 | 16项通过，含快环类型、双轴参数、互斥选项、预算范围和默认预览 |
| 下层运行时/插件 | 33/33通过，涵盖反馈过期、授权租约、实际位置保护、启动证据闸门 |
| 安装入口 | 默认离线检查通过；实测范围在URDF/控制器/下层一致，20秒预算显式传递，不打开设备 |
| 安装应用预览 | 使用ResistancePermissionPolicy，无电机命令发布者 |
| 实际插件ROS回环 | GenericSystem、manager500 Hz、上层许可50 Hz；半秒授权计数增加249次；失联后停在299，恢复心跳也不自动重启 |

算法测试用借用位置/速度接口验证每个2 ms周期重新计算，而非重复同一上层目标；
覆盖独立指数低通参考、启动/停止滞回、反向撤力、命令饱和、方向软区、越界、
非有限反馈、周期超时、单调时钟运行预算与锁存。模拟插件回环状态为零，
只验证加载/调度/授权链，不能据此声称真实非零阻力效果通过。

初版回环断言误以为update ERROR后标签必变inactive，实际Humble仍保留active。
已改为验证授权计数停止及恢复心跳不重启，保留首次失败日志。
GenericSystem本身不是物理停机实现；实际设备停组由原有硬件运行时租约处理，
真实验收还必须独立读取失能状态。

## 可复现入口与证据

- 控制器：`colcon test --packages-select mech_controllers`。
- 启动参数：`python3 -m pytest -q ros2_ws/src/mech_bringup/test/test_dm4310_launch.py`。
- 无设备实际插件：`tools/validation/check_resistance_fast_ros.py --output-dir /absolute/new/evidence-directory`。
- 构建/测试外部日志：`/home/nvidia/mech_attitude_validation/fast_resistance_{controller_tests,colcon_tests,runtime_tests,final_build}.log`。
- 最终回环：`/home/nvidia/mech_attitude_validation/fast_resistance_shadow_final/summary.json`及`manager.log`。
- 首次标签断言失败：`/home/nvidia/mech_attitude_validation/fast_resistance_shadow/manager.initial_label_assumption.log`。
- 构建复用已有依赖覆盖层，警告已记录；不是clean clone完整工程认证。

新快环尚未实机运行。下一步应先准备适配新许可路径的有界记录器，记录真实下层命令、
同步反馈、周期和最终失能，再在明确现场授权下验证阻力方向、手感和抖动。
不自动复用旧应用力矩占位值，也不提高保护阈值或运行时长来替代验证。
