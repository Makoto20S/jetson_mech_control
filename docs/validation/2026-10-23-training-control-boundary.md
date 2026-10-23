# EXO-ARCH-001：控制权边界软件验证

## 改动与范围

按用户确认：上层选模式、给目标和参数；C++控制器唯一实时控制律；底层独立保护。
固定位置阻抗新增结构化计划入口，上层arm捕获参考并发送K/D/cap/许可，
C++以同一RealtimeBuffer帧计算实时PD，保留配置天花板和独立硬件保护。
旧enable兼容台架不变；统一节点拒绝尚未迁移的Python策略motor输出。
字段与限制见[接口合同](../development/training_command_contract.md)。
本轮没有打开串口、启用CAN、发送真实电机命令或修改设备/系统配置，没有提交或推送。

## 实际执行结果

- Jetson ARM64增量构建 `mech_controllers`、`mech_assist_control`、`mech_bringup` 通过。
  使用 `/home/nvidia/mech_dm4310_validation_20261022/{build,install,log}`，
  依赖复用旧underlay有警告，不是clean-clone全工作区验收。
- `mech_assist_control/test` 与 `mech_bringup/test/test_dm4310_launch.py` 顺序同域运行，
  合计133 pytest通过；保留一项既有launch Python版本警告。
- 控制器colcon test及test-result通过：80 gtest用例；包级汇总86项，0错误、0失败、0跳过。
  新覆盖原子参数/参考、实际ROS订阅、标量旁路拒绝、正负命令饱和、重放/旧激活拒绝、
  源年龄不因延迟接收延长、会话内参数变化锁存、参考首帧偏差和超时不自动恢复。
- `check_training_architecture_ros.py` 成功加载实际安装C++插件及双轴GenericSystem，
  使用源码上层节点发出K10/D0.25/cap2的固定计划，两个命令世代0→101；
  中断上层反馈后故障为 `actuator feedback unavailable or stale`，上层各发一次撤许可，
  两轴世代在229停止，后续120ms没有继续刷新。输出证据为
  `/home/nvidia/mech_attitude_validation/training_architecture_shadow_20261023_v2/summary.json`。
  固定mock位置为0，此图不证明非零力矩物理效果，非零PD由C++借用接口测试覆盖。
- 文档624项本地链接、144项Python/12项Bash语法、context（11包/17 ADR）及diff检查通过；
  本地memory三文件validator通过，0错误、0警告。

使用不同DDS域隔离验证，不并发启动同域ROS测试。主要命令：

```bash
ROS_DOMAIN_ID=87 PYTHONPATH=ros2_ws/src/mech_assist_control:$PYTHONPATH \
  python3 -m pytest -q ros2_ws/src/mech_assist_control/test \
  ros2_ws/src/mech_bringup/test/test_dm4310_launch.py
ROS_DOMAIN_ID=87 PYTHONPATH=ros2_ws/src/mech_assist_control:$PYTHONPATH \
  python3 tools/validation/check_training_architecture_ros.py \
  --output-dir /home/nvidia/mech_attitude_validation/training_architecture_shadow_20261023_v2
```

后一个输出目录已存在，复测须选择新的外部目录。

## 失败记录与修正

新增激活时间采样最初影响旧并发测试的注入时钟屏障，测试等待住后已终止；
改为仅结构化入口采样激活时间，完整旧并发/跨生命周期测试最终通过。
首个ROS订阅新测试误用未激活LifecyclePublisher，导致78/79通过；
改为独立普通ROS生产者，最终80/80通过，没有放宽生产控制器检查。
首版双轴mock未初始化command_generation状态，初值NaN使差分断言失败；
补齐mock初值0后v2整链路通过，旧日志保留在未带v2的外部目录。

## 未验收与下一步

本次不声明穿戴、机械安全、实际力矩精度或双轴物理效果通过。
上层仍不自动核对远端manager插件类型或管理硬件生命周期。
模式/参数在激活内固定，后续轨迹或在线调参需要新的合同。
下一步将速度抗阻策略接入同等结构化目标/参数合同，避免许可路径参数分散；
原独立台架和历史实机结论不因此失效或扩展。

## PR #34：层级与纯计划合同收敛

后续用户要求参照仓库现有结构严谨分层。上层策略为顶层，感知/状态估计为反馈支路；
[代码结构](../development/code_structure.md)分开展示控制层级和反馈数据流。
继续沿用contracts/policies/supervisor/routing及ROS适配，不建立第二套控制框架。

- 新增不可变`ImpedancePlan`，参考捕获和参数持有从ROS节点移入纯策略；
  静止检查也位于策略start。监督器校验参考/上限及激活内计划不变，计划不进入Python瞬时力矩处理。
- 规范发送分支实际调用`CommandRouter.route_training`，不再绕过信封路由检查；
  全轴校验成功才返回序列化数据，拒绝过期/重放、非法映射、超限及标量出口旁路。
  ROS发送保留信封issued_at，C++实时控制律和独立下层保护未改动。
- 修复Supervisor用关节样本时间比较独立健康诊断时间的问题；ROS传入当前执行时间，
  合法稍新的诊断不再误判未来，真正未来/陈旧输入仍拒绝，旧关节年龄不续期。

在全新外部输出目录`/home/nvidia/mech_pr34_20261023`，11包ARM64联合构建通过；
5b2d580时全包924项测试通过，随后对受影响应用包重建/测试，当前汇总940项（应用132项），
0错误、0失败、0跳过。C++源码未因本轮纯Python计划重构而改变，未重复其全量构建。
新增回归涵盖不可变参考、静止条件、运行中参数变化、双轴整体准入、时效/重放/映射/标量旁路，
以及独立诊断与测量源时刻的正确比较。

实际安装的双轴GenericSystem+C++插件图再次通过，两个世代0→101，
中断反馈后均在229停止，证据`/home/nvidia/mech_attitude_validation/pr34_layered_training_20261023_v2/summary.json`。
五图入口首轮前四图通过，第五图因上述时间比较错误失败；保留原失败日志，
修复后仅重跑受影响的完整IMU→估计→上层图及双轴C++计划图，两者通过。
100Hz合成姿态图500发/500收，不等于真实IMU实机测量频率。
首次直接pytest仅加载Humble而缺少工作区膝角依赖导致收集失败；加载当前安装后源测试通过，
未通过放宽生产检查消除失败。

本次按用户授权更新PR #34；此前4项绿色CI对应5b2d580，不代表后续新提交的CI已通过。
最终以PR当前HEAD检查为准。没有真实设备操作，最终提交实机审核第3项仍待验收。
