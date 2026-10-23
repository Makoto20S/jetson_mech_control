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
