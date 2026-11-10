# MIT 双轴原始反馈接入 ROS 状态链

验证于 2026-11-10。开发工作树 ph-exo/development，基线 35c8bab，受管源码清单摘要 6726b47164e1b9b66c538dd58ac20434f98c7a14baeb1155ccf0934d1d05d66e。验证顺序为 Windows 历史回放、SSH 同步、Orin 软件测试。两路真实电机均未启动。

## 软件接口

Ak30System 收取逻辑总线 2 的 104 和总线 1 的 105 的原生 0x29 报文。MIT 驱动的同帧只读接口经 joint_state_broadcaster、/mit_comm/dynamic_joint_states 进入 /ref01/state。mit_motors 固定顺序为 [104, 105]。每轴输出原始位置（度）、电气转速（ERPM）、Iq（A）、板温（摄氏度）、原始状态/故障字节、样本是否存在、接收序号、主机单调时刻（微秒）、年龄与有效原因。重复广播旧序号不会产生新样本。单轴超时或故障保留最后收到的数值并撤销 valid，正常轴独立更新。默认超时 60 ms，两轴组合位置最大接收时差 30 ms。来源是同机传输接收时间，回放与测试均未测量电机内部采样时刻。

ref01_mit_state.yaml 选择原始 MIT 来源，两轴均有效且时差合格时写入 joints.position（rad）。Iq 与 ERPM 保留在设备单位字段中，joints.effort、joints.velocity 留空。现场机构零位、方向与轴侧确认仍由 mapping_confirmed 管理，随附配置为 false。IMU 不参与两轴反馈判定。接口及只读启动方式见 ../realtime_state.md。

## Windows 历史回放

使用 2026-11-06 双机零增益、名义零力矩运行的完整收帧文件。读取前用归档的 remote_manifest.json 校验两个 SHA256。解析扩展标志、ID、总线、载荷长度并按历史主机接收事件时标合并。使用正式 StateCore 的消息替身回放，逐帧把状态字段重新编码，与原始八字节比较。窗口外的启动和退出记录也计入全程。没有将历史数据复制到 Orin。

| 电机 / 总线 | 全程逐帧一致 | 20 s 窗口 | 位置 ° | ERPM | Iq A | 板温 °C | 状态字节 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 104 / 2 | 1220 | 1000 | 71.9–72.0 | 0 | −0.04 至 +0.17 | 42 | 0 |
| 105 / 1 | 1234 | 1000 | 44.9–45.0 | 0–20 | −0.03 至 0 | 41–42 | 0 |

- bus2_motor104.jsonl SHA256 a528f63db9f0f7eee1b8829a7dea67a4b385923d39d6c68b57a28b81b17b5532。
- bus1_motor105.jsonl SHA256 6b29a60c3ed423d310ce5b84b3aeaf48f7994bd6c7dc9d3cf5e09920559199c6。
- Windows 运行 verify_state_windows.py --mit-replay 加归档路径，状态/FK 共 31 项、另行运行的 MIT 通信回归 11 项通过。共享观察日志编号 17a631dd。

## Orin 软件测试

- Orin 开发副本构建 ROS 2 Humble 的 servo_imu 组合，10 包通过构建。新的 Ak30System C++ 用例以 FakeSerial 经过实际 CDC 解码，覆盖同帧字段、重复 read、故障码 3 及无电机命令。
- mech_exo_state 的 JUnit 共 56 项通过，含状态核心 27 项、真实 ROS 图 3 项、FK 4 项、时序工具 22 项。ROS 图用两台输入源的消息模拟逆序命名、105 故障、104 超时及 105 恢复。原始反馈接收年龄与广播周期分别处理。
- 软件端到端测试通过。test_mit_feedback_pty.py 在本机独立 ROS 域使用两路新建的 /dev/pts 和实际 AK30 插件，仅激活状态广播器。104 输入 02d00000000c2a00，输出 72.0°、0 ERPM、+0.12 A、42 °C、0、有效。105 输入 01c20000fffe2a00，输出 45.0°、0 ERPM、−0.02 A、42 °C、0、有效。回放时两虚拟端口的电机命令总数为 0。日志编号 1ab59c99。
- 综合组合首轮出现三个既有伺服夹具失败。servo_motion_ros、servo_two_bus_ros 遇到 AF_UNIX path too long，activate_position_controller 的限时拒绝断言收到返回值 3，预期 4。单独重测时将 TMPDIR 设为指向同一项目本地临时目录的短路径，三个用例均通过。首轮日志编号 3ab6092e，复测 01080ff0。此记录区分首轮和定向复测。
- 独立 state 配置同版构建及测试通过，colcon 汇总 60 项（含 4 个 CTest 包装及 56 个 JUnit 用例），0 错误、0 失败、0 跳过。同步状态核对 730 个受管文件完全一致。日志编号 6eb9228f、db9a1340。
- 仅更新 /home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control 的源码及开发构建。报告保存在 Windows，后续硬件验证需独立进行。
