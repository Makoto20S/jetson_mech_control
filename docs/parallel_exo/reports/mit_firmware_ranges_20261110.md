# PH-EXO 双轴 MIT 速度与力矩量程核对

2026-11-10 · 历史归档核对与项目负责人确认

## ROS 配置决定

2026-11-10，项目负责人明确确认 104、105 的 MIT 前馈力矩字段（t_ff）均采用 ±15 N·m，与 ROS 的 AKE60-8 编码档一致。该数值为 12 位字段的归一化端点。此次确认仅覆盖力矩。速度字段的 ROS 编码为 ±40 rad/s，逐机固件速度量程仍待确认。逐轴限幅、反馈 ERPM 换算及 Iq 到力矩的换算各有独立的参数。

| 对象 | 能追溯的记录 | 对固件量程的判断 |
|---|---|---|
| 104，CAN 0x68 | 9 月 motor1 台架记录确认 ID。适配设计记载固件名 AKE60_8_DE_V3.4，标为操作员提供、未独立核实。+0.1 N·m 时 Iq 推算为 0.081–0.111 N·m。低增益速度命令 0.8、1.0 rad/s 对应运动学估计 0.608、0.784 rad/s。 | 力矩 ±15 N·m 由负责人确认，速度待确认。历史响应仅用于核对方向和数量级。 |
| 105，CAN 0x69 | 11 月双轴记录可追溯到本轴命令与反馈。查阅的档案中未找到逐机固件版本与对应的 MIT 命令归一化表。 | 力矩 ±15 N·m 由负责人确认，速度待确认。 |

104 的台架依据见[电机资料复核 §6.2–6.3](../../planning/06_cubemars_material_review.md)和[适配器设计 §5](../../development/ak30_force_control_adapter_design.md)。

## 两套数值的来源

- ROS 的 `ake60_8_ranges()` 指定 ±40 rad/s、±15 N·m。项目 ADR 记录了所用的中文手册表格，厂家英文手册的 AKE60-8 栏也给出这两个端点。[ROS 编码定义](../../../ros2_ws/src/mech_protocol_cubemars/include/mech_protocol_cubemars/ak30_force_wire.hpp) · [ADR-013](../../adr/ADR-013-ak30-protocol-baseline.md) · [CubeMars 官方手册 §4.2，第 39 页](https://img.cubemars.com/products/cubemars-product-parameter/AK-Series-Module-Product-Manual-v3.0.0-Download.pdf#page=39)
- 项目负责人确认的同款电机 STM32 工程在 `cubemars_motor_ctrl.c` 第 29–41 行为 MIT 槽位配置 ±45 rad/s、±12 N·m。该槽旧地址写作十进制 68（0x44），当前 104 的地址为十六进制 0x68。代码没有给出旧地址到当前两个电机的映射。另一槽的 ±40/±15 出现在位置速度模式初始化中。[STM32 配置来源与轴序核对](mit_axis_mapping_20261110.md)
- Orin 的 `/home/nvidia/jetson_mech_control/config/servo_pair.json` 及所查历史版本对电机的身份描述均为 `AK30 IDs104/105`，另存伺服映射和运行限值。原工程 ZIP、已查发布/配置归档中未找到能把两台实物分别关联到 MIT 两个字段端点的记录。下载目录中的 AKE60-8 驱动板 RAR 只含板卡 STEP 模型。

## 11 月 6 日实测帧复核

在 Windows 重新逐行读取双轴时延测试的两份原始 JSONL，仅统计 `stage=transport_send` 且 CAN ID 为本轴 MIT 命令的记录。文件 SHA256 与同批 `remote_manifest.json` 一致。

| 轴 | 文件 | 命令 CAN ID | 条数 | 不同载荷数 | 载荷（十六进制） |
|---|---|---|---:|---:|---|
| 104 | bus2_motor104.jsonl | 0x0868 | 10,005 | 1 | 00 00 00 7F FF 7F F7 FF |
| 105 | bus1_motor105.jsonl | 0x0869 | 10,005 | 1 | 00 00 00 7F FF 7F F7 FF |

这批帧的 Kp、Kd 为零，名义零速度与零前馈力矩都编码为 0x7FF。两套对称量程在当前截断编码下生成相同的中点码。0x29 回报包含角度、ERPM、Iq、温度和状态字，没有 MIT 命令量程字段。伺服运动记录可以核对反馈比例，无法补出 MIT 命令端点。

Windows 输入位于并联外骨骼-ROS框架项目下的 `Mujoco-Linux/output/mit_latency_20261106/mit_latency_20261106_pair20_v01/mit_latency_20261106_pair20_v01/`。104 文件 SHA256 为 `a528f63db9f0f7eee1b8829a7dea67a4b385923d39d6c68b57a28b81b17b5532`，105 为 `6b29a60c3ed423d310ce5b84b3aeaf48f7994bd6c7dc9d3cf5e09920559199c6`。

## 待验证项

纯力矩分支保持 Kp=Kd=0。104、105 已完成 ±0.10 N·m 逐轴实测，电流、编码器方向和撤力结果见[小力矩台架记录](mit_axis_pulse_20261110.md)。轴端力矩的大小需要独立测力参照。启用非零目标速度前，再按轴核对固件版本和相应的速度编码表。
