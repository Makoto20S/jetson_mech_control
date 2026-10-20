# ADR-021：达妙 DM4340 MIT 协议档案

- **Decision ID:** ADR-021
- **Status:** Proposed
- **Date:** 2026-10-13
- **Owner:** 项目负责人（设备参数与实机授权待复核）
- **Scope:** DM4340 MIT 协议、会话与独立 ros2_control 部署

## Status rationale / 状态依据

协议与独立硬件插件已经实现，速度链路已有离体空载 ROS 台架记录，力矩模式
具备软件路径与专用台架入口，尚无独立力矩模式实机验收记录。
ADR 保持 Proposed，不把台架功能通过扩展为最终部署批准。
2026-10-20 用户确认本次启动所需验证全部完成；该声明与本轮独立测量分开记录。
`115200` 的早期口述不作为 CAN 仲裁率依据；当前示例声明 1 Mbit/s，
实际固件、ID、量程、关节映射与失联行为仍须在每次部署中绑定证据。

## Context / 上下文

The manufacturer [motor SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)
defines an 8-byte MIT command with position, velocity, Kp, Kd and torque
fields, and a different feedback layout. CubeMars AK3.0 in this repository
uses extended-frame profiles with different control and feedback semantics.
Sharing a canonical controller does not make their wire protocols compatible.

## Decision / 决策

Register `Dm4340MitStandard` as a separate, configure-time fixed Classic CAN
standard-frame profile. Place wire conversion, mapping and session logic in
`mech_protocol_damiao`, parallel to `mech_protocol_cubemars`. Reuse the core
bus owner and generic controller interfaces in the production composition; never add a Damiao branch to CubeMars or the generic controller.

The production slice selects exactly one command mode at startup: velocity
or effort, paired with command_generation. Position commands are not exposed
by Dm4340System. Each joint owns a separate MIT session; all joints on one
physical gateway share one runtime and one serial owner.
Firmware identity/range, CAN IDs, motor type, encoder/gearbox mapping, target
envelope and watchdog timing must be explicit. An SDK encoding range alone is
not sufficient evidence for physical limits. The codec/session only prepares frames; the runtime owns transmission.
Activation requires explicit configuration and the attached unit's G0-G3 gates.
The separately named supervised effort bench profile is limited to one unloaded
shaft, identity mapping and at most 0.1 Nm; it does not certify a joint mapping
or motor-side timeout. It cannot be selected for velocity mode.

## Alternatives considered / 替代方案

- 在 CubeMars codec 中增加达妙分支：拒绝，协议不同且破坏品牌隔离。
- 将 `115200` 视作 CAN 仲裁率：拒绝，可能是串口或网关参数。
- 用 SDK 常量作实机安全限值：拒绝，它们仅规定编码量化范围。

## Consequences / 后果

### Positive / 正面

控制接口保持统一；协议与会话可在无设备环境验证。

### Negative / 负面与代价

需维护独立的生产组合；缺少设备证据时必须拒绝配置。

## Validation / 验证

离线测试覆盖编码、证据缺失、会话时效、故障、独立插件加载与资源申领。
ARM64 历史联合构建和 ROS 速度台架结果见
[10 月 16 日验证记录](../validation/2026-10-16-exoskeleton-architecture.md)；
[10 月 20 日速度复测](../validation/2026-10-20-dm4340-speed-retest.md)
基于既有隔离构建，不代表本次未提交源码的力矩模式已实机通过。
当前源码的构建和测试结果由对应 PR 记录。

## Review triggers / 重审触发

获得实际固件、收发 CAN ID、物理映射、CAN 接口配置，或发现协议与 SDK
不符时复核。实机运动仍须单独授权。

## Sources / 来源

- [达妙官方 motor-sdk](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)
- [适配契约](../development/adapter_contract_v1.md)
- [固定协议档案](ADR-004-fixed-protocol-profile.md)
