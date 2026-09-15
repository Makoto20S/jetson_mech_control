# Jetson 通用机电控制框架规划索引

> 规划基线：2026-07-28
> 最近收敛：2026-09-16（位置控制器里程碑已发布；T8 速度控制器离线实现与验收）
> 当前状态：Foundation RC2 已完成；PositionCommandController 已随 PR #16 合并，tag `position-controller-complete` 标记位置控制器收口。当前推进 **T8 VelocityCommandController**；T9 EffortCommandController 后续单独实施。设备操作仍逐次授权。

Foundation 及早期 AK3.0 软件/探针阶段的记录保留在 §3；当前实现包含
ADR-014 子模式映射、ADR-015 发送授权、ADR-016 反馈质量、ADR-017 两档刷新
和 ADR-018 上游目标有效期。历史探针的速度/力矩证据不等于新控制器的实机验收。
FND-004 已完成，FND-004A 也已通过并完成 tag/主分支保护切换；
Foundation RC2 已于 2026-08-31 收口并打 tag `v0.1.0-foundation-rc2`。

## 1. 权威边界

当前事实按以下入口读取：

1. 代码、配置和可复现测试描述实际行为；
2. [ADR 索引](../adr/README.md)描述已批准架构与语义；
3. 本索引和 [Foundation 计划](07_framework_bootstrap_plan.md)描述当前顺序；
4. 证据文档保存资料来源、推断边界和待确认项；
5. [历史归档](../archive/README.md)仅用于追溯，不是当前规范。

ADR-001/002/003/004/005/009/012/013 为 Accepted；[ADR-006](../adr/ADR-006-conditional-can0-deployment.md)保持 Proposed。后者表示当前单物理通道及其 transport backend 尚未取得逐台位速率、ID、profile、终端、backend 能力、负载、仲裁和错误证据，不能激活；其 Decision 第 7 条定义了与最终部署 profile 分离的单电机取证台架边界（2026-09-02 owner 批准），台架实测不构成最终 profile 验收。

文档中的结论使用“已确认事实/资料事实”“规划决定/有依据的推断”“用户提供（待独立确认）”或“待确认项”标记。供应商参数、Jetson 运行态和硬件状态未经本轮复核均视为过期或未知。

## 2. 当前目标与边界

- 目标包络：NVIDIA Jetson、最多 6 台 CAN 电机、2 条 CAN/CAN FD 总线、2 台 HI12，以及未来 STM32 传感器节点。
- 当前设备背景：两台基于 AKE60-8 的定制双编码器电机和两台 HI12；实际固件、配置、协议、节点和物理拓扑仍需逐台取证。
- 实际目标机背景：NVIDIA Orin NX 16GB 模组（`P3767-0000`）+ **合众恒跃 HZHY HYAI-311UAV 第三方载板**（2026-08-23 实物照片核验；软件设备树报 `p3768-0000` 是厂商基于 devkit 配置构建镜像的产物）。**2026-08-23 已完成平台迁移**：JetPack 6.2 / L4T R36.4.3 / Ubuntu 22.04.5 / 内核 5.15.148-tegra，`nvidia-l4t-*` 已锁定、ROS 2 Humble 已安装——FND-004A 的原生 Jammy/Humble 前置条件**已满足**。
- CAN 拓扑意向（2026-08-23 用户确认）：电机接入高擎通用盒子（7路CAN功率板）的电源+CAN 通道，Jetson 经 USB CDC 收发；HI12 接入方案待定。激活仍受 ADR-006 证据闸门约束。
- 当前阶段：T8 独立速度控制器与离线链路验收；Foundation 已完成。
- 当前安全边界：T8 只做无设备构建/测试；不启用 CAN、不操作设备、不切换实机部署。
- 当前实现入口：[Foundation v0.1 控制框架搭建计划](07_framework_bootstrap_plan.md)。
- 完整 MVP 与硬件验收入口：[MVP 执行、验证与项目治理计划](03_mvp_delivery_plan.md)。

## 3. 当前实施顺序

| 阶段 | 状态 | 出口 |
|---|---|---|
| FND-000～FND-015、RSP-001/RSP-002、INT-001 | **已完成（2026-08-31）** | 仓库/依赖/CI、九份 ADR、ROS-independent 核心契约、SocketCAN、注入式 USB-CDC、模拟链路、ros2_control 复合 SystemInterface 与集成测试；RC2 取证收口于 `9317d76`（tag `v0.1.0-foundation-rc2`：Jetson ARM64 build/test + sanitizers + 30 min 稳定性 + 首次 vcan 往返） |
| 目标机平台准备 | 已完成（2026-08-23） | HZHY 镜像 + `l4t_initrd_flash` 刷写 JetPack 6.2 / Ubuntu 22.04.5，首启验收、加固（L4T 包锁定）与 ROS 2 Humble 安装完毕；记录见升级教程 §12.0/§14.0 |
| 里程碑治理 | 已完成 | `fnd-004a-passed` tag + `main` PR 保护生效；FND-005 起短生命周期任务分支 + PR |
| ADR-013 协议基线切换 | 已完成（2026-09-01） | 协议基线由 L02（AK2.0）更正为 L07（AK3.0 V3.2.0）；力控扩展帧为第一实现 profile，伺服扩展帧第二；`ProtocolProfile` 重定义并合入 `main`（PR #7，`718b35a`） |
| AK3.0 力控适配器第一切片 | **已合并（2026-09-05，PR #9 `f382324`）** | `mech_protocol_cubemars`：wire 编解码、证据门映射层、`DeviceCodec`/`DeviceSession`（分级看门狗分类 + 故障锁存）、`mech_bringup` 探针（torque/velocity/position，`MECH_BUILD_DEVICE_PROBES=ON`）；双格式透传解码（含 FW 4.8.8 3 字节前缀） |
| 单电机取证台架（ADR-006 Decision 7） | **阶段 2 完成（2026-09-05）** | 渐进验证四步全过：①纯力矩 0.1 N·m effort 回显闭环 → ②Kd 阻尼速度 0.3 rad/s（位移 +0.2°）→ ③hold 零位移位置（位置逐位恒定）→ ④位置步进 +2°/+5°（Kp 误差力矩观测）与 +30° 全程（Kp=1 摩擦稳态误差，物理预期）；`direction_sign=+1.0`、`gear_ratio=8`、B15 输出端速度假设均落实测证据；生产激活仍需 ADR-006 转 Accepted + G0–G3 完整证据 |
| 阶段 3 软件切片（Position 子模式） | **已合并（PR #11）** | `mech_bringup` 新增 `Ak30ForceControlRuntime`（RuntimePort 接线 `Ak30ForceControlSession`，`command_stage()` 首个生产消费方：Following 提交/Holding 冻结/Expired 在 3 周期预算内显式失败且不合成 0.0）+ `Ak30RuntimeParams`（fail-closed 参数解析）+ deployment 示例（URDF/controllers.yaml/launch，位置控制器 spawner 默认注释）；236 离线测试全绿；B14 专项与 B4/B9/B14/B15 厂商复核并行 |
| Torque/Velocity 命令接口切片（ADR-014） | **ADR Accepted + 已合并（PR #12）** | [ADR-014](../adr/ADR-014-ak30-submode-command-interfaces.md)：`CanonicalCommand` 扩展三字段、CompositeSystem 每关节恰好一个命令接口 ∈ {position, velocity, effort}；runtime 按子模式映射（Velocity 强制 effort=0 防前馈叠加）+ `sub_mode` 部署参数 + 三套 URDF 变体（position/torque/velocity）与接口名匹配结构校验；离线测试全绿；真机运行仍逐次授权（ADR-006 Decision 7） |
| 位置控制器里程碑 | **已合并/发布（2026-09-16）** | 独立 `PositionCommandController`，PR #16 与 `position-controller-complete`；位置功能收口不代表精度或实时性认证 |
| T8 速度控制器 | **离线实现及本地/ARM64/sanitizer 验收完成，待 PR 合并** | 独立 `VelocityCommandController`；velocity+command_generation、速度/加速度限制、目标有效期；共享有界目标机制，默认 inactive 的独立部署入口；验收包含实际 manager 到 FakeTransport 帧、过期/停用/重新激活静默 |
| T9 力矩控制器 | **待实施** | 独立 `EffortCommandController`；实机前还需 raw ERPM 超速/新鲜度保护，不能使用 Torque 模式中未证实的标准 velocity |

T8 的接口和启动/停止策略见 [mech_controllers](../../ros2_ws/src/mech_controllers/README.md)
与 [速度部署说明](../../ros2_ws/src/mech_bringup/README.md#t8-velocity-deployment)。

如果烟测后需要修复，必须在新 commit 上重新完整执行 FND-004A；里程碑 tag 不得指向未实际通过的 commit。

## 4. 活动文档地图

| 文档 | 当前职责 | 规范性 |
|---|---|---|
| [01 事实基线与开源调研](01_evidence_and_research.md) | 设备、Jetson、开源候选与证据边界 | 证据层 |
| [02 总体架构与接口设计](02_architecture_and_interfaces.md) | 架构图、接口契约和实现上下文；正式决定反向链接 ADR | 设计说明；ADR 优先 |
| [03 MVP 执行、验证与治理](03_mvp_delivery_plan.md) | 完整 MVP、硬件闸门、量化验收和发布路线 | 当前总路线 |
| [04 证据与来源登记](04_source_register.md) | 本地资料、哈希、官方文档、仓库快照与冲突 | 证据登记 |
| [05 决策与待确认项](05_decisions_and_open_questions.md) | ADR 导航、现场上下文和 OQ-01～OQ-10 | 状态概览；ADR 优先 |
| [06 CubeMars 资料审查](06_cubemars_material_review.md) | AK3.0（L07）力控/伺服双 profile、AKE60-8 参数、HighTorque transport 边界和实机缺口 | 供应商证据层 |
| [07 Foundation 搭建计划](07_framework_bootstrap_plan.md) | Foundation 顺序、核心契约与 Definition of Done；Foundation 后的适配器分工 | Foundation 实施记录；当前活动阶段见本文件 §3 |
| [RSP-001 transport 评估](rsp-001-transport-evaluation.md) | SocketCAN 与 HighTorque USB-CDC/USB2CAN 的离线取舍和能力证据 | 已完成离线评估；不授权真实激活 |
| [FND-000 仓库与资产政策](fnd-000_repository_and_asset_policy.md) | 仓库、许可证、资产、Memory 与分支治理 | 已确认政策 |
| [FND-004 ADR 集合](../adr/README.md) | 架构与语义状态 | 规范入口 |
| [AK3.0 力控适配器设计](../development/ak30_force_control_adapter_design.md) | `mech_protocol_cubemars` 的 wire/mapping/session 设计与实测边界 | 设计记录；ADR 优先 |
| [Orin NX JetPack 6 升级评估与教程](../development/jetson_orin_nx_jetpack6_upgrade_guide.md) | 当前标准开发套件的备份、Direct Flash、首启验收和回滚 | 决策与执行草案；不代表刷机授权 |
| [FND-004A 烟测](../development/jetson_arm64_smoke_test.md) | 目标机原生 smoke 清单与里程碑取证 | 已完成验证步骤 |

## 5. 硬件闸门

| 闸门 | 必须证据 | 未通过时 |
|---|---|---|
| G0 设备身份 | 电机/HI12 型号、固件、协议、ID、位速率和 profile | 只做模拟与离线协议工作 |
| G1 被动总线 | 接线、终端、只读抓包、ID/位速率与负载无冲突 | 停止总线集成 |
| G2 电机反馈 | 不发运动命令即可稳定获得语义明确的反馈和故障 | 不进入命令阶段 |
| G3 台架安全与计量 | 夹具、限位、急停/断能、限流和必要计量均书面通过 | 禁止非零真实命令 |
| G4 集成发布 | 频率、时序、故障、长稳、记录和复现达到 `03` 标准 | 不发布 MVP |

FND-004A 不属于 G0～G3 的替代品，也不证明 vcan、实时性、总线或设备兼容。

## 6. 文档生命周期

本轮收敛保留 `01/04/06` 的证据链，压缩 `02/05` 的重复决策正文，保留 `03/07` 的验收与实施职责，并把初始总体规划提示词移入 [非规范归档](../archive/README.md)。删除的历史叙述仍可由 Git 历史追溯。

活动文档不得引用归档作为当前规范。新增长期决定进入 ADR 或对应正式文档；共享任务进度进入 GitHub Issues/Milestones/PR；个人 `memory/` 不进入 Git。
