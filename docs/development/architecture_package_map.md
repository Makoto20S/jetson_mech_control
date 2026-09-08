# 架构对照：规划 ↔ 包 ↔ 每台电机的实际工作量 / Architecture Cross-Reference

- **Status:** 现行参考文档（随代码演进维护）
- **Date:** 2026-09-07
- **Origin:** 项目负责人 2026-09-07 的架构方向质询——「针对不同的电机，只要是 CAN 通信协议，做好相应的协议适配就可以了，但是我感觉现在你在做的是给每个电机都写个新的东西」。本文以仓库一手证据回答这个问题，并作为以后任何 AI/开发者的分层速查表。
- **Governs:** 无新决策——本文只对照既有决策（ADR-001～014、AdapterContract v1、planning 02）与实际代码结构，不改变任何边界。

## 1. 规划怎么说的 / What the plan says

[AdapterContract v1](adapter_contract_v1.md) 新适配器清单第 1 条（原文）：

> Add a dedicated `mech_protocol_<device>` package; do not add brand branches to core, controllers, or the composite hardware plugin.

即规划的扩展模型是：

```
新电机（不同品牌/协议）  →  新增一个 mech_protocol_<device> 包（协议适配）
新电机（同品牌型号）      →  只加部署配置（URDF 参数、逐台证据值）
框架层（core/controllers/composite plugin）  →  任何情况下不出现品牌分支
```

配套约束（同文件）：配置期固定 `ProtocolProfile`、无运行期自动探测（清单第 3 条）；canonical 语义只在物理映射有证据时导出（ADR-009）；命令接口形状是 canonical 契约（ADR-014，变更需先出 ADR）。

## 2. 实际包结构 ↔ 各层职责 / The six packages

| 包 | 职责 | 品牌相关？ | 换新品牌电机时要动吗 |
|---|---|---|---|
| `mech_control_core` | 协议无关核心：`RawCanFrame`/`BusRuntime`/canonical 命令与状态类型、`AdapterResult`、freshness/TTL、`ProtocolProfile` 枚举 | 仅 `ProtocolProfile` 枚举值命名（`Ak30ForceControlExtended`/`Ak30ServoExtended`，ADR-013 定义）——这是**配置期固定 profile 的注册表**（清单第 3 条：一个 profile 一个显式枚举，无自动探测），不是品牌逻辑分支 | **不动**（新品牌 = 在枚举加一个值 + ADR） |
| `mech_simulation` | FakeTransport/FakeSerial 等离线测试替身 | 否 | **不动** |
| `mech_hardware_ros2_control` | `CompositeSystem`（SystemInterface）：生命周期/claim/switch/看门狗浮出/接口导出。接口形状是**通用** position/velocity/effort（ADR-014） | 否 | **不动** |
| `mech_controllers` | 通用控制器（DemoController 等） | 否 | **不动** |
| `mech_protocol_cubemars` | **唯一的品牌包**：AK3.0 力控+伺服 wire 编解码、证据门映射（`mapping_is_sufficient`）、`DeviceCodec`/`DeviceSession`（分级看门狗、故障锁存） | 是（CubeMars AK3.0） | **新增平行的** `mech_protocol_<新品牌>` 包；本包不动 |
| `mech_bringup` | 部署组合层：探针（bench 验证用）、`Ak30RuntimeParams`（fail-closed URDF 参数解析）、`Ak30ForceControlRuntime`（RuntimePort 接线）、deployment 示例（URDF/controllers/launch） | 部署层（组合点按 profile 分） | 加新品牌的组合点/参数解析；既有内容不动 |

依赖方向（`context_check.py` 逐包钉死）：`bringup → {core, simulation, hardware, controllers, cubemars}`；`hardware → core`；`cubemars → core, simulation`；**没有任何包反向依赖 bringup，core/controllers/hardware 不知道任何品牌。**

## 3. 三个 PR 各改了哪层 / Where each slice actually landed

用 `git diff <base>..<head> --stat -- <路径>` 可随时复核：

| PR | 改动包 | 协议包动了？ | 性质 |
|---|---|---|---|
| PR #9（已合并，`f382324`） | `mech_protocol_cubemars`（新包）+ `mech_bringup`（探针/串口）+ core（USB-CDC codec 所在） | 是——**这是协议适配本身**，规划的「写一次」 | 协议适配 |
| PR #11（OPEN） | `mech_bringup`（runtime/params/示例）+ `mech_hardware_ros2_control`（零品牌改动，仅注入点） | 否 | 部署组合 |
| PR #12（OPEN，ADR-014） | `mech_hardware_ros2_control`（**通用**接口形状：单命令接口 ∈ {position, velocity, effort}）+ `mech_bringup`（sub_mode 参数/变体） | **否——diff 为空** | 通用契约扩展 |

**关键事实**：`git diff 5b7a453..26f0274 -- ros2_ws/src/mech_protocol_cubemars ros2_ws/src/mech_control_core ros2_ws/src/mech_controllers` 输出为空。AK3.0 从 Position 扩展到 Torque/Velocity **没有新写任何协议代码**——三个子模式在 PR #9 就已在协议包里实现完（共享控制模式 ID 8，载荷区分），PR #12 只是把 ros2_control 的通用接口形状放宽到能表达它们。

## 4. 「每电机要写新东西」的真实内容 / What per-motor work actually is

每接入一台**具体电机**，必做且不可避免的工作全部是**配置与证据**，不是框架代码：

| 工作项 | 为什么每台都要 | 属于哪层 |
|---|---|---|
| 逐台实测常量：CAN ID（motor1=104）、零偏（330.07°）、减速比（8）、极对数（14）、Kt、direction_sign | G0–G3 闸门要求一手证据，不写就是猜（AGENTS.md「unknown 不填默认」） | URDF 参数 + memory/证据文档 |
| URDF 部署文件（motor1 有 position/torque/velocity 三个变体） | 部署声明：接口形状必须匹配 sub_mode（ADR-014 fail-closed） | 部署配置 |
| 台架验证（渐进路线四步那种） | 真机激活闸门；换任何新设备都重走 | 流程，不是代码 |
| （仅当**品牌/协议不同**）新的 `mech_protocol_<device>` 包 + ADR-014 式组合点 | 协议适配——这是规划的「写一次，之后同品牌型号复用」 | 协议层 |

**将来第二台同品牌电机的接入面**：URDF 参数 + 逐台证据 + （多电机共享总线的仲裁/带宽分析，ADR-006 范畴）。协议包、core、controllers、composite plugin、runtime、参数解析器**全部复用**。

**将来新品牌（如 HI12）的接入面**：上面全部 + 一个新的 `mech_protocol_hi12` 包 + 它自己的组合点/参数解析 + 先行 ADR（若触及通用契约）。这正是规划预期。

## 5. 已知的真实缺口（诚实记录，2026-09-07）/ Known real gaps

1. ~~**launch → 真机的生产组合点还不存在。**~~ **已补（2026-09-08 组合点切片，PR #13：`mech_bringup/Ak30System` 组合插件 + 三个部署 xacro 改指向它）。** 历史记录（2026-09-07 调查）：`Ak30RuntimeParams::parse()` 当时没有生产调用方；`0x12` 透传初始化只有探针发；pluginlib 构造的 `CompositeSystem` 必然带内置 LoopbackRuntime，直接 `ros2 launch` 会广播假状态。2026-09-06 切片把这一项刻意划在范围外（示例只做结构校验），不是架构漂移。
2. **本文创建时开发处于 owner 暂停状态**（架构方向质询未裁决）：组合点切片设计过、未开工。（2026-09-08 已解除：疑虑解决、切片实施完毕。）
3. **Torque/Velocity 尚无 ros2 控制器**（DemoController 仅 position）；接口形状已就绪，命令方待后续切片。
4. **真机首跑尚未发生**（组合点切片只做离线验证）：broadcaster-only 首跑是下一任务，需 owner 逐次授权 + 到场（ADR-006 Decision 7 边界）。

## 6. 给以后 AI 的判据 / How to tell the layers apart quickly

- 看到一个改动只动 `mech_protocol_<brand>` → 协议适配，规划内。
- 看到改动动 `mech_control_core`/`mech_controllers`/`mech_hardware_ros2_control` 且出现品牌字样 → **违规**，停下（AdapterContract 第 1 条）。**唯一豁免**：core 的 `ProtocolProfile` 枚举（一个 profile 一个显式枚举值，如 `Ak30ForceControlExtended`；新 profile 加枚举值 = 清单第 3 条要求的注册动作，需 ADR）。
- 看到 `mech_hardware_ros2_control` 的通用接口形状变化 → canonical 契约变更，**必须先有 ADR**（第 7 条；先例 ADR-014）。
- 看到 `mech_bringup` 新增 per-brand 组合点/参数解析 → 部署层扩展，规划内；同品牌第二台电机不应需要新的 bringup 代码，只应需要新的 URDF/参数。
- 每台电机的 URDF/实测常量是**证据数据**，不是代码重复——它们各不相同是闸门的要求，不是架构问题。

## 7. 复核命令 / Verification commands

```bash
# 各包改动归属（PR #12 为例；输出为空即协议/core/控制器未动）
git diff 5b7a453..26f0274 --stat -- \
  ros2_ws/src/mech_protocol_cubemars \
  ros2_ws/src/mech_control_core \
  ros2_ws/src/mech_controllers

# 依赖方向钉死在 CI
python3 tools/ci/context_check.py

# 品牌隔离：品牌字样应只出现在 mech_protocol_cubemars、mech_bringup、
# core 的 ProtocolProfile 枚举注册表和 ADR/规划文档里
grep -rn "cubemars" ros2_ws/src/mech_control_core ros2_ws/src/mech_controllers \
  ros2_ws/src/mech_hardware_ros2_control ros2_ws/src/mech_simulation \
  --include='*.hpp' --include='*.cpp'
# （预期：零匹配。core 中允许的 AK30 字样是 ProtocolProfile 枚举值，
#  用 `grep -rn "Ak30" ...` 单独查。）
```

## 8. 来源 / Sources

- [AdapterContract v1](adapter_contract_v1.md)（新适配器清单第 1、3、7 条）。
- [ADR-003 复合 SystemInterface](../adr/ADR-003-composite-system-interface.md)、[ADR-013 AK3.0 协议基线](../adr/ADR-013-ak30-protocol-baseline.md)、[ADR-014 子模式命令接口](../adr/ADR-014-ak30-submode-command-interfaces.md)（Proposed，在 PR #12 分支上）。
- [02 总体架构与接口](../planning/02_architecture_and_interfaces.md)；[AK3.0 力控适配器设计](ak30_force_control_adapter_design.md)。
- 仓库包结构与 `git diff` 实测（2026-09-07 会话）。
- 项目负责人的架构质询原文（2026-09-07，记录于当日 handoff 与本地 memory）。
