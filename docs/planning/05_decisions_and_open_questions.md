# 决策导航与待确认项

> 最近收敛：2026-08-07
> 作用：把正式 ADR、现场上下文和仍会阻塞设备接入的问题放在一个短入口中。
> 边界：本文件不复制 ADR 决策正文，也不代表真实设备、CAN 拓扑或物理力矩已经验证。

## 1. 正式决策导航

当前规范状态见 [ADR 索引](../adr/README.md)：

| ADR | Status | 当前边界 |
|---|---|---|
| [ADR-001](../adr/ADR-001-core-boundary.md) | Accepted | 纯 C++ 核心与薄 ros2_control 适配层 |
| [ADR-002](../adr/ADR-002-bus-runtime-ownership.md) | Accepted | 每条物理 CAN 总线只有一个进程内 `BusRuntime` 写者 |
| [ADR-003](../adr/ADR-003-composite-system-interface.md) | Accepted | Foundation/MVP 使用配置驱动的复合 `SystemInterface` |
| [ADR-004](../adr/ADR-004-fixed-protocol-profile.md) | Accepted | configure 期固定协议代际和 active command profile，ACTIVE 期间不猜测或混发 |
| [ADR-005](../adr/ADR-005-monotonic-time-freshness.md) | Accepted | 单调时钟管理 freshness/TTL，源时间与到达时间独立保存 |
| [ADR-006](../adr/ADR-006-conditional-can0-deployment.md) | Proposed | 当前单 `can0` 只是等待设备和总线证据的条件式 profile |
| [ADR-009](../adr/ADR-009-effort-semantic-gate.md) | Accepted | 标准 `effort` 必须经过物理语义证据闸门 |

Accepted 只接受对应文件的架构或语义，不等于 ARM64、vcan、真实 CAN、设备兼容、500 Hz 或物理力矩精度已经验证。任何冲突以 ADR 状态和正文为准。

## 2. 当前现场上下文

| 项目 | 当前记录 | 证据性质 |
|---|---|---|
| 电机 | 两台基于 AKE60-8 的定制驱控一体电机，加装 CubeMars 双编码器；结构设计不属于软件任务 | 用户确认；实机配置待证 |
| CubeMars 资料 | AK3.0 V3.2 足以离线设计 servo extended 与 force-control extended codec；legacy MIT 单独处理 | 资料事实，见 [06](06_cubemars_material_review.md) |
| HI12 | 当前有两台；标准 CAN 交付通常为 J1939，但每台交付协议、ID、位速率和输出 profile 尚未读取 | 用户提供 + 待确认 |
| CAN | 四台设备计划共用 Jetson `can0`，暂无第二路接口 | 用户提供；ADR-006 仍 Proposed |
| 断能 | 项目方说明已有遥控继电器断能并曾验证 | 用户提供；G3 前仍需当前书面证据 |
| 目标平台 | 目标为 Jetson Orin ARM64 / Ubuntu 22.04 / ROS 2 Humble | 构建基线；FND-004A 尚未验证当前目标机 |

供应商参数、Jetson 状态和硬件状态在本轮未重新读取时不得写成当前事实。

## 3. 已确认的实施解释

- **协议选择：** CubeMars 至少区分 `AK V3 servo extended`、`AK V3 force-control extended` 和 `legacy MIT standard-frame`。配置期绑定，ACTIVE 期间禁止自动探测或混发；详细协议证据见 [06](06_cubemars_material_review.md)。
- **控制职责：** 驱动器负责 FOC、高速电流环和所选模式的内部闭环；Jetson C++ 控制器读取标准状态并产生有界目标。Python 不拥有电机命令接口。
- **最小 demo：** 恒定命令只验证 controller → hardware → session → BusRuntime → CAN → state/diagnostics 链路。目标必须可配置、有限、受限、带 slew/TTL 且可回零；它不是最终控制算法或物理力矩精度验收。
- **频率：** 当前两电机候选正常目标为 500 Hz，HI12 初始 100 Hz；100～200 Hz 用于模拟/首次 bring-up；代码支持 1 kHz 控制循环实验。控制循环、命令、反馈和 IMU 是独立频率，重复快照不得伪装成新样本。
- **标准 effort：** 标准 AKE60-8 的 `Kt = 0.7382 N*m/A` 与输出端公式只是定制实机候选。只有实机固件、方向、减速比、字段和配置证据闭合后才暴露标准 `effort`；否则使用明确的 current/raw 接口。
- **设备扩展：** 新 CAN 设备优先只增加 codec、device session、capability、配置和测试，不为品牌修改 core/controller 公共语义。
- **实施顺序：** 缺少实机配置不阻塞 Foundation；FND-004A 通过后才开始 FND-005，Foundation RC/AdapterContract v1 冻结后再接入真实 CubeMars/HI12 adapter。

架构上下文见 [02](02_architecture_and_interfaces.md)，量化验收与总线预算见 [03](03_mvp_delivery_plan.md)，当前任务顺序见 [07](07_framework_bootstrap_plan.md)。

## 4. 仍待确认的问题

| ID | 当前状态 | 待确认项 | 所需证据 | 影响 |
|---|---|---|---|---|
| OQ-01 | 部分解决 | 基型为 AKE60-8；两台定制件号、驱动板和固件未知 | 每台连接/版本记录、序列号和只读配置导出 | 协议代际、缩放和范围 |
| OQ-02 | 资料部分解决 | V3.2 支持 servo 与 force-control；实机是否属于该代际未知 | 固件版本、golden frame 与被动抓包 | codec、claim、lifecycle |
| OQ-03 | 资料部分解决 | `0x29/0x2A` 使用哪个编码器、能否经 CAN 同读两者未知 | 供应商帧说明、配置和抓包 | `joint/position` 来源 |
| OQ-04 | 资料部分解决 | 标准 Kt/范围已知；定制版是否保持不变未知 | `.AppParams`/`.McParams` 与供应商确认 | 标准 `effort` 映射 |
| OQ-05 | 协议范围解决，性能待测 | 手册允许 1～2000 Hz；定制实机 500 Hz 稳定性未知 | 配置、抓包与 30 min 统计 | 有效闭环带宽 |
| OQ-06 | 未解决 | 命令丢失后的内部 watchdog/neutral 行为 | 固件资料和 G3 后受控断包试验 | 命令租约与进程崩溃安全 |
| OQ-07 | 未解决 | 两台 HI12 的协议、节点、位速率和输出 profile | PNAME/APP_VER、逐台配置读取和抓包 | 单 `can0` 可行性 |
| OQ-08 | 未解决 | CubeMars 与 HI12 能否同位速率且无 ID/终端冲突 | 全设备配置、拓扑和总线预算 | ADR-006 状态 |
| OQ-09 | 未解决 | Jetson 500 Hz/1 kHz 周期与 command-to-wire 性能 | nominal/stress 原始时序 | RT 内核/第二总线需求 |
| OQ-10 | 部分解决 | 标准命令语义已收敛；定制实机物理输出误差未知 | 实机参数和必要的外部计量 | 对外宣称与物理验收 |

## 5. 下一证据动作

1. FND-004A 只验证目标 Jetson 上的 clean clone、context、依赖和五包 build/test，不读取或修改设备。
2. Foundation 后的 G0 取证先分别只读保存两台电机的连接/版本页、基础设置、`.AppParams` 和 `.McParams`；不得执行参数识别、写入、恢复默认或升级。
3. 两台 HI12 必须逐台记录身份、交付协议、节点、位速率和输出 profile，再评估合并总线。
4. 配置证据仍不能回答编码器来源、watchdog、定制 Kt 或终端时，再向供应商提出精确问题。
5. 任何证据与 ADR 冲突时，停止受影响的真实 profile，更新本表并按 ADR review trigger 重审；不得静默改写或继续激活。
