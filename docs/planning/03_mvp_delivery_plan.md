# MVP 执行、验证与项目治理计划

> 2026-08-03 实施顺序更新：本文件保留完整 MVP、硬件闸门和月度验收总路线；当前无硬件 `Foundation v0.1` 的具体执行以 [07_framework_bootstrap_plan.md](07_framework_bootstrap_plan.md) 为准。实机身份和配置不阻塞 Foundation，只阻塞真实设备激活。
>
> 2026-08-07 FND-004 更新（2026-08-19 澄清）：当前架构规范和状态以 [ADR 索引](../adr/README.md) 为准；ADR-001/002/003/004/005/009 已 Accepted，ADR-006 在单物理通道、所选 transport backend 和实机/总线证据完成前保持 Proposed。本文件中的早期推荐不得提升 ADR 状态。
>
> 2026-08-07 文档收敛：本文件只保留完整 MVP、硬件闸门、带宽与量化验收；FND-004A 之后的任务顺序以 [07](07_framework_bootstrap_plan.md) 为准。初始规划输入已移入 [非规范归档](../archive/README.md)。
>
> **2026-09-01 规划修订（[ADR-013](../adr/ADR-013-ak30-protocol-baseline.md)）：协议基线为 L07（AK3.0 V3.2.0）；力控扩展帧（控制模式 ID `8`）为第一实现 profile，伺服扩展帧（控制模式 `0–6,15,16`）为第二实现 profile。~~2026-08-19 曾确认 L02 V1.0.18 适用、伺服优先、运控/MIT 标准帧次之~~——L02 是 AK2.0 驱动器手册，其覆盖的驱动板与本项目的 `AK54-4810-1C-A2` 不符，该 11 位标准帧 profile 在本项目硬件上不存在。**HighTorque ROS2 SDK/FDCAN 示例只作为集成与 USB-CDC raw transport 参考，不改变 Foundation 的 core/BusRuntime/codec 边界。
>
> **2026-09-06 进展标注：** Foundation v0.1 已收口（RC2 tag `v0.1.0-foundation-rc2`，2026-08-31）；AK3.0 力控适配器第一切片（`mech_protocol_cubemars`）已随 PR #9 合入 `main`（2026-09-05），并在单电机取证台架上完成力控三子模式闭环（ADR-006 Decision 7 边界内；G1/G2 在该台架上实测通过，G0 motor1 部分取证）。**本文件的 Week 表与 G0–G4 时间线仍为原规划基线；当前实际进展以[规划索引](README.md) §3 为准。** 生产激活仍受 ADR-006（Proposed）与 G0–G3 完整证据约束。

## 1. 需求分解与完成口径

| 能力 | MVP 必须完成 | 完成不代表 | 性质 |
|---|---|---|---|
| 两台 HI12 | 识别身份/协议/ID/位速率；稳定接收所需字段；保存时间和质量 | 严格同步采样，除非 SYNC_IN 实测证明 | 有依据的推断 |
| 一台基于 AKE60-8 的定制电机 | 识别实际驱动板/固件/配置；稳定反馈；受限台架最小恒定命令链路 demo | 最终控制算法已经完成；标准机型参数已自动适用于定制版；可用于人体 | 有依据的推断 |
| ros2_control | 硬件组件可配置/激活/停用；C++ 控制器可加载/运行/切换 | Humble 自动恢复覆盖所有故障 | 有依据的推断 |
| 实时性 | 在定义负载下达到周期、命令延迟和 freshness 目标 | 通用硬实时认证或所有 Jetson 型号保证 | 有依据的推断 |
| 诊断与复现 | CAN/设备/控制/实验元数据完整；可 vcan/canplayer/rosbag 回放 | ROS 回放复现真实总线仲裁和物理动力学 | 有依据的推断 |
| STM32 | 仓库边界、协议语义、时间/序号/标定/诊断契约 | 固件、ADC 和传感器前端实现 | 已确认事实 |

**有依据的推断**：首月优先级是“证据正确、单设备可控、测试可重复”，不是同时接满 6 台电机或引入学习控制。项目可以在某个真实硬件闸门失败时交付软件骨架和缺陷证据，但不能把未通过的硬件能力标记为完成。

## 2. 建议仓库与 ROS 包结构

**已确认事实 + 有依据的推断**：私有 GitHub 单仓库 `Makoto20S/jetson_mech_control` 已建立，当前已有五个 Foundation package 骨架、manifest、CI 和 ADR；下图是目标目录包络，不表示所有目录或 vendor package 已创建。

```text
/
  README.md
  AGENTS.md
  .codex/config.toml
  .github/
    CODEOWNERS
    pull_request_template.md
    workflows/
  docs/
    adr/
    architecture/
    protocols/
    bringup/
    experiments/
    planning/
  manifests/
    dependencies.repos
    host/
    releases/
  config/
    schema/
    robots/
    devices/
    deployments/
  interfaces/
    dbc/
    golden_frames/
  ros2_ws/src/
    mech_control_core/
    mech_protocol_cubemars/
    mech_protocol_hipnuc/
    mech_hardware_ros2_control/
    mech_controllers/
    mech_interfaces/
    mech_bringup/
    mech_diagnostics/
    mech_simulation/
    mech_learning_bridge/
  firmware/
    stm32_sensor_node/
      README.md
      protocol/
      tests/
  tools/
  tests/
    integration/
    fault_injection/
    hil/
  data/
    README.md
    manifests/
```

| 包/目录 | 职责 | 依赖边界 | 性质 |
|---|---|---|---|
| `mech_control_core` | BusRuntime、router、快照、时间、command lease、故障状态 | Linux/标准 C++，不依赖 ROS | 有依据的推断 |
| `mech_protocol_cubemars` | AK3.0 力控（控制模式 ID `8`，第一实现）与伺服扩展帧（控制模式 `0–6,15,16`，第二实现）的 codec 与 device session | 依赖 core 抽象，不打开 socket；协议 profile 不共用有歧义的打包函数 | 有依据的推断 |
| `mech_transport_hightorque`（Foundation 后按 spike 决定） | HighTorque USB CDC raw CAN/CAN-FD transport backend | 只实现受控 framing/CRC/队列/能力边界；不拥有 motor codec 或 ros2_control 生命周期 | 有依据的推断 |
| `mech_protocol_hipnuc` | J1939/CANopen profile codec、坐标/质量 | CANopen 后端条件依赖 | 有依据的推断 |
| `mech_hardware_ros2_control` | 复合 SystemInterface、接口与 lifecycle | 依赖 core，不含协议位域 | 有依据的推断 |
| `mech_controllers` | 有界测试、PID/阻抗/滑模等 C++ plugins | 只依赖标准接口和 command contract | 有依据的推断 |
| `mech_interfaces` | 非 RT 消息、诊断、策略目标与实验服务 | 不用于逐帧 CAN 序列化 | 有依据的推断 |
| `mech_bringup` | launch、配置组合、部署映射 | 不保存设备秘密或猜测默认 | 有依据的推断 |
| `mech_diagnostics` | decimated 状态、指标与事件发布 | SCHED_OTHER，不写命令 | 有依据的推断 |
| `mech_simulation` | vcan 设备模拟、录包回放、动力学假体 | 无真实硬件副作用 | 有依据的推断 |
| `mech_learning_bridge` | Python/C++ 策略消息边界与 mock policy | 不拥有 CAN/硬件 | 有依据的推断 |
| `firmware/stm32_sensor_node` | 首期只放协议/测试/工具链约定 | 不虚构 ADC/FDCAN 实现 | 已确认事实 |

## 3. 一个月路线图

### Week 1：事实冻结、仓库和无硬件测试底座

**有依据的推断**：除表内写明更早日期外，本节所有待确认项最迟在 Week 1 Day 5 完成；“交付/验收”列就是确认方法。

| 任务 | 依赖 | 负责人 | 协作者 | 评审者 | 交付/验收 | 性质 |
|---|---|---|---|---|---|---|
| 完成 FND-004A、里程碑 tag 与保护分支切换 | FND-004/目标 Jetson | 项目负责人 | C | B | clean clone ARM64 smoke 通过；`fnd-004a-passed` 指向实测 commit；main protection 生效 | 规划决定 |
| G0 设备身份清单 | 可接触设备但先不发运动命令 | B | 项目负责人、A | 项目负责人 | 所有第 13 节必填项有证据或明确 no-go | 待确认项 |
| CAN transport/拓扑验收 | 采购/现有适配器 | B | A | 项目负责人 | SocketCAN 或 HighTorque CDC 的准确型号/固件/通道、ARM64、隔离、稳定身份、bitrate、时间戳/错误能力记录；必要时验收第二通道 | 待确认项 |
| 协议事实表和 golden frames | 本地手册、供应商资料 | 项目负责人 | B | 另一名非作者 | 每种选定 profile 至少正/负/边界帧，来源可追溯 | 有依据的推断 |
| 配置 schema 与 capability 模型 | 设备身份最小字段 | 项目负责人 | B | C | 缺字段、重 ID、路由重叠和超负载配置可拒绝 | 有依据的推断 |
| 纯 C++ 包边界、vcan CI | 仓库和工具链 | 项目负责人 | C | B | x86 clean build、unit test、vcan smoke test | 有依据的推断 |
| Python command contract 与实验 manifest | 核心时间语义 | C | 项目负责人 | B | mock producer 过期/乱序目标被拒绝 | 有依据的推断 |
| 台架接口与安全需求清单 | 电机身份 | A | 项目负责人、B | 项目负责人 | 夹具、限位、断能、传感器、安装尺寸和交付日 | 待确认项 |

**有依据的推断**：Week 1 集成日设在 Day 5。没有 G0/G1 不阻塞 vcan 和软件工作，但阻塞一切真实电机命令。

### Week 2：协议核心、HI12 和模拟闭环

**有依据的推断**：本节所有待确认项由表内负责人执行，最迟在 Week 2 集成日完成；“交付/验收”列就是确认方法。

| 任务 | 依赖 | 负责人 | 协作者 | 评审者 | 交付/验收 | 性质 |
|---|---|---|---|---|---|---|
| 多 transport backend、router、错误帧/统计 | W1 schema/vcan；HighTorque CDC spike | 项目负责人 | B | C | SocketCAN/vcan 与注入式 CDC 的 raw-frame/filter/fan-out/单写者/队列满测试通过 | 有依据的推断 |
| AK3.0 CubeMars 两套 codec 与 HI12 codec | G0 协议选择 | 项目负责人 | B | 另一名非作者 | 力控先通过；伺服扩展帧随后；各自 golden、边界、DLC/ID/字节序、fuzz 全通过 | 有依据的推断 |
| 设备模拟器和故障脚本 | codec | B | C | 项目负责人 | drop/duplicate/reorder/stale/fault/重启可复现 | 有依据的推断 |
| 两台 HI12 逐台只读 bring-up | G1、台架接线 | B | 项目负责人 | A | PNAME/APP_VER/协议/ID/位速率与抓包归档 | 待确认项 |
| 两台 HI12 联合 30 min 接收 | 唯一 ID、正确终端 | B | 项目负责人 | C | 第 6 节 IMU 指标通过 | 待确认项 |
| 时间/多帧质量与 SYNC_IN 实验 | 型号支持同步 | B | A | 项目负责人 | arrival/sample/trigger 关系和不确定度报告 | 待确认项 |
| 实验记录与回放链路 | manifest、存储限额 | C | B | 项目负责人 | 同一抓包重复解码结果一致 | 有依据的推断 |

**有依据的推断**：Week 2 集成物是“无电机输出的完整数据路径”：模拟电机 + 两台真实或模拟 HI12 + 诊断 + 记录。

### Week 3：ros2_control、电机只读与受限台架

**有依据的推断**：本节所有待确认项由表内负责人执行，最迟在 Week 3 集成日完成；G2/G3 是更严格的先后闸门，“交付/验收”列就是确认方法。

| 任务 | 依赖 | 负责人 | 协作者 | 评审者 | 交付/验收 | 性质 |
|---|---|---|---|---|---|---|
| 复合 SystemInterface 生命周期 | core 稳定、schema | 项目负责人 | B | C | configure/activate/deactivate/error 的 vcan 测试 | 有依据的推断 |
| 状态广播和有界 C++ 测试控制器 | hardware interface | 项目负责人 | C | B | 加载、激活、更新、停用，命令 lease 可观察 | 有依据的推断 |
| 控制器 STRICT 切换与失败回滚 | 两个 mock controller | 项目负责人 | C | B | 100 次切换和失败注入满足第 6 节 | 有依据的推断 |
| G2 单电机只读反馈 | G0/G1、正确供电 | 项目负责人 | B、A | A | 不发运动命令，反馈语义/速率/故障与手册一致 | 待确认项 |
| G3 台架完工和书面检查 | 夹具/计量到位 | A | B、项目负责人 | 非作者交叉检查 | checklist、校准证书、急停/断能演练 | 待确认项 |
| 最小恒定命令 demo | SystemInterface、G2/G3、命令字段语义 | 项目负责人 | A、B | B | C++ 控制器以可配置目标贯通 claim、write、lease、codec、CAN 和诊断；目标从零斜坡进入并回零 | 待确认项 |
| 物理力矩语义与计量 | 最小 demo、供应商资料、必要的计量条件 | 项目负责人 | A、B | A | 区分 requested/reported/physical effort；需要时建立 current/effort 映射 | 待确认项 |
| 500 Hz 周期和 command-to-wire 基准 | 系统可运行 | 项目负责人 | C | B | nominal + CPU/GPU/IO 压力指标报告；另测 1 kHz 控制循环候选 | 待确认项 |

**规划决定（2026-07-30）**：恒定力矩命令只是验证框架的最小 demo，不是最终控制目标。demo 目标值来自配置而非硬编码示例，从零开始按 slew 进入并回零；控制器不直接构造 CAN 帧。真实设备实施 SOP 需另行评审，本规划不授权执行。

### Week 4：系统集成、故障注入、长稳和发布

**有依据的推断**：本节所有待确认项由表内负责人执行，最迟在 Week 4 发布评审前完成；“交付/验收”列就是确认方法。

| 任务 | 依赖 | 负责人 | 协作者 | 评审者 | 交付/验收 | 性质 |
|---|---|---|---|---|---|---|
| 一电机 + 两 IMU 集成 | W2/W3 通过 | 项目负责人 | B、A | C | 30 min 功能试验和完整 manifest | 待确认项 |
| 物理输出力矩测量（独立于最小 demo） | G3、供应商语义、必要的标定映射 | 项目负责人 | A、B | A | 在需要宣称输出端 N·m 精度时，正/负/零目标达到量化误差标准 | 待确认项 |
| 故障注入矩阵 | 可控台架 | B | 项目负责人、C | A | 丢帧、陈旧、bus-off/断线、控制器失败均有期望状态 | 待确认项 |
| 8 h 长稳和磁盘配额 | 集成稳定 | C | B | 项目负责人 | 无溢出/泄漏/未归档数据，指标达标 | 待确认项 |
| 环境复现和 ARM64 clean build | 依赖清单冻结 | C | 项目负责人 | B | 新工作区按 manifest 构建、测试、运行模拟器 | 有依据的推断 |
| `v0.1.0-mvp` 评审和发布 | 全部硬指标 | 项目负责人 | 全员 | 至少两名非作者 | tag、ADR、SBOM/许可证、实验报告、已知限制 | 待确认项 |

## 4. 四人分工与协作机制

### 4.1 角色矩阵

| 工作流 | 负责人 | 协作者 | 评审者 | 必须交接物 | 性质 |
|---|---|---|---|---|---|
| 总体架构、core、ros2_control | 项目负责人 | B、C | B | ADR、接口契约、测试、时序报告 | 有依据的推断 |
| CubeMars 协议与电机集成 | 项目负责人 | B、A | A（台架）、B（协议） | 型号/固件证据、golden、bring-up 日志 | 有依据的推断 |
| HI12、CAN、STM32 边界 | B | 项目负责人、A | 项目负责人 | 身份表、DBC/协议对照、抓包、同步报告 | 有依据的推断 |
| 机械接口与台架 | A | 项目负责人、B | 项目负责人 | CAD/安装/限位/断能/计量 checklist | 有依据的推断 |
| Python、数据和模型接口 | C | 项目负责人 | B | 消息契约、mock policy、数据字典、TTL 测试 | 有依据的推断 |
| CI、复现与实验归档 | C | 项目负责人、B | 项目负责人 | workflow、manifest、恢复演练 | 有依据的推断 |
| 发布与风险接受 | 项目负责人 | 全员 | 至少两名非作者 | release checklist、已知限制、签字 | 有依据的推断 |

**有依据的推断**：协议/安全/生命周期 PR 至少两名评审，其中一人必须不是主要作者；普通 PR 至少一名评审。关键模块必须有共同 owner，禁止只有一人持有设备配置、台架步骤或 release 凭据。

**有依据的推断**：每周一冻结接口/风险，每周三做 vcan 集成，每周五做可重复 demo 和 ADR 复盘。每个 demo 由非作者根据 README 重跑；失败结果同样归档，不用口头解释替代证据。

**有依据的推断**：A 自主管理机械任务，但软件团队通过明确的安装界面、方向/零位、夹具、限位、急停和测量需求与其协作；C 首月重点是数据/策略边界、CI 和复现，不为“有 AI”而把网络塞入 MVP 闭环。

## 5. 测试金字塔

| 层级 | 测试对象 | 关键用例 | 运行频率 | 通过标准 | 性质 |
|---|---|---|---|---|---|
| T0 静态/schema | 配置、DBC、API、许可证 | 缺字段、重 ID、单位、固件范围、依赖许可证 | 每 PR | 无 schema/许可证阻断 | 有依据的推断 |
| T1 纯单元 | codec、限幅、时间、状态机 | golden、端序、DLC、边界、NaN、wrap、fuzz/property | 每 PR | codec 分支覆盖目标 >=90%；所有已知帧精确匹配 | 有依据的推断 |
| T2 vcan 集成 | transport、router、队列、filter | 多设备、fan-out、loopback、回放、drop/duplicate/stale | 每 PR/CI | 无死锁、无无界分配、期望 counter/state | 有依据的推断 |
| T3 行为模拟 | device session、ros2_control | 电机/IMU 周期、故障码、命令 watchdog、切换 | 每 PR/夜间 | 生命周期和命令租约可重复 | 有依据的推断 |
| T4 单设备 | 每台 HI12、单电机只读 | 身份、速率、缩放、方向、温度、故障 | 手工 gated | 与手册/抓包/计量一致 | 有依据的推断 |
| T5 台架集成 | 一电机 + 两 IMU | 最小恒定命令 demo、切换、同步、记录；物理力矩计量单列 | 每里程碑 | 第 6 节对应指标 | 有依据的推断 |
| T6 故障注入 | bus、进程、设备、存储 | 断包、延迟、错误帧、bus-off、重启、磁盘阈值 | Week 4/发布 | 每种故障进入预期状态且可恢复 | 有依据的推断 |
| T7 长稳 | 完整 MVP | nominal/stress 8 h，资源和数据完整性 | RC/发布 | 无未解释 fault、overflow、泄漏或数据缺口 | 有依据的推断 |

**已确认事实**：vcan 可以验证 SocketCAN API、路由和报文逻辑，但不模拟真实位级仲裁、终端电阻、收发器、电磁干扰、控制器错误计数或真实 bus-off。

**有依据的推断**：故障注入分层进行：T1/T2 注入损坏/缺失/乱序数据，`canplayer` 做真实日志回放，`canerrsim`/错误帧做软件路径测试，真实 bus-off/断线/电源循环只在 G3 后按台架 SOP 执行。CI 不得自动访问真实电机。

## 6. MVP 可量化验收标准

| 类别 | 必须指标 | 测量窗口/方法 | 性质 |
|---|---|---|---|
| 可构建 | 受保护 `main` 的 clean checkout 或 PR commit 按 manifest 构建；所有 T0~T3 测试通过 | 两次独立 build；保存命令、依赖和 commit/tag | 有依据的推断 |
| 协议 | 所有公开 codec golden 精确匹配；非法 ID/DLC/范围被拒；有效帧 parse error 为 0 | 单元/fuzz + 供应商/真实抓包 | 有依据的推断 |
| HI12 身份 | 两台 PNAME、APP_VER、协议、节点、位速率、输出 profile、坐标和安装方向均记录 | 逐台配置读取与抓包 | 有依据的推断 |
| HI12 稳定性 | 每个配置 PGN/TPDO 30 min；计数达到期望的 >=99.9%；软件 RX overflow=0；最大无解释 gap <=3 个期望周期 | 应用 counter + candump + 接口统计 | 有依据的推断 |
| 电机身份/反馈 | 型号、减速比、HW/FW、模式、ID、位速率、反馈语义已证实；30 min 目标速率下 overflow=0、未知 fault=0 | 只读反馈与手册对照 | 有依据的推断 |
| 最小恒定命令 demo | C++ 控制器持续输出可配置、有界目标；从零按 slew 进入、保持并回零；每周期 target、generation、deadline 与最终编码可追踪；控制器不直接操作 CAN | vcan/模拟器先验收，真实设备阶段保存控制器、核心和 candump 记录 | 规划决定 |
| 物理输出力矩（独立验收） | 只有需要宣称输出端 N·m 精度时才适用；命令峰值不超过“经确认连续额定的 10%”与台架上限两者较小值 | 供应商语义、原始计量数据、校准证书和脚本版本 | 有依据的推断 |
| 物理力矩误差 | 去除 2 s settling 后，均值绝对误差 <= `max(目标绝对值的10%, 3倍传感器不确定度)`；标准差 <= `max(目标绝对值的5%, 3倍传感器不确定度)` | 仅用于物理输出力矩验收，不作为最小 demo 通过条件 | 有依据的推断 |
| 生命周期 | hardware configure/activate/deactivate/cleanup 与 controller load/configure/activate/deactivate 均各成功 10 次 | 自动化 vcan + 一次台架演示 | 有依据的推断 |
| 控制器切换 | STRICT 切换 100 次无资源泄漏；输出变化不越 slew；命令无效窗口 <=2 控制周期 | mock + 台架低能量 | 有依据的推断 |
| 500 Hz 循环 | 2 ms 目标；nominal 与压力各 30 min：初始门槛 p99 周期误差 <=0.25 ms，p99.9 <=0.5 ms，最大 <=1 ms；连续 miss 不超过 1 次 | 单调时间直方图，保留完整原始样本；门槛须在控制需求评审后锁定 | 有依据的推断 |
| 1 kHz 控制候选 | 1 ms controller_manager 循环可配置；先允许电机 I/O 保持 500 Hz，不把重复快照伪装为新反馈 | nominal/stress 对比 500 Hz，记录状态 age 和实际 `dt` | 有依据的推断 |
| 命令 watchdog | controller_manager 不再刷新时，软件命令在 <=3 个控制周期内失效（500 Hz 为 <=6 ms）并进入已定义 fault；分级语义见 [ADR-012](../adr/ADR-012-command-watchdog-and-capability-honesty.md)：软 TTL 后冻结最后一个有效命令，硬 TTL 后显式失败。位置类命令的「neutral」不得实现为运动到零位；驱动器物理行为另行确认 | 模拟线程 stall；真实硬件仅 G3 后 | 有依据的推断 |
| CAN nominal | 每总线平均占用 <=50%，任意 1 s 峰值 <=60%；RX dropped/overrun=0；bus error/bus-off=0 | `canbusload`、接口统计、应用 counter | 有依据的推断 |
| 诊断 | 所有故障注入在 1 个诊断发布周期内可见，包含首次时间、原始码、计数、设备和 lifecycle 状态 | 故障矩阵 | 有依据的推断 |
| 可复现 | 每次正式实验绑定 Git commit/tag、配置 SHA-256、设备身份、host manifest、rosbag/candump URI 与哈希 | manifest 自动校验 | 有依据的推断 |
| 回放 | 同一 golden/candump 回放 3 次，解码值、序号、fault 结果一致；允许墙钟不同 | vcan/canplayer + ROS 级回放 | 有依据的推断 |
| 8 h 长稳 | 无未解释 fault/overflow；RSS 在 warm-up 后增长 <=50 MiB；磁盘不越配额；所有数据有 manifest | nominal + 记录 | 有依据的推断 |

**有依据的推断**：上述抖动、误差、负载和超时是首月工程验收线，不是医学/功能安全指标。若控制需求证明更严，应在 ADR 中提高；不能在测试后为了“通过”降低而不记录风险接受。

**已确认事实**：若整个控制进程被杀死，进程内 TX watchdog 也会停止。只有驱动器自身经确认的命令超时、独立安全 MCU 或物理断能才能覆盖该故障；未确认前，kill-process 测试预期结果是物理安全系统断能，而不是宣称软件能发送 neutral。

## 7. CAN 最坏带宽计算

### 7.1 经典 CAN 帧成本

**已确认事实**：对包含 `d` 个数据字节的经典 CAN 数据帧，下面的保守计算包含 SOF、仲裁、控制、数据、CRC、ACK、EOF 和 3 bit intermission，并对 SOF 到 CRC sequence 采用最坏 bit stuffing：

- **已确认事实**：11 位标准帧未填充位数为 `47 + 8d`，最大填充位为 `floor((33 + 8d)/4)`。
- **已确认事实**：29 位扩展帧未填充位数为 `67 + 8d`，最大填充位为 `floor((53 + 8d)/4)`。

| 帧 | 未填充 bit | 最大 stuffing | 最坏总 bit | 性质 |
|---|---:|---:|---:|---|
| 标准 6 byte | 95 | 20 | 115 | 已确认事实 |
| 标准 8 byte | 111 | 24 | 135 | 已确认事实 |
| 扩展 4 byte | 99 | 21 | 120 | 已确认事实 |
| 扩展 8 byte | 131 | 29 | 160 | 已确认事实 |

**有依据的推断**：总线预算公式为 `utilization = sum(frame_worst_bits × frame_rate) / nominal_bitrate`。实际位模式通常低于最坏 stuffing，但规划使用最坏数；错误帧、重传、配置流量和仲裁延迟另由余量与实测覆盖。

### 7.2 设备 profile 成本

| Profile | 每更新帧 | 每设备每更新最坏 bit | 性质 |
|---|---|---:|---|
| **AK3.0 力控命令 + `0x29` 反馈** | 扩展 8B + 扩展 8B | **320** | **当前第一实现 profile**；三个子模式共用同一 8 字节布局，没有更省的档 |
| AK3.0 伺服 电流/位置/转速命令 + `0x29` 反馈 | 扩展 4B + 扩展 8B | 280 | 第二实现 profile 的较省档 |
| AK3.0 伺服 位置速度环命令 + `0x29` 反馈 | 扩展 8B + 扩展 8B | 320 | 第二实现 profile 的保守预算 |
| 可选 `0x2A` 增量 | 额外扩展 4B | 120 | 本代可选位置反馈帧，由写 Flash 的功能 ID `16` 启用，默认关闭 |
| ~~L02 motion-control/MIT 命令 + 8B 反馈（标准帧）~~ | ~~标准 8B + 标准 8B~~ | ~~270~~ | **2026-09-01 移除**（[ADR-013](../adr/ADR-013-ak30-protocol-baseline.md)）：11 位标准帧 profile 在本项目硬件上不存在 |
| HI12 J1939 accel+gyro+quaternion | 3 × 扩展 8B | 480 | 已确认事实 |
| HI12 J1939 出厂常见四组数据 | 4 × 扩展 8B | 640 | 已确认事实 |
| HI12 CANopen accel+gyro+quaternion | 标准 6B + 6B + 8B | 365 | 已确认事实 |

### 7.3 场景表

| 场景 | 位速率 | 最坏流量 | 占用 | 判断 | 性质 |
|---|---:|---:|---:|---|---|
| 1 力控电机，200 Hz | 1 Mbit/s | 64 kbit/s | 6.4% | bring-up 有充足余量 | 有依据的推断 |
| 2 力控电机，500 Hz | 1 Mbit/s | 320 kbit/s | 32.0% | 当前两电机第一 profile 候选 | 有依据的推断 |
| **上述 2 电机 500 Hz + 2 HI12 J1939 三帧 100 Hz** | 1 Mbit/s | **416 kbit/s** | **41.6%** | **当前单物理通道基线候选**；仍需同位速率、ID、backend 和时序证据 | 有依据的推断 |
| 2 力控电机 + `0x2A` @500 Hz，再加上述 HI12 | 1 Mbit/s | 536 kbit/s | 53.6% | **超出平均 50% 目标；500 Hz 下不得启用 `0x2A`** | 有依据的推断 |
| 2 力控电机 @1 kHz，再加上述 HI12 | 1 Mbit/s | 736 kbit/s | 73.6% | 拒绝作为单通道配置 | 有依据的推断 |
| 上一场景再加 `0x2A` @1 kHz | 1 Mbit/s | 976 kbit/s | 97.6% | 拒绝 | 有依据的推断 |
| 2 伺服电机，current + `0x29`，500 Hz | 1 Mbit/s | 280 kbit/s | 28.0% | 第二 profile 的较省档 | 有依据的推断 |
| 2 HI12，J1939 三帧，100 Hz | 500 kbit/s | 96 kbit/s | 19.2% | 推荐最小输出 profile | 有依据的推断 |
| 2 HI12，J1939 默认四帧，100 Hz | 500 kbit/s | 128 kbit/s | 25.6% | 可行但多发姿态帧 | 有依据的推断 |
| 2 HI12，CANopen 三 TPDO，100 Hz | 500 kbit/s | 73 kbit/s | 14.6% | 只适用于确认 CANopen 固件 | 有依据的推断 |
| 1 力控电机 @200 Hz + 2 J1939 三帧 @100 Hz，共享 500 kbit/s | 500 kbit/s | 160 kbit/s | 32.0% | 纯带宽示例，不解决电机资料值 1 Mbps 与实际共同位速率问题 | 有依据的推断 |
| 6 力控电机，200 Hz | 1 Mbit/s | 384 kbit/s | 38.4% | 六电机设计包络首选基线 | 有依据的推断 |
| 6 力控电机，200 Hz + 2 HI12 | 1 Mbit/s | 480 kbit/s | 48.0% | 接近平均预算上限，需实测 | 有依据的推断 |
| 6 力控电机，250 Hz | 1 Mbit/s | 480 kbit/s | 48.0% | 接近平均预算上限，需实测 | 有依据的推断 |
| 6 力控电机，500 Hz | 1 Mbit/s | 960 kbit/s | 96.0% | 拒绝作为通用配置 | 有依据的推断 |
| 2 HI12 默认 + STM32 假设 4 扩展帧@200 Hz | 500 kbit/s | 256 kbit/s | 51.2% | 仅规划示例，表明 STM32 profile 必须先预算 | 有依据的推断 |

**待确认项**：最后一行不是 STM32 已定协议。成员 B 在传感器数量/采样率确定后重算 Classic/FD 帧成本、最坏响应时间和分片，最迟在 STM32 协议 ADR 评审前完成；在此之前不分配 CAN ID 或固定 payload。

**有依据的推断**：占用率不足以证明 deadline。每个高优先级帧可能延迟低优先级命令，USB 适配器还可能批处理；实施时必须测量 `controller write -> kernel TX -> wire -> feedback RX`，并对所有更高优先级 ID 做最坏响应时间分析。

## 8. 频率、抖动、新鲜度和负载实测计划

### 8.1 频率晋级

| 阶段 | controller_manager | 电机命令/反馈 | HI12 | 条件 | 性质 |
|---|---:|---:|---:|---|---|
| 模拟与首次硬件 bring-up | 100~200 Hz | 100~200 Hz | 100 Hz simulated/actual | 功能、方向、时间和故障字段正确 | 有依据的推断 |
| 当前两电机 MVP 候选 | **500 Hz** | 目标最高 **500 Hz** | 100 Hz actual，必要时 200 Hz | AK3.0 力控 profile + `0x29` 反馈、第 6 节指标和 ADR-006 的单通道/backend 证据全部通过。**2026-09-01 按 [ADR-013](../adr/ADR-013-ak30-protocol-baseline.md) 重算后维持 500 Hz**：力控帧长与切换前相同，两电机加两台 HI12 为 41.6%，仍在 50% 平均预算内。**前提是不启用 `0x2A`**（启用则 53.6%，超预算） | 规划目标；部署仍 Proposed |
| 1 kHz 控制实验 | 1 kHz | 先保持 500 Hz | 100/200 Hz | 状态 age、实际 `dt`、抖动和控制收益通过 | 有依据的推断 |
| 1 kHz 电机 I/O | 1 kHz | 1 kHz | 100/200 Hz | **非当前目标**。协议上限已不再是障碍（L07 支持 1–2000 Hz），**限制来自总线预算**：两电机 1 kHz 加两台 HI12 为 73.6%，明确拒绝。需第二条总线或大幅削减设备后重新预算并通过 ADR | 待确认项 |
| 六电机扩展 | 按控制需求配置 | 200~250 Hz 起测 | 独立预算 | 不沿用当前两电机 profile | 有依据的推断 |

### 8.2 测量矩阵

| 维度 | 水平 | 记录 | 性质 |
|---|---|---|---|
| 系统负载 | idle、nominal ROS、CPU stress、GPU inference、受限磁盘记录 | loop dt、scheduler latency、page faults、CPU、GPU、RSS | 有依据的推断 |
| 总线负载 | 100/200/500/1000 Hz；当前两电机与合成六电机 | CAN 占用、仲裁延迟、TX queue、RX drop、错误状态 | 有依据的推断 |
| 数据质量 | 两 IMU 异步、SYNC_IN、drop/duplicate | field age、gap、coherence、clock offset/uncertainty | 有依据的推断 |
| 控制切换 | effort->effort、停止->激活、失败激活 | generation、命令 gap、slew、回滚状态 | 有依据的推断 |
| 故障 | controller stall、Python stall、bus error、device stale | 检测时延、neutral/fault、恢复步骤 | 有依据的推断 |

**有依据的推断**：主机测量使用 `CLOCK_MONOTONIC` 原始样本，不只报平均值；报告 p50/p95/p99/p99.9/max 和超阈计数。CAN 同时保存 `candump`、`ip -details -statistics` 前后快照、`canbusload` 与应用计数，避免单一工具漏报。

**有依据的推断**：任何性能测试前先 warm-up、预分配、锁页并记录 CPU governor/温度/频率；是否使用 `mlockall`、线程优先级和 CPU affinity 必须通过权限与回滚评审，不能在本规划阶段修改主机。

## 9. Jetson 环境复现、依赖和部署

### 9.1 依赖事实源

| 类别 | 记录方式 | 性质 |
|---|---|---|
| ROS/apt | `package.xml` + rosdep keys + release 时的已安装版本清单 | 有依据的推断 |
| 源依赖 | `manifests/dependencies.repos` 固定 commit，不跟随 floating branch | 有依据的推断 |
| 编译器/构建 | GCC/CMake/colcon 参数、C++17、构建类型、测试选项 | 有依据的推断 |
| 主机 | Jetson 型号、L4T、内核、ROS、CUDA/TensorRT、CAN driver、权限 | 有依据的推断 |
| 设备 | 型号/序列号/HW/FW/profile/配置哈希 | 有依据的推断 |
| 容器 | 开发镜像 Dockerfile、基础镜像 digest、SBOM | 有依据的推断 |

**有依据的推断**：开发者用容器或标准 ROS 22.04 环境运行 T0~T3；目标控制进程原生部署到版本化安装目录。部署包只读，配置通过 schema 校验；host-specific interface mapping 与机器人配置分离。

**有依据的推断**：运行用户不使用永久 root。后续按最小权限设计 CAN 设备访问、memlock 和 realtime priority；systemd、udev、limits、内核或网络变更必须独立 Issue、双人评审、变更前后清单和回滚命令。本轮不执行这些变更。

**有依据的推断**：PREEMPT_RT 是测量触发的优化，不是先决假设。若当前内核不达标，先证明阻塞/缺页/日志已移出 RT 路径，再按 NVIDIA 对当前 L4T 版本的正式说明建立第二启动项；切换失败能返回 generic kernel。

### 9.2 部署流水线

1. **有依据的推断**：CI 在 amd64 执行 T0~T3，并构建 ARM64 目标或在无硬件权限的 Jetson runner 做 native build。
2. **有依据的推断**：合并到 main 产生候选 install artifact、SBOM、测试结果和 SHA-256，不自动部署实验台。
3. **有依据的推断**：人工选择 tag 和 deployment config，预检磁盘、设备身份、接口映射和闸门。
4. **有依据的推断**：先启动只读/模拟模式，再由授权人员切换到台架 ACTIVE；正式实验固定 artifact，不在现场编译修改。
5. **有依据的推断**：回滚切换到上一版本目录和配置，硬件必须先 INACTIVE/断能。

## 10. GitHub、NAS、CI、版本与实验数据

### 10.1 Git 工作流

| 项目 | 规则 | 性质 |
|---|---|---|
| 主仓库 | 私有 GitHub 是唯一可写事实源；NAS 仅 mirror/备份 | 有依据的推断 |
| 分支 | protected `main` + 短生命周期 issue branches；禁止共享 NAS working tree | 有依据的推断 |
| PR | 关联 Issue、验收、风险、测试、硬件影响、数据链接；协议/安全需双评审 | 有依据的推断 |
| CODEOWNERS | 评审人身份明确后按 core/ROS、protocol、data、rig 责任配置；在此之前不猜测账号或制造单人合并死锁 | 规划决定 |
| 版本 | 软件 SemVer；MVP `v0.1.0-mvp`；协议、配置 schema 和数据 manifest 独立 version | 有依据的推断 |
| 提交 | 小而可审；生成物不手工改；硬件结果必须附原始证据 | 有依据的推断 |
| NAS | 定时 `--mirror` 备份和恢复演练；不接受直接开发提交 | 有依据的推断 |

### 10.2 CI 分层

| 流水线 | 触发 | 内容 | 硬件权限 | 性质 |
|---|---|---|---|---|
| fast | 每 PR | format/lint/schema/license/unit/golden | 无 | 有依据的推断 |
| integration | 每 PR | vcan、模拟器、lifecycle、switch、replay | 仅 vcan | 有依据的推断 |
| ARM64 | main/夜间 | native/cross build、unit、package test | 无真实 CAN | 有依据的推断 |
| stress | 夜间/RC | sanitizers、fuzz、长时模拟、性能回归 | 无 | 有依据的推断 |
| HIL | 人工 gated | 单设备/台架/故障/长稳 | 实验室授权且 G3 | 有依据的推断 |

**有依据的推断**：自托管 Jetson runner 默认看不到真实 CAN 设备和电机电源；任何 HIL workflow 需要环境保护、人工确认、最大命令和超时。CI 失败不能通过重跑掩盖，必须保留 flaky counter。

### 10.3 数据与实验管理

| 资产 | 存放 | Git 中保存 | 性质 |
|---|---|---|---|
| 代码、小配置、DBC、golden frames | Git | 全部 | 有依据的推断 |
| 大模型权重 | Git LFS 或模型 registry/release asset，按规模选择 | URI、SHA-256、license、训练 commit | 有依据的推断 |
| rosbag/candump/高速日志 | NAS/对象存储 | manifest、哈希、大小、保留期 | 有依据的推断 |
| 校准和台架原始数据 | 受控实验存储 | 不可变 manifest、传感器证书、处理脚本 commit | 有依据的推断 |
| 发布 artifact/SBOM | GitHub Release + NAS mirror | release manifest | 有依据的推断 |

**有依据的推断**：每个实验目录包含 experiment ID、目的、操作者、UTC 时间、Git commit/tag、dirty 状态、artifact hash、配置 hash、host/device manifest、控制器集合、闸门签字、原始数据 URI/hash、结果和异常。dirty tree 的正式结果不得作为发布证据。

**有依据的推断**：录制按大小/时间分包，正式试验前保证至少 `max(25 GiB, 根分区15%)` 可用空间；低于阈值拒绝开始新录制并报告，不删除用户数据。具体分包大小在磁盘吞吐测试后锁定，初始上限候选为 2 GiB/文件或 10 min/文件。

## 11. `AGENTS.md`、项目配置与 skills 规划

### 11.1 `AGENTS.md`

**规划决定（2026-08-06）**：根 `AGENTS.md` 已建立为所有 AI 对话的强制入口；详细 SOP 位于 `docs/development/ai_collaboration_workflow.md`。每个新项目任务结束或暂停前，AI 必须自动更新并验证 project memory；`write-codex-handoff` 仍按事件触发，仅在用户明确要求写交接，或发生跨人员/机器/阶段、长暂停、上下文风险或未完成高风险工作时 CREATE。后续实现阶段再补充实际验证过的构建命令、包所有权、PR 测试要求和 CODEOWNERS，不能把计划命令写成已通过事实。

### 11.2 `.codex/config.toml`

**有依据的推断**：项目配置只保存可审查的仓库级设置，例如可信 workspace、默认 sandbox、文档路径和非破坏性测试命令；不得保存 token、SSH key、设备秘密、宽泛 sudo 或自动硬件授权。任何提高权限的变更走 PR。

### 11.3 skills 路线

**规划决定（2026-08-06）**：长期连续性由仓库级 `AGENTS.md` + 工具无关 SOP + `project-memory` + 事件触发式 `write-codex-handoff` 共同负责；project memory 由 AI 在每个新项目任务结束/暂停前自动维护，handoff 不因普通任务完成自动产生。当前不新建只负责包装这两个通用 skill 的第三个 continuity skill。任何项目专用 skill 必须先有至少三次成功人工 SOP、稳定输入/输出/禁区、仓库内 fallback 和可重复验证。

| 候选 skill | 前置人工流程 | 输出 | 禁区 | 性质 |
|---|---|---|---|---|
| `jetson-inventory-readonly` | 盘点命令经两次人工使用验证 | 版本/磁盘/接口 manifest | 不安装、不清理、不改服务 | 有依据的推断 |
| `can-capture-analyze` | 抓包 SOP 和过滤规则稳定 | candump + 统计 + 哈希报告 | 默认只读，不发帧 | 有依据的推断 |
| `protocol-golden-frame` | codec schema 和测试模板成熟 | 正/负/边界测试 | 不从相似仓库猜字段 | 有依据的推断 |
| `hardware-bringup-gated` | G0~G3 和回滚流程演练 | checklist 和证据包 | 未批准不启用接口/设备 | 有依据的推断 |
| `experiment-archive` | manifest 和 NAS API 稳定 | 数据哈希、URI、索引 | 不自动删除源数据 | 有依据的推断 |
| `release-check` | CI、SBOM、ADR、HIL 清单稳定 | release readiness 报告 | 不自动发布/部署 | 有依据的推断 |

**有依据的推断**：首月只建立上述路线，不批量创建未经真实流程验证的 skills。AI 输出必须走同样 PR、测试和评审，skills 不能替代项目文档与接口契约。

## 12. 风险清单

| ID | 风险 | 概率 | 影响 | 触发条件 | 缓解/应对 | 负责人 | 性质 |
|---|---|---|---|---|---|---|---|
| R01 | AKE60-8 基型已知但定制件号/驱动板/固件不明 | 高 | 高 | W1D2 仍无两台实机读取证据 | G0 no-go；逐台导出配置；关键参数必填 | 项目负责人 | 有依据的推断 |
| R02 | 把标准 AKE60-8 Kt/范围误用于定制实机 | 中 | 高 | 未核对配置就硬编码 `0.7382` 或范围 | 实机参数/供应商确认前不锁定 effort 映射；物理精度另验收 | 项目负责人 + A | 有依据的推断 |
| R03 | 电机无可靠命令 watchdog | 中 | 高 | 断包后保持非零输出 | 物理断能；缩短软件租约；评估独立安全节点 | B + 项目负责人 | 有依据的推断 |
| R04 | 两 HI12 均为节点 8 或协议不同 | 高 | 中 | 抓包冲突/无输出 | 逐台识别和配置，保存前后证据 | B | 有依据的推断 |
| R05 | 只有 `can0` 且默认位速率不同 | 高 | 高 | 第二接口未到/驱动不支持 | 采购隔离 SocketCAN 适配器；禁止盲目共总线 | B | 有依据的推断 |
| R06 | USB-CAN 延迟/断连 | 中 | 高 | p99.9 或 disconnect 不达标 | 选主线驱动、稳定序列号、HIL 压测；必要时换接口 | B | 有依据的推断 |
| R21 | HighTorque USB-CDC 示例能力或许可不足 | 中 | 高 | 板卡/固件、bitrate、错误/时间戳、队列或许可证证据缺失 | reference-only；注入式离线 spike；保留 SocketCAN backend；未闭合前禁止真实激活 | 项目负责人 | 规划决定 |
| R22 | AK3.0 力控与伺服两种 profile 被误混发/热切换 | 低 | 极高 | 同一 ACTIVE session 接受两种 codec 或自动探测 | 配置期固定；独立 codec/session；negative cross-profile tests；切换需停用/neutral/断能和重新 configure | 项目负责人 | 规划决定 |
| R07 | 非 PREEMPT_RT 抖动超标 | 中 | 高 | 第 6 节周期失败 | 清除 RT 路径阻塞；再评估官方 RT 内核与回滚 | 项目负责人 | 有依据的推断 |
| R08 | 六电机、高频或补充 AK3.0 `0x2A` 导致拥塞 | 中 | 高 | 平均 >50% 或 queue/drop | 当前 力控两电机 500 Hz 与六电机 200~250 Hz 使用独立 profile；`0x2A` 在 500 Hz 下超预算，只有降频或分总线后才评估；必要时分总线 | 项目负责人 | 有依据的推断 |
| R09 | 多帧 IMU 被误当同步样本 | 高 | 中 | 无序号却发布 coherent | 字段级 age；SYNC_IN 实测；质量标志 | B | 有依据的推断 |
| R10 | 坐标、安装方向或 yaw 约定错误 | 中 | 高 | 静态/转台方向测试失败 | 四元数为事实源；安装变换校准；协议专用缩放 | B + A | 有依据的推断 |
| R11 | 第三方代码许可证不清 | 中 | 中 | 根 LICENSE 缺失/复制代码 | 仅参考；依赖前 legal/SPDX scan 和 commit lock | 项目负责人 | 有依据的推断 |
| R12 | Humble 错误恢复/未来 EOL | 中 | 中 | 恢复测试失败或平台升级 | 显式恢复；核心隔离 ROS；单独迁移 ADR | 项目负责人 | 有依据的推断 |
| R13 | Python/GPU 干扰控制 | 中 | 高 | 压力测试抖动/目标过期 | 独立进程、SCHED_OTHER、TTL、CPU/资源测量 | C | 有依据的推断 |
| R14 | 磁盘 80% 导致录制失败 | 高 | 中 | 可用空间低于阈值 | preflight、分包、外部存储；不自动清理 | C | 有依据的推断 |
| R15 | rosbag 回放被误当物理复现 | 中 | 中 | 只用 ROS 时间宣称总线通过 | candump/canplayer + HIL；记录限制 | C + B | 有依据的推断 |
| R16 | STM32 ADC/传感器需求变化 | 高 | 中 | 通道/BW/信号形式改变 | 首月不定前端；只定语义边界 | B + A | 有依据的推断 |
| R17 | 月度范围过大 | 中 | 高 | W2 仍未过 G0/G1 | 并行软件/HW；保留 no-go；不塞入 ML/六电机 | 项目负责人 | 有依据的推断 |
| R18 | 台架或急停不足 | 中 | 极高 | G3 checklist 任何项失败 | 禁止非零命令；机械/电气整改并复审 | A | 有依据的推断 |
| R19 | 设备配置持久化漂移 | 中 | 高 | 断电后 ID/位速率/profile 与清单不同 | 启动只读身份核对；配置 hash；不自动写 Flash | B | 有依据的推断 |
| R20 | 保护规则配置造成绕过或单人合并死锁 | 中 | 中 | required checks 名称错误、强制未知 reviewer、owner 可绕过 main | FND-004A tag 后核验 ruleset；先强制 PR/checks/对话解决，评审人到位后再启用 approval/CODEOWNERS | 项目负责人 | 规划决定 |

## 13. 硬件最小确认清单

### 13.1 CubeMars 电机与驱动器

| 必须信息 | 确认方法 | 负责人 | 最迟 | 性质 |
|---|---|---|---|---|
| 基型 AKE60-8、完整定制件号、序列号 | 基型已由用户确认；补两台实机连接/版本记录或订单/BOM | 项目负责人 | W1D2 | 部分确认 |
| 驱动板 HW、FW、构建/发布日期 | 上位机/启动输出只读、供应商固件包哈希 | B | W1D2 | 待确认项 |
| 当前 AK3.0 力控/伺服 active profile、驱动板/固件、CAN ID、位速率和反馈设置（含单圈模式与 `0x2A` 的 Flash 持久状态） | 每台上位机/供应商工具基础设置截图、只读导出 + 后续被动证据 | B | G0 | 待确认项 |
| `0x29`（以及启用时的 `0x2A`）编码器来源、方向与零位 | 供应商帧说明 + 配置记录 + 后续手动方向校验 | 项目负责人 + A | G2 前 | 待确认项 |
| 定制版 Kt、减速比、允许电流/速度/温度 | 对比标准 V3.2、两台 `.AppParams`/`.McParams` 和供应商确认 | 项目负责人 | G3 前 | 部分确认 |
| 命令超时、neutral、故障复位、上电行为 | 固件说明 + G3 后断包测试 | B | 真实电机最小 demo 前 | 待确认项 |
| 机械连续/峰值力矩与夹具安全上限 | 准确型号数据 + 结构计算 + 计量 | A | G3 | 待确认项 |

### 13.2 两台 HI12

| 必须信息 | 确认方法 | 负责人 | 最迟 | 性质 |
|---|---|---|---|---|
| 完整订购码/PNAME/序列号、接口是否内置收发器 | 铭牌/命令读取/接线核对 | B | W1D2 | 待确认项 |
| APP_VER、交付协议 J1939 或 CANopen | 逐台读取和抓包，不靠帧类型猜 | B | W1D2 | 待确认项 |
| 节点 ID、位速率、启用 PGN/TPDO、周期 | 逐台只读配置；保存 raw 值 | B | W1D3 | 待确认项 |
| 坐标系、安装方向、6-DoF/AHRS profile | 配置读取 + 静态六面/转动测试 | B + A | W2 | 待确认项 |
| 时间字段、SYNC_IN/PPS/SOUT 引脚和固件支持 | 订购码/引脚 + 逻辑分析仪 | B | W2 | 待确认项 |
| 终端电阻、供电、线束屏蔽/拓扑 | 万用表/原理图/现场图 | B + A | G1 | 待确认项 |

### 13.3 CAN、Jetson 与台架

| 必须信息 | 确认方法 | 负责人 | 最迟 | 性质 |
|---|---|---|---|---|
| transport 型号、序列号/稳定 USB 身份、板卡固件、Classic/FD、nominal/data bitrate、时间戳/错误能力 | 采购/BOM、版本化供应商资料；HighTorque CDC 与 SocketCAN 分别验收 | B | G0/G1 | 待确认项 |
| 每段拓扑、线长、终端、共地/隔离、连接器 pinout | 现场图和电气检查 | B + A | G1 | 待确认项 |
| 电源电压、限流、保险/断能、急停 | 原理图和实操断能演练 | A + B | G3 | 待确认项 |
| 刚性夹具、机械限位、防护、允许方向/角度 | CAD/照片/检查表 | A | G3 | 待确认项 |
| 力/力矩传感器型号、量程、精度、校准日期、采样率 | 证书和砝码/基准校验 | A | G3 | 待确认项 |
| Jetson 调度/温控/电源模式基线 | 只读 manifest 和压力测试 | 项目负责人 | W3 | 待确认项 |

### 13.4 未来 STM32

| 必须信息 | 确认方法 | 负责人 | 最迟 | 性质 |
|---|---|---|---|---|
| 开发板准确 revision 与完整原理图 | 实物/BOM/供应商资料 | B | MVP 后设计前 | 待确认项 |
| 压力/扭矩传感器数量、桥路/电压/数字接口、量程、带宽 | 传感器选型和机械需求 | B + A | MVP 后设计前 | 待确认项 |
| ADC/AFE、供电隔离、抗混叠和标定策略 | 电气评审 | B | 原理图冻结前 | 待确认项 |
| Classic CAN/CAN FD 拓扑和消息预算 | 信号表后带宽计算/HIL | B + 项目负责人 | 协议 ADR 前 | 待确认项 |

## 14. 推荐的下一步行动顺序

> **2026-09-06 进展标注：** 下表为原规划基线，其中多项已完成（FND-004A/Foundation RC/力控 golden vectors/单电机台架实测），未完成项仍然有效。当前实际进展以[规划索引](README.md) §3 为准。

### 14.1 硬件闸门进展（2026-09-06 标注）

| 闸门 | 必须证据 | 当前进展 | 未通过时 |
|---|---|---|---|
| G0 设备身份 | 电机/HI12 型号、固件、协议、ID、位速率和 profile | motor1 部分取证（参数/映射实测收口，见 [05 §7](05_decisions_and_open_questions.md)）；完整配置导出、第二台电机与 HI12 仍缺 | 只做模拟与离线协议工作 |
| G1 被动总线 | 接线、终端、只读抓包、ID/位速率与负载无冲突 | 单电机台架实测通过（50 Hz 被动反馈、终端按 owner 决策接受风险）；最终拓扑（多设备共总线）仍待做 | 停止总线集成 |
| G2 电机反馈 | 不发运动命令即可稳定获得语义明确的反馈和故障 | motor1 实测通过（`0x29` 语义明确、状态字节 0）；第二台电机仍待做 | 不进入命令阶段 |
| G3 台架安全与计量 | 夹具、限位、急停/断能、限流和必要计量均书面通过 | 空载台架按 owner 2026-09-02 缩放执行（无负载、失控保护自由滑行、逐次授权、首条命令极小值）；正式书面检查表仍待做 | 禁止非零真实命令 |
| G4 集成发布 | 频率、时序、故障、长稳、记录和复现达到本文件第 6 节标准 | 未开始 | 不发布 MVP |

**台架实测不构成最终部署 profile 的闸门验收**；边界与重审条件见 [ADR-006](../adr/ADR-006-conditional-can0-deployment.md) Decision 第 7 条。

1. **已完成/待证据**：FND-004 已接受 ADR-001/002/003/004/005/009；[ADR-006](../adr/ADR-006-conditional-can0-deployment.md) 继续等待逐台配置、所选 backend 能力、共同位速率、ID、终端、负载、仲裁和错误证据，未转为 Accepted 前不得激活当前单物理通道 profile。
2. **规划决定**：执行 FND-004A；通过后给实际测试的 commit 创建 `fnd-004a-passed` annotated tag，并保护 `main`。FND-005 起所有人通过任务分支和 PR 开发；外部成员可 fork。
3. **待确认项**：Foundation 后由 B 与项目负责人完成电机和两台 HI12 的身份表；任何未知保留为空，不补默认值。
4. **待确认项**：A 在 W1D2 前给出台架接口、机械限位、急停/断能和力矩计量交付计划。
5. **待确认项**：B 在 G0/G1 提交候选 transport 的准确型号/固件/通道能力，以及单物理通道的共同位速率、ID、终端和负载证据；不通过时验收第二个隔离通道、改用合格 backend 或修改 profile。
6. **规划决定（2026-09-01 修订）**：先从 L07 建立**力控** golden/negative/boundary vectors，再建立**伺服扩展帧** vectors；跨 profile 帧必须被拒绝。**每条 golden vector 必须按文档化范围反算验证，不得照抄手册示例**——L07 §4.4.1 力控速度环示例本身有一个十六进制位的笔误。AK V3.2 与第三方实现只作交叉比较。
7. **有依据的推断**：先实现/验证配置 schema、capability、纯 codec、fake/vcan 与注入式 HighTorque CDC framing，不连接真实命令路径。
8. **有依据的推断**：完成单总线 BusRuntime、错误帧、统计、最新命令槽和模拟设备；用压力/故障测试验证边界。
9. **待确认项**：通过 G1 后由成员 B 在 Week 2 逐台只读接入 HI12，随后双设备运行 30 min；不在同一步同时改 ID、位速率和输出 profile。
10. **有依据的推断**：建立复合 SystemInterface 和 mock 电机，完成生命周期、命令租约及 STRICT 切换 100 次。
11. **待确认项**：通过 G2 后由项目负责人和成员 B 在 Week 3 接入真实电机反馈；记录准确缩放、方向、温度、故障和 command watchdog。
12. **待确认项**：G3 书面通过后，由项目负责人执行、A/B 协作，在 Week 3 按独立 SOP 从零逐级执行受限力矩标定；任何指标异常立即断能并归档。
13. **规划决定**：在当前两电机 500 Hz nominal/stress 下完成周期、command-to-wire、CAN 负载和 freshness 测量，再比较 1 kHz controller_manager 候选；1 kHz 电机 I/O 和 RT 内核均由测量触发。
14. **有依据的推断**：Week 4 做一电机+两 IMU、故障矩阵、8 h 长稳和 clean rebuild；不临时加入 STM32 或神经网络控制。
15. **待确认项**：项目负责人在 Week 4 末核对全部发布指标与证据；至少两名非作者评审通过后才创建 `v0.1.0-mvp`，否则发布 RC/缺陷清单，不降低事实标准。
