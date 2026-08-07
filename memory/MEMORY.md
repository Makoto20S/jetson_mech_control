# Project Memory

> Last reviewed: 2026-08-07T13:20:36+08:00

## Project Overview

- 目标是为 NVIDIA Jetson 建立可扩展的通用机电控制框架，规划包络为最多 6 台 CAN 电机、2 条 CAN/CAN FD 总线、2 台 HI12，以及未来的 STM32 传感器节点。
- 当前首批硬件是两台基于 AKE60-8、结构定制并加装 CubeMars 双编码器的驱控一体电机和两台 HI12。结构改造不属于软件任务；两台电机的实际驱动板、固件、配置和编码器 CAN 来源仍需逐台验证。
- 当前权威规划入口是 `docs/planning/README.md`；当前实施顺序在 `docs/planning/07_framework_bootstrap_plan.md`，决策/未知项在 `05`，CubeMars 证据边界在 `06`。

## Architecture

- 采用“纯 C++ 传输/协议/时间/状态/设备核心 + 薄 ros2_control 复合 `SystemInterface`”。ROS 适配层只导出生命周期、标准状态接口和命令接口，不承载厂商 CAN 位域或阻塞式总线收发。
- 每条物理 CAN 总线由单一进程内 `BusRuntime` 协调接收、路由、发送调度、时间戳、错误状态、诊断和命令租约。控制器不得直接打开 SocketCAN 或构造厂商帧。
- CubeMars 协议至少区分 `AK V3 servo extended`、`AK V3 force-control extended` 与 `legacy MIT standard-frame`；设备在 configure 阶段绑定协议代际和 active command profile，ACTIVE 期间不自动探测或混发。
- 接入新 CAN 电机或传感器时，优先新增 codec、device session、capability、配置、标准单位映射和测试。标准接口语义一致时，现有 C++ 控制器和 ros2_control 层应保持不变。

## Confirmed Decisions

### Repository and asset policy (FND-000)

- Decision: The project owner confirmed on 2026-08-06 that the recommended D1–D5 policy is accepted: private GitHub target `Makoto20S/jetson_mech_control` with a later NAS mirror; internal research-use/all-rights-reserved notice; no raw supplier archives, binaries, large logs, or rosbag in the main repository; track the three project-memory files without secrets; and enable branch protection/review/CODEOWNERS after the first runnable CI. The owner authorized creation, baseline commit, and push; the private remote now exists and `main` is synchronized.
- Reason: Freeze ownership, redistribution, asset, continuity, and review boundaries before the first Git baseline.
- Evidence: `docs/planning/fnd-000_repository_and_asset_policy.md`; explicit user confirmations on 2026-08-06; commits `057ad8d` and `69815f6`; authenticated GitHub API and `git ls-remote`; clean clone verification; root inventory of the original empty `.git/`, nested `CubeMars/`, and temporary directories.
- Date: 2026-08-06.
- Scope: FND-001 is complete. NAS target and reviewer identities remain unknown and must not be guessed; branch protection waits for the first runnable CI as specified by D5.

### Foundation-first implementation

- Decision: 在没有实机配置的情况下先完成 `Foundation v0.1`：Git/CI、纯 C++ core、fake/vcan、BusRuntime、reference simulator、薄 ros2_control `SystemInterface` 和模拟 demo controller；由项目负责人统一搭建，AdapterContract v1 冻结后再分工接入 CubeMars、HI12 和 HIL。
- Reason: 实机资料只决定真实 profile、缩放、编码器来源、watchdog 和物理验收，不应阻塞可脱离硬件验证的架构与接口。
- Evidence: `docs/planning/07_framework_bootstrap_plan.md`, `docs/planning/05_decisions_and_open_questions.md` section 3.7.
- Date: 2026-08-03.
- Scope: Foundation 不连接或控制真实设备；设备适配包在核心契约稳定后创建。

### Reproducible Foundation build boundary (FND-002/FND-003)

- Decision: Foundation 构建基线固定为 Ubuntu 22.04、ROS 2 Humble、C++17 和 digest-pinned `ros:humble-ros-base-jammy`；仓库与 CI 默认使用官方 `rosdep`，已配置镜像的主机可通过 `ROSDEP_COMMAND=rosdepc` 显式选择兼容 wrapper，但不得静默改变 CI 标准。WSL 源码可位于 Windows 挂载路径，生成物必须通过 `MECH_OUTPUT_ROOT` 放在 Linux 文件系统以避开 DrvFS I/O 阻塞。
- Reason: 同时保持上游可移植标准、国内镜像可用性与 WSL 构建稳定性，并让 CI 和开发机复用同一 build/test 脚本。
- Evidence: `manifests/dependencies.json`; `docker/ros_humble_jammy/Dockerfile`; `tools/ci/build_workspace.sh`; `ros2_ws/README.md`; 2026-08-07 在 `Ubuntu-22.04` 中用官方 `rosdep` 和 `ROSDEP_COMMAND=rosdepc` 各完成一次五包构建及 30/30 测试；提交 `ee4c64c` 的 GitHub Actions run `31150054330` 成功完成 portable context check 与 pinned Humble Docker build。
- Date: 2026-08-07.
- Scope: 已有 x86_64 Ubuntu/Humble 原生与 amd64 GitHub Actions/Docker build/unit 证据；不代表 ARM64、vcan、性能或真实硬件已验证。

### Current control rates

- Decision: 当前两电机正常 MVP 的控制循环及电机命令/反馈目标为 500 Hz；100~200 Hz 仅用于模拟或首次硬件 bring-up；代码与配置必须支持 1 kHz 控制循环测试。
- Reason: 两台 AK V3 基础反馈加两台 HI12 的保守单总线预算约 41.6%；两电机都启用 `0x2A` @500 Hz 时约 53.6%，1 kHz 电机 I/O 仍需真实总线和 Jetson 时序测量。
- Evidence: `docs/planning/README.md`, `docs/planning/05_decisions_and_open_questions.md`, `docs/planning/06_cubemars_material_review.md`.
- Date: 2026-07-30.
- Scope: 当前两台电机，不是六电机单总线的通用默认值。

### CubeMars evidence baseline

- Decision: 当前 AKE60-8 软件事实源以 AK3.0 V3.2 手册为主；AK2.0 V1.0.18 和标准帧 Arduino MIT demo 仅作为 legacy 对照。
- Reason: V3.2 明确列出 AKE60-8、AK54、扩展帧 servo/force-control、`0x29/0x2A`、1–2000 Hz 反馈和标准 Kt/范围，而旧 demo 的帧类型及 payload 顺序与之冲突。
- Evidence: `docs/planning/04_source_register.md`, `docs/planning/06_cubemars_material_review.md`.
- Date: 2026-08-03.
- Scope: 标准 AKE60-8 参数是定制实机的候选，不得替代每台 CubeMarsTool 配置导出。

### Minimal constant-command demo

- Decision: 恒定力矩/恒定命令只是验证完整控制链路的最小 demo，不是最终控制目标；`2 N*m` 只是解释示例，不是默认配置或固定验收值。
- Reason: demo 只需证明 C++ controller、接口 claim、`SystemInterface::write()`、device session、`BusRuntime`、CAN、状态与诊断可正确贯通。
- Evidence: `docs/planning/05_decisions_and_open_questions.md` section 3.4.
- Date: 2026-07-30.
- Scope: demo 目标必须可配置、有界、从零按 slew 进入并可回零；物理输出力矩精度另行验收。

### Controller and drive responsibilities

- Decision: 驱动器负责 FOC、高速电流环以及所选模式下的内部位置/速度环；Jetson 上的 C++ 控制器读取电机和 IMU 状态，计算目标位置、速度或力矩。
- Reason: 将高频电机底层闭环留在驱动器，同时保留关节级控制算法的可替换性。
- Evidence: `docs/planning/05_decisions_and_open_questions.md` section 3.3.
- Date: 2026-07-30.
- Scope: Python 不直接拥有电机命令接口。

### AI collaboration continuity

- Decision: 根 `AGENTS.md` 是所有 AI 对话的强制入口；任何非平凡新项目任务（包括形成项目结论的调查、决策和验证，即使没有代码修改）结束或暂停前，AI 必须自动用 `project-memory` 更新并验证 `STATE.md` 与 `PLAN.md`，仅在出现新的长期确认事实时更新 `MEMORY.md`。`write-codex-handoff` 是事件触发工具：用户明确要求写交接时使用；跨人员/机器/阶段、长暂停、上下文风险或未完成高风险工作等真实交接事件也可由 AI 主动 CREATE，但普通任务完成、普通对话结束或已有 Memory 检查点不得自动创建。工具无关 SOP 位于 `docs/development/ai_collaboration_workflow.md`。
- Reason: Memory 是每项项目工作的持续状态来源，handoff 是低频、不可变的转交快照；两者职责分离可避免漏记任务状态，也避免把每轮进度制造成过期交接文档。
- Evidence: User-confirmed rule on 2026-08-06; `AGENTS.md`; `docs/development/ai_collaboration_workflow.md`; `docs/planning/03_mvp_delivery_plan.md` section 11.
- Date: 2026-08-06.
- Scope: 当前不创建重复这两个通用 skill 的 continuity skill；项目专用 skill 只有在人工 SOP 至少成功执行三次且输入、输出、禁区和验证稳定后创建。

### Trusted AI skill sources

- Decision: `project-memory` 的批准来源是 `https://github.com/Makoto20S/project-memory`，`write-codex-handoff` 的批准来源是 `https://github.com/Makoto20S/codex-handoff-skill`；项目负责人授权在缺失时从这两个准确仓库安装。当前已核验提交记录在 `manifests/ai_skills.yaml`。
- Reason: 跨机器使用必须有可审查的来源，不能依赖搜索结果、同名 fork 或个人绝对路径。
- Evidence: User-provided repositories; `git ls-remote --symref ... HEAD` on 2026-08-03 resolved both to `main`; no tags were observed.
- Date: 2026-08-03.
- Scope: 已核验提交是审查基线，不代表移动中的 `main` 永久不变，也不证明当前本机安装副本与远端提交完全一致；升级需核对实际 HEAD 并更新 manifest。

## Conventions

- 文档结论明确标记为“已确认事实/资料事实”“规划决定/有依据的推断”“用户提供（待独立确认）”或“待确认项”。
- 标准 AKE60-8 资料已给出输出端 `T = Kt * Iq` 和 `Kt = 0.7382 N*m/A` 候选；只有定制实机配置证明该映射仍成立时才锁定标准 `joint/effort`，否则使用明确的电流接口。
- 厂商专用原始字段放入独立状态或诊断接口，不用相似名称伪装成标准 SI 接口。
- 多速率数据必须保留源时间和递增 age；重复读取旧快照时不得把时间戳改成 `now()`。

## Business Rules

- 命令必须经过 finite、限幅、slew、模式、状态新鲜度和租约/TTL 检查；控制器停止刷新后，旧命令必须失效。
- 配置期固定设备协议代际和 active command profile；ACTIVE 期间禁止自动识别或混发 AK V3 servo/force、legacy MIT。
- 最小 demo 通过只证明框架链路贯通，不代表最终控制算法完成，也不代表物理力矩精度合格。

## Known Pitfalls

- CubeMars 通用资料给出 21-bit 内环和可选 15-bit 外环单圈绝对编码器，但没有说明 CAN `0x29/0x2A` 使用哪个来源或能否同时上传两者；不能先选 `joint/position` 来源，也不能把角差直接当作力矩。
- AK V3.2 的 1–2000 Hz 是反馈配置范围，不是控制循环或实机稳定性证明；反馈设置会写 Flash，不能在周期路径反复发送。
- 两台 HI12 若保持相同默认节点地址会冲突；J1939 与 CANopen 取决于交付固件，不能按帧外观猜测。
- 控制循环、CAN 命令、设备反馈和 IMU 输出是独立频率；高频控制循环不能把重复快照伪装成新反馈。
- 六电机单总线在 500 Hz 的保守经典 CAN 预算接近饱和，因此六电机扩展需使用独立频率预算或分总线。

## Reliable Commands

- 项目 memory 验证：`D:\Work\anaconda\python.exe C:\Users\sinfi\.codex-happysolve-cli\skills\project-memory\scripts\validate_memory.py --root D:\Work\jetson`。
- 当前系统的裸 `python` 命令解析到不可用的 WindowsApps 别名；本项目 memory 脚本使用上述 Anaconda Python。
- Ubuntu 22.04/Humble 默认构建测试：`MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build bash tools/ci/build_workspace.sh`；2026-08-07 以官方 `rosdep` 验证五包和 30/30 测试通过。
- 已配置 rosdepc 镜像的主机可先运行 `rosdepc update --rosdistro humble`，再用 `ROSDEP_COMMAND=rosdepc MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build bash tools/ci/build_workspace.sh`；2026-08-07 验证通过。
- 当前 WSL 必须显式选择 `Ubuntu-22.04`；名为 `Ubuntu` 的另一个发行版未安装 ROS。不要把 `build/install/log` 写到 `/mnt/d`。

## Long-Term Constraints

- 当前现场计划共用单个 `can0`；只有在设备位速率一致、ID 无冲突且负载实测通过后才可激活该拓扑。
- 框架保留未来双总线和多品牌 CAN 设备扩展能力，但不得用软件队列掩盖物理总线带宽不足。
- 真实硬件参数必须来自匹配固件的供应商资料、配置导出、抓包或测量，不得从相似型号补默认值。
- Windows 工作区只作为编辑/Git 环境；ROS 2 Humble build、vcan 和性能证据必须来自 Ubuntu 22.04/ARM64 对应环境。
- 主仓库不得误提交独立 `CubeMars/` 仓库、供应商二进制资料、临时提取文件或构建/实验大数据。
