# AI 协作契约

本文件适用于仓库根目录及全部子目录。所有 AI 编码对话在执行非平凡任务前必须先遵循本文件；更细的统一流程见 `docs/development/ai_collaboration_workflow.md`。

## 1. 权威顺序

发生冲突时按以下顺序处理，并主动修正过期的本地 memory 或 handoff：

1. 用户最新指令；
2. 当前代码、配置和可复现测试结果；
3. Git branch、HEAD、status 和 diff；
4. 已批准的 ADR、接口文档和当前规划；
5. 当前开发者本地的 `memory/MEMORY.md`；
6. 当前开发者本地的 `memory/STATE.md`；
7. 当前开发者本地的 `memory/PLAN.md`；
8. 经验证但可能过期的 handoff；
9. 对话记忆或推测。

不得为了让代码符合 memory/handoff 而覆盖当前工作区事实。

## 2. 必须使用的 Skills

- 对任何非平凡新项目任务（包括代码、配置、测试、文档、架构、计划或项目状态变更，以及形成项目结论的调查、决策和验证），必须使用 `project-memory`。开始时恢复当前开发者的本地上下文；每个新项目任务结束或暂停前，AI 必须自动更新并验证本地 `memory/STATE.md` 与 `memory/PLAN.md`，只有新增长期确认事实时才更新本地 `memory/MEMORY.md`。`memory/` 是每个开发者自己的恢复工具，必须由 `.gitignore` 排除，不得提交或作为共享项目事实源。
- `write-codex-handoff` 不是每次对话都使用。只有用户明确要求写/创建交接文档，或出现跨人员/机器/阶段、长时间暂停、上下文风险或未完成高风险工作等真实交接事件时，才允许 CREATE；普通任务完成、普通对话结束或已有 Memory 检查点不得自动创建 handoff。
- 从 handoff 恢复时，必须先用 `write-codex-handoff` 的 `RESUME` 流程验证结构和时效，再与当前 Git、文件和测试核对。
- 创建 handoff 后必须验证；已宣布为最终交接的文件不可继续改写，后续变化创建新的 successor handoff。
- 两个 skill 的唯一批准来源、已核验提交和用途记录在 `manifests/ai_skills.yaml`。项目负责人已授权在缺失时从其中列出的仓库安装；不得静默替换为 fork 或其他同名来源。
- 新增或扩展 CAN 电机适配时，使用仓库维护的 `skills/mech-can-motor-adapter/SKILL.md`；它整理扩展层选择、协议证据与验证流程，不授予额外实机操作权限。其源文件随本仓库 PR 评审，不替代以上两个上下文 skill。
- 若当前 AI 环境缺少 skill，先读取 manifest，并在当前环境权限允许时安装对应仓库。优先使用已核验提交；若只能安装移动中的 `main`，必须核对实际 HEAD 并报告与 manifest 的差异，不得静默更新基线。
- 若网络、权限或 AI 产品不支持安装，必须明确报告缺失，随后按统一流程手工执行等价步骤；不得声称已运行不存在的 skill 或 validator，也不得另建第四种项目记忆格式。

## 3. 每次非平凡任务的启动步骤

1. 定位最近的 Git root；尚未初始化 Git 时使用当前工作区根。
2. 使用 `project-memory`，初始化或依次读取本地 `memory/MEMORY.md`、`memory/STATE.md`、`memory/PLAN.md`；新 clone 没有这些文件是正常状态。
3. 检查实际 Git root、branch、HEAD、status；必要时查看 diff 和 `STATE.md` 指向的文件。
4. 阅读 `docs/planning/README.md`、当前里程碑文档以及任务涉及的 ADR/接口文档。
5. 仅在存在明确恢复事件时读取并验证指定 handoff；不要默认把目录中最新文件当作当前事实。
6. 从 GitHub Issues/Milestones 或当前里程碑文档选择共享任务；本地 `STATE.md` 的 `Immediate Next Action` 与 `PLAN.md` 只用于个人恢复。若用户另有最新指令，以用户指令为准并同步相应共享/本地状态。
7. 向用户简要报告已恢复的目标、当前状态、风险和本轮动作后再编辑文件。

## 4. 执行和验证规则

- 一次对话默认只推进一个可验收任务或一个紧密相关的任务组，使用现有 Issue/里程碑 ID。
- 先检查现有实现、文档和用户改动；不得静默回退、覆盖或清理不属于本任务的修改。
- 只记录实际运行的命令和实际观察的结果；未运行的测试必须标记原因。
- 代码、配置、测试和 ADR 是事实源；memory 和 handoff 只保存索引、结论和恢复信息，不复制日志或大段代码。
- 供应商参数、硬件状态和远程运行状态未经本轮重新验证均视为未知或过期。
- 未获得当前任务的明确授权且未满足相应 G0-G3 闸门时，不启用 CAN、不发送电机命令、不修改 Jetson 系统或真实设备配置。
- 不把 token、密码、cookie、私钥、`.env` 值、带凭据 URL、个人数据或完整 shell history 写入仓库、memory 或 handoff。

## 5. 结束和检查点

每个新项目任务（包括有实际修改的任务、产生项目状态变化的调查/决策任务，以及未完成而暂停的任务）结束或暂停前必须：

1. 完成与风险相称的测试或检查；
2. 自动更新本地 `memory/STATE.md` 和 `memory/PLAN.md`；共享进度由 GitHub Issues/Milestones 和正式文档维护；
3. 仅在出现新的、已确认且长期有效的事实或决策时更新 `memory/MEMORY.md`；
4. 运行 project-memory validator，修复全部 error，并审查 warning；
5. 说明修改内容、验证结果、未解决风险和唯一具体下一步；
6. 只有满足 handoff 触发条件时才创建并验证 handoff；没有触发条件时明确不创建。

简单问答、无新发现且不形成项目任务的只读检查，不要为了形式更新 memory 或创建 handoff；一旦被定义为新的项目任务，即使没有代码修改，也要按上述规则更新任务状态并验证 memory。

## 6. 项目专用约束

- 当前实施入口是 `docs/planning/README.md` 及其活动任务/设计/操作文档。Foundation 已完成，`docs/planning/07_framework_bootstrap_plan.md` 为实施记录；AK3.0 力控与 SERVO-001 伺服已实现。HI12 和完整安装初始化仍按规划推进，不能把已完成的软件/台架验证当作新的设备操作授权。
- 历史里程碑要求（已完成）：FND-004 完成后、FND-005 开始前执行 FND-004A Jetson ARM64 原生烟测；仅做 clean clone、依赖解析、context check 和五包 build/test，不启用 CAN、不操作真实设备。
- 历史里程碑与持续分支规则：FND-004A 通过后，给实际通过烟测的精确 commit 创建 annotated tag `fnd-004a-passed` 并保护 `main`；FND-005 起所有修改（包括项目负责人自己的修改）均通过短生命周期任务分支和 PR 合并，外部成员可使用 fork。烟测后若有修复，必须在新 commit 上重跑完整 FND-004A 后再打 tag。
- `CubeMars/` 是独立供应商资料仓库，不得进入主仓库。
- Windows 只作为编辑/Git 环境；ROS 2 Humble、vcan、性能和 ARM64 结论必须来自对应 Ubuntu/ARM64 环境。
- 控制器、ros2_control hardware plugin 和厂商 codec/session 的边界不得为单个设备品牌而破坏。


## 当前工程命名

正式名称为 PH-EXO（并联外骨骼控制系统），ROS 标识采用 ph_exo，公共状态话题为 /ph_exo/state。REF-01 是机构模型的历史标识。新建工程级接口和文档沿用 PH-EXO，模型资料按原编号追溯。仓库及部署路径见 docs/parallel_exo/project_management.md。

## 2026-11-05 并联外骨骼统一项目要求

用户明确要求将现有ROS、IMU、MuJoCo与相关部分整合成同一可迁移工程。先读
`docs/parallel_exo/project_management.md`与`docs/parallel_exo/restore.md`。
本分支以整个项目的持续开发与设备迁移为目标，不默认拆成上游贡献任务。
正式源码在本仓库维护，缓存与机器环境放`.local/`，必要模型及测试输入纳入Git。
用户需要时整体提交、推送和发起PR；不自行安装自动提交/推送，不自动创建PR。
旧本地目录是迁入前来源；不得继续在那里修改后声称已经同步本仓库。
Windows/Linux角色分工仍有效；跨平台源码交接记录提交号，测试结论注明实际主机。
当前Orin部署不随Git切换；本次整合不授权电机或传感器操作。

本控制仓库不纳入CAD/MATLAB源工程、建模工具、Viewer或历史报告；仅保留运行所需模型、网格、接口和数值验证输入。


## 2026-11-06 本地源码与Orin开发副本

按用户要求，本地为唯一正式编辑入口；Orin开发目录为
`/home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control`。
先读`docs/parallel_exo/project_management.md`第7节与`local_orin.md`，使用本地
`tools/project/Sync-Orin.ps1`同步、核对和发起远端构建/测试。同步可包含未提交内容。
发现远端独立改动先比对回本地；不强制覆盖，不将源码同步扩大为旧release更新或硬件操作。
`.local`、日志、编译结果和机器配置留在各端；两端源码清单应一致。Git提交/推送/PR仍按用户需要进行。
