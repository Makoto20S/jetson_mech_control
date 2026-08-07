# Contributing

本仓库面向多人协作。共享项目事实必须可由任意 clean clone 复现，个人机器和 AI 会话状态不得成为其他成员的隐式输入。

## 开始工作

1. 阅读根 [`AGENTS.md`](AGENTS.md) 和 [`docs/development/ai_collaboration_workflow.md`](docs/development/ai_collaboration_workflow.md)。
2. 从 GitHub 当前 Milestone/Issue 选择或认领一个有 ID、依赖和验收标准的任务；当前 Foundation 顺序见 [`docs/planning/07_framework_bootstrap_plan.md`](docs/planning/07_framework_bootstrap_plan.md)。
3. 支持 Codex skills 的环境按 [`manifests/ai_skills.yaml`](manifests/ai_skills.yaml) 检查或安装 `project-memory` 与 `write-codex-handoff`。
4. 使用 `project-memory` 初始化或恢复本地 `memory/`。该目录被 Git 忽略，不从他人的 Memory 恢复项目事实。
5. 核对 branch、HEAD、status、任务相关 ADR/接口和当前测试结果后再编辑。

## 信息边界

| 载体 | 内容 | 是否进入 Git |
|---|---|---|
| README、`AGENTS.md`、本文件 | 稳定入口、协作与安全规则 | 是 |
| ADR、接口、构建、部署、安全文档 | 长期项目事实和已批准决策 | 是 |
| manifest、CI、测试和配置 | 可执行、可复现的项目基线 | 是 |
| GitHub Issues/Milestones/PR | 共享 backlog、负责人、依赖、进度和验收证据 | GitHub 托管 |
| `memory/MEMORY.md`、`STATE.md`、`PLAN.md` | 当前开发者的 AI/会话恢复上下文 | 否，本地忽略 |
| `.codex/`、本机路径和个人笔记 | 个人工具与会话状态 | 否，本地忽略 |

长期有效的新结论不能只留在本地 Memory 或 Issue 评论中，应进入相应 ADR 或正式项目文档。README 不记录频繁变化的 commit ID、单次 CI run ID 或个人下一动作。

## Issue 与 Pull Request

- 一个 Issue 默认对应一个可验收任务或紧密相关的任务组；使用现有 FND/RSP/INT 等 ID。
- Issue 至少写明目的、范围、非目标、依赖、完成标准、验证方法和硬件安全边界。
- Pull Request 关联 Issue，说明实际修改、实际运行的验证、未运行项及原因、文档/ADR 影响和剩余风险。
- FND-004A 通过并创建 `fnd-004a-passed` annotated tag 后，`main` 进入保护状态；FND-005 起包括仓库所有者在内不得直接推送 `main`。
- 仓库所有者从最新 `main` 创建同仓短生命周期任务分支（例如 `fnd-005/frame-types`）；外部成员可从 fork 提交。两种方式都必须通过 PR 和 required checks。
- FND-004A 测试的是精确 commit；烟测后的任何修复都要求在新 commit 上重跑完整烟测，tag 不得指向未实际通过的提交。
- 不用共享 `PLAN.md` 代替 Issue 状态；个人 `PLAN.md` 只帮助本地恢复。
- 未满足 G0–G3 且未获得明确授权时，不启用 CAN、不发送电机命令、不修改 Jetson 系统或真实设备配置。

## 文档生命周期

FND-004 已把核心结论迁入[独立 ADR](docs/adr/README.md)，随后完成活动文档收敛：证据链保留，架构/决策概览压缩，初始规划提示词移入[非规范归档](docs/archive/README.md)。ADR-006 因缺少当前单 `can0` 的实机配置和总线证据保持 Proposed；不得把该状态解释为允许激活。归档材料不得继续作为当前规范引用。

FND-004 后、FND-005 前执行 FND-004A Jetson ARM64 原生烟测，步骤见 [`docs/development/jetson_arm64_smoke_test.md`](docs/development/jetson_arm64_smoke_test.md)。该烟测不启用 CAN，也不操作真实设备。
