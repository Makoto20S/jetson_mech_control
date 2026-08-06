# Current Project State

> Last updated: 2026-08-06T20:06:00+08:00
> Repository: D:\Work\jetson
> Branch: main
> HEAD: `main`; FND-001 remote baseline verified at `69815f6`, final status checkpoint pending
> Working tree: FND-001 status/documentation checkpoint prepared; remote baseline synchronized before this checkpoint

## Current Objective

- 完成 FND-001 Git/规划基线：创建私有远端、推送 `main`，并验证可复现的 clean clone 边界；下一步等待确认后进入 FND-002/FND-003。

## Current Status

- 项目仍处于规划阶段：未启用 CAN、未控制真实设备、未修改 Jetson，也未创建实现代码。
- `docs/planning/07_framework_bootstrap_plan.md` 已成为当前实施入口，包含四周路线、Issue 依赖、复用计划和 Foundation Definition of Done。
- 根 `AGENTS.md` 和 `docs/development/ai_collaboration_workflow.md` 已更新 AI 连续性标准：每个新项目任务结束/暂停前由 AI 自动更新并验证 `STATE.md`/`PLAN.md`；handoff 只在用户明确要求或真实转交/阶段/暂停/风险事件触发时创建。
- `manifests/ai_skills.yaml` 已记录两个批准仓库、当前核验提交和缺失安装策略，并已进入首次 Git 基线。
- `docs/planning/fnd-000_repository_and_asset_policy.md` 已标记 Accepted；项目负责人于 2026-08-06 确认 D1–D5、GitHub 目标 `Makoto20S/jetson_mech_control`，并授权创建仓库、commit 及推送。
- 本地 Git 已用 `git init -b main` 初始化；21 个规划/治理/Memory/资产边界文件已提交，`main` 已推送，远端基线曾与本地 `69815f6` 一致；本轮最终状态检查点将在验证后追加并推送。仓库级 Git 作者身份及 `origin` 已配置，邮箱值未写入项目文件。
- GitHub API 确认仓库为 private、默认分支为 `main`；分支保护当前为 false，符合 D5 在首个可运行 CI 前暂不启用的政策。

## Completed Recently

- 只读核验用户提供的两个 skill 仓库，确认默认分支均为 `main`、当前 HEAD 分别为 `ab9d08c...` 与 `6992a97...`，且未观察到 tag；新增可审查的 skill manifest。
- 更新 `AGENTS.md` 和 AI SOP：缺失时允许从准确批准仓库安装，优先核验提交，移动分支发生漂移时必须报告和审查。
- 建立根 `AGENTS.md` 和工具无关的 AI 协作 SOP，统一权威顺序、启动/执行/验证/检查点、handoff 生命周期、跨机器 fallback 和新 skill 创建门槛。
- 对齐 `README.md`、`03_mvp_delivery_plan.md` 与 `07_framework_bootstrap_plan.md`，取消“每次新对话默认读取最新 handoff”，改为日常从 project memory 恢复、只恢复明确指定且验证通过的 handoff。
- 将 `.codex/handoffs/2026-08-03_200534_jetson-control-framework-foundation-v0-1-bootstrap.md` 从占位模板补充为证据分级的可恢复交接，并通过 handoff validator。
- 新建 `docs/planning/07_framework_bootstrap_plan.md`，明确 Foundation-first、模块依赖、Git 资产边界、构建环境、FND-000~015、复用 spike、验收和新对话恢复规则。
- 将 `README.md`、`03_mvp_delivery_plan.md` 和 `05_decisions_and_open_questions.md` 对齐到 Foundation v0.1 当前里程碑。
- 初始化前只读确认根 `.git/` 是空目录；本轮已将其初始化为 `main`。`CubeMars/` 仍是必须忽略的独立嵌套仓库。
- 根据用户 2026-08-06 的补充，修订 handoff 与 project-memory 规则：普通任务不自动创建 handoff；每个新项目任务必须自动保存 Memory 检查点。
- 启动并完成 FND-000 盘点与决策确认；更新政策文档为 Accepted，创建 `.gitignore`、`.gitattributes`、`LICENSE-or-INTERNAL-LICENSE.md`、`manifests/assets.yaml` 和根 `README.md`；初始化本地 `main` 并完成暂存审查。
- 创建首次 Foundation 基线提交 `057ad8d`（`chore: establish foundation repository baseline`）；提交包含 21 个批准路径和 3605 行新增内容，未包含任何禁止路径或秘密扫描匹配。

## In Progress

- 当前规划文档已完成，尚无代码实现进行中。
- FND-000 已完成；FND-001 本地初始化、暂存审查、作者身份、远端目标配置、首次提交、推送和 clean clone 验证均完成。
- AI 治理规范和来源 manifest 已进入 GitHub 基线；可移植 installer/CI 检查仍留待 FND-003。
- 下一阶段为 FND-002/FND-003；未收到进入下一任务的确认前，不创建实现代码或安装依赖。
- 实机配置和 HI12 身份取证已移到 Foundation 后的集成工作，不再阻塞当前阶段。

## Modified Files

- `manifests/ai_skills.yaml`
  - Change: 记录两个批准 skill 仓库、默认分支、已核验提交、用途和安装/升级策略。
  - Reason: 让新机器和新 AI 不依赖搜索结果或个人路径获取关键 skills。
  - Status: Committed in the initial `main` baseline `057ad8d`.
  - Validation: YAML structured parse passed; remote HEAD and tags were checked with `git ls-remote`.
- `AGENTS.md`, `docs/development/ai_collaboration_workflow.md`
  - Change: 新增仓库级 AI 强制入口和详细的跨对话/跨机器连续性 SOP。
  - Reason: 让不同人员和 AI 使用同一恢复、证据、memory 与 handoff 规则。
  - Status: Complete for planning; portable skill manifest/CI enforcement deferred to FND-001/FND-003.
  - Validation: 本地 Markdown 链接和表格结构检查通过。
- `AGENTS.md`, `docs/development/ai_collaboration_workflow.md`, `docs/planning/03_mvp_delivery_plan.md`, `docs/planning/07_framework_bootstrap_plan.md`, `manifests/ai_skills.yaml`
  - Change: 明确 project-memory 在每个新项目任务结束/暂停前由 AI 自动更新并验证；明确 handoff 仅由用户要求或真实交接事件触发，普通任务不自动 CREATE。
  - Reason: 落实项目负责人 2026-08-06 的协作约束，避免 Memory 漏记和 handoff 日志化。
  - Status: Updated; user confirmed D1–D5 and authorized local FND-001 preparation.
  - Validation: 交叉文本搜索未发现冲突规则；YAML 结构解析通过；memory validator returned errors=0 and warnings=0.
- `docs/planning/fnd-000_repository_and_asset_policy.md`, `docs/planning/README.md`, `docs/planning/07_framework_bootstrap_plan.md`
  - Change: 新建 FND-000 决策草案并接入规划导航和当前实施计划。
  - Reason: 在 Git 初始化前明确五项仓库/资产政策及负责人确认边界。
  - Status: Accepted; D1–D5 confirmed by project owner on 2026-08-06.
  - Validation: D1–D5 status/confirmation record, links, and Markdown table shape reviewed; `validate_memory.py` passed with errors=0 warnings=0.
- `.gitignore`, `LICENSE-or-INTERNAL-LICENSE.md`, `manifests/assets.yaml`, `README.md`
  - Change: 建立主仓库资产边界、内部科研使用声明、已观测排除资产哈希清单和恢复入口。
  - Reason: 为 FND-001 的显式暂存审查提供可复现基线，避免误纳入 `CubeMars/`、会话状态、供应商原始资料和生成物。
  - Status: Committed in the initial `main` baseline `057ad8d`.
  - Validation: 资产哈希、YAML、禁区路径、排除链接、秘密模式和 `git diff --cached --check` 检查通过。
- `docs/planning/README.md`, `03_mvp_delivery_plan.md`, `07_framework_bootstrap_plan.md`
  - Change: 增加 AI 标准导航，记录不新增重复 continuity skill 的决定，并统一 Foundation 恢复顺序。
  - Reason: 避免 handoff 被当作每轮日志或“最新文件”被误当当前事实。
  - Status: Reconciled.
  - Validation: 本地 Markdown 链接和表格结构检查通过。
- `.codex/handoffs/2026-08-03_200534_jetson-control-framework-foundation-v0-1-bootstrap.md`
  - Change: 补充目标、当前事实、未决项契约、恢复命令、关键文件和下一次对话动作。
  - Reason: 让后续对话无需依赖已压缩聊天即可从 FND-000 恢复。
  - Status: Complete.
  - Validation: `validate_handoff.py` passed；staleness 检查属于初始化前历史记录，曾因当时无 Git 元数据返回 `UNKNOWN`。
- `docs/planning/07_framework_bootstrap_plan.md`
  - Change: 新增当前可直接执行的 Foundation v0.1 详细计划。
  - Reason: 让项目在缺少实机资料时继续推进，并供新对话/成员恢复。
  - Status: Complete.
  - Validation: 本地链接存在，Markdown 表格结构检查通过。
- `docs/planning/README.md`, `03_mvp_delivery_plan.md`, `05_decisions_and_open_questions.md`
  - Change: 将当前实施里程碑改为 Foundation-first，并保留完整硬件 MVP 作为后续路线。
  - Reason: 反映用户最新执行顺序。
  - Status: Reconciled.
  - Validation: 本地链接和 Markdown 表格结构检查通过。
- `memory/MEMORY.md`, `memory/STATE.md`, `memory/PLAN.md`
  - Change: 保存 Foundation 决策、当前里程碑和下一步。
  - Reason: 支持跨对话和后续实现恢复。
  - Status: Checkpointed.
  - Validation: `validate_memory.py` passed with errors=0 and warnings=0.

## Validation Results

- Skill source verification at 2026-08-03T20:36:00+08:00: both approved GitHub repositories resolved to `main`; project-memory HEAD was `ab9d08cf9841f22feab278caeda115e4744b8635`, handoff HEAD was `6992a973b210c636e6a068a1aacc45dcadf0132e`, and neither returned tags.
- Skill manifest validation at 2026-08-03T20:36:00+08:00: YAML parsed successfully with schema version 1, the two expected names and 40-character verified commits.
- AI loading contract at 2026-08-03T20:38:00+08:00: SOP now distinguishes same-workspace Codex, cloned Codex, AGENTS-aware third-party AI and tools without AGENTS support; no cross-product automatic-load claim is made.
- AI workflow documentation at 2026-08-03T20:26:00+08:00: local Markdown links passed for five modified files; Markdown table shapes passed for four files.
- Handoff CHECK at 2026-08-03T20:26:00+08:00: existing Foundation handoff passed `validate_handoff.py`; staleness remains `UNKNOWN` because the workspace is not yet a Git repository.
- Handoff validation at 2026-08-03T20:11:00+08:00: `validate_handoff.py` passed for the Foundation handoff; `check_staleness.py` returned `UNKNOWN` only because Git metadata is unavailable before FND-001.
- Git inspection at 2026-08-03T15:39:00+08:00: `git rev-parse --show-toplevel` failed with “not a git repository”; current directory is therefore the project root under project-memory policy.
- Planning Markdown validation at 2026-08-03T19:59:00+08:00: local link targets exist and table pipe counts are consistent.
- Memory structural validation at 2026-08-03T20:04:00+08:00: `validate_memory.py` passed with errors=0 and warnings=0.
- Governance rule reconciliation at 2026-08-06T17:16:35+08:00: updated root contract, workflow, planning references and skill manifest; cross-document rule search and Markdown table smoke passed; `ai_skills.yaml` parsed successfully; `validate_memory.py` returned errors=0 and warnings=0. No implementation, Git initialization, hardware or Jetson action was performed.
- FND-000 inventory at 2026-08-06: pre-init root Git probes returned “not a git repository”; root `.git/` was empty; `CubeMars/` contains an independent `.git`, supplier directories and an archive. No remote or hardware configuration was changed.
- FND-001 local baseline at 2026-08-06: `git init -b main` succeeded; staged-path, forbidden-extension, excluded-link, secret-pattern, YAML and `git diff --cached --check` reviews passed; commits `057ad8d` and `69815f6` exist.
- FND-001 remote creation/push at 2026-08-06T20:04:00+08:00: authenticated GitHub API created private `Makoto20S/jetson_mech_control`; `git push -u origin main` succeeded; `git ls-remote` and API branch lookup both returned `69815f62d32811393a9bbf072dce82762348a2d3` for the verified baseline `main`/HEAD.
- FND-001 clean clone at 2026-08-06T20:04:00+08:00: a temporary single-branch clone of baseline `69815f6` had 13 required entry files, 21 tracked paths, no forbidden paths/extensions, clean status, and `validate_memory.py` errors=0 warnings=0; the temporary clone was removed after verification.
- FND-001 baseline commit at 2026-08-06T19:36:00+08:00: `git commit -m "chore: establish foundation repository baseline"` created root commit `057ad8d478a79f192c69caa5446b65e9fb0418c6`; `git log -1`, branch, remote and clean post-commit status were inspected. The commit contains the same 21 reviewed paths that passed forbidden-path, secret-pattern and whitespace checks; project build/tests remain not run because implementation code does not yet exist.
- Stage-transition handoff at 2026-08-06T19:11:45+08:00: created `.codex/handoffs/2026-08-06_191145_jetson-foundation-fnd-000-to-fnd-001-local-baseline.md`; `validate_handoff.py` passed and `check_staleness.py` reported `CURRENT`. This is an event-triggered handoff, not a routine task log.
- FND-000 decision audit at 2026-08-06T17:38:00+08:00: D1–D5 each have structural markers, explicit owner/status/recommendation, required links exist, Markdown table smoke passed, and `validate_memory.py` returned errors=0 warnings=0. Five existing PDFs were observed outside `CubeMars/`; none were deleted or moved.
- Project build/tests: not run because no implementation code exists.

## Current Problems

- NAS 目标位置和具体 CODEOWNERS 评审人仍未知；分支保护按 D5 等首个可运行 CI。
- 两个批准来源和核验提交已记录，但远端没有 tag，当前本机安装副本与核验提交是否完全一致尚未断言；可移植 installer/CI wrapper 仍未实现。
- The target Ubuntu/Humble build environment and CI commands have not yet been created or verified.
- Real-device configuration and single-`can0` coexistence remain unknown, but they no longer block Foundation.

## Blockers

- FND-000/FND-001 已完成；无远端阻塞。真实构建环境、CI 和设备集成仍按后续里程碑推进。
- Real motor/IMU integration remains blocked by device evidence and G0~G3, independently of Foundation.

## Unverified Assumptions

- GitHub target `Makoto20S/jetson_mech_control` exists and is private; branch protection remains intentionally disabled until runnable CI exists.
- D2 已确认内部科研使用、保留所有权利；机构归属和未来公开前审查仍是后续事项。
- D3–D5 已确认采用 `docs/planning/fnd-000_repository_and_asset_policy.md` 的推荐值。
- The two AKE60-8 custom motors and two HI12 devices are planned to share `can0`; compatibility remains unverified.
- The remote power relay has been previously validated by the project team; no independent evidence is recorded locally.
- The custom motors may use the AK3.0 V3.2 protocol and standard AKE60-8 parameters, but actual firmware/configuration has not been read.

## Failed Approaches

- Running the memory initialization script via bare `python` failed because the WindowsApps alias could not start. `D:\Work\anaconda\python.exe` succeeded and should be used for memory scripts in this environment.
- An unauthenticated/ordinary `git ls-remote` initially returned “Repository not found,” which was ambiguous for a private repository. Authenticated API verification later confirmed the account, repository creation, push, and matching remote HEAD.

## Immediate Next Action

- 在项目负责人确认后开始 FND-002/FND-003：固定 Ubuntu 22.04/Humble 构建环境、创建五包 workspace 和最小 CI；继续不启用 CAN 或真实硬件。
