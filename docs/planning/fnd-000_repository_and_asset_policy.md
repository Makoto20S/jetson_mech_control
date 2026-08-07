# FND-000：仓库与资产政策决策记录

> 状态：已确认（Accepted；D4 于 2026-08-07 修订）
> 创建日期：2026-08-06
> 确认日期：2026-08-06
> 最近修订：2026-08-07
> 任务范围：Foundation v0.1 开工前的仓库、许可证、供应商资产、Memory 和协作保护策略
> 证据基线：当前工作区只读盘点、项目负责人对 D1–D5 的明确确认，以及已确认的内部科研使用意图

## 决策摘要

FND-000 冻结第一次 Git 初始化前的边界。项目负责人已确认 D1–D5 均采用推荐值，因此这些政策作为 FND-001 的前置条件。项目负责人随后确认 GitHub owner 为 `Makoto20S`、目标私有仓库为 `jetson_mech_control`，授权创建本地基线提交、创建私有远端及推送；FND-001 已完成并通过远端 clean-clone 验证。

| 编号 | 决策项 | 推荐值 | 当前状态 | 负责人 |
|---|---|---|---|---|
| D1 | 主仓库名称与远端 | `jetson_mech_control`；私有 GitHub 主远端；NAS 作为后续镜像/备份 | 已确认（2026-08-06） | 项目负责人 |
| D2 | 代码许可证 | 内部科研专用、保留所有权利；暂不声明 Apache-2.0 等开源许可证；公开或商业化前另行审查 | 已确认（2026-08-06） | 项目负责人 + 所属单位按需审查 |
| D3 | 供应商/实验资产 | 主仓库不存供应商 PDF/压缩包/可执行文件/大日志/rosbag；`CubeMars/` 整体忽略；Git 仅保存来源、版本、哈希和受控 URI；小型自有测试向量逐项审查后可跟踪 | 已确认（2026-08-06） | 项目负责人 |
| D4 | `memory/` 边界 | `memory/` 仅在每个开发者本地由 `project-memory` 维护并由 Git 忽略；长期事实进入 ADR/正式文档，共享任务状态进入 GitHub Issues/Milestones | 已修订并确认（2026-08-07） | 项目负责人 |
| D5 | 分支保护与评审 | FND-001 基线建立后先保持 `main` 可审查；首个 CI workflow 可运行后启用 `main` 保护、PR review、CODEOWNERS 和 required checks；未经评审不直接推送受保护分支 | 已确认（2026-08-06） | 项目负责人 |

## 确认记录

- **用户确认（2026-08-06）：** 项目负责人明确表示 D1、D2、D3、D4、D5 全部采用推荐值，并授权开始执行 FND-000 及其后续本地 FND-001 基线准备。
- **D4 修订（2026-08-07）：** 项目负责人明确要求个人 Memory 不上传 GitHub，使任意成员的 clean clone 不受其他人的机器、AI 会话或个人计划影响；两项批准 skill 及根 `AGENTS.md` 约束继续保留。
- **Git 实施授权（2026-08-06）：** 项目负责人确认 GitHub owner 为 `Makoto20S`，目标路径为 `Makoto20S/jetson_mech_control`，并授权使用其提供的本地 Git 作者身份创建 commit 后推送。作者邮箱只保存在仓库级 Git 配置中，不写入项目文档或 Memory。
- **已完成的 Git 实施（2026-08-06）：** 在明确授权后创建了私有仓库 `Makoto20S/jetson_mech_control`，将本地 `main` 推送至远端，并确认远端 HEAD 为 `69815f62…`；从远端进行的 clean clone 通过必需文件、资产边界和 Memory 验证。
- **仍未决的实施信息：** NAS 目标位置和具体评审人身份未提供；不猜测 CODEOWNERS 内容。分支保护、PR review 和 required checks 按 D5 等首个可运行 CI 后启用。
- **本轮安全边界：** 仅配置本地 Git/远端并完成基线提交和推送；不安装依赖、不启用 CAN、不发送电机命令、不修改 Jetson。

## 证据与边界

### D1：仓库与远端

- **Repository evidence (2026-08-06):** `D:\Work\jetson` 已初始化为根 Git 仓库，当前分支为 `main`；21 个规划/治理/Memory/资产边界文件已提交，`CubeMars/` 等排除内容未进入索引。
- **Planning evidence:** `docs/planning/07_framework_bootstrap_plan.md` section 5 推荐根目录和临时名称 `jetson_mech_control`，并要求先决定远端再初始化。
- **Remote evidence (2026-08-06):** 本地 `origin` 配置为 `https://github.com/Makoto20S/jetson_mech_control.git`；认证 API 确认仓库为 private，默认分支为 `main`，远端 HEAD 与本地 `69815f62…` 一致；未输出或写入任何凭据值。
- **限制:** 分支保护和评审规则仍按 D5 延后至首个可运行 CI；不得把私有访问控制当作许可证。

### D2：内部科研许可证边界

- **User-reported (2026-08-06):** 代码供课题组科研使用，不以当前公开发布为目标。
- **Recommended policy:** 采用明确的内部声明，例如：`Internal research use only. All rights reserved. Redistribution or public release requires project-owner and institutional approval.`
- **重要限制:** 私有 GitHub 访问控制不等于许可证；学校、实验室或项目单位可能拥有著作权。若以后公开代码、随论文发布或商业转化，必须重新做机构和第三方许可证审查。
- **第三方边界:** 任何复制进主仓库的第三方代码仍须遵守其原许可证；没有许可证证据的代码只允许阅读，不复制实现。

### D3：供应商和实验资产

- **Repository evidence (2026-08-06):** `CubeMars/` 含独立 `.git`、供应商资料目录和 `CubeMars-logo.rar`；它不是主仓库内容。
- **Repository evidence (2026-08-06):** 根目录另有 `.codex/`、`.agents/` 和 `tmp/` 会话/临时内容，不应通过宽泛 `git add .` 纳入基线。
- **Recommended policy:** 主仓库跟踪代码、配置、文档、ADR、清单和经审查的小型 golden/test vectors；供应商原始资料、大二进制、实验数据和生成物存受控外部位置，仓库只保留来源、版本、SHA-256/其他哈希和不含凭据的 URI/索引。
- **许可证审查:** DBC、golden frame 和小型测试向量只有在来源、许可和再分发范围明确时才可跟踪。

### D4：本地 Project Memory 与共享项目状态

- **Superseded baseline:** 2026-08-06 曾决定在私有仓库跟踪三份 Memory；该做法会把个人机器、AI 会话和个人任务快照带给所有 clone，已由 2026-08-07 的最新决定替代。历史提交无需重写，但当前及后续版本不再跟踪 `memory/`。
- **Governance evidence:** `AGENTS.md` 继续强制两个批准 skill：每个开发者在非平凡任务前后使用本地 `project-memory`，handoff 仍按真实转交事件使用 `write-codex-handoff`。
- **Shared baseline:** 已确认架构和长期结论进入 ADR、接口、构建、安全等正式文档；共享 backlog、负责人和状态进入 GitHub Issues/Milestones。README 只给稳定入口和能力状态，不记录频繁变化的 commit/run ID。
- **Local policy:** `memory/MEMORY.md`、`STATE.md`、`PLAN.md` 与 `.codex/` 均保留在每位开发者本地并由 Git 忽略。任何密码、令牌、Cookie、私钥、`.env` 值、凭据 URL、个人数据、完整 shell history、远端敏感细节和瞬态大日志仍禁止写入。

### D5：分支保护与评审

- **Planning evidence:** Foundation 规划要求在第一次提交前明确 branch protection、PR review 和 CODEOWNERS 时机。
- **Recommended policy:** 先用 `main` 建立可审查基线；CI 能运行后启用保护和 required checks，避免在没有可用 CI 前制造无法合并的规则。
- **最低评审规则建议:** 关键架构/安全/硬件边界变更至少一名项目负责人审查；vendor 协议、部署和硬件闸门相关变更不得绕过负责人。

## FND-001 前置条件

D1–D5、GitHub owner/目标路径和 commit/push 授权均已确认，以下 FND-001 步骤已完成：

1. 编写并审查 `.gitignore` 与资产 manifest；
2. 检查 staged paths，确保不含 `CubeMars/`、`.codex/`、`.agents/`、`tmp/`、供应商二进制和构建产物；
3. 初始化根 Git 的 `main` 分支；
4. 将本地 `origin` 设置为已确认的私有远端目标；
5. 使用明确授权创建首次规划/协作基线提交；
6. 创建私有仓库、推送 `main`，并验证远端 HEAD 与 clean clone 资产边界。
