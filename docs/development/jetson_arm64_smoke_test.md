# FND-004A：Jetson ARM64 原生烟测

> 状态：Planned
> 执行时机：FND-004 完成后、FND-005 开始前
> 目标：尽早证明 Foundation 构建基线可在目标 Jetson 原生 ARM64 环境工作

## 范围与安全边界

本任务只执行 clean clone、环境盘点、portable context check、依赖解析和五包 build/test。它不启用或配置 `can0`，不打开 CAN socket，不发送电机命令，不连接或修改 CubeMars/HI12，不调整内核、实时调度、systemd 或设备权限。

若 clean clone 无法构建且需要安装系统包、修改 apt/rosdep 源或更改 Jetson 配置，应先记录缺失项并取得明确授权；不得把环境修复静默并入烟测。

## 前置条件

- 目标机确认为 Jetson Orin ARM64，运行项目支持的 Ubuntu 22.04 / ROS 2 Humble 组合；
- FND-004 ADR 已完成并通过文档检查；
- 测试使用主仓库 clean clone，不复制 Windows/WSL 的 build、install 或 log；
- 工作目录和输出目录位于 Jetson 原生 Linux 文件系统；
- 当前提交的 GitHub CI/context 检查已通过，或明确记录其未通过原因。

## 执行清单

1. 记录但不修改以下环境事实：Jetson 型号、架构、Ubuntu、JetPack/L4T、内核、ROS、GCC/G++、CMake、Python、colcon 和 rosdep 版本。
2. 从主远端 clean clone，确认 branch/commit/status，验证 Git 索引不跟踪 `memory/`、`.codex/`、供应商资产或生成物（`project-memory` 可在 clone 后创建被忽略的本地 `memory/`）。
3. 运行 `python3 tools/ci/context_check.py`。
4. `source /opt/ros/humble/setup.bash`，先运行官方 `rosdep check --from-paths ros2_ws/src --ignore-src --rosdistro humble`。如果依赖已满足，下一步用 `MECH_SKIP_ROSDEP=1` 构建；如果缺少依赖，先记录并按上述授权边界决定是否运行会安装系统包的标准脚本。
5. 在原生 Linux 输出目录运行 `MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-fnd004a MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`；只有已明确授权依赖安装时才去掉 `MECH_SKIP_ROSDEP=1`。
6. 记录发现的架构、编译、链接、测试或依赖差异；不得把 ARM64 成功扩展表述为 vcan、性能、实时性或硬件成功。

## 完成标准

- `uname -m` 等价证据确认 `aarch64`/ARM64；
- portable context check 通过；
- 官方 rosdep 路径可解析依赖，或缺失依赖被明确记录为 blocker；
- 五个 Foundation packages 原生构建完成且当前测试全部通过；
- 版本清单、实际命令、结果摘要和限制附在 FND-004A GitHub Issue；
- 没有 CAN/设备操作或未经授权的系统修改。

FND-004A 通过后才能开始 FND-005。FND-015 仍负责完整 ARM64 clean build、sanitizer、性能、稳定性和 Foundation RC 证据，不能被本烟测替代。
