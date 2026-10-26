# 2026-10-20 集成整理软件验收

## 范围

源码基线为 `aa1ba05`，已合并主线 `d8ae6c5`。环境为 Jetson ARM64、Ubuntu 22.04、ROS 2 Humble。本次整理不访问真实串口、不启用 CAN、不发送电机命令，不修改设备配置；软件结果不能替代实机验收。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| `python3 tools/ci/context_check.py` | 11 包、17 项 ADR，通过 |
| `python3 tools/ci/check_docs.py` | 463 个本地链接，通过（新增本记录前） |
| `python3 tools/ci/check_script_syntax.py` | 96 个 Python、12 个 Bash，通过 |
| `python3 -m unittest discover -s tools/ci -p 'test_*.py'` | 62 项通过 |
| `rosdep check --from-paths ros2_ws/src --ignore-src --rosdistro humble` | 系统依赖全部满足 |
| 普通 ARM64 构建 | 11 包成功 |
| 独占运行普通完整回归 | 11 包、colcon 汇总 741 项，0 errors、0 failures |
| ASan/UBSan 构建及完整回归 | 11 包成功；741 项，0 errors、0 failures；未报告 sanitizer 错误 |

普通构建命令：

```bash
MECH_SKIP_ROSDEP=1 MECH_OUTPUT_ROOT=/home/nvidia/mech_cleanup_validation_20261020 \
  CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 bash tools/ci/build_workspace.sh
```

不与编译并发的完整复测：

```bash
MECH_TEST_ONLY=1 MECH_SKIP_ROSDEP=1 \
  MECH_OUTPUT_ROOT=/home/nvidia/mech_cleanup_validation_20261020 \
  bash tools/ci/build_workspace.sh
```

Sanitizer 命令：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  bash tools/ci/run_sanitizers.sh /home/nvidia/mech_cleanup_sanitizers_20261020
```

插桩检查确认 9 个 C++ 包均带 ASan/UBSan；2 个 Python 包不适用插桩，但执行其测试。泄漏检测遵循脚本默认关闭，不据此声称通过泄漏检查。

直接解析普通与 sanitizer XML，各有 688 个 testcase、0 失败、11 包报告齐全。colcon 的计数包含测试包装层，与直接 XML 口径不同。两轮均有 **5 项实际跳过**：4 项 SocketCAN 测试因没有 vcan，1 项 JTC 参数组合不适用；不采用 colcon 显示的“0 skipped”作为实际跳过数。本任务没有为补测开启 CAN。

## 保留的失败与边界

首次当前源码普通回归与 sanitizer 编译并行时，colcon 汇总 741 项、1 error、3 failures；XML 中两份失败报告分别来自控制器激活脚本和 E5 进程生命周期服务超时。随后停止编译竞争，完整普通回归通过；未修改测试超时阈值。并行负载是可能原因，不能据此认定所有时序问题已消除。

初次单独调用膝角检查器时未先启动被测节点，服务不可用；统一入口负责启动合成节点并清理进程。四项合成图检查覆盖膝角、影子安全、步态影子和组合架构，均不代表真实传感器计量或电机闭环验证。

原始日志和 XML 汇总保存在本机 `/home/nvidia/mech_cleanup_review/evidence/`，不提交原始实验数据。没有运行 x86 容器矩阵或 GitHub Actions，不能声称跨平台 CI 已通过。

独立 DM4340 力矩模式实机验收、真实传感器精度与完整穿戴闭环仍需另行授权验证；历史速度前馈记录不能替代上述结果。
