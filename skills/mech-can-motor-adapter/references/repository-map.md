# 仓库代码地图

核对基线：`jetson_mech_control`，2026-10-09，`4d1bcf62652eb608908c970194890d8b34e0f940`。下列路径均相对**目标仓库根**。文件移动时用 `rg --files` 找实际定义，不回退代码来迎合地图。

## 权威入口

- `AGENTS.md`、`docs/development/ai_collaboration_workflow.md`：上下文、分支、授权、checkpoint。
- `docs/planning/README.md`、`docs/planning/03_mvp_delivery_plan.md`：当前阶段和硬件验收范围。
- `docs/development/adapter_contract_v1.md`：新适配器清单与接口边界。
- `docs/development/architecture_package_map.md`：职责参考；其历史段落中的“速度/力矩控制器尚未实现”等可能过期，必须对照当前代码。
- `docs/adr/README.md`：查当前状态。001–005 对应分层/总线/profile/时间，009 对应 effort 映射，012 对应看门狗与能力，015–018 对应授权/质量/generation/目标寿命；014/019/020 等按任务读取，不能把 Proposed 自动视为已批准。

## 实现定位

| 仓库相对路径 | 用途 |
|---|---|
| `ros2_ws/src/mech_control_core/include/mech_control_core/adapter_template.hpp` | DeviceCodec/DeviceSession 与 device canonical 类型，检查当前签名 |
| `ros2_ws/src/mech_control_core/include/mech_control_core/config.hpp` | ProtocolProfile、配置及能力；新值还需搜索解析/schema/校验调用方 |
| `ros2_ws/src/mech_control_core/include/mech_control_core/runtime.hpp` | BusRuntime、所有权、路由和租约 |
| `ros2_ws/src/mech_control_core/include/mech_control_core/transport.hpp` | transport 契约与能力 |
| `ros2_ws/src/mech_control_core/src/posix_cdc_serial_port.cpp` | 共用串口、inode 锁、实际内核写跟踪，不复制品牌串口实现 |
| `ros2_ws/src/mech_protocol_cubemars/` | wire/codec/session 分层与异常测试参考，不沿用设备常量 |
| `ros2_ws/src/mech_hardware_ros2_control/include/mech_hardware_ros2_control/composite_system.hpp` | RuntimePort、CommandDispatch、CompositeSystem，与 device canonical 类型职责不同 |
| `ros2_ws/src/mech_hardware_ros2_control/README.md` | 状态子集、命令组合、INACTIVE 与多组件切换 |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_force_runtime.hpp` | 力控运行时接线参考 |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_servo_runtime.hpp` | 伺服 RuntimePort、clock 和首故障记录 |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_system.hpp` | 力控生产组合插件 |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_servo_system.hpp` | 伺服生产插件与注入点；类为 final，勿假设可直接派生 |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_runtime_params.hpp` | 力控参数 fail-closed 校验 |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_servo_runtime_params.hpp` | 伺服参数校验 |
| `ros2_ws/src/mech_controllers/README.md` | 已有位置/速度/力矩控制器和目标有效期 |
| `ros2_ws/src/mech_simulation/` | fake transport/serial，离线故障注入 |
| `tools/servo/` | AK3.0 标定/监视/操作/失能工具，不能简单改名冒充新品牌支持 |

## 部署与验证

- 按模式查 `docs/development/ak30_force_control_adapter_design.md`、`docs/development/ak30_servo_position_design.md`、`docs/development/position_feedforward_design.md`。
- 在 `ros2_ws/src/mech_bringup/` 定位 CMake 的 pluginlib 注册、XML、xacro、controller YAML、launch 及测试。
- 新包查 `docker/ros_humble_jammy/Dockerfile`、`tools/ci/context_check.py`、`tools/ci/summarize_test_reports.py`。用当前包清单，不硬编码六包/八包。
- CI 契约：`docs/development/foundation_validation.md`、`.github/workflows/foundation.yml`；执行入口 `tools/ci/build_workspace.sh`、`tools/ci/run_sanitizers.sh`。
- 供应商资料按 `manifests/` 当前规则登记；`CubeMars/` 是独立资料仓库，不进主仓库。可读取不等于可再分发。

只读定位示例：

```text
git rev-parse --show-toplevel
git status --short
rg --files ros2_ws/src docs tools/ci
rg -n "ProtocolProfile|RuntimePort|CommandDispatch" ros2_ws/src
rg -n "pluginlib_export_plugin_description_file|ament_add_gtest" ros2_ws/src
```
