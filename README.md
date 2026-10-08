# Jetson Mechatronic Control Framework

面向 NVIDIA Jetson 与 ROS 2 的机电设备控制框架。项目以供应商无关的 C++ 核心为基础，通过统一的传输、协议和设备会话边界接入 CAN/CAN FD 电机与传感器，并以 `ros2_control` 提供标准化的控制接口。

本项目当前处于研究开发与台架验证阶段。Foundation RC2 已完成，AK3.0 力控及位置伺服链路已接入真实框架。当前伺服工具支持两台电机共用一条 CAN 总线，或分别连接同一 USB-to-CAN 通信板的两条独立总线；提供标定、反馈监视、绝对/相对移动、失能与诊断记录。

截至 2026-10-08，双路适配及 CI 修复已随 [PR #26](https://github.com/Makoto20S/jetson_mech_control/pull/26) 合入主分支。两路原位保持、命令链审计和失能确认已完成，项目负责人另行确认实际脚本实验通过。该结论限于当前台架和操作范围，不等于通用设备即插即用、长期稳定性或原通信板异常根因已确认。

## 设计目标

- **供应商解耦**：控制器、`ros2_control` 适配层、设备协议和底层传输分别演进，新增设备无需修改通用控制核心。
- **确定性与可测试性**：纯 C++ 核心不依赖 ROS 运行时，支持虚拟时钟、模拟设备、故障注入和无硬件测试。
- **明确的实时边界**：每条物理总线由一个 `BusRuntime` 统一拥有写入权，命令租约、状态新鲜度和看门狗使用单调时间语义。
- **保守的硬件激活**：未知的设备参数、协议能力或物理映射不会被默认值掩盖；未满足证据与安全闸门时拒绝激活。
- **可扩展部署**：统一的原始帧接口支持 Fake、SocketCAN/vcan 以及能力受控的 USB-CDC 传输路径。

## 架构概览

```mermaid
flowchart TB
    Controller[ROS 2 controller] --> Hardware[ros2_control SystemInterface]
    Hardware --> Core[Vendor-independent control core]
    Core --> Session[Protocol codec and device session]
    Session --> Runtime[BusRuntime]
    Runtime --> Transport[Raw-frame transport]
    Transport --> Device[CAN / CAN FD devices]

    Simulation[Virtual clock and simulated devices] --> Core
    Simulation --> Transport
```

架构遵循以下核心约束：

- 通用核心只处理帧、时间、配置、能力、路由、状态快照、命令租约与总线运行时，不包含供应商位域。
- `ros2_control` 插件仅负责生命周期和标准接口映射，不在实时 `read()` / `write()` 路径中执行阻塞 I/O。
- 协议 profile 在配置阶段固定，运行期间不自动探测或切换。
- 传输后端必须如实报告时间戳、过滤、位速率和错误能力，不合成无法验证的能力。
- 设备反馈、命令和故障状态均受新鲜度、超时和锁存规则约束。

完整接口约束见 [AdapterContract v1](docs/development/adapter_contract_v1.md)，架构决定见 [ADR 索引](docs/adr/README.md)。

## 当前能力

已实现并纳入自动化验证的主要能力包括：

- ROS 无关的 C++17 控制核心与配置校验；
- 标准帧、扩展帧、Classic CAN 与 CAN FD 数据模型；
- 冲突检测、帧路由、状态快照、命令租约和分级看门狗；
- 单物理通道单写者的 `BusRuntime`；
- 确定性虚拟时钟、Fake transport、模拟设备与故障注入；
- SocketCAN/vcan 路径和可注入串口的 USB-CDC 帧传输实现；
- 复合 `ros2_control::SystemInterface` 与有界位置、速度、力矩 controller 插件；
- AK3.0 力控 profile 和独立的伺服位置 profile，复用厂商 codec/session 与传输边界；
- 一个 controller_manager/JTC 管理两个伺服硬件实例，按总线聚合反馈、逐路失能；
- `servo-move/status/range/control` 操作入口、显式标定映射和限位、105 单机选择；
- 按总线记录有界命令链与原始串口数据，支持独立审计及离线快照归档。

以下内容尚不属于已完成能力：

- 真实 CubeMars 电机或 HI12 传感器的即插即用部署；
- 未经验证的 CAN ID、位速率、方向、减速比、力矩常数或安全限值；
- 真实硬件上的频率、实时性、力矩精度或长期稳定性承诺；
- 自动修改 Jetson 系统、CAN 接口或设备固件。

## ROS 2 包

| 包 | 职责 |
|---|---|
| `mech_control_core` | 帧、时间、配置、能力、路由、状态快照、命令租约和总线运行时 |
| `mech_simulation` | 虚拟时钟、Fake transport、参考设备与确定性故障测试 |
| `mech_hardware_ros2_control` | 连接通用核心的复合 `ros2_control` 硬件插件 |
| `mech_controllers` | 带边界、变化率和超时约束的 C++ controller 插件 |
| `mech_bringup` | 仿真与部署组合、URDF/xacro、launch 和 controller 配置 |
| `mech_protocol_cubemars` | CubeMars AK3.0 力控与位置伺服 profile：wire 编解码、映射、会话与看门狗 |

协议包本身不打开设备；真实串口、硬件插件及部署组合由 `mech_bringup` 负责。力控位置/速度/力矩子模式与伺服位置模式是不同 profile，不能混用其映射和控制入口。HI12 接入、统一安装初始化及完整 MVP 验收仍未完成，见[当前规划](docs/planning/README.md)。

## 已部署台架的伺服操作

以下入口适用于已经安装 `servo-current`、配置 `config/servo_pair.json` 并完成标定的 Jetson 部署目录，普通 clone 不会自动生成这些入口。

```bash
cd ~/jetson_mech_control
./servo-move check       # 离线检查，不打开串口
./servo-status          # 独占只读监视，Ctrl+C 退出
./servo-move            # 等待就绪后，在窗口中输入下面的交互命令
```

进入控制窗口前先退出监视窗口；同一时刻只运行一个入口。

| 交互命令 | 作用 |
|---|---|
| `enable` | 读取新鲜反馈，使能并保持当前位置 |
| `move <104角度> <105角度> <秒数>` | 移动到设备绝对角度，单位为度 |
| `step <104增量> <105增量> <秒数>` | 从当前实际位置做相对移动 |
| `stop` | 取消轨迹并请求保持当前位置，仍可输出力矩 |
| `disable` | 逐台确认失能，保留窗口；可再次 `enable` |
| `quit` | 失能并退出；已经失能时不重复发送 |

双路接线不改变参数顺序：始终为 **104、105、秒数**。`./servo-move --motor-id 105` 只选 105，移动命令相应只接受一个角度和时间。实际目标须同时满足已保存限位与机构允许范围；失能确认不等于机械制动。

标定、双路配置迁移、故障退出、记录路径及恢复步骤见[伺服工具说明](tools/servo/README.md)。框架不会在首次部署时预填现场标定；不要从示例配置推断实际设备参数。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- 支持 C++17 的编译器
- `colcon`、`rosdep` 和 CMake/ament 构建工具

目标平台为 NVIDIA Jetson ARM64；日常开发和 CI 也可在满足上述版本约束的 x86_64 主机上进行。Windows 仅作为编辑和 Git 环境，不用于形成 ROS 2、vcan、性能或 ARM64 验证结论。

## 构建与测试

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build \
  bash tools/ci/build_workspace.sh
```

该脚本会解析 ROS 依赖、构建全部包并运行测试。测试使用模拟设备和虚拟串口，不打开真实电机端口；依赖解析会按 `rosdep` 安装软件包，已配好依赖的主机可设置 `MECH_SKIP_ROSDEP=1`。CI 将镜像构建与测试分开，测试容器提供实时调度权限并保留生产保护时限；最新 PR #26 CI 为六包、590 项测试全部通过。这不是实时性能认证。更多主机配置与输出目录选项见 [ROS 2 workspace 说明](ros2_ws/README.md)。

可单独运行文档和仓库一致性检查：

```bash
python3 tools/ci/context_check.py
python3 tools/ci/check_adrs.py
git diff --check
```

## 硬件安全边界

真实设备接入不是普通构建步骤。任何 CAN 启用或非零电机命令都必须经过分阶段验收：

| 闸门 | 最低要求 |
|---|---|
| G0：设备身份 | 确认型号、固件、协议 profile、节点 ID、位速率和关键参数 |
| G1：被动总线 | 确认接线、终端、通道映射、帧格式以及 ID/位速率无冲突 |
| G2：反馈验证 | 在不发送运动命令的前提下获得稳定且语义明确的反馈与故障状态 |
| G3：台架安全 | 完成固定、限位、断能、限流、看门狗和最小命令策略检查 |

通过软件测试或 ARM64 构建不等于通过硬件激活闸门。当前单通道部署边界记录在 [ADR-006](docs/adr/ADR-006-conditional-can0-deployment.md)，其状态仍为 **Proposed**。

## 文档导航

- [伺服控制工具](tools/servo/README.md)：双机/双路配置、操作、诊断和快照恢复
- [ROS workspace](ros2_ws/README.md)：六包构建、测试与 CI 环境
- [位置伺服设计](docs/development/ak30_servo_position_design.md)：profile、映射及验收边界
- [贡献指南](CONTRIBUTING.md)：分支、Issue、Pull Request、验证和信息边界
- [架构决策记录](docs/adr/README.md)：已接受及待验证的架构约束
- [AdapterContract v1](docs/development/adapter_contract_v1.md)：协议适配器与通用核心的冻结接口
- [规划与证据索引](docs/planning/README.md)：项目路线、硬件闸门和资料来源
- [Foundation 实施计划](docs/planning/07_framework_bootstrap_plan.md)：框架范围与验收标准
- [AI 协作约定](AGENTS.md)：使用 AI 工具参与本仓库开发时必须遵循的流程

## 许可证与第三方资料

本仓库当前采用[内部研究用途声明](LICENSE-or-INTERNAL-LICENSE.md)，并非开放源代码许可证。未经项目负责人及相关机构书面许可，不得重新分发、公开发布、再许可或用于商业用途。

第三方依赖和供应商资料遵循各自许可证与分发限制。资产登记和纳入策略见 [`manifests/assets.yaml`](manifests/assets.yaml)；原始供应商资料、实验数据和生成产物不属于可提交的主仓库内容。
