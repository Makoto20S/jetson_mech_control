# 事实基线与开源调研

> 证据快照：初始验证机盘点日期为 2026-07-28，CubeMars 补充审查日期为 2026-08-03，用户指定的实际目标机盘点日期为 2026-08-08。远程主机、仓库和硬件运行态均可能已经变化；本文件保留来源与历史观察，不作为当前运行状态。当前决策见 [ADR 索引](../adr/README.md)。

## 1. 调研边界与方法

**已确认事实**：初始规划完整阅读了主提示词、四份本地 PDF 和此前汇报稿；2026-08-03 又审查了供应商 `CubeMars/` 仓库中与 AKE60-8、AK3.0、AK54 驱动板和双编码器有关的资料及 demo。2026-08-19 根据用户确认，补充复核了 `company/Panthera-HT_ROS2`、`company/hightorque_fdcan(2)` 和 AK2.0 V1.0.18 手册的适用边界。2026-08-08 对用户后来指定的实际目标机完成了新的只读 SSH 盘点；两次 Jetson 盘点均没有配置 CAN、设备、内核、服务或网络。[L01-L05][L07-L16][E01-E04]

**历史事实**：截至 2026-07-28，工作区根目录虽然存在名为 `.git` 的目录，但 `git status` 返回“not a git repository”；`docs/planning/` 在当时尚不存在。该状态已由 FND-001 建仓替代，当前仓库状态以 Git 为准。旧的 `/home/jetson/UAM_ROS` 只作为只读构建经验参考，不是新架构模板，也未被修改。[E02][E03]

**有依据的推断**：筛选开源组件时，协议名称相似不算兼容证据。核心依赖至少需要明确许可证、目标 ROS 分支、维护状态、可测试 API、ARM64 可构建性和与当前设备/固件一致的真实硬件证据；任一关键项缺失时降级为“封装验证”或“仅参考”。

## 2. 本地硬件与协议事实

### 2.1 基于 AKE60-8 的 CubeMars 定制电机

| 结论 | 证据 | 性质 |
|---|---|---|
| 两台定制电机均基于 AKE60-8，结构重新设计并加装 CubeMars 双编码器；结构不属于软件任务 | 2026-08-03 用户确认 | 用户确认事实 |
| 供应商仓库把标准目录项写作 `AKE60-8 KV80`；定制实机是否保持 KV80、标准减速器和控制参数尚未验证 | CubeMars README 与用户边界 | 待确认项 |
| AK V3.2 把标准 AKE60-8 映射到 `AK54-4810-1C-A2` 驱动板；标准资料值为 48 V、允许 18–52 V、CAN 1 Mbit/s | L07 第 6 页、L08 第 4~8 页 | 资料事实 |
| AK V3.2 servo 使用 29 位扩展帧；电流/速度/位置/位置速度功能 ID 分别为 1/3/4/6，失能为 15，反馈设置为 16 | L07 第 30~35 页 | 资料事实 |
| servo 电流命令为 4 字节 `int32(Iq * 1000)`；位置速度命令为 8 字节 | L07 第 31、34 页 | 资料事实 |
| AK3.0 力控/MIT-like 使用 29 位扩展帧、控制模式 ID 8；payload 顺序为 `Kp, Kd, position, velocity, torque` | L07 第 36~40 页 | 资料事实 |
| `0x29` 扩展反馈为 8 字节，包含位置、ERPM、Iq、驱动板温度和故障码；周期上传可配置 1~2000 Hz | L07 第 41 页 | 资料事实 |
| 可通过反馈设置启用额外 `0x2A` 位置帧；该设置写 Flash，不能周期发送 | L07 第 34~35、42 页 | 资料事实 |
| 通用双编码器资料给出 21-bit 内环和可选 15-bit 外环，均为单圈绝对值；CAN `0x29/0x2A` 使用哪个编码器仍未说明 | L07 第 7、42~43 页；L09 第 6~7 页 | 部分资料事实，实机待确认 |
| 标准 AKE60-8 的 Kt 为 `0.7382 N*m/A`、速度范围 `-40..40 rad/s`、扭矩范围 `-15..15 N*m`；手册把 `T = Kt * Iq` 中的 T 定义为输出端输出扭矩 | L07 第 37~38 页 | 资料事实 |
| V3.2 称 AK3.0 servo 与力控使用无需切换，但两者 ID、payload 和 command claim 不同；实机固件未核对前，ACTIVE 期间仍固定一个命令 profile | L07 第 5、21、30~40 页；架构约束 | 资料事实 + 规划决定 |
| `joint/effort` 可按标准 AKE60-8 参数建立候选映射，但必须先确认定制版仍使用相同 Kt、范围、方向和固件参数 | 标准资料与定制边界 | 有依据的推断 |
| 用户确认 L02 V1.0.18 的 CAN 协议和参数适用于当前电机；伺服扩展帧与运控/MIT 标准帧均纳入目标，先做伺服扩展帧 | 2026-08-19 用户确认；L02 第 27~45 页 | 用户确认事实 + 规划决定 |

### 2.2 HI12

| 结论 | 证据 | 性质 |
|---|---|---|
| 支持 CAN 的标准交付默认 J1939、CAN 2.0B 扩展帧、500 kbit/s、节点地址 8 | 编程手册第 45~47、60 页 | 已确认事实 |
| J1939 位速率可选项包含 125/250/500/800/1000 kbit/s，但现场型号实际支持仍须读取 | 编程手册第 46 页；成员 B 在 W1D3 前逐台读取并抓包确认 | 待确认项 |
| J1939 默认 100 Hz 输出加速度、角速度、俯仰/横滚、航向，共 4 个独立 8 字节扩展帧 | 编程手册第 47~50、60 页 | 已确认事实 |
| 若选择控制常用的加速度、角速度、四元数，则每台每次更新为 3 个独立扩展帧 | 三个公开 PGN 各占一帧 | 已确认事实 |
| CAN 输出帧率规格为 0~200 Hz，实际上限取决于内容、报文长度和配置 | 数据手册第 20 页 | 已确认事实 |
| CANopen 是特定/定制交付从站固件，标准帧；默认 TPDO 周期为 0，不主动输出 | 编程手册第 45、51~55 页 | 已确认事实 |
| CANopen TPDO1/2/4 分别为加速度 `0x180+ID`、角速度 `0x280+ID`、四元数 `0x480+ID` | 编程手册第 52 页 | 已确认事实 |
| CANopen SYNC 令配置为同步模式的 TPDO 各发送一帧，但不证明内部传感器同一时刻采样 | 手册只承诺同步发送 | 有依据的推断 |
| 严格同步应使用支持型号的 `SYNC_IN/PPS`；仍需示波器/逻辑分析仪测量采样到发送关系 | 数据手册第 12~13 页；成员 B 在 W2 用逻辑分析仪确认 | 待确认项 |
| J1939、CANopen 的缩放和航向约定不同，解析器必须按交付协议选择 | 编程手册第 47~57 页 | 已确认事实 |
| J1939 各数据 PGN 没有公开的公共样本序号，不能仅凭相近到达时间宣称组成严格同一采样 | 公开帧定义 | 有依据的推断 |

### 2.3 STM32 传感器节点

| 结论 | 证据 | 性质 |
|---|---|---|
| 达妙 DM-MC-Board02 V1.1 使用 STM32H723VGT6，提供 3 路 CAN FD、SPI、I2C、UART、QSPI 和板载 BMI088 | 本地开发板手册 | 已确认事实 |
| 当前引脚图只明显显示一路 `PA05` ADC 能力，不能据此设计多通道压力/扭矩模拟前端 | 本地手册引脚图 | 已确认事实 |
| 首期只定义节点职责、消息语义、采样时间、序号、状态、标定和诊断边界，不实现固件或 ADC 前端 | 项目要求 | 已确认事实 |
| 混有经典 CAN 节点的物理段不应直接启用 CAN FD 数据帧；STM32 首版应支持经典 CAN 兼容配置 | 经典 CAN 节点对 FD 帧兼容性不能假设 | 有依据的推断 |

## 3. Jetson 证据快照

### 3.1 早期验证机（历史快照）

| 项目 | 2026-07-28 只读结果 | 性质 |
|---|---|---|
| 设备 | NVIDIA Jetson Orin NX Engineering Reference Developer Kit Super；8 核 Cortex-A78AE；约 15 GiB RAM | 已确认事实 |
| 操作系统 | Ubuntu 22.04.5；L4T 36.4.7 | 已确认事实 |
| 内核 | `5.15.148-tegra ... PREEMPT`；没有证据证明是 PREEMPT_RT | 已确认事实 |
| ROS | ROS 2 Humble | 已确认事实 |
| 控制栈 | `ros2_control 2.54.0`、`ros2_controllers 2.53.1`、`realtime_tools 2.15.0` | 已确认事实 |
| CAN 工具 | `can-utils 2020.11.0` 已装；`ros2_socketcan`、`ros2_canopen` 未装 | 已确认事实 |
| CAN 接口 | 仅 `can0`，驱动 mttcan，状态 DOWN/STOPPED；未发现 `can1` | 已确认事实 |
| AI/GPU | TensorRT 10.7、NVIDIA PyTorch 2.5、CUDA runtime 12.6 可见 | 已确认事实 |
| 构建工具 | GCC 11.4、CMake 3.22.1、colcon、rosdep 0.26、vcs 0.3、Docker 29.5.3 | 已确认事实 |
| 存储 | 根分区约 233 GiB，已用 177 GiB，约 80% | 已确认事实 |
| 主要占用 | Ollama 约 36 GiB、workspaces 约 15 GiB、cache 约 13 GiB、旧 UAM 日志约 6.3 GiB | 已确认事实 |
| 旧工程 | `/home/jetson/UAM_ROS` 有约 141 KB 单体硬件源文件、大量未版本化日志和用户修改 | 已确认事实 |

**已确认事实**：NVIDIA 的 Jetson Linux 36.4.4 官方文档说明 Orin 系列可安装开发预览质量的 RT 内核；这不等于该历史验证机的 36.4.7 系统已经使用 RT 内核，也不证明 36.4.4 的包可直接用于 36.4.7。[O06]

**有依据的推断**：MVP 先在当前内核上量化周期抖动、缺页和负载干扰。只有指标不达标且已排除应用层阻塞后，才建立单独的 PREEMPT_RT 变更计划、回滚启动项和兼容性测试；本轮不修改内核。

**有依据的推断**：该历史验证机当时的可用空间尚能支持开发，但 80% 使用率不适合无配额持续录包。任何录制任务必须先检查空间、分包、限额并把正式数据迁移到外部存储；不自动删除现有文件。

### 3.2 用户指定的实际目标机（2026-08-08 快照）

| 项目 | 2026-08-08 只读结果 | 性质 |
|---|---|---|
| 模块与载板 | NVIDIA Orin NX 16GB 模组（`P3767-0000`）+ **HZHY HYAI-311UAV 第三方载板**（2026-08-23 实物照片核验，丝印 `HYAI-311UAV_O_V12`；设备树报 `p3768-0000+p3767-0000` 是厂商基于 devkit 配置构建镜像的产物，早期"参考开发套件"记录据此更正）；约 15 GiB RAM | 已确认事实（实物核验） |
| 操作系统 | JetPack 5.1.4 / L4T 35.6.0 / Ubuntu 20.04.6（2026-08-23 复核更正：L4T 35.6.0 对应 JetPack 5.1.4，早期记录的 5.1.5 有误） | 已确认事实 |
| 内核 | `5.10.216-tegra`，配置包含 `CONFIG_PREEMPT=y`；不等于 PREEMPT_RT | 已确认事实 |
| ROS | ROS 1 Noetic；没有 `/opt/ros/humble`，没有 `colcon` | 已确认事实 |
| 存储 | 128 GB NVMe 原生 ext4 rootfs；约 52 GB 已用、59 GB 可用 | 已确认事实 |
| Docker | Docker daemon 与 NVIDIA Container Toolkit 已安装；目标用户当时不能直接访问 Docker socket | 已确认事实 |
| 实时权限 | 登录环境 `rtprio=0`，memlock 为 64 KiB | 已确认事实 |

**已确认事实（历史快照，已被 2026-08-23 迁移取代）**：上表为迁移前状态。该机当时不满足 FND-004A 的 Jammy/Humble 前置条件。[E04]

**已确认事实（2026-08-23 迁移完成）**：目标机已使用 HZHY 厂商镜像 `flash-300BV12_311UAV_ONX_Jp6.2_SC` 经 `l4t_initrd_flash.sh` 刷写为 **JetPack 6.2 / L4T R36.4.3 / Ubuntu 22.04.5 / 内核 5.15.148-tegra**（rootfs NVMe 全盘 116 GiB），首启验收通过；刷机后加固（`nvidia-l4t-*` 全部 apt-mark hold、禁用无人值守升级）与 ROS 2 Humble（`ros-humble-ros-base` + rosdepc，经审计的鱼香ROS 路线）已完成。**FND-004A 的平台前置条件已满足**，烟测本身尚未执行。过程记录见[升级教程](../development/jetson_orin_nx_jetpack6_upgrade_guide.md) §12.0/§14.0。迁移过程未启用 CAN、未操作设备。

**资料事实**：JetPack 6.2.x（2026-08-09 核对时为 6.2.1 / Jetson Linux 36.4.4；2026-08-23 复核已有 6.2.2 / 36.5.0 与 6.2.3 / 36.5.2）是支持该 Orin NX/参考载板组合的 production release 系列，提供 Ubuntu 22.04 rootfs 和 Linux 5.15。R35 到 R36 迁移涉及 Jetson BSP、QSPI/UEFI、分区和 rootfs，不能用通用 `do-release-upgrade` 替代 NVIDIA 支持的刷写/镜像升级路径。[O08-O13]

**规划决定（已执行完毕，2026-08-23）**：迁移目标 JetPack 6.2.x、禁止 JetPack 7.x 的决定已按计划执行。实际路线为 HZHY 适配镜像 + `l4t_initrd_flash.sh`（早先设想的 SDK Manager Direct Flash 因载板为第三方 HYAI-311UAV 而被否定，见升级教程 §2.2 的载板更正）；备份经双介质哈希验证后授权刷写。完整执行记录与结果见[升级教程](../development/jetson_orin_nx_jetpack6_upgrade_guide.md)。

## 4. 事实、推断与未知项基线

| 主题 | 当前结论 | 确认/验证方式 | 负责人和期限 | 性质 |
|---|---|---|---|---|
| 电机身份 | 基型 AKE60-8 已确认；定制件号、每台序列号、驱动板和固件仍未知 | 两台实机的连接/版本截图与只读参数导出 | 项目负责人 + B，W1D2 | 部分确认 |
| 电机协议 | L02 V1.0.18 的伺服扩展帧与运控/MIT 标准帧是当前实现目标；L02 适用性由用户确认，但每台实际 profile/固件/ID 仍未知 | 读取固件版本、CAN 配置并与两套 L02 golden frame 交叉验证；AK3.0 V3.2 作为补充参考而非强制假设 | 项目负责人，G0 | 部分确认 |
| 输出力矩 | 标准 AKE60-8 有输出端 Kt/范围候选；定制版是否不变以及物理精度未知 | 对比 `.AppParams`/`.McParams`、供应商确认；需要时做外部计量 | 项目负责人 + A，G3 前 | 部分确认 |
| 电机失联行为 | L02 V1.0.18 第 18 页证实存在"失控保护"（失控时间 ms + 失控刹车电流 A，上位机配置）；出厂默认值与启用状态未知，截图字段为 0 疑似默认关闭 | 逐台读取"应用功能"页参数并截图；低能量台架断包测试验证实际行为 | 项目负责人 + B，G0 读取 / G3 验证 | 资料部分解决 |
| 两台 HI12 | 使用 CAN 已知；准确型号/固件/协议/ID/位速率未知 | 读取 PNAME、APP_VER；逐台只读抓包；断电持久化复核 | B，W1D2 | 待确认项 |
| HI12 同步 | 手册有 SYNC_IN/PPS；现场引脚/固件支持未知 | 订购码、引脚、配置读取；逻辑分析仪测量 | B + A，W2 | 待确认项 |
| 总线拓扑 | 用户确认（2026-08-23）电机将接入高擎通用盒子（7路CAN功率板）的 XT30(2+2) 通道；两台 HI12 接盒子通道或独立适配器均为候选。盒子通道=7×`/dev/ttyACM` 已由资料证实；**每通道 nominal 波特率固件固定且数值未知**，与电机 1 Mbps / HI12 500 kbps 的兼容性是当前最关键拓扑未知项 | 高擎书面答复（05 §5.2）+ 实物板卡/固件 G0 核对 + 抓包 | B，G0/G1 | 部分收口，关键项待供应商答复 |
| 控制频率 | 当前 L02 两电机正常目标为最高 500 Hz；100~200 Hz 是 bring-up 档；代码支持 1 kHz 主机循环测试 | 周期直方图、command-to-wire、单物理通道最坏负载、闭环性能和长稳 | 项目负责人，W3~W4 | 有依据的推断 |
| PREEMPT_RT | 当前未证实 | `uname`、内核配置、调度基准；需要时另立变更 | 项目负责人，W3 决策 | 待确认项 |
| STM32 传感器 | 数量、信号形式、采样率、模拟前端未知 | 传感器 BOM、量程/带宽/接口评审 | B + A，MVP 后 | 待确认项 |

## 5. 既有汇报稿的冲突

**已确认事实**：此前汇报稿是早期方案，不是现场硬件证据。它曾假定 AKE60-8、HI12 CANopen、共享 1 Mbit/s、500 Hz、固定 CAN ID 且未纳入 STM32。AKE60-8 基型后来由用户独立确认；其余假设仍不能从早期汇报直接继承。[L06]

| 早期假设 | 当前证据 | 处理 | 性质 |
|---|---|---|---|
| AKE60-8 及其参数 | 用户已确认基型为 AKE60-8；V3.2 给出标准参数，但定制实机配置未读取 | 基型进入配置枚举；Kt、范围、固件和编码器来源仍按每台实机必填 | 有依据的推断 |
| HI12 已是 CANopen | 手册称标准交付通常为 J1939 | 逐台识别交付固件，禁止自动猜协议 | 已确认事实 |
| 全部设备共享 1 Mbit/s | HI12 默认 500 kbit/s，CubeMars 常见配置不能代表现场 | 默认双总线；同总线只作为有条件备选 | 有依据的推断 |
| 500 Hz 固定循环 | 六电机扩展帧保守负载达 96%，但当前只有两电机 | 当前两电机 profile 以 500 Hz 为正常目标；六电机单总线 profile 仍从 200~250 Hz 起测 | 有依据的推断 |
| 固定 ID/缩放 | 设备 ID、型号和固件未知 | 成员 B 与项目负责人在 W1D3 前逐台读取，并写入 schema 验证的配置 | 待确认项 |

## 6. ROS、CAN 与实时基础组件

| 项目 | 许可证 | 维护/发行版/ARM64 | API、线程与证据 | 结论 | 性质 |
|---|---|---|---|---|---|
| [ros2_control][G01] | Apache-2.0 | Humble 分支 2026-07-14 有提交；早期验证机已有 ARM64 包，实际目标机尚未安装 | Controller Manager 执行 `read -> update -> write`；主线程尝试 `SCHED_FIFO/50` | 采用；成本中，债务是 Humble 恢复限制 | 有依据的推断 |
| [ros2_controllers][G02] | Apache-2.0 | Humble 分支 2026-07-22 有提交；早期验证机已安装，实际目标机尚未安装 | 提供状态/IMU 广播和通用控制器；不解决项目特有命令租约 | 选择性采用；成本低，需核对接口版本 | 有依据的推断 |
| [ros2_control_demos][G03] | Apache-2.0 | Humble 分支 2026-07-14 有提交；C++ 跨平台，早期验证栈提供 ARM64 旁证 | 有生命周期、接口和测试样例，无本项目硬件证明 | 参考；成本低，不形成运行依赖 | 有依据的推断 |
| [realtime_tools][G04] | BSD-3-Clause | Humble 分支 2026-07-21 有提交；早期验证机已安装，实际目标机尚未安装 | `RealtimeBuffer` 使用 mutex，RT 侧 `try_to_lock`；不能称为无锁 | 采用；成本低，债务是对象/锁基准 | 有依据的推断 |
| [ros2_socketcan][G05] | package.xml/源码头声明 Apache-2.0；Humble 根 LICENSE 未找到 | Humble 分支停在 2024-07-16；main 2026-03-19；Linux/ARM64 可行但 Jetson 未装 | 生命周期收发节点、接收线程和 ROS 发布；DDS 节点路径不适合作为电机确定性命令链 | 封装/基准；成本中，风险是分支陈旧和许可证复核 | 有依据的推断 |
| [ros2_canopen][G06] | 各包声明 Apache-2.0；Humble 根 LICENSE 未找到 | Humble 2025-09-11；master 2026-06-01；无正式 Jetson/ARM64 matrix，需 CI；README 明示非生产就绪 | Lely 事件循环、master/driver/ros2_control 集成；有 CANopen 设备示例 | 条件封装；成本高，风险是线程所有权和成熟度 | 有依据的推断 |
| [lely-core][G07] | Apache-2.0 | master 最近提交 2023-12-12；跨平台 C/C++，无正式 Jetson matrix | 成熟 CANopen 栈，但直接集成成本高、线程模型需统一 | 传递依赖；成本高，不单独维护 fork | 有依据的推断 |
| [can-utils][G08] | 以 GPL-2.0-only 为主，按文件 SPDX | master 2026-05-12；早期验证机已装旧版，提供 ARM64 可用旁证 | `candump`、`canplayer`、`cangen`、`canbusload`、`canerrsim` 等外部 CLI | 采用；成本低，外部进程避免链接许可债务 | 有依据的推断 |
| [rosbag2][G09] | Apache-2.0 | Humble 2026-07-23；官方 ARM64 ROS 包可用 | ROS 级记录/回放；不是线速 CAN 故障仿真 | 采用；成本低，主要风险是磁盘/CPU | 有依据的推断 |

**已确认事实**：Linux SocketCAN 允许同一接口上多个 socket 订阅相同 ID，匹配帧会复制给所有监听者；错误消息帧默认关闭，应用必须显式设置错误过滤；接口统计可报告 bus-off、error-passive、丢包等状态。[O01]

**有依据的推断**：因此统一总线运行时的理由是单写者调度、统一时间戳/错误状态/队列和诊断，而不是“多个 RAW socket 会抢走帧”。独立只读 `candump` 可在试验时并行存在，但不得成为第二个命令写者。

**已确认事实**：Linux `SO_TIMESTAMPING` 支持软件和在驱动/硬件支持时的硬件时间戳；“接口存在”不代表 mttcan 或 USB 适配器实际提供了可用硬件接收时间戳。[O02]

## 7. CubeMars 驱动候选

| 项目 | 许可证与活动 | 支持范围/硬件证据 | API/实时性与主要缺口 | 结论 | 性质 |
|---|---|---|---|---|---|
| [OpenFieldAutomation-OFA/cubemars_hardware][G10] | package 声明 Apache-2.0，根许可证缺失；2024-09-28 | ROS 2/ros2_control servo SystemInterface；未给正式 distro/ARM64 matrix；作者报告两台 AK70-10 | 直接打开设备；激活/停用、连续切换、回滚、新鲜度、校准力矩不足 | 参考；成本低，风险是许可与型号差异 | 有依据的推断 |
| [Optimal-Robotics-Lab/motor-control-sdk][G11] | Apache-2.0；2025-09-11 | 非 ROS；ARM64 未证明；明示 WIP/不稳定；C++23/Bazel；硬编码 AKE60-8 | 扩展帧 force-control 与 V3.2 方向一致，但仍需逐字节核对且没有两台实机固件证据 | 参考；不直接集成 | 有依据的推断 |
| [mini-cheetah-tmotor-can][G12] | 未找到许可证；2023-01-16 | 非 ros2_control；ARM64 matrix 缺失；记录特定 AK 型号/固件的真实 MIT 硬件 | 陈旧、无许可证 | 参考；只作 MIT golden 旁证，不复制代码 | 有依据的推断 |
| [tmotor-ak-actuators-driver][G13] | MIT；2022-10-11 | 非 ros2_control；ARM64 matrix 缺失；小型 MIT 编解码，主要面向 AK10-9 | 型号覆盖窄、维护停滞、无本项目固件证据 | 参考；成本低但型号债务高 | 有依据的推断 |
| [cubemars_servo_can][G14] | MIT；2026-07-27 | Python/SocketCAN，非确定性 ros2_control；无正式 ARM64 matrix；mock 测试但无 AKE60 确证 | Python 和动态对象不适合确定性 C++ 路径 | 参考；只作 golden 对照，运行集成成本为零 | 有依据的推断 |

**规划决定（2026-08-19）**：不直接采用任何现成 CubeMars/HighTorque 驱动作为核心。电机协议以用户确认适用的 L02 V1.0.18 为当前首要实现依据，先实现伺服 29 位扩展帧，再实现运控/MIT 11 位标准帧；两套 codec/session 共享 canonical 接口但不共享会混淆位域的打包逻辑。AK3.0 V3.2、ROS2 SDK 和其他示例只作交叉参考，不能覆盖 L02 的适用性确认。HighTorque FDCAN 示例只作为 USB CDC raw-CAN transport 参考；每个 backend、codec 和 session 均需以离线 golden/negative/boundary 测试和后续逐台证据闭合，设备能力和缩放按准确配置，不继承第三方库中的型号常量。

**资料事实与边界**：`hightorque_fdcan` 的 `MODE_FDCAN_PASS (0x12)` 记录了 CAN ID、标准/扩展标志、Classic/FD/BRS 标志、长度和 payload，并通过 USB CDC CRC 帧批量收发；它不是电机协议实现。其示例构造函数会打开 `/dev/ttyACM*`、查询板卡版本并在失败时退出，因此只能提炼受控 transport contract，不能原样成为生产 `BusRuntime`。

**资料事实（2026-08-23，`7路CAN主控盒子资料` 与 fdcan 源码交叉审查，SDK 日志截图人工复核）**——通信板此前的多项未知已收口：

- 目标硬件为高擎"通用盒子"（7路CAN功率板，`company/4.png` 实物照片；主控盒子 = 该板 + RK3588 主机装壳）。XT60(2+4) 48V 输入，**7 个 XT30(2+2) 电源+CAN 输出通道**，Type-C USB，板载 YESENSE IMU 模块。
- **7 路通道枚举为 7 个独立 `/dev/ttyACM0~6` CDC 设备**（SDK 日志"Serial Port0~6"截图证实），USB VID/PID `0xCAF1:0xFFFF`，CDC 串口 4 Mbps；每通道一个串口设备，与"每物理通道一个 `BusRuntime` 写者"（ADR-002）天然映射。
- USB 协议：帧头 `0xF7` + cmd + len + CRC8(头) + CRC16(数据体)，命令码 0x00~0x12；权威定义只在 `serial_struct.h` 源码——**说明书协议章节为空占位**，本地 PDF 是飞书 wiki 导出快照（`00 文档说明.txt` 明示会过时）。
- 协议**定义了**报错命令 `MODE_FDCAN_MOTOR_STATE (0x0F)` / `0x11`，但示例库未实现解析（能力可自行实现，语义需向供应商索取）；协议**无时间戳字段**；**无设置 CAN 波特率的命令**——总线速率由板卡固件固定，说明书唯一表述为"FDCAN波特率：5Mbps"（应为数据段；仲裁段速率未记载）。每通道 nominal 速率与第三方 Classic CAN 设备（电机 1 Mbps、HI12 默认 500 kbps）兼容性成为当前最关键的通信板未知项，已列入 [05 §5.2](05_decisions_and_open_questions.md) 问题清单。
- 示例库要求板卡固件 ≥`4.8.8`，而 SDK 文档截图显示 v4.6 板——版本体系混乱，交付板实际固件属 G0 取证项。
- 盒子 SDK（ROS1/python）面向高擎自家电机（型号 4538/4438 等，机器人 "mini pai"），无任何 CubeMars/L02 型号，仅 raw 透传与本项目相关；`hightorque_fdcan` 自有代码**无 LICENSE 与版权头**（内嵌 wjwwood/serial 为 MIT）。

## 8. HI12、STM32 与通用机器人参考

| 项目 | 许可证/活动 | 可复用证据 | 缺口与结论 | 性质 |
|---|---|---|---|---|
| [hipnuc/products][G15] | 根许可证不清，ROS package 声明 Apache-2.0；2026-07-02 | ROS 2 文档面向 Foxy，无正式 ARM64 matrix；有 `hipnuc_imu_can`、J1939/CANopen、DBC 和 Linux canhost | 版本 `0.0.0`；线程直接发布、无 socket filter、设备时间被 host now 替代、多帧无序号组合；参考成本低，直接采用风险高 | 有依据的推断 |
| HiPNUC 解析冲突 | 同上 | 本地 Rev 1.7.2 与仓库解析器逐字段比较 | 仓库 J1939 yaw 取值与手册双表示字段不一致，环境帧把保留值当压力；必须以交付手册和 golden frame 为准 | 有依据的推断 |
| [STM32CubeH7][G19] | 组件化许可：HAL/BSP 为 BSD-3-Clause，CMSIS 为 Apache-2.0，其他中间件按组件；2026-06-26 | STM32H7 官方 HAL/LL、FDCAN 支持 | 后续固件底层采用；本月不创建固件，依赖时生成逐组件 SBOM | 有依据的推断 |
| [libcanard][G17] | MIT；2026-07-02 | 确定内存的 Cyphal/CAN 传输、CAN FD、transfer-ID 等成熟思想 | 引入完整 Cyphal 生态超出 MVP；只参考序号、分片和内存边界 | 有依据的推断 |
| [CANopenNode][G18] | Apache-2.0；2026-07-10 | 嵌入式 CANopen 栈、STM32 可移植 | 只有系统选择 CANopen 时才有价值；首期自定义传感语义无需强行采用 | 有依据的推断 |
| [odri_control_interface][G16] | BSD-3-Clause；2025-06-05 | 低层设备/控制分离和实时机器人接口经验 | 不支持本项目设备或 ros2_control；参考分层与实验接口 | 有依据的推断 |
| [project-march/march][G20] | 仓库根许可证未找到；仓库已归档，最近活动约 2020 | 外骨骼架构历史 | 不复制/依赖，仅参考领域命名和故障经验 | 有依据的推断 |
| [naubiomech/OpenExo][G21] | 硬件 CERN-OHL-P v2.0、软件 LGPL v3.0；2026-06-11 | 多执行器外骨骼嵌入式架构概念 | 非 ROS、混合许可证；仅参考概念 | 有依据的推断 |
| [opensourceleg][G22] | LGPL-2.1，活跃 | Python 实验 API、装置抽象 | 非确定性运行核心；参考实验与数据接口 | 有依据的推断 |

## 9. “采用 / 封装 / 参考 / 拒绝”矩阵

| 类别 | 项目/做法 | 使用边界 | 集成成本 | 主要风险 | 性质 |
|---|---|---|---|---|---|
| 采用 | ros2_control、controller_manager | 生命周期、资源 claim、`read/update/write`、控制器切换 | 中 | Humble 错误恢复能力有限 | 有依据的推断 |
| 采用 | ros2_controllers 中的状态/IMU 广播组件 | 非命令关键的标准发布 | 低 | 接口命名需与 Humble 版本核对 | 有依据的推断 |
| 采用 | realtime_tools | 定长非 RT 到 RT 快照；不宣称 lock-free | 低 | mutex/对象拷贝抖动需测 | 有依据的推断 |
| 采用 | can-utils、rosbag2 | 外部诊断、抓包、回放、记录 | 低 | 磁盘、版本和 GPL 工具边界 | 有依据的推断 |
| 封装 | ros2_socketcan 底层库/节点 | 诊断桥接和 API 基准；不默认进电机路径 | 中 | Humble 分支陈旧、DDS 延迟 | 有依据的推断 |
| 封装 | ros2_canopen + Lely | 仅真实设备确认 CANopen 后的专用后端 | 高 | 项目自述非生产就绪、总线事件循环所有权 | 有依据的推断 |
| 封装 | 自研纯 C++ SocketCAN `BusRuntime` | 确定性命令、路由、时间、错误和最新值队列 | 中 | 需承担测试和维护 | 有依据的推断 |
| 参考 | 所有 CubeMars 候选、HiPNUC 官方示例 | 字段对照、golden frame、硬件经验 | 低 | 型号、固件和许可证差异 | 有依据的推断 |
| 参考 | odri、OpenExo、opensourceleg、MARCH | 分层、实验和领域工作流 | 低 | 不能证明协议兼容 | 有依据的推断 |
| 参考 | libcanard、CANopenNode | STM32 协议的序号、分片、状态设计 | 低 | 过度设计 | 有依据的推断 |
| 拒绝 | motor-control-sdk 直接接入 | 只作 AK V3 force-control 字段对照，不继承其硬编码参数或构建体系 | 无 | WIP、硬编码且没有当前实机固件证据 | 有依据的推断 |
| 拒绝 | 无许可证/陈旧设备驱动作为核心依赖 | 只允许人工阅读，不复制代码 | 无 | 法律和维护风险 | 有依据的推断 |
| 拒绝 | 独立 ROS/DDS CAN 网关进入电机闭环 | 可作只读诊断旁路，不能串在命令路径 | 无 | 调度、复制、生命周期和超时不可控 | 有依据的推断 |

## 10. 调研后的明确边界

**有依据的推断**：开源生态能提供 ros2_control 生命周期、SocketCAN 基础库、CANopen 栈、实时缓冲、工具与测试参考，但没有一个仓库同时证明“当前 AKE60 固件 + 当前两台 HI12 固件 + Jetson Humble + 多协议共享总线 + 可回滚控制器切换”。设备协议适配、统一总线协调、时间/新鲜度和命令租约仍属于本项目核心责任。

**待确认项**：在任何第三方代码进入主分支前，负责人必须锁定 commit、保存许可证扫描结果、记录修改边界、在 ARM64/Humble CI 构建并完成 vcan/HIL；`ros2_socketcan` 和 `ros2_canopen` 还需进行仓库级许可证复核。负责人为项目负责人，最迟在对应集成 Issue 合并前完成。

[L01]: ../archive/codex_ultra_master_planning_prompt.md
[L02]: ../../manifests/assets.yaml
[L07]: ../../manifests/assets.yaml
[L08]: ../../manifests/assets.yaml
[L09]: ../../manifests/assets.yaml
[L10]: ../../manifests/assets.yaml
[L11]: ../../manifests/assets.yaml
[L12]: ../../manifests/assets.yaml
[L03]: ../../manifests/assets.yaml
[L04]: ../../manifests/assets.yaml
[L05]: ../../manifests/assets.yaml
[L06]: 04_source_register.md
[O01]: https://www.kernel.org/doc/html/latest/networking/can.html
[O02]: https://www.kernel.org/doc/html/latest/networking/timestamping.html
[O06]: https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Kernel/RealTimeKernel.html
[G01]: https://github.com/ros-controls/ros2_control
[G02]: https://github.com/ros-controls/ros2_controllers
[G03]: https://github.com/ros-controls/ros2_control_demos
[G04]: https://github.com/ros-controls/realtime_tools
[G05]: https://github.com/autowarefoundation/ros2_socketcan
[G06]: https://github.com/ros-industrial/ros2_canopen
[G07]: https://github.com/lely-industries/lely-core
[G08]: https://github.com/linux-can/can-utils
[G09]: https://github.com/ros2/rosbag2
[G10]: https://github.com/OpenFieldAutomation-OFA/cubemars_hardware
[G11]: https://github.com/Optimal-Robotics-Lab/motor-control-sdk
[G12]: https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can
[G13]: https://github.com/ziteh/tmotor-ak-actuators-driver
[G14]: https://github.com/sam0rr/cubemars_servo_can
[G15]: https://github.com/hipnuc/products
[G16]: https://github.com/open-dynamic-robot-initiative/odri_control_interface
[G17]: https://github.com/OpenCyphal/libcanard
[G18]: https://github.com/CANopenNode/CANopenNode
[G19]: https://github.com/STMicroelectronics/STM32CubeH7
[G20]: https://github.com/project-march/march
[G21]: https://github.com/naubiomech/OpenExo
[G22]: https://github.com/neurobionics/opensourceleg
