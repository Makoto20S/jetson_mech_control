# 证据与来源登记

> 本表记录规划阶段使用过的来源及当时快照，不表示外部链接、远程仓库或硬件状态仍然当前。2026-08-19 用户新增确认已纳入本轮证据修订；它不替代逐台固件/配置取证。初始规划提示词已移入 [非规范归档](../archive/README.md)；当前决策见 [ADR 索引](../adr/README.md)。

## 1. 登记规则

**已确认事实**：除特别说明外，初始在线来源访问日期为 2026-07-28；CubeMars 本地供应商资料复核日期为 2026-08-03；JetPack 6 迁移相关官方来源复核日期为 2026-08-09。仓库“最近提交”来自公开 branch Atom feed 或本地只读 Git 检查，提交日期表示该分支活动，不等于发布、生产就绪或当前设备兼容。供应商原始文件不进入主仓库；链接仅表示受控外部资料索引，文件哈希见 `manifests/assets.yaml`。

**有依据的推断**：本规划引用来源用于做架构决策，不授权复制许可证不明代码。实施时仍需把实际采用 commit、许可证文本、补丁和 SBOM 固定到 release manifest。

## 2. 本地材料

| ID | 来源 | 版本/页数 | 本规划使用的证据 | 性质 |
|---|---|---|---|---|
| L01 | [归档的主规划提示词](../archive/codex_ultra_master_planning_prompt.md) | 2026-07-28 工作区版本 | 历史任务输入：项目目标、MVP、规模、边界和交付清单；非当前规范 | 历史来源 |
| L02 | CubeMars AK 系列驱动手册（受控外部文件，见 `manifests/assets.yaml`；用户确认副本位于 `company/`） | AK2.0 V1.0.18，47 页 | 用户确认其 CAN 协议和 CAN 参数适用于当前两台电机；提供伺服扩展帧与运控/MIT 标准帧两条互斥 profile 的直接实现依据 | 用户确认事实 + 资料事实 |
| L03 | HI12 系列规格书（受控外部文件，见 `manifests/assets.yaml`） | Rev 1.5，30 页 | 型号/接口、SYNC_IN/PPS、CAN 500 kbit/s、0~200 Hz、采样与环境规格 | 已确认事实 |
| L04 | HiPNUC IMU 指令与编程手册（受控外部文件，见 `manifests/assets.yaml`） | Rev 1.7.2，63 页 | J1939/CANopen、PGN/TPDO、缩放、时间、同步、默认配置 | 已确认事实 |
| L05 | DM-MC-Board02 手册（受控外部文件，见 `manifests/assets.yaml`） | V1.1 | STM32H723、3 路 CAN FD、外设与引脚图 | 已确认事实 |
| L06 | 此前组会汇报稿（排除的本地 `presentation/` 资产，见 `manifests/assets.yaml`） | 2026-07-16 工作区版本 | 早期方案和来源线索；不作为现场硬件证据，也不要求 clean clone 包含原文件 | 历史来源 |
| L07 | AK3.0 电机使用说明（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | V3.2.0，52 页 | AKE60-8、AK54、servo/force 扩展帧、`0x29/0x2A`、1–2000 Hz、Kt、CubeMarsTool | 资料事实 |
| L08 | AK54 驱动器安装说明（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | V1.0.0，9 页 | 标准硬件版本、电压、电流、CAN 位速率和线色 | 资料事实 |
| L09 | AKA 系列使用说明（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | V3.0.0，47 页 | CubeMars 内/外环双编码器通用交叉参考 | 仅参考 |
| L10 | AKE60-8 KV80 2D 图（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | 供应商目录版本 | 标准目录身份旁证；结构内容按用户要求排除 | 范围外 |
| L11 | AK V3.0 Arduino demo（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | 供应商仓库版本 | 旧式标准帧 MIT 冲突样本 | 仅参考 |
| L12 | AK CAN demo（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | 供应商仓库版本 | 历史 STM32/DJI/CMESC 混合工程，不绑定 AKE60-8 固件 | 拒绝作为协议依据 |
| L13 | AK V3.2.0 CubeMarsTool（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | 压缩包内仅 `cubemarstool_v3.2.0.exe` | 后续逐台只读版本/配置读取与参数导出 | 工具候选 |
| L14 | AKE60-8 驱动板归档（`CubeMars/` 受控外部文件，见 `manifests/assets.yaml`） | 压缩包内仅 STEP | 证明供应商把标准 AKE60-8 与 AK54 配套；结构文件不参与实现 | 范围外 |
| L15 | `company/Panthera-HT_ROS2` 供应商 ROS2 工作区 | `humble@b08633d`；根 LICENSE 为 MIT | ROS2/ros2_control 集成、配置组织和 HighTorque 电机语义参考；未发现 HI12 codec；不作为项目核心依赖 | 参考源码 |
| L16 | `company/hightorque_fdcan(2)/hightorque_fdcan` | 本地压缩包 SHA-256 `4aa51fd6...c3a860`；示例版本 `1.0.0` | USB CDC 原始 CAN/CAN-FD 透传参考：`MODE_FDCAN_PASS=0x12`、标准/扩展标志、Classic/FD/BRS、长度和 payload；不作为未经版本确认的直接依赖。2026-08-23 复核：自有代码**无 LICENSE/版权头**（内嵌 wjwwood/serial v1.2.1 为 MIT）；协议定义报错命令 `0x0F/0x11` 但示例未解析；无时间戳字段；无设置 CAN 波特率命令；要求板卡固件 ≥4.8.8 | 参考源码，**许可缺失已确认**（再分发/复制前必须取得高擎书面许可，见 05 §5.2 问题 5） |
| L17 | `company/7路CAN主控盒子资料/`（说明书、SDK-ROS1/python 文档、环境配置、SDK 压缩包、STEP；哈希见 `manifests/assets.yaml`） | 飞书 wiki 2026-08-10 前后导出快照；SDK zip 为 ros1 v4.6.2 / python v4.5.4 | 通用盒子（7路CAN功率板）硬件与部署事实：7 通道=7×`/dev/ttyACM`、Type-C、XT60(2+4)/XT30(2+2)、板载 YESENSE IMU、"FDCAN波特率：5Mbps"（数据段）、CDC 串口 4 Mbps；说明书协议章节为空占位，权威协议在 L16 源码；SDK 面向高擎自家电机，无 CubeMars/L02 型号 | 资料事实；`00 文档说明.txt` 明示本地 PDF 会滞后于在线 wiki，引用时注意时效 |

**已确认事实**：`CubeMars/` 本身是干净的嵌套 Git 仓库，来源为 `git@gitee.com:CubeMars/software.git`，复核快照为 `master@15885db`。当前资料树中没有 `.AppParams`、`.McParams`、DBC、EDS、AKE60-8 专用固件或 Linux/SocketCAN SDK。

### 2.1 关键页索引

| 主题 | 页码 | 性质 |
|---|---|---|
| AK3.0 AKE60-8/AK54 与双编码器参数 | L07 第 6~7 页 | 资料事实 |
| CubeMarsTool CAN 配置和参数导出 | L07 第 15~17 页 | 资料事实 |
| AK3.0 servo 扩展帧 | L07 第 30~35 页 | 资料事实 |
| AK3.0 力控/MIT-like 扩展帧和 AKE60-8 Kt/范围 | L07 第 36~40 页 | 资料事实 |
| AK3.0 `0x29` 反馈/1~2000 Hz 与可选 `0x2A` | L07 第 41~42 页 | 资料事实 |
| 双编码器 UART 字段 | L07 第 43 页附近 | 资料事实 |
| L02 伺服扩展帧（ID/功能/`0x29` 反馈） | L02 第 27~40 页附近 | 当前首个实现 profile 的协议依据；实际固件仍需逐台核对 |
| L02 运控/MIT 标准帧（进入/退出/零位及 8 字节反馈） | L02 第 41~45 页附近 | 当前第二实现 profile 的协议依据；ACTIVE 期间不与伺服 profile 混发 |
| L02 标准帧 MIT | L02 第 40~42 页附近；L11 demo | 当前电机的用户确认适用 profile；不再标记为仅 legacy |
| HighTorque USB CDC 透传 framing | L15/L16 及本地源码 | transport backend 参考；CDC header/CRC/版本/bitrate/错误语义仍需独立验证 |
| HI12 CAN 和同步硬件 | L03 第 11~13、20 页 | 已确认事实 |
| HI12 J1939 参数/PGN/周期 | L04 第 45~50、60 页 | 已确认事实 |
| HI12 CANopen TPDO/SDO/SYNC | L04 第 51~55 页 | 已确认事实 |
| HI12 时间/坐标/yaw 约定 | L04 第 7~15、56~57 页 | 已确认事实 |

### 2.2 CubeMars 文件 SHA-256

| ID | SHA-256 |
|---|---|
| L07 | `41069C1810C27EE2C1D03049F7FF68A14F9DC0579304176BB0240E71CCF0C5AA` |
| L08 | `649DFF4FC8EAE9DEFA79920694EA00AED3B6639DA40B1C46AACC8C47652AA04F` |
| L09 | `F0DACCB2954EBCC03CDC05A8F436B839D2AC0CD8737D01F840BCC1EAE28135BC` |
| L10 | `B58324FAB3D84B264DAA0ED76862CA3EBBC348A7FAE7AA2C9A7A577EAC7E65B2` |
| L11 | `2D35EDC8704ADF9AD40CDE29AA714C88D7B1695E3CBBCDBB468548849DD92ABE` |
| L12 | `6FD5183E5FE51C9D8BCE5A321701A6B9F622CE075ADB0F48D663A81E9FAC0423` |
| L13 | `6FDCFBE9402F5E13B1FA32DC76D2EAAAAE8D98DF6647248B26EFED82FC33DE13` |
| L14 | `87508F2B61C578A07195F7F5859B55417AD43A4B0EB9C15C10A86F8DCF86A94C` |

## 3. 本机与 Jetson 只读证据

| ID | 检查 | 时间 | 结果范围 | 性质 |
|---|---|---|---|---|
| E01 | 早期 Jetson 验证机只读系统盘点 | 2026-07-28，Asia/Shanghai | 型号、CPU/RAM、OS/L4T/kernel、ROS 包、CAN 接口、GPU 栈、工具链、磁盘；不是后来指定的实际目标机 | 历史已确认事实 |
| E02 | 工作区只读目录与 `git status` | 2026-07-28 | 当前目录不是有效 Git 仓库；规划目录此前不存在 | 已确认事实 |
| E03 | `/home/jetson/UAM_ROS` 只读盘点 | 2026-07-28 | 单体源文件、未版本化日志、dirty 修改；未做任何改动 | 已确认事实 |
| E04 | 用户指定的实际 Orin NX 目标机只读系统盘点 | 2026-08-08～09，Asia/Shanghai；2026-08-23 只读复核 | `P3767-0000` 模组、L4T 35.6.0（JetPack 5.1.4；早期记录 5.1.5 有误，2026-08-23 按 `jetson_release` 更正）/Ubuntu 20.04、内核、ROS、NVMe、Docker 和登录实时权限；设备树报 `p3768-0000` 后经 E05 证明为镜像产物；没有系统或设备修改 | 当前目标的已确认快照 |
| E05 | 目标机载板实物核验与 HZHY 供应商资料核读 | 2026-08-23，Asia/Shanghai | 现场实物照片丝印 `HYAI-311UAV_O_V12`，与 `company/用户手册/` 的 HZHY HYAI-311UAV 用户手册接口逐项吻合（XT30/J8、SW1 REC、J7 Type-C 烧录口、J2 CAN、Micro HDMI）；**推翻此前"标准 P3768 参考载板"结论**；HZHY 刷机教程 2025-07-11 版含 JetPack 6.2 命令；仅阅读本地资料与照片，未接触设备。照片为临时现场文件，按项目负责人 2026-08-23 决定**不登记为持久资产**；丝印核验结论以本条文字记录为准 | 当前目标的已确认事实（实物证据优先于软件字符串） |
| E06 | 通用盒子（7路CAN功率板）实物照片辨认 | 2026-08-23，Asia/Shanghai | 现场照片（临时文件，不登记）与 L17 说明书逐项吻合：XT60(2+4) 输入、7×XT30(2+2) 电源+CAN 输出、Type-C、YESENSE IMU 模块丝印、调试口；确认项目所称"USB2CAN 通信板"即该板的通信功能；项目负责人同日确认电机将接入该板通道 | 已确认事实 + 用户确认拓扑意向 |

**已确认事实**：E01~E04 没有安装、卸载、清理、启用 CAN、启动电机/IMU、修改服务/网络/内核或写入 Jetson。

## 4. 官方技术文档

| ID | 官方来源 | 用途 | 关键事实 | 性质 |
|---|---|---|---|---|
| O01 | [Linux Kernel SocketCAN](https://www.kernel.org/doc/html/latest/networking/can.html) | socket、filter、loopback、错误状态 | 同 ID 多 socket 都收到匹配帧；错误帧需显式订阅；接口统计含 bus-off 等 | 已确认事实 |
| O02 | [Linux Kernel Timestamping](https://www.kernel.org/doc/html/latest/networking/timestamping.html) | 接收/发送软件与硬件时间戳 | `SO_TIMESTAMPING` 支持多种时间戳，硬件能力取决于 driver/device | 已确认事实 |
| O03 | [ros2_control Humble Controller Manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html) | `read/update/write`、调度、切换 | 主线程尝试 SCHED_FIFO/50；SwitchController 和硬件错误处理语义 | 已确认事实 |
| O04 | [ros2_control Humble Hardware Components](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) | System/Sensor/Actuator 和生命周期 | 硬件组件与接口模型 | 已确认事实 |
| O05 | [ROS 2 Humble Real-time Programming](https://docs.ros.org/en/humble/Tutorials/Demos/Real-Time-Programming.html) | 锁页、动态分配、调度基线 | 实时程序需避免缺页、动态分配和无界阻塞 | 已确认事实 |
| O06 | [NVIDIA Jetson Linux 36.4.4 Real-Time Kernel](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Kernel/RealTimeKernel.html) | Orin RT 内核能力和安装/切换方法 | Orin RT kernel 为 Developer-Preview；可在 generic/RT 间切换 | 已确认事实 |
| O07 | [ros2_canopen Humble Manual](https://ros-industrial.github.io/ros2_canopen/manual/humble/) | CANopen master/driver/config/ros2_control | 文档覆盖 Lely 后端和 Humble 示例；不证明 HI12 交付为 CANopen | 已确认事实 |
| O08 | [NVIDIA JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621) | 实际目标机候选平台 | Production release；Jetson Linux 36.4.4、Ubuntu 22.04 rootfs、Linux 5.15 | 已确认事实，2026-08-09 复核 |
| O09 | [Jetson Linux 36.4.4 Quick Start](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/IN/QuickStart.html) | 支持组合与恢复模式 | 支持 Orin NX `P3767-0000` + `P3768-0000`；Force Recovery USB ID 为 `0955:7323` | 已确认事实，2026-08-09 复核 |
| O10 | [Jetson Linux 36.4.4 Flashing Support](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/FlashingSupport.html) | QSPI/NVMe 刷写、备份与恢复 | 该模块/载板的外部 NVMe 路径使用 initrd flash；刷写同时涉及启动固件和系统盘 | 已确认事实，2026-08-09 复核 |
| O11 | [Jetson Linux update mechanism](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/SoftwarePackagesAndTheUpdateMechanism.html) | OTA 和跨版本边界 | 提供 Debian OTA 与 image-based OTA；通用 `do-release-upgrade` 不是受支持的 Jetson Linux 跨代路径 | 已确认事实，2026-08-09 复核 |
| O12 | [SDK Manager Direct Flash](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson-direct-flash/index.html) | 推荐个人刷机流程 | Direct Flash 为推荐方法；支持 Recovery、NVMe、OEM 配置和刷后 target components | 已确认事实，2026-08-09 复核 |
| O13 | [SDK Manager system requirements](https://docs.nvidia.com/sdk-manager/system-requirements/index.html) | 刷机主机能力和兼容矩阵 | 至少 8 GB RAM；Jetson 完整部署最低约 27 GB host + 16 GB target；具体 JetPack 仍须核对兼容矩阵 | 已确认事实，2026-08-09 复核 |
| O14 | [JetPack 6.2.3 / Jetson Linux 36.5.2 公告](https://forums.developer.nvidia.com/t/jetpack-6-2-3-jetson-linux-36-5-2-is-now-live/379872)、[JetPack 6.2.2 / 36.5 公告](https://forums.developer.nvidia.com/t/jetpack-6-2-2-jetson-linux-36-5-is-now-live/359622) | 目标平台小版本更新 | 6.2.2/6.2.3 为 6.2.1 后的缺陷与安全修复 production release，仍为 Ubuntu 22.04 / 内核 5.15，支持全部 Orin；JetPack 7.x 为 Ubuntu 24.04，不满足本项目 Humble 基线 | 已确认事实，2026-08-23 核对 |
| O15 | [ROS signing key migration guide](https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937) | 目标机 ROS 2 Humble 安装 | 2025 年起 ROS APT 仓库与签名 key 改由 `ros2-apt-source` 包管理；旧手工 keyring 方式的 key 已过期 | 已确认事实，2026-08-23 核对 |
| O16 | 鱼香ROS 一键安装脚本 `http://fishros.com/install`（`sha256 10cbba5e…315d`）及主体 `http://mirror.fishros.com/install/install.py`（`a5e7d6bc…9e63`）、`tools/{base,tool_install_ros,tool_config_system_source}.py` | 目标机 ROS 2 Humble 的实际安装路线 | 源码审计确认：**不执行** `apt upgrade`；但「更换系统源并清理第三方源」选项会 `sudo rm -rf /etc/apt/sources.list.d`；`AptUtils.checkapt()` 遇证书错误会永久写入 `99verify-peer.conf` 关闭 apt TLS 校验；主体经明文 HTTP 拉取后以 root 执行且无签名校验 | 已确认事实（仅限上述两个哈希对应的版本），2026-08-23 审计。脚本每次运行现拉，结论不自动延伸到后续版本 |

**有依据的推断**：O06 针对 36.4.4 文档，不证明 2026-07-28 的 36.4.7 历史验证机使用 RT 内核。实际目标机当前仍是 R35；任何 RT 内核变更都应在完成平台迁移、建立版本匹配证据和回滚方案后另立任务，不可照抄 O06 执行。

## 5. ROS、Linux CAN 与记录组件仓库

| ID | 项目与 URL | 分支快照 | 许可证证据 | 备注 | 性质 |
|---|---|---|---|---|---|
| G01 | [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) | Humble `e65ddd72804f`，2026-07-14 | Apache-2.0 根 LICENSE | 早期验证机已安装 2.54.0；实际目标机尚未安装 Humble | 已确认事实 |
| G02 | [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers) | Humble `73177880c302`，2026-07-22 | Apache-2.0 | 选择性复用 broadcaster/controller | 已确认事实 |
| G03 | [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) | Humble `5efdb018aef2`，2026-07-14 | Apache-2.0 | 生命周期和测试参考 | 已确认事实 |
| G04 | [ros-controls/realtime_tools](https://github.com/ros-controls/realtime_tools) | Humble `3e85fba8f44e`，2026-07-21 | BSD-3-Clause | [RealtimeBuffer 源码](https://github.com/ros-controls/realtime_tools/blob/humble/include/realtime_tools/realtime_buffer.hpp)显示 mutex/try-lock | 已确认事实 |
| G05 | [autowarefoundation/ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan) | Humble `a9204072121c`，2024-07-16；main `e16bb19c51f3`，2026-03-19 | package.xml 与源码头 Apache-2.0；Humble 根 LICENSE 未找到 | Humble 分支与 main 活跃度要分开评估 | 已确认事实 |
| G06 | [ros-industrial/ros2_canopen](https://github.com/ros-industrial/ros2_canopen) | Humble `fef50e54b1c9`，2025-09-11；master `faa2a77551e2`，2026-06-01 | package.xml Apache-2.0；Humble 根 LICENSE 未找到 | README 明示 under development/not production ready | 已确认事实 |
| G07 | [lely-industries/lely-core](https://github.com/lely-industries/lely-core) | master `620d1858eb85`，2023-12-12 | Apache-2.0 | ros2_canopen 后端候选，不单独直连 MVP | 已确认事实 |
| G08 | [linux-can/can-utils](https://github.com/linux-can/can-utils) | master `95aae6bf83ac`，2026-05-12；release `v2025.01` | 以 GPL-2.0-only 为主，按文件 SPDX | 早期验证机是 2020.11.0；作为外部 CLI | 已确认事实 |
| G09 | [ros2/rosbag2](https://github.com/ros2/rosbag2) | Humble `cd220c16a1f2`，2026-07-23 | Apache-2.0 | ROS 级记录/回放，非线速 CAN 仿真 | 已确认事实 |

## 6. CubeMars 候选仓库

| ID | 项目与 URL | 默认分支快照 | 许可证 | 证据用途 | 性质 |
|---|---|---|---|---|---|
| G10 | [OpenFieldAutomation-OFA/cubemars_hardware](https://github.com/OpenFieldAutomation-OFA/cubemars_hardware) | main `1827512f22e3`，2024-09-28 | package 声明 Apache-2.0；根 LICENSE 未找到 | ros2_control servo 映射、AK70-10 硬件经验 | 已确认事实 |
| G11 | [Optimal-Robotics-Lab/motor-control-sdk](https://github.com/Optimal-Robotics-Lab/motor-control-sdk) | main `b8cf1d50b5f0`，2025-09-11 | Apache-2.0 | WIP；AKE60-8 硬编码参数和 AK V3 force-control 字段对照，只作参考 | 已确认事实 |
| G12 | [dfki-ric-underactuated-lab/mini-cheetah-tmotor-can](https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can) | master `089fe7596235`，2023-01-16 | 未找到明确许可证 | 特定 AK/MIT 真实硬件旁证，不复制代码 | 已确认事实 |
| G13 | [ziteh/tmotor-ak-actuators-driver](https://github.com/ziteh/tmotor-ak-actuators-driver) | main `4510840a6d4a`，2022-10-11 | MIT | 小型 MIT codec 对照 | 已确认事实 |
| G14 | [sam0rr/cubemars_servo_can](https://github.com/sam0rr/cubemars_servo_can) | main `b785116f8960`，2026-07-27 | MIT | Python servo mock/golden 对照 | 已确认事实 |

## 7. HI12、STM32 与机器人架构参考

| ID | 项目与 URL | 默认分支快照 | 许可证 | 证据用途 | 性质 |
|---|---|---|---|---|---|
| G15 | [hipnuc/products](https://github.com/hipnuc/products) | master `5a4380272cd7`，2026-07-02 | 根许可证不清；ROS 包声明 Apache-2.0 | J1939/CANopen/DBC/ROS 示例；需与 L04 冲突核对 | 已确认事实 |
| G16 | [open-dynamic-robot-initiative/odri_control_interface](https://github.com/open-dynamic-robot-initiative/odri_control_interface) | main `dcc07e4efc91`，2025-06-05 | BSD-3-Clause | 低层设备与控制分离参考 | 已确认事实 |
| G17 | [OpenCyphal/libcanard](https://github.com/OpenCyphal/libcanard) | master `120600375535`，2026-07-02 | MIT | STM32 序号、分片、确定内存思想 | 已确认事实 |
| G18 | [CANopenNode/CANopenNode](https://github.com/CANopenNode/CANopenNode) | master `9b8beed83672`，2026-07-10 | Apache-2.0 | 条件 CANopen 嵌入式参考 | 已确认事实 |
| G19 | [STMicroelectronics/STM32CubeH7](https://github.com/STMicroelectronics/STM32CubeH7) | master `a2de035db3d8`，2026-06-26 | 根 `LICENSE.md`：HAL/BSP BSD-3-Clause、CMSIS Apache-2.0，其他中间件逐组件 | 后续 STM32H7 HAL/LL/FDCAN 底层 | 已确认事实 |
| G20 | [project-march/march](https://github.com/project-march/march) | develop `f31c47794809`，2020-12-01；仓库归档 | 根 LICENSE 未找到，许可待核 | 外骨骼 ROS 架构历史，不复制代码 | 已确认事实 |
| G21 | [naubiomech/OpenExo](https://github.com/naubiomech/OpenExo) | main `523852aaa184`，2026-06-11 | 根 `LICENSE.md`：硬件 CERN-OHL-P v2.0、软件 LGPL v3.0 | 多执行器嵌入式概念，不作依赖 | 已确认事实 |
| G22 | [neurobionics/opensourceleg](https://github.com/neurobionics/opensourceleg) | main `2fb083d834ba`，2026-07-06 | LGPL-2.1 | Python 实验/API 概念 | 已确认事实 |

## 8. 来源冲突登记

| 冲突 | 较强证据 | 处理 | 性质 |
|---|---|---|---|
| 早期汇报 AKE60-8 vs 现场只知 AKE60 | 用户于 2026-08-03 明确确认基型 AKE60-8 | 基型冲突已解决；定制件号、固件和配置仍逐台确认 | 用户确认事实 |
| 早期汇报 HI12 CANopen vs 标准交付 J1939 | L04 Rev 1.7.2 | 逐台识别交付固件，不自动转换 | 已确认事实 |
| HiPNUC GitHub parser vs Rev 1.7.2 yaw/环境字段 | L04 字段表优先，G15 仅参考 | 为交付固件建立 golden frame；冲突上报供应商 | 有依据的推断 |
| L02/L11 标准帧 MIT vs AK3.0 V3.2 扩展帧力控 | 用户确认 L02 的 CAN 协议/参数适用于当前电机；L07 是另一代补充资料 | 当前分别实现 L02 servo-extended 与 L02 MIT-standard，先做前者；AK V3 profile 保留为独立补充，不用版本号自动覆盖用户适用性证据 | 用户确认事实 + 规划决定 |
| L02 `0x29` 最高 500 Hz vs V3.2 最高 2000 Hz | 两者属于不同资料/profile 范围；用户确认 L02 当前适用 | L02 当前 profile 上限保持 500 Hz；AK V3 profile 若以后被逐台证明适用，再独立使用 1–2000 Hz/`0x2A`，不得跨代覆盖 | 用户确认事实 + 规划决定 |
| motor-control-sdk 扩展帧 force vs 旧规划所称冲突 | L07 证明 AK3.0 本就采用扩展帧 force-control | 撤销“帧类型冲突”结论；SDK 仍因 WIP/硬编码/无实机证据而仅参考 | 有依据的推断 |
| NVIDIA 36.4.4 RT 文档 vs 历史验证机 36.4.7 | E01/O06 | 不安装；先找匹配版本并建立回滚 | 有依据的推断 |
| 早期验证机已是 R36/Humble vs 实际目标机仍是 R35/Noetic | E01/E04 | **已解决（2026-08-23）**：实际目标机完成迁移（JetPack 6.2 / Ubuntu 22.04.5 / Humble），两机平台差异消除；E01 保留为历史旁证 | 已确认事实 |
| L02 V1.0.18 编码器 14-bit vs AK V3.2 双编码器 21/15-bit | 两份手册针对不同代产品 | 定制双编码器实机以哪套规格为准列入供应商问题（05 §5.1 问题 3）；未确认前不得据任一值设计分辨率相关逻辑 | 资料矛盾，待确认 |
| L02 MIT 示例归一化常量（T ±18 N·m 等） vs 第 42 页型号物理极限表 | 手册自身不一致（示例代码不按型号切换常量），且型号表不含 AKE60-8 | MIT codec 归一化常量必须来自逐台设备证据（05 §5.1 问题 2），禁止硬编码示例值 | 资料矛盾，安全相关 |
| ros2_canopen docs 标题 0.0.1 vs Humble package 0.2.13 | O07/G06 package.xml | 依赖锁定 package/commit，不依赖网页标题判断版本 | 有依据的推断 |

## 9. 证据保全建议

**有依据的推断**：实施仓库应把本地手册 SHA-256、供应商邮件/订单引用、设备照片索引、只读配置导出、原始 candump、测试脚本 commit 和解析报告放入不可变实验 manifest。受版权或隐私限制的资料保存外部 URI 和哈希，不强行提交 Git。

**有依据的推断**：任何在线链接失效时，优先使用 release manifest 中锁定的 commit 和许可证副本；更新来源必须通过 PR 说明“事实变化”还是“仅活动日期变化”，不能静默改写已发布实验依据。
