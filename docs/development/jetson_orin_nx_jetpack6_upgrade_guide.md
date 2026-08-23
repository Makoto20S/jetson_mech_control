# Jetson Orin NX 升级到 JetPack 6.2.x / Ubuntu 22.04

> 状态：迁移路线与刷机主机已由项目负责人确认；尚未授权或执行刷机
> 适用设备：NVIDIA Jetson Orin NX 模组 `P3767-0000`（16GB）+ **合众恒跃（HZHY）HYAI-311UAV 第三方载板**（丝印 `HYAI-311UAV_O_V12`），系统盘为 NVMe。**载板不是 NVIDIA 官方 P3768**，身份更正见 [§2.2](#22-载板身份更正2026-08-23实物核验)
> 刷机方法：**HZHY 厂商适配镜像包 + `l4t_initrd_flash.sh` 命令行**（厂商教程流程）；SDK Manager Direct Flash 不再是本方案主路线
> 目标版本：JetPack **6.2.x**（Ubuntu 22.04 / Linux 5.15），以 HZHY 实际提供的 6.2 适配镜像为准；**不得选择 JetPack 7.x**（Ubuntu 24.04，破坏 Humble 基线）
> 刷机主机：本地 Ubuntu 22.04.5 x86_64 主机硬件满足要求；但厂商教程红字要求 Ubuntu 20.04，主机版本兼容性需与厂商确认（见 [§4.1](#41-推荐的刷机主机)）
> 最近核对：2026-08-23（目标机只读复核、载板实物照片比对、HZHY 用户手册与刷机教程核读、JetPack 版本矩阵复核）
> 安全边界：本文不启用 CAN、不连接或操作电机/HI12，也不授权修改当前 Jetson

## 1. 先说结论

对这台标准 NVIDIA 开发套件，个人完成升级的总体难度是 **中等，约 3/5**。它不是需要编译内核或焊接硬件的高级开发任务，但也不是普通的 Ubuntu 软件更新；正确理解应当是“一次带启动固件更新的整机重装”。

| 项目 | 评估 | 说明 |
|---|---:|---|
| 操作复杂度 | 3/5 | SDK Manager 提供图形化流程，难点主要是恢复模式、USB 识别和选对 NVMe |
| 数据风险 | 4/5 | Direct Flash 会重写目标系统盘；没有经过验证的备份就不应开始 |
| 变砖风险 | 2/5 | 标准载板通常能重新进入 Force Recovery 后重刷；中途失败不等于永久损坏 |
| 回滚难度 | 3/5 | 回滚也是重新刷对应 JetPack 并恢复数据，不是点击“撤销” |
| 个人可操作性 | 可以 | 前提是有物理接触、可靠备份、稳定供电和兼容刷机主机 |

如果你会独立重装一台电脑、能按厂商教程完成"按住 REC 键上电进恢复模式 + 命令行刷写"，并愿意预留半天，通常可以自己做。以下情况不建议独自立即操作：

- 只能通过 SSH 远程接触 Jetson，现场无人能接跳线、重启或看显示器；
- NVMe 中有唯一副本的数据，或者不知道哪些现有服务正在使用这些数据；
- 设备当前承担不能中断的生产任务；
- 没有可靠 USB 数据线、稳定电源或可用刷机主机；
- 实际硬件不是本文限定的 `P3767-0000` 模组 + HZHY HYAI-311UAV 载板组合；
- 尚未拿到 HZHY 的 JetPack 6.2 适配镜像包。

建议按首次操作预留 **3～6 小时或半天维护窗口**：

| 阶段 | 常见耗时 |
|---|---:|
| 盘点、决定备份范围并完成备份 | 1～3 小时，主要取决于数据量 |
| 向厂商索取镜像包并下载/校验 | 不可控（取决于厂商响应），应提前数天启动 |
| 刷机主机环境准备与镜像解压 | 30～60 分钟 |
| 恢复模式、系统刷写 | 20～45 分钟 |
| 首次启动和可选 JetPack 组件安装 | 30～90 分钟 |
| ROS 2、项目工具和 FND-004A 前置准备 | 1～2 小时 |

USB、网络或镜像获取问题可能把首次操作延长到一天以上，因此不要安排在必须马上恢复设备的时间段；**镜像包索取应作为前置任务提前完成**。

## 2. 为什么主路线是刷机

Jetson Linux 不只是 Ubuntu rootfs。JetPack 5 到 JetPack 6 的跨代变化同时涉及：

- QSPI 中的启动固件和 UEFI；
- Jetson Linux 内核、设备树和 NVIDIA 板级驱动；
- 分区布局与 NVMe rootfs；
- Ubuntu 20.04 到 22.04 的用户态；
- CUDA、TensorRT 等 NVIDIA 用户态组件。

因此不能用通用 `do-release-upgrade` 把这台设备安全地变成 JetPack 6。NVIDIA 的 Jetson Linux 文档明确禁用通用 Ubuntu release upgrade；JetPack 文档中的 APT 升级步骤面向兼容的 JetPack 6.x 版本，不应套用于当前 R35/JetPack 5 到 R36/JetPack 6 的迁移。

本文推荐路线是（2026-08-23 按 §2.2 载板身份更正后修订）：

1. 验证并备份现有 NVMe 数据（可选加做厂商整盘镜像备份，见 §7.2）；
2. 向 HZHY/供应链索取 **311UAV + Orin NX 的 JetPack 6.2 适配镜像包**并校验；
3. 按 HZHY 教程用 **`l4t_initrd_flash.sh`** 同步刷写 QSPI 与 NVMe（SW1 + J7 Type-C 进恢复模式）；
4. 完成 Ubuntu 22.04 首次启动；
5. 只选择性恢复配置和数据；
6. 原生安装 ROS 2 Humble 和构建工具（注意厂商镜像禁 `apt upgrade`，见 §12/§14）；
7. 再按项目清单执行 FND-004A。

**为什么不用 SDK Manager**：SDK Manager 的 Direct Flash 面向 NVIDIA 官方开发套件的载板/板型配置。本机载板是 HZHY HYAI-311UAV，厂商镜像对内核与设备树做了适配，厂商的全部教程（含 2025-07-11 版新增的 JetPack 6.2 命令）都走 `l4t_initrd_flash.sh` 命令行。`l4t_initrd_flash.sh` 的 board config、分区 XML 和参数由厂商镜像包和教程给定，不需要自行拼装。

也不要把 Orin Nano 的 SD 卡镜像教程直接套到本机。本机是 Orin NX 模块并从 NVMe 启动；必须让刷机流程同时处理正确的 Jetson BSP、QSPI 和 NVMe。

### 2.1 目标 JetPack 小版本的选择与版本禁区

本文最初写作时把目标固定为 JetPack 6.2.1 / L4T 36.4.4。**2026-08-23 复核发现该版本已被两次小版本取代**，因此目标改为“JetPack 6.2.x 中当天可选的最新版本”，具体值在刷机当天确定并记入验收记录。

| JetPack | Jetson Linux | 用户态 | 内核 | 本项目适用性 |
|---|---|---|---|---|
| 6.2.1 | 36.4.4 | Ubuntu 22.04 | 5.15 | 可用但已过时；存在后续版本修复的 CUDA 内存分配缺陷，且社区存在 `36.4.4` / `36.4.7`（rev1）版本号歧义 |
| 6.2.2 | 36.5.0 | Ubuntu 22.04 | 5.15 | **保守可选**；相对 6.2.1 只含缺陷与安全修复，已发布较久 |
| 6.2.3 | 36.5.2 | Ubuntu 22.04 | 5.15 | **首选**；当前最新 6.x production release，含安全修复；支持全部 Orin 模块 |
| 7.x（如 7.2） | 38.x | **Ubuntu 24.04** | 6.8 | **禁止选择**，见下 |

**选择规则（2026-08-23 按厂商镜像路线修订）**：

1. 目标是 HZHY 为 **311UAV + Orin NX** 提供的**最新 JetPack 6.2.x 适配镜像包**；厂商 2025-07-11 版教程已含 JetPack 6.2 刷机命令，外部旁证的包名样例为 `flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210.tar`。
2. 具体 L4T 小版本以厂商实际提供的包为准，记录包名、版本与 SHA-256；不自行混搭官方 BSP 与厂商 rootfs。
3. 若厂商暂无 6.2.x 包而只有 6.0/6.1，先与厂商确认路线再决策，不擅自用官方 devkit 镜像刷第三方载板作为绕过手段。
4. 无论刷入哪一版，都必须把**实际的 JetPack/L4T 版本**写入 FND-004A 环境记录，不得沿用本文示例值。

**为什么禁止 JetPack 7.x**：JetPack 7.x 的 rootfs 是 **Ubuntu 24.04**，其官方 ROS 2 发行版是 Jazzy 而不是 Humble。本项目的依赖清单、CI 与 FND-004A 烟测都钉在 Ubuntu 22.04 / ROS 2 Humble；选择 7.x 会让目标机无法满足 FND-004A 的前置条件，并与 [`02_architecture_and_interfaces.md`](../planning/02_architecture_and_interfaces.md) 第 14 节“不在 MVP 同时维护 Humble/Jazzy 分支”的决定冲突。若厂商未来只提供 7.x 包，那是一个独立的 ADR 与 CI 变更决策，不能通过刷机顺手完成。

**本项目不受 L4T 小版本钉死**：仓库的依赖清单只钉住 Ubuntu 22.04 与 ROS 2 Humble，没有任何构建产物钉住 `36.4.4`。因此在 6.2.x 内部选新版本不会影响 Foundation 的可复现性。

**厂商镜像约束**：确定使用 HZHY 适配镜像后，实际 JetPack 小版本以厂商提供的 6.2 适配镜像包为准，不再自行在 6.2.1/6.2.2/6.2.3 之间挑选官方镜像。上表仍用于理解版本关系和守住"不选 7.x"这条线。

**2026-08-23 已取得镜像包**：`company/flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210.tar.gz`（3,742,482,561 bytes，SHA-256 `7b2227dcb22e1ebcc8a868e629308b896ff3bd01761da647035e4e0cf6d1429e`）。已验证：包内 rootfs 的 `/etc/nv_tegra_release` 为 **R36 REVISION 4.3（= JetPack 6.2 / Ubuntu 22.04 / 内核 5.15）**；`Linux_for_Tegra/` 下 `l4t_initrd_flash.sh`、`l4t_flash_prerequisites.sh`、`l4t_create_default_user.sh`、`l4t_backup_restore.sh`、`jetson-orin-nano-devkit-super-nvme.conf`、两个 flash XML 均在；`initrdlog/` 里保留了厂商 2025-02-11/12 的多次实测刷写日志（使用 devkit-super DTB），证明该包经厂商实际验证。**本次刷机的目标版本就此锁定为 JetPack 6.2 / L4T 36.4.3**，满足 Ubuntu 22.04 基线；不追新到 6.2.2/6.2.3。

### 2.2 载板身份更正（2026-08-23，实物核验）

**本文 2026-08-23 之前的所有版本都假定载板是 NVIDIA 官方参考载板 `P3768-0000`。该假定已被实物照片和供应商手册共同推翻，本节是本方案最重要的一次更正。**

**证据：**

1. 实物照片（`company/1.jpeg`～`3.jpeg`，2026-08-23 拍摄）显示载板 PCB 丝印为 **`HYAI-311UAV_O_V12`**，并贴有同名条码标签（序列号 `553722012670704`）；
2. `company/用户手册/HYAI-311UAV 用户手册.pdf`（北京合众恒跃科技，v2.0，2023-04-14 适配 Orin NX）的接口图与照片逐项吻合：XT30 电源座 J8（12~36V，照片中的黄色插头）、GH1.25 卧贴连接器（J2 CAN、J4 USB2.0、J11/J13 UART）、RJ45（J10）、双 USB3.0 Type-A（J15/J16）、Micro HDMI（J17）、M.2 Key-M NVMe（J26，照片中贴 128GB 手写标签的盘）、板卡尺寸 91.5×55mm；
3. 官方 P3768 载板的标志性特征——DC 电源座、40-pin GPIO header、12-pin button header `J14`、DisplayPort——**这块板全都没有**。

**为什么此前的软件盘点会误判**：设备树 compatible 报 `nvidia,p3768-0000+p3767-0000`、dts 为 `tegra234-p3767-0000-p3768-0000-a0.dts`、`jetson_release` 显示 "NVIDIA Orin NX Developer Kit"。这是因为 HZHY 的适配镜像基于官方 devkit 板型配置构建（厂商教程原话："与官方镜像相比，只更改了内核与设备树"；其 JetPack 6.x 刷机命令直接使用 `jetson-orin-nano-devkit(-super)-nvme` 板型配置）。**模组是真的 P3767-0000 Orin NX 16GB，但载板层面的"标准 NVIDIA 参考组合"结论是软件字符串造成的误判。**

**对本方案的影响（各节已相应修订）：**

| 原假定 | 实际 | 影响章节 |
|---|---|---|
| SDK Manager Direct Flash | **HZHY 镜像包 + `l4t_initrd_flash.sh` 命令行**；SDK Manager 面向官方开发套件，不适用于本载板 | §8～§11 |
| J14 pin 9–10 跳线进恢复模式 | **按住 SW1 按键后上电 3–4 秒**；烧录口为载板 **J7 USB Type-C** | §9 |
| DC 电源座 J16、恢复模式自动上电 | **XT30（J8）12~36V 供电**；J22/SW2 为电源开关 | §9、§4.2 |
| 镜像从 NVIDIA 官方下载 | **必须向 HZHY/供应链索取适配镜像包**（网盘分发） | §5、§8 |
| 刷后可正常 apt 升级 | 厂商红字警告：**适配镜像不能执行 `apt upgrade`**（会覆盖厂商内核/设备树） | §12、§14 |
| 回滚需重刷官方 JetPack 5 | 厂商提供 `l4t_backup_restore.sh` **整盘备份/恢复流程**，回滚能力反而更强 | §7.2、§16 |

**HZHY 关键联系方式**：技术支持 `support@hzhytech.com`（2025 版教程）/ `sam@hzhytech.com`（资料 Readme），电话 010-62129511，官网 hzhytech.com。另注意本机装在 HighTorque 机器人内，载板/镜像的采购链路可能经由 HighTorque——索取镜像时两边都问。

## 3. 本机基线与本次目标

下表在 2026-08-23 通过只读 SSH 重新核对（`/etc/nv_tegra_release`、`/proc/device-tree`、`lsblk`、`ip`、`jetson_release`），正式刷机当天仍必须再查一次：

| 项目 | 2026-08-23 观察值 |
|---|---|
| 载板（实物核验） | **HZHY HYAI-311UAV**（丝印 `HYAI-311UAV_O_V12`，序列号 553722012670704）；**不是 NVIDIA 官方 P3768**，见 §2.2 |
| 软件报告的板型 | `nvidia,p3768-0000+p3767-0000`；dts 为 `tegra234-p3767-0000-p3768-0000-a0.dts` —— 这是厂商基于 devkit 配置构建镜像的产物，不代表物理载板 |
| 模块型号 | NVIDIA Jetson Orin NX 16 GB（`P3767-0000`，`aarch64`，约 15 GiB 可见 RAM）——模组本身为官方标准件 |
| 系统盘 | NVMe `SK900-128GB`，119.2 GiB；rootfs 为 `nvme0n1p15`（117.8 GiB ext4），**已用 58 G / 可用 52 G** |
| 当前平台 | **JetPack 5.1.4** / L4T 35.6.0 / Ubuntu 20.04.6 / Linux 5.10.216-tegra |
| 当前 ROS | 仅 ROS 1 Noetic（`/opt/ros/noetic`）；没有原生 ROS 2 Humble |
| 当前网络 | `eth0` 172.20.15.185/24（与刷机主机 172.20.15.30 同一 L2 网段）；`tailscale0` 100.124.42.119，`tailscaled` 已 enable |
| 当前 CAN | **`can0` 处于 UP、1 Mbit/s、`ERROR-PASSIVE`（tx 错误计数 128）** —— 总线已接线且正在报错 |
| 供应商栈 | `/opt/hightorque_pi_rl`、`/opt/ota_package`、`ota.service`、`livelybot-logger.service` 正在运行；另有 `/opt/RTL-8821CU` 树外 WiFi 驱动 |
| 目标平台 | JetPack 6.2.x / L4T 36.5.x / Ubuntu 22.04 / Linux 5.15-tegra（小版本见 [§2.1](#21-目标-jetpack-小版本的选择与版本禁区)） |

旧文本曾写"未发现第三方载板 BSP 标识，这是本次迁移难度较低的重要原因"——该判断依据的是软件字符串，已被 §2.2 的实物证据推翻：**载板是第三方的，必须使用厂商镜像与厂商刷机流程**。如果刷机当天的实物丝印、NVMe 布局与上述结果不符，应立即停止并重新核对。

**与旧版本记录的更正**：

1. 旧文本写“JetPack 5.1.5 / L4T 35.6.0”。实际 `/etc/nv_tegra_release` 为 `R35 (release), REVISION: 6.0`，`jetson_release` 报 **JetPack 5.1.4**；L4T 35.6.0 对应的是 5.1.4 而不是 5.1.5。以 **L4T 35.6.0** 为准，JetPack 小版本按此更正。同一处错误此前也存在于 [`01_evidence_and_research.md`](../planning/01_evidence_and_research.md)、[`04_source_register.md`](../planning/04_source_register.md) 和 [`05_decisions_and_open_questions.md`](../planning/05_decisions_and_open_questions.md)，已于 2026-08-23 一并更正。
2. 旧文本写“约 52 GB 已使用、59 GB 可用”。这是 2026-08-08 的值；生成 6.6 GB 预刷机归档后，当前为已用 58 G、可用 52 G。

## 4. 准备物品

### 4.1 推荐的刷机主机

- 原生 x86_64 Linux 电脑，管理员权限；
- 至少 8 GB RAM；建议预留 50 GB 以上磁盘空间（厂商镜像包解压后体积可观，且必须以 root 在主机本地盘解压）；
- 稳定网络（下载厂商网盘镜像包）。

**主机 Ubuntu 版本存在一个必须与厂商确认的冲突**：本地主机是 Ubuntu 22.04.5（硬件已核对通过：`x86_64`、31 GiB RAM、根分区可用约 457 GiB）。但 HZHY 2025-07-11 版教程仍红字要求"刷机 PC 的版本必须是 Ubuntu 20.04 的"。旁证显示有人用 Ubuntu 22.04 主机成功刷了 HZHY 的 JetPack 6.2 包，且 NVIDIA 官方对 L4T 36.x 支持 20.04/22.04 主机——但**索取镜像时必须向厂商确认 22.04 主机是否可用**；若厂商坚持 20.04，需准备一台 20.04 物理机或接受风险自担。普通 VirtualBox/VMware 虚拟机不推荐，因为刷写期间 Jetson 会多次 USB 重新枚举，USB 直通容易丢失。

不使用 SDK Manager 与 Windows/WSL2 路线（理由见 §2）。

### 4.2 现场物品

- 一根确认支持数据传输的 USB-C 数据线（接载板 **J7** 烧录口），优先直连主机，不经过 Hub；
- **12~36V 的 XT30 接口电源**（载板 J8；机器人整机供电或台架电源均可，功率按 Orin NX 25W 模式加裕量）；
- 能按到载板 **SW1（REC 按键）** 的物理条件——板子装在机器人机箱内，确认是否需要拆壳；
- Micro HDMI 转接线/显示器、USB 键盘（J17 是 **Micro HDMI**，不是标准 HDMI/DP）和有线网络；
- 一个容量足够、最好使用 ext4 或加密文件系统的备份盘/NAS；
- 至少半天不中断的维护窗口。

## 5. 刷机前的强制停点

以下条件全部满足后才可以执行刷写命令。右栏是 2026-08-23 的实际核对结果。

| # | 停点条件 | 状态（2026-08-23 第三次复核） |
|---|---|---|
| 1 | 已重新确认模块、**实物载板身份**、系统盘和当前 L4T 版本 | **已满足**：模组 P3767-0000 + HZHY HYAI-311UAV，见 §2.2/§3 |
| 2 | **已取得 HZHY 的 311UAV+Orin NX JetPack 6.2 适配镜像包**，确认版本/哈希，并确认不是 7.x | **已满足**：包已就位并验证（R36.4.3/JP6.2，哈希已登记），见 §2.1 |
| 3 | **已与厂商确认 Ubuntu 22.04 刷机主机可用**（或准备 20.04 主机） | **有依据但未经厂商书面确认**：外部旁证用 22.04 主机刷同名包成功，NVIDIA 官方对 L4T 36.x 支持 22.04 主机；本项目决定按 22.04 执行并接受此残余风险，失败时回退串口排查而不是换多个变量 |
| 4 | 已列清要保存的数据、配置、容器数据和未提交代码 | 已满足，归档 manifest 见 §7.3 |
| 5 | **备份已复制到 Jetson 之外，并通过 SHA-256 校验** | **已满足**：`tar.gz` + `manifest.txt` 各存两份于两块独立物理盘，四个哈希全部精确匹配，见 §7.4 |
| 6 | 已接受 NVMe 会被清空、旧系统不能原地恢复 | 需项目负责人书面确认，见 §5.1 |
| 7 | 刷机主机有足够空间，网络、电源和 USB 连接稳定 | **已满足**：镜像已解压至 `/opt/jetson-flash`（8.6 GB，`rootfs` 属主 `root:root`），刷机依赖 10 个包全装，`/usr/bin/python` 软链已建，首启账号已预配置；剩余 439 GiB |
| 8 | **现场可物理接触设备**：能按 SW1、接 J7/XT30、断电重启、看显示器（含必要的拆壳） | **未记录 —— 必须先确认**，见 §5.2 |
| 9 | **已规划刷机后的远程访问恢复方式** | 方案已写好（§12.2），执行留在现场完成 |
| 10 | 已记录旧系统的网络、服务和应用依赖，但不会把旧 `/etc` 整体覆盖到新系统 | 已满足，见 §7.3 与 §13 |
| 11 | **CAN、电机和其他执行器已物理断开或断能，并留有记录** | **未满足 —— `can0` 最近观察为 UP 且 `ERROR-PASSIVE`**，见 §5.2 |

**刷写命令真正开始写入之前，是最后一个无数据损失退出点。** 只要有一项是”未满足”，就停在这里。

按当前状态，剩余缺口为：**第 5 项收尾（manifest + 第二副本）、第 6 项（书面接受）、第 8 项（现场物理条件）、第 11 项（断能）**，另有 §7.3 的 Docker sudo 复核。镜像包这个硬阻塞已解除；剩余项都可以在执行日当天或前一天完成，具体见 §17.1 操作卡。

### 5.1 必须事先接受的不可逆后果

Direct Flash 会同时重写 QSPI 启动固件与 NVMe。以下内容不是“刷完再想办法恢复”，而是必须在动手前就接受的结果：

| 会失去什么 | 说明 | 是否可在新系统重建 |
|---|---|---|
| **供应商 ROS 1 Noetic 机器人栈** | `/opt/hightorque_pi_rl` 依赖 ROS 1 Noetic。**Noetic 没有 Ubuntu 22.04 官方版本**，Jammy 上无法原生安装 | **不能原生重建**；只能容器化、移植到 ROS 2，或放弃 |
| `ota.service` / `/opt/ota_package` | 供应商 OTA 客户端，绑定旧系统布局 | 需向供应商确认是否有 JetPack 6 版本 |
| `livelybot-logger.service` | 供应商日志服务与 `/var/lib/livelybot_logger` | 同上 |
| `/opt/RTL-8821CU` 树外 WiFi 驱动 | 为 5.10-tegra 编译；内核换成 5.15 后**必须重新编译** | 可重建，但需提前准备源码 |
| Tailscale 组网身份 | 新系统没有 Tailscale，节点需重新安装并重新认证 | 可重建，但需要一次本地操作 |
| SSH host key、用户 SSH 授权、账号 | OEM 配置会重建账号；`~/.ssh` 与 host key 全部重置 | 可重建 |
| CUDA 11.4 / TensorRT 8.5 等旧计算栈 | JetPack 6 提供 CUDA 12.6 / TensorRT 10.3 | 版本必然变化，依赖旧 API 的程序需要改 |

**最重要的一条**：如果这台工控机现在还承担着供应商机器人栈的实际运行任务，那么这次升级等于让它暂时**丧失运行该机器人的能力**。这是项目层面的取舍，必须由项目负责人明确决定并记录，不能默认“反正有备份”。

### 5.2 两个当前未闭合的现场前提

**（a）物理接触**：本文 §1 已把“只能通过 SSH 远程接触 Jetson”列为不建议独自操作的情形。当前事实是：设备与刷机主机在同一 L2 网段（172.20.15.0/24），同时也可经 Tailscale 访问，但**没有任何记录证明操作者能物理接触该设备**。刷机必须在现场完成——进入恢复模式需要按住 SW1 并断复电（§9），首启配置需要显示器和键盘。板子装在机器人机箱内（照片可见警示贴纸），SW1/J7/XT30 是否可直接触及、是否需要拆壳，必须提前确认。开始前必须确认并记录：

- [ ] 操作者本人可以接触到这台工控机，SW1 按键、J7 Type-C 口、XT30 电源可触及（或已明确拆壳步骤）；
- [ ] 有可用的 Micro HDMI 显示器 + USB 键盘（或已确认 J6 串口控制台方案）；
- [ ] 有可靠的 USB-C 数据线和 12~36V XT30 电源；
- [ ] 刷机主机可以搬到设备旁，或设备可以搬到刷机主机旁。

**（b）CAN 与执行器断能**：2026-08-23 观察到 `can0` 处于 `UP`、1 Mbit/s、`ERROR-PASSIVE`（tx 错误计数 128）。这说明总线已接线且正在发送失败——通常意味着对端无应答、终端电阻或接线存在问题。刷机过程要反复断电、上电并进入恢复模式，期间驱动器可能上电而主机无法发送命令。因此在断开 DC 之前必须：

- [ ] 物理断开电机与执行器动力电（不是只关软件）；
- [ ] 物理断开或确认 CAN 线束不会驱动任何执行器；
- [ ] 记录断能前后的 `ip -d -s link show can0` 输出作为证据。

这一步同时属于本项目 G0–G3 闸门的安全边界，不能因为“只是刷机”而跳过。

## 6. 第一步：重新盘点旧系统

在旧 Jetson 上运行以下只读命令，并把结果保存在 Jetson 之外。输出可能包含主机名、用户名、磁盘序列号或网络信息，不能提交到仓库或公开发出。

```bash
cat /etc/os-release
head -n 1 /etc/nv_tegra_release
uname -a
cat /proc/device-tree/model
tr '\0' '\n' </proc/device-tree/compatible
lsblk -o NAME,MODEL,SIZE,FSTYPE,MOUNTPOINTS
df -hT
dpkg-query -W 'nvidia-l4t-*' 'nvidia-jetpack' 2>/dev/null
systemctl --failed
```

另外手工记录：

- `/home` 下真正需要保留的目录；
- 每个 Git 仓库的 remote、branch、commit、`git status` 和未提交 diff；
- systemd 服务、udev 规则、网络配置、防火墙和定时任务；
- Docker compose 文件、bind mount 和 named volume 的实际数据位置；
- 应用许可证、模型、标定文件、日志和数据库；
- 当前 SSH 公钥授权方式和 GitHub 私有仓库访问方式，但不记录私钥或密码值。

## 7. 第二步：备份并验证

### 7.1 最低备份范围

至少备份：

- 需要保留的用户目录和项目仓库；
- 未提交修改，最好同时保存 `git bundle` 或远端分支；
- 应用数据、数据库、标定结果和模型；
- `/etc/systemd/system`、`/etc/udev/rules.d` 等自定义配置的参考副本；
- 网络和容器配置；
- 使用应用自身导出机制生成的 Docker volume/数据库备份。

`/etc/NetworkManager/system-connections`、SSH 私钥、证书和应用密钥可能含秘密，只能进入受控、最好加密的备份介质，不得进入 Git 或普通聊天记录。

如果备份盘支持 Linux 权限和扩展属性，可按实际挂载点使用类似命令：

```bash
sudo rsync -aHAX --numeric-ids --info=progress2 \
  /home/ /mnt/jetson-backup/before-jetpack6/home/

sudo rsync -aHAX --numeric-ids \
  /etc/ /mnt/jetson-backup/before-jetpack6/etc-reference/

sync
```

上面的 `/etc` 只用于迁移时逐项对照，**刷机后不得整体恢复**。如果备份介质是 NTFS/exFAT，应使用能保留 owner、权限、ACL 和 xattr 的归档方式，或者把关键应用改用自身导出格式。

至少做一次校验式 dry run，并随机打开关键文件：

```bash
sudo rsync -aHAXcn --numeric-ids \
  /home/ /mnt/jetson-backup/before-jetpack6/home/
```

没有输出通常表示已复制内容一致；仍应额外检查数据库导出、未提交代码和最关键配置能否实际读取。

不要在 Docker 正运行且应用仍写数据库时直接复制整个 `/var/lib/docker`。应先按应用流程停止写入并导出数据，刷后重新创建容器。旧系统上的二进制、Python venv、ROS workspace 的 `build/install/log` 也不应直接搬到 Ubuntu 22.04 使用。

### 7.2 更稳但更麻烦的选择：厂商整盘镜像备份

如果旧环境必须可以完整复原，**HZHY 教程直接提供了整盘备份/恢复流程**（JP5.1.1 版教程 §1.4/§1.5），比通用 NVIDIA Backup and Restore 更贴合本载板：

```bash
# 设备进入恢复模式（§9），在与当前系统匹配的 JP5.x 厂商镜像包 Linux_for_Tegra 下：
sudo ./tools/backup_restore/l4t_backup_restore.sh -b p3509-a02+p3767-0000   # 备份
sudo ./tools/backup_restore/l4t_backup_restore.sh -r p3509-a02+p3767-0000   # 恢复
```

备份映像保存在 `Linux_for_Tegra/tools/backup_restore/images/`（整盘约数十 GB，主机需预留足够空间）。注意：备份期间主机上只能有一台恢复模式设备、不可插入其他存储设备；该流程需要**与当前系统版本匹配的厂商 JP5.x 刷机包**——若决定做整盘备份，向厂商索取镜像时把 JP5.x 包一并要来。这比文件备份复杂，适合把旧系统当作业务资产的情况；对本机而言，它同时构成 §16 回滚路径的基础。

换一块空白 NVMe 再刷可以保护原 NVMe 上的数据，但不能承诺“插回旧盘立刻回滚”，因为刷写还会升级 QSPI/UEFI。真正回到 JetPack 5 仍可能需要用匹配版本重新刷写启动固件。

### 7.3 本次目标机已生成的预刷机归档

2026-08-23 在目标机上完成了只读盘点，并在不停止运行服务、不修改 CAN 状态的情况下生成以下归档：

```text
/home/nvidia/jetson-preflash-backup-20260823.tar.gz
/home/nvidia/jetson-preflash-backup-20260823.manifest.txt
```

实际校验结果：

| 项目 | 结果 |
|---|---|
| 压缩包大小 | `6,659,984,614` bytes |
| 压缩包目录条目 | `115,583` |
| 压缩包 SHA-256 | `4013f0bdcd5a6c5864b768fa76e8dd4050d56fc36e5f467cb1eb0eabba16fe28` |
| manifest SHA-256 | `ba7424ba9c715b3f1a82062a960351053eb2741b2330caec018f4324d144d580` |

归档包含：

- `/home/nvidia` 用户目录、源代码、下载物、未提交 Git 修改、SSH 用户配置和本地工具配置；
- `/opt/hightorque_pi_rl`（排除 `build/`、`devel/`、`logs/`）和 `/opt/ota_package`；
- `/var/lib/livelybot_logger`（排除运行日志目录）及 `/usr/local/bin/logger_service`；
- `/etc/rc.local`、自定义 systemd、udev、NetworkManager、APT 和 SSH 配置参考；
- 平台、磁盘、已安装软件包、启用服务和归档路径清单。

为控制体积和避免保存不一致的生成物，归档排除了 `.cache/`、`.vscode-server/`、回收站、ROS 日志以及名称为 `build/`、`install/`、`devel/`、`logs/` 的目录。

**关于 Docker —— 已复核结案（2026-08-23）**：早先以普通用户查询时因不在 `docker` 组返回 `permission denied`，该结论一度无法验证。已用 sudo 重新执行并确认：

```bash
ssh -t nvidia@172.20.15.185 "sudo docker ps -a; sudo docker volume ls"
# 两条命令均只返回表头，无任何容器、无任何 named volume
```

（注意必须用 `ssh -t` 分配 TTY，否则 `sudo` 报 `a terminal is required to read the password`。）因此**确认没有 Docker 侧数据需要额外导出**，归档范围完整。

这个归档可能包含 SSH 用户配置、NetworkManager 连接信息、`.npmrc` 或其他敏感内容。复制到外部介质后应加密或严格限制权限，不要上传到公共位置。复制完成后，在备份电脑上核对：

```bash
sha256sum jetson-preflash-backup-20260823.tar.gz
# 应得到：
# 4013f0bdcd5a6c5864b768fa76e8dd4050d56fc36e5f467cb1eb0eabba16fe28

tar -tzf jetson-preflash-backup-20260823.tar.gz >/dev/null
```

只有在外部副本已读取并通过 SHA-256 校验后，才可由操作者决定是否删除工控机上的归档。旧系统的 `rc.local`、systemd、udev、网络和 APT 配置只能作为迁移参考，不能在 JetPack 6 上整体覆盖恢复；尤其当前 `can0` 在盘点时为 1 Mbps、`ERROR-PASSIVE`，刷机前必须物理断开 CAN 和执行器。

### 7.4 归档离机进度（2026-08-23 第三次复核更新）

**本项已于 2026-08-23 完成并验证。**归档的两个文件均已离机，存放在两个互相独立的物理磁盘上，SHA-256 全部精确匹配 §7.3 的期望值：

| 位置 | 磁盘 | 文件 | 校验 |
|---|---|---|---|
| `/home/admin2025/` | 刷机主机系统盘 `nvme1n1` | `...tar.gz` + `...manifest.txt` | 两者哈希均精确匹配 ✓ |
| `/media/admin2025/新加卷/jetson-backup/` | 第二块盘 `nvme0n1`（NTFS） | 同上 | 两者哈希均精确匹配 ✓ |

第二份存放于 NTFS 卷不影响 `.tar.gz`（属主/权限信息封装在 tar 内部）；只要不把解开后的目录树直接放 NTFS 即可。工控机上的原件暂时保留，待刷机成功且数据恢复验证后再由操作者决定是否删除。

以下为原始复核记录与完整校验流程，供追溯；要点是至少放到**两个互相独立的位置**：

```bash
# 1) 从刷机主机拉取（同网段直连比走 Tailscale 快得多）
mkdir -p ~/jetson-backup/before-jetpack6
scp nvidia@172.20.15.185:/home/nvidia/jetson-preflash-backup-20260823.tar.gz \
    nvidia@172.20.15.185:/home/nvidia/jetson-preflash-backup-20260823.manifest.txt \
    ~/jetson-backup/before-jetpack6/

# 2) 校验大小与哈希（必须与下列期望值完全一致）
cd ~/jetson-backup/before-jetpack6
stat -c '%n %s' jetson-preflash-backup-20260823.tar.gz
sha256sum jetson-preflash-backup-20260823.tar.gz jetson-preflash-backup-20260823.manifest.txt

# 3) 验证归档结构完整、可解开
tar -tzf jetson-preflash-backup-20260823.tar.gz >/dev/null && echo "ARCHIVE OK"

# 4) 抽查：先列出真实条目，再任选一个非目录条目实际解出内容
#    （归档内部是相对路径，如 home/nvidia/...，没有前导 /）
tar -tzf jetson-preflash-backup-20260823.tar.gz | head -20
tar -xzOf jetson-preflash-backup-20260823.tar.gz '<上面列出的某个文件路径>' | head
```

期望值：

| 文件 | 大小（bytes） | SHA-256 |
|---|---:|---|
| `...tar.gz` | `6659984614` | `4013f0bdcd5a6c5864b768fa76e8dd4050d56fc36e5f467cb1eb0eabba16fe28` |
| `...manifest.txt` | `24705789` | `ba7424ba9c715b3f1a82062a960351053eb2741b2330caec018f4324d144d580` |

补充注意事项：

- 归档由 `root` 生成、模式 `644`，`nvidia` 账号可读，因此用普通账号 `scp` 即可，不需要在工控机上提权。
- 复制走同网段 `172.20.15.185` 而不是 Tailscale 地址，6.6 GB 会快很多。
- 第二个副本所在介质若为 NTFS/exFAT（例如本机的 `/media/admin2025/新加卷`），对 `.tar.gz` 本身没有影响——权限信息保存在 tar 内部。但**不要**把解开后的目录树直接存到 NTFS，那会丢失属主与权限。
- 归档可能包含 SSH 配置、NetworkManager 连接信息等敏感内容。放到外部介质后应加密或收紧权限，不要上传到公共位置，也不要提交进 Git。
- 只有两份外部副本都校验通过后，才允许执行 §9 起的现场刷机步骤；§8 的镜像索取不接触设备，可并行提前进行。

## 8. 第三步：获取厂商镜像包并准备刷机环境

**（本节 2026-08-23 重写：原 SDK Manager 流程不适用于 HZHY 载板，见 §2.2。）**

1. **索取镜像包 —— 已完成（2026-08-23）**。已获取 `flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210.tar.gz`，存放于本仓库被忽略的 `company/` 目录，身份/版本/工具齐备性验证结果见 §2.1。若未来需要重新获取或获取其他版本（如回滚用的 JP5.x 包），渠道为：购买网盘链接的"软件设计/烧写镜像"目录、HZHY 技术支持（`support@hzhytech.com` / `sam@hzhytech.com`，010-62129511）、或 HighTorque 集成渠道。
2. **记录镜像包哈希 —— 已完成**：3,742,482,561 bytes，SHA-256 `7b2227dcb22e1ebcc8a868e629308b896ff3bd01761da647035e4e0cf6d1429e`，已登记入 `manifests/assets.yaml`。
3. 在刷机主机上以 **root** 解压到主机本地盘（厂商红字要求：必须先拷到主机本地盘、必须命令行解压、必须 root，否则刷写会失败）。解压后目录树约 15 GB，本机 457 GB 可用空间充足：

   ```bash
   sudo mkdir -p /opt/jetson-flash && cd /opt/jetson-flash
   sudo tar -xzf /home/admin2025/jetson_mech_control/company/flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210.tar.gz
   cd flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210/Linux_for_Tegra
   ```

4. 首次在这台主机刷 NVIDIA 板卡时，补全刷机依赖：

   ```bash
   sudo ./tools/l4t_flash_prerequisites.sh
   ```

   若后续刷写报 `python: No such file or directory`，按厂商教程补 `sudo ln -s /usr/bin/python3 /usr/bin/python`。

5. **预配置首启账号（推荐执行）**，跳过 oem-config 界面——对装在机器人里、显示器不便的设备尤其有用：

   ```bash
   sudo ./tools/l4t_create_default_user.sh -u nvidia -p <口令> -n nvidia-desktop -a --accept-license
   ```

   不要在共享记录中保留真实口令；刷后第一时间改口令并配置公钥登录（§12.2）。

## 9. 第四步：让 Jetson 进入恢复模式（HZHY 311UAV 流程）

**（本节 2026-08-23 重写：本载板没有官方载板的 J14 跳线 header；恢复模式由 SW1 按键触发，烧录口是 J7 Type-C。依据：HZHY 用户手册 §5.2 与刷机教程 §1.2/§1.3。）**

1. 正常关闭系统，**断开 XT30（J8）供电**并等待完全断电；
2. 用可靠 USB-C 数据线连接载板 **J7（REC 烧录口）** 与刷机主机——注意 J7 在载板装模组的那一面（板边），装在机器人机箱内时确认可触及，必要时先拆壳；
3. **按住载板 `SW1`（REC 按键）**，然后接通 XT30 电源；**上电后保持按住 3~4 秒再松手**；
4. 在刷机主机上确认进入恢复模式：

   ```bash
   lsusb | grep -i 0955
   ```

   对本机 Orin NX 16GB（P3767-0000），必须看到 **`0955:7323`**。厂商教程明确：如果 ID 是其他数值，说明系统已正常启动而**没有**进入恢复模式——断电后重新按住 SW1 再试；如果完全没有 NVIDIA 设备，排查数据线（须支持数据传输）、USB 口、供电，避免经过 Hub；
5. 恢复模式下屏幕无输出、SSH 不可用，这是正常现象。若反复失败，可接载板 **J6（DEBUG，TTL GND/RX/TX）** 串口看启动输出定位问题，不要盲目重试刷写。

## 10. 第五步：执行刷写命令（厂商流程）

**（本节 2026-08-23 重写，替代原 SDK Manager 选择/刷写两节。）**

刷写命令以**镜像包内附带的 README/厂商教程为准**。已对照实际镜像包验证：`jetson-orin-nano-devkit-super-nvme.conf`、`tools/kernel_flash/flash_l4t_t234_nvme.xml`、`bootloader/generic/cfg/flash_t234_qspi.xml` 均存在，且包内 `initrdlog/` 的厂商实测日志与该流程一致。HZHY 2025-07-11 版教程给出的 Orin NX/Nano JetPack 6.2（Super 模式）命令为：

```bash
cd Linux_for_Tegra
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --showlogs --network usb0 jetson-orin-nano-devkit-super-nvme internal
```

（JetPack 6.x 非 Super 模式则为 `flash_l4t_external.xml` + `jetson-orin-nano-devkit-nvme`，作为下述守卫触发时的回退。）

**板型配置为何适用于本机（2026-08-23 逐层核实，这是刷机第一大风险点）**：

1. 配置名里的 "orin-nano" 只是 NVIDIA 的载板家族命名。`jetson-orin-nano-devkit-super-nvme.conf` 第一行即 `source p3768-0000-p3767-0000-super.conf`，指向 **`p3767-0000`＝ Orin NX 16GB**，与本机模组一致；
2. 该配置**自动读取模组 EEPROM 的 `board_sku` 来选 DTB**，不写死。本机实测 `/proc/device-tree/chosen/ids` = **`3767-0000-303`**、`nvidia,sku` = `699-13767-0000-303 F.1`，即 SKU `0000`、FAB `303`，因此会选中 `tegra234-p3768-0000+p3767-0000-nv-super.dtb` 与 `tegra234-bpmp-3767-0000-3768-super.dtb`；两者均已确认存在于包内（分别在 `kernel/dtb/` 与 `bootloader/generic/`）；
3. **Super 模式守卫**：配置中有 `if board_FAB = TS1 或 EB1 → Error: can't be used for super mode; exit 1`。本机 FAB 为 `303`（量产件），**守卫通过**。若将来换成工程样品模组而刷写在此处直接退出，改用非 Super 配置：把命令里的 `flash_l4t_t234_nvme.xml` 换成 `flash_l4t_external.xml`、`jetson-orin-nano-devkit-super-nvme` 换成 `jetson-orin-nano-devkit-nvme`；
4. 包内厂商测试日志显示的是 `p3767-0003`（Orin Nano 8GB），那是厂商当时的测试样机，**不影响本机**——因为 DTB 由 SKU 自动选择。

执行时遵守：

1. 执行前再检查一次外部备份（§7.4）与恢复模式状态（`0955:7323`）；
2. 必须 root、镜像必须已在主机本地盘解压（§8 第 3 条）；
3. **刷写过程不要碰 Type-C 线、板子或机箱**（厂商红字），不让刷机主机休眠；
4. 观察终端输出直到出现 `Flash is successful` / `Flashing success`；日志在 `Linux_for_Tegra/initrdlog/` 下，保存归档；
5. 如果失败，保留日志、记录失败阶段，按 §15 排查后再重试；不要连续更换多个变量；
6. 成功后断电，**不按 SW1** 正常上电，进入首启。

第一次写 QSPI 或第一次启动可能比普通重启久。只要终端仍在输出有效进度，就不要因几分钟没有画面而强制断电。

## 11. 第六步：安装 JetPack 组件（可选）

系统首启并通过 §12 验收后，如需 GPU 栈（CUDA/TensorRT 等），按厂商教程第 2 章在**设备上**安装：

```bash
sudo apt update
sudo apt install nvidia-jetpack
```

当前 ROS/CAN Foundation 不依赖 CUDA，此步可以推迟；先完成 §12～§14 更重要。注意只执行 `apt update` + `apt install`，**不执行 `apt upgrade`**（§12）。

## 12. 第七步：首启验收

先不要恢复旧服务、Docker 或 CAN 配置。在全新的系统上记录以下基线：

```bash
uname -m
cat /etc/os-release
head -n 1 /etc/nv_tegra_release
uname -r
findmnt -no SOURCE,FSTYPE /
lsblk -o NAME,MODEL,SIZE,FSTYPE,MOUNTPOINTS
dpkg-query -W nvidia-l4t-core nvidia-l4t-kernel 2>/dev/null
dpkg-query -W nvidia-jetpack 2>/dev/null
dpkg --audit
systemctl --failed
timedatectl status
```

### 12.0 本机实际执行结果（2026-08-23，刷写成功）

刷写于 2026-08-23 15:10 开始、约 15:26 完成（QSPI 约 4 分钟，NVMe rootfs 经 USB 网络传输约 12 分钟，峰值约 3.6 MB/s，累计传输约 8.6 GB）。终端结尾出现 `Successfully flash the external device` → `Flashing success` → `Flash is successful` → `Reboot device`。日志归档于刷机主机 `~/flash-log-SUCCESS-20260823-*/flash_1-11.3_0_20260823-151024.log`。

首启验收实测：

| 项 | 实测值 | 判定 |
|---|---|---|
| `/etc/nv_tegra_release` | `R36 (release), REVISION: 4.3` | ✅ JetPack 6.2 |
| `/etc/os-release` | `Ubuntu 22.04.5 LTS` | ✅ **迁移目标达成** |
| `uname -r` | `5.15.148-tegra` | ✅ |
| `uname -m` | `aarch64` | ✅ |
| rootfs | `/dev/nvme0n1p1` ext4，116 GiB 总 / 103 GiB 可用 | ✅ 已用满整盘 |
| 模组 | `699-13767-0000-303 F.1` | ✅ 与刷前一致 |
| `dpkg --audit` | 无输出 | ✅ |
| `systemctl --failed` | 仅 `nvgetty.service`（ttyTHS0 串口登录） | ⚠️ 本项目未使用 J6 调试串口，禁用即可 |

两条刷写过程中出现、看似告警但实为正常的信息，记录以免后续误判：

1. `Error: The backup GPT table is corrupt, but the primary appears OK` —— 读取**旧** NVMe 分区表时的提示，该表随后即被覆盖；
2. `The device size indicated in the partition layout xml is smaller than the actual size. This utility will try to fix the GPT.` —— 工具自动把 GPT 扩展到实际盘容量，正是 rootfs 能用满 116 GiB 的原因。

另记录一条对排障有用的观察：进入 `--network usb0` 阶段后，USB 设备 ID 会从恢复模式的 `0955:7323` 变为 **`0955:7035`（Linux for Tegra）**，主机侧出现 `enx*` USB 网络接口并 SSH 进设备执行 `nv_flash_from_network.sh`。此阶段**终端可能长时间无输出**（进度在设备侧产生），`initrdlog` 也停在 QSPI 结束处。判断是否卡死应看 USB 网口流量而非屏幕：

```bash
IF=$(ip -br addr | awk '/^enx/{print $1}')
r1=$(cat /sys/class/net/$IF/statistics/tx_bytes); sleep 15
r2=$(cat /sys/class/net/$IF/statistics/tx_bytes)
echo "$(( (r2-r1)/1024/1024 )) MB / 15s"   # 有持续流量即正常
```

### 12.1 通用验收清单

期望结果：

- `uname -m` 为 `aarch64`；
- Ubuntu 为 **22.04**（若显示 24.04，说明误刷了 JetPack 7.x，必须重刷）；
- `/etc/nv_tegra_release` 为 **R36**，且 revision 与 §2.1 中实际选定的 JetPack 小版本对应；
- 内核为 NVIDIA `5.15...-tegra`；
- 根文件系统来自 NVMe 且为正常 Linux 文件系统；
- **rootfs 分区容量接近整盘**（旧系统为 117.8 GiB）。厂商教程特别提醒镜像与 SSD 容量的适配问题（恢复镜像方式按备份时的盘布局展开；`l4t_initrd_flash.sh` 长命令则按实际盘分配）；用 `lsblk` / `df -h /` 确认，容量明显偏小就要在这一步处理，不要等装完 ROS 再发现；
- `dpkg --audit` 没有损坏包；
- 时间同步、网络和重启正常；
- 只有安装了完整/运行时 JetPack meta package 时才强制要求查询到 `nvidia-jetpack`，不要把“只刷 Jetson OS”误判成刷写失败。

**不要把版本号当成硬性等值判断**：JetPack 与 L4T 的对应关系在小版本上存在歧义（例如社区中 JetPack 6.2.1 同时对应过 `36.4.4` 与 rev1 的 `36.4.7`），`jetson_release` 这类第三方工具的映射表也可能滞后。**以 `/etc/nv_tegra_release` 的 R36 与 revision 为准**，把实际值记录下来即可，不要因为和文档示例值不同就判定刷写失败。同理，刷厂商镜像后 `device-tree/model`、compatible 仍会显示 devkit 字符串（`p3768-0000+p3767-0000` 等）——这是厂商基于 devkit 配置构建的正常现象（§2.2），不是刷错板型。

**系统更新纪律（厂商适配镜像专属约束）**：HZHY 红字警告适配镜像**不能执行 `apt upgrade`**——全量升级可能拉入官方 `nvidia-l4t-kernel` 等包，覆盖厂商修改过的内核/设备树。因此：

- 允许：`apt update`、`apt install <具体包>`（如 ROS 2、构建工具、`nvidia-jetpack`）；
- 禁止：`apt upgrade` / `apt dist-upgrade` / 无人值守升级。首启验收通过后立即执行两道保险：

  ```bash
  # 锁住全部 L4T 系统包，防止误升级覆盖厂商内核/设备树
  dpkg-query -Wf '${Package}\n' 'nvidia-l4t-*' | xargs sudo apt-mark hold
  # 关闭无人值守升级
  sudo dpkg-reconfigure -plow unattended-upgrades   # 选 No
  ```

- 之后若某次 `apt install` 提示要连带升级 `nvidia-l4t-*` 包，中止并单独评估；
- 一如既往禁止 `do-release-upgrade` 和手工把 APT 源指向另一个 L4T 大版本；
- 向厂商确认其推荐的安全补丁更新方式，把答复记入证据。

### 12.2 恢复远程访问（必须在离开现场前完成）

刷完之后，这台工控机在网络上等于一台全新机器。**旧的 Tailscale 节点身份、SSH host key、`~/.ssh/authorized_keys` 和用户账号全部不存在了。** 如果不在现场把远程访问恢复好就离开，后续任何操作都需要再跑一趟。

在现场用显示器/键盘完成：

1. 确认有线网络已连上，记录新的 `eth0` 地址（旧值为 `172.20.15.185`，DHCP 下可能变化）；
2. 安装并启动 SSH 服务，确认可从刷机主机以 `172.20.15.x` 直连登录；
3. 在 Jetson 本地读取新的 host key 指纹，再到客户端清除旧记录：

   ```bash
   # 在 Jetson 本地
   ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub

   # 在客户端，核对指纹一致后再执行
   ssh-keygen -R 172.20.15.185
   ssh-keygen -R 100.124.42.119
   ```

   不要为了消除 host-key 警告直接关闭 `StrictHostKeyChecking`；
4. 重新安装 Tailscale 并 `tailscale up` 完成认证。旧节点 `ubuntu / 100.124.42.119` 需要在 Tailscale 管理端删除或允许覆盖，新节点很可能拿到不同的 100.x 地址；
5. 配置 SSH 公钥登录，避免后续继续使用口令登录；
6. 只有上述都验证通过后，才可以离开现场继续远程操作。

## 13. 第八步：选择性恢复

恢复顺序建议为：

1. 用户数据和项目源码；
2. SSH 公钥授权和私有仓库的最小只读访问；
3. 构建工具和 ROS 2；
4. 逐个重建 systemd 服务、udev 规则和容器；
5. 最后才恢复会接触硬件的部署配置。

以下旧内容不要直接覆盖到新系统：

- `/boot`、`/lib/modules`、NVIDIA 驱动和 device tree；
- `/usr`、`/var/lib/dpkg` 和旧 APT 源；
- 整个旧 `/etc`；
- `/var/lib/docker` 的原始目录；
- Focal/ROS 1 下生成的二进制、Python venv、`build/install/log`；
- 旧版网络和设备规则中未经逐项审查的内容。

每恢复一项服务，就重启并查看该服务日志，避免一次性恢复几十项后无法定位问题。

## 14. 第九步：安装 ROS 2 Humble 并进入项目烟测

JetPack 6.2.x 提供 Ubuntu 22.04 和 NVIDIA BSP，但不会自动满足本项目的 ROS 2 Humble 开发环境。

**两条可用路线**：§14.1 是本机 2026-08-23 实际执行的鱼香ROS 一键脚本路线；§14.2 是 ROS 官方 `ros2-apt-source` 机制。**新机器优先用 §14.2**（来源可审计、key 作用域受限）；§14.1 保留是因为它是本机既成事实，且国内网络下更快。

### 14.0 本机实际执行结果（2026-08-23）

| 项 | 实测值 |
|---|---|
| ROS | `/opt/ros/humble`，`ros-humble-ros-base 0.10.0-1jammy`，284 个 `ros-humble-*` 包 |
| ROS 源 | `/etc/apt/sources.list.d/ros-fish.list` → `http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ jammy main` |
| rosdep | `rosdepc 0.26.0`（`/usr/local/bin/rosdepc`）与 `/usr/bin/rosdep` 并存 |
| 工具链 | colcon、python3-rosdep、build-essential 均已装；**gcc 11.4.0 / cmake 3.22.1 / git 2.34.1** |
| `ros2-apt-source` | **未安装**——本机没有走 §14.2 |
| 系统源 | 装 ROS 前已手工换为清华 `ubuntu-ports`（只改 `sources.list`，**未动** `sources.list.d`） |

装后安全复核全部通过：`nvidia-l4t-apt-source.list` 仍在、45 个 `nvidia-l4t-*` hold 与换源前快照逐行一致、无 L4T 包被改动、无 `99verify-peer.conf`、`systemctl --failed` 为空。回滚快照位于目标机 `/var/backups/pre-fishros-20260823/`。

**一个已知瑕疵（不影响烟测）**：fishros 把 ROS 仓库 key 落进了**已废弃的全局 `/etc/apt/trusted.gpg`**，同时留下一个 **0 字节**的 `/etc/apt/trusted.gpg.d/ros.gpg`。因此每次 `apt update` 都会告警 `Key is stored in legacy trusted.gpg keyring`。后果是该 key 对**所有**仓库生效，而非仅限 ROS 源，作用域弱于 `signed-by=`。若要收口：按 §14.2 装 `ros2-apt-source`，然后删掉那个 0 字节文件并从 `/etc/apt/trusted.gpg` 中移除该 key。

### 14.1 鱼香ROS 路线（本机实际走法）

⚠️ **该脚本经过源码审计（2026-08-23，`install` sha256 `10cbba5e…`、`install.py` `a5e7d6bc…`）。脚本每次运行现拉，结论只对该版本成立。** 审计确认它**不执行** `apt upgrade`，但存在下列必须规避的行为。

```bash
wget http://fishros.com/install -O fishros && . fishros
```

| 提示 | 必选 | 理由 |
|---|---|---|
| 主菜单 | `1` | 一键安装 ROS |
| `====接下来这一步很很很很重要，如果不知道怎么选请选择1====` | **`2` 不更换继续安装** | **必须违背脚本自身建议。** 选 1 会进入换源分支 |
| 镜像源 | `tsinghua` | 与系统源一致 |
| ROS 版本 | `humble` | 注意区分 ROS1／ROS2 |
| 具体版本 | `2` 基础版 | 工控机无显示器；ros2_control 等依赖交由 rosdep 解析 |

**🚨 最高危分支**：若上表第二行误选了 `1`，紧接着会出现「`请选择换源方式,如果不知道选什么请选2`」。**此处必须选 `1`（仅更换系统源）。** 选 `2` 会执行 `sudo rm -rf /etc/apt/sources.list.d`，把 `nvidia-l4t-apt-source.list` 连同其它第三方源一并删除——那 45 个 hold 住的 L4T 包将就此失去候选版本来源。

其它两条审计结论：脚本主体经**明文 HTTP** 从 `mirror.fishros.com` 拉取并以 root 执行，无签名校验；`AptUtils.checkapt()` 一旦遇到证书错误会**永久写入** `/etc/apt/apt.conf.d/99verify-peer.conf` 关闭 apt 的 TLS 校验。跑完务必复核这两处。

### 14.2 官方 `ros2-apt-source` 路线（新机器推荐）

按 ROS 官方 Ubuntu deb 安装页配置 Jammy 的 ROS 2 APT 源。**注意官方做法在 2025 年已经变了**：先启用 universe 源，再安装 `ros2-apt-source` 这个 deb 包，由它提供 ROS 仓库配置与签名 key；今后 key 轮换会随普通升级自动完成。

网上绝大多数教程仍在教旧做法——手工把 `ros.key` 下到 `/usr/share/keyrings/ros-archive-keyring.gpg` 再写 `signed-by=` 源。**不要照抄**：旧 key 已经过期，照抄会直接撞上 `EXPKEYSIG F42ED6FBAB17C654` 而导致仓库不可用。刷机后的系统是干净的，不存在遗留的 `ros2.list`，直接用新机制即可。

### 14.3 目标包、验证与烟测

无论走哪条路线，§12 的更新纪律不变：只做 `apt update` 和 `apt install`，**不要执行教程里常见的 `apt upgrade`**——厂商适配镜像禁止全量升级。

1. 至少确保以下包就位：

   ```text
   ros-humble-ros-base
   python3-colcon-common-extensions
   python3-rosdep
   build-essential
   cmake
   git
   ```

2. 验证 `/opt/ros/humble/setup.bash`、`ros2`、`colcon` 和 `rosdep`（本机为 `rosdepc`）；
3. 初始化/更新 rosdep；
4. 配置私有仓库的 repository-scoped read-only deploy key 或其他已批准的只读方式；
5. 重新运行 FND-004A admission inventory；
6. 严格按 [FND-004A Jetson ARM64 原生烟测](jetson_arm64_smoke_test.md)执行 clean clone、context check、rosdep 和五包 build/test，并记录实际 JetPack／L4T／GCC／CMake 版本。

FND-004A 仍不启用 CAN，不打开设备，不调整实时调度。Docker 是否作为最终运行方式，应在原生平台烟测之后另行决定；升级宿主和使用容器并不冲突。

## 15. 常见故障及处理

| 现象 | 优先检查 | 不要做什么 |
|---|---|---|
| `lsusb` 无 NVIDIA 设备 | 数据线是否支持数据传输、是否直连（无 Hub）、J7 接触、XT30 供电 | 不要盲目重试刷写命令 |
| `lsusb` 有 NVIDIA 但不是 `0955:7323` | 系统已正常启动、未进恢复模式：断电后**按住 SW1** 再上电 3~4 秒（厂商教程备注） | 不要在非恢复模式下执行刷写 |
| 刷写命令报错/中途失败 | 保留 `initrdlog/` 日志；核对 root 解压、本地盘解压、`l4t_flash_prerequisites.sh` 已执行、NVMe 是否有 `nvme0n1` 分区节点 | 不要连续更换多个变量；不要把失败直接判断为永久变砖 |
| 报 `python: No such file or directory` | 厂商教程 §1.2.2：`sudo ln -s /usr/bin/python3 /usr/bin/python` | — |
| 刷完找不到 NVMe rootfs | 刷写日志、NVMe 分区（必要时按厂商《将系统安装至扩展盘》2.1 节重新分区格式化后重试） | 不要改未知 partition XML 试错 |
| 黑屏或启动循环 | J6 串口/显示器日志、镜像与模组版本是否匹配；重新进恢复模式刷写 | 不要运行普通 Ubuntu boot repair 覆盖 Jetson 启动链 |
| SSH 报 host key 变化 | 本地核对新系统指纹后更新客户端记录 | 不要关闭 StrictHostKeyChecking |
| 旧程序启动失败 | 在 Jammy 上重装依赖并从源码重建 | 不要复制 Focal 的二进制/venv 强行运行 |
| 系统显示 Ubuntu 24.04 | 拿到的是 JetPack 7.x 包；按 §2.1 换 6.2.x 包重刷 | 不要试图在 24.04 上凑合装 Humble |
| rootfs 容量远小于 128 GB | 镜像与 SSD 容量适配（§12）；刷写日志 | 不要在装完 ROS 和数据后才处理分区 |
| ROS 源报 `EXPKEYSIG` | 是否照抄了旧 keyring 教程；改用 `ros2-apt-source`（§14.2） | 不要用 `--allow-unauthenticated` 绕过校验 |
| `apt update` 告警 `Key is stored in legacy trusted.gpg keyring` | 走 §14.1 的预期现象，见 §14.0「已知瑕疵」 | 不要因为这条告警去动 `nvidia-l4t-*` 的源或 key |
| 装完 ROS 后 `nvidia-l4t-apt-source.list` 不见了 | 是否在 fishros 换源步骤误选了「清理第三方源」；用 `/var/backups/pre-fishros-*/etc-apt.tar.gz` 还原 | 不要手工重建 L4T 源猜内容 |
| 误执行了 `apt upgrade` | 核对 `nvidia-l4t-kernel`/dtb 包是否被升级、`uname -r` 与设备树是否仍是厂商版本；异常则按 §16 恢复 | 不要继续在被覆盖的内核上部署 |
| WiFi 网卡消失 | `/opt/RTL-8821CU` 是树外驱动，需针对 5.15 内核重新编译 | 不要把 5.10 编出的 `.ko` 拷到新内核 |
| 刷后无法远程连接 | 按 §12.2 在现场恢复网络、SSH 与 Tailscale | 不要指望旧 Tailscale 节点自动回来 |

如果一次重试仍失败，不要连续更换多个变量。保留日志，每次只改变线缆、USB 口、Recovery 状态、下载缓存或组件选择中的一个因素。

## 16. 回滚方案

本方案没有原地撤销按钮。需要回到旧平台时，按优先级：

1. **首选：厂商整盘镜像恢复**。若刷机前按 §7.2 用 `l4t_backup_restore.sh -b` 做了整盘备份，则用同一厂商 JP5.x 刷机包对进入恢复模式的设备执行 `-r` 恢复，QSPI 与 NVMe 一并回到备份时点；
2. 否则：向 HZHY 索取与原系统匹配的 **JP5.x（L4T 35.6）适配镜像包**，按同样的 SW1/J7 流程重刷，再从 §7.3/§7.4 的文件归档恢复数据和配置，重建并验证服务。

因此真正的回滚能力来自三件事：厂商旧版本镜像包仍可获得、备份可读（整盘镜像或文件归档）、恢复步骤经过记录。仅保留旧 NVMe 不足以保证跨 QSPI 大版本后直接启动。

## 17. 对本项目的推荐执行方式

迁移路线已经确认，两个硬阻塞（厂商镜像包、备份主副本离机）已于 2026-08-23 解除。剩余步骤和当前状态如下：

| 步骤 | 内容 | 状态 |
|---|---|---|
| 1 | 数据盘点，明确 58 GB 已用空间中哪些必须保留 | **已完成**（§7.3 归档 + manifest） |
| 2 | 取得并验证 HZHY JetPack 6.2 适配镜像包 | **已完成**：R36.4.3 包已就位并登记哈希，见 §2.1 |
| 3 | 归档 `tar.gz` + `manifest.txt` 离机并校验 | **已完成**：两块独立物理盘各一份，四个哈希全部精确匹配，见 §7.4 |
| 4 | 复核 Docker 是否真的没有容器/named volume（需 sudo，`ssh -t`） | **已完成**：无容器、无 named volume，归档范围完整，见 §7.3 |
| 5 | 确认现场物理接触条件（SW1/J7/XT30 可触及，必要时拆壳；显示器、USB 线） | **未记录**，见 §5.2(a)、§4.2 |
| 6 | 物理断开 CAN 与执行器动力电并留证据 | **未完成**，见 §5.2(b) |
| 7 | 由项目负责人书面接受不可逆后果（尤其是失去 ROS 1 供应商栈） | **未记录**，见 §5.1 |
| 8 | （可选）用厂商 JP5.x 包做整盘镜像备份 | 待决策；需另向厂商索取 JP5.x 包，见 §7.2。若放弃，回滚依赖文件归档 + 未来重新索取 JP5.x 包 |
| 9 | 刷机主机解压镜像、装依赖、预配置账号 | **已完成并验证**（2026-08-23），见 §8、§17.1 第 3 项 |
| 10 | SW1/J7 进恢复模式，`l4t_initrd_flash.sh` 刷写 | 待执行，见 §9、§10 |
| 11 | 首启验收（期望 R36.4.3 / Ubuntu 22.04），锁 L4T 包、关无人值守升级 | 待执行，见 §12 |
| 12 | **在现场恢复网络、SSH 与 Tailscale** 后才离开 | 待执行，见 §12.2 |
| 13 | 选择性恢复数据 + 安装 ROS 2 Humble + FND-004A | ROS 2 Humble **已装**（§14.0，走 §14.1 路线）；数据恢复由项目负责人决定**暂缓**，仅恢复了 `99-livelybot.rules`；FND-004A **待执行** |

剩余人为前提集中在第 4（Docker 复核）、5（现场物理条件）、6（断能）、7（书面接受）项。刷机主机侧的技术准备（镜像、依赖、账号、备份）已全部就绪。§17.1 给出按时间顺序的完整操作卡与执行标注。

真正的风险剩两个：**刷完后习惯性 `apt upgrade` 把厂商内核/设备树升没了**（§12 已给出 apt-mark hold 保险），以及**现场物理条件（拆壳/按键/供电）没提前确认导致维护窗口内卡壳**。

### 17.1 执行日操作卡（按顺序照做）

**T-1 天（远程 + 刷机主机，约 1 小时）** —— 2026-08-23 执行情况标注如下

1. ~~补齐备份收尾（§7.4）：拷出 `manifest.txt` 并校验；两个文件复制到第二介质并再次校验~~ **已完成**；
2. ~~Docker sudo 复核（§7.3）~~ **已完成**：`ssh -t nvidia@172.20.15.185 "sudo docker ps -a; sudo docker volume ls"` 返回无容器、无 named volume，归档范围完整（注意必须 `ssh -t`，否则 `sudo` 报 `a terminal is required to read the password`）；
3. ~~解压镜像包到 `/opt/jetson-flash`、跑 `l4t_flash_prerequisites.sh`、跑 `l4t_create_default_user.sh` 预配置账号（§8 第 3~5 步）~~ **已完成并验证**：解压 8.6 GB、`rootfs` 属主 `root:root`、6 个关键文件就位、包内版本 R36.4.3、10 个依赖包全装、`/usr/bin/python` 软链已建、首启账号与主机名 `nvidia-desktop` 已写入 rootfs（自动登录开启；**账号口令属凭据，不记录于本文档与项目记录，刷后请立即修改并改用公钥登录**）；
4. ~~项目负责人书面确认接受 §5.1 全部不可逆后果（含失去 ROS 1 供应商栈的原生运行能力）~~ **已由项目负责人于 2026-08-23 确认**；
5. ~~确认现场物品与 SW1/J7/XT30 触及方式~~ **已由项目负责人于 2026-08-23 确认**。

**T-1 结论：刷机前置条件全部满足，可进入现场刷机日。**

**执行日 —— 现场（预留半天）**

6. **断能**（§5.2b）：物理断开电机/执行器动力电与 CAN 线束；在旧系统上记录 `ip -d -s link show can0` 输出留证；
7. 正常关机（`sudo shutdown -h now`），拔 XT30；
8. USB-C 连接载板 J7 ↔ 刷机主机；
9. **按住 SW1 → 插上 XT30 上电 → 保持 3~4 秒 → 松手**；主机上 `lsusb | grep -i 0955` 必须显示 `0955:7323`（§9）；
10. 刷写（§10）：

    ```bash
    cd /opt/jetson-flash/flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210/Linux_for_Tegra
    sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 \
      -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
      -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
      --showlogs --network usb0 jetson-orin-nano-devkit-super-nvme internal
    ```

    期间不碰线不碰板，等 `Flash is successful`；把 `initrdlog/` 新日志拷走存档；
11. 断电，拔 USB-C，**不按 SW1** 正常上电，接显示器键盘；
12. 首启验收（§12）：重点确认 `nv_tegra_release` 为 **R36 REVISION 4.3**、Ubuntu 22.04、rootfs 在 NVMe 且容量接近整盘；
13. 立即执行 §12 的两道保险：`apt-mark hold` 全部 `nvidia-l4t-*` + 关闭 unattended-upgrades；
14. 恢复远程访问（§12.2）：网线、SSH、host key 指纹核对、重装 Tailscale 认证、公钥登录；全部验证通过才可离场。

**执行日后（远程即可）**

15. 选择性恢复数据（§13）：从备份归档按需取用户数据/项目源码，禁止整体覆盖 `/etc`；
16. ~~安装 ROS 2 Humble（§14）~~ **已于 2026-08-23 完成**：走 §14.1 鱼香ROS 路线（在「很很很很重要」那步选 **2 不更换继续安装**），装成 `ros-humble-ros-base` + `rosdepc`；装后复核确认 L4T 源与 45 个 hold 均未受影响。只 update+install，全程未执行 `apt upgrade`；
17. 按 [FND-004A 烟测清单](jetson_arm64_smoke_test.md)执行 clean clone、context check、rosdep、五包 build/test，并记录 JetPack/L4T/GCC/CMake 版本；
18. 更新项目记录：实际刷入版本（JP6.2/R36.4.3）、刷写日志归档位置、烟测结果。

任何一步结果与预期不符：停下，保留现场与日志，按 §15 排查；一次只改一个变量。

## 18. 资料来源

以下资料在 2026-08-09 核对过，并在 2026-08-23 补充了载板实物核验、HZHY 供应商文档核读、版本矩阵与 ROS key 机制的复核；实际执行时应再次确认版本页面没有变化。

**HZHY 供应商资料（本地 `company/`，2026-08-23 核读）：**

- `company/1.jpeg`～`3.jpeg`：载板实物照片，丝印 `HYAI-311UAV_O_V12`，载板身份的直接证据；
- `company/用户手册/HYAI-311UAV 用户手册.pdf`（v2.0，2023-04-14）：载板接口位置/引脚定义、SW1（REC 按键）、J7（Type-C 烧录口）、J8（XT30 12~36V）、J2（CAN GH1.25：CANL/CANH）、J6（DEBUG TTL）、91.5×55mm、BSP 为厂商基于 NVIDIA BSP 的移植增量设计；
- `company/用户手册/HZHY NVIDIA 系列产品 快速刷机教程20250711.pdf`（v6.0，2025-07-11）：**含 JetPack 6.2 刷机命令**；载板 REC 按键对应表（311UAV = SW1）；恢复模式 USB ID 表（Orin NX 16GB = `0955:7323`）；红字约束（禁 `apt upgrade`、root 解压、主机 20.04 等）；技术支持 `support@hzhytech.com`；
- `company/用户手册/HZHY NVIDIA Orin NX 快速刷机教程.pdf`（v4.0，JP5.1.1）：SW1/J7 恢复流程照片、`l4t_initrd_flash.sh` 命令样例、`l4t_backup_restore.sh` 整盘备份/恢复流程；
- `company/用户手册/HZHY NVIDIA 系列载板 将系统安装至扩展盘.pdf`、`ORIN NX 启动项设置.pdf`、`HYAI-311UAV-Orin_NX 测试说明 20230412.pdf`、`Readme.txt`（厂商联系方式 `sam@hzhytech.com`）；
- [Jetson刷机（HZHY的盒子）- 博客园](https://www.cnblogs.com/xixixing/p/18736896)：外部旁证，展示 `flash-300BV12_311UAV_ONX_Jp6.2_SC_v1.0.0_20250210.tar` 包在 Ubuntu 22.04 主机上的实际刷写过程；
- [HZHY-AI310UAV 恢复模式提问（NVIDIA 论坛）](https://forums.developer.nvidia.com/t/how-to-entre-force-recovery-mode-of-nx-module-using-hzhy-ai310uav-development-board/205264)：同系列前代载板与厂商 hzhytech.com 的对应关系旁证。

**2026-08-23 新增/复核（NVIDIA/ROS 官方）：**

- [JetPack 6.2.3 / Jetson Linux 36.5.2 发布公告（Orin NX）](https://forums.developer.nvidia.com/t/jetpack-6-2-3-jetson-linux-36-5-2-is-now-live/379872)：当前最新 6.x production release，Ubuntu 22.04 + 内核 5.15，支持全部 Orin 模块；
- [JetPack 6.2.2 / Jetson Linux 36.5 发布公告](https://forums.developer.nvidia.com/t/jetpack-6-2-2-jetson-linux-36-5-is-now-live/359622)：6.2.1 的缺陷与安全修复版本，保守退路；
- [Jetson Linux 36.5.0 GA Release Notes (PDF)](https://docs.nvidia.com/jetson/archives/r36.5/ReleaseNotes/Jetson_Linux_Release_Notes_r36.5.pdf)：L4T 36.5 与 JetPack 6.2.2 的配对关系；
- [JetPack Archive](https://developer.nvidia.com/embedded/jetpack-archive)：JetPack ↔ L4T ↔ 支持模块的完整对照，用于当天确认小版本；
- [Jetson Linux Quick Start（Force Recovery 与 J14 pin 9–10）](https://docs.nvidia.com/jetson/archives/r35.6.0/DeveloperGuide/IN/QuickStart.html)：官方 P3768 载板的 `J14` pin 9–10 短接方法——**仅适用于官方载板，经 §2.2 更正后对本机不适用**，保留作对照；本机恢复模式按 §9 的 SW1/J7 流程；
- [ROS signing key migration guide](https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937)：2025 年起改用 `ros2-apt-source` 包管理 ROS 仓库与签名 key，旧 keyring 做法已过期；
- [ROS 2 Humble Ubuntu deb 安装页](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)：Jammy 上的官方安装步骤（含 `ros2-apt-source`）。

**2026-08-09 原有来源（其中 SDK Manager 相关条目经 §2.2 更正后不再是本方案主路线，保留作背景）：**

- [NVIDIA JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621)：Jetson Linux 36.4.4、Ubuntu 22.04 rootfs、Linux 5.15 和组件版本（**已被 6.2.2/6.2.3 取代，保留作历史对照**）；
- [Jetson Linux 36.4.4 Quick Start](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/IN/QuickStart.html)：官方支持模块/载板、Force Recovery 和 USB ID；
- [Jetson Linux 36.4.4 Flashing Support](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/FlashingSupport.html)：NVMe、initrd flash、Backup and Restore 的官方机制说明（厂商流程即基于这些工具）；
- [Jetson Linux Software Packages and Update Mechanism](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/SoftwarePackagesAndTheUpdateMechanism.html)：Debian OTA、image-based OTA 和 release-upgrade 边界；
- [JetPack Installation and Setup](https://docs.nvidia.com/jetson/jetpack/install-setup/index.html)：SDK Manager 支持范围（不适用于本第三方载板）；
- [SDK Manager Direct Flash](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson-direct-flash/index.html)：同上，保留作背景；
- [SDK Manager System Requirements](https://docs.nvidia.com/sdk-manager/system-requirements/index.html)：同上，保留作背景。
