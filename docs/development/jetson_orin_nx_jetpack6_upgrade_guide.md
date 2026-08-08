# Jetson Orin NX 升级到 JetPack 6.2.1 / Ubuntu 22.04

> 状态：迁移路线与 Ubuntu 22.04 刷机主机已由项目负责人确认；尚未授权或执行刷机
> 适用设备：NVIDIA Jetson Orin NX `P3767-0000` + 参考载板 `P3768-0000`，系统盘为 NVMe
> 目标版本：JetPack 6.2.1 / Jetson Linux 36.4.4 / Ubuntu 22.04
> 刷机主机：项目负责人确认使用 Ubuntu 22.04 电脑；执行前仍须确认其为受支持的 x86_64 环境
> 最近核对：2026-08-09
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

如果你会独立重装一台电脑、能识别主板上的 `REC/FC REC` 与 `GND` 标识，并愿意预留半天，通常可以自己做。以下情况不建议独自立即操作：

- 只能通过 SSH 远程接触 Jetson，现场无人能接跳线、重启或看显示器；
- NVMe 中有唯一副本的数据，或者不知道哪些现有服务正在使用这些数据；
- 设备当前承担不能中断的生产任务；
- 没有可靠 USB 数据线、稳定电源或可用刷机主机；
- 实际硬件不是本文限定的 NVIDIA `P3767-0000 + P3768-0000` 组合。

建议按首次操作预留 **3～6 小时或半天维护窗口**：

| 阶段 | 常见耗时 |
|---|---:|
| 盘点、决定备份范围并完成备份 | 1～3 小时，主要取决于数据量 |
| SDK Manager 安装与镜像下载 | 30～90 分钟，取决于网络 |
| 恢复模式、系统刷写 | 20～45 分钟 |
| 首次启动和可选 JetPack 组件安装 | 30～90 分钟 |
| ROS 2、项目工具和 FND-004A 前置准备 | 1～2 小时 |

USB、网络或 SDK Manager 下载问题可能把首次操作延长到一天，因此不要安排在必须马上恢复设备的时间段。

## 2. 为什么主路线是刷机

Jetson Linux 不只是 Ubuntu rootfs。JetPack 5 到 JetPack 6 的跨代变化同时涉及：

- QSPI 中的启动固件和 UEFI；
- Jetson Linux 内核、设备树和 NVIDIA 板级驱动；
- 分区布局与 NVMe rootfs；
- Ubuntu 20.04 到 22.04 的用户态；
- CUDA、TensorRT 等 NVIDIA 用户态组件。

因此不能用通用 `do-release-upgrade` 把这台设备安全地变成 JetPack 6。NVIDIA 的 Jetson Linux 文档明确禁用通用 Ubuntu release upgrade；JetPack 文档中的 APT 升级步骤面向兼容的 JetPack 6.x 版本，不应套用于当前 R35/JetPack 5 到 R36/JetPack 6 的迁移。

本文推荐路线是：

1. 验证并备份现有 NVMe 数据；
2. 用 NVIDIA SDK Manager 的 **Direct Flash** 同步刷写 QSPI 与 NVMe；
3. 完成 Ubuntu 22.04 首次启动；
4. 只选择性恢复配置和数据；
5. 原生安装 ROS 2 Humble 和构建工具；
6. 再按项目清单执行 FND-004A。

NVIDIA 的 image-based OTA 和 `l4t_initrd_flash.sh` 都是有效工具，但前者更适合已经建立镜像、签名、分区和回滚流程的设备群，后者需要操作者自行承担 board config、分区 XML 和参数正确性。对于一台标准开发套件的第一次迁移，SDK Manager 更容易审查和重试。

也不要把 Orin Nano 的 SD 卡镜像教程直接套到本机。本机是 Orin NX 模块并从 NVMe 启动；必须让刷机流程同时处理正确的 Jetson BSP、QSPI 和 NVMe。

## 3. 本机基线与本次目标

2026-08-08 的只读盘点得到以下快照，正式刷机当天必须重新检查：

| 项目 | 最近观察值 |
|---|---|
| 模块与载板 | `P3767-0000 + P3768-0000`，标准 NVIDIA 参考组合 |
| 架构与内存 | `aarch64`，约 15 GiB RAM |
| 系统盘 | 128 GB NVMe，约 52 GB 已使用、59 GB 可用 |
| 当前平台 | JetPack 5.1.5 / L4T 35.6.0 / Ubuntu 20.04.6 / Linux 5.10-tegra |
| 当前 ROS | ROS 1 Noetic；没有原生 ROS 2 Humble |
| 目标平台 | JetPack 6.2.1 / L4T 36.4.4 / Ubuntu 22.04 / Linux 5.15-tegra |

未发现第三方载板 BSP 标识，这是本次迁移难度较低的重要原因。如果刷机当天的 device tree、载板或 NVMe 布局与上述结果不符，应立即停止并重新选择对应厂商 BSP。

## 4. 准备物品

### 4.1 推荐的刷机主机

本项目已确认使用以下主路线：

- 原生 Ubuntu 22.04 x86_64 电脑；
- 至少 8 GB RAM；
- 官方最低要求为约 27 GB 主机可用空间，实际建议预留 50 GB 以上；
- 稳定互联网连接和 NVIDIA Developer 账号；
- 管理员权限。

执行前在刷机主机运行 `uname -m`，期望为 `x86_64`。当前 SDK Manager 文档也支持 Windows 10/11，通过 SDK Manager 管理的 WSL2 和 APX 驱动执行 JetPack 6.2.1 Direct Flash，但本项目没有选择该路线。普通 VirtualBox/VMware Ubuntu 虚拟机不推荐，因为 Jetson 在刷写期间会多次 USB 重新枚举，USB 直通容易丢失。

若以后因故改用 Windows，必须重新核对 SDK Manager 当天显示的 JetPack 6.2.1 兼容矩阵和 `Direct Flash` 可用状态，不得自行在普通 WSL 终端里拼装低级 flash 命令。

### 4.2 现场物品

- 一根确认支持数据传输的 USB-C 数据线，优先直连主机，不经过 Hub；
- Jetson 原装或能力充足的稳定电源；
- 可接触 `REC/FC REC` 与 `GND` 的跳线帽；
- 推荐准备显示器、键盘和有线网络；
- 一个容量足够、最好使用 ext4 或加密文件系统的备份盘/NAS；
- 至少半天不中断的维护窗口。

## 5. 刷机前的强制停点

以下条件全部满足后才可以点击 SDK Manager 的 Flash：

- [ ] 已重新确认模块、载板、系统盘和当前 L4T 版本；
- [ ] 已列清要保存的数据、配置、容器数据和未提交代码；
- [ ] 备份已经复制到 Jetson 之外，并做过读取或校验验证；
- [ ] 已接受 NVMe 会被清空、旧系统不能原地恢复；
- [ ] 刷机主机有足够空间，网络、电源和 USB 连接稳定；
- [ ] 可以在现场进入 Force Recovery，且刷后能通过显示器或本地网络完成初始化；
- [ ] 已记录旧系统的网络、服务和应用依赖，但不会把旧 `/etc` 整体覆盖到新系统；
- [ ] CAN、电机和其他执行器保持断开或不可动作状态。

**SDK Manager 真正开始写入之前，是最后一个无数据损失退出点。** 如果任何一项不确定，就停在这里。

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

### 7.2 更稳但更麻烦的选择

如果旧环境必须可以完整复原，应在刷机前使用 NVIDIA 文档中的 Backup and Restore/initrd 工作流创建整盘镜像，并在另一块介质上验证可读取性。这比文件备份复杂，适合把旧系统当作业务资产的情况。

换一块空白 NVMe 再刷可以保护原 NVMe 上的数据，但不能承诺“插回旧盘立刻回滚”，因为 Direct Flash 还会升级 QSPI/UEFI。真正回到 JetPack 5 仍可能需要用匹配版本重新刷写启动固件。

## 8. 第三步：准备 SDK Manager

1. 从 NVIDIA 官方页面下载 SDK Manager，不使用第三方镜像或未知安装脚本。
2. 在推荐的 Ubuntu 主机上安装下载的 `.deb`：

   ```bash
   sudo apt install ./sdkmanager_<实际版本>_amd64.deb
   ```

3. 启动并登录 NVIDIA 账号：

   ```bash
   sdkmanager
   ```

4. 如果普通列表不显示 JetPack 6.2.1，先核对兼容矩阵，再使用 SDK Manager 提供的 archived versions 入口；CLI 版本通常可用：

   ```bash
   sdkmanager --archived-versions
   ```

5. 暂时不要点击 Flash。先确认下载目录、主机剩余空间和目标版本均正确。

SDK Manager 界面会随版本调整，本文不依赖按钮的像素位置；关键是核对目标硬件、JetPack 版本、Direct Flash 和 NVMe 四个选择。

## 9. 第四步：让 Jetson 进入 Force Recovery

推荐第一次使用手动恢复模式：

1. 正常关闭 Jetson，等待完全断电；
2. 断开 Jetson DC 电源；
3. 用可靠 USB-C 数据线连接开发套件的刷机 USB-C 口与刷机主机；
4. 按开发套件丝印和 NVIDIA 用户指南，把 12-pin button header 上的 `REC/FC REC` 与 `GND` 短接；不要凭网上不同载板的图片猜针脚；
5. 重新接通 DC 电源；
6. 在 Ubuntu 刷机主机上运行：

   ```bash
   lsusb | grep 0955
   ```

7. 对本机 `P3767-0000`，应看到 NVIDIA USB 设备 `0955:7323`。检测到后再回到 SDK Manager。

恢复模式下屏幕不显示正常 Ubuntu、SSH 也不可用，这是正常现象。若 `lsusb` 没有 NVIDIA `0955` 设备，不要反复点击 Flash；先排查数据线、USB 口、跳线、供电和是否用了 Hub。

SDK Manager 也可能提供 Automatic Setup，通过正在运行的旧系统自动进入恢复模式。它可以使用，但手动方式更容易确认物理状态，也不依赖旧系统远程配置。

## 10. 第五步：在 SDK Manager 中选择刷写内容

逐项核对：

1. **Product Category**：Jetson；
2. **Target Hardware**：让工具自动识别，并确认对应 `P3767-0000` Orin NX/参考开发套件；不要手选成 AGX Orin 或第三方载板；
3. **SDK Version**：JetPack 6.2.1，对应 Jetson Linux 36.4.4 和 Ubuntu 22.04；
4. **Installation Method**：Direct Flash；
5. **Target Storage**：NVMe；不要选 SD Card 或与本机不符的 eMMC；
6. **Jetson OS**：必须选择；
7. **Target SDK Components**：按需要选择。当前 ROS/CAN Foundation 不依赖 CUDA 示例，Host Components、samples 和 documentation 可以不装；需要 GPU 栈时可安装 JetPack target runtime；
8. **OEM Configuration**：第一次且有显示器时推荐 Runtime，刷后亲自完成 Ubuntu 初始化；纯 headless 场景可用 Pre-Config，但必须妥善处理账号凭据；
9. 再次确认界面明确提示会覆盖目标 NVMe。

对于本项目，最容易定位故障的顺序是先确保 **Jetson OS 刷写和首启成功**，再完成可选目标组件。后置 CUDA/SDK 组件安装失败时，通常不需要重刷一个已经正常启动的 OS。

## 11. 第六步：执行刷写

1. 点击 Flash 前，再检查一次外部备份；
2. 保持电源、USB 和主机网络稳定，不让刷机主机休眠；
3. 观察 SDK Manager 的 Details/Terminal 页面；
4. 写入进行中不要拔线、断电或关闭 SDK Manager；
5. 如果失败，先导出 Debug Logs，记录失败阶段和错误，再决定重试；
6. 工具提示刷写完成后，按提示移除恢复跳线并正常启动；
7. 使用 Runtime 配置时，在 Jetson 显示器上完成地区、键盘、用户、网络和时区设置；
8. 如果继续安装 Target SDK Components，在 SDK Manager 提示时输入新系统账号，等待后置安装完成；
9. 导出并保存最终 SDK Manager 日志，但在分享前检查用户名、路径和网络信息。

第一次写 QSPI 或第一次启动可能比普通重启久。只要 SDK Manager 仍在输出有效进度，就不要因几分钟没有桌面而强制断电。

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

期望结果：

- `uname -m` 为 `aarch64`；
- Ubuntu 为 22.04；
- `/etc/nv_tegra_release` 为 R36，目标 revision 为 36.4.4 对应版本；
- 内核为 NVIDIA `5.15...-tegra`；
- 根文件系统来自 NVMe 且为正常 Linux 文件系统；
- `dpkg --audit` 没有损坏包；
- 时间同步、网络和重启正常；
- 只有安装了完整/运行时 JetPack meta package 时才强制要求查询到 `nvidia-jetpack`，不要把“只刷 Jetson OS”误判成刷写失败。

随后在同一 R36 发布支持范围内更新普通软件包并重启。不要运行 `do-release-upgrade`，也不要把 APT 源手工指向另一个 L4T 大版本。

新系统的 SSH host key、IP 地址或主机名可能变化。客户端出现 host-key warning 时，应先在 Jetson 本地显示器上核对新指纹，再清除客户端的旧记录；不要为了消除提示直接关闭 host-key 校验。

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

JetPack 6.2.1 提供 Ubuntu 22.04 和 NVIDIA BSP，但不会自动满足本项目的 ROS 2 Humble 开发环境。刷机验收通过后：

1. 按 ROS 官方的 Ubuntu deb 安装页配置 Jammy 的 ROS 2 APT 源，不复制来源不明的旧 key 命令；
2. 安装至少以下目标包：

   ```text
   ros-humble-ros-base
   python3-colcon-common-extensions
   python3-rosdep
   build-essential
   cmake
   git
   ```

3. 验证 `/opt/ros/humble/setup.bash`、`ros2`、`colcon` 和 `rosdep`；
4. 初始化/更新官方 rosdep；
5. 配置私有仓库的 repository-scoped read-only deploy key 或其他已批准的只读方式；
6. 重新运行 FND-004A admission inventory；
7. 严格按 [FND-004A Jetson ARM64 原生烟测](jetson_arm64_smoke_test.md)执行 clean clone、context check、rosdep 和五包 build/test。

FND-004A 仍不启用 CAN，不打开设备，不调整实时调度。Docker 是否作为最终运行方式，应在原生平台烟测之后另行决定；升级宿主和使用容器并不冲突。

## 15. 常见故障及处理

| 现象 | 优先检查 | 不要做什么 |
|---|---|---|
| SDK Manager 看不到目标 | `lsusb` 是否有 `0955:7323`、REC/GND、数据线、直连 USB、电源 | 不要随便选择一个相近 board profile 强刷 |
| 下载失败或空间不足 | 主机网络、代理、磁盘空间、SDK Manager 下载目录 | 不要在镜像不完整时绕过校验 |
| Flash 中途失败 | 导出日志，重新进入 Recovery，优先重试相同 OS/board/NVMe 组合 | 不要把失败直接判断为永久变砖 |
| OS 已启动但 SDK Components 失败 | Jetson 网络、账号、磁盘、SDK Manager Repair/Resume | 不要为单纯后置组件失败立刻擦盘重刷 |
| 刷完找不到 NVMe rootfs | SDK Manager 的 Target Storage、NVMe 安装与识别、刷写日志 | 不要改未知 partition XML 试错 |
| 黑屏或启动循环 | 本地串口/显示器日志、版本是否匹配；重新 Recovery 刷写 | 不要运行普通 Ubuntu boot repair 覆盖 Jetson 启动链 |
| SSH 报 host key 变化 | 本地核对新系统指纹后更新客户端记录 | 不要关闭 StrictHostKeyChecking |
| 旧程序启动失败 | 在 Jammy 上重装依赖并从源码重建 | 不要复制 Focal 的二进制/venv 强行运行 |

如果一次重试仍失败，不要连续更换多个变量。保留日志，每次只改变线缆、USB 口、Recovery 状态、下载缓存或组件选择中的一个因素。

## 16. 回滚方案

本方案没有原地撤销按钮。需要回到旧平台时：

1. 使用支持原版本的 NVIDIA SDK Manager/Jetson Linux 工具；
2. 重新刷写匹配的 JetPack 5/L4T 35 QSPI 和系统盘；
3. 完成旧系统首次启动；
4. 从已验证备份恢复数据和配置；
5. 重建并验证服务。

因此真正的回滚能力来自三件事：旧版本安装介质/工具仍可获得、备份可读、恢复步骤经过记录。仅保留旧 NVMe 不足以保证跨 QSPI 大版本后直接启动。

## 17. 对本项目的推荐执行方式

迁移路线已经确认，后续按以下低风险顺序执行：

1. 先用半天做数据盘点，明确 52 GB 左右已用空间中哪些必须保留；
2. 在外部介质完成并验证备份；
3. 在已确认的 Ubuntu 22.04 电脑上验证 `x86_64`、可用空间、网络、管理员权限和 USB 识别；
4. 使用 SDK Manager Direct Flash 刷 JetPack 6.2.1 到 NVMe；
5. 先验收干净的 R36/Ubuntu 22.04，再恢复服务；
6. 原生安装 ROS 2 Humble 并执行 FND-004A；
7. 烟测通过后，再设计最终是原生运行还是在同一 JetPack 6 宿主上使用 hardened Humble 容器。

这条路线的主要麻烦是备份和重新配置，而不是刷机按钮本身。对当前标准开发套件，我认为风险可控，也适合个人完成；但必须把它安排成一次有维护窗口、有外部备份的重装，而不是在远程 SSH 会话里尝试升级。

## 18. 官方资料

以下资料在 2026-08-09 核对过；实际执行时应再次确认版本页面没有变化：

- [NVIDIA JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621)：Jetson Linux 36.4.4、Ubuntu 22.04 rootfs、Linux 5.15 和组件版本；
- [Jetson Linux 36.4.4 Quick Start](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/IN/QuickStart.html)：支持模块/载板、Force Recovery 和 USB ID；
- [Jetson Linux 36.4.4 Flashing Support](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/FlashingSupport.html)：NVMe、initrd flash、Backup and Restore；
- [Jetson Linux Software Packages and Update Mechanism](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/SoftwarePackagesAndTheUpdateMechanism.html)：Debian OTA、image-based OTA 和 release-upgrade 边界；
- [JetPack Installation and Setup](https://docs.nvidia.com/jetson/jetpack/install-setup/index.html)：SDK Manager 支持范围和 JetPack 6.x 包升级边界；
- [SDK Manager Direct Flash](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson-direct-flash/index.html)：Direct Flash、Recovery、NVMe、OEM configuration 和后置组件；
- [SDK Manager System Requirements](https://docs.nvidia.com/sdk-manager/system-requirements/index.html)：主机、内存、空间和兼容矩阵；
- [ROS 2 Humble Ubuntu deb installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)：Ubuntu 22.04 上的官方 ROS 2 安装入口。
