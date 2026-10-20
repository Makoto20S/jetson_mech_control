# CtrBoard 硬件验证记录

## 验证范围

本记录验证仓库中的 `mech_ctrboard_bridge` 能够把 STM32 CtrBoard 作为
ROS 2 节点 `/stm32_ctrboard` 接入系统，经 Livelybot USB-CDC 通道读取标准
Classic CAN ID `0x621`，并保存、回放双 IMU 与双足底压力话题。

桥接节点是只接收节点。除 USB 通信板要求的固定
`MODE_FDCAN_PASS` 接收转发配置外，它不发送应用 CAN 帧，也不包含电机控制
路径。

## 验证环境

- 日期：2026-09-28
- 目标：ARM64 Linux，ROS 2 Humble
- USB-CDC 通道：`/dev/ttyACM0`
- ROS 节点：`/stm32_ctrboard`
- 源码基线：`d1308d3` 加当前工作树中的 USB-CDC 适配改动

## 仓库入口

启动文件：

```bash
ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py \
  device_path:=/dev/ttyACM0
```

推荐用仓库脚本完成有界采集。运行前需要 source ROS 2 和构建后的工作区：

```bash
bash tools/bench/record_ctrboard_sensors.sh \
  /path/to/ctrboard_capture 30 /dev/ttyACM0
```

该脚本要求先收到真实设备时间戳，再记录 10 个公开话题；结束时生成
`validation.txt` 和 `SHA256SUMS`，并清理 launch、桥接节点和录包进程。

桥接节点运行时，可以在第二个终端实时查看合并后的传感器状态：

```bash
python3 tools/bench/watch_ctrboard_sensors.py
```

显示内容包括两路 IMU 的 Roll/Pitch/Yaw、左右足底总压力、状态字节、设备
时间戳和实际更新频率。该工具只订阅 ROS 话题，不打开硬件，也不发送命令。

在 Windows PowerShell 中应使用一键启动器，而不是直接运行上述 Linux
Python 命令：

```powershell
.\tools\bench\run_ctrboard_live_monitor.ps1
```

启动器负责 SSH 连接、加载 Jetson 上的 ROS 环境、启动桥接节点和清理进程。
默认持续运行，按 `Ctrl+C` 停止；测试时可用 `-DurationSeconds 10` 限定时长。

## 结果

ARM64 增量构建成功，以下四个包的测试汇总为 458 个测试、0 个错误、
0 个失败：

- `mech_control_core`
- `mech_simulation`
- `mech_ctrboard_bridge`
- `mech_bringup`

主采集结果：

| 项目 | 结果 |
|---|---:|
| 录制时长 | 29.300446273 s |
| 总消息数 | 2940 |
| 话题数 | 10 |
| 每个话题消息数 | 294 |
| 估算频率 | 10.034 Hz |
| rosbag2 数据库大小 | 425984 bytes |
| SHA-256 | `46860298e28ef12d41b2f826f6594b8c9f8d6e9afdad2bc810c188fe193e72d5` |

原始数据保存在本地忽略目录：

```text
out/hardware_validation/ctrboard_validation_20260928_040014/
```

在隔离的 `ROS_DOMAIN_ID` 中回放成功。回放样本包括：

```text
device timestamp: 5559220 ms
IMU2 frame: shank_imu_link
IMU2 Euler: roll=-0.233 deg, pitch=-0.224 deg, yaw=39.972 deg
left FSR total: 0
right FSR total: 0
sensor status: [0, 0, 0, 0]
```

## 结论与限制

仓库中的 USB-CDC 传输、CtrBoard 协议重组与 CRC、ROS 2 节点发布、rosbag
保存和离线回放链路已经完成实机验证。10 个话题在本次采集中具有相同消息数，
未观察到单话题漏发。

本次只有 IMU2 提供非零姿态。IMU1、左右足底压力和四个足底状态字节均为零，
因此当前证据只证明这些字段能够被解析、发布和保存，不能证明第二个 IMU 或
足底压力传感器已经输出有效测量值。部署时还需用 udev 或经批准的用户组规则
替代 `/dev/ttyACM0` 的临时 ACL。
