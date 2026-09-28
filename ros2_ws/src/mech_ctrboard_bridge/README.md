# mech_ctrboard_bridge

Receive-only ROS 2 bridge from the STM32 CtrBoard sensor protocol through
Linux SocketCAN or the Livelybot seven-channel USB-CDC communication board.
The node accepts only standard Classic CAN ID `0x621`. It has no command
subscriptions and never writes an application CAN frame.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `transport` | `socketcan` | `socketcan` or `usb_cdc` |
| `interface` | `can0` | Existing Linux SocketCAN interface |
| `device_path` | `/dev/ttyACM0` | USB-CDC channel when `transport=usb_cdc` |
| `logical_bus` | `1` | Internal label attached to received raw frames |
| `board_version_major` | `4` | Verified USB communication-board firmware major version |
| `board_version_minor` | `8` | Verified USB communication-board firmware minor version |
| `board_version_patch` | `8` | Verified USB communication-board firmware patch version |
| `poll_period_ms` | `1` | Non-blocking receive polling period |
| `imu_1_frame_id` | `imu_1_link` | Frame ID for the first IMU |
| `imu_2_frame_id` | `imu_2_link` | Frame ID for the second IMU |

The operating system must configure the physical interface for Classic CAN at
1 Mbit/s before the node starts. This package never changes the interface,
bitrate, termination, or transceiver state.

For the seven-channel USB board, each physical CAN channel appears as one
`/dev/ttyACM*` device. The runtime user needs read/write permission, normally
through membership in the `dialout` group. The node sends the vendor's fixed
`MODE_FDCAN_PASS` setup record with `send_flag=0` once after opening the CDC
device. This enables receive forwarding and does not carry an application CAN
frame. Firmware older than the declared and bench-tested `4.8.8` protocol is
rejected by the transport.

## Published topics

| Topic | Type | Content |
| --- | --- | --- |
| `imu/imu1/data` | `sensor_msgs/Imu` | IMU 1 quaternion, angular velocity in rad/s, acceleration in m/s^2 |
| `imu/imu2/data` | `sensor_msgs/Imu` | IMU 2 quaternion, angular velocity in rad/s, acceleration in m/s^2 |
| `imu/imu1/euler_deg` | `geometry_msgs/Vector3Stamped` | IMU 1 roll, pitch, yaw in degrees |
| `imu/imu2/euler_deg` | `geometry_msgs/Vector3Stamped` | IMU 2 roll, pitch, yaw in degrees |
| `fsr/left/raw` | `std_msgs/UInt16MultiArray` | 20 left-insole pressure values |
| `fsr/right/raw` | `std_msgs/UInt16MultiArray` | 20 right-insole pressure values |
| `fsr/left/total` | `std_msgs/UInt32` | Left-insole raw pressure sum |
| `fsr/right/total` | `std_msgs/UInt32` | Right-insole raw pressure sum |
| `ctrboard/sensor_status` | `std_msgs/UInt8MultiArray` | Left/right gait phase then left/right active point count |
| `ctrboard/timestamp_ms` | `std_msgs/UInt32` | STM32 millisecond timestamp |

An all-zero quaternion marks a missing or not-yet-initialized IMU. The bridge
publishes an identity quaternion with `orientation_covariance[0] = -1` in that
case. Message headers use ROS receive time; the raw device timestamp is
published separately because no host/device clock synchronization is defined.

## Safety and ownership

Run this bridge only on a dedicated sensor bus or an explicitly approved
shared-bus profile. `SocketCanTransport` is not internally synchronized and a
physical interface must not have competing owners. This package does not relax
ADR-006 or any G0-G3 hardware gate.

After passive traffic and deployment configuration have been reviewed:

```bash
ros2 run mech_ctrboard_bridge ctrboard_bridge_node --ros-args \
  -p interface:=can0 \
  -p imu_1_frame_id:=thigh_imu_link \
  -p imu_2_frame_id:=shank_imu_link
```

For the Livelybot USB communication board:

```bash
ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py \
  device_path:=/dev/ttyACM0 \
  imu_1_frame_id:=thigh_imu_link \
  imu_2_frame_id:=shank_imu_link
```

This launch entry represents the attached STM32 as the ROS 2 node
`/stm32_ctrboard`. The STM32 firmware remains a CAN device and does not run ROS
itself; the bridge owns transport decoding and publishes its sensor interface.

With the bridge running, display both IMU Euler angles, both FSR totals, sensor
status, device time, and the measured update rate in a second terminal:

```bash
python3 tools/bench/watch_ctrboard_sensors.py
```

The monitor refreshes one terminal line at 10 Hz and stops cleanly with
`Ctrl+C`. It does not open the transport or publish commands.

From Windows PowerShell, the repository launcher performs the SSH connection,
sources the target ROS environment, starts the bridge, and runs the monitor:

```powershell
.\tools\bench\run_ctrboard_live_monitor.ps1
```

The launcher defaults to `nvidia@192.168.55.1`, automatically uses the current
bench SSH key when present, and accepts parameters for a different host, key,
remote repository, install overlay, device, or bounded duration.

To preserve and validate an experiment without putting large captures in Git,
source the workspace overlay and run the repository's bounded recording tool:

```bash
bash tools/bench/record_ctrboard_sensors.sh \
  /path/to/ctrboard_capture 30 /dev/ttyACM0
```

The tool starts this package's launch file, requires a live timestamp sample,
records all ten public topics for the requested duration, writes `ros2 bag
info` output, and generates SHA-256 checksums for the bag database. The output
directory must not already exist.
