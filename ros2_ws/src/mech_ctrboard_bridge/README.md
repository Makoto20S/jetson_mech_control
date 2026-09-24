# mech_ctrboard_bridge

Receive-only ROS 2 bridge from the STM32 CtrBoard sensor protocol to Linux
SocketCAN. The node opens an already configured interface and filters standard
Classic CAN ID `0x621`. It has no command subscriptions and never writes a CAN
frame.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `interface` | `can0` | Existing Linux SocketCAN interface |
| `logical_bus` | `1` | Internal label attached to received raw frames |
| `poll_period_ms` | `1` | Non-blocking receive polling period |
| `imu_1_frame_id` | `imu_1_link` | Frame ID for the first IMU |
| `imu_2_frame_id` | `imu_2_link` | Frame ID for the second IMU |

The operating system must configure the physical interface for Classic CAN at
1 Mbit/s before the node starts. This package never changes the interface,
bitrate, termination, or transceiver state.

## Published topics

| Topic | Type | Content |
| --- | --- | --- |
| `imu/1/data` | `sensor_msgs/Imu` | IMU 1 quaternion, angular velocity in rad/s, acceleration in m/s^2 |
| `imu/2/data` | `sensor_msgs/Imu` | IMU 2 quaternion, angular velocity in rad/s, acceleration in m/s^2 |
| `imu/1/euler_deg` | `geometry_msgs/Vector3Stamped` | IMU 1 roll, pitch, yaw in degrees |
| `imu/2/euler_deg` | `geometry_msgs/Vector3Stamped` | IMU 2 roll, pitch, yaw in degrees |
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
