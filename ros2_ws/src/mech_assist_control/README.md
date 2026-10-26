# Whole-exoskeleton application and compatibility entries

`exoskeleton_control_node` is the new whole-robot application entry. The pure
`robot/` modules separate immutable observations/commands, policies, supervision
and controller routing. The ROS wrapper defaults to preview and never starts a
serial bridge or activates hardware. See the
[Chinese architecture and remaining work](../../../docs/development/exoskeleton_control_architecture.md).

```bash
ros2 launch mech_assist_control exoskeleton_preview.launch.py
ros2 service call /exoskeleton/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /exoskeleton/command_envelope
```

Arm requires complete fresh calibrated joint data and healthy estimator diagnostics.
One to six configured joints are supported by the application contract; the current
estimator remains a single knee. Policies: `transparent`, `resistance`,
`hold_velocity`. Higher algorithms implement the same `Policy` contract; insole
and gait/adaptive assistance are not supplied by this entry. The preview launch
has no motor mode argument and supplies no serial or device launch.

Explicit motor compatibility output requires measured coordinate mapping, current
actuator feedback and unique configured controller topics. Routing is independent
of motor brand; existing C++ controllers and hardware gates remain authoritative.
The diagnostic JSON envelope is versioned locally; the existing Float64 controller
input does not yet transport its source sequence/deadline. Multi-topic publication
is not an atomic multi-joint transaction. Physical stop must be checked independently.

## Legacy single-knee entries


`knee_control_node` is the preferred application entry point. It selects
exactly one strategy at startup; `knee_motor_follow_node` remains an alias for
existing bench scripts. Both names run the same ROS node, whose graph name
remains `knee_motor_follow` for compatibility. Do not launch multiple
motor-output instances against one controller topic: startup mode selection
does not provide a distributed lock across ROS processes.

| Strategy | Input | Output | Selected mode |
| --- | --- | --- | --- |
| `velocity_follow` | `imu` | `preview` | Velocity preview (default) |
| `velocity_follow` | `imu` | `motor` | Mapped motor velocity |
| `velocity_follow` | `motor_feedback_simulation` | `motor` | Unloaded periodic bench |
| `resistance` | `imu` | `preview` | Resistance torque preview |
| `resistance` | `imu` | `motor` | Mapped DM4340 effort, bench-only |

All other combinations fail during startup. Runtime parameter changes are
rejected; restart with a different combination. The shared
`/knee_control/arm` and `/knee_control/reset` services apply to the selected
mode. The legacy `/knee_motor_follow/*` services remain available for velocity
modes so the existing ROS bench entry is unchanged.

## Knee resistance

Select `control_strategy:=resistance` on `knee_control_node` for preview-only
resistance training. It reads `knee/joint_state` and `knee/diagnostics` and publishes
`knee_resistance/preview_effort_nm` and `knee_resistance/diagnostics`. Preview
does not publish to a motor controller or enable hardware. Arm it with
`ros2 service call /knee_control/arm std_srvs/srv/Trigger '{}'`; reset a
latched fault with `/knee_control/reset` after correcting the input.

```bash
ros2 run mech_assist_control knee_control_node --ros-args \
  -p control_strategy:=resistance
```

The virtual knee torque is `-sign(omega) * min(Tmax, B *
max(|omega| - deadband, 0))`. It is dissipative, unlike the STM32 AO+PD
reference follower, which can assist motion. Defaults (`B=0.5 Nm s/rad`,
`Tmax=0.3 Nm`, ten-second trial) are preview fixtures, not clinically validated
training settings. A fresh knee sample and healthy diagnostic are required;
stale/invalid input, out-of-range motion and expiry latch the output to zero.

Explicit `output_mode:=motor` instead publishes to
`/dm4340_effort_controller/target_effort`. It requires
`joint_mapping_verified:=true`, a nonzero measured `motor_to_knee_scale`,
the measured offset, fresh motor JointState feedback, and agreement between
mapped shaft motion and the knee estimate. Stale/disagreeing data latch a
fault, send one zero, then cease target refresh. The motor feedback and
effort command must use the same ROS joint coordinates. For `knee_angle = motor_to_knee_scale * motor_angle + offset`, the
commanded motor effort is `motor_to_knee_scale * knee_resistance_effort` by
virtual work. This handles reversed mounting and transmission ratio; the
published effort is capped at `resistance_motor_max_effort_nm` (default 0.3 Nm,
matching the current effort controller). If the mapped effort would assist
the measured motor motion, the node publishes zero for that tick. The
mapping and effort cap are configuration values, not measured defaults. The
DM4340 hardware configuration must select effort mode; velocity and effort controllers
must never claim the same device concurrently. Neither this node nor preview
starts the hardware lifecycle. Zero effort is not a brake, and no worn
exoskeleton resistance trial is justified without mounted, validated IMUs,
physical joint mapping and a separately verified stop procedure.

This package owns the optional application-level link from the calibrated
dual-IMU knee estimate to the existing DM4340 ros2_control velocity controller.
It does not open CAN, activate hardware, enable the motor, or change the
driver's deployment gates. `mech_knee_kinematics` remains receive-only.

`knee_motor_follow_node` subscribes to `knee/joint_state` and
`knee/diagnostics`. The default `output_mode=preview` publishes only
`knee_motor_follow/preview_velocity`. It uses the offline-tested 0.25 Hz
reference, started from the knee angle at arm, with configurable amplitude
(default 20 deg). Angle error, reference rate and knee rate produce a velocity
target; knee-side and motor-side bounds are each 1.5 rad/s and 1 rad/s^2.
This reference and these gains are experimental, not validated assistance
parameters. There is no foot-pressure gate while insoles are disconnected.

Only `control_strategy=velocity_follow` with `output_mode=motor` publishes on
`/dm4340_velocity_controller/target_velocity` (`std_msgs/Float64`, rad/s).
This velocity mode also requires `joint_mapping_verified=true`, a nonzero measured
`motor_to_knee_scale`, and the corresponding offset in radians. It subscribes
to `/joint_states`, selects `motor_joint_name`, and checks that the mapped
motor position agrees with the fresh knee estimate within 0.15 rad. The
scale converts motor position/velocity to knee position/velocity; it is **not**
automatically the DM4340's internal gearbox ratio. Obtain this mapping from
the actual assembly before selecting motor mode.

For an **unloaded shaft-only** periodic trial without IMUs, set
`output_mode=motor` and `input_mode=motor_feedback_simulation`. This separate
mode takes motor JointState position at startup as a virtual zero, treats shaft
motion relative to it as a simulated knee angle, and never subscribes to IMU
topics. It does not assert an anatomical joint mapping or sensor accuracy.
The default limits are 0.5 rad relative shaft travel and five seconds. An
explicit unloaded-bench profile may configure at most 45 deg, 15 seconds,
and 1.0 rad; the IMU-driven mode retains its original bounds. The
physical driver still requires its own verified bringup settings, activation
and stop procedure; this node does not set or bypass them. This mode must not
be used on a worn exoskeleton.

After the existing DM4340 hardware and velocity controller have been
independently activated, the bench-only application node can be started with:

```bash
ros2 run mech_assist_control knee_motor_follow_node --ros-args \
  -p output_mode:=motor -p input_mode:=motor_feedback_simulation
ros2 service call /knee_motor_follow/arm std_srvs/srv/Trigger '{}'
```

This is not a combined startup script and does not activate or disable the
motor. Stop/deactivate the existing controller and hardware using the
established bench procedure. Never set `*_verified` bringup parameters solely
to make this command start.

The IMU-driven modes require healthy knee diagnostics and fresh knee samples before
`/knee_motor_follow/arm` succeeds. Motor mode additionally requires fresh
motor feedback. Shaft-only simulation synthesizes its knee input from fresh
motor feedback and does not require IMU diagnostics. Invalid, repeated or stale IMU/knee data, unhealthy
diagnostics, stale/invalid motor feedback or position mismatch latch a fault.
The node publishes one zero target and stops refreshing, allowing the existing
controller's command TTL and hardware fault handling to take over. Call
`/knee_motor_follow/reset` and then arm again only after new valid data. Motor
mode defaults to a five-second fault limit; the explicit shaft-only bench
profile may use a longer bound. This is a bounded integration
trial, not continuous wearable assistance. A
zero target or ROS shutdown alone is **not** a physical motor stop guarantee.

For a non-actuating inspection in a sourced ROS 2 workspace:

```bash
ros2 run mech_assist_control knee_motor_follow_node
ros2 topic echo /knee_motor_follow/diagnostics
ros2 topic echo /knee_motor_follow/preview_velocity
```

The motor mode is a code path for staged bench integration, not a ready-to-run
wearable launch. It must run with an independently checked motor-to-joint
mapping, both IMUs fixed and physically calibrated, verified motor-side stop
behavior, and the existing `mech_bringup` gates. This package never sets those
gates or activates `dm4340_system`; do not treat simulated ROS messages as
deployment evidence.

## Desktop IMU bench entry

`desk_imu_follow_node` is a separate preview-by-default bench application for
unmounted IMUs. It maps relative-orientation change about standard Y to bounded
velocity; it does not assert a motor-to-knee mechanical mapping. See the
[Chinese bench design](../../../docs/development/desktop_imu_follow_bench.md)
for fixed limits, lifecycle runner and per-session hardware gates. It neither
replaces the calibrated knee application nor enables hardware by itself.

### 无 IMU 阻力预览

`ros2 launch mech_assist_control motor_resistance_preview.launch.py` 从 `/joint_states`
读取电机输出轴编码器，显式 arm 后预览有死区的粘性阻尼力矩，不启动硬件或 IMU。
配置示例的轴名、范围和限幅需按设备核对；默认不发布电机目标。
详见[整机架构无 IMU 入口](../../../docs/development/exoskeleton_control_architecture.md#无-imu-电机阻力入口exo-arch-001)。

### 无 IMU 轴运动助力预览

`ros2 launch mech_assist_control motor_assist_preview.launch.py` 使用独立 motion_assist
策略，按轴运动方向提供有限助力；死区静止归零、速度渐退/截止、指令做功预算和短时限约束。
阻力配置保持原样。此模式没有人体意图识别，不能保证松手即停；默认仅预览，未做真实助力验收。
参数与限制见[整机设计](../../../docs/development/exoskeleton_control_architecture.md#无-imu-轴运动助力motion_assist)。


双轴无IMU抗阻训练：`policy=resistance_training`，预览入口
`dual_motor_resistance_training_preview.launch.py`；逐轴阻尼、时间常数滤波、
静止与反向卸力复用统一supervisor。参数与边界见
[抗阻训练说明](../../../docs/development/resistance_training.md)。

## 可选抗阻快环

新增通用 `ResistanceTrainingController`：借用位置/速度状态，在500 Hz目标更新周期计算有界反向力矩；
Python只发送50 Hz许可。原有应用抗阻路径保留，快环需显式选择。
参数、`resistance_training_fast`入口、许可TTL与下层保护边界见[抗阻快环说明](../../../docs/development/resistance_fast_loop.md)。
软件及无设备回环已验证，新路径实机阻力与抖动尚未验收。

## 默认训练入口：位置阻抗训练

`ros2 launch mech_assist_control motor_training_preview.launch.py` 默认选择
`policy=impedance_training`，只做预览、默认不输出电机许可。
统一上层监督新鲜度/范围/运行时长及显式arm；通用C++
`FixedPositionImpedanceController` 负责固定参考PD和力矩限幅，上层只发0/1许可。
原 `resistance_training` 及所有明确命名的速度抗阻配置保留。
配置、后端配对与参考点生命周期见[位置阻抗训练](../../../docs/development/fixed_position_impedance.md)。

## 规范分层入口（2026-10-23）

上层选择模式、参考角度和K/D/力矩上限；C++唯一执行实时PD；hardware/runtime独立保护。
固定位置控制器显式 `command_source=training_command` 接收原子8字段计划，旧enable台架入口保留且互斥。
统一上层的其他Python反馈策略只允许离线预览，尚未迁移的模式不允许motor输出。
完整字段、生命周期、默认参数和限制见[训练指令合同](../../../docs/development/training_command_contract.md)。
