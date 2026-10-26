# mech_controllers

Bounded C++ controller package boundary. Controllers do not open CAN sockets
or branch on vendor model names.

## PositionCommandController

The plugin `mech_controllers/PositionCommandController` shapes a single joint's
position command with bounds, slew limits and a monotonic target watchdog.
It claims `position` and `command_generation`, reads `position`, and accepts
`std_msgs/msg/Float64` targets on `~/target_position`. It does not calculate
the motor's impedance torque or configure its gains.

The motor1 instance remains `motor1_position_controller`, so its target topic
remains `/motor1_position_controller/target_position`. Target lifetime defaults
remain 100/106 ms; the independent hardware lease remains 4/6 ms (ADR-017/018).
Activation seeds the held position from finite feedback and requires a new
target; deactivation does not substitute a zero command.

### Migration from DemoController

Change plugin types from `mech_controllers/DemoController` to
`mech_controllers/PositionCommandController`. C++ consumers must include
`mech_controllers/position_command_controller.hpp` and use
`mech::mech_controllers::PositionCommandController`. Rebuild dependent packages
and install into a fresh prefix so obsolete headers and plugin metadata cannot
mask an incomplete migration. The old class/header/plugin name has no alias.
The linkable `mech_controllers` library and separate `mech_controllers_plugin`
registration library retain their names and separation.

Update operator scripts that check the controller type together with the
framework deployment. Keep existing controller instance names, target topics,
parameters and gain settings. The rename preserves position behavior; it does
not authorize another hardware run or certify physical accuracy or timing.

## VelocityCommandController (T8)

The independent plugin `mech_controllers/VelocityCommandController` claims
`velocity` and `command_generation` and accepts `std_msgs/msg/Float64` on
`~/target_velocity` in rad/s. It claims no state interfaces. The hardware's
feedback validity gate still applies before activation.

`minimum`/`maximum` bound velocity and must include zero. `max_slew_per_second`
bounds the commanded acceleration in rad/s². These are command shaping limits,
not measured acceleration or overspeed protection. Activation starts the internal
ramp at zero, discards previous targets and requires a fresh explicit target
before refreshing generation. It does not measure or assume a stationary shaft;
the operator must confirm rest before activation/recovery.

Position and Velocity share the concrete `SingleJointCommandController` target
mailbox, limiter, monotonic clock, activation epoch and generation pipeline.
Position retains measured-position seeding; Velocity never reads position.
Both use the ADR-018 upstream policy (100/106 ms by default) independently of
the hardware's 4/6 ms lease. Same-value messages refresh upstream lifetime;
NaN/Inf and inactive submissions do not. Messages are dated when their callback
is consumed; Float64 has no source timestamp, so this does not bound upstream
network/queue age. Use a continuously running publisher and executor.

For normal stopping, continuously publish fresh zero velocity targets until the
command ramp reaches zero **and actual speed is confirmed near zero**, then
deactivate. Stale input freezes command and generation at the soft deadline and
returns ERROR at the hard deadline; deactivation stops generation refresh.
These fault paths do not synthesize zero velocity or guarantee mechanical braking.
Recovery requires reactivation and new targets; stale targets are not replayed.

The motor1 example is `motor1_velocity_controllers.yaml`, paired with the
velocity URDF and `motor1_velocity_bringup.launch.py` in `mech_bringup`. It loads
the controller inactive, with ±0.5 rad/s bounds and 0.5 rad/s² acceleration.
There is no runtime mode switch. Native motor impedance uses PositionCommandController with vendor-side gains; the experimental host impedance plugin has been removed.

## EffortCommandController (T9)

The independent plugin `mech_controllers/EffortCommandController` claims `effort`
and `command_generation`, accepts `std_msgs/msg/Float64` on `~/target_effort`
in N*m, and claims no state interfaces. It uses the same bounded target pipeline
as Position and Velocity. Bounds must contain zero; `max_slew_per_second` is
the commanded effort slew in N*m/s. Activation seeds the internal ramp at zero
and requires a fresh target before refreshing generation.

An explicit fresh zero target ramps commanded effort to zero. **Zero effort is
not a stop or brake**: a free shaft may coast. Observe actual rest separately
before deactivation. Target expiry and deactivation stop refreshing commands;
they do not synthesize braking torque. Reactivation discards previous targets
and restarts the ramp from zero. Hardware feedback validity still gates claims.

The motor1 example uses `motor1_effort_controllers.yaml` and
`motor1_effort_bringup.launch.py`, loads inactive, and bounds effort to +/-0.1 N*m
with 0.2 N*m/s slew. In AK3.0 Torque mode the effort feedback is an Iq-derived
equivalent estimate; standard position/velocity fields are unsupported. The
vendor runtime's raw ERPM guard is separate from this generic controller and
must be checked alongside fresh feedback in the bench procedure.

## 可选抗阻快环

新增通用 `ResistanceTrainingController`：借用位置/速度状态，在500 Hz目标更新周期计算有界反向力矩；
独立台架只发送50 Hz许可。原有Python应用抗阻路径仅保留离线预览；统一motor入口现要求结构化位置阻抗计划。
参数、`resistance_training_fast`入口、许可TTL与下层保护边界见[抗阻快环说明](../../../docs/development/resistance_fast_loop.md)。
软件及无设备回环已验证，新路径实机阻力与抖动尚未验收。

## 固定位置阻抗（显式可选）

新增 `mech_controllers/FixedPositionImpedanceController`：激活时固定编码器参考点，
Jetson每次update计算K15/D0.5的PD力矩并饱和到±7N·m，内部MIT Kp/Kd=0。
`dm4310_dual_bringup.launch.py fixed_position_impedance:=true`默认只做离线配置检查；
真实分支仍默认inactive，没有自动使能。旧速度抗阻与原生位置模式保留。
参数、STM32来源及验证边界见[固定位置阻抗说明](../../../docs/development/fixed_position_impedance.md)。

## 规范分层入口（2026-10-23）

上层选择模式、参考角度和K/D/力矩上限；C++唯一执行实时PD；hardware/runtime独立保护。
固定位置控制器显式 `command_source=training_command` 接收原子8字段计划，旧enable台架入口保留且互斥。
统一上层的其他Python反馈策略只允许离线预览，尚未迁移的模式不允许motor输出。
完整字段、生命周期、默认参数和限制见[训练指令合同](../../../docs/development/training_command_contract.md)。
