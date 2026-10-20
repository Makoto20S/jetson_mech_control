# mech_bringup

## 单总线、互斥电机配置

`launch/motor_backend_bringup.launch.py` 在启动时要求明确指定
`backend:=ak30` 或 `backend:=dm4340`，以及同一物理 CAN 通道对应的
`device_path:=/dev/ttyACM<N>`。它只加载所选品牌的一套 URDF 和控制器配置；
控制器默认保持未激活。缺少品牌或串口、或请求同时选择两种品牌时会直接拒绝。
状态广播器成功加载后才加载所选电机控制器；广播器加载失败或控制器管理器退出时，
启动进程会关闭，以释放已打开的设备。启动失败后应确认设备已释放，再排查配置。
达妙的四项验证参数可显式传入，但默认均为 `false`；未全部验证时，
启动器在创建 ROS 节点前直接拒绝。只有取得每项实际证据后才能将其改为 `true`，
不能为通过启动检查而直接填写 `true`。
切换品牌必须完整停止当前启动进程、确认串口已释放，再启动另一种配置；
不能通过同时运行两个插件来实现切换。底层串口的非阻塞独占锁仍阻止两个
进程同时配置和使用同一设备。两种电机的帧格式无需一致，但实际物理 CAN
速率必须相同；CubeMars 在此配置中的速率尚未实机核实，因此此入口不是
混用或实机启动的验证结果。不要仅因控制器未激活就将硬件视为失能。

## 达妙 DM4340 速度链路（离线集成）

`mech_bringup/Dm4340System` 与 CubeMars 的 `Ak30System` 并列注册为
`ros2_control` 硬件插件。一个达妙插件管理一条 USB-CAN 网关及最多 6 个
达妙关节；每个关节有独立 MIT session、CAN 命令 ID、反馈有效性和命令期限。
反馈 CAN ID 可以相同，接收后按载荷中的电机编号分流。重复命令 ID 或电机编号
在打开串口前拒绝。上层仍使用
`mech_controllers/VelocityCommandController` 输入关节速度（rad/s）；选择
`dm4340_command_mode:=effort` 时改用同一插件的
`mech_controllers/EffortCommandController` 输入关节力矩（N·m），
读取关节位置、速度、力矩；达妙 MIT 编码、ID 和 USB-CAN 网关收发留在
达妙插件内。两种电机不能共享同一个设备参数文件，但不需要在通用控制器里
判断品牌。

两种插件共用 `seven_channel_gateway_options()` 构造 USB-CAN 传输参数。
CubeMars 的 `nominal_bitrate_hz=0` 表示板上 CAN 仲裁率未知，不能据此推断
达妙通道的速率；板级版本证据也只能在确认是同一块物理通信板时复用。

配置示例为 `config/dm4340_velocity.urdf.xacro` 和
`config/dm4340_velocity_controllers.yaml`；只有明确选择 `backend:=dm4340`
时，互斥启动入口才会加载它们。
示例只声明一台电机；同一网关上增加电机应在同一个 `<ros2_control>`
组件中增加关节和对应控制器，不能复制插件并让两个组件打开同一串口。
当前不支持 CubeMars 与达妙在同一物理网关内混用；那需要跨协议组合运行时，
不能靠两个插件争抢串口实现。
示例中的电机侧通讯超时、网关版本、CAN 速率和关节映射验证位默认是
`false`，因此不能直接激活实机。必须先独立核对电机侧超时确实能够在
主机停发后停止输出、物理 CAN 速率为 1 Mbit/s、网关版本和电机到关节的
方向与比例，再据实填写验证位。`nominal_bitrate_hz` 是驱动的已核实声明，
不是对七路板重新设置波特率。电机固件、ID、MIT 范围也必须与实物一致。

插件按启动模式只接受同时申领、同时释放 `velocity` 或 `effort` 与
`command_generation` 的控制器，要求
电机已有有效的使能反馈。若显式设置 `enable_on_activate:=true`，则仅在
`ros2_control` 硬件组件从 `inactive` 手动切换到 `active` 时，等待新鲜的
状态 `0` 或 `1`：状态 `0` 时发送达妙 `0xFC` 使能帧，状态 `1` 时直接接管且
不重复使能；两条路径均须在 1.2 秒内确认状态 `1`。反馈缺失、过期或故障时
激活失败；若已接管或发送使能帧，则尝试发送 `0xFD` 失能。此选项默认
`false`；启动文件将达妙硬件初始化为 `inactive`，不会因启动
`controller_manager` 而使能。硬件停用或故障
时会尝试失能。需先经独立验证并填写前述四项证据，再使用
`ros2 control set_hardware_component_state dm4340_system active` 明确触发使能；
结束时使用 `ros2 control set_hardware_component_state dm4340_system inactive`，
并独立核对反馈状态。只有硬件已激活且收到使能反馈后，才能手动激活预加载的
所选的速度或力矩控制器；话题本身不能使能电机。一次新目标对应一代命令；
旧目标、超时反馈、越界速度/力矩或错误状态均停止继续发送运动命令。
单路撤销申领时只停止并锁住该路；任一路故障或硬截止时停止整组。
停止时会尽力发送中性帧和失能帧。
这些主机侧操作不能代替经过验证的电机侧超时和现场断电措施。
离体电机上的速度控制演示与力矩控制模式是不同验证项。力矩模式目前只通过
ARM64 构建、协议假传输和 ROS 图测试，尚无实际力矩输出记录；不能将速度
演示或硬件激活成功视为抗阻训练通过。力矩模式使用 `Kp=Kd=0` 的 MIT 前馈
力矩，示例上限为关节侧 0.3 N·m，仍保留新鲜使能反馈、速度/力矩反馈限值、
命令期限与失能处理。零力矩并非制动；使用前必须验证电机与膝关节映射、
双 IMU 固定标定及现场停机手段。

仅做离体单电机、无传感器的短时力矩链路检查时，可显式选择
`dm4340_command_mode:=effort supervised_effort_bench:=true`。此模式允许
`motor_side_timeout_verified:=false joint_mapping_verified:=false`，并把
MIT 命令上限收紧为 0.1 N·m；板版本与 CAN 速率仍须核实。它只使用
电机轴坐标 (`joint_scale=1`, `joint_offset_rad=0`)，不能接入膝关节
抗阻策略或人体负载。控制器 YAML 上限仍为 0.3 N·m，实际台架目标
必须不超过 0.1 N·m，否则底层拒绝命令。该模式不提供电机侧断联停机：
主机退出、USB 断开时，零力矩/失能帧可能无法送达。仅在有人现场监护且
能独立切断电机电源的空载固定台架上使用。普通部署没有改变默认门槛。

2026-10-15 在 `/dev/ttyACM1` 只读查询本台 DM4340：参数 20 `Gr` 的
IEEE-754 浮点值为 `40.000`，参数 10 控制模式为 `1`（MIT），参数 9
超时值为 `0`；状态反馈为 `0`（失能）。`Gr` 是电机内部减速比配置，
不能在未确认 MIT 反馈坐标含义和外部联轴器传动比时再次乘以 40。
当前 `joint_scale=1` 仅为离体电机轴示例，不是外骨骼膝关节标定结果。
控制循环只使用位置、速度、力矩反馈及已配置的映射和限值；减速比、
固件和模式等静态参数通过独立诊断读取，不按控制周期重复查询。

ROS 硬件激活不会读取或修改电机参数 9；只按反馈状态选择接管已使能电机
或发送使能指令，并确认使能反馈。参数 9 的诊断留在独立台架工具中，不参与
正常控制循环。
`motor_side_timeout_verified` 仍是部署配置的显式证据门，不会因删除自动
寄存器操作而被置为 `true`。

Deployment composition boundary: bring-up harnesses, device probes, and the
AK3.0 force-control runtime wiring. The `Ak30System` plugin
(`mech_bringup/Ak30System`) is the production composition point that connects
`mech_protocol_cubemars` to `mech_hardware_ros2_control`'s `CompositeSystem`;
all three deployment xacro examples load it.

## Components

- **`Ak30ForceControlRuntime`** — the `RuntimePort` implementation that wires
  the AK3.0 force-control session into `CompositeSystem`. It is the first
  production consumer of `command_stage()` (ADR-012's staged watchdog):
  fresh controller writes are submitted while Following, the last valid
  command is frozen through Holding, and Expired faults the system within
  the 3-cycle budget. It never resolves a missing command to `0.0`. Since
  [ADR-014](../../../docs/adr/ADR-014-ak30-submode-command-interfaces.md)
  it maps commands per sub-mode (Position reads position and optional desired
  velocity/feedforward effort, Velocity reads velocity with effort forced to 0,
  Torque reads effort).
- **`Ak30System`** — the production composition point as a pluginlib
  plugin (`mech_bringup/Ak30System`): it injects the serial→transport→runtime
  chain (`PosixCdcSerialPort` -> `UsbCdcTransport` ->
  `Ak30ForceControlRuntime`) into `CompositeSystem`, and `on_configure` is
  the first device I/O (transport open plus the 0x12 pass-through init). The
  three deployment xacro examples point at it.
- **`Ak30RuntimeParams`** — fail-closed parsing of the URDF `ros2_control`
  hardware parameters into the runtime config. `device_path` is mandatory;
  unknown keys, non-numeric values, and over-budget TTLs reject configure.
  `sub_mode` is an explicit parameter (`position`|`velocity`|`torque`,
  default `position`); the URDF's command interface must match it.
- **`Ak30ServoSystem`** — the position-only servo plugin
  (`mech_bringup/Ak30ServoSystem`). A single serial port and
  `UsbCdcTransport` feed one `Ak30ServoRuntime`, which uses the existing
  `BusRuntime` and one `Ak30ServoPositionSession` per joint. The exported
  state is position alone; each joint declares position and
  `command_generation` commands. A standard position controller may claim
  position alone, while a controller that also claims generation gets the
  stronger refresh rule. No velocity or effort state is inferred from raw
  eRPM or Iq.
- **`Ak30ServoRuntimeParams`** — strict, I/O-free parsing for that servo
  plugin. Hardware parameters must include `profile=ak30_servo_extended`,
  `device_path`, `logical_bus`, `control_period_ns`,
  `command_ttl_ns`, `command_hard_ttl_ns`, and `feedback_ttl_ns`.
  Optional `trace_name` (1–32 lowercase letters, digits or hyphens) partitions
  bounded snapshot filenames when multiple servo components share one manager.
  All other unknown parameters remain rejected. The operator admits the total
  memory budget before starting any component and divides it across the buses.
  Each joint must explicitly supply `drive_id`, separate verified
  `target_scale`/`target_offset` and `feedback_scale`/`feedback_offset`,
  `speed_erpm`, `acceleration_raw`, and
  `position_min_rad`/`position_max_rad`/`position_max_error_rad`.
  Unknown or missing keys, duplicate IDs, unsafe time budgets, or invalid
  interface declarations reject initialization before a serial port is made.
- **`FoundationHarness`** — the Foundation-era simulation harness
  (CompositeSystem + TargetLimiter through exported interfaces).
- **Device probes** (`ak30_torque_probe`, `ak30_position_probe`) — the
  bench-validated bring-up probes, built only with
  `-DMECH_BUILD_DEVICE_PROBES=ON`.
- **Deployment examples** (`config/motor1.urdf.xacro` for Position,
  `config/motor1_torque.urdf.xacro` for Torque,
  `config/motor1_velocity.urdf.xacro` for Velocity,
  `config/motor1_controllers.yaml`, `launch/motor1_bringup.launch.py`) —
  single-motor examples carrying motor1's bench-evidenced parameters,
  one per force-control sub-mode. The position-controller spawner ships
  commented out: uncommenting it arms position commands, which stay
  gated by ADR-006 and per-test owner authorization.

## Servo position composition

`Ak30ServoSystem::on_init` validates the complete declaration, constructs
one stable serial/CDC chain, and installs the runtime into `CompositeSystem`
without opening a port. Configure validates only. Activation lets the
runtime's `BusRuntime` acquire the shared physical-bus registry entry and
open the port, then sends the gateway's existing 0x12 pass-through init once
for that activation before any read/write cycle. A second plugin using the
same physical path cannot open it while the first owns it. Deactivation,
error, cleanup, and active destruction stop the runtime, revoke pending host
commands, and close the port. Reactivation needs new feedback and a new
claim; it sends the gateway init again. The init count is separate from
motor-frame transmit statistics.

Both production plugins construct `PosixCdcSerialPort`. Its open path takes
a nonblocking advisory lock on the opened device inode before terminal
configuration or flush. This excludes another repository process using the
same serial implementation, including one naming the port through a symlink;
close or destruction releases the lock. It does not prevent unrelated tools
that ignore advisory locks from opening the device.

During active cycles, `CompositeSystem` owns exported state, claims and
generation checks; `Ak30ServoRuntime` drives the shared bus and existing
servo sessions. All configured feedback routes must be fresh before any motor
target can be sent. Invalid or stale sampled feedback faults the group and
cancels pending host submissions. Releasing one claim cancels only that
joint's pending target. Stopping submissions does not establish that either
physical motor has stopped; the plugin sends no automatic disable, zero,
hold or keepalive frame. The offline fixtures use synthetic mappings solely
to verify this composition. They are not calibrated deployment parameters.

Command TTLs require `0 < soft < hard`, with hard bounded by three control
periods and 6 ms. An unsent target fails at soft expiry. A target actually
accepted by the transport stops refreshing at soft expiry and remains in the
host Holding stage until its original hard deadline. Expiry is checked even
when the receive budget is exhausted and before a fresh write can replace an
old target. Claim release clears that member's watchdog history. None of
these host states proves that the device's internal trajectory has stopped.

## Position deployment envelope (ADR-019)

`config/motor1.urdf.xacro` requires three hardware parameters in Position
sub-mode: `position_min_rad` and `position_max_rad` (motor1 example: `-12.0`
and `6.0`) bound where the shaft may be commanded to go, and
`position_max_error_rad` (motor1 example: `0.5`) bounds how far a command may
sit from the latest usable feedback position. This bounds the nominal
proportional term `Kp * (command - measured)`: with motor1's `Kp = 1 N*m/rad`,
0.5 rad means a 0.5 N*m proportional-term budget. It does not bound total
torque, which also includes the Kd velocity-error term and optional feedforward.
For scale, static friction on the unloaded shaft is about `0.2 N*m`. Missing,
non-finite, `min >= max` or non-positive values reject the hardware at
initialization (`on_init`), before any device I/O. The other sub-modes do not
require these parameters, but the set is all-or-nothing everywhere: once any
one of the three appears, all three must be present and valid or `on_init`
rejects the hardware.

The check runs in the AK3.0 runtime before submission and never clamps. The
absolute bounds are inclusive and are checked on every Position command; the
error bound is evaluated only on cycles where the feedback sample was judged
usable, so a command is never compared against a stale or absent number. A
violation drops the command — no frame carrying that value is transmitted —
latches the runtime, emits the `PositionEnvelope` telemetry reason, and makes
`read()` fail every cycle until `configure()`/`start()` runs. Claim
cancellation does not clear the latch. Because the gate sits in the shared
hardware layer, it binds **every** controller that claims the joint, including
upstream `ros2_control` controllers that the project did not write; that is the
point of it. The post-latch behaviour is the same one the T9 overspeed latch
showed (see [T9 effort deployment](#t9-effort-deployment)): ros2_control moves
the hardware into its error state and rejects STRICT deactivation there, so
bench tooling must take post-latch rest evidence from a fresh passive
observation. See
[ADR-019](../../../docs/adr/ADR-019-hardware-position-envelope.md); its status
remains Proposed; the historical Kp4 latch is recorded below, and no new
combined-command hardware acceptance is claimed.

### What the error bound costs this deployment

The absolute bounds are unchanged from what the controller YAML already
enforced. The error bound is new, and at motor1's shipped `Kp = 1` it is a real
limit on how far a bench move may reach, not a formality:

- At `Kp = 1` the shaft lags far behind the command. On the 2026-09-05
  progressive bench route a `+10 deg` target moved the shaft `0.2 deg` while
  the command itself completed, and the move settled at `9.8 deg` of error
  (`0.171 rad`); the `+30 deg` run settled at `15.7 deg` (`0.274 rad`). Source:
  `docs/planning/README.md` §3, 阶段 2 ("位置步进 +2°/+5° 与 +30° 全程
  （Kp=1 摩擦稳态误差）").
- `PositionCommandController` ramps its command open-loop at
  `max_slew_per_second: 2.0 rad/s`, regardless of where the shaft actually is.
  It does not wait for the shaft to catch up.

Neither settled error above would trip the bound on its own — both are under
`0.5 rad`. The transient is what does: while the ramp is in flight the command
runs away from a shaft that has barely moved, so command-minus-measured error
peaks well above the error the move eventually settles at. Under the shipped
`position_max_error_rad = 0.5`, a single target much farther than `0.5 rad`
(`29 deg`) from the current position is therefore expected to exceed the bound
part way through the ramp and latch the hardware before the shaft reaches the
target, after which STRICT deactivation will be refused until the controller
manager is restarted (the T9 finding above). This is a prediction from the ramp
shape and the `Kp = 1` bench numbers; the distinct Kp4 trial below later
observed an envelope latch. The `2.0 rad/s` slew rate is likewise not reachable under
this envelope at `Kp = 1`: with `Kd = 1` and zero commanded velocity, holding
`2 rad/s` would need more than `2 rad` of error.

The `0.5 rad` value is the owner-approved one and is kept, because it is the
proportional-term budget (`0.5 N*m` at `Kp = 1`), not a travel budget. Bench procedures
must still plan each move as if it were one: treat `29 deg` as the ceiling on a
single target displacement — the T10 trajectory's `0.1745 rad` step is inside.

## T8 velocity deployment

Use the separate `launch/motor1_velocity_bringup.launch.py` with
`config/motor1_velocity.urdf.xacro` and `config/motor1_velocity_controllers.yaml`.
It serializes the broadcaster and velocity spawners and loads
`motor1_velocity_controller` **inactive**. Launching still opens the physical
transport; inactive does not make this an offline launch. The offline test
evaluates the actual xacro/launch parameters without starting any nodes.

The controller consumes `/motor1_velocity_controller/target_velocity`
(`std_msgs/msg/Float64`, rad/s); its example bounds are ±0.5 rad/s, with a
0.5 rad/s² command ramp. The URDF uses Kp=0, Kd=1, and the runtime forces t_ff=0.
For an approved future bench run: stop the other deployment, verify current
feedback and shaft rest, activate using a STRICT switch, verify ACTIVE, then
send fresh zero targets before any bounded nonzero ramp. Keep publishing at
20–50 Hz. This configuration does not switch the deployed position operator.

Normal stopping requires continued fresh zero targets through the ramp and
confirmation of actual near-zero speed before deactivation. Target loss or
deactivation stops refresh/transmission; it does not promise mechanical braking.
Velocity mode supplies evidenced velocity/effort feedback, **not position**;
the position state placeholder must not be used to infer displacement or rest.
The launch therefore omits robot_state_publisher. Initialization is independent
of that position state; hardware feedback validity is still required to claim.

See [controller semantics](../mech_controllers/README.md) for target lifetime,
activation and recovery. Real velocity-controller acceptance remains a separate
authorized bench task; offline frame mapping does not establish physical limits.

## T9 effort deployment

`launch/motor1_effort_bringup.launch.py` pairs `motor1_torque.urdf.xacro` with
`motor1_effort_controllers.yaml`, serializes the spawners and loads the effort
controller inactive. It opens the physical transport when executed; constructing
the launch description in an offline test does not. The fixed Torque profile
uses Kp=Kd=0 and only the effort field. No hot mode switching is supported.

Targets arrive on `/motor1_effort_controller/target_effort` as Float64 in N*m.
Example limits are +/-0.1 N*m and 0.2 N*m/s, with upstream TTL100/106 ms and
hardware lease4/6 ms. Normal zero return must continue publishing through the
ramp and verify coast-down using fresh raw ERPM; neither zero effort nor
deactivation guarantees a mechanical stop. Physical force accuracy needs
independent metrology; the exported effort is estimated from Iq and Kt.

Torque deployments require an explicit finite positive `torque_max_abs_erpm`
hardware parameter (motor1 example:300). The AK3.0 runtime checks raw signed
ERPM from accepted feedback before submitting pending torque. Exceeding the
absolute limit latches a host fault and stops submission, including when a
later sample in the same drain is below the limit. Claim cancellation cannot
clear that fault. Feedback loss remains subject to the60 ms validity window.
This guard is sample-based and stops sending; it does not brake, guarantee a
physical speed bound or replace local emergency power removal.

Optional feedback telemetry includes `raw_erpm_available` and `raw_erpm`, tied
to accepted RX sequence/time. These are device diagnostics, not a promotion of
Torque mode's unsupported canonical velocity. Tools must reject missing/stale
diagnostics and never use placeholder velocity/position as rest evidence.

Bench acceptance on motor1 (2026-09-17, unloaded shaft, Kp=Kd=0): a zero-effort
1 s trial completed the full activate/zero/rest/deactivate cycle, and a
+0.2 N*m trial with a 5000 ERPM ceiling echoed 0.21-0.22 N*m while static
friction held the shaft, then accelerated at roughly 28 rad/s^2 after breakaway
and latched the overspeed guard within 0.2 s. Pure torque with no damping and
no load has no steady speed, so any raw ceiling below the motor's own limit
ends such a run; a sustained visible spin needs a damping term or a load.

Once the overspeed latch fails `read()` closed, ros2_control moves the hardware
into its error state. A subsequent STRICT deactivation of the effort controller
is rejected there ("Not acceptable command interfaces combination") and the
controller's update returns errors until the manager is shut down; submission
has already stopped and the shaft coasts. Bench tooling must therefore obtain
its post-latch rest evidence from a fresh passive observation instead of from
the deactivation acknowledgement. Physical force accuracy still needs metrology.

## Standard controller deployment (JTC)

`motor1_trajectory_bringup.launch.py` loads the upstream
`joint_trajectory_controller/JointTrajectoryController` using
`config/motor1_trajectory_controllers.yaml` and the Position hardware xacro.
Install `ros-humble-joint-trajectory-controller` on the target host first.
The state broadcaster spawns first; JTC then loads **inactive**. Starting the
launch still opens the physical transport and requires the bench gates below.

JTC claims only `motor1_joint/position`, with position-only state feedback;
it does not claim `command_generation`. This is ADR-017's weak tier: at the
configured 500 Hz, every authorized manager cycle refreshes the hardware
command and can produce a frame. A controller that stays active but stops
writing fresh targets is invisible to that freshness check and may keep its
last target indefinitely. A stalled manager still expires the hardware lease.
This is the accepted weak-tier risk recorded as R23 in the MVP plan.

ADR-019's hardware envelope checks the absolute target and its error from
usable feedback before submission; it applies regardless of controller type.
An offending command is dropped and latched, never clipped into range. The
shipped bounds are `[-12, 6] rad` and a `0.5 rad` maximum error. This is a
command boundary, not a physical stop guarantee or an upstream-target watchdog.
After a latch, obtain fresh passive rest evidence: STRICT deactivation can be
refused while the hardware is in its error state.

The offline integration test loads the actual upstream plugin through
`ControllerManager`, with `CompositeSystem`, the AK3 runtime, and
`FakeTransport`; no serial port is opened. It covers measured-position hold,
a bounded out-and-back trajectory, continued hardware cycling after
controller deactivation, and the envelope's frame rejection and latch.
Humble JTC 2.54 queues a hold in `on_activate()` and writes its command
interface on the next `update()`. To cover the intervening hardware write,
CompositeSystem initializes each new weak position claim from the feedback
already accepted in that cycle (ADR-017). This applies again after release
and reacquisition; it never seeds strong claims, velocity, effort, release-only
or rejected switches. The shipped JTC parameter
`set_last_command_interface_value_as_state_on_activation: false` also makes
JTC's internal trajectory use measured state. Neither mechanism strengthens
the weak-tier silence watchdog or guarantees a physical stop.
A separate persistent Jetson bench operator now exposes
`./trajectory run [Kp=6] [Kd=1]`, plus `check` and `observe`, from the deployment
root. It observes a fresh center per run, keeps the same trajectory timing, and
accepts Kp 0..18 / Kd 0..5. The tuning error envelope is 0.3 rad for Kp <= 10
and 1.8/Kp rad above 10 (0.1 rad at Kp=18). This bounds only the nominal
proportional term, not total torque. Deployment validation is not physical
Kp=6 tracking evidence. See the deployed Chinese operating guide.

Follow-up gain trial (2026-09-18): the owner authorized Kp=4/Kd=1 with the same
trajectory. The temporary candidate tightened position error to 0.075 rad,
retaining a nominal 0.3 N*m proportional-term ceiling (not a total torque bound).
At about 2 s, reference displacement was 5.0 degrees but feedback displacement
was 0.7 degrees. PositionEnvelope (reason 12) latched; the round trip did not
complete. STRICT deactivation was refused after the latch, as described above.
A fresh passive run confirmed rest, zero motor commands, and port release.

Motor1 bench evidence (2026-09-18): an owner-authorized JTC action completed
+0.1745 rad in 4 s, held its target for 1 s, and returned the target in 4 s;
STRICT deactivation, fresh post-stop rest, fault-free feedback and port release
passed. Actual feedback moved only about 0.6 degrees from the start and ended
about 0.5 degrees above it: action success is functional interchangeability
evidence, not accurate trajectory tracking. Two earlier strict zero-motion
hold attempts aborted on 40 ERPM; the owner waived that prerequisite to proceed,
so a successful two-second zero-motion hold is not claimed. ADR-019 remains
Proposed pending owner acceptance; the follow-up Kp=4 trial above observed a
physical position-error envelope latch.

Later diagnostic trials used Kp=18/Kd=5 and the same 10-degree displacement,
with two repeats each of 4 s and 2 s legs. Outbound speed coefficient of
variation fell from 0.453–0.461 to 0.245–0.260 with faster motion, while absolute
speed standard deviation increased from about 17–18 to 21–22 ERPM. An
angle-correlated current/voltage component near a 3.214-degree output period
persisted. Cross-speed voltage/angle fits with a roughly 26 ms relative lag
reached R² 0.960–0.979; neither the fit nor the period identifies a unique
sensor, commutation, encoder or mechanical root cause. Host writes were near
2 ms apart; this does not establish actual CAN delivery or device timing.

All four diagnostic actions completed, but their immediate post-deactivation
rest checks failed; separate fresh passive checks subsequently confirmed rest,
zero motor commands and port release. UART telemetry changed mode about 3 s
after the last host transmission, so deactivation is not proof of immediate
rest or torque disable. Precise tracking and full stopping behavior remain
unqualified.

On 2026-09-21 the owner deferred further jerk/root-cause investigation until
it recurs in practical use, and authorized committing the lifecycle fix and
updating PR #20. This deferral does not claim that jerk is fixed, that stopping
is qualified, or that ADR-019 is accepted. Historical lifecycle validation ran
414 tests each on local Humble, native ARM64 and ASan/UBSan with no failures.
Diagnostic tools and raw captures remain local and are not part of this PR.

## Position desired velocity and feedforward extension

The hardware Position path accepts explicitly declared `position+velocity`,
`position+effort`, and `position+velocity+effort` bundles. The owning controller
writes position [rad], desired velocity [rad/s] and feedforward effort [N*m]
together before each hardware update; configured Kp/Kd are appended by the
adapter. This does not implement a gravity model or change drive calibration.

`position_max_abs_velocity_rad_s` and `position_max_abs_feedforward_nm` default
to zero. A declared auxiliary interface requires its explicit positive bound;
both must be finite and within wire ranges. An invalid auxiliary command
rejects/latches the whole tuple without transmitting an earlier pending target.
This emits `PositionTuple` (reason 13); existing reason numbers are unchanged.
Existing position envelope, feedback freshness and command deadlines still
apply. These limits do not bound total drive torque or physical speed.

Examples (software-only; values are not approved hardware settings):

- `config/motor1_position_velocity.urdf.xacro` with
  `config/motor1_position_velocity_trajectory_controllers.yaml` exposes P+V for
  the installed upstream Humble JTC. Existing position-only files are unchanged.
- `config/motor1_position_velocity_effort.urdf.xacro` exposes the full P+V+E
  bundle for a compatible single controller. **Humble JTC cannot command this
  bundle** because its effort interface must be used alone. No production
  full-tuple controller is shipped by this extension. Offline controller-manager
  tests exercise it through a test controller and decode actual fake frames.

Whole bundles must be claimed/released together. Deploy one owner per tuple;
aggregate hardware switch callbacks cannot identify controller ownership.
New claims clear old auxiliary values, and weak Position takeover seeds the
accepted position. Strong generation applies to the whole tuple; weak owners
remain unable to signal target staleness. Change declared bundles only through
inactive reconfiguration. No new real experiment or persistent deployment is
authorized by these examples; await owner approval after the PR.

## Safety boundary

No default build or test opens a serial device or sends a CAN frame. All
tests run against `mech_simulation::FakeTransport` with injected clocks.
Real-device activation is gated by [ADR-006](../../../docs/adr/ADR-006-conditional-can0-deployment.md)
(still Proposed) and the G0–G3 evidence gates; every real-motor run needs
the owner's explicit per-test authorization.
### 达妙 ROS 台架运动测试

`tools/bench/run_dm4340_ros_motion.py --execute` 只连接已启动的 `controller_manager`，不替代 `motor_backend_bringup.launch.py` 的四项验证门槛。启动参数必须明确包含 `backend:=dm4340`、正确设备路径和 `enable_on_activate:=true`；四项 `*_verified` 仅在取得各自实测证据后填写 `true`，不能仅凭寄存器写入成功设置。

`tools/bench/run_dm4340_ros_motion.py --preflight` 是同一入口的只读检查：查询硬件和控制器是否均为 `inactive`，不激活硬件、不发送电机指令。它要求已有 `controller_manager`，且不能与运动档位选项混用。

脚本要求硬件与速度控制器初始均为 `inactive`。它通过 ROS 服务激活硬件并确认状态，再激活速度控制器。`--execute --diagnose-only` 仅发布零速度，分别观察硬件和控制器约 2 秒；普通 `--execute` 在两段 0.5 秒零速度观察后，向 `/dm4340_velocity_controller/target_velocity` 发布 1.5 rad/s 目标 0.5 秒，再发布零速度 0.8 秒。控制器以 1 rad/s² 限斜率，电机并不会立即收到 1.5 rad/s。脚本读取 `/joint_states` 的 `dm4340_joint`，检查反馈新鲜度、位移不超过 0.35 rad、速度不超过 2 rad/s、最终停稳，并在结束或异常时释放控制器和停用硬件。测试仅适用于已验证方向和自由行程的离体电机轴；返回成功要求实际位移至少 0.005 rad 且停稳。主机清理失败时必须现场断电；脚本不能代替电机侧断联保护。

离体台架需要更明显的单次演示时，可显式使用 `--execute --extended`：非零目标 0.8 秒、零目标 1.0 秒，位移上限 0.8 rad，其他反馈和停机检查不变。此选项不改变默认测试或启动门槛。

五秒档必须显式使用 `--execute --five-second --operator-supervised`：非零目标 5 秒、零目标 2 秒，位移上限 9 rad，反馈速度上限仍为 2 rad/s；启动前还检查沿运动方向的反馈量程。`--operator-supervised` 只声明现场有人可立即断电，**不证明**电机侧断联停机。主机断联时，软件位移限幅无法继续执行。

周期闭环台架档使用 `--execute --follower-simulation --operator-supervised`。脚本在硬件激活并取得真实反馈后启动 `mech_assist_control` 的 `motor_feedback_simulation` 模式，用电机轴相对角度模拟膝角；算法节点发布周期速度目标，经 ROS 速度控制器返回电机。周期阶段 4 秒，位移限值 0.48 rad；随后持续发送零速度 1 秒，再停用控制器和硬件。成功判据包括双向目标、实际轴位置变化至少 0.02 rad、最终速度不超过 0.05 rad/s。它不读取 IMU，也不能证明真实膝角标定或人体助力效果。2026-10-16 的一次空载实测范围为 8.7623--9.0152 rad，目标为 -0.2667--0.2072 rad/s，最终独立回读状态 0。该次启动的四项验证参数由操作者手动设为 `true`，不是各项独立验证完成的证据。

需要更明显、连续的观察时，使用 `--execute --follower-extended --operator-supervised`：目标范围 0--20 度、两个 4 秒周期，算法运行上限 9 秒，仍保持 0.48 rad 位移边界、2 rad/s 反馈速度边界和 1 秒持续零速收尾。2026-10-16 的一次空载实测轴位置范围为 8.7623--9.1072 rad（0.3449 rad，约 19.8 度），双向目标 -0.2593--0.2743 rad/s，结束后独立回读状态 0。此档位同样只验证电机轴模拟膝角闭环，不等同于人体膝角或穿戴控制。

`--execute --follower-40deg --operator-supervised` 是单独的空载大幅度档位：40 度、三个 4 秒周期，运行上限 13 秒、位移边界 0.75 rad，其他反馈速度和零速收尾检查不变。2026-10-16 一次获准的台架实测轴位置 8.7505--9.4665 rad（0.7160 rad，约 41.0 度），双向速度目标 -0.5454--0.5723 rad/s，峰值反馈 0.5690 rad/s，停机后独立回读状态 0。该试验距离 0.75 rad 软件边界仅剩约 0.034 rad 的范围余量，不能据此继续增大角度或用于人体负载。
