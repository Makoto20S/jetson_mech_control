# Dual-IMU knee kinematics (experimental)

## Jetson CAN dashboard

The read-only dashboard consumes the existing CtrBoard ROS bridge topics; it
does not open `/dev/ttyACM0` itself or send CAN/motor commands. On a Jetson
desktop with ROS 2 Humble, `python3-tk`, and access to the Livelybot USB-CAN
channel, build the workspace and start the bridge, estimator, and GUI together:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mech_knee_kinematics mech_ctrboard_bridge
source install/setup.bash
./tools/bench/run_ctrboard_can_dashboard.sh /dev/ttyACM0
```

If running the script from another directory, set `MECH_INSTALL_SETUP` to the
absolute `install/setup.bash` path. The dashboard displays IMU1 (thigh), IMU2
(shank), calibrated knee flexion, two 20-point pressure grids, connection state,
and CSV recording. The two calibration buttons call the existing ROS services:
stand straight and still for the standing zero, flex/extend 3-5 times for the
functional axis, then repeat standing zero. Calibration is held in memory only.
An uncalibrated angle, invalid IMU orientation, or data older than 500 ms is
displayed as unavailable and is blank in the CSV. The pressure grid is indexed
0-19, not an anatomically registered insole map. The GUI does not start the
motor stack and is not a safety monitor for motor control.

The ROS bridge needs exclusive access to the USB-CAN gateway channel. Close
other bridge/terminal readers first. `/dev/ttyUSB0` from a CP210x adapter is
not the Livelybot CAN channel; use the verified `/dev/ttyACM*` endpoint. A
temporary `setfacl` grant is lost when that USB device reconnects.
The GUI distinguishes a live packet stream from bridge safety diagnostics:
older bridge builds that do not publish periodic diagnostics show
"安全诊断未提供" even while IMU samples arrive. Upgrade the bridge separately
before relying on its per-field safety status.

This receive-only ROS 2 node consumes the existing CtrBoard bridge topics
`imu/imu1/data` (thigh) and `imu/imu2/data` (shank). It does not send CAN frames
or motor commands. `dual_imu_knee.py` reuses the PC viewer's pure estimator core, with an additional
matching body-vector basis transform for the ROS adapter. The node uses the same standing relative-quaternion zero,
gyro-based functional hinge-axis calibration, signed twist angle, gyro prediction,
orientation correction and 6 Hz display low-pass filter. Synchronize changes to
the PC core and this packaged copy together. This is **not** Song and Baxter's
complete magnetometer-free orientation-fusion pipeline; anatomical accuracy
still requires an independent synchronized angle reference.

## Calibration and use

Start the existing CtrBoard bridge, then in a sourced ROS 2 workspace:

```bash
ros2 run mech_knee_kinematics knee_kinematics_node
ros2 service call /knee/start_static_calibration std_srvs/srv/Trigger '{}'
```

Keep the knee straight and both sensors still for the default 1.5 seconds.
Then flex and extend the knee 3-5 times during the six-second action capture:

```bash
ros2 service call /knee/start_axis_calibration std_srvs/srv/Trigger '{}'
ros2 topic echo /knee/flexion_deg
ros2 topic echo /knee/diagnostics
```

The functional calibration rotates the shank angular velocity into the thigh
frame, subtracts thigh angular velocity and applies the PC viewer's PCA and
70% dominant-axis quality gate. It stores separate hinge axes in both sensor
frames. Return to a straight standing pose and run the static-zero service once
more, as in the PC workflow. The returned angle is the filtered twist of the
shank relative to the thigh around the calibrated axis. `knee/joint_state`
reports radians and rad/s; `knee/flexion_deg` reports degrees. No angle is
published before both calibrations or on invalid paired data. A software angle
limit or rejected orientation jump is reported in diagnostics, not a motor
safety approval.

The bridge stamps both IMUs from one telemetry packet identically, so this node
pairs by exact ROS header timestamp. A partial pair expires after
`max_pair_delay_ms` (default 100 ms); completed or expired timestamps cannot be
reused, and older packets are ignored. An invalid pair, incomplete pair or
whole-pair timeout (`pair_timeout_ms`, default 500 ms) resets the estimator's
filter state. Calibration capture aborts on a pairing interruption and must be
restarted. Diagnostics include dropped-pair and out-of-order message counts.
Resetting calibration clears the pairing cache and timestamp watermark. Set
`max_pair_delay_ms` below `pair_timeout_ms` to account for topic scheduling on
the target machine.

These timestamps are **host receive times**, not per-sensor hardware sample
times. Matching stamps proves only that the bridge decoded both IMUs from one
telemetry packet. The current wire protocol cannot prove that one IMU has
stopped refreshing while whole packets continue arriving. Hardware-level
freshness requires per-IMU counters or sample timestamps in a versioned
firmware/bridge protocol update; unchanged values alone are not a fault.

## Coordinate basis and sign

The default `basis_mapping=hipnuc` matches the PC viewer and the bridge's
current pass-through of native HiPNUC quaternion values. It maps quaternion
`(q0,q1,q2,q3)` to `(q0,q2,q1,-q3)` and both body vectors by
`(x,y,z)->(y,x,-z)`. The node also converts ROS rad/s to the PC estimator's
deg/s. If a future bridge publishes fully standardized `sensor_msgs/Imu`, use
`basis_mapping=identity` to avoid double conversion. Verify either choice with
isolated physical rotations and gravity; Euler agreement alone is insufficient.

`flexion_sign` defaults to `-1`, matching the PC viewer's initial setting.
After calibration, check one known flexion. If its sign is negative, restart
with `--ros-args -p flexion_sign:=1` and recalibrate. Never use an absolute
value to hide an axis-sign problem.

Calibration is deliberately not persisted: moving either sensor requires a
new static and functional-axis calibration. Record the installation, software
version, and a synchronized side-view video or goniometer measurement for any
accuracy claim. This estimate is not currently a motor safety input.

## Offline closed-loop fixture

### Simulated IMU to motor velocity

`knee_motor_sim` is a separate hardware-free algorithm test. It synthesizes a
thigh/shank IMU pair from a one-DOF virtual motor, uses the same calibrated
dual-IMU estimator as the knee node, and tracks a 0.25 Hz knee-angle reference.
The follower computes a velocity target in rad/s from reference angular rate,
angle error and measured angular rate. It limits the target to 1.5 rad/s and
its slew to 1 rad/s^2. Missing, stale, out-of-range or non-monotonic IMU data
immediately latch a zero target; reset and a fresh sample are needed to re-arm.
The virtual motor is only a first-order speed response, not a DM4340 or human
biomechanics model. There is no ROS motor topic, USB-CAN access or enable path.

```bash
PYTHONPATH=ros2_ws/src/mech_knee_kinematics python3 -m mech_knee_kinematics.motor_sim --csv simulated_motor_loop.csv
PYTHONPATH=ros2_ws/src/mech_knee_kinematics python3 -m unittest discover -s ros2_ws/src/mech_knee_kinematics/test -v
```

The CSV contains normal and IMU-dropout trials with angle reference, true and
estimated virtual knee angles, velocity target and feedback, input validity,
and latched state. Its tracking error describes this fixture only; it cannot
validate motor torque, knee-to-motor mapping, user interaction or real-time
ROS behavior. In particular, the controller's immediate zero target is not
proof that a disconnected physical motor stops. Connecting this algorithm to
the real velocity controller requires motor-feedback freshness, a measured
joint mapping and separate staged bench validation.

`knee_offline_loop` runs without ROS, CAN, STM32 or a motor. It generates a
fixed-thigh/shank-rotation IMU pair, calibrates the same estimator's virtual
hinge axis and zero, applies a bounded virtual PD effort policy with sample
timeout and latched faults, and integrates a one-DOF virtual joint with a
simple human-input term. The startup sample arms the virtual supervisor but
commands zero; only a subsequent fresh sample can produce effort. The
`imu_dropout` trial removes the sensor pair from 4.0 to 4.25 s; effort becomes
zero immediately and stays zero after samples return until an explicit reset
and re-arm. No ROS motor topic or hardware plugin is connected.

The optional `run_trial(True, pressure_gated=True)` fixture also supplies
synthetic foot pressure. A valid off-contact sample commands zero effort while
keeping the virtual supervisor armed; a missing or invalid pressure sample
latches a fault and requires an explicit reset. `contact_off` and
`pressure_dropout` select those two test windows. This is not a ROS runtime
controller, and its pressure threshold is a fixture value rather than a
calibrated shoe-insole threshold.

From the repository root on a host with Python 3.10 or newer:

```bash
PYTHONPATH=ros2_ws/src/mech_knee_kinematics python3 -m mech_knee_kinematics.offline_loop --csv virtual_loop.csv --png virtual_loop.png
PYTHONPATH=ros2_ws/src/mech_knee_kinematics python3 -m unittest discover -s ros2_ws/src/mech_knee_kinematics/test -v
```

The CLI prints baseline and assisted tracking RMSE against its *synthetic*
target. The CSV includes target, true virtual angle, estimated angle, virtual
effort, sensor validity and supervisor state for the baseline, assistance and
dropout trials. PNG export is optional and requires Pillow; CSV-only export
has no plotting dependency. These gains and error numbers are fixture parameters, **not**
an exoskeleton control design or evidence of human tracking accuracy. The
existing motor controller, command leases, device watchdog and G0-G3 gates
remain separate; connecting this output to `target_effort` requires an
independent design and authorized staged verification.

Reference: Song K, Baxter JR, *A Simple Minimum-Setup Pipeline for Using
Leg-Worn Inertial Sensors to Track Knee Flexion*, Sensors 2026,
https://doi.org/10.3390/s26123704.

## ROS shadow assistance check

`shadow_assist_node` is a separate virtual-output monitor. It consumes
`knee/joint_state` and `knee/diagnostics`, then publishes only
`assist_shadow/virtual_effort_nm` and `assist_shadow/diagnostics`. It does not
subscribe to foot pressure (insoles may be disconnected), publish a DM4340
target, open CAN, or enable hardware. The fixed virtual target and bounded
effort are debug fixtures, not exoskeleton control settings.

After physical static and axis calibration of the knee node, call
`/assist_shadow/arm` to enable virtual output. A degraded knee diagnostic,
invalid sample, or missing sample latches zero output. Recovery requires
`/assist_shadow/reset`, fresh healthy diagnostics and a fresh knee sample,
then a new arm request. `sample_timeout_ms` defaults to 300 ms for the
current approximately 10 Hz IMU stream.

The bounded sensor-only smoke test is `bash tools/bench/run_shadow_sensors.sh
12 /dev/ttyACM0` from a sourced ROS 2 environment. It starts the bridge,
knee estimator and shadow monitor, prints their diagnostics and virtual output,
then stops all three processes. It never arms the shadow policy or starts the
motor controller. On Jetson, `python3 tools/validation/check_shadow_ros_graph.py`
tests the virtual output, fault latch, reset and absence of a motor publisher
with synthetic ROS messages.

## Gait-synchronized virtual assistance

`gait_shadow_node` extends the simulation-only path with a periodic reference.
It consumes the estimated knee angle/rate, knee diagnostics, the selected
foot's total pressure, and CtrBoard's foot-phase status. Two successive
`swing -> heel strike` transitions establish a gait period (accepted range
0.6-2.5 s). Each subsequent knee sample advances a 0-1 cycle phase. A
cosine-shaped reference angle and a stance-only sine envelope produce a
bounded virtual PD effort; swing produces zero effort. The reference angle,
pressure threshold, and gains are simulation fixtures, not fitted anatomical
trajectories or motor parameters. This is not the STM32 AO controller: it
borrows the event-synchronized periodic-control idea without copying its
motor output or assuming its unverified gains are safe.

The node publishes only `gait_shadow/virtual_effort_nm`,
`gait_shadow/phase`, and `gait_shadow/diagnostics`. It has no publisher for a
motor target, no CAN handle, and cannot enable a motor. The service
`/gait_shadow/arm` accepts only fresh, healthy knee/pressure inputs after
the period is established. Missing, inconsistent, stale or invalid data
latches zero output; `/gait_shadow/reset` clears the latch and the learned
period. The foot phase is `0=swing, 1=heel strike, 2=midstance, 3=pushoff`.
The bridge does not provide an independent pressure validity bit or a shared
sample identifier for the total and phase topics, so the node checks their
host receive-time proximity but cannot prove they came from one wire packet.

`pressure_validated` defaults to `false`, and arming is therefore disabled.
The current bench insoles were disconnected, so their zero values are
placeholders. Set this parameter to `true` only for a controlled synthetic
test or after an independently observed nonzero insole/contact response.
With a sourced ROS 2 workspace, the virtual node can be started using:

```bash
ros2 run mech_knee_kinematics gait_shadow_node --ros-args -p foot_side:=left
```

The deterministic policy test needs no ROS. The ROS graph test uses synthetic
messages, explicitly opts into simulated pressure, checks phase progression,
stance effort, swing/fault zero and the absence of a motor publisher:

```bash
PYTHONPATH=ros2_ws/src/mech_knee_kinematics python3 -m unittest discover -s ros2_ws/src/mech_knee_kinematics/test -v
python3 tools/validation/check_gait_shadow_ros_graph.py
```

These tests demonstrate a sensor-to-*virtual*-effort software path only. The
IMUs must be fixed on separate leg segments and calibrated against a physical
angle reference, the insoles connected and validated, and a separately
specified joint-to-motor mapping and stop behavior verified before considering
any physical assistance loop.

## Jetson segment attitudes (IMU-ATT-001)

The receive-only node now also publishes each segment orientation, their relative
quaternion, standing-relative orientations and display RPY. It reuses onboard
IMU quaternions, not a new raw-sensor AHRS. See the
[Chinese design and usage guide](../../../docs/development/dual_imu_attitude_design.md)
for frame conventions, topic contracts, launch configuration and acceptance limits.

### Posture viewer

Run `ros2 run mech_knee_kinematics dual_imu_attitude_viewer` in a sourced graphical
Jetson session. Complete the estimator calibrations to enable the two-segment
pose, RPY, knee angle/rate and rolling charts. Stale or unhealthy input greys
the animation and blanks live values. `--demo` provides labelled synthetic
animation without initializing ROS or accessing devices. The viewer only subscribes;
it does not open serial ports, command motors or invoke calibration services.

For unmounted sensors on a desk, use `--raw`: it shows independent I1/I2
orientations, RPY and pitch histories with bridge validity diagnostics; knee
angle remains uncalibrated. Both sensor orientations update atomically on a
matching packet stamp, retaining the previous complete pair only until its
original freshness deadline.

### PR #34 数据质量与时间合同

源header必须与节点ROS时钟同域；IMU `source_max_age_ms` 默认300ms且不得超过配对超时，
迟到但合法的样本保留原年龄，未来/过期源不参与配对。
估计质量或角度/速度校验失败时不发布控制JointState，立即发布诊断WARN，
短暂恢复不能替代下游显式故障复位。原始姿态可视化不等于有效膝角控制输入。
处理及未完成实机项见[审核记录](../../../docs/validation/2026-10-23-pr34-review.md)。
