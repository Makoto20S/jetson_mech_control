# AK3.0 生产组合点切片计划（让 launch 路径接到真机）

- **Date:** 2026-09-08
- **Branch / PR:** `feat/ak30-bench-composition`（已创建，基于 PR #12 分支 @ `649ed1f`），未来开 **PR #13**，base = `feat/ak30-torque-velocity-command-interfaces`（stacked 链 #11→#12→#13，合并顺序严格同序，每层合并后 `gh pr edit <n> --base <下一层>` 重定向）。
- **Status:** 计划已获 owner 批准（2026-09-08 plan mode）；**实施完成（2026-09-08 同日，Commits 2-5 全部落地，见文末 Deviations）**。
- **Goal:** 补上 2026-09-07 调查确认的缺口：launch → 真机之间没有生产组合点。交付 pluginlib 可加载的组合插件，使 `ros2 launch mech_bringup motor1_bringup.launch.py`（broadcaster-only）能收到真电机 50 Hz 反馈。**全部离线验证；真机首跑是下一任务，需 owner 逐次授权 + 到场。**

## Owner 语境（实施前必读）

- Owner 2026-09-07 曾因架构疑虑暂停开发（「给每个电机都写个新东西」的观感）；疑虑已由 `docs/development/architecture_package_map.md`（commit `649ed1f`）澄清，owner 复核后说「可以，没什么问题」并批准本计划。**本切片是架构对照文档 §5 缺口①的修复，不是新框架**——一个 `mech_bringup` 组合点，复用全部不变的 core/controllers/composite。
- 两个 PR（#11/#12）owner 明确「硬件测试跑过之前不合」；本切片正是通往硬件测试（broadcaster-only 首跑）的桥。
- Jetson 新 IP `172.20.15.186`（key SSH 已验证；旧 117 已死）。

## 关键设计决策（探索代理已逐条核实，2026-09-08 会话）

1. **继承法**：去掉 `CompositeSystem` 的 `final`（`composite_system.hpp:39`，一词改动，不改任何方法签名/语义），新类 `mech::mech_bringup::Ak30System : public CompositeSystem`。生命周期/claim/switch/看门狗浮出全部继承自已测试基类，零委托样板。
2. **透传 init 在 on_configure 发**：插件 `on_configure`：构造/打开串口 → `send_pass_through_init()` → 委托基类（其 `runtime_->configure()` 用 `transport.capabilities()`）。匹配 ros2_control「INACTIVE=通信已开始」语义，URDF 仅加载不碰设备。探针实证顺序（open→init→session configure→activate）逐位保留；runtime `start()` 的 `transport_.open()` 变成幂等复检。
3. **串口工厂注入测试接缝**：插件持有 `std::function<std::unique_ptr<CdcSerialPort>(const std::string&)>`（默认构造 `PosixCdcSerialPort`；测试注入 FakeSerial 包装）。测试可逐字节断言 0x12 init 帧、驱动完整生命周期——全部离线。与仓库既有 `set_runtime`-before-`on_init` 接缝同构。

## 已核实的事实基础（设计时不用重查）

- `set_runtime` 在 `on_init` 成功后永久失效（`composite_system.cpp:105-109`：`if (initialized_ || runtime == nullptr) return false;`）——**注入必须发生在插件 `on_init` 内、委托基类之前**（hpp L70-71 注释明文）。
- `info.hardware_parameters` 是 `std::unordered_map<std::string,std::string>`，`Ak30RuntimeParams::parse` 收 `std::map`——插件内做一次构造期转换（on_init 非实时，可分配）。
- `UsbCdcTransport::open()` fail-closed 于 `verified_board_version`（必须 `{4,8,8}`，`UsbCdcCodec::kMinimumBoardVersion`）与 `logical_bus != 0`；`nominal_bitrate_hz=0` 合法（能力三态诚实上报）。
- 探针 options 实证值（`ak30_position_probe.cpp:92-96`）：`logical_bus=1`、`nominal_bitrate_hz=0`、`receive_queue_capacity=128`、`verified_board_version={4,8,8}`。
- runtime 借用 `Transport&`（构造签名 `Ak30ForceControlRuntime(Transport&, Clock, Ak30RuntimeConfig)`，串口/transport 必须比 runtime 长寿）——插件按成员声明顺序持有 serial → transport →（runtime 经 set_runtime 注入基类）。
- **0x12 init 金帧（勿重算 CRC，用已验证字面值）**：`F7 12 06 00 7D 70 08 00 00 00 00 00 00`（13 字节，cfg=0x00）。来源：`posix_cdc_serial_port.cpp:109-146` 与 MEMORY 里 2026-09-02 的实测记录；厂商 fdcan_init 的 cfg 位（0x00 vs 0x07）已实测无差异。
- **CMake 两个硬阻塞**：① mech_bringup 库非 SHARED（`CMakeLists.txt:18`，pluginlib 要求共享库）；② 缺 `pluginlib_export_plugin_description_file`。照抄 `mech_hardware_ros2_control/CMakeLists.txt:35-36` 的注册模式。
- pluginlib 注册三件套（照抄 CompositeSystem 的）：源文件 `PLUGINLIB_EXPORT_CLASS`（注意宏在 `pluginlib/class_list_macros.hpp`）；CMake `pluginlib_export_plugin_description_file(hardware_interface <xml>)`；package.xml `<export><hardware_interface plugin="${prefix}/<xml>"/></export>`。xml 里 `<library path="mech_bringup">` 必须匹配库名（`libmech_bringup.so`）。
- `context_check.py` 不受影响：mech_bringup 内部依赖集已含 `mech_hardware_ros2_control`+`mech_protocol_cubemars`；新增 `pluginlib`/`hardware_interface` 是外部依赖，检查不可见。不新增包。
- `test_deployment_files.cpp` 目前**没有**插件名断言（要新增 pin）；`motor1_bringup.launch.py` 无插件名（插件名来自 xacro），无需改。
- `FakeSerial`（`mech_simulation/fake_serial.hpp`）是纯字节层 CdcSerialPort 双打：`inject_rx`/`take_tx`/`force_next_read|write`——测试经 `UsbCdcCodec` 编解码路径驱动，与既有 transport 测试同构；0x12 init 帧可直接从 `take_tx()` 逐字节断言。
- `PosixCdcSerialPort::open()` 对不存在的设备返回 false 不挂起（`posix_cdc_serial_port.cpp:43-45`）——「默认工厂 fail-closed」可离线测。

## 实施步骤（TDD，一个任务一个 commit）

### Commit 1 — 切片计划文档
`docs/development/plans/2026-09-08-ak30-composition-slice.md`（沿用 2026-09-06/07 plan 模板：Goal/架构/File structure/TDD 任务/验证/Deviations）。

### Commit 2 — `final` 移除 + `Ak30System` 插件类
- `composite_system.hpp`：`class CompositeSystem final` → `class CompositeSystem`（注释说明允许子类化是组合点用法，ADR-003 边界内；品牌分支禁止规则不变）。
- 新 `mech_bringup/include/mech_bringup/ak30_system.hpp` + `src/ak30_system_plugin.cpp`：
  - `on_init(info)`：`Ak30RuntimeParams::parse`（`std::map` 转换；fail → ERROR）→ 暂存 params → 构造 runtime（clock lambda = steady_clock，仿 `ak30_position_probe.cpp:57-61`）→ `set_runtime` → 委托 `CompositeSystem::on_init(info)`。**注意顺序：set_runtime 必须在委托之前。**
  - `on_configure(prev)`：经工厂构造 `CdcSerialPort` → `UsbCdcTransport`（options 对齐探针）→ `transport.open()` 失败 → ERROR → `serial->send_pass_through_init()` 失败 → ERROR → 委托 `CompositeSystem::on_configure`。
  - 工厂成员：默认 `PosixCdcSerialPort`；公开 setter 供测试注入（须在 on_init 前设置，同 set_runtime 纪律）。成员声明顺序保证析构逆序安全（runtime 在基类，serial/transport 在派生类——**注意：派生类成员先析构，基类 runtime 后析构**；runtime 在 stop 后不碰 transport 即可，与既有 harness 一致，写测试时验证）。
  - on_cleanup/on_deactivate 的清理语义继承基类（runtime stop() 不关 transport；如需关闭，插件在 on_cleanup 里 close——对齐探针 teardown `session.deactivate(); transport.close();` 的顺序，但**不要在 on_deactivate 关**，deactivate→activate 重入是合法生命周期）。
  - `PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::Ak30System, hardware_interface::SystemInterface)`。
- CMake：`add_library(${PROJECT_NAME} SHARED ...)` + 新源文件 + `pluginlib_export_plugin_description_file(hardware_interface mech_bringup_plugins.xml)`；package.xml `<export>` 加 `<hardware_interface plugin="${prefix}/mech_bringup_plugins.xml"/>`。
- 新 `mech_bringup_plugins.xml`：注册名 `mech_bringup/Ak30System`，`base_class_type="hardware_interface::SystemInterface"`，`<library path="mech_bringup">`。

### Commit 3 — 插件离线测试（FakeSerial 工厂注入）
新 `test_ak30_system_plugin.cpp`（加入 CMake 测试源列表）：
- 默认工厂：合法参数 on_init 成功、on_configure 对不存在设备 fail-closed（ERROR，不挂起）。
- 注入 FakeSerial 工厂：on_init（URDF 参数 → HardwareInfo）→ on_configure 后 **`take_tx()` 逐字节断言 0x12 金帧**；完整生命周期经导出接口往返（FakeSerial `inject_rx` 厂商反馈帧字节 → read 后 joint states 出现解码值）；未知参数/缺 device_path 在 on_init 拒绝。
- 反馈帧注入参考既有 transport/session 测试的字节构造（feedback 90°/10000 ERPM/2 A fixture，`test_ak30_system_integration.cpp:72-91`）。

### Commit 4 — URDF/部署接线 + 结构校验
- 三个 xacro 的 `<plugin>`：`mech_hardware_ros2_control/CompositeSystem` → `mech_bringup/Ak30System`（`motor1.urdf.xacro:33`、`motor1_torque.urdf.xacro:40`、`motor1_velocity.urdf.xacro:43`）。
- `test_deployment_files.cpp`：新增 pin——三变体都含 `<plugin>mech_bringup/Ak30System</plugin>`。
- mech_bringup README 组件清单加 `Ak30System` 一行。

### Commit 5 — 收尾文档
plan doc Deviations 填写实际执行偏差；`architecture_package_map.md` §5 缺口①改为「已补（PR #13）」；design doc/README 如有对应条目同步。

## 明确不做（范围外）

- 不 launch 真机、不发任何命令、不部署 Jetson（真机 broadcaster-only 首跑是**下一任务**：需 owner 逐次授权 + 到场 + ADR-006 Decision 7 边界；届时先 scp 本分支 → Jetson 干净构建 → broadcaster-only launch 看 /joint_states）。
- 不动 `mech_protocol_cubemars`/`mech_control_core`/`mech_controllers`。
- 不合并任何 PR；`final` 移除是对已合并代码的最小改动，随 PR #13 一起 review。
- Torque/Velocity ros2 控制器不在本切片（DemoController position-only 维持）。

## 验证与交付

1. `MECH_OUTPUT_ROOT=/tmp/<x> MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`（预期 >246 tests 全绿；本机注意 miniconda python3 问题——用干净 env）
2. `bash tools/ci/run_sanitizers.sh /tmp/<y>`（`-fsanitize=address,undefined` 经 CMake cache 核实）
3. `env -i PATH=/usr/bin:/bin:/usr/local/bin HOME=/home/admin2025 /usr/bin/python3 tools/ci/context_check.py` + `check_adrs.py`（10 decisions: 8 Accepted, 2 Proposed）+ `git diff --check` + 新 xml/xacro 过 `xmllint --noout` + docs 链接检查
4. push 走 7890 代理：`git -c http.proxy=http://127.0.0.1:7890 -c https.proxy=http://127.0.0.1:7890 push -u origin feat/ak30-bench-composition`；`HTTPS_PROXY=http://127.0.0.1:7890 gh pr create --base feat/ak30-torque-velocity-command-interfaces ...`；等两套 checks 绿
5. 结束时更新 memory STATE/PLAN + validator 0 errors；handoff 仅在 owner 要求或真实交接事件时创建（本切片交接文档已存在，见下）

## 风险与注意

- 三层 stacked PR（#11→#12→#13）管理复杂度——合并顺序严格，不可逆序；每层合并后重定向下一层，否则 base 断链。
- `final` 移除放宽基类子类化——滥用面由架构对照文档 §6 判据 + context_check 依赖集钉死；本切片的子类是组合点，不是品牌分支。
- 析构顺序：派生类成员（serial/transport）先析构、基类 runtime 后析构——确保 stop 路径后 runtime 不再触碰 transport（现状满足：read/write 都有 started_ 守卫；写测试时用 ASan 复核）。
- pluginlib 宏的 include 路径是 `pluginlib/class_list_macros.hpp`（Humble）。
- 0x12 帧的 CRC **不要重算**——用上文金帧字面值（重算一次踩过坑：09-07 会话的 0.2 N·m 金帧手算 nibble 装包出错，靠逐字节测试拦下）。

## Deviations（实施期发现与计划的事实失配/偏差，2026-09-08）

实施时逐条核实了「已核实的事实基础」，以下偏差全部是执行层面的发现，未推翻任何设计决策：

1. **0x12 金帧的来源重构（偏离计划的 PosixCdcSerialPort::send_pass_through_init 调用路径）。** 计划让插件调用 `serial_->send_pass_through_init()`，但该方法在 `PosixCdcSerialPort` 上、`CdcSerialPort` 接口上没有——工厂返回接口指针，插件拿不到具体类型。修复：金帧提升为共享字面值 `pass_through_init.hpp` 的 `kPassThroughInitFrame` + 自由函数 `send_pass_through_init(CdcSerialPort&)`；`PosixCdcSerialPort::send_pass_through_init()` 改为委托它（删除了运行期 CRC 重算，字面值先经独立 python 复现逐字节核对一致）。这同时消除了一个隐患：原实现每次调用都重算 CRC，与「勿重算金帧」纪律相悖。
2. **`std::function` 工厂不能捕获 `unique_ptr`（move-only 不可拷贝）。** 第一版测试工厂 lambda move 捕获 `unique_ptr<FakeSerial>`，`std::function` 要求可拷贝目标，编译失败。改为 `shared_ptr`（插件持有 `shared_ptr<CdcSerialPort>`），语义不变——插件每个 on_init 生命周期仍只持有一个串口。
3. **串口/transport 对象必须在 on_init 构造、on_configure 只做 I/O（比计划更精确的生命周期分配）。** 计划写「on_configure 经工厂构造串口→transport」，但 runtime 构造签名借用 `Transport&`，若 transport 在 on_configure 才构造，on_init 注入的 runtime 引用将悬垂（实现第一版正是在这里 segfault，被插件测试当场拦下）。修正为：on_init 完成纯对象构造（PosixCdcSerialPort 构造只存路径、UsbCdcTransport 构造只存引用+能力，均不碰设备），整个硬件生命周期内地址稳定；on_configure 才发生第一次设备 I/O（open + 0x12 init），与 ros2_control「INACTIVE=通信已开始」语义和探针顺序一致。
4. **mech_bringup 转 SHARED 引发全仓 PIC 联动。** 计划指出「CMake 两个硬阻塞」（非 SHARED + 缺 pluginlib export），但未预见：静态链接下游五包的 `libmech_bringup.so` 需要 PIC，而 `mech_control_core` 等全部是默认静态库。修复：`tools/ci/build_workspace.sh` 与 `run_sanitizers.sh` 增加 `-DBUILD_SHARED_LIBS=ON`（全部六包转为共享库；gtest 目标链接同一批对象，不受影响——254 tests 0 failures）。Docker CI 走 build_workspace.sh，自动继承。
5. **pluginlib 离线加载验证的 ClassLoader 构造参数易错（计划外补充验证）。** 用独立程序验证 ament 索引注册时，`ClassLoader` 第一参数必须是基类包名 `hardware_interface`（第三参数 attrib_name 默认 `plugin`）；最初传 `mech_bringup` 导致 0 声明类的假阴性。修正后：声明类含 `mech_bringup/Ak30System` 且 `createSharedInstance` 成功——注册链端到端验证（未写入测试，属一次性验证，事实记录于此与 commit message）。
6. **on_cleanup 在 active 状态会被基类拒绝（测试预期修正，非实现缺陷）。** `CompositeSystem::on_cleanup` 有 `if (active_) return ERROR` 守卫，Reactivation 测试按计划草案直接 cleanup 而未先 deactivate，返回 ERROR 而非 SUCCESS。测试改为标准生命周期路径（deactivate→cleanup）——这是基类已测试语义的正确行为，不是 bug。
7. **执行偏差（流程）**：Commit 2 的 CMake 测试源列表编辑与 Commit 3 的测试文件分开提交时顺序颠倒（列表先行、文件后到），已在提交信息中说明；无功能影响。
