# 从 CubeMars 开发提炼的经验

这些是本项目的决策经验，不是其他电机的规格。按当前步骤选择相关主题；实际源码/测试与 ADR 优先。

## 协议与物理映射

- 曾将 AK2.0 手册用于 AK3.0 驱动板讨论，造成 profile 判断错误。先匹配驱动板/固件，再使用“MIT/伺服”等名称对应的字段。来源：ADR-013、MVP 计划。
- ERPM、机械 RPM、输出轴速度不能混用。电流到输出力矩需要 Kt/传动/符号证据；定制电机不能只借同系列规格。来源：ADR-009/013、协议包映射校验。
- 软件零偏、设备临时原点和编码器/FOC 标定是不同量。保持目标成功不能证明零点正确；持久性单独验证，不无授权置零或掉电。

## 网关与总线

- USB CDC 4,000,000 baud 不证明 CAN 仲裁/数据段速率；历史板固件 4.8.8 不证明当前两块板相同。未知值以 unverified 表达。来源：ADR-012、transport 能力。
- 实机回复曾有文档未写出的三字节 CAN ID 前缀，纯离线测试发现不了。先保留 raw bytes，再加入有匹配条件的兼容解析和负例；不“猜偏移直到有数”。来源：usb_cdc_transport 实现与测试。
- 端口别名可能绕过字符串比较。复用 inode flock，并在改 termios/flush 前取得锁，保留 O_CLOEXEC。包已部分写入后背压不能整包重发，需按流协议判故障。来源：posix_cdc_serial_port、PTY 隔离测试。
- 主机 TX、网关回显、另一块同类板的抓包只能证明各自观察点。怀疑 ID/载荷错配时保存 raw USB、预期帧与独立观察，不直接归咎电机、CRC 或内核。

## 运行语义

- Humble 在 INACTIVE 下仍可能调用 read/write。claim 才授权运动命令，撤销须取消 pending；默认数值零不是合法“无命令”标记。来源：ADR-015、CompositeSystem 回归。
- 标准 JTC 不必支持自定义 generation，保留当前强/弱档区别。硬件循环和同一缓冲值不证明上游持续刷新。来源：ADR-017/018。
- 控制器激活有首周期接管空隙；按当前契约从已接受的新鲜反馈初始化位置，不重放旧缓冲；命令组合的辅助字段也要初始化。来源：CompositeSystem、position_feedforward_design。
- Unknown/Stale/Invalid 不得变成零反馈。反馈 freshness 要结合真实回报周期和抖动，不照抄命令租约。来源：ADR-016。
- 停止发送不等于失能或机械静止；硬件故障后 STRICT 停用可能被拒。恢复走明确生命周期/设备协议，ACK 不替代静止观测。来源：hardware/controllers README、servo_disable。

## 验证与诊断

- Codec 完成不代表 launch 已加载生产 runtime，默认 loopback 也会产生“正常”状态。核对 XML/xacro 和真实模块加载，用 fake/PTY 注入实际生产链路。来源：architecture_package_map 历史缺口、bringup 测试。
- 共享 CI 的宿主延迟曾触发毫秒级租约，使同源码树 PR 绿、merge 红。功能用逻辑时间和 peer 同步，精确过期仍测，真实宿主性能单独测。不得放宽生产保护、删生命周期断言或重跑求绿。来源：foundation_validation。
- 测试时钟保持反馈顺序；controller_manager 暂停后的 catch-up 不能超前于合成网关。测试接缝仅 BUILD_TESTING，保留 raw host trace 和时间映射，不提供生产绕过开关。
- ROS 退出可能不执行插件析构，终态回调封存证据、析构兜底；普通 deactivate 可再激活，不能提前封存。来源：Ak30ServoSystem on_shutdown。
- “编译通过、接收数据、电机运动、准确跟踪、安全停止”分别验收。单机历史台架不代表新设备/多机资格，缓存数值也不等于新鲜有效状态。
