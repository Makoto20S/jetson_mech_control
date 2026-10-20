# 新适配器验证矩阵

根据改动选择测试，复用现有设施。离线测试用 fake/vcan/PTY 隔离资源，不自动启用物理接口。

| 层 | 要证明的行为 | 证据 |
|---|---|---|
| Wire/codec | 正常字节、端序/符号/边界、错误 ID/DLC/flag/CRC、NaN/Inf | 独立 golden 与负例，注明版本；不能只靠 encode/decode 自循环 |
| 映射 | 单位、方向、零偏、限位；不支持字段不伪造可用 | 数值向量、配置拒绝、状态子集测试 |
| Session | 固定 profile；重复/乱序/丢失反馈、过期命令、锁存/恢复 | 注入时钟，覆盖精确边界，新目标不能覆盖已有过期 |
| Runtime | 单写者/路由隔离、无 claim 零运动命令、pending 撤销、背压/断连 | fake transport，多资源撤销隔离，错 ID/载荷不跨路由 |
| ROS 组合 | 生产插件、声明、claim/switch、停用/重启无旧目标 | 真实 controller_manager/JTC + fake/PTY，检查编码帧而非仅配置文本 |
| CI | 新包参与依赖、构建、测试、报告、sanitizer | 精确提交的当前 workflow；失败/跳过/缺失分开统计 |
| 已授权台架 | 本设备身份/配置、反馈、受限控制、停止/恢复及多机范围 | 软件/固件/配置/环境和原始证据，不扩大既有授权 |

## 时间与保护

- 分开目标有效期、命令租约、反馈 freshness、ROS readiness 等待与宿主周期性能，每个值有时钟域和依据。
- 逻辑时间验证 soft/hard 边界及“在已过期时写入新目标仍故障”。不要用放宽期限让测试通过。
- ROS 就绪采用有界状态等待，不盲用 sleep；不能替换所有 ROS wall clock 而改变被测功能。
- 测试时间接缝仅限测试构建；raw trace 保留原时间，逻辑审计有独立映射。宿主性能在明确平台/负载/构建下测试，不从共享 VM/sanitizer 取得硬实时结论。
- 普通 PR 默认普通完整 CI；时钟/CI 改动才按当前契约选 20 轮/冷缓存专项。失败全部记录，修复后在新提交验收，不挑一次绿色。

## 覆盖真实接入与已有设备

- 共用 serial/transport/CompositeSystem/schema 有改动时，保留既有 CubeMars 回归。
- 新包进入 Docker rosdep、context 清单、报告汇总、CMake/CTest。核对 discovery，零用例不算通过。
- 新增测试插件时检查 BUILD_TESTING=OFF 不注册它，生产 launch 不落到 loopback。
- Windows 仅可移植检查；ROS/Humble/vcan/原生 ARM64 在匹配 Linux 运行。区分隔离构建与真实 Jetson 部署，依赖安装可能修改系统，不因运行测试自动获得部署授权。

## 失败证据与完成度

保存精确源码/构建、固件依据、脱敏配置、目标/claim/generation、首故障、原始 TX/RX 与编码帧、时间域、进程/ROS 日志、XML 和退出码。有界保留，取消也收集；partial 不称完整。先定位首故障，再解释后续 EIO 或退出。

分别报告：支持的模式/版本及接口；离线/ROS/CI/台架的实际结果与跳过原因；未知参数/能力；具体下一步。禁止用构建成功、发送 ACK、有反馈来替代准确运动或物理停止验收。
