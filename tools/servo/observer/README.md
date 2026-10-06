# 单板 CAN 接收观察与隔离台架发包

本工具为 SERVO-001 三项旁路资格验证提供两个独立角色，使用生产
`PosixCdcSerialPort` 与已有 USB CDC 编码/解析器，不依赖 ROS。
`observe` 对一块板仅写一次固定 13 字节 USB 透传初始化，此后只读；
`fixture` 在无电机总线上产生已知帧。现有 `dual_board` 不修改。
默认仅向 stdout 打印 JSON 计划，不打开串口、不创建输出。

初始化不是已验证的 listen-only 设置。板 B 可能参与 CAN ACK/错误处理，
固件、位速率、板端过滤及 echo 语义仍未知；B 是另一个网关接收路径，
不是独立总线分析仪，也不证明电机内部接受了目标。

## 构建和离线验证

Windows 使用可用的便携 g++，仅生成离线 CLI；Linux 额外链接生产串口实现。
输出目录必须新建，不复用或覆盖已有构建/证据。

```text
python tools/servo/observer/build.py --test --output tmp/servo-observer-build/offline-001
tmp/servo-observer-build/offline-001/servo_observer --help
tmp/servo-observer-build/offline-001/servo_observer --role fixture --profile four-id
```

Linux 可执行文件无 `.exe`；Windows 为 `servo_observer.exe`。
离线测试仅 fake port、临时文件和 Linux PTY，不接触实际 USB/CAN。

## 参数

| 参数 | 范围和用途 |
|---|---|
| `--role` | `observe`（默认）或 `fixture` |
| `--run` | 实际运行；不指定则仅打印计划 |
| `--port` | 实际运行必须为直接 `/dev/serial/by-path/NAME` 链接 |
| `--output` | 实际运行必须提供尚不存在的目录，父目录须存在 |
| `--seconds` | 1–180；默认 5 秒；fixture 同时作为发送时间上限 |
| `--log-mib` | 1–64，默认 16 MiB；内存 capture 硬上限 |
| `--count-only` | observe 专用；保存每次 I/O 的元数据，不保存 RX 原始载荷 |
| `--motors-disconnected` | fixture 实际运行必需的物理隔离声明 |
| `--profile` | fixture 专用，`sequence` 或 `four-id` |
| `--lanes` | sequence 专用，1 或 2，默认 1 |
| `--hz` | sequence 每 ID 1–500 Hz；four-id 仅 1–10 Hz；默认 10 |
| `--frames` | 每 ID 帧数；sequence 可设 1 以发唯一标记帧；four-id 为 4–10 |
| `--drain-ms` | fixture 专用，0–5000 ms，默认 200 |
| `--nonce` | fixture 专用，0–4294967295；默认随机 |

拒绝重复、未知、负数、溢出及角色不适用的参数。sequence 默认帧数为
hz×seconds；four-id 默认每 ID 4 帧。帧数与频率须落在配置时间上限内。
具体 ID、计划数量和载荷以无设备 JSON 计划为准。

sequence 使用扩展 ID `0x1fff0101/0x1fff0102`，独立 nonce/序号标记。
four-id 使用生产编码器生成真实模式 6 命令 ID `0x668/0x669`（84.1°/90.9°、
速度及加速度参数各 100），以及已知
模拟反馈 ID `0x2968/0x2969`。**模拟反馈不是电机测量。** 此角色必须保持
电机断电且 CAN 断开；不能拿资格验证发包工具驱动电机。
所有 CAN 包均单独 write，不新增 joined/batch 变量。
接收另外限制为最多 256 种 ID/格式组合、100 万帧及 100 万次 read；
超限或非法/残留 USB 包中止并标记不完整，原始已捕获字节保留。

## 运行和就绪协作

以下为后续授权操作的模板，路径须换成本轮核验的通道；不是现场执行记录。
先启动观察器，外层等待它的 `ready.json` 且确认进程仍存活，再启动另一块板的
fixture。外层必须有总期限；fixture 完成后，观察器留出尾部接收时间再结束。

```text
sudo servo_observer --role observe --run --port /dev/serial/by-path/B_CHANNEL --seconds 8 --output /tmp/observer-UNIQUE
sudo servo_observer --role fixture --run --motors-disconnected --port /dev/serial/by-path/A_CHANNEL --profile sequence --lanes 1 --hz 10 --frames 1 --seconds 2 --output /tmp/fixture-UNIQUE
```

live 仅 Linux，要求 root 完整扫描 `/proc/PID/task/TID/fd`。by-path 必须解析
到真实 ttyACM，sysfs 字符设备号一致，USB 父设备为 `caf1:ffff`。只检查所选
板的全部接口占用，因此另一物理板可并行。打开前后重新核对实际身份。
运行期 `/run/lock/mech-servo-observer-usb-USB_PARENT.lock` 协调遵守该锁的
新工具；不能阻止第三方程序随后抢占其他接口，不能替代全 fd 扫描或串口锁。
锁文件不跟随 symlink，必须是 root 所有的普通文件。

磁盘准入保留 capture 上限、1 MiB 元数据及额外 1 GiB。
`admission.json`/`plan.json` 在打开串口前持久保存；初始化成功后独占写入并
fsync `ready.json`，包含 role、实际 USB 身份、boot_id、单调时间及 worker PID。
就绪只说明初始化成功并进入采集流程，不保证板端已证 silent 或总线有反馈。

## 证据与限制

每个角色输出 `admission.json`、`plan.json`、`ready.json`、`capture.bin`、
`summary.json`、`supervisor.json`。外层另保存 stdout、stderr、实际退出码及
文件大小/SHA256。正常路径先关闭串口，再导出内存 capture；不持续写 JSON 日志。

原始 schema 2 为小端：外层 16 字节
`kind:u8, port:u8(0), result:u8, reserved:u8, payload_size:u32, end_monotonic_ns:u64`；
载荷前 16 字节为 `begin_monotonic_ns:u64, requested:u32, returned:u32`，
随后为原始字节。每次 read（包括 WouldBlock）都有记录；write 保存请求字节。
成功 write 的 returned 为请求长度；失败 write 的 returned 为 UINT32_MAX，
表示真实接受字节数未知，不把整包请求冒充成功发送。
`count-only` 文件省略 RX 原始载荷，保留唯一初始化 TX 字节用于禁发审计；
`raw_complete=false`，不能做 RX 原始字节完整性审计。

boot_id 与绝对单调时间允许同一 Jetson、同次启动的 A/B 记录对齐；时间戳是
主机 I/O 起止时间，不是 CAN 线上的时间。USB 批量、驱动缓冲和两口延迟会影响
对应关系。重复恒定模式 6 载荷没有唯一帧标记，不能证明逐帧对应、无乱序或抵消丢重。

成功须同时检查退出码 0、summary `complete=true`、supervisor `complete=true`
和 `export_valid=true`；full 模式还须 `raw_complete=true` 并独立审计原始记录。
count-only 的运行成功只说明计数采集结束，不能声明原始 capture 完整。
退出 1 表示运行失败/不完整，退出 2 表示参数或准入错误；保留已有证据，不覆盖重试。

SIGINT/SIGTERM 请求停止；父进程监督上限为 seconds+drain+15 秒，停止请求后
最多另等 5 秒，再强杀并有限等待收尸。子进程安装 parent-death SIGKILL，防止父
进程异常死亡后独自继续发包。强杀可能丢失内存原始 capture；未成功收尸不声明
端口关闭，不可中断的内核状态不能承诺有限 close。损坏或不完整导出不覆盖修复，
必须结合退出码和 supervisor 判断。

## 三项资格验证顺序

1. 无电机先做小量唯一标记阳性和 four-id 原始内容核对。
2. 保持接线和 fixture 参数相同，以 observe full/count-only/full 做 500 Hz
   持续接收和 fixture 发送节拍负载对照；拟定 A 发 60 秒、drain 1000 ms，
   B 采集约 70 秒，两角色均 log 64 MiB。每轮保留 half-period 调度门限，
   验证容量、闭合及原始数量；这不证明正式 ROS 控制零扰动。
3. 全部发包停止后由用户断开板间 CAN 信号，保留 USB，阴性只发一诊断帧。
   唯一接收板断开后 A 可能无 ACK 重发/积压，发送失败可接受，不能要求 A 成功。
   恢复前用户将通信板全断电、确认 USB 消失以清残留，再重连并做新标记阳性；
   不在持续发送时拔线，不带未知待发队列直接重连。

这些资格验证不会接电机或恢复运动。通过后还须独立的接线及具体运动授权。
本目录文档描述实现和拟验收方案，不宣称本工具已做现场验证。
