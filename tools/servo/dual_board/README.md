# 有界双板 USB CDC / CAN 诊断

这是 SERVO-001 异常排查的独立工具，复用生产 `UsbCdcCodec`、
`PosixCdcSerialPort` 和 mode 6 encoder，不经过 ROS 控制器，不改变生产配置。
本轮实现与验证范围仅为离线测试，未运行真实双板收发，也未发送运动命令。

默认只在标准输出打印 JSON 计划，不打开 TTY、不创建输出目录、不发送初始化。
Linux 实际收发须同时指定 `--run --motors-disconnected`、两个不同板的
`/dev/serial/by-path/...` 以及从未存在的 `--output` 目录。
这个声明必须来自操作者对电机 CAN 已物理脱离测试总线的确认；工具无法代替接线检查。
后续现场试验需按当前任务取得明确授权。

## 构建与离线测试

只需 Python 3 与支持 C++17 的 g++，不需要 ROS 或系统安装。
Windows 构建只含离线 CLI，Linux 构建另链接仓库现有串口和日志源码。
不要复制生产构建/安装目录来运行该工具。

```text
python tools/servo/dual_board/build.py --output tmp/servo-dual-board-build/offline-001 --test
```

`--output` 省略时使用仓库 `tmp/servo-dual-board-build`；已有目录一律拒绝，
重试时选择新目录。`--cxx` 可指定编译器路径（默认 `CXX` 环境变量或 `g++`）。
输出为 `servo_dual_board`、`test_diagnostic`，Linux 另有 `test_posix`；
Windows 文件后缀为 `.exe`。每个构建/测试的输出分别保存为日志。
所有 `--test` 用例仅使用 fake port、临时文件或 PTY，不访问真实 USB/CAN。

例如计划两 ID 各 500 Hz，使用同一次系统写入粘合两个完整包：

```text
tmp/servo-dual-board-build/offline-001/servo_dual_board --lanes 2 --hz 500 --seconds 2 --packing joined --nonce 123
```

实际收发入口示例仅供后续授权现场使用；两个路径须替换为本次实际 USB 物理通道：

```text
sudo tmp/servo-dual-board-build/offline-001/servo_dual_board --run --motors-disconnected --tx /dev/serial/by-path/TX_CHANNEL --rx /dev/serial/by-path/RX_CHANNEL --output tmp/dual-board-run-UNIQUE --lanes 1 --hz 10 --seconds 2 --profile sequence --packing separate
```

交换 `--tx` 和 `--rx` 可测试反方向；每次都需要新的输出目录。
程序不配置 CAN 位速率、终端、供电或板固件。

## 参数与分包

所有参数严格解析，未知、重复、缺值、负数及整数溢出均拒绝；数值只接受十进制数字。
默认配置为 sequence / separate、1 ID、每 ID 10 Hz、2 秒、drain 1000 ms、16 MiB。

| 参数 | 范围/含义 |
| --- | --- |
| `--profile` | `sequence`、`constant`、`mode6` |
| `--packing` | `separate`、`joined`、`batch`；后两者要求 `--lanes 2` |
| `--lanes` | 1 或 2 个 ID |
| `--hz` | 每 ID 1–500 Hz |
| `--seconds` | 1–120 秒；每 ID 最多 60000 帧 |
| `--drain-ms` | 收尾接收 200–5000 ms |
| `--log-mib` | 含记录头的二进制采集硬上限 1–64 MiB |
| `--nonce` | 0–4294967295；默认随机，仅 sequence/constant 载荷使用 |
| `--help` | 用法说明 |

`separate` 将每 ID 的完整 CDC 包分别调用一次生产串口写入；
`joined` 将两个完整 CDC 包连接后一次写入；
`batch` 将两个生产 encoder 生成的 CAN 记录放入一个诊断专用 CDC 外层包，重新计算外层 CRC。
这些 DLC8 测试帧每个完整包为 21 字节；两 ID 的 separate 为两次各 21 字节写入，
joined 为一次 42 字节，batch 为一次 35 字节。
后一种形式用于验证板对单包多记录的支持，不能假定未知固件一定接受。
发送边界只说明主机写入方式，USB/串口及固件可能再次拆分或合并数据。

`sequence` 使用扩展 ID `0x1fff0101/0x1fff0102`，8 字节载荷为 nonce 的
4 字节 little endian、序号的 3 字节 little endian、ID 区分字节 `0xa4/0xb5`。
可以逐 ID 检查内容、丢失、重复及乱序，并隔离旧会话流量。

`constant` 同样使用上述 ID，但每次序号字段固定为 0；只能比较每 ID 的内容与数量。
相同载荷的一帧丢失再加一帧重复可能抵消，因此不能证明逐帧对应或顺序。

`mode6` 使用生产 encoder 的真实 mode 6：驱动 ID 104 固定目标 84.1°，
ID 105 固定目标 90.9°，速度输入均为 100 ERPM，加速度原始输入均为 100。
手册对加速度的物理解释仍有边界，不把它当成已确认的实际机械加速度。
实际扩展 CAN ID 为 `0x668/0x669`。
它不把 nonce/序号塞入角度字段，故与 constant 一样无法证明逐帧对应或顺序；
若误接电机，这些字节是真实位置命令，必须先物理隔离电机。

## 准入、收尾与证据

Linux live 要求 root，以便逐进程、逐线程检查当前 `/proc` 中所有 fd。
不可读目录/描述符拒绝，只有已消失的进程/线程及竞态关闭的 fd 可跳过。
检查覆盖两选中 USB 板的其他接口，发现占用即拒绝。
这次检查的可见范围是当前 PID namespace；应在主机而非隐藏进程的容器内运行。
两端必须由 by-path 直接解析为真实 ttyACM 字符设备，与 sysfs 设备号一致，
向上追溯到不同的 `caf1:ffff` USB 父设备；同板的两个接口也会拒绝。
实际 by-path、tty、USB 父路径、接口、bus/dev 编号和可用序列号写入证据。
打开前和打开后复核身份，生产串口仍使用其非阻塞 advisory lock。
读取占用与真正打开之间仍存在竞态，不能阻止不遵守锁的第三方进程后来抢占设备。

目录须从未存在，父目录须已存在；准入要求至少容纳 capture 上限、1 MiB 元数据
及额外 1 GiB 磁盘余量。旧证据不删除也不覆盖。
在打开 TTY 之前先落盘 `admission.json` 保留实际身份与计划。
采集在内存中进行，容量不足、CRC/长度/内容错误、缺帧、重复、乱序、意外流量、
串口异常、调度超时或中断均中止并标记 `complete=false`。
接收只接受生产 decoder 支持的格式；三字节固件前缀只支持包头一次，
多记录包中每条记录都带前缀的形式会拒绝，不能据此直接认定板发送了错误 CAN 数据。
接收启动时先做 100 ms 空闲基线，停止发送后完整等待 drain；
发送调度避免迟到后追赶突发，并记录最小发送间隔/最大迟到及实际经过时间。

正常路径先关闭两串口，再导出 `capture.bin`、`plan.json`、`summary.json`，
每个文件独占创建并 fsync。父进程不打开串口，监督整个子进程，包括 close/导出阶段；
总截止时间为 seconds + drain + 15 秒。SIGINT/SIGTERM 转为停止标志，
父进程转发给子进程并把剩余关闭等待限制到最多 5 秒；超时 SIGKILL 后最多等待 1 秒。
子进程在打开端口前设置 Linux parent-death SIGKILL 并复查父 PID，
避免父进程崩溃/被杀后成为无人监督的发送者。
避免用全局 alarm 在证据导出途中直接杀死整程序。

若关闭/内核 I/O 卡死，强杀可能使内存 capture 全部无法恢复；不伪造原始记录。
若子进程仍不可回收，仅保存 `supervisor.json` 事故元数据，
`ports_close_verified=false`；此时不能宣称端口已经关闭或继续现场运行。
`admission.json` 仍保留参数与身份。若导出中途失败，已有部分文件保留原样，
`supervisor.json` 的 `export_valid=false`，即使存在 summary 也不能视为完整。
程序也无法对机器掉电、不可中断的内核状态或父进程磁盘 fsync 卡死给出硬实时保证。

成功必须同时满足退出码 0、`supervisor.json` 的 `complete=true` / `export_valid=true`
以及 `summary.json` 的 `complete=true`。退出码 1 为诊断失败/不完整，
退出码 2 为参数/准入/外层异常。只打印 dry-run 计划也返回 0，`run_requested=false`。

`capture.bin` 每条记录为 kind:u8、port:u8、result:u8、reserved:u8、
payload_size:u32le、elapsed_ns:u64le，随后原始字节；port 0 是发送板、1 是接收板。
kind 1 是 TX、2 是 RX，包含初始化请求和收发失败结果；
result 对应生产 `TransportResult` 枚举。
这是 `write_all/read_some` 层记录；生产串口若发生半包写入失败，记录是请求包与失败状态，
不冒充逐次 write syscall 的实际成功字节数，也不能证明 CAN 线端已收到。

接收板仍是相同类型网关，不能当作独立 CAN 分析仪；异常不能直接归责发送板，
正常也不排除双向负载、实际电机协议或电气条件引起的问题。
板固件与 CAN 位速率本轮未知，JSON 不使用历史硬编码 4.8.8 冒充实测版本。
单次采集严格有界；多次运行总量由操作者管理，程序不会覆盖/回收旧目录。
