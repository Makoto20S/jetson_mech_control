# 电机隔离的双板双向诊断

用于定位“主机A发送的固定mode6目标与板B收到的ID/载荷不一致”是否与反向反馈负载有关。必须让104/105完全断电且断开CAN，仅两块板供电、CAN互连、各自USB接Jetson。此工具不是运动旁路记录器。

一个受监督进程同时拥有两口。每口仅初始化一次；共同静默采集2秒，若收到CAN帧便拒绝开始发送，保留证据。初始化帧为`f71206007d7008000000000000`，不发旧握手0x05。静默通过后写持久`ready.json`，再建立共同发送时间原点；发送结束后继续接收默认1秒，然后关闭两口、导出。

A每周期分别write两条21字节USB包（separate），扩展帧ID `0x668/0x669`，真实mode6载荷固定为 `000cd528000a000a`（104=84.1°）与 `000ea218000a000a`（105=95.9°）。B可不发，或每周期分别发扩展帧ID `0x2968/0x2969`，合成反馈载荷固定 `03490000ffff2a00` 与 `03be000000252a00`。它们仅为台架测试字节，不是真实电机测量。两模拟反馈每tick成对发送，与真实两机反馈的独立相位及抖动可能不同，不能据此完全复现真实反向负载。nonce只标识本次会话，不写入固定CAN载荷。

默认仅打印JSON计划，不检查或打开TTY，不创建输出目录。参数严格拒绝未知、重复、缺值、非十进制与溢出；没有joined/batch、可改目标、真实反馈或count-only选项。

| 参数 | 范围/意义 |
|---|---|
| `--command-hz` | 1..500，每个命令ID的频率，默认500 |
| `--feedback-hz` | 0..50，每个合成反馈ID的频率，默认0 |
| `--feedback-order` | forward（默认）：B先0x2968后0x2969；reverse：B先0x2969后0x2968，仅改变B组内写入顺序 |
| `--feedback-phase-ms` | 0（默认）或5，B每个tick的主机发送deadline相对共同epoch延后；B不发时必须0 |
| `--rx-gate-ms` | 0（默认）或6；6只允许2秒、command/feedback各10Hz、B forward，phase可0或5 |
| `--seconds` | 1..60，默认60 |
| `--drain-ms` | 1000..5000，默认1000 |
| `--log-mib` | 1..64，每口独立内存上限，默认64；双口最多128MiB |
| `--nonce` | 0..4294967295，默认随机，仅会话标识 |
| `--port-a/--port-b` | 两个不同实际USB板的直接by-path通道 |
| `--output` | 父目录已存在的新目录，拒绝覆盖 |

先在Windows编辑环境离线构建，或在Linux/ARM64隔离目录构建；无需ROS或生产安装。构建目录也必须全新。

```text
python tools/servo/duplex/build.py --output tmp/servo-duplex-build/windows-v1 --test
python tools/servo/duplex/build.py --output /tmp/duplex-offline-build-v1 --test
```

只打印预期计划的短测例子：

```text
servo_duplex --port-a /dev/serial/by-path/platform-3610000.usb-usb-0:3.3.2:1.0 --port-b /dev/serial/by-path/platform-3610000.usb-usb-0:3.4.2:1.0 --command-hz 10 --feedback-hz 10 --seconds 2 --output /tmp/duplex-short-UNIQUE
```

现场需先核对用户当次物理隔离和完整构建/文件SHA，审阅参数后由主任务在该命令添加`--run --motors-disconnected`并以root运行。建议顺序为2秒10/10Hz短测、60秒500/0Hz基线、60秒500/50Hz双向；只执行被审阅的一次，异常或差异保留后停止，不自动反向或重试。

后续反馈顺序对照使用新构建和版本化v3协调器：同一新二进制、2秒10/10Hz，先forward再reverse，逐轮保留并独立审计。A写入顺序、两侧ID/载荷、频率、间隔、静默与尾接收都保持相同；只交换B组内两次write先后。顺序选项指主机USB write顺序，CAN线上顺序会受仲裁等影响，不作保证。v3现场脚本不开放原60秒矩阵，不自动执行下一轮，不能单凭该对照唯一定位哪块板的固件。

相位对照使用新构建和v4协调器：只允许2秒10/10Hz、B固定forward，phase0与phase5逐轮审阅执行。5毫秒只改变B的计划发送deadline，A deadline、两侧载荷/数量/组内先后及共同发送窗口保持不变；主机调度与USB/CAN处理仍可能带来实际延迟，不保证CAN线上精确5毫秒相位。v4不开放500Hz或其他自由时长/频率，不自动重复或推进下一轮。

读取时序对照使用新构建和v5协调器，只开放三格：phase0/gate0基线、phase0/gate6、phase5/gate6，均为2秒10/10Hz、B forward。gate6在发送窗口内以A的每个名义tick为锚点，前6毫秒暂停两端主机read；不停止USB或CAN接收，不改变发帧载荷、计划发送时刻或板端配置。静默预检与尾接收保持原读取行为。此对照会改变主机调度与积压读取，不能把变化解释为CAN线上接收被禁止，或唯一定位固件根因；默认gate0保留原行为。逐轮审阅，不自动重试或延长。

Live仅Linux：按by-path实际解析ttyACM/sysfs的caf1:ffff USB父设备，拒绝同一板不同接口。两板全部接口全/proc/PID/task/TID/fd占用扫描，不可读则拒绝；记录实际USB父路径、总线/枚举编号。打开前后复核身份。沿用observer板级flock前缀，仅协调遵守锁的工具，不能阻止第三方后来抢占；全FD检查仍必需。输出磁盘需双口捕获上限、1MiB元数据与1GiB余量。

每口分别导出`capture-a.bin/capture-b.bin`，均为完整schema2：16字节外头（kind、port=0、result、reserved、payload_size little-endian u32、绝对monotonic结束ns u64），随后16字节IO元数据（开始ns u64、requested u32、returned u32）及原始USB字节。记录每次read/write，包括WouldBlock。失败write只记录请求字节，returned=`UINT32_MAX`表示生产接口未提供真实短写数量，不把它当已发送。

另有`admission.json/plan.json/ready.json/summary.json/supervisor.json`，现场协调器保存stdout/stderr/exit、逐文件SHA清单。退出码0表示采集完成且核心内容匹配，4表示采集完成但内容不匹配，1表示采集/IO/时序失败，2表示参数/准入错误。`summary.complete`只表示采集完整；`supervisor.collection_complete`在0/4时为true，`supervisor.complete`仅0时为true，另记`worker_exit_code`，独立原始审计仍必需；固定载荷只能比较内容及数量，不能证明逐帧对应、顺序或抵消的丢重。板A回报与板B回报可能有共享固件行为；接收板不是独立总线分析仪，USB CRC有效不等于CAN线CRC或电机内部收包证据。固件/实际CAN速率仍未知。

父进程不打开TTY，监督上界为发送时长+2秒静默+drain+15秒导出；中断转发停止，最多5秒后强杀并有限收尸。子进程安装parent-death SIGKILL以防父进程意外死亡后继续发送。强杀会丢失尚在内存的raw，明确不完整不可恢复；不可中断内核状态不能承诺有限close。成功须同时核对退出、summary、supervisor、逐文件SHA及独立原始完整性审计，文件存在不等于导出完整。

当前此文档只描述实现和计划，现场是否执行及结果以各次独立证据目录为准；不修改生产部署、系统配置或旧诊断工具。
