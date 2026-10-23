# 普通 DM4310 双电机位置限位

本入口用于两台普通 DM4310（非48V版）的电机反馈绝对角度限位；不代表人体关节角标定。
复用现有 Damiao codec/session、单写者 runtime、`Dm4340System` 插件和通用控制器，
插件类保留历史名称，不复制控制算法。新增配置期 profile `Dm4310MitStandard` /
`dm4310_mit_standard`，不覆盖 DM4340 的 profile 和配置。

## 三种不同的范围

| 参数 | 用途 | 来源 |
|---|---|---|
| 编码范围 ±12.5 rad / ±30 rad/s / ±10 N·m | MIT字段换算 | [官方 SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py) 普通DM4310默认值，实际设备设置仍须核对 |
| `position_min_rad` / `position_max_rad` | 机构允许的反馈位置区间 | 失能状态手动采集端点，再向内预留余量 |
| `position_guard_margin_rad`，当前候选0.02 rad（1.15°） | 靠近上述边界时拒绝向外速度/力矩命令，并在最终发送前复查 | 原有底层会话机制，适用于新profile |

端点向内预留5°与底层0.02 rad方向保护区是两个独立参数。前者缩小允许区间，
后者位于允许区间内部。两者都不是机械挡块、制动距离或断能后静止的保证。

位置目标在控制器和底层都受上下界限制；实际反馈越界会拒绝反馈并锁存故障，
runtime沿用整组停机路径。速度/力矩模式在边界附近拦截向外命令，允许向内命令；
实际已经越界时不能靠继续发送“向内”命令自动恢复。参见[安全参数](control_safety_parameters.md)。

## 只读端点采集

`tools/bench/dm4310_range_sample.cpp` 是独立、有界采样入口：只使用既有
`PosixCdcSerialPort` / `UsbCdcTransport`，网关声明4.8.8 / CAN1Mbps，通道显式传入。
固定查询电机ID1/2，反馈路由4/5；初始化网关透传后，每台10Hz、20轮，约2秒。
唯一电机操作码为0xCC查询，无使能、失能、位置置零、运动或寄存器写入。
缺回复、异常路由、失能以外的状态、串口错误都会使采集失败。工具不自动重试。

编译（`${CORE_INSTALL}` 指向已验证的core安装包目录）：

```bash
c++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I ros2_ws/src/mech_control_core/include \
  -I ros2_ws/src/mech_protocol_damiao/include \
  tools/bench/dm4310_range_sample.cpp \
  ros2_ws/src/mech_protocol_damiao/src/dm4340_mit_wire.cpp \
  -L "${CORE_INSTALL}/lib" -Wl,-rpath,"${CORE_INSTALL}/lib" \
  -lmech_control_core -pthread -o /tmp/dm4310_range_sample
```

取得当前现场授权、确认电机失能及机构可安全手动摆动后，每个端点分别执行一次：

```bash
/tmp/dm4310_range_sample --confirmed-readonly /dev/ttyACM1 50 > /tmp/endpoint_a.jsonl
# 等操作者手动移动并停稳，再另行采集endpoint_b.jsonl。
```

不要穿戴采集，不要顶机械硬限位或松开无支撑的重力负载。每颗电机的两个端点
可方向相反，生成器分别排序。成对记录不证明区间内任意双轴组合都无机构碰撞。
两次端点之间必须保持供电、零点、接线身份和映射不变；变化后重新采集。

离线生成候选文件：

```bash
python3 tools/bench/dm4310_range.py /tmp/endpoint_a.jsonl /tmp/endpoint_b.jsonl \
  --velocity-range-rad-s 50 --margin-deg 5 --output /tmp/dm4310_limits_candidate.json
```

必须完整40帧/端点、ID路由和原始字节匹配、状态0、反馈速度≤0.05 rad/s，
每轴采样跨度1.7～3秒、帧间隔≤0.2秒、位置波动≤0.005 rad。
使用较保守的稳定窗口边缘向内缩，不仅取中位数。内缩后区间必须能容纳两个
0.02 rad保护区；非法/非有限值拒绝。输出文件独占创建，不覆盖已有标定。
候选始终标记 `motion_authorized=false`、`deployment_verified=false`。

## 接入下层

`dm4310_dual.urdf.xacro` 的缺省上下界都是0，缺省固件ID0、证据声明false，
未加载测量数据不能作为运动配置。下层使用轴反馈坐标 `scale=1, offset=0`；
如果要改成解剖学关节坐标，应重新审核映射并转换边界，不直接修改单位标签。

`dm4310_dual_bringup.launch.py` 默认读取本机
`${XDG_CONFIG_HOME:-$HOME/.config}/jetson_mech_control/dm4310_limits.json`，
也可显式指定 `range_file`。本次实测文件已保存到该默认位置，未提交。
同一上下界写入：

- URDF关节限制；
- ros2_control硬件每轴的 `position_min_rad` / `position_max_rad`；
- 位置控制器的minimum/maximum。

启动入口检查型号、身份、路由、映射、有限值、边界宽度和内缩关系；
范围接受、当前固件/量程证据、网关/CAN、超时和映射声明缺失时，创建节点前拒绝。
固件ID不能沿用4340，未读回和解释前保持0/false。所有硬件、运动控制器及广播器
默认inactive，`enable_on_activate=false`。之后的使能及具体运动测试需要独立准备，
本限位任务不提供自动激活入口。当前effort缺省命令限幅0.1 N·m，不沿用旧8 N·m参数。

启动默认 `check_only=true`，只校验并渲染范围，不创建ROS节点、不打开设备：

```bash
ros2 launch mech_bringup dm4310_dual_bringup.launch.py
```

默认检查实际核对本机保存的范围、URDF关节限制、底层参数及位置控制器上下界
完全一致。默认文件缺失或非法时直接失败，不回退到示例区间。
只有明确指定`check_only:=false`才进入设备节点准备分支；仍必须满足全部证据
及现场授权要求，硬件和控制器保持inactive。检查通过不是运动批准。
离线也可调用`range_mappings()`、`controller_parameters()`及xacro处理生成URDF/YAML。
下次上电或原点、安装、接线、映射有变化，应先只读核对坐标和范围有效性。
端点采集结束位置通常在内缩区间外；启用前须先手动移回两轴各自允许区间内。

实测和验证见[2026-10-22采集记录](../validation/2026-10-22-dm4310-position-limits.md)。

## 当前设备换算与2 N·m抗阻候选（2026-10-22）

两台实际只读RID21/22/23均为12.5/50/10，速度编码使用50 rad/s；
这是通信换算量程，不是允许运动速度，不能据此判定为48V型号。
采样器和候选生成器必须显式使用相同速度量程；SDK型号缺省30保持不变。
范围文件新增`calibration_status=valid`检查；重启后坐标未确认则拒绝。
本次用户确认“端点没有问题”，本机元数据记录为操作人员确认，非再次实测。
原文件已本地备份，角度边界保持原值，motion/deployment字段仍为false。

用户选定的是抗阻训练，每轴力矩命令上限±2 N·m，非位置阻抗。
独立预览配置`dm4310_resistance_training_2nm_preview.yaml`仍禁止输出；其中
±12.5 rad是预览占位区间，真实输出前必须从当前测量文件替换，不能作为机构限位。
硬件入口可离线检查2 N·m候选：

```bash
ros2 launch mech_bringup dm4310_dual_bringup.launch.py resistance_training_2nm:=true
```

该选项只允许effort模式，控制器上下限±2、下层命令饱和±2同步，
Kp=Kd=0；应用层max_command/max_actuator_command均为2。
控制器斜率1000 N·m/s用于在标称500 Hz的一拍内从2卸至0，应用建立斜率仍为0.5 N·m/s。
B=1 N·m·s/rad、滤波0.05秒、死区0.02 rad/s、应用速度保护1 rad/s、
相对位移0.2 rad、候选窗口5秒。静止/反向卸力；2 N·m并非恒定输出。
底层反馈力矩停机阈值仍为5 N·m，与命令饱和独立，未删除其他保护。
真实驱动力矩的计量及驱动器电流硬限幅未验证。

本次未启动电机；当前新电机超时RID9读回0的含义及独立断联保护未验收，
不能以旧4340记录自动满足`motor_side_timeout_verified`。实机需另行完成当前
启动证据与具体参数确认，不改寄存器或自动重试。

### 用户后续加阻力与取消相对试验边界

两次复测记录见[实机记录](../validation/2026-10-22-dm4310-resistance-2nm.md)。
当前独立DM4310预览配置B=4，`enforce_excursion_limit=false`，取消相对起点
±0.2 rad试验约束；`max_excursion_rad`仍是合法备用值，但开关false时不参与计算。
所有旧配置缺省仍true。实测绝对位置边界和下层保护未移除；预览占位角度
不能用于真实输出。此处记录2 N·m候选；后续实机状态见下节。

### 8 N·m上限候选

新增`resistance_training_8nm:=true`选择独立8 N·m力矩命令上限，不能与2nm
同时选择，仅effort可用。默认仍离线检查；实测位置边界不变。应用预览、控制器
和底层限幅同步；经用户明确同意，8 N·m选项的反馈停机阈值同步为8 N·m。
旧8/5组合在初始化拒绝，现增加离线启动校验。8/8实机在倒计时超速退出，
未进入抗阻，最终两台独立读回失能。无倒计时旧参数5秒窗口已完成，但阻力效果未验收；STM32响应方案两次实机后效果仍未验收，后续smooth候选仅离线验证；
见[实机记录](../validation/2026-10-22-dm4310-resistance-8nm.md)。
原理及参数边界见[抗阻训练说明](resistance_training.md)。
