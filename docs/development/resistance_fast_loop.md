# 编码器抗阻 C++ 快环

本实现属于 EXO-ARCH-001，新增可选 `mech_controllers/ResistanceTrainingController`。
它在 controller_manager 每次更新时读取关节位置和速度、计算阻力；DM4310 部署标称为500 Hz。
Python 应用仍以50 Hz监督输入健康、模式和许可，不再在该路径计算阻尼力矩。
旧应用抗阻路径保留且仍为默认；新路径必须显式选择。软件测试通过不代表实机抖动已解决。

## 分层与数据流

```text
Python supervisor（50 Hz、健康/映射/启停）
  → 每轴 ~/enable（Float64，只接受0或1，兼容入口）
  → 通用 C++ ResistanceTrainingController（标称500 Hz）
      ← loaned position / velocity
      → effort / command_generation
  → 原有 hardware plugin / runtime（反馈有效期、命令租约、限位、整组故障）
  → 厂商 session / codec / transport
```

控制器不打开串口、不包含电机品牌判断。电机侧使用纯力矩MIT帧，Kp/Kd为0，
阻力通过力矩前馈字段输出。其他品牌只有在相应硬件适配确实支持同单位的 effort、
position、velocity 和授权计数语义后才能使用，不能由通用接口推断已完成设备兼容。

`~/enable=1`是运行许可，不是1 N·m力矩；0撤销输出。接错旧`~/target_effort`话题会被应用拒绝。
应用必须选择 `input_source=motor_encoder`、`policy=resistance_training` 和
`resistance_execution=controller`，并通过原有输出/映射/唯一发布者校验。
应用诊断中的零力矩占位值标为 `values_are=unused_local_torque`，不能用于证明真实输出。
实际力矩命令须由下层命令或发送轨迹核对，反馈力矩也不是独立力矩计测量。

## 控制规律与参数

2026-10-22抖动修正后：许可有效时持续低通原始速度得到`vf`，短暂静止只撤力，
不反复重置滤波器。同方向速度须连续高于停止门限20ms，且达到启动门限后才建立阻力。
静止或反向立即撤力，并重新确认方向；若滤波方向与当前速度不同，保持零力矩。

正常同向运动仍为粘性阻尼。命令幅值先取`B × min(|vf|, |v|)`，
在停止至启动速度之间乘以连续的`smoothstep`衰减因子，防止降速时残留过大的滤波阻力；
随后按原力矩上限饱和、仅限制增力斜率，保留方向性角度软区。
持续运动达到启动速度后衰减因子为1，稳态仍为`T=-B*v`（未饱和、远离边界时）。
该修改减少短促抖动引起的反复加力，不代表固定输入回放能预测真实闭环稳定性。

下表是初版DM4310快环部署参数（后续可选变更见文末）；通用插件保守默认值不同，详见源码声明。

| 参数 | 快环部署值 | 作用 |
|---|---:|---|
| update_rate | 500 Hz | manager目标更新频率，并非硬实时承诺 |
| damping | 8 N·m·s/rad | 粘性阻尼系数 |
| torque_limit_nm | 8 N·m | C++输出命令饱和上限，不是恒定输出 |
| buildup_nm_per_second | 8 N·m/s | 阻力建立斜率，减力不受此限制 |
| velocity_filter_tau_s | 0.025 s | 速度低通时间常数 |
| start_velocity_rad_s | 0.06 rad/s | 静止后开始建立阻力的阈值 |
| stop_velocity_rad_s | 0.02 rad/s | 停止门限，构成滞回 |
| direction_confirmation_s | 0.02 s | 同方向连续有效运动确认时间；静止/反向重新计时，允许范围[0,0.1]s；0仅关闭计时，仍保留反向撤力 |
| max_velocity_rad_s | 1 rad/s | 实际速度超限锁存故障 |
| position_min/max_rad | 实测文件加载 | 每轴绝对位置范围，不使用预览宽边界 |
| position_margin_rad | 0.02 rad | 朝边界方向的命令软削减区 |
| max_update_period_s | 0.006 s | 更新周期或单调时钟间隔超限即故障 |
| run_limit_s | 默认5 s，配置范围(0,120] | 连续许可的单调时钟运行预算 |
| minimum / maximum | 0 / 1 | 许可通道范围，不是力矩范围 |
| max_slew_per_second | 1000000 | 许可不渐变；力矩斜率由上表独立设置 |
| target TTL soft / hard | 100 / 106 ms | 上游许可有效期，沿用共享控制器机制 |
| 下层命令租约 soft / hard | 4 / 6 ms | 控制更新失联由原有下层处理 |
| 下层反馈有效期 | 40 ms | 原有运行时反馈保护 |
| 下层反馈力矩保护 | 8 N·m | 超限故障停机，区别于命令饱和 |

位置软削减不是机械制动或停止距离保证，实际越界仍触发故障。
通用标量状态接口没有独立反馈时间戳，本控制器不能识别值未变的陈旧缓存；
实际反馈新鲜度仍由硬件运行时验证。控制器对NaN/Inf、越界、超速、更新超时、
时钟异常、运行预算耗尽锁存故障且停止刷新授权计数；恢复许可不会自动重启。
Humble在update报错后可能仍显示active，不能用该标签判断电机是否失能。
真实部署依靠下层租约停组，最终必须独立读取两台失能状态。

## 无设备入口

安装并source覆盖层后，以下启动默认仅检查文件与配置，不创建硬件节点：

```bash
ros2 launch mech_bringup dm4310_dual_bringup.launch.py resistance_training_fast:=true
ros2 launch mech_bringup dm4310_dual_bringup.launch.py resistance_training_fast:=true resistance_run_limit_s:=20
ros2 launch mech_assist_control dm4310_resistance_fast_preview.launch.py
```

硬件入口从实测范围文件同步URDF、控制器与下层范围，默认check_only=true，
实际设备分支仍受原有证据闸门约束、硬件不自动使能、控制器不自动激活。
快环选项与旧2/8 N·m抗阻选项互斥，且只允许effort模式。
上层应用和C++分别有运行预算，配置时必须核对两者，较短者先结束；
不能只改应用为20秒而保留快环5秒后声称支持20秒运行。
源预览配置含宽位置夹具值，仅用于无设备检查，不得复制成机构实测限位。

无设备生产插件回环工具：

```bash
python3 tools/validation/check_resistance_fast_ros.py --output-dir /absolute/new/evidence-directory
```

该工具只使用mock_components/GenericSystem，检查实际插件加载、50 Hz许可下快环授权计数、
许可中断及恢复不自动重启，不打开网关。算法非零输出由借用状态接口的C++测试覆盖。
旧Python实机记录器将应用策略值作为力矩，不能直接用于新快环的输出验收。
后续实机工具必须记录真实下层命令、同步反馈、周期统计和最终独立失能读回。

## 当前验证边界

见[软件验证记录](../validation/2026-10-22-resistance-fast-loop.md)。
已完成ARM64构建、算法/许可/下层保护回归及无设备ROS插件回环。
历史实机与参数变更见文末及[台架记录](../validation/2026-10-22-resistance-fast-bench.md)；
最新算法的[抖动修正与回放](../validation/2026-10-22-resistance-chatter-fix.md)已通过软件验证，
修正后20秒实机已完成，但操作者仍反馈力矩不足；方向确认与增力斜率继续限制实际出力，
阻力手感、真实抖动改善及硬实时性能均未验收。

## 按需移除训练速度门限与增大阻力

`resistance_velocity_guard=false`仅允许显式快环effort配置使用。
它移除额外的1 rad/s训练停机门限，控制器及下层只保留测量文件中VMAX的解码有效范围
（当前两台读回为50 rad/s），不设置速度目标、不写VMAX寄存器，也不声称协议外速度可被正确测量。
这不是把真实转速控制到50 rad/s。配置仍检查速度有限值、实际绝对角度、力矩及失联。
初始激活仍要求静止附近，不能在快速转动时使能。

按用户“阻力增大”生成的候选：B=12 N·m·s/rad、加力斜率20 N·m/s，
滤波25 ms、启停阈值0.06/0.02 rad/s、力矩命令上限及反馈保护8 N·m不变。
0.1 rad/s稳定转动时未限幅阻尼目标由0.8增加到1.2 N·m；
100 ms连续同方向运动的增力预算由0.8增加到2 N·m，静止/反向仍撤力。
该改变可能影响手感与抖动，须独立实机验收，不能把命令上限当作实测输出。

```bash
# 默认仅检查，不访问设备
ros2 launch mech_bringup dm4310_dual_bringup.launch.py resistance_training_fast:=true resistance_velocity_guard:=false resistance_damping:=12 resistance_buildup_nm_per_second:=20 resistance_run_limit_s:=20
# 当前stronger预览已改为下节B24/建立40，需使用下节匹配的硬件参数
ros2 launch mech_assist_control dm4310_resistance_fast_stronger_preview.launch.py
```

上层预览的max_velocity=50是当前设备解码范围夹具；实际部署须与读回量程、
C++及下层一致，且必须加载实测位置范围。默认快环和旧模式均保留原参数。

## 首次快环实机结果

[首次台架记录](../validation/2026-10-22-resistance-fast-bench.md)：20秒计划在约10秒时
因1号反馈1.013 rad/s超过旧1 rad/s门限停组，最终两台独立失能。
实际发送及力矩码变化约500 Hz已观测到；完整双机抗阻及抖动效果未通过验收。
记录器不能用JointState持续发布推断底层正常；
`tools/bench/controller_log_guard.py`补充有界增量错误检测，故障后不得继续计为有效训练。

### 最新二倍阻力候选

上述B12/建立20方案后续已获准实机完成20秒，两台最终独立失能，
实际编码力矩峰值约1.28/0.68N·m，但操作者仍感觉偏弱。
当前`dm4310_resistance_fast_stronger_preview`已更新为B24、建立40N·m/s，
力矩命令及反馈保护仍8N·m，其余滤波/启停/角度/失联参数不变。
原始B8预览及已执行B12外部记录保留。匹配的硬件离线检查参数为：

```bash
ros2 launch mech_bringup dm4310_dual_bringup.launch.py resistance_training_fast:=true resistance_velocity_guard:=false resistance_damping:=24 resistance_buildup_nm_per_second:=40 resistance_run_limit_s:=20
```

相同输入的未饱和理论命令翻倍，不承诺实际不同动作或接近限位时的倍数。
B24已获准执行20秒，但动作及实际力矩近零，抗阻效果未验收。后续使能核对明确两台status=1，因广播器未就绪提前退出，详见[台架记录](../validation/2026-10-22-resistance-fast-bench.md)。启动脚本须先确认广播器active再使能；每次硬件激活仅打印一次逐台新鲜使能状态，不在实时循环逐帧打印。
