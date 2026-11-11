# 双 IMU 独立实机验证（2026-11-06）

## 结果

统一控制仓库开发副本的双 IMU 接收与 REF01 状态接入通过本次有界测试。
用户已确认电机接线完全断开、Orin 与 CtrBoard/IMU 上电。
仅第五路 if08（现场解析 ttyACM4）被打开；没有启动电机驱动或发送电机运动目标。

Windows 先完成采集脚本语法及设备检查模块导入校验，22项状态逻辑测试通过，
随后同步到 Orin 增量构建 IMU 三包；10秒虚拟串口验证通过后执行真实采集。
修复 tools/project/verify_imu_stream.py 的迁移导入路径，并增加 --motor-config 参数。
源码分支 ph-exo/development，基础HEAD 35c8babfbf0cbcc131bba9229d4148b0fbbeaf8c。
实测源码清单 9ce96e6e19294425888f6686d1d9532dde159803d29b179e930b784c56c341fa。

## 观测

- 主机 nvidia-desktop，ROS 2 Humble；使用开发副本 .local/ros/imu 与 .local/ros/state 安装层。
- 有界运行30秒（含DDS发现），原始观察窗口约27秒。
- IMU1/IMU2分别记录2699/2698条，均约100 Hz；数值有限且姿态/角速度/加速度字段均标记可用。
- 2698组成对ROS时间戳；末尾多一条IMU1记录来自有界观察回调结束，不据此推定丢包。
- CtrBoard时间戳2698条，2697次相邻增量均10 ms；135条诊断均healthy，报告CRC错误及丢弃字节均0。
- 接收回调最大间隔约49.7 ms，对应源消息间隔约10.07 ms，记录为接收侧调度抖动迹象。
- /ref01/state记录3395条，两路各2927条valid，accepted从0增至2918；有效跨度内无无效插段。
- 电机始终缺失，关节数组为空、alpha无效、mapping_confirmed=false。
- 停止桥接后两路转为receive_timeout，三类字段均无效；未假装硬件新鲜度或三源对齐已确认。
- 桥接与状态子进程正常退出（0），第五路端口再次检查可用。

## 证据及边界

Orin原始记录：
/home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control/.local/validation/imu_live_20261106_1504/

本机原始副本、逐文件SHA256和独立分析：
并联外骨骼-ROS框架/Mujoco-Linux/output/imu_live_20261106/

辅助observer的passed条件较宽，本次另逐行复核state的有效接收、计数增长及停流失效，
以analysis.json和state.jsonl为集成结论依据，不仅依赖passed布尔值。

这次验证通信、ROS消息与状态质量处理，不验证安装标定、绝对姿态精度、每IMU独立采样新鲜度、
硬件同步、精确线缆丢包率或整机硬实时性能。驱动归一化四元数，不能用输出范数证明原始范数精度。
旧196字节协议缺少独立IMU采样时戳/序号。电机未测试；接线恢复确认后另做被动反馈测试。
参考历史入口：mech_ctrboard_bridge/README.md、docs/parallel_exo/realtime_state.md，
以及原2026-11-05第五路实测记录；本次结果来自统一仓库开发副本，旧正式release未切换。