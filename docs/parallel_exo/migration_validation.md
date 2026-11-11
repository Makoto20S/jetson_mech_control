# 控制工程整合与迁移验证

日期：2026-11-05，Asia/Shanghai。原Git基线：`35c8babfbf0cbcc131bba9229d4148b0fbbeaf8c`。

本次将控制运行所需ROS/IMU配置和源码、REF01运动学/动力学API、MJCF、98个实际引用的STL网格、参数、必要数值基线与恢复工具纳入统一仓库。按用户后续要求移除144个本次初步导入的非必需文件，未删除原项目中的CAD/MATLAB工程或历史资料。

运行验证使用可被Git收录的工作树导出包，传输到不同绝对路径；逐文件核验683个文件的哈希。它是尚未提交的工作树副本，不是已发布分支的clone。

- 目标主机：`nvidia-desktop`，Linux aarch64，Python 3.10.12、ROS 2 Humble。
- 独立目录：`/home/nvidia/parallel_exo_ros_framework/output/git_migration_20261105_2113/control_checkout`。
- 新建独立虚拟环境，安装锁定依赖，`pip check`通过；MuJoCo 3.17.0、NumPy 2.2.6均从新环境加载。
- 原系统缺少ensurepip，因此使用现有pip作为安装工具管理无pip的新环境；没有复制旧site-packages、安装系统软件或修改旧环境。
- 运动学：16,736组冻结数值基线通过，最大误差约`1.55e-15 rad`；不依赖MATLAB运行。
- 动力学：100组正逆计算、独立KKT对照、75个冻结轨迹采样与50 ms原生步进通过；最大闭链残差约`5.01e-7 m`。
- IMU组合：`mech_control_core`、`mech_protocol_ctrboard`、`mech_ctrboard_bridge`构建通过，82项测试全部通过，无失败、错误或跳过。
- 未运行`servo_imu`/`all`整套构建，未测试Windows新环境恢复，未启动硬件或控制器。

运行测试后仅补入来源可核对的现有电机配置副本和文档；这些不是电机实测结论。当前Orin的`current`与`servo-current`仍指向原发布版本，配置原件未修改。

本次完成的是控制源码整合、Linux环境恢复和上述软件验证。ROS调用MuJoCo的控制链及200 Hz整机闭环尚未实现，不因文件统一而改变该状态。

结果摘要见`migration_evidence.json`；逐文件来源和派生修改见`../../manifests/parallel_exo_import_20261105.json`。
本地完整日志保留在迁移工作目录`Mujoco-Linux/output/unified_project_20261105/`，不作为新设备运行依赖。
本次未创建Git提交、推送或PR；用户需要交付时整体提交并推送，新设备才能从远端取得这些改动。
