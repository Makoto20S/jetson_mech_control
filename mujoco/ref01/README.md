# PH-EXO 控制计算接口

本目录仅保留运动学、动力学接口、计算依赖、运行必需的MJCF/STL与参数，以及小型数值验证基线。
不包含CAD/MATLAB工程或建模、Viewer、报告和历史打包工具。
STL是MuJoCo运行模型引用的网格资产，不是CAD编辑工程。
使用仓库根目录的`tools/project/restore.py`恢复、验证或运行接口示例。
接口说明位于`docs/`；测试输入位于`fixtures/`，验证不要求MATLAB安装。
生成结果位于忽略的`output/`。ROS 与 MuJoCo 连续阻抗软件闭环见 [运行说明](../../docs/parallel_exo/impedance_simulation.md)。

## 固定姿态双自由度阻抗（仿真）

`fixed_pose_impedance.py` 的 `FixedPoseImpedance.command(q_rad, dq_rad_s)` 在 `[ψ, β]` 上计算 `K*(q_ref-q)-D*dq`，叠加当前姿态的静态重力补偿，用 `A.T` 求解 `[α1,α2]` 轴端目标。成对限幅用同一缩放系数；`motor104_105_nm` 按名义传动映射重排为 `[104,105]`。输入为有效的姿态和角速度。

在仓库根目录，用本仓库的 MuJoCo Python 环境运行 `mujoco/ref01/tools/verify_fixed_pose.py --output <新的输出目录>`。Windows 解释器为 `.local/venv/Scripts/python.exe`，Orin 为 `.local/venv/bin/python`。省略 `--output` 时只打印检查结果。每个工况独立复位；CSV 记录每个控制区间起点的状态和该区间保持的力矩，`*_end_*` 表示区间结束时的测量。

默认参考为 (0°,20°)，K=(2.4,2.0) N·m/rad，D=(0.18,0.15) N·m·s/rad，周期为 5 ms，模型每步 0.5 ms。0.4–1.0 s 在 ψ 或 β 施加 +0.09 N·m。上述增益和 ±0.20 N·m 限幅仅用于本模型仿真。验收同时检查重力平衡、受扰方向、静态偏移与外力矩/K 的一致性、撤扰后两轴角差/速度的持续收敛、实际执行力矩和闭链残差。

Orin 私有双总线验证可用 `tools/project/test_mit_pair_pty.py --impedance-trace <输出目录>/traces.csv`。从两段轨迹各选一组原样目标，检查真实 ROS 成对控制器到两个虚拟端口的编码与到期停发。

项目名称统一为 PH-EXO，程序标识 ph_exo；ref01 路径与类名作为现有模型接口的兼容标识保留。
