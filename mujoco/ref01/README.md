# REF01控制计算接口

本目录仅保留运动学、动力学接口、计算依赖、运行必需的MJCF/STL与参数，以及小型数值验证基线。
不包含CAD/MATLAB工程或建模、Viewer、报告和历史打包工具。
STL是MuJoCo运行模型引用的网格资产，不是CAD编辑工程。
使用仓库根目录的`tools/project/restore.py`恢复、验证或运行接口示例。
接口说明位于`docs/`；测试输入位于`fixtures/`，验证不要求MATLAB安装。
生成结果位于忽略的`output/`。ROS与MuJoCo闭环尚需后续控制集成。
