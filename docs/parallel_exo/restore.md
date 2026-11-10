# 在新设备恢复 PH-EXO

所有命令均在本仓库根目录执行；可选择任意本地绝对路径。首次需要网络获取Git与锁定的依赖。

## 取得完整源码

在原设备完成提交并推送后，在新设备执行：

```sh
git clone --branch ph-exo/development https://github.com/Ph-533K3R/jetson_mech_control.git
cd jetson_mech_control
git rev-parse HEAD
```

迁移应记录并核对原设备与新设备的完整提交号。当前未提交/未推送的文件无法通过远端clone获得；不需要等PR合并才能检出已推送的开发分支。

## MuJoCo与接口

安装Python 3.10；Ubuntu需要`python3.10-venv`。Windows使用`py -3.10`，Ubuntu 22.04使用`python3`。

```sh
python3 tools/project/restore.py inspect
python3 tools/project/restore.py setup-python
python3 tools/project/restore.py verify
python3 tools/project/restore.py example
```

Windows将上述`python3`替换为`py -3.10`。`setup-python`只在本仓库`.local/venv`创建环境，按`manifests/parallel_exo_python.lock.txt`安装；不同系统和CPU架构重新安装，不复制旧虚拟环境。

如果精简版Linux缺少ensurepip，但机器已有另一Python环境的pip >=22.3，可使用
`python3 tools/project/restore.py setup-python --pip-python /已安装pip的环境/bin/python`。
该选项只借用包安装工具，依赖仍安装到本仓库独立环境；模型运行不依赖那个旧环境。

使用`verify`验证原运动学/动力学接口与新MuJoCo正解/状态逻辑，`example`演示无界面的动力学接口；不打开Viewer或设备，不需要安装CAD或MATLAB。

## ROS 2 Humble

目标为Ubuntu 22.04，安装ROS 2 Humble、colcon、rosdep和构建工具。系统安装按[ROS官方安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)执行；仓库脚本不刷机、不自动升级JetPack或系统。

在已安装并初始化rosdep的设备上，按工程声明解析依赖：

```sh
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble -r -y
bash tools/project/ros_workspace.sh build imu
bash tools/project/ros_workspace.sh test imu
```

`state`构建接入MuJoCo正解的状态包；`state/servo_imu/all`需要先恢复本仓库Python环境。`servo_imu`包含伺服及其依赖，`all`构建全部包。构建结果位于`.local/ros/<组合>/`，不会使用旧部署的安装层。构建和测试不会执行真实设备启动入口。

## 现场配置与IMU

检查`config/parallel_exo/README.md`。仓库已附当前机器人配置`servo_pair.ref01.json`；迁移同一机器人时核对身份后复制为忽略的`servo_pair.local.json`，或向启动脚本传入实际配置路径。固定设备标识对应当前通信板；换板时按实际标识填写，不能把另一台板的序列号当作通用值。

只有准备进行传感器任务时，才显式执行：

```sh
bash tools/project/start_imu.sh /dev/serial/by-id/实际第五路设备 /实际路径/servo_pair.json
```

该入口检查设备存在、权限、端口占用与电机端口冲突，然后启动传感器桥接。电机控制和当前Orin版本切换均另行实施。历史部署的`servo-current`/`current`不会随Git操作变化。

## 后续开发和同步

在这套源码中修改，按需整体提交到开发分支并推送；用户需要时整体发起PR。新设备先保存自己的修改，再获取并合入需要的提交，不自动覆盖本地未提交内容。

恢复控制运行工程不需要旧电脑的SSH目录、CAD/MATLAB项目、历史ZIP、安装日志和缓存。模型重建属于独立建模任务，其结果按版本更新本仓库的运行模型；本仓库集中维护控制运行和验证需要的内容。
