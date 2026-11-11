# PH-EXO 本地编辑与 Orin 开发同步

2026-11-06。正式要求见[项目管理要求](project_management.md)，本页提供操作入口。

本地源码：`C:/Users/admin/Desktop/SSH-ORIN/ROS工程/jetson_mech_control`。
Orin开发副本：`/home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control`。

日常顺序：本地保存修改 → Windows验证 → SSH同步 → 核对两端 → Orin构建/测试 → 继续在本地修改。
同步直接使用当前Git工作树，包括未提交的新文件，不要求先commit、push或PR。

在本地仓库根目录的PowerShell执行：

```powershell
.\tools\project\Sync-Orin.ps1 status
.\tools\project\Sync-Orin.ps1 sync
.\tools\project\Sync-Orin.ps1 build -Profile imu
.\tools\project\Sync-Orin.ps1 test -Profile imu
```

`status`列出差异；`sync`完成后核对逐文件SHA256和Linux执行位。`build`和`test`通过SSH在Orin开发副本中执行。构建组合可选`imu`、`state`、`servo_imu`、`all`。这四个动作都不启动传感器、电机或控制器。

需要Python 3.10+、Git及OpenSSH命令。同步前状态校验现已包含实际MuJoCo；先按恢复说明创建`.local/venv`，PowerShell入口优先选择该环境。其他电脑可用`-Python '解释器完整路径'`指定；源代码不绑定Inkscape或Codex的安装位置。原共享SSH日志入口存在时自动复用；在新设备上也可独立执行，不要求复制共享SSH脚本。

不使用PowerShell时可调用同一入口：

```sh
python tools/project/sync_orin.py status
python tools/project/sync_orin.py sync
python tools/project/sync_orin.py build --profile imu
python tools/project/sync_orin.py test --profile imu
```

SSH连接使用`config/parallel_exo/orin_sync.json`中的`orin-nx`别名，沿用用户SSH配置与主机密钥检查。配置中只保存主机别名和项目路径，不保存密码或密钥。

## 一致范围

源码、模型、配置基线、依赖清单和运行说明同步到 Orin。报告由同步规则排除，新增报告存放在 Windows 仓库的 `reports/` 或 `docs/parallel_exo/reports/`。原始记录及模型试验输出按本页归档流程迁回 Windows。Git 元数据、编译安装目录和机器专用配置分别维护。

本地删除的受管文件会在Orin开发副本中删除。未登记的远端源码、单独修改的受管文件或路径链接会阻止同步，不使用强制覆盖；先把需要保留的远端修改比对回本地，再重新同步。两端已经变成相同内容的修改可以接纳。

传输先进入`.local/sync/incoming`并核验，再更新开发源码；替换前保留本次涉及文件的副本。若应用过程被中断，保留`incomplete.json`并阻止后续同步/构建，需依据其中的文件清单和`before`副本核对恢复，不直接删标记后继续。

同步与构建/测试共用锁，构建结果绑定源码清单。测试要求对应源码已通过构建；本地新增修改后先同步再构建。源码清单摘要比Git HEAD更适合识别未提交开发内容。

## 登录Orin

需要交互终端时，在本地执行`ssh orin-nx`，然后：

```sh
cd /home/nvidia/parallel_exo_ros_framework/dev/jetson_mech_control
```

在Orin查看日志或调试；正式源码继续在本地编辑。优先从本地上述入口发起构建/测试以记录清单；直接运行`bash tools/project/ros_workspace.sh build imu`也会遵守同步锁和中断标记，但不替代本地入口的版本验证记录。

MuJoCo环境按[恢复说明](restore.md)在该开发目录中建立，不能直接搬动旧虚拟环境。旧`current`/`servo-current`仍是已部署版本；本页的开发同步不切换它们。

Git负责按需提交、远程保存与PR，SSH负责本地到Orin的日常开发同步。两条流程独立；同步成功不会自动创建提交、推送或PR。

同步工具按受管清单和哈希处理曾经上传的报告。同步恢复副本用于处理中断。运行期间产生的测试数据按下述流程单独归档。

## 测试结果归档

1. 在 Orin 为本次测试指定独立输出目录，开发副本优先用 `.local/validation/<本次运行标识>/`。等待采集与控制进程退出、日志和 trace 写盘；异常退出的记录也要归档。
2. 记录运行目录下的相对路径、字节数和 SHA256，将原始报文、试验配置、日志和结果文件迁回 Windows 项目的 `Mujoco-Linux/output/`。已有同内容副本可直接核验复用；使用压缩包时逐成员核验。报告在 Windows 编写。
3. 核对 Windows 副本与 Orin 清单完全一致，并确认文件已停止写入，再仅移除本次清单中内容未变的远端文件和已空的运行目录。缺件、哈希不符或仍被进程占用时保留远端原件，待补传后复核。Windows 留存来源、校验清单及清理记录。

源码同步独立于归档。可复现测试输入、程序、模型，以及设备运行需要的配置与安装环境按开发流程维护。
