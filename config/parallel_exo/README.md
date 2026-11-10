# PH-EXO 现场配置

第五路CtrBoard的随源码默认配置位于`ros2_ws/src/mech_ctrboard_bridge/config/ctrboard_usb_cdc.yaml`，传感器坐标标识为`imu_1_link`/`imu_2_link`，不意味着已完成安装标定。

`servo_pair.ref01.json`为2026-11-05读取的现有机器人完整配置副本，保留原有限位、映射和标定说明，将依赖Orin插口位置的by-path改为同一通信板的by-id标识。两种路径已只读核实指向相同端口；未改动Orin配置。来源哈希和派生修改见导入清单。

`servo_pair.example.json`仅说明IMU端口冲突检查所需的`buses[].device_path`结构，不是可启动电机的完整配置，也没有填造关节限位。迁移同一机器人时，核对硬件身份后可复制`servo_pair.ref01.json`为`servo_pair.local.json`（Git忽略）；换机器人时填写实际完整配置，也可向启动脚本显式传入配置路径。

新机器保留同一机器人、同一通信板时可以迁移已确认的现场标定文件；换机器人或更改机械装配时重新核对。标定文件与Git提交号应在部署记录中配对，不能把示例当作实测配置。
