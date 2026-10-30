# 硬件接收检查

本目录保留接收检查程序 `test_hardware.cpp` 和独立的只读实现 `hardware.hpp`。
程序根据 `--robot` 和 `--mode` 选择 ROS 或 SDK2 订阅，每秒刷新 10 次
IMU 四元数、RPY、角速度、加速度和
各关节位置。角度单位为弧度，关节顺序与对应机器人的 `base.yaml` 一致。

程序不依赖 `core/hardware/`，不加载策略、不发送关节命令、不调用仿真 reset/pause
服务、不接管实机运动控制，退出时也不发送阻尼命令。关节或 IMU 超过 1 秒没有新状态时显示
`STALE`；等待关节和 IMU 首帧到齐后才显示数据。按 Ctrl+C 退出。
本工具检查消息接收与关节映射，不验证正式硬件后端的启动、控制和退出逻辑。

构建后直接运行：

```bash
./build/z_deploy/test_hardware --robot g1_23 --mode sim

export Z_NETIF=eth0
./build/z_deploy/test_hardware --robot g1_23 --mode real
```

安装后也可通过 `ros2 run z_deploy test_hardware ...` 启动。实机网卡变量沿用
`source ./z_build.sh --net eth0` 使用的 `Z_NETIF`，填写实际网卡名。

仿真支持 `g1`、`g1_23`、`dr02`、`adam`；编译启用 SDK2 时，实机支持 `g1`、`g1_23`。
这是交互式接收检查工具，不作为自动 CTest 用例运行。
