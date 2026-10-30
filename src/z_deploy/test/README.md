# 硬件状态与按键检查

程序根据 `--robot` 和 `--mode` 选择接收派生类，每秒刷新 10 次 IMU 四元数、
RPY、角速度、加速度和各关节位置。角度单位为弧度，关节顺序与对应机器人的
`base.yaml` 一致。

- `hardware_base.hpp`：接收基类，定义状态/手柄快照、加锁读取和虚函数接口。
- `hardware_sim.hpp`：ROS 状态接收派生类。
- `hardware_g1.hpp`：G1 SDK2 接收派生类，接收状态并解码手柄。
- `hardware_g1_23.hpp`：继承 G1 接收类，指定 23 关节的 SDK 索引。
- `hardware.hpp`：根据机器人和模式创建接收对象。
- `test_hardware.cpp`：参数解析和终端显示。

实机手柄读取 `rt/lowstate` 中的 `wireless_remote`，显示全部按住的按键（如
`LB+DPadUp`）、原始 16 位掩码、最近按下/松开的按键、按键状态变化次数，以及
LX/LY/RX/RY/L2 原始轴值。L1/R1 显示为部署程序使用的 LB/RB；LY/RY 正方向
沿用 SDK 的向上方向。轴值不做死区或滤波，便于检查实际输入。
最近按下/松开的记录保留到下一次同类变化，方便查看两次刷新之间的短按。
手柄接收状态表示 LowState 字段的新鲜度，不代表检测到了遥控器无线连接。
ROS 仿真后端只接收状态话题，手柄区域显示不可用。

程序不依赖 `core/hardware/`，不加载策略、不发送关节命令、不调用仿真 reset/pause
服务、不接管实机运动控制，退出时也不发送阻尼命令。关节或 IMU 超过 1 秒没有新状态时显示
`STALE`；等待关节和 IMU 首帧到齐后才显示数据。按 Ctrl+C 退出。
本工具检查消息接收、关节映射和实机手柄输入，不验证正式硬件后端的启动、控制和退出逻辑。

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
