# z_deploy

MuJoCo + ROS 2 Humble 仿真与策略部署工作区，包含 `robot_mujoco_sim` 仿真节点、`z_msgs` 自定义消息和 `z_deploy` C++ 部署包。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- 系统 Python 3.10（`/usr/bin/python3`，与 ROS 2 Humble 的二进制扩展一致）
- 使用原生 MuJoCo viewer 时需要图形会话（`DISPLAY` 或 `WAYLAND_DISPLAY`）

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 --version
```

## 获取仓库和子模块

Unitree SDK2 位于 `src/z_deploy/library/unitree_sdk2`，作为 Git 子模块固定到项目使用的 SDK2.0 版本。首次获取仓库时初始化子模块：

```bash
git clone --recurse-submodules <repository-url>
cd z_deploy
```

已有工作区执行：

```bash
git submodule update --init --recursive
git -C src/z_deploy/library/unitree_sdk2 describe --tags --always
```

SDK 的预编译库按 CPU 架构放在子模块的 `lib/x86_64` 和 `lib/aarch64` 中。当前 `z_deploy` 默认运行仿真模式；后续启用真实 Unitree 硬件后，直接复用该子模块，不需要把 SDK 源码复制到主仓库。

子模块初始化后，SDK 文件会正常出现在工作区目录中，但主仓库只跟踪 `src/z_deploy/library/unitree_sdk2` 这一条提交指针；`git status` 不会逐个列出 SDK 文件。若暂时不需要本地展开 SDK，可执行 `git submodule deinit src/z_deploy/library/unitree_sdk2`，之后用 `git submodule update --init` 恢复。

## 安装依赖

直接使用系统 Python。ROS 和构建工具依赖由 apt 提供，在项目根目录执行：

```bash
sudo apt install build-essential python3-dev python3-pip python3-setuptools \
  python3-colcon-common-extensions python3-empy python3-catkin-pkg-modules python3-lark \
  python3-numpy python3-scipy python3-yaml \
  ros-humble-ament-cmake ros-humble-ament-package \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-rclpy ros-humble-ament-index-python ros-humble-std-srvs

/usr/bin/python3 -m pip install --user -r requirements.txt
```

`requirements.txt` 安装 MuJoCo、Mjviser、Viser 等仿真依赖到当前用户目录，供系统 Python 直接使用。NumPy 限制为 `<1.25`，兼容 ROS Humble 的消息扩展和 Ubuntu 22.04 自带的 SciPy。不要用 `sudo pip` 安装，也不要在 Conda 或其他虚拟环境中构建、运行本项目。

## 构建 ROS 2 工作区

在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 /usr/bin/colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

也可以使用项目构建脚本（脚本会检查推理运行时、MuJoCo 和仿真模型）：

```bash
source /opt/ros/humble/setup.bash
./z_build.sh
source install/setup.bash
```

只构建部署包：

```bash
colcon build --merge-install --symlink-install --packages-select z_deploy \
  --cmake-args -DBUILD_TESTING=ON
```

重新打开终端时，需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动仿真器

默认读取 `src/simulation/config/sim.yaml`：

```bash
ros2 run robot_mujoco_sim robot_mujoco_sim
```

常用参数：

```bash
# 选择机器人和场景
ros2 run robot_mujoco_sim robot_mujoco_sim --robot h2 --scene empty.xml

# 使用 Mjviser 网页 viewer
ros2 run robot_mujoco_sim robot_mujoco_sim --viewer mjviser

# 让 viewer 相机跟随机器人
ros2 run robot_mujoco_sim robot_mujoco_sim --camera

# 组合使用
ros2 run robot_mujoco_sim robot_mujoco_sim --robot h2 --scene empty.xml --viewer mjviser --camera

# 查看完整帮助
ros2 run robot_mujoco_sim robot_mujoco_sim --help
```

`--scene` 优先在 `robots_model/<robot>_mjcf/mjcf/` 下解析，也支持模型根目录；不带 `.xml` 后缀时会自动补上。例如：

```text
--robot h2 --scene empty
→ robots_model/h2_mjcf/mjcf/empty.xml
```

### Adam

当前 `sim.yaml` 默认选择 Adam。`adam_mjcf/empty.xml` 使用 `robot_fix.xml`，
包含地面、27 个力矩执行器、关节传感器和 IMU，Reset 回到默认站立姿态。
物理步进使用 2000 Hz，避免低惯量手腕在显式 PD 下振荡，状态和 IMU 发布保持 200 Hz。
仿真 ROS 顺序与 `policy/adam/base.yaml` 一致，策略通过 `joint_mapping` 对接。

分别在两个已加载 ROS 环境的终端启动：

```bash
ros2 run robot_mujoco_sim robot_mujoco_sim --robot adam --scene empty.xml --viewer mjviser
ros2 run z_deploy z_deploy_main --robot adam --mode sim
```

部署端从 `Disabled` 开始：按 `P` 进入阻尼模式，按 `0` 用两秒插值到默认姿态，
完成后按 `1` 启动 locomotion；策略运行时按 `P` 返回阻尼模式。
策略为 `policy/adam/locomotion/policy.pt`，输入 450 维、输出 27 维，推理 50 Hz，
每个观测项按 `[4, 3, 2, 1, 0]` 堆叠五个推理步的历史。

## 部署按键

数字键按下表映射为手柄组合键，具体策略仍由各机器人的状态机绑定：

| 数字键 | 手柄组合键 | 数字键 | 手柄组合键 |
| --- | --- | --- | --- |
| `1` | LB + 上 | `5` | RB + 上 |
| `2` | LB + 下 | `6` | RB + 下 |
| `3` | LB + 左 | `7` | RB + 左 |
| `4` | LB + 右 | `8` | RB + 右 |

已有基础按键保持原映射：`0` 对应 A，`P` 对应 LB+X，`R` 重置仿真，
Enter 暂停/继续仿真。`W/S/A/D` 调整平移指令，`Q/E` 调整转向指令，
Space 清零速度；普通方向键仍对应未加 LB/RB 的方向键。
没有绑定策略的组合键保持当前状态。所有机器人状态机的 `check_change()`
使用 `if` 判断，保留原有的按键边沿检测、站立完成限制和策略失败回退。

G1 的策略绑定如下，站立完成后可进入，运行中也可用这些组合键互相切换：

| 数字键 | 手柄组合键 | 策略目录 |
| --- | --- | --- |
| `1` | LB + 上 | `g1/locomotion` |
| `2` | LB + 下 | `g1/amp` |
| `3` | LB + 左 | `g1/gangnam_style` |
| `4` | LB + 右 | `g1/dance_102` |
| `5` | RB + 上 | `g1/amp_zlab` |
| `6` | RB + 下 | `g1/mimic` |
| `7` | RB + 左 | `g1/zlocomotion` |
| `8` | RB + 右 | 预留，保持当前状态 |

`gangnam_style`、`dance_102`、`mimic` 播放结束后自动返回 `locomotion`。
AMP 和 AMP Zlab 使用 LB+X 返回 Passive、Y 返回 GetUp；
其余 G1 策略使用 Y 返回 Passive、RB+A 返回 GetUp。

## Viewer 说明

### native

MuJoCo 原生窗口是默认 viewer：

- `--camera` 或 `enable_camera_follow: 1` 启用跟随相机
- `follow_yaw: 1` 让跟随相机同时跟随机器人绕 Z 轴的 yaw 角；设为 `0` 时只跟随位置
- 吊带长度限制为 `0.2` 到 `连接点 z + 1` 米，按 `7`/`8` 或网页滑块调节
- 吊带启用时，按 `7` 减小长度、按 `8` 增大长度、按 `9` 切换启用状态
- 需要图形会话

### mjviser

Mjviser 通过浏览器提供 Web viewer。启动后终端会显示访问地址，浏览器打开该地址即可。

Mjviser 自带 Pause / Play、Step、Reset、Speed、Keyframe 以及相机和场景可视化设置。吊带启用时，页面还会显示吊带长度滑块和启用开关。Mjviser 的相机跟随由 `--camera` 或 `enable_camera_follow: 1` 控制，`follow_yaw: 1` 时也会跟随 yaw 角。

## 配置文件

配置文件位于 `src/simulation/config/sim.yaml`：

```yaml
robot: "adam"
scene: "empty.xml"
sim_frequency: 2000.0
state_publish_frequency: 200.0
imu_publish_frequency: 200.0
command_timeout: 0.25
print_scene_information: 0
enable_elastic_band: 0
enable_camera_follow: 0
follow_yaw: 0
viewer: "native"
```

`print_scene_information`、`enable_elastic_band`、`enable_camera_follow` 和 `follow_yaw` 使用 `0/1` 配置。命令行参数优先级高于配置文件：`--robot`、`--scene`、`--viewer` 和 `--camera` 可以覆盖对应配置。

## ROS 接口

默认话题和服务名称由 `sim.yaml` 配置。仿真节点提供：

- 关节命令订阅：`<robot>/command`
- 关节状态发布：`<robot>/state`
- IMU 发布：`<robot>/imu`
- 重置服务：`<robot>/reset`
- 暂停服务：`<robot>/pause`
- 继续服务：`<robot>/resume`

例如，H2 默认配置下：

```bash
ros2 service call h2/reset std_srvs/srv/Empty '{}'
ros2 service call h2/pause std_srvs/srv/Empty '{}'
ros2 service call h2/resume std_srvs/srv/Empty '{}'
```

## 常见问题

### 找不到 `rclpy` 或 `z_msgs`

确认当前终端按顺序加载了 ROS 和工作区：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果刚修改了消息或 ROS 包，需要重新构建：

```bash
/usr/bin/python3 /usr/bin/colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 原生 viewer 无法打开

确认当前终端存在图形环境：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

如果使用 SSH，需要配置 X11/Wayland 图形转发；也可以改用：

```bash
ros2 run robot_mujoco_sim robot_mujoco_sim --viewer mjviser
```
