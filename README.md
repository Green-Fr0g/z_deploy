# z_deploy

MuJoCo + ROS 2 Humble 仿真部署工作区，包含 `robot_mujoco_sim` 仿真节点和 `z_msgs` 自定义消息。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10（ROS 2 Humble 的 `rclpy` 和生成消息绑定使用此版本）
- [uv](https://docs.astral.sh/uv/)
- 使用原生 MuJoCo viewer 时需要图形会话（`DISPLAY` 或 `WAYLAND_DISPLAY`）

```bash
source /opt/ros/humble/setup.bash
uv --version
python3 --version
```

## 安装 Python 环境

在项目根目录执行：

```bash
uv venv --python 3.10
uv sync
source .venv/bin/activate
```

`uv sync` 会在项目根目录创建 `.venv`，并安装 MuJoCo、Mjviser、Viser、NumPy、SciPy 和 PyYAML 等依赖。项目要求 Python `>=3.10,<3.11`。

## 构建 ROS 2 工作区

激活 `.venv` 后，在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```

重新打开终端时，需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
```

## 启动仿真器

默认读取 `src/simulation/config/sim.yaml`：

```bash
ros2 run robot_mujoco_sim robot_mujoco_sim
```

也可以直接调用命令行入口：

```bash
robot_mujoco_sim
```

常用参数：

```bash
# 选择机器人和场景
robot_mujoco_sim --robot h2 --scene empty.xml

# 使用 Mjviser 网页 viewer
robot_mujoco_sim --viewer mjviser

# 让 viewer 相机跟随机器人
robot_mujoco_sim --camera

# 组合使用
robot_mujoco_sim --robot h2 --scene empty.xml --viewer mjviser --camera

# 查看完整帮助
robot_mujoco_sim --help
```

`--scene` 会在 `robots_model/<robot>_mjcf/mjcf/` 下解析；不带 `.xml` 后缀时会自动补上。例如：

```text
--robot h2 --scene empty
→ robots_model/h2_mjcf/mjcf/empty.xml
```

## Viewer 说明

### native

MuJoCo 原生窗口是默认 viewer：

- `--camera` 或 `enable_camera_follow: 1` 启用跟随相机
- 吊带启用时，按 `7` 减小长度、按 `8` 增大长度、按 `9` 切换启用状态
- 需要图形会话

### mjviser

Mjviser 通过浏览器提供 Web viewer。启动后终端会显示访问地址，浏览器打开该地址即可。

Mjviser 自带 Pause / Play、Step、Reset、Speed、Keyframe 以及相机和场景可视化设置。吊带启用时，页面还会显示吊带长度滑块和启用开关。Mjviser 的相机跟随由 `--camera` 或 `enable_camera_follow: 1` 控制。

## 配置文件

配置文件位于 `src/simulation/config/sim.yaml`：

```yaml
robot: "h2"
scene: "empty.xml"
sim_frequency: 500.0
state_publish_frequency: 200.0
imu_publish_frequency: 200.0
command_timeout: 0.25
print_scene_information: 0
enable_elastic_band: 0
enable_camera_follow: 0
viewer: "native"
```

`print_scene_information`、`enable_elastic_band` 和 `enable_camera_follow` 使用 `0/1` 配置。命令行参数优先级高于配置文件：`--robot`、`--scene`、`--viewer` 和 `--camera` 可以覆盖对应配置。

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

确认当前终端按顺序加载了 ROS、虚拟环境和工作区：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
```

如果刚修改了消息或 ROS 包，需要重新构建：

```bash
colcon build --symlink-install
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
robot_mujoco_sim --viewer mjviser
```
