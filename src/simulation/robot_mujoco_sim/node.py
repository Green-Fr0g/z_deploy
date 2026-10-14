"""ROS 2 interface: state publishing, command subscription and reset service."""
from __future__ import annotations

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from z_msgs.msg import ImuData, JointData, JointsCmd, JointsData

from .config import SimConfig
from .model import RobotModel


class RobotMujocoSimNode(Node):
    """Bridges the MuJoCo simulation to ROS 2 topics and services.

    ``lock`` guards ``robot.data``: the callbacks below and the simulation loop
    must both hold it while reading or writing physics state.
    """

    def __init__(self, config: SimConfig, robot: RobotModel) -> None:
        super().__init__('robot_mujoco_sim_node')
        self._config = config
        self._robot = robot
        self.lock = threading.Lock()
        self._last_command_time = self.get_clock().now()
        self._paused = False

        self._state_publisher = self.create_publisher(JointsData, config.state_topic, 2)
        self._imu_publisher = self.create_publisher(ImuData, config.imu_topic, 2)
        self.create_subscription(JointsCmd, config.command_topic, self._on_command, 2)

        self.create_service(Empty, config.reset_service, self._on_reset)
        self.create_service(Empty, config.pause_service, self._on_pause)
        self.create_service(Empty, config.resume_service, self._on_resume)

        self._state_timer = self.create_timer(
                    1.0 / config.state_publish_frequency, self._publish_state)
        self._imu_timer = self.create_timer(
            1.0 / config.imu_publish_frequency, self._publish_imu)

    # -- command watchdog ---------------------------------------------------
    def command_is_stale(self) -> bool:
        """Whether no command arrived recently.

        Must be called while holding ``lock``. A ``command_timeout`` of zero or
        less disables the watchdog.
        """
        if self._config.command_timeout <= 0.0:
            return False
        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds * 1e-9
        return elapsed > self._config.command_timeout

    # -- callbacks ----------------------------------------------------------
    def _on_command(self, message: JointsCmd) -> None:
        with self.lock:
            self._robot.apply_joint_commands(message.joints_cmd)
            self._last_command_time = self.get_clock().now()

    def _publish_state(self) -> None:
        with self.lock:
            message = JointsData()
            message.stamp = self.get_clock().now().to_msg()
            for position, velocity, torque in self._robot.read_joint_states():
                joint = JointData()
                joint.position = position
                joint.velocity = velocity
                joint.torque = torque
                message.joints_data.append(joint)
        self._state_publisher.publish(message)

    def _on_reset(self, request, response):
        del request
        with self.lock:
            self._robot.reset_to_keyframe('home')
            self._last_command_time = self.get_clock().now()
        return response

    # -- pause / resume -----------------------------------------------------
    def _on_pause(self, request, response):
        del request
        self._paused = True
        self.get_logger().info('Simulation paused')
        return response

    def _on_resume(self, request, response):
        del request
        self._paused = False
        self._last_command_time = self.get_clock().now()
        self.get_logger().info('Simulation resumed')
        return response

    def is_paused(self) -> bool:
        """Whether the physics loop should hold still.

        Intentionally lock-free: the loop calls this while already holding
        ``lock``, and a plain bool read is atomic under the GIL.
        """
        return self._paused

    def _publish_imu(self) -> None:
        """Publish the trunk IMU and Euler angles in a custom message.

        Covariances stay at zero because the MuJoCo sensors are noiseless.
        """
        with self.lock:
            imu = self._robot.read_imu()
            if imu is None:
                return
            orientation, angular_velocity, linear_acceleration = imu
            message = ImuData()
            message.stamp = self.get_clock().now().to_msg()
            message.orientation_w = float(orientation[0])
            message.orientation_x = float(orientation[1])
            message.orientation_y = float(orientation[2])
            message.orientation_z = float(orientation[3])
            message.angular_velocity_x = float(angular_velocity[0])
            message.angular_velocity_y = float(angular_velocity[1])
            message.angular_velocity_z = float(angular_velocity[2])
            message.linear_acceleration_x = float(linear_acceleration[0])
            message.linear_acceleration_y = float(linear_acceleration[1])
            message.linear_acceleration_z = float(linear_acceleration[2])
            message.roll, message.pitch, message.yaw = _quaternion_to_euler(orientation)
        self._imu_publisher.publish(message)


def _quaternion_to_euler(quaternion: np.ndarray) -> tuple[float, float, float]:
    """Convert a MuJoCo ``w, x, y, z`` quaternion to roll, pitch and yaw."""
    w, x, y, z = (float(value) for value in quaternion[:4])
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(pitch_term, -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)
