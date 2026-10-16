"""Application wiring: viewer, simulation loop, signals and shutdown."""
from __future__ import annotations

import contextlib
import signal
import threading
import time

import rclpy

from .config import SimConfig
from .model import ElasticBand, RobotModel
from .node import RobotMujocoSimNode
from .viewer import ViewerError, create_viewer


class SimulatorApp:
    """Runs the physics loop next to a ROS 2 executor thread.

    Closing the MuJoCo window, pressing Ctrl+C and receiving SIGTERM all lead
    to the same shutdown path: the loop stops, ROS is shut down, the executor
    thread is joined and every graphical resource is released.
    """

    def __init__(self, config: SimConfig, *, show_viewer: bool = True,
                 follow_camera: bool | None = None) -> None:
        self._config = config
        self._show_viewer = show_viewer
        self._follow_camera = (config.enable_camera_follow if follow_camera is None
                               else follow_camera)
        self._stop = threading.Event()

    def run(self) -> int:
        """Start the simulator and block until it is asked to stop."""
        self._stop.clear()
        rclpy.init()
        node = None
        executor = None
        try:
            robot = RobotModel(self._config.model_path, self._config.timestep)
            if self._config.print_scene_information:
                robot.print_scene_information()
            node = RobotMujocoSimNode(self._config, robot)
            duplicates = _wait_for_sole_publisher(node, self._config.state_topic)
            if duplicates:
                node.get_logger().error(
                    f'{duplicates} other publisher(s) already own '
                    f'{self._config.state_topic!r}. Refusing to start: two '
                    'simulators on the same topics interleave their states, so '
                    'the controller sees two different robots and goes unstable. '
                    'Stop the other instance first.')
                return 1
            self._log_startup(node, robot)

            executor = threading.Thread(target=_spin_ros, args=(node,),
                                        name='ros-executor', daemon=True)
            executor.start()
            with _signal_handlers(self._stop):
                elastic_band = ElasticBand() if self._config.enable_elastic_band else None
                if not self._show_viewer:
                    raise ViewerError('A viewer is required to run the simulator.')
                viewer = create_viewer(
                    self._config.viewer,
                    robot=robot,
                    node=node,
                    elastic_band=elastic_band,
                    follow_camera=self._follow_camera,
                    stop_event=self._stop,
                    timestep=self._config.timestep,
                    robot_name=self._config.robot,
                    follow_yaw=self._config.follow_yaw,
                )
                try:
                    viewer.run()
                finally:
                    viewer.close()
        finally:
            self._shutdown(node, executor)
        return 0

    # -- startup and shutdown ----------------------------------------------
    def _log_startup(self, node: RobotMujocoSimNode, robot: RobotModel) -> None:
        config = self._config
        node.get_logger().info(
            f'Loaded {config.model_path} ({robot.model.nq} qpos, '
            f'{robot.num_actuators} actuators, timestep={config.timestep:.6f}s)')
        if self._follow_camera:
            node.get_logger().info('Viewer: open, camera follows the robot')
        else:
            node.get_logger().info('Viewer: open, free camera (--camera to follow the robot)')
        node.get_logger().info(f'Render backend: {config.viewer}')

        node.get_logger().info(
            f'IMU: publishing {config.imu_topic!r} at '
            f'{config.imu_publish_frequency:.0f} Hz')
        
    def _shutdown(self, node, executor: threading.Thread | None) -> None:
        self._stop.set()
        if rclpy.ok():
            rclpy.shutdown()
        if executor is not None:
            executor.join(timeout=2.0)
        if node is not None:
            node.destroy_node()


def _spin_ros(node: RobotMujocoSimNode) -> None:
    """Run the ROS executor until the context is shut down."""
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass


def _wait_for_sole_publisher(node: RobotMujocoSimNode, topic: str,
                             timeout: float = 2.0) -> int:
    """Return how many publishers besides ours share ``topic``.

    Spins the node briefly so DDS discovery can populate the graph. A stale
    ``robot_mujoco_sim`` from a previous session keeps publishing /a1/state and
    /a1/command; the controller then reads two interleaved robots and loses
    balance at random, which is very hard to diagnose from behaviour alone.
    """
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        extra = node.count_publishers(topic) - 1
        if extra > 0:
            return extra
    return 0


@contextlib.contextmanager
def _signal_handlers(stop: threading.Event):
    """Turn Ctrl+C (SIGINT) and SIGTERM into a clean stop request."""

    def handler(signum, frame):
        del signum, frame
        stop.set()

    installed = {sig: signal.signal(sig, handler)
                 for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, previous in installed.items():
            signal.signal(sig, previous)


