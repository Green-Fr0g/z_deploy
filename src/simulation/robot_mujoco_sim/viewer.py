"""Viewer backends for the MuJoCo simulator."""
from __future__ import annotations

import abc
import math
import os
import threading
import time
from typing import TYPE_CHECKING

import mujoco
import numpy as np

from .model import ElasticBand, RobotModel

if TYPE_CHECKING:
    from .node import RobotMujocoSimNode


class ViewerError(RuntimeError):
    """Raised when a configured viewer cannot be started."""


def yaw_degrees(rotation: np.ndarray) -> float:
    """Return the rotation around the world Z axis of a rotation matrix."""
    return math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))


NATIVE_CAMERA_DISTANCE = 1.6
NATIVE_CAMERA_ELEVATION = -18.0


def _mujoco_camera_offset(azimuth: float, elevation: float,
                          distance: float) -> np.ndarray:
    """Return a Viser camera offset matching MuJoCo's free-camera angles."""
    azimuth_radians = math.radians(azimuth)
    elevation_radians = math.radians(elevation)
    # MuJoCo's free camera uses the negative spherical direction for its
    # position.  Mjviser uses the same world axes, but its scene helper uses
    # the opposite elevation sign and a different default distance.
    return np.array((
        -distance * math.cos(elevation_radians) * math.cos(azimuth_radians),
        -distance * math.cos(elevation_radians) * math.sin(azimuth_radians),
        -distance * math.sin(elevation_radians),
    ), dtype=float)


def _mujoco_azimuth_from_offset(offset: np.ndarray) -> float:
    """Recover MuJoCo's azimuth from a camera offset around its target."""
    if math.hypot(float(offset[0]), float(offset[1])) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(-float(offset[1]), -float(offset[0])))


CAMERA_FOLLOW_BODY_REGISTRY: dict[str, str] = {
    'h2': 'torso_link',
    'g1': 'torso_link',
    'g1_23': 'torso_link',
    'dr02': 'body',
}


# Keep suspension attachments explicit and independent from camera behavior.
# The current robot models use the same upper-body link for both features, but
# changing camera framing must not move the elastic-band force to another link.
ELASTIC_BAND_BODY_REGISTRY: dict[str, str] = {
    'h2': 'torso_link',
    'g1': 'torso_link',
    'g1_23': 'torso_link',
    'dr02': 'body',
}


def camera_follow_body(robot_name: str) -> str:
    """Return the registered MuJoCo body used as the camera follow target."""
    key = robot_name.strip().lower()
    try:
        return CAMERA_FOLLOW_BODY_REGISTRY[key]
    except KeyError as error:
        registered = ', '.join(sorted(CAMERA_FOLLOW_BODY_REGISTRY))
        raise ViewerError(
            f'No camera follow body registered for robot {robot_name!r}; '
            f'registered robots: {registered}') from error


def elastic_band_body(robot_name: str) -> str:
    """Return the registered body receiving the elastic-band force."""
    key = robot_name.strip().lower()
    try:
        return ELASTIC_BAND_BODY_REGISTRY[key]
    except KeyError as error:
        registered = ', '.join(sorted(ELASTIC_BAND_BODY_REGISTRY))
        raise ViewerError(
            f'No elastic-band body registered for robot {robot_name!r}; '
            f'registered robots: {registered}') from error


class SimulatorViewer(abc.ABC):
    """Shared state and operations for the two viewer backends."""

    def __init__(self, robot: RobotModel, node: RobotMujocoSimNode,
                 elastic_band: ElasticBand | None, follow_camera: bool,
                 stop_event: threading.Event, robot_name: str,
                 follow_yaw: bool) -> None:
        self._robot = robot
        self._node = node
        self._elastic_band = elastic_band
        self._follow_camera = bool(follow_camera)
        self._follow_yaw = bool(follow_yaw)
        self._stop_event = stop_event
        self._camera_body = (camera_follow_body(robot_name)
                             if self._follow_camera else None)
        if self._camera_body is not None and robot.body_pose(self._camera_body) is None:
            raise ViewerError(
                f'Camera follow body {self._camera_body!r} is missing from '
                f'robot {robot_name!r} model')
        self._elastic_band_body_id: int | None = None
        if elastic_band is not None:
            band_body = elastic_band_body(robot_name)
            body_id = mujoco.mj_name2id(
                robot.model, mujoco.mjtObj.mjOBJ_BODY, band_body)
            if body_id < 0:
                raise ViewerError(
                    f'Elastic-band body {band_body!r} is missing from '
                    f'robot {robot_name!r} model')
            self._elastic_band_body_id = int(body_id)
        self._closed = False

    def _step_simulation(self) -> None:
        """Apply commands, elastic-band forces, and one physics step."""
        if self._node.command_is_stale():
            self._robot.zero_control()
        if self._elastic_band is not None and self._elastic_band_body_id is not None:
            body = self._elastic_band_body_id
            self._robot.data.xfrc_applied[body, :3] = 0.0
            if self._elastic_band.enable:
                self._robot.data.xfrc_applied[body, :3] = self._elastic_band.advance(
                    self._robot.data.qpos[:3], self._robot.data.qvel[:3])
        self._robot.step()

    def _camera_pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Return the configured camera body pose, if camera follow is active."""
        if self._camera_body is None:
            return None
        return self._robot.body_pose(self._camera_body)

    def _reset_robot(self) -> None:
        self._robot.reset_to_keyframe('home')

    def _add_elastic_band_controls(self) -> None:
        """Add backend-specific band controls when the backend supports them."""

    @abc.abstractmethod
    def run(self) -> None:
        """Run the backend until its window/server is closed."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release backend resources."""

    @abc.abstractmethod
    def is_running(self) -> bool:
        """Return whether the backend is still active."""

    @abc.abstractmethod
    def _step(self, model=None, data=None) -> None:
        """Advance one simulation tick for the backend callback."""

    @abc.abstractmethod
    def _render(self, scene=None) -> None:
        """Render one frame and update the follow camera."""

    @abc.abstractmethod
    def _reset(self, model=None, data=None) -> None:
        """Reset the simulation state for the backend callback."""

    @abc.abstractmethod
    def _update_camera(self, scene=None) -> None:
        """Update the backend camera using the shared follow settings."""


class NativeViewer(SimulatorViewer):
    """Passive GLFW viewer using the application's fixed-step loop."""

    def __init__(self, robot: RobotModel, node: RobotMujocoSimNode,
                 elastic_band: ElasticBand | None, follow_camera: bool,
                 stop_event: threading.Event, timestep: float,
                 robot_name: str, follow_yaw: bool = True) -> None:
        super().__init__(robot, node, elastic_band, follow_camera, stop_event,
                         robot_name, follow_yaw)
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ViewerError(
                'No display detected (DISPLAY and WAYLAND_DISPLAY are unset).')
        try:
            import mujoco.viewer
            callback = elastic_band.key_callback if elastic_band is not None else None
            self._viewer = mujoco.viewer.launch_passive(
                robot.model, robot.data, key_callback=callback)
        except Exception as error:
            raise ViewerError(f'Cannot open the MuJoCo viewer: {error}') from error
        self._timestep = timestep

    def run(self) -> None:
        next_step = time.perf_counter()
        next_render = 0.0
        while not self._stop_event.is_set() and self.is_running():
            with self._node.lock:
                self._step()
                if time.perf_counter() >= next_render:
                    next_render = time.perf_counter() + 1.0 / 60.0
                    self._render()
                try:
                    self._viewer.sync()
                except Exception:
                    break
            next_step = _advance_clock(next_step, self._timestep)

    def _step(self, model=None, data=None) -> None:
        del model, data
        if not self._node.is_paused():
            self._step_simulation()

    def _render(self, scene=None) -> None:
        del scene
        if self._follow_camera:
            self._update_camera()

    def _update_camera(self, scene=None) -> None:
        del scene
        if not hasattr(self._viewer, 'cam'):
            return
        pose = self._camera_pose()
        if pose is None:
            return
        position, rotation = pose
        self._viewer.cam.lookat[:] = position
        if self._follow_yaw:
            self._viewer.cam.azimuth = yaw_degrees(rotation)
        self._viewer.cam.elevation = NATIVE_CAMERA_ELEVATION
        self._viewer.cam.distance = NATIVE_CAMERA_DISTANCE

    def _reset(self, model=None, data=None) -> None:
        del model, data
        with self._node.lock:
            self._reset_robot()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._viewer.close()
        except Exception:
            pass

    def is_running(self) -> bool:
        return not self._closed and bool(self._viewer.is_running())


class MjviserViewer(SimulatorViewer):
    """Active viewer adapter for the pip-installed ``mjviser.Viewer``."""

    def __init__(self, robot: RobotModel, node: RobotMujocoSimNode,
                 elastic_band: ElasticBand | None, follow_camera: bool,
                 stop_event: threading.Event, robot_name: str = 'h2',
                 follow_yaw: bool = False) -> None:
        super().__init__(robot, node, elastic_band, follow_camera, stop_event,
                         robot_name, follow_yaw)
        try:
            import mjviser
        except ImportError as error:
            raise ViewerError('viewer="mjviser" requires the pip mjviser package') from error
        viewer_type = getattr(mjviser, 'Viewer', None)
        if viewer_type is None:
            raise ViewerError('Installed mjviser package does not provide Viewer')
        try:
            self._viewer = viewer_type(
                robot.model, robot.data,
                step_fn=self._step,
                render_fn=self._render,
                reset_fn=self._reset,
            )
            # Mjviser tracks its first movable body when this flag is enabled.
            # Set it before ``Viewer.run()`` builds the camera GUI so the
            # checkbox and the initial client camera match --camera behavior.
            self._viewer.scene.camera_tracking_enabled = self._follow_camera
            if self._camera_body is not None:
                # Mjviser normally follows the first movable body. Keep its
                # position tracking consistent with the shared body registry.
                body_id = mujoco.mj_name2id(
                    robot.model, mujoco.mjtObj.mjOBJ_BODY, self._camera_body)
                if hasattr(self._viewer.scene, '_tracked_body_id'):
                    self._viewer.scene._tracked_body_id = int(body_id)
            self._add_elastic_band_controls()
        except Exception as error:
            raise ViewerError(f'Cannot open the Mjviser viewer: {error}') from error

    def _step(self, model=None, data=None) -> None:
        del model, data
        with self._node.lock:
            if not self._node.is_paused():
                self._step_simulation()

    def _render(self, scene=None) -> None:
        if scene is None:
            scene = self._viewer.scene
        with self._node.lock:
            scene.update_from_mjdata(self._robot.data)
            tracking_enabled = getattr(scene, 'camera_tracking_enabled',
                                        self._follow_camera)
            if self._follow_camera and tracking_enabled:
                self._update_camera(scene)

    def _update_camera(self, scene=None) -> None:
        """Keep each connected Viser camera identical to MuJoCo's free view.

        Mjviser keeps the tracked body at the origin by translating the
        rendered scene.  Its built-in camera helper uses a different default
        distance, elevation convention, and azimuth offset, so set the client
        position with MuJoCo's spherical-camera equation directly.
        """
        if scene is None:
            scene = self._viewer.scene
        if self._camera_body is None:
            return
        pose = self._camera_pose()
        if pose is None:
            return
        _, rotation = pose
        clients = scene.server.get_clients().values()
        for client in clients:
            position = np.asarray(client.camera.position, dtype=float)
            look_at = np.asarray(client.camera.look_at, dtype=float)
            if self._follow_yaw:
                azimuth = yaw_degrees(rotation)
            else:
                # With yaw following disabled, preserve the user's current
                # horizontal view while correcting distance/elevation to the
                # native MuJoCo values.
                azimuth = _mujoco_azimuth_from_offset(position - look_at)
            offset = _mujoco_camera_offset(
                azimuth, NATIVE_CAMERA_ELEVATION, NATIVE_CAMERA_DISTANCE)
            client.camera.position = offset
            client.camera.look_at = np.zeros(3)

    def _reset(self, model=None, data=None) -> None:
        del model, data
        with self._node.lock:
            self._reset_robot()

    def _add_elastic_band_controls(self) -> None:
        if self._elastic_band is None:
            return
        try:
            gui = self._viewer.scene.server.gui
            minimum, maximum = self._elastic_band.length_limits
            length = gui.add_slider(
                'Elastic band length', min=minimum, max=maximum, step=0.01,
                initial_value=float(self._elastic_band.length))
            enabled = gui.add_checkbox(
                'Elastic band enabled', initial_value=self._elastic_band.enable)

            @length.on_update
            def _(_) -> None:
                with self._node.lock:
                    self._elastic_band.length = float(length.value)

            @enabled.on_update
            def _(_) -> None:
                with self._node.lock:
                    self._elastic_band.enable = bool(enabled.value)
        except AttributeError as error:
            raise ViewerError('Installed mjviser Viewer does not expose its GUI') from error

    def run(self) -> None:
        try:
            self._viewer.run()
        finally:
            self._closed = True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        server = getattr(self._viewer, '_server', None)
        if server is not None:
            try:
                server.stop()
            except Exception:
                pass

    def is_running(self) -> bool:
        return not self._closed

def _advance_clock(next_step: float, timestep: float) -> float:
    next_step += timestep
    remaining = next_step - time.perf_counter()
    if remaining > 0.0:
        time.sleep(remaining)
        return next_step
    return time.perf_counter()


def create_viewer(backend: str, *, robot: RobotModel, node: RobotMujocoSimNode,
                  elastic_band: ElasticBand | None, follow_camera: bool,
                  stop_event: threading.Event, timestep: float,
                  robot_name: str, follow_yaw: bool = False) -> SimulatorViewer:
    """Create the configured viewer backend behind one stable interface."""
    if backend == 'native':
        return NativeViewer(robot, node, elastic_band, follow_camera, stop_event,
                            timestep, robot_name, follow_yaw)
    if backend == 'mjviser':
        return MjviserViewer(robot, node, elastic_band, follow_camera, stop_event,
                             robot_name, follow_yaw)
    raise ViewerError(f'Unsupported viewer backend: {backend!r}')
