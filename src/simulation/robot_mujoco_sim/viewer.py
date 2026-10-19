"""Viewer backends for the MuJoCo simulator."""
from __future__ import annotations

import abc
import logging
import math
import os
import queue
import threading
import time
from typing import TYPE_CHECKING

import mujoco
import numpy as np

from .model import ElasticBand, RobotModel

if TYPE_CHECKING:
    from .node import RobotMujocoSimNode


_LOGGER = logging.getLogger(__name__)


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

    def _step(self, model=None, data=None) -> None:
        """Advance one tick while holding the shared simulation lock."""
        del model, data
        with self._node.lock:
            if not self._node.is_paused():
                self._step_simulation()

    def _reset(self, model=None, data=None) -> None:
        """Reset the simulation while holding the shared simulation lock."""
        del model, data
        with self._node.lock:
            self._reset_robot()

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
    def _render(self, scene=None) -> None:
        """Render one frame and update the follow camera."""

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
            self._key_events = queue.SimpleQueue()
            callback = self._key_events.put if elastic_band is not None else None
            self._viewer = mujoco.viewer.launch_passive(
                robot.model, robot.data, key_callback=callback)
        except Exception as error:
            raise ViewerError(f'Cannot open the MuJoCo viewer: {error}') from error
        self._timestep = timestep

    def run(self) -> None:
        next_step = time.perf_counter()
        next_render = 0.0
        while not self._stop_event.is_set() and self.is_running():
            self._step()
            if time.perf_counter() >= next_render:
                next_render = time.perf_counter() + 1.0 / 60.0
                self._render()
                # sync() reads/writes robot.data, including GUI edits/reset.
                # Its internal viewer lock does not protect ROS callbacks.
                try:
                    with self._node.lock:
                        self._viewer.sync()
                except Exception:
                    break
            next_step = _advance_clock(next_step, self._timestep)

    def _step(self, model=None, data=None) -> None:
        del model, data
        # GLFW callbacks may hold the viewer lock. Queue keys there rather
        # than acquiring node.lock in the reverse order of sync().
        with self._node.lock:
            while not self._key_events.empty():
                self._elastic_band.key_callback(self._key_events.get_nowait())
        super()._step()

    def _render(self, scene=None) -> None:
        del scene
        if not self._follow_camera or not hasattr(self._viewer, 'cam'):
            return
        self._update_camera()

    def _update_camera(self, scene=None) -> None:
        del scene
        # Copy under the physics lock before acquiring the viewer lock.
        with self._node.lock:
            pose = self._camera_pose()
        if pose is None:
            return
        position, rotation = pose
        with self._viewer.lock():
            cam = self._viewer.cam
            cam.lookat[:] = position
            if self._follow_yaw:
                cam.azimuth = yaw_degrees(rotation)
                cam.elevation = NATIVE_CAMERA_ELEVATION
                cam.distance = NATIVE_CAMERA_DISTANCE

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

    _SCENE_ARRAYS = (
        ('body_xpos', 'xpos'),
        ('body_xmat', 'xmat'),
        ('mocap_pos', 'mocap_pos'),
        ('mocap_quat', 'mocap_quat'),
        ('qpos', 'qpos'),
        ('qvel', 'qvel'),
        ('ctrl', 'ctrl'),
    )

    def __init__(self, robot: RobotModel, node: RobotMujocoSimNode,
                 elastic_band: ElasticBand | None, follow_camera: bool,
                 stop_event: threading.Event, robot_name: str = 'h2',
                 follow_yaw: bool = False) -> None:
        super().__init__(robot, node, elastic_band, follow_camera, stop_event,
                         robot_name, follow_yaw)
        try:
            from ._mjviser import SynchronizedViewer
        except ImportError as error:
            raise ViewerError('viewer="mjviser" requires the pip mjviser package') from error
        self._camera_state_lock = threading.Lock()
        self._camera_update_pending = False
        self._pending_azimuth: float | None = None
        try:
            self._viewer = SynchronizedViewer(
                robot.model, robot.data,
                step_fn=self._step,
                render_fn=self._render,
                reset_fn=self._reset,
                simulation_lock=node.lock,
                camera_gui=self._add_camera_controls,
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
            # All camera writers, including GUI/connect callbacks below, run
            # on the same event loop as incoming browser camera messages.
            self._camera_loop = self._viewer.scene.server.get_event_loop()
            self._add_elastic_band_controls()
        except Exception as error:
            raise ViewerError(f'Cannot open the Mjviser viewer: {error}') from error

    def _render(self, scene=None) -> None:
        if scene is None:
            scene = self._viewer.scene
        snapshot, azimuth = self._scene_snapshot()
        # update_from_arrays() runs mj_forward() on scene.mj_data before
        # taking its own lock. Cover that work as well as GUI scene rebuilds.
        # Never acquire node.lock while holding the scene lock.
        with self._viewer.scene_update_lock():
            scene.update_from_arrays(**snapshot)
        if azimuth is not None:
            self._queue_camera_update(scene, azimuth)

    def _scene_snapshot(self) -> tuple[dict[str, np.ndarray], float | None]:
        """Copy one consistent physics frame for Mjviser and camera follow."""
        with self._node.lock:
            data = self._robot.data
            snapshot = {
                target: np.array(getattr(data, source), copy=True)[None, ...]
                for target, source in self._SCENE_ARRAYS
            }
            snapshot['body_xmat'] = snapshot['body_xmat'].reshape(1, -1, 3, 3)
            pose = self._camera_pose() if self._follow_yaw else None
        return snapshot, None if pose is None else yaw_degrees(pose[1])

    def _update_camera(self, scene=None) -> None:
        """Queue a camera update using the current robot yaw."""
        if scene is None:
            scene = self._viewer.scene
        if self._camera_body is None or not self._follow_yaw:
            return
        with self._node.lock:
            pose = self._camera_pose()
        if pose is None:
            return
        self._queue_camera_update(scene, yaw_degrees(pose[1]))

    def _queue_camera_update(self, scene, azimuth: float) -> None:
        """Coalesce a yaw update and dispatch it to Viser's event loop."""
        loop = self._camera_loop
        if self._closed or loop.is_closed():
            return
        with self._camera_state_lock:
            self._pending_azimuth = azimuth
            if self._camera_update_pending:
                return
            self._camera_update_pending = True
        try:
            loop.call_soon_threadsafe(self._apply_camera_update, scene)
        except RuntimeError:
            with self._camera_state_lock:
                self._camera_update_pending = False

    def _apply_camera_update(self, scene) -> None:
        with self._camera_state_lock:
            azimuth = self._pending_azimuth
            self._camera_update_pending = False
        # Recheck after dispatch: the user may have disabled tracking while
        # this callback was queued. With yaw disabled, scene translation alone
        # follows the robot and preserves the user's camera orientation.
        if self._closed or not scene.camera_tracking_enabled or azimuth is None:
            return
        offset = _mujoco_camera_offset(
            azimuth, NATIVE_CAMERA_ELEVATION, NATIVE_CAMERA_DISTANCE)
        for client in scene.server.get_clients().values():
            self._set_client_camera(client, offset, np.zeros(3))

    @staticmethod
    def _set_client_camera(client, position, look_at, *, fov=None) -> None:
        """Called only on Viser's event loop; atomic() batches messages."""
        try:
            with client.atomic():
                if fov is not None:
                    client.camera.fov = fov
                client.camera.position = position
                client.camera.look_at = look_at
        except (ValueError, RuntimeError) as error:
            _LOGGER.warning('Cannot update camera for client %s: %s',
                            client.client_id, error)

    def _add_camera_controls(self) -> None:
        """Replace upstream worker-thread camera writers with async callbacks."""
        scene = self._viewer.scene
        server = scene.server
        center = np.array(self._robot.model.stat.center, copy=True)
        # Match the shared MuJoCo framing from the first connected frame.
        offset = _mujoco_camera_offset(
            120.0, NATIVE_CAMERA_ELEVATION, NATIVE_CAMERA_DISTANCE)
        with server.gui.add_folder('Camera'):
            tracking = server.gui.add_checkbox(
                'Track camera', initial_value=scene.camera_tracking_enabled)
            fov = server.gui.add_slider(
                'FOV (°)', min=20, max=150, step=1, initial_value=60)

        @tracking.on_update
        async def _(_) -> None:
            with self._viewer.scene_update_lock():
                scene.camera_tracking_enabled = bool(tracking.value)
                scene.needs_update = True
            if tracking.value:
                for client in server.get_clients().values():
                    self._set_client_camera(client, offset, np.zeros(3))

        @fov.on_update
        async def _(_) -> None:
            for client in server.get_clients().values():
                client.camera.fov = math.radians(fov.value)

        @server.on_client_connect
        async def _(client) -> None:
            target = np.zeros(3) if scene.camera_tracking_enabled else center
            self._set_client_camera(client, offset + target, target,
                                    fov=math.radians(fov.value))

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
