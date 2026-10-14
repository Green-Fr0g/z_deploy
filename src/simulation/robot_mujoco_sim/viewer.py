"""Viewer backends for the MuJoCo simulator."""
from __future__ import annotations

import abc
import math
import os
import threading
import time
from typing import TYPE_CHECKING

import numpy as np

from .model import ElasticBand, RobotModel

if TYPE_CHECKING:
    from .node import RobotMujocoSimNode


class ViewerError(RuntimeError):
    """Raised when a configured viewer cannot be started."""


def yaw_degrees(rotation: np.ndarray) -> float:
    """Return the rotation around the world Z axis of a rotation matrix."""
    return math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))


class FollowCamera:
    """Keeps a native viewer camera behind and above a tracked body."""

    def __init__(self, body: str = 'trunk', distance: float = 1.6,
                 elevation: float = -18.0) -> None:
        self._body = body
        self._distance = float(distance)
        self._elevation = float(elevation)

    def update(self, viewer, robot: RobotModel) -> None:
        if not hasattr(viewer, 'cam'):
            return
        pose = robot.body_pose(self._body)
        if pose is None:
            return
        position, rotation = pose
        viewer.cam.lookat[:] = position
        viewer.cam.azimuth = yaw_degrees(rotation)
        viewer.cam.elevation = self._elevation
        viewer.cam.distance = self._distance


class SimulatorViewer(abc.ABC):
    """Common lifecycle for native and web-based viewer backends."""

    @abc.abstractmethod
    def run(self) -> None:
        """Run the backend until its window/server is closed."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release backend resources."""

    @abc.abstractmethod
    def is_running(self) -> bool:
        """Return whether the backend is still active."""


class NativeViewer(SimulatorViewer):
    """Passive GLFW viewer using the application's fixed-step loop."""

    def __init__(self, robot: RobotModel, node: RobotMujocoSimNode,
                 elastic_band: ElasticBand | None, follow_camera: bool,
                 stop_event: threading.Event, timestep: float) -> None:
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
        self._robot = robot
        self._node = node
        self._elastic_band = elastic_band
        self._camera = FollowCamera() if follow_camera else None
        self._stop_event = stop_event
        self._timestep = timestep
        self._closed = False

    def run(self) -> None:
        next_step = time.perf_counter()
        next_render = 0.0
        while not self._stop_event.is_set() and self.is_running():
            with self._node.lock:
                if not self._node.is_paused():
                    _step_robot(self._robot, self._node, self._elastic_band)
                if self._camera is not None and time.perf_counter() >= next_render:
                    next_render = time.perf_counter() + 1.0 / 60.0
                    self._camera.update(self._viewer, self._robot)
                try:
                    self._viewer.sync()
                except Exception:
                    break
            next_step = _advance_clock(next_step, self._timestep)

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
                 stop_event: threading.Event) -> None:
        del stop_event  # Mjviser owns the web viewer loop.
        try:
            import mjviser
        except ImportError as error:
            raise ViewerError('viewer="mjviser" requires the pip mjviser package') from error
        viewer_type = getattr(mjviser, 'Viewer', None)
        if viewer_type is None:
            raise ViewerError('Installed mjviser package does not provide Viewer')
        self._robot = robot
        self._node = node
        self._elastic_band = elastic_band
        self._closed = False
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
            self._viewer.scene.camera_tracking_enabled = bool(follow_camera)
            self._add_elastic_band_controls()
        except Exception as error:
            raise ViewerError(f'Cannot open the Mjviser viewer: {error}') from error

    def _step(self, model, data) -> None:
        del model, data
        with self._node.lock:
            if not self._node.is_paused():
                _step_robot(self._robot, self._node, self._elastic_band)

    def _render(self, scene) -> None:
        with self._node.lock:
            scene.update_from_mjdata(self._robot.data)

    def _reset(self, model, data) -> None:
        del model, data
        with self._node.lock:
            self._robot.reset_to_keyframe('home')

    def _add_elastic_band_controls(self) -> None:
        if self._elastic_band is None:
            return
        try:
            gui = self._viewer.scene.server.gui
            length = gui.add_slider(
                'Elastic band length', min=0.0, max=5.0, step=0.01,
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


def _step_robot(robot: RobotModel, node: RobotMujocoSimNode,
                elastic_band: ElasticBand | None) -> None:
    """Apply shared control logic and advance one physics step."""
    if node.command_is_stale():
        robot.zero_control()
    if elastic_band is not None:
        body = robot.elastic_band_body_id()
        robot.data.xfrc_applied[body, :3] = 0.0
        if elastic_band.enable:
            robot.data.xfrc_applied[body, :3] = elastic_band.advance(
                robot.data.qpos[:3], robot.data.qvel[:3])
    robot.step()


def _advance_clock(next_step: float, timestep: float) -> float:
    next_step += timestep
    remaining = next_step - time.perf_counter()
    if remaining > 0.0:
        time.sleep(remaining)
        return next_step
    return time.perf_counter()


def create_viewer(backend: str, *, robot: RobotModel, node: RobotMujocoSimNode,
                  elastic_band: ElasticBand | None, follow_camera: bool,
                  stop_event: threading.Event, timestep: float) -> SimulatorViewer:
    """Create the configured viewer backend behind one stable interface."""
    if backend == 'native':
        return NativeViewer(robot, node, elastic_band, follow_camera, stop_event, timestep)
    if backend == 'mjviser':
        return MjviserViewer(robot, node, elastic_band, follow_camera, stop_event)
    raise ViewerError(f'Unsupported viewer backend: {backend!r}')
