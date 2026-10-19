"""Regression tests against the pinned MuJoCo/Mjviser/Viser implementations."""
import asyncio
import contextlib
import inspect
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest
import viser

from robot_mujoco_sim.model import RobotModel
from robot_mujoco_sim.viewer import (
    MjviserViewer, NativeViewer, _mujoco_camera_offset,
    NATIVE_CAMERA_DISTANCE, NATIVE_CAMERA_ELEVATION,
)


@pytest.fixture
def backend(monkeypatch):
    model = mujoco.MjModel.from_xml_string('''
        <mujoco>
          <worldbody><body name="torso_link" pos="0 0 1">
            <joint name="joint" type="slide"/>
            <geom type="sphere" size=".1" mass="1"/>
          </body></worldbody>
          <actuator><motor name="motor" joint="joint"/></actuator>
          <keyframe><key name="home"/></keyframe>
        </mujoco>
    ''')
    robot = RobotModel.__new__(RobotModel)
    robot.model = model
    robot.data = mujoco.MjData(model)
    mujoco.mj_forward(model, robot.data)
    node = SimpleNamespace(lock=threading.RLock(), is_paused=lambda: False,
                           command_is_stale=lambda: False)
    server_type = viser.ViserServer
    monkeypatch.setattr(viser, 'ViserServer', lambda: server_type(
        host='127.0.0.1', port=0, verbose=False))
    viewer = MjviserViewer(robot, node, None, True, threading.Event(),
                          follow_yaw=True)
    viewer._viewer._setup_gui()
    try:
        yield viewer
    finally:
        viewer.close()


def on_loop(loop, callback):
    result = Future()

    def run():
        try:
            result.set_result(callback())
        except BaseException as error:
            result.set_exception(error)

    loop.call_soon_threadsafe(run)
    return result.result(timeout=5)


def gui_handle(backend, label):
    handles = backend._viewer.scene.server.gui._gui_input_handle_from_uuid.values()
    return next(handle for handle in handles if handle.label == label)


def test_native_sync_excludes_ros_access():
    viewer = NativeViewer.__new__(NativeViewer)
    viewer._node = SimpleNamespace(lock=threading.RLock())
    viewer._stop_event = threading.Event()
    viewer._closed = False
    viewer._timestep = 0.002
    viewer._step = Mock()
    viewer._render = Mock()
    attempted, acquired = threading.Event(), threading.Event()

    def ros_callback():
        attempted.set()
        with viewer._node.lock:
            acquired.set()

    worker = threading.Thread(target=ros_callback, daemon=True)

    def sync():
        assert viewer._node.lock._is_owned()
        worker.start()
        assert attempted.wait(2)
        assert not acquired.wait(0.05)
        viewer._stop_event.set()

    viewer._viewer = SimpleNamespace(is_running=lambda: True, sync=sync)
    viewer.run()
    worker.join(timeout=2)
    assert acquired.is_set()


def test_native_keyboard_is_applied_under_physics_lock(backend, monkeypatch):
    import mujoco.viewer

    launch = Mock()
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch)
    monkeypatch.setenv('DISPLAY', ':test')
    band = SimpleNamespace(key_callback=Mock())
    backend._node.is_paused = lambda: True
    viewer = NativeViewer(backend._robot, backend._node, band, False,
                          threading.Event(), 0.002, 'h2')
    callback = launch.call_args.kwargs['key_callback']
    with viewer._node.lock:
        # A GLFW producer must finish even while the simulation lock is held.
        with ThreadPoolExecutor(max_workers=1) as executor:
            executor.submit(callback, 7).result(timeout=2)
        viewer._elastic_band.key_callback.assert_not_called()
        viewer._elastic_band.key_callback.side_effect = (
            lambda key: pytest.fail('Keyboard mutation without physics lock')
            if not viewer._node.lock._is_owned() else None)
        viewer._step()
    viewer._elastic_band.key_callback.assert_called_once_with(7)


def test_scene_forward_is_serialized_between_render_and_gui(backend, monkeypatch):
    scene = backend._viewer.scene
    scene.show_contact_points = True
    first_forward, second_snapshot = threading.Event(), threading.Event()
    second_forward, release = threading.Event(), threading.Event()
    original_forward = mujoco.mj_forward
    original_pose = backend._robot.body_pose

    def pose(name):
        if first_forward.is_set():
            second_snapshot.set()
        return original_pose(name)

    def forward(model, data):
        if first_forward.is_set():
            second_forward.set()
        else:
            first_forward.set()
            assert release.wait(3)
        original_forward(model, data)

    monkeypatch.setattr(backend._robot, 'body_pose', pose)
    monkeypatch.setattr(mujoco, 'mj_forward', forward)
    with ThreadPoolExecutor(max_workers=2) as executor:
        first = executor.submit(backend._render)
        assert first_forward.wait(2)
        # GUI refresh invokes the adapter under Mjviser's simulation lock.
        second = executor.submit(backend._viewer._refresh_scene_from_gui)
        try:
            assert second_snapshot.wait(2)
            assert not second_forward.wait(0.05)
        finally:
            release.set()
        first.result(timeout=3)
        second.result(timeout=3)
    assert second_forward.is_set()


def test_upload_uses_snapshot_and_allows_ros_writes(backend, monkeypatch):
    uploading, release = threading.Event(), threading.Event()
    captured = {}

    def upload(**arrays):
        captured.update(arrays)
        uploading.set()
        assert release.wait(3)

    monkeypatch.setattr(backend._viewer.scene, 'update_from_arrays', upload)
    with ThreadPoolExecutor(max_workers=1) as executor:
        rendered = executor.submit(backend._render)
        try:
            assert uploading.wait(2)
            assert backend._node.lock.acquire(timeout=1)
            try:
                backend._robot.data.qpos[:] = 2
                backend._robot.data.ctrl[:] = 3
            finally:
                backend._node.lock.release()
            np.testing.assert_array_equal(captured['qpos'], [[0]])
            np.testing.assert_array_equal(captured['ctrl'], [[0]])
            for name in ('qpos', 'qvel', 'ctrl', 'mocap_pos', 'mocap_quat'):
                assert not np.shares_memory(captured[name], getattr(backend._robot.data, name))
        finally:
            release.set()
        rendered.result(timeout=3)


def test_camera_gui_connect_and_follow_use_event_loop(backend):
    server = backend._viewer.scene.server
    loop = backend._camera_loop
    loop_thread = on_loop(loop, threading.get_ident)
    writes = []

    class Camera:
        def __setattr__(self, name, value):
            assert threading.get_ident() == loop_thread
            writes.append(name)
            object.__setattr__(self, name, value)

    client = SimpleNamespace(client_id=1, camera=Camera(),
                             atomic=contextlib.nullcontext)
    with server._client_lock:
        server._connected_clients[1] = client
    for callback in server._client_connect_cb:
        assert inspect.iscoroutinefunction(callback)
        asyncio.run_coroutine_threadsafe(callback(client), loop).result(timeout=3)
    for label in ('Track camera', 'FOV (°)'):
        for callback in gui_handle(backend, label)._impl.update_cb:
            assert inspect.iscoroutinefunction(callback)
            asyncio.run_coroutine_threadsafe(callback(None), loop).result(timeout=3)
    backend._queue_camera_update(backend._viewer.scene, 45)
    on_loop(loop, lambda: None)
    np.testing.assert_allclose(client.camera.position, _mujoco_camera_offset(
        45, NATIVE_CAMERA_ELEVATION, NATIVE_CAMERA_DISTANCE))
    assert {'position', 'look_at', 'fov'} <= set(writes)


def test_camera_failure_does_not_skip_other_clients(backend, monkeypatch, caplog):
    class InvalidCamera:
        @property
        def position(self):
            return None

        @position.setter
        def position(self, value):
            raise ValueError('Invalid camera pose')

    bad = SimpleNamespace(client_id=1, camera=InvalidCamera(),
                          atomic=contextlib.nullcontext)
    good = SimpleNamespace(client_id=2, camera=SimpleNamespace(),
                           atomic=contextlib.nullcontext)
    monkeypatch.setattr(backend._viewer.scene.server, 'get_clients',
                        lambda: {1: bad, 2: good})
    backend._queue_camera_update(backend._viewer.scene, 30)
    on_loop(backend._camera_loop, lambda: None)
    np.testing.assert_allclose(good.camera.position, _mujoco_camera_offset(
        30, NATIVE_CAMERA_ELEVATION, NATIVE_CAMERA_DISTANCE))
    assert 'Invalid camera pose' in caplog.text
    assert not backend._camera_update_pending


def test_yaw_disabled_preserves_user_camera_while_scene_tracks(backend, monkeypatch):
    backend._follow_yaw = False
    position = np.array([3., 4., 2.])
    look_at = np.array([0.1, 0.2, 0.3])
    camera = SimpleNamespace(position=position.copy(), look_at=look_at.copy())
    client = SimpleNamespace(camera=camera, atomic=contextlib.nullcontext)
    scene = backend._viewer.scene
    monkeypatch.setattr(scene.server, 'get_clients', lambda: {1: client})
    backend._render()
    backend._update_camera(scene)
    on_loop(backend._camera_loop, lambda: None)
    np.testing.assert_array_equal(camera.position, position)
    np.testing.assert_array_equal(camera.look_at, look_at)
    np.testing.assert_allclose(scene._scene_offset,
                               -backend._robot.body_pose('torso_link')[0])


def test_queued_follow_uses_latest_yaw_and_rechecks_tracking(backend):
    scene = backend._viewer.scene
    queued = []
    backend._camera_loop = SimpleNamespace(
        is_closed=lambda: False,
        call_soon_threadsafe=lambda *args: queued.append(args))
    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(lambda yaw: backend._queue_camera_update(scene, yaw), range(30)))
    backend._queue_camera_update(scene, 90)
    assert len(queued) == 1
    assert backend._pending_azimuth == 90
    scene.camera_tracking_enabled = False
    scene.server.get_clients = Mock(side_effect=AssertionError('Tracking is disabled'))
    callback, *args = queued.pop()
    callback(*args)
    assert not backend._camera_update_pending


def test_builtin_joint_editor_waits_for_ros_lock(backend, monkeypatch):
    started = threading.Event()
    monkeypatch.setattr(backend._viewer, '_render', lambda: None)

    def edit():
        started.set()
        backend._viewer._set_joint_qpos(0, 0.25)

    with ThreadPoolExecutor(max_workers=1) as executor:
        with backend._node.lock:
            edited = executor.submit(edit)
            assert started.wait(2)
            np.testing.assert_array_equal(backend._robot.data.qpos, [0])
            assert not edited.done()
        edited.result(timeout=3)
    np.testing.assert_array_equal(backend._robot.data.qpos, [0.25])


def test_physics_flag_waits_for_scene_forward(backend):
    gravity = gui_handle(backend, 'Gravity')
    started = threading.Event()

    def disable_gravity():
        started.set()
        gravity.value = True

    with ThreadPoolExecutor(max_workers=1) as executor:
        with backend._viewer.scene_update_lock():
            updated = executor.submit(disable_gravity)
            assert started.wait(2)
            assert not backend._robot.model.opt.disableflags & int(
                mujoco.mjtDisableBit.mjDSBL_GRAVITY)
            assert not updated.done()
        updated.result(timeout=3)
    assert backend._robot.model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
