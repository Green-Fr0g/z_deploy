"""Adam's model, deployment order and simulator interfaces must agree."""
import pathlib
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from robot_mujoco_sim.config import _robot_scene_path, load_config
from robot_mujoco_sim.model import RobotModel
from robot_mujoco_sim.viewer import camera_follow_body, elastic_band_body


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]


@pytest.fixture
def adam():
    config = load_config(robot='adam', scene='empty.xml')
    return RobotModel(config.model_path, config.timestep)


def test_adam_scene_uses_fixed_model(adam):
    config = load_config(robot='adam', scene='empty.xml')
    scene = ET.parse(config.model_path).getroot()
    assert scene.find('include').get('file') == 'robot_fix.xml'
    assert mujoco.mj_name2id(adam.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor') >= 0
    assert adam.model.nq == 34  # Free root plus 27 hinge joints.
    assert camera_follow_body('adam') == elastic_band_body('adam') == 'torso_link'
    assert adam.body_pose('torso_link') is not None


def test_adam_joint_order_and_home_pose(adam):
    with (REPO_ROOT / 'policy/adam/base.yaml').open() as stream:
        base = yaml.safe_load(stream)['adam']
    with (REPO_ROOT / 'policy/adam/locomotion/config.yaml').open() as stream:
        policy = yaml.safe_load(stream)['adam/locomotion']
    assert adam.num_actuators == base['num_of_dofs'] == policy['num_of_dofs'] == 27
    assert adam.actuator_names == base['joint_names']
    expected_names = [base['joint_names'][i] for i in policy['joint_mapping']]
    assert expected_names == adam.actuator_names
    assert sum(policy['observations_dims_list']) * len(policy['observations_history']) == 450
    states = np.asarray(list(adam.read_joint_states()))
    np.testing.assert_allclose(states[:, 0], base['default_dof_pos'])
    np.testing.assert_allclose(states[:, 1:], 0)
    np.testing.assert_allclose(policy['default_dof_pos'], base['default_dof_pos'])
    np.testing.assert_allclose(policy['rl_kp'], base['fixed_kp'])
    np.testing.assert_allclose(policy['rl_kd'], base['fixed_kd'])
    np.testing.assert_allclose(adam.model.actuator_ctrlrange[:, 1], base['torque_limits'])

    for side in ('left', 'right'):
        geom = mujoco.mj_name2id(adam.model, mujoco.mjtObj.mjOBJ_GEOM,
                               f'{side}_ankle_roll_link_collision_1')
        rotation = adam.data.geom_xmat[geom].reshape(3, 3)
        sole = adam.data.geom_xpos[geom, 2] - np.dot(
            np.abs(rotation[2]), adam.model.geom_size[geom])
        assert 0 < sole < 0.02

    adam.data.qpos[:] = 0
    adam.reset_to_keyframe('home')
    np.testing.assert_allclose(np.asarray(list(adam.read_joint_states()))[:, 0],
                               base['default_dof_pos'])


def test_adam_sensors_and_commands_follow_actuators(adam):
    positions = np.linspace(-0.04, 0.04, 27)
    velocities = np.linspace(-0.2, 0.2, 27)
    for i, joint_id in enumerate(adam.joint_ids):
        adam.data.qpos[adam.model.jnt_qposadr[joint_id]] = positions[i]
        adam.data.qvel[adam.dof_ids[i]] = velocities[i]
    mujoco.mj_forward(adam.model, adam.data)
    states = np.asarray(list(adam.read_joint_states()))
    np.testing.assert_allclose(states[:, 0], positions)
    np.testing.assert_allclose(states[:, 1], velocities)

    commands = [SimpleNamespace(torque=0.1 * (i + 1), position=positions[i] + 0.01,
                                velocity=velocities[i] + 0.02, kp=20.0, kd=2.0)
                for i in range(27)]
    adam.apply_joint_commands(commands)
    expected_torque = np.asarray([c.torque for c in commands]) + 0.24
    np.testing.assert_allclose(adam.data.ctrl, expected_torque)
    mujoco.mj_forward(adam.model, adam.data)
    np.testing.assert_allclose(np.asarray(list(adam.read_joint_states()))[:, 2],
                               expected_torque)
    quaternion, gyro, acceleration = adam.read_imu()
    assert quaternion.shape == (4,) and gyro.shape == acceleration.shape == (3,)
    assert np.isfinite(np.concatenate((quaternion, gyro, acceleration))).all()


def test_scene_resolution_preserves_mjcf_precedence(tmp_path):
    config = tmp_path / 'config/sim.yaml'
    model = tmp_path / 'robots_model/adam_mjcf'
    (model / 'mjcf').mkdir(parents=True)
    root_scene = model / 'empty.xml'
    root_scene.touch()
    assert _robot_scene_path(config, 'adam', 'empty') == root_scene
    nested_scene = model / 'mjcf/empty.xml'
    nested_scene.touch()
    assert _robot_scene_path(config, 'adam', 'empty') == nested_scene


def test_pd_updates_between_ros_commands_and_clears_on_stop(adam):
    commands = [SimpleNamespace(torque=0.0, position=0.0, velocity=0.0, kp=1.0, kd=0.0)
                for _ in range(27)]
    adam.apply_joint_commands(commands)
    commands[0].position = 100.0  # The callback's mutable message is not retained.
    joint = adam.joint_ids[0]
    adam.data.qpos[adam.model.jnt_qposadr[joint]] = 0.2
    mujoco.mj_forward(adam.model, adam.data)
    adam.step()
    assert adam.data.ctrl[0] == pytest.approx(-0.2)
    adam.zero_control()
    adam.step()
    np.testing.assert_allclose(adam.data.ctrl, 0)
    adam.apply_joint_commands(commands)
    adam.reset_to_keyframe('home')
    adam.step()
    np.testing.assert_allclose(adam.data.ctrl, 0)


def test_adam_pd_standing_has_no_fast_wrist_oscillation(adam):
    with (REPO_ROOT / 'policy/adam/base.yaml').open() as stream:
        base = yaml.safe_load(stream)['adam']
    commands = [SimpleNamespace(torque=0.0, position=position, velocity=0.0, kp=kp, kd=kd)
                for position, kp, kd in zip(base['default_dof_pos'], base['fixed_kp'],
                                             base['fixed_kd'])]
    adam.apply_joint_commands(commands)
    for _ in range(int(0.5 / adam.model.opt.timestep)):
        adam.step()
        assert np.isfinite(adam.data.qpos).all()
        assert np.max(np.abs(adam.data.qvel)) < 10
    assert adam.data.qpos[2] > 1.1
    assert not adam.data.warning.number.any()
