"""Configuration loading and robot/scene path resolution for the simulator."""
from __future__ import annotations

import dataclasses
import pathlib

import yaml
try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # Allow config inspection outside a ROS environment.
    get_package_share_directory = None

# This name is part of the model/ROS contract, rather than runtime
# configuration. Keep it here so callers do not duplicate string literals.


@dataclasses.dataclass(frozen=True)
class SimConfig:
    """Validated simulator configuration loaded from ``sim.yaml``."""

    model_path: pathlib.Path
    robot: str = 'h2'
    scene: str = 'empty'
    #Frequency
    sim_frequency: float = 500.0
    state_publish_frequency: float = 200.0
    imu_publish_frequency: float = 200.0
    #topic names
    command_topic: str = '/command'
    state_topic: str = '/state'
    imu_topic: str = '/imu'
    reset_service: str = '/reset'
    pause_service: str = '/pause'
    resume_service: str = '/resume'
    command_timeout: float = 0.25
    print_scene_information: bool = False
    enable_elastic_band: bool = False
    enable_camera_follow: bool = False
    follow_yaw: bool = False
    viewer: str = 'native'

    @property
    def timestep(self) -> float:
        """Physics timestep derived from the configured simulation frequency."""
        return 1.0 / self.sim_frequency


def default_config_path() -> pathlib.Path:
    """Return the shipped ``config/sim.yaml``.

    Checked in order:
    1. next to the Python package (working tree, or ``--symlink-install``),
    2. the package share directory (regular ``colcon build`` installs
       ``data_files`` there).
    """
    candidates = [pathlib.Path(__file__).resolve().parent.parent / 'config' / 'sim.yaml']
    if get_package_share_directory is not None:
        try:
            candidates.append(pathlib.Path(get_package_share_directory('robot_mujoco_sim'))
                             / 'config' / 'sim.yaml')
        except Exception:
            pass
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        'Cannot find simulator configuration; looked in: '
        + ', '.join(str(candidate) for candidate in candidates))

def _robot_scene_path(config_path: pathlib.Path, robot: str, scene: str) -> pathlib.Path:
    """Resolve scenes in either the model's ``mjcf/`` directory or its root."""
    robot_name = pathlib.Path(robot).name
    scene_path = pathlib.Path(scene).expanduser()
    if scene_path.suffix == '':
        scene_path = scene_path.with_suffix('.xml')
    if scene_path.is_absolute():
        return scene_path
    roots = (
        config_path.parent.parent / 'robots_model',
        config_path.parent.parent.parent / 'robots_model',
        pathlib.Path.cwd() / 'src' / 'simulation' / 'robots_model',
    )
    for root in roots:
        model_root = root / f'{robot_name}_mjcf'
        for directory in (model_root / 'mjcf', model_root):
            candidate = (directory / scene_path).resolve()
            if candidate.is_file():
                return candidate
    return (roots[0] / f'{robot_name}_mjcf' / 'mjcf' / scene_path).resolve()


def load_config(path: pathlib.Path | None = None,
                robot: str | None = None,
                scene: str | None = None,
                viewer: str | None = None) -> SimConfig:
    """Read, coerce and validate the simulator configuration.

    ``robot`` and ``scene`` override the YAML selection.
    """
    config_path = pathlib.Path(path or default_config_path()).expanduser().resolve()
    with config_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    section = document
    if not isinstance(section, dict):
        raise ValueError(f'Invalid configuration in {config_path}')

    configured_robot = str(section.get('robot', 'h2')).strip()
    configured_scene = str(section.get('scene', 'empty')).strip()
    selected_robot = str(robot or configured_robot).strip()
    selected_scene = str(scene or configured_scene).strip()
    if not selected_robot:
        raise ValueError('robot must not be empty')
    if not selected_scene:
        raise ValueError('scene must not be empty')
    if pathlib.Path(selected_scene).suffix == '':
        selected_scene += '.xml'
    resolved_scene = _robot_scene_path(config_path, selected_robot, selected_scene)

    config = SimConfig(
        model_path=resolved_scene,
        robot=selected_robot,
        scene=selected_scene,
        sim_frequency=float(section.get('sim_frequency', 500.0)),
        state_publish_frequency=float(section.get('state_publish_frequency', 200.0)),
        imu_publish_frequency=float(section.get('imu_publish_frequency', 200.0)),
        command_timeout=float(section.get('command_timeout', 0.25)),

        command_topic=str(selected_robot+section.get('command_topic', '/command')),
        state_topic=str(selected_robot+section.get('state_topic', '/state')),
        imu_topic=str(selected_robot+section.get('imu_topic', '/imu')),
        reset_service=str(selected_robot+section.get('reset_service', '/reset')),
        pause_service=str(selected_robot+section.get('pause_service', '/pause')),
        resume_service=str(selected_robot+section.get('resume_service', '/resume')),
        
        print_scene_information=bool(section.get('print_scene_information', 0)),
        enable_elastic_band=bool(section.get('enable_elastic_band', 0)),
        enable_camera_follow=bool(section.get('enable_camera_follow', 0)),
        follow_yaw=bool(section.get('follow_yaw', 0)),
        viewer=str(viewer or section.get('viewer', 'native')).strip().lower(),
    )
    _validate(config, config_path)
    return config


def _validate(config: SimConfig, config_path: pathlib.Path) -> None:
    """Reject configurations that would make the loop behave unpredictably."""
    if config.sim_frequency <= 0.0:
        raise ValueError('sim_frequency must be greater than zero')
    if config.state_publish_frequency <= 0.0:
        raise ValueError('state_publish_frequency must be greater than zero')
    if config.imu_publish_frequency <= 0.0:
        raise ValueError('imu_publish_frequency must be greater than zero')
    if not config.model_path.is_file():
        raise FileNotFoundError(f'Model file not found: {config.model_path} '
                                f'(referenced by {config_path})')
    if config.viewer not in {'native', 'mjviser'}:
        raise ValueError('viewer must be either "native" or "mjviser"')


