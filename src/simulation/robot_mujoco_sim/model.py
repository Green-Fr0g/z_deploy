"""MuJoCo model wrapper for the selected robot."""
from __future__ import annotations

import pathlib
from collections.abc import Iterable, Sequence

import mujoco
import numpy as np

class ElasticBand:
    """Virtual spring used to lift humanoid models while tuning controllers."""

    MIN_LENGTH = 0.2

    def __init__(self) -> None:
        self.stiffness = 200.0
        self.damping = 100.0
        self.point = np.array((0.0, 0.0, 3.0), dtype=float)
        self._length = self.MIN_LENGTH
        self.enable = True

    @property
    def length_limits(self) -> tuple[float, float]:
        """Return the valid band length range in metres.

        The upper limit follows the connection point: a band may be at most
        one metre longer than the point's world-Z coordinate.  Keep the lower
        limit at 0.2 m so both the native key controls and web slider always
        use the same safe range.
        """
        minimum = float(self.MIN_LENGTH)
        maximum = max(minimum, float(self.point[2]) + 1.0)
        return minimum, maximum

    @property
    def length(self) -> float:
        minimum, maximum = self.length_limits
        self._length = float(np.clip(self._length, minimum, maximum))
        return self._length

    @length.setter
    def length(self, value: float) -> None:
        minimum, maximum = self.length_limits
        self._length = float(np.clip(float(value), minimum, maximum))

    def advance(self, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        delta = self.point - position[:3]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            return np.zeros(3)
        direction = delta / distance
        speed = float(np.dot(velocity[:3], direction))
        return (self.stiffness * (distance - self.length) - self.damping * speed) * direction

    def key_callback(self, key: int) -> None:
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable


class RobotModel:
    """Owns the MuJoCo model/data pair and the actuator-to-joint mapping.

    Every raw index lookup lives here so that the ROS node and the simulation
    loop only deal with joint level positions, velocities and torques.
    """

    def __init__(self, model_path: pathlib.Path, timestep: float) -> None:
        if timestep <= 0.0:
            raise ValueError('timestep must be greater than zero')
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        # Actuators are the authoritative joint order used by the ROS topics.
        self.joint_ids = np.asarray(self.model.actuator_trnid[:, 0], dtype=np.int32)
        self.dof_ids = np.asarray(
            [self.model.jnt_dofadr[joint_id] for joint_id in self.joint_ids],
            dtype=np.int32,
        )

        # IMU sensors use a fixed XML order: quaternion, gyro, acceleration.
        self._imu_sensor_addresses = [
            self.sensor_address(name)
            for name in ('imu_quat', 'imu_gyro', 'imu_acc')
        ]

        # Joint state sensors follow the actuator names in the XML. Resolve
        # their addresses once so published state comes from sensordata rather
        # than directly from MuJoCo's generalized state arrays.
        self._joint_sensor_addresses: list[tuple[tuple[int, int], tuple[int, int], tuple[int, int]]] = []
        missing_joint_sensors: list[str] = []
        for actuator_name in self.actuator_names:
            position = self.sensor_address(f'{actuator_name}_pos')
            velocity = self.sensor_address(f'{actuator_name}_vel')
            torque = self.sensor_address(f'{actuator_name}_torque')
            if position is None:
                missing_joint_sensors.append(f'{actuator_name}_pos')
            if velocity is None:
                missing_joint_sensors.append(f'{actuator_name}_vel')
            if torque is None:
                missing_joint_sensors.append(f'{actuator_name}_torque')
            if position is not None and velocity is not None and torque is not None:
                self._joint_sensor_addresses.append((position, velocity, torque))
        if missing_joint_sensors:
            raise ValueError(
                'MuJoCo model is missing joint sensors: ' + ', '.join(missing_joint_sensors))

        self._joint_commands: list[tuple[float, float, float, float, float]] = []

        # Models with an explicit home keyframe (for example DR02's floating
        # base) should start in that safe, collision-free pose as well as use
        # it when the viewer or ROS reset service is invoked.
        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, 'home') >= 0:
            self.reset_to_keyframe('home')

    @property
    def num_actuators(self) -> int:
        return int(self.model.nu)

    @property
    def actuator_names(self) -> list[str]:
        names = (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
                 for index in range(self.num_actuators))
        return [name for name in names]

    def sensor_address(self, name: str) -> tuple[int, int] | None:
        """Return the ``(sensordata offset, dimension)`` of a sensor."""
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            return None
        return int(self.model.sensor_adr[sensor_id]), int(self.model.sensor_dim[sensor_id])

    def _sensor_values(self, address: tuple[int, int]) -> np.ndarray:
        start, dimension = address
        return np.array(self.data.sensordata[start:start + dimension], dtype=float, copy=True)

    def read_imu(self) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Return the latest IMU sample, or ``None`` when unavailable."""
        quaternion, gyro, acceleration = self._imu_sensor_addresses
        assert quaternion is not None and gyro is not None and acceleration is not None
        return (self._sensor_values(quaternion), self._sensor_values(gyro),
                self._sensor_values(acceleration))

    def read_joint_states(self) -> Iterable[tuple[float, float, float]]:
        """Yield ``(position, velocity, torque)`` from joint sensors."""
        for position, velocity, torque in self._joint_sensor_addresses:
            yield (
                float(self._sensor_values(position)[0]),
                float(self._sensor_values(velocity)[0]),
                float(self._sensor_values(torque)[0]),
            )

    def apply_joint_commands(self, commands: Sequence[object]) -> None:
        """Store joint setpoints and apply their current feed-forward/PD torque.

        ``commands`` items must expose ``torque``, ``kp``, ``position``, ``kd``
        and ``velocity``, matching ``z_msgs/msg/JointCmd``.
        The setpoints are held between ROS messages; PD is recomputed at every
        physics step instead of holding a stale torque until the next message.
        """
        self._joint_commands = [
            (float(command.torque), float(command.kp), float(command.position),
             float(command.kd), float(command.velocity))
            for command in commands[:self.num_actuators]
        ]
        self._update_joint_control()

    def _update_joint_control(self) -> None:
        for index, (feedforward, kp, target_position, kd, target_velocity) in enumerate(
                self._joint_commands):
            torque = feedforward
            if kp:
                position = float(self._sensor_values(self._joint_sensor_addresses[index][0])[0])
                torque += kp * (target_position - position)
            if kd:
                velocity = float(self._sensor_values(self._joint_sensor_addresses[index][1])[0])
                torque += kd * (target_velocity - velocity)

            self.data.ctrl[index] = torque

    def zero_control(self) -> None:
        self._joint_commands.clear()
        self.data.ctrl[:] = 0.0

    def step(self) -> None:
        self._update_joint_control()
        mujoco.mj_step(self.model, self.data)

    def body_pose(self, name: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Return ``(position, rotation)`` of a body, or ``None`` if absent."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            return None
        position = np.array(self.data.xpos[body_id], dtype=float, copy=True)
        rotation = np.array(self.data.xmat[body_id], dtype=float, copy=True).reshape(3, 3)
        return position, rotation

    def reset_to_keyframe(self, keyframe: str) -> None:
        """Reset the simulation state, falling back to the model default."""
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
        self.zero_control()
        mujoco.mj_forward(self.model, self.data)

    def print_scene_information(self) -> None:
        """Print names and sensor offsets in the same format as Unitree tools."""
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(
                self.model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.model.nsensor):
            name = mujoco.mj_id2name(
                self.model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                self.model.sensor_dim[i],
                )
            index = index + self.model.sensor_dim[i]
        print(" ")



