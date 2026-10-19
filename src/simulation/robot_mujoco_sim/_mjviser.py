"""Synchronization hooks for the pinned mjviser 0.0.14 backend."""
from __future__ import annotations

from typing import ContextManager

import mujoco
from mjviser import Viewer


class SynchronizedViewer(Viewer):
    """Protect built-in state editors with the same lock as ROS callbacks.

    Mjviser takes ``_lock`` around stepping, keyframes and joint/actuator
    edits. It must be reentrant because our step/reset/render callbacks also
    take the simulation lock. Scene uploads use their own lock in the adapter.
    """

    def __init__(self, *args, simulation_lock, camera_gui, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = simulation_lock
        # Replace the built-in camera callbacks, which run in a worker pool.
        self.scene.create_scene_gui = camera_gui

    def _setup_gui(self) -> None:
        with self._lock:
            super()._setup_gui()

    def _sync_sliders(self) -> None:
        with self._lock:
            super()._sync_sliders()

    def scene_update_lock(self) -> ContextManager[None]:
        """Return the lock that protects Mjviser scene data and uploads."""
        return self.scene._update_lock

    def _setup_physics_flags(self) -> None:
        # Upstream flag callbacks do not take _lock. Keep model.opt writes
        # synchronized with both physics and the scene's mj_forward().
        for title, enum, attribute, flags in (
            ('Disable Flags', mujoco.mjtDisableBit, 'disableflags', (
                ('Gravity', 'mjDSBL_GRAVITY'),
                ('Contact', 'mjDSBL_CONTACT'),
                ('Constraint', 'mjDSBL_CONSTRAINT'),
                ('Equality', 'mjDSBL_EQUALITY'),
                ('Limit', 'mjDSBL_LIMIT'),
                ('Spring', 'mjDSBL_SPRING'),
                ('Damper', 'mjDSBL_DAMPER'),
                ('Friction Loss', 'mjDSBL_FRICTIONLOSS'),
                ('Actuation', 'mjDSBL_ACTUATION'),
                ('Sensor', 'mjDSBL_SENSOR'),
                ('Warmstart', 'mjDSBL_WARMSTART'),
                ('Filter Parent', 'mjDSBL_FILTERPARENT'),
                ('Clamp Ctrl', 'mjDSBL_CLAMPCTRL'),
                ('Euler Damp', 'mjDSBL_EULERDAMP'),
                ('Refsafe', 'mjDSBL_REFSAFE'),
                ('Multi CCD', 'mjDSBL_MULTICCD'),
                ('Native CCD', 'mjDSBL_NATIVECCD'),
            )),
            ('Enable Flags', mujoco.mjtEnableBit, 'enableflags', (
                ('Energy', 'mjENBL_ENERGY'),
                ('Override', 'mjENBL_OVERRIDE'),
                ('Fwd Inverse', 'mjENBL_FWDINV'),
                ('Multi CCD', 'mjENBL_MULTICCD'),
            )),
        ):
            with self._server.gui.add_folder(title):
                for label, name in flags:
                    flag = getattr(enum, name, None)
                    if flag is None:
                        continue
                    bit = int(flag)
                    checkbox = self._server.gui.add_checkbox(
                        label,
                        initial_value=bool(getattr(self.model.opt, attribute) & bit))

                    @checkbox.on_update
                    def _(_, cb=checkbox, mask=bit, attr=attribute) -> None:
                        with self._lock, self.scene_update_lock():
                            value = getattr(self.model.opt, attr)
                            value = value | mask if cb.value else value & ~mask
                            setattr(self.model.opt, attr, value)
                            self.scene.needs_update = True
