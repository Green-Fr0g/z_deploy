"""Command line entry point for the multi-robot MuJoCo simulator."""
from __future__ import annotations

import argparse
import sys

from .app import SimulatorApp, ViewerError
from .config import load_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='robot_mujoco_sim',
        description='Run the multi-robot MuJoCo ROS 2 simulator.',
        epilog=(
            'Examples:\n'
            '  robot_mujoco_sim\n'
            '  robot_mujoco_sim --robot h2 --scene empty.xml\n'
            '  robot_mujoco_sim --viewer mjviser --camera'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '-c', '--camera', action='store_true', default=None,
        help='make the viewer camera follow the robot')
    parser.add_argument(
        '-r', '--robot', metavar='NAME',
        help='robot name; resolved in robots_model/<name>_mjcf (overrides sim.yaml)')
    parser.add_argument(
        '-s', '--scene', metavar='FILE',
        help='scene file in the selected robot MJCF directory (default: empty.xml)')
    parser.add_argument(
        '-v', '--viewer', choices=('native', 'mjviser'),
        help='rendering backend: native or mjviser (overrides viewer in sim.yaml)')
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = load_config(robot=args.robot, scene=args.scene, viewer=args.viewer)
    except (OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2

    app = SimulatorApp(config, show_viewer=True, follow_camera=args.camera)
    try:
        return app.run()
    except KeyboardInterrupt:
        return 0
    except ViewerError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
