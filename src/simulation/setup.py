import os

from setuptools import find_packages, setup

package_name = 'robot_mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim.yaml']),
    ] + [
        ('share/' + package_name + '/' + os.path.dirname(path), [path])
        for root, _, files in os.walk('robots_model')
        for path in (os.path.join(root, filename) for filename in files)
    ],
    # ROS Humble ships binary extensions built against NumPy 1.x.
    install_requires=[
        'setuptools', 'PyYAML>=5.4,<7', 'numpy>=1.21,<1.25',
        'scipy>=1.8,<1.16', 'mujoco>=3.6,<4',
        'mjviser==0.0.14', 'viser==1.1.1',
    ],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'robot_mujoco_sim = robot_mujoco_sim.cli:main',
        ],
    },
)
