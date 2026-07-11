import os
from setuptools import find_packages, setup
from glob import glob
package_name = 'robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # Include URDF models
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    # Include World files
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    # Include YAML
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'protos'), glob('protos/*.proto')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theo',
    maintainer_email='theo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_tool = robot_simulation.robot_tool:main',
            'obstacle_avoidance = robot_simulation.obstacle_avoidance_node:main',
            'line_follower = robot_simulation.line_follower:main',
            'wall_follower = robot_simulation.wall_follower:main',
            'teleop_node= robot_simulation.teleop_node:main',
            'niku_controller = robot_simulation.niku_controller:main',
            'niku_controller_pid = robot_simulation.niku_controller_pid:main',
            'niku_controller_viapoints = robot_simulation.niku_controller_viapoints:main',
            'ekf_logger = robot_simulation.ekf_logger:main',
            
        ],
    },
)
