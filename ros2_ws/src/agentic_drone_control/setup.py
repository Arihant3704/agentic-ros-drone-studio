import os
from glob import glob
from setuptools import setup

package_name = 'agentic_drone_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docketrun',
    maintainer_email='docketrun@todo.todo',
    description='Agentic ROS 2 drone control using Ollama and PX4 Offboard Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_controller = agentic_drone_control.px4_offboard_controller:main',
            'agent_node = agentic_drone_control.agent_node:main',
        ],
    },
)
