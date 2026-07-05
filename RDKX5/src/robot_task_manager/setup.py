import os
from glob import glob
from setuptools import setup

package_name = 'robot_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Task manager with state machine for library book-finding robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_manager_node = robot_task_manager.task_manager_node:main',
        ],
    },
)
