import os
from glob import glob
from setuptools import setup

package_name = 'formation_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tortuga',
    maintainer_email='tortuga@example.com',
    description='Leader-follower formations for TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower = formation_control.follower_node:main',
            'tracker = formation_control.tracker_node:main',
        ],
    },
)
