"""Setup for manipulator_hardware package."""
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'manipulator_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['manipulator_hardware.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'minimalmodbus>=2.1.0',
        'pyserial>=3.5',
    ],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='barabashsr@gmail.com',
    description='Hardware interface for ya_robot_manipulator with Modbus RTU',
    license='MIT',
    extras_require={
        'test': [
            'pytest>=7.0',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_hardware_interface = manipulator_hardware.scripts.test_hardware_interface:main',
        ],
    },
)
