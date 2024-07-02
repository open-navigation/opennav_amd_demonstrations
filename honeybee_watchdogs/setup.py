from glob import glob
import os

from setuptools import setup


package_name = 'honeybee_watchdogs'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='steve@opennav.org',
    description='Watchdogs for the honeybees',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ouster_activation_watchdog = honeybee_watchdogs.ouster_activation_watchdog:main',
            'power_off_backpack = honeybee_watchdogs.power_off_backpack:main',
            'capture_system_metrics = honeybee_watchdogs.capture_system_metrics:main',
            'record_rosbag_data = honeybee_watchdogs.record_rosbag_data:main',
            'joystick_estop = honeybee_watchdogs.joystick_estop:main',
        ],
    },
)
