from setuptools import setup
import os
from glob import glob

package_name = 'navpromini_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'PyYAML', 'requests'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='NavPro Mini on-robot fleet agent',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'status_display_node = navpromini_fleet.status_display_node:main',
            'heartbeat_node = navpromini_fleet.heartbeat_node:main',
            'provision_portal = navpromini_fleet.provision_portal:main',
            'register_robot = navpromini_fleet.register_robot:main',
            'fleet_teleop_bridge = navpromini_fleet.fleet_teleop_bridge:main',
            'mode_manager = navpromini_fleet.mode_manager:main',
            'nav_goal_relay = navpromini_fleet.nav_goal_relay:main',
            'upload_map = navpromini_fleet.upload_map:main',
            'map_claim = navpromini_fleet.map_claim:main',
        ],
    },
)
