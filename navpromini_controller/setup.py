from setuptools import setup
import os
from glob import glob

package_name = 'navpromini_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Real-robot bringup for NavProMini',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = navpromini_controller.odom_node:main',
            'battery_node = navpromini_controller.battery_node:main',
            'dock_manager_node = navpromini_controller.dock_manager_node:main',
            'camera_node = navpromini_controller.camera_node:main',
            'dock_tag_node = navpromini_controller.dock_tag_node:main',
            'tag_dock_node = navpromini_controller.tag_dock_node:main',
            'system_monitor_node = navpromini_controller.system_monitor_node:main',
            'app_data_store_node = navpromini_controller.app_data_store_node:main',
        ],
    },
)
