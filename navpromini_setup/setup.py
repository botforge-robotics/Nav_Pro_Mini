from setuptools import setup
import os
from glob import glob

package_name = 'navpromini_setup'

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
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='NavPro Mini robot Wi-Fi setup and hardware bringup',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'status_display_node = navpromini_setup.status_display_node:main',
            'provision_portal = navpromini_setup.provision_portal:main',
        ],
    },
)
