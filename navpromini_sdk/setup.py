from setuptools import setup
import os
from glob import glob

package_name = 'navpromini_sdk'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.handlers'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='HTTP + WebSocket API server for NavProMini',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sdk_server = navpromini_sdk.server:main',
        ],
    },
)
