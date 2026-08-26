from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navpromini_launch_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='nagachaitanya948@gmail.com',
    description='Launch process supervision for NavProMini, plus rosbridge/rosapi. '
                 'Relocated from nav2_mission_planner.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_manager = navpromini_launch_manager.launch_manager:main',
        ],
    },
)
