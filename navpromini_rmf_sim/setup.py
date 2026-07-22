from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navpromini_rmf_sim'


def _site_data_files():
    entries = []
    site_root = 'site'
    if not os.path.isdir(site_root):
        return entries
    for root, _dirs, files in os.walk(site_root):
        for name in files:
            path = os.path.join(root, name)
            if not os.path.isfile(path) or os.path.islink(path) and not os.path.exists(path):
                continue
            dest = os.path.join('share', package_name, root)
            entries.append((dest, [path]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'include', 'adapters'),
            glob('launch/include/adapters/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'bridges'),
            glob('config/bridges/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.py')),
        * _site_data_files(),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='RMF multi-robot simulation bringup for NavProMini',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'import_map_to_site = navpromini_rmf_sim.import_map_to_site:main',
            'generate_site_assets = navpromini_rmf_sim.generate_site_assets:main',
            'publish_amcl_initial_poses = navpromini_rmf_sim.publish_amcl_initial_poses:main',
            'publish_robot_states = navpromini_rmf_sim.publish_robot_states:main',
            # Adapters (demos-style layout under navpromini_rmf_sim.adapters)
            'door_adapter = navpromini_rmf_sim.adapters.door_adapter:main',
            'lift_adapter = navpromini_rmf_sim.adapters.lift_adapter:main',
            'workcell_adapter = navpromini_rmf_sim.adapters.workcell_adapter:main',
            'path_to_nav2_bridge = '
            'navpromini_rmf_sim.adapters.fleet.path_to_nav2_bridge:main',
        ],
    },
)
