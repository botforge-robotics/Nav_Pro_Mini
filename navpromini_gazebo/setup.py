from setuptools import setup
from setuptools.command.build_py import build_py
import os
import subprocess
from glob import glob

package_name = 'navpromini_gazebo'


class CustomBuildPy(build_py):
    def run(self):
        plugin_script = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'plugins', 'build_plugin.sh')
        if os.path.exists(plugin_script):
            subprocess.check_call(['bash', plugin_script])
        super().run()


def get_model_files():
    data_files = []
    for root, _, files in os.walk('models'):
        if files:
            data_files.append((os.path.join('share', package_name, root), [os.path.join(root, f) for f in files]))
    return data_files


setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    cmdclass={'build_py': CustomBuildPy},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'plugins'), glob('plugins/*.so')),
        *get_model_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Gazebo Harmonic simulation bringup for NavProMini',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
