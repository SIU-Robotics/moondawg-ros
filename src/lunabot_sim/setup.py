from setuptools import setup
import os
from glob import glob

package_name = 'lunabot_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Barnes',
    maintainer_email='andrew.barnes@siu.edu',
    description='Gazebo simulation for MoonDawg robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_controller_bridge = lunabot_sim.gazebo_controller_bridge:main',
        ],
    },
)