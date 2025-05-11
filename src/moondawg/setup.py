import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moondawg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=[
        'setuptools',
        'smbus2',
        'pyserial',
        'opencv-python',
        'nav2-simple-commander',
        'slam-toolbox',
        'nav2-bringup',
    ],
    zip_safe=True,
    maintainer='Andrew Barnes',
    maintainer_email='andrew.barnes@siu.edu',
    description='ROS2 package for controlling the SIU Lunabotics robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_parser = moondawg.controller_parser:main',
            'i2c_node = moondawg.i2c_node:main',
            # 'serial_node = moondawg.serial_node:main',
        ],
    },
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering',
    ],
)
