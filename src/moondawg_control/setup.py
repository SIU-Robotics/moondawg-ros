import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moondawg_control'

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
    ],
    install_requires=[
        'setuptools',
        'smbus2',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Andrew Barnes',
    maintainer_email='andrew.barnes@siu.edu',
    description='ROS2 package for controlling the SIU Lunabotics robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_parser = moondawg_control.controller_parser:main',
            'i2c_node = moondawg_control.i2c_node:main',
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
