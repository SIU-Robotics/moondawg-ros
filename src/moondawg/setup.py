import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moondawg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew',
    maintainer_email='andrew.barnes@siu.edu',
    description='Package running on the Raspberry Pi.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_parser = moondawg.controller_parser:main',
            'i2c_node = moondawg.i2c_node:main'
        ],
    },
)
