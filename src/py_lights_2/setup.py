from setuptools import find_packages, setup

package_name = 'py_lights_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nasa',
    maintainer_email='JaredAHillyer@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = py_lights_2.hello:main',
            'talker = py_lights_2.pub:main',
            'listener = py_lights_2.sub:main',
            'light_talker = py_lights_2.light_pub:main',
            'light_listener = py_lights_2.light_sub:main'
        ],
    },
)