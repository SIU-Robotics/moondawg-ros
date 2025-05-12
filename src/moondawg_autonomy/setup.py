from setuptools import find_packages, setup

package_name = 'moondawg_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/autonomy.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Krishna',
    maintainer_email='krishna@example.com',
    description='Autonomy stack for Moondawg robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = moondawg_autonomy.perception_node:main',
            'navigation_node = moondawg_autonomy.navigation_node:main',
            'mission_executor = moondawg_autonomy.mission_executor:main',
        ],
    },
)
