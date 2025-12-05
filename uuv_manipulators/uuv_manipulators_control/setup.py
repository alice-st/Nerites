#!/usr/bin/env python3

from setuptools import setup, find_packages

# Must match <name> in package.xml
package_name = 'uuv_manipulators_control'

setup(
    name=package_name,
    version='0.0.0',

    # Discover Python packages under src/ (expects src/uuv_manipulators_control/)
    packages=find_packages('src'),
    package_dir={'': 'src'},

    data_files=[
        # Register package with ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alice',                 # change if you want
    maintainer_email='alice.stefanopoulou@gmail.com',
    description='ROS 2 control nodes for UUV manipulators',
    license='Apache-2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'gripper_controller = uuv_manipulators_control.gripper_controller:main',
            'joint_position_controller = uuv_manipulators_control.joint_position_controller:main',
            'jt_cartesian_controller = uuv_manipulators_control.jt_cartesian_controller:main',
            'set_joint_config = uuv_manipulators_control.set_joint_config:main',
        ],
    },
)
