#!/usr/bin/env python3

from setuptools import setup, find_packages

# ROS package name (must match <name> in package.xml)
package_name = 'uuv_manipulators_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alice',
    maintainer_email='alice@example.com',
    description='Kinematics utilities, interfaces and nodes for UUV manipulators (ROS 2 port)',
    license='Apache-2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            # Node executable used by the launch file.
            # This assumes:
            #   src/uuv_manipulator_nodes/kinematics_service.py
            # defines a def main().
            'kinematics_service.py = uuv_manipulator_nodes.kinematics_service:main',
        ],
    },
)
