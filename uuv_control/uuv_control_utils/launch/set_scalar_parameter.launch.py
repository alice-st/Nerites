#!/usr/bin/env python3

# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

"""
ROS 2 Launch File – Set Scalar Hydrodynamic Parameters for UUV Simulator (with OpaqueFunction)

This launch file starts a ROS 2 node (`set_scalar_parameter.py`) that calls a
Gazebo service to update a single scalar hydrodynamic parameter in the UUV Simulator.

The node communicates with the Gazebo hydrodynamics plugin and updates properties
like fluid density, damping scaling, or volume scaling by invoking the appropriate
service under the vehicle’s namespace.

Required launch arguments:
    - uuv_name: Namespace of the vehicle (e.g., 'rexrov').
    - service_name: Name of the Gazebo hydrodynamics service to call.
      Must be one of:
        'set_fluid_density', 'set_added_mass_scaling', 'set_damping_scaling',
        'set_volume_scaling', 'set_volume_offset', 'set_added_mass_offset',
        'set_linear_damping_offset', 'set_nonlinear_damping_offset',
        'set_linear_forward_speed_damping_offset'.
    - data: Floating-point value to assign to the selected parameter.

Example usage:
    ros2 launch uuv_control_utils set_scalar_parameter.launch.py uuv_name:=rexrov service_name:=set_damping_scaling data:=10

This will call `/rexrov/set_damping_scaling` and set the damping scaling value to 1028.0.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """Resolve substitutions and create the node at runtime."""
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    service_name = LaunchConfiguration('service_name').perform(context)
    data = LaunchConfiguration('data').perform(context)

    # Build dynamic node name
    dynamic_node_name = f"set_scalar_parameter_{service_name}"

    # Create the Node with resolved values
    set_scalar_node = Node(
        package='uuv_control_utils',
        executable='set_scalar_parameter.py',
        name=dynamic_node_name,
        output='screen',
        parameters=[{
            'service_name': service_name,
            'data': data
        }]
    )

    # Wrap inside the vehicle namespace
    group = GroupAction([
        PushRosNamespace(uuv_name),
        set_scalar_node
    ])

    return [group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', default_value='rexrov', description='Namespace of the vehicle (e.g., rexrov)'),
        DeclareLaunchArgument('service_name', default_value='set_fluid_density', description='Gazebo hydrodynamic service to call'),
        DeclareLaunchArgument('data', default_value='1028.0', description='Scalar value to set for the service'),

        OpaqueFunction(function=launch_setup)
    ])
