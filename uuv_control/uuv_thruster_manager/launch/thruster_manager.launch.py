# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

"""
ROS 2 launch file for starting the Thruster Allocator node
==========================================================

This launch file sets up and runs the `thruster_allocator.py` node from the 
`uuv_thruster_manager` package. The thruster allocator computes and distributes 
desired forces/torques across a vehicle’s thrusters using either a precomputed 
Thruster Allocation Matrix (TAM) or, if requested, recalculates a new TAM.

Functionality:
--------------
- Pushes the `thruster_allocator` node under a specified UUV namespace.
- Loads the vehicle’s thruster configuration (YAML file).
- Optionally resets the TAM by generating a new matrix using URDF geometry.

Launch Arguments:
-----------------
- **model_name**:   Name of the UUV model (used for config lookup).
- **uuv_name**:     Namespace for the vehicle (defaults to `model_name`).
- **base_link**:    Base link frame of the vehicle (default: `base_link`).
- **timeout**:      Timeout for allocator initialization (-1 = no timeout).
- **reset_tam**:    If `true`, forces recalculation of the TAM instead of using the YAML file.

- **output_dir**:   Directory where a newly generated TAM will be stored (only used if `reset_tam` is true).
- **config_file**:  Path to the vehicle’s `thruster_manager.yaml`.
- **tam_file**:     Path to the precomputed TAM file (`TAM.yaml`).
- **urdf_file**:    Path to the vehicle URDF (required for TAM recalculation).

Usage:
------
- To launch using the existing TAM:
    ```
    ros2 launch uuv_thruster_manager thruster_allocator.launch.py model_name:=rexrov
    ```

- To reset and regenerate the TAM:
    ```
    ros2 launch uuv_thruster_manager thruster_allocator.launch.py model_name:=rexrov reset_tam:=true urdf_file:=<path_to_urdf>
    ```
"""

def launch_setup(context, *args, **kwargs):
    # Resolve arguments at runtime
    model_name = LaunchConfiguration('model_name').perform(context)
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    reset_tam = LaunchConfiguration('reset_tam').perform(context).lower() == 'true'
    config_file = LaunchConfiguration('config_file').perform(context)
    tam_file = LaunchConfiguration('tam_file').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)

    # --- Common parameters (always load config file) ---
    params = [config_file]

    # --- Conditional TAM handling ---
    if reset_tam:
        params += [
            {'output_dir': output_dir},
            {'urdf_file': urdf_file}
        ]
    else:
        params += [tam_file]

    # --- Build thruster allocator node ---
    thruster_allocator = Node(
        package='uuv_thruster_manager',
        executable='thruster_allocator.py',
        name='thruster_allocator',
        output='screen',
        remappings=[('tf', '/tf')],
        parameters=params
    )

    # --- Namespace group ---
    return [
        GroupAction([
            PushRosNamespace(uuv_name),
            thruster_allocator
        ])
    ]


def generate_launch_description():
    return LaunchDescription([
        # === Declare all launch arguments ===
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name', default_value=LaunchConfiguration('model_name')),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('timeout', default_value='-1'),
        DeclareLaunchArgument('reset_tam', default_value='true'),

        DeclareLaunchArgument(
            'output_dir',
            default_value=[
                FindPackageShare('uuv_thruster_manager'),
                '/config/',
                LaunchConfiguration('model_name')
            ]
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=[
                FindPackageShare('uuv_thruster_manager'),
                '/config/',
                LaunchConfiguration('model_name'),
                '/thruster_manager.yaml'
            ]
        ),
        DeclareLaunchArgument(
            'tam_file',
            default_value=[
                FindPackageShare('uuv_thruster_manager'),
                '/config/',
                LaunchConfiguration('model_name'),
                '/TAM.yaml'
            ]
        ),
        DeclareLaunchArgument('urdf_file', default_value='/home/alice/ros2_ws/install/x300_description/share/x300_description/robots/generated/x300/robot_description.urdf'),

        # === Main logic ===
        OpaqueFunction(function=launch_setup)
    ])
