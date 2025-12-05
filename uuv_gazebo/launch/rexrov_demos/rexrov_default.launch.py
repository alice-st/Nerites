#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Launch configurations (arguments) ---
    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    joy_id = LaunchConfiguration('joy_id')
    axis_yaw = LaunchConfiguration('axis_yaw')
    axis_x = LaunchConfiguration('axis_x')
    axis_y = LaunchConfiguration('axis_y')
    axis_z = LaunchConfiguration('axis_z')
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='rexrov',
        description='Namespace of the vehicle',
    )

    declare_x = DeclareLaunchArgument(
        'x',
        default_value='0',
        description='Initial x position',
    )

    declare_y = DeclareLaunchArgument(
        'y',
        default_value='0',
        description='Initial y position',
    )

    declare_z = DeclareLaunchArgument(
        'z',
        default_value='-70',
        description='Initial z position',
    )

    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw (rad)',
    )

    declare_joy_id = DeclareLaunchArgument(
        'joy_id',
        default_value='0',
        description='Joystick device ID',
    )

    declare_axis_yaw = DeclareLaunchArgument(
        'axis_yaw',
        default_value='0',
        description='Joystick axis for yaw',
    )

    declare_axis_x = DeclareLaunchArgument(
        'axis_x',
        default_value='4',
        description='Joystick axis for x',
    )

    declare_axis_y = DeclareLaunchArgument(
        'axis_y',
        default_value='3',
        description='Joystick axis for y',
    )

    declare_axis_z = DeclareLaunchArgument(
        'axis_z',
        default_value='1',
        description='Joystick axis for z',
    )

    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='1',
        description='Whether to launch RViz (1 = yes, 0 = no)',
    )

    # --- Paths to included launch files (ROS 2 .launch.py versions) ---
    upload_rexrov_launch_path = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch',
        'upload_rexrov.launch.py',          # assumes you have ported this
    )

    thruster_manager_launch_path = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'launch',
        'thruster_manager.launch.py',       # assumes ROS 2 version
    )

    uuv_teleop_launch_path = os.path.join(
        get_package_share_directory('uuv_teleop'),
        'launch',
        'uuv_teleop.launch.py',             # assumes ROS 2 version
    )

    # --- Parameter YAML files for controllers ---
    cascaded_share_dir = get_package_share_directory('uuv_control_cascaded_pid')
    inertial_yaml = os.path.join(
        cascaded_share_dir, 'config', 'rexrov', 'inertial.yaml'
    )
    vel_pid_yaml = os.path.join(
        cascaded_share_dir, 'config', 'rexrov', 'vel_pid_control.yaml'
    )

    # --- Include upload_rexrov.launch.py ---
    upload_rexrov = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(upload_rexrov_launch_path),
        launch_arguments={
            'mode': 'default',
            'namespace': namespace,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    )

    # --- Include thruster_manager.launch.py ---
    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_manager_launch_path),
        launch_arguments={
            'uuv_name': namespace,
            'model_name': 'rexrov',
        }.items(),
    )

    # --- Controllers in vehicle namespace ---
    controllers_group = GroupAction([
        PushRosNamespace(namespace),

        # AccelerationControl
        Node(
            package='uuv_control_cascaded_pid',
            executable='AccelerationControl.py',
            name='acceleration_control',
            output='screen',
            parameters=[
                inertial_yaml,
                vel_pid_yaml,
                {'tf_prefix': LaunchConfiguration('namespace').perform},  # see note below
            ],
        ),

        # VelocityControl
        Node(
            package='uuv_control_cascaded_pid',
            executable='VelocityControl.py',
            name='velocity_control',
            output='screen',
            parameters=[
                inertial_yaml,
                vel_pid_yaml,
            ],
            remappings=[
                # Inside namespace: 'odom' -> '<ns>/pose_gt', 'cmd_accel' -> '<ns>/cmd_accel'
                ('odom', 'pose_gt'),
                ('cmd_accel', 'cmd_accel'),
            ],
        ),
    ])

    # NOTE: if the tf_prefix handling above complains, replace that parameter with:
    # {'tf_prefix': [namespace, '/']}

    # --- Include teleop launch (uuv_teleop.launch.py) ---
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(uuv_teleop_launch_path),
        launch_arguments={
            'uuv_name': namespace,
            'joy_id': joy_id,
            'output_topic': 'cmd_vel',
            'message_type': 'twist',
            'axis_yaw': axis_yaw,
            'axis_x': axis_x,
            'axis_y': axis_y,
            'axis_z': axis_z,
        }.items(),
    )

    # --- Optional RViz (use rviz2 in ROS 2) ---
    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz',
        'rexrov_default.rviz',
    )

    rviz = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_namespace,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_joy_id,
        declare_axis_yaw,
        declare_axis_x,
        declare_axis_y,
        declare_axis_z,
        declare_launch_rviz,
        upload_rexrov,
        thruster_manager,
        controllers_group,
        teleop,
        rviz,
    ])
