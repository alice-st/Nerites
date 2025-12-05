#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Launch configurations (arguments) ---
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    joy_id = LaunchConfiguration('joy_id')
    namespace = LaunchConfiguration('namespace')

    axis_x = LaunchConfiguration('axis_x')
    axis_y = LaunchConfiguration('axis_y')
    axis_z = LaunchConfiguration('axis_z')
    axis_yaw = LaunchConfiguration('axis_yaw')
    deadman_button = LaunchConfiguration('deadman_button')
    exclusion_buttons = LaunchConfiguration('exclusion_buttons')

    axis_oberon_azimuth = LaunchConfiguration('axis_oberon_azimuth')
    axis_oberon_shoulder = LaunchConfiguration('axis_oberon_shoulder')
    axis_oberon_wrist = LaunchConfiguration('axis_oberon_wrist')

    oberon_exclusion_button = LaunchConfiguration('oberon_exclusion_button')
    oberon_deadman_button = LaunchConfiguration('oberon_deadman_button')
    oberon_home_button = LaunchConfiguration('oberon_home_button')

    gripper_open_button = LaunchConfiguration('gripper_open_button')
    gripper_close_button = LaunchConfiguration('gripper_close_button')

    # --- Declare arguments ---
    declare_x = DeclareLaunchArgument('x', default_value='0', description='Initial x position')
    declare_y = DeclareLaunchArgument('y', default_value='0', description='Initial y position')
    declare_z = DeclareLaunchArgument('z', default_value='-70', description='Initial z position')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw (rad)')
    declare_joy_id = DeclareLaunchArgument('joy_id', default_value='0', description='Joystick device ID')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='rexrov', description='Vehicle namespace')

    declare_axis_x = DeclareLaunchArgument('axis_x', default_value='4', description='Joystick axis for x')
    declare_axis_y = DeclareLaunchArgument('axis_y', default_value='3', description='Joystick axis for y')
    declare_axis_z = DeclareLaunchArgument('axis_z', default_value='1', description='Joystick axis for z')
    declare_axis_yaw = DeclareLaunchArgument('axis_yaw', default_value='0', description='Joystick axis for yaw')
    declare_deadman_button = DeclareLaunchArgument('deadman_button', default_value='-1', description='Deadman button')
    declare_exclusion_buttons = DeclareLaunchArgument(
        'exclusion_buttons',
        default_value='4,5',
        description='Comma-separated list of exclusion buttons',
    )

    declare_axis_oberon_azimuth = DeclareLaunchArgument(
        'axis_oberon_azimuth', default_value='0', description='Axis for Oberon azimuth'
    )
    declare_axis_oberon_shoulder = DeclareLaunchArgument(
        'axis_oberon_shoulder', default_value='1', description='Axis for Oberon shoulder'
    )
    declare_axis_oberon_wrist = DeclareLaunchArgument(
        'axis_oberon_wrist', default_value='3', description='Axis for Oberon wrist'
    )

    declare_oberon_exclusion_button = DeclareLaunchArgument(
        'oberon_exclusion_button', default_value='4', description='Oberon exclusion button'
    )
    declare_oberon_deadman_button = DeclareLaunchArgument(
        'oberon_deadman_button', default_value='5', description='Oberon deadman button'
    )
    declare_oberon_home_button = DeclareLaunchArgument(
        'oberon_home_button', default_value='7', description='Oberon home button'
    )

    declare_gripper_open_button = DeclareLaunchArgument(
        'gripper_open_button', default_value='1', description='Gripper open button'
    )
    declare_gripper_close_button = DeclareLaunchArgument(
        'gripper_close_button', default_value='2', description='Gripper close button'
    )

    # --- Paths to included launch files (ROS 2 .launch.py versions) ---
    upload_rexrov_oberon4_launch_path = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch',
        'upload_rexrov_oberon4.launch.py',   # assuming ROS 2 port
    )

    thruster_manager_launch_path = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'launch',
        'thruster_manager.launch.py',        # assuming ROS 2 port
    )

    uuv_teleop_launch_path = os.path.join(
        get_package_share_directory('uuv_teleop'),
        'launch',
        'uuv_teleop.launch.py',              # assuming ROS 2 port
    )

    oberon_joint_control_launch_path = os.path.join(
        get_package_share_directory('oberon4_control'),
        'launch',
        'joint_control.launch.py',           # assuming ROS 2 port
    )

    # --- YAML configs for cascaded PID ---
    cascaded_share_dir = get_package_share_directory('uuv_control_cascaded_pid')
    inertial_yaml = os.path.join(cascaded_share_dir, 'config', 'rexrov', 'inertial.yaml')
    vel_pid_yaml = os.path.join(cascaded_share_dir, 'config', 'rexrov', 'vel_pid_control.yaml')

    # --- upload_rexrov_oberon4 ---
    upload_rexrov_oberon4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(upload_rexrov_oberon4_launch_path),
        launch_arguments={
            'namespace': namespace,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    )

    # --- thruster_manager ---
    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_manager_launch_path),
        launch_arguments={
            'uuv_name': namespace,
            'model_name': 'rexrov',
        }.items(),
    )

    # --- controllers in namespace ---
    controllers_group = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='uuv_control_cascaded_pid',
            executable='AccelerationControl.py',
            name='acceleration_control',
            output='screen',
            parameters=[
                inertial_yaml,
                vel_pid_yaml,
                {'tf_prefix': [namespace, '/']},
            ],
        ),

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
                # inside namespace: odom -> pose_gt, cmd_accel -> cmd_accel
                ('odom', 'pose_gt'),
                ('cmd_accel', 'cmd_accel'),
            ],
        ),
    ])

    # --- teleop include ---
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
            'deadman_button': deadman_button,
            'exclusion_buttons': exclusion_buttons,
        }.items(),
    )

    # --- Oberon arm joint control ---
    oberon_joint_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oberon_joint_control_launch_path),
        launch_arguments={
            'uuv_name': namespace,
            'arm_name': 'oberon4',
            'deadman_button': oberon_deadman_button,
            'exclusion_button': oberon_exclusion_button,
            'home_button': oberon_home_button,
            'gripper_open_button': gripper_open_button,
            'gripper_close_button': gripper_close_button,
            'axis_azimuth': axis_oberon_azimuth,
            'axis_shoulder': axis_oberon_shoulder,
            'axis_wrist': axis_oberon_wrist,
        }.items(),
    )

    # --- RViz (rviz2 in ROS 2) ---
    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz',
        'rexrov_default.rviz',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_joy_id,
        declare_namespace,
        declare_axis_x,
        declare_axis_y,
        declare_axis_z,
        declare_axis_yaw,
        declare_deadman_button,
        declare_exclusion_buttons,
        declare_axis_oberon_azimuth,
        declare_axis_oberon_shoulder,
        declare_axis_oberon_wrist,
        declare_oberon_exclusion_button,
        declare_oberon_deadman_button,
        declare_oberon_home_button,
        declare_gripper_open_button,
        declare_gripper_close_button,
        upload_rexrov_oberon4,
        thruster_manager,
        controllers_group,
        teleop,
        oberon_joint_control,
        rviz,
    ])
