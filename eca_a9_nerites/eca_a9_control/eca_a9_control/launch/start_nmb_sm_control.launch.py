import os
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('uuv_name', default_value='eca_a9'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('teleop_on', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),
        DeclareLaunchArgument('axis_yaw', default_value='0'),
        DeclareLaunchArgument('axis_x', default_value='4'),
        DeclareLaunchArgument('axis_y', default_value='3'),
        DeclareLaunchArgument('axis_z', default_value='1'),
        DeclareLaunchArgument(
            'Kd', 
            default_value='24.328999405818507,95.16574836816616,25.943377407248825,0,6.388371356010936,79.2844976871164'
        ),
        DeclareLaunchArgument(
            'Ki', 
            default_value='0.0010232768152540483,0.0010232768152540483,0.0010232768152540483,0,0.11901644069756079,0.11901644069756079'
        ),
        DeclareLaunchArgument(
            'slope', 
            default_value='0.9903858668992097,0.9903858668992097,0.9903858668992097,0,0.20796465986893387,0.20796465986893387'
        ),
        DeclareLaunchArgument('look_ahead_delay', default_value='2.0'),
        DeclareLaunchArgument('min_thrust', default_value='40'),
    ]


    control_allocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('eca_a9_control'),
                'launch',
                'start_control_allocator.launch.py'
            )
        )
    )


    controller_with_ned = Node(
            package='uuv_trajectory_control',
            executable='rov_nmb_sm_controller.py',
            name='rov_nmb_sm_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_ned_frame')),
            remappings=[
                ('odom', 'pose_gt_ned'),
                ('trajectory', 'dp_controller/trajectory'),
                ('input_trajectory', 'dp_controller/input_trajectory'),
                ('waypoints', 'dp_controller/waypoints'),
                ('error', 'dp_controller/error'),
                ('reference', 'dp_controller/reference'),
                ('thruster_output', 'thruster_manager/input_stamped'),
                ('auv_command_output', 'control_allocation/control_input'),
            ],
            parameters=[{
                'saturation': 200,
                'thrusters_only': False,
                'Kd': LaunchConfiguration('Kd'),
                'Ki': LaunchConfiguration('Ki'),
                'slope': LaunchConfiguration('slope'),
                'inertial_frame_id': 'world_ned',
                'max_forward_speed': 3.0,
                'use_stamped_poses_only': False,
                'look_ahead_delay': LaunchConfiguration('look_ahead_delay'),
                'min_thrust': LaunchConfiguration('min_thrust'),
                'dubins': {
                    'radius': 25,
                    'max_pitch': 0.09
                }
            }]
        )


    controller_without_ned = Node(
            package='uuv_trajectory_control',
            executable='rov_nmb_sm_controller.py',
            name='rov_nmb_sm_controller',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_ned_frame')),
            remappings=[
                ('odom', 'pose_gt'),
                ('trajectory', 'dp_controller/trajectory'),
                ('input_trajectory', 'dp_controller/input_trajectory'),
                ('waypoints', 'dp_controller/waypoints'),
                ('error', 'dp_controller/error'),
                ('reference', 'dp_controller/reference'),
                ('thruster_output', 'thruster_manager/input_stamped'),
                ('auv_command_output', 'control_allocation/control_input'),
            ],
            parameters=[{
                'saturation': 200,
                'thrusters_only': False,
                'Kd': LaunchConfiguration('Kd'),
                'Ki': LaunchConfiguration('Ki'),
                'slope': LaunchConfiguration('slope'),
                'inertial_frame_id': 'world',
                'max_forward_speed': 3.0,
                'use_stamped_poses_only': False,
                'look_ahead_delay': LaunchConfiguration('look_ahead_delay'),
                'min_thrust': LaunchConfiguration('min_thrust'),
                'dubins': {
                    'radius': 25,
                    'max_pitch': 0.09
                }
            }]
        )


    uuv_group = GroupAction(
        actions=[
            PushRosNamespace(namespace=LaunchConfiguration('uuv_name')),
            controller_without_ned,
            controller_with_ned
        ]
    )


    gui_group = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('eca_a9_control'),
                'rviz2',
                'eca_a9_control.rviz2'  
            )],
            condition=IfCondition(LaunchConfiguration('gui_on'))
        )
    ])

    return LaunchDescription(declare_args + [control_allocator_launch, uuv_group, gui_group])
