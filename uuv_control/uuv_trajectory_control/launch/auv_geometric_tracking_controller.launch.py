# Copyright (c) 2025 The Nerites Authors.
# All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition

"""
This launch file starts the AUV Geometric Tracking Controller node, which receives odometry and a planned trajectory and 
computes the fin angles and thruster commands needed to follow that trajectory. 
The controller uses configurable gains, thrust limits, and fin mappings to generate smooth, constrained movements. 
It supports both ENU (world) and NED frames, publishes reference and error topics for monitoring, 
and can optionally start a trajectory marker publisher for visualization in RViz.
"""

def launch_setup(context, *args, **kwargs):
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    gui_on = LaunchConfiguration('gui_on').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    # Common parameters
    params = {
        'max_forward_speed': LaunchConfiguration('max_forward_speed').perform(context),
        'min_thrust': LaunchConfiguration('min_thrust').perform(context),
        'max_thrust': LaunchConfiguration('max_thrust').perform(context),
        'thruster_topic': LaunchConfiguration('thruster_topic').perform(context),
        'thrust_p_gain': LaunchConfiguration('thrust_p_gain').perform(context),
        'thrust_d_gain': LaunchConfiguration('thrust_d_gain').perform(context),
        'gain_roll': LaunchConfiguration('gain_roll').perform(context),
        'gain_pitch': LaunchConfiguration('gain_pitch').perform(context),
        'gain_yaw': LaunchConfiguration('gain_yaw').perform(context),
        'n_fins': LaunchConfiguration('n_fins').perform(context),
        'map_roll': [float(x) for x in LaunchConfiguration('map_roll').perform(context).split(',')],
        'map_pitch': [float(x) for x in LaunchConfiguration('map_pitch').perform(context).split(',')],
        'map_yaw': [float(x) for x in LaunchConfiguration('map_yaw').perform(context).split(',')],
        'fin_topic_prefix': LaunchConfiguration('fin_topic_prefix').perform(context),
        'fin_topic_suffix': LaunchConfiguration('fin_topic_suffix').perform(context),
        'max_fin_angle': LaunchConfiguration('max_fin_angle').perform(context),
        'idle_radius': LaunchConfiguration('idle_radius').perform(context),
        'look_ahead_delay': LaunchConfiguration('look_ahead_delay').perform(context),
        'desired_pitch_limit': LaunchConfiguration('desired_pitch_limit').perform(context),
        'yaw_error_limit': LaunchConfiguration('yaw_error_limit').perform(context),
        'dubins': {
            'radius': LaunchConfiguration('dubins_radius').perform(context),
            'max_pitch': LaunchConfiguration('dubins_max_pitch').perform(context),
        },
        'thruster_config': {
            'topic_prefix': LaunchConfiguration('thruster_topic_prefix').perform(context),
            'topic_suffix': LaunchConfiguration('thruster_topic_suffix').perform(context),
            'frame_base': LaunchConfiguration('thruster_frame_base').perform(context),
            'max_thrust': LaunchConfiguration('max_thrust').perform(context),
            'conversion_fcn': LaunchConfiguration('thruster_conversion_fcn').perform(context),
            'conversion_fcn_params': {
                'gain': LaunchConfiguration('thruster_gain').perform(context),
                'input': [float(x) for x in LaunchConfiguration('thruster_input').perform(context).split(',')],
                'output': [float(x) for x in LaunchConfiguration('thruster_output').perform(context).split(',')],
            },
        }
    }

    actions = []

    # GUI visualization node (optional)
    if gui_on == 'true':
        actions.append(
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_control_utils',
                    executable='trajectory_marker_publisher.py',
                    name='trajectory_marker_publisher',
                    output='screen'
                )
            ])
        )

    # Controller node (world frame)
    actions.append(
        GroupAction([
            PushRosNamespace(namespace=uuv_name),
            Node(
                package='uuv_trajectory_control',
                executable='auv_geometric_tracking_controller.py',
                name='auv_geometric_tracking_controller',
                output='screen',
                condition=UnlessCondition(LaunchConfiguration('use_ned_frame')),
                remappings=[
                    ('odom', 'pose_gt'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static')
                ],
                parameters=[{
                    **params,
                    'base_link': 'base_link',
                    'is_underactuated': True,
                    'inertial_frame_id': 'world'
                }]
            ),
            Node(
                package='uuv_trajectory_control',
                executable='auv_geometric_tracking_controller.py',
                name='auv_geometric_tracking_controller',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_ned_frame')),
                remappings=[('odom', 'pose_gt_ned')],
                parameters=[{
                    **params,
                    'base_link': 'base_link_ned',
                    'is_underactuated': True,
                    'inertial_frame_id': 'world_ned'
                }]
            )
        ])
    )

    return actions

def generate_launch_description():
    return LaunchDescription([
        # Declare all launch arguments
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('max_forward_speed', default_value='2'),
        DeclareLaunchArgument('dubins_radius', default_value='10'),
        DeclareLaunchArgument('dubins_max_pitch', default_value='0.09'),
        DeclareLaunchArgument('min_thrust', default_value='70'),
        DeclareLaunchArgument('max_thrust', default_value='200'),
        DeclareLaunchArgument('thruster_topic', default_value='thrusters/id_0/input'),
        DeclareLaunchArgument('thrust_p_gain', default_value='1'),
        DeclareLaunchArgument('thrust_d_gain', default_value='1'),
        DeclareLaunchArgument('gain_roll', default_value='1'),
        DeclareLaunchArgument('gain_pitch', default_value='1'),
        DeclareLaunchArgument('gain_yaw', default_value='1'),
        DeclareLaunchArgument('n_fins', default_value='4'),
        DeclareLaunchArgument('map_roll', default_value='0,0,0,0'),
        DeclareLaunchArgument('map_pitch', default_value='0,0,0,0'),
        DeclareLaunchArgument('map_yaw', default_value='0,0,0,0'),
        DeclareLaunchArgument('max_fin_angle', default_value='0.0'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='input'),
        DeclareLaunchArgument('idle_radius', default_value='10.0'),
        DeclareLaunchArgument('look_ahead_delay', default_value='5.0'),
        DeclareLaunchArgument('desired_pitch_limit', default_value='0.26'),
        DeclareLaunchArgument('yaw_error_limit', default_value='1.0'),
        DeclareLaunchArgument('thruster_topic_prefix', default_value='thrusters'),
        DeclareLaunchArgument('thruster_topic_suffix', default_value='input'),
        DeclareLaunchArgument('thruster_frame_base', default_value='thruster_'),
        DeclareLaunchArgument('thruster_conversion_fcn', default_value='proportional'),
        DeclareLaunchArgument('thruster_gain', default_value='0.5'),
        DeclareLaunchArgument('thruster_input', default_value='0,1,2,3'),
        DeclareLaunchArgument('thruster_output', default_value='0,1,2,3'),

        # OpaqueFunction to resolve LaunchConfigurations dynamically
        OpaqueFunction(function=launch_setup)
    ])
