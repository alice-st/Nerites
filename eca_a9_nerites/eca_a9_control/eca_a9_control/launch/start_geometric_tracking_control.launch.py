from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument('uuv_name', default_value='eca_a9'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.1'),
        DeclareLaunchArgument('dubins_radius', default_value='10'),
        DeclareLaunchArgument('dubins_max_pitch', default_value='0.26'),
        DeclareLaunchArgument('min_thrust', default_value='0'),
        DeclareLaunchArgument('max_thrust', default_value='120'),
        DeclareLaunchArgument('thruster_topic', default_value='thrusters/id_0/input'),
        DeclareLaunchArgument('thrust_p_gain', default_value='5.0'),
        DeclareLaunchArgument('thrust_d_gain', default_value='0.01'),
        DeclareLaunchArgument('gain_roll', default_value='0.2'),
        DeclareLaunchArgument('gain_pitch', default_value='0.8'),
        DeclareLaunchArgument('gain_yaw', default_value='0.8'),
        DeclareLaunchArgument('n_fins', default_value='4'),
        DeclareLaunchArgument('map_roll', default_value='1,1,1,1'),
        DeclareLaunchArgument('map_pitch', default_value='1,1,-1,-1'),
        DeclareLaunchArgument('map_yaw', default_value='-1,1,1,-1'),
        DeclareLaunchArgument('max_fin_angle', default_value='1.396263402'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='input'),
        DeclareLaunchArgument('idle_radius', default_value='30.0'),
        DeclareLaunchArgument('look_ahead_delay', default_value='0.0'),
        DeclareLaunchArgument('desired_pitch_limit', default_value='0.26'),
        DeclareLaunchArgument('yaw_error_limit', default_value='1.57'),
        DeclareLaunchArgument('thruster_topic_prefix', default_value='thrusters'),
        DeclareLaunchArgument('thruster_topic_suffix', default_value='input'),
        DeclareLaunchArgument('thruster_frame_base', default_value='thruster_'),
        DeclareLaunchArgument('thruster_conversion_fcn', default_value='proportional'),
        DeclareLaunchArgument('thruster_gain', default_value='0.000049'),
        DeclareLaunchArgument('thruster_input', default_value='1,0,0,0'),
        DeclareLaunchArgument('thruster_output', default_value='1,0,0,0')
    ]

    # RViz visualization group
    gui_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('gui_on')),
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace(namespace=LaunchConfiguration('uuv_name')),
                    Node(
                        package='uuv_control_utils',
                        executable='trajectory_marker_publisher.py',
                        name='trajectory_marker_publisher',
                        output='screen'
                    )
                ]
            )
        ]
    )

    # Geometric tracking controller without NED frame
    controller_without_ned = Node(
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
        parameters=[
            {'max_forward_speed': LaunchConfiguration('max_forward_speed')},
            {'base_link': 'base_link'},
            {'is_underactuated': True},
            {'inertial_frame_id': 'world'},
            {'min_thrust': LaunchConfiguration('min_thrust')},
            {'max_thrust': LaunchConfiguration('max_thrust')},
            {'thruster_topic': LaunchConfiguration('thruster_topic')},
            {'thrust_p_gain': LaunchConfiguration('thrust_p_gain')},
            {'thrust_d_gain': LaunchConfiguration('thrust_d_gain')},
            {'gain_roll': LaunchConfiguration('gain_roll')},
            {'gain_pitch': LaunchConfiguration('gain_pitch')},
            {'gain_yaw': LaunchConfiguration('gain_yaw')},
            {'n_fins': LaunchConfiguration('n_fins')},
            {'map_roll': LaunchConfiguration('map_roll')},
            {'map_pitch': LaunchConfiguration('map_pitch')},
            {'map_yaw': LaunchConfiguration('map_yaw')},
            {'fin_topic_prefix': LaunchConfiguration('fin_topic_prefix')},
            {'fin_topic_suffix': LaunchConfiguration('fin_topic_suffix')},
            {'max_fin_angle': LaunchConfiguration('max_fin_angle')},
            {'idle_radius': LaunchConfiguration('idle_radius')},
            {'look_ahead_delay': LaunchConfiguration('look_ahead_delay')},
            {'desired_pitch_limit': LaunchConfiguration('desired_pitch_limit')},
            {'yaw_error_limit': LaunchConfiguration('yaw_error_limit')},
            {'dubins': {
                'radius': LaunchConfiguration('dubins_radius'),
                'max_pitch': LaunchConfiguration('dubins_max_pitch')
            }},
            {'thruster_config': {
                'topic_prefix': LaunchConfiguration('thruster_topic_prefix'),
                'topic_suffix': LaunchConfiguration('thruster_topic_suffix'),
                'frame_base': LaunchConfiguration('thruster_frame_base'),
                'max_thrust': LaunchConfiguration('max_thrust'),
                'conversion_fcn': LaunchConfiguration('thruster_conversion_fcn'),
                'conversion_fcn_params': {
                    'gain': LaunchConfiguration('thruster_gain'),
                    'input': LaunchConfiguration('thruster_input'),
                    'output': LaunchConfiguration('thruster_output')
                }
            }}
        ]
    )

    # Geometric tracking controller with NED frame
    controller_with_ned = Node(
        package='uuv_trajectory_control',
        executable='auv_geometric_tracking_controller.py',
        name='auv_geometric_tracking_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ned_frame')),
        remappings=[
            ('odom', 'pose_gt_ned'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        parameters=[
            {'max_forward_speed': LaunchConfiguration('max_forward_speed')},
            {'base_link': 'base_link_ned'},
            {'is_underactuated': True},
            {'inertial_frame_id': 'world_ned'},
            {'min_thrust': LaunchConfiguration('min_thrust')},
            {'max_thrust': LaunchConfiguration('max_thrust')},
            {'thruster_topic': LaunchConfiguration('thruster_topic')},
            {'thrust_p_gain': LaunchConfiguration('thrust_p_gain')},
            {'thrust_d_gain': LaunchConfiguration('thrust_d_gain')},
            {'gain_roll': LaunchConfiguration('gain_roll')},
            {'gain_pitch': LaunchConfiguration('gain_pitch')},
            {'gain_yaw': LaunchConfiguration('gain_yaw')},
            {'n_fins': LaunchConfiguration('n_fins')},
            {'map_roll': LaunchConfiguration('map_roll')},
            {'map_pitch': LaunchConfiguration('map_pitch')},
            {'map_yaw': LaunchConfiguration('map_yaw')},
            {'fin_topic_prefix': LaunchConfiguration('fin_topic_prefix')},
            {'fin_topic_suffix': LaunchConfiguration('fin_topic_suffix')},
            {'max_fin_angle': LaunchConfiguration('max_fin_angle')},
            {'idle_radius': LaunchConfiguration('idle_radius')},
            {'look_ahead_delay': LaunchConfiguration('look_ahead_delay')},
            {'desired_pitch_limit': LaunchConfiguration('desired_pitch_limit')},
            {'yaw_error_limit': LaunchConfiguration('yaw_error_limit')},
            {'dubins': {
                'radius': LaunchConfiguration('dubins_radius'),
                'max_pitch': LaunchConfiguration('dubins_max_pitch')
            }},
            {'thruster_config': {
                'topic_prefix': LaunchConfiguration('thruster_topic_prefix'),
                'topic_suffix': LaunchConfiguration('thruster_topic_suffix'),
                'frame_base': LaunchConfiguration('thruster_frame_base'),
                'max_thrust': LaunchConfiguration('max_thrust'),
                'conversion_fcn': LaunchConfiguration('thruster_conversion_fcn'),
                'conversion_fcn_params': {
                    'gain': LaunchConfiguration('thruster_gain'),
                    'input': LaunchConfiguration('thruster_input'),
                    'output': LaunchConfiguration('thruster_output')
                }
            }}
        ]
    )

    # Grouping actions
    controller_group = GroupAction(
        actions=[
            PushRosNamespace(namespace=LaunchConfiguration('uuv_name')),
            controller_without_ned,
            controller_with_ned
        ]
    )

    return LaunchDescription(launch_arguments + [gui_group, controller_group])
