from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Resolve arguments
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    joy_id = LaunchConfiguration('joy_id').perform(context)

    actions = []

    # Include the finned_uuv_teleop.launch.py file
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('uuv_teleop'), '/launch/finned_uuv_teleop.launch.py'
            ]),
            launch_arguments={
                'uuv_name': uuv_name,
                'joy_id': joy_id,
                'gain_roll': '1,1,1,1',
                'gain_pitch': '1,1,-1,-1',
                'gain_yaw': '-1,1,1,-1',
                'thruster_rotor_gain': '0.000049',
                'max_thrust': '100',
                'thruster_joy_gain': '-1.0'
            }.items()
        )
    )

    # Launch RViz2 with the custom config
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([
                    get_package_share_directory('eca_a9_control'),
                    'rviz',
                    'eca_a9_control.rviz'
                ])
            ]
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', default_value='eca_a9'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        OpaqueFunction(function=launch_setup)
    ])
