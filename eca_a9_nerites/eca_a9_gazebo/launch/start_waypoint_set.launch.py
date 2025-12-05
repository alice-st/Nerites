from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        #Declare the interpolator argument
        DeclareLaunchArgument('interpolator', default_value='cubic'),

        #Include send_waypoints_file.launch.py with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('uuv_control_utils'),
                '/launch/send_waypoints_file.launch.py'
            ]),
            launch_arguments={
                'uuv_name': 'eca_a9',
                'filename': PathJoinSubstitution([
                    FindPackageShare('eca_a9_gazebo'),
                    'config',
                    'waypoint_set.yaml'
                ]),
                'interpolator': LaunchConfiguration('interpolator'),
            }.items()
        )
    ])
