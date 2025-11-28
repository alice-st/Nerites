from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # ------------------------
    # Declare all launch arguments
    # ------------------------
    args = [
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('start_time', default_value='-1'),
        DeclareLaunchArgument('radius', default_value='8'),
        DeclareLaunchArgument('center_x', default_value='0'),
        DeclareLaunchArgument('center_y', default_value='0'),
        DeclareLaunchArgument('center_z', default_value='-30'),
        DeclareLaunchArgument('n_points', default_value='50'),
        DeclareLaunchArgument('n_turns', default_value='1'),
        DeclareLaunchArgument('delta_z', default_value='5.0'),
        DeclareLaunchArgument('heading_offset', default_value='0'),
        DeclareLaunchArgument('duration', default_value='150'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.3'),
    ]

    # ------------------------
    # Node definition
    # ------------------------
    helical_node = Node(
        package='uuv_control_utils',
        executable='start_helical_trajectory.py',
        name='start_helical_trajectory',
        output='screen',
        parameters=[{
            'start_time': LaunchConfiguration('start_time'),
            'radius': LaunchConfiguration('radius'),
            # Combine center_x, center_y, center_z into one string: "x,y,z"
            'center': PythonExpression([
                "str(", LaunchConfiguration('center_x'), ") + ',' + "
                "str(", LaunchConfiguration('center_y'), ") + ',' + "
                "str(", LaunchConfiguration('center_z'), ")"
            ]),
            'n_points': LaunchConfiguration('n_points'),
            'heading_offset': LaunchConfiguration('heading_offset'),
            'max_forward_speed': LaunchConfiguration('max_forward_speed'),
            'duration': LaunchConfiguration('duration'),
            'n_turns': LaunchConfiguration('n_turns'),
            'delta_z': LaunchConfiguration('delta_z')
        }]
    )

    # ------------------------
    # Group for namespace
    # ------------------------
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        helical_node
    ])

    # ------------------------
    # Return full launch description
    # ------------------------
    return LaunchDescription(args + [group])
