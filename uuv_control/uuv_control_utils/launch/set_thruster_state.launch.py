from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # ----------------------------------
    # Declare launch arguments
    # ----------------------------------
    args = [
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('starting_time', default_value='0.0'),
        DeclareLaunchArgument('thruster_id', default_value='0'),
        DeclareLaunchArgument('is_on', default_value='0'),
        DeclareLaunchArgument('duration', default_value='-1'),
    ]

    # ----------------------------------
    # Sanitize node name: remove '.' and '-'
    # ----------------------------------
    dynamic_node_name = PythonExpression([
        # Build a safe string for the node name
        "'set_thruster_state_' + "
        "str(", LaunchConfiguration('starting_time'), ").replace('.', '_').replace('-', 'neg_') + "
        "'_' + str(", LaunchConfiguration('thruster_id'), ")"
    ])

    # ----------------------------------
    # Node definition
    # ----------------------------------
    thruster_state_node = Node(
        package='uuv_control_utils',
        executable='set_thruster_state.py',
        name=dynamic_node_name,
        output='screen',
        parameters=[{
            'starting_time': LaunchConfiguration('starting_time'),
            'thruster_id': LaunchConfiguration('thruster_id'),
            'is_on': LaunchConfiguration('is_on'),
            'duration': LaunchConfiguration('duration')
        }]
    )

    # ----------------------------------
    # Namespace group
    # ----------------------------------
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        thruster_state_node
    ])

    return LaunchDescription(args + [group])
