from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def launch_setup(context, *args, **kwargs):
    # Resolve launch arguments to strings
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Build the frame names directly in Python (no PythonExpression needed)
    base_link = f"/{uuv_name}/base_link"
    base_link_ned = f"/{uuv_name}/base_link_ned"

    # Node: static_transform_publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sname_frame_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '3.141592653589793',
            base_link,
            base_link_ned
        ],
        parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}]
    )

    # Group under namespace
    group = GroupAction([
        PushRosNamespace(uuv_name),
        static_tf_node
    ])

    return [group]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('use_sim_time'),

        OpaqueFunction(function=launch_setup)
    ])
