from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    """Opaque function callback to dynamically build all includes and nodes."""
    actions = []

    # Resolve runtime arguments
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    record = LaunchConfiguration('record').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)
    bag_filename = LaunchConfiguration('bag_filename').perform(context)

    # --- Include upload_eca_a9_default.launch.py ---
    upload_path = os.path.join(
        get_package_share_directory('eca_a9_description'),
        'launch',
        'upload_eca_a9_default.launch.py'
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upload_path),
            launch_arguments={
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
            }.items()
        )
    )

    # --- Include start_geometric_tracking_control.launch.py ---
    tracking_control_path = os.path.join(
        get_package_share_directory('eca_a9_control'),
        'launch',
        'start_geometric_tracking_control.launch.py'
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracking_control_path)
        )
    )

    # --- Include record_demo.launch.py ---
    record_demo_path = os.path.join(
        get_package_share_directory('eca_a9_gazebo'),
        'launch',
        'record_demo.launch.py'
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(record_demo_path),
            launch_arguments={
                'record': record,
                'use_ned_frame': use_ned_frame,
                'bag_filename': bag_filename,
            }.items()
        )
    )

    # --- RViz Node ---
    rviz_config = os.path.join(
        get_package_share_directory('uuv_gazebo'),
        'rviz2',
        'controller_demo.rviz'
    )

    rviz_node = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    actions.append(rviz_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-5'),
        DeclareLaunchArgument('yaw', default_value='0'),

        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.bag'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        # Use OpaqueFunction to build everything dynamically
        OpaqueFunction(function=launch_setup)
    ])
