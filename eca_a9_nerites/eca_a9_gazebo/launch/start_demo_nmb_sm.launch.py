from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Opaque function callback to dynamically assemble launch actions."""
    actions = []

    # Resolve LaunchConfigurations
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    gui_on = LaunchConfiguration('gui_on').perform(context)
    record = LaunchConfiguration('record').perform(context)
    bag_filename = LaunchConfiguration('bag_filename').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    # -------------------------
    # Include upload_eca_a9_default.launch.py
    # -------------------------
    upload_eca_a9_path = os.path.join(
        get_package_share_directory('eca_a9_description'),
        'launch',
        'upload_eca_a9_default.launch.py'
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upload_eca_a9_path),
            launch_arguments={
                'namespace': uuv_name,
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
            }.items()
        )
    )

    # -------------------------
    # Trajectory marker publisher (conditional on gui_on)
    # -------------------------
    if gui_on == 'true':
        actions.append(
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_control_utils',
                    executable='trajectory_marker_publisher.py',
                    name='trajectory_marker_publisher',
                    output='screen',
                    remappings=[
                        ('trajectory', 'dp_controller/trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('reference', 'dp_controller/reference')
                    ]
                )
            ])
        )

    # -------------------------
    # Include start_nmb_sm_control.launch.py
    # -------------------------
    start_control_path = os.path.join(
        get_package_share_directory('eca_a9_control'),
        'launch',
        'start_nmb_sm_control.launch.py'
    )
    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(start_control_path))
    )

    # -------------------------
    # Include send_waypoints_file.launch.py
    # -------------------------
    send_waypoints_path = os.path.join(
        get_package_share_directory('uuv_control_utils'),
        'launch',
        'send_waypoints_file.launch.py'
    )
    waypoint_file = os.path.join(
        get_package_share_directory('eca_a9_gazebo'),
        'config',
        'waypoint_set.yaml'
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(send_waypoints_path),
            launch_arguments={
                'uuv_name': uuv_name,
                'filename': waypoint_file,
                'interpolator': 'dubins'
            }.items()
        )
    )

    # -------------------------
    # Include record_demo.launch.py
    # -------------------------
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

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('uuv_name', default_value='eca_a9'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.bag'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('joy_id', default_value='0'),

        # Use OpaqueFunction to build everything dynamically
        OpaqueFunction(function=launch_setup)
    ])
