from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    debug = LaunchConfiguration('debug').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    launch_file = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'launch',
        f"upload_rexrov_{mode}.launch.py"
    )

    if not os.path.exists(launch_file):
        raise FileNotFoundError(f"Could not find {launch_file}")

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'debug': debug,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'use_ned_frame': use_ned_frame,
            'namespace': namespace,
        }.items()
    )

    return [include]

def generate_launch_description():
    # Declare launch arguments (just like XML)
    args = [
        DeclareLaunchArgument('debug', default_value='0'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mode', default_value='default'),
        DeclareLaunchArgument('namespace', default_value='rexrov'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
    ]

    return LaunchDescription(args + [
        OpaqueFunction(function=launch_setup)
    ])
