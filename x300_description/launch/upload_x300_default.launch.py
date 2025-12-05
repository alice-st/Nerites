from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os
import pathlib
import xacro

from nerites_utils.time import is_sim_time


def to_bool(value: str):
    """Convert a string or boolean to a proper boolean value."""
    if isinstance(value, bool):
        return value
    if not isinstance(value, str):
        raise ValueError(f"Invalid type for conversion to bool: {value}")
    valid = {'true': True, '1': True, 'false': False, '0': False}
    if value.lower() in valid:
        return valid[value]
    raise ValueError(f"Invalid value for conversion to bool: {value}")


def launch_setup(context, *args, **kwargs):
    """Set up the robot description, TFs, and state publisher for x300."""
    actions = []

    # -------------------------
    # Resolve launch arguments
    # -------------------------
    namespace = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    debug = LaunchConfiguration('debug').perform(context)
    use_world_ned = LaunchConfiguration('use_ned_frame').perform(context)
    write_file_on_disk = LaunchConfiguration('write_file_on_disk').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    sim_time = is_sim_time(return_param=False, use_subprocess=True)

    # -------------------------
    # Process the Xacro file into URDF
    # -------------------------
    xacro_file = os.path.join(
        get_package_share_directory('x300_description'),
        'robots',
        f'x300_default.urdf.xacro'
    )
    
    if not pathlib.Path(xacro_file).exists():
        raise FileNotFoundError(f'Xacro file does not exist: {xacro_file}')

    output_dir = os.path.join(
        get_package_share_directory('x300_description'),
        'robots',
        'generated',
        namespace,
    )

    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, 'robot_description.urdf')

    mappings = {
        'ns': namespace,
        'debug': debug,
        'inertial_reference_frame': 'world_ned' if to_bool(use_world_ned) else 'world',
        'robot_name': namespace
    }

    urdf_content = xacro.process_file(xacro_file, mappings=mappings).toxml()

    if to_bool(write_file_on_disk):
        with open(output_file, 'w') as f:
            f.write(urdf_content)

    # -------------------------
    # Include message_to_tf.launch
    # -------------------------
    message_to_tf_path = os.path.join(
        get_package_share_directory('uuv_assistants'),
        'launch',
        'message_to_tf.launch.py'
    )
    if not pathlib.Path(message_to_tf_path).exists():
        raise FileNotFoundError(f"Launch file does not exist: {message_to_tf_path}")

    message_to_tf_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(message_to_tf_path),
        launch_arguments=[
            ('namespace', namespace),
            ('world_frame', 'world'),
            ('child_frame_id', f'/{namespace}/base_link'),
            ('use_sim_time', str(sim_time).lower())
        ]
    )

    # -------------------------
    # Define nodes
    # -------------------------
    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='log',
        arguments=[
            '-gazebo_namespace', '/gazebo',
            '-x', x, '-y', y, '-z', z,
            '-R', roll, '-P', pitch, '-Y', yaw,
            '-entity', namespace,
            '-topic', 'robot_description'
        ]
    )

    static_tf_base_root_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_root_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', f'{namespace}/base_link'],
        output='log'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[
            {'robot_description': urdf_content},
            {'use_sim_time': sim_time}
        ]
    )

    # Group nodes under the namespace
    group = GroupAction([
        PushRosNamespace(namespace),
        urdf_spawner,
        robot_state_publisher,
        static_tf_base_root_base_link
    ])

    return [group, message_to_tf_launch]


def generate_launch_description():
    """Generate and return the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='1'),
        DeclareLaunchArgument('namespace', default_value='x300', description='Namespace for the robot'),
        DeclareLaunchArgument('x', default_value='0', description='X position of the robot'),
        DeclareLaunchArgument('y', default_value='0', description='Y position of the robot'),
        DeclareLaunchArgument('z', default_value='-20', description='Z position of the robot'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Roll orientation'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch orientation'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw orientation'),
        DeclareLaunchArgument('mode', default_value='default'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('write_file_on_disk', default_value='true'),

        # Use OpaqueFunction to dynamically set up all actions
        OpaqueFunction(function=launch_setup),
    ])
