from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Lc
from ament_index_python.packages import get_package_share_directory

import os
import pathlib
import xacro

from plankton_utils.time import is_sim_time

def to_bool(value: str):
    if isinstance(value, bool):
        return value
    if not isinstance(value, str):
        raise ValueError('String to bool, invalid type ' + str(value))

    valid = {'true': True, '1': True,
             'false': False, '0': False}

    if value.lower() in valid:
        return valid[value]

    raise ValueError('String to bool, invalid value: %s' % value)


# =============================================================================
def launch_setup(context, *args, **kwargs):
    # Perform substitutions
    debug = Lc('debug').perform(context)
    namespace = Lc('namespace').perform(context)
    x = Lc('x').perform(context)
    y = Lc('y').perform(context)
    z = Lc('z').perform(context)
    roll = Lc('roll').perform(context)
    pitch = Lc('pitch').perform(context)
    yaw = Lc('yaw').perform(context)
    
    use_world_ned = Lc('use_ned_frame').perform(context)
    is_write_on_disk = Lc('write_file_on_disk').perform(context)
    pose_noise = Lc('pose_noise').perform(context)

    # Request sim time value from the global node
    res = is_sim_time(return_param=False, use_subprocess=True)

    # Xacro file path
    xacro_file = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'rexrov_default_noisy_pose.xacro'
    )

    # Output path for URDF if we want to write to disk
    path = os.path.join(
        get_package_share_directory('uuv_descriptions'),
        'robots',
        'generated',
        namespace,
    )

    if not pathlib.Path(path).exists():
        try:
            os.makedirs(path)
        except OSError:
            print(f"Creation of the directory {path} failed")

    output = os.path.join(path, 'robot_description.urdf')

    if not pathlib.Path(xacro_file).exists():
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    # Prepare mappings for xacro
    if to_bool(use_world_ned):
        mappings = {
            'debug': debug,
            'namespace': namespace,
            'noise_amp': pose_noise,
            'inertial_reference_frame': 'world_ned'
        }
    else:
        mappings = {
            'debug': debug,
            'namespace': namespace,
            'noise_amp': pose_noise,
            'inertial_reference_frame': 'world'
        }

    #Process xacro into URDF XML string
    doc = xacro.process(xacro_file, mappings=mappings)

    #Optionally write the URDF to disk
    if is_write_on_disk:
        with open(output, 'w') as file_out:
            file_out.write(doc)

    #Spawn URDF into Gazebo
    args = (
        f"-gazebo_namespace /gazebo "
        f"-x {x} -y {y} -z {z} "
        f"-R {roll} -P {pitch} -Y {yaw} "
        f"-entity {namespace} -topic robot_description").split()

    urdf_spawner = Node(
        name='urdf_spawner',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        parameters=[{'use_sim_time': res}],
        arguments=args
    )

    #Robot state publisher
    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': res},
            {'robot_description': doc}  # directly inject URDF string
        ]
    )

    #Include message_to_tf.launch
    message_to_tf_launch = os.path.join(
        get_package_share_directory('uuv_assistants'),
        'launch',
        'message_to_tf.launch'
    )

    if not pathlib.Path(message_to_tf_launch).exists():
        raise FileNotFoundError(f"Launch file not found: {message_to_tf_launch}")

    launch_args = [
        ('namespace', namespace),
        ('world_frame', 'world'),
        ('child_frame_id', '/' + namespace + '/base_link'),
        ('use_sim_time', str(res).lower()),
    ]
    

    message_to_tf_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(message_to_tf_launch),
        launch_arguments=launch_args
    )

    publish_body_sname = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uuv_assistants'),
                'launch',
                'publish_body_sname.launch.py'
            )
        ),
        launch_arguments={'uuv_name': namespace, 
        "use_sim_time": str(res).lower()}.items()
    )


    group = GroupAction([
        PushRosNamespace(namespace),
        urdf_spawner,
        robot_state_publisher,
    ])

    return [group, publish_body_sname, message_to_tf_launch]

# =============================================================================
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='0'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='-20'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('namespace', default_value='rexrov'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('write_file_on_disk', default_value='false'),
        DeclareLaunchArgument('pose_noise', default_value='0.02'),
        OpaqueFunction(function=launch_setup)
    ])
