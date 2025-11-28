from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    gui_on = LaunchConfiguration('gui_on').perform(context)
    use_params_file = LaunchConfiguration('use_params_file').perform(context)

    controller_config_file = LaunchConfiguration('controller_config_file').perform(context)
    model_params_file = LaunchConfiguration('model_params_file').perform(context)

    thruster_manager_output_dir = LaunchConfiguration('thruster_manager_output_dir').perform(context)
    thruster_manager_config_file = LaunchConfiguration('thruster_manager_config_file').perform(context)
    tam_file = LaunchConfiguration('tam_file').perform(context)

    # Parse inline PID gains if needed
    Kp = [float(x) for x in LaunchConfiguration('Kp').perform(context).split(',')]
    Kd = [float(x) for x in LaunchConfiguration('Kd').perform(context).split(',')]
    Ki = [float(x) for x in LaunchConfiguration('Ki').perform(context).split(',')]

    actions = []

    # Thruster Manager include
    actions.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                get_package_share_directory('uuv_thruster_manager'), '/launch/thruster_manager.launch.py'
            ]),
            launch_arguments={
                'uuv_name': uuv_name,
                'model_name': LaunchConfiguration('model_name').perform(context),
                'output_dir': thruster_manager_output_dir,
                'config_file': thruster_manager_config_file,
                'tam_file': tam_file
            }.items()
        )
    )

    # RViz trajectory marker publisher if gui_on=true
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
                        ('waypoints', 'dp_controller/waypoints')
                    ]
                )
            ])
        )

    # Controller node: File-based or inline mode
    if use_params_file == 'true':
        # --- Load controller parameters from YAML file ---
        actions.append(
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_trajectory_control',
                    executable='rov_ua_pid_controller.py',
                    name='rov_ua_pid_controller',
                    output='screen',
                    remappings=[
                        ('odom', 'pose_gt'),
                        ('trajectory', 'dp_controller/trajectory'),
                        ('input_trajectory', 'dp_controller/input_trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('error', 'dp_controller/error'),
                        ('reference', 'dp_controller/reference'),
                        ('thruster_output', 'thruster_manager/input_stamped'),
                        ('tf', '/tf'),
                        ('tf_static', '/tf_static')
                    ],
                    parameters=[controller_config_file]
                )
            ])
        )
    else:
        # --- Use inline Kp/Kd/Ki values ---
        actions.append(
            GroupAction([
                PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_trajectory_control',
                    executable='rov_ua_pid_controller.py',
                    name='rov_ua_pid_controller',
                    output='screen',
                    remappings=[
                        ('odom', 'pose_gt'),
                        ('trajectory', 'dp_controller/trajectory'),
                        ('input_trajectory', 'dp_controller/input_trajectory'),
                        ('waypoints', 'dp_controller/waypoints'),
                        ('error', 'dp_controller/error'),
                        ('reference', 'dp_controller/reference'),
                        ('thruster_output', 'thruster_manager/input_stamped'),
                        ('tf', '/tf'),
                        ('tf_static', '/tf_static')
                    ],
                    parameters=[
                        {
                            'saturation': float(LaunchConfiguration('saturation').perform(context)),
                            'Kp': Kp,
                            'Kd': Kd,
                            'Ki': Ki
                        }
                    ]
                )
            ])
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Declare all arguments
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=LaunchConfiguration('uuv_name')),
        DeclareLaunchArgument('saturation', default_value='1200'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('gui_on', default_value='true'),

        DeclareLaunchArgument('Kp', default_value='10.0,10.0,10.0,10.0'),
        DeclareLaunchArgument('Kd', default_value='1.0,1.0,1.0,1.0'),
        DeclareLaunchArgument('Ki', default_value='0.5,0.5,0.5,0.5'),

        DeclareLaunchArgument(
            'controller_config_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_trajectory_control'),
                'config', 'controllers', 'ua_pid', LaunchConfiguration('model_name'), 'params.yaml'
            ])
        ),

        DeclareLaunchArgument(
            'thruster_manager_output_dir',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_thruster_manager'),
                'config', LaunchConfiguration('model_name')
            ])
        ),
        DeclareLaunchArgument(
            'thruster_manager_config_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_thruster_manager'),
                'config', LaunchConfiguration('model_name'), 'thruster_manager.yaml'
            ])
        ),
        DeclareLaunchArgument(
            'tam_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_thruster_manager'),
                'config', LaunchConfiguration('model_name'), 'TAM.yaml'
            ])
        ),
         DeclareLaunchArgument(
            'model_params_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_trajectory_control'),
                'config', 'models', LaunchConfiguration('model_name'), 'params.yaml'
            ])
        ),

        # Use OpaqueFunction to dynamically handle inline params
        OpaqueFunction(function=launch_setup)
    ])
