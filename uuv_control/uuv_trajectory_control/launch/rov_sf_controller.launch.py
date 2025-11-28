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
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    controller_config_file = LaunchConfiguration('controller_config_file').perform(context)
    model_params_file = LaunchConfiguration('model_params_file').perform(context)

    thruster_manager_output_dir = LaunchConfiguration('thruster_manager_output_dir').perform(context)
    thruster_manager_config_file = LaunchConfiguration('thruster_manager_config_file').perform(context)
    tam_file = LaunchConfiguration('tam_file').perform(context)

    # Parse inline controller params
    Kd = [float(x) for x in LaunchConfiguration('Kd').perform(context).split(',')]
    lam = float(LaunchConfiguration('lambda').perform(context))
    c_val = float(LaunchConfiguration('c').perform(context))

    actions = []

    #Include Thruster Manager
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

    #Add RViz trajectory marker publisher if enabled
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

    #Launch Controller Nodes (parameter file mode OR inline mode)
    if use_params_file == 'true':
        # --- Using YAML controller_config_file ---
        for condition, inertial_frame, odom_topic in [
            (IfCondition(use_ned_frame), 'world_ned', 'pose_gt_ned'),
            (UnlessCondition(use_ned_frame), 'world', 'pose_gt')
        ]:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_sf_controller.py',
                        name='rov_sf_controller',
                        output='screen',
                        condition=condition,
                        remappings=[
                            ('odom', odom_topic),
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
                            controller_config_file,
                            model_params_file,
                            {'inertial_frame_id': inertial_frame}
                        ]
                    )
                ])
            )
    else:
        # --- Inline Kd, lambda, c ---
        for condition, inertial_frame, odom_topic in [
            (IfCondition(use_ned_frame), 'world_ned', 'pose_gt_ned'),
            (UnlessCondition(use_ned_frame), 'world', 'pose_gt')
        ]:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_sf_controller.py',
                        name='rov_sf_controller',
                        output='screen',
                        condition=condition,
                        remappings=[
                            ('odom', odom_topic),
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
                            model_params_file,
                            {
                                'saturation': float(LaunchConfiguration('saturation').perform(context)),
                                'Kd': Kd,
                                'lambda': lam,
                                'c': c_val,
                                'inertial_frame_id': inertial_frame
                            }
                        ]
                    )
                ])
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        #Declare launch arguments
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=LaunchConfiguration('uuv_name')),
        DeclareLaunchArgument('saturation', default_value='5000'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        DeclareLaunchArgument('Kd', default_value='100.0,100.0,100.0,100.0,100.0,100.0'),
        DeclareLaunchArgument('lambda', default_value='1.0'),
        DeclareLaunchArgument('c', default_value='1.0'),

        DeclareLaunchArgument(
            'controller_config_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_trajectory_control'),
                'config', 'controllers', 'pid', LaunchConfiguration('model_name'), 'params.yaml'
            ])
        ),
        DeclareLaunchArgument(
            'model_params_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('uuv_trajectory_control'),
                'config', 'models', LaunchConfiguration('model_name'), 'params.yaml'
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

        #Use OpaqueFunction to resolve LaunchConfigurations dynamically
        OpaqueFunction(function=launch_setup)
    ])
