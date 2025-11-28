from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # Resolve runtime arguments
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    saturation = LaunchConfiguration('saturation').perform(context)
    gui_on = LaunchConfiguration('gui_on').perform(context)
    use_params_file = LaunchConfiguration('use_params_file').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)
    max_forward_speed = LaunchConfiguration('max_forward_speed').perform(context)
    K = LaunchConfiguration('K').perform(context)
    Kd = LaunchConfiguration('Kd').perform(context)
    Ki = LaunchConfiguration('Ki').perform(context)
    slope = LaunchConfiguration('slope').perform(context)

    # Build file paths dynamically
    controller_config_file = os.path.join(
        get_package_share_directory('uuv_trajectory_control'),
        'config/controllers/nmb_sm', model_name, 'params.yaml'
    )
    thruster_manager_output_dir = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'config', model_name
    )
    thruster_manager_config_file = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'config', model_name, 'thruster_manager.yaml'
    )
    tam_file = os.path.join(
        get_package_share_directory('uuv_thruster_manager'),
        'config', model_name, 'TAM.yaml'
    )

    actions = []

    # === Thruster manager include ===
    actions.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('uuv_thruster_manager'),
                'launch/thruster_manager.launch'
            )),
            launch_arguments={
                'uuv_name': uuv_name,
                'model_name': model_name,
                'output_dir': thruster_manager_output_dir,
                'config_file': thruster_manager_config_file,
                'tam_file': tam_file
            }.items()
        )
    )

    # === RViz marker publisher group ===
    actions.append(
        GroupAction([
            PushRosNamespace(namespace=uuv_name),
            Node(
                condition=IfCondition(gui_on),
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

    # === Param file case ===
    if use_params_file == 'true':
        if use_ned_frame == 'true':
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nmb_sm_controller.py',
                        name='rov_nmb_smcontroller',
                        output='screen',
                        remappings=[
                            ('odom', 'pose_gt_ned'),
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
                            {'inertial_frame_id': 'world_ned'},
                            {'saturation': float(saturation)},
                            {'max_forward_speed': float(max_forward_speed)}
                        ]
                    )
                ])
            )
        else:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nmb_sm_controller.py',
                        name='rov_nmb_smcontroller',
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
                            controller_config_file,
                            {'inertial_frame_id': 'world'},
                            {'saturation': float(saturation)},
                            {'max_forward_speed': float(max_forward_speed)}
                        ]
                    )
                ])
            )
    else:
        # === Manual param case ===
        if use_ned_frame == 'true':
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nmb_sm_controller.py',
                        name='rov_nmb_smcontroller',
                        output='screen',
                        remappings=[
                            ('odom', 'pose_gt_ned'),
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
                            {'saturation': float(saturation)},
                            {'K': [K]},
                            {'Kd': [Kd]},
                            {'Ki': [Ki]},
                            {'slope': [slope]},
                            {'inertial_frame_id': 'world_ned'},
                            {'max_forward_speed': float(max_forward_speed)}
                        ]
                    )
                ])
            )
        else:
            actions.append(
                GroupAction([
                    PushRosNamespace(namespace=uuv_name),
                    Node(
                        package='uuv_trajectory_control',
                        executable='rov_nmb_sm_controller.py',
                        name='rov_nmb_smcontroller',
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
                            {'saturation': float(saturation)},
                            {'K': [K]},
                            {'Kd': [Kd]},
                            {'Ki': [Ki]},
                            {'slope': [slope]},
                            {'inertial_frame_id': 'world'},
                            {'max_forward_speed': float(max_forward_speed)}
                        ]
                    )
                ])
            )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name'),
        DeclareLaunchArgument('model_name', default_value=LaunchConfiguration('uuv_name')),
        DeclareLaunchArgument('saturation', default_value='1200'),
        DeclareLaunchArgument('gui_on', default_value='true'),
        DeclareLaunchArgument('use_params_file', default_value='false'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('max_forward_speed', default_value='0.5'),
        DeclareLaunchArgument('K', default_value='5,5,5,5,5,5'),
        DeclareLaunchArgument('Kd', default_value='4118.98,4118.98,4118.98,8000.0,8000.0,8000.0'),
        DeclareLaunchArgument('Ki', default_value='0.06144,0.06144,0.06144,0.078,0.078,0.078'),
        DeclareLaunchArgument('slope', default_value='0.182,0.182,0.182,3.348,3.348,3.348'),
        OpaqueFunction(function=launch_setup)
    ])
