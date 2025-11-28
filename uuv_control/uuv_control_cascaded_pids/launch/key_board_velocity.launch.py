from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ------------------------
    # Declare launch arguments
    # ------------------------
    args = [
        DeclareLaunchArgument('model_name'),
        DeclareLaunchArgument('uuv_name', default_value=LaunchConfiguration('model_name')),
    ]

    # ------------------------
    # Include thruster_manager.launch.py
    # ------------------------
    thruster_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': LaunchConfiguration('uuv_name'),
            'model_name': LaunchConfiguration('model_name')
        }.items()
    )

    # ------------------------
    # Acceleration control node
    # ------------------------
    accel_control_node = Node(
        package='uuv_control_cascaded_pid',
        executable='acceleration_control.py',
        name='acceleration_control',
        output='log',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uuv_control_cascaded_pid'),
                'config',
                LaunchConfiguration('model_name'),
                'inertial.yaml'
            ])
        ]
    )

    # ------------------------
    # Velocity control node
    # ------------------------
    vel_control_node = Node(
        package='uuv_control_cascaded_pid',
        executable='velocity_control.py',
        name='velocity_control',
        output='log',
        remappings=[
            ('odom', PythonExpression(["'/' + '", LaunchConfiguration('uuv_name'), "' + '/pose_gt'"])),
            ('cmd_accel', PythonExpression(["'/' + '", LaunchConfiguration('uuv_name'), "' + '/cmd_accel'"]))
        ],
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uuv_control_cascaded_pid'),
                'config',
                LaunchConfiguration('model_name'),
                'vel_pid_control.yaml'
            ])
        ]
    )

    # ------------------------
    # Group namespace
    # ------------------------
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('uuv_name')),
        accel_control_node,
        vel_control_node
    ])

    # ------------------------
    # Include uuv_keyboard_teleop.launch.py
    # ------------------------
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_teleop'),
                'launch',
                'uuv_keyboard_teleop.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': LaunchConfiguration('uuv_name'),
            'output_topic': TextSubstitution(text='cmd_vel'),
            'message_type': TextSubstitution(text='twist')
        }.items()
    )

    # ------------------------
    # Return the full launch description
    # ------------------------
    return LaunchDescription(args + [
        thruster_manager_launch,
        group,
        teleop_launch
    ])
