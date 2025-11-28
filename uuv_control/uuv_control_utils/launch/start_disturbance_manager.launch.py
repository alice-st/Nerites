from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    uuv_name = LaunchConfiguration('uuv_name')
    use_file = LaunchConfiguration('use_file')
    disturbance_file = LaunchConfiguration('disturbance_file')

    current_starting_time = LaunchConfiguration('current_starting_time')
    current_vel = LaunchConfiguration('current_vel')
    current_horz_angle = LaunchConfiguration('current_horz_angle')
    current_vert_angle = LaunchConfiguration('current_vert_angle')
    current_duration = LaunchConfiguration('current_duration')

    force_x = LaunchConfiguration('force_x')
    force_y = LaunchConfiguration('force_y')
    force_z = LaunchConfiguration('force_z')
    force = LaunchConfiguration('force')

    torque_x = LaunchConfiguration('torque_x')
    torque_y = LaunchConfiguration('torque_y')
    torque_z = LaunchConfiguration('torque_z')
    torque = LaunchConfiguration('torque')

    wrench_starting_time = LaunchConfiguration('wrench_starting_time')
    wrench_duration = LaunchConfiguration('wrench_duration')
    
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('uuv_name'))
    ld.add_action(DeclareLaunchArgument('use_file', default_value='true'))
    ld.add_action(DeclareLaunchArgument('disturbance_file', default_value=os.path.join(get_package_share_directory('uuv_tutorial_disturbances'), 'config', 'disturbances.yaml')))
    ld.add_action(DeclareLaunchArgument('current_starting_time', default_value='20.0'))
    ld.add_action(DeclareLaunchArgument('current_vel', default_value='1'))
    ld.add_action(DeclareLaunchArgument('current_horz_angle', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('current_vert_angle', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('current_duration', default_value='10'))
    ld.add_action(DeclareLaunchArgument('force_x', default_value='0'))
    ld.add_action(DeclareLaunchArgument('force_y', default_value='2000'))
    ld.add_action(DeclareLaunchArgument('force_z', default_value='0'))
    ld.add_action(DeclareLaunchArgument('force', default_value='0, 2000, 0'))
    ld.add_action(DeclareLaunchArgument('torque_x', default_value='0'))
    ld.add_action(DeclareLaunchArgument('torque_y', default_value='0'))
    ld.add_action(DeclareLaunchArgument('torque_z', default_value='0'))
    ld.add_action(DeclareLaunchArgument('torque', default_value='0, 0, 0'))
    ld.add_action(DeclareLaunchArgument('wrench_starting_time', default_value='30'))
    ld.add_action(DeclareLaunchArgument('wrench_duration', default_value='20'))

    ld.add_action(
        GroupAction([
            PushRosNamespace(namespace=uuv_name),
                Node(
                    package='uuv_control_utils',
                    executable='disturbance_manager.py',
                    name='disturbance_manager',
                    output='screen',
                    parameters=[disturbance_file],
                    condition=IfCondition(use_file)
                ),
                
                Node(
                    package='uuv_control_utils',
                    executable='disturbance_manager.py',
                    name='disturbance_manager',
                    output='screen',
                    parameters=[{
                        'disturbances': {
                            'd1': {
                                'type': 'current',
                                'starting_time': current_starting_time,
                                'velocity': current_vel,
                                'horizontal_angle': current_horz_angle,
                                'vertical_angle': current_vert_angle,
                                'duration': current_duration
                            },
                            'd2': {
                                'type': 'wrench',
                                'starting_time': wrench_starting_time,
                                'duration': wrench_duration,
                                'force': [force],
                                'torque': [torque]
                            }
                        }
                    }],condition=UnlessCondition(use_file)
                )
            ]))
    
    return ld