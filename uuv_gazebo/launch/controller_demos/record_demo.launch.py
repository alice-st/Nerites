from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------------
    # Declare arguments
    # ----------------------------------
    args = [
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.bag'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
    ]

    # ----------------------------------
    # Define rosbag record nodes
    # ----------------------------------

    # Topics for NED frame
    ned_topics = [
        '/rexrov/dp_controller/trajectory',
        '/rexrov/dp_controller/reference',
        '/rexrov/pose_gt_ned',
        '/hydrodynamics/current_velocity',
        '/rexrov/thruster_manager/input',
        '/rexrov/wrench_perturbation',
        '/rexrov/thrusters/0/thrust',
        '/rexrov/thrusters/1/thrust',
        '/rexrov/thrusters/2/thrust',
        '/rexrov/thrusters/3/thrust',
        '/rexrov/thrusters/4/thrust',
        '/rexrov/thrusters/5/thrust',
        '/rexrov/thrusters/6/thrust',
        '/rexrov/thrusters/7/thrust',
    ]

    # Topics for ENU frame
    enu_topics = [
        '/rexrov/dp_controller/trajectory',
        '/rexrov/dp_controller/reference',
        '/rexrov/pose_gt',
        '/hydrodynamics/current_velocity',
        '/rexrov/thruster_manager/input',
        '/rexrov/wrench_perturbation',
        '/rexrov/thrusters/0/thrust',
        '/rexrov/thrusters/1/thrust',
        '/rexrov/thrusters/2/thrust',
        '/rexrov/thrusters/3/thrust',
        '/rexrov/thrusters/4/thrust',
        '/rexrov/thrusters/5/thrust',
        '/rexrov/thrusters/6/thrust',
        '/rexrov/thrusters/7/thrust',
    ]

    # NED-frame recorder (runs only if use_ned_frame is true)
    ned_recorder = Node(
        package='rosbag2',
        executable='record',
        name='recording',
        output='screen',
        arguments=['-O', LaunchConfiguration('bag_filename')] + ned_topics,
        condition=IfCondition(LaunchConfiguration('use_ned_frame'))
    )

    # ENU-frame recorder (runs only if use_ned_frame is false)
    enu_recorder = Node(
        package='rosbag2',
        executable='record',
        name='recording',
        output='screen',
        arguments=['-O', LaunchConfiguration('bag_filename')] + enu_topics,
        condition=UnlessCondition(LaunchConfiguration('use_ned_frame'))
    )

    # Group so we can turn recording on/off with `record` arg
    record_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('record')),
        actions=[ned_recorder, enu_recorder]
    )

    # ----------------------------------
    # Return LaunchDescription
    # ----------------------------------
    return LaunchDescription(args + [record_group])
