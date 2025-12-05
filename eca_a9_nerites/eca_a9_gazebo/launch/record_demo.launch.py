from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Topics that are common in both modes
    common_topics = [
        "/eca_a9/dp_controller/trajectory",
        "/eca_a9/dp_controller/reference",
        "/eca_a9/dp_controller/error",
        "/hydrodynamics/current_velocity",
        "/eca_a9/wrench_perturbation",
        "/eca_a9/control_allocation/control_input",
        "/eca_a9/thrusters/0/thrust",
        "/eca_a9/thrusters/0/input",
        "/eca_a9/fins/0/input",
        "/eca_a9/fins/1/input",
        "/eca_a9/fins/2/input",
        "/eca_a9/fins/3/input",
        "/eca_a9/fins/0/output",
        "/eca_a9/fins/1/output",
        "/eca_a9/fins/2/output",
        "/eca_a9/fins/3/output",
    ]

    # Build args for ros2 bag
    bag_filename = LaunchConfiguration('bag_filename')
    ned_topics = " ".join([*common_topics, "/eca_a9/pose_gt_ned"])
    enu_topics = " ".join([*common_topics, "/eca_a9/pose_gt"])

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag_filename', default_value='recording.bag'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),

        # Only record if 'record' is true
        GroupAction(
            condition=IfCondition(LaunchConfiguration('record')),
            actions=[
                # Record NED version
                Node(
                    package='rosbag2',
                    executable='record',
                    name='recording_ned',
                    condition=IfCondition(LaunchConfiguration('use_ned_frame')),
                    arguments=['-O', bag_filename] + ned_topics.split(),
                    output='screen'
                ),
                # Record ENU version
                Node(
                    package='rosbag2',
                    executable='record',
                    name='recording_enu',
                    condition=UnlessCondition(LaunchConfiguration('use_ned_frame')),
                    arguments=['-O', bag_filename] + enu_topics.split(),
                    output='screen'
                ),
            ]
        )
    ])
